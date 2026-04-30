#include "climbing_controller.h"


// ------------------------------------------------------------
// ClimbingController
// 说明:
// - 本文件是原 Climbing_task.cpp 的类封装版本。
// - 控制逻辑保持不变, 仅把全局变量/静态函数收敛到类内。
// - 任务层仅负责周期调用 AutoTask1ms() + TaskEntry1ms()。
// ------------------------------------------------------------

ClimbingController::ClimbingController()
    : climb_state_(STEP_IDLE),              // 状态机初始为 IDLE
      state_tick_(0),                       // 状态进入时间戳
      is_zero_recorded_(0),                 // 软零点尚未记录
      start_pos_front_(0.0f),               // 上台阶前腿起始角
      start_pos_rear_(0.0f),                // 上台阶后腿起始角
      desc_start_pos_front_(0.0f),          // 下台阶前腿起始角
      desc_start_pos_rear_(0.0f),           // 下台阶后腿起始角
      wheel_target_angle_l_(0.0f),          // 左轮目标角
      wheel_target_angle_r_(0.0f),          // 右轮目标角
      is_lift_pid_mode_(0),                 // 当前未启用 lift PID
      prev_climb_state_(STEP_IDLE),         // 上一次状态
      auto_running_(0),                     // 自动流程未运行
      auto_state_enter_tick_(0),            // 自动流程状态计时
      descend_mode_(0),                     // 当前非下台阶模式
      init_pose_active_(0),                 // 初始化抬腿姿态未激活
      rear_lift_delayed_flag_(0),
      up_mode_(CLIMB_UP_MODE_20CM) {}       // 默认上40cm参数

void ClimbingController::HandleStateTransition(uint32_t current_time, uint8_t state_changed)
{
    if (state_changed == 0) return;

    state_tick_ = current_time;

    rear_lift_delayed_flag_ = 0;

    // 轮子斜坡同步当前位置
    slope_wheel_l_angle_.Set_Now_Real(motor_wheel_l_.Get_Now_Angle());
    slope_wheel_r_angle_.Set_Now_Real(motor_wheel_r_.Get_Now_Angle());

    // 提取公共起始角
    float wheel_start_l = motor_wheel_l_.Get_Now_Angle();
    float wheel_start_r = motor_wheel_r_.Get_Now_Angle();

    // 提取腿部当前绝对位置作为规划起点
    float front_now = motor_lift_front_.Get_Now_Angle();
    float rear_now  = motor_lift_rear_.Get_Now_Angle();
    bool is_20cm = (up_mode_ == CLIMB_UP_MODE_20CM);

    // 状态切换瞬间，下发五次多项式规划指令
    switch (climb_state_)
    {
        case STEP_IDLE:
        case STEP_DONE:
            // 归位到初始零点
            planner_front_pos_.Plan(front_now, start_pos_front_, 1.0f);
            planner_rear_pos_.Plan(rear_now, start_pos_rear_, 1.0f);
            break;

        case STEP_SETUP:
            planner_front_pos_.Plan(front_now, start_pos_front_ + (is_20cm ? POS_FRONT_RETRACT_20cm : POS_FRONT_RETRACT_40cm), TIME_SETUP / 1000.0f);
            planner_rear_pos_.Plan(rear_now, start_pos_rear_ + (is_20cm ? POS_REAR_RETRACT_20cm : POS_REAR_RETRACT_40cm), TIME_SETUP / 1000.0f);
            break;

        case STEP_TOUCH_DOWN:
            planner_front_pos_.Plan(front_now, start_pos_front_ + (is_20cm ? POS_FRONT_TOUCH_20cm : POS_FRONT_TOUCH_40cm), TIME_TOUCH / 1000.0f);
            planner_rear_pos_.Plan(rear_now, start_pos_rear_ + (is_20cm ? POS_REAR_TOUCH_20cm : POS_REAR_TOUCH_40cm), TIME_TOUCH / 1000.0f);
            break;

        case STEP_GLOBAL_LIFT:
            // 顶升阶段前腿立即规划，后腿依赖延时，在 UpdateStateTargets 中处理
            planner_front_pos_.Plan(front_now, start_pos_front_ + (is_20cm ? POS_FRONT_LIFT_20cm : POS_FRONT_LIFT_40cm), TIME_LIFT / 1000.0f);
            break;

        case STEP_DRIVE_FWD:
            wheel_target_angle_l_ = wheel_start_l + WHEEL_TRAVEL_UP_RAD;
            wheel_target_angle_r_ = wheel_start_r - WHEEL_TRAVEL_UP_RAD;
            break;

        case STEP_RETRACT:
            planner_front_pos_.Plan(front_now, start_pos_front_ + (is_20cm ? POS_FRONT_FINAL_20cm : POS_FRONT_FINAL_40cm), TIME_RETRACT / 1000.0f);
            planner_rear_pos_.Plan(rear_now, start_pos_rear_ + (is_20cm ? POS_REAR_FINAL_20cm : POS_REAR_FINAL_40cm), TIME_RETRACT / 1000.0f);
            break;

        // ---------- 下台阶部分 ----------
        case STEP_DESCEND_SETUP:
            planner_front_pos_.Plan(front_now, start_pos_front_ + POS_FRONT_Init, TIME_DESC_SETUP / 1000.0f);
            planner_rear_pos_.Plan(rear_now, start_pos_rear_ + POS_REAR_Init, TIME_DESC_SETUP / 1000.0f);
            break;

        case STEP_DESCEND_TOUCH:
            planner_front_pos_.Plan(front_now, desc_start_pos_front_ + DESCEND_FRONT_TOUCH_TARGET, TIME_DESC_TOUCH / 1000.0f);
            planner_rear_pos_.Plan(rear_now, desc_start_pos_rear_ + DESCEND_REAR_TOUCH_TARGET, TIME_DESC_TOUCH / 1000.0f);
            break;

        case STEP_DESCEND_GLOBAL_DOWN:
            planner_front_pos_.Plan(front_now, desc_start_pos_front_ + DESCEND_FRONT_GLOBAL_DOWN_TARGET, TIME_DESC_GLOBAL_DOWN / 1000.0f);
            planner_rear_pos_.Plan(rear_now, desc_start_pos_rear_ + DESCEND_REAR_GLOBAL_DOWN_TARGET, TIME_DESC_GLOBAL_DOWN / 1000.0f);
            break;

        case STEP_DESCEND_DRIVE:
            wheel_target_angle_l_ = wheel_start_l + WHEEL_TRAVEL_DESCEND_RAD;
            wheel_target_angle_r_ = wheel_start_r - WHEEL_TRAVEL_DESCEND_RAD;
            break;

        case STEP_DESCEND_RAISE:
            planner_front_pos_.Plan(front_now, start_pos_front_ + DESCEND_FRONT_RAISE_TARGET, TIME_DESC_RAISE / 1000.0f);
            planner_rear_pos_.Plan(rear_now, start_pos_rear_ + DESCEND_REAR_RAISE_TARGET, TIME_DESC_RAISE / 1000.0f);
            break;

        case STEP_DESCEND_RELEASE:
            planner_front_pos_.Plan(front_now, start_pos_front_ + DESCEND_FRONT_RAISE_TARGET, TIME_DESC_RELEASE / 1000.0f); // 保持举起姿势
            planner_rear_pos_.Plan(rear_now, start_pos_rear_ + DESCEND_REAR_RAISE_TARGET, TIME_DESC_RELEASE / 1000.0f);
            wheel_target_angle_l_ = wheel_start_l + WHEEL_TRAVEL_DESCEND_RELEASE_RAD;
            wheel_target_angle_r_ = wheel_start_r - WHEEL_TRAVEL_DESCEND_RELEASE_RAD;
            break;
            
        case STEP_DESCEND_DONE:
            planner_front_pos_.Plan(front_now, start_pos_front_ + POS_FRONT_Init, 1.0f);
            planner_rear_pos_.Plan(rear_now, start_pos_rear_ + POS_REAR_Init, 1.0f);
            break;

        default:
            break;
    }

    prev_climb_state_ = climb_state_;
}

void ClimbingController::RecordSoftZero(void)
{
    // 上电首次记录软零点, 并同步斜坡内部状态
    start_pos_front_ = motor_lift_front_.Get_Now_Angle();
    start_pos_rear_ = motor_lift_rear_.Get_Now_Angle();

    // 强行锁死规划器在初始零点
    planner_front_pos_.ForceSetPosition(start_pos_front_);
    planner_rear_pos_.ForceSetPosition(start_pos_rear_);

    // 轮子斜坡同步当前位置和目标位置
    slope_wheel_l_angle_.Set_Now_Real(motor_wheel_l_.Get_Now_Angle());
    slope_wheel_r_angle_.Set_Now_Real(motor_wheel_r_.Get_Now_Angle());
    slope_wheel_l_angle_.Set_Target(motor_wheel_l_.Get_Now_Angle());
    slope_wheel_r_angle_.Set_Target(motor_wheel_r_.Get_Now_Angle());

    slope_wheel_l_angle_.TIM_Calculate_PeriodElapsedCallback();
    slope_wheel_r_angle_.TIM_Calculate_PeriodElapsedCallback();

    is_zero_recorded_ = 1;
}

bool ClimbingController::TryTimeTransition(uint32_t now, uint32_t delay_ms, ClimbingState_e next_state)
{
    if ((now - auto_state_enter_tick_) >= delay_ms)
    {
        climb_state_ = next_state;
        auto_state_enter_tick_ = now;
        return true;
    }
    return false;
}


// uint8_t ClimbingController::IsWheelAngleDone(void)
// {
//     // 判断左右轮是否到达目标角(带容差)
//     float err_l = wheel_target_angle_l_ - motor_wheel_l_.Get_Now_Angle();
//     float err_r = wheel_target_angle_r_ - motor_wheel_r_.Get_Now_Angle();
//     return (Math_Abs(err_l) <= WHEEL_ANGLE_DONE_TOL_RAD && Math_Abs(err_r) <= WHEEL_ANGLE_DONE_TOL_RAD);
// }

void ClimbingController::SetLiftPidMode(uint8_t enable_lift)
{
    // 如果目标模式与当前模式一致，直接返回，避免重复设置
    if (is_lift_pid_mode_ == enable_lift) return; 

    if (enable_lift)
    {
        motor_lift_front_.PID_Omega.Set_K_P(PID_FRONT_OMEGA_KP_LIFT);
        motor_lift_front_.PID_Angle.Set_K_P(PID_FRONT_ANGLE_KP_LIFT);
        motor_lift_rear_.PID_Omega.Set_K_P(PID_REAR_OMEGA_KP_LIFT);
        motor_lift_rear_.PID_Angle.Set_K_P(PID_REAR_ANGLE_KP_LIFT);
        motor_lift_rear_.PID_Omega.Set_K_I(PID_REAR_OMEGA_KI_LIFT);
        motor_lift_rear_.PID_Angle.Set_K_I(PID_REAR_ANGLE_KI_LIFT);
    }
    else
    {
        motor_lift_front_.PID_Omega.Set_K_P(PID_FRONT_OMEGA_KP_NORMAL);
        motor_lift_front_.PID_Angle.Set_K_P(PID_FRONT_ANGLE_KP_NORMAL);
        motor_lift_rear_.PID_Omega.Set_K_P(PID_REAR_OMEGA_KP_NORMAL);
        motor_lift_rear_.PID_Angle.Set_K_P(PID_REAR_ANGLE_KP_NORMAL);
        motor_lift_rear_.PID_Omega.Set_K_I(PID_REAR_OMEGA_KI_NORMAL);
        motor_lift_rear_.PID_Angle.Set_K_I(PID_REAR_ANGLE_KI_NORMAL);
    }
    is_lift_pid_mode_ = enable_lift;
}

void ClimbingController::SetWheelSlopeStep(float wheel_step)
{
    slope_wheel_l_angle_.Set_Increase_Value(wheel_step);
    slope_wheel_l_angle_.Set_Decrease_Value(wheel_step);
    slope_wheel_r_angle_.Set_Increase_Value(wheel_step);
    slope_wheel_r_angle_.Set_Decrease_Value(wheel_step);
}

void ClimbingController::UpdatePidAndSlopeByState(void)
{
    // 1. 轮子斜坡：仅取决于当前是上台阶还是下台阶大类
    uint8_t is_descend_state = (climb_state_ >= STEP_DESCEND_SETUP && climb_state_ <= STEP_DESCEND_DONE);
    SetWheelSlopeStep(is_descend_state ? WHEEL_SLOPE_STEP_DESCEND : WHEEL_SLOPE_STEP_UP);

    // 2. 腿部斜坡与 PID 模式：按具体状态精确映射
    switch (climb_state_)
    {
        case STEP_GLOBAL_LIFT:
        case STEP_DRIVE_FWD:
        case STEP_DESCEND_TOUCH:
        case STEP_DESCEND_GLOBAL_DOWN:
        case STEP_DESCEND_DRIVE:
        case STEP_DESCEND_RAISE:
            SetLiftPidMode(1); // Lift PID (重载)
            break;
        default:
            SetLiftPidMode(0); // 恢复 Normal PID
            break;
    }
}

void ClimbingController::UpdateStateTargets(uint32_t current_time)
{
    // 大部分状态下，轮子的目标角保持不变
    slope_wheel_l_angle_.Set_Target(climb_state_ == STEP_DRIVE_FWD || climb_state_ == STEP_DESCEND_DRIVE || climb_state_ == STEP_DESCEND_RELEASE ? wheel_target_angle_l_ : motor_wheel_l_.Get_Now_Angle());
    slope_wheel_r_angle_.Set_Target(climb_state_ == STEP_DRIVE_FWD || climb_state_ == STEP_DESCEND_DRIVE || climb_state_ == STEP_DESCEND_RELEASE ? wheel_target_angle_r_ : motor_wheel_r_.Get_Now_Angle());

    // 仅仅处理后腿的延时抬升规划
    if (climb_state_ == STEP_GLOBAL_LIFT && rear_lift_delayed_flag_ == 0)
    {
        if ((current_time - state_tick_) >= TIME_LIFT_REAR_DELAY)
        {
            bool is_20cm = (up_mode_ == CLIMB_UP_MODE_20CM);
            float rear_now = motor_lift_rear_.Get_Now_Angle();
            float target_pos = start_pos_rear_ + (is_20cm ? POS_REAR_LIFT_20cm : POS_REAR_LIFT_40cm);
            float time_left_sec = (TIME_LIFT - TIME_LIFT_REAR_DELAY) / 1000.0f;
            
            planner_rear_pos_.Plan(rear_now, target_pos, time_left_sec);
            rear_lift_delayed_flag_ = 1; // 标记已规划，不再重复进入
        }
    }
}

void ClimbingController::RunLiftAndWheelControl(void)
{
    // 控制主链路:
    // 1) 斜坡计算 -> 2) 下发目标角 -> 3) PID 计算
    float target_front = planner_front_pos_.GetNextPosition(0.001f);
    float target_rear  = planner_rear_pos_.GetNextPosition(0.001f);

    motor_lift_front_.Set_Target_Angle(target_front);
    motor_lift_rear_.Set_Target_Angle(target_rear);

    // 轮子斜坡保持不变
    slope_wheel_l_angle_.TIM_Calculate_PeriodElapsedCallback();
    slope_wheel_r_angle_.TIM_Calculate_PeriodElapsedCallback();

    if (climb_state_ == STEP_GLOBAL_LIFT)
    {
        if (motor_wheel_l_.Get_Control_Method() != Control_Method_OMEGA)
        {
            motor_wheel_l_.Set_Control_Method(Control_Method_OMEGA);
            motor_wheel_r_.Set_Control_Method(Control_Method_OMEGA);
            motor_wheel_l_.PID_Angle.Set_Integral_Error(0.0f);
            motor_wheel_l_.PID_Omega.Set_Integral_Error(0.0f);
            motor_wheel_r_.PID_Angle.Set_Integral_Error(0.0f);
            motor_wheel_r_.PID_Omega.Set_Integral_Error(0.0f);
        }

        motor_wheel_l_.Set_Target_Omega(WHEEL_CREEP_OMEGA_RADPS);
        motor_wheel_r_.Set_Target_Omega(-WHEEL_CREEP_OMEGA_RADPS);
    }
    else
    {
        if (motor_wheel_l_.Get_Control_Method() != Control_Method_ANGLE)
        {
            motor_wheel_l_.Set_Control_Method(Control_Method_ANGLE);
            motor_wheel_r_.Set_Control_Method(Control_Method_ANGLE);
            motor_wheel_l_.PID_Angle.Set_Integral_Error(0.0f);
            motor_wheel_l_.PID_Omega.Set_Integral_Error(0.0f);
            motor_wheel_r_.PID_Angle.Set_Integral_Error(0.0f);
            motor_wheel_r_.PID_Omega.Set_Integral_Error(0.0f);
        }

        motor_wheel_l_.Set_Target_Angle(slope_wheel_l_angle_.Get_Out());
        motor_wheel_r_.Set_Target_Angle(slope_wheel_r_angle_.Get_Out());
    }

    motor_lift_front_.TIM_PID_PeriodElapsedCallback();
    motor_lift_rear_.TIM_PID_PeriodElapsedCallback();
    motor_wheel_l_.TIM_PID_PeriodElapsedCallback();
    motor_wheel_r_.TIM_PID_PeriodElapsedCallback();
}

void ClimbingController::Init(CAN_HandleTypeDef *hcan)
{
    // 初始化顺序:
    // 1) 斜坡规划器 2) 电机对象 3) 底盘模块 4) PID 参数 5) 运行状态变量
    slope_wheel_l_angle_.Init(WHEEL_SLOPE_STEP_UP, WHEEL_SLOPE_STEP_UP, Slope_First_REAL);
    slope_wheel_r_angle_.Init(WHEEL_SLOPE_STEP_UP, WHEEL_SLOPE_STEP_UP, Slope_First_REAL);

    motor_lift_front_.Init(hcan, CAN_Motor_ID_0x201, Control_Method_ANGLE);
    motor_lift_rear_.Init(hcan, CAN_Motor_ID_0x202, Control_Method_ANGLE);
    motor_wheel_l_.Init(hcan, CAN_Motor_ID_0x203, Control_Method_ANGLE);
    motor_wheel_r_.Init(hcan, CAN_Motor_ID_0x204, Control_Method_ANGLE);

    motor_lift_front_.PID_Omega.Init(PID_FRONT_OMEGA_KP_NORMAL, 5.0f, 0.0f, 0.0f, 10000.0f, 12000.0f);
    motor_lift_front_.PID_Angle.Init(PID_FRONT_ANGLE_KP_NORMAL, 0.5f, 0.0f, 0.0f, 10000.0f, 12000.0f);

    motor_lift_rear_.PID_Omega.Init(PID_REAR_OMEGA_KP_NORMAL, 5.0f, 0.0f, 0.0f, 12000.0f, 14000.0f);
    motor_lift_rear_.PID_Angle.Init(PID_REAR_ANGLE_KP_NORMAL, 0.5f, 0.0f, 0.0f, 12000.0f, 14000.0f);

    motor_wheel_l_.PID_Omega.Init(PID_WHEEL_OMEGA_KP, PID_WHEEL_OMEGA_KI, 0.0f, 0.0f, 10000.0f, 12000.0f);
    motor_wheel_l_.PID_Angle.Init(PID_WHEEL_ANGLE_KP, PID_WHEEL_ANGLE_KI, 0.0f, 0.0f, 10000.0f, 12000.0f);
    motor_wheel_r_.PID_Omega.Init(PID_WHEEL_OMEGA_KP, PID_WHEEL_OMEGA_KI, 0.0f, 0.0f, 10000.0f, 12000.0f);
    motor_wheel_r_.PID_Angle.Init(PID_WHEEL_ANGLE_KP, PID_WHEEL_ANGLE_KI, 0.0f, 0.0f, 10000.0f, 12000.0f);

    climb_state_ = STEP_IDLE;
    is_zero_recorded_ = 0;
    is_lift_pid_mode_ = 0;
    descend_mode_ = 0;
    auto_running_ = 0;
    prev_climb_state_ = STEP_IDLE;
}

void ClimbingController::CAN_RxCallback(uint32_t std_id, uint8_t *data)
{
    // 201~204 分别对应: 前腿/后腿/左轮/右轮
    if (std_id == 0x201)
    {
        motor_lift_front_.CAN_RxCpltCallback(data);
    }
    else if (std_id == 0x202)
    {
        motor_lift_rear_.CAN_RxCpltCallback(data);
    }
    else if (std_id == 0x203)
    {
        motor_wheel_l_.CAN_RxCpltCallback(data);
    }
    else if (std_id == 0x204)
    {
        motor_wheel_r_.CAN_RxCpltCallback(data);
    }
}

void ClimbingController::AutoStart(void)
{
    init_pose_active_ = 0;
    auto_running_ = 1;
    descend_mode_ = 0;
    climb_state_ = STEP_SETUP;
    auto_state_enter_tick_ = HAL_GetTick();
}

void ClimbingController::AutoStart20cm(void)
{
    up_mode_ = CLIMB_UP_MODE_20CM;
    AutoStart();
}

void ClimbingController::AutoStart40cm(void)
{
    up_mode_ = CLIMB_UP_MODE_40CM;
    AutoStart();
}

void ClimbingController::Prepare40cm(void)
{
    init_pose_active_ = 0;
    auto_running_ = 0;
    descend_mode_ = 0;
    up_mode_ = CLIMB_UP_MODE_40CM;
    climb_state_ = STEP_SETUP;
}

void ClimbingController::AutoStartFromTouch20cm(void)
{
    init_pose_active_ = 0;
    auto_running_ = 1;
    descend_mode_ = 0;
    up_mode_ = CLIMB_UP_MODE_20CM;
    climb_state_ = STEP_TOUCH_DOWN;
    auto_state_enter_tick_ = HAL_GetTick();
}

void ClimbingController::AutoStartFromTouch40cm(void)
{
    init_pose_active_ = 0;
    auto_running_ = 1;
    descend_mode_ = 0;
    up_mode_ = CLIMB_UP_MODE_40CM;
    climb_state_ = STEP_TOUCH_DOWN;
    auto_state_enter_tick_ = HAL_GetTick();
}

void ClimbingController::DescendAutoStart(void)
{
    init_pose_active_ = 0;
    auto_running_ = 1;
    descend_mode_ = 1;
    desc_start_pos_front_ = motor_lift_front_.Get_Now_Angle();
    desc_start_pos_rear_ = motor_lift_rear_.Get_Now_Angle();
    climb_state_ = STEP_DESCEND_SETUP;
    auto_state_enter_tick_ = HAL_GetTick();
}

void ClimbingController::DescendAutoStart20cm(void)
{
    DescendAutoStart();
}

void ClimbingController::InitPoseStart(void)
{
    auto_running_ = 0;
    descend_mode_ = 0;
    init_pose_active_ = 1;
    climb_state_ = STEP_IDLE;
}

void ClimbingController::AutoTask1ms(void)
{
    if (auto_running_ == 0) return;

    uint32_t now = HAL_GetTick();

    switch (climb_state_)
    {
    // ---------- 下台阶自动流程 ----------
    case STEP_DESCEND_SETUP:
        if (TryTimeTransition(now, TIME_DESC_SETUP, STEP_DESCEND_TOUCH)) {
            desc_start_pos_front_ = motor_lift_front_.Get_Now_Angle();
            desc_start_pos_rear_ = motor_lift_rear_.Get_Now_Angle();
        }
        break;
    case STEP_DESCEND_TOUCH:
        TryTimeTransition(now, TIME_DESC_TOUCH, STEP_DESCEND_GLOBAL_DOWN);
        break;
    case STEP_DESCEND_GLOBAL_DOWN:
        TryTimeTransition(now, TIME_DESC_GLOBAL_DOWN, STEP_DESCEND_DRIVE);
        break;
    case STEP_DESCEND_DRIVE:
        TryTimeTransition(now, DESCEND_DRIVE_TIME_MS, STEP_DESCEND_RAISE);
        break;
    case STEP_DESCEND_RAISE:
        TryTimeTransition(now, TIME_DESC_RAISE, STEP_DESCEND_RELEASE);
        break;
    case STEP_DESCEND_RELEASE:
        if (TryTimeTransition(now, TIME_DESC_RELEASE, STEP_DESCEND_DONE))
        {
            auto_running_ = 0; // 流程结束
        }
        break;
    case STEP_DESCEND_DONE:
        auto_running_ = 0;
        descend_mode_ = 0;
        break;

    // ---------- 上台阶自动流程 ----------
    case STEP_SETUP:
        TryTimeTransition(now, TIME_SETUP, STEP_TOUCH_DOWN);
        break;
    case STEP_TOUCH_DOWN:
        TryTimeTransition(now, TIME_TOUCH, STEP_GLOBAL_LIFT);
        break;
    case STEP_GLOBAL_LIFT:
        TryTimeTransition(now, TIME_LIFT, STEP_DRIVE_FWD);
        break;
    case STEP_DRIVE_FWD:
        TryTimeTransition(now, TIME_DRIVE, STEP_RETRACT);
        break;
    case STEP_RETRACT:
        if (TryTimeTransition(now, TIME_RETRACT, STEP_DONE)) {
            auto_running_ = 0;
        }
        break;
    case STEP_DONE:
        auto_running_ = 0;
        descend_mode_ = 0;
        break;
    default:
        auto_running_ = 0;
        descend_mode_ = 0;
        break;
    }
}

void ClimbingController::EmergencyStop(void)
{
    // 急停: 清空输出并回到 IDLE
    init_pose_active_ = 0;
    auto_running_ = 0;
    descend_mode_ = 0;
    climb_state_ = STEP_IDLE;

    motor_lift_front_.Set_Out(0.0f);
    motor_lift_rear_.Set_Out(0.0f);
    motor_wheel_l_.Set_Out(0.0f);
    motor_wheel_r_.Set_Out(0.0f);
    // 强行锁住轨迹在此时的实际位置
    planner_front_pos_.ForceSetPosition(motor_lift_front_.Get_Now_Angle());
    planner_rear_pos_.ForceSetPosition(motor_lift_rear_.Get_Now_Angle());
    motor_wheel_l_.Set_Target_Angle(motor_wheel_l_.Get_Now_Angle());
    motor_wheel_r_.Set_Target_Angle(motor_wheel_r_.Get_Now_Angle());
}

void ClimbingController::TaskEntry1ms(void)
{
    // 1. 状态转移与参数更新
    uint32_t current_time = HAL_GetTick();
    HandleStateTransition(current_time, climb_state_ != prev_climb_state_);
    UpdatePidAndSlopeByState();

    // 2. 单次初始化拦截
    if (is_zero_recorded_ == 0) {
        RecordSoftZero();
    }

    // 3. 特殊模式：初始化姿态保持
    if (init_pose_active_ != 0U) {
         planner_front_pos_.ForceSetPosition(start_pos_front_ + POS_FRONT_Init);
        planner_rear_pos_.ForceSetPosition(start_pos_rear_ + POS_REAR_Init);
        slope_wheel_l_angle_.Set_Target(motor_wheel_l_.Get_Now_Angle());
        slope_wheel_r_angle_.Set_Target(motor_wheel_r_.Get_Now_Angle());
        RunLiftAndWheelControl();
        return;
    }

    // 4. 正常状态机控制更新
    UpdateStateTargets(current_time);
    RunLiftAndWheelControl();
}

//vofa调试接口
void ClimbingController::ManualNext(void)
{
    init_pose_active_ = 0;
    auto_running_ = 0;
    descend_mode_ = 0;
    state_tick_ = HAL_GetTick();

    switch (climb_state_)
    {
    case STEP_IDLE: climb_state_ = STEP_SETUP; break;
    case STEP_SETUP: climb_state_ = STEP_TOUCH_DOWN; break;
    case STEP_TOUCH_DOWN: climb_state_ = STEP_GLOBAL_LIFT; break;
    case STEP_GLOBAL_LIFT: climb_state_ = STEP_DRIVE_FWD; break;
    case STEP_DRIVE_FWD: climb_state_ = STEP_RETRACT; break;
    case STEP_RETRACT: climb_state_ = STEP_DONE; break;
    case STEP_DONE: climb_state_ = STEP_IDLE; break;
    default: climb_state_ = STEP_IDLE; break;
    }
}

//vofa调试接口
void ClimbingController::DescendManualNext(void)
{
    init_pose_active_ = 0;
    auto_running_ = 0;
    descend_mode_ = 1;
    state_tick_ = HAL_GetTick();

    if (climb_state_ < STEP_DESCEND_SETUP || climb_state_ > STEP_DESCEND_DONE) {
        climb_state_ = STEP_DESCEND_SETUP;
        return;
    }

    switch (climb_state_)
    {
    case STEP_DESCEND_SETUP:
        desc_start_pos_front_ = motor_lift_front_.Get_Now_Angle();
        desc_start_pos_rear_ = motor_lift_rear_.Get_Now_Angle();
        climb_state_ = STEP_DESCEND_TOUCH;
        break;
    case STEP_DESCEND_TOUCH: climb_state_ = STEP_DESCEND_GLOBAL_DOWN; break;
    case STEP_DESCEND_GLOBAL_DOWN: climb_state_ = STEP_DESCEND_DRIVE; break;
    case STEP_DESCEND_DRIVE: climb_state_ = STEP_DESCEND_RAISE; break;
    case STEP_DESCEND_RAISE: climb_state_ = STEP_DESCEND_RELEASE; break;
    case STEP_DESCEND_RELEASE: climb_state_ = STEP_DESCEND_DONE; break;
    case STEP_DESCEND_DONE:
    default:
        climb_state_ = STEP_IDLE;
        descend_mode_ = 0;
        break;
    }
}
