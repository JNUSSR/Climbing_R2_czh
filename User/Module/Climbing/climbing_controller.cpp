#include "climbing_controller.h"

#include "Mecanum/task_mecanum_chassis.h"

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
      chassis_external_control_(0),         // 底盘外部接管关闭
      init_pose_active_(0),                 // 初始化抬腿姿态未激活
      up_mode_(CLIMB_UP_MODE_40CM) {}       // 默认上40cm参数

float ClimbingController::ClampC620Output(float out)
{
    // C620 输出限幅, 避免超出驱动可接受范围
    if (out > 16384.0f)
    {
        return 16384.0f;
    }
    if (out < -16384.0f)
    {
        return -16384.0f;
    }
    return out;
}

void ClimbingController::ApplyMotorOutputWithComp(Class_Motor_C620 &motor, float comp, uint8_t can_byte_index)
{
    // 把重力补偿叠加到 PID 输出上, 并写回 CAN 发送缓冲区
    float final_out = motor.Get_Out() + comp;
    final_out = ClampC620Output(final_out);
    motor.Set_Out(final_out);

    int16_t out_int = (int16_t)final_out;
    CAN1_0x200_Tx_Data[can_byte_index] = (uint8_t)(out_int >> 8);
    CAN1_0x200_Tx_Data[can_byte_index + 1] = (uint8_t)(out_int);
}

void ClimbingController::ComputeGravityCompensation(float *front_comp, float *rear_comp)
{
    // 按状态机阶段切换补偿方向/大小:
    // - 收腿/悬空段: 以向上托举为主
    // - 顶升/承载段: 以向下支撑为主
    *front_comp = 0.0f;
    *rear_comp = 0.0f;

    switch (climb_state_)
    {
    case STEP_DESCEND_SETUP:
    case STEP_DESCEND_WAIT_TRIGGER:
    case STEP_DESCEND_TOUCH:
        *front_comp = GRAVITY_COMPENSATION_FRONT;
        *rear_comp = GRAVITY_COMPENSATION_REAR;
        break;
    case STEP_DESCEND_GLOBAL_DOWN:
    case STEP_DESCEND_DRIVE:
    case STEP_DESCEND_RAISE:
        *front_comp = COMP_FRONT_LIFT;
        *rear_comp = COMP_REAR_LIFT;
        break;
    case STEP_SETUP:
    case STEP_WAIT_TRIGGER:
    case STEP_TOUCH_DOWN:
        *front_comp = GRAVITY_COMPENSATION_FRONT;
        *rear_comp = GRAVITY_COMPENSATION_REAR;
        break;
    case STEP_GLOBAL_LIFT:
    case STEP_DRIVE_FWD:
        *front_comp = COMP_FRONT_LIFT;
        *rear_comp = COMP_REAR_LIFT;
        break;
    case STEP_RETRACT:
        *front_comp = GRAVITY_COMPENSATION_FRONT;
        *rear_comp = GRAVITY_COMPENSATION_REAR;
        break;
    default:
        break;
    }
}

void ClimbingController::HandleStateTransition(uint32_t current_time, uint8_t state_changed)
{
    // 仅在状态变化时执行一次的逻辑:
    // 1) 更新时间戳
    // 2) 离开 WAIT_TRIGGER 时底盘立即停车
    // 3) 斜坡规划器与当前实测角度对齐
    // 4) 进入 DRIVE 状态时重算轮子目标角
    if (state_changed == 0)
    {
        return;
    }

    state_tick_ = current_time;

    if ((prev_climb_state_ == STEP_WAIT_TRIGGER || prev_climb_state_ == STEP_DESCEND_WAIT_TRIGGER) &&
        climb_state_ != STEP_WAIT_TRIGGER &&
        climb_state_ != STEP_DESCEND_WAIT_TRIGGER)
    {
        Task_Mecanum_Chassis_StopImmediate();
    }

    slope_front_pos_.Set_Now_Real(motor_lift_front_.Get_Now_Angle());
    slope_rear_pos_.Set_Now_Real(motor_lift_rear_.Get_Now_Angle());

    slope_wheel_l_angle_.Set_Now_Real(motor_wheel_l_.Get_Now_Angle());
    slope_wheel_r_angle_.Set_Now_Real(motor_wheel_r_.Get_Now_Angle());

    if (climb_state_ == STEP_DRIVE_FWD)
    {
        float wheel_start_l = motor_wheel_l_.Get_Now_Angle();
        float wheel_start_r = motor_wheel_r_.Get_Now_Angle();
        wheel_target_angle_l_ = wheel_start_l + WHEEL_TRAVEL_UP_RAD;
        wheel_target_angle_r_ = wheel_start_r - WHEEL_TRAVEL_UP_RAD;
    }
    else if (climb_state_ == STEP_DESCEND_DRIVE)
    {
        float wheel_start_l = motor_wheel_l_.Get_Now_Angle();
        float wheel_start_r = motor_wheel_r_.Get_Now_Angle();
        wheel_target_angle_l_ = wheel_start_l + WHEEL_TRAVEL_DESCEND_RAD;
        wheel_target_angle_r_ = wheel_start_r - WHEEL_TRAVEL_DESCEND_RAD;
    }

    prev_climb_state_ = climb_state_;
}

uint8_t ClimbingController::IsWheelAngleDone(void)
{
    // 判断左右轮是否到达目标角(带容差)
    float err_l = wheel_target_angle_l_ - motor_wheel_l_.Get_Now_Angle();
    float err_r = wheel_target_angle_r_ - motor_wheel_r_.Get_Now_Angle();
    return (Math_Abs(err_l) <= WHEEL_ANGLE_DONE_TOL_RAD && Math_Abs(err_r) <= WHEEL_ANGLE_DONE_TOL_RAD);
}

void ClimbingController::UpdatePidAndSlopeByState(void)
{
    // 根据当前状态决定:
    // - 使用 normal PID 还是 lift PID
    // - 腿部斜坡组(上台阶 A/B, 下台阶 G1/G2)
    // - 轮子斜坡(上台阶/下台阶)
    uint8_t is_descend_state = (climb_state_ >= STEP_DESCEND_SETUP && climb_state_ <= STEP_DESCEND_DONE);
    uint8_t is_lift_pid_state = 0;
    uint8_t is_slope_group_b = 0;
    uint8_t is_slope_group_a = 0;
    //先判断是上台阶还是下台阶
    if (is_descend_state)  //如果是下台阶
    {
        is_lift_pid_state = (climb_state_ == STEP_DESCEND_TOUCH ||
                             climb_state_ == STEP_DESCEND_GLOBAL_DOWN ||
                             climb_state_ == STEP_DESCEND_DRIVE ||
                             climb_state_ == STEP_DESCEND_RAISE);
        is_slope_group_a = (climb_state_ == STEP_DESCEND_SETUP || climb_state_ == STEP_DESCEND_WAIT_TRIGGER);
        is_slope_group_b = (climb_state_ == STEP_DESCEND_TOUCH ||
                            climb_state_ == STEP_DESCEND_GLOBAL_DOWN ||
                            climb_state_ == STEP_DESCEND_DRIVE ||
                            climb_state_ == STEP_DESCEND_RAISE);
    }
    else //上台阶
    {
        is_lift_pid_state = (climb_state_ == STEP_GLOBAL_LIFT || climb_state_ == STEP_DRIVE_FWD);
        is_slope_group_b = (climb_state_ == STEP_TOUCH_DOWN || climb_state_ == STEP_GLOBAL_LIFT);
        is_slope_group_a = (climb_state_ == STEP_SETUP || climb_state_ == STEP_RETRACT);
    }

    if (is_lift_pid_state)
    {
        if (is_lift_pid_mode_ == 0)  // 当前处于需要 lift PID 的工况（上/下台阶重载阶段）
        {
            motor_lift_front_.PID_Omega.Set_K_P(PID_FRONT_OMEGA_KP_LIFT);
            motor_lift_front_.PID_Angle.Set_K_P(PID_FRONT_ANGLE_KP_LIFT);
            motor_lift_rear_.PID_Omega.Set_K_P(PID_REAR_OMEGA_KP_LIFT);
            motor_lift_rear_.PID_Angle.Set_K_P(PID_REAR_ANGLE_KP_LIFT);
            motor_lift_rear_.PID_Omega.Set_K_I(PID_REAR_OMEGA_KI_LIFT);
            motor_lift_rear_.PID_Angle.Set_K_I(PID_REAR_ANGLE_KI_LIFT);
            is_lift_pid_mode_ = 1; //只需要进一次，保持当前 PID 参数直到状态改变到非 lift 状态再切回 normal PID 参数
        }
    }
    else
    {
        if (is_lift_pid_mode_ == 1)
        {
            motor_lift_front_.PID_Omega.Set_K_P(PID_FRONT_OMEGA_KP_NORMAL);
            motor_lift_front_.PID_Angle.Set_K_P(PID_FRONT_ANGLE_KP_NORMAL);
            motor_lift_rear_.PID_Omega.Set_K_P(PID_REAR_OMEGA_KP_NORMAL);
            motor_lift_rear_.PID_Angle.Set_K_P(PID_REAR_ANGLE_KP_NORMAL);
            motor_lift_rear_.PID_Omega.Set_K_I(PID_REAR_OMEGA_KI_NORMAL);
            motor_lift_rear_.PID_Angle.Set_K_I(PID_REAR_ANGLE_KI_NORMAL);
            is_lift_pid_mode_ = 0;
        }
    }

    if (is_slope_group_b)
    {
        if (is_descend_state)
        {
            slope_front_pos_.Set_Increase_Value(FRONT_SLOPE_STEP_DESC_G2);
            slope_front_pos_.Set_Decrease_Value(FRONT_SLOPE_STEP_DESC_G2);
            slope_rear_pos_.Set_Increase_Value(REAR_SLOPE_STEP_DESC_G2);
            slope_rear_pos_.Set_Decrease_Value(REAR_SLOPE_STEP_DESC_G2);
        }
        else
        {
            slope_front_pos_.Set_Increase_Value(FRONT_SLOPE_STEP_TOUCH_LIFT);
            slope_front_pos_.Set_Decrease_Value(FRONT_SLOPE_STEP_TOUCH_LIFT);
            slope_rear_pos_.Set_Increase_Value(REAR_SLOPE_STEP_TOUCH_LIFT);
            slope_rear_pos_.Set_Decrease_Value(REAR_SLOPE_STEP_TOUCH_LIFT);
        }
    }
    else if (is_slope_group_a)
    {
        if (is_descend_state)
        {
            slope_front_pos_.Set_Increase_Value(FRONT_SLOPE_STEP_DESC_G1);
            slope_front_pos_.Set_Decrease_Value(FRONT_SLOPE_STEP_DESC_G1);
            slope_rear_pos_.Set_Increase_Value(REAR_SLOPE_STEP_DESC_G1);
            slope_rear_pos_.Set_Decrease_Value(REAR_SLOPE_STEP_DESC_G1);
        }
        else
        {
            slope_front_pos_.Set_Increase_Value(FRONT_SLOPE_STEP_SETUP_RETRACT);
            slope_front_pos_.Set_Decrease_Value(FRONT_SLOPE_STEP_SETUP_RETRACT);
            slope_rear_pos_.Set_Increase_Value(REAR_SLOPE_STEP_SETUP_RETRACT);
            slope_rear_pos_.Set_Decrease_Value(REAR_SLOPE_STEP_SETUP_RETRACT);
        }
    }

    if (is_descend_state)
    {
        slope_wheel_l_angle_.Set_Increase_Value(WHEEL_SLOPE_STEP_DESCEND);
        slope_wheel_l_angle_.Set_Decrease_Value(WHEEL_SLOPE_STEP_DESCEND);
        slope_wheel_r_angle_.Set_Increase_Value(WHEEL_SLOPE_STEP_DESCEND);
        slope_wheel_r_angle_.Set_Decrease_Value(WHEEL_SLOPE_STEP_DESCEND);
    }
    else
    {
        slope_wheel_l_angle_.Set_Increase_Value(WHEEL_SLOPE_STEP_UP);
        slope_wheel_l_angle_.Set_Decrease_Value(WHEEL_SLOPE_STEP_UP);
        slope_wheel_r_angle_.Set_Increase_Value(WHEEL_SLOPE_STEP_UP);
        slope_wheel_r_angle_.Set_Decrease_Value(WHEEL_SLOPE_STEP_UP);
    }
}

void ClimbingController::UpdateStateTargets(uint32_t current_time)
{
    // 每个状态对应一组目标:
    // - 前/后腿目标角
    // - 左/右轮目标角
    // - WAIT_TRIGGER 阶段可触发底盘前移
    switch (climb_state_)
    {
    case STEP_IDLE:
        slope_front_pos_.Set_Target(start_pos_front_);
        slope_rear_pos_.Set_Target(start_pos_rear_);
        slope_wheel_l_angle_.Set_Target(motor_wheel_l_.Get_Now_Angle());
        slope_wheel_r_angle_.Set_Target(motor_wheel_r_.Get_Now_Angle());
        break;
    case STEP_SETUP:
        if (up_mode_ == CLIMB_UP_MODE_20CM)
        {
            slope_front_pos_.Set_Target(start_pos_front_ + POS_FRONT_RETRACT_20cm);
            slope_rear_pos_.Set_Target(start_pos_rear_ + POS_REAR_RETRACT_20cm);
        }
        else
        {
            slope_front_pos_.Set_Target(start_pos_front_ + POS_FRONT_RETRACT_40cm);
            slope_rear_pos_.Set_Target(start_pos_rear_ + POS_REAR_RETRACT_40cm);
        }
        slope_wheel_l_angle_.Set_Target(motor_wheel_l_.Get_Now_Angle());
        slope_wheel_r_angle_.Set_Target(motor_wheel_r_.Get_Now_Angle());
        break;
    case STEP_WAIT_TRIGGER:
    {
        // 自动流程中, 在时间窗口内给底盘前移速度
        float wait_vx = 0.0f;
        if (!chassis_external_control_ && auto_running_ && (current_time - state_tick_) < WAIT_TRIGGER_FORWARD_TIME_MS)
        {
            wait_vx = WAIT_TRIGGER_FORWARD_VX;
        }
        if (!chassis_external_control_)
        {
            Task_Mecanum_Chassis_SetMotion(wait_vx, 0.0f, 0.0f);
        }
        slope_wheel_l_angle_.Set_Target(motor_wheel_l_.Get_Now_Angle());
        slope_wheel_r_angle_.Set_Target(motor_wheel_r_.Get_Now_Angle());
        break;
    }
    case STEP_TOUCH_DOWN:
        if (up_mode_ == CLIMB_UP_MODE_20CM)
        {
            slope_front_pos_.Set_Target(start_pos_front_ + POS_FRONT_TOUCH_20cm);
            slope_rear_pos_.Set_Target(start_pos_rear_ + POS_REAR_TOUCH_20cm);
        }
        else
        {
            slope_front_pos_.Set_Target(start_pos_front_ + POS_FRONT_TOUCH_40cm);
            slope_rear_pos_.Set_Target(start_pos_rear_ + POS_REAR_TOUCH_40cm);
        }
        slope_wheel_l_angle_.Set_Target(motor_wheel_l_.Get_Now_Angle());
        slope_wheel_r_angle_.Set_Target(motor_wheel_r_.Get_Now_Angle());
        break;
    case STEP_GLOBAL_LIFT:
        if (up_mode_ == CLIMB_UP_MODE_20CM)
        {
            slope_front_pos_.Set_Target(start_pos_front_ + POS_FRONT_LIFT_20cm);
            slope_rear_pos_.Set_Target(start_pos_rear_ + POS_REAR_LIFT_20cm);
        }
        else
        {
            slope_front_pos_.Set_Target(start_pos_front_ + POS_FRONT_LIFT_40cm);
            slope_rear_pos_.Set_Target(start_pos_rear_ + POS_REAR_LIFT_40cm);
        }
        slope_wheel_l_angle_.Set_Target(motor_wheel_l_.Get_Now_Angle());
        slope_wheel_r_angle_.Set_Target(motor_wheel_r_.Get_Now_Angle());
        break;
    case STEP_DRIVE_FWD:
        slope_wheel_l_angle_.Set_Target(wheel_target_angle_l_);
        slope_wheel_r_angle_.Set_Target(wheel_target_angle_r_);
        break;
    case STEP_RETRACT:
        slope_front_pos_.Set_Target(start_pos_front_ + POS_FRONT_FINAL);
        slope_rear_pos_.Set_Target(start_pos_rear_ + POS_REAR_FINAL);
        slope_wheel_l_angle_.Set_Target(motor_wheel_l_.Get_Now_Angle());
        slope_wheel_r_angle_.Set_Target(motor_wheel_r_.Get_Now_Angle());
        break;
    case STEP_DONE:
        slope_wheel_l_angle_.Set_Target(motor_wheel_l_.Get_Now_Angle());
        slope_wheel_r_angle_.Set_Target(motor_wheel_r_.Get_Now_Angle());
        break;
    case STEP_DESCEND_SETUP:
        slope_front_pos_.Set_Target(desc_start_pos_front_ + POS_FRONT_RETRACT_20cm);
        slope_rear_pos_.Set_Target(desc_start_pos_rear_ + POS_REAR_RETRACT_20cm);
        slope_wheel_l_angle_.Set_Target(motor_wheel_l_.Get_Now_Angle());
        slope_wheel_r_angle_.Set_Target(motor_wheel_r_.Get_Now_Angle());
        break;
    case STEP_DESCEND_WAIT_TRIGGER:
    {
        // 下台阶等待触发阶段, 同样在窗口内给底盘前移速度
        float wait_vx = 0.0f;
        if (!chassis_external_control_ && descend_mode_ && auto_running_ && (current_time - state_tick_) < DESCEND_WAIT_TIME_MS)
        {
            wait_vx = DESCEND_WAIT_FORWARD_VX;
        }
        if (!chassis_external_control_)
        {
            Task_Mecanum_Chassis_SetMotion(wait_vx, 0.0f, 0.0f);
        }
        slope_wheel_l_angle_.Set_Target(motor_wheel_l_.Get_Now_Angle());
        slope_wheel_r_angle_.Set_Target(motor_wheel_r_.Get_Now_Angle());
        break;
    }
    case STEP_DESCEND_TOUCH:
        slope_front_pos_.Set_Target(desc_start_pos_front_ + DESCEND_FRONT_TOUCH_TARGET);
        slope_rear_pos_.Set_Target(desc_start_pos_rear_ + DESCEND_REAR_TOUCH_TARGET);
        slope_wheel_l_angle_.Set_Target(motor_wheel_l_.Get_Now_Angle());
        slope_wheel_r_angle_.Set_Target(motor_wheel_r_.Get_Now_Angle());
        break;
    case STEP_DESCEND_GLOBAL_DOWN:
        slope_front_pos_.Set_Target(desc_start_pos_front_ + DESCEND_FRONT_GLOBAL_DOWN_TARGET);
        slope_rear_pos_.Set_Target(desc_start_pos_rear_ + DESCEND_REAR_GLOBAL_DOWN_TARGET);
        slope_wheel_l_angle_.Set_Target(motor_wheel_l_.Get_Now_Angle());
        slope_wheel_r_angle_.Set_Target(motor_wheel_r_.Get_Now_Angle());
        break;
    case STEP_DESCEND_DRIVE:
        slope_wheel_l_angle_.Set_Target(wheel_target_angle_l_);
        slope_wheel_r_angle_.Set_Target(wheel_target_angle_r_);
        break;
    case STEP_DESCEND_RAISE:
        slope_front_pos_.Set_Target(desc_start_pos_front_);
        slope_rear_pos_.Set_Target(desc_start_pos_rear_);
        slope_wheel_l_angle_.Set_Target(motor_wheel_l_.Get_Now_Angle());
        slope_wheel_r_angle_.Set_Target(motor_wheel_r_.Get_Now_Angle());
        break;
    case STEP_DESCEND_DONE:
        slope_wheel_l_angle_.Set_Target(motor_wheel_l_.Get_Now_Angle());
        slope_wheel_r_angle_.Set_Target(motor_wheel_r_.Get_Now_Angle());
        break;
    }
}

void ClimbingController::UpdateChassisControl(void)
{
    // 非 WAIT_TRIGGER 状态下强制清零底盘指令, 防止残留速度
    if (!chassis_external_control_ && climb_state_ != STEP_WAIT_TRIGGER && climb_state_ != STEP_DESCEND_WAIT_TRIGGER)
    {
        Task_Mecanum_Chassis_SetMotion(0.0f, 0.0f, 0.0f);
    }

    Task_Mecanum_Chassis_Task1ms();
}

void ClimbingController::RunLiftAndWheelControl(void)
{
    // 控制主链路:
    // 1) 斜坡计算 -> 2) 下发目标角 -> 3) PID 计算
    slope_front_pos_.TIM_Calculate_PeriodElapsedCallback();
    slope_rear_pos_.TIM_Calculate_PeriodElapsedCallback();
    slope_wheel_l_angle_.TIM_Calculate_PeriodElapsedCallback();
    slope_wheel_r_angle_.TIM_Calculate_PeriodElapsedCallback();

    motor_lift_front_.Set_Target_Angle(slope_front_pos_.Get_Out());
    motor_lift_rear_.Set_Target_Angle(slope_rear_pos_.Get_Out());
    motor_wheel_l_.Set_Target_Angle(slope_wheel_l_angle_.Get_Out());
    motor_wheel_r_.Set_Target_Angle(slope_wheel_r_angle_.Get_Out());

    motor_lift_front_.TIM_PID_PeriodElapsedCallback();
    motor_lift_rear_.TIM_PID_PeriodElapsedCallback();
    motor_wheel_l_.TIM_PID_PeriodElapsedCallback();
    motor_wheel_r_.TIM_PID_PeriodElapsedCallback();
}

void ClimbingController::ApplyGravityCompensationAndSend(void)
{
    // 在 PID 输出基础上叠加重力补偿并发送 CAN
    float target_comp_front = 0.0f;
    float target_comp_rear = 0.0f;
    ComputeGravityCompensation(&target_comp_front, &target_comp_rear);

    ApplyMotorOutputWithComp(motor_lift_front_, target_comp_front, 0);
    ApplyMotorOutputWithComp(motor_lift_rear_, target_comp_rear, 2);

    TIM_CAN_PeriodElapsedCallback();
}

void ClimbingController::Init(CAN_HandleTypeDef *hcan)
{
    // 初始化顺序:
    // 1) 斜坡规划器 2) 电机对象 3) 底盘模块 4) PID 参数 5) 运行状态变量
    slope_front_pos_.Init(FRONT_SLOPE_STEP_SETUP_RETRACT, FRONT_SLOPE_STEP_SETUP_RETRACT, Slope_First_REAL);
    slope_rear_pos_.Init(REAR_SLOPE_STEP_SETUP_RETRACT, REAR_SLOPE_STEP_SETUP_RETRACT, Slope_First_REAL);
    slope_wheel_l_angle_.Init(WHEEL_SLOPE_STEP_UP, WHEEL_SLOPE_STEP_UP, Slope_First_REAL);
    slope_wheel_r_angle_.Init(WHEEL_SLOPE_STEP_UP, WHEEL_SLOPE_STEP_UP, Slope_First_REAL);

    motor_lift_front_.Init(hcan, CAN_Motor_ID_0x201, Control_Method_ANGLE);
    motor_lift_rear_.Init(hcan, CAN_Motor_ID_0x202, Control_Method_ANGLE);
    motor_wheel_l_.Init(hcan, CAN_Motor_ID_0x203, Control_Method_ANGLE);
    motor_wheel_r_.Init(hcan, CAN_Motor_ID_0x204, Control_Method_ANGLE);

    Task_Mecanum_Chassis_Init();

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

void ClimbingController::NextStep(void)
{
    init_pose_active_ = 0;
    if (climb_state_ == STEP_IDLE)
    {
        climb_state_ = STEP_SETUP;
    }
    else if (climb_state_ == STEP_WAIT_TRIGGER)
    {
        climb_state_ = STEP_TOUCH_DOWN;
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

uint8_t ClimbingController::IsAutoRunning(void) const
{
    return auto_running_;
}

void ClimbingController::AutoTask1ms(void)
{
    // 自动流程推进器: 仅按时间/到位条件切状态, 不做控制输出
    uint32_t now = HAL_GetTick();

    if (auto_running_ == 0)
    {
        return;
    }

    switch (climb_state_)
    {
    // ---------- 下台阶自动流程 ----------
    case STEP_DESCEND_SETUP:
        if ((now - auto_state_enter_tick_) >= TIME_DESC_SETUP)
        {
            climb_state_ = STEP_DESCEND_WAIT_TRIGGER;
            auto_state_enter_tick_ = now;
        }
        break;
    case STEP_DESCEND_WAIT_TRIGGER:
        if ((now - auto_state_enter_tick_) >= DESCEND_WAIT_TIME_MS)
        {
            climb_state_ = STEP_DESCEND_TOUCH;
            auto_state_enter_tick_ = now;
        }
        break;
    case STEP_DESCEND_TOUCH:
        if ((now - auto_state_enter_tick_) >= TIME_DESC_TOUCH)
        {
            climb_state_ = STEP_DESCEND_GLOBAL_DOWN;
            auto_state_enter_tick_ = now;
        }
        break;
    case STEP_DESCEND_GLOBAL_DOWN:
        if ((now - auto_state_enter_tick_) >= TIME_DESC_GLOBAL_DOWN)
        {
            climb_state_ = STEP_DESCEND_DRIVE;
            auto_state_enter_tick_ = now;
        }
        break;
    case STEP_DESCEND_DRIVE:
        if (IsWheelAngleDone())
        {
            climb_state_ = STEP_DESCEND_RAISE;
            auto_state_enter_tick_ = now;
        }
        break;
    case STEP_DESCEND_RAISE:
        if ((now - auto_state_enter_tick_) >= TIME_DESC_RAISE)
        {
            climb_state_ = STEP_DESCEND_DONE;
            auto_state_enter_tick_ = now;
            auto_running_ = 0;
        }
        break;
    case STEP_DESCEND_DONE:
        auto_running_ = 0;
        descend_mode_ = 0;
        break;

    // ---------- 上台阶自动流程 ----------
    case STEP_SETUP:
        if ((now - auto_state_enter_tick_) >= TIME_SETUP)
        {
            climb_state_ = STEP_WAIT_TRIGGER;
            auto_state_enter_tick_ = now;
        }
        break;
    case STEP_WAIT_TRIGGER:
        if ((now - auto_state_enter_tick_) >= WAIT_TRIGGER_FORWARD_TIME_MS)
        {
            climb_state_ = STEP_TOUCH_DOWN;
            auto_state_enter_tick_ = now;
        }
        break;
    case STEP_TOUCH_DOWN:
        if ((now - auto_state_enter_tick_) >= TIME_TOUCH)
        {
            climb_state_ = STEP_GLOBAL_LIFT;
            auto_state_enter_tick_ = now;
        }
        break;
    case STEP_GLOBAL_LIFT:
        if ((now - auto_state_enter_tick_) >= TIME_LIFT)
        {
            climb_state_ = STEP_DRIVE_FWD;
            auto_state_enter_tick_ = now;
        }
        break;
    case STEP_DRIVE_FWD:
        if (IsWheelAngleDone())
        {
            climb_state_ = STEP_RETRACT;
            auto_state_enter_tick_ = now;
        }
        break;
    case STEP_RETRACT:
        if ((now - auto_state_enter_tick_) >= TIME_RETRACT)
        {
            climb_state_ = STEP_DONE;
            auto_state_enter_tick_ = now;
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

void ClimbingController::ManualNext(void)
{
    // 手动调试: 按上台阶状态顺序推进一步
    init_pose_active_ = 0;
    auto_running_ = 0;
    descend_mode_ = 0;
    state_tick_ = HAL_GetTick();

    switch (climb_state_)
    {
    case STEP_IDLE:
        climb_state_ = STEP_SETUP;
        break;
    case STEP_SETUP:
        climb_state_ = STEP_WAIT_TRIGGER;
        break;
    case STEP_WAIT_TRIGGER:
        climb_state_ = STEP_TOUCH_DOWN;
        break;
    case STEP_TOUCH_DOWN:
        climb_state_ = STEP_GLOBAL_LIFT;
        break;
    case STEP_GLOBAL_LIFT:
        climb_state_ = STEP_DRIVE_FWD;
        break;
    case STEP_DRIVE_FWD:
        climb_state_ = STEP_RETRACT;
        break;
    case STEP_RETRACT:
        climb_state_ = STEP_DONE;
        break;
    case STEP_DONE:
        climb_state_ = STEP_IDLE;
        break;
    default:
        climb_state_ = STEP_IDLE;
        break;
    }
}

void ClimbingController::DescendManualNext(void)
{
    // 手动调试: 按下台阶状态顺序推进一步
    init_pose_active_ = 0;
    auto_running_ = 0;
    descend_mode_ = 1;
    state_tick_ = HAL_GetTick();

    if (climb_state_ < STEP_DESCEND_SETUP || climb_state_ > STEP_DESCEND_DONE)
    {
        desc_start_pos_front_ = motor_lift_front_.Get_Now_Angle();
        desc_start_pos_rear_ = motor_lift_rear_.Get_Now_Angle();
        climb_state_ = STEP_DESCEND_SETUP;
        return;
    }

    switch (climb_state_)
    {
    case STEP_DESCEND_SETUP:
        climb_state_ = STEP_DESCEND_WAIT_TRIGGER;
        break;
    case STEP_DESCEND_WAIT_TRIGGER:
        climb_state_ = STEP_DESCEND_TOUCH;
        break;
    case STEP_DESCEND_TOUCH:
        climb_state_ = STEP_DESCEND_GLOBAL_DOWN;
        break;
    case STEP_DESCEND_GLOBAL_DOWN:
        climb_state_ = STEP_DESCEND_DRIVE;
        break;
    case STEP_DESCEND_DRIVE:
        climb_state_ = STEP_DESCEND_RAISE;
        break;
    case STEP_DESCEND_RAISE:
        climb_state_ = STEP_DESCEND_DONE;
        break;
    case STEP_DESCEND_DONE:
    default:
        climb_state_ = STEP_IDLE;
        descend_mode_ = 0;
        break;
    }
}

void ClimbingController::ManualReset(void)
{
    // 回到 IDLE, 并让轮子目标锁定在当前角度
    init_pose_active_ = 0;
    auto_running_ = 0;
    descend_mode_ = 0;
    climb_state_ = STEP_IDLE;
    state_tick_ = HAL_GetTick();

    motor_wheel_l_.Set_Target_Angle(motor_wheel_l_.Get_Now_Angle());
    motor_wheel_r_.Set_Target_Angle(motor_wheel_r_.Get_Now_Angle());
}

void ClimbingController::ManualGoto(ClimbingState_e state)
{
    // 直接跳状态, 非 DRIVE 状态时锁住轮子目标
    init_pose_active_ = 0;
    auto_running_ = 0;
    descend_mode_ = 0;
    climb_state_ = state;
    state_tick_ = HAL_GetTick();

    if (state != STEP_DRIVE_FWD && state != STEP_DESCEND_DRIVE)
    {
        motor_wheel_l_.Set_Target_Angle(motor_wheel_l_.Get_Now_Angle());
        motor_wheel_r_.Set_Target_Angle(motor_wheel_r_.Get_Now_Angle());
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
    motor_wheel_l_.Set_Target_Angle(motor_wheel_l_.Get_Now_Angle());
    motor_wheel_r_.Set_Target_Angle(motor_wheel_r_.Get_Now_Angle());
}

void ClimbingController::SetChassisExternalControl(uint8_t enable)
{
    // 1: 底盘外部接管, 0: 状态机内部控制
    chassis_external_control_ = (enable != 0U) ? 1U : 0U;
}

void ClimbingController::TaskEntry1ms(void)
{
    // 1ms 主控制入口:
    // 状态变化处理 -> PID/斜坡参数切换 -> 软零点记录 -> 目标更新 -> 控制执行 -> 补偿发送
    uint32_t current_time = HAL_GetTick();
    uint8_t state_changed = (climb_state_ != prev_climb_state_);
    HandleStateTransition(current_time, state_changed);
    UpdatePidAndSlopeByState();

    if (is_zero_recorded_ == 0)
    {
        // 上电首次记录软零点, 并同步斜坡内部状态
        start_pos_front_ = motor_lift_front_.Get_Now_Angle();
        start_pos_rear_ = motor_lift_rear_.Get_Now_Angle();

        slope_front_pos_.Set_Now_Real(start_pos_front_);
        slope_rear_pos_.Set_Now_Real(start_pos_rear_);
        slope_front_pos_.Set_Target(start_pos_front_);
        slope_rear_pos_.Set_Target(start_pos_rear_);
        slope_wheel_l_angle_.Set_Now_Real(motor_wheel_l_.Get_Now_Angle());
        slope_wheel_r_angle_.Set_Now_Real(motor_wheel_r_.Get_Now_Angle());
        slope_wheel_l_angle_.Set_Target(motor_wheel_l_.Get_Now_Angle());
        slope_wheel_r_angle_.Set_Target(motor_wheel_r_.Get_Now_Angle());

        slope_front_pos_.TIM_Calculate_PeriodElapsedCallback();
        slope_rear_pos_.TIM_Calculate_PeriodElapsedCallback();
        slope_wheel_l_angle_.TIM_Calculate_PeriodElapsedCallback();
        slope_wheel_r_angle_.TIM_Calculate_PeriodElapsedCallback();

        is_zero_recorded_ = 1;
    }

    if (init_pose_active_ != 0U)
    {
        slope_front_pos_.Set_Target(start_pos_front_ + POS_FRONT_Init);
        slope_rear_pos_.Set_Target(start_pos_rear_ + POS_REAR_Init);
        slope_wheel_l_angle_.Set_Target(motor_wheel_l_.Get_Now_Angle());
        slope_wheel_r_angle_.Set_Target(motor_wheel_r_.Get_Now_Angle());

        UpdateChassisControl();
        RunLiftAndWheelControl();
        ApplyGravityCompensationAndSend();
        return;
    }

    UpdateStateTargets(current_time);
    UpdateChassisControl();
    RunLiftAndWheelControl();
    ApplyGravityCompensationAndSend();
}

ClimbingState_e ClimbingController::GetState(void) const
{
    return climb_state_;
}

Class_Motor_C620 &ClimbingController::GetMotorLiftFront(void)
{
    return motor_lift_front_;
}

Class_Motor_C620 &ClimbingController::GetMotorLiftRear(void)
{
    return motor_lift_rear_;
}

Class_Motor_C620 &ClimbingController::GetMotorWheelL(void)
{
    return motor_wheel_l_;
}

Class_Motor_C620 &ClimbingController::GetMotorWheelR(void)
{
    return motor_wheel_r_;
}

Class_Slope &ClimbingController::GetSlopeFrontPos(void)
{
    return slope_front_pos_;
}

Class_Slope &ClimbingController::GetSlopeRearPos(void)
{
    return slope_rear_pos_;
}
