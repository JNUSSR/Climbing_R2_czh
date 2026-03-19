/**
 * @file Climbing_Task.cpp
 * @brief 20cm 台阶攀爬任务 (Soft Zero 版)
 */

#include "Climbing_Task.h"
#include "dvc_motor.h"
#include "alg_slope.h"
#include "wheelchassis.h"

// 外部CAN句柄声明 (定义在can.c中)
extern CAN_HandleTypeDef hcan1;

// ---------------------------------------------
// 全局变量
// ---------------------------------------------
ClimbingState_e g_climbState = STEP_IDLE;
uint32_t g_stateTick = 0;

// 电机对象
Class_Motor_C620 Motor_Lift_Front;
Class_Motor_C620 Motor_Lift_Rear;
Class_Motor_C620 Motor_Wheel_L;
Class_Motor_C620 Motor_Wheel_R;
extern Class_Motor_C620 G_Chassis_Motor[4];

// 斜坡规划器 (位置模式) - 移除static以便调试
Class_Slope Slope_Front_Pos;
Class_Slope Slope_Rear_Pos;

// 软零点记录
static uint8_t is_zero_recorded = 0;
static float Start_Pos_Front = 0.0f;
static float Start_Pos_Rear = 0.0f;
static float Desc_Start_Pos_Front = 0.0f;
static float Desc_Start_Pos_Rear = 0.0f;
static uint8_t is_lift_pid_mode = 0;
static ClimbingState_e prev_climb_state = STEP_IDLE;

// 自动连贯动作状态
static uint8_t g_auto_running = 0;
static uint32_t g_auto_state_enter_tick = 0;
static uint8_t g_descend_mode = 0;

static float Clamp_C620_Output(float out)
{
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

static void Apply_Motor_Output_With_Comp(Class_Motor_C620 &motor, float comp, uint8_t can_byte_index)
{
    float final_out = motor.Get_Out() + comp;
    final_out = Clamp_C620_Output(final_out);
    motor.Set_Out(final_out);

    int16_t out_int = (int16_t)final_out;
    CAN1_0x200_Tx_Data[can_byte_index] = (uint8_t)(out_int >> 8);
    CAN1_0x200_Tx_Data[can_byte_index + 1] = (uint8_t)(out_int);
}

static void Compute_Gravity_Compensation(float *front_comp, float *rear_comp)
{
    *front_comp = 0.0f;
    *rear_comp = 0.0f;

    switch (g_climbState)
    {
    case STEP_DESCEND_SETUP:
    case  STEP_DESCEND_WAIT_TRIGGER:
    case STEP_DESCEND_TOUCH:
    case STEP_DESCEND_GLOBAL_DOWN:
    case STEP_DESCEND_DRIVE:
    case STEP_DESCEND_RAISE:
        *front_comp = GRAVITY_COMPENSATION_FRONT;
        *rear_comp = GRAVITY_COMPENSATION_REAR;
        break;

    case STEP_SETUP:
    case STEP_WAIT_TRIGGER:
    case STEP_TOUCH_DOWN:
        *front_comp = GRAVITY_COMPENSATION_FRONT;
        *rear_comp = GRAVITY_COMPENSATION_REAR;
        break;

    case STEP_GLOBAL_LIFT:
        *front_comp = COMP_FRONT_LIFT;
        *rear_comp = COMP_REAR_LIFT;
        break;

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

static void Handle_State_Transition(uint32_t current_time, uint8_t state_changed)
{
    if (state_changed == 0)
    {
        return;
    }

    g_stateTick = current_time;

    if ((prev_climb_state == STEP_WAIT_TRIGGER || prev_climb_state == STEP_DESCEND_WAIT_TRIGGER) &&
        g_climbState != STEP_WAIT_TRIGGER &&
        g_climbState !=   STEP_DESCEND_WAIT_TRIGGER)
    {
        wheelChassis_StopImmediate();
    }

    Slope_Front_Pos.Set_Now_Real(Motor_Lift_Front.Get_Now_Angle());
    Slope_Rear_Pos.Set_Now_Real(Motor_Lift_Rear.Get_Now_Angle());

    prev_climb_state = g_climbState;
}

static void Update_Pid_And_Slope_By_State(void)
{
    uint8_t is_descend_state = (g_climbState >= STEP_DESCEND_SETUP && g_climbState <= STEP_DESCEND_DONE); // 是否处于下台阶状态
    uint8_t is_lift_pid_state = 0;
    uint8_t is_slope_group_b = 0;
    uint8_t is_slope_group_a = 0;

    if (is_descend_state)
    {
        is_lift_pid_state = (g_climbState == STEP_DESCEND_TOUCH ||
                             g_climbState == STEP_DESCEND_GLOBAL_DOWN ||
                             g_climbState == STEP_DESCEND_DRIVE ||
                             g_climbState == STEP_DESCEND_RAISE);
        is_slope_group_a = (g_climbState == STEP_DESCEND_SETUP || g_climbState == STEP_DESCEND_WAIT_TRIGGER);
        is_slope_group_b = (g_climbState == STEP_DESCEND_TOUCH ||
                            g_climbState == STEP_DESCEND_GLOBAL_DOWN ||
                            g_climbState == STEP_DESCEND_DRIVE ||
                            g_climbState == STEP_DESCEND_RAISE);
    }
    else
    {
        is_lift_pid_state = (g_climbState == STEP_GLOBAL_LIFT || g_climbState == STEP_DRIVE_FWD);
        is_slope_group_b = (g_climbState == STEP_TOUCH_DOWN || g_climbState == STEP_GLOBAL_LIFT);
        is_slope_group_a = (g_climbState == STEP_SETUP || g_climbState == STEP_RETRACT);
    }

    if (is_lift_pid_state)
    {
        if (is_lift_pid_mode == 0)
        {
            Motor_Lift_Front.PID_Omega.Set_K_P(PID_FRONT_OMEGA_KP_LIFT);
            Motor_Lift_Front.PID_Angle.Set_K_P(PID_FRONT_ANGLE_KP_LIFT);
            Motor_Lift_Rear.PID_Omega.Set_K_P(PID_REAR_OMEGA_KP_LIFT);
            Motor_Lift_Rear.PID_Angle.Set_K_P(PID_REAR_ANGLE_KP_LIFT);
            Motor_Lift_Rear.PID_Omega.Set_K_I(PID_REAR_OMEGA_KI_LIFT);
            Motor_Lift_Rear.PID_Angle.Set_K_I(PID_REAR_ANGLE_KI_LIFT);
            is_lift_pid_mode = 1;
        }
    }
    else
    {
        if (is_lift_pid_mode == 1)
        {
            Motor_Lift_Front.PID_Omega.Set_K_P(PID_FRONT_OMEGA_KP_NORMAL);
            Motor_Lift_Front.PID_Angle.Set_K_P(PID_FRONT_ANGLE_KP_NORMAL);
            Motor_Lift_Rear.PID_Omega.Set_K_P(PID_REAR_OMEGA_KP_NORMAL);
            Motor_Lift_Rear.PID_Angle.Set_K_P(PID_REAR_ANGLE_KP_NORMAL);
            Motor_Lift_Rear.PID_Omega.Set_K_I(PID_REAR_OMEGA_KI_NORMAL);
            Motor_Lift_Rear.PID_Angle.Set_K_I(PID_REAR_ANGLE_KI_NORMAL);
            is_lift_pid_mode = 0;
        }
    }

    if (is_slope_group_b)
    {
        if (is_descend_state)
        {
            Slope_Front_Pos.Set_Increase_Value(FRONT_SLOPE_STEP_DESC_G2);
            Slope_Front_Pos.Set_Decrease_Value(FRONT_SLOPE_STEP_DESC_G2);
            Slope_Rear_Pos.Set_Increase_Value(REAR_SLOPE_STEP_DESC_G2);
            Slope_Rear_Pos.Set_Decrease_Value(REAR_SLOPE_STEP_DESC_G2);
        }
        else
        {
            Slope_Front_Pos.Set_Increase_Value(FRONT_SLOPE_STEP_TOUCH_LIFT);
            Slope_Front_Pos.Set_Decrease_Value(FRONT_SLOPE_STEP_TOUCH_LIFT);
            Slope_Rear_Pos.Set_Increase_Value(REAR_SLOPE_STEP_TOUCH_LIFT);
            Slope_Rear_Pos.Set_Decrease_Value(REAR_SLOPE_STEP_TOUCH_LIFT);
        }
    }
    else if (is_slope_group_a)
    {
        if (is_descend_state)
        {
            Slope_Front_Pos.Set_Increase_Value(FRONT_SLOPE_STEP_DESC_G1);
            Slope_Front_Pos.Set_Decrease_Value(FRONT_SLOPE_STEP_DESC_G1);
            Slope_Rear_Pos.Set_Increase_Value(REAR_SLOPE_STEP_DESC_G1);
            Slope_Rear_Pos.Set_Decrease_Value(REAR_SLOPE_STEP_DESC_G1);
        }
        else
        {
            Slope_Front_Pos.Set_Increase_Value(FRONT_SLOPE_STEP_SETUP_RETRACT);
            Slope_Front_Pos.Set_Decrease_Value(FRONT_SLOPE_STEP_SETUP_RETRACT);
            Slope_Rear_Pos.Set_Increase_Value(REAR_SLOPE_STEP_SETUP_RETRACT);
            Slope_Rear_Pos.Set_Decrease_Value(REAR_SLOPE_STEP_SETUP_RETRACT);
        }
    }
}

static void Update_State_Targets(uint32_t current_time)
{
    switch (g_climbState)
    {
    case STEP_IDLE:
        Slope_Front_Pos.Set_Target(Start_Pos_Front);
        Slope_Rear_Pos.Set_Target(Start_Pos_Rear);
        Motor_Wheel_L.Set_Target_Omega(0.0f);
        Motor_Wheel_R.Set_Target_Omega(0.0f);
        break;
    // ========== 上台阶流程 ==========
    case STEP_SETUP: 
        Slope_Front_Pos.Set_Target(Start_Pos_Front + POS_FRONT_RETRACT);
        Slope_Rear_Pos.Set_Target(Start_Pos_Rear + POS_REAR_RETRACT);
        break;

    case STEP_WAIT_TRIGGER:
    {
        MotionControl wait_motion = {0.0f, 0.0f, 0.0f};
        if (g_auto_running && (current_time - g_stateTick) < WAIT_TRIGGER_FORWARD_TIME_MS)
        {
            wait_motion.Vx = WAIT_TRIGGER_FORWARD_VX;
        }
        wheelChassis_Task(wait_motion);
        break;
    }

    case STEP_TOUCH_DOWN:
        Slope_Front_Pos.Set_Target(Start_Pos_Front + POS_FRONT_TOUCH);
        Slope_Rear_Pos.Set_Target(Start_Pos_Rear + POS_REAR_TOUCH);
        break;

    case STEP_GLOBAL_LIFT:
        Slope_Front_Pos.Set_Target(Start_Pos_Front + POS_FRONT_LIFT);
        Slope_Rear_Pos.Set_Target(Start_Pos_Rear + POS_REAR_LIFT);
        break;

    case STEP_DRIVE_FWD:
        Motor_Wheel_L.Set_Target_Omega(SPEED_DRIVE_FWD);
        Motor_Wheel_R.Set_Target_Omega(-SPEED_DRIVE_FWD);
        break;

    case STEP_RETRACT:
        Slope_Front_Pos.Set_Target(Start_Pos_Front + POS_FRONT_FINAL);
        Slope_Rear_Pos.Set_Target(Start_Pos_Rear + POS_REAR_FINAL);
        Motor_Wheel_L.Set_Target_Omega(0.0f);
        Motor_Wheel_R.Set_Target_Omega(0.0f);
        break;

    case STEP_DONE:
        Motor_Wheel_L.Set_Target_Omega(0.0f);
        Motor_Wheel_R.Set_Target_Omega(0.0f);
        break;

    // ========== 下台阶流程 ==========
    case STEP_DESCEND_SETUP:
        Slope_Front_Pos.Set_Target(Desc_Start_Pos_Front + POS_FRONT_RETRACT);
        Slope_Rear_Pos.Set_Target(Desc_Start_Pos_Rear + POS_REAR_RETRACT);
        Motor_Wheel_L.Set_Target_Omega(0.0f);
        Motor_Wheel_R.Set_Target_Omega(0.0f);
        break;

    case  STEP_DESCEND_WAIT_TRIGGER:
    {
        MotionControl wait_motion = {0.0f, 0.0f, 0.0f};
        if (g_descend_mode && g_auto_running && (current_time - g_stateTick) < DESCEND_WAIT_TIME_MS)
        {
            wait_motion.Vx = DESCEND_WAIT_FORWARD_VX;
        }
        wheelChassis_Task(wait_motion);
        Motor_Wheel_L.Set_Target_Omega(0.0f);
        Motor_Wheel_R.Set_Target_Omega(0.0f);
        break;
    }

    case STEP_DESCEND_TOUCH:
        Slope_Front_Pos.Set_Target(Desc_Start_Pos_Front + DESCEND_FRONT_TOUCH_TARGET);
        Slope_Rear_Pos.Set_Target(Desc_Start_Pos_Rear + DESCEND_REAR_TOUCH_TARGET);
        Motor_Wheel_L.Set_Target_Omega(0.0f);
        Motor_Wheel_R.Set_Target_Omega(0.0f);
        break;

    case STEP_DESCEND_GLOBAL_DOWN:
        Slope_Front_Pos.Set_Target(Desc_Start_Pos_Front + DESCEND_FRONT_GLOBAL_DOWN_TARGET);
        Slope_Rear_Pos.Set_Target(Desc_Start_Pos_Rear + DESCEND_REAR_GLOBAL_DOWN_TARGET);
        Motor_Wheel_L.Set_Target_Omega(0.0f);
        Motor_Wheel_R.Set_Target_Omega(0.0f);
        break;

    case STEP_DESCEND_DRIVE:
        Motor_Wheel_L.Set_Target_Omega(SPEED_DRIVE_FWD);
        Motor_Wheel_R.Set_Target_Omega(-SPEED_DRIVE_FWD);
        break;

    case STEP_DESCEND_RAISE:
        Slope_Front_Pos.Set_Target(Desc_Start_Pos_Front);
        Slope_Rear_Pos.Set_Target(Desc_Start_Pos_Rear);
        Motor_Wheel_L.Set_Target_Omega(0.0f);
        Motor_Wheel_R.Set_Target_Omega(0.0f);
        break;

    case STEP_DESCEND_DONE:
        Motor_Wheel_L.Set_Target_Omega(0.0f);
        Motor_Wheel_R.Set_Target_Omega(0.0f);
        break;
    }
}

static void Update_Chassis_Control(void)
{
    if (g_climbState != STEP_WAIT_TRIGGER && g_climbState !=  STEP_DESCEND_WAIT_TRIGGER)
    {
        MotionControl stop_motion = {0.0f, 0.0f, 0.0f};
        wheelChassis_Task(stop_motion);
    }

    G_Chassis_Motor[0].TIM_PID_PeriodElapsedCallback();
    G_Chassis_Motor[1].TIM_PID_PeriodElapsedCallback();
    G_Chassis_Motor[2].TIM_PID_PeriodElapsedCallback();
    G_Chassis_Motor[3].TIM_PID_PeriodElapsedCallback();
}

static void Run_Lift_And_Wheel_Control(void)
{
    Slope_Front_Pos.TIM_Calculate_PeriodElapsedCallback();
    Slope_Rear_Pos.TIM_Calculate_PeriodElapsedCallback();

    Motor_Lift_Front.Set_Target_Angle(Slope_Front_Pos.Get_Out());
    Motor_Lift_Rear.Set_Target_Angle(Slope_Rear_Pos.Get_Out());

    Motor_Lift_Front.TIM_PID_PeriodElapsedCallback();
    Motor_Lift_Rear.TIM_PID_PeriodElapsedCallback();
    Motor_Wheel_L.TIM_PID_PeriodElapsedCallback();
    Motor_Wheel_R.TIM_PID_PeriodElapsedCallback();
}

static void Apply_Gravity_Compensation_And_Send(void)
{
    float target_comp_front = 0.0f;
    float target_comp_rear = 0.0f;
    Compute_Gravity_Compensation(&target_comp_front, &target_comp_rear);

    Apply_Motor_Output_With_Comp(Motor_Lift_Front, target_comp_front, 0);
    Apply_Motor_Output_With_Comp(Motor_Lift_Rear, target_comp_rear, 2);

    TIM_CAN_PeriodElapsedCallback();
}

// ---------------------------------------------
// 初始化
// ---------------------------------------------
void Climbing_Task_Init(void)
{
    // 初始化斜坡: 真实值优先模式（默认使用组A）
    Slope_Front_Pos.Init(FRONT_SLOPE_STEP_SETUP_RETRACT, FRONT_SLOPE_STEP_SETUP_RETRACT, Slope_First_REAL);
    Slope_Rear_Pos.Init(REAR_SLOPE_STEP_SETUP_RETRACT, REAR_SLOPE_STEP_SETUP_RETRACT, Slope_First_REAL);

    // 电机初始化 (ID请根据实际修改)
    Motor_Lift_Front.Init(&hcan1, CAN_Motor_ID_0x201, Control_Method_ANGLE);
    Motor_Lift_Rear.Init(&hcan1, CAN_Motor_ID_0x202, Control_Method_ANGLE);
    
    Motor_Wheel_L.Init(&hcan1, CAN_Motor_ID_0x203, Control_Method_OMEGA);
    Motor_Wheel_R.Init(&hcan1, CAN_Motor_ID_0x204, Control_Method_OMEGA);

    // 轮式底盘初始化（用于 STEP_WAIT_TRIGGER 阶段前进）
    wheelChassis_Init();

    //抬升机构PID初始化
    // 前腿 (负载小) - 修正PID参数
    Motor_Lift_Front.PID_Omega.Init(PID_FRONT_OMEGA_KP_NORMAL, 5.0f, 0.0f, 0.0f, 10000.0f, 12000.0f);
    Motor_Lift_Front.PID_Angle.Init(PID_FRONT_ANGLE_KP_NORMAL, 0.5f, 0.0f, 0.0f, 10000.0f, 12000.0f);
 
    // Motor_Lift_Front.PID_Omega.Init(0.0f, 0.0f, 0.0f, 0.0f, 10000.0f, 12000.0f);
    // Motor_Lift_Front.PID_Angle.Init(0.0f, 0.0f, 0.0f, 0.0f, 10000.0f, 12000.0f);
    // 后腿 (负载大) - 修正PID参数
    Motor_Lift_Rear.PID_Omega.Init(PID_REAR_OMEGA_KP_NORMAL, 5.0f, 0.0f, 0.0f, 12000.0f, 14000.0f);
    Motor_Lift_Rear.PID_Angle.Init(PID_REAR_ANGLE_KP_NORMAL, 0.5f, 0.0f, 0.0f, 12000.0f, 14000.0f);
    //  Motor_Lift_Rear.PID_Omega.Init(0.0f, 0.0f, 0.0f, 0.0f, 12000.0f, 14000.0f);  
    //  Motor_Lift_Rear.PID_Angle.Init(0.0f, 0.5f, 0.0f, 0.0f, 12000.0f, 14000.0f);  

    //轮子PID初始化
    Motor_Wheel_L.PID_Omega.Init(85.3f, 97.3f, 0.0f, 0.0f, 10000.0f, 12000.0f);
    Motor_Wheel_R.PID_Omega.Init(85.3f, 97.3f, 0.0f, 0.0f, 10000.0f, 12000.0f);

    g_climbState = STEP_IDLE;
    is_zero_recorded = 0;
    is_lift_pid_mode = 0;
    g_descend_mode = 0;
}

/**
 * @brief 外部信号触发下一步
 */
void Climbing_Next_Step(void)
{
    if (g_climbState == STEP_IDLE) 
        g_climbState = STEP_SETUP;
    else if (g_climbState == STEP_WAIT_TRIGGER) 
        g_climbState = STEP_TOUCH_DOWN; 
}

/**
 * @brief 启动自动连贯动作（一次完整流程）
 *
 * 流程: STEP_SETUP -> STEP_WAIT_TRIGGER -> STEP_TOUCH_DOWN -> STEP_GLOBAL_LIFT -> STEP_DRIVE_FWD -> STEP_RETRACT -> STEP_DONE
 * 时间由 Climbing_task.h 中 TIME_* 参数控制。
 */
void Climbing_Auto_Start(void)
{
    g_auto_running = 1;
    g_descend_mode = 0;
    g_climbState = STEP_SETUP;
    g_auto_state_enter_tick = HAL_GetTick();
}

/**
 * @brief 启动下台阶自动流程（一次）
 */
void Climbing_Descend_Auto_Start(void)
{
    g_auto_running = 1;
    g_descend_mode = 1;
    //记录下台阶软零点
    Desc_Start_Pos_Front = Motor_Lift_Front.Get_Now_Angle();
    Desc_Start_Pos_Rear = Motor_Lift_Rear.Get_Now_Angle();
    g_climbState = STEP_DESCEND_SETUP;
    g_auto_state_enter_tick = HAL_GetTick();
}

/**
 * @brief 查询自动流程是否在运行
 */
uint8_t Climbing_Is_Auto_Running(void)
{
    return g_auto_running;
}

/**
 * @brief 自动状态推进器（1ms调用）
 *
 * 注意：本函数只负责“切状态”，具体控制执行仍由 Climbing_Task_Entry() 完成。
 */
void Climbing_Auto_Task_1ms(void)
{
    uint32_t now = HAL_GetTick();

    if (g_auto_running == 0)
    {
        return;
    }

    switch (g_climbState)
    {
        case STEP_DESCEND_SETUP:
            if ((now - g_auto_state_enter_tick) >= TIME_DESC_SETUP)
            {
                g_climbState =STEP_DESCEND_WAIT_TRIGGER;
                g_auto_state_enter_tick = now;
            }
            break;

        case  STEP_DESCEND_WAIT_TRIGGER:
            if ((now - g_auto_state_enter_tick) >= DESCEND_WAIT_TIME_MS)
            {
                g_climbState = STEP_DESCEND_TOUCH;
                g_auto_state_enter_tick = now;
            }
            break;

        case STEP_DESCEND_TOUCH:
            if ((now - g_auto_state_enter_tick) >= TIME_DESC_TOUCH)
            {
                g_climbState = STEP_DESCEND_GLOBAL_DOWN;
                g_auto_state_enter_tick = now;
            }
            break;

        case STEP_DESCEND_GLOBAL_DOWN:
            if ((now - g_auto_state_enter_tick) >= TIME_DESC_GLOBAL_DOWN)
            {
                g_climbState = STEP_DESCEND_DRIVE;
                g_auto_state_enter_tick = now;
            }
            break;

        case STEP_DESCEND_DRIVE:
            if ((now - g_auto_state_enter_tick) >= DESCEND_DRIVE_TIME_MS)
            {
                g_climbState = STEP_DESCEND_RAISE;
                g_auto_state_enter_tick = now;
            }
            break;

        case STEP_DESCEND_RAISE:
            if ((now - g_auto_state_enter_tick) >= TIME_DESC_RAISE)
            {
                g_climbState = STEP_DESCEND_DONE;
                g_auto_state_enter_tick = now;
                g_auto_running = 0;
            }
            break;

        case STEP_DESCEND_DONE:
            g_auto_running = 0;
            g_descend_mode = 0;
            break;

        case STEP_SETUP:
            if ((now - g_auto_state_enter_tick) >= TIME_SETUP)
            {
                g_climbState = STEP_WAIT_TRIGGER;
                g_auto_state_enter_tick = now;
            }
            break;

        case STEP_WAIT_TRIGGER:
            if ((now - g_auto_state_enter_tick) >= WAIT_TRIGGER_FORWARD_TIME_MS)
            {
                g_climbState = STEP_TOUCH_DOWN;
                g_auto_state_enter_tick = now;
            }
            break;

        case STEP_TOUCH_DOWN:
            if ((now - g_auto_state_enter_tick) >= TIME_TOUCH)
            {
                g_climbState = STEP_GLOBAL_LIFT;
                g_auto_state_enter_tick = now;
            }
            break;

        case STEP_GLOBAL_LIFT:
            if ((now - g_auto_state_enter_tick) >= TIME_LIFT)
            {
                g_climbState = STEP_DRIVE_FWD;
                g_auto_state_enter_tick = now;
            }
            break;

        case STEP_DRIVE_FWD:
            if ((now - g_auto_state_enter_tick) >= TIME_DRIVE)
            {
                g_climbState = STEP_RETRACT;
                g_auto_state_enter_tick = now;
            }
            break;

        case STEP_RETRACT:
            if ((now - g_auto_state_enter_tick) >= TIME_RETRACT)
            {
                g_climbState = STEP_DONE;
                g_auto_state_enter_tick = now;
                g_auto_running = 0;
            }
            break;

        case STEP_DONE:
            g_auto_running = 0;
            g_descend_mode = 0;
            break;

        default:
            // 运行中如果被切到其他状态，自动流程结束，避免状态冲突
            g_auto_running = 0;
            g_descend_mode = 0;
            break;
    }
}

// ========== 手动调试接口实现 ==========

/**
 * @brief 手动下一步 - 按状态机顺序跳转
 */
void Climbing_Manual_Next(void)
{
    g_auto_running = 0;
    g_descend_mode = 0;
    g_stateTick = HAL_GetTick();  // 重置计时
    
    switch (g_climbState)
    {
        case STEP_IDLE:         g_climbState = STEP_SETUP;        break;
        case STEP_SETUP:        g_climbState = STEP_WAIT_TRIGGER; break;
        case STEP_WAIT_TRIGGER: g_climbState = STEP_TOUCH_DOWN;   break;
        case STEP_TOUCH_DOWN:   g_climbState = STEP_GLOBAL_LIFT;  break;
        case STEP_GLOBAL_LIFT:  g_climbState = STEP_DRIVE_FWD;    break;
        case STEP_DRIVE_FWD:    g_climbState = STEP_RETRACT;      break;
        case STEP_RETRACT:      g_climbState = STEP_DONE;         break;
        case STEP_DONE:         g_climbState = STEP_IDLE;         break;
        default:                g_climbState = STEP_IDLE;         break;
    }
}

/**
 * @brief 下台阶手动下一步（0x91）
 *
 * 行为与 0x10 类似：每次调用按下台阶状态顺序推进一步。
 */
void Climbing_Descend_Manual_Next(void)
{
    g_auto_running = 0;
    g_descend_mode = 1;
    g_stateTick = HAL_GetTick();

    // 第一次进入下台阶手动模式时，记录当前位置作为相对起点
    if (g_climbState < STEP_DESCEND_SETUP || g_climbState > STEP_DESCEND_DONE)
    {
        Desc_Start_Pos_Front = Motor_Lift_Front.Get_Now_Angle();
        Desc_Start_Pos_Rear = Motor_Lift_Rear.Get_Now_Angle();
        g_climbState = STEP_DESCEND_SETUP;
        return;
    }

    switch (g_climbState)
    {
    case STEP_DESCEND_SETUP:
        g_climbState =STEP_DESCEND_WAIT_TRIGGER;
        break;
    case  STEP_DESCEND_WAIT_TRIGGER:
        g_climbState = STEP_DESCEND_TOUCH;
        break;
    case STEP_DESCEND_TOUCH:
        g_climbState = STEP_DESCEND_GLOBAL_DOWN;
        break;
    case STEP_DESCEND_GLOBAL_DOWN:
        g_climbState = STEP_DESCEND_DRIVE;
        break;
    case STEP_DESCEND_DRIVE:
        g_climbState = STEP_DESCEND_RAISE;
        break;
    case STEP_DESCEND_RAISE:
        g_climbState = STEP_DESCEND_DONE;
        break;
    case STEP_DESCEND_DONE:
    default:
        g_climbState = STEP_IDLE;
        g_descend_mode = 0;
        break;
    }
}

/**
 * @brief 复位到IDLE状态
 */
void Climbing_Manual_Reset(void)
{
    g_auto_running = 0;
    g_descend_mode = 0;
    g_climbState = STEP_IDLE;
    g_stateTick = HAL_GetTick();
    
    // 停止轮子
    Motor_Wheel_L.Set_Target_Omega(0.0f);
    Motor_Wheel_R.Set_Target_Omega(0.0f);
}

/**
 * @brief 直接跳转到指定状态
 */
void Climbing_Manual_Goto(ClimbingState_e state)
{
    g_auto_running = 0;
    g_descend_mode = 0;
    g_climbState = state;
    g_stateTick = HAL_GetTick();
    
    // 如果不是驱动状态，停止轮子
    if (state != STEP_DRIVE_FWD) {
        Motor_Wheel_L.Set_Target_Omega(0.0f);
        Motor_Wheel_R.Set_Target_Omega(0.0f);
    }
}

/**
 * @brief 紧急停止 - 所有电机输出归零
 */
void Climbing_Emergency_Stop(void)
{
    g_auto_running = 0;
    g_descend_mode = 0;
    g_climbState = STEP_IDLE;
    
    Motor_Lift_Front.Set_Out(0.0f);
    Motor_Lift_Rear.Set_Out(0.0f);
    Motor_Wheel_L.Set_Out(0.0f);
    Motor_Wheel_R.Set_Out(0.0f);
    Motor_Wheel_L.Set_Target_Omega(0.0f);
    Motor_Wheel_R.Set_Target_Omega(0.0f);
}

// ---------------------------------------------
// 任务主体 (1ms 调用一次)
// ---------------------------------------------
void Climbing_Task_Entry(void)
{
    uint32_t current_time = HAL_GetTick();
    uint8_t state_changed = (g_climbState != prev_climb_state);
    Handle_State_Transition(current_time, state_changed);
    Update_Pid_And_Slope_By_State();

    // 1. 上电第一刻：记录软零点
    if (is_zero_recorded == 0) {
        Start_Pos_Front = Motor_Lift_Front.Get_Now_Angle();
        Start_Pos_Rear  = Motor_Lift_Rear.Get_Now_Angle();
        
        // 【关键修复】必须同步设置 Now_Real、Target，并手动设置初始 Out
        // 斜坡规划器的 Out 和 Now_Planning 默认是 0，需要强制同步
        Slope_Front_Pos.Set_Now_Real(Start_Pos_Front);
        Slope_Rear_Pos.Set_Now_Real(Start_Pos_Rear);
        Slope_Front_Pos.Set_Target(Start_Pos_Front);
        Slope_Rear_Pos.Set_Target(Start_Pos_Rear);
        
        // 强制执行一次计算，让 Out 同步到当前位置
        Slope_Front_Pos.TIM_Calculate_PeriodElapsedCallback();
        Slope_Rear_Pos.TIM_Calculate_PeriodElapsedCallback();
        
        is_zero_recorded = 1;
    }


    Update_State_Targets(current_time);
    Update_Chassis_Control();
    Run_Lift_And_Wheel_Control();
    Apply_Gravity_Compensation_And_Send();
}
