#ifndef TEST_FEEDBACK_CLIMBING_CONTROLLER_H
#define TEST_FEEDBACK_CLIMBING_CONTROLLER_H

#include "stm32f4xx_hal.h"
#include "dvc_motor.h"
#include "alg_slope.h"

// ==========================================
// 0. 基础换算工具
// ==========================================

#define TASK_FREQ_HZ        (1000.0f)
#define MM_TO_M(x)          ((x) / 1000.0f)
#define TURNS_TO_RADS(x)    ((x) * 2.0f * PI)

// 丝杆导程: 每转一圈对应的直线位移(mm)
#define SCREW_LEAD_MM_PER_TURN   (135.0f)
#define movingmm(x_mm)       TURNS_TO_RADS((x_mm) / SCREW_LEAD_MM_PER_TURN)

// 前/后腿方向修正（-1.0f 反向）
#define FRONT_LEG_DIR        (-1.0f)
#define REAR_LEG_DIR         (1.0f)
#define movingmm_front(x_mm) movingmm((x_mm) * FRONT_LEG_DIR)
#define movingmm_rear(x_mm)  movingmm((x_mm) * REAR_LEG_DIR)

// 避免和 dvc_motor.h 里的同名宏冲突
#define RC_RPM_TO_RADPS(x)  ((x) * 2.0f * PI / 60.0f)

// ==========================================
// 1. 机械位置参数(上/下台阶)  以下都是负值向上，正值向下
// ==========================================
//初始状态
#define POS_FRONT_Init       movingmm_front(-220.0f)
#define POS_REAR_Init        movingmm_rear(-80.0f)

//上20cm台阶
#define POS_FRONT_RETRACT_20cm    movingmm_front(-220.0f)
#define POS_REAR_RETRACT_20cm     movingmm_rear(-10.0f)

#define POS_FRONT_TOUCH_20cm      movingmm_front(-200.0f)
#define POS_REAR_TOUCH_20cm       movingmm_rear(0.0f)

#define POS_FRONT_LIFT_20cm       movingmm_front(20.0f)
#define POS_REAR_LIFT_20cm        movingmm_rear(220.0f)

//上40cm台阶
#define POS_FRONT_RETRACT_40cm    movingmm_front(-420.0f)
#define POS_REAR_RETRACT_40cm     movingmm_rear(-75.0f)

#define POS_FRONT_TOUCH_40cm      movingmm_front(-400.0f)
#define POS_REAR_TOUCH_40cm       movingmm_rear(0.0f)

#define POS_FRONT_LIFT_40cm       movingmm_front(20.0f)
#define POS_REAR_LIFT_40cm        movingmm_rear(413.0f)

#define POS_FRONT_FINAL_20cm movingmm_front(-220.0f)
#define POS_REAR_FINAL_20cm  movingmm_rear(-10.0f)
#define POS_FRONT_FINAL_40cm movingmm_front(-220.0f)
#define POS_REAR_FINAL_40cm  movingmm_rear(-10.0f)

//下20cm台阶
// 下台阶：状态3 触地目标
#define DESCEND_FRONT_TOUCH_TARGET     movingmm_front(420.0f)
#define DESCEND_REAR_TOUCH_TARGET      movingmm_rear(0.0f)

// 下台阶：状态4 全局下降目标
#define DESCEND_FRONT_GLOBAL_DOWN_TARGET movingmm_front(470.0f)
#define DESCEND_REAR_GLOBAL_DOWN_TARGET  movingmm_rear(50.0f)

// 下台阶：状态6 抬起过渡目标
// 前脚保持下层地面接触，后脚保持上层台阶对应高度
#define DESCEND_FRONT_RAISE_TARGET       movingmm_front(0.0f)
#define DESCEND_REAR_RAISE_TARGET        movingmm_rear(-200.0f)

// 下台阶：状态7 脱离台阶，轮子再前移 0.1m
#define WHEEL_TRAVEL_DESCEND_RELEASE_M   (0.1f)
#define WHEEL_TRAVEL_DESCEND_RELEASE_RAD (WHEEL_TRAVEL_DESCEND_RELEASE_M / WHEEL_RADIUS_M)

// ==========================================
// 2. 速度、斜坡、时间参数
// ==========================================

// 轮子角度模式：距离->角度（rad）
//上台阶
#define WHEEL_RADIUS_M                MM_TO_M(50.0f)
#define WHEEL_TRAVEL_UP_M             (0.6f)
#define WHEEL_TRAVEL_UP_RAD           (WHEEL_TRAVEL_UP_M / WHEEL_RADIUS_M)
//下台阶
#define WHEEL_TRAVEL_DESCEND_M        (0.6f)
#define WHEEL_TRAVEL_DESCEND_RAD      (WHEEL_TRAVEL_DESCEND_M / WHEEL_RADIUS_M)
#define WHEEL_ANGLE_DONE_TOL_RAD      (0.25f) //轮子角度到位容忍度 即误差到某个值时认为轮子已经到位

// 轮子角度斜坡
#define WHEEL_SLOPE_RPM_UP            (120.0f) //up
#define WHEEL_SLOPE_RPM_DESCEND       (50.0f) //descend
#define WHEEL_SLOPE_STEP_UP           (RC_RPM_TO_RADPS(WHEEL_SLOPE_RPM_UP) / TASK_FREQ_HZ)
#define WHEEL_SLOPE_STEP_DESCEND      (RC_RPM_TO_RADPS(WHEEL_SLOPE_RPM_DESCEND) / TASK_FREQ_HZ)

// 轮子角度模式 PID
#define PID_WHEEL_OMEGA_KP            (85.3f)
#define PID_WHEEL_OMEGA_KI            (10.3f)
#define PID_WHEEL_ANGLE_KP            (30.0f)
#define PID_WHEEL_ANGLE_KI            (1.0f)
#define WHEEL_CREEP_OMEGA_RADPS       (0.3f)

#define SETUP_TEST_REAR_DIR          (1.0f)
#define SETUP_TEST_REAR_COMP         (2200.0f)


// 动态PID参数（按工况切换）
#define PID_FRONT_OMEGA_KP_NORMAL   (80.5f) //正常前轮P
#define PID_FRONT_ANGLE_KP_NORMAL   (40.8f)
#define PID_REAR_OMEGA_KP_NORMAL    (85.0f) //正常后轮P
#define PID_REAR_ANGLE_KP_NORMAL    (50.0f)
#define PID_REAR_OMEGA_KI_NORMAL    (5.0f) //正常后轮I
#define PID_REAR_ANGLE_KI_NORMAL    (0.5f)

#define PID_FRONT_OMEGA_KP_LIFT     (135.0f) //抬升前轮P
#define PID_FRONT_ANGLE_KP_LIFT     (75.0f)
#define PID_REAR_OMEGA_KP_LIFT      (165.0f) //抬升后轮P
#define PID_REAR_ANGLE_KP_LIFT      (96.0f)
#define PID_REAR_OMEGA_KI_LIFT      (8.0f) //抬升后轮I
#define PID_REAR_ANGLE_KI_LIFT      (0.8f)

// --- 上台阶时间参数 ---
#define TIME_SETUP           500  // 给2秒让它缩腿
#define TIME_TOUCH           500  // 触地时间
#define TIME_LIFT            1500  // 顶升时间
#define TIME_LIFT_REAR_DELAY 100   // 顶升阶段后脚延时启动
#define TIME_DRIVE           1200  // 平移时间
#define TIME_RETRACT         2000  // 收腿时间

// --- 下台阶时间参数 ---
#define TIME_DESC_SETUP      500
#define TIME_DESC_TOUCH      2000  // 下台阶：状态3 触地时间
#define TIME_DESC_GLOBAL_DOWN  1000 // 下台阶：状态4 全局下降时间
#define DESCEND_DRIVE_TIME_MS          1800 // 下台阶：状态5 平移时间（左右轮速度环）
#define TIME_DESC_RAISE      1500 // 下台阶：状态6 抬升时间
#define TIME_DESC_RELEASE    500 // 下台阶：状态7 脱离时间

// ==========================================
// 3. 状态定义
// ==========================================

typedef enum {
    STEP_IDLE = 0,            // 空闲
    STEP_SETUP,               // 上台阶: 收腿准备
    STEP_TOUCH_DOWN,          // 上台阶: 前腿找台阶, 后腿找地
    STEP_GLOBAL_LIFT,         // 上台阶: 全局顶升
    STEP_DRIVE_FWD,           // 上台阶: 轮子平移
    STEP_RETRACT,             // 上台阶: 收腿复位
    STEP_DONE,                // 上台阶完成

    STEP_DESCEND_SETUP,       // 下台阶: 收腿准备
    STEP_DESCEND_TOUCH,       // 下台阶: 触地
    STEP_DESCEND_GLOBAL_DOWN, // 下台阶: 全局下降
    STEP_DESCEND_DRIVE,       // 下台阶: 轮子平移
    STEP_DESCEND_RAISE,       // 下台阶: 抬起
    STEP_DESCEND_RELEASE,     // 下台阶: 轮子前移脱离台阶
    STEP_DESCEND_DONE         // 下台阶完成
} ClimbingState_e;

typedef enum {
    CLIMB_UP_MODE_20CM = 0,
    CLIMB_UP_MODE_40CM
} ClimbUpMode_e;

class QuinticPlanner {
public:
    enum State { UNPLANNING, PLANNING };

    explicit QuinticPlanner(float pos)
        : q0(pos), qf(pos), state(UNPLANNING), current_target(pos) {}

    void Plan(float start_pos, float target_pos, float time_duration) {
        if (time_duration <= 0.0f) return;
        q0 = start_pos;
        qf = target_pos;
        T = time_duration;
        t = 0.0f;
        state = PLANNING;
        current_target = start_pos;
    }

    float GetNextPosition(float dt) {
        if (state == PLANNING) {
            t += dt;
            if (t >= T) {
                t = T;
                state = UNPLANNING;
                current_target = qf;
            } else {
                float tau = t / T;
                float tau3 = tau * tau * tau;
                float scale = tau3 * (10.0f - 15.0f * tau + 6.0f * tau * tau);
                current_target = q0 + (qf - q0) * scale;
            }
        }
        return current_target;
    }

    bool IsPlanning() const { return state == PLANNING; }
    float GetCurrentTarget() const { return current_target; }

    void ForceSetPosition(float pos) {
        q0 = pos;
        qf = pos;
        current_target = pos;
        state = UNPLANNING;
    }

private:
    float q0, qf;
    float t, T;
    State state;
    float current_target;
};

#endif // TEST_FEEDBACK_CLIMBING_CONTROLLER_H


class ClimbingController {
private:
    // ===== 状态变量 =====
    ClimbingState_e climb_state_;
    uint32_t state_tick_;

    // ===== 执行器对象 =====
    Class_Motor_C620 motor_lift_front_;
    Class_Motor_C620 motor_lift_rear_;
    Class_Motor_C620 motor_wheel_l_;
    Class_Motor_C620 motor_wheel_r_;

    // ===== 前后腿用五次多项式 两个轮子用斜坡规划器 =====
    QuinticPlanner planner_front_pos_{0.0f};
    QuinticPlanner planner_rear_pos_{0.0f};
    Class_Slope slope_wheel_l_angle_; // 轮子易打滑，保留斜坡模式
    Class_Slope slope_wheel_r_angle_;

    // ===== 运行时数据 =====
    uint8_t is_zero_recorded_;
    float start_pos_front_;
    float start_pos_rear_;
    float desc_start_pos_front_;
    float desc_start_pos_rear_;
    float wheel_target_angle_l_;
    float wheel_target_angle_r_;
    uint8_t is_lift_pid_mode_;
    ClimbingState_e prev_climb_state_;
    uint8_t rear_lift_delayed_flag_;

    // ===== 自动流程相关 =====
    uint8_t auto_running_;
    uint32_t auto_state_enter_tick_;
    uint8_t descend_mode_;
    uint8_t chassis_external_control_;
    uint8_t init_pose_active_;
    ClimbUpMode_e up_mode_;

    // ===== 内部控制流程 =====
    void ApplyMotorOutputWithComp(Class_Motor_C620 &motor, float comp, uint8_t can_byte_index);
    void HandleStateTransition(uint32_t current_time, uint8_t state_changed);
    uint8_t IsWheelAngleDone(void);
    void RecordSoftZero(void); //记录上电软零点
    void UpdatePidAndSlopeByState(void);
    void UpdateStateTargets(uint32_t current_time);
    void RunLiftAndWheelControl(void);
    void SetLiftPidMode(uint8_t enable_lift);
    void SetLegSlopeStep(float front_step, float rear_step);
    void SetWheelSlopeStep(float wheel_step);
    //时间延时状态跳转辅助函数
    bool TryTimeTransition(uint32_t now, uint32_t delay_ms, ClimbingState_e next_state);
public:
    ClimbingController();

    float GetFrontTargetAngle(void) { return motor_lift_front_.Get_Target_Angle(); }
    float GetFrontNowAngle(void) { return motor_lift_front_.Get_Now_Angle(); }
    float GetFrontOut(void) { return motor_lift_front_.Get_Out(); }

    float GetRearTargetAngle(void) { return motor_lift_rear_.Get_Target_Angle(); }
    float GetRearNowAngle(void) { return motor_lift_rear_.Get_Now_Angle(); }
    float GetRearOut(void) { return motor_lift_rear_.Get_Out(); }

    ClimbingState_e GetState(void) { return climb_state_; }

    // 初始化电机/斜坡/PID
    void Init(CAN_HandleTypeDef *hcan);
    // 1ms 主控制入口: 目标更新 + PID + 补偿 + 发送
    void TaskEntry1ms(void);
    // 1ms 自动流程推进器: 仅切状态
    void AutoTask1ms(void);
    // CAN 回调分发: 201~204
    void CAN_RxCallback(uint32_t std_id, uint8_t *data);

    // 启动上/下台阶自动流程
    void AutoStart(void);
    void AutoStart20cm(void);
    void AutoStart40cm(void);
    void Prepare40cm(void);
    void AutoStartFromTouch20cm(void);
    void AutoStartFromTouch40cm(void);
    void DescendAutoStart(void);
    void DescendAutoStart20cm(void);
    void InitPoseStart(void);

    // 手动调试接口
    void ManualNext(void);
    void DescendManualNext(void);
    void EmergencyStop(void);
    // 底盘外部接管开关: 1 外部接管, 0 状态机控制
    void SetChassisExternalControl(uint8_t enable);
};
