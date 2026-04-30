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
#define SCREW_LEAD_MM_PER_TURN   (150.0f)
#define movingmm(x_mm)       TURNS_TO_RADS((x_mm) / SCREW_LEAD_MM_PER_TURN)

// 避免和 dvc_motor.h 里的同名宏冲突
#define RC_RPM_TO_RADPS(x)  ((x) * 2.0f * PI / 60.0f)

// ==========================================
// 1. 机械位置参数(上/下台阶)
// ==========================================
//初始状态
#define POS_FRONT_Init       movingmm(-220.0f)
#define POS_REAR_Init        movingmm(-10.0f)

//上20cm台阶
#define POS_FRONT_RETRACT_20cm    movingmm(-220.0f)
#define POS_REAR_RETRACT_20cm     movingmm(-10.0f)

#define POS_FRONT_TOUCH_20cm      movingmm(-200.0f)
#define POS_REAR_TOUCH_20cm       movingmm(0.0f)

#define POS_FRONT_LIFT_20cm       movingmm(20.0f)
#define POS_REAR_LIFT_20cm        movingmm(220.0f)

//上40cm台阶
#define POS_FRONT_RETRACT_40cm    movingmm(-420.0f)
#define POS_REAR_RETRACT_40cm     movingmm(-75.0f)

#define POS_FRONT_TOUCH_40cm      movingmm(-400.0f)
#define POS_REAR_TOUCH_40cm       movingmm(0.0f)

#define POS_FRONT_LIFT_40cm       movingmm(20.0f)
#define POS_REAR_LIFT_40cm        movingmm(413.0f)

#define POS_FRONT_FINAL      movingmm(-420.0f)
#define POS_REAR_FINAL       movingmm(-75.0f)

//下20cm台阶
// 下台阶：状态3 触地目标
#define DESCEND_FRONT_TOUCH_TARGET     movingmm(20.0f)
#define DESCEND_REAR_TOUCH_TARGET      movingmm(0.0f)

// 下台阶：状态4 全局下降目标
#define DESCEND_FRONT_GLOBAL_DOWN_TARGET movingmm(28.0f)
#define DESCEND_REAR_GLOBAL_DOWN_TARGET  movingmm(8.0f)

// ==========================================
// 2. 速度、斜坡、时间参数
// ==========================================

//斜坡规划参数
// 组A: STEP_SETUP / STEP_RETRACT
#define FRONT_SLOPE_RPM_SETUP_RETRACT  (105.0f) //前腿的距离比较长 速度给大一些
#define REAR_SLOPE_RPM_SETUP_RETRACT   (90.0f)
// 组B: STEP_TOUCH_DOWN / STEP_GLOBAL_LIFT 抬升的速度需要大一点 抬升的距离相同，给相同速度
#define FRONT_SLOPE_RPM_TOUCH_LIFT     (140.0f)
#define REAR_SLOPE_RPM_TOUCH_LIFT      (140.0f)

#define FRONT_SLOPE_STEP_SETUP_RETRACT (RC_RPM_TO_RADPS(FRONT_SLOPE_RPM_SETUP_RETRACT) / TASK_FREQ_HZ)
#define REAR_SLOPE_STEP_SETUP_RETRACT  (RC_RPM_TO_RADPS(REAR_SLOPE_RPM_SETUP_RETRACT) / TASK_FREQ_HZ)
#define FRONT_SLOPE_STEP_TOUCH_LIFT    (RC_RPM_TO_RADPS(FRONT_SLOPE_RPM_TOUCH_LIFT) / TASK_FREQ_HZ)
#define REAR_SLOPE_STEP_TOUCH_LIFT     (RC_RPM_TO_RADPS(REAR_SLOPE_RPM_TOUCH_LIFT) / TASK_FREQ_HZ)

// 下台阶斜坡组A（状态1/2，normal PID）
#define FRONT_SLOPE_RPM_DESC_G1        (95.0f)
#define REAR_SLOPE_RPM_DESC_G1         (95.0f)
#define FRONT_SLOPE_STEP_DESC_G1       (RC_RPM_TO_RADPS(FRONT_SLOPE_RPM_DESC_G1) / TASK_FREQ_HZ)
#define REAR_SLOPE_STEP_DESC_G1        (RC_RPM_TO_RADPS(REAR_SLOPE_RPM_DESC_G1) / TASK_FREQ_HZ)

// 下台阶斜坡组B（状态3/4/5，lift PID）
#define FRONT_SLOPE_RPM_DESC_G2        (130.0f)
#define REAR_SLOPE_RPM_DESC_G2         (130.0f)
#define FRONT_SLOPE_STEP_DESC_G2       (RC_RPM_TO_RADPS(FRONT_SLOPE_RPM_DESC_G2) / TASK_FREQ_HZ)
#define REAR_SLOPE_STEP_DESC_G2        (RC_RPM_TO_RADPS(REAR_SLOPE_RPM_DESC_G2) / TASK_FREQ_HZ)

// 轮子角度模式：距离->角度（rad）
//上台阶
#define WHEEL_RADIUS_M                MM_TO_M(50.0f)
#define WHEEL_TRAVEL_UP_M             (0.6f)
#define WHEEL_TRAVEL_UP_RAD           (WHEEL_TRAVEL_UP_M / WHEEL_RADIUS_M)
//下台阶
#define WHEEL_TRAVEL_DESCEND_M        (0.62f)
#define WHEEL_TRAVEL_DESCEND_RAD      (WHEEL_TRAVEL_DESCEND_M / WHEEL_RADIUS_M)
#define WHEEL_ANGLE_DONE_TOL_RAD      (0.25f) //轮子角度到位容忍度 即误差到某个值时认为轮子已经到位

// 轮子角度斜坡
#define WHEEL_SLOPE_RPM_UP            (120.0f) //up
#define WHEEL_SLOPE_RPM_DESCEND       (120.0f) //descend
#define WHEEL_SLOPE_STEP_UP           (RC_RPM_TO_RADPS(WHEEL_SLOPE_RPM_UP) / TASK_FREQ_HZ)
#define WHEEL_SLOPE_STEP_DESCEND      (RC_RPM_TO_RADPS(WHEEL_SLOPE_RPM_DESCEND) / TASK_FREQ_HZ)

// 轮子角度模式 PID
#define PID_WHEEL_OMEGA_KP            (85.3f)
#define PID_WHEEL_OMEGA_KI            (10.3f)
#define PID_WHEEL_ANGLE_KP            (30.0f)
#define PID_WHEEL_ANGLE_KI            (1.0f)

// WAIT_TRIGGER 阶段底盘前进参数（自动流程专用）
#define WAIT_TRIGGER_FORWARD_TIME_MS   (1500U)
#define WAIT_TRIGGER_FORWARD_VX        (0.0f)

#define SETUP_TEST_REAR_DIR          (1.0f)
#define SETUP_TEST_REAR_COMP         (2200.0f)

// ==========================================
// 3. 重力补偿参数
// 补偿方向说明：
// - 负值：向上托（收腿/提腿时，克服腿自身重力）
// - 正值：向下推（顶升/撑车时，支撑整车重量）
// ==========================================

// 收腿/悬腿状态：向上托
#define GRAVITY_COMPENSATION_FRONT  (-400.0f)
#define GRAVITY_COMPENSATION_REAR   (-1500.0f)

// 顶升状态：向下推
#define COMP_FRONT_LIFT             (1200.0f)
#define COMP_REAR_LIFT              (3000.0f)

// 后腿顶升起步加强（克服静摩擦/瞬时载荷）没用
#define COMP_REAR_LIFT_BOOST        (1800.0f)
#define COMP_REAR_BOOST_TIME_MS     (260U)

// 顶升/悬空阶段最小保压输出，防止位置环把力卸掉 没用
#define HOLD_MIN_FRONT_LIFT         (600.0f)
#define HOLD_MIN_REAR_LIFT          (2500.0f)

// 动态PID参数（按工况切换）
#define PID_FRONT_OMEGA_KP_NORMAL   (60.5f) //正常前轮P
#define PID_FRONT_ANGLE_KP_NORMAL   (40.8f)
#define PID_REAR_OMEGA_KP_NORMAL    (85.0f) //正常后轮P
#define PID_REAR_ANGLE_KP_NORMAL    (50.0f)
#define PID_REAR_OMEGA_KI_NORMAL    (5.0f) //正常后轮I
#define PID_REAR_ANGLE_KI_NORMAL    (0.5f)

#define PID_FRONT_OMEGA_KP_LIFT     (115.0f) //抬升前轮P
#define PID_FRONT_ANGLE_KP_LIFT     (75.0f)
#define PID_REAR_OMEGA_KP_LIFT      (165.0f) //抬升后轮P
#define PID_REAR_ANGLE_KP_LIFT      (96.0f)
#define PID_REAR_OMEGA_KI_LIFT      (8.0f) //抬升后轮I
#define PID_REAR_ANGLE_KI_LIFT      (0.8f)

// --- 上台阶时间参数 ---
#define TIME_SETUP           500  // 给2秒让它缩腿
#define WAIT_TRIGGER         1000  // 等待底盘移动到位的时间窗口（自动流程专用） 
#define TIME_TOUCH           500  // 触地时间
#define TIME_LIFT            1500  // 顶升时间
#define TIME_DRIVE           1200  // 平移时间
#define TIME_RETRACT         2000  // 收腿时间

// --- 下台阶时间参数 ---
#define TIME_DESC_SETUP      500
#define DESCEND_WAIT_TIME_MS           1500// 下台阶：状态2 底盘前进参数（wheelchassis）
#define DESCEND_WAIT_FORWARD_VX        (0.0f) // 下台阶：状态2 底盘前进速度
#define TIME_DESC_TOUCH      2000  // 下台阶：状态3 触地时间
#define TIME_DESC_GLOBAL_DOWN  1000 // 下台阶：状态4 全局下降时间
#define DESCEND_DRIVE_TIME_MS          1800 // 下台阶：状态5 平移时间（左右轮速度环）
#define TIME_DESC_RAISE      1500 // 下台阶：状态6 抬升时间

// ==========================================
// 3. 状态定义
// ==========================================

typedef enum {
    STEP_IDLE = 0,            // 空闲
    STEP_SETUP,               // 上台阶: 收腿准备
    STEP_WAIT_TRIGGER,        // 上台阶: 等待触发/底盘前移
    STEP_TOUCH_DOWN,          // 上台阶: 前腿找台阶, 后腿找地
    STEP_GLOBAL_LIFT,         // 上台阶: 全局顶升
    STEP_DRIVE_FWD,           // 上台阶: 轮子平移
    STEP_RETRACT,             // 上台阶: 收腿复位
    STEP_DONE,                // 上台阶完成

    STEP_DESCEND_SETUP,       // 下台阶: 收腿准备
    STEP_DESCEND_WAIT_TRIGGER,// 下台阶: 等待触发/底盘前移
    STEP_DESCEND_TOUCH,       // 下台阶: 触地
    STEP_DESCEND_GLOBAL_DOWN, // 下台阶: 全局下降
    STEP_DESCEND_DRIVE,       // 下台阶: 轮子平移
    STEP_DESCEND_RAISE,       // 下台阶: 抬起
    STEP_DESCEND_DONE         // 下台阶完成
} ClimbingState_e;

typedef enum {
    CLIMB_UP_MODE_20CM = 0,
    CLIMB_UP_MODE_40CM
} ClimbUpMode_e;

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

    // ===== 斜坡规划器 =====
    Class_Slope slope_front_pos_;
    Class_Slope slope_rear_pos_;
    Class_Slope slope_wheel_l_angle_;
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

    // ===== 自动流程相关 =====
    uint8_t auto_running_;
    uint32_t auto_state_enter_tick_;
    uint8_t descend_mode_;
    uint8_t chassis_external_control_;
    uint8_t init_pose_active_;
    ClimbUpMode_e up_mode_;

    // ===== 内部控制流程 =====
    float ClampC620Output(float out);
    void ApplyMotorOutputWithComp(Class_Motor_C620 &motor, float comp, uint8_t can_byte_index);
    void ComputeGravityCompensation(float *front_comp, float *rear_comp);
    void HandleStateTransition(uint32_t current_time, uint8_t state_changed);
    uint8_t IsWheelAngleDone(void);
    void UpdatePidAndSlopeByState(void);
    void UpdateStateTargets(uint32_t current_time);
    void UpdateChassisControl(void);
    void RunLiftAndWheelControl(void);
    void ApplyGravityCompensationAndSend(void);

public:
    ClimbingController();

    // 初始化电机/斜坡/PID
    void Init(CAN_HandleTypeDef *hcan);
    // 1ms 主控制入口: 目标更新 + PID + 补偿 + 发送
    void TaskEntry1ms(void);
    // 1ms 自动流程推进器: 仅切状态
    void AutoTask1ms(void);
    // CAN 回调分发: 201~204
    void CAN_RxCallback(uint32_t std_id, uint8_t *data);

    // 单步推进(兼容旧接口)
    void NextStep(void);
    // 启动上/下台阶自动流程
    void AutoStart(void);
    void AutoStart20cm(void);
    void AutoStart40cm(void);
    void DescendAutoStart(void);
    void DescendAutoStart20cm(void);
    void InitPoseStart(void);
    uint8_t IsAutoRunning(void) const;

    // 手动调试接口
    void ManualNext(void);
    void DescendManualNext(void);
    void ManualReset(void);
    void ManualGoto(ClimbingState_e state);
    void EmergencyStop(void);
    // 底盘外部接管开关: 1 外部接管, 0 状态机控制
    void SetChassisExternalControl(uint8_t enable);

    // 调试/观测接口
    ClimbingState_e GetState(void) const;
    Class_Motor_C620 &GetMotorLiftFront(void);
    Class_Motor_C620 &GetMotorLiftRear(void);
    Class_Motor_C620 &GetMotorWheelL(void);
    Class_Motor_C620 &GetMotorWheelR(void);
    Class_Slope &GetSlopeFrontPos(void);
    Class_Slope &GetSlopeRearPos(void);
};

#endif // TEST_FEEDBACK_CLIMBING_CONTROLLER_H
