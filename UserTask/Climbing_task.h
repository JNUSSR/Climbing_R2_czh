#ifndef CLIMBING_TASK_H
#define CLIMBING_TASK_H

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

// ==========================================
// 0. 基础计算工具 (PI定义在drv_math.h中)
// ==========================================

#define TASK_FREQ_HZ        (1000.0f) 

// 辅助宏：圈数 -> 弧度
#define TURNS_TO_RADS(x)    ((x) * 2.0f * PI)

// 【修改点】改个名字，避免和 dvc_motor.h 里的宏冲突
// 你的 dvc_motor.h 里 RPM_TO_RADPS 是个常数，这里我们需要一个转换公式
#define RC_RPM_TO_RADPS(x)  ((x) * 2.0f * PI / 60.0f)

// ==========================================
// 1. 机械位置配置 (48mm 带轮, 导程 ~15cm)
// ==========================================
// 逻辑：上电软零点 (0) = 趴在地上
// 正数 (+) = 向下伸长(顶起)
// 负数 (-) = 向上收缩(提腿)

// [State 1: 准备姿态] 
// 前腿缩 1.5 圈 (~22cm) 避让台阶 -1.5
#define POS_FRONT_RETRACT    TURNS_TO_RADS(-1.5f)
// 后腿缩 0.5 圈 (~7.5cm) 避让地面 
#define POS_REAR_RETRACT     TURNS_TO_RADS(-0.5f)

// [State 3: 触地预备] 
// 前腿伸到 -1.3 圈 (~20cm处)，刚好摸到台阶面
#define POS_FRONT_TOUCH      TURNS_TO_RADS(-1.2f)
// 后腿回到 0 圈，重新踩实地面
#define POS_REAR_TOUCH       TURNS_TO_RADS(0.0f)

// [State 4: 同步顶升]
// 关键：从State 3开始，前后腿"等距伸出"以保持车身水平
// 前腿从 -1.3 圈 → +1.0 圈 (增加 2.3 圈)
#define POS_FRONT_LIFT       TURNS_TO_RADS(1.0f)
// 后腿从 0 圈 → +2.3 圈 (增加 2.3 圈) - 修正：原来是2.5，现改为等距
#define POS_REAR_LIFT        TURNS_TO_RADS(2.2f)

// [State 6: 收腿复位]
// 回到 0 点 (趴在台阶上)
#define POS_FRONT_FINAL      TURNS_TO_RADS(-1.5f)
#define POS_REAR_FINAL       TURNS_TO_RADS(-0.5f)

// ==========================================
// 2. 速度与时间参数
// ==========================================

// [腿部斜坡速度]
// 四组参数：前/后腿 × 两组状态
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
#define FRONT_SLOPE_RPM_DESC_G2        (110.0f)
#define REAR_SLOPE_RPM_DESC_G2         (110.0f)
#define FRONT_SLOPE_STEP_DESC_G2       (RC_RPM_TO_RADPS(FRONT_SLOPE_RPM_DESC_G2) / TASK_FREQ_HZ)
#define REAR_SLOPE_STEP_DESC_G2        (RC_RPM_TO_RADPS(REAR_SLOPE_RPM_DESC_G2) / TASK_FREQ_HZ)

// [平移驱动速度]
// 保持 60 RPM 不变 (约 0.3~0.5 m/s)
#define DRIVE_SPEED_RPM      (250.0f)
#define SPEED_DRIVE_FWD      RC_RPM_TO_RADPS(DRIVE_SPEED_RPM)

// WAIT_TRIGGER 阶段底盘前进参数（自动流程专用）
#define WAIT_TRIGGER_FORWARD_TIME_MS   (1500U)
#define WAIT_TRIGGER_FORWARD_VX        (5.0f)

// 下台阶：状态2 底盘前进参数（wheelchassis）
#define DESCEND_WAIT_TIME_MS           (1500U)
#define DESCEND_WAIT_FORWARD_VX        (WAIT_TRIGGER_FORWARD_VX)

// 下台阶：状态4 全局下降时间
#define TIME_DESC_GLOBAL_DOWN 500

// 下台阶：状态5 平移时间（左右轮速度环）
#define DESCEND_DRIVE_TIME_MS          (2000U)

// 下台阶：状态3 触地目标
#define DESCEND_FRONT_TOUCH_TARGET     TURNS_TO_RADS(1.2f)
#define DESCEND_REAR_TOUCH_TARGET      TURNS_TO_RADS(0.0f)

// 下台阶：状态4 全局下降目标
#define DESCEND_FRONT_GLOBAL_DOWN_TARGET TURNS_TO_RADS(1.4f)
#define DESCEND_REAR_GLOBAL_DOWN_TARGET  TURNS_TO_RADS(0.2f)

// 后腿测试方向：+1 或 -1（按机构方向调整）
#define SETUP_TEST_REAR_DIR          (1.0f)
// STEP_SETUP 测试时给后腿附加补偿，帮助克服静摩擦/负载
#define SETUP_TEST_REAR_COMP         (2200.0f)



// ==========================================
// 3. 重力补偿参数
// ==========================================
// 补偿方向说明：
// - 负值：向上托（收腿/提腿时，克服腿自身重力）
// - 正值：向下推（顶升/撑车时，支撑整车重量）

// 收腿/悬腿状态：向上托
#define GRAVITY_COMPENSATION_FRONT  (-400.0f)
#define GRAVITY_COMPENSATION_REAR   (-1500.0f)

// 顶升状态：向下推
#define COMP_FRONT_LIFT             (1200.0f)
#define COMP_REAR_LIFT              (3000.0f)

// 后腿顶升起步加强（克服静摩擦/瞬时载荷）
#define COMP_REAR_LIFT_BOOST        (1800.0f)
#define COMP_REAR_BOOST_TIME_MS     (260U)

// 顶升/悬空阶段最小保压输出，防止位置环把力卸掉
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

// --- 时间参数 ---
#define TIME_SETUP           500  // 给2秒让它缩腿
#define WAIT_TRIGGER         1500  // 等待底盘移动到位的时间窗口（自动流程专用） 
#define TIME_TOUCH           500  // 触地时间
#define TIME_LIFT            1500  // 顶升时间
#define TIME_DRIVE           2000  // 平移时间
#define TIME_RETRACT         4000  // 收腿时间

// --- 下台阶时间参数 ---
#define TIME_DESC_SETUP      500
#define TIME_DESC_TOUCH      500
#define TIME_DESC_RAISE      1500

// ==========================================
// 3. 状态定义
// ==========================================
typedef enum {
    STEP_IDLE = 0,      // 上电等待
    STEP_SETUP,         // 1. 前后腿均收起 (悬空)
    STEP_WAIT_TRIGGER,  // 2. 底盘移动中...等待到位信号
    STEP_TOUCH_DOWN,    // 3. 前腿找台阶，后腿找地面
    STEP_GLOBAL_LIFT,   // 4. 全局顶升
    STEP_DRIVE_FWD,     // 5. 盲跑平移
    STEP_RETRACT,       // 6. 收腿复位
    STEP_DONE,          // 完成

    // 下台阶状态机
    STEP_DESCEND_SETUP, // 1. 下台前后腿均收起 (悬空)
    STEP_DESCEND_WAIT_TRIGGER, // 2. 下台底盘移动中...等待到位信号
    STEP_DESCEND_TOUCH,       // 3. 下台触地
    STEP_DESCEND_GLOBAL_DOWN, // 4. 下台全局下降
    STEP_DESCEND_DRIVE,       // 5. 下台平移
    STEP_DESCEND_RAISE,       // 6. 下台后前后腿上升（提起）
    STEP_DESCEND_DONE         // 7. 下台完成
} ClimbingState_e;

void Climbing_Task_Init(void);
void Climbing_Task_Entry(void);
void Climbing_Next_Step(void); 

// ========== 自动连贯动作接口 ==========
// 串口收到 0x67 后调用：按时间参数自动跑完整流程（一次）
void Climbing_Auto_Start(void);
// 串口收到 0x69 后调用：按时间参数自动跑下台阶流程（一次）
void Climbing_Descend_Auto_Start(void);
// 1ms 周期调用：自动状态推进器（建议在 FreeRTOS 任务里调用）
void Climbing_Auto_Task_1ms(void);
// 查询自动流程是否正在运行
uint8_t Climbing_Is_Auto_Running(void);

// ========== 手动调试接口 ==========
void Climbing_Manual_Next(void);           // 手动下一步
void Climbing_Descend_Manual_Next(void);   // 下台阶手动下一步（0x91）
void Climbing_Manual_Reset(void);          // 复位到IDLE
void Climbing_Manual_Goto(ClimbingState_e state);  // 直接跳转
void Climbing_Emergency_Stop(void);        // 紧急停止

#ifdef __cplusplus
}
#endif

#endif // CLIMBING_TASK_H
