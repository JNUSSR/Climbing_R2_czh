/**
 * @file host_communication_task.cpp
 * @brief 上位机通信逻辑实现 - 爬坡调试版
 */

#include "host_communication_task.h"
#include "drv_uart.h"
#include "usart.h"
#include "dvc_motor.h"
#include "Climbing_Task.h"
#include "alg_slope.h"  // 添加斜坡头文件
#include "cmsis_os.h"
#include <string.h>

/* Private variables ---------------------------------------------------------*/

float g_motor_3508_target_speed = 0.0f;

#define HOST_RX_BUFFER_SIZE 128

// VOFA+ JustFloat 帧尾
static const uint8_t VOFA_TAIL[4] = {0x00, 0x00, 0x80, 0x7f};

// 外部电机对象声明 (定义在Climbing_task.cpp)
extern Class_Motor_C620 Motor_Lift_Front;
extern Class_Motor_C620 Motor_Lift_Rear;
extern Class_Motor_C620 Motor_Wheel_L;
extern Class_Motor_C620 Motor_Wheel_R;
extern ClimbingState_e g_climbState;

// 【调试用】声明斜坡规划器，直接观察其输出
extern Class_Slope Slope_Front_Pos;
extern Class_Slope Slope_Rear_Pos;

/* Private function prototypes -----------------------------------------------*/

static void Host_UART_Callback(uint8_t *Buffer, uint16_t Length);

/* Function implementation ---------------------------------------------------*/

/**
 * @brief 任务初始化
 */
void Host_Communication_Task_Init(void)
{
    UART_Init(&huart6, Host_UART_Callback, HOST_RX_BUFFER_SIZE);
}

/**
 * @brief 串口接收回调 - 解析VOFA+指令
 * 
 * 指令表 (单字节Hex):
 * 0x10 - 下一步 (Next Step)
 * 0x11 - 复位到IDLE
 * 0x12 - 紧急停止 (电机输出归零)
 * 0x20 - 直接跳到 STEP_SETUP
 * 0x21 - 直接跳到 STEP_WAIT_TRIGGER
 * 0x22 - 直接跳到 STEP_TOUCH_DOWN
 * 0x23 - 直接跳到 STEP_GLOBAL_LIFT
 * 0x24 - 直接跳到 STEP_DRIVE_FWD
 * 0x25 - 直接跳到 STEP_RETRACT
 * 0x67 - 启动自动连贯动作（一次完整流程）
 * 0x69 - 启动下台阶自动流程（一次完整流程）
 * 0x91 - 下台阶手动下一步
 */
static void Host_UART_Callback(uint8_t *Buffer, uint16_t Length)
{
    if (Length == 0 || Buffer == NULL) return;

    // 自动流程运行时：只允许复位/急停；忽略手动步进与手动跳转
    if (Climbing_Is_Auto_Running() &&
        Buffer[0] != 0x11 &&
        Buffer[0] != 0x12)
    {
        return;
    }

    switch (Buffer[0])
    {
        // ========== 顺序控制 ==========
        case 0x10: // 下一步
            Climbing_Manual_Next();
            break;
        case 0x91: // 下台阶手动下一步
            Climbing_Descend_Manual_Next();
            break;

        case 0x11: // 复位到IDLE
            Climbing_Manual_Reset();
            break;

        case 0x12: // 紧急停止
            Climbing_Emergency_Stop();
            break;

        // ========== 直接跳转 ==========
        case 0x20:
            Climbing_Manual_Goto(STEP_SETUP);
            break;
        case 0x21:
            Climbing_Manual_Goto(STEP_WAIT_TRIGGER);
            break;
        case 0x22:
            Climbing_Manual_Goto(STEP_TOUCH_DOWN);
            break;
        case 0x23:
            Climbing_Manual_Goto(STEP_GLOBAL_LIFT);
            break;
        case 0x24:
            Climbing_Manual_Goto(STEP_DRIVE_FWD);
            break;
        case 0x25:
            Climbing_Manual_Goto(STEP_RETRACT);
            break;

        // ========== 自动流程 ==========
        case 0x67:
            Climbing_Auto_Start();
            break;
        case 0x69:
            Climbing_Descend_Auto_Start();
            break;

        default:
            break;
    }
}

/**
 * @brief 发送波形数据到VOFA+ (JustFloat协议)
 * 
 * 通道说明 (后腿最小策动测试版):
 * CH1: Rear Target Omega
 * CH2: Rear Now Omega
 * CH3: Rear Out
 * CH4: Rear Target Angle
 * CH5: Rear Now Angle
 * CH6: Climb State
 * CH7: Front Out (对照)
 */
void Host_Send_Waveform(void)
{
    float tx_data[7];

    // tx_data[0] = Motor_Lift_Rear.Get_Target_Omega();   // CH1
    // tx_data[1] = Motor_Lift_Rear.Get_Now_Omega();      // CH2
    // tx_data[2] = Motor_Lift_Rear.Get_Out();            // CH3
    // tx_data[3] = Motor_Lift_Rear.Get_Target_Angle();   // CH4
    // tx_data[4] = Motor_Lift_Rear.Get_Now_Angle();      // CH5
    // tx_data[5] = (float)g_climbState;                  // CH6
    // tx_data[6] = Motor_Lift_Front.Get_Out();           // CH7

    tx_data[0] = Motor_Lift_Front.Get_Target_Angle();  // CH1: 前腿目标 (应该跟随斜坡输出)
    tx_data[1] = Motor_Lift_Front.Get_Now_Angle();     // CH2: 前腿实际
    tx_data[2] = Motor_Lift_Rear.Get_Target_Angle();      // CH3: 后腿目标
    tx_data[3] = Motor_Lift_Rear.Get_Now_Angle();      // CH4: 后腿实际
    tx_data[4] = (float)g_climbState;                  // CH5: 状态机
    tx_data[5] = Motor_Lift_Front.Get_Out();           // CH6: 前腿PID输出
    tx_data[6] = Motor_Lift_Rear.Get_Out();            // CH7: 后腿PID输出
    
    // 发送数据 + 帧尾
    HAL_UART_Transmit(&huart6, (uint8_t*)tx_data, sizeof(tx_data), 10);
    HAL_UART_Transmit(&huart6, (uint8_t*)VOFA_TAIL, 4, 10);
}

/**
 * @brief 任务主循环 - 周期发送波形
 */
void Host_Communication_Task(void *argument)
{
    Host_Communication_Task_Init();
    
    for(;;)
    {
        Host_Send_Waveform();
        osDelay(10);  // 100Hz 发送频率
    }
}
