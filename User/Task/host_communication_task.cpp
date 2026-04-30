/**
 * @file host_communication_task.cpp
 * @brief 上位机通信逻辑实现 - 爬坡调试版
 */

#include "host_communication_task.h"
#include "drv_uart.h"
#include "usart.h"
#include "dvc_motor.h"
#include "climbingTask2.h"
#include "Mecanum/task_mecanum_chassis.h"
#include "alg_slope.h"  // 添加斜坡头文件
#include "cmsis_os.h"
#include <string.h>

/* Private variables ---------------------------------------------------------*/

float g_motor_3508_target_speed = 0.0f;

#define HOST_RX_BUFFER_SIZE 128

#define MECANUM_CMD_VX 1.0f
#define MECANUM_CMD_VY 1.0f
#define MECANUM_CMD_VW 1.5f
#define MECANUM_DEMO_STEP_MS 1000U

#define MECANUM_DEMO_STEP_FORWARD 0U
#define MECANUM_DEMO_STEP_BACKWARD 1U
#define MECANUM_DEMO_STEP_LEFT 2U
#define MECANUM_DEMO_STEP_RIGHT 3U
#define MECANUM_DEMO_STEP_ROTATE_RIGHT 4U
#define MECANUM_DEMO_STEP_ROTATE_LEFT 5U
#define MECANUM_DEMO_STEP_COUNT 6U

// VOFA+ JustFloat 帧尾
static const uint8_t VOFA_TAIL[4] = {0x00, 0x00, 0x80, 0x7f};


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
 * 0x66 - 初始化抬腿姿态（保持）
 * 0x67 - 启动上20cm台阶自动流程（一次完整流程）
 * 0x68 - 启动上40cm台阶自动流程（一次完整流程）
 * 0x69 - 启动下20cm台阶自动流程（一次完整流程）
 * 0x91 - 下台阶手动下一步
 * 0x99 - 麦轮底盘演示流程（每步约1秒）：
 *        前进 -> 后退 -> 左移 -> 右移 -> 右旋 -> 左旋 -> 停车
 */
static void Host_UART_Callback(uint8_t *Buffer, uint16_t Length)
{
    if (Length == 0 || Buffer == NULL) return;

    switch (Buffer[0])
    {
        // ========== 顺序控制 ==========
        case 0x10: // 下一步
            Climbing_Manual_Next();
            break;

        // ========== 自动流程 ==========
        case 0x66:
            Climbing_Init_Pose_Start();
            break;
        case 0x67:
            Climbing_Auto_Start_20cm();
            break;
        case 0x68:
            Climbing_Auto_Start_40cm();
            break;
        case 0x69:
            Climbing_Descend_Auto_Start_20cm();
            break;
        case 0x65:
            Climbing_Descend_Manual_Next();
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
    ClimbingController &ctrl = Climbing_Get_Controller();

    // tx_data[0] = Motor_Lift_Rear.Get_Target_Omega();   // CH1
    // tx_data[1] = Motor_Lift_Rear.Get_Now_Omega();      // CH2
    // tx_data[2] = Motor_Lift_Rear.Get_Out();            // CH3
    // tx_data[3] = Motor_Lift_Rear.Get_Target_Angle();   // CH4
    // tx_data[4] = Motor_Lift_Rear.Get_Now_Angle();      // CH5
    // tx_data[5] = (float)g_climbState;                  // CH6
    // tx_data[6] = Motor_Lift_Front.Get_Out();           // CH7

    tx_data[0] = ctrl.GetFrontTargetAngle();  // CH1: 前腿目标
    tx_data[1] = ctrl.GetFrontNowAngle();     // CH2: 前腿实际
    tx_data[2] = ctrl.GetRearTargetAngle();   // CH3: 后腿目标
    tx_data[3] = ctrl.GetRearNowAngle();      // CH4: 后腿实际
    tx_data[4] = (float)ctrl.GetState();      // CH5: 状态机
    tx_data[5] = ctrl.GetFrontOut();          // CH6: 前腿PID输出
    tx_data[6] = ctrl.GetRearOut();           // CH7: 后腿PID输出

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
