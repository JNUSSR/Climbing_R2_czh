//
// Created by chengfeng on 2026/4/15.
//

#ifndef TEST_FEEDBACK_DOCKING_CONTROLLER_H
#define TEST_FEEDBACK_DOCKING_CONTROLLER_H

#include "dvc_motor.h"
#include "mavlink.h"
#include <cmath>
#include <algorithm>

enum class DockingState : uint8_t {
    Init,
    Testing,
    Running,
    Docking,
    Debug
};

// GPIO定义(气缸控制引脚)
#define Cylinder_CYLINDER_GPIO_PORT  GPIOE
#define Cylinder_CYLINDER_GPIO_PIN   GPIO_PIN_4
#define Cylinder_CYLINDER_POWER_GPIO_PORT GPIOF//引脚供电控制，默认为常电，拉高供电
#define Cylinder_CYLINDER_POWER_GPIO_PIN GPIO_PIN_1

class DockingController {
private:
    // 物理参数与控制常量
    static constexpr float LEAD_0 = 0.004f;//y方向的丝杆导程
    static constexpr float LEAD_1 = 0.001f;//x方向的丝杆导程
    static constexpr float Y_MIN  = 0.0f;
    static constexpr float Y_MAX  = 0.04f;
    static constexpr float X_MIN  = 0.0f;
    static constexpr float X_MAX  = 0.052f;
    static constexpr float PI_2   = 6.2831853f; // 2 * PI

    // --- 相机外参 (相机坐标系到机械执行末端坐标系的变换参数) ---
    // 安装平移偏移量 (单位: m)
    static constexpr float CAMERA_OFFSET_X = -0.014f;
    static constexpr float CAMERA_OFFSET_Y = -0.044f;
    // 相机安装俯仰角 (单位: 弧度，例如相机低头俯视作物 15度 = 15 * PI / 180)
    static constexpr float CAMERA_PITCH_RAD = 0.0f;   // 如果完全水平，填0即可

    static constexpr uint32_t motor_x_ID = 0x205;
    static constexpr uint32_t motor_y_ID = 0x206;

    Enum_CAN_Motor_ID enum_motor_x_ID = CAN_Motor_ID_0x205;
    Enum_CAN_Motor_ID enum_motor_y_ID = CAN_Motor_ID_0x206;

    Class_Motor_C610 motor[2];
    DockingState state;

    float current_x, current_y;
    float origin_x, origin_y;
    float debug_x, debug_y;
    bool docking_action_executed = false;

    // 内部计算函数
    float computeMotor0Angle(float target_y);
    float computeMotor1Angle(float target_x);
    void Cylinder_Init();

    // 视觉坐标系到机械坐标系的刚体变换解算
    void transformVisionToMechanical(float vision_x, float vision_z, float& out_mech_x, float& out_mech_y);

public:
    DockingController();

    // 气缸控制接口（用于独立测试）
    void Cylinder_Push();
    void Cylinder_Pull();

    // 初始化外设与参数
    void Init(CAN_HandleTypeDef* hcan);

    // 供外部CAN中断调用的接口
    void CAN_RxCallback(uint32_t stdId, uint8_t* data);

    // 主循环更新函数 (由 FreeRTOS 任务按特定频率调用)
    void Update(const mavlink_apriltag_t* vision_data);

    DockingState Get_State();

    // 定时器/CAN发送回调
    void PeriodElapsedCallback();
};

#endif //TEST_FEEDBACK_DOCKING_CONTROLLER_H