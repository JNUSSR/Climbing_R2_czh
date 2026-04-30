//
// Created by chengfeng on 2026/4/15.
//

#include "docking_controller.h"
#include "cmsis_os2.h"
#include "drv_bsp.h"
#include "main.h"
#include "stm32f4xx_hal_gpio.h"
#include "cmsis_os.h"

DockingController::DockingController()
    : state(DockingState::Init),
      current_x(X_MIN), current_y(Y_MAX),
      origin_x(X_MIN), origin_y(Y_MAX),
      debug_x(0),debug_y(0)
{}

void DockingController::Cylinder_Init() {
    // 默认拉回，避免上电时气缸误动作
    HAL_GPIO_WritePin(Cylinder_CYLINDER_GPIO_PORT, Cylinder_CYLINDER_GPIO_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Cylinder_CYLINDER_POWER_GPIO_PORT, Cylinder_CYLINDER_POWER_GPIO_PIN, GPIO_PIN_SET);
}

void DockingController::Cylinder_Push() {
    HAL_GPIO_WritePin(Cylinder_CYLINDER_GPIO_PORT, Cylinder_CYLINDER_GPIO_PIN, GPIO_PIN_SET);
}

void DockingController::Cylinder_Pull() {
    HAL_GPIO_WritePin(Cylinder_CYLINDER_GPIO_PORT, Cylinder_CYLINDER_GPIO_PIN, GPIO_PIN_RESET);
}

void DockingController::Init(CAN_HandleTypeDef* hcan) {
    motor[0].PID_Omega.Init(500.0f, 200.0f, 0.0f, 0.0f, 1000.0f, 3000.0f);
    motor[1].PID_Omega.Init(500.0f, 200.0f, 0.0f, 0.0f, 1000.0f, 3000.0f);
    motor[0].PID_Angle.Init(5.0f, 0.0f, 0.0f, 0.0f, 0.0f, 10.0f);
    motor[1].PID_Angle.Init(5.0f, 0.0f, 0.0f, 0.0f, 0.0f, 40.0f);

    motor[0].Init(hcan, enum_motor_y_ID, Control_Method_ANGLE);
    motor[1].Init(hcan, enum_motor_x_ID, Control_Method_ANGLE);

    Cylinder_Init();
}

void DockingController::CAN_RxCallback(uint32_t stdId, uint8_t* data) {
    if (stdId == motor_y_ID) {
        motor[0].CAN_RxCpltCallback(data);
    } else if (stdId == motor_x_ID) {
        motor[1].CAN_RxCpltCallback(data);
    }
}

float DockingController::computeMotor0Angle(float target_y) {
    float clamped_y = std::clamp(target_y, Y_MIN, Y_MAX);
    return -(clamped_y - origin_y) / LEAD_0 * PI_2;
}

float DockingController::computeMotor1Angle(float target_x) {
    float clamped_x = std::clamp(target_x, X_MIN, X_MAX);
    return (clamped_x - origin_x) / LEAD_1 * PI_2;
}

// 基于齐次变换矩阵的坐标系映射 (包含倾角补偿)
void DockingController::transformVisionToMechanical(float vision_x, float vision_z, float& out_mech_x, float& out_mech_y) {
    // X轴通常不受俯仰角影响 (假设仅绕X轴倾斜)
    float transformed_x = vision_x;

    // Z轴深度由于相机低头/抬头，投影在机械水平Y轴上会产生三角函数变化
    // 这里使用旋转矩阵展开的简化形式 (相机坐标系Z向前，Y向下)
    float transformed_z_to_y = vision_z * std::cos(CAMERA_PITCH_RAD);

    out_mech_x = transformed_x + CAMERA_OFFSET_X + current_x;
    out_mech_y = transformed_z_to_y + CAMERA_OFFSET_Y + current_y;
}

void DockingController::Update(const mavlink_apriltag_t* vision_data) {
    switch (state) {
        case DockingState::Init: {
            // 每轮任务开始前，气缸回到上拉状态
            Cylinder_Pull();
            docking_action_executed = false;

            current_x = X_MAX / 2.0f;
            current_y = Y_MIN;

            float error0 = std::fabs(motor[0].Get_Now_Angle() - computeMotor0Angle(current_y));
            float error1 = std::fabs(motor[1].Get_Now_Angle() - computeMotor1Angle(current_x));

            if (error0 < 0.5f && error1 < 0.5f) {
                state = DockingState::Testing;
            }
            break;
        }
        case DockingState::Testing: {
            if (vision_data != nullptr) {
                float target_mech_x = 0.0f;
                float target_mech_y = 0.0f;

                // 进行包含姿态矩阵的坐标转换
                transformVisionToMechanical(vision_data->x, vision_data->z, target_mech_x, target_mech_y);

                if (target_mech_x < X_MAX && target_mech_x > X_MIN &&
                    target_mech_y < Y_MAX && target_mech_y > Y_MIN) {
                    current_x = target_mech_x;
                    current_y = target_mech_y;
                    state = DockingState::Running;
                }
            }
            break;
        }
        case DockingState::Running: {
            float error0 = std::fabs(motor[0].Get_Now_Angle() - computeMotor0Angle(current_y));
            float error1 = std::fabs(motor[1].Get_Now_Angle() - computeMotor1Angle(current_x));

            if (error0 < 0.5f && error1 < 0.5f) {
                state = DockingState::Docking;
            }
            break;
        }
        case DockingState::Docking:{
            // 对接动作：气缸下压
            if (!docking_action_executed) {
                Cylinder_Push();
                docking_action_executed = true;
            }
            osDelay(1000); // 等待气缸动作完成
            state = DockingState::Init; // 循环回初始状态，准备下一轮对接

            break;
        }
        case DockingState::Debug: {
            current_x = debug_x;//通过调试修改状态和debug_x debug_y值
            current_y = debug_y;
            break;
        }
    }

    // 2. 将最终计算的角度下发给底层控制
    if (state != DockingState::Debug) {
        motor[0].Set_Target_Angle(computeMotor0Angle(current_y));
        motor[1].Set_Target_Angle(computeMotor1Angle(current_x));
    } else {
        // Debug 模式使用 0 作为原点计算
        motor[0].Set_Target_Angle(-(current_y - 0) / LEAD_0 * PI_2);
        motor[1].Set_Target_Angle((current_x - 0) / LEAD_1 * PI_2);
    }
}

void DockingController::PeriodElapsedCallback() {
    motor[0].TIM_PID_PeriodElapsedCallback();
    motor[1].TIM_PID_PeriodElapsedCallback();
}

DockingState DockingController::Get_State()
{
    return state;
}
