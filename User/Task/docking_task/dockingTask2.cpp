//
// Created by chengfeng on 2026/4/15.
//

#include "dockingTask2.h"
#include "cmsis_os2.h"
#include "main.h"
#include "docking_controller.h"
#include "drv_can.h"
#include "mavlink.h"
#include "stm32f4xx_hal_gpio.h"
#include "drv_bsp.h"
#include "uart_printf.h"

DockingController dockingCtrl;

void Docking_CAN_Rx_Dispatch(Struct_CAN_Rx_Buffer *Rx_Buffer) {
    dockingCtrl.CAN_RxCallback(Rx_Buffer->Header.StdId, Rx_Buffer->Data);
}

void DockingTask() {
    dockingCtrl.Init(&hcan1);

    mavlink_apriltag_t vision_data;
    bool has_vision = false;
    osDelay(1000);
    
    for (;;) {

        if (dockingCtrl.Get_State() == DockingState::Testing)
        {
            has_vision = (osMessageQueueGet(apriltagQueueHandle, &vision_data, 0, 0) == osOK);
        }

        dockingCtrl.Update(has_vision ? &vision_data : nullptr);

        dockingCtrl.PeriodElapsedCallback();
        TIM_CAN_PeriodElapsedCallback();

        // dockingCtrl.Cylinder_Push();
        // uart_printf("%d\r\n", HAL_GPIO_ReadPin(Cylinder_CYLINDER_GPIO_PORT, Cylinder_CYLINDER_GPIO_PIN));
        // osDelay(2000);
        // dockingCtrl.Cylinder_Pull();
        // uart_printf("%d\r\n", HAL_GPIO_ReadPin(Cylinder_CYLINDER_GPIO_PORT, Cylinder_CYLINDER_GPIO_PIN));

        osDelay(1);
    }
}