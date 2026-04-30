# climbing_task README

本文档对应：

- `UserTask/Climbing_task.h`
- `UserTask/Climbing_task.cpp`

用于说明 `climbing_task` 的整体结构、模块职责、主要函数、状态机流程与参数含义，方便后续调试与维护。

---

## 1. 模块定位

`climbing_task` 是整车动作控制核心，负责：

- 上台阶流程控制（手动/自动）
- 下台阶流程控制（手动/自动）
- 前后腿目标规划（斜坡规划）
- 前后腿 PID 参数按工况动态切换
- 重力补偿叠加与 CAN 输出注入
- 底盘在指定状态的前进控制

调度关系：

- `Climbing_Auto_Task_1ms()`：只做自动流程“状态推进”（按时间切状态）
- `Climbing_Task_Entry()`：每 1ms 执行具体控制（目标、PID、补偿、发送）

---

## 2. 文件结构说明

### 2.1 `Climbing_task.h`

主要内容：

- 统一数学宏（圈数转弧度、rpm 转 rad/s）
- 机械位置常量（上台/下台的目标位）
- 速度、时间、斜坡、PID、补偿参数
- 状态机枚举 `ClimbingState_e`
- 对外接口声明（自动流程、手动调试、急停）

### 2.2 `Climbing_task.cpp`

主要内容：

- 全局电机对象与斜坡对象
- 自动流程运行标志、模式标志、软零点变量
- 一组内部静态函数（分模块执行）
- 对外接口函数实现
- `Climbing_Task_Entry()` 主控制入口

---

## 3. 状态机说明

## 3.1 上台阶状态

- `STEP_IDLE`
- `STEP_SETUP`
- `STEP_WAIT_TRIGGER`
- `STEP_TOUCH_DOWN`
- `STEP_GLOBAL_LIFT`
- `STEP_DRIVE_FWD`
- `STEP_RETRACT`
- `STEP_DONE`

## 3.2 下台阶状态

- `STEP_DESCEND_SETUP`
- `STEP_DESCEND_WAIT_TRIGGER`
- `STEP_DESCEND_TOUCH`
- `STEP_DESCEND_GLOBAL_DOWN`
- `STEP_DESCEND_DRIVE`
- `STEP_DESCEND_RAISE`
- `STEP_DESCEND_DONE`

---

## 4. 代码模块（内部函数）说明

以下函数均在 `Climbing_task.cpp` 内部（`static`）：

- `Clamp_C620_Output(float out)`
  - 对 C620 输出做限幅（`[-16384, 16384]`）

- `Apply_Motor_Output_With_Comp(Class_Motor_C620 &motor, float comp, uint8_t can_byte_index)`
  - 将“PID 输出 + 补偿”写回电机对象
  - 并同步写入 `CAN1_0x200_Tx_Data` 指定字节位

- `Compute_Gravity_Compensation(float *front_comp, float *rear_comp)`
  - 按当前状态给出前/后腿补偿值
  - 上台阶与下台阶使用不同状态分支

- `Handle_State_Transition(uint32_t current_time, uint8_t state_changed)`
  - 处理状态切换瞬间逻辑：
  - 更新时间戳、离开等待状态时底盘立即停车、斜坡与真实角度对齐

- `Update_Pid_And_Slope_By_State()`
  - 根据状态切换 PID 工况（normal / lift）
  - 根据状态切换斜坡组（上台阶组A/B、下台阶组G1/G2）

- `Update_State_Targets(uint32_t current_time)`
  - 按状态设置腿目标角和轮子目标速度
  - 包含上台阶与下台阶动作目标

- `Update_Chassis_Control()`
  - 在非等待状态清零底盘速度
  - 周期执行 4 个底盘电机 PID 计算

- `Run_Lift_And_Wheel_Control()`
  - 执行前后腿斜坡计算
  - 下发腿目标角
  - 执行腿和轮电机 PID

- `Apply_Gravity_Compensation_And_Send()`
  - 计算补偿并注入输出
  - 调用 `TIM_CAN_PeriodElapsedCallback()` 发送

---

## 5. 对外接口函数说明

## 5.1 初始化与主循环

- `void Climbing_Task_Init(void)`
  - 初始化斜坡、电机、底盘、PID、状态变量

- `void Climbing_Task_Entry(void)`
  - 1ms 主控制入口
  - 顺序：状态处理 -> 参数切换 -> 软零点 -> 目标更新 -> PID执行 -> 补偿发送

## 5.2 自动流程接口

- `void Climbing_Auto_Start(void)`
  - 启动上台阶自动流程（一次）

- `void Climbing_Descend_Auto_Start(void)`
  - 启动下台阶自动流程（一次）
  - 记录下台阶起始角度（相对位姿起点）

- `void Climbing_Auto_Task_1ms(void)`
  - 自动流程状态推进（按时间参数）

- `uint8_t Climbing_Is_Auto_Running(void)`
  - 查询自动流程运行标志

## 5.3 手动调试接口

- `void Climbing_Manual_Next(void)`
  - 上台阶手动下一步（类似 0x10）

- `void Climbing_Descend_Manual_Next(void)`
  - 下台阶手动下一步（类似 0x91）

- `void Climbing_Manual_Reset(void)`
  - 复位到 `STEP_IDLE` 并停止轮速

- `void Climbing_Manual_Goto(ClimbingState_e state)`
  - 直接跳转到指定状态

- `void Climbing_Emergency_Stop(void)`
  - 急停，输出归零

---

## 6. 参数说明（`Climbing_task.h`）

## 6.1 机械位置参数

- `POS_FRONT_RETRACT` / `POS_REAR_RETRACT`
  - 收腿准备位

- `POS_FRONT_TOUCH` / `POS_REAR_TOUCH`
  - 上台阶触地位

- `POS_FRONT_LIFT` / `POS_REAR_LIFT`
  - 上台阶顶升位

- `POS_FRONT_FINAL` / `POS_REAR_FINAL`
  - 上台阶末端复位位

- `DESCEND_FRONT_TOUCH_TARGET` / `DESCEND_REAR_TOUCH_TARGET`
  - 下台阶触地阶段目标

- `DESCEND_FRONT_GLOBAL_DOWN_TARGET` / `DESCEND_REAR_GLOBAL_DOWN_TARGET`
  - 下台阶全局下降阶段目标

## 6.2 速度参数

- `DRIVE_SPEED_RPM` / `SPEED_DRIVE_FWD`
  - 轮子平移速度（速度环）

- `WAIT_TRIGGER_FORWARD_VX`
  - 上台阶等待状态下的底盘前进速度（wheelchassis）

- `DESCEND_WAIT_FORWARD_VX`
  - 下台阶等待状态下的底盘前进速度（wheelchassis）

## 6.3 斜坡参数

上台阶：

- 组A（`SETUP/RETRACT`）：
  - `FRONT/REAR_SLOPE_RPM_SETUP_RETRACT`
- 组B（`TOUCH_DOWN/GLOBAL_LIFT`）：
  - `FRONT/REAR_SLOPE_RPM_TOUCH_LIFT`

下台阶：

- 组G1（状态1/2，normal PID）：
  - `FRONT/REAR_SLOPE_RPM_DESC_G1`
- 组G2（状态3/4/5，lift PID）：
  - `FRONT/REAR_SLOPE_RPM_DESC_G2`

## 6.4 PID 参数

- `PID_*_NORMAL`
  - 常规工况参数

- `PID_*_LIFT`
  - 负载较大工况参数（顶升/下降主承载段）

## 6.5 重力补偿参数

- `GRAVITY_COMPENSATION_FRONT`
- `GRAVITY_COMPENSATION_REAR`
- `COMP_FRONT_LIFT`
- `COMP_REAR_LIFT`

说明：补偿值最终通过 `Apply_Motor_Output_With_Comp()` 注入到输出。

## 6.6 时间参数

上台阶自动：

- `TIME_SETUP`
- `WAIT_TRIGGER_FORWARD_TIME_MS`
- `TIME_TOUCH`
- `TIME_LIFT`
- `TIME_DRIVE`
- `TIME_RETRACT`

下台阶自动：

- `TIME_DESC_SETUP`
- `DESCEND_WAIT_TIME_MS`
- `TIME_DESC_TOUCH`
- `TIME_DESC_GLOBAL_DOWN`
- `DESCEND_DRIVE_TIME_MS`
- `TIME_DESC_RAISE`

---

## 7. 自动流程时序简图

上台阶自动：

`SETUP -> WAIT_TRIGGER -> TOUCH_DOWN -> GLOBAL_LIFT -> DRIVE_FWD -> RETRACT -> DONE`

下台阶自动：

`DESCEND_SETUP -> DESCEND_WAIT_TRIGGER -> DESCEND_TOUCH -> DESCEND_GLOBAL_DOWN -> DESCEND_DRIVE -> DESCEND_RAISE -> DESCEND_DONE`

---

## 8. 调试建议

- 先只调时间参数，确认动作顺序正确
- 再调斜坡速度，控制动作快慢与平滑度
- 最后调 PID 与补偿，解决负载不足/震荡
- 用上位机同时观察：状态、前后腿目标角、实际角、输出
