# Climbing 模块说明

本文档说明当前 `ClimbingController + climbingTask2 + host_communication_task` 的协作关系、状态机和指令映射。

对应文件：

- `User/Module/Climbing/climbing_controller.h`
- `User/Module/Climbing/climbing_controller.cpp`
- `User/Task/climbing_task/climbingTask2.h`
- `User/Task/climbing_task/climbingTask2.cpp`
- `User/Task/host_communication_task.cpp`

---

## 1. 架构总览

当前采用 **Module + Task** 分层：

- `ClimbingController`（Module）
  - 负责全部攀爬控制逻辑（状态机、目标规划、PID切换、重力补偿、CAN输出）
- `climbingTask2`（Task）
  - 负责任务周期调用和对外兼容接口转发
- `host_communication_task`（串口命令）
  - 负责把上位机字节命令映射为攀爬动作接口

调用链：

1. 上位机发命令（UART）
2. `Host_UART_Callback()` 解析命令
3. 调用 `Climbing_*` 对外函数（在 `climbingTask2`）
4. `climbingTask2` 转发给 `ClimbingController`
5. `ClimbingTask()` 周期执行 `AutoTask1ms()` + `TaskEntry1ms()`

---

## 2. ClimbingController（核心控制）

### 2.1 主要职责

- 管理上台阶/下台阶状态机
- 按状态设置前后腿和左右轮目标
- 按工况切换 normal/lift PID
- 叠加重力补偿并发 CAN
- 提供自动流程、手动流程、急停、初始化姿态等接口

### 2.2 状态机

上台阶：

`STEP_IDLE -> STEP_SETUP -> STEP_WAIT_TRIGGER -> STEP_TOUCH_DOWN -> STEP_GLOBAL_LIFT -> STEP_DRIVE_FWD -> STEP_RETRACT -> STEP_DONE`

下台阶：

`STEP_DESCEND_SETUP -> STEP_DESCEND_WAIT_TRIGGER -> STEP_DESCEND_TOUCH -> STEP_DESCEND_GLOBAL_DOWN -> STEP_DESCEND_DRIVE -> STEP_DESCEND_RAISE -> STEP_DESCEND_DONE`

### 2.3 两种上台阶模式

通过 `ClimbUpMode_e` 选择：

- `CLIMB_UP_MODE_20CM`
- `CLIMB_UP_MODE_40CM`

在以下状态按模式使用不同目标参数：

- `STEP_SETUP`
- `STEP_TOUCH_DOWN`
- `STEP_GLOBAL_LIFT`

### 2.4 初始化抬腿模式（0x66）

- 接口：`InitPoseStart()`
- 行为：进入并保持初始化抬腿目标
  - `POS_FRONT_Init`
  - `POS_REAR_Init`
- 不会在上电自动进入，仅收到 `0x66` 后进入
- 收到其他动作指令会退出该模式

---

## 3. climbingTask2（任务层）

### 3.1 任务入口

`ClimbingTask()` 内部流程：

- `climbingCtrl.Init(&hcan1)`
- 1ms循环：
  - `climbingCtrl.AutoTask1ms()`
  - `climbingCtrl.TaskEntry1ms()`

### 3.2 兼容接口

保留并转发旧接口（方便其他文件不大改）：

- `Climbing_Task_Init()`
- `Climbing_Task_Entry()`
- `Climbing_Auto_Task_1ms()`
- `Climbing_Manual_*`
- `Climbing_Emergency_Stop()`

新增动作接口：

- `Climbing_Auto_Start_20cm()`
- `Climbing_Auto_Start_40cm()`
- `Climbing_Descend_Auto_Start_20cm()`
- `Climbing_Init_Pose_Start()`

CAN分发接口：

- `Climbing_CAN_Rx_Dispatch(Struct_CAN_Rx_Buffer *Rx_Buffer)`

---

## 4. host_communication_task（命令映射）

当前关键命令如下：

- `0x66`：初始化抬腿姿态（保持）
- `0x67`：上 20cm 台阶（自动一次）
- `0x68`：上 40cm 台阶（自动一次）
- `0x69`：下 20cm 台阶（自动一次）
- `0x10`：手动下一步（上台阶序列）
- `0x91`：下台阶手动下一步
- `0x11`：复位到 IDLE
- `0x12`：急停

---

## 5. 关键参数（当前版本）

### 5.1 初始化姿态

- `POS_FRONT_Init = movingmm(-220.0f)`
- `POS_REAR_Init  = movingmm(-10.0f)`

### 5.2 上20cm

- `POS_FRONT_RETRACT_20cm = movingmm(-220.0f)`
- `POS_REAR_RETRACT_20cm  = movingmm(-10.0f)`
- `POS_FRONT_TOUCH_20cm   = movingmm(-200.0f)`
- `POS_REAR_TOUCH_20cm    = movingmm(0.0f)`
- `POS_FRONT_LIFT_20cm    = movingmm(20.0f)`
- `POS_REAR_LIFT_20cm     = movingmm(220.0f)`

### 5.3 上40cm

- `POS_FRONT_RETRACT_40cm = movingmm(-420.0f)`
- `POS_REAR_RETRACT_40cm  = movingmm(-75.0f)`
- `POS_FRONT_TOUCH_40cm   = movingmm(-400.0f)`
- `POS_REAR_TOUCH_40cm    = movingmm(0.0f)`
- `POS_FRONT_LIFT_40cm    = movingmm(20.0f)`
- `POS_REAR_LIFT_40cm     = movingmm(413.0f)`

### 5.4 下20cm

- `DESCEND_FRONT_TOUCH_TARGET       = movingmm(20.0f)`
- `DESCEND_REAR_TOUCH_TARGET        = movingmm(0.0f)`
- `DESCEND_FRONT_GLOBAL_DOWN_TARGET = movingmm(28.0f)`
- `DESCEND_REAR_GLOBAL_DOWN_TARGET  = movingmm(8.0f)`

---

## 6. 调试建议

- 先测命令链路：`0x66 -> 0x67 -> 0x68 -> 0x69`
- 再看状态变化是否符合预期（VOFA波形里的 state 通道）
- 若动作时序对但姿态不准，优先调位置参数；若抖动/无力，再调 PID 与补偿
