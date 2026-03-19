# alg_slope 使用说明

本文档说明 `User/alg_slope.h` 与 `User/alg_slope.cpp` 的使用方法、调用时序和常见问题。

## 1. 功能简介

`Class_Slope` 是一个离散斜坡规划器，用于让目标值按固定步长平滑变化，避免控制目标突变。

- 适用场景：电机位置目标平滑过渡、速度给定平滑变化
- 输出变量：`Out`
- 计算入口：`TIM_Calculate_PeriodElapsedCallback()`

注意：规划器本身不包含时间基准，周期由外部调用频率决定（例如 1ms 一次）。

## 2. 关键接口

### 2.1 初始化

```cpp
void Init(float __Increase_Value, float __Decrease_Value, Enum_Slope_First __Slope_First = Slope_First_REAL);
```

- `__Increase_Value`：每个控制周期向目标增加的最大步长
- `__Decrease_Value`：每个控制周期向目标减小的最大步长
- `__Slope_First`：优先策略
  - `Slope_First_REAL`：真实值优先
  - `Slope_First_TARGET`：目标值优先（硬规划）

### 2.2 输入/参数设置

```cpp
Set_Now_Real(float now_real);
Set_Target(float target);
Set_Increase_Value(float inc);
Set_Decrease_Value(float dec);
```

### 2.3 计算与输出

```cpp
TIM_Calculate_PeriodElapsedCallback();
float out = Get_Out();
```

## 3. 推荐调用时序

每个控制周期（例如 1ms）：

1. 更新真实值（必要时）：`Set_Now_Real(feedback)`
2. 写入目标值：`Set_Target(target)`
3. 调用斜坡计算：`TIM_Calculate_PeriodElapsedCallback()`
4. 读取输出：`Get_Out()` 并下发到控制器

示例（位置控制）：

```cpp
Slope_Pos.Set_Now_Real(motor.Get_Now_Angle());
Slope_Pos.Set_Target(target_angle);
Slope_Pos.TIM_Calculate_PeriodElapsedCallback();
motor.Set_Target_Angle(Slope_Pos.Get_Out());
```

## 4. 真实值优先与目标值优先

### 4.1 `Slope_First_REAL`

当 `Now_Real` 处于 `Now_Planning` 与 `Target` 之间时，输出会对齐到真实值，适合存在跟踪延迟、需要避免“规划跑飞”的场景。

### 4.2 `Slope_First_TARGET`

规划严格按步长向目标推进，不主动贴合真实反馈。适合仿真或执行器跟踪能力很强的场景。

## 5. 步长与速度换算

如果你希望按物理速度配置（如 rpm），可按下面思路换算：

```text
rad/s = rpm * 2*pi / 60
每周期步长(rad/tick) = rad/s / 控制频率(Hz)
```

例如控制频率 1000Hz，目标速度 30rpm：

```text
step = (30 * 2*pi / 60) / 1000 ≈ 0.00314 rad/tick
```

## 6. 常见问题

### 问题1：一上电输出跳变

原因：`Out`/`Now_Planning` 初值与机械真实位置不一致。

处理：初始化或切换控制阶段时，先调用一次：

```cpp
Slope.Set_Now_Real(feedback_now);
Slope.Set_Target(feedback_now);
```

`alg_slope.h` 里的 `Set_Now_Real()` 已经同步了 `Out` 和 `Now_Planning`，可用于快速对齐。

### 问题2：响应太慢或太冲

- 太慢：增大 `Increase_Value`/`Decrease_Value`
- 太冲：减小步长，或降低外环目标变化速度

### 问题3：状态切换时突兀

在状态切换点，建议先 `Set_Now_Real()` 同步，再更新新目标，避免跨状态遗留误差造成突变。

## 7. 在你当前工程中的典型用法

你在 `Climbing_task.cpp` 中使用了两个规划器：

- `Slope_Front_Pos`
- `Slope_Rear_Pos`

并在 1ms 周期中先算斜坡、再将 `Get_Out()` 写给电机角度目标。这是正确用法。

如果后续需要更细调节，可为不同状态配置不同步长（比如 `GLOBAL_LIFT` 更小步长，`RETRACT` 更大步长）。
