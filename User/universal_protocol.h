/**
 * @file universal_protocol.h
 * @brief 通用上下位机通信协议定义
 * @version 1.0
 * @date 2025-11-23
 * 
 * 协议特点：
 * - 双向通信支持
 * - 多设备/多命令支持
 * - CRC8 + CRC16 双重校验
 * - 序列号防丢包
 * - 可变长度数据段
 */

#ifndef UNIVERSAL_PROTOCOL_H
#define UNIVERSAL_PROTOCOL_H

#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif

#pragma pack(push, 1)

/*============================ 帧格式定义 ============================*/

// 帧头定义
#define FRAME_HEADER_SOF        0xA5    // 帧起始标志
#define FRAME_HEADER_LEN        5       // 帧头长度(不含SOF)
#define FRAME_MAX_DATA_LEN      64      // 数据段最大长度
#define FRAME_TAIL_LEN          2       // 帧尾CRC16长度

// 帧头结构 (5字节)
typedef struct {
    uint8_t  sof;           // 起始字节 0xA5
    uint16_t data_length;   // 数据段长度(cmd_id + data)
    uint8_t  seq;           // 包序号(循环0-255)
    uint8_t  crc8;          // 帧头CRC8校验
} FrameHeader;

/*============================ 命令ID定义 ============================*/

// 命令ID枚举 (0x0000-0xFFFF)
typedef enum {
    /*---------- 上位机 -> 下位机 (0x00xx) ----------*/
    
    // 底盘控制 (0x01xx)
    CMD_CHASSIS_SPEED       = 0x0101,  // 底盘速度控制(vx,vy,wz)
    CMD_CHASSIS_POSITION    = 0x0102,  // 底盘位置控制(x,y,yaw)
    CMD_CHASSIS_MODE        = 0x0103,  // 底盘模式切换
    
    // 机械臂控制 (0x02xx)
    CMD_ARM_JOINT_POS       = 0x0201,  // 机械臂关节位置
    CMD_ARM_JOINT_VEL       = 0x0202,  // 机械臂关节速度
    CMD_ARM_CARTESIAN       = 0x0203,  // 末端笛卡尔坐标
    CMD_ARM_GRIPPER         = 0x0204,  // 夹爪控制
    
    // 云台控制 (0x03xx)
    CMD_GIMBAL_ANGLE        = 0x0301,  // 云台角度控制(pitch,yaw)
    CMD_GIMBAL_SPEED        = 0x0302,  // 云台速度控制
    
    // 单个电机控制 (0x04xx)
    CMD_MOTOR_CONTROL       = 0x0401,  // 单电机控制
    CMD_MOTOR_GROUP         = 0x0402,  // 电机组控制
    
    // 系统命令 (0x0Fxx)
    CMD_SYSTEM_RESET        = 0x0F01,  // 系统复位
    CMD_SYSTEM_CONFIG       = 0x0F02,  // 系统配置
    CMD_HEARTBEAT           = 0x0FFF,  // 心跳包
    
    /*---------- 下位机 -> 上位机 (0x80xx) ----------*/
    
    // 底盘反馈 (0x81xx)
    CMD_CHASSIS_FEEDBACK    = 0x8101,  // 底盘状态反馈
    CMD_CHASSIS_ODOM        = 0x8102,  // 里程计数据
    
    // 机械臂反馈 (0x82xx)
    CMD_ARM_FEEDBACK        = 0x8201,  // 机械臂状态反馈
    CMD_ARM_JOINT_STATE     = 0x8202,  // 关节状态
    
    // 云台反馈 (0x83xx)
    CMD_GIMBAL_FEEDBACK     = 0x8301,  // 云台状态反馈
    
    // 传感器数据 (0x90xx)
    CMD_IMU_DATA            = 0x9001,  // IMU数据
    CMD_ENCODER_DATA        = 0x9002,  // 编码器数据
    CMD_FORCE_SENSOR        = 0x9003,  // 力传感器数据
    
    // 系统状态 (0xF0xx)
    CMD_SYSTEM_STATUS       = 0xF001,  // 系统状态
    CMD_ERROR_REPORT        = 0xFE00,  // 错误报告
    CMD_DEBUG_INFO          = 0xFF00,  // 调试信息
} CommandID;

/*============================ 数据包定义 ============================*/

// 底盘速度控制 (CMD_CHASSIS_SPEED)
typedef struct {
    float vx;           // 前进速度 (m/s)
    float vy;           // 横向速度 (m/s)
    float wz;           // 旋转角速度 (rad/s)
} ChassisSpeedCmd;

// 底盘位置控制 (CMD_CHASSIS_POSITION)
typedef struct {
    float x;            // 目标X位置 (m)
    float y;            // 目标Y位置 (m)
    float yaw;          // 目标航向角 (rad)
} ChassisPositionCmd;

// 底盘反馈数据 (CMD_CHASSIS_FEEDBACK)
typedef struct {
    uint32_t timestamp;     // 时间戳 (ms)
    float position_x;       // X位置 (m)
    float position_y;       // Y位置 (m)
    float yaw;              // 航向角 (rad)
    float velocity_x;       // 实际速度X (m/s)
    float velocity_y;       // 实际速度Y (m/s)
    float angular_vel;      // 实际角速度 (rad/s)
    uint8_t status;         // 状态标志
} ChassisFeedback;

// 机械臂关节位置控制 (CMD_ARM_JOINT_POS)
#define MAX_JOINT_NUM 6
typedef struct {
    uint8_t joint_num;          // 关节数量
    float positions[MAX_JOINT_NUM];  // 各关节目标位置 (rad)
} ArmJointPosCmd;

// 机械臂反馈数据 (CMD_ARM_FEEDBACK)
typedef struct {
    uint32_t timestamp;         // 时间戳 (ms)
    uint8_t joint_num;          // 关节数量
    float positions[MAX_JOINT_NUM];     // 当前位置 (rad)
    float velocities[MAX_JOINT_NUM];    // 当前速度 (rad/s)
    float efforts[MAX_JOINT_NUM];       // 当前力矩/电流 (N·m or A)
    uint8_t status[MAX_JOINT_NUM];      // 各关节状态
} ArmFeedback;

// 云台角度控制 (CMD_GIMBAL_ANGLE)
typedef struct {
    float pitch;        // 俯仰角 (rad)
    float yaw;          // 偏航角 (rad)
} GimbalAngleCmd;

// 电机控制 (CMD_MOTOR_CONTROL)
typedef struct {
    uint8_t motor_id;       // 电机ID (0-255)
    uint8_t control_mode;   // 控制模式: 0=停止, 1=速度, 2=位置, 3=力矩
    float target_value;     // 目标值
} MotorControlCmd;

// IMU数据 (CMD_IMU_DATA)
typedef struct {
    uint32_t timestamp;     // 时间戳 (ms)
    float accel[3];         // 加速度 (m/s²) [x,y,z]
    float gyro[3];          // 角速度 (rad/s) [x,y,z]
    float quaternion[4];    // 四元数 [w,x,y,z]
} IMUData;

// 系统状态 (CMD_SYSTEM_STATUS)
typedef struct {
    uint32_t timestamp;         // 时间戳 (ms)
    uint8_t battery_level;      // 电池电量 (0-100%)
    float battery_voltage;      // 电池电压 (V)
    float battery_current;      // 电池电流 (A)
    uint16_t error_flags;       // 错误标志位
    uint8_t system_mode;        // 系统模式
    uint8_t reserved;           // 保留字节
} SystemStatus;

// 错误报告 (CMD_ERROR_REPORT)
typedef struct {
    uint32_t timestamp;     // 时间戳 (ms)
    uint16_t error_code;    // 错误代码
    uint8_t device_id;      // 设备ID
    uint8_t severity;       // 严重程度: 0=info, 1=warning, 2=error, 3=fatal
    char message[32];       // 错误描述
} ErrorReport;

// 心跳包 (CMD_HEARTBEAT)
typedef struct {
    uint32_t timestamp;     // 时间戳 (ms)
    uint8_t alive;          // 存活标志 (0xAA)
} Heartbeat;

#pragma pack(pop)

/*============================ 函数接口 ============================*/

/**
 * @brief 计算CRC8校验值（用于帧头）
 * @param data 数据指针
 * @param length 数据长度
 * @return uint8_t CRC8值
 */
uint8_t CRC8_Calculate(const uint8_t *data, uint16_t length);

/**
 * @brief 计算CRC16校验值（用于整帧）
 * @param data 数据指针
 * @param length 数据长度
 * @return uint16_t CRC16值
 */
uint16_t CRC16_Calculate(const uint8_t *data, uint16_t length);

/**
 * @brief 打包数据帧
 * @param cmd_id 命令ID
 * @param data 数据指针
 * @param data_len 数据长度
 * @param seq 序列号
 * @param out_buffer 输出缓冲区
 * @return uint16_t 打包后的总长度
 */
uint16_t Protocol_PackFrame(uint16_t cmd_id, const uint8_t *data, uint16_t data_len, 
                            uint8_t seq, uint8_t *out_buffer);

/**
 * @brief 解析数据帧
 * @param buffer 接收缓冲区
 * @param length 接收长度
 * @param cmd_id 输出命令ID
 * @param data 输出数据指针
 * @param data_len 输出数据长度
 * @return uint8_t 1=成功, 0=失败
 */
uint8_t Protocol_ParseFrame(const uint8_t *buffer, uint16_t length, 
                            uint16_t *cmd_id, uint8_t **data, uint16_t *data_len);

/**
 * @brief 验证帧头
 * @param header 帧头指针
 * @return uint8_t 1=有效, 0=无效
 */
uint8_t Protocol_VerifyHeader(const FrameHeader *header);

#ifdef __cplusplus
}
#endif

#endif // UNIVERSAL_PROTOCOL_H
