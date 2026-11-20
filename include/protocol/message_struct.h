 /* message_struct.h
 *
 * Created on: Nov 19, 2026 11:47
 * Description:
 *  all values are using SI units (e.g. meter/second/radian)
 *
 * 
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

// 协议基础枚举必须先于消息结构定义，保证头文件可独立编译。
typedef enum {
  CONST_OFF = 0x00,
  CONST_ON = 0x01,
  BREATH = 0x02,
  CUSTOM = 0x03
} AgxLightMode;

typedef struct {
  AgxLightMode mode;
  uint8_t custom_value;
} AgxLightOperation;

typedef enum {
  VEHICLE_STATE_NORMAL = 0x00,
  VEHICLE_STATE_ESTOP = 0x01,
  VEHICLE_STATE_EXCEPTION = 0x02
} AgxVehicleState;

typedef enum {
  CONTROL_MODE_STANDBY = 0x00,
  CONTROL_MODE_CAN = 0x01,
  CONTROL_MODE_UART = 0x02,
  CONTROL_MODE_RC = 0x03
} AgxControlMode;

typedef enum {
  BRAKE_MODE_UNLOCK = 0x00,
  BRAKE_MODE_LOCK = 0x01
} AgxBrakeMode;

typedef enum {
  RC_SWITCH_UP = 0,
  RC_SWITCH_MIDDLE,
  RC_SWITCH_DOWN
} AgxRcSwitchState;

/***************** Control messages *****************/

//纵向速度 ,车身角速度,横向速度,转向角
typedef struct {
  float linear_velocity;
  float angular_velocity;
  float lateral_velocity;
  float steering_angle;
} MotionCommandMessage;

//表示是否启用指令灯光控制,分别保存前后灯操作
typedef struct {
  bool enable_cmd_ctrl;
  AgxLightOperation front_light;
  AgxLightOperation rear_light;
} LightCommandMessage;

//表示刹车锁定与释放
typedef struct {
  bool enable_braking;
} BrakingCommandMessage;

//运动模式消息
typedef struct {
  uint8_t motion_mode;
} MotionModeCommandMessage;


/***************** state feedback messages *****************/

//系统状态消息,正常、急,停或异常状态,CAN、串口、遥控器等控制模式,电池电压 ,错误标志集合
typedef struct {
  AgxVehicleState vehicle_state;
  AgxControlMode control_mode;
  float battery_voltage;
  uint16_t error_code;
} SystemStateMessage;

//运动状态消息,线速度、角速度、横向速度和转向角
typedef struct {
  float linear_velocity;
  float angular_velocity;  // only valid for differential drivering
  float lateral_velocity;
  float steering_angle;  // only valid for ackermann steering
} MotionStateMessage;



// for ranger,运动模式反馈：  
typedef struct {
  uint8_t motion_mode;
  uint8_t mode_changing;
} MotionModeStateMessage;


typedef LightCommandMessage LightStateMessage;

//遥控器反馈,当前运动模式和是否正在切换模式
typedef struct {
  AgxRcSwitchState swa;
  AgxRcSwitchState swb;
  AgxRcSwitchState swc;
  AgxRcSwitchState swd;
  int8_t stick_right_v;
  int8_t stick_right_h;
  int8_t stick_left_v;
  int8_t stick_left_h;
  int8_t var_a;
} RcStateMessage;


/***************** actuator feedback messages *****************/

//执行器高频反馈,电机编号,每分钟转数,电流,脉冲计数
typedef struct {
  uint8_t motor_id;
  int16_t rpm;
  float current;
  int32_t pulse_count;
} ActuatorHSStateMessage;

//执行器低频反馈与驱动状态位,电机编号。驱动器电压。驱动器温度。电机温度。驱动状态位。
typedef struct {
  uint8_t motor_id;
  float driver_voltage;
  float driver_temp;
  float motor_temp;
  uint8_t driver_state;
} ActuatorLSStateMessage;

//电机角度：
typedef struct {
  float angle_5;
  float angle_6;
  float angle_7;
  float angle_8;
} MotorAngleMessage;

//电机速度
typedef struct {
  float speed_1;
  float speed_2;
  float speed_3;
  float speed_4;
} MotorSpeedMessage;


/***************** Sensor messages ******************/

typedef struct {
  float left_wheel;
  float right_wheel;
} OdometryMessage;

typedef struct {
  float accel_x;
  float accel_y;
  float accel_z;
} ImuAccelMessage;

typedef struct {
  float gyro_x;
  float gyro_y;
  float gyro_z;
} ImuGyroMessage;

//欧拉角表示的姿态信息,分别为偏航角、俯仰角和横滚角
typedef struct {
  float yaw;
  float pitch;
  float roll;
} ImuEulerMessage;

//安全碰撞传感器反馈,触发状态
typedef struct {
  uint8_t trigger_state;
} SafetyBumperMessage;

//超声波相关数据
typedef struct {
  uint8_t sensor_id;
  uint8_t distance[8];
} UltrasonicMessage;

//uwb相对距离、角度、状态和通道数据
typedef struct {
  uint8_t sensor_id;
  float relative_distance;
  float relative_angle;
  bool is_normal;
  int8_t channels[3];
} UwbMessage;

//电池基本状态,剩余电量状态 SOC,电池健康状态 SOH,电压 电流,温度
typedef struct {
  uint8_t battery_soc;
  uint8_t battery_soh;
  float voltage;
  float current;
  float temperature;
} BmsBasicMessage;

//BMS 扩展状态和保护掩码
typedef struct {
  uint8_t alarm_status_1;
  uint8_t alarm_status_2;
  uint8_t warn_status_1;
  uint8_t warn_status_2;
} BmsExtendedMessage;



/************  Query/config messages ****************/

//版本请求消息
typedef struct {
  bool request;
} VersionRequestMessage;

//版本响应消息,包含一个字节数组,用于存储版本信息
typedef struct {
  uint8_t bytes[8];
} VersionResponseMessage;

//模式配置：
typedef struct {
  AgxControlMode mode;
} ControlModeConfigMessage;

//刹车配置：
typedef struct {
  AgxBrakeMode mode;
} BrakeModeConfigMessage;

//转向中立位设置请求
typedef struct {
  bool set_as_neutral;
} SteerNeutralRequestMessage;

//转向中立位设置响应
typedef struct {
  bool neutral_set_successful;
} SteerNeutralResponseMessage;

//故障清除编号：
typedef enum {
  CLEAR_ALL_FAULT = 0x00,
  CLEAR_MOTOR1_FAULT = 0x01,
  CLEAR_MOTOR2_FAULT = 0x02,
  CLEAR_MOTOR3_FAULT = 0x03,
  CLEAR_MOTOR4_FAULT = 0x04
} FaultClearCode;

///状态复位配置消息,包含一个字节,用于指定要清除的故障编号
typedef struct {
  uint8_t error_clear_byte;
} StateResetConfigMessage;




//////////////////////////////////////////////////////

//MsgType`：告诉程序当前消息属于哪一类,SDK 内部的消息类型标签
typedef enum {

  MsgUnkonwn = 0x00,

  // command
  MsgMotionCommand,
  MsgLightCommand,
  MsgBrakingCommand,
  MsgSetMotionModeCommand,

  
  // state feedback
  MsgSystemState,
  MsgMotionState,
  MsgLightState,
  MsgMotionModeState,
  MsgRcState,

  
  // actuator feedback
  MsgActuatorHSState,
  MsgActuatorLSState,
  MsgMotorAngle,
  MsgMotorSpeed,
  
  // sensor
  MsgOdometry,
  MsgImuAccel,
  MsgImuGyro,
  MsgImuEuler,
  MsgSafetyBumper,
  MsgUltrasonic,
  MsgUwb,
  MsgBmsBasic,
  MsgBmsExtended,
  
  
  // query/config
  MsgVersionRequest,
  MsgVersionResponse,
  MsgControlModeConfig,
  MsgSteerNeutralRequest,
  MsgSteerNeutralResponse,
  MsgStateResetConfig,
  MsgBrakeModeConfig,

} MsgType;



typedef struct {

  //类型标签加联合体,一个统一消息对象，包含“当前是什么消息”和“这条消息的数据”。
  MsgType type;  //消息类型标签。

  union {        //消息数据存储区。
   
    // command
    MotionCommandMessage motion_command_msg;
    LightCommandMessage light_command_msg;
    BrakingCommandMessage braking_command_msg;
    MotionModeCommandMessage motion_mode_msg;


    // core state feedback
    SystemStateMessage system_state_msg;
    MotionStateMessage motion_state_msg;
    LightStateMessage light_state_msg;
    MotionModeStateMessage motion_mode_state_msg;
    RcStateMessage rc_state_msg;


    // actuator feedback
    ActuatorHSStateMessage actuator_hs_state_msg;
    ActuatorLSStateMessage actuator_ls_state_msg;


    // sensor
    OdometryMessage odometry_msg;
    ImuAccelMessage imu_accel_msg;
    ImuGyroMessage imu_gyro_msg;
    ImuEulerMessage imu_euler_msg;
    SafetyBumperMessage safety_bumper_msg;
    UltrasonicMessage ultrasonic_msg;
    UwbMessage uwb_msg;
    BmsBasicMessage bms_basic_msg;
    BmsExtendedMessage bms_extended_msg;


    // query/config
    VersionRequestMessage version_request_msg;
    VersionResponseMessage version_response_msg;
    ControlModeConfigMessage control_mode_config_msg;
    BrakeModeConfigMessage brake_mode_config_msg;
    SteerNeutralRequestMessage steer_neutral_request_msg;
    SteerNeutralResponseMessage steer_neutral_response_msg;
    StateResetConfigMessage state_reset_config_msg;

    MotorAngleMessage motor_angle_msg;
    MotorSpeedMessage motor_speed_msg;

  } body;
  
} Message;

// 兼容早期解析器中的命名，后续代码统一使用 Message/Msg*。
typedef Message AgxMessage;
#define AgxMsgUnkonwn MsgUnkonwn
#define AgxMsgMotionCommand MsgMotionCommand
#define AgxMsgLightCommand MsgLightCommand
#define AgxMsgBrakingCommand MsgBrakingCommand
#define AgxMsgSetMotionModeCommand MsgSetMotionModeCommand
#define AgxMsgSystemState MsgSystemState
#define AgxMsgMotionState MsgMotionState
#define AgxMsgLightState MsgLightState
#define AgxMsgMotionModeState MsgMotionModeState
#define AgxMsgRcState MsgRcState
#define AgxMsgActuatorHSState MsgActuatorHSState
#define AgxMsgActuatorLSState MsgActuatorLSState
#define AgxMsgOdometry MsgOdometry
#define AgxMsgImuAccel MsgImuAccel
#define AgxMsgImuGyro MsgImuGyro
#define AgxMsgImuEuler MsgImuEuler
#define AgxMsgSafetyBumper MsgSafetyBumper
#define AgxMsgUltrasonic MsgUltrasonic
#define AgxMsgUwb MsgUwb
#define AgxMsgBmsBasic MsgBmsBasic
#define AgxMsgBmsExtended MsgBmsExtended
#define AgxMsgVersionRequest MsgVersionRequest
#define AgxMsgVersionResponse MsgVersionResponse
#define AgxMsgControlModeConfig MsgControlModeConfig
#define AgxMsgBrakeModeConfig MsgBrakeModeConfig
#define AgxMsgSteerNeutralRequest MsgSteerNeutralRequest
#define AgxMsgSteerNeutralResponse MsgSteerNeutralResponse
#define AgxMsgStateResetConfig MsgStateResetConfig
#define AgxMsgMotorAngle MsgMotorAngle
#define AgxMsgMotorSpeed MsgMotorSpeed




/**************** Feedback messages *****************/

//系统掩码：一个整数可以同时表示多个状态

//底盘系统错误  SystemStateMessage::error_code
#define SYSTEM_ERROR_BATTERY_FAULT_MASK ((uint16_t)0x0001)          //0000 0000 0000 0001     电池严重错误
#define SYSTEM_ERROR_BATTERY_WARNING_MASK ((uint16_t)0x0002)        //0000 0000 0000 0010     电池警告
#define SYSTEM_ERROR_RC_SIGNAL_LOSS_MASK ((uint16_t)0x0004)         //0000 0000 0000 0100     遥控器信号丢失
#define SYSTEM_ERROR_MOTOR1_COMM_MASK ((uint16_t)0x0008)            //0000 0000 0000 1000     1号电机通信错误
#define SYSTEM_ERROR_MOTOR2_COMM_MASK ((uint16_t)0x0010)            //0000 0000 0001 0000     2号电机通信错误
#define SYSTEM_ERROR_MOTOR3_COMM_MASK ((uint16_t)0x0020)            //0000 0000 0010 0000     3号电机通信错误
#define SYSTEM_ERROR_MOTOR4_COMM_MASK ((uint16_t)0x0040)            //0000 0000 0100 0000     4号电机通信错误
#define SYSTEM_ERROR_STEER_ENCODER_MASK ((uint16_t)0x0080)          //0000 0000 1000 0000     转向编码器错误
#define SYSTEM_ERROR_MOTOR_DRIVER_MASK ((uint16_t)0x0100)           //0000 0001 0000 0000     电机驱动错误
#define SYSTEM_ERROR_HL_COMM_MASK ((uint16_t)0x0200)                //0000 0010 0000 0000     高层通信错误


 
//电机驱动器状态  ActuatorLSStateMessage::driver_state
#define DRIVER_STATE_INPUT_VOLTAGE_LOW_MASK ((uint8_t)0x01)            //0000 0001     输入电压过低
#define DRIVER_STATE_MOTOR_OVERHEAT_MASK ((uint8_t)0x02)              //0000 0010     电机过热
#define DRIVER_STATE_DRIVER_OVERLOAD_MASK ((uint8_t)0x04)              //0000 0100     驱动过载
#define DRIVER_STATE_DRIVER_OVERHEAT_MASK ((uint8_t)0x08)              //0000 1000     驱动过热
#define DRIVER_STATE_SENSOR_FAULT_MASK ((uint8_t)0x10)                //0001 0000     传感器故障
#define DRIVER_STATE_DRIVER_FAULT_MASK ((uint8_t)0x20)                //0010 0000     驱动故障
#define DRIVER_STATE_DRIVER_ENABLED_MASK ((uint8_t)0x40)              //0100 0000     驱动器使能
#define DRIVER_STATE_DRIVER_RESET_MASK ((uint8_t)0x80)                //1000 0000     驱动器复位



//电池扩展状态  BmsExtendedMessage
#define BMS_PROT1_CHARGING_CURRENT_NONZERO_MASK ((uint8_t)0x01)           //0000 0001     充电电流不为零
#define BMS_PROT1_CHARGING_OVERCURRENT_SET_MASK ((uint8_t)0x02)         //0000 0010     充电过流
#define BMS_PROT1_DISCHARGING_CURRENT_NONZERO_MASK ((uint8_t)0x10)      //0001 0000     放电电流不为零
#define BMS_PROT1_DISCHARGING_OVERCURRENT_SET_MASK ((uint8_t)0x20)      //0010 0000     放电过流
#define BMS_PROT1_DISCHARGING_SHORTCIRCUIT_SET_MASK ((uint8_t)0x40)     //0100 0000     放电短路

#define BMS_PROT2_CORE_OPENCIRCUIT_SET_MASK ((uint8_t)0x01)              //0000 0001     核心开路
#define BMS_PROT2_TEMP_SENSOR_OPENCIRCUIT_SET_MASK ((uint8_t)0x02)        //0000 0010     温度传感器开路
#define BMS_PROT2_CORE_OVERVOLTAGE_SET_MASK ((uint8_t)0x10)               //0001 0000     核心过压
#define BMS_PROT2_CORE_UNDERVOLTAGE_SET_MASK ((uint8_t)0x20)              //0010 0000     核心欠压
#define BMS_PROT2_TOTAL_OVERVOLTAGE_SET_MASK ((uint8_t)0x40)              //0100 0000     总电压过压
#define BMS_PROT2_TOTAL_UNDERVOLTAGE_SET_MASK ((uint8_t)0x80)             //1000 0000     总电压欠压

#define BMS_PROT3_CHARGING_OVERTEMP_SET_MASK ((uint8_t)0x04)              //0000 0100     充电过温
#define BMS_PROT3_DISCHARGING_OVERTEMP_SET_MASK ((uint8_t)0x08)           //0000 1000     放电过温
#define BMS_PROT3_CHARGING_UNDERTEMP_SET_MASK ((uint8_t)0x10)             //0001 0000     充电欠温
#define BMS_PROT3_DISCHARGING_UNDERTEMP_SET_MASK ((uint8_t)0x20)          //0010 0000     放电欠温
#define BMS_PROT3_CHARGING_TEMPDIFF_SET_MASK ((uint8_t)0x40)              //0100 0000     充电温差过大
#define BMS_PROT3_DISCHARGING_TEMPDIFF_SET_MASK ((uint8_t)0x80)           //1000 0000     放电温差过大

#define BMS_PROT4_CHARGING_MOS_STATE_SET_MASK ((uint8_t)0x01)             //0000 0001     充电MOS状态
#define BMS_PROT4_DISCHARGING_MOS_STATE_SET_MASK ((uint8_t)0x02)          //0000 0010     放电MOS状态
#define BMS_PROT4_CHARGING_MOS_FAILURE_SET_MASK ((uint8_t)0x04)           //0000 0100     充电MOS故障
#define BMS_PROT4_DISCHARGING_MOS_FAILURE_SET_MASK ((uint8_t)0x08)        //0000 1000     放电MOS故障
#define BMS_PROT4_WEAK_SIGNAL_SWITCH_OPEN_SET_MASK ((uint8_t)0x10)         //0001 0000     弱信号开关打开

#ifdef __cplusplus
}
#endif
