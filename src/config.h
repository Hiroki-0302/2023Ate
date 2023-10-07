#pragma once
#include <Arduino.h>
enum {
 CFG_RS485_MAXMODULES = 4,
 CFG_CANFD_MAXMODULES = 8,
 CFG_CONTROL_PERIOD=20,//50Hz
 CFG_RS485_BAUD = 115200,
 CFG_MU_MAXLEN = 12, // 純粋な受信データのバイト数(-> *DR=...\r\nを含まない)
};
/**
 * @brief 接続ボード種類設定
 *
 */#pragma once
#include <Arduino.h>
#include "struct.h"

#define CFG_RS485_MAXMODULES 4
#define CFG_CANFD_MAXMODULES 8
#define CFG_CONTROL_PERIOD 1 // 1000Hz
#define CFG_RS485_BAUD 115200
#define CFG_UART_BAUD 38400
#define CFG_MU_MAXLEN 12 // 純粋な受信データのバイト数(-> *DR=...\r\nを含まない)

/**
 * @brief 接続ボード種類設定
 *
 */
#define ADDR_CANFD(x) (x | 0xf0)
#define ADDR_RS485(x) (x | 0xe0)
#define ADDR_HUB_UART(portnum, x) (x | portnum << 5)
// portnumは0 = H-UART , 1~3 = S-UART1~3

enum addr : uint8_t
{
    ADDR_7101_C = ADDR_CANFD(0x01),
    ADDR_7101_R = ADDR_RS485(0x01),
    ADDR_7102_C = ADDR_CANFD(0x02),
    ADDR_7104_R = ADDR_RS485(0x04),
    ADDR_7104_U = ADDR_HUB_UART(0, 0x04),
    ADDR_7105 = ADDR_CANFD(0x05),
    ADDR_7106_0 = ADDR_HUB_UART(1, 0x06),
    ADDR_7106_1 = ADDR_HUB_UART(2, 0x07),
    ADDR_IGNORE=0xFF,
};
namespace cmd7101
{
    enum : uint8_t
    {
        STATUS = 0x00,
        SENS_DATA = 0x01,
        REQUEST = 0x02,
        RESPONCE = 0x0f,
    };
};
namespace cmd7102
{
    enum : uint8_t
    {
        STATUS = 0x00,
        DATA = 0x01,
        REQUEST = 0x02,
        RESPONCE = 0x0f,
    };
};
namespace configAddr7104{
enum: uint8_t{
    MU_CH=0x01,
    EMG_STOP=0x02,
    };
};
namespace cmd7104
{
    enum :uint8_t
    {
        REQ_SENS = 0x00,
        REQ_MU,
        SET_ACT,
        SEND_MU,
        NTF_EM,//notify emergency stop state change
        CONFIG,
    };
    enum act_type:uint8_t{
        EMG_STOP=1<<4,
        BUZZER=2<<4,
        };
};
namespace configAddr7105{
enum: uint8_t{
    MU_CH=0x01,
    };
};
namespace cmd7105
{
    enum : uint8_t
    {
        REQ_SENS = 0x00,
        RECV_MU,
        SET_ACT,
        SEND_MU,
        DATA_RELAY,
        CONFIG,
    };
};
namespace cmd7106
{
    enum : uint8_t
    {
        REQ_SENS = 0x00,
        SET_LED,
        SET_FET,
        SET_MOTOR,
        SET_RMMOTOR,
        SET_LINK,
    };

};

enum RS485InterfaceIndex
{
    RS485_led1,
    RS485_led2,
    RS485_led3,
    RS485_led4,
    RS485_servo1,
    RS485_servo2,
    RS485_servo3,
    RS485_servo4,
    RS485_power1,
    RS485_power2,
    RS485_power3,
    RS485_power4,
    RS485_sensor1,
    RS485_sensor2,
    RS485_sensor3,
    RS485_sensor4,
    RS485_other1,
    RS485_other2,
    RS485_other3,
    RS485_other4,
};
enum CANFDInterfaceIndex
{
    CANFD_omni1,
    CANFD_omni2,
    CANFD_omni3,
    CANFD_omni4,
    CANFD_motor1,
    CANFD_motor2,
    CANFD_motor3,
    CANFD_motor4,
    CANFD_sensor1,
    CANFD_sensor2,
    CANFD_sensor3,
    CANFD_sensor4,
    CANFD_sensor5,
    CANFD_sensor6,
    CANFD_sensor7,
    CANFD_sensor8,

};
enum ActuatorType
{
    act_wheel,
    act_syoukou,
    act_catch,
};
enum BehaviorId
{
    omni_xy_drive,
    omni_turn,
};
enum EventBitID{
    cmd_errlog=1<<0,
    cmd_senslog=1<<1,
    cmd_showtask=1<<2,
    
};
// struct SensorData
//   {
//     struct Power
//     {
//       float voltage;
//       float current;
//       uint32_t lastUpdate;
//     } power;
//     struct Omni
//     {
//       float x;
//       float y;
//       float theta;
//       uint32_t lastUpdate;
//     } omni;
//     struct Motor_Feedback
//     {
//       float speed;
//       float current;
//       uint32_t lastUpdate;
//     } motor[4];
//     struct Servo_Feedback
//     {
//       float angle;
//       float current;
//       uint32_t lastUpdate;
//     } servo[4];
//     struct LED
//     {
//       float current;
//       uint32_t lastUpdate;
//     } led;
//     struct limitSwitch
//     {
//       bool state;
//       uint32_t lastUpdate;
//     } limitSwitch[4];
//     struct analog
//     {
//       uint16_t value;
//       uint32_t lastUpdate;
//     } analog[4];
//     struct variable
//     {
//       int value;
//       TickType_t lastUpdate;
//     } variable[4];

//   };