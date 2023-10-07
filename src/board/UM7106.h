/**
 * @file CM7101.h
 * @author K.Fukushima@nnct ( )
 * @brief 基板定義ファイル
 * @version 0.1
 * @date 2023-02-17
 *
 * @copyright Copyright (c) 2023
 *
 */
#pragma once
#include <Arduino.h>
#define MCP23017_WIRE Wire1
namespace pindef
{
    enum : pin_size_t
    {
        // BUZZER = 22,

        // WCLASSIC_SCL=3,
        // WCLASSIC_SDA=2,

        SCL = 3,
        SDA = 2,

        // MU_RTS = 19,
        // MU_TX = 8,
        // MU_RX = 9,

        // RS485_RTS = 18,
        // RS485_RX = 17,
        // RS485_TX = 16,

        // CAN_MOSI = 15,
        // CAN_SCK = 14,
        // CAN_CS = 13,
        // CAN_MISO = 12,
        // CAN_INT = 11,

        // DEBUG_TX = 10,
        // DEBUG_RX = 7,

        UART_RX = 13,
        UART_TX = 12,

    };
#define MCPA(b) 0b1000000 + b
#define MCPB(b) 0b1001000 + b
    enum func_pin : pin_size_t
    {
        Motor0PWM = 11,
        Motor1PWM = 10,
        Motor2PWM = 5,
        Motor3PWM = 4,
        Motor3A = MCPB(0),
        Motor3B = MCPB(1),
        Motor2A = MCPB(3),
        Motor2B = MCPB(2),
        Motor1A = MCPB(4),
        Motor1B = MCPB(5),
        Motor0A = MCPB(7),
        Motor0B = MCPB(6),
        XMotor0PWM = 6,
        XMotor1PWM = 7,
        XMotor2PWM = 8,
        XMotor3PWM = 9,
        XMotor0DIR = MCPA(7),
        XMotor1DIR = MCPA(6),
        XMotor2DIR = MCPA(5),
        XMotor3DIR = MCPA(4),
        FET1 = MCPA(3),
        FET2 = MCPA(2),
        FET3 = MCPA(0),
        FET4 = MCPA(1),
        RE1A = 17, // rotary encoder
        RE1B = 20,
        RE1R = 16,
        RE2A = 21,
        RE2B = 22,
        RE2R = 20,
        RE3A = 27,
        RE3B = 28,
        RE3R = 14,
        RE4A = 0,
        RE4B = 19,
        RE4R = 15,

        TX_PICO = 1,

    };
};

// #define HAS_BUZZER

// #define HAS_OLED
// #define OLED_I2C Wire

// #define IS_MASTER

// //#define ALT_WIICLASSIC
// //#define WCLASSIC_I2C Wire1

// //RS485
// #define HAS_RS485
// #define RS485_SERIAL Serial1
// #define RS485_MASTER
// //MU
// #define HAS_MU
// #define MU_SERIAL Serial2
// const uint8_t CFG_MU_CH=8;

// //CANFD
// #define HAS_CANFD
// #define HAS_CONTROLLER
// #define CAN_SPI SPI1
// enum cfg{
//     CFG_CANFD_ID=1,
// };
// UART
#define HAS_UART
#define UART_SERIAL Serial1
//uart0=Serial1

#define BOARD_PICO