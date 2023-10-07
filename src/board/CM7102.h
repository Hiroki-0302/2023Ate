/**
 * @file CM7102.h
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
namespace pindef
{
    enum : pin_size_t
    {
        BUZZER = 22,

        SCL = 21,
        SDA = 20,

        // MU_RTS = 19,
        // MU_TX = 8,
        // MU_RX = 9,

        // RS485_RTS = 18,
        // RS485_RX = 17,
        // RS485_TX = 16,

        CAN_MOSI = 3,
        CAN_SCK = 2,
        CAN_CS = 1,
        CAN_MISO = 4,
        CAN_INT = 0,

        // DEBUG_TX = 10,
        // DEBUG_RX = 7,
    };

};
typedef enum PinMap
{
    p1 = 0,
    p2,
    p3,
    p4,
    pa1 = 0,
    pa2,
    pb1,
    pb2
} pinmap_t;
typedef  pin_size_t portPin[4];
constexpr portPin PA = {
    18,19,13,9};
constexpr portPin PB = {
    27, 28, 6, 5};
constexpr portPin PC = {
    16, 17, 15, 14};
constexpr portPin PD = {
    20, 26, 8, 7};

#define HAS_BUZZER

// RS485
//  #define HAS_RS485
//  #define RS485_SERIAL Serial1

// MU
//  #define HAS_MU
//  #define MU_SERIAL Serial2
//  const uint8_t CFG_MU_CH=8;

// CANFD
 #define HAS_CANFD
#define CAN_SPI SPI
#define OLED_I2C Wire
enum cfg
{
    CFG_CANFD_ID = 2,
};

#define BOARD_PICO
