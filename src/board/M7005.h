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
namespace pindef
{
    enum 
    {
        // BUZZER = 22,
        // WCLASSIC_SCL=3,
        // WCLASSIC_SDA=2,

        SCL = 22,
        SDA = 21,

        // MU_RTS = 19,
        // MU_TX = 8,
        // MU_RX = 9,

        RS485_RTS = 23,
        RS485_RX = 2,
        RS485_TX = 4,

        // CAN_MOSI = 15,
        // CAN_SCK = 14,
        // CAN_CS = 13,
        // CAN_MISO = 12,
        // CAN_INT = 11,

        // DEBUG_TX = 10,
        // DEBUG_RX = 7,
    };
    enum func_pin{
        GP0 = 13,
        GP1 = 18,
        GP2 = 14,
        GP3 = 27,
        GP4 = 19,
        GP5 = 26,
        GP6 = 25,
        GP7 = 33,
        GP8 = 32,
        GPI9 = 35,
        GPI10 = 34,
        LED=12,
        SPTX=17,
        SPRX=16,

    };
};

// #define HAS_BUZZER
// #define HAS_OLED
// #define OLED_I2C Wire
// #define IS_MASTER

// #define ALT_WIICLASSIC
// #define WCLASSIC_I2C Wire1

// RS485
#define HAS_RS485
#define RS485_SERIAL Serial1

    // MU
    //  #define HAS_MU
    //  #define MU_SERIAL Serial2
    //  const uint8_t CFG_MU_CH=8;

    // //CANFD
    // #define HAS_CANFD
    // #define HAS_CONTROLLER
    // #define CAN_SPI SPI1
    // enum cfg{
    //     CFG_CANFD_ID=1,
    // };

#define BOARD_ESP32
