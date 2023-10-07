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
namespace pindef{
enum common_pin: pin_size_t
{
    BUZZER = 22,

    WCLASSIC_SCL=3,
    WCLASSIC_SDA=2,

    SCL = 21,
    SDA = 20,

    MU_RTS = 19,
    MU_TX = 8,
    MU_RX = 9,

    RS485_RTS = 18,
    RS485_RX = 17,
    RS485_TX = 16,
    
    CAN_MOSI = 15,
    CAN_SCK = 14,
    CAN_CS = 13,
    CAN_MISO = 12,
    CAN_INT = 11,
    
    DEBUG_TX = 10,
    DEBUG_RX = 7,

    UART_TX =5,
    UART_RX =6,

};
enum func_pin:pin_size_t{
    GP0=0,
    GP1=1,
    GP2=2,
    GP3=3,
    GP4=4,
    GP5=5,
    GP6=6,
};
};

#define HAS_BUZZER 

#define HAS_OLED
#define OLED_I2C Wire

#define IS_MASTER 

//#define ALT_WIICLASSIC
//#define WCLASSIC_I2C Wire1

//RS485
#define HAS_RS485
#define RS485_SERIAL Serial1
#define RS485_MASTER
//MU
#define HAS_MU
#define MU_SERIAL Serial2
const uint8_t CFG_MU_CH=8;

//CANFD
#define HAS_CANFD
#define HAS_CONTROLLER
#define CAN_SPI SPI1
enum cfg{
    CFG_CANFD_ID=1,
};
//UART
#define HAS_UART
#define UART_SERIAL Serial3

#define BOARD_PICO