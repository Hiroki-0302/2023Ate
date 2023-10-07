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
#define SerialHPORT Serial1
namespace pindef{
enum : pin_size_t
{
   
    SCL = 21,
    SDA = 20,

    MU_RTS = 22,
    MU_TX = 19,
    MU_RX = 18,
    
    CAN_MOSI = 3,
    CAN_SCK = 2,
    CAN_CS = 1,
    CAN_MISO = 4,
    CAN_INT = 0,
     
   
  

};
enum func_pin:pin_size_t{
    GPD1=5,
    GPA1=26,
    GPA2=27,
    GPA3=28,
    SDA2=6,
    SCL2=7,
    SERVOTX=8,
    SERVORX=9,
    HTX=12,
    HRX=13,
    TX1=14,
    RX1=15,
    TX2=10,
    RX2=11,
    TX3=17,
    RX3=16,
};
};

// #define HAS_BUZZER 

// #define HAS_OLED
// #define OLED_I2C Wire

// #define IS_MASTER 

//#define ALT_WIICLASSIC
//#define WCLASSIC_I2C Wire1

//RS485
// #define HAS_RS485
// #define RS485_SERIAL Serial1
// #define RS485_MASTER
//MU
#define HAS_CONTROLLER
extern SerialPIO SerialMU;
#define HAS_MU
#define  MU_USE_SOFTWARESERIAL
#define MU_SERIAL SerialMU
const uint8_t CFG_MU_CH=0x08;

//CANFD
#define HAS_CANFD
// #define HAS_CONTROLLER
#define CAN_SPI SPI
#define CANFD_CH (addr::ADDR_7105&0xf)
//UART

#define BOARD_PICO