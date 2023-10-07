
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

//設計ミス(初回発注)
//RELAYピンが未接続
//5VSENSピンが未接続
namespace pindef{
enum common_pin: pin_size_t
{
    BUZZER = 22,

   
    MU_RTS = 19,
    MU_TX = 9,
    MU_RX = 8,

    RS485_RTS = 18,
    RS485_RX = 17,
    RS485_TX = 16,
    
    UART_TX =4,
    UART_RX =5,

    TINY202_TX=10,

};
enum func_pin:pin_size_t{
ESTOP1=15,
ESTOP2=14,
ESTOP3=13,
CUR_SENS=28,
VOLT_SENS=27,
VOLT_SENS_5V=26,
ALED=12,
RELAY=20,

};
};

#define HAS_BUZZER 
#define BUZZER_TYPE_ACTIVE

// #define HAS_OLED
// #define OLED_I2C Wire

// #define IS_MASTER 

//#define ALT_WIICLASSIC
//#define WCLASSIC_I2C Wire1

//RS485
#define HAS_RS485
#define RS485_SERIAL Serial1
#define RS485_MASTER
//MU
extern SerialPIO mu_serial;
#define HAS_MU
#define MU_USE_SOFTWARESERIAL
#define MU_SERIAL mu_serial
const uint8_t CFG_MU_CH=8;

//CANFD

//UART
#define HAS_UART
#define UART_SERIAL Serial2

#define BOARD_PICO