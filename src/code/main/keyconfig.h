/**
 * @file keyconfig.h
 * @author K.Fukushima@nnct ( )
 * @brief Bチームコントローラー設定
 * @version 0.1
 * @date 2023-07-04
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once
#include "Arduino.h"
#include "main_header.h"

namespace keyConf{
enum keyMapping:uint8_t{
    arm_angle_up=button::UP,
    arm_angle_down=button::DOWN,
    arm_move_left=button::LEFT,
    arm_move_right=button::RIGHT,
    arm_air=button::TriggerR,
    arm_push=button::Y,
    arm_pull=button::A,
    wheel_left=button::LstickY,
    wheel_right=button::RstickY,
    gui_left=button::BACK,
    gui_right=button::START,
    gui_enter=button::XBOX,
};
enum AnalogSetting:int16_t{
    wheelLeft_invert=false,
    wheelRight_invert=false,
    deadzone_Lstick=0,
    deadzone_Rstick=0,
};

};