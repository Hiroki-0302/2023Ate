#pragma once

#include <Arduino.h>

struct UM7104_sens
{
    union
    {
        struct
        {
            int16_t temp;
            struct
            {
                uint8_t voltage;
                uint8_t current;
            } power_15, power_sens1, power_sens2;
            union
            {
                struct
                {
                    unsigned sw1 : 1;
                    unsigned sw2 : 1;
                    unsigned sw3 : 1;
                    unsigned virtual_sw : 1;
                    unsigned em : 1;
                    unsigned reserved : 3;
                } __attribute__((__packed__));
                uint8_t status;
            };
        };
        uint8_t raw[9];
    };
};
struct UM7106_sens
{
    union
    {
        struct
        {
            uint8_t current[4];
            uint16_t rotenc_deltapos[4];
            uint8_t rotenc_pos;
            uint8_t status;
        };
        uint8_t raw[14];
    };
};
struct UM7105_sens
{
    union
    {
        struct
        {
            uint8_t online;
            uint8_t status;
            uint8_t mu_ch;
        };
        uint8_t raw[5];
    };
};
struct config
{
    union
    {
        struct
        {
            uint8_t addr;
            uint16_t value;
        } cfg[4] = {};
        uint8_t raw[12];
    };
};
struct UM7105_mu
{
    union
    {
        struct
        {
            uint8_t emergency;
            uint8_t ctrl[5];
            uint8_t data[12];
            uint8_t data_len;
        };
        uint8_t raw[19];
    };
};

struct UM7101_data
{
};
struct UM7106_meca_motor
{ // direct
 union 
    {
        int16_t speed[4];
        uint8_t raw[8];
    };
};

struct UM7106_help_motor
{ // direct
 union 
    {
        int16_t speed[4]; 
        uint8_t raw[8];
    };
};
struct UM7106_led
{ // direct
 union 
    {
        struct
        {
            uint8_t r;
            uint8_t g;
            uint8_t b;
            uint8_t mode;
        };
        uint8_t raw[4];
    };
};

struct UM7106_link_motor
{ // direct
 union 
    {
        struct{
            int16_t speed;
            int16_t abuspeed;
            uint8_t bal_arg ;
            uint8_t start_arg ;
            uint8_t stop_arg ;
        };
        uint8_t raw[7];
    };
};

struct UM7106_custom
{
    union
    {
        struct
        {
            uint8_t type;
            uint8_t select;
            int16_t value[4];
        };
        uint8_t raw[10];
    };
};
struct UM7105_relay
{
    enum port_sel : uint8_t
    {
        HSERIAL,
        SSERIAL1,
        SSERIAL2,
        SSERIAL3,
    };
    union
    {
        struct
        {
            uint8_t sendfrom;
            uint8_t id;
            uint8_t port;
            uint8_t cmd;
            uint8_t data[32];
        };
        uint8_t raw[36];
    };
};
