#pragma once
#include <Arduino.h>

// struct LIMITS {
//     uint8_t pin;
//     volatile bool status;
// };
// LIMITS limit[4];

// class LIMITS
// {
// public:
//     LIMITS(uint8_t pin) : pin(pin) {}
//     volatile bool status;
//     void init()
//     {
//         pinMode(pin, INPUT_PULLUP);
//         attachInterrupt(digitalPinToInterrupt(pin), interruptHandler, FALLING);
//         status = LOW;
//         instance = this;
//     }
// private:
//     uint8_t pin;
//     static LIMITS* instance;
//     static ulong lastDebugTime;
//     static void interruptHandler()
//     {
//         if (instance != nullptr)
//         {
//             instance->push();
//         }
//     }
//     void push()
//     {
//         noInterrupts(); // 割り込みを一時的に無効にする
//         status = digitalRead(pin);
//         interrupts(); // 割り込みを有効にする
//     }
// };

// LIMITS* LIMITS::instance = nullptr;
// class LIMITS
// {
// public:
//     LIMITS(uint8_t pin) : pin(pin) {}
//     uint8_t pin;
//     volatile bool status;
//     void init()
//     {
//         pinMode(pin, INPUT_PULLUP);
//         status = HIGH; // pullupなので押したら0になる
//     }
//     void readSw()
//     {
//         status = digitalRead(pin);
//     }
// private:
// };
class LIMITS
{
public:
    LIMITS(uint8_t pin) : pin(pin) {}
    void init()
    {
        pinMode(pin, INPUT_PULLUP);//短絡→0v 
    }

    bool push()
    {
        return digitalRead(pin); // 押されてたら0が返ってくる
    }

private:
    uint8_t pin;
};


