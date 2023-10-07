#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP23X17.h>
#include <stdint.h>
#include <debug.h>

namespace exIO
{
    bool init(TwoWire &wire = Wire);
    void digitalWrite(uint8_t pin, uint8_t val);
    void pinMode(uint8_t pin, uint8_t val);
}
class MD
{
public:
    MD(uint8_t pwm, uint8_t a, uint8_t b = 255) : pwm(pwm), a(a), b(b) {}
    void init()
    {
        exIO::pinMode(pwm, OUTPUT);
        exIO::pinMode(a, OUTPUT);
        if (b != 255)
            exIO::pinMode(b, OUTPUT);
    }
    void setSpeed(int16_t speed)
    {
        this->speed = speed;
        analogWrite(pwm, abs(speed));
        exIO::digitalWrite(a, speed > 0);
        if (b != 255)
            exIO::digitalWrite(b, speed < 0);
    }
    void stop()
    {
        setSpeed(0);
    }
    void brake()
    {
        if (b == 255)
            return;
        speed=0;
        analogWrite(pwm, 0);
        exIO::digitalWrite(a, 1);
        exIO::digitalWrite(b, 1);
    }
    int16_t getSpeed()
    {
        return speed;
    }
private:
    int16_t speed; 
    uint8_t pwm;
    uint8_t a;
    uint8_t b;
};
class FET
{
public:
    FET(uint8_t pin) : pin(pin) {}
    void init()
    {
        exIO::pinMode(pin, OUTPUT);
    }
    void on()
    {
        state = true;
        exIO::digitalWrite(pin, 1);
    }
    void off()
    {
        state = false;
        exIO::digitalWrite(pin, 0);
    }
    void set(bool val)
    {
        state = val;
        exIO::digitalWrite(pin, val);
    }
    bool get()
    {
        return state;
    }

private:
    uint8_t pin;
    bool state;
};
class ROTA {
public:
    ROTA(uint8_t A, uint8_t B, uint8_t X) : A(A), B(B), X(X), count(0) {}
    void init() {
        exIO::pinMode(A, INPUT_PULLUP);
        exIO::pinMode(B, INPUT_PULLUP);
        exIO::pinMode(X, INPUT_PULLUP);
        inst = this;  // インスタンスを設定
        attachInterrupt(digitalPinToInterrupt(A), pulseWrapper, RISING);
    }    
    void pulse() {
        if (digitalRead(A) ^ digitalRead(B)) {
            count++;
        } else {
            count--;
        }
    }
    double getrps(){
        unsigned long lastMillis;
        if(millis()- lastMillis >= 1000){
            return count/256;
            PRINT("rps:%f",count/256);
            lastMillis = millis();
        }
    }
  
private:
    uint8_t A;
    uint8_t B;
    uint8_t X;
    uint32_t count;
    static ROTA* inst;  // 静的ポインタを追加  
    static void pulseWrapper() {
        if (inst != nullptr) {
            inst->pulse();
        }
    }    

};
