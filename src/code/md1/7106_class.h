#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP23X17.h>
#include <stdlib.h>

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

class SABERTOOTH : public SerialPIO
{
public:
    SABERTOOTH(uint8_t pin, uint8_t address) : address(address), SerialPIO(pin, 255) {}
    bool status;
    void init()
    {
        this->begin(9600);
        status = 0;
    }
    void drive(int16_t speed)
    {
        this->write(address);
        this->write(speed < 0 ? 1 : 0);
        this->write((uint8_t)abs(speed));
        this->write((address + (speed < 0 ? 1 : 0) + abs(speed)) & 0b01111111);
        this->flush();
    }
private : uint8_t address;
};
class ROTA {
public:
    ROTA(uint8_t A, uint8_t B, uint8_t X) : A(A), B(B), X(X), count(0) {}
    bool flag;
    void init() {
        exIO::pinMode(A, INPUT_PULLUP);
        exIO::pinMode(B, INPUT_PULLUP);
        exIO::pinMode(X, INPUT_PULLUP);
        inst = this;  // インスタンスを設定
        flag = 0;
        attachInterrupt(digitalPinToInterrupt(A), pulseWrapper, RISING);
    }    
    void pulse() {
        if (digitalRead(A) ^ digitalRead(B)) {
            count++;
        } else {
            count--;
        }
    }
    bool Xsta(){
        return digitalRead(X);
    }
    void setPos(){
        count = 0;
    }
    int pos() {
        count %= 256;
        if(count < 0){
            count = -1*count;
        }
        return count;
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

// class LED : public Adafruit_NeoPixel
// {
//     public:
//     LED(uint16_t n, uint8_t pin ,neoPixelType type) : n(n),pin(pin),type(NEO_GRB + NEO_KHZ800) {}
//     void init()
//     {
//         this->begin();
//         this->clear();
//         this->show();
//         this->setBrightness(250);
//     }
//     void flash(uint8_t i ,uint8_t r,uint8_t g, uint8_t b )
//     {
//         this->setPixelColor(i,this->Color(r,g,b));
//         this->show();
//     }
//     void all(uint8_t r, uint8_t g, uint8_t b){
//         for(int j=0; j<=n; j++){
//             this->setPixelColor(i,this->Color(r,g,b));
//         }
//         this->show();
//     }

//     private:
//         uint16_t n;//光らせたいLEDの個数
//         uint8_t i;//光らせたいLEDの番号
//         uint8_t pin;
//         neoPixelType type;
//         uint8_t r;//赤の値
//         uint8_t g;//緑の値
//         uint8_t b;//青の値
// };