#ifdef PJ_EMG
#include <Arduino.h>
#include <NRUnified.h>
#include "driver_comm/UART.h"
#include "driver_comm/TinyComm.h"
#include "driver_comm/USBSerial.h"
#include "driver_act/buzzer.h"

#include <SerialPIO.h>
#include "struct.h"
#include "project.h"
#include "config.h"
#include "board/UM7104.h"
#include "Adafruit_NeoPixel.h"

constexpr float board_v15_offset = -0.25 / 14.6;
// 最大電圧14.6Vのとき0.25V高く表示されるため
constexpr uint16_t volt15_multiplier = (3.3f / 1024.0f) * 11.0f * 10.0f * (1 + board_v15_offset) * (float)(1 << 12);
// 分圧とAD変換から電圧を求める係数
// 電圧：10k:100kΩ分圧=1/11倍されている,voltageはuint8_tのため10倍して少数1位まで送信

Logger logger("main");
SerialPIO mu_serial(pindef::MU_TX, pindef::MU_RX); // 仮
SerialPIO sens_serial(255, pindef::TINY202_TX);
TinyComm sens(&sens_serial, 8);
CommManager::Board brd_main(ADDR_7101_C, "main");
CommManager::Board brd_emg(ADDR_7104_U, "emg");
UARTDriver uart(&UART_SERIAL);
USBSerial usb;

Adafruit_NeoPixel aled(1, pindef::ALED, NEO_GRB + NEO_KHZ800);
uint32_t lastCommTime = 0;
bool emg_stop = true;
bool sw_en[3] = {true,false,false};

Buzzer buzzer(pindef::BUZZER, true);

UM7104_sens sens_data;
struct tiny_data
{
    union
    {
        uint16_t data[4]; // mA
        uint8_t buf[8];
    };
} sens_tiny_data;

bool request(CommMessage msg) // メイン基板からの命令受信時
{
    Serial.print("request:");
    Serial.print(msg.id);
    Serial.print(",");
    Serial.println(msg.cmd);

    if (msg.cmd == cmd7104::SET_ACT)
    {

        if (msg.data[0] == cmd7104::BUZZER)
        {
            buzzer.add(0, msg.data[1], msg.data[2]);
        }
        if (msg.data[0] == cmd7104::EMG_STOP)
        {
            sens_data.virtual_sw = !!msg.data[1]; // 二重反転によるbool化
        }
    }
    if (msg.cmd == cmd7104::REQ_SENS)
    {
        brd_main.send(cmd7104::REQ_SENS, sens_data.raw, sizeof(sens_data.raw));
    }
    lastCommTime = millis();
    return true;
}
void statusLED(uint8_t r, uint8_t g, uint8_t b)
{
    aled.setPixelColor(0, r, g, b);
    aled.show();
}
void setEmStop(bool en)
{
    if (!emg_stop && en) // EMG STOP
    {
        digitalWrite(pindef::RELAY, LOW);
        statusLED(50, 0, 0); //赤→ストップ
        uint8_t _emg_stop = emg_stop;
        brd_main.send(cmd7104::NTF_EM, &_emg_stop, 1);
        PRINT("EMSTOP on");
    }
    else if (emg_stop && !en) // ON
    {
        digitalWrite(pindef::RELAY, HIGH);
        statusLED(0, 50, 0); //緑→オン
        uint8_t _emg_stop = emg_stop;
        brd_main.send(cmd7104::NTF_EM, &_emg_stop, 1);
        PRINT("EMSTOP off");
        buzzer.stop();
    }
    emg_stop = en; // フラグ更新
}

void serialErrorCall(uint8_t error)
{
    PRINT("error:%d\n", error);
}
bool defaultCallback(CommMessage msg)
{
    PRINT("defaultCallback: id=%d, cmd=%d\n", msg.id, msg.cmd);

    return true;
}
void commandReceived(CMDMessage &c)
{
    if (c.match("cur "))
    {
        PRINT("sens:temp%d,cur%d,volt:%d", sens_data.temp, sens_data.power_15.current, sens_data.power_15.voltage);
    }
}
bool configProcess(CommMessage msg)
{
    config c;
    memcpy(&c, msg.data, sizeof(c));
    for (uint8_t i = 0; i < 4; i++)
    {
        switch (c.cfg[i].addr)
        {
        case 0:
            // 空
            break;
        case configAddr7104::EMG_STOP:
            logger.info("v_sw:%d", c.cfg[i].value);
            sens_data.virtual_sw = (bool)c.cfg[i].value;
            break;
        default:
            logger.warn("unknown config addr:%d", c.cfg[i].addr);
        }
    }
    return true;
}
void setup()
{
    delay(1000);
    usb.init();
    logger.info("setup");
    sens_serial.begin(39060, SERIAL_8N1);
    UART_SERIAL.setTX(pindef::UART_TX);
    UART_SERIAL.setRX(pindef::UART_RX);
    UART_SERIAL.begin(CFG_UART_BAUD);
    // attiny202 at 20MHz,38400で通信時、 オシロで1ビットあたりの時間を測定し逆数をとったらこうなった。個体差ではなく計算誤差らしい...
    // https://github.com/SpenceKonde/megaTinyCore/blob/master/megaavr/extras/Ref_Serial.md
    // 16 MHz のクラシックは、U2X による計算エラーのため、完璧なクロックを仮定すると 2.12% 速く動作します
    // Tiny側を2％下げる:37600か、こっちを2％上げるかを決める
    pinMode(pindef::ALED, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(pindef::RELAY, OUTPUT);
    pinMode(pindef::BUZZER, OUTPUT);
    pinMode(pindef::MU_RTS, OUTPUT);
    pinMode(pindef::ESTOP1, INPUT_PULLUP);
    pinMode(pindef::ESTOP2, INPUT_PULLUP);
    pinMode(pindef::ESTOP3, INPUT_PULLUP);
    pinMode(pindef::CUR_SENS, INPUT);
    pinMode(pindef::VOLT_SENS, INPUT);
    pinMode(pindef::VOLT_SENS_5V, INPUT);
    usb.setCommandCallback(commandReceived);

    aled.begin();
    aled.setBrightness(50);
    statusLED(0,0,50);//青→準備
    buzzer.init();

    CommManager::addBoard(&brd_emg);
    CommManager::addBoard(&brd_main);
    CommManager::addCommMethod(&uart); // first must be default method
    CommManager::addCommMethod(&usb);
    CommManager::init(ADDR_7104_U);
    CommManager::setCommReceiveCallbackDefault(defaultCallback);
    brd_main.addLink(cmd7104::SET_ACT, NULL, 0, request);
    brd_main.addLink(cmd7104::REQ_SENS, NULL, 0, request);
    brd_main.addLink(cmd7104::CONFIG, NULL, 0, configProcess);

    logger.info("setup done");
}
void sensorUpdate()
{
    sens_data.temp = round(analogReadTemp() * 10);
    uint16_t volt15 = ((analogRead(pindef::VOLT_SENS) * volt15_multiplier) >> 12) - 1;
    sens_data.power_15.voltage = volt15 > 255 ? 255 : volt15;
    int16_t current = map(analogRead(pindef::CUR_SENS), 0, 1023, 200, -80);
    // 0~3.3V->-40~16A(5V:40Aのため) 電圧センサの電流方向を逆にしているため符号反転
    // 電流は符号消して5倍して送信->0.2A単位で送信
    sens_data.power_15.current = abs(current) > 255 ? 255 : abs(current);
    if (sens.read(sens_tiny_data.buf))
    {
        // 4.3V/4096
        sens_data.power_sens1.current = map(sens_tiny_data.data[0], 0, 4096, 0, 43000);
        sens_data.power_sens2.current = map(sens_tiny_data.data[1], 0, 4096, 0, 43000);
        sens_data.power_sens2.voltage = map(sens_tiny_data.data[2], 0, 4096, 0, 43000);
        sens_data.power_sens1.voltage = map(sens_tiny_data.data[3], 0, 4096, 0, 43000);

        // PRINT("sens recv");
    }
}
bool isEMSwitchPushed()
{
    sens_data.sw1 = digitalRead(pindef::ESTOP1);
    sens_data.sw2 = digitalRead(pindef::ESTOP2);
    sens_data.sw3 = digitalRead(pindef::ESTOP3);
    bool sw_all = false;
    sw_all |= sens_data.sw1 && sw_en[0];
    sw_all |= sens_data.sw2 && sw_en[1];
    sw_all |= sens_data.sw3 && sw_en[2];
    sw_all |= sens_data.virtual_sw;
    return sw_all;
}
void emStateUpdate()
{
    bool em = false;
    if (isEMSwitchPushed())
    {
        em = true;
    }
    if (millis() - lastCommTime > 1000)
    {
        em = true;
        statusLED(25,0,25);//紫→
    }
    setEmStop(em);
}
uint32_t lastPotato = 0;

#define buzz_type_potato
// #define buzz_type_simple

void buzzEmgSound()
{
#ifdef buzz_type_potato
    if (emg_stop && millis() - lastPotato > 923)
    {
        buzzer.add(0, 923 / 4, 392 * 2);
        buzzer.add(923 / 4, 923 / 4, 349 * 2);
        buzzer.add(923 / 2, 923 / 4, 392 * 2);
        lastPotato = millis();
    }
#endif
#ifdef buzz_type_simple
    if (emg_stop && millis() - lastPotato > 4000)
    {
        logger.info("buzz");
        lastPotato = millis();
        buzzer.add(0, 20, 1318);
        buzzer.add(100, 20, 1318);
    }
#endif
}
void loop()
{
    //logger.info("sens");
    sensorUpdate();
    //logger.info("comm");
    emStateUpdate();
    //logger.info("buz");
    buzzEmgSound();
    //logger.info("update");
    digitalWrite(LED_BUILTIN, millis() % 1000 > 500);
    CommManager::update();
    //lastCommTime = millis();
    buzzer.update();
}

#endif
