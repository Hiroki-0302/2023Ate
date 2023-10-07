#ifdef PJ_MD0
#include <Arduino.h>
#include <NRUnified.h>
#include "driver_comm/UART.h"
#include "driver_comm/TinyComm.h"
#include "driver_comm/USBSerial.h"
#include <SerialPIO.h>
#include "struct.h"
#include "project.h"
#include "config.h"
#include "board/UM7106.h"
#include "7106_class.h"
#include <encoder.hpp>

extern "C"{
    #include <hardware/watchdog.h>
    #include <pico/bootrom.h>
};

/*

UARTTX側のダイオードを反転する！！！！！！！！！！！！
*/
// #include "library/FeetechServo.h"
CommManager::Board brd_main(ADDR_7101_C, "main");
CommManager::Board brd_mdc0(ADDR_7106_0, "mdc0");
UARTDriver uart(&UART_SERIAL);
USBSerial usb;
Logger logger;
uint32_t lastReceived = 0;
struct sens_cur_t
{
    union
    {
        uint16_t cur[4];
        uint8_t buf[8];
    };
} sens_data;

enum report_flag_t
{
    report_current = 1 << 0,
    report_motor = 1 << 1,
    report_fet = 1 << 2,
    report_rotenc = 1 << 3,
};
uint8_t report_enable_flag = 0xff;
uint16_t sens_report_interval = 0;
uint32_t sens_report_last = 0;
SerialPIO sens_serial(255, pindef::TX_PICO);
TinyComm sens(&sens_serial, 8);
ROTA* ROTA::inst = nullptr;
MD md03a[4] = {
    MD(pindef::Motor0PWM, pindef::Motor0A, pindef::Motor0B),
    MD(pindef::Motor1PWM, pindef::Motor1A, pindef::Motor1B),
    MD(pindef::Motor2PWM, pindef::Motor2A, pindef::Motor2B),
    MD(pindef::Motor3PWM, pindef::Motor3A, pindef::Motor3B)};
MD extmd[4] = {
    MD(pindef::XMotor0PWM, pindef::XMotor0DIR),
    MD(pindef::XMotor1PWM, pindef::XMotor1DIR),
    MD(pindef::XMotor2PWM, pindef::XMotor2DIR),
    MD(pindef::XMotor3PWM, pindef::XMotor3DIR)};
FET fet[4] = {
    FET(pindef::FET1),
    FET(pindef::FET2),
    FET(pindef::FET3),
    FET(pindef::FET4)};
ROTA rota[4] = {
    ROTA(pindef::RE1A,pindef::RE1B,pindef::RE1R),
    ROTA(pindef::RE2A,pindef::RE2B,pindef::RE2R),
    ROTA(pindef::RE3A,pindef::RE3B,pindef::RE3R),
    ROTA(pindef::RE4A,pindef::RE4B,pindef::RE4R)
};
bool comm_recv_set(CommMessage msg)
{
    UM7106_custom c;
    memcpy(c.raw, msg.data, sizeof(c));
    digitalWrite(LED_BUILTIN, HIGH);
    switch (msg.cmd)
    {
    case cmd7106::SET_FET:
        for(uint8_t i=0;i<4;i++){
            fet[i].set(c.value[i]);
        }
        break;
    case cmd7106::SET_RMMOTOR:
        if(c.select>1){
            logger.print("rm select out of range");
            break;
        }
        logger.print("rm set to %d",c.value[0]);
        break; 

        // case cmd7106
    }
    digitalWrite(LED_BUILTIN, LOW);
    return true;
}
bool comm_recv_request(CommMessage msg)
{
    lastReceived = millis();
    switch (msg.cmd)
    {
    case cmd7106::REQ_SENS:
    {
        UM7106_sens s;
        for (uint8_t i = 0; i < 4; i++)
        {
            s.current[i] = sens_data.cur[i];
            // s.rotenc_pos
            // s.rotenc_deltapos
            // s.status
        }
        brd_main.send(cmd7106::REQ_SENS, s.raw, sizeof(UM7106_sens));
        PRINT("sens data send");
    }
    break;
    }
    return true;
}
bool comm_set_motor(CommMessage msg)
{
    UM7106_meca_motor motor_set_buf;
    memcpy(motor_set_buf.raw, msg.data, sizeof(motor_set_buf));
 
    for(int i=0; i<4;i++){
           if(motor_set_buf.speed[i]<=255&&motor_set_buf.speed[i]>=-255){
            md03a[i].setSpeed(motor_set_buf.speed[i]);
        }
    }
    return true;
}
void report_status(uint8_t report_flag = 0B11111111)
{
    if (report_flag & report_current)
        PRINT("cur:%05d,%05d,%05d,%05d\n", sens_data.cur[0], sens_data.cur[1], sens_data.cur[2], sens_data.cur[3]);
    if (report_motor)
        PRINT("mot:%4d,%4d,%4d,%4d\n    %4d,%4d,%4d,%4d\n", md03a[0].getSpeed(), md03a[1].getSpeed(), md03a[2].getSpeed(), md03a[3].getSpeed(), extmd[0].getSpeed(), extmd[1].getSpeed(), extmd[2].getSpeed(), extmd[3].getSpeed());
    // if(report_rotenc)PRINT("rot:%4d,%4d,%4d,%4d",rotenc[0].readPos(),rotenc[1].readPos(),rotenc[2].readPos(),rotenc[3].readPos());
    if (report_fet)
        PRINT("fet:%d,%d,%d,%d\n", fet[0].get(), fet[1].get(), fet[2].get(), fet[3].get());
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
    if (c.match("fet "))
    {
        PRINT("fet %d millisec set\n", c.args[0].Value);
        fet[0].set(1);
        digitalWrite(LED_BUILTIN, 1);
        // delay(c.args[0].Value);
        fet[0].set(0);
        digitalWrite(LED_BUILTIN, 0);
        PRINT("fet off");
    }

    if (c.match("mtr "))
    {
        int32_t num;
        int32_t spd;
        if (c.hasOption('S'))
        {
        }
        if (c.getArg('n', &num) && c.getArg('s', &spd))
        {
            if(constrain(spd,-255,255)!=spd){
                PRINT("speed out of range");
                return;
            }
            if (num < 4)
            {
                md03a[num].setSpeed(spd);
                PRINT("motor %d set to %d\n", num, spd);
            }
            else if (num < 8)
            {
                extmd[num - 4].setSpeed(spd);
                PRINT("motor %d set to %d\n", num, spd);
            }
            else
            {
                PRINT("motor %d is not exist\n", num);
            }
        }
        else
        {
            PRINT("mtr command usage: mtr ns [motor number] [speed(-255~255)]");
        }
        return;
    }
    if (c.match("sens"))
    {
        int32_t interval;
        if (c.getArg('a', &interval))
        {
            PRINT("sensor auto report enabled");
            sens_report_interval = interval;
            return;
        }
        bool invert = false;

        if (c.hasOption('D'))
        {
            invert = true;
        }
        if (c.hasOption('F'))
            invert ? report_enable_flag &= ~(report_fet) : report_enable_flag |= report_fet;
        if (c.hasOption('M'))
            invert ? report_enable_flag &= ~(report_motor) : report_enable_flag |= report_motor;
        if (c.hasOption('C'))
            invert ? report_enable_flag &= ~(report_current) : report_enable_flag |= report_current;
        PRINT("report en:cur:%d,mot:%d,fet:%d", report_enable_flag & report_current, report_enable_flag & report_motor, report_enable_flag & report_fet);

        PRINT("board status:");
        report_status(report_enable_flag);
    }
    if (c.match("reb "))
      {
        logger.info("rebooting");
        delay(250);
        if (c.hasOption('I'))
        {
          reset_usb_boot(0, 0);
        }
        else
        {
          watchdog_reboot(0, 0, 0);
        }
      }
}
void periph_init()
{
    usb.init();
    sens_serial.begin(38400, SERIAL_8N1);
    MCP23017_WIRE.setSCL(pindef::SCL);
    MCP23017_WIRE.setSDA(pindef::SDA);
    logger.info(exIO::init(MCP23017_WIRE)? "IO init ok":"IO init failed");
    UART_SERIAL.setTX(pindef::UART_TX);
    UART_SERIAL.setRX(pindef::UART_RX);
    UART_SERIAL.begin(CFG_UART_BAUD);
    uart.init([](uint8_t err){logger.info("uart error:%d",err);},Logger::DEBUG);
}
void setup()
{
    delay(1000);
    periph_init();
    for(int i=0; i<4; i++) rota[i].init();
    logger.info("setup");
    usb.setCommandCallback(commandReceived);
    CommManager::addBoard(&brd_main);
    CommManager::addBoard(&brd_mdc0);
    
    CommManager::addCommMethod(&uart); // first must be default method
    CommManager::addCommMethod(&usb);
    CommManager::init(ADDR_7106_0);
    CommManager::setCommReceiveCallbackDefault(defaultCallback);
    brd_main.addLink(cmd7106::SET_FET, NULL, 0, comm_recv_set);
    brd_main.addLink(cmd7106::SET_MOTOR, NULL, 0, comm_set_motor);
    brd_main.addLink(cmd7106::REQ_SENS, NULL, 0, comm_recv_request);
    brd_main.addLink(cmd7106::SET_RMMOTOR, NULL, 0, comm_recv_set);

    analogWriteFreq(10000);
    logger.info("actuator init");
    for (uint8_t i = 0; i < 4; i++)
    {
        md03a[i].init();
        extmd[i].init();
        fet[i].init();
    }
    logger.info("boot ok");
}
// uc_setup完了後電源を切るまで繰り返し実行
uint32_t nocomm_elapsed = 0;
void loop()
{
    if (sens_report_interval && sens_report_interval + sens_report_last < millis())
    {
        sens_report_last = millis();
        report_status(report_current);
    }
    if (sens.read(sens_data.buf))
    {
        // 0.13v/A
        // 4.3V/1024
        for (int i = 0; i < 4; i++)
        {
            sens_data.cur[i] = map(sens_data.cur[i], 0, 4096, 0, 43000 / 13);
        }
        // PRINT("sens recv");
    }
    CommManager::update();
    if(millis()-lastReceived>1500&&millis()-nocomm_elapsed>1000){
        logger.fatal("no comm:failsafe");
        for (uint8_t i = 0; i < 4; i++)
        {
            md03a[i].setSpeed(0);
            extmd[i].setSpeed(0);
        }
        nocomm_elapsed=millis();

    }
}

#endif
