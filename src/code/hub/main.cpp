#ifdef PJ_HUB

#include <Arduino.h>
#include <NRUnified.h>
#include "driver_comm/UART.h"
#include "driver_comm/CANFD.h"
#include "driver_comm/MU.h"
#include "driver_comm/USBSerial.h"
#include <SerialPIO.h>
#include "board/CM7105.h"
#include "code/main/7101_class.h"
extern "C"
{
#include <hardware/watchdog.h>
#include <pico/bootrom.h>
};
#include "struct.h"
#include "project.h"
#include "config.h"

#include "driver_sens/MPU6050.h"
#define MU_DEFAULT_CH 2
// #include "library/FeetechServo.h"

Logger logger("main");
USBSerial usb;
CANFD can(pindef::CAN_CS, CAN_SPI, 255); // 割り込み使用しない
CommManager::Board brd_main(ADDR_7101_C, "main");
CommManager::Board brd_hub(ADDR_7105, "hub");

SerialPIO SerialMU(pindef::MU_RX, pindef::MU_TX); // UARTはtx-rx,rx-txになるため
MU mu_comm(&SerialMU, [](uint8_t err)
           { logger.error("MU error:%d\n", err); });
CommManager::Board mu(ADDR_IGNORE, "MU");
uint8_t mu_ch = MU_DEFAULT_CH;
uint32_t mu_freq = 0;
uint32_t mu_lastRecvCheck = 0;
uint32_t mu_recv_cnt = 0;
uint32_t mu_error_cnt = 0;
bool mu_online = false;
// CANFD can(pindef::CAN_CS, CAN_SPI, pindef::CAN_INT);
uint8_t mu_buf[16];
SerialPIO ser_hub[] =
    {
        SerialPIO(pindef::TX3, pindef::RX3, 64),
        SerialPIO(pindef::TX2, pindef::RX2, 64),

};
UARTDriver Serial_port[3] =
    {
        UARTDriver(&SerialHPORT),
        UARTDriver(&ser_hub[0]),
        UARTDriver(&ser_hub[1]),
};
CommMessage recv_buf[4];
UM7105_relay relay_buf;

MPU6050 imu(Wire);
MPU6050_sens imu_sens_zero;

bool relayDataProcess(CommMessage d) // メインから来たデータの転送
{

    logger.debug("relay recv: id=%d, cmd=%d, port=%d,from=%d\n", relay_buf.id, relay_buf.cmd, relay_buf.port, relay_buf.sendfrom);
    if (relay_buf.port >= sizeof(Serial_port) / sizeof(Serial_port[0]))
        return false;
    CommMessage data = {
        .fromid = relay_buf.sendfrom,
        .id = relay_buf.id,
        .cmd = relay_buf.cmd,
    };
    memcpy(data.data, relay_buf.data, 32);
    data.len = 32;
    Serial_port[relay_buf.port].send(data);
    // LOGS("send:%d bytes", Serial_port[relay_buf.port].send(&data));
    return true;
}

bool muSendDataProcess(CommMessage c) // メインから来たデータをMUに送信
{
    return true;
}

void serialErrorCall(uint8_t error)
{
    PRINT("error:%d\n", error);
}
bool defaultCallback(CommMessage msg)
{
    PRINT("defaultCallback:from=%d, id=%d, cmd=%d\n", msg.fromid, msg.id, msg.cmd);
    // if (msg.fromid != ADDR_7101_C)
    // {
    //     UM7105_relay relay;
    //     relay.id = msg.id;
    //     relay.cmd = msg.cmd;
    //     relay.sendfrom = msg.fromid;
    //     memcpy(relay.data, msg.data, 32);
    //     brd_main.send(cmd7105::DATA_RELAY, relay.raw, sizeof(relay.raw));
    // }
    return true;
}
bool muReceived(CommMessage msg)
{
    mu_recv_cnt++;
    logger.debug("muReceived len=%d,data[0]=%d\n", msg.len, msg.data[0]);
    UM7105_mu d = {};
    mu_online = true;
    d.emergency = 0;
    if (msg.len >= 5)
    {
        memcpy(d.ctrl, msg.data, 5);
        memcpy(d.data, msg.data + 5, msg.len - 5);
        d.data_len = msg.len - 5;
    }


    if (msg.len == 1 && msg.data[0] == 'E')
    {
        d.emergency = 1;
    }
    brd_main.send(cmd7105::RECV_MU, d.raw, sizeof(d));
    mu_lastRecvCheck = millis();
    return true;
}
void canErrorCB(uint32_t err)
{
    PRINT("CAN error:%d\n", err);
}
void cmdProcess(CMDMessage &c)
{
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
void muErrorCallBack(uint8_t err)
{
    logger.error("MU error:%d", err);
    mu_error_cnt++;
}
void periph_init()
{
    usb.init();
    delay(1000);
    CAN_SPI.setTX(pindef::CAN_MOSI);
    CAN_SPI.setRX(pindef::CAN_MISO);
    CAN_SPI.setSCK(pindef::CAN_SCK);
    CAN_SPI.begin();
    while (!can.init(canErrorCB, Logger::INFO))
    {
        logger.error("CAN init failed");
        delay(1000);
    }
    SerialHPORT.setTX(pindef::HTX);
    SerialHPORT.setRX(pindef::HRX);
    SerialHPORT.setFIFOSize(64);
    SerialHPORT.begin(CFG_UART_BAUD);
    ser_hub[0].begin(CFG_UART_BAUD);
    ser_hub[1].begin(CFG_UART_BAUD);
    // mu
    SerialMU.begin(19200);
    //mu_comm.setErrCallback(muErrorCallBack);
    mu_comm.init(2);
    mu_freq = mu_comm.getFreqFromCH(mu_ch);
    for(auto &s:Serial_port){
        s.init(serialErrorCall,Logger::DEBUG);
    }
    Wire.setSCL(pindef::SCL);
    Wire.setSDA(pindef::SDA);
    Wire.begin();
    imu.init();

    MPU6050_setting s;
    //library default
    s.acc_range=MPU6050_setting::range_16g;
    s.gyro_range=MPU6050_setting::range_250dps;    
    imu.applySetting(s);
    imu.update(imu_sens_zero);
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

        case configAddr7105::MU_CH:
            mu_ch = c.cfg[i].value;
            mu_freq = mu_comm.getFreqFromCH(mu_ch);
            mu_comm.init(mu_ch);
            logger.info("set mu ch:%d", mu_ch);
            break;
        default:
            logger.warn("unknown config addr:%d", c.cfg[i].addr);
        }
    }
    return true;
}
bool sensRequest(CommMessage c)
{
    UM7105_sens s;
    s.mu_ch = mu_ch;
    s.online = mu_online;
    s.status = 0;
    brd_main.send(cmd7105::REQ_SENS, s.raw, sizeof(s.raw));
    return true;
}
void tShowMUStat(){
    logger.info("mu recv:%d/s, error:%d/s",mu_recv_cnt,mu_error_cnt);
    mu_recv_cnt=0;
    mu_error_cnt=0;
}
void setup()
{
    logger.setLevel(Logger::INFO);
    pinMode(LED_BUILTIN, OUTPUT);
    delay(1000);
    periph_init(); // 物理ペリフェラル初期化
    logger.info("peri init done");
    // 通信コールバック等初期化
    usb.setCommandCallback(cmdProcess);
    CommManager::addBoard(&brd_main);
    CommManager::addBoard(&brd_hub);
    CommManager::addBoard(&mu);
    CommManager::addCommMethod(&can);
    CommManager::addCommMethod(&usb);
    CommManager::addCommMethod(&mu_comm);
    CommManager::init(ADDR_7105);
    CommManager::setCommReceiveCallbackDefault(defaultCallback);
    brd_main.addLink(cmd7105::DATA_RELAY, relay_buf.raw, sizeof(relay_buf), &relayDataProcess);
    brd_main.addLink(cmd7105::REQ_SENS, NULL, 0, &sensRequest);
    brd_main.addLink(cmd7105::SEND_MU, mu_buf, 16, &muSendDataProcess);
    brd_main.addLink(cmd7105::CONFIG, NULL, 0, &configProcess);
    mu.addLink(255, mu_buf, 16, &muReceived);
    // brd_main.addLink(cmd7105::REQ_SENS, mu_buf, 16, &uc_muReceieved);
    logger.info("setup done");
}
// uc_setup完了後電源を切るまで繰り返し実行
uint32_t lastIMU = 0;
MPU6050_sens imu_sens;  
float real_theta=0;
void loop()
{
    for (int i = 0; i < 3; i++)
    {
        if (Serial_port[i].receive(recv_buf[i]))
        {
            UM7105_relay d;
            d.id = recv_buf[i].id;
            d.sendfrom = recv_buf[i].fromid;
            d.cmd = recv_buf[i].cmd;
            d.port = (UM7105_relay::port_sel)i;
            memcpy(d.data, recv_buf[i].data, 32);
            brd_main.send(cmd7105::DATA_RELAY, d.raw, sizeof(d));
            //logger.info("recv:%d bytes from %d", recv_buf[i].len, recv_buf[i].fromid);
        }
    }
    if (millis() - mu_lastRecvCheck > 500)
    {
        mu_online = false;

        mu_lastRecvCheck = millis();
    }
    CommManager::update();
    digitalWrite(LED_BUILTIN, millis() % 1000 > 500);
    if(millis()-lastIMU>=100){
        lastIMU=millis();
        // imu.update(imu_sens);
        // logger.info("imu:%5d,%5d,%5d|%5d,%5d,%5d",imu_sens.getAccX(),imu_sens.getAccY(),imu_sens.getAccZ(),imu_sens.getGyroX(),imu_sens.getGyroY(),imu_sens.getGyroZ());
        // real_theta+=map(imu_sens.getGyroX()-imu_sens_zero.getGyroX(),-32768,32767,-250,250)*0.1;
        // logger.info("theta:%f",real_theta);
    }
}

#endif