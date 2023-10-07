// ビルド環境選択時に定義
#ifdef PJ_MAIN
#include <Arduino.h>
#include <NRUnified.h>//ロボコンライブラリ群
#include <driver_comm/CANFD.h>//基板通信用

#include "7101_class.h" //class定義用
#include "driver_comm/USBSerial.h"//PCとの通信用
#include "board/CM7101.h"//この基板のピン配置等情報ヘッダ
#include "Debug.h"//デバッグ用
#include <struct.h>//通信データ等の構造体
#include <project.h>//チームごとの共有データ
#include <config.h>//通信速度,アドレス等共通パラメータ保存用
#include <math.h>//M_PI用
#include <SerialPIO.h>

#define power 3//モーター電圧調整用(係数)
#define direction 8.0              //何方向か、8.0か16.0
/*m.speed[0]        メカナム用モータ0~3の4コ
  m.speed[1]
  m.speed[2]
  m.speed[3]
  hm.speed[0]　　　上下モータ
  hm.speed[1]      ひも(お助けモータ)
  hm.speed[2]      左右モータ
*/
/*limit[0]      上下機構の上のリミットスイッチ
　limit[1]      上下機構の下のリミットスイッチ
  limit[2]      左右機構の1つ目
  limit[3]      左右機構の2つ目
*/
Logger logger("main");//メインプログラムのデバッガー
USBSerial usb;//PCとの通信管理
CommManager::Board brd_emg(ADDR_7104_U, "7104U", ADDR_7105);//非常停止(ハブポート0経由)
CommManager::Board brd_hub(ADDR_7105, "7105");//ハブ
CommManager::Board brd_md0(ADDR_7106_0, "7106_0", ADDR_7105);//MD(ハブポート1経由)
CommManager::Board brd_md1(ADDR_7106_1, "7106_1", ADDR_7105);//MD(ハブポート2経由)
CommManager::Board brd_main(ADDR_7101_C, "7101C");//この基板
uint16_t onlinestate = 0;//各基板接続状況
// CANFD can(pindef::CAN_CS, CAN_SPI, pindef::CAN_INT);
CANFD can(pindef::CAN_CS, CAN_SPI, 255); //CANFD通信管理 割り込み使用しない設定

//各基板データ構造体
UM7104_sens sens7104;
UM7105_sens sens7105;
UM7106_sens sens7106_0;
UM7106_sens sens7106_1;
UM7105_mu mu7105;
UM7105_relay relay7105;
UM7106_meca_motor m = {};
UM7106_help_motor hm = {};
UM7106_link_motor lm = {0,0,150,0,90};//定数調整
UM7106_custom c = {};
UM7106_sens s = {};
UM7106_led ledc = {};

//mu管理変数
uint8_t target_mu_ch  = 8;
uint8_t current_mu_ch = 0;
uint32_t mu_freq = 0;
bool emergency = false;

//コントローラー管理
ControllerData cd;//受信バッファ
ControllerManager controller;//コントローラー管理クラス

struct logbuf
{
    bool push(std::string s)
    {
        buf[top] = s;
        top < 7 ? top++ : top = 0;
        return true;
    }
    bool get(std::string &returnto, uint8_t n)
    {
        if (n >= 8)
            return false;
        int8_t index = top - n - 1;
        if (index < 0)
            index += 8;
        returnto = buf[index];
        return true;
    }
    uint8_t top = 0;
    std::string buf[8] = {};
};
#include "driver_acc/OLEDDriver.h"//OLEDの制御
#include "ui_bitmap.h"//OLEDに表示する画像等のデータ
Logger logger_oled("oled");
OLEDDriver oled = OLEDDriver(&OLED_I2C);
std::queue<std::string> logqueue;
logbuf oled_log;

//リミットスイッチ
LIMITS limit[4]{
    LIMITS(pindef::Limit1), //上
    LIMITS(pindef::Limit2), //下
    LIMITS(pindef::GP0), //右
    LIMITS(pindef::GP1), //左
};


// コントローラーのデータが更新されたときに実行される
void controllerReceived()
{
   //メカナムホイール
        // int X = controller.stick.L.real()+1; //-16 ~ 14 → -15 ~ 15
        // int Y = controller.stick.L.imag()+1; //-16 ~ 14 → -15 ~ 15
        int X = controller.stick.L.real(); //-16 ~ 14
        int Y = controller.stick.L.imag(); //-16 ~ 14
        double abs_value = sqrt(pow(X,2)+pow(Y,2));             //大きさ どれくらい倒したか

     if(controller.stick.L != controller.stick_old.L){         //変化があった時       16方向

            // X++;
            // Y++;
            PRINT("%d,%d",X,Y);
            double degree = atan((double)Y/X);          //角度(弧度法)


       //tanの変域が-π/2~π/2なので0~2πの範囲に絞る
       if(X == 0){

            if(Y == 0){
            }else if(Y > 0){
            degree = M_PI/2;             //初期状態(X,Y)(0,0)
            }else if(Y < 0){             //Y < 0
            degree = M_PI*3/2;
            }

        }else if(X > 0){

        if(Y == 0){
            degree = degree;
        }else if(Y > 0){                //第一象限
            degree = degree;
        }else if(Y < 0){                //第四象限
            degree = degree + 2*M_PI;
        }

        }else if(X < 0){                //第二,三象限
           degree = degree + M_PI;

        }

        //degreeの値を切り捨てor切り上げで8,16方向に
        if(degree >= (2.0*direction-1.0)/direction*M_PI && degree < 2.0*M_PI){        //角度が0(上方向)の時
                    degree = 0.0;
        }
        if(degree >= 0.0*M_PI && degree < 1.0/direction*M_PI){                        //角度が0(下方向)の時
            degree = 0.0;
        }

        for (int i = 0;i < direction;i++){

                if(degree >= (-1.0 + 2.0*i)/direction*M_PI  && degree < (1.0 + 2.0*i)/direction*M_PI){
                    degree = 1.0/direction*2.0*M_PI*i;

                }

        }
        //  PRINT("degree = %f",degree);
        //  PRINT("degree = %fπ",degree/M_PI);

        //モーターの速度
          for(int i = 0;i < (int)direction;i++){
                if(degree == (double)1/direction*2.0*i*M_PI){
                m.speed[0] = (-cos(M_PI/direction*2.0*i)+sin(M_PI/direction*2.0*i))*abs_value*power;
                m.speed[1] = (+cos(M_PI/direction*2.0*i)+sin(M_PI/direction*2.0*i))*abs_value*power;
                m.speed[2] = (+cos(M_PI/direction*2.0*i)+sin(M_PI/direction*2.0*i))*abs_value*power;
                m.speed[3] = (-cos(M_PI/direction*2.0*i)+sin(M_PI/direction*2.0*i))*abs_value*power;
                brd_md0.send(cmd7106::SET_MOTOR, m.raw, sizeof(m.raw));
            }
            }
          PRINT("abs_value = %f",abs_value );
          PRINT("m.speed[0] = %f",m.speed[0]);
          PRINT("m.speed[1] = %f",m.speed[1]);
          PRINT("m.speed[2] = %f",m.speed[2]);
          PRINT("m.speed[3] = %f",m.speed[3]);


        }

        int Xr = controller.stick.R.real();
        int Yr = controller.stick.R.imag();
        double abs_value_R = sqrt(pow(Xr,2)+pow(Yr,2));
        PRINT("(Xr,Yr) = (%d,%d)",Xr,Yr);

    if(controller.stick.R != controller.stick_old.R){//旋回
        if(Xr < 0){
            m.speed[0] = -abs_value_R*3;
            m.speed[2] =  abs_value_R*3;
        }else if(Xr > 0){
            m.speed[0] = abs_value_R*3;
            m.speed[2] = -abs_value_R*3;
        }else{
            if(Yr > 0){
                m.speed[0] = -abs_value_R*3;
                m.speed[2] =  abs_value_R*3;
            }else if(Yr < 0){
                m.speed[0] = abs_value_R*3;
                m.speed[2] = -abs_value_R*3;
            }else{
                m.speed[0] = 0;
                m.speed[2] = 0;
            }
        }
            m.speed[1] =  m.speed[0];
            m.speed[3] =  m.speed[2];
            brd_md0.send(cmd7106::SET_MOTOR, m.raw, sizeof(m.raw));
    }

     //変化があった時          座標的
   /* if(controller.stick.L != controller.stick_old.L){

        PRINT("%d,%d",X,Y);
        m.speed[0] = 2*(Y-X);
        m.speed[1] = 2*(Y+X);
        m.speed[2] = 2*(Y+X);
        m.speed[3] = 2*(Y-X);
        brd_md0.send(cmd7106::SET_MOTOR, m.raw, sizeof(m.raw));

    }*/

    //変化がなかったとき
    if(controller.stick.L == controller.stick_old.L && controller.stick.R == controller.stick_old.R){
        int X = controller.stick.L.real()+1; //-16 ~ 14 → -15 ~ 15
        int Y = controller.stick.L.imag()+1; //-16 ~ 14 → -15 ~ 15

        int Xr = controller.stick.R.real()+1; //-16 ~ 14 → -15 ~ 15
        int Yr = controller.stick.R.imag()+1; //-16 ~ 14 → -15 ~ 15
        // PRINT("%d,%d",X,Y);
        m.speed[0] = 0;
        m.speed[1] = 0;
        m.speed[2] = 0;
        m.speed[3] = 0;

        brd_md0.send(cmd7106::SET_MOTOR, m.raw, sizeof(m.raw));
    }

    //リンク
    if(controller.isPressed(button::A)){//初期位置
        ledc.r = 0;
        ledc.g = 0;
        ledc.b = 250;
        ledc.mode = 2;
        brd_md1.send(cmd7106::SET_LED,ledc.raw,sizeof(ledc.raw));
        PRINT("/A");
        lm.speed = 1000;
        brd_md1.send(cmd7106::SET_LINK,lm.raw,sizeof(lm.raw));
    }
    if(controller.isPressed(button::B)){//位置調整
        lm.speed = -1000;
        lm.abuspeed = 30;
        brd_md1.send(cmd7106::SET_LINK,lm.raw,sizeof(lm.raw));
    }
    if(controller.isPressed(button::UP)){//前進
        lm.speed = 50;
        lm.abuspeed = 30;
        brd_md1.send(cmd7106::SET_LINK,lm.raw,sizeof(lm.raw));
    }
    if(controller.isPressed(button::DOWN)){//後退
        lm.speed = -50;
        lm.abuspeed = -50;
        brd_md1.send(cmd7106::SET_LINK,lm.raw,sizeof(lm.raw));
    }
    if(controller.isReleased(button::UP)||controller.isReleased(button::DOWN)){//停止
        lm.speed = 0;
        lm.abuspeed = 0;
        brd_md1.send(cmd7106::SET_LINK,lm.raw,sizeof(lm.raw));
    }
    //お助けアイテム
        //上下
    if(controller.ctrl.Trigger.L > 3){//LTを押したとき 上昇する
        ledc.mode = 3;
        ledc.r= controller.ctrl.Trigger.L * 17;
        ledc.g=0;
        ledc.b=0;
        if(limit[0].push()){//リミットスイッチが押されてなかったら
            ledc.r = 10;
            ledc.g = 50;    
            ledc.b = 30;
            hm.speed[0] =  controller.ctrl.Trigger.L * (power+2);
            brd_md1.send(cmd7106::SET_MOTOR,hm.raw,sizeof(hm.raw));
        }
        brd_md1.send(cmd7106::SET_LED, ledc.raw,sizeof(ledc.raw));

    } else if(controller.ctrl.Trigger.R > 3){//RTを押したとき 下降する
        if(limit[1].push()){
            hm.speed[0] =  controller.ctrl.Trigger.R * (power+2) * (-1) ;
            brd_md1.send(cmd7106::SET_MOTOR,hm.raw,sizeof(hm.raw));
        }
    } else {//LT,RTを押していないとき
        hm.speed[0] = 0;
        brd_md1.send(cmd7106::SET_MOTOR,hm.raw,sizeof(hm.raw));
    }

        //ひも
    if(controller.isPressed(button::X)){//Xボタンを押したとき ひもを引っ張る
        hm.speed[1] = 50;
        brd_md1.send(cmd7106::SET_MOTOR,hm.raw,sizeof(hm.raw));
    }else if(controller.isPressed(button::Y)){//Yボタンを押したとき ひもを緩める
        hm.speed[1] = -50;
        brd_md1.send(cmd7106::SET_MOTOR,hm.raw,sizeof(hm.raw));
    }
    if(controller.isReleased(button::X)||controller.isReleased(button::Y)){
        hm.speed[1] = 0;
        brd_md1.send(cmd7106::SET_MOTOR,hm.raw,sizeof(hm.raw));
    }

        //左右
    if(controller.isPressed(button::L)){//左右に振る
        if(limit[2].push()){
            ledc.r = 50;
            ledc.g = 50;
            ledc.b = 50;
            ledc.mode = 20;
            brd_md1.send(cmd7106::SET_LED,ledc.raw,sizeof(ledc.raw)); 
            hm.speed[2] = 20;
            brd_md1.send(cmd7106::SET_MOTOR,hm.raw,sizeof(hm.raw));
        }
    }else if(controller.isPressed(button::R)){
        if(limit[3].push()){
            hm.speed[2] = -20;
            brd_md1.send(cmd7106::SET_MOTOR,hm.raw,sizeof(hm.raw));
        }
    }
    if(controller.isReleased(button::L)||controller.isReleased(button::R)){
        hm.speed[2] = 0;
        brd_md1.send(cmd7106::SET_MOTOR,hm.raw,sizeof(hm.raw));
    }
}

//mu受信時に実行
bool muReceived(CommMessage msg)
{
    if (!mu7105.emergency)
    {
        //コントローラーデータの受信、非常停止解除処理
        memcpy(&cd, mu7105.ctrl, sizeof(mu7105.ctrl));
        controller.ctrl = cd;
        controller.update();
        controllerReceived();
        if(mu7105.emergency!=emergency){
            logger.info("emergency:off",emergency);
            oled_log.push("emergency off");
            config c;
            c.cfg[0].addr = configAddr7104::EMG_STOP;
            c.cfg[0].value = 0;
            brd_emg.send(cmd7104::CONFIG, c.raw, sizeof(c.raw));
        }
        emergency = false;
    }
    else
    {
        //非常停止
        emergency = true;
        logger.error("emergency!");
        if(mu7105.emergency!=emergency){
            logger.info("emergency:on",emergency);
            oled_log.push("emergency!");
            config c;
            c.cfg[0].addr = configAddr7104::EMG_STOP;
            c.cfg[0].value = 1;
            brd_emg.send(cmd7104::CONFIG, c.raw, sizeof(c.raw));
        }
        controller.ctrl = ControllerData(); // clear
    }
    controller.ctrl_old = controller.ctrl;
    // PRINT("muReceived\n");
    return true;
}
//muチャンネル変更
void changeMU_CH(uint8_t ch)
{
    config c;
    c.cfg[0].addr = configAddr7105::MU_CH;
    c.cfg[0].value = ch;
    brd_hub.send(cmd7105::CONFIG, c.raw, sizeof(c.raw));
}
//センサーデータ受信時に実行
bool sensorDataProcess(CommMessage msg)
{
    //PRINT("process: id=%d, cmd=%d", msg.fromid, msg.cmd);
    if (msg.fromid == brd_emg.address)
    {
        //PRINT("volt:%4.1f", sens7104.power_15.voltage / 10.0f);
        //PRINT("curr:%4.1f", sens7104.power_15.current / 10.0f);
        if (sens7104.virtual_sw != emergency)
        {
            logger.info("emerg set:%d", emergency);
            config c;
            c.cfg[0].addr = configAddr7104::EMG_STOP;
            c.cfg[0].value = emergency;
            brd_emg.send(cmd7104::CONFIG, c.raw, sizeof(c.raw));
        }
    }
    if (msg.fromid == brd_hub.address)
    {
        current_mu_ch = sens7105.mu_ch;
        if (sens7105.mu_ch != target_mu_ch)
        {
            changeMU_CH(target_mu_ch);
            logger.info("change mu_ch:%d", current_mu_ch);
        }
    }
    if (msg.fromid == brd_md0.address)
    {
       // PRINT("cur1:%5d, cur2:%5d, cur3:%5d, cur4:%5d\n", sens7106_0.current[0] * 10, sens7106_0.current[1] * 10, sens7106_0.current[2] * 10, sens7106_0.current[3] * 10);
    }
    if (msg.fromid == brd_md1.address)
    {
       // PRINT("%d\n",msg.data);
    }
    return true;
}
//ハブを経由する基板からの受信処理（変更不要）
bool relayDataProcess(CommMessage msg)
{
    CommMessage c;
    c.id = relay7105.id;
    c.cmd = relay7105.cmd;
    c.fromid = relay7105.sendfrom;
    // logger.info("relayDataProcess: from=%d, id=%d, cmd=%d", c.fromid, c.id, c.cmd);
    memcpy(c.data, relay7105.data, sizeof(relay7105.data));
    CommManager::linkProcess(c);
    return true;
}
//フィルターに該当しなかったデータの処理
bool defaultCallback(CommMessage msg)
{
    //PRINT("defaultCallback: id=%d, cmd=%d\n", msg.id, msg.cmd);
    return true;
}
//ハブを経由する基板への送信処理（変更不要）
bool relaySend(CommMessage msg)
{
    // PRINT("relaySend: id=%d, cmd=%d\n", msg.id, msg.cmd);
    UM7105_relay rel;
    rel.cmd = msg.cmd;
    rel.id = msg.id;
    rel.sendfrom = msg.fromid;
    rel.port = msg.id >> 5;
    // logger.info("id:%d port:%d",rel.id, rel.port);
    memcpy(rel.data, msg.data, sizeof(rel.data));
    brd_hub.send(cmd7105::DATA_RELAY, rel.raw, sizeof(rel.raw));
    return true;
}
//CANエラー時に実行
void canErrorCB(uint32_t err)
{
    logger.error("can err:%d", err);
}
//PCからのコマンド受信時に実行
void cmdCB(CMDMessage &c)
{
}
//通信の初期化
void peripheral_init()
{
    usb.init();
    CAN_SPI.setTX(pindef::CAN_MOSI);
    CAN_SPI.setRX(pindef::CAN_MISO);
    CAN_SPI.setSCK(pindef::CAN_SCK);
    CAN_SPI.begin();
    while (!can.init(canErrorCB, Logger::INFO))
    {
        logger.error("CAN init failed");
        delay(1000);
    }
}

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);

    peripheral_init();
    for(int i=0; i<4; i++) limit[i].init();
    usb.setCommandCallback(cmdCB);

    CommManager::init(ADDR_7101_C);
    CommManager::addCommMethod(&can);
    CommManager::addCommMethod(&usb);
    CommManager::addBoard(&brd_emg);
    CommManager::addBoard(&brd_hub);
    CommManager::addBoard(&brd_md0);
    CommManager::addBoard(&brd_md1);
    CommManager::addBoard(&brd_main);
    CommManager::setCommReceiveCallbackDefault(defaultCallback);
    CommManager::setRelayCallback(relaySend);

    brd_hub.addLink(cmd7105::RECV_MU, mu7105.raw, sizeof(mu7105.raw), muReceived);
    brd_emg.addLink(cmd7104::REQ_SENS, sens7104.raw, sizeof(sens7104.raw), sensorDataProcess);
    brd_hub.addLink(cmd7105::REQ_SENS, sens7105.raw, sizeof(sens7105.raw), sensorDataProcess);
    brd_md0.addLink(cmd7106::REQ_SENS, sens7106_0.raw, sizeof(sens7106_0.raw), sensorDataProcess);
    brd_md1.addLink(cmd7106::REQ_SENS, sens7106_1.raw, sizeof(sens7106_1.raw), sensorDataProcess);
    brd_hub.addLink(cmd7105::DATA_RELAY, relay7105.raw, sizeof(relay7105.raw), relayDataProcess);

}
uint32_t lastUpdate = 0;
void loop()
{
    CommManager::update();
    if (millis() - lastUpdate > 500){
        PRINT("%d%d%d%d",limit[0].push(),limit[1].push(),limit[2].push(),limit[3].push());
        if(limit[0].push() || limit[1].push()){
            hm.speed[0] = 0;
            brd_md1.send(cmd7106::SET_MOTOR,hm.raw,sizeof(hm.raw));
        }
        if(limit[2].push() || limit[3].push()){
            hm.speed[2] = 0;
            brd_md1.send(cmd7106::SET_MOTOR,hm.raw,sizeof(hm.raw));
        }
    }
    if (millis() - lastUpdate > 500)
    {
        lastUpdate = millis();
        uint8_t data[1] = {0};
        brd_emg.send(cmd7104::REQ_SENS, data, 1);
        brd_hub.send(cmd7105::REQ_SENS, data, 1);
        brd_md0.send(cmd7106::REQ_SENS, data, 1);
        brd_md1.send(cmd7106::REQ_SENS, data, 1);
    };
    digitalWrite(LED_BUILTIN, millis() % 1000 > 500);
    // digitalWrite(LED_BUILTIN, limit[0].status);
    // PRINT("%d",limit[0].status);
    //digitalWrite(LED_BUILTIN, limit[0].push());
    //digitalWrite(LED_BUILTIN, millis() % 1000 > 500);

     // m.speed[0] = 60;
      //brd_md0.send(cmd7106::SET_MOTOR, m.raw, sizeof(m.raw));


    // UM7106_motor m = {};
    // for(int i=0; i<4; i++){
    //   m.speed[i] = 50;
    // }
    // brd_md0.send(cmd7106::SET_MOTOR, m.raw, sizeof(m.raw));

    // UM7106_motor m = {};//前進
    // for(int i=0; i<4; i++){
    //     m.speed[i] = 50;
    // }
    //brd_md0.send(cmd7106::SET_MOTOR, m.raw, sizeof(m.raw));

    // UM7106_motor m = {};//後進
    // for(int i=0; i<4; i++){
    //     m.speed[i] = -50;
    // }
    // brd_md0.send(cmd7106::SET_MOTOR, m.raw, sizeof(m.raw));

    // UM7106_motor m = {};//右
    // for(int i=0; i<4; i++){
    //     if(i==0 || i==3) m.speed[i] = 50;
    //     else m.speed[i] = -50;
    // }
    // brd_md0.send(cmd7106::SET_MOTOR, m.raw, sizeof(m.raw));

    // UM7106_motor m = {};//左
    // for(int i=0; i<4; i++){
    //     if(i==0 || i==3) m.speed[i] = -50;
    //     else m.speed[i] = 50;
    // }
    // brd_md0.send(cmd7106::SET_MOTOR, m.raw, sizeof(m.raw));

    // UM7106_motor m = {};//右旋回
    // for(int i=0; i<4; i++){
    //     if(i == 2 || i == 0) m.speed[i] = 50;
    //     else m.speed[i] = -50;
    // }
    // brd_md0.send(cmd7106::SET_MOTOR, m.raw, sizeof(m.raw));

    // UM7106_motor m = {};//左旋回
    // for(int i=0; i<4; i++){
    //     if(i == 2 || i == 0) m.speed[i] = -50;
    //     else m.speed[i] = 50;
    // }
    // brd_md0.send(cmd7106::SET_MOTOR, m.raw, sizeof(m.raw));

    // UM7106_motor m = {};//右斜め前進
    // for(int i=0; i<4; i++){
    //     if(i==0 || i==3) m.speed[i] = 50;
    //     else m.speed[i] = 0;
    // }
    // brd_md0.send(cmd7106::SET_MOTOR, m.raw, sizeof(m.raw));

    // UM7106_motor m = {};//左斜め後進
    // for(int i=0; i<4; i++){
    //     if(i==0 || i==3) m.speed[i] = 0;//-50
    //     else m.speed[i] = 0; //50
    // }
    // brd_md0.send(cmd7106::SET_MOTOR, m.raw, sizeof(m.raw));

    // UM7106_motor m = {};//左斜め前進
    // for(int i=0; i<4; i++){
    //     if(i==1 || i==2) m.speed[i] = 50;
    //     else m.speed[i] = 0;
    // }
    // brd_md0.send(cmd7106::SET_MOTOR, m.raw, sizeof(m.raw));

//         UM7106_motor m = {};//右斜め後進
//     for(int i=0; i<4; i++){
//         if(i==1 || i==2) m.speed[i] = -50;
//         else m.speed[i] = 0;
//     }
//     brd_md0.send(cmd7106::SET_MOTOR, m.raw, sizeof(m.raw));
}

// OLED搭載のボードを利用している場合、OLEDの初期化時に呼び出される。
// OLEDのページを追加する場合などに使う

// // コマンド受信時に実行
// void uc_CommandReceived(CMDMessage &c)
// {
//     if(c.match("mtr ")){
//     int32_t n,pwm;
//         if(c.getArg('n',&n)&&c.getArg('s',&pwm)){
//             if(n<0||n>3)return;
//             UM7106_motor m={};
//             m.select=n;
//             m.value=pwm;
//             brd_md0.send(cmd7106::SET_MOTOR,m.raw,sizeof(m.raw));
//             PRINT("mtr %d %d\n",n,pwm);
//         }
//     }
// }
constexpr uint8_t MAX_CHAR_PR = 21;
constexpr char version[] = "0.0.1";

std::map<char[3], bool> status;

void bootscreen(OLEDDriver &s)
{
    if (s.getFrame() == 0)
    {
        oled_log.push("Ateam ver." + std::string(version));
        oled_log.push("Booting...");
    }
    for (uint8_t i = 0; i < 8; i++)
    {
        std::string t;
        oled_log.get(t, i);
        s.setCursor(0, s.charY(i));
        s.print(t.c_str());
    }
    s.display();
    if (s.getFrame() > 30)
    {
        oled.setPage(1);
    }
}

const uint8_t ctrl_posX = 43;
const uint8_t ctrl_posY = 0;
void maskControllerStatus(OLEDDriver &oled, ControllerData *c)
{
    for (int i = 0; i < 15; i++)
    {
        if (!bitRead(c->buffer[i / 8], i % 8))
            oled.fillRect(buttonPos[i][0] + ctrl_posX, buttonPos[i][1] + ctrl_posY, buttonPos[i][2], buttonPos[i][3], 0);
    }
    oled.fillRect(offsetStick[0][0] + ctrl_posX + (c->Lstick.X * 6 / 15), offsetStick[0][1] + ctrl_posY + 6 - (c->Lstick.Y * 6 / 15), 2, 2, 1);
    oled.fillRect(offsetStick[1][0] + ctrl_posX + (c->Rstick.X * 6 / 15), offsetStick[1][1] + ctrl_posY + 6 - (c->Rstick.Y * 6 / 15), 2, 2, 1);
    oled.fillRect(offsetTrig[0][0] + ctrl_posX, offsetTrig[0][1] + ctrl_posY, 2, 6 - round((float)c->Trigger.L * 0.375), 0);
    oled.fillRect(offsetTrig[1][0] + ctrl_posX, offsetTrig[1][1] + ctrl_posY, 2, 6 - round((float)c->Trigger.R * 0.375), 0);
    oled.setText(ui_ctrlVal, " X  Y");
    oled.setText(ui_ctrlValX, "L%2d %2d", c->Lstick.X-8, c->Lstick.Y-8);
    oled.setText(ui_ctrlValY, "R%2d %2d", c->Rstick.X, c->Rstick.Y);
    oled.setText(ui_ctrlValT, "T%2d %2d", c->Trigger.L, c->Trigger.R);
}

void mainscreen_init(OLEDDriver &s)
{
    logger_oled.info("mainscreen_init");
    for (int i = 0; i < sizeof(main_ui_data) / sizeof(main_ui_data[0]); i++)
    {
        oled.addElement(main_ui_data[i]);
    }
}

void mainscreen(OLEDDriver &s)
{
    s.setText(ui_connection, emergency ? "EM Stop" : sens7105.online ? "Ready"
                                                                     : "Waiting");
    s.setText(ui_ch, "C\nH");
    s.setText(ui_channel, "%02X", current_mu_ch);
    s.setBitmap(ui_ctrlPos, b_controller);
    // xQueuePeek(queueController, &controllerData, 0);
    maskControllerStatus(s, &controller.ctrl);
    s.draw(OLEDData{.type = OLEDType::pix, .x = (uint8_t)(s.getFrame() % 128), .y = 32, .color = false});
    for (int i = 0; i < 4; i++)
    {
        std::string t;
        oled_log.get(t, i);
        s.setText(ui_print1 + i, t.c_str());
    }
    s.display();
}
void logview(OLEDDriver &s)
{
}
void setup1()
{
    delay(500);
    OLED_I2C.setSDA(pindef::SDA);
    OLED_I2C.setSCL(pindef::SCL);
    oled.init();
    oled.setPageCallback(0, bootscreen);
    oled.setPageCallback(1, mainscreen, mainscreen_init);
    oled.setPageCallback(2, logview);
    oled.setFPS(15);
    oled.setTextWrap(false);
    oled.setPage(0);
}
void loop1()
{
    oled.update();
}

#endif
