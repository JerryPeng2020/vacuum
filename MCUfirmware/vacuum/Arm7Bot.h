/****************************************************
/* 7Bot(V2) class for Arduino platform
/* Author: Jerry Peng
/* Date: Aug 8th, 2021
/* 
/* Version 1.0 (set version 2.3 to 1.0, as a new stable release version)
/* www.7bot.cc
/*  
/* Description: 
    1. Change comunication command to register read and write version.
    2. Reduce pose control commands from 2 bytes to 1 byte. 
    3. Add CRC to ensure command transmission.
    4. Add read joint angle function

Changes:
  1. update all angular parameters (Unit: Degree) name to from [pos] to [angle]
  2. add IK 
  3. change IK calculation from double precision to float precision
  4. add pose record function by click buttons

  2021-5-26 add IK5 UART command
  2021-9-21 disable valve, valve is useless due to two results:
            1. if valve on, valve will heat up quickly;
            2. valve on(Hight)/off(Low) do not affect result at all.



需要优化的地方：
（一）硬件
1. 加机械限位，防止joint[0]，套圈。 同时说明书中加注意事项说明，通过servo[0]开孔看线判断有没有套圈
2. 电路板增加reset按键

（二）固件：
0. 增加电机间几何限位检测（当前角度控制还没有）
1. 增加软件握手check机制
2. 增加软硬件pose上传/下载功能(暂缓，优先支持智能系统应用搭建)
5. 增加外力等卡位检测（控制角度与读取角度不收敛）  FSM_status = 4


V3.0
2. WiFi/BT 功能及配置
5. 将舵机运动范围适当扩大？？？ 暂缓，除非建模上大改

/***************************************************/

#ifndef _ARM7BOT_H
#define _ARM7BOT_H

#include "ServoProtocol.h"
#include "EEPROM.h"

#include "PVector.h"
#include "PressFilter.h"

#define SERVO_NUM 7

/* Hardware Pins */
#define PUMP 32     // pump
#define VALVE 33    // valve
#define BUZZER 25   // buzzer

#define BUTTON1 14  // button1
#define BUTTON2 27  // button2
#define BUTTON3 26  // button3

/* System Parameters */
#define WAKEUP_TIME 2000  // 7Bot wake up time,  Unit: ms
#define INIT_SPEED 600  // after 7Bot wake up reset normal motion speed
#define INIT_TIME 1000  // after 7Bot wake up reset normal motion time

// Device ROM Data
#define DEVICE_TYPE 7
#define VERSION 10   // version 1.0

// EEPROM： 17 for register(currently, 9 used), 1 for record pose num, 7*254 for poses
//          254 poses can enable poses download from PC posible!!!!!!!!!!!!!!!!!!!!!!!
//          Add poses download command !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#define EEPROM_SIZE 1796
#define REG_EEPROM_SIZE 9
#define RECORD_POSE_NUM_ID 17
#define RECORD_POSE_DATA_ID 18
#define MAX_POSE_NUM 254

// Command Header
#define BEGIN_FLAG_0 0xAA
#define BEGIN_FLAG_1 0x77

/* Register ID */
// ROM
#define DEVICE_TYPE_ID 0
#define VERSION_ID 1
#define MAC_ID 2
// EEPROM
#define EEPROM_ID 11
#define DEVICE_ID 11
#define BAUDRATE_ID 12
#define OFFSET_ID 13
// RAM 
#define EEPROM_LOCK_ID 28
#define MOTOR_STATUS_ID 29
#define EFFECTOR_ID 30
#define VACUUM_ID 31
#define SPEED_ID 32
#define TIME_ID 39
#define ANGLE_ID 46
#define END_LENGTH_ID 53
#define IK_ID 54
#define IK6_DATA_LENGTH 9
#define IK7_DATA_LENGTH 12
#define IK5_ID 68
#define IK5_DATA_LENGTH 6
#define BTN_ENABLE_ID 66
#define ANGLE_FEEDBACK_FREQ_ID 82
#define ANGLE_FEEDBACK_ID 83
#define LOAD_FEEDBACK_FREQ_ID 90
#define LOAD_FEEDBACK_ID 91


class Arm7Bot : public ServoProtocol {

  public:

    boolean wakeupFlage = false;
    int wakeupBeginTime = 0;

    uint64_t chipID;
    int angleG[SERVO_NUM];        // Unit: degree, [0~180]
    int angleG_pre[SERVO_NUM];    // for angle update check 
    boolean newAngle[SERVO_NUM];  // individual axis angle update flag check 
    int servoPos[SERVO_NUM];    // Unit: 10-bits value, [0~1023]
    

    const boolean reverse[SERVO_NUM] = {false, true, true, true, true, true, false};
    //degree to value, for servo ID(0~3): 850(value) equal 180(degree)  for servo ID(4~6): 625(value) equal 180(degree)
    const float dToV[SERVO_NUM] = {4.722, 4.722, 4.722, 4.722, 3.472, 3.472, 3.472}; // degree to value 
    const float vToD[SERVO_NUM] = {0.212, 0.212, 0.212, 0.212, 0.288, 0.288, 0.288}; // value to degree


    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    /* System Offsets */
    /* Mechanical Structure  (servoPos VS [Geometrical Angle]) */
    // offset = pos - angle*dToV;
    // !!!! actually, angle here means: 90(degree) angle should be 90*dToV, 425 for servo ID(0~3), 312 for servo ID(4~6)
    // Unit: Raw Value
    const int sysOffsets[SERVO_NUM] = {86, 20, 141, 86, 199, 199, 730}; 

    // offsets of induvidual robotic arm, Unit:Degree
    int individualOffsets[SERVO_NUM] = {0, 0, 0, 0, 0, 0, 0}; 

    //standard calibration pose (Structure Angle) Unit: Degree
    int initAngle[SERVO_NUM] = {90,  90,  65,  90,  90,  90, 80};

    int readJointAngle(u8 ID);
    void setJointAngle(u8 ID, int angle);
    int readJointLoad(u8 ID);
    int readJointSpeed(u8 ID);

    void alarm(int level, int type);

    Arm7Bot();
    void init();
    void receiveCommand();
    void vacuum(bool status);   // true: open,  false:close

    // auto feedbacks
    long timeBuf_pose;
    long timeBuf_load;
    void angleFeedback(); // motor pose feedback
    void loadFeedback(); // motor load feedback

    // UART
    bool beginFlag = false;
    bool haveInstruction = false;
    int instruction = 0;
    int cnt = 0;
    int dataBuf[120];
    int rxBuf_pre;

    // CRC
    void sendData(uint8_t *data, int len);
    uint16_t CRC16_MODBUS(uint8_t *data, int len);
    uint8_t InvertUint8( uint8_t dBuf);
    uint16_t InvertUint16( uint16_t dBuf);

    // command register: Verion 2.3
    uint8_t comReg[100];



    /* Kinematics */
    // constrains
    const float MinParallelogramAngle = 19; // 7Bot V2, 2020-05-22 test, Unit: Degree
    const float minAngle = 0.33161256; // radians(19)
    const float thetaMin[SERVO_NUM] = { 0, 0, 0, 0, 0, 0, 0};
    const float thetaMax[SERVO_NUM] = {PI, PI, PI, PI, PI, PI, PI/2};
    
    /*
    7Bot(V2)机械臂几何结构:

    a: 后臂长
    b: 拉杆舵盘长
    c: 前臂长
    d: 2、3号电机出轴与1号电机出轴在Y方向上的偏差
    e: 2、3号电机出轴在Z方向离原点的距离
    f: 5号电机出轴到6号电机舵盘外沿的距离
    g: 6号电机舵盘半径
    h: 肘部偏移距离（后臂与肘部固定螺丝中心到前臂中轴线的投影距离）

    joint[0]: 坐标原点（电机1出轴中心与底座上盖板外平面交点）
    joint[1]: 2、3号电机出轴中轴线中心点
    joint[2]: 拉杆与拉杆舵盘交点到theta[0] = 90°时YZ平面的投影交点！！！
    joint[3]: 后臂与肘部固定螺丝中轴线中心点
    joint[4]: 前臂中轴线在肘部的起始点 (肘部偏移)
    joint[5]: 前臂前端点（到5号电机出轴）
    joint[6]: 执行器前端点， 当endEffectorLength = 0时（当前预设默认），为6号电机舵盘外侧中心点
    joint[7]: 舵盘边缘点
    joint[8]: 舵盘上与7的对称点

    以下描述均在theta[0] = Pi/2 时
    theta[0]: 电机1转角弧度
    theta[1]: 电机2转角弧度， 后臂与Y方向夹角
    theta[2]: 电机3转角弧度， theta[2]-rad65 = 拉杆舵盘与-Y方向夹角
    theta[3]: 电机4转角弧度
    theta[4]: 电机5转角弧度
    theta[5]: 电机6转角弧度
    theta[6]: 电机7转角弧度
    */

   // frame length, Unit:mm
    float a=120.0, b=40.0, c=198.5, d=29.35, e=74.50, f=22.46, h=25.0, g=20;
     float endEffectorLength = 0;  // 用于前端执行器IK6，Joint[6]为执行器前端点。 暂预设为0，不开放任意前端IK6功能。
    const float rad65 = 1.134464;
    int angleRangeCheck();
    PVector joint[9];
    void calcJoints();
    PVector arbitraryRotate(PVector point, PVector pointA, PVector pointB, float _angle);
    PVector zAxiRotate(PVector point, float _angle);
    PVector calcProjectionPt(PVector pt0, PVector pt1, PVector nVec);

    float theta[SERVO_NUM];  // Unit: rad
    int IK5(PVector pt);
    int IK6(PVector j6, PVector vec56_d);
    int IK7(PVector j6, PVector vec56_d, PVector vec67_d);

    bool setIK6 = false;
    bool setIK7 = false;
    bool setIK5 = false;


    // system
    // timer
    unsigned long time_500ms = 0;  // LED blink interval
    unsigned long time_1000ms = 0;  // pose play interval
    // FSM_status3, playCnt
    int playCnt = 1;
    // LED 
    bool ledStatus = false;
    void firmwareSystem();

    // Pose Record
    int poseCnt = 0;   // pose recorded number on EEPROM

    // buzzer
    // Buzzer Tone
    const int notes[8] = {262, 294, 330, 349, 392, 440, 494, 520};   // Set notes C, D, E, F, G, A, B
    void buzzInit();
    void buzzOn(int note);
    void buzzOff();
    // button
    #define BUTTON_NUM 3
    const int button_pin[BUTTON_NUM] = {BUTTON1, BUTTON2, BUTTON3};
    unsigned long time_300ms = 0;   // long button press timer
    bool last_reading[BUTTON_NUM];
    bool reading[BUTTON_NUM];
    unsigned long last_debounce_time[BUTTON_NUM] = {0, 0, 0};
    unsigned long debounce_delay = 50;
    // button state buffers
    bool last_state[BUTTON_NUM];
    // Long press detection
    PressFilter pressFilters[BUTTON_NUM];
    bool btLongPress = false;
    // buzzer delay
    bool shortBuz = false;
    bool longBuz = false;
    unsigned long shortBuzBegin = 0;
    unsigned long longBuzBegin = 0;
    // Press Pressed
    bool btS[BUTTON_NUM];
    bool btL[BUTTON_NUM];
    void btnDetect(); 


    //////////////////////////////////// FSM
    int FSM_Status = 1;
    int pre_FSM_Status = 1;
    bool addPoseFlag = false;
    bool isReleaseFlag = false;
    bool addGrabPoseFlag = false;
    bool addReleasePoseFlag = false;
    bool clearPoseFlag = false;
    int vacuumCupState = 0;  // 0-release, 1-grab 
    void FSMdetector();



};


#endif
