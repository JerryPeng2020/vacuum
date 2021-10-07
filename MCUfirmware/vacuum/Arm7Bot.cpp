#include "Arm7Bot.h"

/* Constructor */
Arm7Bot::Arm7Bot() {}

// initialize 7Bot
void Arm7Bot::init()
{

  /* Read ROM data to comReg(command register) */
  comReg[DEVICE_TYPE_ID] = DEVICE_TYPE;
  comReg[VERSION_ID] = VERSION;

  // Get chip ID (MAC address), length = 6 bytes
  chipID = ESP.getEfuseMac();
  for (int i = 0; i < 6; i++)
    comReg[MAC_ID + i] = chipID >> (40 - i * 8);

  /* EEPROM init and Read EEPROM data to comReg */
  //
  // indivudual EEPROM: will set every uninitialized byte(0x00) to 0xFF.
  //                    If they these bytes have been set to a value(include 0x00),
  //                    the stored data will not change anymore.
  if (!EEPROM.begin(EEPROM_SIZE))
  { /*  Serial.println("failed to initialise EEPROM"); */
  }
  else
  { /* Serial.println("Successfully initialise EEPROM"); */
  }

  // read EEPROM data to comReg
  for (int i = 0; i < REG_EEPROM_SIZE; i++)
  {
    comReg[EEPROM_ID + i] = EEPROM.read(i);
  }

  // read EEPROM data to pose recorded number
  int tmp = EEPROM.read(RECORD_POSE_NUM_ID);
  if(tmp<255) poseCnt = tmp;   // if tmp == 255, it means the EEPROM addr just init

  // Device Serial Port Init: read EEPROM(1) & setup serial port
  if (comReg[BAUDRATE_ID] == 0)
    Serial.begin(115200);
  else if (comReg[BAUDRATE_ID] == 3)
    Serial.begin(1000000);
  //  else if(buadrateIndex == 7) Serial.begin(9600);
  else
    Serial.begin(115200);

  /*  
  for (int i = 0; i < EEPROM_SIZE; i++)
  {
    Serial.print(comReg[EEPROM_ID + i]); Serial.print(", ");
  }
  Serial.println();
  */

  // read individual offsets from EEPROM via comReg
  for (int i = 0; i < SERVO_NUM; i++)
  {
    int offset = comReg[OFFSET_ID + i];
    if (offset != 255 && offset != 0)
      individualOffsets[i] = offset - 128;
  }

  /* Init comReg RAM data */
  comReg[EEPROM_LOCK_ID] = 1;  // eepromLock, 0-disable EEPROM write protect lock(can write),  1-enable it
  comReg[MOTOR_STATUS_ID] = 1; // motor status: 0-protect, 1-servo, 2-forceless
  comReg[EFFECTOR_ID] = 0;     // endEffectorMode: 0-vacuum, 1-gripper;
  comReg[VACUUM_ID] = 0;       // vacuum status: 0-release, 1-grip;
  for (int i = 0; i < SERVO_NUM; i++)
    comReg[SPEED_ID + i] = INIT_SPEED;
  for (int i = 0; i < SERVO_NUM; i++)
    comReg[TIME_ID + i] = INIT_TIME;
  for (int i = 0; i < SERVO_NUM; i++)
    comReg[ANGLE_ID + i] = initAngle[i];
  for (int i = 0; i < SERVO_NUM; i++)
    comReg[ANGLE_FEEDBACK_ID + i] = 0; // joint angle (read only to master)
  comReg[ANGLE_FEEDBACK_FREQ_ID] = 0;  // angle feedback frequency
  comReg[BTN_ENABLE_ID] = 1;  // function button enable

  /* Init 7Bot Components */
  // init Servo Serial Port
  Serial2.begin(1000000);
  pSerial = &Serial2;

  // init input buttons
  for (int i = 0; i < BUTTON_NUM; i++) {
    pinMode(button_pin[i], INPUT);
    digitalWrite(button_pin[i], HIGH); //Enable the pullup resistor on the button
    last_reading[i] = ! digitalRead(button_pin[i]);
  }
  time_300ms = millis();

  // init buzzer pin
  buzzInit();
  // init LED
  pinMode(LED_BUILTIN, OUTPUT);
  // init pump & valve pins
  pinMode(PUMP, OUTPUT);
  digitalWrite(PUMP, LOW);
  pinMode(VALVE, OUTPUT);
  digitalWrite(VALVE, LOW);

  for (int i = 0; i < SERVO_NUM; i++)
  {
    // setServoTorque(i, 1);
    // setServoTime(i, WAKEUP_TIME);
    setServoSpeed(i, 200);
    setJointAngle(i, initAngle[i]);
  }
  wakeupBeginTime = millis();
}

// scan and receive command from USB-UART port
void Arm7Bot::receiveCommand()
{
  // after 7Bot wakeup, reset its motion speed
  if (!wakeupFlage)
  {
    if (millis() - wakeupBeginTime > WAKEUP_TIME)
    {
      wakeupFlage = true; // 7Bot already waken up
      for (int i = 0; i < SERVO_NUM; i++)
        setServoSpeed(i, INIT_SPEED);
    }
  }

  /*  recive commands（V2.3) */
  while (Serial.available() > 0)
  {
    // read data
    int rxBuf = Serial.read();
    if (!beginFlag)
    {
      beginFlag = (rxBuf_pre == BEGIN_FLAG_0 && rxBuf == BEGIN_FLAG_1) ? true : false; // Beginning Flag
    }
    else
    {
      if (instruction == 0)
        instruction = rxBuf;
      else
      {
        switch (instruction)
        {

        // Read register
        case 0x03:
          dataBuf[cnt++] = rxBuf;
          if (cnt >= 4)
          {
            beginFlag = false;
            instruction = 0;
            cnt = 0;
            uint8_t data_read[] = {0xAA, 0x77, 0x03, dataBuf[0], dataBuf[1]};
            uint16_t crc = CRC16_MODBUS(data_read, 5);
            int high = crc / 256;
            int low = crc % 256;
            int addr = dataBuf[0]; // begin address
            int num = dataBuf[1];  // reg number

            // Serial.write(num);   // num debug

            // CRC check pass, send reg data
            if (low == dataBuf[2] && high == dataBuf[3])
            {
              // prepare data
              uint8_t data_send[5 + num];
              data_send[0] = 0xAA;
              data_send[1] = 0x77;
              data_send[2] = 0x03;
              data_send[3] = addr;
              data_send[4] = num;
              for (int i = 0; i < num; i++)
              {
                // read angle register
                if((addr+i) >= ANGLE_FEEDBACK_ID && (addr+i) < ANGLE_FEEDBACK_ID + SERVO_NUM)
                  comReg[addr+i] = readJointAngle(addr+i-ANGLE_FEEDBACK_ID);
                // read load register
                else if((addr+i) >= LOAD_FEEDBACK_ID && (addr+i) < LOAD_FEEDBACK_ID + SERVO_NUM)
                  comReg[addr+i] = readJointLoad(addr+i-LOAD_FEEDBACK_ID);
                
                // read data
                data_send[5 + i] = comReg[addr + i];
              }

              // send data with CRC inside
              sendData(data_send, num + 5);
            }
          }
          break;

        // write reg
        case 0x04:
          dataBuf[cnt++] = rxBuf;
          // get data length
          if (cnt >= 2)
          {
            if (cnt >= dataBuf[1] + 4)
            {
              beginFlag = false;
              instruction = 0;
              cnt = 0;
              int addr = dataBuf[0];
              int num = dataBuf[1];
              // data read
              uint8_t data_read[5 + num];
              data_read[0] = 0xAA;
              data_read[1] = 0x77;
              data_read[2] = 0x04;
              data_read[3] = addr;
              data_read[4] = num;

              for (int i = 0; i < num; i++)
              {
                data_read[5 + i] = dataBuf[2 + i];
              }
              // CRC check
              uint16_t crc = CRC16_MODBUS(data_read, num + 5);
              int high = crc / 256;
              int low = crc % 256;

              // CRC check pass, write reg
              if (low == dataBuf[num + 2] && high == dataBuf[num + 3])
              {
                // write comReg
                for (int i = addr; i < addr + num; i++)
                {
                  // update writeable addr data
                  if (i >= EEPROM_ID && i <= ANGLE_FEEDBACK_FREQ_ID || i == LOAD_FEEDBACK_FREQ_ID)
                    comReg[i] = dataBuf[2 + i - addr];
                  // immediate execute command
                  if (i == MOTOR_STATUS_ID)
                    for (int j = 0; j < SERVO_NUM; j++)
                      setServoTorque(j, comReg[MOTOR_STATUS_ID]);
                  if (i == VACUUM_ID)
                    vacuum(comReg[VACUUM_ID]);
                  for (int j = 0; j < SERVO_NUM; j++)
                  { // can be optimized
                    if (i == TIME_ID + j)
                      setServoTime(j, comReg[TIME_ID + j] * 100); // Time is less important than speed Setting
                    if (i == SPEED_ID + j)
                      setServoSpeed(j, comReg[SPEED_ID + j] * 10);
                    if (i == ANGLE_ID + j)
                      setJointAngle(j, comReg[ANGLE_ID + j]);
                  }

                  // update set IK flag
                  if(i >= IK_ID && i<IK_ID+IK6_DATA_LENGTH) 
                    setIK6 = true;
                  else if(i>=IK_ID+IK6_DATA_LENGTH && i<IK_ID+IK7_DATA_LENGTH)
                    setIK7 = true;

                  if(i>=IK5_ID && i<IK5_ID+IK5_DATA_LENGTH)
                    setIK5 = true;
                
                }

                // IK5 execute
                if(setIK5) {
                  setIK5 = false;
                  // calculate IK5 parameters
                  PVector j5 = PVector(
                    comReg[IK5_ID+0]*256+comReg[IK5_ID+1] - 1024, 
                    comReg[IK5_ID+2]*256+comReg[IK5_ID+3] - 1024,
                    comReg[IK5_ID+4]*256+comReg[IK5_ID+5] - 1024
                  );
                  // IK5
                  int IK5_status = IK5(j5);
                  if (IK5_status == 0) {
                    for (int i = 0; i < SERVO_NUM-4; i++) {
                      angleG[i] = (int)degrees(theta[i]);
                      setJointAngle(i, angleG[i]);
                    }
                  }
                  else {    
                    // IK5 error
                    // alarm(1, 5);
                  }    
                }


                // IK6 execute
                if(setIK6 && !setIK7) {
                  setIK6 = false;
                  // calculate IK6 parameters
                  PVector j6 = PVector(
                    comReg[IK_ID+0]*256+comReg[IK_ID+1] - 1024, 
                    comReg[IK_ID+2]*256+comReg[IK_ID+3] - 1024,
                    comReg[IK_ID+4]*256+comReg[IK_ID+5] - 1024
                    );
                  PVector vec56 = PVector(
                    comReg[IK_ID+6] - 128,
                    comReg[IK_ID+7] - 128,
                    comReg[IK_ID+8] - 128
                  );
                  // IK6
                  int IK6_status = IK6(j6, vec56);
                  if (IK6_status == 0) {
                    for (int i = 0; i < SERVO_NUM-2; i++) {
                      angleG[i] = (int)degrees(theta[i]);
                      setJointAngle(i, angleG[i]);
                    }

                  }
                  else {    
                    // IK6 error
                    alarm(1, 6);
                  }
                }
                else if(setIK7) {
                  setIK7 = false;
                  // calculate IK6 parameters
                  PVector j6 = PVector(
                    comReg[IK_ID+0]*256+comReg[IK_ID+1] - 1024, 
                    comReg[IK_ID+2]*256+comReg[IK_ID+3] - 1024,
                    comReg[IK_ID+4]*256+comReg[IK_ID+5] - 1024
                    );
                  PVector vec56 = PVector(
                    comReg[IK_ID+6] - 128,
                    comReg[IK_ID+7] - 128,
                    comReg[IK_ID+8] - 128
                  );
                  // calculate IK7 parameters
                  PVector vec67 = PVector(
                    comReg[IK_ID+9] - 128,
                    comReg[IK_ID+10] - 128,
                    comReg[IK_ID+11] - 128
                  );
                  // IK7
                  int IK7_status = IK7(j6, vec56, vec67);
                  if (IK7_status == 0) {
                    for (int i = 0; i < SERVO_NUM-1; i++) {
                      angleG[i] = (int)degrees(theta[i]);
                      setJointAngle(i, angleG[i]);
                    }
                  }
                  else {    
                    // IK7 error
                    alarm(1, 7);
                  }
                }

                // if need write EEPROM, check lock frist and then write EEPROM
                if (comReg[EEPROM_LOCK_ID] == 0)
                {
                  for (int i = EEPROM_ID; i < EEPROM_ID + REG_EEPROM_SIZE; i++)
                    if (i >= addr && i < addr + num)
                      EEPROM.write(i - EEPROM_ID, comReg[i]);
                }
                EEPROM.commit();
              }
            }
          }
          break;

        default:
          beginFlag = false;
          instruction = 0;
          cnt = 0;
          break;
        }
      }
    }

    rxBuf_pre = rxBuf;
  }
}


// 7Bot alarm: send alarm massage through UART & buzzer on for a while
void Arm7Bot::alarm(int level, int type) {
  // send alarm massage
  uint8_t data_send[5];
  data_send[0] = 0xAA;
  data_send[1] = 0x77;
  data_send[2] = 0x08;
  data_send[3] = level;
  data_send[4] = type;
  sendData(data_send, 5);
  // 
  buzzOn(2);   
  delay(100);
  buzzOff(); 
}

// send feedback to USB-UART port
void Arm7Bot::angleFeedback()
{
  // prepare data
  uint8_t data_send[5 + SERVO_NUM];
  data_send[0] = 0xAA;
  data_send[1] = 0x77;
  data_send[2] = 0x05;
  data_send[3] = ANGLE_FEEDBACK_ID;
  data_send[4] = SERVO_NUM;

  for (int i = 0; i < SERVO_NUM; i++)
  {
    comReg[ANGLE_FEEDBACK_ID + i] = readJointAngle(i);
    data_send[5 + i] = comReg[ANGLE_FEEDBACK_ID + i];
  }
  sendData(data_send, SERVO_NUM + 5);
}


// send load feedback to USB-UART port
void Arm7Bot::loadFeedback()
{
  // prepare data
  uint8_t data_send[5 + SERVO_NUM];
  data_send[0] = 0xAA;
  data_send[1] = 0x77;
  data_send[2] = 0x05;
  data_send[3] = LOAD_FEEDBACK_ID;
  data_send[4] = SERVO_NUM;

  for (int i = 0; i < SERVO_NUM; i++)
  {
    comReg[LOAD_FEEDBACK_ID + i] = readJointLoad(i);
    data_send[5 + i] = comReg[LOAD_FEEDBACK_ID + i];
  }
  sendData(data_send, SERVO_NUM + 5);
}

// vacuum control
void Arm7Bot::vacuum(bool status)
{
  if (status)
  {
    digitalWrite(PUMP, HIGH);
    digitalWrite(VALVE, HIGH);
  }
  else
  {
    digitalWrite(PUMP, LOW);
    digitalWrite(VALVE, LOW);  // 2021-9-21: disable valve
  }
}

// read joint angle (after offset calibration with range 0~180)
int Arm7Bot::readJointAngle(u8 ID)
{
  int angleRead;
  int valueRead = readServoPos(ID);                                                         // read potentiometer 10 bits raw data
  float angleTmp = float(valueRead - sysOffsets[ID]) * vToD[ID] + float(individualOffsets[ID]); // convert raw data to angular data
    // range limit to 0~180, can be extend a little bit later!!！
    if (angleTmp < 0)
      angleTmp = 0;
    else if (angleTmp > 180)
      angleTmp = 180;
    if (reverse[ID])
      angleRead = 180 - round(angleTmp);
    else
      angleRead = round(angleTmp);
    return angleRead;
}

// Joint angle set, involve in servo offsets compensation
// ID: 0~6
void Arm7Bot::setJointAngle(u8 ID, int angle)
{
  newAngle[ID] = false; // valid command detection
  angleG[ID] = angle;
  if (angleG_pre[ID] != angleG[ID])
    newAngle[ID] = true;
  angleG_pre[ID] = angleG[ID];
  // only sent command to servo when new angle received
  if (newAngle[ID])
  {
    float angleTmp = float(angleG[ID]);
    if (reverse[ID])
      angleTmp = 180.0 - float(angleG[ID]);
    servoPos[ID] = round(angleTmp * dToV[ID] + float(sysOffsets[ID]) - float(individualOffsets[ID]) * dToV[ID]); // Unit: value
    if (servoPos[ID] < 0)
      servoPos[ID] = 0;
    else if (servoPos[ID] > 1023)
      servoPos[ID] = 1023;
    setServoPos(ID, servoPos[ID]);
  }
}

// read joint load (after offset calibration with range 0~255)
int Arm7Bot::readJointLoad(u8 ID)
{
  int loadRead;
  int valueRead = readServoLoad(ID);  // read potentiometer 10 bits raw data, >1023 means minus
  if(valueRead > 1023) valueRead = 1023 - valueRead;

  // re-range 10bit raw data to 8bit signed data
  loadRead = valueRead/8 + 128;
  return loadRead;
}

// read joint Speed (after offset calibration with range 0~255)
int Arm7Bot::readJointSpeed(u8 ID)
{
  int speedRead;
  int valueRead = readServoSpeed(ID);  // read potentiometer 10 bits raw data, >1023 means minus
  // if(valueRead > 1023) valueRead = 1023 - valueRead;

  // // re-range 10bit raw data to 8bit signed data
  // loadRead = valueRead/8 + 128;
  // return loadRead;
}

// Be care!!!! sizeof(data) do not work properly, maybe because of
// the uint8_t data is not treat correctly in sizeof().
void Arm7Bot::sendData(uint8_t *data, int len)
{
  uint16_t crc = CRC16_MODBUS(data, len);
  int high = crc / 256;
  int low = crc % 256;
  for (int i = 0; i < len; i++)
    Serial.write(data[i]);
  Serial.write(low);
  Serial.write(high);
}

uint16_t Arm7Bot::CRC16_MODBUS(uint8_t *data, int len)
{
  uint16_t wCRCin = 0xFFFF;
  uint16_t wCPoly = 0x8005;
  uint8_t wbyte = 0;
  int j = 0;
  while (len > 0)
  {
    len--;
    wbyte = data[j++];
    wbyte = InvertUint8(wbyte);
    wCRCin ^= (wbyte << 8);
    for (int i = 0; i < 8; i++)
    {
      if ((wCRCin & 0x8000) != 0)
        wCRCin = uint16_t((wCRCin << 1) ^ wCPoly);
      else
        wCRCin = uint16_t(wCRCin << 1);
    }
  }
  wCRCin = InvertUint16(wCRCin);
  return (wCRCin);
}

uint8_t Arm7Bot::InvertUint8(uint8_t dBuf)
{
  int i;
  uint8_t tmp = 0;
  for (i = 0; i < 8; i++)
  {
    if ((dBuf & (1 << i)) != 0)
      tmp |= 1 << (7 - i);
  }
  return tmp;
}

uint16_t Arm7Bot::InvertUint16(uint16_t dBuf)
{
  int i;
  uint16_t tmp;
  tmp = 0;
  for (i = 0; i < 16; i++)
  {
    if ((dBuf & (1 << i)) != 0)
      tmp |= 1 << (15 - i);
  }
  return tmp;
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* Kinematics */

// Valid range check
// return: [0]-no error, [1]-some angle out of range, [2]-theta1 & theta2 cosntrain
int Arm7Bot::angleRangeCheck() {
  for (int i = 0; i < SERVO_NUM; i++) {
    if (theta[i] < thetaMin[i] || theta[i] > thetaMax[i]) return 1;
  }
  if (theta[2]-rad65+theta[1]+minAngle>PI || theta[2]-rad65+theta[1]<minAngle)  return 2;
  return 0;
}


// Calculate model joints: calculate jointTmp[0]~[8] from valid thetaTmp[0]~[5]
void Arm7Bot::calcJoints() {
  joint[1] = PVector(joint[0].x, joint[0].y + d, joint[0].z + e);
  joint[2] = PVector(0, -b * cos(theta[2] - rad65), b * sin(theta[2] - rad65));    joint[2].add(joint[1]);
  joint[3] = PVector(0,  a * cos(theta[1]), a * sin(theta[1]));     joint[3].add(joint[1]);
  joint[4] = PVector(0,  h * sin(theta[2] - rad65),  h * cos(theta[2] - rad65));     joint[4].add(joint[3]);
  joint[5] = PVector(0,  c * cos(theta[2] - rad65), -c * sin(theta[2] - rad65));    joint[5].add(joint[4]);
  joint[6] = PVector(0,  (f+endEffectorLength) * sin(theta[2] - rad65 + theta[4]), (f+endEffectorLength) * cos(theta[2] - rad65 + theta[4]));   joint[6].add(joint[5]);
  joint[7] = PVector(0, -g * cos(theta[2] - rad65 + theta[4]), g * sin(theta[2] - rad65 + theta[4]));   joint[7].add(joint[6]);
  joint[7] = arbitraryRotate(joint[7], joint[6], joint[5], theta[5]); 
  joint[6] = arbitraryRotate(joint[6], joint[5], joint[4], theta[3] - HALF_PI); 
  joint[7] = arbitraryRotate(joint[7], joint[5], joint[4], theta[3] - HALF_PI); 
  joint[8] = PVector(2 * joint[6].x - joint[7].x, 2 * joint[6].y - joint[7].y, 2 * joint[6].z - joint[7].z);
  for (int i = 1; i < 9; i++) {
    joint[i] = zAxiRotate(joint[i], theta[0] - HALF_PI);
  }
}

  ///////////////////////////////////////////////////////////////////////////
  /* Calculation in 3D-space  */
  ///////////////////////////////////////////////////////////////////////////  
  // Function: calculte point rotate along Z-axi
PVector Arm7Bot::zAxiRotate(PVector point, float _angle) {
  PVector pt;
  pt = PVector( cos(_angle) * point.x - sin(_angle) * point.y, sin(_angle) * point.x + cos(_angle) * point.y, point.z );
  return pt;
}

  // Function: calculte point rotate along arbitrary axis
  // point:  input 3D point, which need to be rotated 
  // pointA: beginning point on roration axis
  // pointB: terminal point on ratation axis
  // angle:  rotation angle
PVector Arm7Bot::arbitraryRotate(PVector point, PVector pointA, PVector pointB, float _angle) {
  PVector pt = PVector(0, 0, 0);
  float x = point.x, y = point.y, z = point.z;
  float u = pointB.x - pointA.x, v = pointB.y - pointA.y, w = pointB.z - pointA.z;
  float l = sqrt(u * u + v * v + w * w);
  u /= l; v /= l; w /= l;
  float a = pointA.x, b = pointA.y, c = pointA.z;
  float u2 = u * u, v2 = v * v, w2 = w * w;
  float au = a * u, av = a * v, aw = a * w;
  float bu = b * u, bv = b * v, bw = b * w;
  float cu = c * u, cv = c * v, cw = c * w;
  float ux = u * x, uy = u * y, uz = u * z;
  float vx = v * x, vy = v * y, vz = v * z;
  float wx = w * x, wy = w * y, wz = w * z;
  pt.x = (a * (v2 + w2) - u * (bv + cw - ux - vy - wz)) * (1 - cos(_angle)) + x * cos(_angle) + (-cv + bw - wy + vz) * sin(_angle);
  pt.y = (b * (u2 + w2) - v * (au + cw - ux - vy - wz)) * (1 - cos(_angle)) + y * cos(_angle) + (cu - aw + wx - uz) * sin(_angle);
  pt.z = (c * (u2 + v2) - w * (au + bv - ux - vy - wz)) * (1 - cos(_angle)) + z * cos(_angle) + (-bu + av - vx + uy) * sin(_angle);
  return pt;
}

  // Function: calculte point rotate along arbitrary axis
  // pt0: the point which need to calculate its projection
  // pt1: an point on the plane
  // n: normal vector of the plane
PVector Arm7Bot::calcProjectionPt(PVector pt0, PVector pt1, PVector nVec) {
  PVector n = PVector(nVec.x, nVec.y, nVec.z);
  n.normalize();
  PVector vec10 = PVector(pt0.x - pt1.x, pt0.y - pt1.y, pt0.z - pt1.z);
  float dot = vec10.dot(n);
  PVector projectionPt = PVector(pt0.x - dot * n.x, pt0.y - dot * n.y, pt0.z - dot * n.z);
  return projectionPt;
}




//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* Invers Kinematics */

// Function: IK, input Joint[5], calculate theta[0]~[2]
// return: [0]no error; [1]out of valid range
// input unit:mm
int Arm7Bot::IK5(PVector pt) {
  float x = pt.x, y = pt.y, z = pt.z;
  int status = 1;
  theta[0] = atan(y / x); 
  if (theta[0] < 0) theta[0] = PI + theta[0];  // convert to 0~180(degree) range
  // !!! convert to joint[1] as original point coordinate system 
  x -= d * cos(theta[0]);
  y -= d * sin(theta[0]);
  z -= e;
  // elbow offset compensate
  float lengthA = sqrt(x * x + y * y + z * z);   // the opposite line of angle A
  float lengthC = sqrt(h * h + c * c);           // the opposite line of angle C
  float offsetAngle = atan(h / c);
  // calculate theta[0~2],some parameters can be pre-calculated
  float angleA = acos( (a * a + lengthC * lengthC - lengthA * lengthA) / (2 * a * lengthC) );
  float angleB = atan( z / sqrt(x * x + y * y) );
  float angleC = acos( (a * a + lengthA * lengthA - lengthC * lengthC) / (2 * a * lengthA) );
  theta[1] = angleB + angleC;
  theta[2] = PI - angleA - angleB - angleC + offsetAngle;
  theta[2] += rad65;  // 65 degrees offsets

  // range check
  if (theta[1] > thetaMin[1] && theta[1] < thetaMax[1] &&
      theta[2] > thetaMin[2] && theta[2] < thetaMax[2] &&
      theta[2]-rad65+theta[1]+minAngle<PI && theta[2]-rad65+theta[1]>minAngle) {
    status = 0;
  }
  return status;
}

// Function: IK, input joint[6] & Vector(joint[5] to joint[6] direction), calculate theta[0]~[4]
// return: [0]no error; [1]IK3 out of valid range; [2]IK5-theta4 out of range
int Arm7Bot::IK6(PVector j6, PVector vec56_d) {
  int status = -1;
  // 1- calculate joint[5](temp)
  PVector vec56_u =  PVector(vec56_d.x, vec56_d.y, vec56_d.z); // cannot use vec56_u = vec56_d notation!!! this notation is bi-direction
  vec56_u.normalize();
  PVector j5 =  PVector(j6.x - (f+endEffectorLength) * vec56_u.x, j6.y - (f+endEffectorLength) * vec56_u.y, j6.z - (f+endEffectorLength) * vec56_u.z);
  PVector vec56 =  PVector(j6.x - j5.x, j6.y - j5.y, j6.z - j5.z);
  
  // 2- calculate theta[0]~[2]
  int IK5_status = IK5(j5);
  if (IK5_status != 0) return IK5_status;
  joint[5] = j5;

  // 3- calculate theta[3]
  //(1)- calculate j6_0: position of joint[6] when theta[3]=theta[4]=0;
  theta[3] = 0.; theta[4] = 0.;
  calcJoints();
  PVector j6_0 = joint[6];
  PVector vec56_0 =  PVector(j6_0.x - j5.x, j6_0.y - j5.y, j6_0.z - j5.z);
  //(2)- caculate j6p: projection point of joint[6] on plane which perpendicular with vec45 and intersect at joint[5]
  PVector vec45 =  PVector(joint[5].x - joint[4].x, joint[5].y - joint[4].y, joint[5].z - joint[4].z);
  PVector j6p = calcProjectionPt(j6, j5, vec45);
  PVector vec56p =  PVector(j6p.x - j5.x, j6p.y - j5.y, j6p.z - j5.z);
  //(3)- calculate theta[3]
  theta[3] = acos( vec56_0.dot(vec56p) / (j5.dist(j6_0) * j5.dist(j6p)) );
  
  // 4- calculate theta[4]
  //(1)- calculate theta[4]
  theta[4] = acos( vec56.dot(vec56p) / (j5.dist(j6) * j5.dist(j6p)) );
  //(2)- calculate joint[6] from theta[0]~[4]
  calcJoints();
   //(3)- check
  float dist = j6.dist(joint[6]);
  if (dist < 1) {  // rounding error threshold: 1mm(big enough)  
    return 0;
  }
  theta[3] = PI - theta[3];  // rotate to the other side
  theta[4] = PI - theta[4];  // rotate to the other side
  calcJoints();
  dist = j6.dist(joint[6]);
  if (dist < 1) {  // rounding error threshold: 1mm(big enough) 
    return 0;
  }
  else {
    return 2;
  }
}

// Function: IK, input joint[6], Vector(joint[5] to joint[6] direction) & Vector(joint[6] to joint[7]), calculate theta[0]~[5]
// return: [0]no error; [1]IK3 out of valid range; [2]IK5-theta4 out of range;
int Arm7Bot::IK7(PVector j6, PVector vec56_d, PVector vec67_d) {
  int status = -1;
  // 1- calculate theta[0]~[4]
  int IK6_status = IK6(j6, vec56_d);
  if (IK6_status != 0) return IK6_status;

  // 2- calculate j7p
  PVector vec67_u =  PVector(vec67_d.x, vec67_d.y, vec67_d.z);
  vec67_u.normalize();
  PVector j7 =  PVector(j6.x + g * vec67_u.x, j6.y + g * vec67_u.y, j6.z + g * vec67_u.z);
  //strokeWeight(5); stroke(0,255,0);  drawLine(j7, j6);
  // if vec67_d is not perpendicular with vec56_d, set vec67_d as its projection on the plane,
  // which is perpendicular to vec56 and intersect at joint[6]
  PVector j7p = calcProjectionPt(j7, j6, vec56_d);
  //strokeWeight(5); stroke(0,200,0);  drawLine(j7p, j6);

  // 3- calculate theta[5]
  //(1)- calculate j7_0: position of joint[7] when theta[5]=90degree;
  theta[5] = 0; 
  calcJoints();
  PVector j7_0 = joint[7];
  //strokeWeight(5); stroke(255,0,255);  drawLine(j7_0, j6);
  //(2)- calculate two vectors
  PVector vec67_0 =  PVector(j7_0.x - j6.x, j7_0.y - j6.y, j7_0.z - j6.z);
  PVector vec67p =  PVector(j7p.x - j6.x, j7p.y - j6.y, j7p.z - j6.z);
  //(3)- calculate theta[5]
  float thetaTmp5 = acos( vec67_0.dot(vec67p) / (j6.dist(j7_0) * j6.dist(j7p)) );
  theta[5] = -thetaTmp5;
  if (vec67_d.x < 0) theta[5] = -theta[5];
  if (theta[5] < 0) theta[5] = PI + theta[5];  // symmetry operater, used for symetrical front end, as claw
  calcJoints();
  return 0;
}





// Function: firmware
void Arm7Bot::firmwareSystem() {

  if(comReg[BTN_ENABLE_ID]==1) btnDetect();    // button
  if(comReg[BTN_ENABLE_ID]==1) FSMdetector();  // FSM
  else if(comReg[BTN_ENABLE_ID]==0) FSM_Status = 1;

  // FSM-1: Receive and execute serial command
  if (FSM_Status == 1) {
    // LED on (Status Indicater)
    digitalWrite(LED_BUILTIN, LOW); 

    // 1. monitor serial port
    receiveCommand();
    // 2. send auto feedback datas
    if (comReg[ANGLE_FEEDBACK_FREQ_ID] != 0)
      if (millis() - timeBuf_pose > 1000/comReg[ANGLE_FEEDBACK_FREQ_ID]) {
        timeBuf_pose = millis();
        angleFeedback();
      }
    if (comReg[LOAD_FEEDBACK_FREQ_ID] != 0)
      if (millis() - timeBuf_load > 1000/comReg[LOAD_FEEDBACK_FREQ_ID]) {
        timeBuf_load = millis();
        loadFeedback();
      }
  }
  else if (FSM_Status == 2) {
    // LED blink (Status Indicater)
    if (millis() - time_500ms >= 500)  {
      time_500ms = millis();
      digitalWrite(LED_BUILTIN, !ledStatus); 
      ledStatus = !ledStatus;
    }

    // set robot joints to forceless mode
    if (pre_FSM_Status != 2) 
      for(int i=0; i<SERVO_NUM; i++)
        setServoTorque(i, 2);

    // add a normal pose
    if (addPoseFlag) {
      addPoseFlag = false;
      // read pose 
      int storeData[SERVO_NUM];
      for(int i=0; i<SERVO_NUM; i++) storeData[i] = readJointAngle(i);
      // for vacuum cup
      if (vacuumCupState == 1)  storeData[6] = 0;
      else storeData[6] = 80;
      // count pose number(MaxNum = 254) & update
      if (poseCnt < MAX_POSE_NUM) poseCnt++; Serial.print("AddRecPose: "); Serial.println(poseCnt);
      EEPROM.write(RECORD_POSE_NUM_ID, (uint8_t)poseCnt);
      // store pose data to EEPROM
      for (int i = 0; i < SERVO_NUM; i++)
        EEPROM.write(RECORD_POSE_DATA_ID + poseCnt * SERVO_NUM + i, storeData[i]);
      EEPROM.commit();
    } // END- add a normal pose

    // add a grab pose
    if (addGrabPoseFlag) {
      addGrabPoseFlag = false;
      // read pose 
      int storeData[SERVO_NUM];
      for(int i=0; i<SERVO_NUM; i++) storeData[i] = readJointAngle(i);
      // close claw  & vacuum on
      setServoTorque(6, 1);
      setJointAngle(6, 15);
      vacuum(true);
      // 
      storeData[6] = 15;
      // count pose number(MaxNum = 254) & update
      if (poseCnt < MAX_POSE_NUM) poseCnt++; Serial.print("AddRecPose: "); Serial.println(poseCnt);
      EEPROM.write(RECORD_POSE_NUM_ID, (uint8_t)poseCnt);
      // store pose data to EEPROM
      for (int i = 0; i < SERVO_NUM; i++)
        EEPROM.write(RECORD_POSE_DATA_ID + poseCnt * SERVO_NUM + i, storeData[i]);
      EEPROM.commit();
      // set vacuum status
      vacuumCupState = 1;
    } // END- add a grab pose

    // add a release pose
    if (addReleasePoseFlag) {
      addReleasePoseFlag = false;
      // read pose 
      int storeData[SERVO_NUM];
      for(int i=0; i<SERVO_NUM; i++) storeData[i] = readJointAngle(i);
      // open claw  & vacuum off
      setServoTorque(6, 1);
      setJointAngle(6, 80);
      vacuum(false);
      //
      storeData[6] = 80;
      // count pose number(MaxNum = 254) & update
      if (poseCnt < MAX_POSE_NUM) poseCnt++; Serial.print("AddRecPose: "); Serial.println(poseCnt);
      EEPROM.write(RECORD_POSE_NUM_ID, (uint8_t)poseCnt);
      // store pose data to EEPROM
      for (int i = 0; i < SERVO_NUM; i++)
        EEPROM.write(RECORD_POSE_DATA_ID + poseCnt * SERVO_NUM + i, storeData[i]);
      EEPROM.commit();
      // set vacuum status 
      vacuumCupState = 0;
    } // END- add a release pose

    // clear the poses
    if (clearPoseFlag) {
      clearPoseFlag = false; Serial.println("Clear Poses");
      isReleaseFlag = false;
      poseCnt = 0;
      EEPROM.write(RECORD_POSE_NUM_ID, (uint8_t)poseCnt);
      EEPROM.commit();
    }
  }
  // FSM-3: Execute the record poses
  else if (FSM_Status == 3) {
    // LED on (Status Indicater)
    digitalWrite(LED_BUILTIN, HIGH); 
    

    // set robot joints to normal mode
    if (pre_FSM_Status != 3) {
      for(int i=0; i<SERVO_NUM; i++)
        setServoTorque(i, 1);
    }

    // move one pose per second
    if (millis() - time_1000ms >= 1000)  {
      time_1000ms = millis();
      //
      if (poseCnt != 0) {
        if (playCnt > poseCnt) playCnt = 1; 
        Serial.print("PlayPose: "); Serial.println(playCnt);
        //read stored datas
        int storeData[SERVO_NUM];
        for (int i = 0; i < SERVO_NUM; i++) 
          storeData[i] = (int)EEPROM.read(RECORD_POSE_DATA_ID + playCnt * SERVO_NUM + i);

        // set joints
        for (int i = 0; i < SERVO_NUM; i++) {
          setJointAngle(i, storeData[i]);
        }
        if(storeData[6]<40) vacuum(true);
        else vacuum(false);
        playCnt++;
      }
    }  // END- 1000ms

  }  // END_FSM-status3
  // If external forces too high for a while: FSM_Status = 4;
  // This protection mechanism should be test for a while... 
  else if (FSM_Status == 4) {
    // set robot joints to protection
    if (pre_FSM_Status != 4)
      for(int i=0; i<SERVO_NUM; i++)
        setServoTorque(i, 0);

  }

  pre_FSM_Status = FSM_Status;
}



/* Buzzer */
// init buzzer
void Arm7Bot::buzzInit() {
  ledcSetup(0, notes[0], 8);  // channel, freq, resolution
  ledcAttachPin(BUZZER, 0);          // pin, channel
}
// turn on buzzer with note (0~7)
void Arm7Bot::buzzOn(int note) {
  ledcWriteTone(0, notes[note]); // Buzzer on: channel, freq
}
// turn off buzzer
void Arm7Bot::buzzOff() {
  ledcWriteTone(0, 0);
}

/* Press button detection */
void Arm7Bot::btnDetect() {
  // 1. short button press detection 
  for (int i = 0; i < BUTTON_NUM; i++) {
    reading[i] = ! digitalRead(button_pin[i]);
    if (last_reading[i] != reading[i]) last_debounce_time[i] = millis();
    if ( (millis() - last_debounce_time[i])  > debounce_delay) {
      bool state = reading[i];
      if (last_state[i] && !state) { // when btn release
        if (btLongPress) btLongPress = false;
        else {
          shortBuz = true;
          shortBuzBegin = millis();
          if(i>0) buzzOn(i); //btnS[0] will not buzz
          // button[i] short pressed and release once
          btS[i] = true;
        }
      }
      last_state[i] = state;
    }
    last_reading[i] = reading[i];
  }
  // 2. long button press(1.8s) detection
  if ( (millis() - time_300ms >= 300) && !btLongPress)  {
    time_300ms = millis();

    for (int i = 0; i < BUTTON_NUM; i++) {
      int press2s = pressFilters[i].filter(digitalRead(button_pin[i]));
      if (press2s == 0) {
        btL[i] = true;
        btLongPress = true;
        longBuz = true;
        longBuzBegin = millis();
        buzzOn(i); 
      }
    }
  }
  // 3. Button Press Buzzer
  // short buzzer close
  if ( (millis() - shortBuzBegin >= 100) && shortBuz )  {
    shortBuz = false;
    buzzOff();
  }
  // long buzzer close
  if ( (millis() - longBuzBegin >= 1000) && longBuz)  {
    longBuz = false;
    buzzOff();
  }

}


// Firmware FSM Function.
// Status: 1- Serial , 2- Record Pose, 3- Play Pose.
void Arm7Bot::FSMdetector() {

  // restart ESP32 
  if(btL[0]){
    btL[0] = false;
    ESP.restart();
  }


  if (btS[1]) {
    btS[1] = false;
    vacuum(true);
  }

  if (btS[2]) {
    btS[2] = false;
    vacuum(false);
  }

} 
