// BUG: The Arduino starts outputting garbage after running for a while
// BUG: Board hangs after a while, after storing about 10 frames in memory.
#include <Arduino_FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>
#include <Wire.h>

#define PIN_SENSOR_1  5
#define PIN_SENSOR_2  6
#define PIN_SENSOR_3  7
#define VOLT_DIVIDER  A0
#define VOUT          A1

unsigned long startTime;
const byte MPU = 0x68;                    // I2C address of the MPU-6050
const byte NUM_GY521 = 3;                 // Number of sensors

// Global variables below used for I-Frame, global since must keep in memory, a resend is possible.
char IFramesBuffer[16][100];
byte index = 0;
byte numSend = 0;         // The current number of frame sent to RPi,         EXCLUDING H-Frame.
byte numReceive = 0;      // The current number of frames received from RPi,  INCLUDING H-Frame.
byte frameNum = 0;        // The current I-Frame number created in ReadValues

SemaphoreHandle_t UninterruptedReadSemaphore = NULL;  // Ensures ReadValues run properly without SendValues running
TickType_t prevWakeTimeRead;
TickType_t prevWakeTimeSend;

const TickType_t READ_FREQUENCY = 20;   // runs the task ReadValues every READ_FREQUENCY ms
const TickType_t SEND_FREQUENCY = 3;    // runs the task SendValues every SEND_FREQUENCY ms.
                                        // Must run much more frequently than ReadValues since each run of SendValues reads only one byte.
const byte START = 0x7e;
const byte STOP = 0x7e;
const byte final2Bits_HFrame = 0x03;
const byte final2Bits_SFrame = 0x01;
const byte SFRAME_REJ = 0x01;
const byte SFRAME_RR = 0x00;

// Returns the checksum number
uint16_t crc16(uint8_t const *buf, byte len) {
  /* Sample use
    uint16_t chk = crc16(&buf[1], 2);  // Calculate checksum on bytes at index 1 and 2
    bool is_valid = chk == (buf[3] << 8) | buf[4]
  */
  uint16_t remainder = 0x0000;
  uint16_t poly = 0x1021;
  for (byte i = 0; i < len; ++i) {
    remainder ^= (buf[i] << 8);
    for (byte bit = 8; bit > 0; --bit) {
      if (remainder & 0x8000) remainder = (remainder << 1) ^ poly;
      else                    remainder = (remainder << 1);
    }
  }
  return remainder;
}

bool isFrameCorrect(byte* buf, char type) {
  // Calculates the length of buf. Also START byte check.
  byte len;
  if (buf[0] != START)  return false; // does not start with START byte, fail
  else                  len = 1;      // start with the first data byte

  while (buf[len] != STOP && len < 100) { // Arbitary number 100 set to prevent infinite loop in case frame does not terminate properly
    len++;
  }
  len++;                                  // include STOP byte as part of len

  // Checks control_byte2
  byte check;
  bool isFrameCorrect;
  if (type == 'H')      check = final2Bits_HFrame;
  else if (type == 'S') check = final2Bits_SFrame;

  if (type == 'I')  isFrameCorrect = !(buf[2] & 0b1);             // checks whether bit[0] == 0;
  else              isFrameCorrect = ( (buf[2] & 0b11) == check);
  //

  int checkNum = buf[len - 3] << 8 | buf[len - 2];
  //Serial.print("Calc checksum is "); Serial.print(crc16(&buf[1], len - 4)); Serial.print(" vs input checksum is "); Serial.println(checkNum);
  return (buf[1] & 0b1) && isFrameCorrect && crc16(&buf[1], len - 4) == checkNum && buf[len - 1] == STOP;
}

// Handshake between Arduino and RPi
void establishContact() {
  bool handshake = false;
  bool expectStopByte = false;
  // msg[0] = START, msg[1] = numReceive, msg[2] = frame, msg[3] = checkNum, msg[4] = checkNum, msg[5] = STOP
  byte msg[6];
  byte i = -1;           // to fill buf
  while (!handshake) {
    if (Serial.available() > 0) {
      msg[++i] = Serial.read();

      if (i == 0 && msg[i] != START) { // If receiving first byte but is not START byte
        Serial.println("Error, frame doesnt start with 0x7e");
        i = -1;
        expectStopByte = false;
        memset(msg, NULL, 1);
      }
      else if (msg[i] == START && !expectStopByte) {  // If is receiving the START byte
        expectStopByte = true;                        // this is the START byte, the next such byte should be STOP byte
      }
      else if (msg[i] == STOP && expectStopByte) {  // If receiving STOP byte
        Serial.println("Frame terminated, expect receive H-Frame");
        if (!isFrameCorrect(msg, 'H')) {
          Serial.println("H-frame invalid, not doing anything");
          expectStopByte = false;
          memset(msg, NULL, i + 1);
          i = -1;
          continue;
        }
        else {
          // If handshake, repeat message back to primary
          Serial.print("Returning bytes: ");
          Serial.write(msg, i + 1);
          Serial.println("");
          //Serial.write(msg, i + 1);
          Serial.println("Success");
          handshake = true;
          numReceive = (numReceive + 1) & 0x7F;

          memset(msg, NULL, i + 1);
          i = -1;
          expectStopByte = false;
        }
      }
      else {  // receiving the other bytes in between
        Serial.print("byte is ");
        Serial.write(msg[i]);
        Serial.println("");
      }
      delay(10);
    }
  }
}

void sendIFrame(byte i) {
  Serial.print("Frame number "); Serial.print(i); Serial.print(" = ");
  Serial.write(START);
  byte len = strlen(IFramesBuffer[i]);
  Serial.write(IFramesBuffer[i][0]);
  Serial.write(IFramesBuffer[i][1] & 0xFE); // Changes the bit[0] from 1 back to 0
  for (byte j = 2; j < len; j++) {
    Serial.write(IFramesBuffer[i][j]);
  }
  Serial.write(STOP);
}

// Reads in the gyro & accel values, packets it into an I-Frame and pushes it to CircularBuffer.
void ReadValues(void *pvParameters) {
  unsigned long currentTime;
  int AcX[3], AcY[3], AcZ[3], GyX[3], GyY[3], GyZ[3];
  char AcXChar[4], AcYChar[4], AcZChar[4], GyXChar[4], GyYChar[4], GyZChar[4],
       voltChar[6], currentChar[6], powerChar[6], energyChar[6];
  prevWakeTimeRead = xTaskGetTickCount();
  while (true) {
    if (xSemaphoreTake(UninterruptedReadSemaphore, 0) == pdTRUE) {
      for (byte sensor_loops = 0; sensor_loops < NUM_GY521; sensor_loops++) {
        digitalWrite(PIN_SENSOR_1, HIGH);
        digitalWrite(PIN_SENSOR_2, HIGH);
        digitalWrite(PIN_SENSOR_3, HIGH);
        digitalWrite(PIN_SENSOR_1 + sensor_loops, LOW);
        Wire.beginTransmission(MPU);
        Wire.write(0x3B);                             // starting with register 0x3B (ACCEL_XOUT_H)
        Wire.endTransmission(false);
        Wire.requestFrom(MPU, 14, true);              // request a total of 14 registers
        AcX[sensor_loops] = Wire.read() << 8 | Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
        AcY[sensor_loops] = Wire.read() << 8 | Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
        AcZ[sensor_loops] = Wire.read() << 8 | Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
        Wire.read() << 8 | Wire.read();               // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L) - For temperature, UNNECESSARY
        GyX[sensor_loops] = Wire.read() << 8 | Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
        GyY[sensor_loops] = Wire.read() << 8 | Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
        GyZ[sensor_loops] = Wire.read() << 8 | Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
        Wire.endTransmission(true);
        AcX[sensor_loops] = AcY[sensor_loops] = AcZ[sensor_loops] = GyX[sensor_loops] = GyY[sensor_loops] = GyZ[sensor_loops] = 6875 + 10 * sensor_loops; // Dummy data, REMOVE
        Serial.print("Frame No "); Serial.print(sensor_loops); Serial.print(" | ");
        Serial.print("AcX = "); Serial.print(AcX[sensor_loops]);
        Serial.print(" | AcY = "); Serial.print(AcY[sensor_loops]);
        Serial.print(" | AcZ = "); Serial.print(AcZ[sensor_loops]);
        // Serial.print(" | Tmp = "); Serial.print(Tmp / 340.00 + 36.53);  //equation for temperature in degrees C from datasheet
        Serial.print(" | GyX = "); Serial.print(GyX[sensor_loops]);
        Serial.print(" | GyY = "); Serial.print(GyY[sensor_loops]);
        Serial.print(" | GyZ = "); Serial.println(GyZ[sensor_loops]);
      }
      currentTime = millis();
      int voltMeasurement = analogRead(VOLT_DIVIDER);
      int currentMeasurement = analogRead(VOUT);
      double voltVal = (double) voltMeasurement * 5 * 2 / 1023.0;                                         // Volt, voltage divider halves voltage
      double currentVout = (double) currentMeasurement * 5 / 1023.0;                                      // INA169 Vout
      double currentVal = (currentVout * 1000.0) * 1000.0 / (10 * 10000.0);                               // mA, Rs = 10 Ohms Rl = 10k Ohms
      double powerVal = voltVal * currentVal;                                                             // mW
      double energyVal = energyVal + ((powerVal / 1000) * ( ((double)(currentTime - startTime)) / 1000)); // Joules
      Serial.print("Voltage(V) = "); Serial.print(voltVal);
      Serial.print(" | Current(mA) = "); Serial.print(currentVal);
      Serial.print(" | Power(mW) = "); Serial.print(powerVal);
      Serial.print(" | Energy(J) = "); Serial.println(energyVal);
      startTime = currentTime;

      IFramesBuffer[index][0] = numReceive << 1 | 0b1;
      IFramesBuffer[index][1] = frameNum << 1 | 0b1;   // prevents IFramesBuffer[index][1] to be recognized as terminating string character '\0' or 0x00.
      IFramesBuffer[index][2] = '\0';                 // to allow strcat to work properly
      // dtostrf(floatvar, StringLengthIncDecimalPoint, numVarsAfterDecimal, charbuf);
      for (byte sensor_loops = 0; sensor_loops < NUM_GY521; sensor_loops++) {
        itoa(AcX[sensor_loops], AcXChar, 36);      // convert int to char[], with a base of 36 (0-9, a-z)
        strcat(IFramesBuffer[index], AcXChar);
        strcat(IFramesBuffer[index], ",");
        memset(AcXChar, NULL, 4);
        itoa(AcY[sensor_loops], AcYChar, 36);      // convert int to char[], with a base of 36 (0-9, a-z)
        strcat(IFramesBuffer[index], AcYChar);
        strcat(IFramesBuffer[index], ",");
        memset(AcYChar, NULL, 4);
        itoa(AcZ[sensor_loops], AcZChar, 36);      // convert int to char[], with a base of 36 (0-9, a-z)
        strcat(IFramesBuffer[index], AcZChar);
        strcat(IFramesBuffer[index], ",");
        memset(AcZChar, NULL, 4);
        itoa(GyX[sensor_loops], GyXChar, 36);      // convert int to char[], with a base of 36 (0-9, a-z)
        strcat(IFramesBuffer[index], GyXChar);
        strcat(IFramesBuffer[index], ",");
        memset(GyXChar, NULL, 4);
        itoa(GyY[sensor_loops], GyYChar, 36);      // convert int to char[], with a base of 36 (0-9, a-z)
        strcat(IFramesBuffer[index], GyYChar);
        strcat(IFramesBuffer[index], ",");
        memset(GyYChar, NULL, 4);
        itoa(GyZ[sensor_loops], GyZChar, 36);      // convert int to char[], with a base of 36 (0-9, a-z)
        strcat(IFramesBuffer[index], GyZChar);
        strcat(IFramesBuffer[index], ",");
        memset(GyZChar, NULL, 4);
      }
      dtostrf(voltVal, 0, 2, voltChar);      // convert double to char[] with 2dp.
      strcat(IFramesBuffer[index], voltChar);
      strcat(IFramesBuffer[index], ",");
      dtostrf(currentVal, 0, 2, currentChar);// convert double to char[] with 2dp.
      strcat(IFramesBuffer[index], currentChar);
      strcat(IFramesBuffer[index], ",");
      dtostrf(powerVal, 0, 2, powerChar);    // convert double to char[] with 2dp.
      strcat(IFramesBuffer[index], powerChar);
      strcat(IFramesBuffer[index], ",");
      dtostrf(energyVal, 0, 2, energyChar);  // convert double to char[] with 2dp.
      strcat(IFramesBuffer[index], energyChar);
      //Serial.print("Current IFrame before checksum is "); Serial.println(IFramesBuffer[index]);
      byte len = strlen(IFramesBuffer[index]);
      IFramesBuffer[index][1] &= 0xFE;  // Converts bit[0] back to the original intended 0 for checksum calculation.
      int checksum = crc16(&IFramesBuffer[index][0], len);
      IFramesBuffer[index][1] |= 0b1;   // Converts bit[0] to 1 again for complete string storage purposes.
      IFramesBuffer[index][len++] = checksum >> 8;
      IFramesBuffer[index][len++] = checksum & 0xFF;
      IFramesBuffer[index][len] = '\0';
      Serial.print("Length of I-Frame is "); Serial.println(len);
      Serial.print("IFramesBuffer["); Serial.print(index); Serial.print("] is: "); Serial.write(IFramesBuffer[frameNum]);
      frameNum = (frameNum + 1) & 0x7F; // keeps the range of frameNum between 0 - 127
      index = (index + 1) & 0xF;
      xSemaphoreGive(UninterruptedReadSemaphore);
      vTaskDelayUntil(&prevWakeTimeRead, READ_FREQUENCY);
    }
  }
}

// Reads the accelerometer and gyroscope values from the Queue
// Checks for an S-Frame so that Arduino knows that it is ok to send data
void SendValues(void *pvParameters) {
  byte buf[50];               // buf[0] = START, buf[1] = numReceive, buf[2] = frame, buf[3] = checkNum, buf[4] = checkNum, buf[5] = STOP
  bool expectStopByte = false;// Signal when a START byte has occured, so that the next such byte is a STOP byte.
  byte i = -1;                // used to fill buf

  prevWakeTimeSend = xTaskGetTickCount();
  while (true) {
    if (xSemaphoreTake(UninterruptedReadSemaphore, 0) == pdTRUE) {
      if (Serial.available() > 0) {
        buf[++i] = Serial.read();
        if (i == 0 && buf[i] != START) {    // if expecting starting byte but receive otherwise
          Serial.println("Error, Frame does not start with 0x7e");
          i = -1;
          expectStopByte = false;
          memset(buf, NULL, 1);
        }
        else if (buf[i] == START && !expectStopByte) { // START byte received, the next identical byte received will be a STOP byte
          expectStopByte = true;
        }
        else if (buf[i] == STOP && expectStopByte) {  // STOP byte is received, terminate the frame
          Serial.println("Frame terminated");
          if (buf[2] == final2Bits_HFrame && isFrameCorrect(&buf[0], 'H')) {  // is a H-Frame, verify its correct
            Serial.print("Returning bytes: ");
            Serial.write(buf, i + 1);
            Serial.println("");
            //Serial.write(msg, i + 1);
            Serial.println("Success");
          }
          else if ( ( (buf[2] & 0b11) == final2Bits_SFrame) && isFrameCorrect(&buf[0], 'S') ) { // is an S-Frame, verify its correct
            // Check whether need to resend data
            // Trim to only frame[3:2]]. If true, RPi rejected the frame sent by Arduino, must resend
            byte RPiReceive = (buf[1] >> 1) & 0xF;
            numReceive = (numReceive + 1) & 0x7F;     // keeps numReceive between 0 - 127
            if ( ( (buf[2] >> 2) & 0b11) == SFRAME_REJ) {
              Serial.print("RPi has not received all frames, resending frames "); Serial.print(RPiReceive); Serial.print(" to "); Serial.println(numSend);
              if (RPiReceive > numSend) {   // frame number has exceeded 128
                for (byte i = RPiReceive; i < 16; i++) {
                  sendIFrame(i);
                }
                for (byte i = 0; i <= numSend; i++) {
                  sendIFrame(i);
                }
              }
              else {
                for (byte i = RPiReceive; i <= numSend; i++) {
                  sendIFrame(i);
                }
              }
            }
            else if ( ( (buf[2] >> 2) & 0b11) == SFRAME_RR) {
              Serial.print("RPi ready to receive frame number "); Serial.println(RPiReceive);
              sendIFrame(RPiReceive);
              numSend = (numSend + 1) & 0x7F;                   // Keeps send sequence no within 0-127
            }
          }
          else if ( ( (buf[1] & 0b1) == 0b1) && ( (buf[2] & 0b1) == 0b0) && isFrameCorrect(&buf[0], 'I') ) { // is an I-Frame, verify its correct
            numReceive = (numReceive + 1) & 0x7F;     // keeps numReceive between 0 - 127
            char iFrameMsg[50];
            byte i = 0;
            iFrameMsg[i++] = buf[3];
            for (byte j = 4; buf[j + 2] != STOP; j++) { // since checksum is 2 bytes long, followed by STOP byte, stop when hit 1st checksum byte
              iFrameMsg[i++] = buf[j];
            }
            iFrameMsg[i] = '\0';  // terminate the string properly
            Serial.print("The I-Frame Message is: " ); Serial.println(iFrameMsg);
            // if(iFrameMsg.equals("send me data")) {   // Intepret the message sent in I-Frame
            // Send S-Frame back to RPi
            Serial.println("I-Frame received! Sending S-Frame");
            Serial.write(START);
            byte msg[2];
            msg[0] = buf[1];
            msg[1] = 0x01;
            Serial.write(msg[0]);
            Serial.write(msg[1]);
            int checkNum = crc16(&msg[0], 2);
            Serial.write(checkNum);
            Serial.write(STOP);
          }
          memset(buf, NULL, i + 1);
          i = -1;
          expectStopByte = false;
          xSemaphoreGive(UninterruptedReadSemaphore);
        }
        else  Serial.print("byte is "); Serial.write(buf[i]); Serial.println("");
      }
      xSemaphoreGive(UninterruptedReadSemaphore);
      vTaskDelayUntil(&prevWakeTimeSend, SEND_FREQUENCY);
    }
  }
}

void setup() {
  pinMode(PIN_SENSOR_1, OUTPUT);
  pinMode(PIN_SENSOR_2, OUTPUT);
  pinMode(PIN_SENSOR_3, OUTPUT);
  pinMode(VOLT_DIVIDER, INPUT); // Voltage Divider output
  pinMode(VOUT, INPUT); // INA169 VOut
  for (byte sensor_loops = 0; sensor_loops < NUM_GY521; sensor_loops++) {
    digitalWrite(PIN_SENSOR_1, HIGH);
    digitalWrite(PIN_SENSOR_2, HIGH);
    digitalWrite(PIN_SENSOR_3, HIGH);
    digitalWrite(PIN_SENSOR_1 + sensor_loops, LOW);
    Wire.begin();
    Wire.beginTransmission(MPU);
    Wire.write(0x6B); // PWR_MGMT_1 register
    Wire.write(0); // set to zero (wakes up the MPU-6050)
    Wire.endTransmission(true);
    Wire.beginTransmission(MPU);
    Wire.write(0x1C); // ACCEL_CONFIG register
    Wire.write(8); // FS_SEL 01: 4g
    Wire.endTransmission(true);
    Wire.beginTransmission(MPU);
    Wire.write(0x1B); // GYRO_CONFIG register
    Wire.write(16); // FS_SEL 10: 1000 degrees per second
    Wire.endTransmission(true);
  }
  startTime = millis();

  // initialize Serial communication at 9600 bits per second:
  Serial.begin(9600);
  //Serial.begin(9600);
  Serial.println("Start up");
  establishContact();
  Serial.println("Finish contact");
  UninterruptedReadSemaphore = xSemaphoreCreateMutex();
  xSemaphoreGive(UninterruptedReadSemaphore);
  xTaskCreate(ReadValues, "ReadValues", 2000, NULL, 3, NULL);
  xTaskCreate(SendValues, "SendValues", 2500, NULL, 2, NULL);
  Serial.println("Starting Task Scheduler");
  vTaskStartScheduler();
}

void loop() {}
