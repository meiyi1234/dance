// TO-DO: Settle case where the frames that need to be sent exceed over 128 (warps back to frame 0). E.g. Frame 127 - 128 & Frame 0.
// TO-DO: Unable to test CircularBuffer since the data provided only happens once now. Not even sure if Task Scheduler works correctly w/o receiving constant flux of values.
// TO-DO: Due to above, REJ S-Frame cannot be tested as well.
#include <CircularBuffer.h>
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
CircularBuffer<char*, 512> IFramesBuffer;
double AcXRx, AcYRx, AcZRx, GyXRx, GyYRx, GyZRx;
double voltRx, currentRx, powerRx, energyRx;
byte numSend = 0;         // The current number of frame sent to RPi,         EXCLUDING H-Frame.
byte numReceive = 0;      // The current number of frames received from RPi,  INCLUDING H-Frame.

QueueHandle_t xQueue0;
SemaphoreHandle_t UninterruptedReadSemaphore = NULL;  // Ensures ReadValues run properly without SendValues running
SemaphoreHandle_t BlockReadSemaphore = NULL;          // Stops ReadValues from continuously running
TickType_t prevWakeTimeRead;
TickType_t prevWakeTimeSend;

const byte START = 0x7e;
const byte STOP = 0x7e;
const byte final2Bits_HFrame = 0x03;
const byte final2Bits_SFrame = 0x01;
const byte SFRAME_REJ = 0x01;
const byte SFRAME_RR = 0x00;

// Returns the checksum number
uint16_t crc16(uint8_t const *buf, int len) {
  /* Sample use
    uint16_t chk = crc16(&buf[1], 2);  // Calculate checksum on bytes at index 1 and 2
    bool is_valid = chk == (buf[3] << 8) | buf[4]
  */
  uint16_t remainder = 0x0000;
  uint16_t poly = 0x1021;
  for (int i = 0; i < len; ++i) {
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

  // Checks control_byte1
  byte check;
  bool isFrameCorrect;
  if (type == 'H')      check = final2Bits_HFrame;
  else if (type == 'S') check = final2Bits_SFrame;

  if (type == 'I')  isFrameCorrect = (buf[2] == 0x00 || buf[2] == 0x02);
  else              isFrameCorrect = (buf[2] == check);
  //

  int checkNum = buf[len - 3] << 8 | buf[len - 2];
  Serial.print("Calc checksum is "); Serial.print(crc16(&buf[1], len - 4)); Serial.print(" vs input checksum is "); Serial.println(checkNum);
  return isFrameCorrect && crc16(&buf[1], len - 4) == checkNum && buf[len - 1] == STOP;
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
          numReceive = 1;

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

// Reads in the gyro & accel values, packets it into an I-Frame and pushes it to CircularBuffer.
void ReadValues(void *pvParameters) {
  unsigned long currentTime;
  double AcX[3], AcY[3], AcZ[3], GyX[3], GyY[3], GyZ[3];
  char AcXChar[5], AcYChar[5], AcZChar[5], GyXChar[5], GyYChar[5], GyZChar[5],
       voltChar[6], currentChar[6], powerChar[6], energyChar[6];
  char iframe[100];                                 // The full I-Frame to be sent
  prevWakeTimeRead = xTaskGetTickCount();
  while (true) {
    if (xSemaphoreTake(BlockReadSemaphore, 0) == pdTRUE) {
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
          Serial.print(sensor_loops); Serial.print(" | ");
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

        iframe[0] = numReceive << 1 | 0b1;
        iframe[1] = numSend << 1 | 0b1;   // prevents iframe[1] to be recognized as terminating string character '\0' or 0x00.
        iframe[2] = '\0';                 // to allow strcat to work properly
        // dtostrf(floatvar, StringLengthIncDecimalPoint, numVarsAfterDecimal, charbuf);
        for (byte sensor_loops = 0; sensor_loops < NUM_GY521; sensor_loops++) {
          dtostrf(AcX[sensor_loops], 0, 0, AcXChar);      // convert double to char[] with no dp, effectively making it int.
          strcat(iframe, AcXChar);
          strcat(iframe, ",");
          memset(AcXChar, NULL, 5);
          dtostrf(AcY[sensor_loops], 0, 0, AcYChar);      // convert double to char[] with no dp, effectively making it int.
          strcat(iframe, AcYChar);
          strcat(iframe, ",");
          memset(AcYChar, NULL, 5);
          dtostrf(AcZ[sensor_loops], 0, 0, AcZChar);      // convert double to char[] with no dp, effectively making it int.
          strcat(iframe, AcZChar);
          strcat(iframe, ",");
          memset(AcZChar, NULL, 5);
          dtostrf(GyX[sensor_loops], 0, 0, GyXChar);      // convert double to char[] with no dp, effectively making it int.
          strcat(iframe, GyXChar);
          strcat(iframe, ",");
          memset(GyXChar, NULL, 5);
          dtostrf(GyY[sensor_loops], 0, 0, GyYChar);      // convert double to char[] with no dp, effectively making it int.
          strcat(iframe, GyYChar);
          strcat(iframe, ",");
          memset(GyYChar, NULL, 5);
          dtostrf(GyZ[sensor_loops], 0, 0, GyZChar);      // convert double to char[] with no dp, effectively making it int.
          strcat(iframe, GyZChar);
          strcat(iframe, ",");
          memset(GyZChar, NULL, 5);
        }
        dtostrf(voltVal, 0, 2, voltChar);      // convert double to char[] with 2dp.
        strcat(iframe, voltChar);
        strcat(iframe, ",");
        dtostrf(currentVal, 0, 2, currentChar);// convert double to char[] with 2dp.
        strcat(iframe, currentChar);
        strcat(iframe, ",");
        dtostrf(powerVal, 0, 2, powerChar);    // convert double to char[] with 2dp.
        strcat(iframe, powerChar);
        strcat(iframe, ",");
        dtostrf(energyVal, 0, 2, energyChar);  // convert double to char[] with 2dp.
        strcat(iframe, energyChar);
        Serial.print("Current IFrame before checksum is "); Serial.println(iframe);
        int len = strlen(iframe);
        Serial.print("Length of I-Frame is "); Serial.println(len);
        iframe[1] &= 0xFE;  // Converts bit[0] back to the original intended 0 for checksum calculation.
        int checksum = crc16(&iframe[0], len);
        iframe[1] |= 0b1;   // Converts bit[0] to 1 again for complete string storage purposes.
        iframe[len++] = checksum >> 8;
        iframe[len] = checksum & 0xFF;
        Serial.print("Checksum is "); Serial.println(checksum);
        Serial.print("Complete IFrame is "); Serial.println(iframe);  // Why is there an elusive character 'E' behind the checksum??
        IFramesBuffer.push(iframe);       // store the I-Frame into the circular buffer.
      }
    }
    vTaskDelayUntil(&prevWakeTimeRead, (4 / portTICK_PERIOD_MS));
    xSemaphoreGive(UninterruptedReadSemaphore);
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
            numReceive = buf[1] >> 1;
            if ((buf[2] >> 2 & 0b11) == SFRAME_REJ) {
              Serial.print("RPi has not received all frames, resending frames "); Serial.print(numReceive); Serial.print(" to "); Serial.println(numSend);
              for (int i = numReceive; i <= numSend; i++) {
                Serial.write(START);
                int len = strlen(IFramesBuffer[i]);
                Serial.write(IFramesBuffer[i][0]);
                Serial.write(IFramesBuffer[i][1] & 0xFE); // Changes the bit[0] from 1 back to 0
                for (int j = 2; j < len - 1; j++) {
                  Serial.write(IFramesBuffer[i][j]);
                }
                Serial.write(STOP);
                Serial.print("Frame number "); Serial.print(i); Serial.print(" = "); Serial.write(IFramesBuffer[i]);
              }
            }
            else if ((buf[2] >> 2 & 0b11) == SFRAME_RR) {
              Serial.print("RPi ready to receive frame number "); Serial.println(numReceive);
              Serial.write(START);
              int len = strlen(IFramesBuffer[numReceive]);
              Serial.write(IFramesBuffer[numReceive][0]);
              Serial.write(IFramesBuffer[numReceive][1] & 0xFE); // Changes the bit[0] from 1 back to 0
              for (int j = 2; j < len - 1; j++) {
                Serial.write(IFramesBuffer[numReceive][j]);
              }
              Serial.write(STOP);
              numSend++;
            }
          }
          else if ( ( (buf[1] & 0b1) == 0b1) && ( (buf[2] & 0b1) == 0b0) && isFrameCorrect(&buf[0], 'I') ) { // is an I-Frame, verify its correct
            char iFrameMsg[50];
            byte i = 0;
            iFrameMsg[i++] = buf[3];
            for (byte j = 4; buf[j] != STOP; j++) {
              iFrameMsg[i++] = buf[j];
            }
            iFrameMsg[i - 2] = '\0';  // terminate the string just after the info, discarding the checksum completely
            Serial.print("The I-Frame Message is " ); Serial.println(iFrameMsg);
            // if(iFrameMsg.equalsIgnoreCase("send me data")) {   // Intepret the message sent in I-Frame
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
        }
        else  Serial.print("byte is "); Serial.write(buf[i]); Serial.println("");
      }
      vTaskDelayUntil(&prevWakeTimeSend, (2 / portTICK_PERIOD_MS));
      xSemaphoreGive(UninterruptedReadSemaphore);
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
  BlockReadSemaphore = xSemaphoreCreateBinary();
  xQueue0 = xQueueCreate(36, sizeof(double));
  xSemaphoreGive(UninterruptedReadSemaphore);
  xSemaphoreGive(BlockReadSemaphore);
  xTaskCreate(ReadValues, "ReadValues", 2000, NULL, 2, NULL);
  xTaskCreate(SendValues, "SendValues", 2500, NULL, 2, NULL);
  Serial.println("Starting Task Scheduler");
  vTaskStartScheduler();
}

void loop() {}
