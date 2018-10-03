#include <Wire.h>

#include <Arduino_FreeRTOS.h>
#include <semphr.h>

#define START_STOP_BYTE 0x7E
#define ESCAPE_BYTE 0x7D
#define MPU 0x68  // I2C address of the MPU-6050
#define NUM_GY521 3
#define SFRAME_REJ 0x01
#define SFRAME_RR 0x00

const byte final2Bits_HFrame = 0x03;
const byte final2Bits_SFrame = 0x01;

typedef enum Frame { IFrame=0, SFrame=1, HFrame=3 } Frame;

unsigned long startTime;
uint8_t send_seq = 1, recv_seq = 0;
char send_buf[8][128];
uint8_t buf_idx = 0;

// FreeRTOS data structures
QueueHandle_t xSerialSendQueue;
SemaphoreHandle_t xSerialSemaphore;  // Allow only one task to access the serial port


void setup() {
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(A0, INPUT); // Voltage Divider output
  pinMode(A2, INPUT); // INA169 VOut

  Serial.begin(115200);    // pc
  Serial3.begin(115200);   // rpi
  Serial.println("Ready");

  xSerialSendQueue = xQueueCreate(16, sizeof(uint8_t));

  xSerialSemaphore = xSemaphoreCreateMutex();
  xSemaphoreGive(xSerialSemaphore);

  establishContact();

  setupSensors();

  // Highest priority
  xTaskCreate(TaskReadSensors, "Read sensors", 512, NULL, 4, NULL);
  xTaskCreate(TaskSend, "Serial send", 512, NULL, 3, NULL);
  // Lowest priority since mostly ack messages will be sent by primary
  xTaskCreate(TaskRecv, "Serial recv", 512, NULL, 2, NULL);

  Serial.println("Starting scheduler");
  vTaskStartScheduler();
}

// Handshake between Arduino and RPi
void establishContact() {
  bool handshake = false;
  bool expectStopByte = false;
  // msg[0] = START, msg[1] = recv_seq, msg[2] = frame, msg[3] = checkNum, msg[4] = checkNum, msg[5] = START_STOP_BYTE
  byte msg[6];
  byte i = -1;           // to fill buf
  while (!handshake) {
    if (Serial3.available() > 0) {
      msg[++i] = Serial3.read();

      if (i == 0 && msg[i] != START_STOP_BYTE) { // If receiving first byte but is not START byte
        Serial.println("Error, frame doesnt start with 0x7e");
        i = -1;
        expectStopByte = false;
        memset(msg, NULL, 1);
      }
      else if (msg[i] == START_STOP_BYTE && !expectStopByte) {  // If is receiving the START byte
        expectStopByte = true;                        // this is the START byte, the next such byte should be START_STOP_BYTE byte
      }
      else if (msg[i] == START_STOP_BYTE && expectStopByte) {  // If receiving START_STOP_BYTE byte
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
          Serial3.write(msg, i + 1);
          Serial.println("Success");
          handshake = true;
          recv_seq = (recv_seq + 1) & 0x7F;

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

void setupSensors() {
  for (int i=0; i<NUM_GY521; i++) {
    digitalWrite(5, HIGH);
    digitalWrite(6, HIGH);
    digitalWrite(7, HIGH);
    digitalWrite(5 + i, LOW);
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
}

void TaskReadSensors(void *pvParameters) {
  uint16_t buf_len, checksum;
  int AcX[NUM_GY521], AcY[NUM_GY521], AcZ[NUM_GY521];
  int GyX[NUM_GY521], GyY[NUM_GY521], GyZ[NUM_GY521];
  int voltMeasurement, currentMeasurement;
  float voltVal, currentVout, currentVal, powerVal, energyVal = 0;
  char voltStr[8], currentStr[8], powerStr[8], energyStr[8];
  unsigned long currentTime, timeDelta;  // startTime defined globallly

  TickType_t prevWakeTime = xTaskGetTickCount();
  const TickType_t xDelay = 1500 / portTICK_PERIOD_MS;  // 10ms

  for (;;) {
    for (uint8_t i=0; i<NUM_GY521; i++) {
      digitalWrite(5, HIGH);
      digitalWrite(6, HIGH);
      digitalWrite(7, HIGH);
      digitalWrite(5 + i, LOW);

      Wire.beginTransmission(MPU);
      Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
      Wire.endTransmission(false);
      Wire.requestFrom(MPU, 14, true); // request a total of 14 registers
      AcX[i] = Wire.read() << 8 | Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
      AcY[i] = Wire.read() << 8 | Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
      AcZ[i] = Wire.read() << 8 | Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
      Wire.read() << 8 | Wire.read();           // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L) Unused - leave because wire.read() goes through this reg
      GyX[i] = Wire.read() << 8 | Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
      GyY[i] = Wire.read() << 8 | Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
      GyZ[i] = Wire.read() << 8 | Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

      Wire.endTransmission(true);
    }

    voltMeasurement = analogRead(A0);
    currentMeasurement = analogRead(A2);

    currentTime = millis();
    timeDelta = currentTime - startTime;
    startTime = currentTime;

    voltVal = (float) voltMeasurement * 5 * 2 / 1023.0; // Volt, voltage divider halves voltage
    currentVout = (float) currentMeasurement * 5 / 1023.0; // INA169 Vout
    currentVal = (currentVout * 1000.0) * 1000.0 / (0.1 * 10000.0); // mA, Rs = 10 Ohms Rl = 10k Ohms
    powerVal = voltVal * currentVal; // mW
    energyVal = energyVal + ((powerVal / 1000) * ((float) timeDelta / 1000)); // Joules

    dtostrf(voltVal, 0, 2, voltStr);
    dtostrf(currentVal, 0, 0, currentStr);
    dtostrf(powerVal, 0, 0, powerStr);
    dtostrf(energyVal, 0, 1, energyStr);

    // Package into I-frame
    xSemaphoreTake(xSerialSemaphore, portMAX_DELAY);  // TODO: remove

    buf_len = 0;
    buf_len += sprintf(send_buf[buf_idx] + buf_len,
                       "%c%c%c",
                       START_STOP_BYTE, (recv_seq << 1) | 0x1, send_seq << 1);  // Control fields

    Serial.print("buf after 3 bytes: "); Serial.println(send_buf[buf_idx]);

    for (uint8_t i=0; i<NUM_GY521; i++) {  // Sensor readings
      buf_len += sprintf(send_buf[buf_idx] + buf_len,
                         "%d,%d,%d,%d,%d,%d,",
                         AcX[i], AcY[i], AcZ[i], GyX[i], GyY[i], GyZ[i]);
      Serial.print("buf after sensor readings "); Serial.print(i);
      Serial.print(": "); Serial.println(send_buf[buf_idx]);
    }

    buf_len += sprintf(send_buf[buf_idx] + buf_len,  // Telemetry
                       "%s,%s,%s,%s",
                       voltStr, currentStr, powerStr, energyStr);

    Serial.print("buf after telemetry"); Serial.println(send_buf[buf_idx]);

    Serial.print("buf len: "); Serial.println(buf_len);
    Serial.print("str len: "); Serial.println(strlen(send_buf[buf_idx]));
    checksum = crc16(&send_buf[buf_idx][1], buf_len - 1);   // Don't include start byte in checksum

    buf_len += sprintf(send_buf[buf_idx] + buf_len,
                       "%c%c%c",
                       checksum >> 8, checksum & 0xFF, START_STOP_BYTE);

    // TaskSend handles sending data
    xQueueSend(xSerialSendQueue, &buf_idx, portMAX_DELAY);

    buf_idx = (buf_idx + 1) & 0x07;
    send_seq = (send_seq + 1) & 0x7F;

    // xSemaphoreTake(xSerialSemaphore, portMAX_DELAY);
    // TODO: remove
    for (uint8_t i=0; i<NUM_GY521; i++) {
      Serial.print(i); Serial.print(" | ");
      Serial.print("AcX = ");      Serial.print(AcX[i]);
      Serial.print(" | AcY = ");   Serial.print(AcY[i]);
      Serial.print(" | AcZ = ");   Serial.print(AcZ[i]);
      Serial.print(" | GyX = ");   Serial.print(GyX[i]);
      Serial.print(" | GyY = ");   Serial.print(GyY[i]);
      Serial.print(" | GyZ = "); Serial.println(GyZ[i]);
    }

    Serial.print("Voltage(V) = ");        Serial.print(voltVal);
    Serial.print(" | Current(mA) = "); Serial.print(currentVal);
    Serial.print(" | Power(mW) = ");     Serial.print(powerVal);
    Serial.print(" | Energy(J) = ");  Serial.println(energyVal);
    Serial.print("Checksum: "); Serial.println(checksum);

    xSemaphoreGive(xSerialSemaphore);

    // Wait
    vTaskDelayUntil(&prevWakeTime, xDelay);
  }
}


void TaskSend(void *pvParameters) {
  uint8_t buf_read_idx, byt;
  uint16_t len;

  for (;;) {
    xQueueReceive(xSerialSendQueue, &buf_read_idx, portMAX_DELAY);
    xSemaphoreTake(xSerialSemaphore, portMAX_DELAY);

    Serial.print("Received buf index: "); Serial.println(buf_read_idx);
    Serial.println("BUF -------------");
    Serial.write(send_buf[buf_read_idx]);
    Serial.println("");

    len = strlen(send_buf[buf_read_idx]);
    // Escape before sending
    for (uint16_t i = 0; i < len; i++) {
      byt = send_buf[buf_read_idx][i];
      if ((byt == START_STOP_BYTE || byt == ESCAPE_BYTE)
           && !(i == 0 || i == len-1)) {
        Serial.println("Escaped!");
        Serial3.write(ESCAPE_BYTE);
        Serial3.write(byt ^ 0x20);
      }
      else {
        Serial3.write(byt);
      }
    }

    xSemaphoreGive(xSerialSemaphore);
  }
}

void sendIFrame(byte i) {
  Serial.print("Frame number "); Serial.print(i); Serial.print(" = ");
  Serial.write(START_STOP_BYTE);
  byte len = strlen(send_buf[i]);
  Serial.write(send_buf[i][0]);
  Serial.write(send_buf[i][1] & 0xFE); // Changes the bit[0] from 1 back to 0
  for (byte j = 2; j < len; j++) {
    Serial.write(send_buf[i][j]);
  }
  Serial.write(START_STOP_BYTE);
}

void TaskRecv(void *pvParameters) {
  bool is_stop_byte = false;
  byte buf[50];               // buf[0] = START, buf[1] = recv_seq, buf[2] = frame, buf[3] = checkNum, buf[4] = checkNum, buf[5] = START_STOP_BYTE
  bool expectStopByte = false;// Signal when a START byte has occured, so that the next such byte is a START_STOP_BYTE byte.
  byte i = -1;                // used to fill buf

  for (;;) {
    xSemaphoreTake(xSerialSemaphore, portMAX_DELAY);
    delay(100);
//    if (Serial.available() <= 0) {
//      continue;
//    }
//    buf[++i] = Serial.read();
//    if (i == 0 && buf[i] != START_STOP_BYTE) {    // if expecting starting byte but receive otherwise
//      Serial.println("Error, Frame does not start with 0x7e");
//      i = -1;
//      expectStopByte = false;
//      memset(buf, NULL, 1);
//    }
//    else if (buf[i] == START_STOP_BYTE && !expectStopByte) { // START byte received, the next identical byte received will be a START_STOP_BYTE byte
//      expectStopByte = true;
//    }
//    else if (buf[i] == START_STOP_BYTE && expectStopByte) {  // START_STOP_BYTE byte is received, terminate the frame
//      Serial.println("Frame terminated");
//      if (buf[2] == final2Bits_HFrame && isFrameCorrect(&buf[0], 'H')) {  // is a H-Frame, verify its correct
//        Serial.print("Returning bytes: ");
//        Serial.write(buf, i + 1);
//        Serial.println("");
//        //Serial.write(msg, i + 1);
//        Serial.println("Success");
//      }
//      else if ( ( (buf[2] & 0b11) == final2Bits_SFrame) && isFrameCorrect(&buf[0], 'S') ) { // is an S-Frame, verify its correct
//        // Check whether need to resend data
//        // Trim to only frame[3:2]]. If true, RPi rejected the frame sent by Arduino, must resend
//        byte RPiReceive = (buf[1] >> 1) & 0xF;
//        recv_seq = (recv_seq + 1) & 0x7F;     // keeps recv_seq between 0 - 127
//        if ( ( (buf[2] >> 2) & 0b11) == SFRAME_REJ) {
//          Serial.print("RPi has not received all frames, resending frames "); Serial.print(RPiReceive); Serial.print(" to "); Serial.println(send_seq);
//          if (RPiReceive > send_seq) {   // frame number has exceeded 128
//            for (byte i = RPiReceive; i < 16; i++) {
//              sendIFrame(i);
//            }
//            for (byte i = 0; i <= send_seq; i++) {
//              sendIFrame(i);
//            }
//          }
//          else {
//            for (byte i = RPiReceive; i <= send_seq; i++) {
//              sendIFrame(i);
//            }
//          }
//        }
//        else if ( ( (buf[2] >> 2) & 0b11) == SFRAME_RR) {
//          Serial.print("RPi ready to receive frame number "); Serial.println(RPiReceive);
//          sendIFrame(RPiReceive);
//          send_seq = (send_seq + 1) & 0x7F;                   // Keeps send sequence no within 0-127
//        }
//      }
//      else if ( ( (buf[1] & 0b1) == 0b1) && ( (buf[2] & 0b1) == 0b0) && isFrameCorrect(&buf[0], 'I') ) { // is an I-Frame, verify its correct
//        recv_seq = (recv_seq + 1) & 0x7F;     // keeps recv_seq between 0 - 127
//        char iFrameMsg[50];
//        byte i = 0;
//        iFrameMsg[i++] = buf[3];
//        for (byte j = 4; buf[j + 2] != START_STOP_BYTE; j++) { // since checksum is 2 bytes long, followed by START_STOP_BYTE byte, stop when hit 1st checksum byte
//          iFrameMsg[i++] = buf[j];
//        }
//        iFrameMsg[i] = '\0';  // terminate the string properly
//        Serial.print("The I-Frame Message is: " ); Serial.println(iFrameMsg);
//        // if(iFrameMsg.equals("send me data")) {   // Intepret the message sent in I-Frame
//        // Send S-Frame back to RPi
//        Serial.println("I-Frame received! Sending S-Frame");
//        Serial.write(START_STOP_BYTE);
//        byte msg[2];
//        msg[0] = buf[1];
//        msg[1] = 0x01;
//        Serial.write(msg[0]);
//        Serial.write(msg[1]);
//        int checkNum = crc16(&msg[0], 2);
//        Serial.write(checkNum);
//        Serial.write(START_STOP_BYTE);
//      }
//      memset(buf, NULL, i + 1);
//      i = -1;
//      expectStopByte = false;
//    }
//    else {
//      Serial.print("byte is "); Serial.write(buf[i]); Serial.println("");
//    }

    xSemaphoreGive(xSerialSemaphore);
  }
}

Frame get_frame_sort(byte* buf, int len) {
  uint8_t sort = buf[2] & 0x03;
  if (sort == 0 || sort == 2) {
    return IFrame;
  }
  else if (sort == 1) {
    return SFrame;
  }
  else {  // sort 3
    return HFrame;
  }
}


bool isFrameCorrect(byte* buf, char type) {
  // Calculates the length of buf. Also START byte check.
  byte len;
  if (buf[0] != START_STOP_BYTE)  return false; // does not start with START byte, fail
  else                  len = 1;      // start with the first data byte

  while (buf[len] != START_STOP_BYTE && len < 100) { // Arbitary number 100 set to prevent infinite loop in case frame does not terminate properly
    len++;
  }
  len++;                                  // include START_STOP_BYTE byte as part of len

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
  return (buf[1] & 0b1) && isFrameCorrect && crc16(&buf[1], len - 4) == checkNum && buf[len - 1] == START_STOP_BYTE;
}

uint16_t crc16(char* buf, int len) {
    char str1[5], str2[5];
    sprintf(str1, "%02X", buf[0]);
    sprintf(str2, "%02X", buf[len-1]);
    Serial.print("first byte: ");  Serial.print(str1);
    Serial.print(", last byte: "); Serial.println(str2);

    uint16_t remainder = 0x0000;
    uint16_t poly = 0x1021;
    for (int byte = 0; byte < len; ++byte) {
        remainder ^= (buf[byte] << 8);
        for (uint8_t bit = 8; bit > 0; --bit) {
            if (remainder & 0x8000) {
                remainder = (remainder << 1) ^ poly;
            } else {
                remainder = (remainder << 1);
            }
        }
    }
    return remainder;
}

void loop()
{
  // Empty. Things are done in Tasks.
}
