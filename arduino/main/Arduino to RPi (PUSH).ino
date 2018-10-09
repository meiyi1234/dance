#include <Wire.h>

#include <Arduino_FreeRTOS.h>
#include <semphr.h>

// Fixed constants - DO NOT TOUCH.
#define START_STOP_BYTE 0x7E
#define ESCAPE_BYTE     0x7D
#define MPU             0x68  // I2C address of the MPU-6050
#define SFRAME_RR       0x00
#define SFRAME_RNR      0x02
#define SFRAME_REJ      0x01
const byte final2Bits_HFrame = 0x03;
const byte final2Bits_SFrame = 0x01;

// Constants that may be adjusted according to our needs
#define VOLT_DIVIDER      A0
#define VOUT              A2
#define NUM_GY521         3
#define BAUD_RATE         115200
#define BUFFER_SIZE       32
#define MIN_IFRAME_LENGTH 50
#define MAX_IFRAME_LENGTH 135
const TickType_t xDelay = 1000 / portTICK_PERIOD_MS;    // 10ms - Period of TaskReadSensors

unsigned long startTime;
uint8_t send_seq = 1, recv_seq = 0; // used for filling in control byte fields when sending frames
uint8_t lastSent = 0;                // keeps track of the current last frame put in buffer
uint8_t lastACK = BUFFER_SIZE - 1;   // keeps track of the last acknowledged frame by the RPi. Set to BUFFER_SIZE - 1 to account for first ACK case after handshake.
uint8_t freeBuffer = BUFFER_SIZE;   // if = 0, stop reading new values
char send_buf[BUFFER_SIZE][MAX_IFRAME_LENGTH];
uint8_t buf_idx = 0;

// FreeRTOS data structures
QueueHandle_t xSerialSendQueue;
SemaphoreHandle_t xSerialSemaphore;  // Allow only one task to access the serial port


void setup() {
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(VOLT_DIVIDER, INPUT); // Voltage Divider output
  pinMode(VOUT, INPUT);         // INA169 VOut

  Serial.begin(BAUD_RATE);  // PC Serial
  Serial3.begin(BAUD_RATE);  // RPi Serial 
  Serial.println("Ready");

  xSerialSendQueue = xQueueCreate(BUFFER_SIZE, sizeof(uint8_t));
  xSerialSemaphore = xSemaphoreCreateMutex();
  xSemaphoreGive(xSerialSemaphore);

  establishContact();
  setupSensors();

  xTaskCreate(TaskReadSensors, "Read sensors", 512, NULL, 4, NULL); // Highest priority
  xTaskCreate(TaskSend, "Serial send", 512, NULL, 3, NULL);
  xTaskCreate(TaskRecv, "Serial recv", 512, NULL, 2, NULL);         // Lowest priority since mostly ack messages will be sent by primary

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
    if (Serial3.available() <= 0) {    // RPi Serial
      continue;
    }
    msg[++i] = Serial3.read();         // RPi Serial

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
        Serial3.write(msg, i + 1);          // RPi Serial
        Serial.println("Success");
        handshake = true;
        recv_seq = (recv_seq + 1) & 0x7F;   // Keeps the sequence number between 0 - 127 to fit into control_byte

        memset(msg, NULL, i + 1);
        i = -1;
        expectStopByte = false;
      }
    }
    else {  // receiving the other bytes in between
      Serial.print("byte is "); Serial.write(msg[i]); Serial.println("");
    }
    delay(10);
  }
}

void setupSensors() {
  for (uint8_t i = 0; i < NUM_GY521; i++) {
    digitalWrite(5, HIGH);
    digitalWrite(6, HIGH);
    digitalWrite(7, HIGH);
    digitalWrite(5 + i, LOW);
    Wire.begin();
    Wire.beginTransmission(MPU);
    Wire.write(0x6B);           // PWR_MGMT_1 register
    Wire.write(0);              // set to zero (wakes up the MPU-6050)
    Wire.endTransmission(true);
    Wire.beginTransmission(MPU);
    Wire.write(0x1C);           // ACCEL_CONFIG register
    Wire.write(8);              // FS_SEL 01: 4g
    Wire.endTransmission(true);
    Wire.beginTransmission(MPU);
    Wire.write(0x1B);           // GYRO_CONFIG register
    Wire.write(16);             // FS_SEL 10: 1000 degrees per second
    Wire.endTransmission(true);
  }
  startTime = millis();
}

uint16_t writeEscaped(char* buf, uint16_t write_start, char* data) {
  // Writes characters in data to buf starting at write_start. Escapes START_STOP_BYTE
  // and ESCAPE_BYTE if they are found. Returns length of data written.
  char byt;
  uint16_t pos = write_start;
  for (uint16_t i = 0; i < strlen(data); i++) {
    byt = data[i];
    if (byt == START_STOP_BYTE || byt == ESCAPE_BYTE) {
      buf[pos++] = ESCAPE_BYTE;
      buf[pos++] = byt ^ 0x20;
    }
    else {
      buf[pos++] = byt;
    }
  }
  buf[pos] = '\0';
  return pos - write_start;
}

void TaskReadSensors(void *pvParameters) {
  uint16_t buf_len, checksum;
  int AcX[NUM_GY521], AcY[NUM_GY521], AcZ[NUM_GY521];
  int GyX[NUM_GY521], GyY[NUM_GY521], GyZ[NUM_GY521];
  int voltMeasurement, currentMeasurement;
  float voltVal, currentVout, currentVal, powerVal, energyVal = 0;
  char voltStr[8], currentStr[8], powerStr[8], energyStr[8];
  char control_chars[3], check_chars[3];
  unsigned long currentTime, timeDelta;  // startTime defined globallly

  TickType_t prevWakeTime = xTaskGetTickCount();
  for (;;) {
    /*if (freeBuffer <= 0) {  // if no free buffer space left, stop reading new values
      Serial.println("No more free buffer space!");
      vTaskDelayUntil(&prevWakeTime, xDelay);
      continue;
    }*/
    xSemaphoreTake(xSerialSemaphore, portMAX_DELAY);
    
    for (uint8_t i = 0; i < NUM_GY521; i++) {
      digitalWrite(5, HIGH);
      digitalWrite(6, HIGH);
      digitalWrite(7, HIGH);
      digitalWrite(5 + i, LOW);

      Wire.beginTransmission(MPU);
      Wire.write(0x3B);                         // starting with register 0x3B (ACCEL_XOUT_H)
      Wire.endTransmission(false);
      Wire.requestFrom(MPU, 14, true);          // request a total of 14 registers
      AcX[i] = Wire.read() << 8 | Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
      AcY[i] = Wire.read() << 8 | Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
      AcZ[i] = Wire.read() << 8 | Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
      Wire.read() << 8 | Wire.read();           // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L) Unused - leave because wire.read() goes through this reg
      GyX[i] = Wire.read() << 8 | Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
      GyY[i] = Wire.read() << 8 | Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
      GyZ[i] = Wire.read() << 8 | Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

      AcX[i] = AcY[i] = AcZ[i] = GyX[i] = GyY[i] = GyZ[i] = 69800 + 10 * i; // Dummy values, delete in actual

      Wire.endTransmission(true);
    }

    voltMeasurement = analogRead(VOLT_DIVIDER);
    currentMeasurement = analogRead(VOUT);

    currentTime = millis();
    timeDelta = currentTime - startTime;
    startTime = currentTime;

    voltVal = (float) voltMeasurement * 5 * 2 / 1023.0;                       // Volt, voltage divider halves voltage
    currentVout = (float) currentMeasurement * 5 / 1023.0;                    // INA169 Vout
    currentVal = (currentVout * 1000.0) * 1000.0 / (0.1 * 10000.0);           // mA, Rs = 10 Ohms Rl = 10k Ohms
    powerVal = voltVal * currentVal;                                          // mW
    energyVal = energyVal + ((powerVal / 1000) * ((float) timeDelta / 1000)); // Joules

    dtostrf(voltVal, 0, 2, voltStr);
    dtostrf(currentVal, 0, 0, currentStr);
    dtostrf(powerVal, 0, 0, powerStr);
    dtostrf(energyVal, 0, 1, energyStr);

    // Package into I-frame
    buf_len = 0;
    
    // Recv, send seq
    control_chars[0] = (recv_seq << 1) | 0x1;
    control_chars[1] = send_seq << 1;
    control_chars[2] = '\0';
    // Escape if contains 7D/7E
    buf_len += writeEscaped(send_buf[buf_idx], buf_len, control_chars);

    // Sensor reading - no need to escape since str won't contain ascii 7D={ or 7E=~
    for (uint8_t i = 0; i < NUM_GY521; i++) {
      buf_len += sprintf(send_buf[buf_idx] + buf_len,
                         "%d,%d,%d,%d,%d,%d,",
                         AcX[i], AcY[i], AcZ[i], GyX[i], GyY[i], GyZ[i]);
    }

    // Telemetry - no need to escape since str won't contain ascii 7D={ or 7E=~
    buf_len += sprintf(send_buf[buf_idx] + buf_len,
                       "%s,%s,%s,%s",
                       voltStr, currentStr, powerStr, energyStr);

    // Checksum
    checksum = crc16(&send_buf[buf_idx][0], buf_len);
    check_chars[0] = checksum >> 8;
    check_chars[1] = checksum & 0xFF;
    check_chars[2] = '\0';
    // Escape checksum if contains 7D/7E
    buf_len += writeEscaped(send_buf[buf_idx], buf_len, check_chars);

    // TaskSend handles sending data
    xQueueSend(xSerialSendQueue, &buf_idx, portMAX_DELAY);

    freeBuffer -= 1;
    buf_idx = (buf_idx + 1) & (BUFFER_SIZE - 1);
    send_seq = (send_seq + 1) & 0x7F;             // Keeps the sequence number between 0 - 127 to fit into control_byte

    // TODO: remove
    for (uint8_t i = 0; i < NUM_GY521; i++) {
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
    Serial.write(send_buf[buf_read_idx]);
    Serial.println("");

    Serial3.write(START_STOP_BYTE);             // RPi Serial
    len = strlen(send_buf[buf_read_idx]);
    Serial.print("send_buf["); Serial.print(buf_read_idx); Serial.print("] is of length "); Serial.println(len);

    // Escape before sending
    Serial3.write(send_buf[buf_read_idx], len); // RPi Serial
    Serial3.write(START_STOP_BYTE);             // RPi Serial
    lastSent = (lastSent + 1) & (BUFFER_SIZE - 1);
    xSemaphoreGive(xSerialSemaphore);
  }
}

void TaskRecv(void *pvParameters) {
  byte buf[80];               // buf[0] = START, buf[1] = recv_seq, buf[2] = frame, buf[3] = checkNum, buf[4] = checkNum, buf[5] = START_STOP_BYTE
  bool expectStopByte = false;// Signal when a START byte has occured, so that the next such byte is a START_STOP_BYTE byte.
  byte i = -1;                // used to fill buf

  for (;;) {
    xSemaphoreTake(xSerialSemaphore, portMAX_DELAY);
    if (Serial3.available() <= 0) {                // RPi Serial
      delay(10);
      xSemaphoreGive(xSerialSemaphore);
      continue;
    }
    buf[++i] = Serial3.read();                     // RPi Serial
    if (i == 0 && buf[i] != START_STOP_BYTE) {    // if expecting starting byte but receive otherwise
      Serial.println("Error, Frame does not start with 0x7e");
      i = -1;
      expectStopByte = false;
      memset(buf, NULL, 1);
    }
    else if (buf[i] == START_STOP_BYTE && !expectStopByte) { // START byte received, the next identical byte received will be a START_STOP_BYTE byte
      expectStopByte = true;
    }
    else if (buf[i] == START_STOP_BYTE && expectStopByte) {  // START_STOP_BYTE byte is received, terminate the frame
      Serial.println("Frame terminated");
      if (buf[2] == final2Bits_HFrame && isFrameCorrect(&buf[0], 'H')) {  // is a H-Frame, verify its correct
        Serial.print("Returning bytes: ");
        Serial.write(buf, i + 1);
        Serial.println("");
        Serial3.write(buf, i + 1);         // RPi Serial
        Serial.println("Success");
      }
      else if ( ( (buf[2] & 0b11) == final2Bits_SFrame) && isFrameCorrect(&buf[0], 'S') ) { // is an S-Frame, verify its correct
        // Check whether need to resend data
        // Trim to only frame[3:2]]. If true, RPi rejected the frame sent by Arduino, must resend
        uint8_t RPiReceive = (buf[1] >> 1) & (BUFFER_SIZE - 1);
        recv_seq = (recv_seq + 1) & 0x7F;                       // keeps recv_seq between 0 - 127 to fit within control_byte
        byte SFrameType = (buf[2] >> 2) & 0b11;
        if (SFrameType == SFRAME_REJ) {
          Serial.print("RPi has not received all frames, resending frames "); Serial.print(RPiReceive); Serial.print(" to "); Serial.println(lastSent);
          if (RPiReceive > lastSent) {
            for (uint8_t i = RPiReceive; i < BUFFER_SIZE; i++) {
              xQueueSend(xSerialSendQueue, &i, portMAX_DELAY);
            }
            for (uint8_t i = 0; i <= lastSent; i++) {
              xQueueSend(xSerialSendQueue, &i, portMAX_DELAY);
            }
          }
          else {
            for (uint8_t i = RPiReceive; i <= lastSent; i++) {
              xQueueSend(xSerialSendQueue, &i, portMAX_DELAY);
            }
          }
        }
        else if ( (SFrameType == SFRAME_RR) || (SFrameType == SFRAME_RNR) ) {
          Serial.print("RPi has successully received frames "); Serial.print((lastACK + 1) & (BUFFER_SIZE - 1)); Serial.print(" to "); Serial.println((RPiReceive - 1) & (BUFFER_SIZE - 1));
          uint8_t firstACK = (lastACK + 1) & (BUFFER_SIZE - 1); // the first newly ACK frame
          lastACK = (RPiReceive == 0) ? (BUFFER_SIZE - 1) : (RPiReceive - 1);
          freeBuffer += ( (firstACK > lastACK) ? BUFFER_SIZE : 0) + lastACK - firstACK + 1;
          if (firstACK > lastACK) {
            // We do not need to reset the whole buffer as that wastes time. We only reset until at the least STOP byte is cleared even for the shortest possible I-Frame.
            for (uint8_t i = firstACK; i < BUFFER_SIZE; i++) {
              memset(send_buf[i] + MIN_IFRAME_LENGTH, NULL, MAX_IFRAME_LENGTH - MIN_IFRAME_LENGTH);
            }
            for (uint8_t i = 0; i <= lastACK; i++) {
              memset(send_buf[i] + MIN_IFRAME_LENGTH, NULL, MAX_IFRAME_LENGTH - MIN_IFRAME_LENGTH);
            }
          }
          else {
            for (uint8_t i = firstACK; i <= lastACK; i++) {
              memset(send_buf[i] + MIN_IFRAME_LENGTH, NULL, MAX_IFRAME_LENGTH - MIN_IFRAME_LENGTH);
            }
          }
          if (SFrameType == SFRAME_RNR) {
            Serial.println("Waiting for RPi to finish operations...");
            delay(200);     // RPi might be expanding its CircularBuffer, wait. TO-DO: Is RPi going to signal when its ready?
          }
        }
      }
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
      memset(buf, NULL, i + 1);
      i = -1;
      expectStopByte = false;
    }
    else {
      Serial.print("byte is "); Serial.write(buf[i]); Serial.println("");
    }
    xSemaphoreGive(xSerialSemaphore);
  }
}

bool isFrameCorrect(byte* buf, char type) {
  // Calculates the length of buf. Also START byte check.
  byte len;
  if (buf[0] != START_STOP_BYTE)  return false; // does not start with START byte, fail
  else                            len = 1;      // start with the first data byte

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
  sprintf(str2, "%02X", buf[len - 1]);
  Serial.print("first byte: ");  Serial.print(str1);
  Serial.print(", last byte: "); Serial.println(str2);

  uint16_t remainder = 0x0000;
  uint16_t poly = 0x1021;
  for (int byte = 0; byte < len; ++byte) {
    remainder ^= (buf[byte] << 8);
    for (uint8_t bit = 8; bit > 0; --bit) {
      if (remainder & 0x8000) remainder = (remainder << 1) ^ poly;
      else                    remainder = (remainder << 1);
    }
  }
  return remainder;
}

void loop() {
  // Empty. Things are done in Tasks.
}
