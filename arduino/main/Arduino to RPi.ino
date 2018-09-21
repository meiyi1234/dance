// TO-DO: Settle case where the frames that need to be sent exceed over 128 (warps back to frame 0). E.g. Frame 127 - 128 & Frame 0.
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

const byte MPU = 0x68;                    // I2C address of the MPU-6050
const byte NUM_GY521 = 3;                 // Number of sensors
byte sensor_loops;
unsigned long startTime, currentTime, timeDelta;

// Global variables below used for I-Frame, global since must keep in memory, a resend is possible.
CircularBuffer<char*, 128> IFrames;
double AcXRx, AcYRx, AcZRx, GyXRx, GyYRx, GyZRx;
double voltRx, currentRx, powerRx, energyRx;
byte numSend = 0;

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

bool isFrameCorrect(byte* buf, char type) {
  byte check;
  bool isFrameCorrect;
  byte len = sizeof(buf);
  int checkNum = buf[len - 3] << 8 | buf[len - 2];

  // Checks the control_byte1
  if (type == 'H') {          // if is H-Frame
    check = final2Bits_HFrame;
  }
  else if (type == 'S') {     // if is S-Frame
    check = final2Bits_SFrame;
  }
  if (type == 'I') {          // if is I-Frame
    isFrameCorrect = (buf[2] == 0x00 || buf[2] == 0x02);
  }
  else {                      // is a H-Frame or S-Frame
    isFrameCorrect = (buf[2] == check);
  }
  //

  return buf[0] == START && isFrameCorrect && crc16(&buf[1], 2) == checkNum && buf[len - 1] == STOP;
}

// Handshake between Arduino and RPi
void establishContact() {
  bool handshake = false;
  bool expectStopByte = false;
  // msg[0] = START, msg[1] = receive_seq, msg[2] = frame, msg[3] = checkNum, msg[4] = checkNum, msg[5] = STOP
  byte msg[256];
  byte i = -1;           // to fill buf
  while (!handshake) {
    if (Serial3.available() > 0) {
      msg[++i] = Serial3.read();

      if (i == 0 && msg[i] != START) { // If is receiving first byte but is not START byte
        Serial.print("Error, frame doesnt start with 0x7e");
        i = -1;
        expectStopByte = false;
        memset(msg, NULL, 1);
      }
      else if (msg[i] == START && !expectStopByte) {  // If is receiving the START byte
        expectStopByte = true;                        // this is the START byte, the next such byte should be STOP byte
      }
      else if (msg[i] == STOP && expectStopByte) {  // If receiving STOP byte
        if (!isFrameCorrect(msg, 'H')) {
          Serial.println("H-frame invalid, not doing anything");
          memset(msg, NULL, 1);
          i = -1;
          break;
        }

        // If handshake, repeat message back to primary
        Serial.print("Returning bytes: ");
        Serial.write(msg, i + 1);
        Serial.println("");
        Serial3.write(msg, i + 1);
        Serial.println("Success");
        handshake = true;

        memset(msg, NULL, i + 1);
        i = -1;
        expectStopByte = false;
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

void setup() {
  pinMode(PIN_SENSOR_1, OUTPUT);
  pinMode(PIN_SENSOR_2, OUTPUT);
  pinMode(PIN_SENSOR_3, OUTPUT);
  pinMode(VOLT_DIVIDER, INPUT); // Voltage Divider output
  pinMode(VOUT, INPUT); // INA169 VOut
  sensor_loops = 0;
  while (sensor_loops < NUM_GY521) {
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
    sensor_loops++;
  }
  startTime = millis();

  // initialize Serial communication at 38400 bits per second:
  Serial3.begin(9600);
  Serial.begin(9600);
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
  vTaskStartScheduler();
}

void serialWriteDouble(double data) {
  byte * b = (byte *) &data;
  Serial3.write(b[0]);
  Serial3.write(b[1]);
  Serial3.write(b[2]);
  Serial3.write(b[3]);
}

// Reads in the gyro & accel values
void ReadValues(void *pvParameters) {
  int AcXRead, AcYRead, AcZRead, GyXRead, GyYRead, GyZRead;
  double voltVal, currentVal, powerVal, energyVal;
  prevWakeTimeRead = xTaskGetTickCount();
  while (true) {
    if (xSemaphoreTake(BlockReadSemaphore, 0) == pdTRUE) {
      if (xSemaphoreTake(UninterruptedReadSemaphore, 0) == pdTRUE) {
        sensor_loops = 0;
        while (sensor_loops < NUM_GY521) {
          digitalWrite(PIN_SENSOR_1, HIGH);
          digitalWrite(PIN_SENSOR_2, HIGH);
          digitalWrite(PIN_SENSOR_3, HIGH);
          digitalWrite(PIN_SENSOR_1 + sensor_loops, LOW);
          Wire.beginTransmission(MPU);
          Wire.write(0x3B);                     // starting with register 0x3B (ACCEL_XOUT_H)
          Wire.endTransmission(false);
          Wire.requestFrom(MPU, 14, true);      // request a total of 14 registers
          int AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
          int AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
          int AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
          int Tmp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L) - Unnecessary
          int GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
          int GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
          int GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
          Wire.endTransmission(true);
          Serial.print(sensor_loops); Serial.print(" | ");
          Serial.print("AcX = "); Serial.print(AcX);
          Serial.print(" | AcY = "); Serial.print(AcY);
          Serial.print(" | AcZ = "); Serial.print(AcZ);
          // Serial.print(" | Tmp = "); Serial.print(Tmp / 340.00 + 36.53);  //equation for temperature in degrees C from datasheet
          Serial.print(" | GyX = "); Serial.print(GyX);
          Serial.print(" | GyY = "); Serial.print(GyY);
          Serial.print(" | GyZ = "); Serial.println(GyZ);
          sensor_loops++;
        }
        currentTime = millis();
        int voltMeasurement = analogRead(A0);
        int currentMeasurement = analogRead(A1);
        timeDelta = currentTime - startTime;
        double voltVal = (double) voltMeasurement * 5 * 2 / 1023.0;                      // Volt, voltage divider halves voltage
        double currentVout = (double) currentMeasurement * 5 / 1023.0;                   // INA169 Vout
        double currentVal = (currentVout * 1000.0) * 1000.0 / (10 * 10000.0);            // mA, Rs = 10 Ohms Rl = 10k Ohms
        double powerVal = voltVal * currentVal;                                          // mW
        double energyVal = energyVal + ((powerVal / 1000) * ((double) timeDelta / 1000));// Joules
        Serial.print("Voltage(V) = "); Serial.print(voltVal);
        Serial.print(" | Current(mA) = "); Serial.print(currentVal);
        Serial.print(" | Power(mW) = "); Serial.print(powerVal);
        Serial.print(" | Energy(J) = "); Serial.println(energyVal);
        startTime = currentTime;

        xQueueSendToBack(xQueue0, &AcXRead, 0);
        xQueueSendToBack(xQueue0, &AcYRead, 0);
        xQueueSendToBack(xQueue0, &AcZRead, 0);
        xQueueSendToBack(xQueue0, &GyXRead, 0);
        xQueueSendToBack(xQueue0, &GyYRead, 0);
        xQueueSendToBack(xQueue0, &GyZRead, 0);
        xQueueSendToBack(xQueue0, &voltVal, 0);
        xQueueSendToBack(xQueue0, &currentVal, 0);
        xQueueSendToBack(xQueue0, &powerVal, 0);
        xQueueSendToBack(xQueue0, &energyVal, 0);

        vTaskDelayUntil(&prevWakeTimeRead, (4 / portTICK_PERIOD_MS));
        xSemaphoreGive(UninterruptedReadSemaphore);
      }
    }
  }
}

// Reads the accelerometer and gyroscope values from the Queue
// Checks for an S-Frame so that Arduino knows that it is ok to send data
void SendValues(void *pvParameters) {
  uint8_t *buf;         // buf[0] = START, buf[1] = receive_seq, buf[2] = frame, buf[3] = checkNum, buf[4] = checkNum, buf[5] = STOP
  bool expectStopByte;  // Signal when a START byte has occured, so that the next such byte is a STOP byte.
  byte i = -1;          // used to fill buf
  byte receive_seq;     // Read receive sequence number from RPi to compare with numSend
  double AcXRx, AcYRx, AcZRx, GyXRx, GyYRx, GyZRx;  // Reads only one set of Gyro & Accel from the Queue at one time,
                                                    // next cycle takes in the next set of readings from another sensor. 16 bits long.
  int AcX, AcY, AcZ, GyX, GyY, GyZ;                 // to typecast the double values in the queue back to int
  char AcXChar[4], AcYChar[4], AcZChar[4], GyXChar[4], GyYChar[4], GyZChar[4],
       voltChar[8], currentChar[8], powerChar[8], energyChar[8],
       startChar[3], control1Char[2], control2Char[2], checksumChar[4], stopChar[3];
  double voltRx, currentRx, powerRx, energyRx;
  char iframe[256];                                 // The full I-Frame to be sent
   
  prevWakeTimeSend = xTaskGetTickCount();
  while (true) {
    if (xSemaphoreTake(UninterruptedReadSemaphore, 0) == pdTRUE) {
      expectStopByte = false;
      if (Serial3.available() > 0) {
        buf[++i] = Serial3.read();
        if (i == 0 && buf[i] != START) {    // if expecting starting byte but receive otherwise
          Serial.print("Error, Frame does not start with 0x7e");
          i = -1;
          expectStopByte = false;
          memset(buf, NULL, 1);
        }
        else if (buf[i] == START && !expectStopByte) { // START byte received, the next identical byte received will be a STOP byte
          expectStopByte = true;
        }
        else if (buf[i] == STOP && expectStopByte) {  // STOP byte is received, terminate the frame
          if (buf[2] == final2Bits_HFrame && isFrameCorrect(&buf[0], 'H')) {  // is a H-Frame, verify its correct
            establishContact();
            vTaskDelayUntil(&prevWakeTimeSend, (2 / portTICK_PERIOD_MS));
            xSemaphoreGive(BlockReadSemaphore);
            xSemaphoreGive(UninterruptedReadSemaphore);
            return;
          }
          else if (buf[2] == final2Bits_SFrame && isFrameCorrect(&buf[0], 'S')) { // is a S-Frame, verify its correct
            // Check whether need to resend data
            // Trim to only frame[3:2]]. If true, RPi rejected the frame sent by Arduino, must resend
            receive_seq = (int)(buf[1] >> 1);
            if ((buf[2] >> 2 & 0b11) == SFRAME_REJ) {
              receive_seq = buf[1] >> 1;
              for(int i = receive_seq; i <= numSend; i++) {
                int len = strlen(IFrames[i]);
                for(int j = 0; j < len; j++) {
                  Serial3.write(IFrames[i][j]);
                }
              }
            }
            else if ((buf[2] >> 2 & 0b11) == SFRAME_RR) {
              receive_seq = buf[1] >> 1;
              xQueueReceive(xQueue0, &AcXRx, 0);
              xQueueReceive(xQueue0, &AcYRx, 0);
              xQueueReceive(xQueue0, &AcZRx, 0);
              xQueueReceive(xQueue0, &GyXRx, 0);
              xQueueReceive(xQueue0, &GyYRx, 0);
              xQueueReceive(xQueue0, &GyZRx, 0);
              xQueueReceive(xQueue0, &voltRx, 0);
              xQueueReceive(xQueue0, &currentRx, 0);
              xQueueReceive(xQueue0, &powerRx, 0);
              xQueueReceive(xQueue0, &energyRx, 0);
              Serial.println("Receive from Queue");
              AcX = (int)AcXRx;
              AcY = (int)AcYRx;
              AcZ = (int)AcZRx;
              GyX = (int)GyXRx;
              GyY = (int)GyYRx;
              GyZ = (int)GyZRx;
              itoa(START, startChar, 16);                       // convert the START byte into char[], therefore containing "7e"
              strcpy(iframe, startChar);
              itoa((receive_seq << 1 | 0b1), control1Char, 16); // 1 byte only therefore only containing one char
              strcat(iframe, control1Char);
              itoa((numSend << 1), control2Char, 16);           // 1 byte only therefore only containing one char
              strcat(iframe, control2Char);
              // Should I put a comma before the info part?
              itoa(AcX, AcXChar, 10);     // convert int to char[], with a radix of 10
              strcat(iframe, AcXChar);
              strcat(iframe, ",");
              itoa(AcY, AcYChar, 10);     // convert int to char[], with a radix of 10
              strcat(iframe, AcYChar);
              strcat(iframe, ",");
              itoa(AcZ, AcZChar, 10);     // convert int to char[], with a radix of 10
              strcat(iframe, AcZChar);
              strcat(iframe, ",");
              itoa(GyX, GyXChar, 10);     // convert int to char[], with a radix of 10
              strcat(iframe, GyXChar);
              strcat(iframe, ",");
              itoa(GyY, GyYChar, 10);     // convert int to char[], with a radix of 10
              strcat(iframe, GyYChar);
              strcat(iframe, ",");
              itoa(GyZ, GyZChar, 10);     // convert int to char[], with a radix of 10
              strcat(iframe, GyZChar);
              strcat(iframe, ",");
              itoa(voltRx, voltChar, 10); // convert int to char[], with a radix of 10
              strcat(iframe, voltChar);
              strcat(iframe, ",");
              itoa(voltRx, currentChar, 10);// convert int to char[], with a radix of 10
              strcat(iframe, currentChar);
              strcat(iframe, ",");
              itoa(voltRx, powerChar, 10); // convert int to char[], with a radix of 10
              strcat(iframe, powerChar);
              strcat(iframe, ",");
              itoa(voltRx, energyChar, 10); // convert int to char[], with a radix of 10
              strcat(iframe, energyChar);
              strcat(iframe, ",");
              
              byte msg[30];
              msg[0] = buf[1];
              msg[1] = numSend << 1;
              msg[2] = AcX;
              msg[4] = AcY;
              msg[6] = AcZ;
              msg[8] = GyX;
              msg[10] = GyY;
              msg[12] = GyZ;
              msg[14] = voltRx;
              msg[18] = currentRx;
              msg[22] = powerRx;
              msg[26] = energyRx;
              int checksum = crc16(&msg[0], 30);
              itoa(checksum, checksumChar, 10);
              strcat(iframe, checksumChar);
              itoa(STOP, stopChar, 16);   // convert the STOP byte into char[], therefore containing "7e"
              strcat(iframe, stopChar);
              IFrames.push(iframe);       // store the I-Frame into the circular buffer.
              numSend++;
              int len = strlen(iframe);
              for(int i = 0; i < len; i++) {
                Serial3.write(iframe[i]);
              }
            }
          }
          else if ((buf[2] == 0x00 || buf[2] == 0x02) && isFrameCorrect(&buf[0], 'I')) { // is an I-Frame, verify its correct

            // Send S-Frame back to RPi
            Serial3.write(START);
            byte msg[2];
            msg[0] = buf[1];
            msg[1] = 0x01;
            Serial3.write(msg[0]);
            Serial3.write(msg[1]);
            int checkNum = crc16(&msg[0], 2);
            Serial3.write(checkNum);
            Serial3.write(STOP);
          }
        }
        else {
          Serial.write(buf[i]); Serial.print(", ");
        }
      }
      vTaskDelayUntil(&prevWakeTimeSend, (2 / portTICK_PERIOD_MS));
      xSemaphoreGive(UninterruptedReadSemaphore);
    }
  }
}
void loop() {}
