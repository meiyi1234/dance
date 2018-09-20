#include <Arduino_FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>
#include <Wire.h>

const byte MPU = 0x68;                    // I2C address of the MPU-6050
const byte NUM_GY521 = 3;                 // Number of sensors
byte sensor_loops;
int AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;  // Reads only one set of Gyro & Accel at one time,
                                        // next cycle takes in the next set of readings from another sensor. 16 bits long.
double voltVal, currentVal, powerVal, energyVal = 0;
unsigned long startTime, currentTime, timeDelta;

// Global variables below used for I-Frame, global since must keep in memory, a resend is possible.
double AcXRx, AcYRx, AcZRx, GyXRx, GyYRx, GyZRx;
double voltRx, currentRx, powerRx, energyRx;
int checksum, control;
byte receive_seq = 0;
byte send_seq = 0;

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

bool isHFrameCorrect(byte* buf) {
  int checkNum = buf[3] << 8 | buf[4];
  return buf[0] == START && buf[2] == final2Bits_HFrame && crc16(&buf[1], 2) == checkNum && buf[5] == STOP;
}

bool isSFrameCorrect(byte *buf) {
  int checkNum = buf[3] << 8 | buf[4];
//  Serial.print("S original checksum is: "); Serial.print(checkNum); Serial.print("\t"); Serial.print("S calculated checksum is: "); Serial.println(crc16(&buf[1], 2));
  return buf[0] == START && buf[2] == final2Bits_SFrame && crc16(&buf[1], 2) == checkNum && buf[5] == STOP;
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
          if (!isHFrameCorrect(msg)) {
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
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(A0, INPUT); // Voltage Divider output
  pinMode(A1, INPUT); // INA169 VOut
  sensor_loops = 0;
  while (sensor_loops < NUM_GY521) {
    digitalWrite(5, HIGH);
    digitalWrite(6, HIGH);
    digitalWrite(7, HIGH);
    digitalWrite(5 + sensor_loops, LOW);
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

void serialDoubleWrite(double data) {
  byte * b = (byte *) &data;
  Serial3.write(b[0]);
  Serial3.write(b[1]);
  Serial3.write(b[2]);
  Serial3.write(b[3]);
}

// Reads in the gyro & accel values
void ReadValues(void *pvParameters) {
  prevWakeTimeRead = xTaskGetTickCount();
  while (true) {
    if (xSemaphoreTake(BlockReadSemaphore, 0) == pdTRUE) {
      if (xSemaphoreTake(UninterruptedReadSemaphore, 0) == pdTRUE) {
        sensor_loops = 0;
        while (sensor_loops < NUM_GY521) {
          digitalWrite(5, HIGH);
          digitalWrite(6, HIGH);
          digitalWrite(7, HIGH);
          digitalWrite(5 + sensor_loops, LOW);
          Wire.beginTransmission(MPU);
          Wire.write(0x3B);                     // starting with register 0x3B (ACCEL_XOUT_H)
          Wire.endTransmission(false);
          Wire.requestFrom(MPU, 14, true);      // request a total of 14 registers
          AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
          AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
          AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
          Tmp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L) - Unnecessary
          GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
          GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
          GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
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
        voltVal = (double) voltMeasurement * 5 * 2 / 1023.0;                      // Volt, voltage divider halves voltage
        double currentVout = (double) currentMeasurement * 5 / 1023.0;                   // INA169 Vout
        currentVal = (currentVout * 1000.0) * 1000.0 / (10 * 10000.0);            // mA, Rs = 10 Ohms Rl = 10k Ohms
        powerVal = voltVal * currentVal;                                          // mW
        energyVal = energyVal + ((powerVal / 1000) * ((double) timeDelta / 1000));// Joules
        Serial.print("Voltage(V) = "); Serial.print(voltVal);
        Serial.print(" | Current(mA) = "); Serial.print(currentVal);
        Serial.print(" | Power(mW) = "); Serial.print(powerVal);
        Serial.print(" | Energy(J) = "); Serial.println(energyVal);
        startTime = currentTime;
  
        xQueueSendToBack(xQueue0, &AcX, 0);
        xQueueSendToBack(xQueue0, &AcY, 0);
        xQueueSendToBack(xQueue0, &AcZ, 0);
        xQueueSendToBack(xQueue0, &GyX, 0);
        xQueueSendToBack(xQueue0, &GyY, 0);
        xQueueSendToBack(xQueue0, &GyZ, 0);
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
  uint8_t *buf;             // buf[0] = START, buf[1] = receive_seq, buf[2] = frame, buf[3] = checkNum, buf[4] = checkNum, buf[5] = STOP
  bool expectStopByte;
  bool canSendNewIFrame = false;
  bool serialStillSending;  // Checks whether is there any incoming new bytes
  byte i = -1;               // used to fill buf
  prevWakeTimeSend = xTaskGetTickCount();
  while (true) {
    if (xSemaphoreTake(UninterruptedReadSemaphore, 0) == pdTRUE) {
      canSendNewIFrame = expectStopByte = false;
      serialStillSending = true;
      while(serialStillSending) {
        if (Serial3.available() > 0) {
          buf[++i] = Serial3.read();
          if(buf[i] == -1) {                  // there is no data available
            serialStillSending = false;
          }
          else if(i == 0 && buf[i] != START) {     // if expecting starting byte but receive otherwise
            Serial.print("Error, Frame does not start with 0x7e");
            i = -1;
            expectStopByte = false;
            memset(buf, NULL, 1);
          }
          else if (buf[i] == START && !expectStopByte) { // START byte received, the next identical byte received will be a STOP byte
            expectStopByte = true;
          }
          else if (buf[i] == STOP && expectStopByte) {  // STOP byte is received, terminate the frame
            if(buf[2] == final2Bits_HFrame && isHFrameCorrect(&buf[0])) {   // is a H-Frame, verify its correct
              establishContact();
              vTaskDelayUntil(&prevWakeTimeSend, (2 / portTICK_PERIOD_MS));      
              xSemaphoreGive(BlockReadSemaphore);
              xSemaphoreGive(UninterruptedReadSemaphore);
              return;
            }
            else if(buf[2] == final2Bits_SFrame && isSFrameCorrect(&buf[0])) {
              // Check whether need to resend data
              // Trim to only frame[3:2]]. If true, RPi rejected the frame sent by Arduino, must resend
              receive_seq = buf[1] >> 1;
              if((buf[2] >> 2 & 0b11) == SFRAME_REJ) {
                Serial3.write(START);
                Serial3.write(control);
                Serial3.write((int)AcXRx);
                Serial3.write((int)AcYRx);
                Serial3.write((int)AcZRx);
                Serial3.write((int)GyXRx);
                Serial3.write((int)GyYRx);
                Serial3.write((int)GyZRx);
                serialDoubleWrite(voltRx);
                serialDoubleWrite(currentRx);
                serialDoubleWrite(powerRx);
                serialDoubleWrite(energyRx);
                Serial3.write(checksum);
                Serial3.write(STOP);
              }
              else if((buf[2] >> 2 & 0b11) == SFRAME_RR) {
                canSendNewIFrame = true;
              }
            }
          }
        }
      }
      memset(buf, NULL, 1);     // Reset buffer to reuse later in I-Frame
      if(canSendNewIFrame || receive_seq == 0) {
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
        Serial3.write(START);
        control = ((receive_seq << 1 | 0b1) << 8) | (send_seq++ << 1);
        Serial3.write(control);
        Serial3.write((int)AcXRx);
        Serial3.write((int)AcYRx);
        Serial3.write((int)AcZRx);
        Serial3.write((int)GyXRx);
        Serial3.write((int)GyYRx);
        Serial3.write((int)GyZRx);
        serialDoubleWrite(voltRx);
        serialDoubleWrite(currentRx);
        serialDoubleWrite(powerRx);
        serialDoubleWrite(energyRx);
        buf[0] = (receive_seq << 1 | 0b1);
        buf[1] = (send_seq << 1 | 0b1);
        checksum = crc16(&buf[0], 2);
        Serial3.write(checksum);
        Serial3.write(STOP);
      }
      vTaskDelayUntil(&prevWakeTimeSend, (2 / portTICK_PERIOD_MS));
      xSemaphoreGive(UninterruptedReadSemaphore);
    }
  }
}
void loop() {}
