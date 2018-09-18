/* TO-DO: control is not calculated in I-Frame yet.
          checksum is not calculated in all frames yet.
*/
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
int voltMeasurement, currentMeasurement;
double voltVal, currentVout, currentVal, powerVal, energyVal = 0;
unsigned long startTime, currentTime, timeDelta;
double AcXRx, AcYRx, AcZRx, GyXRx, GyYRx, GyZRx;
double voltRx, currentRx, powerRx, energyRx;
int checksum, control;

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

byte receive_seq = 0;
bool firstIFrame = false;

void establishContact() {
  bool handshake = false;
  while (!handshake) {
    if (Serial.available()) {
      // HFrame contains START (8 bits - byte), receive_seq (8 bits), type H-frame (8 bits),
      // checksum (16 bits - integer), STOP (8 bits)
      uint64_t HFrame = (uint64_t)(Serial.read());
      
      // Check whether start, stop & RR frames are correct
      if(isHFrameCorrect(HFrame)) {
        // Send it back to the RPi, handshake verified
        Serial.write(START);
        receive_seq = (byte)((HFrame >> 32) % 0x100);
        Serial.write(receive_seq);
        Serial.write(final2Bits_HFrame);
        
        Serial.write(STOP);
        handshake = firstIFrame = true;
      }
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
  
  // initialize serial communication at 38400 bits per second:
  Serial.begin(38400);
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

bool isHFrameCorrect(uint64_t HFrame) {
  byte check = (byte)((HFrame >> 24) % 0x100);
  return (byte)(HFrame >> 40) == START && (byte)(HFrame % 0x100) == STOP && check == final2Bits_HFrame;
}

bool isSFrameCorrect(uint64_t SFrame) {
  byte check = (byte)((SFrame >> 24) % 0x04);
  return (byte)(SFrame >> 40) == START && (byte)(SFrame % 0x100) == STOP && check == final2Bits_SFrame;
}

void serialDoubleWrite(double data) {
  byte * b = (byte *) &data;
  Serial.write(b[0]);
  Serial.write(b[1]);
  Serial.write(b[2]);
  Serial.write(b[3]);
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
        voltMeasurement = analogRead(A0);
        currentMeasurement = analogRead(A1);
        timeDelta = currentTime - startTime;
        voltVal = (double) voltMeasurement * 5 * 2 / 1023.0;                      // Volt, voltage divider halves voltage
        currentVout = (double) currentMeasurement * 5 / 1023.0;                   // INA169 Vout
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
void SendValues(void *pvParameters) {
  bool canSendIFrame = false;
  prevWakeTimeSend = xTaskGetTickCount();
  while (true) {
    if (xSemaphoreTake(UninterruptedReadSemaphore, 0) == pdTRUE) {
      if (Serial.available() > 0) {
        xSemaphoreGive(BlockReadSemaphore);
        xSemaphoreGive(UninterruptedReadSemaphore);
        vTaskDelayUntil(&prevWakeTimeSend, (1 / portTICK_PERIOD_MS));
                
        if(!firstIFrame) {          // if not the first IFrame, need to receive S-Frame to signal ACK from RPi
          uint64_t newFrame = (uint64_t)Serial.read();
          // SFrame contains START (8 bits), receive_seq (8 bits), type SFrame (8 bits),
          // checksum (16 bits), STOP (8 bits)
          
          // Check what type of frame is it
          if((byte)(newFrame % 0b100) == final2Bits_HFrame) {      // is a H-Frame
            if(isHFrameCorrect(newFrame)) {
              establishContact();  
              vTaskDelayUntil(&prevWakeTimeSend, (2 / portTICK_PERIOD_MS));      
              xSemaphoreGive(BlockReadSemaphore);
              xSemaphoreGive(UninterruptedReadSemaphore);
            }
          }
          else if((byte)(newFrame % 0b100) == final2Bits_SFrame) { // is a S-Frame
            if(isSFrameCorrect(newFrame)) {
              // Check whether need to resend data
              byte resend = (byte)((newFrame >> 26) % 0x04);
              if(resend == SFRAME_REJ) {          // RPi rejects the frame sent by Arduino, must resend
                Serial.write(START);
                Serial.write(control);
                Serial.write((int)AcXRx);
                Serial.write((int)AcYRx);
                Serial.write((int)AcZRx);
                Serial.write((int)GyXRx);
                Serial.write((int)GyYRx);
                Serial.write((int)GyZRx);
                serialDoubleWrite(voltRx);
                serialDoubleWrite(currentRx);
                serialDoubleWrite(powerRx);
                serialDoubleWrite(energyRx);
                Serial.write(checksum);
                Serial.write(STOP);
              }
              else if(resend == SFRAME_RR) {
                canSendIFrame = true;
              }
            }
          }
        }
        else {  // Assume we can simply send an IFrame. TODO: Check serial port for any possible HFrame  
          canSendIFrame = true;
          firstIFrame = false;
        }        
        if(canSendIFrame) {
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
          Serial.println("Receive");
          Serial.write(START);
          Serial.write(control);
          Serial.write((int)AcXRx);
          Serial.write((int)AcYRx);
          Serial.write()int)AcZRx);
          Serial.write((int)GyXRx);
          Serial.write((int)GyYRx);
          Serial.write((int)GyZRx);
          serialDoubleWrite(voltRx);
          serialDoubleWrite(currentRx);
          serialDoubleWrite(powerRx);
          serialDoubleWrite(energyRx);
          Serial.write(checksum);
          Serial.write(STOP);
          vTaskDelayUntil(&prevWakeTimeSend, (2 / portTICK_PERIOD_MS));
          xSemaphoreGive(UninterruptedReadSemaphore);
        }  
        vTaskDelayUntil(&prevWakeTimeSend, (2 / portTICK_PERIOD_MS));
        xSemaphoreGive(UninterruptedReadSemaphore);
      }
    }
  }
}
void loop() {}
