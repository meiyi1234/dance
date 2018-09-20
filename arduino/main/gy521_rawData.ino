#include <Wire.h>
const int MPU = 0x68; // I2C address of the MPU-6050
const int NUM_GY521 = 3;
int8_t sensor_loops;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
int voltMeasurement, currentMeasurement;
double voltVal, currentVout, currentVal, powerVal, energyVal = 0;
unsigned long startTime, currentTime, timeDelta;

void setup() {
  // put your setup code here, to run once:
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(A0, INPUT); // Voltage Divider output
  pinMode(A2, INPUT); // INA169 VOut
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
  
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  sensor_loops = 0;
  while (sensor_loops < NUM_GY521) {
    digitalWrite(5, HIGH);
    digitalWrite(6, HIGH);
    digitalWrite(7, HIGH);
    digitalWrite(5 + sensor_loops, LOW);
    Wire.beginTransmission(MPU);
    Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU,14,true); // request a total of 14 registers
    AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
    AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
    Wire.endTransmission(true);
    Serial.print(sensor_loops); Serial.print(" | ");
    Serial.print("AcX = "); Serial.print(AcX);
    Serial.print(" | AcY = "); Serial.print(AcY);
    Serial.print(" | AcZ = "); Serial.print(AcZ);
    //Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
    Serial.print(" | GyX = "); Serial.print(GyX);
    Serial.print(" | GyY = "); Serial.print(GyY);
    Serial.print(" | GyZ = "); Serial.println(GyZ);
    sensor_loops++;
  }
  currentTime = millis();
  voltMeasurement = analogRead(A0);
  currentMeasurement = analogRead(A2);
  timeDelta = currentTime - startTime;
  voltVal = (double) voltMeasurement * 5 * 2 / 1023.0; // Volt, voltage divider halves voltage
  currentVout = (double) currentMeasurement * 5 / 1023.0; // INA169 Vout
  currentVal = (currentVout * 1000.0) * 1000.0 / (1.26 * 10000.0); // mA, Rs = 10 Ohms Rl = 10k Ohms
  powerVal = voltVal * currentVal; // mW
  energyVal = energyVal + ((powerVal / 1000) * ((double) timeDelta / 1000)); // Joules
  Serial.print("Voltage(V) = "); Serial.print(voltVal);
  Serial.print(" | Current(mA) = "); Serial.print(currentVal);
  Serial.print(" | Power(mW) = "); Serial.print(powerVal);
  Serial.print(" | Energy(J) = "); Serial.println(energyVal);
  startTime = currentTime;
  
  delay(500);
}
