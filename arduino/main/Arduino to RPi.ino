#include <Arduino_FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>

QueueHandle_t xQueue0;
SemaphoreHandle_t UninterruptedReadSemaphore = NULL;  //Ensures ReadAcce run properly without SendAcce running
SemaphoreHandle_t BlockReadSemaphore = NULL;          //Stops ReadAcce from continuously running
TickType_t prevWakeTimeRead;
TickType_t prevWakeTimeSend;

const int xpin0 = A0;
const int ypin0 = A1;
const int zpin0 = A2;

const int xpin1 = A5;
const int ypin1 = A6;
const int zpin1 = A7;

const int xpin2 = A8;
const int ypin2 = A9;
const int zpin2 = A10;

const int xpin3 = A13;
const int ypin3 = A14;
const int zpin3 = A15;

int readByte = 0;

void establishContact() {
  bool handshake = false;
  bool ack = false;
  while (!handshake) {
    if (Serial.available() && Serial.read() == 'H') {
      handshake = true;
    }
  }
  while (!ack) {  // Requesting for new set of data
    Serial.write('A');
    if (Serial.available() && Serial.read() == 'A') {
       ack = true;
    }
    delay(10);
  }
  Serial.write('\n');
  readByte = 'A';
}

void setup() {
  // initialize serial communication at 38400 bits per second:
  Serial.begin(38400);
  Serial.println("Start up");
  establishContact();
  Serial.println("Finish contact");
  UninterruptedReadSemaphore = xSemaphoreCreateMutex();
  BlockReadSemaphore = xSemaphoreCreateBinary();
  xQueue0 = xQueueCreate(36, sizeof(int));
  xSemaphoreGive(UninterruptedReadSemaphore);
  xSemaphoreGive(BlockReadSemaphore);
  xTaskCreate(ReadAcce, "ReadAcce", 2000, NULL, 2, NULL);
  xTaskCreate(SendAcce, "SendAcce", 2500, NULL, 2, NULL);
  vTaskStartScheduler();
}

void loop() {}

// Reads in the accelerometer values
void ReadAcce(void *pvParameters) {
  int x0, y0, z0, x1, y1, z1, x2, y2, z2, x3, y3, z3;
  prevWakeTimeRead = xTaskGetTickCount();
  while (true) {
    if (xSemaphoreTake(BlockReadSemaphore, 0) == pdTRUE) {
      if (xSemaphoreTake(UninterruptedReadSemaphore, 0) == pdTRUE) {

        x0 = analogRead(xpin0);
        delay(1);
        y0 = analogRead(ypin0);
        delay(1);
        z0 = analogRead(zpin0);
        delay(1);

        x1 = analogRead(xpin1);
        delay(1);
        y1 = analogRead(ypin1);
        delay(1);
        z1 = analogRead(zpin1);
        delay(1);

        x2 = analogRead(xpin2);
        delay(1);
        y2 = analogRead(ypin2);
        delay(1);
        z2 = analogRead(zpin2);
        delay(1);

        x3 = analogRead(xpin3);
        delay(1);
        y3 = analogRead(ypin3);
        delay(1);
        z3 = analogRead(zpin3);

        Serial.println("Read");
        xQueueSendToBack(xQueue0, &x0, 0);
        xQueueSendToBack(xQueue0, &y0, 0);
        xQueueSendToBack(xQueue0, &z0, 0);
        xQueueSendToBack(xQueue0, &x1, 0);
        xQueueSendToBack(xQueue0, &y1, 0);
        xQueueSendToBack(xQueue0, &z1, 0);
        xQueueSendToBack(xQueue0, &x2, 0);
        xQueueSendToBack(xQueue0, &y2, 0);
        xQueueSendToBack(xQueue0, &z2, 0);
        xQueueSendToBack(xQueue0, &x3, 0);
        xQueueSendToBack(xQueue0, &y3, 0);
        xQueueSendToBack(xQueue0, &z3, 0);

        vTaskDelayUntil(&prevWakeTimeRead, (4 / portTICK_PERIOD_MS));
        xSemaphoreGive(UninterruptedReadSemaphore);
      }
    }
  }
}

// Reads the accelerometer values from the Queue
void SendAcce(void *pvParameters) {
  int x0Rx, y0Rx, z0Rx, x1Rx, y1Rx, z1Rx, x2Rx, y2Rx, z2Rx, x3Rx, y3Rx, z3Rx;
  char x0Char[4], y0Char[4], z0Char[4], x1Char[4], y1Char[4], z1Char[4], x2Char[4], y2Char[4], z2Char[4], x3Char[4], y3Char[4], z3Char[4];
  int frameNum = 0;
  char frameNumChar[4];
  char messageStr[256];
  unsigned int len;
  bool sendFlag = false;
  char checkSum = 0;

  prevWakeTimeSend = xTaskGetTickCount();
  while (true) {
    if (xSemaphoreTake(UninterruptedReadSemaphore, 0) == pdTRUE) {
      if (Serial.available() > 0) {
        readByte = Serial.read();
      }
      if (readByte == 'A') {
        sendFlag = false;
        xSemaphoreGive(BlockReadSemaphore);
        xSemaphoreGive(UninterruptedReadSemaphore);
        vTaskDelayUntil(&prevWakeTimeSend, (1 / portTICK_PERIOD_MS));
        xQueueReceive(xQueue0, &x0Rx, 0);
        xQueueReceive(xQueue0, &y0Rx, 0);
        xQueueReceive(xQueue0, &z0Rx, 0);
        xQueueReceive(xQueue0, &x1Rx, 0);
        xQueueReceive(xQueue0, &y1Rx, 0);
        xQueueReceive(xQueue0, &z1Rx, 0);
        xQueueReceive(xQueue0, &x2Rx, 0);
        xQueueReceive(xQueue0, &y2Rx, 0);
        xQueueReceive(xQueue0, &z2Rx, 0);
        xQueueReceive(xQueue0, &x3Rx, 0);
        xQueueReceive(xQueue0, &y3Rx, 0);
        xQueueReceive(xQueue0, &z3Rx, 0);

        Serial.println("Receive");

        // Create messageStr char[]
        itoa(frameNum, frameNumChar, 10);
        strcpy(messageStr, frameNumChar);
        strcat(messageStr, ",");
        // A0
        itoa(x0Rx, x0Char, 10);    //convert int x0Rx to a string / char[] x0Char, with a radix of base 10 (decimal)
        strcat(messageStr, x0Char);
        strcat(messageStr, ",");
        itoa(y0Rx, y0Char, 10);
        strcat(messageStr, y0Char);
        strcat(messageStr, ",");
        itoa(z0Rx, z0Char, 10);
        strcat(messageStr, z0Char);
        strcat(messageStr, ",");
        // A1
        itoa(x1Rx, x1Char, 10);    //convert int to string / char[]
        strcat(messageStr, x1Char);
        strcat(messageStr, ",");
        itoa(y1Rx, y1Char, 10);
        strcat(messageStr, y1Char);
        strcat(messageStr, ",");
        itoa(z1Rx, z1Char, 10);
        strcat(messageStr, z1Char);
        strcat(messageStr, ",");
        // A2
        itoa(x2Rx, x2Char, 10);    //convert int to string / char[]
        strcat(messageStr, x2Char);
        strcat(messageStr, ",");
        itoa(y2Rx, y2Char, 10);
        strcat(messageStr, y2Char);
        strcat(messageStr, ",");
        itoa(z2Rx, z2Char, 10);
        strcat(messageStr, z2Char);
        strcat(messageStr, ",");
        // A3
        itoa(x3Rx, x3Char, 10);    //convert int to string / char[]
        strcat(messageStr, x3Char);
        strcat(messageStr, ",");
        itoa(y3Rx, y3Char, 10);
        strcat(messageStr, y3Char);
        strcat(messageStr, ",");
        itoa(z3Rx, z3Char, 10);
        strcat(messageStr, z3Char);
        strcat(messageStr, ",");

        len = strlen(messageStr);

        for (int i = 0; i < len; i++) {
          checkSum ^= messageStr[i];
        }

        messageStr[len] = checkSum;
        messageStr[len + 1] = '\n';
//        Serial.println(0b1111 % 0b100);
        for (int j = 0; j < len + 2; j++) {
          Serial.write(messageStr[j]);
        }
        sendFlag = true;
        frameNum++;
        readByte = 0;
        checkSum = 0;
      }
      else if (readByte == 'R') {     //Resend message
        for (int k = 0; k < len + 2; k++) {
          Serial.write(messageStr[k]);
        }
        sendFlag = true;
        readByte = 0;
        vTaskDelayUntil(&prevWakeTimeSend, (2 / portTICK_PERIOD_MS));
        xSemaphoreGive(UninterruptedReadSemaphore);
      }
      else if (readByte == 'H') {
        frameNum = 0;
        sendFlag = false;
        readByte = 0;
        checkSum = 0;
        establishContact();  
        vTaskDelayUntil(&prevWakeTimeSend, (2 / portTICK_PERIOD_MS));      
        xSemaphoreGive(BlockReadSemaphore);
        xSemaphoreGive(UninterruptedReadSemaphore);
      }
      else {
        vTaskDelayUntil(&prevWakeTimeSend, (2 / portTICK_PERIOD_MS));
        xSemaphoreGive(UninterruptedReadSemaphore);
      }
    }
  }
}
