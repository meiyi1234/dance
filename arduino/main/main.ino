int inByte = 0;
byte msg[256];
//byte message[] = {0x7E, 0x03, 0x02, 0xBB, 0x06, 0x2E, 0x2E, 0x2E, 0x2E, 0x2E,0x2E, 0x2E, 0x2E, 0x2E, 0x2E,0x2E, 0x2E, 0x2E, 0x2E, 0x2E, 0x2E, 0x2E, 0x2E, 0x2E, 0x2E, 0x2E, 0x2E, 0x2E, 0x2E, 0x2E, 0x2E, 0x2E, 0x2E, 0x2E, 0x01, 0x01, 0x03, 0x7E };
bool is_stop_byte = false;
int i = -1;

typedef enum Frame { IFrame=0, SFrame=1, HFrame=3 } Frame;

bool is_hframe_valid(byte* buf);
uint16_t crc16(uint8_t const *buf, int len);
bool wait_for_handshake();
Frame get_frame_sort(byte* buf);


void setup() {
  Serial.begin(9600);  // pc
//  Serial.begin(9600); // rpi
  Serial.println("Ready");
}

void loop() {
    if (Serial.available() > 0) {
      i++;
      msg[i] = Serial.read();

      if (i == 0 && msg[i] != 0x7e) {
        Serial.print("Error, frame doesnt start with 0x7e");
        i = -1;
        is_stop_byte = false;
        memset(msg, NULL, 1);
      }

      if (msg[i] == 0x7e && !is_stop_byte) {
        is_stop_byte = true;  // this was start, next 0x7e will be stop byte
      }
      else if (msg[i] == 0x7e && is_stop_byte) {
        Frame sort = get_frame_sort(msg);
        if (sort == HFrame) {
          if (!is_hframe_valid(msg)) {
            Serial.println("H-frame invalid, not doing anything");
            return;
          }
          
          // If handshake, repeat message back to primary
          Serial.write("Returning bytes: ");
          Serial.write(msg, i+1);
          Serial.write("\n");
          Serial.write(msg, i+1);
        }
        
        memset(msg, NULL, i+1);
        i = -1;
        is_stop_byte = false;
      }
      else {
        Serial.write("byte is ");
        Serial.write(msg[i]);
        Serial.println("");
      }
      
      delay(10);
    }
}

Frame get_frame_sort(byte* buf) {
  uint8_t sort = buf[2] & 0x03;
  if (sort == 0) {
    return IFrame;
  }
  else if (sort == 1) {
    return SFrame;
  }
  else if (sort == 3) {
    return HFrame;
  }
}

bool is_hframe_valid(byte* buf) {
  uint8_t send_seq = ((uint8_t) buf[1]) >> 1;
  uint8_t control2 = (uint8_t) buf[2];
  uint16_t checksum = (buf[3] << 8) | buf[4];

  return control2 == 3 && checksum == crc16(&buf[1], 2);  // crc 2nd and 3rd bytes. note: hshake frame crc is 0x6530
}

uint16_t crc16(uint8_t const *buf, int len) {
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
