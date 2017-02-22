#include <Encoder.h>
#include <FlexCAN.h>
#include <TeensyCANBase.h>

struct encoderData {
  long lastRead;
  long pos;
  long rate;
};

Encoder leftEncoder(16, 17);
encoderData leftData;
Encoder rightEncoder(18, 19);
encoderData rightData;

void resetLeftEncoder(byte * msg) {
  if (msg[0] == 0x72 && msg[1] == 0x65 && msg[2] == 0x73 && msg[3] == 0x65 && msg[4] == 0x74 && msg[5] == 0x65 && msg[6] == 0x6e && msg[7] == 0x63) {
    leftEncoder.write(0);
    leftData.pos = 0;
    leftData.rate = 0;
    Serial.println("reset");
  }
}

void resetRightEncoder(byte * msg) {
  if (msg[0] == 0x72 && msg[1] == 0x65 && msg[2] == 0x73 && msg[3] == 0x65 && msg[4] == 0x74 && msg[5] == 0x65 && msg[6] == 0x6e && msg[7] == 0x63) {
    rightEncoder.write(0);
    rightData.pos = 0;
    rightData.rate = 0;
    Serial.println("reset");
  }
}

void setup() {
  Serial.begin(9600);
  delay(1000);
  Serial.println("Basic Encoder Test:");
  pinMode(24, OUTPUT);
  pinMode(25, OUTPUT);
  digitalWrite(24, HIGH);
  CAN_begin();
  leftData.lastRead = 0;
  leftData.pos = -999;
  leftData.rate = 0;
  CAN_add_id(0x610, &resetLeftEncoder);
  rightData.lastRead = 0;
  rightData.pos = -999;
  rightData.rate = 0;
  CAN_add_id(0x611, &resetRightEncoder);
}

void writeLongs(uint32_t id, long value1, long value2) {
  byte * msg = new byte[8];

  for (int i = 0; i < 4; i++) {
    msg[i] = (value1 >> i * 8) & 0xFF;
  }
  for (int i = 0; i < 4; i++) {
    msg[i + 4] = (value2 >> i * 8) & 0xFF;
  }

  digitalWrite(25, HIGH);
  CAN_write(id, msg);
  digitalWrite(25, LOW);

  delete msg;
}

void loop() {
  long newPos = leftEncoder.read();
  if (newPos != leftData.pos) {
    leftData.rate = ((double) 1000000.0 * (newPos - leftData.pos)) / ((double) (micros() - leftData.lastRead));
    Serial.println(leftData.rate);
    leftData.pos = newPos;
    leftData.lastRead = micros();
  }
  else {
    if ((micros() - leftData.lastRead) > 1000) {
      leftData.rate = 0;
    }
  }
  writeLongs(0x610, leftData.pos, leftData.rate);
  newPos = rightEncoder.read();
  if (newPos != rightData.pos) {
    rightData.rate = ((double) 1000000.0 * (newPos - rightData.pos)) / ((double) (micros() - rightData.lastRead));
    rightData.pos = newPos;
    rightData.lastRead = micros();
  }
  else {
    if ((micros() - rightData.lastRead) > 1000) {
      rightData.rate = 0;
    }
  }
  writeLongs(0x611, rightData.pos, rightData.rate);
  delay(10);
}
