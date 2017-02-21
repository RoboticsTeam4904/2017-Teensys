#include <Encoder.h>
#include <FlexCAN.h>
#include <TeensyCANBase.h>

struct encoderData {
  long lastRead;
  long position;
  long rate;
  Encoder* enc;
  uint32_t canID;
};

encoderData leftEncoderData;
encoderData rightEncoderData;

void resetLeftEncoder(byte * msg) {
  if (msg[0] == 0x72 && msg[1] == 0x65 && msg[2] == 0x73 && msg[3] == 0x65 && msg[4] == 0x74 && msg[5] == 0x65 && msg[6] == 0x6e && msg[7] == 0x63) {
    leftEncoderData.enc->write(0);
    leftEncoderData.position = 0;
    leftEncoderData.rate = 0;
    Serial.println("reset");
  }
}

void resetRightEncoder(byte * msg) {
  if (msg[0] == 0x72 && msg[1] == 0x65 && msg[2] == 0x73 && msg[3] == 0x65 && msg[4] == 0x74 && msg[5] == 0x65 && msg[6] == 0x6e && msg[7] == 0x63) {
    rightEncoderData.enc->write(0);
    rightEncoderData.position = 0;
    rightEncoderData.rate = 0;
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
  leftEncoderData.lastRead = 0;
  leftEncoderData.position = -999;
  leftEncoderData.rate = 0;
  leftEncoderData.enc = new Encoder (16, 17);
  leftEncoderData.canID = 0x610;
  CAN_add_id(0x610, &resetLeftEncoder);
  rightEncoderData.lastRead = 0;
  rightEncoderData.position = -999;
  rightEncoderData.rate = 0;
  rightEncoderData.enc = new Encoder (18, 19);
  rightEncoderData.canID = 0x611;
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

void canEncoder(encoderData encData) {
  long newPos = encData.enc->read();
  if (newPos != encData.position) {
    encData.rate = ((double) 1000000.0 * (newPos - encData.position)) / ((double) (micros() - encData.lastRead));
    encData.position = newPos;
    encData.lastRead = micros();
  }
  else {
    if ((micros() - encData.lastRead) > 1000) {
      encData.rate = 0;
    }
  }
  writeLongs (encData.canID, encData.position, encData.rate);
}

void loop() {
  canEncoder (leftEncoderData);
  canEncoder (rightEncoderData);
  delay(10);
}
