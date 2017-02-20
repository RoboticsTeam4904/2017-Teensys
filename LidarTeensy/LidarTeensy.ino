
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

void setup() {
  Serial.begin(9600);
  Serial.println("Basic Encoder Test:");
  leftEncoderData.lastRead = 0;
  leftEncoderData.position = -999;
  leftEncoderData.rate = 0;
  leftEncoderData.enc = new Encoder (9,10);
  leftEncoderData.canID = 0x610;
  rightEncoderData.lastRead = 0;
  rightEncoderData.position = -999;
  rightEncoderData.rate = 0;
  rightEncoderData.enc = new Encoder (11, 12);
  rightEncoderData.canID = 0x611;
}

void writeLongs(uint32_t id, long value1, long value2){
  byte * msg = new byte[8];

  for(int i = 0; i < 4; i++){
    msg[i] = (value1 >> i*8) & 0xFF;
  }
  for(int i = 0; i < 4; i++){
    msg[i + 4] = (value2 >> i*8) & 0xFF;
  }
  
  CAN_write(id, msg);

  delete msg;
}

void canEncoder (encoderData encData) {
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

long oldPosition  = -999;

void loop() {
canEncoder (leftEncoderData);
canEncoder (rightEncoderData);
delay(10);
}
