#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h>
#endif
#include <Encoder.h>
#include <FlexCAN.h>
#include <TeensyCANBase.h>

#define LED_STRIP_PIN 0
#define POWER_LED_PIN 25

const int numberLeds = 87;

Adafruit_NeoPixel ledStrip = Adafruit_NeoPixel(numberLeds, LED_STRIP_PIN, NEO_GRB + NEO_KHZ800);

int ledTeamColor;
int ledMatchState;
float ledWaveDistance;
const float ledWaveWidth = 3.0f;
const float ledWaveSpeedScale = 32768.0f; // Relative to encoder ticks

struct encoderData {
  long lastRead;
  long pos;
  long rate;
};

Encoder leftEncoder(7, 8);
encoderData leftData;
Encoder rightEncoder(9, 10);
encoderData rightData;
Encoder flywheelEncoder(11, 12);
encoderData flywheelData;

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

void resetFlywheelEncoder(byte * msg) {
  if (msg[0] == 0x72 && msg[1] == 0x65 && msg[2] == 0x73 && msg[3] == 0x65 && msg[4] == 0x74 && msg[5] == 0x65 && msg[6] == 0x6e && msg[7] == 0x63) {
    flywheelEncoder.write(0);
    flywheelData.pos = 0;
    flywheelData.rate = 0;
    Serial.println("reset");
  }
}

void setup(){
  Serial.begin(9600); // Begin debugging serial
  ledStrip.begin();
  CAN_begin();
  ledTeamColor = 1; // default to blue
  ledMatchState = 0; // default to disabled
  ledWaveDistance = 0;
  CAN_add_id(0x600, &changeLEDs); // MatchInformer callback
  leftData.lastRead = 0;
  leftData.pos = -999;
  leftData.rate = 0;
  CAN_add_id(0x610, &resetLeftEncoder);
  rightData.lastRead = 0;
  rightData.pos = -999;
  rightData.rate = 0;
  CAN_add_id(0x611, &resetRightEncoder);
  flywheelData.lastRead = 0;
  flywheelData.pos = -999;
  flywheelData.rate = 0;
  CAN_add_id(0x612, &resetFlywheelEncoder);
  pinMode(POWER_LED_PIN, OUTPUT);
  digitalWrite(POWER_LED_PIN, HIGH);
  delay(1000);
  Serial.println("Teensy Online");
}

void writeLongs(uint32_t id, long value1, long value2) {
  byte * msg = new byte[8];

  for (int i = 0; i < 4; i++) {
    msg[i] = (value1 >> i * 8) & 0xFF;
  }
  for (int i = 0; i < 4; i++) {
    msg[i + 4] = (value2 >> i * 8) & 0xFF;
  }

  CAN_write(id, msg);

  delete msg;
}

void loop() {
  CAN_update();

  // LED strip code
  if (ledMatchState == 0) {
    rainbow(10);
  }
  else {
    colorWave(255 - ledTeamColor * 255, 0, ledTeamColor * 255);
  }

  // Encoder code
  long newPos = leftEncoder.read();
  if (newPos != leftData.pos) {
    leftData.rate = ((double) 1000000.0 * (newPos - leftData.pos)) / ((double) (micros() - leftData.lastRead));
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

  ledWaveDistance += (float) rightData.rate / ledWaveSpeedScale;
  if(ledWaveDistance > numberLeds){
    ledWaveDistance -= numberLeds;
  }
  
  newPos = flywheelEncoder.read();
  if (newPos != flywheelData.pos) {
    flywheelData.rate = ((double) 1000000.0 * (newPos - flywheelData.pos)) / ((double) (micros() - flywheelData.lastRead));
    flywheelData.pos = newPos;
    flywheelData.lastRead = micros();
  }
  else {
    if ((micros() - flywheelData.lastRead) > 1000) {
      flywheelData.rate = 0;
    }
  }
  writeLongs(0x612, flywheelData.pos, flywheelData.rate);
  
  delay(10);
}

void colorWave(int R, int G, int B) {
  for (uint32_t i = 0; i < ledStrip.numPixels(); i++) {
    float colorScale = sin(ledWaveDistance + i*ledWaveWidth);
    ledStrip.setPixelColor(i, ledStrip.Color(R * colorScale, G * colorScale, B * colorScale));
  }
  ledStrip.show();
}

void changeLEDs(byte* msg) {
  ledMatchState = msg[2];
  ledTeamColor = msg[0];

  /*Serial.print(" Mode: ");
    Serial.print(ledMatchState);
    Serial.print(" Color: ");
    Serial.println(ledTeamColor);*/
}

void rainbow(uint8_t wait) {
  uint16_t i, j;
  j = (millis() / wait) % 256;
  for (i = 0; i < ledStrip.numPixels(); i++) {
    ledStrip.setPixelColor(i, Wheel((i + j) & 255));
  }
  ledStrip.show();
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if (WheelPos < 85) {
    return ledStrip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if (WheelPos < 170) {
    WheelPos -= 85;
    return ledStrip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return ledStrip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}


