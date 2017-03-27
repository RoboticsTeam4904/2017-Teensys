#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h>
#endif
#include <FlexCAN.h>
#include <TeensyCANBase.h>

#define PIN 12



// Parameter 1 = number of pixels in strip
// Parameter 2 = Arduino pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(87, PIN, NEO_GRB + NEO_KHZ800);

int teamColor;
int flashdelay = 160;
int mode;
int flywheelSpeed;
int encoderLow;
int encoderHigh;
int flywheelThresholdLow = 8;
int flywheelThresholdHigh = 3;
int flywheelThresholdMax = 5;

void setup() {
  Serial.begin(9600);

  strip.begin();
  teamColor = 1;
  mode = 0;
  Serial.setTimeout(5);
  CAN_add_id(0x600, &changeLEDs);
  CAN_add_id(0x02041480 | 6, &flywheel);
  CAN_begin();
  pinMode(9, OUTPUT);
  digitalWrite(9, HIGH);
  delay(1000);
  Serial.println("Teensy 3.X CAN Encoder");
}

void loop() {
  CAN_update();

  if(mode == 0){ // Disabled
    rainbow(10);
  }
  else {
    colorFull(strip.Color(255 - teamColor*255, 0, teamColor*255));
  }
  delay(10);
}


/* int threshold = random(7) + 30;
  for (uint32_t i = 0; i < strip.numPixels(); i++) {
  if (i < threshold) {
    strip.setPixelColor(i, strip.Color(0, 50, 0));
  } else {
    if (i < 50) strip.setPixelColor(i, strip.Color(150, 20, 0));
    else if (i<53) strip.setPixelColor(i, strip.Color(0, 50, 0));
    else strip.setPixelColor(i, strip.Color(150, 0, 0));
  }
  }
  delay(100);
  strip.show(); */


void colorWipe(uint32_t c, uint8_t wait) {
  for (uint16_t i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
    strip.show();
    delay(wait);
  }
}

void colorFull(uint32_t color) {
  for (uint32_t i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, color);
  }
  strip.show();
}

void rainbowCycle(uint8_t wait) {
  uint16_t i, j;

  j = (millis() / wait) % 256;
  for (i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
  }
  strip.show();
}

void rainbow(uint8_t wait) {
  uint16_t i, j;
  j = (millis() / wait) % 256;
  for (i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, Wheel((i + j) & 255));
  }
  strip.show();
}


// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}


void changeLEDs(byte* msg) {
  mode = msg[2];
  teamColor = msg[0];
  
  Serial.print(millis());
  Serial.print(" Mode: ");
  Serial.print(mode);
  Serial.print(" Color: ");
  Serial.println(teamColor);
}

// For reference,
// https://github.com/wpilibsuite/allwpilib/blob/99b2b65148e7ea4b49d06f31bde5cd09b4f25119/hal/lib/Athena/ctre/CanTalonSRX.cpp
void flywheel(byte* msg) {
  encoderHigh = msg[4];
  encoderLow = msg[5];
  flywheelSpeed = (encoderHigh << 8) | encoderLow;
  flywheelSpeed <<= 16; // Apparently this does a sign extend
  flywheelSpeed >>= 16;
  Serial.print(flywheelSpeed);
}


