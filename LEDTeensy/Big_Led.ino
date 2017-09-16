
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

int status;
int teamColor;
int flashdelay = 160;
int autonmode;
int mode;
int flywheelSpeed;
int flywheelThresholdLow = 8;
int flywheelThresholdHigh = 3;
int flywheelThresholdMax = 5;

void setup() {
  Serial.begin(9600);
  status = 1;
  teamColor = 1;
  autonmode = 1;
  Serial.setTimeout(5);
  CAN_add_id(0x600, &changeLEDs);
  CAN_begin();
  delay(1000);
  Serial.println("Teensy 3.X CAN Encoder");
}

void loop() {
  CAN_update();

  if (mode == 0) { //disabled
    status = 5;
  }

  if (mode == 2) { //enabled
    status = 1;
  }

  if (mode == 3) { //flywheel on
    if (flywheelSpeed < flywheelThresholdLow) {
      status = 2;
    } else if (flywheelSpeed > flywheelThresholdLow && flywheelSpeed < flywheelThresholdHigh) {
      status = 3;
    } else if (flywheelSpeed > flywheelThresholdHigh && flywheelSpeed < flywheelThresholdMax) {
      status = 4;
    } else {
      status = 7;
    }

  }

  if (mode == 1) { //autonomous period
    status = 6;
  }
  switch (status) {


    case 1: //team color

      if (teamColor == 0) {
        set_Led(255, 0, 0);
      } else {
        set_Led(0, 0, 255);
      }
      break;

    case 2:
      set_Led(255, 0, 0);
      break;

    case 3:
      set_Led(247, 106, 0);
      break;

    case 4:

      if (millis() % 500 > 250) {
        set_Led(0, 255, 0);
      } else {
        set_Led(0, 0, 0);
      }
      break;

    case 5:
      rainbow(10);
      //rainbowCycle(10);
      break;

    case 6:

      if (autonmode == 0) {
        set_Led(255, 0, 255);
      } else {
        set_Led(255, 50, 0);
      }
      break;

    case 7:

      if (millis() % 500 > 250) {
        if (teamColor == 0) {
          set_Led(255, 0, 0);
        } else {
          set_Led(0, 0, 255);
        }
      } else {
        set_Led(0, 0, 0);
      }
      break;
  }
  //status = 4;
  int userinput = Serial.parseInt();
  if (userinput > 0 ) status = userinput;
  //Serial.println(status);
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

void rainbow(uint8_t wait) {
  uint16_t i, j;
  j = (millis() / wait) % 256;
  Wheel(j);
}


// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    set_Led(255-WheelPos*3, 0, WheelPos*3);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    set_Led(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  set_Led(WheelPos * 3, 255 - WheelPos * 3, 0);
}


void changeLEDs(byte* msg) {
  mode = msg[2];
  autonmode = msg[1];
  teamColor = msg[0];
  flywheelSpeed = msg[3] + (msg[4] << 8);
  
  Serial.print(millis());
  Serial.print(" Mode: ");
  Serial.print(mode);
  Serial.print(" Auto: ");
  Serial.print(autonmode);
  Serial.print(" Color: ");
  Serial.println(teamColor);
}

void set_Led(int Red, int Green, int Blue){
  analogWrite(7, Red/8);
  analogWrite(8, Green/8);
  analogWrite(10, Blue/8);
}
