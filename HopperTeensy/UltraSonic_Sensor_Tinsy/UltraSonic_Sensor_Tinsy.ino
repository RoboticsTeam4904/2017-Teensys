#include <Encoder.h>
#include <TeensyCANBase.h>
#include <FlexCAN.h>

const int trigPinLeft = 12;
const int echoPinLeft = 11;
const int trigPinRight = 2;
const int echoPinRight = 1;
float durationLeft;
float durationRight;
long pos;
long rate;
void setup() {
  // put your setup code here, to run once:
  CAN_begin();
  pinMode(trigPinLeft, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPinLeft, INPUT);
  pinMode(trigPinRight, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPinRight, INPUT);
  Serial.begin(9600);
}

int i = 0;
void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(trigPinLeft, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPinLeft, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinLeft, LOW);
  durationLeft = pulseIn(echoPinLeft, HIGH);
  long distanceLeft = round(durationLeft * 3.4 / 2.0);
  Serial.println("dist: ");
  Serial.println(distanceLeft);
  Serial.println("i: ");
  Serial.println(i);
  i = i + 1;
  pos = distanceLeft;
  Serial.println("pos (after 10 lines)");
  Serial.println(pos);
  
//  digitalWrite(trigPinRight, LOW);
//  delayMicroseconds(2);
//  digitalWrite(trigPinRight, HIGH);
//  delayMicroseconds(10);
//  digitalWrite(trigPinRight, LOW);
//  durationRight = pulseIn(echoPinRight, HIGH);
//  long distanceRight = Math.round(durationRight * 3.4 / 2.0); //multiplied by 100 for storage as an int
//  Serial.println(distanceRight);
//  rate = distanceRight;
    rate = 0.0;

  CAN_update();

  uint8_t * data = (uint8_t *) malloc(8);

    data[0] = pos & 0xff;
    data[1] = (pos >> 8) & 0xff;
    data[2] = (pos >> 16) & 0xff;
    data[3] = (pos >> 24) & 0xff;
    Serial.println("pos: ");
    Serial.println(pos);

    data[4] = rate & 0xff;
    data[5] = (rate >> 8) & 0xff;
    data[6] = (rate >> 16) & 0xff;
    data[7] = (rate >> 24) & 0xff;
    Serial.println(rate);

  CAN_write(0x614, data);
  delete data;
}
