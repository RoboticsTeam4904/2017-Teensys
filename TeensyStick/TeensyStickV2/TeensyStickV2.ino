/* Basic USB Joystick Example
   Teensy becomes a USB joystick

   You must select Joystick from the "Tools > USB Type" menu

   Pushbuttons should be connected to digital pins 0 and 1.
   Wire each button between the digital pin and ground.
   Potentiometers should be connected to analog inputs 0 to 1.

   This example code is in the public domain.
*/

void setup() {
  pinMode(0, INPUT_PULLUP);
  pinMode(1, INPUT_PULLUP);
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);
  pinMode(6, INPUT_PULLUP);
  pinMode(7, INPUT_PULLUP);
  pinMode(8, INPUT_PULLUP);
  pinMode(9, INPUT_PULLUP);
  pinMode(10, INPUT_PULLUP);
  pinMode(11, INPUT_PULLUP);
  pinMode(12, INPUT_PULLUP);
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
  pinMode(15, INPUT_PULLUP);
  pinMode(16, INPUT_PULLUP);
  pinMode(17, INPUT_PULLUP);
  pinMode(18, INPUT_PULLUP);
  pinMode(19, INPUT_PULLUP);
  pinMode(20, INPUT_PULLUP);
  pinMode(21, INPUT_PULLUP);
//  Serial.begin(9600);
//  while(!Serial){}
}

void loop() {

  // read the digital inputs and set the buttons
  Joystick.button(1, not digitalRead(0));
  Joystick.button(2, not digitalRead(1));
  Joystick.button(3, not digitalRead(2));
  Joystick.button(4, not digitalRead(3));
  Joystick.button(5, not digitalRead(4));
  Joystick.button(6, not digitalRead(5));
  Joystick.button(7, not digitalRead(6));
  Joystick.button(8, not digitalRead(7));
  Joystick.button(9, not digitalRead(8));
  Joystick.button(10, not digitalRead(9));
  Joystick.button(11, not digitalRead(10));
  Joystick.button(12, not digitalRead(11));
  Joystick.button(13, not digitalRead(12));
  Joystick.button(14, not digitalRead(13));
  Joystick.button(15, not digitalRead(14));
  Joystick.button(16, not digitalRead(15));
  Joystick.button(17, not digitalRead(16));
  Joystick.button(18, not digitalRead(17));
  Joystick.button(19, not digitalRead(18));
  Joystick.button(20, not digitalRead(19));
  Joystick.button(21, not digitalRead(20));
  Joystick.button(22, not digitalRead(21));
  Joystick.button(23, not digitalRead(22));
  Joystick.button(24, not digitalRead(23));
//  Serial.println(not digitalRead(0));
//  Serial.println(not digitalRead(1));
//  Serial.println(not digitalRead(2));
//  Serial.println(not digitalRead(3));
//  Serial.println(not digitalRead(4));
//  Serial.println(not digitalRead(5));
//  Serial.println(not digitalRead(6));
//  Serial.println(not digitalRead(7));
//  Serial.println(not digitalRead(13));
//  Serial.println(not digitalRead(14));
//  Serial.println(not digitalRead(15));
//  Serial.println(not digitalRead(16));
//  Serial.println(not digitalRead(17));
//  Serial.println(not digitalRead(18));

  // a brief delay, so this runs 20 times per second
  delay(50);
}

