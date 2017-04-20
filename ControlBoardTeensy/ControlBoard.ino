void setup() {
  for (int i = 0; i < 22; i++) {
    pinMode(i, INPUT_PULLUP);
  }
}

void loop() {
  // slider
  Joystick.Y(analogRead(0));

  // 6 buttons, 8 switches. Note that buttons 7–22 are switches, each 2 a pair.
  for (int i = 0; i < 22; i++) {
    Joystick.button(i+1, digitalRead(i));
  }

  // a brief delay, so this runs 20 times per second
  delay(50);
}
