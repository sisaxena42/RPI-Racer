#include <Servo.h>

Servo s1;
int pin = 14;
int dly = 500;
bool hasRun = false;

void setup() {
  pinMode(pin, OUTPUT);
  s1.attach(pin);
  Serial.begin(19200);
  while (!Serial);  // Wait for serial connection
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd == "RUN" && !hasRun) {
      moveServo();
      hasRun = true;
    }
  }
}

void moveServo() {
  s1.write(0); delay(dly);
  s1.write(90); delay(dly);
  s1.write(180); delay(dly);
  s1.write(90); delay(dly);
  Serial.println("Done");
}
