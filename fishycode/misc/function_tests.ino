#include <Servo.h>

Servo esc;
Servo steer;

void setup() {
  esc.attach(9);      // motor ESC signal
  steer.attach(14);   // steering servo signal

  esc.writeMicroseconds(1500);  // neutral
  steer.writeMicroseconds(1500); // center
  delay(2500);                  // let ESC arm
}

void loop() {
  // example: small forward, center steering
  esc.writeMicroseconds(1553);
  steer.writeMicroseconds(1500);
  delay(2000);

  // turn left
  steer.writeMicroseconds(1300);
  delay(2000);

  // turn right
  steer.writeMicroseconds(1700);
  delay(2000);

  // stop
  esc.writeMicroseconds(1500);
  steer.writeMicroseconds(1500);
  delay(2000);
}