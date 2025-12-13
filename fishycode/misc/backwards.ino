#include <Servo.h>

Servo esc;

const int ESC_PIN  = 9;

void setup() {
  
  esc.attach(ESC_PIN);
  esc.writeMicroseconds(1388); // neutral
  delay(3000);

  esc.writeMicroseconds(1520); // forward
  delay(1000);

  esc.writeMicroseconds(1388); // neutral
  delay(1000);

  esc.writeMicroseconds(1064); // reverse
}

void loop() {}