#include <Servo.h>

Servo esc;

const int ESC_PIN  = 9;

// Your measured values from the receiver:
const int ESC_MIN  = 1235;  // reverse
const int ESC_NEUT = 1475;  // neutral
const int ESC_MAX  = 1736;  // forward

void setup() {
  Serial.begin(19200);
  esc.attach(ESC_PIN);

  // Start at NEUTRAL
  esc.writeMicroseconds(ESC_NEUT);
  Serial.println("Send: NEUTRAL");
}

void loop() {
  // Wait for user input via serial
  if (Serial.available()) {
    char c = Serial.read();

    if (c == 'f') {
      esc.writeMicroseconds(ESC_MAX);
      Serial.println("Send: FULL FORWARD");
    }
    if (c == 'n') {
      esc.writeMicroseconds(ESC_NEUT);
      Serial.println("Send: NEUTRAL");
    }
    if (c == 'r') {
      esc.writeMicroseconds(ESC_MIN);
      Serial.println("Send: FULL REVERSE");
    }
  }
}
