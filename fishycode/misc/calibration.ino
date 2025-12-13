#include <Servo.h>

Servo esc;
Servo servo;

const int ESC_PIN  = 9;
const int SERVO_PIN = 14;

// Your measured values from the receiver:
const int ESC_MIN  = 1050;  // reverse
const int ESC_NEUT = 1475;  // neutral
const int ESC_MAX  = 1530;  // forward

void setup() {
  Serial.begin(19200);
  esc.attach(ESC_PIN);
  servo.attach(SERVO_PIN);
  

  // Start at NEUTRAL
  esc.writeMicroseconds(ESC_NEUT);
  servo.writeMicroseconds(1500);
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
      servo.writeMicroseconds(1500);
      Serial.println("Send: NEUTRAL");
    }
    if (c == 'b') {
      esc.writeMicroseconds(ESC_MIN);
      Serial.println("Send: FULL REVERSE");
    }
    if (c == 'l') {
      servo.writeMicroseconds(1300);
      Serial.println("Send: TURN LEFT");
    }
    if (c == 'r') {
       servo.writeMicroseconds(1800);
      Serial.println("Send: TURN RIGHT");
    }
    
  }
}