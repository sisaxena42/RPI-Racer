#include <Servo.h>

Servo steering_servo;
Servo drive_motor;

const int STEERING_PIN = 14;
const int MOTOR_PIN = 9;

const int FRAME_WIDTH  = 1280;
const int FRAME_HEIGHT = 1080;

const int MOTOR_SPEED = 40;

void setup() {
  steering_servo.attach(STEERING_PIN);
  drive_motor.attach(MOTOR_PIN);
  Serial.begin(19200);
  while (!Serial);
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    int commaIdx = input.indexOf(',');
    if (commaIdx == -1) return;

    int x = input.substring(0, commaIdx).toInt();
    int y = input.substring(commaIdx + 1).toInt();

    // Determine region and act
    handleMovement(x, y);
  }
}

void handleMovement(int x, int y) {
  int col = (x < FRAME_WIDTH / 3) ? 0 : (x < 2 * FRAME_WIDTH / 3) ? 1 : 2;
  int row = (y < FRAME_HEIGHT / 3) ? 0 : (y < 2 * FRAME_HEIGHT / 3) ? 1 : 2;

  if (col == 1 && row == 1) {
    // Center region → Idle
    setDrivePower(0);
    steering_servo.write(90);
    return;
  }

  // Movement mapping
  switch (row) {
    case 0:  // Top row
      setDrivePower(MOTOR_SPEED);
      steerByColumn(col, true); // true = forward turn
      break;
    case 1:  // Middle row
      setDrivePower(MOTOR_SPEED);
      steerByColumn(col, false);
      break;
    case 2:  // Bottom row
      setDrivePower(-MOTOR_SPEED);
      steerByColumn(col, false);
      break;
  }
}

void steerByColumn(int col, bool topRow) {
  switch (col) {
    case 0: steering_servo.write(60); break;  // Left
    case 1: steering_servo.write(90); break;  // Center
    case 2: steering_servo.write(120); break; // Right
  }
}

void setDrivePower(int power) {
  power = constrain(power, -100, 100);
  int signal = map(power, -100, 100, 1050, 1950);
  drive_motor.writeMicroseconds(signal);
}
