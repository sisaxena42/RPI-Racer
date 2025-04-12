#include <Servo.h>

Servo steering_servo;
Servo drive_motor;

const int STEERING_PIN = 14;
const int MOTOR_PIN = 9;

const int CIRCLE_DURATION_MS = 3000;  // Adjust based on how tight you want the circle
const int DRIVE_POWER = 40;           // Adjust power level (0–100)
const int TURN_ANGLE = 60;            // 90 is straight, lower is left, higher is right

bool hasRun = false;

void setup() {
  steering_servo.attach(STEERING_PIN);
  drive_motor.attach(MOTOR_PIN);

  Serial.begin(19200);
  while (!Serial);
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd == "RUN" && !hasRun) {
      driveInCircle();
      hasRun = true;
      Serial.println("Done");
    }
  }
}

void driveInCircle() {
  // Turn the wheels
  steering_servo.write(TURN_ANGLE);  // Example: 60 for left turn

  // Start driving
  set_esc_power(DRIVE_POWER);
  delay(CIRCLE_DURATION_MS);

  // Stop the car
  set_esc_power(0);
  steering_servo.write(90);  // Re-center steering
}

void set_esc_power(int power) {
  power = constrain(power, -100, 100);
  int signal_min = 1050;
  int signal_max = 1950;
  int signal_output = map(power, -100, 100, signal_min, signal_max);
  drive_motor.writeMicroseconds(signal_output);
}
