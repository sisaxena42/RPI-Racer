#include <Servo.h>

Servo esc_1;
bool hasRun = false;

void setup() {
  Serial.begin(19200);
  while (!Serial); // wait for serial

  esc_1.attach(8);  // ESC signal

  // ARMING SEQUENCE
  esc_1.writeMicroseconds(1500); // neutral
  delay(2000);                   // wait for ESC to arm (listen for beeps)
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd == "RUN" && !hasRun) {
      run_esc_test();
      hasRun = true;
      Serial.println("Done");
    }
  }
}

void run_esc_test() {
  // start at neutral, then gentle forward only for now
  set_esc_power(0);   delay(1000);
  set_esc_power(20);  delay(1000);
  set_esc_power(50);  delay(1000);
  set_esc_power(80);  delay(1000);
  set_esc_power(0);   delay(1000);
}

void set_esc_power(int power) {
  power = constrain(power, -100, 100);
  int signal_min = 1000;
  int signal_max = 2000;
  int signal_output = map(power, -100, 100, signal_min, signal_max);
  esc_1.writeMicroseconds(signal_output);
}
