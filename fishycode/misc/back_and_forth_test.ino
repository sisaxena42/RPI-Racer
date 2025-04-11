bool hasRun = false;

void setup() {
  esc_1.attach(9);    // PWM pin
  esc_2.attach(11);   // PWM pin
  Serial.begin(19200);
  while (!Serial);    // Wait for serial connection
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
  set_esc_power(0);  delay(500);
  set_esc_power(20); delay(500);
  set_esc_power(40); delay(500);
  set_esc_power(0);  delay(500);
}