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
void set_esc_power (int power){
  power = constrain(power, -100, 100);
  int signal_min = 1050;
  int signal_max = 1950;
  int signal_output = map(power, -100, 100, signal_min, signal_max); //map(value, fromLow, fromHigh, toLow, toHigh)
  esc_1.writeMicroseconds(signal_output);
}