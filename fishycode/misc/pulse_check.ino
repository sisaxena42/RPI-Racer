const int RX_PIN = 2;

void setup() {
  Serial.begin(19200);
  pinMode(RX_PIN, INPUT);
}

void loop() {
  unsigned long pw = pulseIn(RX_PIN, HIGH, 30000);

  if (pw > 900) {  // filter out 0 values
    Serial.print("Pulse width: ");
    Serial.print(pw);
    Serial.println(" us");
  }

  delay(200);
}