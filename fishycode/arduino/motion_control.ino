
#include <SoftwareSerial.h>


// Define pins for the car control
const int motorPin1 = 9;  // Example pin for motor 1 (adjust accordingly)
const int motorPin2 = 10; // Example pin for motor 2 (adjust accordingly)




void setup() {
 // put your setup code here, to run once:
Serial.begin(9600); // Use Serial1 to communicate with Raspberry Pi via UART (TX/RX)


 // Initialize motor control pins (adjust according to your motor driver)
 pinMode(motorPin1, OUTPUT);
 pinMode(motorPin2, OUTPUT);
}


void loop() {
 // put your main code here, to run repeatedly:
if (Serial.available()) {
   int position = Serial.parseInt(); // Read position sent by Pi over UART
  
   // Assuming position is a number between 0-8 corresponding to the quadrants
   controlCarMovement(position);
 }
}


void controlCarMovement(int position) {
 // Define motor control behavior based on position (example logic)
 if (position == 4) {
   // Move forward if in the center (position 4)
   digitalWrite(motorPin1, HIGH);
   digitalWrite(motorPin2, LOW);
 }
 else if (position == 2 || position == 1 || position == 3) {
   // Turn right if in the right half (positions 2, 1, 3)
   digitalWrite(motorPin1, HIGH); // Motor 1 forward
   digitalWrite(motorPin2, HIGH);  // Motor 2 also forward (adjust turning speed as needed)
 }
 else if (position == 6 || position == 7 || position == 5) {
   // Turn left if in the left half (positions 6, 7, 5)
   digitalWrite(motorPin1, LOW);   // Motor 1 reverse
   digitalWrite(motorPin2, HIGH);  // Motor 2 forward (adjust turning speed as needed)
 }
 // Additional logic for forward/backward or other control based on positions
}
