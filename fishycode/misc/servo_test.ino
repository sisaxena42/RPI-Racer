/*
 * Servo Example
 */
#include <Servo.h>

Servo s1;             // Instantiate Servo Object
Servo s2;

int pin = 14; 
//int pin2 = 15;        // Pin 14 is Analog 0
int dly = 500;        // Set Delay to 500ms. Change this to change the speed
int hasRun = 0;
void setup() {
  pinMode(pin, OUTPUT);
  s1.attach(pin); 
  //pinMode(pin2, OUTPUT);   // Attach to Analog 0
  //s2.attach(pin2);
  Serial.begin(19200);
  
}

void loop() {
  if(hasRun <= 3) {
  s1.write(0); 
  //s2.write(0);     // Set position 0 degrees
  delay(dly);
  
  s1.write(90); 
  //s2.write(90);    // Set Position 90 degrees
  delay(dly);
  
  s1.write(180);
  //s2.write(180);    // Set Position 180 degrees
  delay(dly);
  
  s1.write(90);
  //s2.write(90);     // Set Position 90 degrees
  delay(dly);
  hasRun++; 
  }
  //s1.detach();
  Serial.println();
}
