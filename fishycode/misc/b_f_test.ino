#include <Servo.h>

Servo esc_1;  // create servo object to control the PWM signal
Servo esc_2;  // for controlling a second motor via a second esc

void setup() {
  esc_1.attach(9);  // make sure to use a PWM capable pin
  esc_2.attach(11);  // make sure to use a PWM capable pin
}

void loop() {  
  set_esc_power(0);
  delay(500);
  set_esc_power(20);
  delay(500);
  set_esc_power(40);
  delay(500);
  // set_esc_power(30);
  // delay(500);
  // set_esc_power(30);
  // delay(500);
  // set_esc_power(10);
  // delay(500);
  // set_esc_power(80);
  // delay(500);
  // set_esc_power(60);
  // delay(500);
  // set_esc_power(40);
  // delay(500);
  // set_esc_power(20);
  // delay(500);
  // set_esc_power(0);
  // delay(500);
}

void set_esc_power (int power){
  power = constrain(power, -100, 100);
  int signal_min = 1050;
  int signal_max = 1950;
  int signal_output = map(power, -100, 100, signal_min, signal_max); //map(value, fromLow, fromHigh, toLow, toHigh)
  esc_1.writeMicroseconds(signal_output);
}
