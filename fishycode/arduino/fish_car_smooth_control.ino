#include <Servo.h>

// ============================================
// HARDWARE CONFIGURATION
// ============================================

Servo steering_servo;
Servo drive_motor;

const int STEERING_PIN = 14;
const int MOTOR_PIN = 9;

// ============================================
// CAMERA FRAME CONFIGURATION
// ============================================

const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;

// ============================================
// MOTION CONTROL PARAMETERS
// ============================================

// Speed settings (percentage of max power)
const int MAX_MOTOR_SPEED = 7;        // Maximum speed for safety
const int MAX_REV_SPEED = 20;
const int MIN_MOTOR_SPEED = 6;        // Minimum speed to overcome friction
const int IDLE_SPEED = 0;              // Stopped

// ESC calibration (measured from your receiver)
const int ESC_MIN  = 1064;  // full reverse (from backwards.ino)
const int ESC_NEUT = 1388;  // neutral
const int ESC_MAX  = 1530;  // full forward

// Steering settings (servo angles in degrees)
const int STEERING_CENTER = 90;        // Straight ahead
const int STEERING_LEFT = 60;          // Maximum left turn
const int STEERING_RIGHT = 120;        // Maximum right turn
const int STEERING_SLIGHT_LEFT = 75;   // Gentle left turn
const int STEERING_SLIGHT_RIGHT = 105; // Gentle right turn

// Smoothing parameters
const float SPEED_SMOOTHING = 0.15;    // Speed change rate (0.0-1.0, lower = smoother)
const float STEERING_SMOOTHING = 0.20; // Steering change rate (0.0-1.0, lower = smoother)
const int MAX_SPEED_CHANGE = 5;        // Maximum speed change per iteration
const int MAX_STEERING_CHANGE = 3;     // Maximum steering change per iteration (degrees)

// Timing
const unsigned long COMMAND_TIMEOUT = 1000; // ms - stop if no command received
const unsigned long LOOP_DELAY = 20;        // ms - main loop delay

// ============================================
// STATE VARIABLES
// ============================================

// Current smoothed values
float current_speed = 0.0;             // Current motor speed (-100 to 100)
float current_steering = STEERING_CENTER; // Current steering angle

// Target values from fish position
float target_speed = 0.0;
float target_steering = STEERING_CENTER;

// Last command tracking
unsigned long last_command_time = 0;
bool emergency_stop = false;

// ============================================
// SETUP
// ============================================

void setup() {
  // Initialize servos
  steering_servo.attach(STEERING_PIN);
  drive_motor.attach(MOTOR_PIN);
  
  // Initialize serial communication
  Serial.begin(19200);
  while (!Serial);
  
  // Set initial safe state
  steering_servo.write(STEERING_CENTER);
  //setDrivePower(0);
//  drive_motor.writeMicroseconds(ESC_NEUT);
  delay(3000); // allow ESC arm

  
  Serial.println("Fish Car Smooth Control Initialized");
  Serial.println("Waiting for commands...");
  
  last_command_time = millis();
}

// ============================================
// MAIN LOOP
// ============================================

void loop() {
  // Check for incoming commands
  if (Serial.available()) {
    processCommand();
  }
  
  // Check for command timeout
  if (millis() - last_command_time > COMMAND_TIMEOUT) {
    if (!emergency_stop) {
      Serial.println("WARNING: Command timeout - stopping");
      emergency_stop = true;
      target_speed = 0;
      target_steering = STEERING_CENTER;
    }
  }
  
  // Apply smooth transitions
  smoothTransition();
  
  // Apply current values to hardware
  applyMotorControl();
  
  delay(LOOP_DELAY);
}

// ============================================
// COMMAND PROCESSING
// ============================================

void processCommand() {
  String input = Serial.readStringUntil('\n');
  input.trim();
  
  // Expected format: "x,y,speed_factor,boundary_factor,obstacle"
  // Example: "320,240,1.0,0.0,0"
  
  int comma1 = input.indexOf(',');
  if (comma1 == -1) return;
  
  int comma2 = input.indexOf(',', comma1 + 1);
  if (comma2 == -1) return;
  
  int comma3 = input.indexOf(',', comma2 + 1);
  if (comma3 == -1) return;
  
  int comma4 = input.indexOf(',', comma3 + 1);
  if (comma4 == -1) return;
  
  // Parse values
  int x = input.substring(0, comma1).toInt();
  int y = input.substring(comma1 + 1, comma2).toInt();
  float speed_factor = input.substring(comma2 + 1, comma3).toFloat();
  float boundary_factor = input.substring(comma3 + 1, comma4).toFloat();
  int obstacle = input.substring(comma4 + 1).toInt();
  
  // Update last command time
  last_command_time = millis();
  emergency_stop = false;
  
  // Process movement based on fish position and safety factors
  calculateTargetMovement(x, y, speed_factor, boundary_factor, obstacle);
}

// ============================================
// MOVEMENT CALCULATION
// ============================================

void calculateTargetMovement(int x, int y, float speed_factor, float boundary_factor, int obstacle) {
  // If obstacle detected or at boundary, stop immediately
  if (obstacle == 1 || speed_factor < 0.01) {
    target_speed = 0;
    target_steering = STEERING_CENTER;
    return;
  }
  
  // Determine grid position (3x3 grid)
  int col = (x < FRAME_WIDTH / 3) ? 0 : (x < 2 * FRAME_WIDTH / 3) ? 1 : 2;
  int row = (y < FRAME_HEIGHT / 3) ? 0 : (y < 2 * FRAME_HEIGHT / 3) ? 1 : 2;
  
  // Center region (1,1) - idle/slow
  if (col == 1 && row == 1) {
    target_speed = 0;
    target_steering = STEERING_CENTER;
    return;
  }
  
  // Calculate base speed and direction based on row
  float base_speed = 0;
  bool moving_backward = false;
  
  switch (row) {
    case 0:  // Top row - fish is far, move forward
      base_speed = MAX_MOTOR_SPEED;
      moving_backward = false;
      break;
      
    case 1:  // Middle row - move forward slowly
      base_speed = MAX_MOTOR_SPEED * 0.7;
      moving_backward = false;
      break;
      
    case 2:  // Bottom row - fish is close, move backward
      base_speed = -MAX_MOTOR_SPEED;
      moving_backward = true;
      break;
  }
  
  // Apply speed factor (from boundary and other constraints)
  base_speed *= speed_factor;
  
  // Ensure minimum speed if moving
  if (abs(base_speed) > 0 && abs(base_speed) < MIN_MOTOR_SPEED) {
    base_speed = (base_speed > 0) ? MIN_MOTOR_SPEED : -MIN_MOTOR_SPEED;
  }
  
  target_speed = base_speed;
  
  // Debug output
  Serial.print("Grid: (");
  Serial.print(col);
  Serial.print(",");
  Serial.print(row);
  Serial.print(") Speed: ");
  Serial.print(target_speed);
  Serial.print(" Steering: ");
  
  // Calculate steering based on column
  // Use gentler turns for smoother motion
  switch (col) {
    case 0:  // Left column - always turn left
      target_steering = STEERING_SLIGHT_LEFT;
      break;
      
    case 1:  // Center column
      target_steering = STEERING_CENTER;
      break;
      
    case 2:  // Right column - always turn right
      target_steering = STEERING_SLIGHT_RIGHT;
      break;
  }
  
  // Apply sharper turns only if fish is in corner regions
  if (row == 0 || row == 2) {  // Top or bottom row
    if (col == 0) {  // Left corner - always turn left
      target_steering = STEERING_LEFT;
    } else if (col == 2) {  // Right corner - always turn right
      target_steering = STEERING_RIGHT;
    }
  }
  
  Serial.println(target_steering);
}

// ============================================
// SMOOTH TRANSITION
// ============================================

void smoothTransition() {
  // Smooth speed transition
  float speed_diff = target_speed - current_speed;
  float speed_change = speed_diff * SPEED_SMOOTHING;
  
  // Limit maximum change per iteration
  if (abs(speed_change) > MAX_SPEED_CHANGE) {
    speed_change = (speed_change > 0) ? MAX_SPEED_CHANGE : -MAX_MOTOR_SPEED;
  }
  
  current_speed += speed_change;
  
  // Smooth steering transition
  float steering_diff = target_steering - current_steering;
  float steering_change = steering_diff * STEERING_SMOOTHING;
  
  // Limit maximum change per iteration
  if (abs(steering_change) > MAX_STEERING_CHANGE) {
    steering_change = (steering_change > 0) ? MAX_STEERING_CHANGE : -MAX_STEERING_CHANGE;
  }
  
  current_steering += steering_change;
  
  // Constrain values to safe ranges
  current_speed = constrain(current_speed, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
  current_steering = constrain(current_steering, STEERING_LEFT, STEERING_RIGHT);
}

// ============================================
// MOTOR CONTROL
// ============================================

void applyMotorControl() {
  // Apply steering
  steering_servo.write((int)current_steering);
  
  // Apply drive power
  setDrivePower((int)current_speed);
}

void setDrivePower(int power) {
  // Constrain power to safe range
  power = constrain(power, -100, 100);
  
  // Map power (-100 to 100) to ESC microseconds using calibrated values
  // ESC_NEUT (1388) is neutral/stopped
  int signal;
  if (power == 0) {
    signal = ESC_NEUT;
  } else if (power > 0) {
    // Forward: map 1 to 100 => ESC_NEUT to ESC_MAX
    signal = map(power, 0, 100, ESC_NEUT, ESC_MAX);
  } else {
    // Reverse: map -100 to -1 => ESC_MIN to ESC_NEUT
    signal = map(power, -100, 0, ESC_MIN, ESC_NEUT);
  }
  
  drive_motor.writeMicroseconds(signal);
}
//int powerToPulse(int power) {
//  power = constrain(power, -100, 100);
//  if (power == 0) return ESC_NEUT;
//
//  if (power > 0) {
//    // 0..100 => ESC_NEUT..ESC_MAX
//    long up = ESC_MAX - ESC_NEUT;
//    return ESC_NEUT + (up * power) / 100;
//  } else {
//    // -100..0 => ESC_MIN..ESC_NEUT
//    long down = ESC_NEUT - ESC_MIN;
//    int mag = -power;
//    return ESC_NEUT - (down * mag) / 100;
//  }
//}
//
//void setDrivePower1(int power) {
//  static bool wasForward = false;
//  static unsigned long lastNeutralTime = 0;
//
//  power = constrain(power, -100, 100);
//
//  // Track neutral time
//  if (power == 0) {
//    lastNeutralTime = millis();
//    drive_motor.writeMicroseconds(ESC_NEUT);
//    wasForward = false; // treat as reset
//    return;
//  }
//
//  // If requesting reverse, enforce "neutral gap" first (car ESC safety)
//  if (power < 0) {
//    // If we recently were forward OR haven?t held neutral long enough, force neutral first
//    if (wasForward || (millis() - lastNeutralTime < NEUTRAL_BEFORE_REVERSE_MS)) {
//      drive_motor.writeMicroseconds(ESC_NEUT);
//      return; // wait until neutral has been held long enough
//    }
//  }
//
//  // If requesting forward, mark forward state
//  if (power > 0) {
//    wasForward = true;
//  }
//
//  // Output pulse
//  int signal = powerToPulse(power);
//  drive_motor.writeMicroseconds(signal);
//}


// ============================================
// UTILITY FUNCTIONS
// ============================================

void emergencyStop() {
  current_speed = 0;
  target_speed = 0;
  current_steering = STEERING_CENTER;
  target_steering = STEERING_CENTER;
  
  steering_servo.write(STEERING_CENTER);
  setDrivePower(0);
  
  Serial.println("EMERGENCY STOP ACTIVATED");
}