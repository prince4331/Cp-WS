// Simple motor test for BTS7960 motor drivers
// This will cycle motors forward, backward, and stop

#include <Arduino.h>

#define MOTOR_RIGHT_RPWM  6  // Right motor forward
#define MOTOR_RIGHT_LPWM  7  // Right motor backward
#define MOTOR_LEFT_RPWM   4  // Left motor forward
#define MOTOR_LEFT_LPWM   5  // Left motor backward

void handleCommand(char cmd);

void setup() {
  Serial.begin(115200);
  
  pinMode(MOTOR_RIGHT_RPWM, OUTPUT);
  pinMode(MOTOR_RIGHT_LPWM, OUTPUT);
  pinMode(MOTOR_LEFT_RPWM, OUTPUT);
  pinMode(MOTOR_LEFT_LPWM, OUTPUT);
  
  // Stop all motors
  analogWrite(MOTOR_RIGHT_RPWM, 0);
  analogWrite(MOTOR_RIGHT_LPWM, 0);
  analogWrite(MOTOR_LEFT_RPWM, 0);
  analogWrite(MOTOR_LEFT_LPWM, 0);
  
  Serial.println("Motor Test Ready");
  Serial.println("Commands:");
  Serial.println("  F = Forward");
  Serial.println("  B = Backward");
  Serial.println("  L = Left motor only");
  Serial.println("  R = Right motor only");
  Serial.println("  S = Stop");
  delay(2000);
}

void loop() {
  // Auto test sequence
  Serial.println("\n=== Starting Auto Test Sequence ===");
  
  // Test 1: Both motors forward slow
  Serial.println("Test 1: Both motors FORWARD (PWM 150)");
  analogWrite(MOTOR_LEFT_RPWM, 150);
  analogWrite(MOTOR_LEFT_LPWM, 0);
  analogWrite(MOTOR_RIGHT_RPWM, 150);
  analogWrite(MOTOR_RIGHT_LPWM, 0);
  delay(2000);
  
  // Stop
  Serial.println("STOP");
  analogWrite(MOTOR_LEFT_RPWM, 0);
  analogWrite(MOTOR_LEFT_LPWM, 0);
  analogWrite(MOTOR_RIGHT_RPWM, 0);
  analogWrite(MOTOR_RIGHT_LPWM, 0);
  delay(1000);
  
  // Test 2: Both motors backward slow
  Serial.println("Test 2: Both motors BACKWARD (PWM 150)");
  analogWrite(MOTOR_LEFT_RPWM, 0);
  analogWrite(MOTOR_LEFT_LPWM, 150);
  analogWrite(MOTOR_RIGHT_RPWM, 0);
  analogWrite(MOTOR_RIGHT_LPWM, 150);
  delay(2000);
  
  // Stop
  Serial.println("STOP");
  analogWrite(MOTOR_LEFT_RPWM, 0);
  analogWrite(MOTOR_LEFT_LPWM, 0);
  analogWrite(MOTOR_RIGHT_RPWM, 0);
  analogWrite(MOTOR_RIGHT_LPWM, 0);
  delay(1000);
  
  // Test 3: Left motor only
  Serial.println("Test 3: LEFT motor only (PWM 150)");
  analogWrite(MOTOR_LEFT_RPWM, 150);
  analogWrite(MOTOR_LEFT_LPWM, 0);
  analogWrite(MOTOR_RIGHT_RPWM, 0);
  analogWrite(MOTOR_RIGHT_LPWM, 0);
  delay(2000);
  
  // Stop
  Serial.println("STOP");
  analogWrite(MOTOR_LEFT_RPWM, 0);
  analogWrite(MOTOR_LEFT_LPWM, 0);
  analogWrite(MOTOR_RIGHT_RPWM, 0);
  analogWrite(MOTOR_RIGHT_LPWM, 0);
  delay(1000);
  
  // Test 4: Right motor only
  Serial.println("Test 4: RIGHT motor only (PWM 150)");
  analogWrite(MOTOR_LEFT_RPWM, 0);
  analogWrite(MOTOR_LEFT_LPWM, 0);
  analogWrite(MOTOR_RIGHT_RPWM, 150);
  analogWrite(MOTOR_RIGHT_LPWM, 0);
  delay(2000);
  
  // Stop
  Serial.println("STOP");
  analogWrite(MOTOR_LEFT_RPWM, 0);
  analogWrite(MOTOR_LEFT_LPWM, 0);
  analogWrite(MOTOR_RIGHT_RPWM, 0);
  analogWrite(MOTOR_RIGHT_LPWM, 0);
  
  Serial.println("\n=== Test Complete - Waiting 5 seconds ===\n");
  delay(5000);
  
  // Check for serial commands
  if (Serial.available()) {
    char cmd = Serial.read();
    handleCommand(cmd);
  }
}

void handleCommand(char cmd) {
  switch(cmd) {
    case 'F':
    case 'f':
      Serial.println("Forward");
      analogWrite(MOTOR_LEFT_RPWM, 200);
      analogWrite(MOTOR_LEFT_LPWM, 0);
      analogWrite(MOTOR_RIGHT_RPWM, 200);
      analogWrite(MOTOR_RIGHT_LPWM, 0);
      break;
      
    case 'B':
    case 'b':
      Serial.println("Backward");
      analogWrite(MOTOR_LEFT_RPWM, 0);
      analogWrite(MOTOR_LEFT_LPWM, 200);
      analogWrite(MOTOR_RIGHT_RPWM, 0);
      analogWrite(MOTOR_RIGHT_LPWM, 200);
      break;
      
    case 'L':
    case 'l':
      Serial.println("Left motor only");
      analogWrite(MOTOR_LEFT_RPWM, 200);
      analogWrite(MOTOR_LEFT_LPWM, 0);
      analogWrite(MOTOR_RIGHT_RPWM, 0);
      analogWrite(MOTOR_RIGHT_LPWM, 0);
      break;
      
    case 'R':
    case 'r':
      Serial.println("Right motor only");
      analogWrite(MOTOR_LEFT_RPWM, 0);
      analogWrite(MOTOR_LEFT_LPWM, 0);
      analogWrite(MOTOR_RIGHT_RPWM, 200);
      analogWrite(MOTOR_RIGHT_LPWM, 0);
      break;
      
    case 'S':
    case 's':
      Serial.println("Stop");
      analogWrite(MOTOR_LEFT_RPWM, 0);
      analogWrite(MOTOR_LEFT_LPWM, 0);
      analogWrite(MOTOR_RIGHT_RPWM, 0);
      analogWrite(MOTOR_RIGHT_LPWM, 0);
      break;
  }
}
