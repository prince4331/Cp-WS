// Automated right motor endurance test
// Runs right motor forward for 60s, reverse for 60s, loops.
// Left motor stays off.

#include <Arduino.h>

const uint8_t MOTOR_LEFT_RPWM  = 4;
const uint8_t MOTOR_LEFT_LPWM  = 5;
const uint8_t MOTOR_RIGHT_RPWM = 6;
const uint8_t MOTOR_RIGHT_LPWM = 7;

const uint8_t PWM_FULL = 255;
const unsigned long PHASE_DURATION_MS = 60UL * 1000UL;  // 1 minute per direction

enum Phase { FORWARD, REVERSE };
Phase current_phase = FORWARD;
unsigned long phase_start_ms = 0;

void setRightMotor(int dir) {
  if (dir > 0) {
    analogWrite(MOTOR_RIGHT_RPWM, PWM_FULL);
    analogWrite(MOTOR_RIGHT_LPWM, 0);
  } else if (dir < 0) {
    analogWrite(MOTOR_RIGHT_RPWM, 0);
    analogWrite(MOTOR_RIGHT_LPWM, PWM_FULL);
  } else {
    analogWrite(MOTOR_RIGHT_RPWM, 0);
    analogWrite(MOTOR_RIGHT_LPWM, 0);
  }
}

void stopLeftMotor() {
  analogWrite(MOTOR_LEFT_RPWM, 0);
  analogWrite(MOTOR_LEFT_LPWM, 0);
}

void setup() {
  pinMode(MOTOR_LEFT_RPWM, OUTPUT);
  pinMode(MOTOR_LEFT_LPWM, OUTPUT);
  pinMode(MOTOR_RIGHT_RPWM, OUTPUT);
  pinMode(MOTOR_RIGHT_LPWM, OUTPUT);

  stopLeftMotor();
  setRightMotor(1);
  current_phase = FORWARD;
  phase_start_ms = millis();

  Serial.begin(115200);
  Serial.println("Right motor endurance test started (1 min forward/back loops)");
}

void loop() {
  unsigned long now = millis();
  if (now - phase_start_ms >= PHASE_DURATION_MS) {
    phase_start_ms = now;
    if (current_phase == FORWARD) {
      current_phase = REVERSE;
      setRightMotor(-1);
      Serial.println("Phase change: RIGHT motor reverse");
    } else {
      current_phase = FORWARD;
      setRightMotor(1);
      Serial.println("Phase change: RIGHT motor forward");
    }
  }

  // keep left motor off
  stopLeftMotor();
}
