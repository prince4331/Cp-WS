// Simple switch test for emergency stop and bumper
// Reads pin states and reports them continuously

#include <Arduino.h>

#define ESTOP_MAIN_PIN    22   // Main emergency stop
#define FRONT_STOP_PIN    24   // Front bumper/emergency button

void setup() {
  Serial.begin(115200);
  
  // Setup with pull-ups
  pinMode(ESTOP_MAIN_PIN, INPUT_PULLUP);
  pinMode(FRONT_STOP_PIN, INPUT_PULLUP);
  
  Serial.println("=== Switch Test Program ===");
  Serial.println("Reading Pin 22 (Emergency Stop) and Pin 24 (Bumper/Button)");
  Serial.println("Format: Pin22=X Pin24=Y");
  Serial.println("1 = HIGH (not pressed/released)");
  Serial.println("0 = LOW (pressed/activated)");
  Serial.println("==============================");
  delay(2000);
}

void loop() {
  int pin22_state = digitalRead(ESTOP_MAIN_PIN);
  int pin24_state = digitalRead(FRONT_STOP_PIN);
  
  Serial.print("Pin22=");
  Serial.print(pin22_state);
  Serial.print(" Pin24=");
  Serial.println(pin24_state);
  
  delay(500);  // Update every 500ms
}
