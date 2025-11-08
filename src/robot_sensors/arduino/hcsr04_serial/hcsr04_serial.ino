// HC-SR04 Ultrasonic Sensor → Arduino Mega → Serial Output
// Prints distance in cm (one value per line)
// Part of autonomous floor cleaning robot project

#define TRIG_PIN 44
#define ECHO_PIN 45

void setup() {
  Serial.begin(115200);        // match baud rate on the Pi/ROS side
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

float readDistanceCm() {
  digitalWrite(TRIG_PIN, LOW); 
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); 
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  unsigned long us = pulseIn(ECHO_PIN, HIGH, 30000UL); // timeout 30 ms
  if (us == 0) return NAN;
  return us * 0.034f / 2.0f;  // speed of sound: 340 m/s = 0.034 cm/µs
}

void loop() {
  float d = readDistanceCm();
  if (!isnan(d)) {
    Serial.println(d, 2);  // print cm as text line with 2 decimals
  }
  delay(100);  // 10 Hz update rate
}
