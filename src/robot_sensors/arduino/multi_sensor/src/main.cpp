// Multi-Sensor Arduino Mega → ROS Bridge
// Autonomous Floor Cleaning Robot - All Sensors + Motor Control
// Sensors: 7x Ultrasonic (obstacle + tank levels), 8x IR (4 object + 4 stair),
//          2x Encoders, Emergency Stops, Battery telemetry, BNO055 IMU
// Actuators: 2x BTS7960 motor drivers, buzzer, status LED

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// ============= ULTRASONIC SENSOR PINS (HC-SR04) =============
// Obstacle array trigger pins
#define US_FRONT_TRIG   44
#define US_FRIGHT_TRIG  42
#define US_FLEFT_TRIG   40
#define US_RIGHT_TRIG   38
#define US_LEFT_TRIG    36

// Obstacle array echo pins
#define US_FRONT_ECHO   45
#define US_FRIGHT_ECHO  43
#define US_FLEFT_ECHO   41
#define US_RIGHT_ECHO   39
#define US_LEFT_ECHO    37

// Water level sensors (clean & dirty tanks)
#define US_CLEAN_TRIG   34
#define US_CLEAN_ECHO   35
#define US_DIRTY_TRIG   32
#define US_DIRTY_ECHO   33

// ============= SAFETY & AUXILIARY IO =============
#define ESTOP_MAIN_PIN    22   // Main emergency stop (NC -> LOW when pressed)
#define FRONT_STOP_PIN    24   // Front bumper/pressure switch (NO -> HIGH when pressed)
#define BUZZER_PIN        26   // Buzzer output
#define STATUS_LED_PIN    28   // Status LED output

// Relay polarity: set to true if your relay turns ON when driven HIGH.
const bool RELAY_ACTIVE_HIGH = false;

// Relay outputs
#define RELAY_VACUUM_PIN      30
#define RELAY_BRUSH_MAIN_PIN  31
#define RELAY_BRUSH_LEFT_PIN  27
#define RELAY_BRUSH_RIGHT_PIN 29

// ============= BATTERY SENSORS (ANALOG) =============
#define BATTERY_VOLT_PIN   A0
#define BATTERY_CURR_PIN   A1
#define BATTERY_TEMP_PIN   A2

// ============= IR SENSOR PINS =============
// Object detection IR sensors (digital)
#define IR_FRIGHT_OBJ   46
#define IR_FLEFT_OBJ    48
#define IR_BRIGHT_OBJ   50
#define IR_BLEFT_OBJ    52

// Stair detection IR sensors (digital)
#define IR_FRIGHT_STAIR 47
#define IR_FLEFT_STAIR  49
#define IR_BRIGHT_STAIR 51
#define IR_BLEFT_STAIR  53

// ============= ENCODER PINS =============
#define ENCODER_LEFT    3
#define ENCODER_RIGHT   2

// ============= MOTOR DRIVER PINS (BTS7960) =============
#define MOTOR_RIGHT_RPWM  6
#define MOTOR_RIGHT_LPWM  7
#define MOTOR_LEFT_RPWM   4
#define MOTOR_LEFT_LPWM   5

// ============= CONSTANTS & STATE =============
constexpr uint8_t ANALOG_SAMPLES = 4;

volatile long encoder_left_count = 0;
volatile long encoder_right_count = 0;

// Motor control
int motor_left_pwm = 0;
int motor_right_pwm = 0;

// Command buffer
String inputString = "";
boolean stringComplete = false;

// IMU
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
bool imu_available = false;

// Output states
bool buzzer_requested = false;
bool led_requested = false;
bool vacuum_requested = false;
bool brush_main_requested = false;
bool brush_left_requested = false;
bool brush_right_requested = false;

bool vacuum_active = false;
bool brush_main_active = false;
bool brush_left_active = false;
bool brush_right_active = false;

bool safety_lock_active = false;
unsigned long safety_lock_released_at = 0;
const unsigned long SAFETY_DEBOUNCE_MS = 200;  // require this long of clear signal before restoring outputs

void setMotor(int pin_rpwm, int pin_lpwm, int pwm_value);

inline void setRelayOutput(int pin, bool enabled) {
  if (RELAY_ACTIVE_HIGH) {
    digitalWrite(pin, enabled ? HIGH : LOW);
  } else {
    digitalWrite(pin, enabled ? LOW : HIGH);
  }
}

// Encoder interrupt handlers
void encoderLeftISR() {
  --encoder_left_count;
}

void encoderRightISR() {
  --encoder_right_count;
}

void applyOutputs(bool safety_lock) {
  digitalWrite(BUZZER_PIN, buzzer_requested ? HIGH : LOW);
  digitalWrite(STATUS_LED_PIN, led_requested ? HIGH : LOW);

  const bool vacuum_should = safety_lock ? false : vacuum_requested;
  const bool brush_main_should = safety_lock ? false : brush_main_requested;
  const bool brush_left_should = safety_lock ? false : brush_left_requested;
  const bool brush_right_should = safety_lock ? false : brush_right_requested;

  vacuum_active = vacuum_should;
  brush_main_active = brush_main_should;
  brush_left_active = brush_left_should;
  brush_right_active = brush_right_should;

  setRelayOutput(RELAY_VACUUM_PIN, vacuum_should);
  setRelayOutput(RELAY_BRUSH_MAIN_PIN, brush_main_should);
  setRelayOutput(RELAY_BRUSH_LEFT_PIN, brush_left_should);
  setRelayOutput(RELAY_BRUSH_RIGHT_PIN, brush_right_should);
}

int readAnalogAverage(uint8_t pin) {
  long total = 0;
  for (uint8_t i = 0; i < ANALOG_SAMPLES; ++i) {
    total += analogRead(pin);
  }
  return static_cast<int>(total / ANALOG_SAMPLES);
}

// Read ultrasonic distance in cm
float readUltrasonicCm(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long us = pulseIn(echoPin, HIGH, 30000UL); // 30ms timeout
  if (us == 0) return -1.0f; // No echo
  return us * 0.034f / 2.0f; // Convert to cm
}

void setup() {
  Serial.begin(115200);

  // Setup Ultrasonic Trigger pins
  pinMode(US_FRONT_TRIG, OUTPUT);
  pinMode(US_FRIGHT_TRIG, OUTPUT);
  pinMode(US_FLEFT_TRIG, OUTPUT);
  pinMode(US_RIGHT_TRIG, OUTPUT);
  pinMode(US_LEFT_TRIG, OUTPUT);
  pinMode(US_CLEAN_TRIG, OUTPUT);
  pinMode(US_DIRTY_TRIG, OUTPUT);

  // Setup Ultrasonic Echo pins
  pinMode(US_FRONT_ECHO, INPUT);
  pinMode(US_FRIGHT_ECHO, INPUT);
  pinMode(US_FLEFT_ECHO, INPUT);
  pinMode(US_RIGHT_ECHO, INPUT);
  pinMode(US_LEFT_ECHO, INPUT);
  pinMode(US_CLEAN_ECHO, INPUT);
  pinMode(US_DIRTY_ECHO, INPUT);

  // Setup IR Object Detection pins
  pinMode(IR_FRIGHT_OBJ, INPUT);
  pinMode(IR_FLEFT_OBJ, INPUT);
  pinMode(IR_BRIGHT_OBJ, INPUT);
  pinMode(IR_BLEFT_OBJ, INPUT);

  // Setup IR Stair Detection pins
  pinMode(IR_FRIGHT_STAIR, INPUT);
  pinMode(IR_FLEFT_STAIR, INPUT);
  pinMode(IR_BRIGHT_STAIR, INPUT);
  pinMode(IR_BLEFT_STAIR, INPUT);

  // Setup Encoder pins with interrupts
  pinMode(ENCODER_LEFT, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT), encoderLeftISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT), encoderRightISR, RISING);

  // Setup Motor Driver pins
  pinMode(MOTOR_RIGHT_RPWM, OUTPUT);
  pinMode(MOTOR_RIGHT_LPWM, OUTPUT);
  pinMode(MOTOR_LEFT_RPWM, OUTPUT);
  pinMode(MOTOR_LEFT_LPWM, OUTPUT);

  // Setup safety / auxiliary IO
  pinMode(ESTOP_MAIN_PIN, INPUT_PULLUP);    // NC switch, use pull-up so LOW when pressed
  pinMode(FRONT_STOP_PIN, INPUT);           // NO switch, assume external pulldown
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(RELAY_VACUUM_PIN, OUTPUT);
  pinMode(RELAY_BRUSH_MAIN_PIN, OUTPUT);
  pinMode(RELAY_BRUSH_LEFT_PIN, OUTPUT);
  pinMode(RELAY_BRUSH_RIGHT_PIN, OUTPUT);
  safety_lock_active = false;
  applyOutputs(safety_lock_active);

  // Initialize motors to stopped
  analogWrite(MOTOR_RIGHT_RPWM, 0);
  analogWrite(MOTOR_RIGHT_LPWM, 0);
  analogWrite(MOTOR_LEFT_RPWM, 0);
  analogWrite(MOTOR_LEFT_LPWM, 0);

  // Initialise IMU
  Wire.begin();
  imu_available = bno.begin();
  if (imu_available) {
    bno.setExtCrystalUse(true);
  }
}

// Read all sensor data and output as pipe-separated sections
void readAllSensors() {
  // Read 7 ultrasonic sensors (cm)
  float us_front = readUltrasonicCm(US_FRONT_TRIG, US_FRONT_ECHO);
  float us_fright = readUltrasonicCm(US_FRIGHT_TRIG, US_FRIGHT_ECHO);
  float us_fleft = readUltrasonicCm(US_FLEFT_TRIG, US_FLEFT_ECHO);
  float us_right = readUltrasonicCm(US_RIGHT_TRIG, US_RIGHT_ECHO);
  float us_left = readUltrasonicCm(US_LEFT_TRIG, US_LEFT_ECHO);
  float us_clean = readUltrasonicCm(US_CLEAN_TRIG, US_CLEAN_ECHO);
  float us_dirty = readUltrasonicCm(US_DIRTY_TRIG, US_DIRTY_ECHO);

  // Read 4 IR object sensors (0 = object detected, 1 = clear)
  int ir_fright_obj = digitalRead(IR_FRIGHT_OBJ);
  int ir_fleft_obj = digitalRead(IR_FLEFT_OBJ);
  int ir_bright_obj = digitalRead(IR_BRIGHT_OBJ);
  int ir_bleft_obj = digitalRead(IR_BLEFT_OBJ);

  // Read 4 IR stair sensors (0 = stair/drop detected, 1 = floor)
  int ir_fright_stair = digitalRead(IR_FRIGHT_STAIR);
  int ir_fleft_stair = digitalRead(IR_FLEFT_STAIR);
  int ir_bright_stair = digitalRead(IR_BRIGHT_STAIR);
  int ir_bleft_stair = digitalRead(IR_BLEFT_STAIR);

  // Read emergency/bumper switches
  bool estop_main_active = (digitalRead(ESTOP_MAIN_PIN) == LOW);    // NC, LOW when pressed
  bool front_stop_active = (digitalRead(FRONT_STOP_PIN) == HIGH);   // NO, HIGH when pressed

  safety_lock_active = estop_main_active || front_stop_active;
  if (!safety_lock_active) {
    if (safety_lock_released_at == 0) {
      safety_lock_released_at = millis();
    }
  } else {
    safety_lock_released_at = 0;
  }

  // Immediately stop motors when safety is triggered
  if (safety_lock_active) {
    motor_left_pwm = 0;
    motor_right_pwm = 0;
    setMotor(MOTOR_LEFT_RPWM, MOTOR_LEFT_LPWM, 0);
    setMotor(MOTOR_RIGHT_RPWM, MOTOR_RIGHT_LPWM, 0);
  }

  applyOutputs(safety_lock_active);

  // Read encoder counts atomically
  long enc_left, enc_right;
  noInterrupts();
  enc_left = encoder_left_count;
  enc_right = encoder_right_count;
  interrupts();

  // Read battery telemetry
  int batt_voltage_raw = readAnalogAverage(BATTERY_VOLT_PIN);
  int batt_current_raw = readAnalogAverage(BATTERY_CURR_PIN);
  int batt_temp_raw = readAnalogAverage(BATTERY_TEMP_PIN);

  // Read IMU data if available
  bool imu_has_sample = false;
  imu::Quaternion quat;
  imu::Vector<3> gyro;
  imu::Vector<3> linacc;
  if (imu_available) {
    quat = bno.getQuat();
    gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);        // rad/s
    linacc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);    // m/s^2
    imu_has_sample = true;
  }

  // Output format:
  // US:front,fright,fleft,right,left
  // |USW:clean,dirty
  // |IR_OBJ:fr,fl,br,bl
  // |IR_STAIR:fr,fl,br,bl
  // |ENC:left,right
  // |ESTOP:main,front
  // |BAT:volt_raw,curr_raw,temp_raw
  // |IMU:qw,qx,qy,qz,gx,gy,gz,ax,ay,az   (or IMU:NA if unavailable)
  // |STATE:buzzer,led

  Serial.print("US:");
  Serial.print(us_front, 2); Serial.print(",");
  Serial.print(us_fright, 2); Serial.print(",");
  Serial.print(us_fleft, 2); Serial.print(",");
  Serial.print(us_right, 2); Serial.print(",");
  Serial.print(us_left, 2);

  Serial.print("|USW:");
  if (us_clean >= 0) Serial.print(us_clean, 2); else Serial.print("-1");
  Serial.print(",");
  if (us_dirty >= 0) Serial.print(us_dirty, 2); else Serial.print("-1");

  Serial.print("|IR_OBJ:");
  Serial.print(ir_fright_obj); Serial.print(",");
  Serial.print(ir_fleft_obj); Serial.print(",");
  Serial.print(ir_bright_obj); Serial.print(",");
  Serial.print(ir_bleft_obj);

  Serial.print("|IR_STAIR:");
  Serial.print(ir_fright_stair); Serial.print(",");
  Serial.print(ir_fleft_stair); Serial.print(",");
  Serial.print(ir_bright_stair); Serial.print(",");
  Serial.print(ir_bleft_stair);

  Serial.print("|ENC:");
  Serial.print(enc_left); Serial.print(",");
  Serial.print(enc_right);

  Serial.print("|ESTOP:");
  Serial.print(estop_main_active ? 1 : 0); Serial.print(",");
  Serial.print(front_stop_active ? 1 : 0);

  Serial.print("|BAT:");
  Serial.print(batt_voltage_raw); Serial.print(",");
  Serial.print(batt_current_raw); Serial.print(",");
  Serial.print(batt_temp_raw);

  Serial.print("|IMU:");
  if (imu_has_sample) {
    Serial.print(quat.w(), 4); Serial.print(",");
    Serial.print(quat.x(), 4); Serial.print(",");
    Serial.print(quat.y(), 4); Serial.print(",");
    Serial.print(quat.z(), 4); Serial.print(",");
    Serial.print(gyro.x(), 4); Serial.print(",");
    Serial.print(gyro.y(), 4); Serial.print(",");
    Serial.print(gyro.z(), 4); Serial.print(",");
    Serial.print(linacc.x(), 4); Serial.print(",");
    Serial.print(linacc.y(), 4); Serial.print(",");
    Serial.print(linacc.z(), 4);
  } else {
    Serial.print("NA");
  }

  Serial.print("|STATE:");
  Serial.print(buzzer_requested ? 1 : 0); Serial.print(",");
  Serial.print(led_requested ? 1 : 0);

  Serial.print("|RELAY:");
  Serial.print(vacuum_active ? 1 : 0); Serial.print(",");
  Serial.print(brush_main_active ? 1 : 0); Serial.print(",");
  Serial.print(brush_left_active ? 1 : 0); Serial.print(",");
  Serial.print(brush_right_active ? 1 : 0);

  Serial.println();
}

// Set motor speed (-255 to 255)
void setMotor(int pin_rpwm, int pin_lpwm, int pwm_value) {
  if (pwm_value > 0) {
    // Forward
    analogWrite(pin_rpwm, constrain(pwm_value, 0, 255));
    analogWrite(pin_lpwm, 0);
  } else if (pwm_value < 0) {
    // Backward
    analogWrite(pin_rpwm, 0);
    analogWrite(pin_lpwm, constrain(-pwm_value, 0, 255));
  } else {
    // Stop
    analogWrite(pin_rpwm, 0);
    analogWrite(pin_lpwm, 0);
  }
}

// Process serial commands
void processCommand(String cmd) {
  if (cmd.startsWith("M:")) {
    // Motor command: M:left_pwm,right_pwm
    // Example: M:200,-150
    cmd.remove(0, 2);  // Remove "M:"

    int comma_pos = cmd.indexOf(',');
    if (comma_pos > 0) {
      motor_left_pwm = cmd.substring(0, comma_pos).toInt();
      motor_right_pwm = cmd.substring(comma_pos + 1).toInt();

      // Apply motor commands
      setMotor(MOTOR_LEFT_RPWM, MOTOR_LEFT_LPWM, motor_left_pwm);
      setMotor(MOTOR_RIGHT_RPWM, MOTOR_RIGHT_LPWM, motor_right_pwm);
    }
  }
  else if (cmd == "S") {
    // Stop command
    motor_left_pwm = 0;
    motor_right_pwm = 0;
    setMotor(MOTOR_LEFT_RPWM, MOTOR_LEFT_LPWM, 0);
    setMotor(MOTOR_RIGHT_RPWM, MOTOR_RIGHT_LPWM, 0);
  }
  else if (cmd.startsWith("BZ:")) {
    buzzer_requested = (cmd.substring(3).toInt() != 0);
    applyOutputs(safety_lock_active);
  }
  else if (cmd.startsWith("LED:")) {
    led_requested = (cmd.substring(4).toInt() != 0);
    applyOutputs(safety_lock_active);
  }
  else if (cmd.startsWith("VAC:")) {
    vacuum_requested = (cmd.substring(4).toInt() != 0);
    applyOutputs(safety_lock_active);
  }
  else if (cmd.startsWith("BRM:")) {
    brush_main_requested = (cmd.substring(4).toInt() != 0);
    applyOutputs(safety_lock_active);
  }
  else if (cmd.startsWith("BRL:")) {
    brush_left_requested = (cmd.substring(4).toInt() != 0);
    applyOutputs(safety_lock_active);
  }
  else if (cmd.startsWith("BRR:")) {
    brush_right_requested = (cmd.substring(4).toInt() != 0);
    applyOutputs(safety_lock_active);
  }
}

// Serial event handler
void serialEvent() {
  while (Serial.available()) {
    char inChar = static_cast<char>(Serial.read());
    if (inChar == '\n') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }
}

void loop() {
  // Process motor / auxiliary commands if received
  if (stringComplete) {
    processCommand(inputString);
    inputString = "";
    stringComplete = false;
  }

  // Read and publish sensor data
  readAllSensors();
  delay(100); // 10 Hz update rate
}
