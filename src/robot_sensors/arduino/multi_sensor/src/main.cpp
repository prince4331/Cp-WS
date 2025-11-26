// Autonomous Floor Cleaning Robot - All Sensors + Motor Control
// Sensors: 7x Ultrasonic (obstacle + tank levels), 8x IR (4 object + 4 stair),
//          2x Encoders, Emergency Stops, INA219 (battery V/I), DHT11 (temp), BNO055 IMU
// Actuators: 2x BTS7960 motor drivers, buzzer, status LED

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_INA219.h>
#include <DHT.h>

// ============= ULTRASONIC SENSOR PINS (HC-SR04) =============
// Obstacle array trigger pins
#define US_FRONT_TRIG   44
#define US_FRIGHT_TRIG  42
#define US_FLEFT_TRIG   36
#define US_RIGHT_TRIG   38
#define US_LEFT_TRIG    32  // SWAPPED: Now using pins 32-33 for LEFT obstacle

// Obstacle array echo pins
#define US_FRONT_ECHO   45
#define US_FRIGHT_ECHO  43
#define US_FLEFT_ECHO   37
#define US_RIGHT_ECHO   39
#define US_LEFT_ECHO    33  // SWAPPED: Now using pins 32-33 for LEFT obstacle

// Water level sensors (clean & dirty tanks)
#define US_CLEAN_TRIG   34
#define US_CLEAN_ECHO   35
#define US_DIRTY_TRIG   40  // SWAPPED: Now using pins 40-41 for DIRTY WATER
#define US_DIRTY_ECHO   41  // SWAPPED: Now using pins 40-41 for DIRTY WATER  
// ============= SAFETY & AUXILIARY IO =============
#define ESTOP_MAIN_PIN    22   // Main emergency stop (NO -> HIGH when pressed)
#define FRONT_STOP_PIN    24   // Front bumper/pressure switch (NO -> HIGH when pressed)
#define BUZZER_PIN        26   // Buzzer output
#define STATUS_LED_PIN    28   // Status LED output

// Relay polarity: set to true if your relay turns ON when driven HIGH.
const bool RELAY_ACTIVE_HIGH = false;  // Relays are active-LOW (ON when pin is LOW)

// Output pins
#define RELAY_BRUSH_MAIN_PIN  30  // Scrubber (relay - active LOW)
#define VACUUM_PIN            31  // Vacuum Pump (direct connection - normally LOW, HIGH when ON)
#define RELAY_BRUSH_LEFT_PIN  27  // Sweeping Brush (relay - active LOW)
#define RELAY_BRUSH_RIGHT_PIN 29  // Water Pump (relay - active LOW)

// ============= BATTERY SENSORS =============
// Analog pins for battery monitoring
#define BATTERY_VOLTAGE_PIN  A0  // Battery voltage sensor
#define BATTERY_CURRENT_PIN  A1  // Battery current sensor
// DHT11 temperature sensor digital pin
#define DHT_PIN              A2  // DHT11 data pin (temperature)
#define DHT_TYPE             DHT11    // DHT sensor type

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
// RIGHT MOTOR: SWAPPED RPWM and LPWM to invert direction
#define MOTOR_RIGHT_RPWM  7  // Swapped: was 6
#define MOTOR_RIGHT_LPWM  6  // Swapped: was 7
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

// Battery monitoring (INA219 + DHT11)
Adafruit_INA219 ina219;
DHT dht(DHT_PIN, DHT_TYPE);
bool ina219_available = false;
bool dht_available = false;

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

// E-STOP toggle mechanism
bool estop_latched = false;           // Latched E-STOP state (toggle on/off)
bool estop_last_button_state = LOW;   // Previous button state for edge detection
unsigned long estop_last_press_time = 0;  // Debounce timer
const unsigned long ESTOP_DEBOUNCE_MS = 50;  // Debounce delay

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

  // Vacuum is direct connection (not relay) - LOW=off, HIGH=on
  digitalWrite(VACUUM_PIN, vacuum_should ? HIGH : LOW);
  
  // Others are relays with ACTIVE_HIGH logic
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
  // Setup Ultrasonic Trigger pins
  pinMode(US_FRONT_TRIG, OUTPUT);
  pinMode(US_FRIGHT_TRIG, OUTPUT);
  pinMode(US_FLEFT_TRIG, OUTPUT);   // Re-enabled
  pinMode(US_RIGHT_TRIG, OUTPUT);
  pinMode(US_LEFT_TRIG, OUTPUT);
  pinMode(US_CLEAN_TRIG, OUTPUT);
  pinMode(US_DIRTY_TRIG, OUTPUT);

  // Setup Ultrasonic Echo pins
  pinMode(US_FRONT_ECHO, INPUT);
  pinMode(US_FRIGHT_ECHO, INPUT);
  pinMode(US_FLEFT_ECHO, INPUT);    // Re-enabled
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
  pinMode(ESTOP_MAIN_PIN, INPUT_PULLUP);    // NO switch, use pull-up so HIGH when pressed
  pinMode(FRONT_STOP_PIN, INPUT_PULLUP);    // NO switch with pull-up, will be HIGH when pressed
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(VACUUM_PIN, OUTPUT);              // Vacuum - direct connection (not relay)
  pinMode(RELAY_BRUSH_MAIN_PIN, OUTPUT);    // Scrubber relay
  pinMode(RELAY_BRUSH_LEFT_PIN, OUTPUT);    // Sweeping brush relay
  pinMode(RELAY_BRUSH_RIGHT_PIN, OUTPUT);   // Water pump relay
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

  // Initialize INA219 current/voltage sensor
  ina219_available = ina219.begin();
  if (ina219_available) {
    // INA219 will measure voltage and current via I2C
    // Default: 32V, 2A range (auto-calibrated)
  }

  // Initialize DHT11 temperature sensor
  dht.begin();
  dht_available = true;  // DHT11 doesn't have begin() return value
}

// Read all sensor data and output as pipe-separated sections
void readAllSensors() {
  // Read ultrasonic sensors (cm)
  float us_front = readUltrasonicCm(US_FRONT_TRIG, US_FRONT_ECHO);
  float us_fright = readUltrasonicCm(US_FRIGHT_TRIG, US_FRIGHT_ECHO);
  float us_fleft = readUltrasonicCm(US_FLEFT_TRIG, US_FLEFT_ECHO);  // Re-enabled
  float us_right = readUltrasonicCm(US_RIGHT_TRIG, US_RIGHT_ECHO);
  float us_left = readUltrasonicCm(US_LEFT_TRIG, US_LEFT_ECHO);
  float us_clean = readUltrasonicCm(US_CLEAN_TRIG, US_CLEAN_ECHO);
  float us_dirty = readUltrasonicCm(US_DIRTY_TRIG, US_DIRTY_ECHO);

  // Read 4 IR object sensors
  // INVERTED: Physical sensors output LOW when object present (Active LOW)
  // After inversion: 1 = object detected (DANGER), 0 = clear (SAFE)
  int ir_fright_obj = !digitalRead(IR_FRIGHT_OBJ);
  int ir_fleft_obj = !digitalRead(IR_FLEFT_OBJ);
  int ir_bright_obj = !digitalRead(IR_BRIGHT_OBJ);
  int ir_bleft_obj = !digitalRead(IR_BLEFT_OBJ);

  // Read 4 IR stair sensors
  // INVERTED: Physical sensors output LOW when floor detected (Active LOW)
  // After inversion: 1 = floor detected (SAFE), 0 = no floor/stair (DANGER)
  int ir_fright_stair = !digitalRead(IR_FRIGHT_STAIR);
  int ir_fleft_stair = !digitalRead(IR_FLEFT_STAIR);
  int ir_bright_stair = !digitalRead(IR_BRIGHT_STAIR);
  int ir_bleft_stair = !digitalRead(IR_BLEFT_STAIR);

  // Read emergency/bumper switches
  // Pin 22 (ESTOP_MAIN_PIN): NO switch - reads LOW (0) when safe, HIGH (1) when pressed
  // Pin 24 (FRONT_STOP_PIN): NO switch - reads LOW (0) when safe, HIGH (1) when pressed
  int pin22_state = digitalRead(ESTOP_MAIN_PIN);
  int pin24_state = digitalRead(FRONT_STOP_PIN);
  
  // E-STOP toggle logic: Press once to LOCK, press again to UNLOCK
  if (pin22_state == HIGH && estop_last_button_state == LOW) {
    // Rising edge detected (button just pressed)
    if (millis() - estop_last_press_time > ESTOP_DEBOUNCE_MS) {
      // Toggle the latched state
      estop_latched = !estop_latched;
      estop_last_press_time = millis();
    }
  }
  estop_last_button_state = pin22_state;
  
  // Front bumper remains immediate (not latched)
  bool front_stop_active = (pin24_state == HIGH);   // NO: HIGH when bumper pressed

  // Check IR sensors for hazards
  // Object detection: 1 = object detected (DANGER), 0 = clear
  bool object_detected = (ir_fright_obj == 1) || (ir_fleft_obj == 1) || 
                         (ir_bright_obj == 1) || (ir_bleft_obj == 1);
  
  // Stair detection: 0 = stair/drop detected (DANGER), 1 = floor
  bool stair_detected = (ir_fright_stair == 0) || (ir_fleft_stair == 0) || 
                        (ir_bright_stair == 0) || (ir_bleft_stair == 0);

  // Safety lock is active if: E-STOP latched OR any other emergency condition
  safety_lock_active = estop_latched || front_stop_active || object_detected || stair_detected;
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

  // Read battery telemetry from analog pins and DHT11
  float bus_voltage = 0.0;    // Voltage in volts
  float current_mA = 0.0;      // Current in mA
  float temperature_C = 0.0;   // Temperature in Celsius
  
  // Read battery voltage from A0 (0-1023 ADC, assuming voltage divider)
  // Calibrated for 25.8V max: voltage = (ADC / 1023) * 25.8V
  int voltage_adc = analogRead(BATTERY_VOLTAGE_PIN);
  bus_voltage = (voltage_adc / 1023.0) * 25.8;
  
  // Read battery current from A1 (0-1023 ADC, assuming current sensor)
  // Assuming 0-10A range: current = (ADC / 1023) * 10A = 10000mA
  int current_adc = analogRead(BATTERY_CURRENT_PIN);
  current_mA = (current_adc / 1023.0) * 10000.0;
  
  if (dht_available) {
    temperature_C = dht.readTemperature();      // Temperature in Celsius
    if (isnan(temperature_C)) {
      temperature_C = 0.0;  // Handle NaN from DHT11
    }
  }
  
  // Convert to "raw" values for ROS node compatibility
  // ROS node expects: voltage_raw * 0.0254 = volts, current_raw * 0.001 = amps
  // So: voltage_raw = volts / 0.0254, current_raw = (mA/1000) / 0.001 = mA
  int batt_voltage_raw = (int)(bus_voltage / 0.0254);
  int batt_current_raw = (int)(current_mA);  // Already in mA, matches 0.001 scale
  int batt_temp_raw = (int)(temperature_C / 0.1);  // Scale to match 0.1 degC per count

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
  Serial.print(pin22_state); Serial.print(",");
  Serial.print(pin24_state); Serial.print(",");
  Serial.print(safety_lock_active ? 1 : 0); Serial.print(",");
  Serial.print(estop_latched ? 1 : 0);  // Add latched state for debugging

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

  Serial.print("|ESTOP:");
  Serial.print(estop_latched ? 1 : 0); Serial.print(",");
  Serial.print(digitalRead(FRONT_STOP_PIN)); Serial.print(",");
  Serial.print(safety_lock_active ? 1 : 0);

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

      // Apply motor commands ONLY if safety is not active
      if (safety_lock_active) {
        motor_left_pwm = 0;
        motor_right_pwm = 0;
      }
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
  else if (cmd.startsWith("RELAY:")) {
    // RELAY:X,Y -> map to vacuum/brush_main/brush_left/brush_right
    // X = 1..4  , Y = 0|1
    String data = cmd.substring(6); // remove "RELAY:"
    int comma = data.indexOf(',');
    if (comma > 0) {
      int relayNum = data.substring(0, comma).toInt();
      int state = data.substring(comma + 1).toInt();
      bool on = (state != 0);

      switch (relayNum) {
        case 1: // Vacuum Pump (direct connection, not relay)
          vacuum_requested = on;
          break;
        case 2: // Scrubber (relay on pin 30)
          brush_main_requested = on;
          break;
        case 3: // Sweeping Brush (relay on pin 27)
          brush_left_requested = on;
          break;
        case 4: // Water Pump (relay on pin 29)
          brush_right_requested = on;
          break;
        default:
          // invalid relay number
          break;
      }
      applyOutputs(safety_lock_active);
      // Optionally echo acknowledgement
      Serial.print("OK:RELAY:");
      Serial.print(relayNum);
      Serial.print(",");
      Serial.println(state);
    }
  }
  else if (cmd.startsWith("ESTOP:")) {
    // ESTOP:0 = release, ESTOP:1 = engage
    String data = cmd.substring(6);
    int state = data.toInt();
    estop_latched = (state != 0);
    Serial.print("OK:ESTOP:");
    Serial.println(estop_latched ? 1 : 0);
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
