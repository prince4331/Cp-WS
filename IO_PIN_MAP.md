# Autonomous Floor Cleaning Robot – I/O Pin Map

This document summarizes every hardware input/output used on the Arduino Mega 2560 controller that feeds the ROS 2 sensor/actuator stack. Use it alongside the wiring harness to double‑check connections and understand which ROS topics each signal drives.

## Quick Reference Overview

| Group                | Devices Covered                                                             | Core ROS Topics / Usage               |
|----------------------|-----------------------------------------------------------------------------|--------------------------------------|
| Ultrasonic           | 5 obstacle sensors + 2 tank level sensors                                   | `/ultrasonic/*`, `/water_level/*`    |
| Infrared             | 4 object + 4 stair cliff sensors                                            | `/ir/*`                              |
| Safety & Switches    | Main E‑stop, front touch bar                                                | `/safety/*`                          |
| Motion Feedback      | Left/right wheel encoders                                                    | `/encoder/*`, `/odom`                |
| Actuation            | Motor drivers, buzzer, status LED                                           | `/cmd_vel`, `/buzzer/*`, `/indicator/*` |
| Power & Health       | Battery voltage, current, temperature sensors                               | `/battery/state`                     |
| IMU                  | Adafruit BNO055 (9‑DoF)                                                     | `/imu/data`                          |

## Detailed Pin Tables

### Ultrasonic Sensors (HC‑SR04)

| Function            | Trigger Pin | Echo Pin | Frame ID / ROS Topic           | Notes                         |
|---------------------|-------------|----------|--------------------------------|-------------------------------|
| Front obstacle      | 44          | 45       | `ultrasonic_front` → `/ultrasonic/front` | Forward-facing                |
| Front right         | 42          | 43       | `ultrasonic_front_right` → `/ultrasonic/front_right` | ~45° right                   |
| Front left          | 40          | 41       | `ultrasonic_front_left` → `/ultrasonic/front_left` | ~45° left                    |
| Right side          | 38          | 39   S    | `ultrasonic_right` → `/ultrasonic/right` | 90° right                     |
| Left side           | 36          | 37       | `ultrasonic_left` → `/ultrasonic/left` | 90° left                      |
| Clean water level   | 34          | 35       | `water_clean_level` → `/water_level/clean` | Tank depth (clean water)     |
| Dirty water level   | 32          | 33       | `water_dirty_level` → `/water_level/dirty` | Tank depth (dirty water)     |

All range measurements are converted to meters inside `multi_sensor_node.py`.

### Infrared Sensors

| Sensor Role               | Arduino Pin | Frame ID                 | ROS Topic                        | True Means…                      |
|---------------------------|-------------|--------------------------|----------------------------------|----------------------------------|
| IR object – front right   | 46          | `ir_front_right`         | `/ir/front_right/object`         | Object detected                  |
| IR object – front left    | 48          | `ir_front_left`          | `/ir/front_left/object`          | Object detected                  |
| IR object – back right    | 50          | `ir_back_right`          | `/ir/back_right/object`          | Object detected                  |
| IR object – back left     | 52          | `ir_back_left`           | `/ir/back_left/object`           | Object detected                  |
| IR stair – front right    | 47          | `ir_front_right`         | `/ir/front_right/stair`          | Drop/stair detected              |
| IR stair – front left     | 49          | `ir_front_left`          | `/ir/front_left/stair`           | Drop/stair detected              |
| IR stair – back right     | 51          | `ir_back_right`          | `/ir/back_right/stair`           | Drop/stair detected              |
| IR stair – back left      | 53          | `ir_back_left`           | `/ir/back_left/stair`            | Drop/stair detected              |

Sensor logic: digital LOW (`0`) on the Arduino is reported as `True` in ROS.

### Safety Inputs

| Device                       | Pin | ROS Topic             | Active State | Effect                                                           |
|------------------------------|-----|-----------------------|--------------|------------------------------------------------------------------|
| Main emergency stop (NO)     | 24  | `/safety/estop_main`  | LOW → `True` | `multi_sensor` forces motors to zero and toggles `/safety/estop` |
| Front bumper / pressure bar  | 22  | `/safety/front_switch`| HIGH → `True`| Also triggers `/safety/estop` and motor shutdown                 |

Both inputs are aggregated into `/safety/estop` (`True` if any are active).

### Motion Feedback (Encoders)

| Wheel | Pin (interrupt) | ROS Topic         | Notes                                                |
|-------|-----------------|-------------------|------------------------------------------------------|
| Left  | 3 (`INT1`)      | `/encoder/left`   | Rising edge count; odometry node converts to pose    |
| Right | 2 (`INT0`)      | `/encoder/right`  |                                                      |

Encoder tick counts also appear in `/sensors/raw` and drive `/odom`.

### Actuation Outputs

| Device                      | Pins / Bus                              | Command Source                     | Notes                                    |
|-----------------------------|-----------------------------------------|------------------------------------|------------------------------------------|
| Right motor (BTS7960)       | RPWM `6`, LPWM `7`                      | `motor_controller_node` via `M:*`  | Driven by `/cmd_vel` → PWM mapping       |
| Left motor (BTS7960)        | RPWM `4`, LPWM `5`                      | Same as above                      |                                          |
| Vacuum motor relay          | Digital `30`                            | `/vacuum/cmd` (`VAC:1/0`)          | Input 1 on relay module (default assumes low-level trigger; flip `RELAY_ACTIVE_HIGH` if yours is high-level) |
| Main brush relay            | Digital `31`                            | `/brush/main/cmd` (`BRM:1/0`)      | Relay module input 2                     |
| Side brush relay (left)     | Digital `27`                            | `/brush/left/cmd` (`BRL:1/0`)      | Relay module input 3                     |
| Side brush relay (right)    | Digital `29`                            | `/brush/right/cmd` (`BRR:1/0`)     | Relay module input 4                     |
| Buzzer                      | Digital `26`                            | `/buzzer/cmd` (`BZ:1/0` to MCU)    | State mirrored on `/buzzer/state`        |
| Status LED                  | Digital `28`                            | `/indicator/cmd` (`LED:1/0`)       | State mirrored on `/indicator/state`     |

Motors are hard-stopped when any safety input triggers.

### Power & Health Telemetry

| Measurement        | Analog Pin | Scaling Parameter (ROS)                                | ROS Field                     |
|--------------------|------------|--------------------------------------------------------|-------------------------------|
| Battery voltage    | A0         | `battery_voltage_scale`, `battery_voltage_offset` (default scale assumes 19.1k/6.2k divider) | `/battery/state.voltage`      |
| Battery current    | A1         | `battery_current_scale`, `battery_current_offset`     | `/battery/state.current`      |
| Battery temperature| A2         | `battery_temp_scale`, `battery_temp_offset`           | `/battery/state.temperature`  |

All raw ADC readings are averaged in firmware before conversion.

### IMU (Adafruit BNO055)

| Connection | Arduino Pins | ROS Topic         | Notes                                   |
|------------|--------------|-------------------|-----------------------------------------|
| I²C SDA    | 20           | `/imu/data`       | Requires 3.3 V power and pull-ups       |
| I²C SCL    | 21           | `/imu/data`       | Uses quaternion + gyro + linear accel   |
| Power      | 5 V / GND    | —                 | `bno.begin()` at address `0x28`         |

If the IMU is absent, firmware reports `IMU:NA` and ROS will skip publishing.

## Supporting Files

- Firmware definitions: `src/robot_sensors/arduino/multi_sensor/src/main.cpp`
- ROS bridge: `src/robot_sensors/robot_sensors/multi_sensor_node.py`
- Static transforms: `src/robot_sensors/robot_sensors/robot_tf_publisher.py`
- Detailed wiring guide: `src/robot_sensors/SENSORS_README.md`

Keep this table up to date when adding new peripherals so the ROS 2 dashboard and diagnostics remain accurate.
