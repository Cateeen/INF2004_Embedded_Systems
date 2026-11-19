# Pico Car: Testing and Integration Plan

This document outlines the complete testing and integration strategy for your Raspberry Pi Pico W autonomous robot car system.

## System Architecture Overview

Your system consists of 8 major subsystems running on a dual-core RP2040 with micro T-Kernel 3.0:

1. **Line Detection & Barcode Reading** - ADC-based line sensing and barcode scanning
2. **Odometry** - Wheel encoder tracking for position estimation
3. **IMU Processing** - LSM303DLHC accelerometer/magnetometer for orientation
4. **Obstacle Avoidance** - HC-SR04 ultrasonic distance sensing
5. **Motion Control** - Motor drive and speed regulation via PID
6. **Vehicle Controller** - High-level steering and safety logic
7. **GPIO & Interrupt Routing** - Hardware event dispatch
8. **WiFi & MQTT** - Telemetry and remote control

Message flow is organized through centralized buffers with multiple policies (keep-latest, ordered, priority).

---

## Testing Levels and Strategy

### Level 1: Unit Tests (Individual Subsystems)

Each subsystem has its own unit tests run at startup when `TEST=1` is used.

**Key Components to Test:**

#### 1.1 Odometry Subsystem
- **Unit Tests (S2-U01 through S2-U0X)**
  - Encoder pulse counting accuracy
  - Odometry math (distance, angle calculations)
  - Two-encoder synchronization
  - Edge cases: zero counts, count wrapping, missed pulses
  - Producer-consumer interaction between encoder and odometry tasks
  
- **Test Code Location:** `app_program/odometry/odometry_test.c`, `odometry_producer_test.c`
- **Test Build:** `make TEST=1` includes these automatically
- **Hardware Needed:** Encoders with signals, or GPIO signal simulator

**Automated Checks:**
```
[TEST] ODO PASS         - odometry calculations
[TEST] ODO-PRODUCER PASS - producer tests
[TEST] ODO-TASK PASS    - task synchronization
```

---

#### 1.2 Line Detection & Barcode Subsystem
- **Unit Tests (S3-U01 through S3-U0X)**
  - ADC line reading and threshold detection
  - Line classification (on-line, left deviation, right deviation)
  - Barcode scanning and decoding
  - Intersection detection
  - Confidence metrics
  
- **Test Code Location:** `app_program/line_barcode/line_observation_test.c`, `barcode_scanner_test.c`
- **Hardware Needed:** Line sensor array (typically 8-16 ADC channels)

**Automated Checks:**
```
[TEST] LINE PASS       - line classification
[TEST] BAR PASS        - barcode decoder
[TEST] LB-TASK PASS    - line barcode task
[TEST] LINE-ADC PASS   - ADC reader
```

---

#### 1.3 IMU Subsystem
- **Unit Tests (S4-U01 through S4-U0X)**
  - LSM303DLHC I2C communication
  - Acceleration to roll/pitch conversion
  - Magnetometer to yaw conversion
  - Shock and tilt detection
  - Terrain classification
  - Calibration and zero-offset handling
  
- **Test Code Location:** `app_program/imu/imu_processor_test.c`, `imu_task_test.c`
- **Hardware Needed:** LSM303DLHC sensor on I2C bus
- **Host-Side Tests:** `testing/integration/imu_i2c_host/test.c` (runs on Linux/WSL without robot)

**Automated Checks:**
```
[TEST] IMU PASS       - IMU processing
[TEST] IMU-TASK PASS  - task tests
```

---

#### 1.4 Obstacle Avoidance Subsystem
- **Unit Tests (S5-U01 through S5-U0X)**
  - HC-SR04 ultrasonic pulse timing
  - Distance calculation from echo pulse width
  - Multi-sensor scanning (left, center, right, etc.)
  - Obstacle detection and classification
  - Emergency stop triggering
  - Command processing (scan, reset, abort)
  
- **Test Code Location:** `app_program/obstacle/obs_profile_test.c`, `obs_task_test.c`
- **Hardware Needed:** HC-SR04 ultrasonic sensors
- **Host-Side Tests:** `testing/integration/hcsr04_host/test.c` (distance conversion, runs without robot)

**Automated Checks:**
```
[TEST] OBS PASS      - obstacle profiling
[TEST] OBS-TASK PASS - task tests
```

---

#### 1.5 Motion Control Subsystem
- **Unit Tests (S6-U01 through S6-U0X)**
  - Motor PWM generation
  - Wheel speed feedback processing
  - PID controller tuning
  - Speed ramping and acceleration profiles
  - Stop/arm/velocity/turn modes
  - Motion status reporting
  - Fault detection (stalled motors, odometry mismatch)
  
- **Test Code Location:** `app_program/motion/wheel_speed_test.c`, `motion_task_test.c`
- **Hardware Needed:** Motors, motor driver, speed encoder feedback
- **Simulation:** `motion_task_test.c` includes simulated odometry for PID tuning

**Automated Checks:**
```
[TEST] MOT PASS      - wheel speed control
[TEST] MOT-TASK PASS - simulation tests
```

---

#### 1.6 Vehicle Controller Subsystem
- **Unit Tests (S7-U01 through S7-U0X)**
  - Line-following steering logic
  - Speed adaptation based on line confidence
  - Obstacle response (stop, turn, reverse)
  - Intersection handling
  - Barcode processing and navigation
  - Safety gates (speed limits, emergency stop)
  - State machine transitions
  
- **Test Code Location:** `app_program/controller/car_gate_test.c`, `line_steering_test.c`, `controller_monitor_test.c`
- **Simulation:** Most tests use simulated sensor data

**Automated Checks:**
```
[TEST] VC PASS        - controller gate and line steering
[TEST] VC-MONITOR PASS - monitor tests
[TEST] VC-TASK PASS   - vehicle controller task tests
```

---

#### 1.7 GPIO & Interrupt Routing
- **Unit Tests (S8-U01 through S8-U0X)**
  - GPIO interrupt dispatch
  - Pin assignment verification
  - Interrupt timing (latency from GPIO to handler)
  - Edge detection (rising, falling, both)
  
- **Test Code Location:** `app_program/gpio/gpio_irq_test.c`

**Automated Checks:**
```
[TEST] GPIO PASS - interrupt routing
```

---

#### 1.8 WiFi & MQTT
- **Unit Tests (S9-U01 through S9-U0X)**
  - WiFi connection to SSID
  - MQTT broker connection
  - Telemetry message formatting
  - Command reception and parsing
  - Network reconnection after loss
  - MQTT keep-alive / heartbeat
  
- **Test Code Location:** `app_program/wifi/wifi_service.c`, `wifi_mqtt_bridge.c`
- **Hardware Needed:** WiFi network access

**Automated Checks:**
```
[TEST] WIFI PASS        - service tests
[TEST] WIFI-BRIDGE PASS - bridge tests
```

---

### Level 2: Integration Tests (Between Subsystems)

Integration tests verify that message passing and subsystem interaction work correctly.

#### 2.1 Encoder → Odometry Integration (INT-01)

**What:** Encoder pulses correctly feed into odometry calculation

**Test Case:**
1. Drive forward with known encoder counts
2. Verify odometry reports correct distance
3. Drive in circle, verify heading calculation

**Expected Result:** Distance within 5% of actual, heading within 5 degrees

**Manual Test Steps:**
- Mark start and end points (e.g., 1 meter)
- Drive forward
- Check reported distance against actual

**Automation:** `testing/integration/encoder_odometry/`
- Need encoder pulse generator or simulator
- Verify message buffer `VEHICLE_BUFFER_WHEEL_ODOMETRY` receives correct values

---

#### 2.2 Line Sensor → Line Observation Integration (INT-02)

**What:** ADC readings correctly map to line position and confidence

**Test Case:**
1. Position sensor over line
2. Verify line_detected flag = 1
3. Move sensor left/right
4. Verify lateral_error and confidence values

**Expected Result:**
- On line: lateral_error ≈ 0, confidence ≈ 255
- Off line: line_detected = 0
- Linearity: lateral_error scales with position

**Automation:**
- Mock ADC values with known patterns
- Verify `VEHICLE_BUFFER_LINE_OBSERVATIONS` messages

---

#### 2.3 IMU → Orientation Integration (INT-03)

**What:** I2C IMU data correctly converts to roll/pitch/yaw estimates

**Test Case:**
1. Tilt robot 45 degrees
2. Verify roll_mdeg ≈ 45000 (millidegrees)
3. Rotate horizontally
4. Verify yaw_mdeg changes

**Expected Result:**
- Roll/pitch within ±10 degrees at 45-degree tilt
- Yaw rotates smoothly, no jumps > 10 degrees

**Host-Side Unit Test (runs on Linux/WSL):**
```sh
cd testing/integration/imu_i2c_host
gcc -std=c11 -Wall -Wextra -Werror -Isys -I../../../app_program/imu \
  test.c ../../../app_program/imu/lsm303dlhc_i2c.c \
  ../../../app_program/imu/lsm303dlhc_reader.c \
  ../../../app_program/imu/lsm303dlhc_accel.c -o imu_i2c_host_test
./imu_i2c_host_test
```

**Robot Test:**
- Mount robot on level surface
- Run IMU task, verify zero roll/pitch
- Tilt and verify angle reporting

---

#### 2.4 Obstacle Sensor → Obstacle Event Integration (INT-04)

**What:** HC-SR04 echo pulses correctly convert to distance and obstacle events

**Test Case:**
1. Place object 20cm from sensor
2. Verify distance_mm ≈ 200
3. Move object away
4. Verify distance increases linearly

**Expected Result:**
- Distance within ±2cm of actual
- No false detections in open air

**Host-Side Unit Test:**
```sh
cd testing/integration/hcsr04_host
gcc -std=c11 -Wall -Wextra -Werror -I../../../app_program/obstacle \
  test.c ../../../app_program/obstacle/hcsr04.c \
  ../../../app_program/obstacle/hcsr04_capture.c -o hcsr04_host_test
./hcsr04_host_test
```

**Robot Test:**
- Verify at multiple distances (15cm, 30cm, 50cm, 100cm)
- Check for timeout handling with no object

---

#### 2.5 Odometry + Controller → Motion Integration (INT-05)

**What:** Motion commands correctly execute based on controller output

**Test Case:**
1. Command VELOCITY mode with linear_velocity = 50 mm/s
2. Verify motor PWM output
3. Measure actual wheel speed via encoder feedback
4. Verify speed-to-PWM mapping accuracy

**Expected Result:**
- Requested speed within 10% of actual
- No oscillation in speed (PID stability)

**Automation:**
- Monitor `VEHICLE_BUFFER_MOTION_STATUS` for actual motor PWM
- Compare against `VEHICLE_BUFFER_MOTION_COMMANDS`

---

#### 2.6 Controller + Obstacle → Safety Gate Integration (INT-06)

**What:** Obstacle detection correctly halts motion via safety gate

**Test Case:**
1. Robot moving forward at 100 mm/s
2. Introduce obstacle at 20cm
3. Verify robot stops immediately
4. Check emergency stop code is set

**Expected Result:**
- Stop within 100ms of obstacle detection
- motion_status shows emergency stop

**Manual Test:**
- Drive robot toward wall
- Measure stop distance (< 15cm from wall)

---

#### 2.7 Line + Barcode → Navigation Integration (INT-07)

**What:** Barcode events correctly trigger navigation waypoints

**Test Case:**
1. Print barcodes on track (start, checkpoint, end)
2. Robot follows line and detects barcodes
3. Verify each barcode event is logged
4. Check navigation state changes appropriately

**Expected Result:**
- All barcodes detected in sequence
- No false barcode detections

**Manual Test:**
- Create test track with printed barcodes
- Drive robot, log all barcode events
- Verify sequence matches expected

---

#### 2.8 Telemetry → WiFi Integration (INT-08)

**What:** Motion/sensor data correctly publishes to MQTT

**Test Case:**
1. Robot connected to WiFi
2. MQTT broker receiving heartbeat
3. Move robot or trigger sensor events
4. Verify telemetry messages appear in MQTT

**Expected Result:**
- Heartbeat every 10 seconds
- All sensor data published with < 500ms latency

**Manual Test:**
- Use MQTT client (e.g., `mosquitto_sub` or MQTT.fx) to subscribe to robot topic
- Verify messages appear

---

### Level 3: System Tests (End-to-End Product Requirements)

System tests verify the robot meets its original requirements.

#### 3.1 Line Following (SYS-01)

**Requirement:** Robot tracks a black line on white background

**Test Setup:**
- Print or tape a line track on floor (straight sections, curves, intersections)
- Mark start and finish

**Test Steps:**
1. Place robot on start
2. Command line-follow mode
3. Robot autonomously follows line to finish
4. Record time and completion distance

**Success Criteria:**
- Reaches finish without leaving line
- Time within acceptable threshold (e.g., < 60 seconds for 3-meter track)
- No collisions with obstacles

**Evidence:** Video showing full run

---

#### 3.2 Barcode Navigation (SYS-02)

**Requirement:** Robot detects barcodes and executes waypoint navigation

**Test Setup:**
- Print/tape barcodes at 3-5 waypoints along line
- Each barcode encodes a target speed or action

**Test Steps:**
1. Robot starts with barcode "START"
2. Robot reads each barcode
3. Robot adapts speed or direction per barcode
4. Robot ends at barcode "END"

**Success Criteria:**
- All barcodes read correctly
- Navigation matches expected sequence
- Robot completes course

**Evidence:** Console log showing barcode detections, video of run

---

#### 3.3 Obstacle Avoidance (SYS-03)

**Requirement:** Robot detects obstacles and takes evasive action

**Test Setup:**
- Line track with obstacles (boxes, cones) placed ahead at 20cm, 50cm, 100cm

**Test Steps:**
1. Robot approaches obstacle
2. Robot detects at target distance
3. Robot slows/stops
4. Robot steers around (optional) or waits
5. Robot resumes after obstacle passes

**Success Criteria:**
- Obstacle detected within ±5cm of setpoint distance
- Robot stops before collision
- Can resume after obstacle removed

**Evidence:** Video showing obstacle detection and response, console logs

---

#### 3.4 Heading Stability (SYS-04)

**Requirement:** Robot maintains heading despite obstacles and sensor noise

**Test Setup:**
- Flat, open area; no line, no obstacles

**Test Steps:**
1. Robot armed, stationary on level surface
2. Heading recorded via IMU
3. Robot driven forward in straight line
4. Heading monitored continuously
5. Verify minimal yaw drift

**Success Criteria:**
- Yaw drift < 5 degrees over 2-meter distance
- No oscillation in heading

**Evidence:** Logged IMU data, video showing straight path

---

#### 3.5 Telemetry & Remote Control (SYS-05)

**Requirement:** Robot connects to network and accepts remote commands

**Test Setup:**
- Robot and MQTT broker on same network
- Remote control client (dashboard or CLI MQTT publisher)

**Test Steps:**
1. Robot powers on and connects to WiFi
2. Telemetry starts publishing (heartbeat, sensor data)
3. Send VELOCITY command via MQTT
4. Robot executes command
5. Send STOP command
6. Robot stops

**Success Criteria:**
- WiFi connection successful within 10 seconds
- MQTT messages appear with < 500ms latency
- Commands execute within 100ms
- Robot responds to STOP immediately

**Evidence:** Console logs, MQTT client screenshots

---

#### 3.6 Dual-Core Performance (SYS-06)

**Requirement:** System operates stably on dual-core RP2040

**Test Setup:**
- Enable SMP (dual-core) build: `make SMP=1`
- Stress-test with all subsystems active

**Test Steps:**
1. Build and flash SMP firmware
2. Run for 5 minutes with continuous operation
3. Monitor CPU load and task scheduling
4. Check for deadlocks or race conditions

**Success Criteria:**
- No system hangs or crashes
- All tasks meet their deadlines
- Message buffers do not overflow

**Evidence:** Console output showing "dual-core" message, uptime

---

#### 3.7 Power & Startup (SYS-07)

**Requirement:** Robot powers on reliably and reaches operational state

**Test Setup:**
- Robot on bench, USB power only
- Measure startup time to first autonomous action

**Test Steps:**
1. Power on (USB or battery)
2. Count time to first motor movement or sensor reading
3. Verify all LEDs illuminate
4. Check console output for any errors

**Success Criteria:**
- Boot time < 5 seconds
- All subsystems initialize without errors
- Ready for commands

**Evidence:** Console log showing boot messages

---

#### 3.8 Long-Duration Run (SYS-08)

**Requirement:** Robot operates continuously for extended period (e.g., 1 hour)

**Test Setup:**
- Line track, obstacles, and waypoints for realistic operation

**Test Steps:**
1. Robot runs autonomously on line-following task
2. Monitor for 1 hour continuously
3. Log any errors or anomalies
4. Verify telemetry publishing throughout

**Success Criteria:**
- No crashes or hangs
- Consistent sensor readings
- Memory not leaking
- All telemetry messages published

**Evidence:** Console logs from entire run, uptime counter

---

## Testing Checklist

### Pre-Flight Checks (Before Each Test)

- [ ] Pico W powered and USB serial connected
- [ ] WiFi SSID and password configured in `config/wifi_secrets.h`
- [ ] MQTT broker IP and credentials correct
- [ ] All sensors connected and powered
- [ ] Motors and motor driver functioning
- [ ] Encoders reading pulses
- [ ] Console output visible (115200 baud, 8N1)

### Build & Flash Commands

```bash
# Build unit tests only
cd build_make
make TEST=1 -j8

# Build unit tests + template examples
make TEST=1 EXAMPLES=1 -j8

# Build release (no tests)
make -j8

# Single-core build (for comparison)
make TEST=1 SMP=0 -j8

# Flash the .uf2 file
# Hold BOOTSEL button, plug in USB, release button
# Drag mtk3pico_smp1_usb_cdc_wifi.uf2 to RPI-RP2 mass storage device
```

### Serial Console Capture

```bash
# macOS
cat /dev/cu.usbmodem* | tee testing/subsystemN/logs/2026-11-13_test.txt

# Linux / WSL
cat /dev/ttyACM0 | tee testing/subsystemN/logs/2026-11-13_test.txt
```

### Test Execution Workflow

1. **Define test** in appropriate `TEST_LOG.md` (subsystem or integration)
2. **Add automated check** if possible to `*_test.c`
3. **Build** with `make TEST=1`
4. **Flash** .uf2 to Pico
5. **Capture** console output to `logs/YYYY-MM-DD_<description>.txt`
6. **Record** result in `TEST_LOG.md`: date, ID, commit hash, tester, PASS/FAIL, log file
7. **Commit** both test log and console output file together

### Test Log Format

```markdown
## Test Cases

| ID | Test | How | Expected |
|---|---|---|---|
| INT-01 | Encoder → odometry | Drive 1m, verify distance | Distance ±5% |

## Results

| Date | Test | Commit | Tester | Result | Log |
|---|---|---|---|---|---|
| 2026-11-13 | INT-01 | abc1234 | @user | PASS | logs/2026-11-13_encoder_odometry.txt |
```

---

## Continuous Integration & Regression Testing

### Per-PR Checklist

Before merging any PR:
1. Run all unit tests (`make TEST=1`)
2. Verify no new test failures
3. If adding new subsystem, add unit tests to `*_test.c`
4. Update `TEST_LOG.md` with results
5. Capture and commit console logs

### Before Demo or Hand-In

Run complete system test suite:
```bash
# 1. Unit tests
make TEST=1 -j8
# Power cycle Pico, capture output
# (make TEST=1 EXAMPLES=1 also runs the template demo tests, but drives GP19:
#  disconnect the right encoder B wire first)

# 2. Host-side integration tests
cd testing/integration/i2c_host && gcc ... && ./test
cd testing/integration/imu_i2c_host && gcc ... && ./test
cd testing/integration/hcsr04_host && gcc ... && ./test

# 3. System tests (manual)
# Line following: drive track, record time
# Obstacle avoidance: place objects, verify stops
# Telemetry: verify MQTT messages
# Long-run: 30-60 minute autonomous operation
```

---

## Known Limitations & Workarounds

### 1. Host-Side Tests Do NOT Verify

- Actual I2C timing or bus timing
- Real sensor hardware
- GPIO interrupt latency
- Power consumption
- Temperature effects

**Workaround:** Always run final tests on actual robot hardware.

### 2. Simulated Odometry in Motion Tests

Motion controller tests (`motion_task_test`) use simulated encoder feedback, not real wheel motion.

**Workaround:** Run motion integration tests (INT-05) on robot with real motors and encoders.

### 3. WiFi Only Available on Pico W

Standard Pico (non-W) cannot run WiFi tests. WiFi tests (SYS-05) require Pico W.

---

## Troubleshooting

### Test Hangs or Timeout

**Problem:** Test runs but never completes

**Steps:**
1. Check console output for [TEST] messages
2. Verify Pico has power and is not in reset loop
3. Recompile with `make clean` and rebuild
4. Check if task is blocked in infinite loop (search code for `for(;;)`)

---

### Test Crashes (Prints ERROR or Fault)

**Problem:** Test starts but system crashes with error code

**Steps:**
1. Note the error code and subsystem
2. Check `app_program/subsystemN/subsystemN_test.c` for the test
3. Add `tm_printf` debug statements to isolate failure point
4. Verify hardware connections
5. Check for stack overflow: increase `TASK_STACK_SIZE` if needed

---

### Incorrect Sensor Readings

**Problem:** Sensor values make no sense (e.g., distance = 0xFFFF)

**Steps:**
1. Verify sensor powered and connected
2. Check I2C/ADC configuration in `config/`
3. Run integration test to verify communication
4. Measure voltage at sensor pins with multimeter
5. Swap sensor to verify hardware not faulty

---

### Memory Leaks or Slowdown Over Time

**Problem:** System works fine for a while, then slows down or crashes

**Steps:**
1. Check console for message buffer overflow warnings
2. Verify no infinite loops in tasks
3. Check for large stack allocations that could fragment memory
4. Run SYS-08 (long-duration test) to reproduce
5. Add heap size monitoring (`tk_ref_mpf` calls)

---

## Next Steps

1. **Start with Unit Tests:** Run `make TEST=1` and verify all [TEST] messages show PASS
2. **Run Host-Side Integration Tests:** Test IMU and obstacle sensor on development machine
3. **Test on Robot Hardware:** Encoder → odometry, line sensor, IMU tilt detection
4. **System Integration:** Obstacle avoidance, line following with controller
5. **End-to-End Validation:** Full autonomous runs with all subsystems active

---

## Appendix: Test Report Headers

Each `TEST_LOG.md` should start with:

```markdown
# Subsystem X Testing

Subsystem: [Name]
Owners: [Team members]
Last Updated: [Date]

## Test Cases

[Table of test definitions]

## Results

[Table of test execution results]

## Notes

[Any observations, known issues, or future work]
```

---

**Document Version:** 1.0  
**Last Updated:** 2026-11-13  
**Next Review:** After initial system integration
