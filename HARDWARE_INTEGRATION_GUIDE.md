# Hardware Verification & Integration Guide

## Pin Configuration Summary

This guide helps you verify that all sensors and motors are wired correctly and communicating.

### I2C Bus (Sensors: IMU, possibly other I2C devices)

| Pin | Function | Device | Notes |
|-----|----------|--------|-------|
| GPIO4 | I2C0_SDA | LSM303DLHC Accel/Mag | Pull-up required (~4.7kΩ to 3.3V) |
| GPIO5 | I2C0_SCL | LSM303DLHC Accel/Mag | Pull-up required (~4.7kΩ to 3.3V) |
| 3.3V | Power | I2C devices | 50-100mA available for all I2C peripherals |
| GND | Ground | All | Common ground with robot power system |

**Verify I2C Communication:**

```bash
cd testing/integration/imu_i2c_host
gcc -std=c11 -Wall -Wextra -Werror -Isys -I../../../app_program/imu \
  test.c ../../../app_program/imu/lsm303dlhc_i2c.c \
  ../../../app_program/imu/lsm303dlhc_reader.c \
  ../../../app_program/imu/lsm303dlhc_accel.c -o imu_test
./imu_test
```

If this passes on your dev machine, I2C driver code is correct. If on-robot test fails:
- Check SDA/SCL voltages: should be ~3.3V when idle, drop to 0V during transfers
- Measure pull-up resistors: should be ~4.7kΩ to 3.3V
- Verify LSM303DLHC address with `i2cdetect` tool (address 0x19 for accel, 0x1E for mag)

---

### ADC Channels (Line Sensors)

| Pin | ADC Channel | Function | Notes |
|-----|-------------|----------|-------|
| GPIO26 | ADC0 | Line sensor 0 (typically left) | 0-3.3V analog input |
| GPIO27 | ADC1 | Line sensor 1 (center-left) | 0-3.3V analog input |
| GPIO28 | ADC2 | Line sensor 2 (center-right) | 0-3.3V analog input |
| GPIO29 | ADC3 | Line sensor 3 (right) | 0-3.3V analog input |
| (+more) | ADC0-3 | Additional sensors | Max 4 ADC inputs on RP2040 |

**Expected Voltage Ranges:**
- White surface: 0.0V - 0.5V (dark)
- Black surface: 2.5V - 3.3V (bright due to IR reflection)
- Gray (line boundary): 1.0V - 2.0V

**Verify ADC Reading:**

1. Open console, monitor `line_adc_reader_test()` output
2. Point sensor at white surface: should read low values
3. Point sensor at black line: should read high values
4. Sweep sensor across line: values should transition smoothly

---

### GPIO: Encoders

| Pin | Function | Motor | Notes |
|-----|----------|-------|-------|
| GPIOXX | Encoder A | Left wheel | Rising edge = count increment |
| GPIOXX | Encoder B | Left wheel | Determines direction (forward/reverse) |
| GPIOXX | Encoder A | Right wheel | Rising edge = count increment |
| GPIOXX | Encoder B | Right wheel | Determines direction |

**Check in:** `app_program/odometry/encoder.h` for actual GPIO assignments

**Verify Encoder Signals:**

1. Spin left wheel by hand while watching console
2. Should see left encoder count increase
3. Spin right wheel: right count should increase
4. Reverse both wheels: counts should increment regardless of direction

If counts don't change:
- Check GPIO pins connected to encoder outputs
- Verify encoder outputs are logic-level (0V/3.3V or 0V/5V with level shifter)
- Use oscilloscope to confirm edges on GPIO pins

---

### GPIO: Motor PWM

| Pin | Function | Motor | Notes |
|-----|----------|-------|-------|
| GPIOXX | PWM A | Left motor | 0-100% duty cycle controls speed |
| GPIOXX | PWM B | Right motor | 0-100% duty cycle controls speed |

**Check in:** `app_program/motion/motor_driver.h` for actual GPIO assignments

**Verify Motor PWM:**

1. Robot on blocks (wheels off ground)
2. Send VELOCITY command for forward motion
3. Listen for motor sound/vibration
4. Check PWM with oscilloscope:
   - Frequency: typically 1-20 kHz
   - Duty cycle: 0% (stopped) to 100% (full speed)
   - When moving forward: both motors at same duty cycle

If motors don't move:
- Check motor driver power supply voltage
- Verify PWM pins connected to motor driver control inputs
- Check motor power connectors (may need separate 6-12V supply)

---

### GPIO: Ultrasonic Sensors (HC-SR04)

| Pin | Function | Direction | Notes |
|-----|----------|-----------|-------|
| GPIOXX | Trigger | Output (Pico → Sensor) | 10μs pulse starts measurement |
| GPIOXX | Echo | Input (Sensor → Pico) | High time = distance (us/58 = cm) |
| (repeat) | Trigger | Front | Multiple sensors possible |
| (repeat) | Echo | Front | |

**Check in:** `app_program/obstacle/obs_pico_port.h` for actual GPIO assignments

**Verify Ultrasonic Sensor:**

```bash
cd testing/integration/hcsr04_host
gcc -std=c11 -Wall -Wextra -Werror -I../../../app_program/obstacle \
  test.c ../../../app_program/obstacle/hcsr04.c \
  ../../../app_program/obstacle/hcsr04_capture.c -o hcsr04_test
./hcsr04_test
```

On robot:
1. Place hand 20cm from sensor
2. Check console for distance_mm ≈ 200
3. Move hand away, distance should increase
4. Wait 30ms between measurements (sensor needs cool-down)

If distance is always 0xFFF (max):
- Check trigger pin connected and toggling
- Check echo pin connected and has pull-up resistor
- Verify sensor power (5V, 15-50mA), not 3.3V
- Use oscilloscope to measure echo pulse width

---

### GPIO: Other Hardware

| Pin | Function | Device | Typical Usage |
|-----|----------|--------|---------------|
| GPIOXX | Status LED | On-board or external | Blink pattern shows system state |
| GPIOXX | Safety button | Emergency stop | Momentary connection to GND |

**Check in:** Subsystem header files for exact GPIO assignments

---

## Testing Protocol: Hardware Integration Checklist

### Step 1: Power & Connection Verification

- [ ] Pico W powered via USB (5V, 0.5A minimum for base; 1A recommended with sensors)
- [ ] All sensor power lines connected (3.3V for I2C/logic, 5V for ultrasonic)
- [ ] Common ground between Pico and all peripherals
- [ ] No shorts between power and ground
- [ ] Multimeter: Measure 3.3V and 5V rails under load

**Test Command:**
```bash
make TEST=1 -j8
# Look for startup message confirming microcontroller is alive
```

---

### Step 2: I2C Bus Verification

**On Robot:**
1. Build and flash `make TEST=1`
2. Watch console for I2C error messages
3. If test hangs on I2C init, check:
   - Voltage on SDA/SCL pins (should be ~3.3V idle)
   - Pull-up resistors present and correct value
   - No short circuits on I2C pins
   - I2C address correct (default 0x19 for accel, 0x1E for mag)

**On Host (Linux/WSL):**
```bash
cd testing/integration/imu_i2c_host
gcc -std=c11 -Wall -Wextra -Werror -Isys -I../../../app_program/imu \
  test.c ../../../app_program/imu/lsm303dlhc_i2c.c \
  ../../../app_program/imu/lsm303dlhc_reader.c \
  ../../../app_program/imu/lsm303dlhc_accel.c -o imu_test
./imu_test
# If passes on host, I2C protocol code is correct
# If fails on robot, issue is electrical/I2C bus
```

---

### Step 3: ADC (Line Sensor) Verification

1. **White surface test:**
   - Point sensor over white paper
   - Console should show low ADC values (< 500)

2. **Black surface test:**
   - Point sensor over black tape
   - Console should show high ADC values (> 2000)

3. **Threshold test:**
   - Gradually transition from white to black
   - Confidence value should be 255 when clearly over line
   - Lateral error should be 0 when centered

**If ADC reads all zeros:**
- Check GPIO pin connected to ADC channel
- Verify capacitive coupling on ADC input (if present)
- Check ADC reference voltage (should be 3.3V)

**If ADC reads all 4095:**
- Check sensor is powered
- Verify sensor output voltage (0-3.3V, not 0-5V without level shifter)
- Check for stuck connection (floating pin)

---

### Step 4: GPIO Interrupt Verification (Encoders & Ultrasonic)

**Encoder Test:**
1. Spin left wheel by hand
2. Console should show increasing left encoder count
3. Spin right wheel: right count increases
4. Check direction bits are correct (forward vs. backward)

**If counts don't increase:**
- Verify GPIO pins in oscilloscope (should see 0V/3.3V edges)
- Check encoder polarity (may need to swap A/B channels)
- Verify interrupt handler is registered in GPIO dispatcher

**Ultrasonic Test:**
1. Place hand in front of sensor
2. Console should show distance decreasing as hand approaches
3. Remove hand: distance returns to 3000+ mm (no object)

**If ultrasonic always reads max (0xFFF):**
- Check echo capture is enabled in `gpio_irq_router`
- Verify echo pin connected and toggling on oscilloscope
- Check trigger pulse is 10μs width (may be too short/long)

---

### Step 5: Motor & Speed Control Verification

**Visual/Audio Test:**
1. Command VELOCITY mode (linear_velocity = 50 mm/s)
2. Listen for motor humming
3. Feel wheel resistance (should rotate with small effort)

**Odometry Feedback Test:**
1. Command forward motion
2. Measure wheel movement distance with tape
3. Compare against reported odometry distance
4. Target: Within 5% accuracy

**PWM Signal Test (Oscilloscope):**
1. Connect scope to PWM pin
2. Command motion
3. Observe:
   - Frequency: 1-20 kHz (typical)
   - Duty cycle changes with speed command
   - Both motors same frequency/phase

**If motors don't move:**
- Check motor power supply voltage (6-12V, separate from Pico 5V)
- Verify motor driver IC is powered
- Check direction pins (if using H-bridge)
- Measure back-EMF when spinning by hand (indicates coils OK)

---

### Step 6: WiFi & Telemetry Verification

**Prerequisites:**
- WiFi SSID and password correct in `config/wifi_secrets.h`
- MQTT broker IP and port correct
- Robot in range of WiFi network

**Test Steps:**
1. Open console
2. Look for WiFi connection message (usually within 5 seconds of boot)
3. Verify MQTT broker connection
4. Use MQTT client to subscribe to telemetry topic:
   ```bash
   mosquitto_sub -h 192.168.1.10 -t "mtk3smp-pico/heartbeat" -v
   ```
5. Should see heartbeat message every 10 seconds

**If WiFi fails to connect:**
- Verify SSID is correct (case-sensitive)
- Check password in `wifi_secrets.h`
- Confirm WiFi network is 2.4GHz (Pico W doesn't support 5GHz)
- Check antenna connection on Pico W (small metal piece should be at angle)

**If MQTT connection fails:**
- Verify broker IP address correct
- Ping broker from Pico machine: `ping 192.168.1.10`
- Check firewall allowing MQTT port 1883
- Verify broker is running (`sudo systemctl status mosquitto`)

---

## Debugging Tools & Techniques

### Console Output Analysis

```
[TEST] ODO PASS       ✓ Odometry tests passed
[TEST] IMU FAIL       ✗ IMU test failed - check I2C
[TEST] OBS PASS       ✓ Obstacle detection OK
ERROR: [code]         ✗ System error - halt here
```

**Common errors:**
- `ERROR: I2C timeout` - SDA/SCL stuck low, check pull-ups
- `ERROR: ADC not ready` - ADC not initialized, check config
- `ERROR: MQTT connection failed` - Network/broker issue
- `ERROR: Motor stalled` - Odometry doesn't match motor PWM

### Oscilloscope / Logic Analyzer

**Key signals to check:**
1. **I2C bus:** SDA and SCL should toggle together, not stuck
2. **Encoder A/B:** Rising edges should match wheel rotation
3. **Ultrasonic echo:** Pulse width proportional to distance
4. **Motor PWM:** Frequency stable, duty cycle changes with speed command
5. **GPIO interrupts:** Signal edges aligned with interrupt handler timing

### Multimeter Checks

| Measurement | Expected | If Wrong |
|---|---|---|
| 3.3V rail (idle) | 3.3V ± 0.1V | Voltage regulator failure or short |
| 5V rail (idle) | 5.0V ± 0.2V | Power supply issue |
| I2C SDA (idle) | 2.8-3.3V | Stuck low: short or missing pull-up |
| Motor supply | 6-12V | Motor power issue |
| GND continuity | < 0.1Ω | Broken ground connection |

### Adding Debug Output

Edit `app_program/subsystemN/subsystemN.c` to add debug prints:

```c
if (result < E_OK) {
    tm_printf((UB*)"[DEBUG] I2C read failed: %d at address 0x%02x\n", 
              (int)result, i2c_address);
}
```

Rebuild: `make TEST=1 -j8`

---

## Integration Testing Workflow

### Phase 1: Individual Subsystem Tests (on Robot)

```bash
# Build with tests
cd build_make
make TEST=1 -j8

# Flash to Pico, capture output
# Each [TEST] message indicates one subsystem ready
```

**Expected Output:**
```
[TEST] ODO PASS      odometry calculations
[TEST] LINE PASS     line classification
[TEST] IMU PASS      IMU processing
[TEST] OBS PASS      obstacle profiling
[TEST] MOT PASS      wheel speed control
[TEST] GPIO PASS     interrupt routing
```

All should show PASS before proceeding.

---

### Phase 2: Pairs of Subsystems (on Robot)

Once all units pass, test interaction pairs:

| Pair | How to Test | Expected |
|---|---|---|
| Encoder → Odometry | Spin wheel 10 rotations, check distance | ~3000mm ± 150mm |
| Line → Controller | Drive on line, check lateral error changes | Smooth oscillation, not erratic |
| IMU → Status | Tilt robot, check roll/pitch angles | Proportional to tilt angle |
| Obstacle → Stop | Drive toward wall, check stop distance | Stop before 10cm |
| Motor + Encoder | Command speed, check speed feedback | Commanded ≈ actual ±10% |

---

### Phase 3: Full System (Track Test)

1. **Simple track:** Straight line, 2 meters
   - Robot starts, follows line, reaches end
   - Time < 30 seconds

2. **Obstacle test:** Line track with box 50cm away
   - Robot approaches, detects, stops
   - Resumes after obstacle removed

3. **Long run:** 30 minutes continuous operation
   - No crashes, consistent performance
   - Memory not leaking (log file size stable)

---

## Sensor Datasheets & Reference

### LSM303DLHC (Accelerometer/Magnetometer)

- **I2C Address:** 0x19 (accelerometer), 0x1E (magnetometer)
- **Full scale:** ±2g, ±4g, ±8g, ±16g (configurable)
- **Sensitivity:** 1mg/LSB (at ±2g)
- **Datasheet:** Search "LSM303DLHC datasheet" for detailed register map

### HC-SR04 (Ultrasonic Range)

- **Measurement range:** 2cm - 400cm
- **Accuracy:** ±3%
- **Trigger pulse:** 10μs at least once per 60ms
- **Echo formula:** Distance (cm) = Echo time (μs) / 58
- **Datasheet:** Search "HC-SR04 datasheet" for timing diagrams

### Typical Line Sensor

- **Type:** Reflective IR sensor array (8-16 channels)
- **Output:** Analog voltage 0-3.3V per channel
- **Threshold:** ~1.5-2V for line/no-line decision
- **Typical values:** White: 0.2V, Black: 2.8V

---

## Final Verification Checklist

Before considering your pico car "integration complete":

- [ ] All unit tests pass ([TEST] PASS for every subsystem)
- [ ] No ERROR messages in console output
- [ ] Encoder counts increase when wheels spin
- [ ] Line sensor shows transition when moving sensor across line
- [ ] IMU shows roll/pitch changing when tilting
- [ ] Ultrasonic distance changes when approaching obstacle
- [ ] Motors start spinning when motion commands sent
- [ ] WiFi connects within 10 seconds of boot
- [ ] MQTT telemetry appears in subscription
- [ ] Robot drives in straight line for 2 meters
- [ ] Robot stops within 15cm of obstacle
- [ ] System runs continuously for 30+ minutes without crash

---

**Ready to Roll!** Once all checks pass, your pico car is ready for autonomous missions. 🚀
