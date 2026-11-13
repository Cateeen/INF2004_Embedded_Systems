# Test ID Reference & Tracking Matrix

Use this document to track which tests have been run and which still need completion.

---

## Unit Tests (Level 1)

### Odometry Subsystem (S1-Uxx)

| Test ID | Description | Type | Status | Commit | Log |
|---------|-------------|------|--------|--------|-----|
| S1-U01 | Encoder pulse counting accuracy | Auto | ⬜ | - | - |
| S1-U02 | Distance calculation from encoder counts | Auto | ⬜ | - | - |
| S1-U03 | Angle/heading calculation | Auto | ⬜ | - | - |
| S1-U04 | Two-encoder synchronization | Auto | ⬜ | - | - |
| S1-U05 | Zero count edge case handling | Auto | ⬜ | - | - |
| S1-U06 | Count wrapping and rollover | Auto | ⬜ | - | - |

**Build:** `make TEST=1 EXAMPLES=1`  
**Check for:** `[TEST] ODO PASS`, `[TEST] ODO-PRODUCER PASS`, `[TEST] ODO-TASK PASS`  
**Log location:** `testing/subsystem1/logs/`

---

### Line Detection & Barcode (S2-Uxx)

| Test ID | Description | Type | Status | Commit | Log |
|---------|-------------|------|--------|--------|-----|
| S2-U01 | ADC line sensor reading | Auto | ⬜ | - | - |
| S2-U02 | Line threshold detection | Auto | ⬜ | - | - |
| S2-U03 | Line classification (left/center/right) | Auto | ⬜ | - | - |
| S2-U04 | Confidence metric calculation | Auto | ⬜ | - | - |
| S2-U05 | Barcode scanning and decoding | Auto | ⬜ | - | - |
| S2-U06 | Intersection detection | Auto | ⬜ | - | - |
| S2-U07 | ADC reader producer task | Auto | ⬜ | - | - |

**Build:** `make TEST=1 EXAMPLES=1`  
**Check for:** `[TEST] LINE PASS`, `[TEST] BAR PASS`, `[TEST] LB-TASK PASS`, `[TEST] LINE-ADC PASS`  
**Log location:** `testing/subsystem2/logs/`

---

### IMU Subsystem (S3-Uxx)

| Test ID | Description | Type | Status | Commit | Log |
|---------|-------------|------|--------|--------|-----|
| S3-U01 | LSM303DLHC I2C communication | Auto | ⬜ | - | - |
| S3-U02 | Accelerometer data reading | Auto | ⬜ | - | - |
| S3-U03 | Magnetometer data reading | Auto | ⬜ | - | - |
| S3-U04 | Roll calculation from acceleration | Auto | ⬜ | - | - |
| S3-U05 | Pitch calculation from acceleration | Auto | ⬜ | - | - |
| S3-U06 | Yaw calculation from magnetometer | Auto | ⬜ | - | - |
| S3-U07 | Shock detection threshold | Auto | ⬜ | - | - |
| S3-U08 | Tilt detection and terrain classification | Auto | ⬜ | - | - |

**Build:** `make TEST=1 EXAMPLES=1`  
**Check for:** `[TEST] IMU PASS`, `[TEST] IMU-TASK PASS`  
**Host test:** `testing/integration/imu_i2c_host/test.c`  
**Log location:** `testing/subsystem3/logs/`

---

### Obstacle Avoidance (S4-Uxx)

| Test ID | Description | Type | Status | Commit | Log |
|---------|-------------|------|--------|--------|-----|
| S4-U01 | HC-SR04 pulse timing | Auto | ⬜ | - | - |
| S4-U02 | Distance calculation from echo time | Auto | ⬜ | - | - |
| S4-U03 | Multi-sensor scanning (left/center/right) | Auto | ⬜ | - | - |
| S4-U04 | Obstacle detection and classification | Auto | ⬜ | - | - |
| S4-U05 | Emergency stop trigger | Auto | ⬜ | - | - |
| S4-U06 | Command processing (scan/reset/abort) | Auto | ⬜ | - | - |

**Build:** `make TEST=1 EXAMPLES=1`  
**Check for:** `[TEST] OBS PASS`, `[TEST] OBS-TASK PASS`  
**Host test:** `testing/integration/hcsr04_host/test.c`  
**Log location:** `testing/subsystem4/logs/`

---

### Motion Control (S5-Uxx)

| Test ID | Description | Type | Status | Commit | Log |
|---------|-------------|------|--------|--------|-----|
| S5-U01 | Motor PWM generation | Auto | ⬜ | - | - |
| S5-U02 | Wheel speed feedback processing | Auto | ⬜ | - | - |
| S5-U03 | PID controller output | Auto | ⬜ | - | - |
| S5-U04 | Speed ramping profile | Auto | ⬜ | - | - |
| S5-U05 | Stop/Arm/Velocity/Turn mode switching | Auto | ⬜ | - | - |
| S5-U06 | Motion status reporting | Auto | ⬜ | - | - |
| S5-U07 | Motor stall detection | Auto | ⬜ | - | - |

**Build:** `make TEST=1 EXAMPLES=1`  
**Check for:** `[TEST] MOT PASS`, `[TEST] MOT-TASK PASS`  
**Log location:** `testing/subsystem5/logs/`

---

### Vehicle Controller (S6-Uxx)

| Test ID | Description | Type | Status | Commit | Log |
|---------|-------------|------|--------|--------|-----|
| S6-U01 | Line-following steering logic | Auto | ⬜ | - | - |
| S6-U02 | Speed adaptation based on confidence | Auto | ⬜ | - | - |
| S6-U03 | Obstacle response (stop/turn/reverse) | Auto | ⬜ | - | - |
| S6-U04 | Intersection handling | Auto | ⬜ | - | - |
| S6-U05 | Barcode waypoint processing | Auto | ⬜ | - | - |
| S6-U06 | Safety gates (speed limits, emergency stop) | Auto | ⬜ | - | - |
| S6-U07 | State machine transitions | Auto | ⬜ | - | - |

**Build:** `make TEST=1 EXAMPLES=1`  
**Check for:** `[TEST] VC PASS`, `[TEST] VC-MONITOR PASS`, `[TEST] VC-TASK PASS`  
**Log location:** `testing/subsystem6/logs/`

---

### GPIO & Interrupts (S7-Uxx)

| Test ID | Description | Type | Status | Commit | Log |
|---------|-------------|------|--------|--------|-----|
| S7-U01 | GPIO interrupt dispatch | Auto | ⬜ | - | - |
| S7-U02 | Pin assignment verification | Auto | ⬜ | - | - |
| S7-U03 | Rising edge detection | Auto | ⬜ | - | - |
| S7-U04 | Falling edge detection | Auto | ⬜ | - | - |
| S7-U05 | Interrupt latency measurement | Manual | ⬜ | - | - |

**Build:** `make TEST=1 EXAMPLES=1`  
**Check for:** `[TEST] GPIO PASS`  
**Log location:** `testing/subsystem7/logs/`

---

### WiFi & MQTT (S8-Uxx)

| Test ID | Description | Type | Status | Commit | Log |
|---------|-------------|------|--------|--------|-----|
| S8-U01 | WiFi connection to SSID | Auto | ⬜ | - | - |
| S8-U02 | MQTT broker connection | Auto | ⬜ | - | - |
| S8-U03 | Telemetry message formatting | Auto | ⬜ | - | - |
| S8-U04 | Command reception and parsing | Auto | ⬜ | - | - |
| S8-U05 | Network reconnection after loss | Manual | ⬜ | - | - |
| S8-U06 | MQTT keep-alive / heartbeat | Auto | ⬜ | - | - |

**Build:** `make TEST=1 EXAMPLES=1`  
**Check for:** `[TEST] WIFI PASS`, `[TEST] WIFI-BRIDGE PASS`  
**Log location:** `testing/subsystem8/logs/`

---

## Integration Tests (Level 2)

| Test ID | Description | Subsystems | Type | Status | Commit | Log |
|---------|-------------|------------|------|--------|--------|-----|
| INT-01 | Encoder → Odometry distance | S1+S5 | Auto | ⬜ | - | - |
| INT-02 | Line Sensor → Observation | S2 ADC input | Auto | ⬜ | - | - |
| INT-03 | IMU → Orientation (Host test) | S3 | Auto | ⬜ | - | testing/integration/imu_i2c_host/ |
| INT-04 | HC-SR04 → Distance (Host test) | S4 | Auto | ⬜ | - | testing/integration/hcsr04_host/ |
| INT-05 | Motion + Odometry feedback | S5+S1 | Auto | ⬜ | - | - |
| INT-06 | Controller + Obstacle Safety | S6+S4 | Manual | ⬜ | - | - |
| INT-07 | Line + Barcode Navigation | S2+S6 | Manual | ⬜ | - | - |
| INT-08 | Telemetry → WiFi Publishing | S8 | Manual | ⬜ | - | - |

**Log location:** `testing/integration/TEST_LOG.md`

---

## System Tests (Level 3)

| Test ID | Description | Hardware | Type | Success Criteria | Status | Commit | Log |
|---------|-------------|----------|------|------------------|--------|--------|-----|
| SYS-01 | Line Following | Track, 3m | Manual | Complete without leaving line | ⬜ | - | - |
| SYS-02 | Barcode Navigation | Track + barcodes | Manual | Read all barcodes, correct sequence | ⬜ | - | - |
| SYS-03 | Obstacle Avoidance | Track + objects | Manual | Detect at ±5cm, stop before collision | ⬜ | - | - |
| SYS-04 | Heading Stability | Open area | Manual | Yaw drift < 5° over 2m | ⬜ | - | - |
| SYS-05 | Telemetry & Remote | Network | Manual | Commands execute < 100ms | ⬜ | - | - |
| SYS-06 | Dual-Core Stability | Stress test | Manual | 5 min runtime, no hangs | ⬜ | - | - |
| SYS-07 | Power & Startup | USB/Battery | Manual | Boot < 5s, no errors | ⬜ | - | - |
| SYS-08 | Long-Duration Run | Track, 1 hour | Manual | No crashes, consistent perf | ⬜ | - | - |

**Log location:** `testing/system/TEST_LOG.md`

---

## Status Legend

| Symbol | Meaning |
|--------|---------|
| ⬜ | Not started |
| 🟨 | In progress |
| ✅ | PASS |
| ❌ | FAIL |

---

## Quick Status Summary

```
Level 1 (Unit Tests):    [ _ / 56 ] Complete
Level 2 (Integration):   [ _ /  8 ] Complete
Level 3 (System):        [ _ /  8 ] Complete

OVERALL: [ _ / 72 ] Tests Passed
```

---

## How to Use This Document

1. **After each test run,** update the Status column (✅, ❌, or 🟨)
2. **Record the commit hash** (first 7 chars from `git rev-parse --short HEAD`)
3. **Link the test log file** if not auto-captured
4. **Update the summary** at the bottom
5. **Commit this document** with your test results

Example update:

```
| S1-U01 | Encoder pulse... | Auto | ✅ | abc1234 | testing/subsystem1/logs/2026-11-13_encoder.txt |
```

---

## Tracking Commands

Print current test matrix:
```bash
# Count complete tests
grep "✅" TESTING_REFERENCE.md | wc -l

# Count failed tests
grep "❌" TESTING_REFERENCE.md | wc -l

# Count in-progress
grep "🟨" TESTING_REFERENCE.md | wc -l
```

---

## Submission Checklist

Before considering testing complete:

- [ ] All unit tests (S1-S8) show ✅ PASS
- [ ] All integration tests (INT-01 through INT-08) show ✅ PASS
- [ ] All system tests (SYS-01 through SYS-08) show ✅ PASS
- [ ] All test logs committed to `testing/*/logs/`
- [ ] TEST_LOG.md files updated for each subsystem
- [ ] TESTING_REFERENCE.md shows [ 72 / 72 ] Complete
- [ ] No ERROR messages in final system test run
- [ ] Robot operates continuously for 30+ minutes without crash

---

**First Test Target:** S1-U01 through S8-U06 (all unit tests)  
**Second Target:** INT-01 through INT-08 (all integration tests)  
**Final Target:** SYS-01 through SYS-08 (all system tests)

---

**Document Updated:** 2026-11-13  
**Total Tests:** 72  
**Current Progress:** 0/72 ⬜

Start with QUICK_TEST_CHECKLIST.md to run your first tests!
