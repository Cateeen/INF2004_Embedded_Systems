# Testing & Integration Summary - Pico Car Project

## What You Have

A comprehensive autonomous robot car system with 8 subsystems:
- **Odometry:** Wheel encoder tracking
- **Line Detection & Barcode:** ADC-based line following with barcode waypoints
- **IMU:** Orientation and shock detection
- **Obstacle Avoidance:** Ultrasonic range sensing
- **Motion Control:** Motor PWM and speed PID
- **Vehicle Controller:** High-level steering and safety logic
- **GPIO/Interrupts:** Hardware event routing
- **WiFi/MQTT:** Network telemetry and remote commands

All running on **dual-core Raspberry Pi Pico W** with **micro T-Kernel 3.0** RTOS.

---

## Three Documents Created for You

### 1. **QUICK_TEST_CHECKLIST.md** ← START HERE (15 minutes)
**Purpose:** Get your first test running today

**What you'll do:**
1. Build unit tests: `make TEST=1 -j8`
2. Flash to Pico and capture console output
3. Verify all subsystems work
4. Save test results

**Success criteria:** See `[TEST] PASS` messages for all subsystems

---

### 2. **TESTING_AND_INTEGRATION_PLAN.md** ← Full Strategy
**Purpose:** Complete testing roadmap (all levels)

**Contains:**
- **Level 1 (Unit Tests):** 8 subsystems × 8+ test cases each
  - Odometry, line detection, IMU, obstacle, motion, controller, GPIO, WiFi
  - Where the tests live, how to build/run them
  
- **Level 2 (Integration Tests):** 8 subsystem pairs
  - Encoder→Odometry, Line→Observation, IMU→Orientation, Obstacle→Event
  - Motion+Odometry, Controller+Safety, Line+Navigation, Telemetry→WiFi
  - Host-side and robot-side testing approaches

- **Level 3 (System Tests):** 8 end-to-end product tests
  - Line following, barcode navigation, obstacle avoidance, heading stability
  - Telemetry & remote control, dual-core performance, power-up, long-duration runs
  - Video evidence and success criteria for each

**How to use:** Pick a test from each level, follow the steps, record results in TEST_LOG.md

---

### 3. **HARDWARE_INTEGRATION_GUIDE.md** ← Troubleshooting
**Purpose:** Verify all sensors/motors are wired correctly

**Contains:**
- **Pin configuration table:** I2C, ADC, GPIO, PWM assignments
- **Verification protocol:** Step-by-step checks for each subsystem
  - I2C bus (IMU)
  - ADC (line sensors)
  - GPIO (encoders, ultrasonic)
  - Motor PWM
  - WiFi connectivity

- **Debugging tools:** Oscilloscope signals, multimeter measurements, console error codes
- **Integration testing workflow:** Phase 1 (individual), Phase 2 (pairs), Phase 3 (full system)

**How to use:** If a test fails, follow the troubleshooting steps for that subsystem

---

## Testing Flow Chart

```
START
  ↓
Run QUICK_TEST_CHECKLIST
  ├─ Build: make TEST=1
  ├─ Flash: Copy .uf2 to Pico
  ├─ Capture: cat /dev/ttyACM0 | tee output.txt
  └─ Save: testing/subsystem*/logs/
  ↓
All [TEST] PASS?
  ├─ NO → Check HARDWARE_INTEGRATION_GUIDE
  │       (Fix GPIO/I2C/ADC issues)
  │       → Back to "Run QUICK_TEST_CHECKLIST"
  │
  └─ YES → Proceed to Level 2 Integration Tests
        ├─ Pick an integration test from TESTING_AND_INTEGRATION_PLAN
        ├─ Run it (manual or automated)
        ├─ Record result in testing/integration/TEST_LOG.md
        └─ Repeat for other integration tests
             ↓
          Level 2 tests passing?
             ├─ NO → Use HARDWARE_INTEGRATION_GUIDE to debug
             └─ YES → Proceed to Level 3 System Tests
                    ├─ Line following on actual track
                    ├─ Obstacle avoidance test
                    ├─ Telemetry to WiFi
                    ├─ Long-duration run (30 mins)
                    └─ Record all in testing/system/TEST_LOG.md
                        ↓
                    System tests passing?
                        ├─ NO → Back to Level 1-2 debugging
                        └─ YES → READY FOR DEPLOYMENT! 🚀
```

---

## Build Commands Quick Reference

```bash
# Unit tests (all subsystems)
cd build_make && make TEST=1 -j8

# Unit tests + template examples
make TEST=1 EXAMPLES=1 -j8

# Release build (no tests)
make -j8

# Single-core build (for comparison)
make TEST=1 SMP=0 -j8

# Clean rebuild
make clean && make TEST=1 -j8
```

---

## Test Recording Template

For each test you run, update the appropriate `TEST_LOG.md`:

```markdown
## Test Cases

| ID | Test | How | Expected |
|---|---|---|---|
| INT-01 | Encoder→Odometry | Drive 1m, verify distance | Distance ±5% |
| INT-02 | Line→Observation | Sweep sensor left/right | Lateral error proportional |

## Results

| Date | Test | Commit | Tester | Result | Log |
|---|---|---|---|---|---|
| 2026-11-13 | INT-01 | abc1234 | @username | PASS | logs/2026-11-13_encoder.txt |
| 2026-11-13 | INT-02 | abc1234 | @username | PASS | logs/2026-11-13_line.txt |
```

**Important:** Always record the commit hash (`git rev-parse --short HEAD`)

---

## Key Milestones

### Milestone 1: Unit Tests Complete ✓
- All 8 subsystems show `[TEST] PASS`
- No ERROR messages in console
- Estimated time: 1 day

### Milestone 2: Integration Tests Complete ✓
- All 8 subsystem pairs tested
- Motor speeds match controller commands
- Line following steady on straight sections
- Obstacle detection working reliably
- Estimated time: 3-5 days

### Milestone 3: System Tests Complete ✓
- Robot follows line track autonomously
- Stops for obstacles, resumes after
- Barcode navigation working
- WiFi telemetry working
- 30-minute stress test passes
- Estimated time: 1 week

### Milestone 4: Deployment Ready 🚀
- All tests passing and documented
- Test logs committed to git
- Hardware verified and validated
- Ready for demo or competition

---

## Common Issues & Solutions

### Problem: Tests hang or crash
**Solution:** Check HARDWARE_INTEGRATION_GUIDE for GPIO/I2C/power issues

### Problem: Sensor reads garbage values
**Solution:** Verify pin connections, check voltage with multimeter

### Problem: Motor won't move
**Solution:** Check motor power supply (separate 6-12V), verify motor driver IC

### Problem: WiFi won't connect
**Solution:** Verify SSID/password in config/wifi_secrets.h, check 2.4GHz network

### Problem: Odometry distance is wrong
**Solution:** Calibrate wheel circumference, verify encoder pulses per rotation

### Problem: Tests take too long
**Solution:** Some tests run every subsystem; filter with smaller make targets

---

## When to Move On

| Milestone | Criteria to Proceed |
|---|---|
| Unit Tests | All [TEST] PASS, no ERROR messages |
| Integration Tests | 6/8 integration tests passing |
| System Tests | Autonomous line following works, 5+ min uptime |
| Deployment | 8/8 system tests passing, git history clean |

---

## Document Navigation

| Want to... | Read... |
|---|---|
| Get first test running in 15 min | QUICK_TEST_CHECKLIST.md |
| Understand full testing strategy | TESTING_AND_INTEGRATION_PLAN.md |
| Fix hardware connection issues | HARDWARE_INTEGRATION_GUIDE.md |
| Know what to do next | This document (TESTING_SUMMARY.md) |

---

## Next Steps

1. **TODAY:** Follow QUICK_TEST_CHECKLIST.md (15 minutes)
2. **THIS WEEK:** Complete one Level 2 integration test
3. **NEXT WEEK:** Run full Level 3 system test suite
4. **THEN:** Deploy with confidence! 🎉

---

## Support Resources

- **Pico SDK:** https://github.com/raspberrypi/pico-sdk (for hardware reference)
- **LSM303DLHC Datasheet:** Search "LSM303DLHC datasheet" (I2C addresses, registers)
- **HC-SR04 Guide:** Search "HC-SR04 datasheet" (ultrasonic timing)
- **micro T-Kernel:** Built-in, see app_main.c comments for API usage

---

**Last Updated:** 2026-11-13  
**Status:** Ready for testing  
**Next Review:** After Milestone 1 completion

Good luck! 🚀
