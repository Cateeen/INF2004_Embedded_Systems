# Quick Test Checklist - Get Started in 15 Minutes

## Phase 1: Verify Build & Unit Tests (5 mins)

```bash
cd build_make
make TEST=1 -j8
```

Expected: Build completes successfully, creates `mtk3pico_smp1_usb_cdc_wifi.uf2`

---

## Phase 2: Flash & Capture Output (5 mins)

1. **Prepare Pico for flash:**
   - Hold BOOTSEL button on Pico W
   - Plug in USB cable
   - Release BOOTSEL (Pico appears as mass storage device labeled RPI-RP2)

2. **Copy firmware:**
   ```bash
   # macOS/Linux
   cp build_make/mtk3pico_smp1_usb_cdc_wifi.uf2 /Volumes/RPI-RP2/
   # Or drag-and-drop in file explorer
   ```

3. **Open console in new terminal:**
   ```bash
   # macOS
   cat /dev/cu.usbmodem* | tee test_output.txt
   
   # Linux / WSL
   cat /dev/ttyACM0 | tee test_output.txt
   ```

4. **Power on Pico:** Unplug and re-plug USB (or press RESET button)

5. **Watch for tests:** You should see:
   ```
   === micro T-Kernel on Raspberry Pi Pico W ...
   [TEST] INT-03 PASS  vehicle buffers
   [TEST] ODO PASS     odometry calculations
   [TEST] ODO-PRODUCER PASS
   [TEST] LINE PASS    line classification
   ...
   [TEST] === test run end ===
   ```

6. **Save output:** Stop with Ctrl-C, log is saved to `test_output.txt`

---

## Phase 3: Quick Subsystem Health Checks (5 mins)

After unit tests pass, manually verify each subsystem:

### ✓ Encoders Working?
- **Check:** Console should show encoder pulse counts incrementing
- **If fails:** Verify encoder GPIO pins connected, check for voltage on pins

### ✓ Line Sensors Ready?
- **Check:** Hover line sensor over black and white surfaces
- **Expected:** Readings should change significantly
- **If fails:** Check ADC channel configuration, verify sensor power

### ✓ IMU Responding?
- **Check:** Tilt robot 45 degrees, check console for roll/pitch values
- **Expected:** `roll_mdeg` should show ~45000
- **If fails:** Check I2C address (0x19 or 0x1E), verify pull-ups on SDA/SCL

### ✓ Ultrasonic Working?
- **Check:** Place hand 20cm from HC-SR04, watch console
- **Expected:** distance_mm ≈ 200
- **If fails:** Check trigger pin (GPIO), echo pin (GPIO), verify 5V power

### ✓ Motors Spinning?
- **Check:** Listen for motor hum when motion task starts
- **Expected:** Gentle humming or rotation
- **If fails:** Check motor driver power, verify PWM pins connected

### ✓ WiFi Connected?
- **Check:** Console shows MQTT connection message
- **Expected:** "MQTT connected to 192.168.1.10:1883"
- **If fails:** Verify SSID and password in `config/wifi_secrets.h`, restart

---

## Phase 4: Run First Integration Test (Optional, 10 mins)

If you want to test on the development machine (no robot needed):

```bash
# Test obstacle sensor distance conversion
cd testing/integration/hcsr04_host
gcc -std=c11 -Wall -Wextra -Werror -I../../../app_program/obstacle \
  test.c ../../../app_program/obstacle/hcsr04.c \
  ../../../app_program/obstacle/hcsr04_capture.c -o hcsr04_test
./hcsr04_test
```

Expected output:
```
Testing HC-SR04 distance calculations...
Test case 1: Echo width 1176 us → 20.0 mm ✓
...
All tests passed!
```

---

## Phase 5: Record Your Results

1. **Create test log:**
   ```bash
   mkdir -p testing/subsystemX/logs
   cp test_output.txt testing/subsystemX/logs/2026-11-13_unit_tests.txt
   ```

2. **Update TEST_LOG.md:** Add a row:
   ```markdown
   | 2026-11-13 | UNIT | abc1234 | Your Name | PASS | logs/2026-11-13_unit_tests.txt |
   ```

3. **Commit:**
   ```bash
   git add testing/
   git commit -m "Unit tests: all subsystems verified on 2026-11-13"
   ```

---

## Common Issues & Quick Fixes

| Issue | Fix |
|-------|-----|
| "command not found: arm-none-eabi-gcc" | Add to ~/.bashrc: `export PATH="$HOME/tools/arm-gnu-toolchain-*/bin:$PATH"` |
| "PICO_SDK_PATH not found" | Set: `export PICO_SDK_PATH="$HOME/pico/pico-sdk"` |
| No USB device appears | Try different USB cable, different USB port, hold BOOTSEL longer |
| Tests hang after boot | Check power supply (needs 5V, 1A minimum), restart Pico |
| Console garbage text | Check baud rate is 115200, try different terminal app |
| MQTT tests fail | Verify broker IP in `config/wifi_secrets.h`, check WiFi SSID/password |

---

## Success Criteria

**You're good to proceed if:**
- [ ] All unit tests show [TEST] PASS
- [ ] Console output shows no ERROR messages
- [ ] Each subsystem responds to basic commands
- [ ] Test output file saved in `testing/subsystemX/logs/`

**Next steps:** See `TESTING_AND_INTEGRATION_PLAN.md` for detailed integration and system tests.

---

**Time to complete:** 15 minutes  
**You'll know if it works:** You see [TEST] PASS messages and no crashes

Good luck! 🚀
