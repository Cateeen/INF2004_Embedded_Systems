# Testing

All test records for the project live in this folder. A test only counts as
done when it is **written down here with its result and a saved log**.

```
testing/
├── README.md              this guide
├── subsystem1/            one folder per subsystem, kept by its owner
│   ├── TEST_LOG.md        test cases + results
│   └── logs/              saved console output, photos, scope captures
├── ...
├── subsystem5/
├── integration/           tests across two or more subsystems (shared)
└── system/                tests of the whole product against its requirements (shared)
```

## Three levels of test

| Level | ID format | Tests what | Written by | Where |
|---|---|---|---|---|
| Unit | `S1-U01`, `S2-U01`, … | one subsystem on its own | the subsystem owner | `subsystemN/TEST_LOG.md` |
| Integration | `INT-01`, … | an interface between subsystems (TEAM.md section 5) | the owners on both sides | `integration/TEST_LOG.md` |
| System | `SYS-01`, … | the finished product against a requirement | whole team | `system/TEST_LOG.md` |

Test IDs never change and are never reused, even if a test is dropped.

## How to run and record a test

The RP2040 I2C register-mock tests run on a host, not on the robot. From
`testing/integration/i2c_host` in Ubuntu/WSL:

```sh
gcc -std=c11 -Wall -Wextra -Werror -Wno-old-style-declaration -I. test.c -o /tmp/i2c_host_test
/tmp/i2c_host_test
```

They do not verify electrical timing or actual I2C hardware.

The LSM303DLHC transport tests also run against a host I2C/API mock, not the
robot. From `testing/integration/imu_i2c_host`:

```sh
gcc -std=c11 -Wall -Wextra -Werror -Isys -I../../../app_program/subsystem4/imu \
  -I../../../device/include \
  test.c ../../../app_program/subsystem4/imu/lsm303dlhc_i2c.c \
  ../../../app_program/subsystem4/imu/lsm303dlhc_reader.c \
  ../../../app_program/subsystem4/imu/lsm303dlhc_accel.c -o imu_i2c_host_test
./imu_i2c_host_test
```

These tests verify software configuration and mocked transfer handling only;
they do not verify the sensor, address selection, bus timing, or timestamp
accuracy on the robot.

The HSR04 distance conversion test is hardware-independent. From
`testing/integration/hcsr04_host`:

```sh
gcc -std=c11 -Wall -Wextra -Werror -I../../../app_program/subsystem5/obstacle \
  test.c ../../../app_program/subsystem5/obstacle/hcsr04.c \
  ../../../app_program/subsystem5/obstacle/hcsr04_capture.c -o hcsr04_host_test
./hcsr04_host_test
```

It checks only Echo pulse-width conversion and input validation, not GPIO,
level shifting, sensor acquisition, or servo operation.

The servo diagnostic's PWM sequencing can be checked without hardware. From
`testing/integration/servo_host`:

```sh
gcc -std=c11 -Wall -Wextra -Werror -I../../../include -DSERVO_TEST_US=0 test.c -o servo_host_test
./servo_host_test
gcc -std=c11 -Wall -Wextra -Werror -I../../../include -DSERVO_TEST_US=1500 test.c -o servo_host_test
./servo_host_test
gcc -std=c11 -Wall -Wextra -Werror -I../../../include -I../../../kernel/knlinc pwm_startup_test.c -o pwm_startup_test
./pwm_startup_test
gcc -std=c11 -Wall -Wextra -Werror calibration_test.c ../../../app_program/subsystem5/obstacle/obs_servo_calibration.c -o calibration_test
./calibration_test
```

These host tests do not verify electrical signals, servo movement, clearance,
or physical scan angles.

1. **Define it first.** Add a row to the *Test cases* table in the right
   `TEST_LOG.md`: ID, what it checks, how, and the expected result.
2. **Automate it if you can.** Add a `TEST_CHECK` with the same ID to
   the relevant module's test source (see `app_program/test_report.h`).
   A test that needs eyes or instruments (an LED blinks, a motor turns, a
   scope shows 1 kHz) is a *manual* test. Write its steps in the table instead.
3. **Build the test image and flash it.**
   ```sh
   cd build_make
   make TEST=1 -j8        # vehicle software startup tests
   ```
   The test image runs vehicle tests at power-up and prints
   `[TEST] <ID> PASS|FAIL` line per check. (A normal `make` leaves the tests out.)
4. **Save the console output** to `logs/`, named `YYYY-MM-DD_<what>.txt`:
   ```sh
   # macOS (find the port name with: ls /dev/cu.usbmodem*)
   cat /dev/cu.usbmodem1101 | tee ../testing/subsystem1/logs/2026-10-14_unit.txt
   # Linux / WSL with usbipd
   cat /dev/ttyACM0 | tee ../testing/subsystem1/logs/2026-10-14_unit.txt
   ```
   Start the command first, then reset the Pico. Press Ctrl-C when it is done.
   On Windows with PuTTY, use *Session → Logging → All session output*.
   For manual tests, save photos or scope screenshots in `logs/` the same way
   (keep each file under a few MB). Videos go in `videos/` instead (see below).
5. **Record the result.** Add a row to the *Results* table: date, test ID,
   the commit you tested (`git rev-parse --short HEAD`), who ran it,
   PASS/FAIL, and the log file. Commit the log and the table together.

## Videos of tests and demos

A manual test (an LED, a motor, a sensor reacting) is best shown with a short
video. Record 15–60 s showing the test ID and the hardware, share it as an
**unlisted YouTube** link (the only way videos are shared), list it in
**`videos/README.md`**, and put the link in the *Log / evidence* column of
the test's result row. `videos/README.md` explains how.

## Rules

- **Never edit or delete an old result.** If a test fails, record the FAIL,
  fix the code, re-run, and add a new PASS row. The history is the evidence.
- **Always record the commit.** A result is meaningless without the exact
  code it was run on. Commit your work *before* testing.
- **A FAIL needs a note** saying what went wrong and, later, which commit fixed it.
- Before merging a change into `main`, re-run your subsystem's unit tests.
  Before a demo or hand-in, run every test and record the full set.
