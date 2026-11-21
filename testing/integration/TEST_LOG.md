# Integration — test log

Tests that two or more subsystems work together. There is one test (or more) for every interface listed in TEAM.md section 5. Both owners agree on the test and sign off the result.

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

INT-01 and INT-02 describe the removed example system and are retained for
historical reference; they cannot be run against the current vehicle firmware.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| INT-01 | samples flow from S2 to S3 (≥ 5 in 2 s) | IF-01, IF-02 | auto | `subsystem3_test.c` | `[TEST] INT-01 PASS` |
| INT-02 | summary lines flow from S3 to S4 (≥ 2 in 3.5 s) | IF-03 | auto | `subsystem4_test.c` | `[TEST] INT-02 PASS` |
| INT-03 | vehicle message buffers copy messages in order and hold their configured capacity | vehicle message buffers | auto | `vehicle_buffers_test()` before tasks start | `[TEST] INT-03 PASS` |
| ODO-U01 | quadrature decoder counts forward, reverse and invalid transitions | encoder decoding | auto | `odometry_test()` before tasks start | `[TEST] ODO-U01 PASS` |
| ODO-U02 | wheel distance and speed use the measured 2500 counts per revolution | odometry calculations | auto | `odometry_test()` before tasks start | `[TEST] ODO-U02 PASS` |
| ODO-U03 | invalid wheel calibration and elapsed time are rejected | odometry calculations | auto | `odometry_test()` before tasks start | `[TEST] ODO-U03 PASS` |
| ODO-U04 | per-wheel polarity and averaged distance | odometry producer | auto | `odometry_producer_test()` before tasks start | `[TEST] ODO-U04 PASS` |
| ODO-U05 | encoder and clock wrap, ambiguous counts and invalid input | odometry producer | auto | `odometry_producer_test()` before tasks start | `[TEST] ODO-U05 PASS` |
| ODO-U06 | simultaneous wheel events produce independent encoder counts | encoder snapshot | auto | `odometry_test()` before tasks start | `[TEST] ODO-U06 PASS` |
| ODO-U07 | ambiguous, inconsistent and missed transitions latch invalid | encoder snapshot | auto | `odometry_test()` before tasks start | `[TEST] ODO-U07 PASS` |
| ODO-U08 | simulated snapshot becomes a message in each odometry buffer | odometry task | auto | `odometry_task_test()` before tasks start | `[TEST] ODO-U08 PASS` |
| ODO-U09 | invalid snapshots, timestamps and full queues cause task faults | odometry task | auto | `odometry_task_test()` before tasks start | `[TEST] ODO-U09 PASS` |
| ODO-U10 | a full opted-in motion odometry queue causes a delivery fault | odometry task | auto | `odometry_task_test()` before tasks start | `[TEST] ODO-U10 PASS` |
| INT-04 | independent odometry message delivery and full-buffer reporting | odometry buffers | auto | `odometry_producer_test()` before tasks start | `[TEST] INT-04 PASS` |
| INT-05 | optional copied motion odometry and independent full-buffer handling | odometry buffers | auto | `odometry_producer_test()` before tasks start | `[TEST] INT-05 PASS` |
| ODO-H01 | live encoder counts and signs for each wheel | GPIO encoder bring-up | manual | Build `TEST=1 ODOMETRY_TEST=1` or `SENSOR_TEST=encoder`; with motor power off, rotate each wheel slowly by hand and compare count deltas over several marked revolutions in both directions. | Each wheel changes independently and reproducibly; about 2500 absolute counts per revolution, opposite sign when reversed. |
| LINE-U01 | line classification from the center and right sensors | line observation | auto | `line_observation_test()` before tasks start | `[TEST] LINE-U01 PASS` |
| LINE-U02 | barcode input and missing readings do not change line classification | line observation | auto | `line_observation_test()` before tasks start | `[TEST] LINE-U02 PASS` |
| LINE-H01 | live line observations from the center and right sensors | ADC, odometry and line/barcode task | manual | Build `SENSOR_TEST=line_barcode`; keep motor power disconnected and roll the robot across blank track, center-only, right-only and both-sensor line/junction positions. | Console reports `LOST`, `LEFT`, `RIGHT` and `JUNCTION` for the corresponding sensor combinations; no task fault is reported. |
| LINE-H02 | live line detection independently of barcode and odometry | line-only bench image | manual | Build `SENSOR_TEST=line`; keep motor power disconnected and wheels still. Present white to both sensors, then black to center only, right only, and both. | `LOST`, `LEFT`, `RIGHT` and `JUNCTION` appear in order without encoder movement. |
| BAR-U01 | forward Code 39 frame and undersized result buffer | barcode decoder | auto | `barcode_scanner_test()` before tasks start | `[TEST] BAR-U01 PASS` |
| BAR-U02 | reverse Code 39 frame preserves payload order | barcode decoder | auto | `barcode_scanner_test()` before tasks start | `[TEST] BAR-U02 PASS` |
| BAR-U03 | invalid configuration, glitch, gap, direction and width separation | barcode decoder | auto | `barcode_scanner_test()` before tasks start | `[TEST] BAR-U03 PASS` |
| BAR-H01 | live Code 39 decoding from the ADC sensor and wheel odometry | line/barcode task | manual | Build `SENSOR_TEST=line_barcode`; keep motor power disconnected and roll the robot steadily over a known Code 39 label in both directions. | Console prints the exact expected payload in both directions; no line/barcode or odometry fault is reported. |
| BAR-H02 | independent Code 39 decoding and module calibration | barcode-only bench image | manual | Build `SENSOR_TEST=barcode`; with motor power disconnected, roll slowly over Code 39 data `AB` from white space, forward and reverse. Rebuild with the reported `BARCODE_TEST_MODULE_COUNTS` and repeat. | `decoded=AB` in both directions before and after calibration, with a positive module-count recommendation and no line reports. |
| LB-U01 | simulated line observations and a decoded barcode through message buffers | line/barcode task | auto | `line_barcode_task_test()` before tasks start | `[TEST] LB-U01 PASS` |
| LB-U02 | wheel-count, sequence and timestamp wrap with odd deltas | line/barcode task | auto | `line_barcode_task_test()` before tasks start | `[TEST] LB-U02 PASS` |
| LB-U03 | future or stale odometry, unsynchronized edges and full queues | line/barcode task | auto | `line_barcode_task_test()` before tasks start | `[TEST] LB-U03 PASS` |
| ADC-U01 | invalid or duplicate channels and acquisition limits rejected | line ADC reader | auto | `line_adc_reader_test()` before tasks start | `[TEST] ADC-U01 PASS` |
| ADC-U02 | independent thresholds and black polarity | line ADC reader | auto | `line_adc_reader_test()` before tasks start | `[TEST] ADC-U02 PASS` |
| ADC-U03 | original ADC channel mapping, threshold and black polarity imported into the task reader | line ADC reader migration | auto | `line_adc_reader_test()` before tasks start | `[TEST] ADC-U03 PASS` |
| IMU-U01 | flat, ramp and hump state transitions | IMU processing | auto | `imu_processor_test()` before tasks start | `[TEST] IMU-U01 PASS` |
| IMU-U02 | tilt and invalid input handling | IMU processing | auto | `imu_processor_test()` before tasks start | `[TEST] IMU-U02 PASS` |
| IMU-U03 | independent timeout state and millisecond wrap | IMU processing | auto | `imu_processor_test()` before tasks start | `[TEST] IMU-U03 PASS` |
| IMU-U04 | typed estimates, latched safety flags, invalid and stale inputs, sequence and clock wrap | IMU producer | auto | `imu_processor_test()` before tasks start | `[TEST] IMU-U04 PASS` |
| IMU-U05 | queued and future odometry, sequence and clock wrap, copied estimates | IMU task | auto | `imu_task_test()` before tasks start | `[TEST] IMU-U05 PASS` |
| IMU-U06 | missing, stale or invalid odometry, sensor and timestamp errors, and full output buffer | IMU task | auto | `imu_task_test()` before tasks start | `[TEST] IMU-U06 PASS` |
| IMU-U07 | original LSM303DLHC normal-mode signed data, axis mapping, sensitivity and invalid input | IMU decoding | auto | `imu_processor_test()` before tasks start | `[TEST] IMU-U07 PASS` |
| IMU-U08 | fresh transfer adapter, mapping, partial and invalid transfers, error latch | IMU reader adapter | auto | `imu_processor_test()` before tasks start | `[TEST] IMU-U08 PASS` |
| I2C-U01 | RP2040 command ordering, repeated start, STOP completion and wake-up lock ordering | kernel I2C driver | host mock | compile and run `testing/integration/i2c_host/test.c` | `[TEST] I2C-U01 PASS` |
| I2C-U02 | RP2040 I2C timeout and abort propagation | kernel I2C driver | host mock | compile and run `testing/integration/i2c_host/test.c` | `[TEST] I2C-U02 PASS` |
| I2C-U03 | IMU-only I2C driver interrupt timing and trace argument validation | kernel I2C diagnostic | host mock | compile `testing/integration/i2c_host/test.c` with `-DAPP_SENSOR_TEST=2` and run | `[TEST] I2C-U03 PASS` |
| I2C-U04 | premature completion signals during send, receive and STOP; stale masked interrupt | kernel I2C driver | host mock | compile and run `testing/integration/i2c_host/test.c` in both modes | `[TEST] I2C-U04 PASS` |
| IMU-I2C-U01 | LSM303DLHC control-register setup/readback, fresh six-byte sample and timestamp | IMU I2C transport | host mock | compile and run `testing/integration/imu_i2c_host/test.c` | `[TEST] IMU-I2C-U01 PASS` |
| IMU-I2C-U02 | no identity read, short transfer, data-ready timeout, output preservation and fault latch | IMU I2C transport | host mock | compile and run `testing/integration/imu_i2c_host/test.c` | `[TEST] IMU-I2C-U02 PASS` |
| IMU-I2C-U03 | IMU task startup preserves timing configuration and receives the transport callback and context | IMU task integration | host mock | compile and run `testing/integration/imu_i2c_host/test.c` | `[TEST] IMU-I2C-U03 PASS` |
| IMU-I2C-U04 | imported address, original register mode, vehicle axis signs and accelerometer scale | IMU hardware configuration migration | host mock | compile and run `testing/integration/imu_i2c_host/test.c` | `[TEST] IMU-I2C-U04 PASS` |
| IMU-I2C-U05 | standalone diagnostic control readback, direct reads, errors and microsecond timing | IMU hardware diagnostic | host mock | compile and run `testing/integration/imu_i2c_host/test.c` | `[TEST] IMU-I2C-U05 PASS` |
| IMU-I2C-U06 | standalone data-ready transition, inconclusive status, timeout and transfer error | IMU hardware diagnostic | host mock | compile and run `testing/integration/imu_i2c_host/test.c` | `[TEST] IMU-I2C-U06 PASS` |
| IMU-I2C-U07 | standalone BDU experiment: held, changed and inconclusive byte patterns; transfer failure | IMU hardware diagnostic | host mock | compile and run `testing/integration/imu_i2c_host/test.c` | `[TEST] IMU-I2C-U07 PASS` |
| IMU-I2C-U08 | experimental 0x32-family profile, readback failures, data-ready timeout and fault latch | IMU I2C transport | host mock | compile and run `testing/integration/imu_i2c_host/test.c` | `[TEST] IMU-I2C-U08 PASS` |
| IMU-I2C-U09 | standalone trace-clock comparison reports kernel and raw timer deltas | IMU hardware diagnostic | host mock | compile and run `testing/integration/imu_i2c_host/test.c` | `[TEST] IMU-I2C-U09 PASS` |
| OBS-U01 | clear scan and centred obstacle | obstacle profiling | auto | `obs_profile_test()` before tasks start | `[TEST] OBS-U01 PASS` |
| OBS-U02 | scan edge clearances and cluster selection | obstacle profiling | auto | `obs_profile_test()` before tasks start | `[TEST] OBS-U02 PASS` |
| OBS-U03 | invalid scan input rejected | obstacle profiling | auto | `obs_profile_test()` before tasks start | `[TEST] OBS-U03 PASS` |
| OBS-U04 | ahead threshold, scan results, closest range, invalid input and timestamp wrap | obstacle producer | auto | `obs_profile_test()` before tasks start | `[TEST] OBS-U04 PASS` |
| OBS-U05 | ahead observations, requested scan, cancellation, reacquisition and clock wrap | obstacle task | auto | `obs_task_test()` before tasks start | `[TEST] OBS-U05 PASS` |
| OBS-U06 | stale or invalid sensor, premature or stale commands, sequence gaps, scan timeout and full event buffer | obstacle task | auto | `obs_task_test()` before tasks start | `[TEST] OBS-U06 PASS` |
| OBS-U07 | HSR04 Echo pulse width to millimetres, invalid sound speed and reserved sentinel range | ultrasonic distance conversion | host test | compile and run `testing/integration/hcsr04_host/test.c` | `[TEST] OBS-U07 PASS` |
| OBS-U08 | HSR04 rising/falling Echo capture, wraparound, timeout, reset and restart | ultrasonic Echo capture | host test | compile and run `testing/integration/hcsr04_host/test.c` | `[TEST] OBS-U08 PASS` |
| SERVO-U01 | default servo diagnostic disables PWM; a selected pulse is enabled only after the warning delay | servo diagnostic | host test | run both configurations described in `testing/README.md` | both mock runs pass |
| SERVO-U02 | RP2040 startup releases PWM reset when the physical timer is disabled | PWM startup | host test | `pwm_startup_test` as described in `testing/README.md` | PWM reset released |
| SERVO-U03 | bounded scan mapping, direction and invalid calibration | servo calibration | host test | `calibration_test` as described in `testing/README.md` | bounded mapping passes |
| MOT-U01 | independent wheel correction and limits | wheel speed control | auto | `wheel_speed_test()` before tasks start | `[TEST] MOT-U01 PASS` |
| MOT-U02 | integral anti-windup, derivative and reset | wheel speed control | auto | `wheel_speed_test()` before tasks start | `[TEST] MOT-U02 PASS` |
| MOT-U03 | invalid inputs and numeric overflow rejected | wheel speed control | auto | `wheel_speed_test()` before tasks start | `[TEST] MOT-U03 PASS` |
| MOT-U04 | signed wheel speeds, stale or missing odometry, counter and timestamp wrap | motion feedback | auto | `wheel_speed_test()` before tasks start | `[TEST] MOT-U04 PASS` |
| MOT-U05 | baseline, forward and reverse wheel control, retained output and clock wrap | motion control | auto | `wheel_speed_test()` before tasks start | `[TEST] MOT-U05 PASS` |
| MOT-U06 | wrong direction, stale or pre-start feedback, sequence gaps and invalid timing latch a zero-output fault | motion control | auto | `wheel_speed_test()` before tasks start | `[TEST] MOT-U06 PASS` |
| MOT-U07 | simulated movement, pending feedback, timestamp-based expiry and stop precedence | motion task simulation | auto | `motion_task_test()` before tasks start | `[TEST] MOT-U07 PASS` |
| MOT-U08 | invalid or expired commands, emergency stop and odometry sequence gaps | motion task simulation | auto | `motion_task_test()` before tasks start | `[TEST] MOT-U08 PASS` |
| MOT-U09 | exact command expiry, stale commands and clock/sequence wrap | motion task simulation | auto | `motion_task_test()` before tasks start | `[TEST] MOT-U09 PASS` |
| MOT-U10 | simulation does not publish motion status | motion task simulation | auto | `motion_task_test()` before tasks start | `[TEST] MOT-U10 PASS` |
| MOT-U11 | forward and reverse actuator commands preserve the imported left/right motor polarity | motion task actuator mapping | auto | `motion_task_test()` before tasks start | `[TEST] MOT-U11 PASS` |
| MOT-U12 | hardware status shape reports commanded PWM and latched faults | motion task status contract | auto | `motion_task_test()` before tasks start | `[TEST] MOT-U12 PASS` |
| VC-U01 | go, emergency stop and fault motion vetoes | controller gate | auto | `car_gate_test()` before tasks start | `[TEST] VC-U01 PASS` |
| VC-U02 | obstacles veto line following | controller gate | auto | `car_gate_test()` before tasks start | `[TEST] VC-U02 PASS` |
| VC-U03 | line-following rearm requires a go edge with safety conditions clear | controller gate | auto | `car_gate_test()` before tasks start | `[TEST] VC-U03 PASS` |
| VC-U04 | line, obstacle and IMU safety vetoes require a fresh go edge | controller safety | auto | `car_gate_test()` before tasks start | `[TEST] VC-U04 PASS` |
| VC-U05 | scanning blocks motion until a fresh ahead observation and new go edge | controller safety | auto | `car_gate_test()` before tasks start | `[TEST] VC-U05 PASS` |
| VC-U06 | sequence gaps, stale data, scan deadlines and emergencies latch safety faults | controller safety | auto | `car_gate_test()` before tasks start | `[TEST] VC-U06 PASS` |
| VC-U07 | timestamp and sequence wrap, with backward-clock rejection | controller safety | auto | `car_gate_test()` before tasks start | `[TEST] VC-U07 PASS` |
| VC-U08 | copied line, IMU and obstacle messages keep the controller waiting without arming | controller monitor | auto | `controller_monitor_test()` before tasks start | `[TEST] VC-U08 PASS` |
| VC-U09 | malformed, missing or out-of-sequence messages and subsystem faults enter the faulted state | controller monitor | auto | `controller_monitor_test()` before tasks start | `[TEST] VC-U09 PASS` |
| VC-U10 | state transitions and new go edge after startup or sensor veto | controller state simulation | auto | `car_gate_test()` before tasks start | `[TEST] VC-U10 PASS` |
| VC-U11 | stale data, emergency latch and clock wrap | controller state simulation | auto | `car_gate_test()` before tasks start | `[TEST] VC-U11 PASS` |
| VC-U12 | scan blocks following until post-scan ahead data and new go edge | controller state simulation | auto | `car_gate_test()` before tasks start | `[TEST] VC-U12 PASS` |
| VC-U13 | fresh sensor safety and trusted local GO gate ARM/STOP commands | vehicle controller task | auto | `vehicle_controller_task_test()` before tasks start | `[TEST] VC-U13 PASS` |
| VC-U14 | local emergency input sends a latched emergency stop | vehicle controller task | auto | `vehicle_controller_task_test()` before tasks start | `[TEST] VC-U14 PASS` |
| VC-U15 | missing sensor deadlines latch controller fault and request emergency stop | vehicle controller task | auto | `vehicle_controller_task_test()` before tasks start | `[TEST] VC-U15 PASS` |
| VC-U16 | nonzero commanded output status blocks arming and requests emergency stop | vehicle controller task | auto | `vehicle_controller_task_test()` before tasks start | `[TEST] VC-U16 PASS` |
| VC-U17 | stale motion status faults despite fresh sensor observations | vehicle controller task | auto | `vehicle_controller_task_test()` before tasks start | `[TEST] VC-U17 PASS` |
| VC-U18 | junction, left and right positions preserve the source PWM decisions | line steering migration | auto | `line_steering_test()` before tasks start | `[TEST] VC-U18 PASS` |
| VC-U19 | lost-line hold expires at 300 ms with a zero-output latch | line steering migration | auto | `line_steering_test()` before tasks start | `[TEST] VC-U19 PASS` |
| VC-U20 | stop latch persists until explicit reset | line steering migration | auto | `line_steering_test()` before tasks start | `[TEST] VC-U20 PASS` |
| VC-U21 | invalid line positions fail closed until reset | line steering migration | auto | `line_steering_test()` before tasks start | `[TEST] VC-U21 PASS` |
| GO-U01 | a button held at startup cannot enable GO | local GO button | auto | `go_button_test()` before tasks start | `[TEST] GO-U01 PASS` |
| GO-U02 | a single noisy sample does not toggle GO | local GO button | auto | `go_button_test()` before tasks start | `[TEST] GO-U02 PASS` |
| GO-U03 | distinct stable presses toggle GO once each | local GO button | auto | `go_button_test()` before tasks start | `[TEST] GO-U03 PASS` |
| WIFI-U01 | disconnected queue retention, status delivery, transient publish retry and sequence wrap | WiFi service | auto | `wifi_service_test()` before tasks start | `[TEST] WIFI-U01 PASS` |
| WIFI-U02 | invalid or stale telemetry, invalid link, full status buffer and backend errors | WiFi service | auto | `wifi_service_test()` before tasks start | `[TEST] WIFI-U02 PASS` |
| WIFI-U03 | legacy dashboard command decode, malformed lengths and reserved command types | WiFi command protocol | auto | `wifi_service_test()` before tasks start | `[TEST] WIFI-U03 PASS` |
| WIFI-U04 | copied bridge queue, full buffer, disconnected retention, publish retry, clock wrap, stale and invalid packets | IF-07 | auto | `wifi_mqtt_bridge_test()` before tasks start | `[TEST] WIFI-U04 PASS` |
| WIFI-U05 | coherent link snapshots, initialization, sequence wrap and in-progress writer | IF-08 | auto | `wifi_mqtt_bridge_test()` before tasks start | `[TEST] WIFI-U05 PASS` |
| WIFI-U06 | legacy telemetry golden bytes, sequence wrap, invalid fields and packets | IF-07 | auto | `wifi_service_test()` and `wifi_mqtt_bridge_test()` before tasks start | `[TEST] WIFI-U06 PASS` |
| WIFI-U07 | producer freshness, copied queue, backpressure, sequence wrap and service delivery | IF-09 | auto | `wifi_service_test()` before tasks start | `[TEST] WIFI-U07 PASS` |
| GPIO-U01 | simultaneous GPIO events reach the correct registered owners | GPIO interrupt router | auto | `gpio_irq_test()` before tasks start | `[TEST] GPIO-U01 PASS` |
| GPIO-U02 | overlapping pin ownership and invalid masks rejected | GPIO interrupt router | auto | `gpio_irq_test()` before tasks start | `[TEST] GPIO-U02 PASS` |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | INT-01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
