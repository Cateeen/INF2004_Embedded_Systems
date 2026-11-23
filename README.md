# INF2004_Embedded_Systems
Embedded System Project

## Buddy 2 - Motion Control

**Status:** Initial implementation merged into `main`. Hardware integration and testing are pending.

### Overview

The Buddy 2 module provides motor control, encoder feedback, and PID control for the embedded system.

### Implemented Features

- **Motor Driver:** PWM-based motor speed control, forward/reverse commands, braking, and emergency-stop latch.
- **Encoder:** Encoder processing module for motion feedback.
- **PID Controller:** PID control module for closed-loop motion control.
- **Unit Tests:** Tests for the encoder and PID controller.
- **Hardware Test:** Raspberry Pi Pico W application that initialises the motor driver and keeps the motors in the emergency-stopped state.

### Hardware Configuration

The current motor driver uses provisional GPIO assignments:

| Motor | Pins |
|---|---|
| Motor 1 | GP8 and GP9 |
| Motor 2 | GP10 and GP11 |

PWM frequency: 20 kHz.

**Important:** These GPIO assignments have not been verified against the team's actual motor-controller board. Do not flash or operate the motors until the board, wiring, power requirements, and safety behaviour have been checked.

### Outstanding Work

1. Confirm the motor-controller board and GPIO assignments.
2. Verify braking and safe direction changes.
3. Resolve the automatic UF2 conversion crash in `picotool`.
4. Integrate Buddy 2 with the other system modules.
5. Perform physical hardware testing after safety checks.

## µT-Kernel Vehicle Architecture

The migrated vehicle task architecture is enabled with
`PICOCAR_USE_UTKERNEL=ON`. The µT-Kernel port must provide a header at
`<include-dir>/tk/tkernel.h`; pass that parent directory through
`PICOCAR_UTKERNEL_INCLUDE_DIR`.

Example configuration:

```text
cmake -S . -B build -G Ninja ^
  -DPICOCAR_USE_UTKERNEL=ON ^
  -DPICOCAR_ENABLE_MQTT=ON ^
  -DPICOCAR_UTKERNEL_INCLUDE_DIR=C:\path\to\utkernel\include ^
  -DPICOCAR_ODOMETRY_COUNTS_PER_REV=<verified-count>
```

The calibrated `PICOCAR_ODOMETRY_COUNTS_PER_REV` default is `2500`. Set it to
`0` to disable odometry, or override it if the encoder/wheel setup changes.
The default wheel circumference is 20 cm and can be overridden with
`PICOCAR_ODOMETRY_WHEEL_CIRCUMFERENCE_CM`.

Build and flash the `encoder_calibration_test` target to measure the encoder
counts without enabling the motors. Rotate each wheel exactly one mechanical
revolution and record the stable left/right counts. Repeat several times and
use the average absolute count as `PICOCAR_ODOMETRY_COUNTS_PER_REV`. Compare
the left and right readings; if they differ substantially, calibrate each side
separately before relying on odometry.

Only the WiFi Service Task may call the MQTT/lwIP adapter. The Vehicle
Controller communicates with it through the typed message buffers and must not
include network headers.

## Buddy 1 + Buddy 4 - Pico W MQTT/IMU Test

The `buddy4_imu_test` firmware also runs Buddy 1's telemetry publisher. It
samples the IMU at 100 Hz, polls the Wi-Fi/MQTT state machine, publishes the
Buddy 1 five-byte binary telemetry packet once per second, and subscribes to
`robot/command`. The test maps hump activity to `TERRAIN_HUMP`; shock, tilt,
and IMU fault states map to `VEHICLE_FAULT` and `FAULT_IMU`. Line detection,
obstacle detection, and battery percentage are not available in this test and
are reported as unknown/zero.

The packet format does not carry a hump measurement or separate shock/tilt
fields. Incoming commands are decoded and printed over USB serial, but are not
connected to motor control. This prevents a network test from moving the car
without an agreed command-to-motion and safety policy.

### Configure Wi-Fi and broker

1. Copy `include/wifi_mqtt_config.example.h` to
   `include/wifi_mqtt_config.h`.
2. Set the 2.4 GHz Wi-Fi SSID/password and the broker host/IP and port. Broker
   hostnames are resolved through DNS. An empty MQTT username/password means
   anonymous broker access; otherwise set both values.
3. Keep the local `wifi_mqtt_config.h` private. It is git-ignored.
4. The current adapter uses plain MQTT (normally port 1883), without TLS. Use
   it only on a trusted network; TLS is not implemented in this adapter.

If the local configuration is absent or incomplete, the firmware still builds
but reports that Wi-Fi/MQTT did not initialize. `buddy1_poll()` must continue
to run regularly; the Wi-Fi and MQTT connection process is asynchronous.

### Build and bench test

From the repository root, configure and build the Pico W firmware:

```powershell
cmake -S . -B build
cmake --build build --target buddy4_imu_test
```

Flash `build/buddy4_imu_test.uf2` only after checking the Pico W, IMU wiring,
power, and motor-controller safety. For a bench test, keep the drive wheels
off the ground or disconnect motor power.

On a computer that can reach the same broker, subscribe to telemetry and
heartbeat:

```text
mosquitto_sub -h <broker-host> -p <port> -t robot/telemetry -F "%x"
mosquitto_sub -h <broker-host> -p <port> -t robot/heartbeat -F "%x"
```

Telemetry is binary, so inspect it as hex rather than expecting JSON text. A
two-byte command can be sent to verify the receive queue and USB log. For
example, the following PowerShell creates a `STOP` command with sequence 1;
it is only logged by this test firmware and does not drive the motors:

```powershell
[IO.File]::WriteAllBytes("stop.bin", [byte[]](0x80, 0x01))
mosquitto_pub -h <broker-host> -p <port> -t robot/command -f stop.bin
```

When using broker authentication, configure the MQTT username/password in the
local header and pass equivalent options to the Mosquitto clients. The
`buddy4_imu_test` target build checks compilation only; a live broker and
physical Pico W are required to verify wireless operation.

### IMU-only hardware diagnostic

Use the independent `buddy4_imu_only_test` target to check the LSM303DLHC
accelerometer without initializing the Pico W Wi-Fi stack or MQTT:

```powershell
ninja -C build buddy4_imu_only_test
```

Flash `build/buddy4_imu_only_test.uf2`, open the USB serial monitor, and press
RESET. The test checks the accelerometer's `WHO_AM_I` register (expected
`0x33`) and prints raw and scaled XYZ samples every 200 ms. If no device
responds, it reports the configured I2C pins/address and retries once per
second. This isolates the accelerometer test from Buddy 1 networking.
