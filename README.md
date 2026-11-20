# micro T-Kernel 3.0 project for Raspberry Pi Pico W

A starting point for your project: micro T-Kernel 3.0 running on **both cores**
of the Pico W's RP2040, with a USB serial console and the WiFi driver built in.

You need a **Raspberry Pi Pico W**, a USB cable, and a computer running
macOS, Linux, or Windows with **WSL (Ubuntu)**. On Windows, run every
command below inside the WSL Ubuntu terminal.

---

## 1. Set up the Pico C SDK and the compiler

You need two separate things:

| What | Why | Where it comes from |
|---|---|---|
| **Pico C SDK** | USB and WiFi driver source code | `git clone` (step 1.2) |
| **Arm GNU Toolchain** (`arm-none-eabi-gcc`) | the compiler | a separate download (step 1.3) |

> ⚠️ **The SDK does NOT include the compiler.** You must install the compiler
> on its own and put its `bin` folder on your `PATH` (step 1.4).

**1.1 Install the basic tools: git, make and a host C++ compiler**

```sh
# macOS
xcode-select --install

# Linux / WSL (Ubuntu)
sudo apt update && sudo apt install -y git make g++
```

**1.2 Download the Pico C SDK** (any folder works; this guide uses `~/pico`)

```sh
mkdir -p ~/pico && cd ~/pico
git clone --branch 2.2.0 https://github.com/raspberrypi/pico-sdk.git
cd pico-sdk
git submodule update --init lib/tinyusb lib/cyw43-driver
```

The `submodule` line is required: it downloads the USB and WiFi drivers.
(SDK 2.3.0 has also been tested and works.)

**1.3 Install the Arm GNU Toolchain (the compiler)**

- **macOS / Windows (WSL) / Linux:** download from
  <https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads>.
  Pick the **AArch32 bare-metal target (arm-none-eabi)** package for your
  computer. For WSL, choose the **Linux x86_64** `.tar.xz`.
  Extract it, e.g. to `~/tools/arm-gnu-toolchain-<version>/`.
- **Linux / WSL shortcut:** `sudo apt install -y gcc-arm-none-eabi`
  installs it straight onto your PATH. If you use this, skip the PATH line in 1.4.

Find the folder that contains `arm-none-eabi-gcc`. It is the toolchain's
**`bin`** folder, e.g. `~/tools/arm-gnu-toolchain-15.3.rel1/bin`.

**1.4 Tell your shell where the compiler and SDK are**

Add these two lines to the end of `~/.zshrc` (macOS) or `~/.bashrc` (Linux/WSL),
**changing both paths to match your computer**:

```sh
export PATH="$HOME/tools/arm-gnu-toolchain-15.3.rel1/bin:$PATH"   # folder that contains arm-none-eabi-gcc
export PICO_SDK_PATH="$HOME/pico/pico-sdk"                         # the SDK from step 1.2
```

Open a **new** terminal, then check both:

```sh
arm-none-eabi-gcc --version     # must print a version, not "command not found"
ls $PICO_SDK_PATH               # must list the SDK files
```

If `arm-none-eabi-gcc` is "not found", your `PATH` does not point to the
toolchain's `bin` folder. Fix that before going further.

---

## 2. Clone this repo and build the `.uf2` file

**2.1 Clone your project**

```sh
cd ~/pico
git clone <URL-of-this-repo> my-project
cd my-project
```

**2.2 Build**

```sh
cd build_make
make -j8
```

A successful build ends with a line like
`tools/elf2uf2 mtk3pico_smp1_usb_cdc_wifi.elf mtk3pico_smp1_usb_cdc_wifi.uf2`.
Your firmware is **`build_make/mtk3pico_smp1_usb_cdc_wifi.uf2`**.

If `make` stops with an error, it tells you which one is wrong:
`arm-none-eabi-gcc not found` means fix `PATH`, and
`PICO_SDK_PATH is not set` means fix `PICO_SDK_PATH` (step 1.4).

To build a single-core, UART-only or test version, see section 3.

**2.3 Flash it onto the Pico W**

1. Hold the **BOOTSEL** button and plug the Pico W into USB, then release.
   A drive called **RPI-RP2** appears.
2. Copy the `.uf2` file onto that drive. On WSL, run `explorer.exe .` in
   `build_make` to open the folder in Windows. The Pico reboots by itself.

**2.4 See the output**

Open the Pico's USB serial port at 115200 baud:

```sh
screen /dev/tty.usbmodem* 115200      # macOS   (quit: Ctrl-A then K)
screen /dev/ttyACM0 115200            # Linux
```

On Windows, use PuTTY or the VS Code *Serial Monitor* on the new COM port.
Vehicle tasks are opt-in and require verified hardware configuration.

**2.5 Write your own code**

Your code lives in **`app_program/`**. Every `.c` file there, and in its
subfolders, is compiled automatically.

- **`app_main.c` → `usermain()` is where the program starts.** It creates
  vehicle message buffers first, then starts the selected vehicle or diagnostic tasks.
- Do not copy the source `integration/car_main.c` as another entry point.
  Its default path is a bare-metal `main()` that launches core-specific loops;
  µT-Kernel already owns startup and calls `usermain()`. The source file's
  optional `PICOCAR_USE_UTKERNEL` branch delegates to the source
  `vehicle_system_run()`, not this target's startup configuration. Map its
  responsibilities as follows:
  - `vc_core()` / `vc_task()` → the vehicle-controller and motion tasks;
    kernel tasks and message buffers replace the hand-launched core loop.
  - `obs_core()` / `obs_step()` → the obstacle task, which owns its sensor and
    scan operations.
  - `line_status_init()` / `car_measurements_init()` → line/barcode and
    odometry producers, which publish copied messages instead of shared state.
  - Source `line_steering.c` is preserved under `app_program/subsystem2/controller/` as a
    hardware-independent decision module. It retains its PWM intent values
    but is not wired to the motion task or started by application startup.
  - `car_link_init()` → still needs a target-local input provider. Do not call
    blocking `tm_getchar()` from the controller callback; the callback must
    return within its task period.
  Vehicle tasks remain opt-in until local-input binding, motion authorization,
  and hardware configurations are verified.
- **Subsystem folders:** `app_program/subsystem1/` … `subsystem6/` contain
  WiFi, control and motion, line/barcode, IMU, obstacle, and odometry/GPIO
  modules respectively. Their task headers provide their public interfaces.
- `app.h` holds the definitions shared by the whole team.
- **Drivers go in your subsystem folder too**, as a `drv_<name>.c` /
  `drv_<name>.h` pair, e.g. `subsystem2/drv_ultrasonic.c`. They are compiled
  automatically. Do not add drivers under `device/`: that folder holds the
  kernel's own serial, I2C and ADC drivers, and adding to it means editing
  kernel and makefile files shared by the whole team.

**Working as a team:** record owners, pins, priorities and the interfaces
between subsystems in **`TEAM.md`**, and record every test in **`testing/`**
(see `testing/README.md`). `make TEST=1` tests the vehicle buffers, the
hardware-independent encoder/odometry calculations, line classification, and
Code 39 barcode decoding, IMU processing, obstacle profiling, wheel control
controller safety predicates and GPIO interrupt routing. Short videos
of tests and demos are shared as unlisted YouTube links, listed in **`videos/`**.

The line/barcode task consumes line odometry and publishes line observations
and decoded barcodes. It and the ADC reader are not started by default.
The reader exclusively opens the kernel ADC, configures only the explicitly
selected GP26–GP28 inputs, and requires per-sensor thresholds and polarity.
Its `line_adc_reader_default_config()` imports the original repository's
barcode/center/right ADC channels (2/1/0), 1000-count thresholds, and
black-above polarity; callers still choose the acquisition-span limit.
The supplied pin map assigns left to GP28 (ADC2), center to GP27
(ADC1), and right to GP26 (ADC0); the GPIO numbers were confirmed despite
reversed ADC0/ADC2 labels in the supplied table. The sensor used for barcode
acquisition is not yet confirmed, and no thresholds are assumed.
Verify wiring, pin ownership and 3.3 V-compatible sensor outputs before use;
GP29 is reserved for the radio. A conversion or timing failure latches the
reader faulted and retains ADC ownership until restart. ADC pins remain analog
after a normal close. The barcode timestamp precedes its eight conversions;
the configured maximum span rejects completed reads that took too long, but
does not bound blocking inside the ADC driver or detect missed edges. The
driver can wait indefinitely for ADC readiness or synchronous completion;
independent safety supervision is required. Keep the reader storage alive
until a normal close, or until reboot after a fault, because an interrupted
device request may still access its conversion buffer.
Startup requires an owned sensor-reader callback, barcode settings,
sampling period and maximum sample/odometry age. The reader timestamps
barcode acquisition using the same operating clock as odometry. It stops on stale, missing or
out-of-sequence odometry, unsynchronized barcode edges, read or delivery errors,
and retries fault delivery. Consumers must also fail safe on stale data when a
fault cannot be delivered. The scanner uses the mean of polarity-normalized
wheel count changes, so compatible wheel count scales are required. Tests use
simulated samples; sensor thresholds, encoder direction, sampling/odometry
timing and barcode behavior must be verified on hardware before use.
`line_adc_reader_start_task()` optionally binds the actual ADC reader to the
line/barcode task; ADC open and pin setup occur inside that task. It requires
caller-supplied channel selection, thresholds, polarity and timing limits.
The motor- and servo-free `make SENSOR_TEST=line_barcode` diagnostic starts
this reader with the imported channel map, the encoder/odometry source, and
the production line/barcode task. It prints live line observations and
decoded Code 39 payloads. Its wheel calibration and
`LINE_BARCODE_TEST_COUNTS_PER_MODULE` setting are in
`app_program/hardware_sensor_test.c`; tune them to the measured robot and
printed barcode before scanning. Keep motor power disconnected during this
diagnostic. It tests sensor/odometry decoding and line observations, not
powered motor steering.

The IMU processor consumes simulated or caller-provided acceleration,
distance and timestamps; it does not start a task or access the sensor.
Its thresholds, mounting orientation and odometry inputs need hardware
verification before use in the vehicle. The single-owner IMU producer converts
successful accelerometer samples and fresh, ordered odometry into copied
estimate messages. Only accelerometer-derived roll and pitch are marked
valid; dynamic acceleration can make them inaccurate. Yaw and gyro fields are
not measured. The message includes packed terrain telemetry and a latched tilt
indicator. The shock field is retained but collision detection is disabled, so
it does not report impacts. The caller must report rejected samples, check sample age,
retain future odometry until its timestamp, and deliver messages; no IMU task
or hardware reader is started by default. The inactive IMU task owns its
reader callback, consumes the dedicated odometry buffer, and publishes
estimates. It stops on invalid or stale inputs, clock, scheduling or delivery
errors, and retries fault delivery. It requires explicitly configured sampling
and age limits. Consumers must also fail safe on stale estimates if fault
delivery fails or the reader blocks; real timing and sensor behavior are
unverified.

The accelerometer responds at 0x19 on I2C1 GP2/GP3, but its exact identity is
unconfirmed: the observed WHO_AM_I is 0x32, not the LSM303DLHC's 0x33. The
portable accelerometer decoder expects a verified fresh six-byte read configured for little-endian,
normal-mode +/-2g operation, and an explicit vehicle-axis mapping. It does not
configure the sensor or access I2C. Bus ownership and any other devices are
not yet confirmed. The kernel I2C driver is disabled; if explicitly enabled,
startup initializes only I2C1 on GP2/GP3 as `iicb`. Confirm ownership before
enabling it. I2C0 is still configured on motor pins GP8/GP9 and must not be
initialized until its pins are reassigned.
The RP2040 I2C driver's transaction sequencing has host-level register-mock
tests. The post-unlock wake-up ordering reduced latency in a hardware diagnostic,
but the shared-driver fix and its timeout behavior still need hardware testing.
Timeouts and bus faults must be handled independently; do not treat a software
build as sensor verification.
The inactive LSM303DLHC reader adapter connects a supplied fresh-transfer
callback to the IMU task. The transport must configure the verified mode,
establish that each sample is new, supply all six bytes and a valid acquisition
timestamp, and own the bus; the adapter cannot independently verify those
claims. An opt-in I2C1 transport is now available in
`app_program/subsystem4/imu/lsm303dlhc_i2c.c`: it opens `iicb` exclusively, accepts the
sensor address and register profile as configuration (the imported default is
0x19 and the original DLHC profile), checks control-register readback but does
not verify sensor identity. The optional 0x32-family profile uses CTRL1 `0x2f`
and CTRL4 `0x80`: 100 Hz and BDU on the candidate family, whose exact identity
and data coherence remain unconfirmed. Both profiles assume normal-mode +/-2g.
The transport uses the original 1.05 mg/LSB scale and X/Y/Z signs (-,+,-),
waits for ZYXDA, and reads all six output bytes. It supplies timestamps on the
`tk_get_otm` clock that precede the corresponding acquisition. The ready wait
is capped at 100 ms; kernel I2C transaction timeouts can add up to the driver's
configured timeout for each bus operation. Any transport error latches a
fault. `lsm303dlhc_i2c_start_imu_task()` wires this reader into the IMU task
with static context lifetime; sensor initialization and reads execute in the
IMU task context. Opt-in vehicle startup selects the 0x32-family profile.
I2C remains disabled by default; bus sharing and timestamps have not been
verified on physical hardware. The original source also initialized the
magnetometer, but the current vehicle message/processor path consumes only
accelerometer data.

The obstacle task has an opt-in Pico callback adapter in
`app_program/subsystem5/obstacle/obs_pico_port.c`. It uses the shared GPIO dispatcher for
Echo edges and runs the task on core 0. It is not registered or started by
application startup by default. Its generic default configuration remains
unverified. The reported safe servo positions are 1500 us right, 1700 us ahead and 1900 us
left; the physical scan limits were measured at 20 degrees to either side of
center. The adapter interpolates within these positions and rejects commands
outside the calibrated angle range. The vehicle and ultrasonic test profiles
mark the owner's measured Echo voltage and servo limits as verified.
The nine nominal scan pulses are 1500, 1550, 1600, 1650, 1700, 1750, 1800,
1850 and 1900 us, in right-to-left order. The software profile coordinates are
80–120 degrees with 100 degrees ahead; they represent the measured -20 to +20
degree physical sweep, not absolute compass headings.

The obstacle profile module processes simulated distance scans only. Its
geometry and thresholds are placeholders; a clear scan or computed bypass is
not proof that the path is safe. The obstacle producer emits typed ahead and
scan observations with an explicitly configured emergency distance; emergency
indications are not latched. Scan distance is the minimum measured range,
while target distance belongs to the selected profile. Consumers must check
freshness, latch safety conditions and independently veto motion. Bypass
directions are advisory and unverified. The inactive obstacle task monitors
ahead and starts scans only on controller request. With the reported single
scanning sensor, forward monitoring pauses during scans: the controller must
confirm the vehicle is stopped before requesting a scan and keep it stopped
until a fresh ahead observation arrives after scanning or cancellation.
The task cannot independently stop motors or verify that precondition. Its
optional initialization callback, polling callback and stop callback share the
obstacle task's owner context. The polling and stop callbacks must return
promptly; a blocked callback or failed fault delivery requires independent
freshness supervision. The reported ultrasonic sensor is an HSR04; its
Echo voltage/level shifting must be verified for the actual wiring.
Do not connect a 5 V Echo directly to the Pico W or activate the sensor/servo
until those details are verified. The full callback adapter and obstacle
geometry have not been verified on hardware.
The inactive `hcsr04_distance_mm()` helper converts a measured Echo high-pulse
width to millimetres using an explicitly supplied sound speed; it does not
measure the Echo pulse or initialize GPIO. The companion Echo capture state
machine validates rising/falling edge timestamps, timer wrap and timeout, but
also does not access GPIO; a hardware integration must capture both edges
without missing the short pulse.

Wheel-speed control calculates PWM magnitudes only. The opt-in hardware motion
task can now apply them through `app_program/subsystem2/motion/motor_driver.c`; application
startup does not call it. The driver starts braked with emergency stop latched
and only its initializing task may control it. Hardware mode requires an
explicit, fresh ARM command while stopped, followed by a fresh VELOCITY command;
STOP does not re-arm it. It applies the original left/right motor polarity
mapping, but wiring and direction must be checked with the wheels raised.
Keep I2C0 disabled or move its GP8/GP9 pins before using the motors. Gains,
encoder signs, emergency-stop behavior, independent stopping and real-time
behavior remain unverified; this port has only build-level validation.
The motion feedback adapter calculates signed wheel speeds from consecutive,
fresh odometry messages with explicit per-wheel calibration and timing limits.
The standalone motion control step combines it with the PID for straight
forward or reverse movement. It waits for two fresh samples, rejects unexpected
wheel direction, and latches a zero-output fault on stale or invalid feedback.
It must be checked periodically even without new messages; it cannot stop
hardware or replace an independent watchdog if its caller stalls. Neither
component consumes a buffer or drives motors. The feedback age and interval
limits must not exceed 250 ms.

The motion task has a simulation entry point and a separate opt-in hardware
entry point. Both consume the dedicated command and odometry buffers; only
hardware mode publishes motion status, and neither entry point is started by
default. Straight velocity commands use signed cm/s and expire from their
timestamp; turns and nonzero angular velocities are rejected. Hardware mode
requires the controller to send ARM only after its safety checks; an unarmed
velocity command faults. Ordinary stop takes precedence over movement already
queued, while emergency stop latches a fault until restart. Start the controller
before the hardware motion task so the status buffer has a consumer; status
delivery failure faults and stops the motion task. The controller must handle
faults, supervise freshness independently, and have active consumers for the
other odometry buffers. Its task period must be no more than 50 ms to keep up
with the 100 ms status publication interval. The hardware path has only
build-level validation and has not been flashed or tested on the robot.

The motion-status message reports task-commanded signed PWM, command activity
and latched faults, with sequence numbers and timestamps. It does not report
ARM acceptance, is not motor telemetry, and cannot prove that the vehicle has
physically stopped. The controller requires a fresh status reporting zero
commanded output before requesting ARM, and faults on stale, invalid, faulted
or non-idle status. Do not enable VELOCITY control until an explicit arm-ready
acknowledgement and an independently validated physical-stop contract exist.

The standalone controller safety monitor validates line, IMU and obstacle
freshness and ordering, latches faults, and requires a new go edge after a
veto. Its thresholds require robot validation. During a scan it blocks
following until a post-scan ahead reading; the caller must independently verify
the vehicle is stopped before requesting the scan and keep it stopped.
The monitor does not start a controller task, send commands, verify a motor
stop, or clear an emergency stop. The controller and motion task must enforce
those responsibilities when integrated, including a watchdog independent of
the monitor's periodic checks.
An opt-in controller monitor task connects these checks to the line, IMU,
forward-obstacle and fault buffers for diagnostics only. Start it before the
producers and never alongside another consumer of those buffers. It never
arms motion, requests scans, sends commands or verifies a physical stop; keep
motor power safely disabled. It steps the advisory state machine with go held
low; faults enter the faulted state and stop monitoring until restart. This is
not the vehicle controller task or a substitute for independent motor safety.

The standalone controller state machine is an advisory simulation of waiting,
following, blocked, scanning and faulted states. Its single owner supplies
sensor observations to its safety monitor and calls it every cycle with a
current trusted local go input. A go signal already high at startup, or held
high through a veto, cannot arm or rearm; turn GO off and on again after
conditions clear. Faults and emergency stop remain latched until restart.
The state machine does not start a task, steer, send motion commands, request
scans or verify physical stopping. `FOLLOWING` is permission to follow, not
proof that a vehicle is following a line. Scanning still requires independently
verified stopping and a fresh post-scan ahead observation. Remote commands are
not connected. Independent motor-stop and watchdog mechanisms remain required.

An opt-in `vehicle_controller_task` now connects this safety gate to the line,
IMU, obstacle, fault and motion-status buffers and sends
ARM/STOP/emergency-stop messages. Its required local-input callback supplies
GO and emergency-stop levels; dashboard commands alone cannot arm. It
sends no VELOCITY commands yet because the motion task still rejects steering
commands. The motion status is commanded state, not physical feedback. The
controller exclusively consumes those buffers, so do not start it alongside
`controller_monitor_start()`. It has not been hardware-tested.

With `VEHICLE=1`, the local GO input is a momentary button between GP20 and
GND, with the internal pull-up enabled. After an initial release, each
debounced press toggles GO; releasing the button does not turn GO off. The
controller samples it every 50 ms and requires two consecutive matching
samples. If a safety veto occurs while GO is on, toggle it off and on after
the veto clears to request rearming. The button and software STOP are not an
emergency stop. The emergency input is still hardcoded inactive; an independent
motor-power cutoff is required. Servo clearance and scan geometry have been
verified; the flag in `app_main.c` enables the vehicle startup path. The GO
button still controls whether the vehicle controller may run.

For a standalone IMU hardware check, build `make SENSOR_TEST=imu`. This test
and the opt-in vehicle startup select the experimental 0x32-family profile:
CTRL1 `0x2f` and CTRL4 `0x80`, which select 100 Hz and BDU on the candidate
family. The exact device is unconfirmed, so its actual rate may differ.
It first compares the raw trace timer with kernel time over a requested 250 ms;
the expected ratio is approximately 1000 timer ticks per kernel millisecond.
Do not interpret the trace's microsecond values if this comparison disagrees.
Readback does not prove that BDU works or that all three axes
belong to one conversion. At the BDU
prompt, move the board continuously for five seconds. The diagnostic reads each
low byte twice without reading its high byte, then reads the high byte and
checks the low byte again. `consistent-not-proof` is compatible with BDU;
`inconclusive` may mean there was insufficient movement or the low bytes
coincidentally matched. `changed-while-held` does not match the candidate
family's documented BDU behavior. This is not a full coherence test.
It reports the identity register without rejecting any value, then reads the
status and accelerometer together, then checks whether data-ready clears after
the read and reasserts within 100 ms. If data-ready is already set when checked
after the read, the result is inconclusive: a new sample may have arrived in between.
A detected transition does not establish exact sampling rate or sample age.
Readings remain freshness-unverified and `t_read` is the completion time of the
reads, not an acquisition timestamp. `burst_us` and `status_us` measure each
I2C API call using the RP2040 microsecond timer; they include scheduling and
driver overhead, not just wire time. These measurements do not prove that the
XYZ bytes are from one measurement.
The IMU-only build also records the burst's time inside the I2C driver and the
first and last I2C interrupt offsets, plus the first receive command and first
and last receive interrupts. It also reports when the task resumed, when STOP
completion was armed and whether STOP was already pending. Comparing these
with `burst_us` helps locate the delay; interrupts and scheduling can still
affect the measurements.
The shared I2C driver now signals completion after releasing its lock and
rejects premature wake-ups; trace instrumentation remains IMU-test-only.
Cross-core timeout behavior still needs hardware validation.
The `initial_open` line shows the first data-ready-gated sample before the
probes for comparison with `startup_sample=1` after the sensor is reopened.
The diagnostic requests a 20 ms delay between its five direct reads;
kernel timer granularity, freshness polling and I2C transactions add to the
interval. It then reinitializes the sensor and continuously exercises the
vehicle's data-ready-gated reader with a requested 50 ms delay. `t_lower` is
the conservative timestamp lower bound, and `age_bound` is its age measured
immediately before printing; neither is an exact acquisition time. The standalone test does not
run the IMU task's 200 ms age check or test controller disarming. The live
reader feeds the production IMU processor with zero travel (no hump-height
measurement). The live reader prints its first five samples after reopening
the sensor (`startup_sample=1` through `5`), then reports about once per
second (`startup_sample=0`) without slowing the sampling loop. When the flag
first latches, `TILT DETECTED` and the triggering reading print immediately.
With motor power
disconnected, start flat, wait for the live readings, and try a slow level
lift. Reset the Pico with the robot flat before trying a sideways lift. A
latched flag does not clear when the robot is set down.
With motors disconnected, confirm timely readings and test sensor-disconnect fault
handling and controller disarm separately before vehicle operation.

For a pickup-only check, build `make SENSOR_TEST=imu_tilt`. It opens the IMU
once and runs the same detector without the BDU probe or close/reopen sequence.
This bench diagnostic logs the first reading as `warmup_sample` without
evaluating it for tilt because a startup outlier was observed on the robot.
It then evaluates every reading, prints the first five evaluated samples, and
continues logging about once per second. The vehicle IMU task is unchanged.
With motor power disconnected, wait for `tilt_detected=0` on level ground
before lifting. Reset on level ground between trials.

For a manual GP15 servo check, build `make SENSOR_TEST=servo`: this default
image emits no servo PWM. To command a single pulse width, build, for example,
`make SENSOR_TEST=servo SERVO_TEST_US=1500`. The allowed values are 0 (off)
or 1000–2000 microseconds. The value is not a guarantee of safe travel.
After the serial monitor opens, the diagnostic warns for five seconds, then
continuously outputs only that pulse; it never sweeps or stops automatically,
even if the serial monitor disconnects. Disconnect servo power
before flashing or changing the setup. Start with the linkage removed and
verify that the commanded position is clear before reattaching it. Rebuild and
reflash for each small change; keep clear and stop before the mechanism
reaches the board. Disconnect servo power if anything is unsafe. This mode
does not verify servo limits or the ultrasonic Echo voltage, and does not
change the vehicle safety flags.
The diagnostic also prints PWM and GPIO register values; these do not prove
the signal or power reaches the servo.

For the HC-SR04/servo hardware diagnostic, build
`make SENSOR_TEST=ultrasonic`. It reports live distances on the USB serial
console and moves the servo to its ahead position. GP4 Echo safety and the
servo's 1500/1700/1900 us calibration are recorded as verified in this test
configuration. If the sensor, wiring, voltage divider or servo setup changes,
re-measure and update the configuration before running it.

The opt-in application WiFi service supports simulated callbacks or an outbound
MQTT bridge backend.
The template's built-in `cyw43_utk_task` owns CYW43, lwIP and MQTT; neither the
application task nor its callbacks may call those APIs. The service consumes copied
telemetry and reports connection status; it holds a message when the backend
reports temporary backpressure and faults on stale or invalid telemetry and
delivery failures. A ready controller must consume status and faults, and
publishers must handle full buffers. No service backend or remote command
handling is started by default. Callbacks must return
promptly and an independent supervisor must detect a stalled service.
The standalone command decoder understands the earlier dashboard's two-byte
packet, but is not connected to the service or controller. Its sequence wraps
at 255, and the packet has no sender timestamp or authentication; decoding
alone cannot prove freshness, prevent replay or authorize movement.

The outbound MQTT bridge adds a separate copied message buffer for the network
owner. After buffer initialization and startup tests, a single startup owner may
call `wifi_service_start_mqtt(period_ms, status_period_ms, telemetry_max_age_ms,
owner_max_age_ms)` in a `NET=lwip MQTT=1` build. The last argument bounds the
age of network-owner link snapshots and must be at least the service period;
allow for scheduling and the network poll interval. A missing, stale or
inconsistent snapshot stops the service. Until the first owner poll, status is
disconnected for at most that interval. Retry and error counts are not measured
by this backend and are reported as zero. `wifi_telemetry_make()` builds a
copied telemetry message containing the legacy five-byte packet. Its seven-bit
wire sequence comes from the low bits of the message's 32-bit sequence; the
producer must advance that sequence only after successful queueing and use the
operating-clock timestamp of its oldest input. The opt-in single-owner
`wifi_telemetry_producer_init()` and `wifi_telemetry_producer_send()` manage
that sequence, check input age and return queue-full errors without advancing
it. Start the service first, use a producer age limit no greater than the
service's, and handle all send errors. A clock fault latches until restart.
The producer does not acquire or verify sensor data. The legacy state map must be agreed
with the controller. It has no unknown battery value, so do not fabricate one
or publish until a verified battery measurement is available. The network owner
publishes the payload on `robot/telemetry` at QoS 0. No source of verified
telemetry data is connected, and neither bridge nor vehicle tasks start by default. Submissions
report a full queue; acceptance is not broker delivery, and QoS 0 can lose data.
The bridge retains a pending packet on temporary MQTT backpressure and stops
until reboot on stale/invalid data, clock or publish errors, reporting a
communication fault when possible. Stopping the service requests bridge shutdown;
one publish already in progress may still complete, and queued packets are
abandoned. Restart requires reboot, including if service task startup fails
after bridge creation. A stalled network owner or full fault queue
requires independent supervision. Queued packets are checked for age on dequeue;
they can become stale while disconnected. MQTT in this template is unencrypted;
use only on a trusted network. No incoming commands are connected.

### Read-only dashboard

`dashboard/` is a host-side Node.js application, separate from the Pico
firmware. Run `npm ci` and `npm start` in that directory. It subscribes only to
`robot/telemetry`, decodes the target's five-byte packet, ignores retained
packets, and marks data stale after five seconds. Configure `MQTT_URL`,
`MQTT_USER`, `MQTT_PASSWORD`, and optionally `PORT` through the process
environment. The HTTP server binds to `127.0.0.1`; do not expose it directly to
an untrusted network. The dashboard has no command-publish or barcode-mapping
route and is not a safety monitor. Firmware telemetry remains opt-in and has no
verified sensor-data producer connected by default.

The encoder bring-up uses the shared GPIO dispatcher. All interrupt clients
must be registered by one core-0 startup owner before the dispatcher starts;
live registration is not supported. A robot test reported left -2500 and right
2500 counts after one forward wheel revolution, and a 20 cm circumference.
Baseline deltas, reverse direction, repeatability and IRQ timing remain to be
verified. Pending edge bits do not record edge order or timestamps; fast or
simultaneous transitions can be missed.
The encoder source must be registered before dispatcher startup while the wheels
are stationary. Snapshots require a task pinned to core 0. Ambiguous or
inconsistent edge observations latch the snapshot invalid until restart; other
missed edges may still go undetected. The bring-up diagnostic prints invalid
counts for troubleshooting only.

The odometry producer can build messages from caller-supplied encoder snapshots
and send copies to separate line and IMU buffers. Motion publication is opt-in
through `odometry_task_start_with_motion()` and uses its own buffer; start its
consumer before opting in or the full buffer will stop odometry. An odometry task connects the
source and producer but is not started by the default image. Before enabling it,
verify both wheel polarities and calibrations, initialize the buffers and start
the shared dispatcher with the wheels stationary. Its startup caller must be
pinned to core 0 and handle any error. The task latches faults and stops
publishing after invalid input, clock, scheduling or delivery failures; it
reports the fault to the controller buffer, retrying if delivery fails. Fault
timestamps use the last known operating time (zero if unavailable). Consumers
must reject stale data and detect sequence gaps, including if fault delivery is
blocked. Motion feedback derives wheel speeds from the copied counts; no
wheel-speed messages are provided.
The task and encoder hardware have not been tested on the robot.

Pins: use **GP0–GP22 and GP26–GP28**. GP0/GP1 are free unless your team
builds with `CRASH_REPORT=1` (section 3), which needs them. Never use
GP23/GP24/GP25/GP29: they are wired to the Pico W radio. The Pico W's
on-board LED is also on the radio, so use an external LED instead.

After editing, run `make -j8` again in `build_make` and copy the new `.uf2`.

---

## 3. Build options: `SMP`, `CONSOLE`, `WIFI`, `TEST`, `CRASH_REPORT`

`make` on its own uses the defaults. To build differently, put options
after `make`. They can be combined in any order:

```sh
make -j8                          # defaults: SMP=1 CONSOLE=usb_cdc WIFI=cyw43 TEST=0 CRASH_REPORT=0
make SMP=0 -j8                    # single-core build
make CONSOLE=uart WIFI=none -j8   # no USB console, radio off (no Pico SDK needed)
make TEST=1 -j8                   # vehicle buffer test, no example tasks
make TEST=1 ODOMETRY_TEST=1 -j8  # live encoder bring-up, no motors
make SMP=0 SENSOR_TEST=encoder -j8 # encoder-only diagnostic, skips startup tests
make SENSOR_TEST=line_barcode -j8  # live line/barcode diagnostic, no motors
make VEHICLE=1 -j8                # start the vehicle tasks at boot (needs verified hardware, see app_main.c)
make CRASH_REPORT=1 -j8           # also see crash reports, on GP0/GP1 (uses those pins)
```

To set several options at once, list them all, separated by spaces. For
example, to track down a crash in the test build while running on one core only:

```sh
make SMP=0 TEST=1 CRASH_REPORT=1 -j8
#  -> build_make/mtk3pico_smp0_usb_cdc_wifi_crash_test.uf2
```

Any option you leave out keeps its default (here `CONSOLE=usb_cdc` and
`WIFI=cyw43`). Options are not remembered: give them on every `make`, as
the next plain `make` goes back to the defaults.

| Option | Values (**default**) | What it does | Image name | C macro your code can test |
|---|---|---|---|---|
| `SMP` | `0`, **`1`** | `1` runs the kernel on both RP2040 cores; `0` uses core 0 only | `smp1` / `smp0` | `TK_SUPPORT_SMP` (1 or 0) |
| `CONSOLE` | **`usb_cdc`**, `uart` | where `tm_printf` output goes. `usb_cdc` = USB cable; `uart` = UART0 only (GP0 TX / GP1 RX, needs a USB-serial adapter) | `_usb_cdc` / `_uart` | `TM_CONSOLE_USB_CDC` |
| `WIFI` | **`cyw43`**, `none` | `cyw43` powers up the Pico W radio and its driver task; `none` leaves the radio off | `_wifi` / *(nothing)* | `TM_WIFI_CYW43` |
| `TEST` | **`0`**, `1` | `1` runs vehicle startup tests | `_test` / *(nothing)* | `APP_TEST` (1 or 0) |
| `ODOMETRY_TEST` | **`0`**, `1` | `1` starts live encoder bring-up on GP16–19; requires `TEST=1` | `_odometry` / *(nothing)* | `APP_ODOMETRY_TEST` (1 or 0) |
| `SENSOR_TEST` | **`none`**, `ir`, `imu`, `imu_tilt`, `ultrasonic`, `servo`, `encoder`, `line_barcode` | Starts one hardware diagnostic without vehicle tasks or startup tests; `line_barcode` runs the live ADC/encoder decoder without starting motors | `_sensor_<name>` / *(nothing)* | `APP_SENSOR_TEST` |
| `VEHICLE` | **`0`**, `1` | `1` starts every vehicle task at boot: GPIO dispatcher, odometry, line/barcode ADC, IMU, obstacle, hardware motion and the arming controller. Startup refuses to run until the hardware verification flags in `app_program/app_main.c` are set after measuring on the robot. Requires `TEST=0`. | `_vehicle` / *(nothing)* | `APP_VEHICLE` (1 or 0) |
| `CRASH_REPORT` | **`0`**, `1` | `1` also copies the console to UART0 on **GP0 (TX) / GP1 (RX)**, 115200 baud, so you can see the last messages and the kernel's crash report (see below). **Uses GP0/GP1.** `CONSOLE=uart` always does this. | `_crash` / *(nothing)* | `TM_CONSOLE_UART` (1 or 0) |

Encoder bring-up prints signed counts and individual wheel distances every
second. GP16/17 are left A/B and GP18/19 are right A/B. It assumes 2500
transitions per wheel revolution and a 200 mm circumference; verify the signs,
calibration and circumference on each wheel. It does not initialize motors,
send odometry messages or start the vehicle controller. Rotate wheels slowly
by hand first: GPIO edge interrupts can miss transitions at high speeds.
The `SENSOR_TEST=encoder` image skips the startup tests and vehicle buffers;
it can help isolate encoder bring-up from a startup-test failure. It does not
fix a kernel fault or prove the encoder readings are reliable.

For vehicle bring-up, check the first sensor error on the serial console.
The IMU logs whether an I2C transfer failed or a register value did not match
what the driver expected. The line ADC logs whether an individual channel
read failed or the full sample exceeded its time budget. Later queue and
controller errors can follow when a sensor task stops.

**Why `CRASH_REPORT` exists.** USB output is not sent the moment you call
`tm_printf`: it waits in a buffer until the kernel's USB task sends it. If the
program crashes, no task runs again, so the last messages and the kernel's
crash report never reach your USB terminal. The terminal just goes quiet.
UART output *is* sent at once, even inside the crash handler. So when you
need to know why the Pico crashed or froze, build with `CRASH_REPORT=1`,
connect a USB-serial adapter (adapter RX → GP0, adapter GND → GND), and read
it at 115200 baud. Normal `tm_printf` output on USB works the same with
either setting. If a subsystem has claimed GP0/GP1 in `TEAM.md`, unplug that
hardware before using `CRASH_REPORT=1`.

The options are part of the image name, e.g.
`mtk3pico_smp1_usb_cdc_wifi_test.uf2`, so you can always tell which build
you are flashing. Switching options needs no `make clean`: the makefile
notices and recompiles everything.

Using an option in C code:

```c
#if TK_SUPPORT_SMP
    ctsk.tskatr |= TA_ASSPRC;   /* core affinity only exists on the dual-core build */
#endif

#if APP_TEST
    /* code that only exists in the test build */
#endif
```

### Where the options live in the makefile

The options are handled in **`build_make/pico_rp2040.mk`**. To find each piece, search the file for the
names in the left column (e.g. `grep -n "TEST" build_make/pico_rp2040.mk`).
Each option goes through the same steps:

| Step | What to look for | Example for `TEST` |
|---|---|---|
| 1. Default value | `NAME ?= value` near the top of the file | `TEST ?= 0` |
| 2. Allowed values | an `$(error Unknown ...)` check just below it | `$(error Unknown TEST '$(TEST)'; use 0 or 1)` |
| 3. Image-name suffix | `NAME_SUFFIX :=` and the `EXE_FILE :=` line | `TEST_SUFFIX := _test` |
| 4. C macro for the code | a `CFLAGS += -D...` line | `CFLAGS += -DAPP_TEST=$(TEST)` |
| 5. Rebuild on change | the `PROFILE_ID :=` line | `...-test$(TEST)` |
| 6. Extra source files (`CONSOLE`, `WIFI` only) | an `ifeq ($(CONSOLE),usb_cdc)` or `ifeq ($(WIFI),cyw43)` block that adds the USB/radio files and SDK include paths | — |

- **Changing a default:** edit the `?=` line from step 1, e.g.
  `CONSOLE ?= uart`. Every `make` without that option then uses the new value.
- **Adding your own option** (say `DEBUG=1` to turn on extra prints): copy
  how `TEST` is done. Add steps 1 to 5 next to the `TEST` lines, then
  use `#if APP_DEBUG` in your code. **Step 5 matters:** without it, switching
  the option can quietly link files that were compiled with the old setting.
- **Kernel settings** such as the maximum number of tasks, semaphores or
  mailboxes are not make options. They are in `config/config.h` (`CNF_MAX_...`).

The makefile also has `NET=lwip` and `MQTT=1` (TCP/IP and MQTT over WiFi).
They need the SDK's `lib/lwip` submodule and a `config/wifi_secrets.h`
(copy `config/wifi_secrets.example.h`). This guide does not cover them.

---

*micro T-Kernel 3.0 © Ken Sakamura / TRON Forum, distributed under the
T-License 2.2. This dual-core RP2040 port is derived from the official
μT-Kernel 3.0 BSP (`tron-forum/mtk3_bsp`, branch `pico_rp2040`).*
