# Team and subsystems

The subsystem folders contain the vehicle modules. Fill in their owners and
keep task and interface details current as integration progresses.

Fill this in at the start of the project and keep it up to date. It is the
single place that says **who owns what**, so that five people can work in
one firmware image without clashing over pins, priorities, or kernel objects.

A **subsystem** is one student's part of the product, with its own folder.
It may contain several tasks and own several peripherals (see section 3).

Rule: **only the owner edits a subsystem's folders.** To change another
subsystem, ask its owner or open a pull request for them to review.

## 1. Members and subsystems

| # | Subsystem (what it does) | Student ID | GitHub username | Code | Tests |
|---|---|---|---|---|---|
| 1 | WiFi | | | `app_program/subsystem1/` | `testing/subsystem1/` |
| 2 | Controller and motion | | | `app_program/subsystem2/` | `testing/subsystem2/` |
| 3 | Line and barcode | | | `app_program/subsystem3/` | `testing/subsystem3/` |
| 4 | IMU | | | `app_program/subsystem4/` | `testing/subsystem4/` |
| 5 | Obstacle detection | | | `app_program/subsystem5/` | `testing/subsystem5/` |
| 6 | Odometry and GPIO | | | `app_program/subsystem6/` | `testing/integration/` |

Shared files (`app_program/app_main.c`, `app_program/app.h`, this file)
are changed only by agreement of the team. The exception is claiming an empty
row in this file (a pin, a peripheral, an interface ID), which anyone may do
(section 2).

## 2. Pins

One owner per pin. Claim a pin here **before** you wire or code it.

**Why:** two subsystems using the same pin is one of the most common and
hardest-to-find mistakes in a team project. The code builds without any
error, but the hardware misbehaves. Each subsystem can pass its own tests
and then fail once they are combined, and two outputs driving the same pin
against each other can damage it. Writing the claim down first means a clash
is spotted when someone edits this table, not hours later on the bench.

**No instructor check needed.** The team keeps this table itself. You do not
need the instructor, or the rest of the team, to check or approve a claim:
if the row is empty, fill it in and commit. If two people claim the same
pin, they agree between themselves who moves. Only the *reserved* rows are
fixed.

| Pin | Owner | Used for |
|---|---|---|
| GP23, GP24, GP25, GP29 | *reserved* | Pico W radio |
| GP0 | | (taken whenever the team builds with `CRASH_REPORT=1`) |
| GP1 | | (taken whenever the team builds with `CRASH_REPORT=1`) |
| GP2 | IMU | I2C1 SDA |
| GP3 | IMU | I2C1 SCL |
| GP4 | obstacle detection | ultrasonic echo input |
| GP5 | obstacle detection | ultrasonic trigger output |
| GP6 | | |
| GP7 | | |
| GP8 | motion control | motor 1 A, left wheel |
| GP9 | motion control | motor 1 B, left wheel |
| GP10 | motion control | motor 2 A, right wheel |
| GP11 | motion control | motor 2 B, right wheel |
| GP12 | | |
| GP13 | | |
| GP14 | | |
| GP15 | obstacle detection | scanning servo PWM |
| GP16 | odometry bring-up | left encoder A |
| GP17 | odometry bring-up | left encoder B |
| GP18 | odometry bring-up | right encoder A |
| GP19 | odometry bring-up | right encoder B |
| GP20 | vehicle controller | active-low GO toggle button to GND; internal pull-up (the dashboard Start / Stop act on the same toggle, IF-12) |
| GP21 | | |
| GP22 | | |
| GP26 | line/barcode | right IR sensor, ADC0 |
| GP27 | line/barcode | center IR sensor, ADC1 |
| GP28 | line/barcode | left IR sensor, ADC2 |

These assignments record the supplied pin-connection table; they do not
enable any device or establish safe electrical levels. The table's ADC0 and
ADC2 labels were reversed relative to its GPIO numbers; the GPIO numbers were
confirmed. Verify sensor output voltages and calibration before activation.
The sensor used for barcode acquisition and the motor driver remain
unverified. No independent motor-disable arrangement is available.

For the GP15 servo, the operator reported safe positions of 1500 us right,
1700 us ahead and 1900 us left, with clearance. The physical scan limits have
been measured at 20 degrees to each side of center. The obstacle adapter uses
these pulses as bounded calibration points and maps them to its 80–120 degree
profile coordinates (100 degrees ahead). `VEHICLE_SERVO_LIMITS_VERIFIED` in
`app_program/app_main.c` is set because these positions and angles were
physically checked.

## 3. Peripherals

**Subsystems, tasks and peripherals are three different things:**

| Term | What it is | Example |
|---|---|---|
| **Subsystem** | one student's part of the product, one folder | "environment sensing", "motor control", "user interface" |
| **Task** | one thread of execution. A subsystem may have **as many tasks as it needs** | a sampling task and a filtering task, both in "environment sensing" |
| **Peripheral** | one hardware block on the RP2040 | I2C0, UART1, the ADC |

A driver is **not** a subsystem. It is code *inside* the subsystem that
needs that hardware: the "environment sensing" subsystem contains the I2C
sensor code, and owns I2C0. Write it as `drv_<name>.c` / `drv_<name>.h` in
that subsystem's folder, never under `device/`, and call it only from the
peripheral's owning task. The kernel's own drivers in `device/` (serial,
I2C, ADC) may be used instead; switching one on in `config/config_device.h`
changes shared configuration, so record it in the table below first.

**The rule: each peripheral is driven by exactly one task, its *owning
task*.** That task can be in any subsystem, and one subsystem may own
several peripherals (one owning task each, or one task owning all of them).
Every other task, in the same subsystem or another one, asks the owning task
through a kernel object (section 5) and never touches that peripheral itself.

*Why:* this port runs tasks on both cores at the same time. If two tasks
drive the same I2C or UART block at once, their transfers get mixed up, and
the drivers have no locking to stop it.

| Peripheral | Owner subsystem | Owning task | Pins |
|---|---|---|---|
| UART1 | | | |
| I2C0 | | | |
| I2C1 | IMU | IMU task | GP2, GP3 |
| SPI0 | | | |
| SPI1 | | | |
| ADC | line/barcode | line/barcode task | GP26–GP28 |
| PWM slices 4, 5 | motion control | motion task; **lent** to the vehicle bypass task while it drives round an obstacle (below) | GP8–GP11 |
| PWM slice 7 | obstacle detection | obstacle task | GP15 |
| WiFi radio, TCP/IP, MQTT | *kernel* | `cyw43_utk_task` (built in) | GP23–25, GP29 |

I2C1 sharing with other devices is not yet confirmed. The kernel I2C driver
is enabled (`DEVCNF_USE_IIC`); startup initializes only I2C1 on GP2/GP3 as
`iicb`. I2C0 still has GP8/GP9 configured and must not be initialized without
reassigning its motor pins. PWM ownership covers the entire slice, including
any unused channel on the same slice.

**Motor lending.** The motion task initializes the motor driver and is its
owner; the driver refuses calls from any other task. The obstacle bypass
must drive the motors directly, so `vehicle_bypass` borrows them with
`motion_task_borrow_motors()`: the motion task lends only while it is stopped
and armed, and does not touch the motors meanwhile; the bypass gives them
back (`motion_task_return_motors()`) before it reports DONE. A STOP or
emergency stop sent to the motion task while they are lent revokes them (the
borrower brakes at its next driver call). Exactly one task writes the motors
at any moment. Never call `motor_driver_*` from any other task.

### WiFi and MQTT are special

The radio and the network stack belong to the kernel's built-in WiFi service
task, `cyw43_utk_task` (`lib/libwifi/sysdepend/pico_rp2040/`). **No student
task may call `cyw43_...`, lwIP or MQTT functions, not even from a
"WiFi subsystem".** The calls are not safe from any other task, on either core.

What students *can* do:

- **Read the connection state** from any task, which is always safe:
  ```c
  #include "cyw43_utk.h"
  T_CYW43_UTK_STATUS st;
  cyw43_utk_get_status(&st);   /* st.ready, st.link_status, st.ip_addr, ... */
  ```
- **Send or receive data over the network** by adding code that *runs
  inside the service task*. The MQTT client is in
  `lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c`. It publishes a heartbeat
  today. A "cloud / reporting" subsystem would add a kernel object
  (e.g. a message buffer) that application tasks fill. The MQTT code then
  reads it with `TMO_POL` (never blocking) and publishes it.
  Because this changes shared library code, **agree it with the team and
  the instructor first**, and record it as an interface in section 5.
  It needs the `NET=lwip MQTT=1` build (README section 3).

## 4. Task priorities

1 is the highest. 1–4 are used by the kernel: the USB console (3) and the
WiFi radio (4) both run on core 0 and pre-empt every vehicle task there,
sometimes for several milliseconds. The control loop therefore runs on
core 1, and only the tasks that must read core-0 hardware stay on core 0.

| Subsystem | Task | Priority | Core | Why |
|---|---|---|---|---|
| *kernel* | USB console | 3 | 0 | built in |
| *kernel* | WiFi radio, lwIP, MQTT (`cyw43_utk_task`) | 4 | 0 | built in |
| 2 | motion | 8 | 1 | wheel PID: highest of the control loop |
| 2 | vehicle controller | 9 | 1 | |
| 3 | line/barcode | 9 | 1 | a sample interrupted for > 10 ms is re-read, up to 3 times, before it faults |
| 4 | IMU | 10 | 1 | |
| 2 | vehicle bypass | 10 | 0 | reads the encoders directly |
| 5 | obstacle | 10 | 0 | ultrasonic/servo GPIO registration needs core 0 |
| 6 | odometry, encoder bring-up | 10 | 0 | encoder GPIO interrupts need core 0 |
| 2 | vehicle startup (one-shot) | 11 | 0 | GPIO registrations need core 0 |
| 1 | WiFi service | 12 | any | only moves telemetry between buffers |
| 2 | controller monitor (not in `VEHICLE=1`) | 10 | any | |

Core: `TP_PRC1` = core 0, `TP_PRC2` = core 1, or "any". Do not move a
core-1 task to core 0 or "any", or give a vehicle task a priority of 4 or
below, without re-running the vehicle with the network on: the line and IMU
tasks fault on timing (`line ADC sample span exceeded budget`, IMU fault
code 17) when the radio pre-empts them.

## 5. Interfaces between subsystems

Every arrow between two subsystems is listed here. The **provider** creates
the kernel object and publishes its ID in its public header; the **user** only
sends to it or waits on it. Agree on the data format before coding.

| ID | From → To | Mechanism | Object ID name | Data / meaning | Agreed by |
|---|---|---|---|---|---|
| IF-06 | | | | | |
| IF-07 | application WiFi service → built-in network owner | copied message buffer, nonblocking | bridge-owned, private | legacy five-byte telemetry on `robot/telemetry`, QoS 0; opt-in, no delivery guarantee | team and instructor agreement confirmed |
| IF-08 | built-in network owner → application WiFi service | atomic link snapshot | bridge-owned, private | link and MQTT readiness with operating-clock timestamp; stale snapshots fault the service | team and instructor agreement confirmed |
| IF-09 | telemetry producer → application WiFi service | copied message buffer, nonblocking | `g_mbf_telemetry` | ordered telemetry messages; producer handles full-buffer and other errors | pending |
| IF-10 | motion task → controller | copied message buffer | `g_mbf_motion_status` | sequenced 100 ms commanded-PWM, active-command and latched-fault state; start/stop and faults published at once, PWM changes while running only at the 100 ms period; controller period ≤50 ms; moving status accepted only while a VELOCITY or TURN is live (+1 status period after STOP); `lent` = 1 while the motors are lent to the bypass (PWM fields are then the bypass's outputs; lend and return published at once), accepted only while a bypass runs; no ARM acknowledgement or physical feedback | implemented, unit-tested (VC-U22, VC-U28); hardware validation pending (VC-H01, OBS-H04) |
| IF-11 | vehicle controller → motion task | copied message buffer, nonblocking | `g_mbf_motion_commands` | ARM/STOP/emergency-stop; VELOCITY while following the line: `linear_velocity` v and `angular_velocity` w in cm/s, wheel targets left = v − w, right = v + w, \|w\| < \|v\|, \|v\| + \|w\| ≤ 20; TURN (junction spin): linear 0, angular = each wheel's speed, + = left; one per controller cycle, 200 ms lifetime; switching VELOCITY ↔ TURN needs STOP | implemented, unit-tested (MOT-U13, MOT-U15, VC-U22, VC-U26); hardware validation pending (VC-H01) |
| IF-12 | local input provider → vehicle controller | task-context callback | configured callback/context | GP20 button toggles GO; with `VEHICLE_DASHBOARD_RUN 1` (`app_main.c`) the dashboard's Start / Stop act on the same toggle (Start = a press that turns GO on, re-pressing if already on; Stop = GO off), read through `dashboard_take_run_command()` (one atomic word, no network access); emergency-stop input is not wired | hardware validation pending (WIFI-H04) |
| IF-13 | line/barcode task → vehicle controller | copied message buffer, nonblocking | `g_mbf_line_observations` | per sample: line position, `intersection_detected` (a confirmed junction: left sensor, then both line sensors), `picked_up` = 1 while all three IR sensors have read black (no floor) for `LINE_PICKUP_MS` = 500 ms; the controller stops on a pick-up while running; GO turns itself off, so one press starts it again | implemented, unit-tested (LB-U05, LB-U06, VC-U26, VC-U28); hardware validation pending (VC-H03) |
| IF-14 | motion task ↔ vehicle bypass task | function calls + one atomic word (`motion_task_borrow_motors` / `motion_task_return_motors`) | motion-task private | motor lending (section 3): requested by the bypass, granted by the motion task only while stopped and armed, withdrawn safely on timeout or abort; revoked by STOP / emergency stop | implemented; hardware validation pending (OBS-H04) |
| IF-15 | any vehicle task → network owner (dashboard) | copied message buffer, nonblocking (`dashboard_report_*`) + atomic words from the dashboard | `dashboard_link` private | events to `robot/barcode`, `robot/event`, `robot/telemetry/detail`: barcode detected and decoded, junction action done, obstacle distance and profile, avoidance phase, line rejoined, hump height (≤ 5 Hz) and peak, pick-up; from `robot/command`: next junction turn, Start / Stop. The next junction turn resets to LEFT on Start, Stop and pick-up | implemented; hardware validation pending (WIFI-H03–H05) |

Each interface is tested by at least one integration test
(`testing/integration/TEST_LOG.md`).

## 6. Kernel object budget

Limits are set in `config/config.h`. If the team needs more, raise the
limit there. The previous example's usage is no longer applicable; record
vehicle usage as it is verified.

| Object | Limit | S1 | S2 | S3 | S4 | S5 |
|---|---|---|---|---|---|---|
| Task | 32 | | | | | |
| Semaphore | 16 | | | | | |
| Event flag | 16 | | | | | |
| Mailbox | 12 | | | | | |
| Mutex | 12 | | | | | |
| Message buffer | 12 | | | | | |
| Fixed memory pool | 12 | | | | | |
| Variable memory pool | 12 | | | | | |
| Cyclic handler | 12 | | | | | |
| Alarm handler | 12 | | | | | |

(The kernel's USB and WiFi service tasks also use a few of these.)
