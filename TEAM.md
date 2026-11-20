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
| GP20 | vehicle controller | active-low GO toggle button to GND; internal pull-up |
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
| I2C1 | IMU | IMU task (planned) | GP2, GP3 |
| SPI0 | | | |
| SPI1 | | | |
| ADC | line/barcode | line/barcode task (planned) | GP26–GP28 |
| PWM slices 4, 5 | motion control | motion task (planned) | GP8–GP11 |
| PWM slice 7 | obstacle detection | obstacle task (planned) | GP15 |
| WiFi radio, TCP/IP, MQTT | *kernel* | `cyw43_utk_task` (built in) | GP23–25, GP29 |

I2C1 sharing with other devices is not yet confirmed. The kernel I2C driver
remains disabled; when enabled, startup initializes only I2C1 on GP2/GP3 as
`iicb`. I2C0 still has GP8/GP9 configured and must not be initialized without
reassigning its motor pins. Confirm bus ownership before enabling I2C. PWM
ownership covers the entire slice, including any unused channel on the same
slice.

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

1 is the highest. 1–4 are used by the kernel (USB console 3, WiFi 4).
The vehicle tasks below currently use priority 10.

| Subsystem | Priority | Tasks |
|---|---|---|
| 1 | 10 | WiFi service |
| 2 | 10 | motion, vehicle controller, controller monitor |
| 3 | 10 | line/barcode |
| 4 | 10 | IMU |
| 5 | 10 | obstacle |
| 6 | 10 | odometry, encoder bring-up |

Core: `TP_PRC1` = core 0, `TP_PRC2` = core 1, or "any".

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
| IF-10 | motion task → controller | copied message buffer | `g_mbf_motion_status` | sequenced 100 ms commanded-PWM, active-command and latched-fault state; controller period ≤50 ms; no ARM acknowledgement or physical feedback | implementation draft; hardware validation pending |
| IF-11 | vehicle controller → motion task | copied message buffer, nonblocking | `g_mbf_motion_commands` | ARM/STOP/emergency-stop; VELOCITY is not yet issued | implementation in progress |
| IF-12 | local input provider → vehicle controller | task-context callback | configured callback/context | GP20 button toggles GO; emergency-stop input is not wired | hardware validation pending |

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
