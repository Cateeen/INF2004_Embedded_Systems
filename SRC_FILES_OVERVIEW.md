# Source Files Overview - What Each Component Does

## 🏗️ Architecture Overview

The PicoCar embedded system is organized as a **modular robotics platform** running on the RP2040 (Raspberry Pi Pico) with **Micro T-Kernel RTOS**. It uses a **message-passing architecture** where independent tasks communicate via message buffers.

```
┌─────────────────────────────────────────────────────────────────┐
│                     Vehicle System (RTOS)                       │
├─────────────────────────────────────────────────────────────────┤
│                                                                   │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐         │
│  │ Motion   │  │   IMU    │  │   Line   │  │Obstacle  │  ... │
│  │ Control  │  │  Sensor  │  │  Follow  │  │Detection │       │
│  │  Task    │  │  Task    │  │  Task    │  │  Task    │       │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘  └────┬─────┘       │
│       │             │             │             │              │
│       └─────────────┼─────────────┼─────────────┘              │
│                     │ Message Buffers                           │
│                     ▼                                            │
│        ┌────────────────────────────┐                          │
│        │   Hardware Abstraction     │                          │
│        ├────────────────────────────┤                          │
│        │ • PWM Motor Driver         │                          │
│        │ • I2C IMU Driver           │                          │
│        │ • ADC Line Sensors         │                          │
│        │ • Ultrasonic Sensor        │                          │
│        │ • Encoder Quadrature       │                          │
│        │ • GPIO ISR Dispatcher      │                          │
│        └────────────────────────────┘                          │
│                     │                                            │
│                     ▼                                            │
│        ┌────────────────────────────┐                          │
│        │   RP2040 Hardware          │                          │
│        ├────────────────────────────┤                          │
│        │ • PWM Slices               │                          │
│        │ • I2C Ports                │                          │
│        │ • ADC Channels             │                          │
│        │ • GPIO Pins                │                          │
│        └────────────────────────────┘                          │
│                                                                   │
└─────────────────────────────────────────────────────────────────┘
```

---

## 📂 File Organization by Category

### 1. **HARDWARE DRIVERS** (Low-level hardware control)

#### [motor_driver.c](motor_driver.c) / [motor_driver.h](../include/motor_driver.h)
- **Purpose:** Control DC motors via PWM
- **Hardware:** Cytron Robo Pico motor controller, GPIO 8-11 (PWM)
- **Functions:**
  - `motor_driver_init()` - Initialize PWM hardware
  - `motor_set_speed()` - Command motor speed (-100 to +100%)
  - `motor_brake_all()` - Brake both motors
  - `motor_emergency_stop()` - Emergency halt and latch
  - `motor_clear_emergency_stop()` - Clear latch
- **Key Feature:** Braking interval before direction reversal (50ms)

#### [imu_driver.c](imu_driver.c) / [imu_driver.h](../include/imu_driver.h)
- **Purpose:** Read accelerometer/magnetometer data
- **Hardware:** LSM303 sensor on I2C (GP2=SDA, GP3=SCL, 400 kHz)
- **Functions:**
  - `imu_driver_init()` - Initialize I2C and configure sensors
  - `imu_driver_read_accel()` - Read 3-axis accelerometer
- **Key Feature:** 2ms I2C timeout for safety

#### [line_following.c](line_following.c) / [buddy3.h](../include/buddy3.h)
- **Purpose:** Read IR reflectance sensors for line detection
- **Hardware:** 3x IR sensors on ADC channels (Center, Right, Barcode)
- **Functions:**
  - `line_following_init()` - Initialize ADC
  - `line_following_read()` - Read sensor values and detect black/white
- **Key Feature:** Oversampling for noise reduction

#### [encoder.c](encoder.c) / [encoder.h](../include/encoder.h)
- **Purpose:** Decode quadrature encoder signals
- **Hardware:** None (pure logic, processes GPIO edges)
- **Functions:**
  - `encoder_init()` - Initialize with current state
  - `encoder_update()` - Process quadrature transitions
  - `encoder_get_count()` - Read accumulated count
  - `encoder_reset_count()` - Reset counter

---

### 2. **SENSOR PROCESSING** (Turn raw data into usable estimates)

#### [imu_processor.c](imu_processor.c) / [imu_processor.h](../include/imu_processor.h)
- **Purpose:** Convert raw accelerometer data to roll/pitch angles
- **Functions:**
  - `imu_processor_init()` - Initialize state
  - `imu_processor_update()` - Process accelerometer + distance → roll/pitch
- **Algorithm:** Integrates acceleration with distance (odometry) for better estimation

#### [wheel_feedback.c](wheel_feedback.c) / [wheel_feedback.h](../include/wheel_feedback.h)
- **Purpose:** Track wheel motion from encoder counts
- **Functions:**
  - `wheel_feedback_update_encoder()` - Process encoder edges
  - `wheel_feedback_get_distance()` - Calculate distance traveled
  - `wheel_feedback_get_speed()` - Calculate current speed
- **Key Feature:** Separate left/right tracking

#### [wheel_odometry.c](wheel_odometry.c) / [wheel_odometry.h](../include/wheel_odometry.h)
- **Purpose:** Calculate total distance from wheel encoders
- **Functions:**
  - `wheel_odometry_update()` - Sum left+right encoder counts
  - `wheel_odometry_get_distance_mm()` - Return total distance

#### [wheel_feedback_pid.c](wheel_feedback_pid.c) / [wheel_feedback_pid.h](../include/wheel_feedback_pid.h)
- **Purpose:** PID control for constant wheel speed
- **Functions:**
  - `wheel_feedback_pid_update()` - Adjust motor command based on actual speed
  - `wheel_feedback_pid_reset()` - Reset PID state

#### [pid_controller.c](pid_controller.c) / [pid_controller.h](../include/pid_controller.h)
- **Purpose:** Generic PID controller (reusable)
- **Functions:**
  - `pid_init()` - Configure PID gains (Kp, Ki, Kd)
  - `pid_update()` - Calculate control output
  - `pid_reset()` - Clear accumulated error

#### [barcode_scanner.c](barcode_scanner.c) / [barcode_scanner.h](../include/barcode_scanner.h)
- **Purpose:** Detect barcode markers on floor
- **Functions:**
  - `barcode_scan()` - Analyze IR readings for barcode pattern
  - Returns barcode reject codes if invalid

---

### 3. **MOTION PLANNING & CONTROL** (High-level movement)

#### [motion_control.c](motion_control.c) / [motion_control.h](../include/motion_control.h)
- **Purpose:** Execute motion commands (forward, backward, turn, stop)
- **State Machine:**
  - IDLE → WAITING → RUNNING → STOPPING
- **Functions:**
  - `motion_init()` - Initialize
  - `motion_process_command()` - Accept motion command
  - `motion_update()` - Run state machine, adjust motors based on feedback
  - `motion_get_status()` - Return current state
  - `motion_is_busy()` - Check if moving
- **Features:**
  - Braking before direction change
  - Feedback-based speed adjustment
  - Emergency stop integration

#### [line_steering.c](line_steering.c) / [line_steering.h](../include/line_steering.h)
- **Purpose:** Calculate steering correction to stay on line
- **Functions:**
  - `line_steering_update()` - Analyze line position → steering command
  - `line_steering_reset()` - Reset state

#### [line_status.c](line_status.c) / [line_status.h](../include/line_status.h)
- **Purpose:** Track if robot is on the line
- **Functions:**
  - `line_status_init()` - Initialize
  - `line_status_publish()` - Publish line presence via message

#### [wheel_speed_control.c](wheel_speed_control.c) / [wheel_speed_control.h](../include/wheel_speed_control.h)
- **Purpose:** Synchronize left/right motor speeds
- **Functions:**
  - `wheel_speed_control_update()` - Balance motor commands

#### [motion_feedback_bridge.c](motion_feedback_bridge.c) / [motion_feedback_bridge.h](../include/motion_feedback_bridge.h)
- **Purpose:** Connect feedback systems to motion controller
- **Functions:**
  - `motion_feedback_bridge_update()` - Integrate feedback into motion update

---

### 4. **OBSTACLE DETECTION & AVOIDANCE** (Ultrasonic scanning)

#### [obs_scan.c](obs_scan.c) / [obs_scan.h](../include/obs_scan.h)
- **Purpose:** State machine for ultrasonic scanning and obstacle avoidance
- **State Machine:**
  - IDLE → SCANNING → DECIDING → AVOIDING → RESUME
- **Functions:**
  - `obs_init()` - Initialize
  - `obs_step()` - Main state machine iteration
- **Features:**
  - Coarse scan (6 angles)
  - Fine scan (narrow beam around obstacles)
  - Proximity detection
  - Avoidance path planning

#### [obs_port.h](../include/obs_port.h) / obs_port_pico_bare.c / obs_port_pico_tk.c
- **Purpose:** Hardware abstraction for ultrasonic sensor
- **Functions:**
  - `obs_port_init()` - Initialize PWM servo and GPIO
  - `obs_port_trigger()` - Pulse ultrasonic transmitter
  - `obs_port_echo_time_us()` - Measure echo duration
  - `obs_port_servo_angle()` - Rotate sensor to angle
- **Variants:** 
  - `obs_port_pico_bare.c` - Bare-metal (direct GPIO)
  - `obs_port_pico_tk.c` - RTOS version

#### [obs_profile.c](obs_profile.c) / [obs_profile.h](../include/obs_profile.h)
- **Purpose:** Convert raw ultrasonic distance to position in space
- **Functions:**
  - `obs_profile_distance_to_lateral()` - Calculate obstacle X/Y position

#### [obs_pack.c](obs_pack.c) / [obs_pack.h](../include/obs_pack.h)
- **Purpose:** Bit-packing utilities for efficient message encoding
- **Functions:**
  - `field_put()` / `field_get()` - Read/write bit fields
  - Used by obstacle detection to pack scan results

---

### 5. **TASK IMPLEMENTATIONS** (RTOS task entry points)

Each task is a concurrent activity that runs in the RTOS scheduler. Tasks communicate via message buffers.

#### [motion_control_task.c](motion_control_task.c)
- **Purpose:** Execute motion commands and control motors
- **Task Entry:** `motion_control_task(INT stacd, void *exinf)`
- **Loop Period:** 10 ms
- **What it does:**
  1. Receives motion commands from message buffer
  2. Updates motion state machine
  3. Sends motor commands
  4. Publishes motion status
- **Dependencies:** 
  - `motor_driver` (hardware control)
  - `motion_control` (state machine)
  - `g_mbf_motion_commands` (input)
  - `g_mbf_motion_status`, `g_mbf_faults` (output)

#### [imu_task.c](imu_task.c)
- **Purpose:** Read IMU sensor and publish orientation estimates
- **Task Entry:** `imu_task(INT stacd, void *exinf)`
- **Loop Period:** 50 ms
- **What it does:**
  1. Read accelerometer via I2C
  2. Process data with `imu_processor`
  3. Publish roll/pitch estimates
  4. Handle I2C faults
- **Dependencies:**
  - `imu_driver` (hardware I2C communication)
  - `imu_processor` (angle calculation)
  - `g_mbf_imu_odometry` (input from odometry task)
  - `g_mbf_imu_estimates`, `g_mbf_faults` (output)

#### [line_barcode_task.c](line_barcode_task.c)
- **Purpose:** Detect line position and barcode markers
- **Task Entry:** `line_barcode_task(INT stacd, void *exinf)`
- **Loop Period:** 20 ms
- **What it does:**
  1. Read IR line sensors via ADC
  2. Detect line position (left/center/right)
  3. Detect barcode markers using encoder count
  4. Publish line observations and barcode events
- **Dependencies:**
  - `line_following` (ADC reading + black/white detection)
  - `barcode_scanner` (barcode detection)
  - `g_mbf_line_odometry` (encoder counts)
  - `g_mbf_line_observations`, `g_mbf_barcode_events` (output)

#### [obstacle_detection_task.c](obstacle_detection_task.c)
- **Purpose:** Scan for obstacles with ultrasonic sensor
- **Task Entry:** `obstacle_detection_task(INT stacd, void *exinf)`
- **Loop Period:** Variable (depends on scan)
- **What it does:**
  1. Runs obstacle scan state machine
  2. Triggers ultrasonic measurements
  3. Publishes obstacle positions
  4. Requests avoidance commands
- **Dependencies:**
  - `obs_scan` (state machine)
  - `obs_port` (hardware control)
  - `g_mbf_obstacle_commands` (output requests)
  - `g_mbf_obstacle_events` (output observations)

#### [vehicle_odometry_task.c](vehicle_odometry_task.c)
- **Purpose:** Process encoder pulses and publish distance traveled
- **Task Entry:** `vehicle_odometry_task(INT stacd, void *exinf)`
- **Loop Period:** Interrupt-driven (triggered by encoder edges)
- **What it does:**
  1. Receives GPIO interrupts from encoders
  2. Updates encoder counts via dispatcher
  3. Calculates distance/speed
  4. Publishes odometry to other tasks
- **Dependencies:**
  - `vehicle_gpio_dispatcher` (ISR routing)
  - `wheel_feedback` (encoder processing)
  - Publishes to `g_mbf_line_odometry`, `g_mbf_imu_odometry` (output)

#### [wifi_service_task.c](wifi_service_task.c)
- **Purpose:** WiFi communication and telemetry
- **Task Entry:** `wifi_service_task(INT stacd, void *exinf)`
- **Loop Period:** Variable (WiFi events)
- **What it does:**
  1. Connect to WiFi/MQTT broker
  2. Receive remote commands
  3. Publish vehicle telemetry
  4. Handle MQTT messages
- **Dependencies:**
  - `wifi_mqtt_port_pico` (Pico WiFi driver)
  - `g_mbf_remote_commands` (input)
  - `g_mbf_telemetry` (output)

#### [vehicle_controller_task.c](vehicle_controller_task.c)
- **Purpose:** High-level behavior orchestration
- **Task Entry:** `vehicle_controller_task(INT stacd, void *exinf)`
- **Loop Period:** 50 ms
- **What it does:**
  1. Decide robot behavior (line follow, obstacle avoid, etc.)
  2. Coordinate subsystems
  3. Implement state machine (exploration, emergency stop, etc.)
  4. Send motion commands
- **Dependencies:**
  - Reads all sensor message buffers
  - Sends to `g_mbf_motion_commands` (output)

---

### 6. **SYSTEM & INTEGRATION** (Bring it all together)

#### [vehicle_system.c](vehicle_system.c) / [vehicle_system.h](../include/vehicle_system.h)
- **Purpose:** RTOS initialization and task creation
- **Functions:**
  - `vehicle_system_run()` - Main system startup
    1. Create message buffers
    2. Create all tasks
    3. Start RTOS scheduler (never returns)
  - Declares global message buffer IDs used by all tasks
- **Message Buffers Created:**
  - Sensor outputs: line, barcode, IMU, obstacle
  - Motion inputs/outputs: motion_commands, motion_status
  - Coordination: remote_commands, telemetry, faults
  - Internal: line_odometry, imu_odometry

#### [vehicle_tasks.c](vehicle_tasks.c) / [vehicle_tasks.h](../include/vehicle_tasks.h)
- **Purpose:** Task creation helpers
- **Functions:**
  - `create_task()` - Generic task creation
  - `create_all_tasks()` - Create all vehicle tasks
  - Defines task priorities and stack sizes

#### [vehicle_gpio_dispatcher.c](vehicle_gpio_dispatcher.c) / [vehicle_gpio_dispatcher.h](../include/vehicle_gpio_dispatcher.h)
- **Purpose:** Route GPIO interrupts to appropriate handlers
- **What it does:**
  - Registers ISR callbacks for GPIO pins
  - On interrupt: Call registered callback (encoder pulse handlers)
  - Dispatches to `vehicle_odometry_task` via callback
- **Functions:**
  - `vehicle_gpio_dispatcher_init()` - Initialize
  - `vehicle_gpio_dispatcher_register()` - Register handler

#### [car_feedback.c](car_feedback.c) / [car_feedback.h](../include/car_feedback.h)
- **Purpose:** Encoder edge detection callbacks
- **Functions:**
  - `car_feedback_left_edge()` - Left encoder pulse
  - `car_feedback_right_edge()` - Right encoder pulse
  - Called by GPIO dispatcher

#### [car_measurements.c](car_measurements.c) / [car_measurements.h](../include/car_measurements.h)
- **Purpose:** Distance measurement tracking
- **Functions:**
  - `car_measurements_init()` - Initialize
  - `car_measurements_publish_distance()` - Publish distance to telemetry

#### [car_gate.c](car_gate.c)
- **Purpose:** Supervisory control - implements "gate" mechanism
- **Functions:**
  - `car_gate_set_control()` - Allow/deny motor commands
  - Used for safety: only trusted local input can authorize GO

---

### 7. **COMMUNICATION & MESSAGING**

#### [vehicle_messages.h](../include/vehicle_messages.h)
- **Purpose:** Define all message types and structures
- **Message Types:**
  - `line_observation_message_t` - Line sensor data
  - `barcode_event_message_t` - Barcode detection
  - `imu_estimate_message_t` - Orientation estimate
  - `obstacle_event_message_t` - Obstacle position
  - `motion_command_message_t` - Motion request
  - `motion_status_message_t` - Motion state
  - `fault_message_t` - System faults
  - And more...
- **Common Header:** All messages have sequence, timestamp, version

#### [telemetry_protocol.c](telemetry_protocol.c) / [telemetry_protocol.h](../include/telemetry_protocol.h)
- **Purpose:** Serialize/deserialize telemetry for WiFi transmission
- **Functions:**
  - `telemetry_encode()` - Convert messages to network format
  - `telemetry_decode()` - Parse network data

#### [buddy1.c](buddy1.c) / [buddy1.h](../include/buddy1.h)
- **Purpose:** WiFi/MQTT helper module for message handling
- **Functions:**
  - `buddy1_init()` - Initialize
  - `buddy1_process()` - Handle WiFi messages

---

### 8. **HARDWARE ABSTRACTION & PORTS**

#### [wifi_mqtt_port_pico.c](wifi_mqtt_port_pico.c) / [wifi_mqtt_port.h](../include/wifi_mqtt_port.h)
- **Purpose:** Pico-specific WiFi/MQTT driver
- **Hardware:** CYW43 WiFi chip on Pico W
- **Functions:**
  - `wifi_mqtt_port_init()` - Connect WiFi
  - `wifi_mqtt_port_disconnect()` - Disconnect
  - `wifi_mqtt_port_set_callback()` - Set message handler
  - `wifi_mqtt_port_poll()` - Check for messages

---

## 🔄 Data Flow Example: "Move Forward"

```
user sends command via WiFi
        ↓
wifi_service_task receives via MQTT
        ↓
Publishes to: g_mbf_remote_commands (message buffer)
        ↓
vehicle_controller_task receives command
        ↓
Decides: "Execute motion_forward"
        ↓
Publishes to: g_mbf_motion_commands (message buffer)
        ↓
motion_control_task receives motion command
        ↓
Calls: motion_process_command() → motor_set_speed()
        ↓
motor_driver updates PWM hardware
        ↓
Motors start turning
        ↓
GPIO interrupts: encoder edges triggered
        ↓
vehicle_gpio_dispatcher routes to: car_feedback_left/right_edge()
        ↓
vehicle_odometry_task updates distance count
        ↓
Publishes to: g_mbf_line_odometry, g_mbf_imu_odometry
        ↓
motion_control_task receives odometry feedback
        ↓
motion_control → motion_update() → wheel_speed_control()
        ↓
Adjusts motor speeds to maintain balance
        ↓
Publishes to: g_mbf_motion_status
        ↓
vehicle_controller_task monitors status
        ↓
When desired distance reached:
        ↓
Sends stop command → motion_control halts motors
```

---

## 📊 Task Priority & Timing

| Task | Priority | Period | Purpose |
|------|----------|--------|---------|
| **Motion Control** | 3 (high) | 10 ms | Motor control (critical for real-time) |
| **Vehicle Controller** | 4 | 50 ms | Behavior decision making |
| **IMU** | 6 | 50 ms | Orientation sensing |
| **Odometry** | 7 | Interrupt | Encoder pulse processing |
| **Line/Barcode** | 8 | 20 ms | Line detection |
| **Obstacle Detection** | 5 | Variable | Ultrasonic scanning |
| **WiFi/Network** | 10 (low) | Variable | Communication |

**Lower number = higher priority (preempts lower priority tasks)**

---

## 🛡️ Safety Features

1. **Motor Emergency Stop**
   - `motor_emergency_stop()` latches stop state
   - Further motor commands rejected until cleared
   - Triggered on fault detection

2. **Gate Mechanism**
   - `car_gate.c` - Only trusted local input (GPIO) can authorize motion
   - WiFi commands cannot directly control motors
   - Commands go through vehicle_controller decision logic

3. **Fault Detection & Reporting**
   - I2C timeout (2ms) prevents sensor hang
   - Fault messages published to `g_mbf_faults`
   - Vehicle controller monitors faults
   - System can shut down on critical faults

4. **Encoder-Based Speed Feedback**
   - `wheel_feedback_pid` adjusts motor command based on actual speed
   - Detects wheel slip or blockage
   - Synchronizes left/right speeds

---

## 📈 Performance Characteristics

- **Motion Update Latency:** 10 ms (100 Hz control loop)
- **I2C Communication:** 2 ms timeout
- **Encoder Resolution:** Quadrature × wheel diameter
- **Network Latency:** Variable (WiFi dependent)
- **Total Responsive Time:** ~50-100 ms (task scheduling + processing)

---

## ✅ Summary Table

| Component | File | Purpose | Type |
|-----------|------|---------|------|
| **Motor Driver** | motor_driver.c | PWM motor control | Hardware Driver |
| **IMU Driver** | imu_driver.c | I2C sensor reading | Hardware Driver |
| **Line Sensor** | line_following.c | ADC IR reading | Hardware Driver |
| **Encoder** | encoder.c | Quadrature decoding | Hardware Driver |
| **IMU Processor** | imu_processor.c | Roll/pitch calculation | Sensor Processing |
| **Wheel Feedback** | wheel_feedback.c | Distance tracking | Sensor Processing |
| **Wheel Odometry** | wheel_odometry.c | Total distance | Sensor Processing |
| **Motion Control** | motion_control.c | State machine | Motion Planning |
| **Line Steering** | line_steering.c | Steering calculation | Motion Planning |
| **Obstacle Detection** | obs_scan.c | Ultrasonic scanning | Obstacle Avoidance |
| **Motion Task** | motion_control_task.c | Motor execution | RTOS Task |
| **IMU Task** | imu_task.c | Sensor polling | RTOS Task |
| **Line Task** | line_barcode_task.c | Line detection | RTOS Task |
| **Obstacle Task** | obstacle_detection_task.c | Ultrasonic scanning | RTOS Task |
| **Odometry Task** | vehicle_odometry_task.c | Encoder processing | RTOS Task |
| **WiFi Task** | wifi_service_task.c | Network comm | RTOS Task |
| **Controller Task** | vehicle_controller_task.c | Behavior orchestration | RTOS Task |
| **System Boot** | vehicle_system.c | RTOS setup | System |
| **GPIO Dispatcher** | vehicle_gpio_dispatcher.c | ISR routing | System |

---

Good luck with your embedded systems project! 🚀
