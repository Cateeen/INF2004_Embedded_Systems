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
