# Device Driver Porting Checklist for Micro T-Kernel

## Overview
This checklist guides you through porting device drivers from bare-metal Pico SDK to micro T-Kernel. The hardware control logic stays the same; the RTOS integration happens at the task level.

---

## Phase 1: Pre-Porting Analysis ✓

### Understanding Current Architecture
- [ ] Review `car_main.c` - understand bare-metal boot sequence
- [ ] Review `vehicle_system.c` - understand RTOS task creation
- [ ] Review existing tasks (`imu_task.c`, `motion_control_task.c`)
- [ ] Identify which drivers are needed:
  - [ ] `motor_driver.c` - PWM motor control
  - [ ] `imu_driver.c` - I2C sensor communication
  - [ ] `line_following.c` - ADC-based line sensing
  - [ ] `encoder.c` - Quadrature decoding
  - [ ] `vehicle_gpio_dispatcher.c` - ISR management

---

## Phase 2: Driver Code Preparation

### Copy Driver Files
- [ ] Copy `src/motor_driver.c` → new project `src/`
- [ ] Copy `include/motor_driver.h` → new project `include/`
- [ ] Copy `src/imu_driver.c` → new project `src/`
- [ ] Copy `include/imu_driver.h` → new project `include/`
- [ ] Copy `src/line_following.c` → new project `src/`
- [ ] Copy `include/buddy3.h` (line sensor config) → new project `include/`
- [ ] Copy `src/encoder.c` → new project `src/`
- [ ] Copy `include/encoder.h` → new project `include/`
- [ ] Copy `src/vehicle_gpio_dispatcher.c` → new project `src/`
- [ ] Copy `include/vehicle_gpio_dispatcher.h` → new project `include/`

### Code Review - No Changes at This Stage
- [ ] Verify files compile in isolation
- [ ] Verify no RTOS-specific includes (`<tk/tkernel.h>`)
- [ ] Verify no task scheduling calls (no `tk_dly_tsk()`, etc.)
- [ ] ✅ **These drivers remain pure Pico SDK code**

---

## Phase 3: Motor Driver Thread Safety

### Add Mutex Support to motor_driver.c

#### Step 1: Add Includes
- [ ] Add `#include <tk/tkernel.h>` at top of `motor_driver.c`

#### Step 2: Add Mutex Variable
```c
// Add after existing static variables:
static ID s_motor_mutex = NULL;
```
- [ ] Add static mutex ID declaration

#### Step 3: Add Initialization Function
- [ ] Create new function `motor_driver_mutex_init()`:
```c
void motor_driver_mutex_init(void) {
    T_CMTX config = {0};
    config.mtxatr = TA_TFIFO;
    s_motor_mutex = tk_cre_mtx(&config);
    if (s_motor_mutex < 0) {
        printf("ERROR: motor mutex creation failed\n");
    }
}
```
- [ ] Add declaration to `motor_driver.h`:
```c
void motor_driver_mutex_init(void);
```

#### Step 4: Add Helper Function
- [ ] Create `motor_initialized()` for status checking:
```c
bool motor_initialized(void) {
    return motor_initialized;  // Returns existing static flag
}
```
- [ ] Add to `motor_driver.h`:
```c
bool motor_initialized(void);
```

#### Step 5: Protect All Public Functions
- [ ] Wrap `motor_set_speed()` with mutex:
```c
void motor_set_speed(motor_id_t motor, int16_t speed_percent) {
    if (s_motor_mutex != NULL) {
        tk_loc_mtx(s_motor_mutex, TMO_FEVR);  // Lock
    }
    // ... existing code ...
    if (s_motor_mutex != NULL) {
        tk_unl_mtx(s_motor_mutex);  // Unlock
    }
}
```
- [ ] Wrap `motor_brake_all()` with mutex
- [ ] Wrap `motor_emergency_stop()` with mutex
- [ ] Wrap `motor_clear_emergency_stop()` with mutex
- [ ] ✅ Read-only function `motor_is_emergency_stopped()` doesn't need mutex

---

## Phase 4: Task Integration

### Motion Control Task

#### File: motion_control_task.c

**Initialization Section (runs once at task startup):**
- [ ] Call `motor_driver_init()` **before** `motion_control_init()`
  ```c
  void motion_control_task(INT stacd, void *exinf) {
      motor_driver_init();           // ← Add this
      motor_driver_mutex_init();     // ← Add this
      error = motion_control_init();
      // ...
  }
  ```

**Include Headers:**
- [ ] Add `#include <tk/tkernel.h>`

**Main Loop (replace sleep):**
- [ ] Find `sleep_ms(MOTION_UPDATE_PERIOD_MS);`
- [ ] Replace with `tk_dly_tsk(MOTION_UPDATE_PERIOD_MS);`

**Error Handling:**
- [ ] Add startup failure handling:
  ```c
  if (!motor_initialized()) {
      publish_fault(now_ms(), MOTION_FAULT_START_FAILED);
      return;  // Exit task on critical failure
  }
  ```

**Testing Checklist:**
- [ ] [ ] Compile: `make` (no errors)
- [ ] [ ] Link: All symbols resolve
- [ ] [ ] Run: Task starts without crashes
- [ ] [ ] Motors: Respond to commands
- [ ] [ ] Timing: 10ms period maintained

---

### IMU Task

#### File: imu_task.c

**Initialization Section:**
- [ ] Verify `imu_driver_init()` is called (should already be there)

**Main Loop:**
- [ ] Find all `sleep_ms()` calls in the loop
- [ ] Replace with `tk_dly_tsk()`:
  ```c
  // OLD:
  sleep_ms(IMU_SAMPLE_PERIOD_MS);
  
  // NEW:
  tk_dly_tsk(IMU_SAMPLE_PERIOD_MS);
  ```

**Include Headers:**
- [ ] Verify `#include <tk/tkernel.h>` present (should be for `tk_rcv_mbf`)

**Testing Checklist:**
- [ ] [ ] Compile: No errors
- [ ] [ ] Link: All symbols resolve
- [ ] [ ] Run: Task starts without crashes
- [ ] [ ] I2C: Sensor reads valid accelerometer data
- [ ] [ ] Timing: 50ms period maintained

---

### Line Barcode Task

#### File: line_barcode_task.c

**Initialization Section:**
- [ ] Call `line_following_init()` at task startup:
  ```c
  void line_barcode_task(INT stacd, void *exinf) {
      line_following_init();      // ← Add this
      // ... rest of init ...
  }
  ```

**Main Loop:**
- [ ] Replace `sleep_ms()` with `tk_dly_tsk()`

**Include Headers:**
- [ ] Verify `#include <tk/tkernel.h>` present

**Testing Checklist:**
- [ ] [ ] Compile: No errors
- [ ] [ ] ADC: Sensor values read correctly
- [ ] [ ] Detection: Black/white transitions detected

---

### Obstacle Detection Task

#### File: obstacle_detection_task.c

**Initialization Section:**
- [ ] Call `obs_port_init()` at task startup:
  ```c
  void obstacle_detection_task(INT stacd, void *exinf) {
      obs_port_init();            // ← Add this
      // ... rest of init ...
  }
  ```

**Main Loop:**
- [ ] Replace `sleep_ms()` with `tk_dly_tsk()`

**Include Headers:**
- [ ] Verify `#include <tk/tkernel.h>` present

**Testing Checklist:**
- [ ] [ ] Compile: No errors
- [ ] [ ] Ultrasonic: Valid distance readings

---

## Phase 5: Boot Code Changes

### File: vehicle_system.c

**Remove these lines:**
```c
// ❌ DELETE THESE:
motor_driver_init();
imu_driver_init();
line_following_init();
// ... any other driver init calls
```
- [ ] Remove all driver initialization calls
- [ ] Drivers will initialize inside their own tasks

**Keep these:**
- [ ] Message buffer creation
- [ ] Task creation
- [ ] Task startup (`tk_sta_tsk()`)

**Checklist:**
- [ ] [ ] Removed all driver init calls
- [ ] [ ] Message buffers still created
- [ ] [ ] Tasks still created
- [ ] [ ] Compilation succeeds

---

### File: car_main.c

**Verify no changes needed:**
- [ ] `main()` calls `vehicle_system_run()`
- [ ] No driver initialization in `main()`
- [ ] No `multicore_launch_core1()` calls (remove if present)

**Checklist:**
- [ ] [ ] No driver init calls in main
- [ ] [ ] Calls `vehicle_system_run()` correctly
- [ ] [ ] No explicit core launches

---

## Phase 6: Compilation and Building

### Setup Build Environment
- [ ] Ensure micro T-Kernel includes in path
- [ ] Verify `CMakeLists.txt` includes RTOS libraries
- [ ] Check `-I` paths point to `<tk/tkernel.h>`

### Compilation Steps
- [ ] [ ] `cmake .` (configure)
- [ ] [ ] `make` (build)
- [ ] [ ] Check for errors:
  ```
  undefined reference to 'tk_cre_mtx'  → Missing RTOS library
  'sleep_ms' called in task context    → Need tk_dly_tsk()
  'motor_driver_init' not in task      → Need to call in task startup
  ```

### Fixing Common Errors
| Error | Cause | Fix |
|-------|-------|-----|
| `undefined reference to 'tk_*'` | RTOS library not linked | Add `-ltkernel` to CMakeLists.txt |
| `'sleep_ms' in task loop` | Using bare-metal delay | Replace with `tk_dly_tsk()` |
| `motor_set_speed() not locked` | Race condition risk | Add mutex lock/unlock |
| `motor_driver_init() called twice` | Init in both main and task | Move to task only |

**Testing Checklist:**
- [ ] [ ] No compilation errors
- [ ] [ ] No link errors
- [ ] [ ] Binary size reasonable (<100KB for RP2040)

---

## Phase 7: Functional Testing

### Unit Tests (Already Existing)
- [ ] `tests/test_motor_driver.c` - Run existing tests
- [ ] `tests/test_encoder.c` - Verify quadrature logic
- [ ] `tests/test_imu_processor.c` - Verify math

**Expected Results:**
- [ ] [ ] All existing unit tests pass
- [ ] [ ] No RTOS-specific tests needed for drivers

### Hardware Initialization Test
```c
// Add temporary test in motion_control_task startup:
if (!motor_initialized()) {
    printf("FAIL: Motor initialization failed\n");
    return;
}
printf("PASS: Motor initialized\n");

// Test I2C
imu_driver_read_accel(&data);
if (!data.i2c_ok) {
    printf("FAIL: IMU I2C read failed\n");
} else {
    printf("PASS: IMU I2C working\n");
}

// Test ADC
ir_readings_t ir;
line_following_read(&ir);
printf("PASS: ADC working, center=%d\n", ir.center);
```

**Checklist:**
- [ ] [ ] Add temporary initialization logging
- [ ] [ ] Verify each driver starts correctly
- [ ] [ ] Check for initialization order issues
- [ ] [ ] Remove logging after verification

### Hardware Functional Test
- [ ] [ ] Connect motors to Pico
  - Set them on the ground (not in air)
  - Verify voltage (5V or 3.3V per motor spec)
- [ ] [ ] Run motion control task
  - Check motor response to commands
  - Verify no jittery/unstable behavior
  - Emergency stop works

- [ ] [ ] Connect IMU (LSM303)
  - Verify I2C communication
  - Rotate robot → check accelerometer values change
  - Check pitch/roll calculations in `imu_processor`

- [ ] [ ] Connect line sensors (IR)
  - Place robot on black line
  - Verify center sensor detects black
  - Move to white → verify detection changes

- [ ] [ ] Connect ultrasonic sensor
  - Move obstacle closer/farther
  - Verify distance readings change appropriately
  - Check timeout handling if sensor disconnected

**Checklist:**
- [ ] [ ] Motors respond correctly
- [ ] [ ] IMU reads valid data
- [ ] [ ] Line sensors functional
- [ ] [ ] Ultrasonic distance working
- [ ] [ ] No I2C/ADC communication errors

---

## Phase 8: Concurrency and Timing

### Task Timing Verification
Run with `printf()` timestamp logging:

```c
// In each task startup:
printf("TASK STARTED: task_name at %lu ms\n", now_ms());

// In main loop:
printf("TASK LOOP: task_name iteration at %lu ms\n", now_ms());
```

**Expected Timing:**
- Motion control: ~10ms per iteration
- IMU: ~50ms per sample
- Line barcode: Variable (depends on line position)
- Obstacle: Variable (depends on sensor speed)

**Checklist:**
- [ ] [ ] Motion task: 10ms period consistent
- [ ] [ ] IMU task: 50ms period maintained
- [ ] [ ] No task blocked or stuck
- [ ] [ ] Timestamps monotonically increase

### Concurrency Stress Test
Run all tasks simultaneously:

```c
// In vehicle_system_run(), start all tasks at once:
tk_sta_tsk(g_tsk_motion_control, 0);
tk_sta_tsk(g_tsk_imu, 0);
tk_sta_tsk(g_tsk_obstacle_detection, 0);
tk_sta_tsk(g_tsk_line_barcode, 0);
tk_sta_tsk(g_tsk_wifi_service, 0);
```

**Stress Test Scenario:**
- All tasks reading from sensors simultaneously
- Motion task sending motor commands
- Multiple message buffer sends/receives

**Expected Behavior:**
- No crashes
- No deadlocks
- I2C timeouts still respected
- No message buffer overflow

**Checklist:**
- [ ] [ ] All tasks start without crashes
- [ ] [ ] I2C communication stable under load
- [ ] [ ] No priority inversion observed
- [ ] [ ] Message buffers don't overflow
- [ ] [ ] Task timing still met

---

## Phase 9: Edge Case Testing

### I2C Failure Scenarios
- [ ] Disconnect IMU during operation
  - Expected: `i2c_ok = false` flagged
  - Fault message sent
  - Task recovers on reconnect
- [ ] Loose I2C wires
  - Expected: 2ms timeout triggered
  - No system freeze (timeout protection works)

**Checklist:**
- [ ] [ ] I2C timeout protects against hangs
- [ ] [ ] Fault handling works
- [ ] [ ] Task recovers gracefully

### Motor Emergency Stop
- [ ] Motor task running at full speed
- [ ] Another task calls `motor_emergency_stop()`
  - Expected: Motors stop immediately
  - Mutex protection prevents race
  - Further commands rejected
- [ ] `motor_clear_emergency_stop()` called
  - Expected: Motors ready to move again

**Checklist:**
- [ ] [ ] Emergency stop works from any task
- [ ] [ ] No race condition crashes
- [ ] [ ] Motors prevent motion until cleared

### Message Buffer Overflow
- [ ] Publish messages faster than they're consumed
  - Expected: Message buffer holds N messages
  - Oldest message dropped when full
  - No system crash
  
**Checklist:**
- [ ] [ ] No buffer overflow crashes
- [ ] [ ] Latest messages received
- [ ] [ ] Task continues running

---

## Phase 10: Final Validation

### Code Review Checklist
- [ ] No `sleep_ms()` in task loops
- [ ] No driver init in `main()`
- [ ] All drivers init inside their tasks
- [ ] Motor mutex used in all motor functions
- [ ] No multicore explicit launches
- [ ] All `#include <tk/tkernel.h>` present in task files

### Documentation Checklist
- [ ] Update task documentation with driver init requirements
- [ ] Document mutex usage in motor driver comments
- [ ] Add timing constraints to task documentation
- [ ] Record any hardware-specific GPIO/I2C/ADC configurations

### Performance Checklist
- [ ] No task priority inversions
- [ ] No missed deadlines
- [ ] I2C communication robust
- [ ] ADC sampling consistent
- [ ] PWM output stable

**Final Verification:**
- [ ] [ ] Robot operates correctly with all drivers
- [ ] [ ] No crashes or hangs
- [ ] [ ] All sensors providing valid data
- [ ] [ ] Motors responsive and safe
- [ ] [ ] System stable for extended run (1+ hour)

---

## Troubleshooting Guide

### Problem: "Task crashes on startup"
**Symptoms:** Task starts then immediately exits
**Likely Cause:** Driver initialization failed (returned error)
**Solution:**
1. Add error checking in task startup
2. Add `printf()` statements before each init
3. Check hardware connections
4. Verify driver parameters (pins, I2C address, etc.)

### Problem: "Motors won't move"
**Symptoms:** Motor commands sent but no movement
**Likely Cause:** 
- Motor driver not initialized
- Mutex preventing all access
- PWM configuration issue
**Solution:**
1. Verify `motor_driver_init()` called
2. Check `motor_initialized()` returns true
3. Verify GPIO pins configured correctly
4. Check PWM frequency (should be 20 kHz max)

### Problem: "I2C hangs the system"
**Symptoms:** System freezes when reading IMU
**Likely Cause:** I2C timeout not working
**Solution:**
1. Verify 2ms timeout configured in `imu_driver.c`
2. Check I2C GPIO pins (GP2=SDA, GP3=SCL)
3. Verify pull-up resistors on I2C bus
4. Test with wire shaking (loosen connector)

### Problem: "Task misses timing deadline"
**Symptoms:** Task not completing within expected period
**Likely Cause:**
- Another higher-priority task blocking
- I2C timeout expiring on every read
- Too much message buffer traffic
**Solution:**
1. Check task priorities (adjust if needed)
2. Verify I2C slave device present and responsive
3. Reduce message buffer publish rate
4. Profile task execution time

### Problem: "Messages lost"
**Symptoms:** Message buffer overflow, messages dropped
**Likely Cause:** Tasks producing faster than consuming
**Solution:**
1. Increase message buffer size in `vehicle_messages.h`
2. Increase priority of consuming task
3. Reduce publishing frequency
4. Add flow control (back-pressure from receiver)

---

## Sign-Off Checklist

**Before Declaring Success:**

- [ ] All drivers compile with zero errors
- [ ] All tasks link successfully
- [ ] System boots and enters RTOS scheduler
- [ ] All drivers initialize without errors
- [ ] All hardware responds correctly:
  - [ ] Motors move on command
  - [ ] IMU reads accelerometer data
  - [ ] Line sensors detect black/white
  - [ ] Ultrasonic measures distance
- [ ] All tasks meet their timing deadlines
- [ ] No task crashes or hangs
- [ ] Concurrency tests pass (all tasks simultaneously)
- [ ] I2C timeout protection verified (plug/unplug IMU)
- [ ] Motor emergency stop works from any task
- [ ] Extended runtime test passes (no degradation)
- [ ] Code review complete (no RTOS violations)
- [ ] Documentation updated

**When all items checked:**
✅ **Driver porting to Micro T-Kernel is complete!**

---

## Next Steps

1. **Integration Testing:** Run with rest of vehicle system
2. **Performance Tuning:** Adjust task priorities if needed
3. **Field Testing:** Test on actual robot hardware
4. **Documentation:** Generate final technical spec
5. **Maintenance:** Monitor for timing/concurrency issues

---

## Reference Files

- **Main Porting Guide:** `DRIVER_PORTING_GUIDE.md`
- **Quick Reference:** `DRIVER_CHANGES_QUICK_REFERENCE.txt`
- **Code Example:** `DRIVER_PORTING_CODE_EXAMPLE.c`
- **This Checklist:** `DRIVER_PORTING_CHECKLIST.md`

Good luck! 🚀
