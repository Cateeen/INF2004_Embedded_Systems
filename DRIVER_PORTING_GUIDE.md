# Device Driver Porting Guide to Micro T-Kernel

## Overview
When porting the existing bare-metal device drivers to the micro T-Kernel architecture, the **hardware control logic stays the same**, but the **threading model, initialization, and timing** change significantly.

---

## Critical Changes Required

### 1. **Driver Initialization Timing**
**Bare-Metal (Current):**
```c
int main(void) {
    stdio_init_all();
    motor_driver_init();      // Called before motor control task
    imu_driver_init();
    multicore_launch_core1(vc_core);
}
```

**Micro T-Kernel (Change to):**
- Drivers must be initialized **within their respective tasks**, not in `main()`
- Multiple tasks may need the same driver → use a **one-time initialization pattern** with a static flag
- Example from your existing code (see `imu_task.c:101`):
  ```c
  void imu_task(INT stacd, void *exinf) {
      imu_driver_init();      // Called INSIDE the task
      imu_processor_init(&state);
      
      for (;;) {
          imu_driver_read_accel(&accel);
          tk_dly_tsk(IMU_SAMPLE_PERIOD_MS);  // Task-based delay
      }
  }
  ```

**Action Items:**
- ✅ Move `motor_driver_init()` → inside `motion_control_task()`
- ✅ Move `imu_driver_init()` → inside `imu_task()` (already done)
- ✅ Move `line_following_init()` → inside line barcode task
- ✅ Move `obs_port_init()` → inside obstacle detection task
- ✅ Update `vehicle_system.c` to remove bare-metal initialization calls

---

### 2. **Blocking vs. Task-Safe Operations**

**Problem:**
Current drivers use Pico SDK blocking calls like:
```c
imu_driver_read_accel(accel_data_t *out) {
    if (!lsm_read_regs(LSM303_ACCEL_ADDR, ...)) {  // Blocks in I2C
        out->i2c_ok = false;
        return;
    }
}
```

**Micro T-Kernel Solution:**
- I2C timeouts are **already built into Pico SDK** (`i2c_write_timeout_us()`, `i2c_read_timeout_us()`)
- These are **safe** because I2C operations are synchronous and complete within their timeout
- **No change needed** to I2C driver blocking behavior ✅

**PWM Operations:**
- Motor PWM writes are **non-blocking** in both models ✅
- No changes needed

**ADC Operations:**
- Line-following ADC reads are **non-blocking** ✅
- No changes needed

**Changes Required:**
- Replace `sleep_ms()` with `tk_dly_tsk()` for task-based delays
- Avoid calling Pico SDK's `sleep_ms()` in RTOS context

---

### 3. **Replace All `sleep_ms()` Calls**

**Before:**
```c
motor_driver_init() {
    sleep_ms(50);  // ❌ Blocks the entire Pico
}

imu_driver_init() {
    sleep_ms(10);
    // ...
    sleep_ms(50);
}
```

**After:**
```c
// In motor_driver.c, imu_driver.c, etc.
// Option 1: Delay during init (acceptable if called in main boot sequence)
// Option 2: Use tk_dly_tsk() if called from a task context

// RECOMMENDATION: Keep sleep_ms() in init functions if they're called once during boot
// but replace with tk_dly_tsk() in periodic task loops
```

**Action Items:**
- ✅ `motor_driver_init()`: Keep `sleep_ms()` (called once during boot)
- ✅ `imu_driver_init()`: Keep `sleep_ms()` (called once during boot)
- ✅ In task loops: Replace `sleep_ms()` with `tk_dly_tsk(milliseconds)`

**Example Fix:**
```c
// OLD - In motion_control_task.c
sleep_ms(MOTION_UPDATE_PERIOD_MS);

// NEW - In motion_control_task.c
tk_dly_tsk(MOTION_UPDATE_PERIOD_MS);  // Microkernel-safe delay
```

---

### 4. **Hardware Resource Protection**

**Problem:**
Current bare-metal code uses global static variables without protection:
```c
static bool emergency_stopped = true;
static motor_state_t motor_states[2] = {0};

void motor_set_speed(motor_id_t motor, int16_t speed_percent) {
    // ❌ No protection if another task calls this
}
```

**Micro T-Kernel Solution:**

For **motor driver**, add mutex protection:
```c
// In motor_driver.c
static ID s_motor_mutex = NULL;

void motor_driver_init_mutex(void) {
    T_CMTX config = {0};
    config.mtxatr = TA_TFIFO;
    s_motor_mutex = tk_cre_mtx(&config);
}

void motor_set_speed(motor_id_t motor, int16_t speed_percent) {
    if (s_motor_mutex != NULL) {
        tk_loc_mtx(s_motor_mutex, TMO_FEVR);  // Lock
    }
    
    // Update motor state...
    
    if (s_motor_mutex != NULL) {
        tk_unl_mtx(s_motor_mutex);  // Unlock
    }
}
```

**For I2C and ADC:**
- These are accessed from task contexts only
- No cross-task contention expected
- **No mutex needed** ✅

**Action Items:**
- ⚠️ Add mutex protection to `motor_driver.c` functions
- ✅ No changes needed for `imu_driver.c`
- ✅ No changes needed for `line_following.c`

---

### 5. **Multicore Architecture**

**Bare-Metal (Current):**
```c
/* vc_core - core 1. Motors on this core only. */
static void vc_core(void) {
    vc_task();  // Motor control runs on core 1
}

multicore_launch_core1(vc_core);
```

**Micro T-Kernel (Change to):**
- Microkernel scheduler automatically distributes tasks across cores
- Remove explicit `multicore_launch_core1()` calls
- Use task priorities instead of core assignment
- Example:
  ```c
  // In vehicle_tasks.c
  #define MOTION_TASK_PRIORITY  3   // Will run on appropriate core
  #define IMU_TASK_PRIORITY     6   // Scheduler decides
  ```

**Action Items:**
- ✅ Remove `multicore_launch_core1()` from bare-metal code
- ✅ Use only task-based scheduling via priorities
- ✅ No changes to driver code needed (stays single-core agnostic)

---

### 6. **Inter-Task Communication**

**Bare-Metal (Current):**
Uses direct function calls and global queues:
```c
// In car_observer.c
obs_core() {
    obs_init();
    car_observer_init();
    for (;;) {
        obs_step();
        car_observer_poll();
    }
}
```

**Micro T-Kernel (Already Implemented):**
Uses message buffers:
```c
// In imu_task.c
tk_rcv_mbf(g_mbf_imu_odometry, &odometry, TMO_POL);
tk_snd_mbf(g_mbf_imu_estimates, &message, sizeof(message), TMO_POL);
```

**Action Items:**
- ✅ Drivers remain isolated (no message buffer knowledge)
- ✅ Tasks wrap driver calls and publish messages
- ✅ No changes to driver code needed

---

## Implementation Checklist

### Phase 1: Minimal Changes (Keep Core Drivers)
- [ ] Copy `motor_driver.h/c`, `imu_driver.h/c`, `line_following.c`, `encoder.h/c`
- [ ] Copy `vehicle_gpio_dispatcher.h/c` (GPIO interrupt handling)
- [ ] No modifications to driver code at this stage

### Phase 2: Task Integration
- [ ] Update `motion_control_task.c`:
  - [ ] Call `motor_driver_init()` at task start
  - [ ] Replace `sleep_ms()` with `tk_dly_tsk()` in main loop
  - [ ] Handle motor emergency stop via global `g_mbf_faults` message buffer
- [ ] Verify `imu_task.c` initialization (already done)
- [ ] Update line barcode task:
  - [ ] Call `line_following_init()` at task start
  - [ ] Replace `sleep_ms()` calls
- [ ] Update obstacle detection task:
  - [ ] Call `obs_port_init()` at task start

### Phase 3: Hardware Protection
- [ ] Add mutex to `motor_driver.c` for thread-safe state access
- [ ] Test motor emergency stop handling across tasks

### Phase 4: Timing & Scheduling
- [ ] Verify I2C timeouts are still effective (2ms) under RTOS load
- [ ] Tune task priorities if needed
- [ ] Test with real hardware under load

---

## Code Changes Summary

### motor_driver.c
```diff
  void motor_driver_init(void) {
-     sleep_ms(50);     // OK to keep (boot-time only)
+     // Add mutex creation if accessed from multiple tasks
  }

  void motor_set_speed(motor_id_t motor, int16_t speed_percent) {
+     // Lock mutex before accessing shared state
+     tk_loc_mtx(s_motor_mutex, TMO_FEVR);
      // ... existing code ...
+     tk_unl_mtx(s_motor_mutex);
  }
```

### imu_driver.c
```diff
  // ✅ No changes needed - already boot-time only
  void imu_driver_init(void) {
      i2c_init(IMU_I2C_PORT, IMU_I2C_BAUDRATE);
      sleep_ms(10);  // OK - boot-time
      lsm_write_reg(...);
      sleep_ms(50);  // OK - boot-time
  }
```

### motion_control_task.c
```diff
  void motion_control_task(INT stacd, void *exinf) {
      motion_init();
      motor_driver_init();      // Add: Initialize driver
      
      for (;;) {
          // ... process commands ...
-         sleep_ms(MOTION_UPDATE_PERIOD_MS);
+         tk_dly_tsk(MOTION_UPDATE_PERIOD_MS);
      }
  }
```

### vehicle_system.c
```diff
  void vehicle_system_run(void) {
-     motor_driver_init();     // ❌ Remove
-     imu_driver_init();       // ❌ Remove
-     line_following_init();   // ❌ Remove
      
      // Create message buffers...
      create_all_message_buffers();
      
      // Create tasks (which call their own driver inits)...
      create_all_tasks();
      
      // Start scheduler
      tk_sta_tsk(g_tsk_motion_control, 0);
      tk_sta_tsk(g_tsk_imu, 0);
      // ... etc ...
  }
```

---

## Testing Strategy

1. **Driver-level tests** (bare-metal, existing):
   - Use `hardware_test/` directory tests (no changes needed)
   - Verify PWM, I2C, ADC still work in test harness

2. **Task integration tests**:
   - Start motion task → verify motors initialize
   - Start IMU task → verify I2C sensor reads
   - Verify no initialization race conditions

3. **Concurrency tests**:
   - Launch multiple tasks simultaneously
   - Verify motor mutex prevents simultaneous writes
   - Verify no I2C/ADC conflicts

4. **Timing validation**:
   - Measure task execution times
   - Verify 2ms I2C timeout is still effective
   - Check for deadline misses (IMU 50ms, motion 10ms, etc.)

---

## Summary

| Aspect | Bare-Metal | Micro T-Kernel | Action |
|--------|-----------|-----------------|--------|
| **Driver Code** | Bare Pico SDK | Same Pico SDK | ✅ No changes |
| **Init Location** | `main()` | Inside each task | ⚠️ **Move init calls** |
| **I2C Blocking** | OK (no preemption) | Still OK (timeout-safe) | ✅ No changes |
| **PWM Control** | Non-blocking | Non-blocking | ✅ No changes |
| **Task Delays** | `sleep_ms()` | `tk_dly_tsk()` | ⚠️ **Replace in loops** |
| **Hardware Mutex** | Not needed | Recommended (motors) | ⚠️ **Add to motor_driver** |
| **Multicore Launch** | Explicit `multicore_launch_core1()` | Scheduler-managed | ✅ Remove boot code |

**Estimated Effort:**
- Driver code reuse: **90%** (minimal changes)
- Boilerplate updates: **~2-3 hours** for task wrappers
- Testing & validation: **~1-2 hours**

---

## References

- [Micro T-Kernel Documentation](docs/tk_api.txt)
- [Task Synchronization](docs/task_messaging.txt)
- [RP2040 Hardware Abstraction](https://github.com/raspberrypi/pico-sdk)
- Existing implementation: `vehicle_system.c`, `imu_task.c`, `motion_control_task.c`
