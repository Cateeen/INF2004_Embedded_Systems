# Device Driver Porting to Micro T-Kernel - START HERE 🚀

## Quick Summary

You have **4 device drivers** that need to be ported from bare-metal Pico SDK to Micro T-Kernel:

| Driver | Hardware | Current | RTOS Status |
|--------|----------|---------|-------------|
| [motor_driver.c](motor_driver.c) | PWM motors (GP8-11) | ✅ Works | ⚠️ **ADD MUTEX** |
| [imu_driver.c](imu_driver.c) | I2C sensor (GP2,3) | ✅ Works | ✅ Unchanged |
| [line_following.c](line_following.c) | ADC sensors | ✅ Works | ✅ Unchanged |
| [encoder.c](encoder.c) | Pure logic | ✅ Works | ✅ Unchanged |

**Good news:** 90% of driver code stays the same. Only the **task integration** changes.

---

## 4 Critical Changes You MUST Make

### 1️⃣ Move Driver Initialization to Tasks
```c
// ❌ OLD (bare-metal):
int main(void) {
    motor_driver_init();  // ← Wrong place
}

// ✅ NEW (RTOS):
void motion_control_task(INT stacd, void *exinf) {
    motor_driver_init();  // ← Correct place (called by task)
}
```

### 2️⃣ Replace `sleep_ms()` with `tk_dly_tsk()`
```c
// ❌ OLD:
sleep_ms(10);  // Blocks entire CPU

// ✅ NEW:
tk_dly_tsk(10);  // Yields to scheduler, other tasks run
```

### 3️⃣ Add Mutex to Motor Driver
```c
// ⚠️ NEW: Protect shared motor state
tk_loc_mtx(s_motor_mutex, TMO_FEVR);  // Lock
motor_set_speed(MOTOR_1, 50);         // Protected access
tk_unl_mtx(s_motor_mutex);            // Unlock
```

### 4️⃣ Remove Bare-Metal Boot Code
```c
// ❌ DELETE from main():
motor_driver_init();
multicore_launch_core1(vc_core);

// ✅ Let RTOS scheduler handle it
// (Each task initializes its own drivers)
```

---

## Which Files Need Changes?

### ✅ **NO CHANGES NEEDED** (Just copy):
- [motor_driver.c](motor_driver.c)
- [imu_driver.c](imu_driver.c) 
- [line_following.c](line_following.c)
- [encoder.c](encoder.c)
- [vehicle_gpio_dispatcher.c](vehicle_gpio_dispatcher.c)

### ⚠️ **NEED CHANGES** (20 minutes each):
- [motor_driver.c](motor_driver.c) → Add mutex (1 hour)
- [motion_control_task.c](integration/vehicle_ctrl.c) → Init + sleep replacement
- [imu_task.c](src/imu_task.c) → Replace sleep_ms()
- [line_barcode_task.c](src/line_barcode_task.c) → Init + sleep replacement
- [obstacle_detection_task.c](src/obstacle_detection_task.c) → Init + sleep replacement
- [vehicle_system.c](src/vehicle_system.c) → Remove driver init calls

---

## Documentation Provided (Read in Order)

### 1. **This File** (you are here) - 2 min read
   Quick overview and checklist

### 2. **[DRIVER_PORTING_GUIDE.md](DRIVER_PORTING_GUIDE.md)** - 15 min read
   Comprehensive explanation with:
   - Why each change is needed
   - Detailed examples
   - Testing strategy

### 3. **[DRIVER_PORTING_CODE_EXAMPLE.c](DRIVER_PORTING_CODE_EXAMPLE.c)** - 20 min study
   Real code with before/after comments showing:
   - Exact changes needed
   - Mutex pattern usage
   - Boot code differences

### 4. **[DRIVER_CHANGES_QUICK_REFERENCE.txt](DRIVER_CHANGES_QUICK_REFERENCE.txt)** - Quick lookup
   ASCII diagrams and checklists:
   - What to change and why
   - Top 5 mistakes to avoid
   - Testing steps

### 5. **[DRIVER_PORTING_CHECKLIST.md](DRIVER_PORTING_CHECKLIST.md)** - Step-by-step
   Detailed 10-phase checklist:
   - Every single task to complete
   - Troubleshooting guide
   - Sign-off criteria

---

## Quick Start (30 minutes)

### Step 1: Understand the Problem (5 min)
1. Read the **Summary** section above ✅
2. Look at [DRIVER_CHANGES_QUICK_REFERENCE.txt](DRIVER_CHANGES_QUICK_REFERENCE.txt)
   - Focus on the "CRITICAL CHANGES" box

### Step 2: See the Solution (10 min)
1. Open [DRIVER_PORTING_CODE_EXAMPLE.c](DRIVER_PORTING_CODE_EXAMPLE.c)
2. Find section "CHANGE #3d: Replace sleep_ms()" 
3. Read the motion_control_task() function carefully
4. Note the three main additions:
   - `motor_driver_init()` call
   - `motor_driver_mutex_init()` call
   - `tk_dly_tsk()` replacement

### Step 3: Plan Your Changes (5 min)
1. Open [DRIVER_PORTING_CHECKLIST.md](DRIVER_PORTING_CHECKLIST.md)
2. Read Phase 3 (Motor Mutex)
3. Read Phase 4 (Task Integration) - just the overview
4. Create a checklist in your project

### Step 4: Start Coding (10 min)
1. Start with **motion_control_task.c** (simplest)
2. Add `motor_driver_init()` call at task startup
3. Find and replace first `sleep_ms()` with `tk_dly_tsk()`
4. Compile and verify no errors
5. Then move to other tasks

---

## Implementation Order (Do This)

### Phase 1: Preparation (15 min)
- [ ] Copy all driver files to new project
- [ ] Copy vehicle_system.c framework
- [ ] Verify compilation (should have missing init errors - that's OK)

### Phase 2: Motor Mutex (30 min)
- [ ] Add mutex variable to motor_driver.c
- [ ] Add `motor_driver_mutex_init()` function
- [ ] Add `motor_initialized()` function
- [ ] Wrap each motor function with lock/unlock

### Phase 3: Motion Control Task (20 min)
- [ ] Add `motor_driver_init()` call at startup
- [ ] Add `motor_driver_mutex_init()` call
- [ ] Find all `sleep_ms()` in main loop
- [ ] Replace with `tk_dly_tsk()`

### Phase 4: Other Tasks (20 min each)
- [ ] IMU task: Replace `sleep_ms()`
- [ ] Line barcode task: Add init + replace sleep
- [ ] Obstacle task: Add init + replace sleep

### Phase 5: Boot Code (10 min)
- [ ] Remove all `motor_driver_init()` calls from main
- [ ] Remove `multicore_launch_core1()` calls
- [ ] Keep message buffer creation

### Phase 6: Test (30 min)
- [ ] Compile: `make` (should succeed)
- [ ] Link: Check for errors
- [ ] Run: Verify no crashes
- [ ] Motors: Test movement
- [ ] Sensors: Test I2C/ADC reads

---

## Common Mistakes (Don't Do These!)

| ❌ Mistake | ✅ Solution |
|-----------|-----------|
| Call `motor_driver_init()` in main | Call inside `motion_control_task()` |
| Use `sleep_ms()` in task loops | Use `tk_dly_tsk()` instead |
| No mutex on motor state | Add `tk_loc_mtx()` / `tk_unl_mtx()` |
| Keep `multicore_launch_core1()` | Remove it, use RTOS scheduling |
| Initialize driver every loop iteration | Initialize once in startup, use static flag |
| Forget to add `#include <tk/tkernel.h>` | Add to all task files |

---

## Testing Checklist

**After completing each phase:**

```
Phase 1: Compilation
  [ ] No compiler errors
  [ ] No linker errors (might have undefined refs - OK for now)
  [ ] Binary size < 200KB

Phase 2: Motor Mutex
  [ ] motor_driver.c compiles
  [ ] motor_driver_mutex_init() compiles
  [ ] No undefined references to tk_cre_mtx

Phase 3: Motion Task
  [ ] motion_control_task.c compiles
  [ ] motor_driver_init() called in task
  [ ] All sleep_ms() replaced with tk_dly_tsk()
  [ ] Task starts without errors
  [ ] Motors respond to commands

Phase 4: All Tasks
  [ ] All tasks compile
  [ ] All tasks start without errors
  [ ] All drivers initialize successfully
  [ ] Sensors reading data
  [ ] Motors moving

Phase 5: Boot Code
  [ ] vehicle_system.c compiles
  [ ] car_main.c doesn't call driver inits
  [ ] RTOS scheduler starts
  [ ] Tasks initialize their own drivers

Phase 6: Full System
  [ ] 60-minute runtime test passes
  [ ] No crashes or hangs
  [ ] All sensors stable
  [ ] Motors responsive
  [ ] Message buffers working
```

---

## If Something Goes Wrong

### "Task crashes on startup"
→ Check: Did you call driver init before motion_control_init()?
→ Check: Are error messages printed? (Add printf before each init)

### "Motors won't move"
→ Check: Is motor_driver_init() called at task startup?
→ Check: Is mutex preventing all access? (Test without mutex first)
→ Check: Are GPIO pins configured? (Check motor_driver.c pin definitions)

### "I2C freezes the system"
→ Check: Are 2ms timeouts configured? (They are in imu_driver.c)
→ Check: Are I2C pull-ups connected? (GP2 and GP3)
→ Check: Is slave device (IMU) responding? (Probe with oscilloscope)

### "sleep_ms still used somewhere"
```bash
grep -r "sleep_ms" src/
# Should show nothing in task code
# OK if in driver init functions (called once)
```

### "Compilation error: undefined reference to 'tk_cre_mtx'"
→ Add `-ltkernel` to CMakeLists.txt linking

---

## Success Criteria (You're Done When)

✅ All these are true:
- Drivers compile with zero errors
- Tasks link without undefined references
- System boots and enters RTOS scheduler
- Motors initialize and respond to commands
- IMU reads accelerometer data
- Line sensors detect black/white
- All tasks meet timing deadlines (10ms motion, 50ms IMU, etc.)
- Concurrent access to motors doesn't crash
- I2C timeout protects against sensor disconnects
- Motor emergency stop works from any task
- System stable for 1+ hour continuous run

When all pass: **You're done! 🎉**

---

## Next Steps

1. **Now:** Read [DRIVER_PORTING_CODE_EXAMPLE.c](DRIVER_PORTING_CODE_EXAMPLE.c) (~20 min)
2. **Then:** Use [DRIVER_PORTING_CHECKLIST.md](DRIVER_PORTING_CHECKLIST.md) for step-by-step
3. **Reference:** Keep [DRIVER_CHANGES_QUICK_REFERENCE.txt](DRIVER_CHANGES_QUICK_REFERENCE.txt) open while coding
4. **Deep Dive:** Read [DRIVER_PORTING_GUIDE.md](DRIVER_PORTING_GUIDE.md) if you need rationale

---

## Key Insight

> The drivers don't know about the RTOS. They remain pure, portable Pico SDK code.
> The RTOS integration happens at the **task wrapper level**, not the driver level.
> This keeps drivers reusable in both bare-metal AND RTOS contexts. ✨

---

## Support

If stuck:
1. Check [DRIVER_PORTING_CHECKLIST.md](DRIVER_PORTING_CHECKLIST.md) → "Troubleshooting Guide"
2. Find your error in the table
3. Follow the suggested solution
4. If still stuck, check the specific phase documentation

---

**Estimated Total Time: 2-4 hours depending on familiarity with RTOS**

Good luck! 🚀
