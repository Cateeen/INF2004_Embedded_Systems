/*
 * EXAMPLE: Porting motion_control_task.c to Micro T-Kernel
 * This demonstrates the actual code changes needed
 */

#include "motion_control_task.h"
#include "motion_control.h"
#include "motor_driver.h"
#include "vehicle_messages.h"
#include "vehicle_system.h"
#include <tk/tkernel.h>              /* ← RTOS includes */
#include "pico/stdlib.h"
#include <stdio.h>
#include <string.h>

#define MOTION_UPDATE_PERIOD_MS       10U
#define MOTION_STATUS_PERIOD_MS       100U

static uint32_t s_status_sequence;
static uint32_t s_fault_sequence;
static uint32_t s_last_status_ms;
static motion_status_t s_last_published_status;
static bool s_command_active;
static bool s_fault_reported;

/* ═════════════════════════════════════════════════════════════════════════════
   CHANGE #1: Helper for current time in milliseconds
   
   Previously used: get_absolute_time() / to_ms_since_boot()
   These still work, but we can also use RTOS time services if needed
   ═════════════════════════════════════════════════════════════════════════════ */
static uint32_t now_ms(void) {
    return to_ms_since_boot(get_absolute_time());  /* ✅ Still works */
}

/* ═════════════════════════════════════════════════════════════════════════════
   CHANGE #2: Existing message publish functions stay the same
   ═════════════════════════════════════════════════════════════════════════════ */
static void publish_fault(uint32_t timestamp_ms, uint16_t code) {
    fault_message_t message;
    INT result;

    memset(&message, 0, sizeof(message));
    message.header.type = VEHICLE_MESSAGE_FAULT;
    message.header.version = VEHICLE_MESSAGE_VERSION;
    message.header.sequence = ++s_fault_sequence;
    message.header.timestamp_ms = timestamp_ms;
    message.source = VEHICLE_FAULT_MOTION;
    message.recoverable = 0U;
    message.code = code;

    result = tk_snd_mbf(g_mbf_faults, &message, (INT)sizeof(message),
                        TMO_POL);
    if (result != E_OK) {
        printf("motion: fault message send failed (%d)\n", (int)result);
    }
    s_fault_reported = true;
}

static void publish_status(uint32_t timestamp_ms) {
    motion_status_message_t message;
    motion_status_t status = motion_get_status();
    INT result;

    memset(&message, 0, sizeof(message));
    message.header.type = VEHICLE_MESSAGE_MOTION_STATUS;
    message.header.version = VEHICLE_MESSAGE_VERSION;
    message.header.sequence = ++s_status_sequence;
    message.header.timestamp_ms = timestamp_ms;
    message.active = motion_is_busy() ? 1U : 0U;
    message.faulted = (status == MOTION_ERROR) ? 1U : 0U;
    message.fault_code = (uint16_t)status;

    result = tk_snd_mbf(g_mbf_motion_status, &message, (INT)sizeof(message),
                        TMO_POL);
    if (result != E_OK) {
        printf("motion: status message send failed (%d)\n", (int)result);
    }
}

/* ═════════════════════════════════════════════════════════════════════════════
   CHANGE #3: THE MAIN TASK ENTRY POINT
   
   This is where initialization moves, and sleep_ms() gets replaced
   ═════════════════════════════════════════════════════════════════════════════ */

/*
 * BEFORE (Bare-Metal) - for reference:
 * 
 * void motion_init(void) {
 *     printf("Motion control initializing...\n");
 *     motor_driver_init();        ← Initialized in bare-metal boot
 *     motion_control_init();
 *     
 *     for (;;) {
 *         // ... process commands ...
 *         sleep_ms(MOTION_UPDATE_PERIOD_MS);  ← WRONG for RTOS
 *     }
 * }
 */

/*
 * AFTER (Micro T-Kernel) - actual implementation:
 */
void motion_control_task(INT stacd, void *exinf) {
    motion_command_message_t command;
    motion_status_t status;
    uint32_t timestamp_ms;
    INT result;
    ER error;

    (void)stacd;      /* Unused RTOS parameters */
    (void)exinf;

    /* ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ */
    /* INITIALIZATION SECTION - Called once at task startup                   */
    /* ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ */
    
    printf("Motion control task starting...\n");
    
    /* ⚠️ CHANGE #3a: Call motor_driver_init() HERE, not in main() */
    motor_driver_init();
    if (!motor_initialized()) {
        printf("motion: ERROR - motor driver init failed\n");
        publish_fault(now_ms(), MOTION_FAULT_START_FAILED);
        return;  /* Exit task on critical failure */
    }
    
    /* ⚠️ CHANGE #3b: Initialize motor mutex for thread-safety */
    motor_driver_mutex_init();
    
    /* ⚠️ CHANGE #3c: Motion control initialization (unchanged) */
    error = motion_control_init();
    if (error != E_OK) {
        printf("motion: ERROR - motion_control_init failed (%d)\n", (int)error);
        publish_fault(now_ms(), MOTION_FAULT_START_FAILED);
        return;
    }

    /* Initialize task state */
    s_status_sequence = 0U;
    s_fault_sequence = 0U;
    s_last_status_ms = now_ms();
    s_last_published_status = MOTION_OK;
    s_command_active = false;
    s_fault_reported = false;

    printf("Motion control task initialized successfully\n");

    /* ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ */
    /* MAIN TASK LOOP - Runs forever                                          */
    /* ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ */
    for (;;) {
        timestamp_ms = now_ms();

        /* Receive latest motion command (non-blocking poll) */
        result = tk_rcv_mbf(g_mbf_motion_commands, &command, 
                            (INT)sizeof(command), TMO_POL);
        
        if (result == (INT)sizeof(command)) {
            /* Process valid command */
            s_command_active = true;
            s_fault_reported = false;
            
            error = motion_process_command(&command);
            if (error != E_OK) {
                printf("motion: command processing failed (%d)\n", (int)error);
                publish_fault(timestamp_ms, MOTION_FAULT_INVALID_COMMAND);
            }
        } else if (result < 0 && result != E_TMOUT) {
            /* Error receiving command */
            printf("motion: command receive failed (%d)\n", (int)result);
            if (!s_fault_reported) {
                publish_fault(timestamp_ms, MOTION_FAULT_TIMEOUT);
            }
        }
        /* else: E_TMOUT means no message waiting, which is OK */

        /* Update motion control state */
        error = motion_control_update(timestamp_ms);
        if (error != E_OK) {
            printf("motion: control update failed (%d)\n", (int)error);
            if (!s_fault_reported) {
                publish_fault(timestamp_ms, MOTION_FAULT_UPDATE_FAILED);
            }
        }

        /* Publish status if changed or time-periodic */
        status = motion_get_status();
        if ((status != s_last_published_status) ||
            (timestamp_ms - s_last_status_ms >= MOTION_STATUS_PERIOD_MS)) {
            publish_status(timestamp_ms);
            s_last_published_status = status;
            s_last_status_ms = timestamp_ms;
        }

        /* ───────────────────────────────────────────────────────────────── */
        /* ⚠️ CRITICAL CHANGE #3d: Replace sleep_ms() with tk_dly_tsk()   */
        /* ───────────────────────────────────────────────────────────────── */
        
        /* WRONG for RTOS:
         *   sleep_ms(MOTION_UPDATE_PERIOD_MS);
         *   ^ This blocks the entire CPU, preventing other tasks from running
         */
        
        /* CORRECT for RTOS:
         *   The task delays and yields to scheduler, allowing other tasks
         *   to run. When the delay expires, this task resumes.
         */
        
        tk_dly_tsk(MOTION_UPDATE_PERIOD_MS);  /* ← Microsecond-safe delay */
        
        /* At this point, the scheduler may have run other tasks:
         * - IMU sensor reading
         * - Line following processing
         * - WiFi communication
         * - etc.
         * 
         * When MOTION_UPDATE_PERIOD_MS (10ms) expires, this task resumes.
         */
    }
    
    /* ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ */
    /* NOTE: Code below never executes (infinite for loop above)              */
    /* ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ */
    
    printf("Motion control task exiting (should not happen)\n");
    motor_emergency_stop();  /* Safety: stop motors if task exits */
}


/* ═════════════════════════════════════════════════════════════════════════════
   HELPER FUNCTIONS - Normally in motor_driver.h (ADDITIONS for RTOS)
   ═════════════════════════════════════════════════════════════════════════════ */

/*
 * These functions need to be added to motor_driver.c for thread safety:
 * 
 * static ID s_motor_mutex = NULL;
 * 
 * void motor_driver_mutex_init(void) {
 *     T_CMTX config = {0};
 *     config.mtxatr = TA_TFIFO;
 *     s_motor_mutex = tk_cre_mtx(&config);
 *     if (s_motor_mutex < 0) {
 *         printf("ERROR: failed to create motor mutex (%d)\n", 
 *                (int)s_motor_mutex);
 *     }
 * }
 * 
 * bool motor_initialized(void) {
 *     return motor_initialized;
 * }
 * 
 * // All public motor functions should acquire/release the mutex:
 * void motor_set_speed(motor_id_t motor, int16_t speed_percent) {
 *     if (s_motor_mutex != NULL) {
 *         tk_loc_mtx(s_motor_mutex, TMO_FEVR);
 *     }
 *     // ... existing motor control code ...
 *     if (s_motor_mutex != NULL) {
 *         tk_unl_mtx(s_motor_mutex);
 *     }
 * }
 */


/* ═════════════════════════════════════════════════════════════════════════════
   BOOT CODE (vehicle_system.c) - What NOT to do anymore
   ═════════════════════════════════════════════════════════════════════════════ */

/*
 * BEFORE (Bare-Metal) - car_main.c:
 * 
 * int main(void) {
 *     stdio_init_all();
 *     sleep_ms(3000);
 * 
 *     // ❌ WRONG: Initializing drivers in main()
 *     motor_driver_init();        
 *     line_following_init();
 *     car_measurements_init();
 *     
 *     // ❌ WRONG: Explicit multicore launch
 *     multicore_launch_core1(vc_core);
 *     
 *     obs_core();  // Main loop on core 0
 *     return 0;
 * }
 */

/*
 * AFTER (Micro T-Kernel) - vehicle_system_run():
 * 
 * void vehicle_system_run(void) {
 *     ER error;
 * 
 *     // ✅ Initialize message buffers (no drivers yet!)
 *     error = create_all_message_buffers();
 *     if (error != E_OK) {
 *         printf("ERROR: message buffer creation failed (%d)\n", (int)error);
 *         return;
 *     }
 * 
 *     // ✅ Initialize GPIO dispatcher (ISR-based)
 *     vehicle_gpio_dispatcher_init();
 * 
 *     // ✅ Create all tasks (but don't start them yet)
 *     error = create_all_tasks();
 *     if (error != E_OK) {
 *         printf("ERROR: task creation failed (%d)\n", (int)error);
 *         return;
 *     }
 * 
 *     // ✅ Start the RTOS scheduler
 *     // Tasks will initialize their drivers when they start
 *     tk_sta_tsk(g_tsk_motion_control, 0);
 *     tk_sta_tsk(g_tsk_imu, 0);
 *     tk_sta_tsk(g_tsk_obstacle_detection, 0);
 *     tk_sta_tsk(g_tsk_line_barcode, 0);
 *     tk_sta_tsk(g_tsk_wifi_service, 0);
 *     // ... etc
 * 
 *     // This call never returns; scheduler takes over
 *     tk_dis_dsp();  // Disable dispatcher (optional on RP2040)
 * }
 */


/* ═════════════════════════════════════════════════════════════════════════════
   SUMMARY OF CHANGES
   ═════════════════════════════════════════════════════════════════════════════ */

/*
 * ✅ What STAYS THE SAME:
 *   - motor_driver.c (hardware control logic)
 *   - motor_set_speed() calls
 *   - message buffer send/receive patterns
 *   - timestamp handling with to_ms_since_boot()
 * 
 * ⚠️ What CHANGES:
 *   - motor_driver_init()   : moved from main() to task startup
 *   - sleep_ms()            : changed to tk_dly_tsk() in task loops
 *   - Mutex protection      : added to motor_driver.c
 *   - Boot code             : removed driver init, added task creation
 *   - Multicore launch      : removed explicit launch, use RTOS scheduling
 * 
 * ❌ What BREAKS if you don't change:
 *   - Using sleep_ms() in task loops → blocks entire scheduler
 *   - Calling motor_driver_init() in main() → conflicts with task init
 *   - No mutex on motors → race condition crash
 *   - Calling multicore_launch_core1() → scheduler confusion
 * 
 * 🎯 KEY PRINCIPLE:
 *   Drivers are still pure, bare-metal code (no RTOS knowledge)
 *   The RTOS integration happens at the TASK wrapper level
 *   This keeps drivers portable and testable in standalone contexts
 */
