#include "motion_control_task.h"

#include "motion_control.h"
#include "motor_driver.h"
#include "vehicle_messages.h"
#include "vehicle_system.h"

#include "pico/stdlib.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define MOTION_UPDATE_PERIOD_MS       10U
#define MOTION_DEFAULT_FORWARD_SPEED  1500U
#define MOTION_DEFAULT_BACKWARD_SPEED 1400U
#define MOTION_DEFAULT_LEFT_TURN_MS   450U
#define MOTION_DEFAULT_RIGHT_TURN_MS  450U
#define MOTION_DEFAULT_PWM             30U
#define MOTION_COMMAND_MAX_AGE_MS     250U
#define MOTION_STATUS_PERIOD_MS       100U

#define MOTION_FAULT_INVALID_COMMAND  1U
#define MOTION_FAULT_START_FAILED     2U
#define MOTION_FAULT_UPDATE_FAILED    3U
#define MOTION_FAULT_TIMEOUT           4U

static uint32_t s_status_sequence;
static uint32_t s_fault_sequence;
static uint32_t s_command_deadline_ms;
static uint32_t s_last_status_ms;
static motion_status_t s_last_published_status;
static bool s_command_active;
static bool s_fault_reported;

static uint32_t now_ms(void)
{
    return to_ms_since_boot(get_absolute_time());
}

static void publish_fault(uint32_t timestamp_ms, uint16_t code)
{
    fault_message_t message;
    INT result;

    if (s_fault_reported && code == MOTION_FAULT_UPDATE_FAILED) {
        return;
    }

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

static void publish_status(uint32_t timestamp_ms)
{
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

static void publish_status_if_needed(uint32_t timestamp_ms)
{
    motion_status_t status = motion_get_status();

    if ((status != s_last_published_status) ||
        (timestamp_ms - s_last_status_ms >= MOTION_STATUS_PERIOD_MS)) {
        publish_status(timestamp_ms);
        s_last_published_status = status;
        s_last_status_ms = timestamp_ms;
    }
}

static bool command_is_fresh(const motion_command_message_t *command,
                             uint32_t timestamp_ms)
{
    uint32_t age = timestamp_ms - command->header.timestamp_ms;

    return age <= MOTION_COMMAND_MAX_AGE_MS;
}

static bool start_velocity_command(const motion_command_message_t *command)
{
    float amount;
    motion_direction_t direction;

    if (command->duration_ms == 0U) {
        return false;
    }

    if (command->linear_velocity != 0) {
        direction = (command->linear_velocity > 0) ?
                    MOTION_FORWARD : MOTION_BACKWARD;
        amount = fabsf((float)command->linear_velocity) *
                  (float)command->duration_ms / 1000.0f;
        return motion_start(direction, amount);
    }

    if (command->angular_velocity != 0) {
        direction = (command->angular_velocity > 0) ?
                    MOTION_RIGHT : MOTION_LEFT;
        amount = fabsf((float)command->angular_velocity) *
                  (float)command->duration_ms / 1000.0f;
        return motion_start(direction, amount);
    }

    return false;
}

static void stop_motion(uint32_t timestamp_ms, uint16_t fault_code)
{
    motion_emergency_stop();
    s_command_active = false;
    if (fault_code != 0U) {
        publish_fault(timestamp_ms, fault_code);
    }
}

static void handle_command(const motion_command_message_t *command,
                           uint32_t timestamp_ms)
{
    bool started = false;

    if (!command_is_fresh(command, timestamp_ms)) {
        publish_fault(timestamp_ms, MOTION_FAULT_INVALID_COMMAND);
        return;
    }

    switch (command->mode) {
        case VEHICLE_MOTION_STOP:
            motion_stop();
            s_command_active = false;
            break;
        case VEHICLE_MOTION_EMERGENCY_STOP:
            stop_motion(timestamp_ms, 0U);
            break;
        case VEHICLE_MOTION_VELOCITY:
            if (!motion_is_armed()) {
                if (!motion_arm()) {
                    publish_fault(timestamp_ms, MOTION_FAULT_START_FAILED);
                    return;
                }
            }
            started = start_velocity_command(command);
            if (!started) {
                publish_fault(timestamp_ms, MOTION_FAULT_INVALID_COMMAND);
                return;
            }
            s_command_deadline_ms = timestamp_ms + command->duration_ms;
            s_command_active = true;
            s_fault_reported = false;
            break;
        default:
            publish_fault(timestamp_ms, MOTION_FAULT_INVALID_COMMAND);
            break;
    }
}

static void receive_commands(uint32_t timestamp_ms)
{
    motion_command_message_t command;
    INT received;

    do {
        received = tk_rcv_mbf(g_mbf_motion_commands, &command, TMO_POL);
        if (received == (INT)sizeof(command)) {
            handle_command(&command, timestamp_ms);
        } else if (received < 0 && received != E_TMOUT) {
            printf("motion: command receive failed (%d)\n", (int)received);
            publish_fault(timestamp_ms, MOTION_FAULT_INVALID_COMMAND);
            return;
        }
    } while (received > 0);
}

void motion_control_task(INT stacd, void *exinf)
{
    motion_config_t configuration = {
        MOTION_DEFAULT_FORWARD_SPEED,
        MOTION_DEFAULT_BACKWARD_SPEED,
        MOTION_DEFAULT_LEFT_TURN_MS,
        MOTION_DEFAULT_RIGHT_TURN_MS,
        MOTION_DEFAULT_PWM
    };
    uint32_t timestamp_ms;

    (void)stacd;
    (void)exinf;
    s_status_sequence = 0U;
    s_fault_sequence = 0U;
    s_command_deadline_ms = 0U;
    s_command_active = false;
    s_fault_reported = false;
    s_last_status_ms = 0U;
    s_last_published_status = MOTION_ERROR;

    motion_init();
    if (!motion_configure(&configuration) || !motion_arm()) {
        publish_fault(now_ms(), MOTION_FAULT_START_FAILED);
        for (;;) {
            (void)tk_dly_tsk(MOTION_UPDATE_PERIOD_MS);
        }
    }

    for (;;) {
        timestamp_ms = now_ms();
        receive_commands(timestamp_ms);

        if (s_command_active &&
            ((int32_t)(timestamp_ms - s_command_deadline_ms) >= 0)) {
            stop_motion(timestamp_ms, MOTION_FAULT_TIMEOUT);
        } else {
            motion_update();
            if (motion_get_status() == MOTION_ERROR) {
                stop_motion(timestamp_ms, MOTION_FAULT_UPDATE_FAILED);
            }
        }

        publish_status_if_needed(timestamp_ms);
        (void)tk_dly_tsk(MOTION_UPDATE_PERIOD_MS);
    }
}
