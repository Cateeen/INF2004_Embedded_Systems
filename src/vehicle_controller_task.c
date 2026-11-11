#include "vehicle_controller_task.h"

#include "obstacle_detection_task.h"
#include "telemetry_protocol.h"
#include "vehicle_messages.h"
#include "vehicle_system.h"

#include "pico/stdlib.h"

#include <stdio.h>
#include <string.h>

#define CONTROLLER_PERIOD_MS          20U
#define CONTROLLER_COMMAND_MS         100U
#define CONTROLLER_TELEMETRY_MS       200U
#define CONTROLLER_OBSTACLE_STOP_MM   120U

typedef enum {
    CONTROLLER_IDLE = 0,
    CONTROLLER_CALIBRATING,
    CONTROLLER_NAVIGATING,
    CONTROLLER_PROCESSING,
    CONTROLLER_COMPLETED,
    CONTROLLER_FAULT
} controller_state_t;

typedef struct {
    line_observation_message_t line;
    imu_estimate_message_t imu;
    obstacle_event_message_t obstacle;
    barcode_event_message_t barcode;
    motion_status_message_t motion;
    network_status_message_t network;
    bool line_valid;
    bool imu_valid;
    bool obstacle_valid;
    bool barcode_valid;
    bool motion_valid;
    bool network_valid;
} controller_snapshot_t;

static controller_state_t s_state;
static controller_snapshot_t s_snapshot;
static uint32_t s_command_sequence;
static uint32_t s_telemetry_sequence;
static uint32_t s_last_telemetry_ms;
static uint32_t s_last_remote_sequence;
static uint32_t s_last_obstacle_sequence;
static uint32_t s_last_line_sequence;
static bool s_have_remote_sequence;
static bool s_fault_latched;

static uint32_t now_ms(void)
{
    return to_ms_since_boot(get_absolute_time());
}

static bool receive_line(void)
{
    line_observation_message_t message;
    INT received = tk_rcv_mbf(g_mbf_line_observations, &message, TMO_POL);

    if (received == (INT)sizeof(message)) {
        s_snapshot.line = message;
        s_snapshot.line_valid = true;
        return true;
    }
    return false;
}

static void receive_latest_sensor_values(void)
{
    while (receive_line()) {
    }
}

static void receive_latest_imu(void)
{
    imu_estimate_message_t message;
    INT received;

    do {
        received = tk_rcv_mbf(g_mbf_imu_estimates, &message, TMO_POL);
        if (received == (INT)sizeof(message)) {
            s_snapshot.imu = message;
            s_snapshot.imu_valid = message.validity != 0U;
        }
    } while (received > 0);
}

static void receive_latest_obstacle(void)
{
    obstacle_event_message_t message;
    INT received;

    do {
        received = tk_rcv_mbf(g_mbf_obstacle_events, &message, TMO_POL);
        if (received == (INT)sizeof(message)) {
            s_snapshot.obstacle = message;
            s_snapshot.obstacle_valid = true;
        }
    } while (received > 0);
}

static void receive_latest_barcode(void)
{
    barcode_event_message_t message;
    INT received;

    do {
        received = tk_rcv_mbf(g_mbf_barcode_events, &message, TMO_POL);
        if (received == (INT)sizeof(message) && message.valid != 0U) {
            s_snapshot.barcode = message;
            s_snapshot.barcode_valid = true;
        }
    } while (received > 0);
}

static void receive_latest_motion_status(void)
{
    motion_status_message_t message;
    INT received;

    do {
        received = tk_rcv_mbf(g_mbf_motion_status, &message, TMO_POL);
        if (received == (INT)sizeof(message)) {
            s_snapshot.motion = message;
            s_snapshot.motion_valid = true;
        }
    } while (received > 0);
}

static void receive_latest_network_status(void)
{
    network_status_message_t message;
    INT received;

    do {
        received = tk_rcv_mbf(g_mbf_network_status, &message, TMO_POL);
        if (received == (INT)sizeof(message)) {
            s_snapshot.network = message;
            s_snapshot.network_valid = true;
        }
    } while (received > 0);
}

static void publish_motion_command(uint8_t mode, int16_t linear_velocity,
                                   int16_t angular_velocity, uint16_t duration,
                                   uint8_t stop_reason, uint32_t timestamp_ms)
{
    motion_command_message_t message;
    INT result;

    memset(&message, 0, sizeof(message));
    message.header.type = VEHICLE_MESSAGE_MOTION_COMMAND;
    message.header.version = VEHICLE_MESSAGE_VERSION;
    message.header.sequence = ++s_command_sequence;
    message.header.timestamp_ms = timestamp_ms;
    message.linear_velocity = linear_velocity;
    message.angular_velocity = angular_velocity;
    message.duration_ms = duration;
    message.mode = mode;
    message.stop_reason = stop_reason;

    result = tk_snd_mbf(g_mbf_motion_commands, &message, (INT)sizeof(message),
                        TMO_POL);
    if (result != E_OK) {
        printf("controller: motion command send failed (%d)\n", (int)result);
    }
}

static void publish_telemetry(uint32_t timestamp_ms)
{
    telemetry_message_t message;
    int length;
    uint8_t line_state = LINE_UNKNOWN;
    uint8_t obstacle_state = OBSTACLE_UNKNOWN;
    uint8_t vehicle_state = VEHICLE_IDLE;
    INT result;

    if (s_snapshot.line_valid) {
        line_state = s_snapshot.line.line_detected ? LINE_FOUND : LINE_LOST;
    }
    if (s_snapshot.obstacle_valid) {
        obstacle_state = s_snapshot.obstacle.obstacle_class ?
                         OBSTACLE_DETECTED : OBSTACLE_CLEAR;
    }
    if (s_state == CONTROLLER_NAVIGATING) {
        vehicle_state = VEHICLE_RUNNING;
    } else if (s_state == CONTROLLER_FAULT) {
        vehicle_state = VEHICLE_FAULT;
    } else if (s_state == CONTROLLER_COMPLETED) {
        vehicle_state = VEHICLE_COMPLETE;
    } else if (s_state == CONTROLLER_PROCESSING) {
        vehicle_state = VEHICLE_PAUSED;
    }

    memset(&message, 0, sizeof(message));
    message.header.type = VEHICLE_MESSAGE_TELEMETRY;
    message.header.version = VEHICLE_MESSAGE_VERSION;
    message.header.sequence = ++s_telemetry_sequence;
    message.header.timestamp_ms = timestamp_ms;
    message.event_type = MSG_TELEMETRY;
    message.priority = (s_state == CONTROLLER_FAULT) ? 3U : 1U;
    length = snprintf(message.payload, sizeof(message.payload),
                      "state=%u,line=%u,obstacle=%u,imu=%u,motion=%u,wifi=%u,mqtt=%u",
                      (unsigned)vehicle_state, (unsigned)line_state,
                      (unsigned)obstacle_state, s_snapshot.imu_valid ? 1U : 0U,
                      s_snapshot.motion_valid && s_snapshot.motion.active,
                      s_snapshot.network_valid && s_snapshot.network.connected,
                      s_snapshot.network_valid && s_snapshot.network.mqtt_ready);
    if (length < 0) {
        return;
    }
    message.payload_length = (length >= (int)sizeof(message.payload)) ?
                             (uint16_t)(sizeof(message.payload) - 1U) :
                             (uint16_t)length;

    result = tk_snd_mbf(g_mbf_telemetry, &message, (INT)sizeof(message),
                        TMO_POL);
    if (result != E_OK) {
        printf("controller: telemetry send failed (%d)\n", (int)result);
    }
    s_last_telemetry_ms = timestamp_ms;
}

static bool remote_is_new(const remote_command_message_t *message)
{
    if (!s_have_remote_sequence ||
        message->header.sequence != s_last_remote_sequence) {
        s_last_remote_sequence = message->header.sequence;
        s_have_remote_sequence = true;
        return true;
    }
    return false;
}

static void handle_remote_command(const remote_command_message_t *message,
                                  uint32_t timestamp_ms)
{
    if (!remote_is_new(message)) {
        return;
    }

    switch (message->command) {
        case CMD_START:
        case CMD_RESUME:
            if (s_state == CONTROLLER_IDLE || s_state == CONTROLLER_PROCESSING) {
                s_state = CONTROLLER_NAVIGATING;
            }
            break;
        case CMD_PAUSE:
            if (s_state == CONTROLLER_NAVIGATING) {
                s_state = CONTROLLER_PROCESSING;
                publish_motion_command(VEHICLE_MOTION_STOP, 0, 0, 100U,
                                       CMD_PAUSE, timestamp_ms);
            }
            break;
        case CMD_STOP:
            s_state = CONTROLLER_IDLE;
            publish_motion_command(VEHICLE_MOTION_STOP, 0, 0, 100U,
                                   CMD_STOP, timestamp_ms);
            break;
        case CMD_JUNCTION:
            if (s_state == CONTROLLER_NAVIGATING) {
                s_state = CONTROLLER_PROCESSING;
            }
            break;
        case CMD_REQUEST_STATUS:
            publish_telemetry(timestamp_ms);
            break;
        default:
            break;
    }
}

static void receive_remote_commands(uint32_t timestamp_ms)
{
    remote_command_message_t message;
    INT received;

    do {
        received = tk_rcv_mbf(g_mbf_remote_commands, &message, TMO_POL);
        if (received == (INT)sizeof(message)) {
            handle_remote_command(&message, timestamp_ms);
        }
    } while (received > 0);
}

static void receive_faults(uint32_t timestamp_ms)
{
    fault_message_t message;
    INT received;

    do {
        received = tk_rcv_mbf(g_mbf_faults, &message, TMO_POL);
        if (received == (INT)sizeof(message) &&
            message.source != VEHICLE_FAULT_SYSTEM &&
            (message.recoverable == 0U ||
             message.source == VEHICLE_FAULT_SENSOR ||
             message.source == VEHICLE_FAULT_MOTION)) {
            s_fault_latched = true;
            s_state = CONTROLLER_FAULT;
            publish_motion_command(VEHICLE_MOTION_EMERGENCY_STOP, 0, 0, 100U,
                                   message.code, timestamp_ms);
        }
    } while (received > 0);
}

static void handle_obstacle(uint32_t timestamp_ms)
{
    if (!s_snapshot.obstacle_valid) {
        return;
    }
    if (s_snapshot.obstacle.header.sequence == s_last_obstacle_sequence) {
        return;
    }
    s_last_obstacle_sequence = s_snapshot.obstacle.header.sequence;

    if (s_snapshot.obstacle.emergency ||
        s_snapshot.obstacle.distance_mm <= CONTROLLER_OBSTACLE_STOP_MM) {
        s_state = CONTROLLER_PROCESSING;
        publish_motion_command(VEHICLE_MOTION_EMERGENCY_STOP, 0, 0, 100U,
                               1U, timestamp_ms);
        return;
    }
    if (s_snapshot.obstacle.obstacle_class &&
        s_state == CONTROLLER_NAVIGATING) {
        s_state = CONTROLLER_PROCESSING;
        publish_motion_command(VEHICLE_MOTION_STOP, 0, 0, 100U, 2U,
                               timestamp_ms);
        (void)obstacle_detection_request_scan();
    }
}

static void handle_line_and_barcode(void)
{
    if (s_state == CONTROLLER_NAVIGATING &&
        s_snapshot.line_valid &&
        s_snapshot.line.header.sequence != s_last_line_sequence) {
        s_last_line_sequence = s_snapshot.line.header.sequence;
        if (s_snapshot.line.intersection_detected) {
            s_state = CONTROLLER_PROCESSING;
        }
    }
    if (s_state == CONTROLLER_PROCESSING && s_snapshot.barcode_valid) {
        s_snapshot.barcode_valid = false;
        s_state = CONTROLLER_NAVIGATING;
    }
}

static void issue_line_command(uint32_t timestamp_ms)
{
    int16_t angular_velocity = 0;

    if (!s_snapshot.line_valid || !s_snapshot.line.line_detected) {
        publish_motion_command(VEHICLE_MOTION_STOP, 0, 0,
                               CONTROLLER_COMMAND_MS, 3U, timestamp_ms);
        return;
    }
    if (s_snapshot.line.lateral_error < 0) {
        angular_velocity = -30;
    } else if (s_snapshot.line.lateral_error > 0) {
        angular_velocity = 30;
    }
    publish_motion_command(VEHICLE_MOTION_VELOCITY, 15, angular_velocity,
                           CONTROLLER_COMMAND_MS, 0U, timestamp_ms);
}

void vehicle_controller_task(INT stacd, void *exinf)
{
    uint32_t timestamp_ms;

    (void)stacd;
    (void)exinf;
    memset(&s_snapshot, 0, sizeof(s_snapshot));
    s_state = CONTROLLER_IDLE;
    s_command_sequence = 0U;
    s_telemetry_sequence = 0U;
    s_last_telemetry_ms = 0U;
    s_have_remote_sequence = false;
    s_last_obstacle_sequence = 0U;
    s_last_line_sequence = 0U;
    s_fault_latched = false;

    for (;;) {
        timestamp_ms = now_ms();
        receive_latest_sensor_values();
        receive_latest_imu();
        receive_latest_obstacle();
        receive_latest_barcode();
        receive_latest_motion_status();
        receive_latest_network_status();
        receive_remote_commands(timestamp_ms);
        receive_faults(timestamp_ms);

        if (!s_fault_latched) {
            handle_obstacle(timestamp_ms);
            handle_line_and_barcode();
            if (s_state == CONTROLLER_NAVIGATING &&
                (!s_snapshot.motion_valid || !s_snapshot.motion.active)) {
                issue_line_command(timestamp_ms);
            }
        }

        if (timestamp_ms - s_last_telemetry_ms >= CONTROLLER_TELEMETRY_MS) {
            publish_telemetry(timestamp_ms);
        }
        (void)tk_dly_tsk(CONTROLLER_PERIOD_MS);
    }
}
