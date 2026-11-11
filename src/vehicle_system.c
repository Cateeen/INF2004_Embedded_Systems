#include "vehicle_system.h"
#include "vehicle_gpio_dispatcher.h"

#include "vehicle_messages.h"
#include "vehicle_tasks.h"

#include <stddef.h>
#include <stdint.h>

#define VEHICLE_TASK_MESSAGE_BUFFER_ATTRIBUTE  TA_TFIFO

ID g_mbf_line_observations;
ID g_mbf_barcode_events;
ID g_mbf_imu_estimates;
ID g_mbf_obstacle_events;
ID g_mbf_obstacle_commands;
ID g_mbf_line_odometry;
ID g_mbf_imu_odometry;
ID g_mbf_motion_commands;
ID g_mbf_motion_status;
ID g_mbf_remote_commands;
ID g_mbf_network_status;
ID g_mbf_telemetry;
ID g_mbf_faults;


static uint8_t s_line_observation_storage[
    VEHICLE_MB_LINE_CAPACITY * sizeof(line_observation_message_t)];
static uint8_t s_barcode_event_storage[
    VEHICLE_MB_BARCODE_CAPACITY * sizeof(barcode_event_message_t)];
static uint8_t s_imu_estimate_storage[
    VEHICLE_MB_IMU_CAPACITY * sizeof(imu_estimate_message_t)];
static uint8_t s_obstacle_event_storage[
    VEHICLE_MB_OBSTACLE_CAPACITY * sizeof(obstacle_event_message_t)];
static uint8_t s_obstacle_command_storage[
    VEHICLE_MB_OBSTACLE_COMMAND_CAPACITY * sizeof(obstacle_command_message_t)];
static uint8_t s_line_odometry_storage[
    VEHICLE_MB_ODOMETRY_CAPACITY * sizeof(wheel_odometry_message_t)];
static uint8_t s_imu_odometry_storage[
    VEHICLE_MB_ODOMETRY_CAPACITY * sizeof(wheel_odometry_message_t)];
static uint8_t s_motion_command_storage[
    VEHICLE_MB_MOTION_CAPACITY * sizeof(motion_command_message_t)];
static uint8_t s_motion_status_storage[
    VEHICLE_MB_STATUS_CAPACITY * sizeof(motion_status_message_t)];
static uint8_t s_remote_command_storage[
    VEHICLE_MB_REMOTE_CAPACITY * sizeof(remote_command_message_t)];
static uint8_t s_network_status_storage[
    VEHICLE_MB_NETWORK_CAPACITY * sizeof(network_status_message_t)];
static uint8_t s_telemetry_storage[
    VEHICLE_MB_TELEMETRY_CAPACITY * sizeof(telemetry_message_t)];
static uint8_t s_fault_storage[
    VEHICLE_MB_FAULT_CAPACITY * sizeof(fault_message_t)];

typedef struct {
    ID *id;
    void *storage;
    INT storage_size;
    INT maximum_message_size;
} message_buffer_definition_t;

static ER create_message_buffer(const message_buffer_definition_t *definition)
{
    T_CMBF configuration;

    if ((definition == NULL) ||
        (definition->id == NULL) ||
        (definition->storage == NULL) ||
        (definition->storage_size <= 0) ||
        (definition->maximum_message_size <= 0)) {
        return E_PAR;
    }

    configuration.exinf = NULL;
    configuration.mbfatr = VEHICLE_TASK_MESSAGE_BUFFER_ATTRIBUTE;
    configuration.bufsz = definition->storage_size;
    configuration.maxmsz = definition->maximum_message_size;
    configuration.mbf = definition->storage;

    *(definition->id) = tk_cre_mbf(&configuration);
    if (*(definition->id) < 0) {
        return *(definition->id);
    }

    return E_OK;
}

static ER create_all_message_buffers(void)
{
    static const message_buffer_definition_t definitions[] = {
        { &g_mbf_line_observations, s_line_observation_storage,
          (INT)sizeof(s_line_observation_storage),
          (INT)sizeof(line_observation_message_t) },
        { &g_mbf_barcode_events, s_barcode_event_storage,
          (INT)sizeof(s_barcode_event_storage),
          (INT)sizeof(barcode_event_message_t) },
        { &g_mbf_imu_estimates, s_imu_estimate_storage,
          (INT)sizeof(s_imu_estimate_storage),
          (INT)sizeof(imu_estimate_message_t) },
        { &g_mbf_obstacle_events, s_obstacle_event_storage,
          (INT)sizeof(s_obstacle_event_storage),
          (INT)sizeof(obstacle_event_message_t) },
        { &g_mbf_obstacle_commands, s_obstacle_command_storage,
          (INT)sizeof(s_obstacle_command_storage),
          (INT)sizeof(obstacle_command_message_t) },
        { &g_mbf_line_odometry, s_line_odometry_storage,
          (INT)sizeof(s_line_odometry_storage),
          (INT)sizeof(wheel_odometry_message_t) },
        { &g_mbf_imu_odometry, s_imu_odometry_storage,
          (INT)sizeof(s_imu_odometry_storage),
          (INT)sizeof(wheel_odometry_message_t) },
        { &g_mbf_motion_commands, s_motion_command_storage,
          (INT)sizeof(s_motion_command_storage),
          (INT)sizeof(motion_command_message_t) },
        { &g_mbf_motion_status, s_motion_status_storage,
          (INT)sizeof(s_motion_status_storage),
          (INT)sizeof(motion_status_message_t) },
        { &g_mbf_remote_commands, s_remote_command_storage,
          (INT)sizeof(s_remote_command_storage),
          (INT)sizeof(remote_command_message_t) },
        { &g_mbf_network_status, s_network_status_storage,
          (INT)sizeof(s_network_status_storage),
          (INT)sizeof(network_status_message_t) },
        { &g_mbf_telemetry, s_telemetry_storage,
          (INT)sizeof(s_telemetry_storage),
          (INT)sizeof(telemetry_message_t) },
        { &g_mbf_faults, s_fault_storage,
          (INT)sizeof(s_fault_storage),
          (INT)sizeof(fault_message_t) }
    };
    UINT index;
    ER error;

    for (index = 0U; index < (UINT)(sizeof(definitions) / sizeof(definitions[0]));
         index++) {
        error = create_message_buffer(&definitions[index]);
        if (error != E_OK) {
            return error;
        }
    }

    return E_OK;
}

/*
 * The task-startup module remains a separate owner so task creation policy
 * cannot be mixed with message-buffer storage.
 */
void vehicle_system_run(void)
{
    ER error = create_all_message_buffers();

    if (error != E_OK) {
        for (;;) {
            (void)tk_dly_tsk(1000);
        }
    }

    vehicle_gpio_dispatcher_init();
    error = vehicle_tasks_create_and_start();
    if (error != E_OK) {
        for (;;) {
            (void)tk_dly_tsk(1000);
        }
    }

    for (;;) {
        (void)tk_dly_tsk(1000);
    }
}
