#ifndef VEHICLE_MESSAGES_H
#define VEHICLE_MESSAGES_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Inter-task messages are copied into µT-Kernel message buffers.  Messages
 * therefore contain values only; they must not contain pointers to task-owned
 * or stack storage.
 */

#define VEHICLE_MESSAGE_VERSION            1U
#define VEHICLE_BARCODE_PAYLOAD_SIZE       32U
#define VEHICLE_TELEMETRY_PAYLOAD_SIZE     128U
#define VEHICLE_REMOTE_ARGUMENT_SIZE       32U

typedef enum {
    VEHICLE_MESSAGE_LINE_OBSERVATION = 1,
    VEHICLE_MESSAGE_BARCODE_EVENT,
    VEHICLE_MESSAGE_IMU_ESTIMATE,
    VEHICLE_MESSAGE_OBSTACLE_EVENT,
    VEHICLE_MESSAGE_OBSTACLE_COMMAND,
    VEHICLE_MESSAGE_WHEEL_ODOMETRY,
    VEHICLE_MESSAGE_MOTION_COMMAND,
    VEHICLE_MESSAGE_MOTION_STATUS,
    VEHICLE_MESSAGE_REMOTE_COMMAND,
    VEHICLE_MESSAGE_NETWORK_STATUS,
    VEHICLE_MESSAGE_TELEMETRY,
    VEHICLE_MESSAGE_FAULT
} vehicle_message_type_t;

typedef enum {
    VEHICLE_BUFFER_LINE_OBSERVATIONS = 1,
    VEHICLE_BUFFER_BARCODE_EVENTS,
    VEHICLE_BUFFER_IMU_ESTIMATES,
    VEHICLE_BUFFER_OBSTACLE_EVENTS,
    VEHICLE_BUFFER_OBSTACLE_COMMANDS,
    VEHICLE_BUFFER_LINE_ODOMETRY,
    VEHICLE_BUFFER_IMU_ODOMETRY,
    VEHICLE_BUFFER_MOTION_COMMANDS,
    VEHICLE_BUFFER_MOTION_STATUS,
    VEHICLE_BUFFER_REMOTE_COMMANDS,
    VEHICLE_BUFFER_NETWORK_STATUS,
    VEHICLE_BUFFER_TELEMETRY,
    VEHICLE_BUFFER_FAULTS
} vehicle_buffer_id_t;

typedef enum {
    VEHICLE_BUFFER_KEEP_LATEST = 1,
    VEHICLE_BUFFER_KEEP_ORDERED,
    VEHICLE_BUFFER_PRIORITY
} vehicle_buffer_policy_t;

typedef enum {
    VEHICLE_MOTION_STOP = 0,
    VEHICLE_MOTION_VELOCITY,
    VEHICLE_MOTION_TURN,
    VEHICLE_MOTION_EMERGENCY_STOP
} vehicle_motion_mode_t;

typedef enum {
    VEHICLE_FAULT_NONE = 0,
    VEHICLE_FAULT_SENSOR,
    VEHICLE_FAULT_MOTION,
    VEHICLE_FAULT_COMMUNICATION,
    VEHICLE_FAULT_SYSTEM
} vehicle_fault_source_t;

typedef enum {
    VEHICLE_OBSTACLE_SCAN_REQUEST = 1,
    VEHICLE_OBSTACLE_BYPASS_COMPLETE,
    VEHICLE_OBSTACLE_ABORT,
    VEHICLE_OBSTACLE_RESET
} vehicle_obstacle_command_t;

#define VEHICLE_FAULT_CODE_IMU_I2C 1U

typedef struct {
    uint16_t type;
    uint16_t version;
    uint32_t sequence;
    uint32_t timestamp_ms;
} vehicle_message_header_t;

typedef struct {
    vehicle_message_header_t header;
    int16_t lateral_error;
    uint16_t confidence;
    uint8_t line_detected;
    uint8_t intersection_detected;
} line_observation_message_t;

typedef struct {
    vehicle_message_header_t header;
    char payload[VEHICLE_BARCODE_PAYLOAD_SIZE];
    uint8_t length;
    uint8_t valid;
} barcode_event_message_t;

typedef struct {
    vehicle_message_header_t header;
    int32_t roll_mdeg;
    int32_t pitch_mdeg;
    int32_t yaw_mdeg;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    uint16_t validity;
} imu_estimate_message_t;

typedef struct {
    vehicle_message_header_t header;
    uint16_t distance_mm;
    uint8_t obstacle_class;
    uint8_t emergency;
} obstacle_event_message_t;

typedef struct {
    vehicle_message_header_t header;
    uint8_t command;
} obstacle_command_message_t;

typedef struct {
    vehicle_message_header_t header;
    int32_t left_count;
    int32_t right_count;
    int32_t distance_mm;
    uint8_t valid;
} wheel_odometry_message_t;

typedef struct {
    vehicle_message_header_t header;
    int16_t linear_velocity;
    int16_t angular_velocity;
    uint16_t duration_ms;
    uint8_t mode;
    uint8_t stop_reason;
} motion_command_message_t;

typedef struct {
    vehicle_message_header_t header;
    int16_t motor_1_pwm;
    int16_t motor_2_pwm;
    uint8_t active;
    uint8_t faulted;
    uint16_t fault_code;
} motion_status_message_t;

typedef struct {
    vehicle_message_header_t header;
    uint8_t command;
    int32_t value;
    char argument[VEHICLE_REMOTE_ARGUMENT_SIZE];
} remote_command_message_t;

typedef struct {
    vehicle_message_header_t header;
    uint8_t connected;
    uint8_t mqtt_ready;
    uint8_t retry_count;
    uint8_t error_code;
} network_status_message_t;

typedef struct {
    vehicle_message_header_t header;
    uint8_t event_type;
    uint8_t priority;
    uint16_t payload_length;
    char payload[VEHICLE_TELEMETRY_PAYLOAD_SIZE];
} telemetry_message_t;

typedef struct {
    vehicle_message_header_t header;
    uint8_t source;
    uint8_t recoverable;
    uint16_t code;
} fault_message_t;

/*
 * These limits describe the initial queue contract.  They are not µT-Kernel
 * object IDs; the task startup module will create the actual message buffers.
 */
#define VEHICLE_MB_LINE_CAPACITY       4U
#define VEHICLE_MB_BARCODE_CAPACITY    8U
#define VEHICLE_MB_IMU_CAPACITY        2U
#define VEHICLE_MB_OBSTACLE_CAPACITY   4U
#define VEHICLE_MB_OBSTACLE_COMMAND_CAPACITY 4U
#define VEHICLE_MB_ODOMETRY_CAPACITY   2U
#define VEHICLE_MB_MOTION_CAPACITY     2U
#define VEHICLE_MB_STATUS_CAPACITY     2U
#define VEHICLE_MB_REMOTE_CAPACITY     4U
#define VEHICLE_MB_NETWORK_CAPACITY    2U
#define VEHICLE_MB_TELEMETRY_CAPACITY  8U
#define VEHICLE_MB_FAULT_CAPACITY      4U

#endif
