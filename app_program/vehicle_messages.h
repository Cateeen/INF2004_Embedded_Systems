#ifndef VEHICLE_MESSAGES_H
#define VEHICLE_MESSAGES_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Inter-task messages are copied into µT-Kernel message buffers. Messages
 * therefore contain values only; they must not contain pointers to task-owned
 * or stack storage.
 */

#define VEHICLE_MESSAGE_VERSION            1U
#define VEHICLE_BARCODE_PAYLOAD_SIZE       32U
#define VEHICLE_TELEMETRY_PAYLOAD_SIZE     128U
#define VEHICLE_REMOTE_ARGUMENT_SIZE       32U
#define VEHICLE_MOTION_STATUS_PERIOD_MS  100U
/* A clear validity bit means the corresponding fields are not measurements. */
#define VEHICLE_IMU_VALID_ROLL_PITCH       0x0001U
#define VEHICLE_IMU_VALID_YAW              0x0002U
#define VEHICLE_IMU_VALID_GYRO             0x0004U

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
    VEHICLE_BUFFER_FAULTS,
    VEHICLE_BUFFER_MOTION_ODOMETRY
} vehicle_buffer_id_t;

typedef enum {
    VEHICLE_BUFFER_KEEP_LATEST = 1,
    VEHICLE_BUFFER_KEEP_ORDERED,
    VEHICLE_BUFFER_PRIORITY
} vehicle_buffer_policy_t;

typedef enum {
    VEHICLE_MOTION_STOP = 0,
    VEHICLE_MOTION_ARM,
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

typedef enum {
    VEHICLE_OBSTACLE_AHEAD_READING = 1,
    VEHICLE_OBSTACLE_SCAN_CLEAR,
    VEHICLE_OBSTACLE_SCAN_FOUND
} vehicle_obstacle_event_class_t;

typedef enum {
    VEHICLE_OBSTACLE_ADVICE_NONE = 0,
    VEHICLE_OBSTACLE_ADVICE_LEFT,
    VEHICLE_OBSTACLE_ADVICE_RIGHT,
    VEHICLE_OBSTACLE_ADVICE_REVERSE
} vehicle_obstacle_advice_t;

/* Side of the robot centreline the scanned obstacle mainly lies on. */
typedef enum {
    VEHICLE_OBSTACLE_SIDE_UNKNOWN = 0,
    VEHICLE_OBSTACLE_SIDE_CENTRE,
    VEHICLE_OBSTACLE_SIDE_LEFT,
    VEHICLE_OBSTACLE_SIDE_RIGHT
} vehicle_obstacle_side_t;

/* obstacle_event_message_t.edges_seen bits. width_mm is a measurement only
 * when both bits are set; otherwise it is a lower bound or meaningless. */
#define VEHICLE_OBSTACLE_EDGE_LEFT_SEEN    0x01U
#define VEHICLE_OBSTACLE_EDGE_RIGHT_SEEN   0x02U

#define VEHICLE_FAULT_CODE_IMU_I2C 1U
#define VEHICLE_FAULT_CODE_ODOMETRY_SOURCE 2U
#define VEHICLE_FAULT_CODE_ODOMETRY_CALCULATION 3U
#define VEHICLE_FAULT_CODE_ODOMETRY_DELIVERY 4U
#define VEHICLE_FAULT_CODE_ODOMETRY_CLOCK 5U
#define VEHICLE_FAULT_CODE_ODOMETRY_SCHEDULE 6U
#define VEHICLE_FAULT_CODE_LINE_SENSOR 7U
#define VEHICLE_FAULT_CODE_LINE_ODOMETRY 8U
#define VEHICLE_FAULT_CODE_LINE_BARCODE 9U
#define VEHICLE_FAULT_CODE_LINE_DELIVERY 10U
#define VEHICLE_FAULT_CODE_LINE_CLOCK 11U
#define VEHICLE_FAULT_CODE_LINE_SCHEDULE 12U
#define VEHICLE_FAULT_CODE_IMU_SENSOR 13U
#define VEHICLE_FAULT_CODE_IMU_ODOMETRY 14U
#define VEHICLE_FAULT_CODE_IMU_PROCESSING 15U
#define VEHICLE_FAULT_CODE_IMU_DELIVERY 16U
#define VEHICLE_FAULT_CODE_IMU_CLOCK 17U
#define VEHICLE_FAULT_CODE_IMU_SCHEDULE 18U
#define VEHICLE_FAULT_CODE_OBSTACLE_SENSOR 19U
#define VEHICLE_FAULT_CODE_OBSTACLE_COMMAND 20U
#define VEHICLE_FAULT_CODE_OBSTACLE_DELIVERY 21U
#define VEHICLE_FAULT_CODE_OBSTACLE_CLOCK 22U
#define VEHICLE_FAULT_CODE_OBSTACLE_SCHEDULE 23U
#define VEHICLE_FAULT_CODE_OBSTACLE_SCAN_TIMEOUT 24U
#define VEHICLE_FAULT_CODE_MOTION_COMMAND 25U
#define VEHICLE_FAULT_CODE_MOTION_ODOMETRY 26U
#define VEHICLE_FAULT_CODE_MOTION_CONTROL 27U
#define VEHICLE_FAULT_CODE_MOTION_CLOCK 28U
#define VEHICLE_FAULT_CODE_MOTION_SCHEDULE 29U
#define VEHICLE_FAULT_CODE_MOTION_EMERGENCY 30U
#define VEHICLE_FAULT_CODE_MOTION_ACTUATOR 36U
#define VEHICLE_FAULT_CODE_MOTION_STATUS 37U
#define VEHICLE_FAULT_CODE_WIFI_BACKEND 31U
#define VEHICLE_FAULT_CODE_WIFI_DELIVERY 32U
#define VEHICLE_FAULT_CODE_WIFI_TELEMETRY 33U
#define VEHICLE_FAULT_CODE_WIFI_CLOCK 34U
#define VEHICLE_FAULT_CODE_WIFI_SCHEDULE 35U

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
    /* 1 while all three IR sensors have read black (no floor) for
     * LINE_PICKUP_MS: the robot has been lifted. Appended. */
    uint8_t picked_up;
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
    uint16_t terrain_telemetry;
    uint8_t shock_detected;
    uint8_t tilt_detected;
} imu_estimate_message_t;

typedef struct {
    vehicle_message_header_t header;
    /* Minimum reported range, or 0x0fff when all measurements report none. */
    uint16_t distance_mm;
    uint8_t obstacle_class;
    uint8_t emergency;
    uint16_t target_distance_mm;
    uint16_t width_mm;
    uint16_t left_mm;
    uint16_t right_mm;
    uint16_t lateral_mm;
    uint16_t advance_mm;
    uint8_t advisory_action;
    /* SCAN_FOUND only (0 otherwise). Appended: earlier fields are unchanged. */
    uint8_t object_side;      /* vehicle_obstacle_side_t */
    uint8_t edges_seen;       /* VEHICLE_OBSTACLE_EDGE_* bits */
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
    /* Signed commanded motor PWM, not measured motion or proof of a stop. */
    int16_t motor_1_pwm;
    int16_t motor_2_pwm;
    /* active is accepted command state; ARM acceptance is not reported. */
    uint8_t active;
    uint8_t faulted;
    uint16_t fault_code;
    /* 1 while the motors are lent (motion_task_borrow_motors): motor_*_pwm
     * are then the borrower's outputs. Appended. */
    uint8_t lent;
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
 * These limits describe the initial queue contract. They are not µT-Kernel
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
