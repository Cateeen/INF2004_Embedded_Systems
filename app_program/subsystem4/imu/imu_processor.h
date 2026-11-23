/* 
Declarations for the signal processing state machine logic
Separate math and logic from hardware 
*/

#ifndef IMU_PROCESSOR_H
#define IMU_PROCESSOR_H

#include "imu_types.h"

/* Each state is owned by one IMU task. The caller must reject failed I2C
 * samples and supply monotonic, wrapping milliseconds and odometry metres. */
bool imu_processor_init(imu_state_t *state);

/* Treat this reading (ideally an average taken while the robot is still on
 * level ground) as level: its pitch and roll become offsets subtracted from
 * every later sample. Rejects readings more than IMU_LEVEL_MAX_DEG from flat
 * or with az below OVERTURN_MIN_AZ. Call after init; init clears the offsets. */
bool imu_processor_set_level(imu_state_t *state, const accel_data_t *level);

/* Startup level calibration from IMU_LEVEL_SAMPLES still readings. Reset,
 * add one sample per period, then apply once imu_level_calibrator_full(). */
typedef struct {
    float    sum[3];
    float    min[3];
    float    max[3];
    uint16_t count;
} imu_level_calibrator_t;

typedef enum {
    IMU_LEVEL_OK = 0,
    IMU_LEVEL_INCOMPLETE,  /* fewer than IMU_LEVEL_SAMPLES readings */
    IMU_LEVEL_MOVING,      /* an axis varied more than IMU_LEVEL_MAX_SPREAD_MS2 */
    IMU_LEVEL_NOT_LEVEL    /* average rejected by imu_processor_set_level() */
} imu_level_result_t;

void imu_level_calibrator_reset(imu_level_calibrator_t *calibrator);
/* False for a NULL or non-finite sample, which is not added. */
bool imu_level_calibrator_add(imu_level_calibrator_t *calibrator, const accel_data_t *sample);
bool imu_level_calibrator_full(const imu_level_calibrator_t *calibrator);
/* Applies the averaged reading with imu_processor_set_level(); on any result
 * other than IMU_LEVEL_OK the state is unchanged. */
imu_level_result_t imu_processor_apply_level(imu_state_t *state,
                                             const imu_level_calibrator_t *calibrator);

/* Initialise for the robot: terrain filter IMU_TERRAIN_FILTER_ALPHA and mount
 * yaw IMU_MOUNT_YAW_DEG, no level offsets yet. */
bool imu_processor_init_vehicle(imu_state_t *state);

/* Undo the IMU board's rotation about the vertical axis relative to the
 * chassis (|yaw_deg| <= IMU_MOUNT_YAW_MAX_DEG), applied after the level
 * offsets to every angle, including tilt detection. 0 (the init default)
 * leaves the angles unchanged. Measure it with a still robot resting one
 * rear wheel on a block: see IMU_MOUNT_YAW_DEG. */
bool imu_processor_set_mount_yaw(imu_state_t *state, float yaw_deg);

/* Exponential smoothing of the pitch and roll used for hump detection,
 * alpha in (0, 1]. 1 (the init default) disables it. Tilt detection always
 * uses unfiltered angles. */
bool imu_processor_set_terrain_filter(imu_state_t *state, float alpha);

/* Hump height model: the robot rests on two rear wheels (IMU_TRACK_WIDTH_M
 * apart) and a front-centre ball (IMU_WHEELBASE_M ahead of the rear axle).
 * The filtered pitch and roll give each contact's height relative to the
 * others; the lowest contact is taken as the ground. The hump height is the
 * highest contact, so a bump under either wheel, both wheels or the ball is
 * measured without odometry. current_distance_m is kept for the API and
 * recorded at hump start. */

bool imu_processor_update(imu_state_t *state, const accel_data_t *accel,
                          float current_distance_m, uint32_t now_ms);

#endif