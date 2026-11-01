/*
* obs_cfg.h - Obstacle Scan and Avoid Task Configuration
* PLACEHOLDER values to be measured or agreed upon using the robocar.
* Angle Conventions: 0 degree = robot right, 90 degree = robot front, 180 degree = robot left
*/ 

#ifndef OBS_CFG_H
#define OBS_CFG_H

#include <stdint.h>

/* Pins */
#define OBS_PIN_ECHO    4U  /* GP4 (Grove 3), GPIO Output */
#define OBS_PIN_TRIG    5U  /* GP5 (Grove 3), GPIO Input */
#define OBS_PIN_SERVO   15U /* GP15 (Servo Port), PVM Slice 7 */

/* Scanning Geometry*/
#define OBS_ANGLE_AHEAD_DEG     100U
#define OBS_ANGLE_MIN_DEG       60U
#define OBS_ANGLE_MAX_DEG       125U
#define OBS_COARSE_COUNT        6U
#define OBS_COARSE_STEP_DEG     13U
#define OBS_FINE_STEP_DEG       5U
#define OBS_FINE_MAX_SAMPLES    14U /*(125-60)/5 + 1*/

/* Ranging (HC-SR04: 2cm.. 4m, 15 deg beam, us/58 = cm)*/
#define OBS_MIN_RANGE_MM        20U
#define OBS_MAX_RANGE_MM        1000U   /* PLACEHOLDER: Capped for now to shorten echo wait */
#define OBS_MAX_PULSE_US        ((OBS_MAX_RANGE_MM * 58U)/10U)
#define OBS_DIST_BITS           12U
#define OBS_DIST_NONE           0xFFFU /* "No objects in range"*/

/* Timings */
#define OBS_MEASURE_GAP_MS      60U /* Datasheet Suggests >60ms; PLACEHOLDER to be tuned*/
#define OBS_SERVO_MS_PER_DEG    2U  /* PLACEHOLDER approx 0.12s per 60 deg, got to test*/
#define OBS_SERVO_EXTRA_MS      20U /* PLACEHOLDER: SETTING MARGIN*/
#define OBS_RISE_TIMEOUT_MS     5U  /* ECHO must rise soon after TRIG*/
#define OBS_ECHO_TIMEOUT_MS     8U  /* > OBS_MAX_PULSE_US*/
#define OBS_ECHO_FAIL_LIMIT     3U  /* Consecutive no response -> fault */
#define OBS_IDLE_PING_MS        100U
#define OBS_SCAN_WAIT_MS        2000U /* wait for Vehicle Contoller's SCAN_REQ */
#define OBS_BYPASS_TIMEOUT_MS   4000U /* PLACEHOLDER (A grade <3s) */
#define OBS_RECOVERY_TIMEOUT_MS 5000U /* PLACEHOLDER: TO AGREE WITH BUDDY 3*/
#define OBS_LINE_POLL_MS        20U
#define OBS_LINE_DEBOUNCE       3U 
#define OBS_MAX_RESCANS         1U

/* Thresholds (PLACEHOLDER: TO be derived from d_stop and arena)*/
#define OBS_TRIGGER_MM          300U /* Forward pings, to raise a proximity alert when robot is too close to an object*/
#define OBS_BLOCK_MM            500U /* Sample reading below 500mm is "blocked" by an obstacle */
#define OBS_SIDE_RANGE_MM       600U /* Maximum distance sideways to considered when calculating side clearance */
#define OBS_ABORT_MM            120U /* Emergency safety limit in case something pop's up*/
#define OBS_CLUSTER_GAP_MM      60U  /* Used to profile different objects / obstacles */

/* Robot Geometry, in mm (PLACEHOLDER: to be measured)*/
#define OBS_ROBOT_WIDTH_MM      130U
#define OBS_ROBOT_LENGTH_MM     240U
#define OBS_SAFETY_MARGIN_MM    20U
#define OBS_DEPTH_MIN_MM        100U /* Assumed minimum obstacle depth*/
#define OBS_SEARCH_EXTRA_MM     50U

/*Width correction for the 15 deg beam */
#ifndef OBS_BEAM_TRIM_DEG
#define OBS_BEAM_TRIM_DEG       0.0f
#endif

/* Servo (PLACEHOLDER: set to servo's real endpoints)*/
#define OBS_SERVO_MIN_US        500U
#define OBS_SERVO_MAX_US        2500U
#define OBS_SERVO_REVERSED      0U

/* Packed Field Limits (see obs_pack.h)*/
#define OBS_OFFSET_MAX_MM       1023U /* 10 bit lateral / advance fields*/

#endif /* OBS_CFG_H */