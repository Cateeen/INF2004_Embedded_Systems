#ifndef IMU_PROCESSOR_CONFIG_H
#define IMU_PROCESSOR_CONFIG_H

#define GRAVITY_MS2        9.80665f
#define IMU_ACCEL_SENSITIVITY_2G_G_PER_LSB 0.00105f

#define LIFT_ROLL_DEG      15.0f // catch hand pickup; flat track stays ~0 deg
#define LIFT_PITCH_DEG     35.0f
#define OVERTURN_MIN_AZ    6.5f // catches gentle lift deceleration

#define TILT_THRESHOLD_DEG 30.0f
/* A tilt (lifted or overturned) clears once the attitude has been normal
 * for this long, so the robot can be started again after being put down
 * (the controller still needs a fresh GO press). */
#define IMU_TILT_CLEAR_MS 1000U
/* Vehicle: lifted/overturned only counts once the raw attitude has stayed
 * past the limits this long (4 samples). The check uses the unfiltered
 * reading, and a bump over a tape seam, the caster jolting or the jerk of a
 * junction spin can put one sample past them (az < 6.5, roll > 15 deg) and
 * stopped the run; a real lift lasts far longer (and pick-up is also caught
 * by the line sensors). */
#define IMU_TILT_CONFIRM_MS 200U

/* Robot geometry: two rear drive wheels and one front-centre ball caster.
 * Measured between ground contact points. */
#define IMU_TRACK_WIDTH_M  0.115f /* W: left to right rear tyre centre */
#define IMU_WHEELBASE_M    0.080f /* L: rear axle line to ball contact */

/* Hump starts when any contact is this far above the lowest one, and ends
 * once every contact is back within IMU_HUMP_END_M. Real floors vary by
 * about 1-1.5 deg (2-3 mm across the robot), so stay above that. */
#define IMU_HUMP_START_M   0.005f
/* Vehicle: a hump starts only once the height has stayed above
 * IMU_HUMP_START_M this long. The accelerometer cannot tell tilt from
 * acceleration: speeding up or braking at 1 m/s^2 reads as ~6 deg of pitch
 * (5 mm needs only 3.6 deg pitch or 2.5 deg roll on this chassis), and line
 * steering, junction stops and spins do that for a few hundred ms at a time;
 * a ramp tilts the robot for seconds. Raise it if level ground still shows
 * humps; lower it if ramp mode engages too late on the ramp. */
#define IMU_HUMP_CONFIRM_MS 400U
#define IMU_HUMP_END_M     0.003f
#define MIN_HUMP_HEIGHT_M  0.005f /* smallest peak worth reporting */

/* Rising: safety reset only. The geometric height cannot drift, so a slow
 * push over a ramp must not be cut short. Falling: time allowed after coming
 * off a hump to settle below IMU_HUMP_END_M before monitoring resumes. */
#define IMU_HUMP_TIMEOUT_MS 30000U
#define IMU_HUMP_SETTLE_MS  4000U
/* A hump only ends once every contact has stayed below IMU_HUMP_END_M this
 * long, so a brief dip on one ramp does not split it into several humps. */
#define IMU_HUMP_END_HOLD_MS 300U

/* Smoothing of the pitch/roll used for hump height, per 50 ms sample
 * (about 200 ms): a hand push or motor acceleration is not read as a slope. */
#define IMU_TERRAIN_FILTER_ALPHA 0.2f

/* Largest mounting tilt imu_processor_set_level() will accept as "level". */
#define IMU_LEVEL_MAX_DEG  20.0f
/* Startup level calibration: average this many still samples (about 1 s at
 * 50 ms) and reject them if any axis moved more than this. */
#define IMU_LEVEL_SAMPLES        20U
#define IMU_LEVEL_MAX_SPREAD_MS2 0.8f

/* IMU board rotation about vertical relative to the chassis. Measured with
 * the robot still, level-calibrated, right wheel on a 10 mm block: roll
 * 5.29 deg / pitch -3.26 deg, whose direction is 4.1 deg away from the
 * geometric atan((h/2L)/(h/W)) for W/L = 115/80. Rotating by this makes the
 * left wheel and ball read 0 and the right wheel 10.1 mm. */
#define IMU_MOUNT_YAW_DEG     (-4.1f)
#define IMU_MOUNT_YAW_MAX_DEG 45.0f

#endif
