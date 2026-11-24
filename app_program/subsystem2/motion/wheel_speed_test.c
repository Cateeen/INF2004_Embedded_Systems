#include "../../app.h"
#include "wheel_speed_test.h"

#if APP_TEST
#include "../../test_report.h"
#include "wheel_speed_control.h"
#include "motion_feedback.h"
#include "motion_control.h"

static BOOL correction_test(void)
{
	wheel_speed_control_t control;
	wheel_speed_config_t config = {2.0f, 0.0f, 0.0f, 10.0f, 20U, 60U};
	wheel_speed_output_t out;

	if (!wheel_speed_control_init(&control, &config) ||
	    !wheel_speed_control_update(&control, 10.0f, 8.0f, 0.1f, &out) ||
	    out.left_pwm_percent != 20U || out.right_pwm_percent != 24U ||
	    !wheel_speed_control_update(&control, 7.0f, 15.0f, 0.1f, &out) ||
	    out.left_pwm_percent != 26U || out.right_pwm_percent != 10U)
		return false;
	config.kp = 100.0f;
	return wheel_speed_control_init(&control, &config) &&
	       wheel_speed_control_update(&control, 0.0f, 100.0f, 0.1f, &out) &&
	       out.left_pwm_percent == 60U && out.right_pwm_percent == 0U;
}

static BOOL pid_state_test(void)
{
	wheel_speed_control_t control;
	wheel_speed_config_t config = {0.0f, 1.0f, 0.0f, 20.0f, 20U, 25U};
	wheel_speed_output_t out;

	if (!wheel_speed_control_init(&control, &config) ||
	    !wheel_speed_control_update(&control, 0.0f, 0.0f, 0.25f, &out) ||
	    out.left_pwm_percent != 25U ||
	    !wheel_speed_control_update(&control, 0.0f, 0.0f, 0.25f, &out) ||
	    out.left_pwm_percent != 25U ||
	    !wheel_speed_control_update(&control, 25.0f, 25.0f, 0.25f, &out) ||
	    out.left_pwm_percent != 24U) return false;

	config = (wheel_speed_config_t){0.0f, 0.0f, 1.0f, 10.0f, 20U, 60U};
	return wheel_speed_control_init(&control, &config) &&
	       wheel_speed_control_update(&control, 10.0f, 10.0f, 0.1f, &out) &&
	       out.left_pwm_percent == 20U &&
	       wheel_speed_control_update(&control, 9.0f, 10.0f, 0.1f, &out) &&
	       out.left_pwm_percent == 30U &&
	       wheel_speed_control_reset(&control) &&
	       wheel_speed_control_update(&control, 9.0f, 10.0f, 0.1f, &out) &&
	       out.left_pwm_percent == 20U;
}

static BOOL feedback_test(void)
{
	motion_feedback_t feedback;
	motion_feedback_config_t config = {{2500U, 20.0f}, {2500U, 20.0f}, 50U, 250U};
	motion_wheel_speeds_t speeds = {0U, 0.0f, 99.0f, 99.0f};
	wheel_odometry_message_t message = {0};
	BOOL ok;

	ok = motion_feedback_init(&feedback, &config);
	message.header.type = VEHICLE_MESSAGE_WHEEL_ODOMETRY;
	message.header.version = VEHICLE_MESSAGE_VERSION;
	message.header.sequence = 10U;
	message.header.timestamp_ms = 100U;
	message.valid = 1U;
	ok = motion_feedback_update(&feedback, &message, 100U, &speeds) ==
	     MOTION_FEEDBACK_BASELINE && speeds.left_cm_per_second == 99.0f && ok;
	message.header.sequence = 11U;
	message.header.timestamp_ms = 200U;
	message.left_count = 1250;
	message.right_count = -625;
	ok = motion_feedback_update(&feedback, &message, 210U, &speeds) ==
	     MOTION_FEEDBACK_READY &&
	     speeds.left_cm_per_second > 99.99f && speeds.left_cm_per_second < 100.01f &&
	     speeds.right_cm_per_second < -49.99f && speeds.right_cm_per_second > -50.01f &&
	     speeds.timestamp_ms == 200U && ok;
	message.header.sequence = 13U;
	message.header.timestamp_ms = 210U;
	ok = motion_feedback_update(&feedback, &message, 210U, &speeds) ==
	     MOTION_FEEDBACK_INVALID && feedback.last_sequence == 11U &&
	     speeds.timestamp_ms == 200U && ok;
	message.header.sequence = 12U;
	ok = motion_feedback_update(&feedback, &message, 209U, &speeds) ==
	     MOTION_FEEDBACK_INVALID &&
	     motion_feedback_update(&feedback, &message, 261U, &speeds) ==
	     MOTION_FEEDBACK_INVALID && ok;
	message.header.timestamp_ms = 451U;
	ok = motion_feedback_update(&feedback, &message, 451U, &speeds) ==
	     MOTION_FEEDBACK_INVALID && feedback.last_sequence == 11U && ok;
	message.valid = 0U;
	ok = motion_feedback_update(&feedback, &message, 451U, &speeds) ==
	     MOTION_FEEDBACK_INVALID && ok;

	ok = motion_feedback_init(&feedback, &config) && ok;
	message.valid = 1U;
	message.header.sequence = UINT32_MAX;
	message.header.timestamp_ms = UINT32_MAX - 50U;
	message.left_count = INT32_MAX - 1;
	message.right_count = INT32_MIN + 2;
	ok = motion_feedback_update(&feedback, &message, UINT32_MAX - 50U, &speeds) ==
	     MOTION_FEEDBACK_BASELINE && ok;
	message.header.sequence = 0U;
	message.header.timestamp_ms = 49U;
	message.left_count = INT32_MIN + 1;
	message.right_count = INT32_MAX;
	ok = motion_feedback_update(&feedback, &message, 49U, &speeds) ==
	     MOTION_FEEDBACK_READY && speeds.left_cm_per_second > 0.239f &&
	     speeds.left_cm_per_second < 0.241f && speeds.right_cm_per_second < -0.239f &&
	     speeds.right_cm_per_second > -0.241f && ok;
	message.header.sequence = 1U;
	message.header.timestamp_ms = 50U;
	message.left_count = 1;
	ok = motion_feedback_update(&feedback, &message, 50U, &speeds) ==
	     MOTION_FEEDBACK_INVALID && feedback.last_sequence == 0U && ok;
	config.right.counts_per_revolution = 0U;
	return !motion_feedback_init(&feedback, &config) && feedback.initialized && ok;
}

static BOOL invalid_test(void)
{
	wheel_speed_control_t control;
	wheel_speed_config_t config = {2.0f, 0.0f, 0.0f, 10.0f, 20U, 60U};
	wheel_speed_output_t out = {1U, 1U};

	config.max_pwm_percent = 101U;
	if (wheel_speed_control_init(&control, &config)) return false;
	config.max_pwm_percent = 60U;
	if (!wheel_speed_control_init(&control, &config) ||
	    wheel_speed_control_set_targets(&control, 0.0f, 10.0f) ||
	    wheel_speed_control_set_targets(&control, 10.0f, __builtin_huge_valf()) ||
	    control.left_target_cm_s != 10.0f || control.right_target_cm_s != 10.0f ||
	    !wheel_speed_control_update(&control, 10.0f, 10.0f, 0.1f, &out) ||
	    wheel_speed_control_update(&control, -1.0f, 10.0f, 0.1f, &out) ||
	    out.left_pwm_percent != 0U || out.right_pwm_percent != 0U ||
	    !control.left_pid.first_update || !control.right_pid.first_update ||
	    wheel_speed_control_update(&control, 10.0f, 10.0f, 0.251f, &out) ||
	    wheel_speed_control_update(&control, __builtin_huge_valf(), 10.0f, 0.1f, &out))
		return false;

	config.kp = __FLT_MAX__;
	return wheel_speed_control_init(&control, &config) &&
	       !wheel_speed_control_update(&control, 0.0f, 0.0f, 0.1f, &out) &&
	       out.left_pwm_percent == 0U && out.right_pwm_percent == 0U &&
	       control.left_pid.first_update && control.right_pid.first_update;
}

/* Per-wheel feed-forward: each wheel starts from the PWM for its own target. */
static BOOL feed_forward_test(void)
{
	wheel_speed_control_t control;
	/* Vehicle model: 22 % holds 15 cm/s, +1.4 % per cm/s. No PID action. */
	wheel_speed_config_t config = {0.0f, 0.0f, 0.0f, 12.0f, 22U, 60U, 1.4f, 15.0f};
	wheel_speed_output_t out;

	/* A 4/18 turn: 22 - 1.4 * 11 = 6.6 -> 7 %; 22 + 1.4 * 3 = 26.2 -> 26 %. */
	if (!wheel_speed_control_init(&control, &config) ||
	    !wheel_speed_control_set_targets(&control, 4.0f, 18.0f) ||
	    !wheel_speed_control_update(&control, 4.0f, 18.0f, 0.05f, &out) ||
	    out.left_pwm_percent != 7U || out.right_pwm_percent != 26U)
		return false;
	/* Low targets stay above zero with this model: 22 - 1.4 * 14.9 = 1.1 %. */
	if (!wheel_speed_control_set_targets(&control, 0.1f, 18.0f) ||
	    !wheel_speed_control_update(&control, 0.1f, 18.0f, 0.05f, &out) ||
	    out.left_pwm_percent != 1U)
		return false;
	/* Clamped to 0..max (reference 30 cm/s): 22 - 1.4 * 26 < 0; 22 + 1.4 * 30 > 60. */
	config.ff_ref_speed_cm_s = 30.0f;
	if (!wheel_speed_control_init(&control, &config) ||
	    !wheel_speed_control_set_targets(&control, 4.0f, 60.0f) ||
	    !wheel_speed_control_update(&control, 4.0f, 60.0f, 0.05f, &out) ||
	    out.left_pwm_percent != 0U || out.right_pwm_percent != 60U)
		return false;
	config.ff_ref_speed_cm_s = 15.0f;
	/* The PID corrects around it, and its limits follow it: a wheel far too
	 * slow saturates at max (60), not at base + (max - base). */
	config.kp = 100.0f;
	if (!wheel_speed_control_init(&control, &config) ||
	    !wheel_speed_control_set_targets(&control, 4.0f, 18.0f) ||
	    !wheel_speed_control_update(&control, 4.0f, 0.0f, 0.05f, &out) ||
	    out.left_pwm_percent != 7U || out.right_pwm_percent != 60U ||
	    control.right_pid.output_max > 60.0f - 26.1f ||
	    control.left_pid.output_min < -6.7f)
		return false;
	/* Zero slope is the old flat base for both wheels. */
	config.kp = 0.0f;
	config.ff_pwm_per_cm_s = 0.0f;
	if (!wheel_speed_control_init(&control, &config) ||
	    !wheel_speed_control_set_targets(&control, 4.0f, 18.0f) ||
	    !wheel_speed_control_update(&control, 4.0f, 18.0f, 0.05f, &out) ||
	    out.left_pwm_percent != 22U || out.right_pwm_percent != 22U)
		return false;
	/* Invalid: negative slope, slope without a reference, non-finite. */
	config.ff_pwm_per_cm_s = -1.0f;
	if (wheel_speed_control_init(&control, &config)) return false;
	config.ff_pwm_per_cm_s = 1.4f;
	config.ff_ref_speed_cm_s = 0.0f;
	if (wheel_speed_control_init(&control, &config)) return false;
	config.ff_ref_speed_cm_s = __builtin_huge_valf();
	return !wheel_speed_control_init(&control, &config);
}

static wheel_odometry_message_t control_odometry(uint32_t sequence, uint32_t timestamp,
						  int32_t left, int32_t right)
{
	wheel_odometry_message_t message = {0};
	message.header.type = VEHICLE_MESSAGE_WHEEL_ODOMETRY;
	message.header.version = VEHICLE_MESSAGE_VERSION;
	message.header.sequence = sequence;
	message.header.timestamp_ms = timestamp;
	message.left_count = left;
	message.right_count = right;
	message.valid = 1U;
	return message;
}

static BOOL control_flow_test(void)
{
	motion_control_t control = {0};
	motion_control_config_t config = {
		{{2500U, 20.0f}, {2500U, 20.0f}, 150U, 250U},
		{2.0f, 0.0f, 0.0f, 10.0f, 20U, 60U}
	};
	wheel_odometry_message_t message = control_odometry(UINT32_MAX, UINT32_MAX - 50U, 0, 0);
	wheel_speed_output_t output = {99U, 99U};
	BOOL ok;

	ok = motion_control_start(&control, &config, 1, UINT32_MAX - 50U);
	ok = motion_control_step(&control, &message, UINT32_MAX - 50U, &output) ==
	     MOTION_CONTROL_WAITING && output.left_pwm_percent == 0U &&
	     output.right_pwm_percent == 0U && ok;
	message = control_odometry(0U, 49U, 125, 100);
	ok = motion_control_step(&control, &message, 49U, &output) == MOTION_CONTROL_READY &&
	     output.left_pwm_percent == 20U && output.right_pwm_percent == 24U && ok;
	ok = motion_control_step(&control, NULL, 50U, &output) == MOTION_CONTROL_READY &&
	     output.left_pwm_percent == 20U && output.right_pwm_percent == 24U && ok;
	motion_control_stop(&control);
	ok = !control.running && !control.have_output &&
	     control.output.left_pwm_percent == 0U && control.speed.left_pid.first_update && ok;

	ok = motion_control_start(&control, &config, -1, 200U) && ok;
	message = control_odometry(1U, 200U, 0, 0);
	ok = motion_control_step(&control, &message, 200U, &output) ==
	     MOTION_CONTROL_WAITING && ok;
	message = control_odometry(2U, 300U, -125, -125);
	ok = motion_control_step(&control, &message, 300U, &output) == MOTION_CONTROL_READY &&
	     output.left_pwm_percent == 20U && output.right_pwm_percent == 20U && ok;
	ok = motion_control_step(&control, NULL, 450U, &output) == MOTION_CONTROL_READY && ok;
	ok = motion_control_step(&control, NULL, 451U, &output) == MOTION_CONTROL_FAULT &&
	     output.left_pwm_percent == 0U && output.right_pwm_percent == 0U && ok;
	return ok;
}

static BOOL control_fault_test(void)
{
	motion_control_t control = {0};
	motion_control_config_t config = {
		{{2500U, 20.0f}, {2500U, 20.0f}, 150U, 250U},
		{2.0f, 0.0f, 0.0f, 10.0f, 20U, 60U}
	};
	wheel_odometry_message_t message;
	wheel_speed_output_t output;
	BOOL ok;

	ok = motion_control_start(&control, &config, 1, 100U);
	message = control_odometry(1U, 100U, 0, 0);
	ok = motion_control_step(&control, &message, 100U, &output) ==
	     MOTION_CONTROL_WAITING && ok;
	message = control_odometry(2U, 200U, -1, 1);
	ok = motion_control_step(&control, &message, 200U, &output) == MOTION_CONTROL_FAULT &&
	     control.faulted && !control.running && output.left_pwm_percent == 0U &&
	     output.right_pwm_percent == 0U && control.speed.left_pid.first_update && ok;
	message = control_odometry(3U, 201U, 0, 0);
	ok = motion_control_step(&control, &message, 201U, &output) == MOTION_CONTROL_FAULT && ok;

	ok = motion_control_start(&control, &config, 1, 300U) && ok;
	message = control_odometry(1U, 299U, 0, 0);
	ok = motion_control_step(&control, &message, 300U, &output) == MOTION_CONTROL_FAULT && ok;
	ok = motion_control_start(&control, &config, 1, 400U) && ok;
	message = control_odometry(1U, 400U, 0, 0);
	ok = motion_control_step(&control, &message, 400U, &output) == MOTION_CONTROL_WAITING && ok;
	message = control_odometry(3U, 450U, 0, 0);
	ok = motion_control_step(&control, &message, 450U, &output) == MOTION_CONTROL_FAULT && ok;
	ok = motion_control_start(&control, &config, 1, 500U) && ok;
	message = control_odometry(1U, 500U, 0, 0);
	ok = motion_control_step(&control, &message, 500U, &output) == MOTION_CONTROL_WAITING && ok;
	message = control_odometry(2U, 651U, 0, 0);
	ok = motion_control_step(&control, &message, 651U, &output) == MOTION_CONTROL_FAULT && ok;
	ok = motion_control_start(&control, &config, 1, 700U) && ok;
	ok = motion_control_step(&control, NULL, 699U, &output) == MOTION_CONTROL_FAULT && ok;
	config.feedback.max_interval_ms = 251U;
	ok = !motion_control_start(&control, &config, 1, 800U) &&
	     control.faulted && !control.running && control.output.left_pwm_percent == 0U && ok;
	return ok;
}

BOOL wheel_speed_test(void)
{
	BOOL correction_ok;
	BOOL pid_ok;
	BOOL invalid_ok;
	BOOL feedback_ok;
	BOOL control_ok;
	BOOL control_faults_ok;
	BOOL feed_forward_ok;
	TEST_INIT();

	correction_ok = correction_test();
	feed_forward_ok = feed_forward_test();
	pid_ok = pid_state_test();
	invalid_ok = invalid_test();
	feedback_ok = feedback_test();
	control_ok = control_flow_test();
	control_faults_ok = control_fault_test();
	TEST_CHECK("MOT-U01", correction_ok, "independent wheel correction and limits");
	TEST_CHECK("MOT-U02", pid_ok, "anti-windup, derivative and reset");
	TEST_CHECK("MOT-U03", invalid_ok, "invalid and overflow inputs fail safely");
	TEST_CHECK("MOT-U04", feedback_ok, "signed wheel feedback and stale inputs");
	TEST_CHECK("MOT-U05", control_ok, "straight control waits for fresh wheel feedback");
	TEST_CHECK("MOT-U06", control_faults_ok, "control faults latch and clear output");
	TEST_CHECK("MOT-U14", feed_forward_ok,
		   "per-wheel feed-forward from each target, clamped, PID limits follow it");
	TEST_SUMMARY("wheel speed");
	return correction_ok && pid_ok && invalid_ok && feedback_ok &&
	       control_ok && control_faults_ok && feed_forward_ok;
}
#endif
