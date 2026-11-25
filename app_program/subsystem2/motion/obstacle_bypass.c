/*
 * obstacle_bypass.c - spin scan, pull out, drive past and rejoin the line.
 * See obstacle_bypass.h. Moved from spin_width_hwtest.c (bench-tested:
 * OBS-H03, obstacle centred and 5 cm to each side) with the GO button and
 * the obstacle queue replaced by the io callbacks.
 */
#include "../../app.h"
#include "../../subsystem5/obstacle/obs_profile_config.h"
#include "../../subsystem5/obstacle/spin_scan.h"
#include "../../subsystem6/odometry/encoder_source.h"
#include "motor_driver.h"
#include "wheel_speed_control.h"
#include "obstacle_bypass.h"

#include <tk/syslib.h>

#ifndef APP_SENSOR_TEST
#define APP_SENSOR_TEST 0
#endif
#ifndef APP_VEHICLE
#define APP_VEHICLE 0
#endif

#if APP_VEHICLE || APP_SENSOR_TEST == 15

/* Same calibration as the other motion tests and the vehicle (app_main.c). */
#define COUNTS_PER_REV     2500.0f
#define WHEEL_CIRC_MM      204.0f
#define LEFT_SIGN          (-1)
#define RIGHT_SIGN         1
#define LEFT_MOTOR_FWD     1
#define RIGHT_MOTOR_FWD    (-1)
#define PID_KP_V           2.0f
#define PID_KI_V           4.0f
#define PID_BASE_V         22U
#define PID_MAX_V          60U
#define PID_PERIOD_MS      50U

#define PI_F               3.14159265f
#define DEG                (PI_F / 180.0f)
#define LOOP_MS            10U
#define SLOW_ZONE_DEG      15.0f
#define SETTLE_MS          600U    /* after a big turn or a straight leg */
#define STEP_SETTLE_MS     150U    /* after a scan step, before the reading */
#define STEP_TIMEOUT_MS    1500U
#define TURN_TIMEOUT_MS    8000U
#define STALL_MS           1000U
#define STALL_DEG          0.3f
#define STALL_MM           2.0f
#define WRONG_WAY_DEG      5.0f
#define WRONG_WAY_MM       20.0f
#define READ_TIMEOUT_MS    800U
#define CENTRE_ZONE_DEG    20.0f   /* the obstacle's ahead echo must be within this */
#define RETURN_LEAD_DEG    3.5f    /* first guess of the coast after the scan's return turn */
#define RETURN_LEAD_MAX    10.0f
#define BYPASS_LEAD_DEG    3.0f    /* turns back to the line brake this early */
#define HEADING_GAIN       0.5f    /* cm/s per degree of heading error */
#define MAX_PULLOUT_MM     500.0f  /* never drive further sideways than this */
#define TIE_MM             10.0f   /* right wins if it sticks out at most this much more */

typedef struct {
	float size;
	float threshold;
} stepper_t;

typedef struct {
	float x, y, h;
	float l, r;              /* last wheel travel, mm */
	float base;              /* absolute heading of the line */
} pose_t;

typedef obstacle_bypass_status_t status_t;

static const obstacle_bypass_config_t *s_cfg;
static const obstacle_bypass_io_t *s_io;
static spin_scan_config_t s_scan;
static spin_scan_sample_t s_samples[SPIN_SCAN_MAX_SAMPLES];
static unsigned s_count;
static stepper_t s_sweep_step, s_find_step;
static float s_return_lead = RETURN_LEAD_DEG;
static wheel_speed_control_t s_pid;
static pose_t s_pose;

static float absf(float v) { return v < 0.0f ? -v : v; }

static uint32_t now_ms(void)
{
	SYSTIM t;

	tk_get_otm(&t);
	return t.lo;
}

static void poll(void)
{
	if (s_io->poll != NULL) s_io->poll(s_io->context);
}

static void emit(const obstacle_bypass_event_t *event)
{
	if (s_io->event != NULL) s_io->event(s_io->context, event);
}

static uint16_t mm_u16(float mm)
{
	if (!(mm > 0.0f)) return 0U;
	return mm >= 65535.0f ? 65535U : (uint16_t)(mm + 0.5f);
}

/* Free room beside the obstacle on one side, for the PROFILE report. */
static uint16_t side_room_mm(const spin_scan_side_t *side)
{
	return mm_u16(side->object_seen ? side->gap_mm : side->reach_mm);
}

static bool aborted(void)
{
	return s_io->abort(s_io->context);
}

static void wait_ms(uint32_t ms)
{
	uint32_t t0 = now_ms();

	while (now_ms() - t0 < ms) {
		poll();
		tk_dly_tsk(LOOP_MS);
	}
}

/* A distance measured at or after `after` (ms). */
static bool fresh_distance(uint32_t after, uint16_t *mm)
{
	uint32_t t0 = now_ms();

	for (;;) {
		uint16_t d;
		uint32_t ts;

		poll();
		if (s_io->latest_ahead(s_io->context, &d, &ts) && (int32_t)(ts - after) >= 0) {
			*mm = d;
			return true;
		}
		if (now_ms() - t0 > READ_TIMEOUT_MS) return false;
		tk_dly_tsk(LOOP_MS);
	}
}

/* A reading taken from now on; no echo is read once more, so a single dropped
 * echo does not count as a miss. */
static bool distance_here(uint16_t *mm)
{
	if (!fresh_distance(now_ms(), mm)) return false;
	if (!spin_scan_valid_mm(*mm) && !fresh_distance(now_ms(), mm)) return false;
	return true;
}

/* Wheel travel since boot, mm, + = forward. */
static bool read_wheels(float *l, float *r)
{
	encoder_snapshot_t s;
	const float mm_per_count = WHEEL_CIRC_MM / COUNTS_PER_REV;

	if (encoder_source_snapshot(&s) < E_OK || !s.valid) return false;
	*l = (float)(LEFT_SIGN * s.left_count) * mm_per_count;
	*r = (float)(RIGHT_SIGN * s.right_count) * mm_per_count;
	return true;
}

static float heading_of(float l, float r)
{
	return (r - l) / s_cfg->track_mm / DEG;
}

/* Absolute heading since boot, degrees, + = left. */
static bool read_heading(float *deg)
{
	float l, r;

	if (!read_wheels(&l, &r)) return false;
	*deg = heading_of(l, r);
	return true;
}

/* dir +1 = turn left (left wheel back, right wheel forward), -1 = right. */
static void spin(int dir, unsigned pwm)
{
	int16_t left = (int16_t)(-dir * (int)pwm);
	int16_t right = (int16_t)(dir * (int)pwm);

	(void)motor_set_speed(MOTOR_1, (int16_t)(left * LEFT_MOTOR_FWD));
	(void)motor_set_speed(MOTOR_2, (int16_t)(right * RIGHT_MOTOR_FWD));
}

static void drive_forward(uint8_t pwm_l, uint8_t pwm_r)
{
	(void)motor_set_speed(MOTOR_1, (int16_t)((int)pwm_l * LEFT_MOTOR_FWD));
	(void)motor_set_speed(MOTOR_2, (int16_t)((int)pwm_r * RIGHT_MOTOR_FWD));
}

/* "-12.3" (tm_printf has no %f). */
static const char *f1(char *buf, float v)
{
	int t = (int)__builtin_lroundf(v * 10.0f);
	unsigned a = (unsigned)(t < 0 ? -t : t);
	unsigned whole = a / 10U, i = 0U, n = 0U;
	char digits[10];

	if (t < 0) buf[i++] = '-';
	do { digits[n++] = (char)('0' + whole % 10U); whole /= 10U; } while (whole && n < 9U);
	while (n) buf[i++] = digits[--n];
	buf[i++] = '.';
	buf[i++] = (char)('0' + a % 10U);
	buf[i] = '\0';
	return buf;
}

/* Heading as L12.3 / R12.3 / C0.0. */
static const char *fdeg(char *buf, float deg)
{
	char tmp[16];

	buf[0] = deg > 0.05f ? 'L' : (deg < -0.05f ? 'R' : 'C');
	f1(tmp, absf(deg));
	for (unsigned i = 0U; ; i++) {
		buf[i + 1U] = tmp[i];
		if (tmp[i] == '\0') break;
	}
	return buf;
}

#define LOG(...) do { tm_printf((UB*)"%s", s_io->tag); tm_printf((UB*)__VA_ARGS__); } while (0)

/* Turn to an absolute heading, brake and settle; braking starts `lead`
 * degrees before the target, for the coast. */
static status_t turn_to(float target, float lead)
{
	float h, start, progress_h;
	uint32_t t0 = now_ms(), progress_ms = 0U;
	int dir;
	status_t status = OBSTACLE_BYPASS_OK;

	if (!read_heading(&h)) return OBSTACLE_BYPASS_ENCODER;
	start = progress_h = h;
	dir = target > h ? 1 : -1;
	for (;;) {
		float remaining;
		uint32_t elapsed = now_ms() - t0;

		if (!read_heading(&h)) { status = OBSTACLE_BYPASS_ENCODER; break; }
		remaining = (target - h) * (float)dir;
		if (remaining <= lead) break;
		if (aborted()) { status = OBSTACLE_BYPASS_ABORTED; break; }
		if (elapsed > TURN_TIMEOUT_MS) { status = OBSTACLE_BYPASS_TIMEOUT; break; }
		if ((start - h) * (float)dir > WRONG_WAY_DEG) { status = OBSTACLE_BYPASS_WRONG_WAY; break; }
		if ((h - progress_h) * (float)dir >= STALL_DEG) {
			progress_h = h;
			progress_ms = elapsed;
		} else if (elapsed - progress_ms > STALL_MS) {
			status = OBSTACLE_BYPASS_STALL;
			break;
		}
		spin(dir, remaining > SLOW_ZONE_DEG ? s_cfg->spin_pwm : s_cfg->spin_pwm_slow);
		poll();
		tk_dly_tsk(LOOP_MS);
	}
	(void)motor_brake_all();
	wait_ms(SETTLE_MS);
	return status;
}

/* One scan step (dir +1 = left, -1 = right): pulse until the heading has
 * moved by the stepper's learned threshold, brake, settle. The threshold
 * adapts so the step including the coast averages `size`. */
static status_t step(int dir, stepper_t *stepper)
{
	float h0, h, progress_h, moved;
	uint32_t t0 = now_ms(), progress_ms = 0U;
	status_t status = OBSTACLE_BYPASS_OK;

	if (!read_heading(&h0)) return OBSTACLE_BYPASS_ENCODER;
	progress_h = h0;
	for (;;) {
		uint32_t elapsed = now_ms() - t0;

		if (!read_heading(&h)) { status = OBSTACLE_BYPASS_ENCODER; break; }
		moved = (h - h0) * (float)dir;
		if (moved >= stepper->threshold) break;
		if (aborted()) { status = OBSTACLE_BYPASS_ABORTED; break; }
		if (elapsed > STEP_TIMEOUT_MS) { status = OBSTACLE_BYPASS_TIMEOUT; break; }
		if (-moved > WRONG_WAY_DEG) { status = OBSTACLE_BYPASS_WRONG_WAY; break; }
		if ((h - progress_h) * (float)dir >= STALL_DEG) {
			progress_h = h;
			progress_ms = elapsed;
		} else if (elapsed - progress_ms > STALL_MS) {
			status = OBSTACLE_BYPASS_STALL;
			break;
		}
		spin(dir, s_cfg->spin_pwm);
		poll();
		tk_dly_tsk(LOOP_MS);
	}
	(void)motor_brake_all();
	wait_ms(STEP_SETTLE_MS);
	if (!read_heading(&h)) return OBSTACLE_BYPASS_ENCODER;
	moved = (h - h0) * (float)dir;
	if (status == OBSTACLE_BYPASS_OK) {
		stepper->threshold += (stepper->size - moved) * 0.5f;
		if (stepper->threshold < 0.2f) stepper->threshold = 0.2f;
		if (stepper->threshold > stepper->size) stepper->threshold = stepper->size;
	}
	return status;
}

static void print_reading(const char *label, float deg, uint16_t mm)
{
	char b1[16];

	if (spin_scan_valid_mm(mm))
		LOG("  %s%6s deg  %4u mm\n", label, fdeg(b1, deg), (unsigned)mm);
	else
		LOG("  %s%6s deg  NONE\n", label, fdeg(b1, deg));
}

/* Heading (relative to base) and a reading taken after the robot stopped. */
static status_t read_here(float base, float *deg, uint16_t *mm)
{
	float h;

	if (!read_heading(&h)) return OBSTACLE_BYPASS_ENCODER;
	if (!distance_here(mm)) return OBSTACLE_BYPASS_SENSOR;
	*deg = h - base;
	return OBSTACLE_BYPASS_OK;
}

static status_t take_sample(float base, float *deg, uint16_t *mm)
{
	status_t st;

	if (s_count >= SPIN_SCAN_MAX_SAMPLES) return OBSTACLE_BYPASS_FULL;
	st = read_here(base, deg, mm);
	if (st != OBSTACLE_BYPASS_OK) return st;
	s_samples[s_count].deg = *deg;
	s_samples[s_count].mm = *mm;
	s_count++;
	print_reading("", *deg, *mm);
	return OBSTACLE_BYPASS_OK;
}

static void report_side(bool left, const spin_scan_side_t *side)
{
	char b1[16], b2[16], b3[16];

	if (side->object_seen)
		LOG("  free %-5s   : %s mm gap to the next object (it is %s mm %s of "
		    "centre, %s mm ahead of the turning point) -> %s\n", left ? "LEFT" : "RIGHT",
		    f1(b1, side->gap_mm), f1(b2, side->object_mm), left ? "left" : "right",
		    f1(b3, side->object_fwd_mm), side->passable ? "PASSABLE" : "too narrow");
	else
		LOG("  free %-5s   : nothing else seen out to ~%s mm %s of centre -> %s\n",
		    left ? "LEFT" : "RIGHT", f1(b1, side->reach_mm), left ? "left" : "right",
		    side->passable ? "PASSABLE" : "not checked far enough");
	if (side->ignored)
		LOG("                 (%u echo%s more than %s mm ahead ignored: past the bypass)\n",
		    side->ignored, side->ignored == 1U ? "" : "es", f1(b1, side->ahead_limit_mm));
}

static const char *edge_text(bool seen, bool hidden)
{
	return seen ? "seen" : (hidden ? "HIDDEN by a nearer object" : "BEYOND the scan");
}

static bool analyse(spin_scan_result_t *r)
{
	char b1[16], b2[16], b3[16], b4[16];

	spin_scan_sort(s_samples, s_count);
	if (!spin_scan_analyse(s_samples, s_count, &s_scan, r)) {
		LOG("RESULT: no echo within %s deg of ahead and %s mm -- nothing to measure\n",
		    f1(b1, CENTRE_ZONE_DEG), f1(b2, s_cfg->max_obstacle_mm));
		return false;
	}
	LOG("RESULT (%u readings, steps averaged %s deg):\n", s_count, f1(b1, r->mean_step_deg));
	LOG("  closest      : %u mm from the sensor at %s deg\n",
	    (unsigned)(r->closest_mm + 0.5f), fdeg(b1, r->closest_deg));
	LOG("  heard from   : %s to %s deg  (left edge %s, right edge %s)\n",
	    fdeg(b1, r->right_deg), fdeg(b2, r->left_deg),
	    edge_text(r->left_seen, r->left_hidden), edge_text(r->right_seen, r->right_hidden));
	if (r->left_seen && r->right_seen)
		LOG("  edge step    : left %s deg, right %s deg (edge uncertainty is half of each)\n",
		    f1(b1, s_samples[r->left_index + 1U].deg - s_samples[r->left_index].deg),
		    f1(b2, s_samples[r->right_index].deg - s_samples[r->right_index - 1U].deg));
	LOG("  raw width    : %s mm  (left %s, right %s of centreline)\n",
	    f1(b1, r->raw_width_mm), f1(b2, r->raw_left_mm), f1(b3, r->raw_right_mm));
	LOG("  WIDTH        : %s mm with trim %s deg  (left %s, right %s)%s\n",
	    f1(b1, r->width_mm), f1(b4, s_cfg->trim_deg), f1(b2, r->left_mm), f1(b3, r->right_mm),
	    r->left_seen && r->right_seen ? "" : "  LOWER BOUND: an edge is hidden or beyond the scan");
	{
		char c1[16], c2[16], c3[16], c4[16], c5[16], c6[16];

		LOG("  CSV,%s,%s,%s,%s,%s,%u,%u,%s\n",
		    f1(c1, r->closest_mm), fdeg(c2, r->right_deg), fdeg(c3, r->left_deg),
		    f1(c4, r->width_mm), f1(c5, r->raw_width_mm), r->left_seen ? 1U : 0U,
		    r->right_seen ? 1U : 0U, f1(c6, s_cfg->trim_deg));
	}
	if (s_cfg->true_width_mm > 0.0f) {
		float suggested;

		if (spin_scan_calibrate_trim(r, &s_scan, s_cfg->true_width_mm, &suggested))
			LOG("  calibration  : true %s mm, error %s mm; trim %s deg would match it\n",
			    f1(b3, s_cfg->true_width_mm), f1(b1, r->width_mm - s_cfg->true_width_mm),
			    f1(b2, suggested));
		else
			LOG("  calibration  : skipped, an edge is hidden or beyond the scan\n");
	}
	report_side(true, &r->left_free);
	report_side(false, &r->right_free);
	return true;
}

/* Find pass: step right from heading 0 until the obstacle (at ahead_mm) is
 * lost. Returns the sweep start heading (relative, negative = right). */
static status_t find_right_edge(float base, uint16_t ahead_mm, float *start)
{
	spin_scan_edge_t edge;
	char b1[16], b2[16];
	float deg = 0.0f;
	uint16_t mm;
	status_t st;

	spin_scan_edge_start(&edge, ahead_mm, 0.0f);
	LOG("finding the right edge (%s deg steps, not measured):\n", f1(b1, s_cfg->find_step_deg));
	while (-deg < s_cfg->half_deg) {
		st = step(-1, &s_find_step);
		if (st != OBSTACLE_BYPASS_OK) return st;
		st = read_here(base, &deg, &mm);
		if (st != OBSTACLE_BYPASS_OK) return st;
		print_reading("find ", deg, mm);
		if (spin_scan_edge_update(&edge, &s_scan, deg, mm)) break;
	}
	*start = spin_scan_sweep_start_deg(&s_scan, &edge, s_cfg->step_deg);
	if (edge.lost)
		LOG("right edge near %s deg%s; sweep starts at %s deg\n",
		    fdeg(b1, spin_scan_edge_deg(&edge)),
		    edge.hidden ? " (a nearer object takes over)" : "", fdeg(b2, *start));
	else
		LOG("right edge not found within R%s deg; sweep starts there\n", f1(b1, s_cfg->half_deg));
	return OBSTACLE_BYPASS_OK;
}

/* Scan and turn back to the start heading `base`. *found = an obstacle. */
static status_t scan(float base, spin_scan_result_t *r, bool *found)
{
	spin_scan_edge_t left_edge;
	float start, deg, h;
	uint16_t ahead_mm, mm;
	bool adaptive;
	status_t st;
	char b1[16];

	*found = false;
	s_count = 0U;
	if (!distance_here(&ahead_mm)) return OBSTACLE_BYPASS_SENSOR;
	/* Find-then-sweep needs the obstacle straight ahead. A flat face beside
	 * the line may not echo there: the reading is then the background, so
	 * sweep everything and let the analysis find the obstacle. */
	adaptive = !s_cfg->full_sweep && spin_scan_valid_mm(ahead_mm) &&
		   (float)ahead_mm <= s_cfg->max_obstacle_mm;
	if (adaptive) {
		LOG("obstacle ahead at %u mm\n", (unsigned)ahead_mm);
		st = find_right_edge(base, ahead_mm, &start);
		if (st != OBSTACLE_BYPASS_OK) return st;
	} else {
		if (s_cfg->full_sweep)
			LOG("full sweep requested");
		else if (spin_scan_valid_mm(ahead_mm))
			LOG("ahead reads %u mm (beyond %s mm: background, not the obstacle)",
			    (unsigned)ahead_mm, f1(b1, s_cfg->max_obstacle_mm));
		else
			LOG("nothing heard ahead");
		tm_printf((UB*)": full sweep R..L %s deg\n", f1(b1, s_cfg->half_deg));
		start = -s_cfg->half_deg;
	}
	/* The find pass usually ends past the start already: sweep from there
	 * rather than turning back (no reversal, and the room is still covered). */
	if (!read_heading(&h)) return OBSTACLE_BYPASS_ENCODER;
	if (h - base > start) {
		st = turn_to(base + start, 0.0f);   /* overshooting outwards is harmless */
		if (st != OBSTACLE_BYPASS_OK) return st;
	}

	LOG("measuring, turning left (heading, sensor distance):\n");
	spin_scan_edge_start(&left_edge, ahead_mm, 0.0f);
	st = take_sample(base, &deg, &mm);
	for (;;) {
		if (st != OBSTACLE_BYPASS_OK) break;
		if (adaptive && deg >= 0.0f)
			(void)spin_scan_edge_update(&left_edge, &s_scan, deg, mm);
		if (adaptive ? spin_scan_sweep_done(&s_scan, &left_edge, deg) : deg >= s_cfg->half_deg)
			break;
		st = step(1, &s_sweep_step);
		if (st == OBSTACLE_BYPASS_OK) st = take_sample(base, &deg, &mm);
	}
	(void)motor_brake_all();
	if (st != OBSTACLE_BYPASS_OK) return st;
	LOG("turning back to the start heading (braking %s deg early)\n", f1(b1, s_return_lead));
	{
		float before = base + deg;
		float dir;

		(void)read_heading(&before);
		dir = base > before ? 1.0f : -1.0f;
		st = turn_to(base, s_return_lead);
		if (st != OBSTACLE_BYPASS_OK) return st;
		if (read_heading(&h)) {
			/* + = went past the start heading: brake earlier next time. */
			float past = (h - base) * dir;

			s_return_lead += 0.5f * past;
			if (s_return_lead < 0.0f) s_return_lead = 0.0f;
			if (s_return_lead > RETURN_LEAD_MAX) s_return_lead = RETURN_LEAD_MAX;
			LOG("back at %s deg from the start heading\n", fdeg(b1, h - base));
		}
	}
	*found = analyse(r);
	return OBSTACLE_BYPASS_OK;
}

/* ---- Bypass ------------------------------------------------------------- */

/* Integrate the pose from the wheel travel since the last call (a turn in
 * place adds almost nothing). */
static bool pose_update(void)
{
	float l, r, h, ds;

	if (!read_wheels(&l, &r)) return false;
	h = heading_of(l, r) - s_pose.base;
	ds = 0.5f * ((l - s_pose.l) + (r - s_pose.r));
	s_pose.x += ds * -__builtin_sinf(0.5f * (h + s_pose.h) * DEG);   /* midpoint heading */
	s_pose.y += ds * __builtin_cosf(0.5f * (h + s_pose.h) * DEG);
	s_pose.h = h;
	s_pose.l = l;
	s_pose.r = r;
	return true;
}

static bool pose_reset(float base)
{
	float l, r;

	if (!read_wheels(&l, &r)) return false;
	s_pose = (pose_t){ 0.0f, 0.0f, heading_of(l, r) - base, l, r, base };
	return true;
}

/* Turn to a heading relative to the line (base) and keep the pose. */
static status_t turn_rel(float heading, float lead)
{
	status_t st = turn_to(s_pose.base + heading, lead);

	if (!pose_update() && st == OBSTACLE_BYPASS_OK) st = OBSTACLE_BYPASS_ENCODER;
	return st;
}

/* Drive forward on `heading` (relative to the line) with the wheel-speed PID
 * and a heading hold until the pose's x (axis_y false) or y (axis_y true)
 * reaches `target`. */
static status_t drive_until(float heading, bool axis_y, float target)
{
	const float v = s_cfg->speed_cm_s;
	wheel_speed_output_t out = { PID_BASE_V, PID_BASE_V };
	float start, sign, limit, l0, r0, pl, pr, prog_l = 0.0f, prog_r = 0.0f;
	uint32_t t0 = now_ms(), next = t0 + PID_PERIOD_MS, last = t0;
	uint32_t prog_l_ms = 0U, prog_r_ms = 0U;
	status_t status = OBSTACLE_BYPASS_OK;

	if (!pose_update()) return OBSTACLE_BYPASS_ENCODER;
	start = axis_y ? s_pose.y : s_pose.x;
	if (absf(target - start) < 1.0f) return OBSTACLE_BYPASS_OK;
	sign = target > start ? 1.0f : -1.0f;
	limit = absf(target - start) + 150.0f;        /* overtravel guard */
	l0 = pl = s_pose.l;
	r0 = pr = s_pose.r;
	if (!wheel_speed_control_reset(&s_pid) || !wheel_speed_control_set_targets(&s_pid, v, v))
		return OBSTACLE_BYPASS_PID;
	drive_forward(out.left_pwm_percent, out.right_pwm_percent);

	for (;;) {
		uint32_t t, run;
		float dl, dr, now_pos;

		tk_dly_tsk(LOOP_MS);
		poll();
		if (!pose_update()) { status = OBSTACLE_BYPASS_ENCODER; break; }
		t = now_ms();
		run = t - t0;
		dl = s_pose.l - l0;
		dr = s_pose.r - r0;
		now_pos = axis_y ? s_pose.y : s_pose.x;

		if ((now_pos - target) * sign >= 0.0f) break;
		if (aborted()) { status = OBSTACLE_BYPASS_ABORTED; break; }
		if (dl < -WRONG_WAY_MM || dr < -WRONG_WAY_MM) { status = OBSTACLE_BYPASS_WRONG_WAY; break; }
		if (absf(dl - prog_l) >= STALL_MM) { prog_l = dl; prog_l_ms = run; }
		if (absf(dr - prog_r) >= STALL_MM) { prog_r = dr; prog_r_ms = run; }
		if (run - prog_l_ms > STALL_MS || run - prog_r_ms > STALL_MS) {
			status = OBSTACLE_BYPASS_STALL;
			break;
		}
		if (0.5f * (dl + dr) > limit) { status = OBSTACLE_BYPASS_TIMEOUT; break; }

		if ((int32_t)(t - next) < 0) continue;
		next += PID_PERIOD_MS;
		{
			float dt = (float)(t - last) / 1000.0f;
			float sl = (s_pose.l - pl) / 10.0f / dt;
			float sr = (s_pose.r - pr) / 10.0f / dt;
			/* Heading hold: e > 0 = must turn left. */
			float e = heading - s_pose.h;
			float tl = v - HEADING_GAIN * e;
			float tr = v + HEADING_GAIN * e;

			pl = s_pose.l;
			pr = s_pose.r;
			last = t;
			if (sl < 0.0f) sl = 0.0f;
			if (sr < 0.0f) sr = 0.0f;
			if (tl < 1.0f) tl = 1.0f;
			if (tr < 1.0f) tr = 1.0f;
			if (!wheel_speed_control_set_targets(&s_pid, tl, tr) ||
			    !wheel_speed_control_update(&s_pid, sl, sr, dt, &out)) {
				status = OBSTACLE_BYPASS_PID;
				break;
			}
		}
		drive_forward(out.left_pwm_percent, out.right_pwm_percent);
	}
	(void)motor_brake_all();
	wait_ms(SETTLE_MS);
	if (!pose_update() && status == OBSTACLE_BYPASS_OK) status = OBSTACLE_BYPASS_ENCODER;
	return status;
}

static const char *side_state(bool usable, bool seen)
{
	return usable ? "ok" : (seen ? "too narrow" : "edge not seen");
}

/* Facing the line: is anything in the strip the car will cross to rejoin it?
 * Two readings; either one inside the strip counts. */
static status_t look_across(float path_mm, bool *blocked, uint16_t *nearest)
{
	unsigned k;

	*blocked = false;
	*nearest = OBS_DIST_NONE;
	for (k = 0U; k < 2U; k++) {
		uint16_t mm;

		if (!distance_here(&mm)) return OBSTACLE_BYPASS_SENSOR;
		if (spin_scan_valid_mm(mm)) {
			if (*nearest == OBS_DIST_NONE || mm < *nearest) *nearest = mm;
			if ((float)mm <= path_mm) *blocked = true;
		}
	}
	return OBSTACLE_BYPASS_OK;
}

/* Drive forward beside the obstacle, looking across at the line every
 * pass_step_mm until nothing is in the way, then rejoin the line. */
static status_t pass_and_rejoin(bool right, float face_y)
{
	char b1[16], b2[16], b3[16], b4[16];
	const float to_line = right ? 90.0f : -90.0f;    /* heading that faces the line */
	const float band = 0.5f * s_cfg->car_width_mm + s_cfg->clear_mm;
	float look_y = face_y + s_cfg->min_depth_mm;
	float blocked_y = face_y;                        /* last look that saw it */
	float clear_y;
	bool blocked = true;
	status_t st;

	{
		const obstacle_bypass_event_t ev = { OBSTACLE_BYPASS_EVENT_PASSING, 0U, 0U, 0U, right };
		emit(&ev);
	}
	LOG("PASS: face ~%s mm ahead of the start; driving past, looking across every %s mm\n",
	    f1(b1, face_y), f1(b2, s_cfg->pass_step_mm));
	for (;;) {
		float path;
		uint16_t nearest;

		st = drive_until(0.0f, true, look_y);
		if (st != OBSTACLE_BYPASS_OK) return st;
		st = turn_rel(to_line, 0.0f);
		if (st != OBSTACLE_BYPASS_OK) return st;
		/* The sensor is sensor_offset_mm towards the line from the axle; the
		 * strip runs from it to the far side of the car on the line. */
		path = absf(s_pose.x) - s_cfg->sensor_offset_mm + band;
		st = look_across(path, &blocked, &nearest);
		if (st != OBSTACLE_BYPASS_OK) return st;
		if (nearest != OBS_DIST_NONE)
			LOG("  look at y %s: %u mm (rejoin path %s mm) -> %s\n", f1(b1, s_pose.y),
			    (unsigned)nearest, f1(b2, path), blocked ? "OBSTACLE beside" : "clear");
		else
			LOG("  look at y %s: NONE (rejoin path %s mm) -> clear\n",
			    f1(b1, s_pose.y), f1(b2, path));
		if (!blocked) break;
		blocked_y = s_pose.y;
		if (s_pose.y - face_y >= s_cfg->max_depth_mm) {
			LOG("  still beside it %s mm past the face: giving up, not rejoining\n",
			    f1(b1, s_cfg->max_depth_mm));
			return OBSTACLE_BYPASS_TOO_LONG;
		}
		st = turn_rel(0.0f, BYPASS_LEAD_DEG);
		if (st != OBSTACLE_BYPASS_OK) return st;
		look_y = s_pose.y + s_cfg->pass_step_mm;
	}
	/* The back lies between the last blocked look and this one. */
	clear_y = s_pose.y;
	LOG("  obstacle back between y %s and %s: length ~%s mm (+/- %s)\n",
	    f1(b1, blocked_y), f1(b2, clear_y), f1(b3, 0.5f * (blocked_y + clear_y) - face_y),
	    f1(b4, 0.5f * s_cfg->pass_step_mm));
	/* The car sweeps +/- band along y while crossing back: move on that far. */
	st = turn_rel(0.0f, BYPASS_LEAD_DEG);
	if (st != OBSTACLE_BYPASS_OK) return st;
	LOG("REJOIN: forward %s mm more (half car + clearance), then back to the line\n", f1(b1, band));
	st = drive_until(0.0f, true, clear_y + band);
	if (st != OBSTACLE_BYPASS_OK) return st;
	st = turn_rel(to_line, 0.0f);
	if (st != OBSTACLE_BYPASS_OK) return st;
	st = drive_until(to_line, false, 0.0f);
	if (st != OBSTACLE_BYPASS_OK) return st;
	st = turn_rel(0.0f, BYPASS_LEAD_DEG);
	if (st != OBSTACLE_BYPASS_OK) return st;
	LOG("  done: car centre x %s mm from the line, %s mm ahead of the start, heading %s\n",
	    f1(b1, s_pose.x), f1(b2, s_pose.y), fdeg(b4, s_pose.h));
	{
		const obstacle_bypass_event_t ev = { OBSTACLE_BYPASS_EVENT_REJOINED, 0U, 0U, 0U, right };
		emit(&ev);
	}
	return OBSTACLE_BYPASS_OK;
}

/* Choose the side, pull out to it and (rejoin) drive past and rejoin the
 * line. The robot is back at the start heading (base), at the turning point. */
static status_t bypass(float base, const spin_scan_result_t *r)
{
	char b1[16], b2[16], b3[16], b4[16], b5[16], b6[16];
	spin_scan_bypass_t plan;
	const char *why;
	bool right;
	status_t st;

	if (!spin_scan_plan_bypass(r, s_cfg->car_width_mm, s_cfg->clear_mm, s_cfg->margin_mm,
				   TIE_MM, &plan)) {
		LOG("BYPASS: no usable side (right %s, left %s) -- not moving\n",
		    side_state(plan.right_usable, r->right_seen),
		    side_state(plan.left_usable, r->left_seen));
		return OBSTACLE_BYPASS_NO_SIDE;
	}
	right = plan.side == SPIN_SCAN_BYPASS_RIGHT;
	{
		const obstacle_bypass_event_t ev = {
			OBSTACLE_BYPASS_EVENT_PROFILE, mm_u16(r->width_mm),
			side_room_mm(&r->left_free), side_room_mm(&r->right_free), right
		};
		emit(&ev);
	}
	if (!plan.preferred)
		why = right ? "left blocked or its edge not seen" : "right blocked or its edge not seen";
	else if (!right)
		why = "sticks out less on the left";
	else
		why = r->right_mm > r->left_mm ? "within the tie: right by default" : "sticks out less on the right";
	LOG("BYPASS: %s side (%s); edge at x %s mm, car centre to x %s mm "
	    "(edge + %s margin + %s clearance + %s half car; x + = right)\n",
	    right ? "RIGHT" : "LEFT", why, f1(b1, plan.edge_x), f1(b2, plan.target_x),
	    f1(b3, s_cfg->margin_mm), f1(b4, s_cfg->clear_mm), f1(b5, 0.5f * s_cfg->car_width_mm));
	if (absf(plan.target_x) > MAX_PULLOUT_MM) {
		LOG("  more than %s mm sideways: not moving\n", f1(b1, MAX_PULLOUT_MM));
		return OBSTACLE_BYPASS_TOO_FAR;
	}
	if (!pose_reset(base)) return OBSTACLE_BYPASS_ENCODER;
	if (absf(plan.target_x) < 5.0f) {
		LOG("  already clear on that side\n");
	} else {
		LOG("  turning 90 deg %s\n", right ? "right" : "left");
		st = turn_rel(right ? -90.0f : 90.0f, 0.0f);
		if (st != OBSTACLE_BYPASS_OK) return st;
		LOG("  driving sideways at %s cm/s\n", f1(b1, s_cfg->speed_cm_s));
		st = drive_until(right ? -90.0f : 90.0f, false, plan.target_x);
		if (st != OBSTACLE_BYPASS_OK) return st;
		LOG("  turning back along the line\n");
		st = turn_rel(0.0f, BYPASS_LEAD_DEG);
		if (st != OBSTACLE_BYPASS_OK) return st;
		LOG("  pulled out: car centre x %s mm (target %s), heading %s; car side %s mm "
		    "from the measured edge\n", f1(b1, s_pose.x), f1(b2, plan.target_x),
		    fdeg(b3, s_pose.h),
		    f1(b6, absf(s_pose.x - plan.edge_x) - 0.5f * s_cfg->car_width_mm));
	}
	if (!s_cfg->rejoin) {
		LOG("  (rejoin off: stops beside the obstacle)\n");
		return OBSTACLE_BYPASS_OK;
	}
	/* Where the face is along the line, from the closest echo (as the scan's
	 * free-gap check measures it). */
	return pass_and_rejoin(right, (r->closest_mm + s_cfg->sensor_offset_mm) *
					     __builtin_cosf(r->closest_deg * DEG));
}

obstacle_bypass_config_t obstacle_bypass_default_config(void)
{
	obstacle_bypass_config_t c;

	c.track_mm = 116.0f;
	c.spin_pwm = 25U;
	c.spin_pwm_slow = 20U;
	/* 70: from the 20 cm stop the scan must turn ~54 deg to see a free lane
	 * (car + margin + clearances, 320 mm) beside a 9 cm box, plus the beam
	 * trim; 60 deg was enough from 30 cm. */
	c.half_deg = 70.0f;
	c.step_deg = 3.0f;
	c.find_step_deg = 6.0f;
	c.sensor_offset_mm = 70.0f;
	c.cluster_mm = 60.0f;
	c.lane_mm = 200.0f;
	c.beyond_mm = 400.0f;
	c.max_obstacle_mm = 450.0f;
	/* 15.0: 9 cm box from the 20 cm stop (SENSOR_TEST=spin_width at 228 mm
	 * read 119.7 mm with 11.6, the 30 cm value; 15.0 matches 90 mm). */
	c.trim_deg = 15.0f;
	c.true_width_mm = 0.0f;
	c.full_sweep = false;
	c.bypass = true;
	c.rejoin = true;
	c.car_width_mm = 140.0f;
	c.clear_mm = 50.0f;
	c.margin_mm = 80.0f;
	c.speed_cm_s = 15.0f;
	c.min_depth_mm = 50.0f;
	c.pass_step_mm = 100.0f;
	c.max_depth_mm = 600.0f;
	return c;
}

static bool config_valid(const obstacle_bypass_config_t *c)
{
	return c->track_mm >= 50.0f && c->track_mm <= 300.0f &&
	       c->spin_pwm <= 80U && c->spin_pwm_slow >= 10U && c->spin_pwm_slow <= c->spin_pwm &&
	       c->half_deg >= 20.0f && c->half_deg <= 75.0f &&
	       c->step_deg >= 1.0f && c->step_deg <= 5.0f &&
	       c->find_step_deg >= c->step_deg && c->find_step_deg <= 10.0f &&
	       c->sensor_offset_mm >= 0.0f && c->sensor_offset_mm <= 200.0f &&
	       c->cluster_mm >= 20.0f && c->cluster_mm <= 300.0f &&
	       c->lane_mm >= 0.0f && c->lane_mm <= 500.0f &&
	       c->beyond_mm >= 100.0f && c->beyond_mm <= 1000.0f &&
	       c->max_obstacle_mm >= 200.0f && c->max_obstacle_mm <= 900.0f &&
	       c->trim_deg >= 0.0f && c->trim_deg <= 40.0f &&
	       c->car_width_mm >= 80.0f && c->car_width_mm <= 250.0f &&
	       c->clear_mm >= 0.0f && c->clear_mm <= 200.0f &&
	       c->margin_mm >= 0.0f && c->margin_mm <= 200.0f &&
	       c->speed_cm_s >= 5.0f && c->speed_cm_s <= 20.0f &&
	       c->min_depth_mm >= 0.0f && c->min_depth_mm <= 300.0f &&
	       c->pass_step_mm >= 30.0f && c->pass_step_mm <= 200.0f &&
	       c->max_depth_mm >= 100.0f && c->max_depth_mm <= 1000.0f;
}

obstacle_bypass_status_t obstacle_bypass_run(const obstacle_bypass_config_t *config,
					     const obstacle_bypass_io_t *io)
{
	const wheel_speed_config_t pid_config = {
		PID_KP_V, PID_KI_V, 0.0f, config != NULL ? config->speed_cm_s : 15.0f,
		PID_BASE_V, PID_MAX_V
	};
	spin_scan_result_t r;
	float base;
	bool found;
	status_t st;

	if (config == NULL || io == NULL || io->latest_ahead == NULL || io->abort == NULL ||
	    io->tag == NULL || !config_valid(config))
		return OBSTACLE_BYPASS_CONFIG;
	s_cfg = config;
	s_io = io;
	if (!wheel_speed_control_init(&s_pid, &pid_config)) return OBSTACLE_BYPASS_CONFIG;
	{
		/* Bypassing: a side is free only if the car fits beside the
		 * obstacle with the margin and the clearance on both sides. */
		float lane = config->car_width_mm + config->margin_mm + 2.0f * config->clear_mm;

		if (!config->bypass || lane < config->lane_mm) lane = config->lane_mm;
		s_scan = (spin_scan_config_t){
			config->sensor_offset_mm, config->cluster_mm, config->trim_deg,
			CENTRE_ZONE_DEG, config->half_deg, lane, config->beyond_mm,
			config->max_obstacle_mm
		};
	}
	/* Steppers keep their learned threshold across runs unless resized. */
	if (s_sweep_step.size != config->step_deg)
		s_sweep_step = (stepper_t){ config->step_deg, config->step_deg * 0.5f };
	if (s_find_step.size != config->find_step_deg)
		s_find_step = (stepper_t){ config->find_step_deg, config->find_step_deg * 0.5f };

	if (!read_heading(&base)) return OBSTACLE_BYPASS_ENCODER;
	st = scan(base, &r, &found);
	if (st == OBSTACLE_BYPASS_OK && !found) st = OBSTACLE_BYPASS_NOT_FOUND;
	if (st == OBSTACLE_BYPASS_OK && config->bypass) st = bypass(base, &r);
	(void)motor_brake_all();
	return st;
}

#endif /* APP_VEHICLE || APP_SENSOR_TEST == 15 */

const char *obstacle_bypass_status_text(obstacle_bypass_status_t status)
{
	switch (status) {
	case OBSTACLE_BYPASS_OK:        return "ok";
	case OBSTACLE_BYPASS_NOT_FOUND: return "no obstacle echo near ahead";
	case OBSTACLE_BYPASS_NO_SIDE:   return "no usable side to pass on";
	case OBSTACLE_BYPASS_TOO_FAR:   return "pull-out would be too far sideways";
	case OBSTACLE_BYPASS_TOO_LONG:  return "still beside the obstacle at the length limit";
	case OBSTACLE_BYPASS_ABORTED:   return "stopped (GO or a fault)";
	case OBSTACLE_BYPASS_TIMEOUT:   return "timeout: move did not finish";
	case OBSTACLE_BYPASS_STALL:     return "stalled: wheels not turning (raise spin PWM or speed)";
	case OBSTACLE_BYPASS_WRONG_WAY: return "moved the WRONG WAY: motor or encoder sign";
	case OBSTACLE_BYPASS_ENCODER:   return "encoder INVALID";
	case OBSTACLE_BYPASS_SENSOR:    return "no distance reading from the HC-SR04";
	case OBSTACLE_BYPASS_FULL:      return "too many scan readings";
	case OBSTACLE_BYPASS_PID:       return "wheel-speed PID rejected its input";
	case OBSTACLE_BYPASS_CONFIG:    return "bad configuration";
	default:                        return "unknown";
	}
}

bool obstacle_bypass_is_fault(obstacle_bypass_status_t status)
{
	return status >= OBSTACLE_BYPASS_ABORTED;
}
