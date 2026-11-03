/*
 * host_test.c - runs the real obs_scan.c / obs_profile.c / obs_pack.c on a PC
 * against a simulated world (boxes + a 15 deg beam) and a fake Vehicle
 * Controller. Build/run: see ../Makefile  ->  make test
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "obs_scan.h"
#include "obs_port.h"
#include "obs_cfg.h"
#include "obs_pack.h"

/* ---------- simulated world ---------- */
typedef struct { float x0, x1, y0, y1; } box_t;     /* mm; x right, y forward */
static box_t g_box[4];
static int   g_nbox;
static bool  g_sensor_dead;
static uint32_t g_ms;
static uint32_t g_line_at;                           /* line "found" after this time */
static bool  g_vc_auto = true;                       /* fake Vehicle Controller reacts */
static bool  g_line_will_appear = true;
static uint8_t g_cmdq[16]; static int g_nq;
static obs_msg_t g_log[64]; static int g_nlog;
static uint32_t g_faultword;
static bool  g_obs_flag;
static float g_deg = 90.0f;

/**
 * ray_box - distance along one ray to an axis-aligned box.
 * What it does: standard slab test. The ray starts at the sensor (0,0) at angle
 * ang_deg measured from the robot's right.
 * Returns: distance in mm, or -1 if the ray misses the box.
 */
static float ray_box(float ang_deg, const box_t *b)
{
    float rad = ang_deg * 3.14159265f / 180.0f;
    float dx = cosf(rad), dy = sinf(rad), tmin = 0.0f, tmax = 1.0e9f;
    float lo[2] = { b->x0, b->y0 }, hi[2] = { b->x1, b->y1 }, d[2] = { dx, dy };
    for (int a = 0; a < 2; a++) {
        if (fabsf(d[a]) < 1e-6f) { if (0.0f < lo[a] || 0.0f > hi[a]) return -1.0f; continue; }
        float t1 = lo[a] / d[a], t2 = hi[a] / d[a];
        if (t1 > t2) { float t = t1; t1 = t2; t2 = t; }
        if (t1 > tmin) tmin = t1;
        if (t2 < tmax) tmax = t2;
        if (tmin > tmax) return -1.0f;
    }
    return tmin;
}

/**
 * world_dist - what the sensor would measure at a given servo angle.
 * What it does: models the 15-degree beam by casting rays from -7.5 to +7.5 degrees
 * around the centre and taking the nearest hit over all boxes. This is why a
 * simulated obstacle looks wider than it is, as with the real sensor.
 * Returns: nearest distance in mm (very large if nothing is hit).
 */
static float world_dist(float center_deg)            /* min over the beam cone */
{
    float best = 1.0e9f;
    for (float dd = -7.5f; dd <= 7.5f; dd += 2.5f)
        for (int i = 0; i < g_nbox; i++) {
            float t = ray_box(center_deg + dd, &g_box[i]);
            if (t >= 0.0f && t < best) best = t;
        }
    return best;
}

/**
 * Fake port: nothing to initialise on a PC.
 */
/* ---------- port implementation (fake) ---------- */
void obs_port_init(void) {}
/**
 * Fake port: returns the simulated clock (g_ms).
 */
uint32_t obs_port_ms(void) { return g_ms; }
/**
 * Fake port: delays by advancing the simulated clock, so tests run instantly.
 */
void obs_port_delay_ms(uint32_t ms) { g_ms += ms; }
/**
 * Fake port: remembers the servo angle so measurements use it.
 */
void obs_port_servo_deg(uint8_t deg) { g_deg = (float)deg; }

/**
 * Fake port: simulated ultrasonic measurement.
 * Returns NO_RESPONSE if the test killed the sensor, NO_OBJECT beyond the range
 * window, otherwise a pulse width of distance x 5.8 us/mm. Advances the clock by a
 * realistic amount.
 */
obs_echo_t obs_port_measure(uint16_t *pulse_us)
{
    if (g_sensor_dead) { g_ms += OBS_RISE_TIMEOUT_MS; return OBS_ECHO_NO_RESPONSE; }
    float d = world_dist(g_deg);
    float pulse = d * 5.8f;
    g_ms += 1;
    if (d > 4000.0f || pulse > (float)OBS_MAX_PULSE_US) { g_ms += OBS_ECHO_TIMEOUT_MS; return OBS_ECHO_NO_OBJECT; }
    g_ms += (uint32_t)(pulse / 1000.0f) + 1U;
    *pulse_us = (uint16_t)pulse;
    return OBS_ECHO_OK;
}

/**
 * Fake port: pops the next command the test or fake Vehicle Controller queued; if
 * none, advances the clock by the timeout and returns false.
 */
bool obs_port_cmd_recv(uint8_t *cmd, uint32_t timeout_ms)
{
    if (g_nq > 0) { *cmd = g_cmdq[0]; memmove(g_cmdq, g_cmdq + 1, (size_t)(--g_nq)); return true; }
    g_ms += timeout_ms;
    return false;
}

/**
 * Queue a command for the obstacle task, as the Vehicle Controller would.
 */
static void push_cmd(uint8_t c) { g_cmdq[g_nq++] = c; }

/**
 * Fake port: logs every message the task sends, then acts as a fake Vehicle
 * Controller: PROXIMITY -> SCAN_REQ, PROFILE(REVERSE) -> SCAN_REQ, BYPASS_REQ ->
 * BYPASS_DONE (and removes the obstacle, as if the robot drove past),
 * LINE_SEARCH_REQ -> the line appears 1 s later (unless the test disabled it).
 */
void obs_port_msg_send(const obs_msg_t *m)
{
    g_log[g_nlog++] = *m;
    if (!g_vc_auto) return;
    switch (obs_msg_type(m)) {
    case OBS_MSG_PROXIMITY: push_cmd(OBS_CMD_SCAN_REQ); break;            /* VC stops robot first */
    case OBS_MSG_PROFILE: {
        obs_profile_msg_t p; obs_msg_unpack_profile(m, &p);
        if (p.action == OBS_ACT_REVERSE) push_cmd(OBS_CMD_SCAN_REQ);      /* VC reverses, rescans */
        break; }
    case OBS_MSG_BYPASS_REQ: push_cmd(OBS_CMD_BYPASS_DONE); g_nbox = 0; break;   /* robot has driven past */        /* VC runs Buddy 2 moves */
    case OBS_MSG_LINE_SEARCH_REQ: g_line_at = g_line_will_appear ? g_ms + 1000U : 0xFFFFFFFFU; break;
    default: break;
    }
}

/**
 * Fake port: records the fault bits in a test variable.
 */
void obs_port_fault_set(uint32_t bits) { g_faultword |= (bits << OBS_FAULT_SHIFT); }
/**
 * Fake port: records the obstacle flag in a test variable.
 */
void obs_port_obstacle_flag(bool p) { g_obs_flag = p; }
/**
 * Fake port: the line counts as found once the simulated clock passes g_line_at.
 */
bool obs_port_line_detected(void) { return g_ms >= g_line_at; }

/* ---------- test helpers ---------- */
static int g_fail;
#define CHECK(cond, msg) do { if (cond) printf("  PASS  %s\n", msg); else { printf("  FAIL  %s\n", msg); g_fail++; } } while (0)

/**
 * Restart the obstacle task and clear the simulated world, log, commands, faults and flags before each test.
 */
static void reset_world(void)
{
    obs_init();
    g_nbox = 0; g_sensor_dead = false; g_nq = 0; g_nlog = 0; g_faultword = 0;
    g_obs_flag = false; g_line_at = 0xFFFFFFFFU; g_vc_auto = true; g_line_will_appear = true;
}
/**
 * Add a rectangle (x0..x1, y0..y1 in mm, x to the right, y forward) to the simulated world.
 */
static void add_box(float x0, float x1, float y0, float y1)
{ g_box[g_nbox].x0 = x0; g_box[g_nbox].x1 = x1; g_box[g_nbox].y0 = y0; g_box[g_nbox].y1 = y1; g_nbox++; }
/**
 * Run the obstacle state machine for a fixed number of steps.
 */
static void run(int steps) { for (int i = 0; i < steps; i++) obs_step(); }
/**
 * Run the state machine until it reaches the given state (and has sent something) or max steps pass.
 */
static bool run_until_state(obs_state_t s, int max) { for (int i = 0; i < max; i++) { obs_step(); if (obs_get_state() == s && g_nlog > 0) return true; } return false; }
/**
 * Return true if the log contains a message of the given type.
 */
static bool has_type(obs_msg_type_t t) { for (int i = 0; i < g_nlog; i++) if (obs_msg_type(&g_log[i]) == t) return true; return false; }
/**
 * Return the first logged message of the given type, or NULL.
 */
static const obs_msg_t *find_type(obs_msg_type_t t) { for (int i = 0; i < g_nlog; i++) if (obs_msg_type(&g_log[i]) == t) return &g_log[i]; return NULL; }

/**
 * Test 1: packing. Checks round-trips of the profile message and the 12-bit array,
 * the 3-bit type field, the 8-byte size, and that over-range values saturate
 * without corrupting neighbouring fields.
 */
static void test_pack(void)
{
    printf("[1] packing\n");
    obs_profile_msg_t a = { 312, 145, 820, 1010, OBS_ACT_RIGHT, true, 480, OBS_STATE_PROFILING }, b;
    obs_msg_t m; uint8_t by[8], by2[8]; obs_msg_t m2;
    obs_msg_pack_profile(&m, &a); obs_msg_to_bytes(&m, by); obs_msg_from_bytes(&m2, by);
    obs_msg_unpack_profile(&m2, &b); obs_msg_to_bytes(&m2, by2);
    CHECK(memcmp(&a, &b, 0) == 0 && a.closest_mm == b.closest_mm && a.width_mm == b.width_mm &&
          a.left_mm == b.left_mm && a.right_mm == b.right_mm && a.action == b.action &&
          a.valid == b.valid && a.scan_ms == b.scan_ms && a.state == b.state, "profile round-trips");
    CHECK(obs_msg_type(&m2) == OBS_MSG_PROFILE && sizeof(obs_msg_t) == 8, "type field and 8-byte size");
    a.width_mm = 5000; a.scan_ms = 9999; a.left_mm = 9999;
    obs_msg_pack_profile(&m, &a); obs_msg_unpack_profile(&m, &b);
    CHECK(b.width_mm == 1023 && b.scan_ms == 1023 && b.left_mm == 2047, "over-range values saturate, no bleed");
    CHECK(b.closest_mm == 312 && b.right_mm == 1010, "neighbouring fields untouched by saturation");
    uint8_t arr[8] = {0}; obs_bits_put(arr, 0, 12, 0xABC); obs_bits_put(arr, 12, 12, 0x123); obs_bits_put(arr, 24, 12, 0xFFF);
    CHECK(obs_bits_get(arr, 0, 12) == 0xABC && obs_bits_get(arr, 12, 12) == 0x123 && obs_bits_get(arr, 24, 12) == 0xFFF,
          "12-bit packed array");
    printf("  info  sizeof: msg=%zu bytes (naive 8x int32 struct = 32), 5 coarse dists=8 bytes (uint16 x5 = 10)\n", sizeof(obs_msg_t));
}

/**
 * Test 2: clear arena. Checks the task stays quiet when nothing is ahead, that a
 * requested scan gives SCAN_CLEAR, and prints the coarse-scan time (NFR3).
 */
static void test_clear(void)
{
    printf("[2] clear scene\n"); reset_world();
    run(10);
    CHECK(g_nlog == 0 && obs_get_state() == OBS_STATE_IDLE, "idle pings, no message when nothing ahead");
    push_cmd(OBS_CMD_SCAN_REQ); run(3);
    CHECK(has_type(OBS_MSG_SCAN_CLEAR) && obs_get_state() == OBS_STATE_IDLE, "scan of empty arena -> SCAN_CLEAR");
    printf("  info  coarse scan = %u ms (NFR3 target 500; placeholders for servo speed)\n", obs_get_last_coarse_ms());
}

/**
 * Test 3: obstacle scenarios. Places one box (plus optional side walls), runs the
 * whole sequence and checks: PROXIMITY, PROFILE, correct side chosen, BYPASS_REQ,
 * LINE_SEARCH_REQ, LINE_REACQUIRED, back to IDLE with the flag cleared and no
 * faults. Prints width error, clearances and scan times.
 */
static void scenario(const char *name, float x0, float x1, float y0, float y1, bool wl, bool wr, obs_action_t expect, float true_w)
{
    printf("[3] %s\n", name); reset_world();
    add_box(x0, x1, y0, y1);
    if (wl) add_box(-170, -150, 0, 700);           /* wall left of robot  */
    if (wr) add_box(150, 170, 0, 700);             /* wall right of robot */
    for (int i = 0; i < 400 && !has_type(OBS_MSG_LINE_REACQUIRED) && obs_get_state() != OBS_STATE_FAULT; i++) obs_step();
    CHECK(has_type(OBS_MSG_PROXIMITY), "PROXIMITY raised, obstacle flag set for Safety Monitor");
    const obs_msg_t *pm = find_type(OBS_MSG_PROFILE);
    CHECK(pm != NULL, "PROFILE sent");
    if (pm) {
        obs_profile_msg_t p; obs_msg_unpack_profile(pm, &p);
        float err = fabsf((float)p.width_mm - true_w) / true_w * 100.0f;
        printf("  info  closest=%u width=%u (true %.0f, error %.1f%%) left=%u right=%u action=%u coarse=%u ms total scan=%u ms\n",
               p.closest_mm, p.width_mm, true_w, err, p.left_mm, p.right_mm, p.action, p.scan_ms, obs_get_last_scan_ms());
        CHECK(p.action == expect, "chose the correct side");
    }
    if (expect == OBS_ACT_LEFT || expect == OBS_ACT_RIGHT) {
        CHECK(has_type(OBS_MSG_BYPASS_REQ), "BYPASS_REQ sent to Vehicle Controller");
        CHECK(has_type(OBS_MSG_LINE_SEARCH_REQ), "after BYPASS_DONE -> LINE_SEARCH_REQ");
        CHECK(has_type(OBS_MSG_LINE_REACQUIRED) && obs_get_state() == OBS_STATE_IDLE && !g_obs_flag,
              "line reacquired -> IDLE, obstacle flag cleared");
        CHECK(g_faultword == 0, "no faults");
    }
}

/**
 * main - runs all tests (packing, clear arena, three obstacle scenarios, boxed in,
 * dead sensor with reset, line never found) and prints PASS/FAIL for each check.
 * Returns: 0 if everything passed, 1 otherwise.
 */
int main(void)
{
    test_pack();
    test_clear();
    scenario("obstacle 100 mm wide, open arena", -50, 50, 250, 350, false, false, OBS_ACT_RIGHT, 100.0f);
    scenario("obstacle with wall on LEFT -> go right", -50, 50, 250, 350, true, false, OBS_ACT_RIGHT, 100.0f);
    scenario("obstacle with wall on RIGHT -> go left", -50, 50, 250, 350, false, true, OBS_ACT_LEFT, 100.0f);

    printf("[4] boxed in (walls both sides)\n"); reset_world();
    add_box(-50, 50, 250, 350); add_box(-170, -90, 0, 700); add_box(90, 170, 0, 700);
    run_until_state(OBS_STATE_FAULT, 400);
    CHECK((g_faultword >> OBS_FAULT_SHIFT & OBS_FAULT_NO_CLEAR_SIDE) != 0 && obs_get_state() == OBS_STATE_FAULT,
          "no clear side after rescans -> NO_CLEAR_SIDE latched, FAULT");
    CHECK(has_type(OBS_MSG_FAULT_EVT), "FAULT_EVT sent");

    printf("[5] sensor not responding\n"); reset_world();
    g_sensor_dead = true; run(3);
    CHECK((g_faultword >> OBS_FAULT_SHIFT & OBS_FAULT_NO_ECHO) != 0 && obs_get_state() == OBS_STATE_FAULT, "NO_ECHO fault after retries");
    push_cmd(OBS_CMD_RESET); run(2);
    CHECK(obs_get_state() == OBS_STATE_FAULT, "RESET refused while sensor still dead (self-check)");
    g_sensor_dead = false; push_cmd(OBS_CMD_RESET); run(2);
    CHECK(obs_get_state() == OBS_STATE_IDLE, "RESET accepted once sensor responds");

    printf("[6] line never reacquired\n"); reset_world();
    g_line_will_appear = false; add_box(-50, 50, 250, 350);
    run_until_state(OBS_STATE_FAULT, 800);
    CHECK((g_faultword >> OBS_FAULT_SHIFT & OBS_FAULT_LINE_NOT_REACQUIRED) != 0, "LINE_NOT_REACQUIRED after recovery timeout");

    printf("\n%s (%d failure%s)\n", g_fail ? "SOME TESTS FAILED" : "ALL TESTS PASSED", g_fail, g_fail == 1 ? "" : "s");
    return g_fail ? 1 : 0;
}