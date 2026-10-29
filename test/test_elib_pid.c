/* test_elib_pid.c - PID controller unit tests (v2)
 *
 * Numeric expectations are hand-derived from the four-quadrant
 * formulas in the design sheet; the incremental motor-loop regression
 * is additionally cross-checked against a Python mirror of the
 * algorithm (2026-10-29, convergence ±10 confirmed).
 */
#include <stdio.h>
#include <assert.h>
#include <math.h>
#include <string.h>

#include "elib_pid.h"
#include "elib_pid_util.h"

#define EPSILON 1e-6f

static int tests_run = 0;
static int tests_passed = 0;

#define RUN_TEST(fn) do { \
    printf("Test: %s... ", #fn); \
    tests_run++; \
    fn(); \
    tests_passed++; \
    printf("PASSED\n"); \
} while (0)

/* === Utility function tests === */

static void test_clamp_mid(void) {
    elib_pid_val_t result = elib_pid_util_clamp(5.0f, 0.0f, 10.0f);
    assert(fabsf(result - 5.0f) < EPSILON);
}

static void test_clamp_below_min(void) {
    elib_pid_val_t result = elib_pid_util_clamp(-5.0f, 0.0f, 10.0f);
    assert(fabsf(result - 0.0f) < EPSILON);
}

static void test_clamp_above_max(void) {
    elib_pid_val_t result = elib_pid_util_clamp(15.0f, 0.0f, 10.0f);
    assert(fabsf(result - 10.0f) < EPSILON);
}

static void test_clamp_at_boundaries(void) {
    elib_pid_val_t r1 = elib_pid_util_clamp(0.0f, 0.0f, 10.0f);
    elib_pid_val_t r2 = elib_pid_util_clamp(10.0f, 0.0f, 10.0f);
    assert(fabsf(r1 - 0.0f) < EPSILON);
    assert(fabsf(r2 - 10.0f) < EPSILON);
}

static void test_dead_zone_inside(void) {
    elib_pid_val_t result = elib_pid_util_dead_zone(0.5f, 1.0f);
    assert(fabsf(result - 0.0f) < EPSILON);
}

static void test_dead_zone_outside(void) {
    elib_pid_val_t result = elib_pid_util_dead_zone(2.0f, 1.0f);
    assert(fabsf(result - 2.0f) < EPSILON);
}

static void test_dead_zone_negative(void) {
    elib_pid_val_t result = elib_pid_util_dead_zone(-2.0f, 1.0f);
    assert(fabsf(result - (-2.0f)) < EPSILON);
}

static void test_dead_zone_zero_width(void) {
    elib_pid_val_t result = elib_pid_util_dead_zone(0.5f, 0.0f);
    assert(fabsf(result - 0.5f) < EPSILON);
}

static void test_dead_zone_at_boundary(void) {
    elib_pid_val_t result = elib_pid_util_dead_zone(1.0f, 1.0f);
    assert(fabsf(result - 0.0f) < EPSILON);
}

static void test_params_valid_ok(void) {
    elib_pid_params_t p = {
        .kp = 1.0f, .ki = 0.1f, .kd = 0.01f,
        .dt = 0.01f, .dead_zone = 0.0f,
        .out_min = -100.0f, .out_max = 100.0f,
        .d_filter_fn = NULL, .d_filter_ctx = NULL
    };
    assert(elib_pid_util_params_valid(&p) == 1);
}

static void test_params_valid_null(void) {
    assert(elib_pid_util_params_valid(NULL) == 0);
}

static void test_params_valid_bad_dt(void) {
    elib_pid_params_t p = {
        .kp = 1.0f, .ki = 0.1f, .kd = 0.01f,
        .dt = 0.0f, .dead_zone = 0.0f,
        .out_min = -100.0f, .out_max = 100.0f,
        .d_filter_fn = NULL, .d_filter_ctx = NULL
    };
    assert(elib_pid_util_params_valid(&p) == 0);
}

static void test_params_valid_bad_limits(void) {
    elib_pid_params_t p = {
        .kp = 1.0f, .ki = 0.1f, .kd = 0.01f,
        .dt = 0.01f, .dead_zone = 0.0f,
        .out_min = 100.0f, .out_max = -100.0f,
        .d_filter_fn = NULL, .d_filter_ctx = NULL
    };
    assert(elib_pid_util_params_valid(&p) == 0);
}

static void test_params_valid_equal_limits(void) {
    elib_pid_params_t p = {
        .kp = 1.0f, .ki = 0.1f, .kd = 0.01f,
        .dt = 0.01f, .dead_zone = 0.0f,
        .out_min = 100.0f, .out_max = 100.0f,
        .d_filter_fn = NULL, .d_filter_ctx = NULL
    };
    assert(elib_pid_util_params_valid(&p) == 0);
}

static void test_params_valid_negative_dead_zone(void) {
    elib_pid_params_t p = {
        .kp = 1.0f, .ki = 0.1f, .kd = 0.01f,
        .dt = 0.01f, .dead_zone = -1.0f,
        .out_min = -100.0f, .out_max = 100.0f,
        .d_filter_fn = NULL, .d_filter_ctx = NULL
    };
    assert(elib_pid_util_params_valid(&p) == 0);
}

static void test_delta_valid(void) {
    elib_pid_params_t p = {
        .kp = 1.0f, .ki = 0.1f, .kd = 0.01f,
        .dt = 0.01f, .dead_zone = 0.0f,
        .out_min = -100.0f, .out_max = 100.0f,
        .d_filter_fn = NULL, .d_filter_ctx = NULL
    };

    p.delta_min = -500.0f;
    p.delta_max = 500.0f;
    assert(elib_pid_util_delta_valid(&p) == 1);

    p.delta_min = 10.0f; p.delta_max = 5.0f;   /* crossed */
    assert(elib_pid_util_delta_valid(&p) == 0);

    p.delta_min = 0.0f;  p.delta_max = 0.0f;   /* silent zero-output trap */
    assert(elib_pid_util_delta_valid(&p) == 0);
}

/* === Position PID tests === */

static elib_pid_params_t make_pos_params(void) {
    elib_pid_params_t p = {
        .kp = 2.0f, .ki = 0.5f, .kd = 0.1f,
        .dt = 0.01f, .dead_zone = 0.0f,
        .out_min = -100.0f, .out_max = 100.0f,
        .d_filter_fn = NULL, .d_filter_ctx = NULL
    };
    return p;
}

static void test_pos_init_valid(void) {
    elib_pid_pos_ctx_t ctx;
    elib_pid_params_t p = make_pos_params();
    elib_pid_err_t err = elib_pid_pos_init(&ctx, &p, -50.0f, 50.0f, 10.0f,
                                           ELIB_PID_POS_AW_CLAMP);
    assert(err == ELIB_PID_OK);
    assert(ctx.bit_flags.initialized == 1);
    assert(ctx.bit_flags.first_step == 1);
}

static void test_pos_init_null_ctx(void) {
    elib_pid_params_t p = make_pos_params();
    assert(elib_pid_pos_init(NULL, &p, -50.0f, 50.0f, 10.0f,
                             ELIB_PID_POS_AW_CLAMP)
           == ELIB_PID_ERR_INVALID_PARAM);
}

static void test_pos_init_null_params(void) {
    elib_pid_pos_ctx_t ctx;
    assert(elib_pid_pos_init(&ctx, NULL, -50.0f, 50.0f, 10.0f,
                             ELIB_PID_POS_AW_CLAMP)
           == ELIB_PID_ERR_INVALID_PARAM);
}

static void test_pos_init_bad_integral_limits(void) {
    elib_pid_pos_ctx_t ctx;
    elib_pid_params_t p = make_pos_params();
    assert(elib_pid_pos_init(&ctx, &p, 50.0f, -50.0f, 10.0f,
                             ELIB_PID_POS_AW_CLAMP)
           == ELIB_PID_ERR_INVALID_PARAM);
}

static void test_pos_init_bad_sep_threshold(void) {
    elib_pid_pos_ctx_t ctx;
    elib_pid_params_t p = make_pos_params();
    assert(elib_pid_pos_init(&ctx, &p, -50.0f, 50.0f, -1.0f,
                             ELIB_PID_POS_AW_CLAMP)
           == ELIB_PID_ERR_INVALID_PARAM);
}

static void test_pos_init_unknown_aw_bit_refused(void) {
    elib_pid_pos_ctx_t ctx;
    elib_pid_params_t p = make_pos_params();

    /* bit 2 and up are not anti-windup modes: refuse, never truncate */
    assert(elib_pid_pos_init(&ctx, &p, -50.0f, 50.0f, 10.0f, 0x04u)
           == ELIB_PID_ERR_INVALID_PARAM);
    assert(elib_pid_pos_init(&ctx, &p, -50.0f, 50.0f, 10.0f, 0xFFu)
           == ELIB_PID_ERR_INVALID_PARAM);

    /* both valid bits combined */
    assert(elib_pid_pos_init(&ctx, &p, -50.0f, 50.0f, 10.0f,
                             ELIB_PID_POS_AW_CLAMP | ELIB_PID_POS_AW_CONDITION)
           == ELIB_PID_OK);
    assert(ctx.bit_flags.anti_windup_mode == 0x03u);

    /* zero (both modes off) is legal */
    assert(elib_pid_pos_init(&ctx, &p, -50.0f, 50.0f, 10.0f, 0)
           == ELIB_PID_OK);
}

static void test_pos_ignores_delta_fields(void) {
    elib_pid_pos_ctx_t ctx;
    elib_pid_params_t p = make_pos_params();
    /* position mode does not use delta_min/delta_max: untouched and ok */
    p.delta_min = 0.0f;
    p.delta_max = 0.0f;
    assert(elib_pid_pos_init(&ctx, &p, -50.0f, 50.0f, 10.0f, 0)
           == ELIB_PID_OK);
}

static void test_pos_compute_not_initialized(void) {
    elib_pid_pos_ctx_t ctx;
    elib_pid_val_t output;
    (void)memset(&ctx, 0, sizeof(ctx));
    assert(elib_pid_pos_compute(&ctx, 10.0f, 5.0f, &output)
           == ELIB_PID_ERR_NOT_INITIALIZED);
}

static void test_pos_compute_null_output(void) {
    elib_pid_pos_ctx_t ctx;
    elib_pid_params_t p = make_pos_params();
    elib_pid_val_t output;
    elib_pid_pos_init(&ctx, &p, -50.0f, 50.0f, 10.0f, ELIB_PID_POS_AW_CLAMP);
    assert(elib_pid_pos_compute(&ctx, 10.0f, 5.0f, NULL)
           == ELIB_PID_ERR_INVALID_PARAM);
    (void)output;
}

static void test_pos_compute_p_only(void) {
    elib_pid_pos_ctx_t ctx;
    elib_pid_params_t p = make_pos_params();
    p.ki = 0.0f;
    p.kd = 0.0f;
    elib_pid_pos_init(&ctx, &p, -50.0f, 50.0f, 100.0f, 0);

    elib_pid_val_t output;
    assert(elib_pid_pos_compute(&ctx, 10.0f, 5.0f, &output) == ELIB_PID_OK);
    assert(fabsf(output - 10.0f) < EPSILON);
}

static void test_pos_compute_output_clamped(void) {
    elib_pid_pos_ctx_t ctx;
    elib_pid_params_t p = make_pos_params();
    p.ki = 0.0f;
    p.kd = 0.0f;
    p.out_max = 5.0f;
    elib_pid_pos_init(&ctx, &p, -50.0f, 50.0f, 100.0f, 0);

    elib_pid_val_t output;
    elib_pid_pos_compute(&ctx, 100.0f, 0.0f, &output);
    assert(fabsf(output - 5.0f) < EPSILON);
}

static void test_pos_compute_dead_zone(void) {
    elib_pid_pos_ctx_t ctx;
    elib_pid_params_t p = make_pos_params();
    p.ki = 0.0f;
    p.kd = 0.0f;
    p.dead_zone = 2.0f;
    elib_pid_pos_init(&ctx, &p, -50.0f, 50.0f, 100.0f, 0);

    elib_pid_val_t output;
    elib_pid_pos_compute(&ctx, 10.0f, 9.0f, &output);
    assert(fabsf(output - 0.0f) < EPSILON);
}

static void test_pos_compute_integral_separation(void) {
    elib_pid_pos_ctx_t ctx;
    elib_pid_params_t p = make_pos_params();
    p.kd = 0.0f;
    elib_pid_pos_init(&ctx, &p, -100.0f, 100.0f, 5.0f, 0);

    elib_pid_val_t output;
    elib_pid_pos_compute(&ctx, 100.0f, 90.0f, &output);
    /* error = 10 > sep_threshold(5): integral frozen, output = kp*10 */
    assert(fabsf(output - 20.0f) < EPSILON);
}

static void test_pos_compute_integral_accumulates(void) {
    elib_pid_pos_ctx_t ctx;
    elib_pid_params_t p = make_pos_params();
    p.kd = 0.0f;
    elib_pid_pos_init(&ctx, &p, -100.0f, 100.0f, 100.0f, 0);

    elib_pid_val_t output;
    elib_pid_pos_compute(&ctx, 100.0f, 90.0f, &output);
    /* kp*10 + ki*integral; integral = 10*0.01 = 0.1 */
    assert(fabsf(output - 20.05f) < EPSILON);

    elib_pid_pos_compute(&ctx, 100.0f, 90.0f, &output);
    /* integral = 0.2 */
    assert(fabsf(output - 20.1f) < EPSILON);
}

static void test_pos_compute_anti_windup_clamp(void) {
    elib_pid_pos_ctx_t ctx;
    elib_pid_params_t p = make_pos_params();
    p.kd = 0.0f;
    p.out_max = 100.0f;
    p.out_min = -100.0f;
    elib_pid_pos_init(&ctx, &p, -1.0f, 1.0f, 100.0f,
                      ELIB_PID_POS_AW_CLAMP);

    elib_pid_val_t output;
    for (int i = 0; i < 1000; i++) {
        elib_pid_pos_compute(&ctx, 100.0f, 0.0f, &output);
    }
    assert(ctx.integral <= 1.0f + EPSILON);
}

static void test_pos_compute_anti_windup_condition(void) {
    elib_pid_pos_ctx_t ctx;
    elib_pid_params_t p = make_pos_params();
    p.kd = 0.0f;
    p.out_max = 25.0f;
    p.out_min = -25.0f;
    elib_pid_pos_init(&ctx, &p, -100.0f, 100.0f, 100.0f,
                      ELIB_PID_POS_AW_CONDITION);

    elib_pid_val_t output;
    elib_pid_pos_compute(&ctx, 100.0f, 0.0f, &output);
    assert(fabsf(output - 25.0f) < EPSILON);
    elib_pid_val_t integral_after_step1 = ctx.integral;

    elib_pid_pos_compute(&ctx, 100.0f, 0.0f, &output);
    assert(fabsf(output - 25.0f) < EPSILON);
    /* second step could not integrate: rolled back */
    assert(fabsf(ctx.integral - integral_after_step1) < EPSILON);
}

static void test_pos_reset(void) {
    elib_pid_pos_ctx_t ctx;
    elib_pid_params_t p = make_pos_params();
    elib_pid_pos_init(&ctx, &p, -50.0f, 50.0f, 100.0f, 0);

    elib_pid_val_t output;
    elib_pid_pos_compute(&ctx, 10.0f, 5.0f, &output);

    assert(elib_pid_pos_reset(&ctx) == ELIB_PID_OK);
    assert(fabsf(ctx.integral - 0.0f) < EPSILON);
    assert(fabsf(ctx.prev_error - 0.0f) < EPSILON);
    assert(ctx.bit_flags.first_step == 1);
}

static void test_pos_deinit(void) {
    elib_pid_pos_ctx_t ctx;
    elib_pid_params_t p = make_pos_params();
    elib_pid_pos_init(&ctx, &p, -50.0f, 50.0f, 10.0f, 0);

    elib_pid_pos_deinit(&ctx);
    assert(ctx.bit_flags.initialized == 0);

    elib_pid_val_t output;
    assert(elib_pid_pos_compute(&ctx, 10.0f, 5.0f, &output)
           == ELIB_PID_ERR_NOT_INITIALIZED);
}

/* ---- derivative placement (v2 centerpiece) ---- */

static int filter_calls;

static elib_pid_val_t counting_d_filter_fn(elib_pid_val_t raw_d,
                                           elib_pid_val_t dt,
                                           void *user_ctx) {
    (void)dt;
    (void)user_ctx;
    filter_calls++;
    return raw_d * 0.5f;
}

/* position, derivative on ERROR: a setpoint jump differentiates
 * straight into the D term */
static void test_pos_d_on_error_kick(void) {
    elib_pid_pos_ctx_t ctx;
    elib_pid_params_t p = make_pos_params();
    p.ki = 0.0f;
    p.kd = 0.1f;
    p.out_min = -200.0f;   /* wide bounds: the raw kick value must be
                               visible un-clamped */
    p.out_max = 200.0f;
    p.d_on_meas = 0u;              /* classic form */
    elib_pid_pos_init(&ctx, &p, -100.0f, 100.0f, 100.0f, 0);

    elib_pid_val_t output;
    /* step 1: seeding - D is zero even in error form, no startup kick */
    elib_pid_pos_compute(&ctx, 10.0f, 0.0f, &output);
    assert(fabsf(output - 20.0f) < EPSILON);

    /* step 2: setpoint jumps 10 -> 20, measurement still 0
     * e goes 10 -> 20, d_raw = (20-10)/0.01 = 1000, D = 100 */
    elib_pid_pos_compute(&ctx, 20.0f, 0.0f, &output);
    assert(fabsf(output - 140.0f) < EPSILON);
}

/* position, derivative on MEASUREMENT ("derivative first"): the same
 * setpoint jump does NOT touch the D term */
static void test_pos_d_on_meas_no_setpoint_kick(void) {
    elib_pid_pos_ctx_t ctx;
    elib_pid_params_t p = make_pos_params();
    p.ki = 0.0f;
    p.kd = 0.1f;
    p.d_on_meas = 1u;
    elib_pid_pos_init(&ctx, &p, -100.0f, 100.0f, 100.0f, 0);

    elib_pid_val_t output;
    /* step 1: seeding - D zero */
    elib_pid_pos_compute(&ctx, 10.0f, 0.0f, &output);
    assert(fabsf(output - 20.0f) < EPSILON);

    /* step 2: setpoint jumps 10 -> 20, measurement unchanged: the
     * measurement history does not move, D stays 0, output is pure P */
    elib_pid_pos_compute(&ctx, 20.0f, 0.0f, &output);
    assert(fabsf(output - 40.0f) < EPSILON);

    /* step 3: measurement jumps 0 -> 5: NOW the D term reacts
     * d_raw = (0 - 5)/0.01 = -500, D = -50; e = 15 => P = 30 */
    elib_pid_pos_compute(&ctx, 20.0f, 5.0f, &output);
    assert(fabsf(output - (-20.0f)) < EPSILON);
}

static void test_pos_reset_rearms_seeding(void) {
    elib_pid_pos_ctx_t ctx;
    elib_pid_params_t p = make_pos_params();
    p.ki = 0.0f;
    p.kd = 0.1f;
    p.d_on_meas = 0u;
    elib_pid_pos_init(&ctx, &p, -100.0f, 100.0f, 100.0f, 0);

    elib_pid_val_t output;
    elib_pid_pos_compute(&ctx, 10.0f, 0.0f, &output);   /* seeding */
    elib_pid_pos_compute(&ctx, 10.0f, 5.0f, &output);   /* D active */

    assert(elib_pid_pos_reset(&ctx) == ELIB_PID_OK);
    /* seeding again: D zero on the first step after reset */
    elib_pid_pos_compute(&ctx, 10.0f, 0.0f, &output);
    assert(fabsf(output - 20.0f) < EPSILON);
}

static void test_pos_d_filter(void) {
    elib_pid_pos_ctx_t ctx;
    elib_pid_params_t p = make_pos_params();
    p.ki = 0.0f;
    p.kd = 0.1f;
    p.d_on_meas = 0u;
    p.d_filter_fn = counting_d_filter_fn;
    filter_calls = 0;
    elib_pid_pos_init(&ctx, &p, -100.0f, 100.0f, 100.0f, 0);

    elib_pid_val_t output;
    /* seeding step: D = 0 AND the filter is never called (its state
     * must not be dragged by a zero-in kick) */
    elib_pid_pos_compute(&ctx, 10.0f, 0.0f, &output);
    assert(fabsf(output - 20.0f) < EPSILON);
    assert(filter_calls == 0);

    /* second step, measurement moves 0 -> 5:
     * e goes 10 -> 5, d_raw = (5-10)/0.01 = -500, filtered -250,
     * D = -25, P = 10 */
    elib_pid_pos_compute(&ctx, 10.0f, 5.0f, &output);
    assert(fabsf(output - (-15.0f)) < EPSILON);
    assert(filter_calls == 1);

    /* reset: seeding again, filter still untouched */
    assert(elib_pid_pos_reset(&ctx) == ELIB_PID_OK);
    elib_pid_pos_compute(&ctx, 10.0f, 0.0f, &output);
    assert(fabsf(output - 20.0f) < EPSILON);
    assert(filter_calls == 1);
}

static void test_pos_set_params(void) {
    elib_pid_pos_ctx_t ctx;
    elib_pid_params_t p = make_pos_params();
    p.ki = 0.0f;
    p.kd = 0.0f;
    elib_pid_pos_init(&ctx, &p, -50.0f, 50.0f, 100.0f, 0);

    elib_pid_val_t output;
    elib_pid_pos_compute(&ctx, 10.0f, 5.0f, &output);
    /* kp=2, error=5, output=10 */
    assert(fabsf(output - 10.0f) < EPSILON);

    /* change kp at runtime */
    elib_pid_params_t new_p = p;
    new_p.kp = 4.0f;
    assert(elib_pid_pos_set_params(&ctx, &new_p) == ELIB_PID_OK);

    elib_pid_pos_compute(&ctx, 10.0f, 5.0f, &output);
    /* kp=4, error=5, output=20 */
    assert(fabsf(output - 20.0f) < EPSILON);
}

static void test_pos_set_params_not_initialized(void) {
    elib_pid_pos_ctx_t ctx;
    (void)memset(&ctx, 0, sizeof(ctx));
    elib_pid_params_t p = make_pos_params();
    assert(elib_pid_pos_set_params(&ctx, &p)
           == ELIB_PID_ERR_NOT_INITIALIZED);
}

/* === Incremental PID tests === */

static elib_pid_params_t make_inc_params(void) {
    elib_pid_params_t p = {
        .kp = 2.0f, .ki = 0.5f, .kd = 0.1f,
        .dt = 0.01f, .dead_zone = 0.0f,
        .out_min = -100.0f, .out_max = 100.0f,
        .delta_min = -1000.0f, .delta_max = 1000.0f, /* explicit, never
                                                         both zero */
        .d_filter_fn = NULL, .d_filter_ctx = NULL
    };
    return p;
}

static void test_inc_init_valid(void) {
    elib_pid_inc_ctx_t ctx;
    elib_pid_params_t p = make_inc_params();
    assert(elib_pid_inc_init(&ctx, &p) == ELIB_PID_OK);
    assert(ctx.bit_flags.initialized == 1);
    assert(ctx.bit_flags.first_step == 1);
}

static void test_inc_init_null_ctx(void) {
    elib_pid_params_t p = make_inc_params();
    assert(elib_pid_inc_init(NULL, &p) == ELIB_PID_ERR_INVALID_PARAM);
}

static void test_inc_init_null_params(void) {
    elib_pid_inc_ctx_t ctx;
    assert(elib_pid_inc_init(&ctx, NULL) == ELIB_PID_ERR_INVALID_PARAM);
}

/* both-zero delta limits: the classic silent zero-output trap */
static void test_inc_delta_both_zero_refused(void) {
    elib_pid_inc_ctx_t ctx;
    elib_pid_params_t p = make_inc_params();

    p.delta_min = 0.0f;
    p.delta_max = 0.0f;
    assert(elib_pid_inc_init(&ctx, &p) == ELIB_PID_ERR_INVALID_PARAM);

    /* and at runtime update too */
    {
        elib_pid_params_t base = make_inc_params();
        assert(elib_pid_inc_init(&ctx, &base) == ELIB_PID_OK);
        assert(elib_pid_inc_set_params(&ctx, &p)
               == ELIB_PID_ERR_INVALID_PARAM);
    }
}

static void test_inc_delta_crossed_refused_at_runtime(void) {
    elib_pid_inc_ctx_t ctx;
    elib_pid_params_t p = make_inc_params();
    assert(elib_pid_inc_init(&ctx, &p) == ELIB_PID_OK);

    p.delta_min = 10.0f;
    p.delta_max = -10.0f;
    assert(elib_pid_inc_set_params(&ctx, &p)
           == ELIB_PID_ERR_INVALID_PARAM);
}

static void test_inc_compute_not_initialized(void) {
    elib_pid_inc_ctx_t ctx;
    elib_pid_val_t output;
    (void)memset(&ctx, 0, sizeof(ctx));
    assert(elib_pid_inc_compute(&ctx, 10.0f, 5.0f, &output)
           == ELIB_PID_ERR_NOT_INITIALIZED);
}

static void test_inc_compute_null_output(void) {
    elib_pid_inc_ctx_t ctx;
    elib_pid_params_t p = make_inc_params();
    elib_pid_inc_init(&ctx, &p);
    assert(elib_pid_inc_compute(&ctx, 10.0f, 5.0f, NULL)
           == ELIB_PID_ERR_INVALID_PARAM);
}

static void test_inc_compute_first_step(void) {
    elib_pid_inc_ctx_t ctx;
    elib_pid_params_t p = make_inc_params();
    p.kd = 0.0f;
    elib_pid_inc_init(&ctx, &p);

    elib_pid_val_t output;
    assert(elib_pid_inc_compute(&ctx, 10.0f, 5.0f, &output) == ELIB_PID_OK);
    /* error=5, prev_error=0, prev2_error=0
     * delta_P = 2*(5-0) = 10
     * delta_I = 0.5*5*0.01 = 0.025
     * delta_D = 0 (seeding AND kd=0)
     * output = 0 + 10 + 0.025 = 10.025 */
    assert(fabsf(output - 10.025f) < EPSILON);
}

static void test_inc_compute_output_clamped(void) {
    elib_pid_inc_ctx_t ctx;
    elib_pid_params_t p = make_inc_params();
    p.kd = 0.0f;
    p.out_max = 5.0f;
    elib_pid_inc_init(&ctx, &p);

    elib_pid_val_t output;
    elib_pid_inc_compute(&ctx, 100.0f, 0.0f, &output);
    assert(fabsf(output - 5.0f) < EPSILON);
}

static void test_inc_compute_delta_clamped(void) {
    elib_pid_inc_ctx_t ctx;
    elib_pid_params_t p = make_inc_params();
    p.ki = 0.0f;
    p.kd = 0.0f;
    p.delta_min = -5.0f;
    p.delta_max = 5.0f;
    elib_pid_inc_init(&ctx, &p);

    elib_pid_val_t output;
    /* delta_P would be 2*10 = 20, clamped to +5 */
    elib_pid_inc_compute(&ctx, 10.0f, 0.0f, &output);
    assert(fabsf(output - 5.0f) < EPSILON);

    /* setpoint collapses: delta_P = 2*(0-10) = -20, clamped to -5
     * -> out = 0 */
    elib_pid_inc_compute(&ctx, 0.0f, 0.0f, &output);
    assert(fabsf(output - 0.0f) < EPSILON);
}

static void test_inc_compute_dead_zone(void) {
    elib_pid_inc_ctx_t ctx;
    elib_pid_params_t p = make_inc_params();
    p.kd = 0.0f;
    p.dead_zone = 2.0f;
    elib_pid_inc_init(&ctx, &p);

    elib_pid_val_t output;
    /* error=1.0, within dead_zone=2.0 => error becomes 0 => delta_u=0
     * => output=0 */
    elib_pid_inc_compute(&ctx, 10.0f, 9.0f, &output);
    assert(fabsf(output - 0.0f) < EPSILON);
}

static void test_inc_compute_natural_anti_windup(void) {
    elib_pid_inc_ctx_t ctx;
    elib_pid_params_t p = make_inc_params();
    p.kd = 0.0f;
    p.out_max = 10.0f;
    p.out_min = -10.0f;
    elib_pid_inc_init(&ctx, &p);

    elib_pid_val_t output;
    /* Run many steps with large error - output should stay clamped */
    for (int i = 0; i < 1000; i++) {
        elib_pid_inc_compute(&ctx, 100.0f, 0.0f, &output);
    }
    assert(output <= 10.0f + EPSILON);
    /* Incremental PID naturally anti-winds up because prev_output is
     * the clamped value */
}

static void test_inc_reset(void) {
    elib_pid_inc_ctx_t ctx;
    elib_pid_params_t p = make_inc_params();
    elib_pid_inc_init(&ctx, &p);

    elib_pid_val_t output;
    elib_pid_inc_compute(&ctx, 10.0f, 5.0f, &output);

    assert(elib_pid_inc_reset(&ctx) == ELIB_PID_OK);
    assert(fabsf(ctx.prev_error - 0.0f) < EPSILON);
    assert(fabsf(ctx.prev2_error - 0.0f) < EPSILON);
    assert(fabsf(ctx.prev_output - 0.0f) < EPSILON);
    assert(fabsf(ctx.prev_measurement - 0.0f) < EPSILON);
    assert(fabsf(ctx.prev2_measurement - 0.0f) < EPSILON);
    assert(ctx.bit_flags.first_step == 1);
}

static void test_inc_deinit(void) {
    elib_pid_inc_ctx_t ctx;
    elib_pid_params_t p = make_inc_params();
    elib_pid_inc_init(&ctx, &p);

    elib_pid_inc_deinit(&ctx);
    assert(ctx.bit_flags.initialized == 0);

    elib_pid_val_t output;
    assert(elib_pid_inc_compute(&ctx, 10.0f, 5.0f, &output)
           == ELIB_PID_ERR_NOT_INITIALIZED);
}

/* incremental, derivative on MEASUREMENT */
static void test_inc_d_on_meas(void) {
    elib_pid_inc_ctx_t ctx;
    elib_pid_params_t p = make_inc_params();
    p.ki = 0.0f;
    p.kd = 0.1f;
    p.d_on_meas = 1u;
    elib_pid_inc_init(&ctx, &p);

    elib_pid_val_t output;
    /* step 1: seeding - D zero, delta_P = 2*(10-0) = 20 */
    elib_pid_inc_compute(&ctx, 10.0f, 0.0f, &output);
    assert(fabsf(output - 20.0f) < EPSILON);

    /* step 2: setpoint jumps 10 -> 20, measurement still 0:
     * d_raw = (2*0 - 0 - 0)/dt = 0, delta = P term only (20) */
    elib_pid_inc_compute(&ctx, 20.0f, 0.0f, &output);
    assert(fabsf(output - 40.0f) < EPSILON);

    /* step 3: measurement jumps 0 -> 5:
     * e = 15, d_raw = (2*0 - 5 - 0)/0.01 = -500, D = -50
     * delta_P = 2*(15-20) = -10, delta = -60 => out = -20 */
    elib_pid_inc_compute(&ctx, 20.0f, 5.0f, &output);
    assert(fabsf(output - (-20.0f)) < EPSILON);
}

static void test_inc_d_filter(void) {
    elib_pid_inc_ctx_t ctx;
    elib_pid_params_t p = make_inc_params();
    p.ki = 0.0f;
    p.kd = 0.1f;
    p.d_on_meas = 0u;
    p.d_filter_fn = counting_d_filter_fn;
    filter_calls = 0;
    elib_pid_inc_init(&ctx, &p);

    elib_pid_val_t output;
    /* seeding: delta_P = 20, D = 0, filter not called */
    elib_pid_inc_compute(&ctx, 10.0f, 0.0f, &output);
    assert(fabsf(output - 20.0f) < EPSILON);
    assert(filter_calls == 0);

    /* measurement moves 0 -> 5: e goes 10 -> 5
     * d_raw = (5 - 2*10 + 0)/0.01 = -1500, filtered -750, D = -75
     * delta_P = 2*(5-10) = -10, delta = -85, out = 20-85 = -65 */
    elib_pid_inc_compute(&ctx, 10.0f, 5.0f, &output);
    assert(fabsf(output - (-65.0f)) < EPSILON);
    assert(filter_calls == 1);

    /* reset re-arms seeding, filter untouched */
    assert(elib_pid_inc_reset(&ctx) == ELIB_PID_OK);
    elib_pid_inc_compute(&ctx, 10.0f, 0.0f, &output);
    assert(fabsf(output - 20.0f) < EPSILON);
    assert(filter_calls == 1);
}

static void test_inc_set_params(void) {
    elib_pid_inc_ctx_t ctx;
    elib_pid_params_t p = make_inc_params();
    p.kd = 0.0f;
    elib_pid_inc_init(&ctx, &p);

    elib_pid_val_t output;
    elib_pid_inc_compute(&ctx, 10.0f, 5.0f, &output);
    elib_pid_val_t out1 = output;

    /* change kp at runtime */
    elib_pid_params_t new_p = p;
    new_p.kp = 4.0f;
    assert(elib_pid_inc_set_params(&ctx, &new_p) == ELIB_PID_OK);

    elib_pid_inc_compute(&ctx, 10.0f, 5.0f, &output);
    /* output should differ from first call due to new kp */
    assert(output != out1);
}

static void test_inc_set_params_not_initialized(void) {
    elib_pid_inc_ctx_t ctx;
    (void)memset(&ctx, 0, sizeof(ctx));
    elib_pid_params_t p = make_inc_params();
    assert(elib_pid_inc_set_params(&ctx, &p)
           == ELIB_PID_ERR_NOT_INITIALIZED);
}

/*
 * Motor speed control scenario (closed-loop regression; parameter
 * set found and verified with the Python mirror - the old v1 numbers
 * (kp=2, ki=0.01, kd=0.5, dead_zone=0) LIMIT-CYCLE around error 836
 * and can never reach +/-10):
 * - PWM range: 0-4000 (0-100% duty cycle)
 * - Control period: 100ms (dt = 0.1)
 * - Max PWM step per cycle: 500 (delta limits = slew rate)
 * - Derivative on measurement, gate dead zone ±10
 * - Motor model: speed += pwm*0.1 - speed*0.05
 */
static void test_inc_motor_speed_control(void) {
    elib_pid_inc_ctx_t ctx;
    elib_pid_params_t params = {
        .kp = 1.0f,
        .ki = 1.0f,
        .kd = 0.1f,
        .dt = 0.1f,
        .dead_zone = 10.0f,
        .out_min = 0.0f,
        .out_max = 4000.0f,
        .delta_min = -500.0f,
        .delta_max = 500.0f,
        .d_on_meas = 1u,        /* engineering default */
        .d_filter_fn = NULL,
        .d_filter_ctx = NULL,
    };
    elib_pid_inc_init(&ctx, &params);

    elib_pid_val_t setpoint = 2000.0f;
    elib_pid_val_t speed = 0.0f;
    elib_pid_val_t pwm = 0.0f;

    for (int i = 0; i < 2000; i++) {
        elib_pid_val_t pid_out;
        elib_pid_inc_compute(&ctx, setpoint, speed, &pid_out);

        /* slew rate limit on top (application-side, like the README
         * example) */
        elib_pid_val_t delta = pid_out - pwm;
        if (delta > 500.0f) delta = 500.0f;
        if (delta < -500.0f) delta = -500.0f;
        pwm += delta;

        /* motor model: first-order response */
        speed += pwm * 0.1f - speed * 0.05f;
    }

    elib_pid_val_t err = setpoint - speed;
    assert(fabsf(err) <= 10.0f);
}

int main(void) {
    printf("=== elib-pid tests ===\n\n");

    /* Utility tests */
    printf("--- Utility functions ---\n");
    RUN_TEST(test_clamp_mid);
    RUN_TEST(test_clamp_below_min);
    RUN_TEST(test_clamp_above_max);
    RUN_TEST(test_clamp_at_boundaries);
    RUN_TEST(test_dead_zone_inside);
    RUN_TEST(test_dead_zone_outside);
    RUN_TEST(test_dead_zone_negative);
    RUN_TEST(test_dead_zone_zero_width);
    RUN_TEST(test_dead_zone_at_boundary);
    RUN_TEST(test_params_valid_ok);
    RUN_TEST(test_params_valid_null);
    RUN_TEST(test_params_valid_bad_dt);
    RUN_TEST(test_params_valid_bad_limits);
    RUN_TEST(test_params_valid_equal_limits);
    RUN_TEST(test_params_valid_negative_dead_zone);
    RUN_TEST(test_delta_valid);

    /* Position PID tests */
    printf("\n--- Position PID ---\n");
    RUN_TEST(test_pos_init_valid);
    RUN_TEST(test_pos_init_null_ctx);
    RUN_TEST(test_pos_init_null_params);
    RUN_TEST(test_pos_init_bad_integral_limits);
    RUN_TEST(test_pos_init_bad_sep_threshold);
    RUN_TEST(test_pos_init_unknown_aw_bit_refused);
    RUN_TEST(test_pos_ignores_delta_fields);
    RUN_TEST(test_pos_compute_not_initialized);
    RUN_TEST(test_pos_compute_null_output);
    RUN_TEST(test_pos_compute_p_only);
    RUN_TEST(test_pos_compute_output_clamped);
    RUN_TEST(test_pos_compute_dead_zone);
    RUN_TEST(test_pos_compute_integral_separation);
    RUN_TEST(test_pos_compute_integral_accumulates);
    RUN_TEST(test_pos_compute_anti_windup_clamp);
    RUN_TEST(test_pos_compute_anti_windup_condition);
    RUN_TEST(test_pos_d_on_error_kick);
    RUN_TEST(test_pos_d_on_meas_no_setpoint_kick);
    RUN_TEST(test_pos_reset);
    RUN_TEST(test_pos_reset_rearms_seeding);
    RUN_TEST(test_pos_deinit);
    RUN_TEST(test_pos_d_filter);
    RUN_TEST(test_pos_set_params);
    RUN_TEST(test_pos_set_params_not_initialized);

    /* Incremental PID tests */
    printf("\n--- Incremental PID ---\n");
    RUN_TEST(test_inc_init_valid);
    RUN_TEST(test_inc_init_null_ctx);
    RUN_TEST(test_inc_init_null_params);
    RUN_TEST(test_inc_delta_both_zero_refused);
    RUN_TEST(test_inc_delta_crossed_refused_at_runtime);
    RUN_TEST(test_inc_compute_not_initialized);
    RUN_TEST(test_inc_compute_null_output);
    RUN_TEST(test_inc_compute_first_step);
    RUN_TEST(test_inc_compute_output_clamped);
    RUN_TEST(test_inc_compute_delta_clamped);
    RUN_TEST(test_inc_compute_dead_zone);
    RUN_TEST(test_inc_compute_natural_anti_windup);
    RUN_TEST(test_inc_d_on_meas);
    RUN_TEST(test_inc_reset);
    RUN_TEST(test_inc_deinit);
    RUN_TEST(test_inc_d_filter);
    RUN_TEST(test_inc_set_params);
    RUN_TEST(test_inc_set_params_not_initialized);
    RUN_TEST(test_inc_motor_speed_control);

    printf("\n%d/%d tests passed\n", tests_passed, tests_run);
    return (tests_passed == tests_run) ? 0 : 1;
}
