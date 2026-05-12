/* test_elib_pid.c - PID Controller Unit Tests */
#include <stdio.h>
#include <assert.h>
#include <math.h>
#include <string.h>
#include "../include/elib_pid_defs.h"
#include "../include/elib_pid_pos.h"
#include "../src/elib_pid_util.h"

#define EPSILON 1e-6f

static int tests_run = 0;
static int tests_passed = 0;

#define RUN_TEST(fn) do { \
    printf("Test: %s... ", #fn); \
    tests_run++; \
    fn(); \
    tests_passed++; \
    printf("PASSED\n"); \
} while(0)

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
    elib_pid_pos_err_t err = elib_pid_pos_init(&ctx, &p, -50.0f, 50.0f, 10.0f,
                                                ELIB_PID_POS_ANTI_WINDUP_CLAMP);
    assert(err == ELIB_PID_POS_OK);
    assert(ctx.initialized == 1);
}

static void test_pos_init_null_ctx(void) {
    elib_pid_params_t p = make_pos_params();
    elib_pid_pos_err_t err = elib_pid_pos_init(NULL, &p, -50.0f, 50.0f, 10.0f,
                                                ELIB_PID_POS_ANTI_WINDUP_CLAMP);
    assert(err == ELIB_PID_POS_ERR_INVALID_PARAM);
}

static void test_pos_init_null_params(void) {
    elib_pid_pos_ctx_t ctx;
    elib_pid_pos_err_t err = elib_pid_pos_init(&ctx, NULL, -50.0f, 50.0f, 10.0f,
                                                ELIB_PID_POS_ANTI_WINDUP_CLAMP);
    assert(err == ELIB_PID_POS_ERR_INVALID_PARAM);
}

static void test_pos_init_bad_integral_limits(void) {
    elib_pid_pos_ctx_t ctx;
    elib_pid_params_t p = make_pos_params();
    elib_pid_pos_err_t err = elib_pid_pos_init(&ctx, &p, 50.0f, -50.0f, 10.0f,
                                                ELIB_PID_POS_ANTI_WINDUP_CLAMP);
    assert(err == ELIB_PID_POS_ERR_INVALID_PARAM);
}

static void test_pos_compute_not_initialized(void) {
    elib_pid_pos_ctx_t ctx;
    memset(&ctx, 0, sizeof(ctx));
    elib_pid_val_t output;
    elib_pid_pos_err_t err = elib_pid_pos_compute(&ctx, 10.0f, 5.0f, &output);
    assert(err == ELIB_PID_POS_ERR_NOT_INITIALIZED);
}

static void test_pos_compute_null_output(void) {
    elib_pid_pos_ctx_t ctx;
    elib_pid_params_t p = make_pos_params();
    elib_pid_pos_init(&ctx, &p, -50.0f, 50.0f, 10.0f, ELIB_PID_POS_ANTI_WINDUP_CLAMP);
    elib_pid_pos_err_t err = elib_pid_pos_compute(&ctx, 10.0f, 5.0f, NULL);
    assert(err == ELIB_PID_POS_ERR_INVALID_PARAM);
}

static void test_pos_compute_p_only(void) {
    elib_pid_pos_ctx_t ctx;
    elib_pid_params_t p = make_pos_params();
    p.ki = 0.0f;
    p.kd = 0.0f;
    elib_pid_pos_init(&ctx, &p, -50.0f, 50.0f, 100.0f, 0);

    elib_pid_val_t output;
    elib_pid_pos_err_t err = elib_pid_pos_compute(&ctx, 10.0f, 5.0f, &output);
    assert(err == ELIB_PID_POS_OK);
    /* error = 5.0, output = kp * error = 2.0 * 5.0 = 10.0 */
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
    /* error = 100, kp*error = 200, clamped to 5.0 */
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
    /* error = 1.0, within dead zone of 2.0 */
    elib_pid_pos_compute(&ctx, 10.0f, 9.0f, &output);
    /* dead zone returns 0 when |error| <= dead_zone */
    assert(fabsf(output - 0.0f) < EPSILON);
}

static void test_pos_compute_integral_separation(void) {
    elib_pid_pos_ctx_t ctx;
    elib_pid_params_t p = make_pos_params();
    p.kd = 0.0f;
    /* threshold = 5.0, error = 10.0 > threshold => no integral */
    elib_pid_pos_init(&ctx, &p, -100.0f, 100.0f, 5.0f, 0);

    elib_pid_val_t output;
    elib_pid_pos_compute(&ctx, 100.0f, 90.0f, &output);
    /* error = 10 > threshold 5, integral skipped, output = kp * error = 20 */
    assert(fabsf(output - 20.0f) < EPSILON);
}

static void test_pos_compute_integral_accumulates(void) {
    elib_pid_pos_ctx_t ctx;
    elib_pid_params_t p = make_pos_params();
    p.kd = 0.0f;
    elib_pid_pos_init(&ctx, &p, -100.0f, 100.0f, 100.0f, 0);

    elib_pid_val_t output;
    /* Step 1: error = 10 */
    elib_pid_pos_compute(&ctx, 100.0f, 90.0f, &output);
    /* integral = 10 * 0.01 = 0.1, output = 2*10 + 0.5*0.1 = 20.05 */
    assert(fabsf(output - 20.05f) < EPSILON);

    /* Step 2: same error, integral accumulates */
    elib_pid_pos_compute(&ctx, 100.0f, 90.0f, &output);
    /* integral = 0.1 + 10*0.01 = 0.2, output = 2*10 + 0.5*0.2 = 20.1 */
    assert(fabsf(output - 20.1f) < EPSILON);
}

static void test_pos_compute_anti_windup_clamp(void) {
    elib_pid_pos_ctx_t ctx;
    elib_pid_params_t p = make_pos_params();
    p.kd = 0.0f;
    p.out_max = 100.0f;
    p.out_min = -100.0f;
    elib_pid_pos_init(&ctx, &p, -1.0f, 1.0f, 100.0f,
                       ELIB_PID_POS_ANTI_WINDUP_CLAMP);

    elib_pid_val_t output;
    /* Drive integral hard */
    for (int i = 0; i < 1000; i++) {
        elib_pid_pos_compute(&ctx, 100.0f, 0.0f, &output);
    }
    /* Integral should be clamped to 1.0 */
    assert(ctx.integral <= 1.0f + EPSILON);
}

static void test_pos_compute_anti_windup_condition(void) {
    elib_pid_pos_ctx_t ctx;
    elib_pid_params_t p = make_pos_params();
    p.kd = 0.0f;
    p.out_max = 25.0f;
    p.out_min = -25.0f;
    elib_pid_pos_init(&ctx, &p, -100.0f, 100.0f, 100.0f,
                       ELIB_PID_POS_ANTI_WINDUP_CONDITION);

    elib_pid_val_t output;
    /* Step 1: error = 100, output = kp*100 = 200, clamped to 25 */
    elib_pid_pos_compute(&ctx, 100.0f, 0.0f, &output);
    assert(fabsf(output - 25.0f) < EPSILON);
    elib_pid_val_t integral_after_step1 = ctx.integral;

    /* Step 2: same error, output saturated => conditional integration stops integral */
    elib_pid_pos_compute(&ctx, 100.0f, 0.0f, &output);
    assert(fabsf(output - 25.0f) < EPSILON);
    /* Integral should not have grown because output was saturated */
    assert(fabsf(ctx.integral - integral_after_step1) < EPSILON);
}

static void test_pos_reset(void) {
    elib_pid_pos_ctx_t ctx;
    elib_pid_params_t p = make_pos_params();
    elib_pid_pos_init(&ctx, &p, -50.0f, 50.0f, 100.0f, 0);

    elib_pid_val_t output;
    elib_pid_pos_compute(&ctx, 10.0f, 5.0f, &output);

    elib_pid_pos_err_t err = elib_pid_pos_reset(&ctx);
    assert(err == ELIB_PID_POS_OK);
    assert(fabsf(ctx.integral - 0.0f) < EPSILON);
    assert(fabsf(ctx.prev_error - 0.0f) < EPSILON);
}

static void test_pos_deinit(void) {
    elib_pid_pos_ctx_t ctx;
    elib_pid_params_t p = make_pos_params();
    elib_pid_pos_init(&ctx, &p, -50.0f, 50.0f, 10.0f, 0);

    elib_pid_pos_deinit(&ctx);
    assert(ctx.initialized == 0);

    elib_pid_val_t output;
    elib_pid_pos_err_t err = elib_pid_pos_compute(&ctx, 10.0f, 5.0f, &output);
    assert(err == ELIB_PID_POS_ERR_NOT_INITIALIZED);
}

static elib_pid_val_t test_d_filter_fn(elib_pid_val_t raw_d, elib_pid_val_t dt, void *user_ctx) {
    (void)dt; (void)user_ctx;
    return raw_d * 0.5f;
}

static void test_pos_d_filter(void) {
    elib_pid_pos_ctx_t ctx;
    elib_pid_params_t p = make_pos_params();
    p.ki = 0.0f;
    p.d_filter_fn = test_d_filter_fn;
    elib_pid_pos_init(&ctx, &p, -50.0f, 50.0f, 100.0f, 0);

    elib_pid_val_t output;
    /* error=10, d_raw=(10-0)/0.01=1000, d=500, output=2*10+0.1*500=70 */
    elib_pid_pos_compute(&ctx, 10.0f, 0.0f, &output);
    assert(fabsf(output - 70.0f) < EPSILON);
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

    /* Position PID tests */
    printf("\n--- Position PID ---\n");
    RUN_TEST(test_pos_init_valid);
    RUN_TEST(test_pos_init_null_ctx);
    RUN_TEST(test_pos_init_null_params);
    RUN_TEST(test_pos_init_bad_integral_limits);
    RUN_TEST(test_pos_compute_not_initialized);
    RUN_TEST(test_pos_compute_null_output);
    RUN_TEST(test_pos_compute_p_only);
    RUN_TEST(test_pos_compute_output_clamped);
    RUN_TEST(test_pos_compute_dead_zone);
    RUN_TEST(test_pos_compute_integral_separation);
    RUN_TEST(test_pos_compute_integral_accumulates);
    RUN_TEST(test_pos_compute_anti_windup_clamp);
    RUN_TEST(test_pos_compute_anti_windup_condition);
    RUN_TEST(test_pos_reset);
    RUN_TEST(test_pos_deinit);
    RUN_TEST(test_pos_d_filter);

    printf("\n%d/%d tests passed\n", tests_passed, tests_run);
    return (tests_passed == tests_run) ? 0 : 1;
}
