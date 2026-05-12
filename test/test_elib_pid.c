/* test_elib_pid.c - PID Controller Unit Tests */
#include <stdio.h>
#include <assert.h>
#include <math.h>
#include "../include/elib_pid_defs.h"
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

    printf("\n%d/%d tests passed\n", tests_passed, tests_run);
    return (tests_passed == tests_run) ? 0 : 1;
}
