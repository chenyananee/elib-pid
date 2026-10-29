/* elib_pid.h - PID controller (position & incremental), single umbrella
 *
 * Rules of the game:
 *  - zero dynamic memory: every controller context is user allocated
 *    and self contained; no globals anywhere in the library
 *  - both modes share one parameter structure (copied in at init and
 *    on runtime update); the derivative can run on the error
 *    (classic form) or on the measurement ("derivative first",
 *    no setpoint kick) via d_on_meas - the measurement form is the
 *    engineering default worth picking for moving setpoints
 *  - the first step after init/reset is a seeding step: the derivative
 *    term evaluates to zero (no imaginary history, no startup impulse)
 *    and the error/measurement history is only recorded from there on
 *  - gates on inputs: every documented invalid configuration is
 *    rejected at init AND at runtime parameter update - the library
 *    never silently interpolates a nonsense parameter into defined
 *    behavior
 *  - value type is configurable (ELIB_PID_VAL_T, default float);
 *    only the four basic arithmetic operations are used, so fixed
 *    point types need no function-call shims
 */
#ifndef ELIB_PID_H
#define ELIB_PID_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ELIB_PID_VERSION "2.0.0"

/* ---- value type configuration ----
 *
 * Auto-completes when undefined to float. Override with -D for your
 * own type (fixed point, double, ...) as long as it supports the four
 * basic arithmetic operations and comparisons. */
#ifndef ELIB_PID_VAL_T
#define ELIB_PID_VAL_T float
#endif

typedef ELIB_PID_VAL_T elib_pid_val_t;

/* ---- error codes (shared by both modes) ---- */

typedef enum {
    ELIB_PID_OK = 0,
    ELIB_PID_ERR_INVALID_PARAM,   /* init/update rejected bad arguments */
    ELIB_PID_ERR_NOT_INITIALIZED  /* API called before a valid init */
} elib_pid_err_t;

/* ---- derivative filter callback (incomplete differentiation) ----
 *
 * Receives the RAW derivative (before the kd multiplication), the
 * control period, and the user context; returns the filtered value.
 * Called once per step while kd != 0 and history exists (never on the
 * seeding step, so the filter state is not disturbed by a zero-in
 * kick). NULL = no filtering. */
typedef elib_pid_val_t (*elib_pid_d_filter_fn)(
    elib_pid_val_t raw_d,
    elib_pid_val_t dt,
    void *user_ctx
);

/* ---- shared parameters (both modes) ----
 *
 * Copied into the controller context at init and on set_params, so the
 * structure may live on the caller's stack.
 *
 * delta_min/delta_max only apply to the INCREMENTAL controller: they
 * clamp each increment. Both zero is REJECTED (silent zero-output
 * trap); to disable the rate limit use explicit extreme values.
 * Position mode ignores them.
 *
 * d_on_meas nonzero selects derivative on measurement; zero selects
 * derivative on the error. */
typedef struct {
    elib_pid_val_t kp;                  /* Proportional gain */
    elib_pid_val_t ki;                  /* Integral gain */
    elib_pid_val_t kd;                  /* Derivative gain */
    elib_pid_val_t dt;                  /* Time step, must be > 0 */
    elib_pid_val_t dead_zone;           /* Dead zone width (>= 0, symmetric) */
    elib_pid_val_t out_min;             /* Output lower limit */
    elib_pid_val_t out_max;             /* Output upper limit, must be > out_min */
    elib_pid_val_t delta_min;           /* Incremental PID: increment lower
                                             limit; not both-zero with
                                             delta_max */
    elib_pid_val_t delta_max;           /* Incremental PID: increment upper
                                             limit */
    uint8_t        d_on_meas;           /* nonzero = derivative on
                                             measurement (no setpoint kick) */
    elib_pid_d_filter_fn d_filter_fn;   /* Derivative filter callback,
                                             NULL = no filter */
    void *d_filter_ctx;                 /* Derivative filter user context */
} elib_pid_params_t;

/* ================================================================
 * Position PID
 * ================================================================ */

/* Anti-windup mode (bitmask, combinable; exactly these two bits) */
typedef enum {
    ELIB_PID_POS_AW_CLAMP     = (1 << 0),  /* Integral clamping */
    ELIB_PID_POS_AW_CONDITION = (1 << 1),  /* Conditional integration */
} elib_pid_pos_anti_windup_t;

/* Position PID context (statically allocated by user) */
typedef struct {
    elib_pid_params_t params;
    elib_pid_val_t integral;                /* Integral accumulator */
    elib_pid_val_t integral_min;            /* Integral clamp lower limit */
    elib_pid_val_t integral_max;            /* Integral clamp upper limit */
    elib_pid_val_t integral_sep_threshold;  /* Integral separation threshold */
    elib_pid_val_t prev_error;              /* Previous (dead-zone) error */
    elib_pid_val_t prev_measurement;        /* Previous measurement
                                                 (derivative on measurement) */
    struct {
        uint8_t anti_windup_mode : 2; /* Validated anti-windup bitmask */
        uint8_t initialized      : 1;
        uint8_t first_step       : 1; /* Seeding step pending */
    } bit_flags;
} elib_pid_pos_ctx_t;

/**
 * @brief Initialize position PID controller
 * @param ctx User-allocated context pointer
 * @param params PID parameters (copied into context)
 * @param integral_min Integral clamp lower limit
 * @param integral_max Integral clamp upper limit
 * @param integral_sep_threshold Integral separation threshold (>= 0)
 * @param anti_windup_mode Anti-windup bitmask (CLAMP|CONDITION, 0 = off)
 * @return elib_pid_err_t error code
 */
elib_pid_err_t elib_pid_pos_init(elib_pid_pos_ctx_t *ctx,
                                 const elib_pid_params_t *params,
                                 elib_pid_val_t integral_min,
                                 elib_pid_val_t integral_max,
                                 elib_pid_val_t integral_sep_threshold,
                                 uint8_t anti_windup_mode);

/**
 * @brief Deinitialize position PID controller
 * @param ctx Context pointer
 */
void elib_pid_pos_deinit(elib_pid_pos_ctx_t *ctx);

/**
 * @brief Reset internal state (integral, error/measurement history,
 *        re-arm the seeding step)
 * @param ctx Context pointer
 * @return elib_pid_err_t error code
 */
elib_pid_err_t elib_pid_pos_reset(elib_pid_pos_ctx_t *ctx);

/**
 * @brief Update PID parameters at runtime (e.g. for fuzzy PID tuning)
 * @param ctx Context pointer
 * @param params New PID parameters (copied into context)
 * @return elib_pid_err_t error code
 */
elib_pid_err_t elib_pid_pos_set_params(elib_pid_pos_ctx_t *ctx,
                                       const elib_pid_params_t *params);

/**
 * @brief Compute one step of position PID
 * @param ctx Context pointer
 * @param setpoint Target value
 * @param measurement Current value
 * @param output Output pointer
 * @return elib_pid_err_t error code
 */
elib_pid_err_t elib_pid_pos_compute(elib_pid_pos_ctx_t *ctx,
                                    elib_pid_val_t setpoint,
                                    elib_pid_val_t measurement,
                                    elib_pid_val_t *output);

/* ================================================================
 * Incremental PID
 * ================================================================ */

/* Incremental PID context (statically allocated by user) */
typedef struct {
    elib_pid_params_t params;
    elib_pid_val_t prev_error;        /* e(k-1)                             */
    elib_pid_val_t prev2_error;       /* e(k-2), derivative on error        */
    elib_pid_val_t prev_output;       /* u(k-1)                             */
    elib_pid_val_t prev_measurement;  /* m(k-1), derivative on measurement  */
    elib_pid_val_t prev2_measurement; /* m(k-2), derivative on measurement  */
    struct {
        uint8_t initialized : 1;
        uint8_t first_step  : 1;      /* Seeding step pending */
    } bit_flags;
} elib_pid_inc_ctx_t;

/**
 * @brief Initialize incremental PID controller
 * @param ctx User-allocated context pointer
 * @param params PID parameters (copied into context)
 * @return elib_pid_err_t error code
 */
elib_pid_err_t elib_pid_inc_init(elib_pid_inc_ctx_t *ctx,
                                 const elib_pid_params_t *params);

/**
 * @brief Deinitialize incremental PID controller
 * @param ctx Context pointer
 */
void elib_pid_inc_deinit(elib_pid_inc_ctx_t *ctx);

/**
 * @brief Reset internal state (error/measurement/output history,
 *        re-arm the seeding step; note the output restarts from 0)
 * @param ctx Context pointer
 * @return elib_pid_err_t error code
 */
elib_pid_err_t elib_pid_inc_reset(elib_pid_inc_ctx_t *ctx);

/**
 * @brief Update PID parameters at runtime (e.g. for fuzzy PID tuning)
 * @param ctx Context pointer
 * @param params New PID parameters (copied into context)
 * @return elib_pid_err_t error code
 */
elib_pid_err_t elib_pid_inc_set_params(elib_pid_inc_ctx_t *ctx,
                                       const elib_pid_params_t *params);

/**
 * @brief Compute one step of incremental PID
 * @param ctx Context pointer
 * @param setpoint Target value
 * @param measurement Current value
 * @param output Output pointer
 * @return elib_pid_err_t error code
 */
elib_pid_err_t elib_pid_inc_compute(elib_pid_inc_ctx_t *ctx,
                                    elib_pid_val_t setpoint,
                                    elib_pid_val_t measurement,
                                    elib_pid_val_t *output);

#ifdef __cplusplus
}
#endif

#endif /* ELIB_PID_H */
