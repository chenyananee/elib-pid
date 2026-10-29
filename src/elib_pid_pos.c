/* elib_pid_pos.c - Position PID Controller Implementation
 *
 * Derivative can run on the error (classic) or on the measurement
 * ("derivative first": a setpoint step produces no D-term impulse).
 * The step right after init/reset is a seeding step: the derivative
 * term is zero (no imaginary history to differentiate against) and the
 * history is recorded from this step on. */
#include "elib_pid.h"
#include "elib_pid_util.h"
#include <string.h>

elib_pid_err_t elib_pid_pos_init(elib_pid_pos_ctx_t *ctx,
                                 const elib_pid_params_t *params,
                                 elib_pid_val_t integral_min,
                                 elib_pid_val_t integral_max,
                                 elib_pid_val_t integral_sep_threshold,
                                 uint8_t anti_windup_mode) {
    if (ctx == NULL || params == NULL) {
        return ELIB_PID_ERR_INVALID_PARAM;
    }
    if (!elib_pid_util_params_valid(params)) {
        return ELIB_PID_ERR_INVALID_PARAM;
    }
    if (integral_min >= integral_max) {
        return ELIB_PID_ERR_INVALID_PARAM;
    }
    if (integral_sep_threshold < (elib_pid_val_t)0) {
        return ELIB_PID_ERR_INVALID_PARAM;
    }
    /* refuse unknown mask bits instead of silently truncating them */
    if ((uint8_t)(anti_windup_mode & (uint8_t)~ELIB_PID_AW_ALL) != 0u) {
        return ELIB_PID_ERR_INVALID_PARAM;
    }

    (void)memset(ctx, 0, sizeof(elib_pid_pos_ctx_t));
    (void)memcpy(&ctx->params, params, sizeof(elib_pid_params_t));
    ctx->integral_min = integral_min;
    ctx->integral_max = integral_max;
    ctx->integral_sep_threshold = integral_sep_threshold;
    ctx->bit_flags.anti_windup_mode =
        (uint8_t)(anti_windup_mode & ELIB_PID_AW_ALL);
    ctx->bit_flags.initialized = 1;
    ctx->bit_flags.first_step = 1;

    return ELIB_PID_OK;
}

elib_pid_err_t elib_pid_pos_set_params(elib_pid_pos_ctx_t *ctx,
                                       const elib_pid_params_t *params) {
    if (ctx == NULL || params == NULL) {
        return ELIB_PID_ERR_INVALID_PARAM;
    }
    if (!ctx->bit_flags.initialized) {
        return ELIB_PID_ERR_NOT_INITIALIZED;
    }
    if (!elib_pid_util_params_valid(params)) {
        return ELIB_PID_ERR_INVALID_PARAM;
    }

    (void)memcpy(&ctx->params, params, sizeof(elib_pid_params_t));

    return ELIB_PID_OK;
}

void elib_pid_pos_deinit(elib_pid_pos_ctx_t *ctx) {
    if (ctx == NULL) {
        return;
    }
    ctx->bit_flags.initialized = 0;
}

elib_pid_err_t elib_pid_pos_reset(elib_pid_pos_ctx_t *ctx) {
    if (ctx == NULL) {
        return ELIB_PID_ERR_INVALID_PARAM;
    }
    if (!ctx->bit_flags.initialized) {
        return ELIB_PID_ERR_NOT_INITIALIZED;
    }

    ctx->integral = (elib_pid_val_t)0;
    ctx->prev_error = (elib_pid_val_t)0;
    ctx->prev_measurement = (elib_pid_val_t)0;
    ctx->bit_flags.first_step = 1;

    return ELIB_PID_OK;
}

elib_pid_err_t elib_pid_pos_compute(elib_pid_pos_ctx_t *ctx,
                                    elib_pid_val_t setpoint,
                                    elib_pid_val_t measurement,
                                    elib_pid_val_t *output) {
    elib_pid_val_t error, unused;
    elib_pid_val_t integral_prev;
    const elib_pid_params_t *p;

    if (ctx == NULL || output == NULL) {
        return ELIB_PID_ERR_INVALID_PARAM;
    }
    if (!ctx->bit_flags.initialized) {
        return ELIB_PID_ERR_NOT_INITIALIZED;
    }

    p = &ctx->params;

    /* error through the gate dead zone */
    error = setpoint - measurement;
    error = elib_pid_util_dead_zone(error, p->dead_zone);

    /* integral (skipped entirely ki == 0; separation pauses it) */
    integral_prev = ctx->integral;
    if (p->ki != (elib_pid_val_t)0) {
        int integrate = 1;
        if (ctx->integral_sep_threshold > (elib_pid_val_t)0) {
            if (error > ctx->integral_sep_threshold ||
                error < -ctx->integral_sep_threshold) {
                integrate = 0;
            }
        }
        if (integrate) {
            ctx->integral += error * p->dt;
        }
        if (ctx->bit_flags.anti_windup_mode & ELIB_PID_POS_AW_CLAMP) {
            ctx->integral = elib_pid_util_clamp(ctx->integral,
                                                ctx->integral_min,
                                                ctx->integral_max);
        }
    }

    /* derivative: seeding step yields zero (and skips the filter, so
     * the user's filter state is not disturbed by a zero-in kick) */
    if (p->kd != (elib_pid_val_t)0) {
        elib_pid_val_t d;
        if (!ctx->bit_flags.first_step) {
            elib_pid_val_t d_raw;
            if (p->d_on_meas != 0u) {
                d_raw = (ctx->prev_measurement - measurement) / p->dt;
            } else {
                d_raw = (error - ctx->prev_error) / p->dt;
            }
            d = d_raw;
            if (p->d_filter_fn != NULL) {
                d = p->d_filter_fn(d_raw, p->dt, p->d_filter_ctx);
            }
        } else {
            d = (elib_pid_val_t)0;
        }
        unused = p->kd * d;
    } else {
        unused = (elib_pid_val_t)0;
    }

    /* proportional + integral */
    if (p->kp != (elib_pid_val_t)0) {
        unused += p->kp * error;
    }
    if (p->ki != (elib_pid_val_t)0) {
        elib_pid_val_t with_integral = unused + p->ki * ctx->integral;

        /* anti-windup: conditional integration */
        if (ctx->bit_flags.anti_windup_mode & ELIB_PID_POS_AW_CONDITION) {
            elib_pid_val_t clamped =
                elib_pid_util_clamp(with_integral, p->out_min, p->out_max);
            if (with_integral != clamped) {
                ctx->integral = integral_prev;
                with_integral = clamped;
            }
        }
        unused = with_integral;
    }

    /* output clamping */
    unused = elib_pid_util_clamp(unused, p->out_min, p->out_max);

    /* history recording (errors are the dead-zone treated ones; the
     * measurement is always the raw one) */
    ctx->prev_error = error;
    ctx->prev_measurement = measurement;
    ctx->bit_flags.first_step = 0;

    *output = unused;
    return ELIB_PID_OK;
}
