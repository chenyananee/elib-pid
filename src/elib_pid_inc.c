/* elib_pid_inc.c - Incremental PID Controller Implementation
 *
 * u(k) = u(k-1) + Δu with the increment clamped to
 * [delta_min, delta_max] and the output clamped to
 * [out_min, out_max] (the clamped output feeds back as u(k-1), which
 * makes windup impossible by construction).
 *
 * Derivative on error (classic) or on measurement ("derivative
 * first"), first step after init/reset is a seeding step with zero
 * derivative. The P term always stays on the error: its fast kick on
 * setpoint movement IS the proportional response the user asked for;
 * only the D term carries the switch. */
#include "elib_pid.h"
#include "elib_pid_util.h"
#include <string.h>

elib_pid_err_t elib_pid_inc_init(elib_pid_inc_ctx_t *ctx,
                                 const elib_pid_params_t *params) {
    if (ctx == NULL || params == NULL) {
        return ELIB_PID_ERR_INVALID_PARAM;
    }
    if (!elib_pid_util_params_valid(params)) {
        return ELIB_PID_ERR_INVALID_PARAM;
    }
    if (!elib_pid_util_delta_valid(params)) {
        return ELIB_PID_ERR_INVALID_PARAM;
    }

    (void)memset(ctx, 0, sizeof(elib_pid_inc_ctx_t));
    (void)memcpy(&ctx->params, params, sizeof(elib_pid_params_t));
    ctx->bit_flags.initialized = 1;
    ctx->bit_flags.first_step = 1;

    return ELIB_PID_OK;
}

elib_pid_err_t elib_pid_inc_set_params(elib_pid_inc_ctx_t *ctx,
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
    if (!elib_pid_util_delta_valid(params)) {
        return ELIB_PID_ERR_INVALID_PARAM;
    }

    (void)memcpy(&ctx->params, params, sizeof(elib_pid_params_t));

    return ELIB_PID_OK;
}

void elib_pid_inc_deinit(elib_pid_inc_ctx_t *ctx) {
    if (ctx == NULL) {
        return;
    }
    ctx->bit_flags.initialized = 0;
}

elib_pid_err_t elib_pid_inc_reset(elib_pid_inc_ctx_t *ctx) {
    if (ctx == NULL) {
        return ELIB_PID_ERR_INVALID_PARAM;
    }
    if (!ctx->bit_flags.initialized) {
        return ELIB_PID_ERR_NOT_INITIALIZED;
    }

    ctx->prev_error = (elib_pid_val_t)0;
    ctx->prev2_error = (elib_pid_val_t)0;
    ctx->prev_output = (elib_pid_val_t)0;
    ctx->prev_measurement = (elib_pid_val_t)0;
    ctx->prev2_measurement = (elib_pid_val_t)0;
    ctx->bit_flags.first_step = 1;

    return ELIB_PID_OK;
}

elib_pid_err_t elib_pid_inc_compute(elib_pid_inc_ctx_t *ctx,
                                    elib_pid_val_t setpoint,
                                    elib_pid_val_t measurement,
                                    elib_pid_val_t *output) {
    elib_pid_val_t error, delta_u, out;
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

    /* increment: P always on the error; I plain; D per the switch */
    delta_u = (elib_pid_val_t)0;
    if (p->kp != (elib_pid_val_t)0) {
        delta_u += p->kp * (error - ctx->prev_error);
    }
    if (p->ki != (elib_pid_val_t)0) {
        delta_u += p->ki * error * p->dt;
    }
    if (p->kd != (elib_pid_val_t)0) {
        elib_pid_val_t d;
        if (!ctx->bit_flags.first_step) {
            elib_pid_val_t d_raw;
            if (p->d_on_meas != 0u) {
                d_raw = ((elib_pid_val_t)2 * ctx->prev_measurement
                         - measurement - ctx->prev2_measurement) / p->dt;
            } else {
                d_raw = (error
                         - (elib_pid_val_t)2 * ctx->prev_error
                         + ctx->prev2_error) / p->dt;
            }
            d = d_raw;
            if (p->d_filter_fn != NULL) {
                d = p->d_filter_fn(d_raw, p->dt, p->d_filter_ctx);
            }
        } else {
            d = (elib_pid_val_t)0;
        }
        delta_u += p->kd * d;
    }

    /* increment rate limit (limits are validated at init/update) */
    delta_u = elib_pid_util_clamp(delta_u, p->delta_min, p->delta_max);

    out = ctx->prev_output + delta_u;

    /* output clamping (fed back => natural anti-windup) */
    out = elib_pid_util_clamp(out, p->out_min, p->out_max);

    /* history recording (errors are the dead-zone treated ones; the
     * measurements are always the raw ones) */
    ctx->prev2_error = ctx->prev_error;
    ctx->prev_error = error;
    ctx->prev_output = out;
    ctx->prev2_measurement = ctx->prev_measurement;
    ctx->prev_measurement = measurement;
    ctx->bit_flags.first_step = 0;

    *output = out;
    return ELIB_PID_OK;
}
