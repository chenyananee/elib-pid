/* elib_pid_inc.c - Incremental PID Controller Implementation */
#include "../include/elib_pid_inc.h"
#include "elib_pid_util.h"
#include <string.h>

elib_pid_inc_err_t elib_pid_inc_init(elib_pid_inc_ctx_t *ctx,
                                      const elib_pid_params_t *params) {
    if (ctx == NULL || params == NULL) {
        return ELIB_PID_INC_ERR_INVALID_PARAM;
    }
    if (!elib_pid_util_params_valid(params)) {
        return ELIB_PID_INC_ERR_INVALID_PARAM;
    }

    memset(ctx, 0, sizeof(elib_pid_inc_ctx_t));
    memcpy(&ctx->params, params, sizeof(elib_pid_params_t));
    ctx->initialized = 1;

    return ELIB_PID_INC_OK;
}

elib_pid_inc_err_t elib_pid_inc_set_params(elib_pid_inc_ctx_t *ctx,
                                            const elib_pid_params_t *params) {
    if (ctx == NULL || params == NULL) {
        return ELIB_PID_INC_ERR_INVALID_PARAM;
    }
    if (!ctx->initialized) {
        return ELIB_PID_INC_ERR_NOT_INITIALIZED;
    }
    if (!elib_pid_util_params_valid(params)) {
        return ELIB_PID_INC_ERR_INVALID_PARAM;
    }

    memcpy(&ctx->params, params, sizeof(elib_pid_params_t));

    return ELIB_PID_INC_OK;
}

void elib_pid_inc_deinit(elib_pid_inc_ctx_t *ctx) {
    if (ctx == NULL) {
        return;
    }
    ctx->initialized = 0;
}

elib_pid_inc_err_t elib_pid_inc_reset(elib_pid_inc_ctx_t *ctx) {
    if (ctx == NULL) {
        return ELIB_PID_INC_ERR_INVALID_PARAM;
    }
    if (!ctx->initialized) {
        return ELIB_PID_INC_ERR_NOT_INITIALIZED;
    }

    ctx->prev_error = (elib_pid_val_t)0;
    ctx->prev2_error = (elib_pid_val_t)0;
    ctx->prev_output = (elib_pid_val_t)0;

    return ELIB_PID_INC_OK;
}

elib_pid_inc_err_t elib_pid_inc_compute(elib_pid_inc_ctx_t *ctx,
                                         elib_pid_val_t setpoint,
                                         elib_pid_val_t measurement,
                                         elib_pid_val_t *output) {
    if (ctx == NULL || output == NULL) {
        return ELIB_PID_INC_ERR_INVALID_PARAM;
    }
    if (!ctx->initialized) {
        return ELIB_PID_INC_ERR_NOT_INITIALIZED;
    }

    elib_pid_params_t *p = &ctx->params;

    /* Error */
    elib_pid_val_t error = setpoint - measurement;

    /* Dead zone */
    error = elib_pid_util_dead_zone(error, p->dead_zone);

    /* Incremental terms */
    elib_pid_val_t delta_p = p->kp * (error - ctx->prev_error);
    elib_pid_val_t delta_i = p->ki * error * p->dt;

    elib_pid_val_t d_raw = (error - (elib_pid_val_t)2 * ctx->prev_error + ctx->prev2_error) / p->dt;
    elib_pid_val_t d = d_raw;
    if (p->d_filter_fn != NULL) {
        d = p->d_filter_fn(d_raw, p->dt, p->d_filter_ctx);
    }
    elib_pid_val_t delta_d = p->kd * d;

    /* Incremental output */
    elib_pid_val_t delta_u = delta_p + delta_i + delta_d;
    elib_pid_val_t out = ctx->prev_output + delta_u;

    /* Output clamping */
    out = elib_pid_util_clamp(out, p->out_min, p->out_max);

    /* Save state */
    ctx->prev2_error = ctx->prev_error;
    ctx->prev_error = error;
    ctx->prev_output = out;

    *output = out;
    return ELIB_PID_INC_OK;
}
