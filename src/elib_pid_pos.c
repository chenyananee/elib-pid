/* elib_pid_pos.c - Position PID Controller Implementation */
#include "../include/elib_pid_pos.h"
#include "elib_pid_util.h"
#include <string.h>

elib_pid_pos_err_t elib_pid_pos_init(elib_pid_pos_ctx_t *ctx,
                                      const elib_pid_params_t *params,
                                      elib_pid_val_t integral_min,
                                      elib_pid_val_t integral_max,
                                      elib_pid_val_t integral_sep_threshold,
                                      uint32_t anti_windup_mode) {
    if (ctx == NULL || params == NULL) {
        return ELIB_PID_POS_ERR_INVALID_PARAM;
    }
    if (!elib_pid_util_params_valid(params)) {
        return ELIB_PID_POS_ERR_INVALID_PARAM;
    }
    if (integral_min >= integral_max) {
        return ELIB_PID_POS_ERR_INVALID_PARAM;
    }
    if (integral_sep_threshold < (elib_pid_val_t)0) {
        return ELIB_PID_POS_ERR_INVALID_PARAM;
    }

    memset(ctx, 0, sizeof(elib_pid_pos_ctx_t));
    memcpy(&ctx->params, params, sizeof(elib_pid_params_t));
    ctx->integral_min = integral_min;
    ctx->integral_max = integral_max;
    ctx->integral_sep_threshold = integral_sep_threshold;
    ctx->anti_windup_mode = anti_windup_mode;
    ctx->initialized = 1;

    return ELIB_PID_POS_OK;
}

void elib_pid_pos_deinit(elib_pid_pos_ctx_t *ctx) {
    if (ctx == NULL) {
        return;
    }
    ctx->initialized = 0;
}

elib_pid_pos_err_t elib_pid_pos_reset(elib_pid_pos_ctx_t *ctx) {
    if (ctx == NULL) {
        return ELIB_PID_POS_ERR_INVALID_PARAM;
    }
    if (!ctx->initialized) {
        return ELIB_PID_POS_ERR_NOT_INITIALIZED;
    }

    ctx->integral = (elib_pid_val_t)0;
    ctx->prev_error = (elib_pid_val_t)0;

    return ELIB_PID_POS_OK;
}

elib_pid_pos_err_t elib_pid_pos_compute(elib_pid_pos_ctx_t *ctx,
                                         elib_pid_val_t setpoint,
                                         elib_pid_val_t measurement,
                                         elib_pid_val_t *output) {
    if (ctx == NULL || output == NULL) {
        return ELIB_PID_POS_ERR_INVALID_PARAM;
    }
    if (!ctx->initialized) {
        return ELIB_PID_POS_ERR_NOT_INITIALIZED;
    }

    elib_pid_params_t *p = &ctx->params;

    /* Error */
    elib_pid_val_t error = setpoint - measurement;

    /* Dead zone */
    error = elib_pid_util_dead_zone(error, p->dead_zone);

    /* Save integral before modification for conditional anti-windup */
    elib_pid_val_t integral_prev = ctx->integral;

    /* Integral with separation check */
    int integrate = 1;
    if (ctx->integral_sep_threshold > (elib_pid_val_t)0) {
        if (error > ctx->integral_sep_threshold || error < -ctx->integral_sep_threshold) {
            integrate = 0;
        }
    }

    if (integrate) {
        ctx->integral += error * p->dt;
    }

    /* Anti-windup: clamping */
    if (ctx->anti_windup_mode & ELIB_PID_POS_ANTI_WINDUP_CLAMP) {
        ctx->integral = elib_pid_util_clamp(ctx->integral,
                                             ctx->integral_min,
                                             ctx->integral_max);
    }

    /* Derivative */
    elib_pid_val_t d_raw = (error - ctx->prev_error) / p->dt;
    elib_pid_val_t d = d_raw;
    if (p->d_filter_fn != NULL) {
        d = p->d_filter_fn(d_raw, p->dt, p->d_filter_ctx);
    }

    /* PID output */
    elib_pid_val_t out = p->kp * error + p->ki * ctx->integral + p->kd * d;

    /* Anti-windup: conditional integration */
    if (ctx->anti_windup_mode & ELIB_PID_POS_ANTI_WINDUP_CONDITION) {
        elib_pid_val_t out_clamped = elib_pid_util_clamp(out, p->out_min, p->out_max);
        if (out != out_clamped) {
            /* Output saturated, revert integral */
            ctx->integral = integral_prev;
            out = out_clamped;
        }
    }

    /* Output clamping */
    out = elib_pid_util_clamp(out, p->out_min, p->out_max);

    /* Save state */
    ctx->prev_error = error;

    *output = out;
    return ELIB_PID_POS_OK;
}
