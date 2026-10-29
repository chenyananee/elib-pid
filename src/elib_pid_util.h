/* elib_pid_util.h - internal helpers shared by both controller modes */
#ifndef ELIB_PID_UTIL_H
#define ELIB_PID_UTIL_H

#include "elib_pid.h"

#ifdef __cplusplus
extern "C" {
#endif

/* valid anti-windup bitmask (position mode only) */
#define ELIB_PID_AW_ALL (ELIB_PID_POS_AW_CLAMP | ELIB_PID_POS_AW_CONDITION)

/* Dead zone processing (gate mode): zero inside the symmetric zone,
 * untouched outside. A zero-width zone passes everything through. */
static inline elib_pid_val_t elib_pid_util_dead_zone(elib_pid_val_t error,
                                                     elib_pid_val_t dead_zone) {
    if (error > dead_zone || error < -dead_zone) {
        return error;
    }
    return (elib_pid_val_t)0;
}

static inline elib_pid_val_t elib_pid_util_clamp(elib_pid_val_t val,
                                                 elib_pid_val_t min_val,
                                                 elib_pid_val_t max_val) {
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}

/* common parameter validity (applies to both modes; the delta fields
 * are only meaningful to the incremental mode and are NOT checked
 * here) */
static inline int elib_pid_util_params_valid(const elib_pid_params_t *params) {
    if (params == NULL) return 0;
    if (params->dt <= (elib_pid_val_t)0) return 0;
    if (params->out_min >= params->out_max) return 0;
    if (params->dead_zone < (elib_pid_val_t)0) return 0;
    return 1;
}

/* incremental-mode-only delta limit validity.
 * - crossed limits are rejected outright;
 * - BOTH limits zero is rejected: with clamp(x, 0, 0) the increment is
 *   forced to exactly zero on every step, so the controller output
 *   would silently stick at zero forever - the classic misconfiguration
 *   this library exists to refuse. To disable the rate limit, use
 *   explicit extreme values (e.g. +/- FLT_MAX). */
static inline int elib_pid_util_delta_valid(const elib_pid_params_t *params) {
    if (params->delta_min > params->delta_max) return 0;
    if (params->delta_min == (elib_pid_val_t)0 &&
        params->delta_max == (elib_pid_val_t)0) return 0;
    return 1;
}

#ifdef __cplusplus
}
#endif

#endif /* ELIB_PID_UTIL_H */
