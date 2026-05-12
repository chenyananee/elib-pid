/* elib_pid_util.h - PID Controller Internal Utilities */
#ifndef ELIB_PID_UTIL_H
#define ELIB_PID_UTIL_H

#include "../include/elib_pid_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Dead zone processing (gate mode)
 * @return 0 if |error| <= dead_zone, else error unchanged
 */
static inline elib_pid_val_t elib_pid_util_dead_zone(elib_pid_val_t error,
                                                       elib_pid_val_t dead_zone) {
    if (error > dead_zone || error < -dead_zone) {
        return error;
    }
    return (elib_pid_val_t)0;
}

/**
 * @brief Clamp value to [min_val, max_val]
 */
static inline elib_pid_val_t elib_pid_util_clamp(elib_pid_val_t val,
                                                   elib_pid_val_t min_val,
                                                   elib_pid_val_t max_val) {
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}

/**
 * @brief Validate PID parameters
 * @return 1 if valid, 0 if invalid
 */
static inline int elib_pid_util_params_valid(const elib_pid_params_t *params) {
    if (params == NULL) return 0;
    if (params->dt <= (elib_pid_val_t)0) return 0;
    if (params->out_min >= params->out_max) return 0;
    if (params->dead_zone < (elib_pid_val_t)0) return 0;
    return 1;
}

#ifdef __cplusplus
}
#endif

#endif /* ELIB_PID_UTIL_H */
