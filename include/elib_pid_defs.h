/* elib_pid_defs.h - PID Controller Common Definitions */
#ifndef ELIB_PID_DEFS_H
#define ELIB_PID_DEFS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* User-overridable value type (default: float) */
#ifndef ELIB_PID_VAL_T
#define ELIB_PID_VAL_T float
#endif

typedef ELIB_PID_VAL_T elib_pid_val_t;

/**
 * @brief Derivative filter callback
 * @param raw_d Raw derivative value
 * @param dt Time step
 * @param user_ctx User context pointer
 * @return Filtered derivative value
 */
typedef elib_pid_val_t (*elib_pid_d_filter_fn)(
    elib_pid_val_t raw_d,
    elib_pid_val_t dt,
    void *user_ctx
);

/**
 * @brief Shared PID parameters (used by both position and incremental modes)
 */
typedef struct {
    elib_pid_val_t kp;                  /* Proportional gain */
    elib_pid_val_t ki;                  /* Integral gain */
    elib_pid_val_t kd;                  /* Derivative gain */
    elib_pid_val_t dt;                  /* Time step, must be > 0 */
    elib_pid_val_t dead_zone;           /* Dead zone width (>= 0, symmetric) */
    elib_pid_val_t out_min;             /* Output lower limit */
    elib_pid_val_t out_max;             /* Output upper limit, must be > out_min */
    elib_pid_d_filter_fn d_filter_fn;   /* Derivative filter callback, NULL = no filter */
    void *d_filter_ctx;                 /* Derivative filter user context */
} elib_pid_params_t;

#ifdef __cplusplus
}
#endif

#endif /* ELIB_PID_DEFS_H */
