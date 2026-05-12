/* elib_pid_pos.h - Position PID Controller */
#ifndef ELIB_PID_POS_H
#define ELIB_PID_POS_H

#include "elib_pid_defs.h"
#include "elib_pid_pos_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Anti-windup mode (bitmask, combinable) */
typedef enum {
    ELIB_PID_POS_ANTI_WINDUP_CLAMP     = (1 << 0),  /* Integral clamping */
    ELIB_PID_POS_ANTI_WINDUP_CONDITION = (1 << 1),  /* Conditional integration */
} elib_pid_pos_anti_windup_t;

/* Position PID context (statically allocated by user) */
typedef struct {
    elib_pid_params_t params;
    elib_pid_val_t integral;                /* Integral accumulator */
    elib_pid_val_t prev_error;              /* Previous error */
    elib_pid_val_t integral_min;            /* Integral clamp lower limit */
    elib_pid_val_t integral_max;            /* Integral clamp upper limit */
    elib_pid_val_t integral_sep_threshold;  /* Integral separation threshold */
    uint32_t       anti_windup_mode;        /* Anti-windup mode bitmask */
    int            initialized;
} elib_pid_pos_ctx_t;

/**
 * @brief Initialize position PID controller
 * @param ctx User-allocated context pointer
 * @param params PID parameters (copied into context)
 * @param integral_min Integral clamp lower limit
 * @param integral_max Integral clamp upper limit
 * @param integral_sep_threshold Integral separation threshold (>= 0)
 * @param anti_windup_mode Anti-windup mode bitmask
 * @return elib_pid_pos_err_t error code
 */
elib_pid_pos_err_t elib_pid_pos_init(elib_pid_pos_ctx_t *ctx,
                                      const elib_pid_params_t *params,
                                      elib_pid_val_t integral_min,
                                      elib_pid_val_t integral_max,
                                      elib_pid_val_t integral_sep_threshold,
                                      uint32_t anti_windup_mode);

/**
 * @brief Deinitialize position PID controller
 * @param ctx Context pointer
 */
void elib_pid_pos_deinit(elib_pid_pos_ctx_t *ctx);

/**
 * @brief Reset internal state (integral, error history)
 * @param ctx Context pointer
 * @return elib_pid_pos_err_t error code
 */
elib_pid_pos_err_t elib_pid_pos_reset(elib_pid_pos_ctx_t *ctx);

/**
 * @brief Update PID parameters at runtime (e.g. for fuzzy PID tuning)
 * @param ctx Context pointer
 * @param params New PID parameters (copied into context)
 * @return elib_pid_pos_err_t error code
 */
elib_pid_pos_err_t elib_pid_pos_set_params(elib_pid_pos_ctx_t *ctx,
                                            const elib_pid_params_t *params);

/**
 * @brief Compute one step of position PID
 * @param ctx Context pointer
 * @param setpoint Target value
 * @param measurement Current value
 * @param output Output pointer
 * @return elib_pid_pos_err_t error code
 */
elib_pid_pos_err_t elib_pid_pos_compute(elib_pid_pos_ctx_t *ctx,
                                         elib_pid_val_t setpoint,
                                         elib_pid_val_t measurement,
                                         elib_pid_val_t *output);

#ifdef __cplusplus
}
#endif

#endif /* ELIB_PID_POS_H */
