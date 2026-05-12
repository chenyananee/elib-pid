/* elib_pid_inc.h - Incremental PID Controller */
#ifndef ELIB_PID_INC_H
#define ELIB_PID_INC_H

#include "elib_pid_defs.h"
#include "elib_pid_inc_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Incremental PID context (statically allocated by user) */
typedef struct {
    elib_pid_params_t params;
    elib_pid_val_t prev_error;    /* e(k-1) */
    elib_pid_val_t prev2_error;   /* e(k-2) */
    elib_pid_val_t prev_output;   /* u(k-1) */
    int            initialized;
} elib_pid_inc_ctx_t;

/**
 * @brief Initialize incremental PID controller
 * @param ctx User-allocated context pointer
 * @param params PID parameters (copied into context)
 * @return elib_pid_inc_err_t error code
 */
elib_pid_inc_err_t elib_pid_inc_init(elib_pid_inc_ctx_t *ctx,
                                      const elib_pid_params_t *params);

/**
 * @brief Deinitialize incremental PID controller
 * @param ctx Context pointer
 */
void elib_pid_inc_deinit(elib_pid_inc_ctx_t *ctx);

/**
 * @brief Reset internal state
 * @param ctx Context pointer
 * @return elib_pid_inc_err_t error code
 */
elib_pid_inc_err_t elib_pid_inc_reset(elib_pid_inc_ctx_t *ctx);

/**
 * @brief Update PID parameters at runtime (e.g. for fuzzy PID tuning)
 * @param ctx Context pointer
 * @param params New PID parameters (copied into context)
 * @return elib_pid_inc_err_t error code
 */
elib_pid_inc_err_t elib_pid_inc_set_params(elib_pid_inc_ctx_t *ctx,
                                            const elib_pid_params_t *params);

/**
 * @brief Compute one step of incremental PID
 * @param ctx Context pointer
 * @param setpoint Target value
 * @param measurement Current value
 * @param output Output pointer
 * @return elib_pid_inc_err_t error code
 */
elib_pid_inc_err_t elib_pid_inc_compute(elib_pid_inc_ctx_t *ctx,
                                         elib_pid_val_t setpoint,
                                         elib_pid_val_t measurement,
                                         elib_pid_val_t *output);

#ifdef __cplusplus
}
#endif

#endif /* ELIB_PID_INC_H */
