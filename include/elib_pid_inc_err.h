/* elib_pid_inc_err.h - Incremental PID Error Codes */
#ifndef ELIB_PID_INC_ERR_H
#define ELIB_PID_INC_ERR_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ELIB_PID_INC_OK = 0,
    ELIB_PID_INC_ERR_INVALID_PARAM,
    ELIB_PID_INC_ERR_NOT_INITIALIZED,
} elib_pid_inc_err_t;

#ifdef __cplusplus
}
#endif

#endif /* ELIB_PID_INC_ERR_H */
