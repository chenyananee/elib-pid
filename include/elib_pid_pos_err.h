/* elib_pid_pos_err.h - Position PID Error Codes */
#ifndef ELIB_PID_POS_ERR_H
#define ELIB_PID_POS_ERR_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ELIB_PID_POS_OK = 0,
    ELIB_PID_POS_ERR_INVALID_PARAM,
    ELIB_PID_POS_ERR_NOT_INITIALIZED,
} elib_pid_pos_err_t;

#ifdef __cplusplus
}
#endif

#endif /* ELIB_PID_POS_ERR_H */
