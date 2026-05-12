# elib-pid

嵌入式 PID 算法控制库，支持增量式和位置式两种模式。

## Features

- **位置式 PID** - 支持积分分离、抗积分饱和（积分限幅 + 条件积分）
- **增量式 PID** - 天然抗积分饱和
- **不完全微分** - 用户可注册微分滤波回调函数
- **死区配置** - 两种模式均支持对称死区
- **输出限幅** - 两种模式均支持输出上下限钳制
- **模式解耦** - 增量式和位置式独立文件，按需编译
- **运行时参数更新** - 支持运行中动态修改 Kp/Ki/Kd 等参数，配合模糊算法
- **零动态内存** - 所有资源由用户静态分配
- **类型可配置** - 默认 float，可宏覆盖为定点数或其他类型

## Quick Start

### 1. 定义 PID 参数

```c
#include "elib_pid.h"

elib_pid_params_t params = {
    .kp = 2.0f,
    .ki = 0.5f,
    .kd = 0.1f,
    .dt = 0.01f,           /* 10ms 控制周期 */
    .dead_zone = 0.5f,
    .out_min = -100.0f,
    .out_max = 100.0f,
    .d_filter_fn = NULL,    /* 不使用微分滤波 */
    .d_filter_ctx = NULL,
};
```

### 2. 使用位置式 PID

```c
elib_pid_pos_ctx_t ctx;
elib_pid_pos_init(&ctx, &params,
                  -50.0f, 50.0f,    /* 积分限幅 */
                  10.0f,            /* 积分分离阈值 */
                  ELIB_PID_POS_ANTI_WINDUP_CLAMP | ELIB_PID_POS_ANTI_WINDUP_CONDITION);

/* 控制循环 */
elib_pid_val_t output;
elib_pid_pos_compute(&ctx, setpoint, measurement, &output);
```

### 3. 使用增量式 PID

```c
elib_pid_inc_ctx_t ctx;
elib_pid_inc_init(&ctx, &params);

/* 控制循环 */
elib_pid_val_t output;
elib_pid_inc_compute(&ctx, setpoint, measurement, &output);
```

### 4. 使用微分滤波

```c
elib_pid_val_t my_d_filter(elib_pid_val_t raw_d, elib_pid_val_t dt, void *user_ctx) {
    /* 一阶低通滤波 */
    float alpha = 0.1f;
    float *prev = (float *)user_ctx;
    *prev = alpha * raw_d + (1.0f - alpha) * (*prev);
    return *prev;
}

float filter_state = 0.0f;
params.d_filter_fn = my_d_filter;
params.d_filter_ctx = &filter_state;
```

### 5. 运行时更新参数（模糊 PID）

```c
/* 模糊算法计算出新的 Kp/Ki/Kd */
elib_pid_params_t new_params = params;
new_params.kp = fuzzy_kp;
new_params.ki = fuzzy_ki;
new_params.kd = fuzzy_kd;

/* 运行中更新，不影响积分和误差历史 */
elib_pid_pos_set_params(&ctx, &new_params);
```

### 6. 增量式 PID 控制电机转速

目标：调整到目标转速正负 10 以内

```c
/* 参数：PWM 0-4000，100ms 控制周期 */
elib_pid_params_t params = {
    .kp = 2.0f,
    .ki = 0.01f,
    .kd = 0.5f,
    .dt = 0.1f,
    .dead_zone = 0.0f,
    .out_min = 0.0f,
    .out_max = 4000.0f,
    .d_filter_fn = NULL,
    .d_filter_ctx = NULL,
};

elib_pid_inc_ctx_t ctx;
elib_pid_inc_init(&ctx, &params);

/* 控制循环（100ms 周期） */
elib_pid_val_t speed = 0.0f;    /* 当前转速反馈 */
elib_pid_val_t pwm = 0.0f;      /* 当前 PWM 输出 */
elib_pid_val_t setpoint = 2000.0f;
elib_pid_val_t max_step = 500.0f;  /* 每周期最大步长 */

while (1) {
    elib_pid_val_t pid_out;
    elib_pid_inc_compute(&ctx, setpoint, speed, &pid_out);

    /* 限幅保护（增量式已限幅，此处用于额外步长限制） */
    elib_pid_val_t delta = pid_out - pwm;
    if (delta > max_step) delta = max_step;
    if (delta < -max_step) delta = -max_step;
    pwm += delta;

    set_pwm((uint16_t)pwm);
    speed = read_encoder_speed();  /* 读取编码器反馈 */
    delay_ms(100);

    /* 判断是否收敛到目标 ±10 以内 */
    if (fabsf(setpoint - speed) <= 10.0f) {
        break;  /* 达到目标 */
    }
}
```

## API Reference

### 位置式 PID (`elib_pid_pos.h`)

- `elib_pid_pos_init(ctx, params, integral_min, integral_max, sep_threshold, anti_windup_mode)` - 初始化
- `elib_pid_pos_deinit(ctx)` - 反初始化
- `elib_pid_pos_reset(ctx)` - 重置内部状态
- `elib_pid_pos_set_params(ctx, params)` - 运行时更新参数
- `elib_pid_pos_compute(ctx, setpoint, measurement, &output)` - 计算一步输出

### 增量式 PID (`elib_pid_inc.h`)

- `elib_pid_inc_init(ctx, params)` - 初始化
- `elib_pid_inc_deinit(ctx)` - 反初始化
- `elib_pid_inc_reset(ctx)` - 重置内部状态
- `elib_pid_inc_set_params(ctx, params)` - 运行时更新参数
- `elib_pid_inc_compute(ctx, setpoint, measurement, &output)` - 计算一步输出

## Build

无构建系统，将 `include/` 和 `src/` 加入你的项目：

```bash
# 只用位置式
gcc app.c src/elib_pid_pos.c src/elib_pid_util.c -Iinclude -lm

# 只用增量式
gcc app.c src/elib_pid_inc.c src/elib_pid_util.c -Iinclude -lm

# 两者都用
gcc app.c src/elib_pid_pos.c src/elib_pid_inc.c src/elib_pid_util.c -Iinclude -lm
```

## License

MIT License
