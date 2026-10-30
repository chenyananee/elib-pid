[简体中文](README.md) | **English**

# elib-pid

An embedded PID control library: positional + incremental dual modes. C99, zero dynamic memory, no global state, configurable value type (float by default, overridable to fixed point).

## Features

- **Two independently implemented modes**: positional (integral separation + dual anti-windup) and incremental (inherent anti-windup + delta clamping); compile only the one you need
- **Configurable derivative-on-measurement**: the D term can differentiate the measurement (the engineering default; no kick on setpoint steps) or the error (classic form), switched at runtime by one parameter
- **Seed step**: D is always 0 on the first step after init/reset—no startup impulse from "differentiating nothing", and the user's derivative filter state is not disturbed either
- **Incomplete differentiation**: derivative filter callback (e.g. first-order low-pass) registered directly
- **Configuration is validation**: every documented illegal parameter is **rejected** at init and at runtime update (both-zero delta clamps rejected, anti-windup bitmask out-of-range rejected, parameter cross-checks complete in both directions)
- **Deadband / output clamping / delta clamping**: the three essentials; both modes share one parameter struct
- **Zero dynamic memory**: ctx is entirely statically allocated by the user

## Math

Error `e(k) = setpoint - measurement`, first passed through a gated **deadband** (when |e| <= dead_zone, e is recorded as 0; dead_zone=0 disables it).

### Positional Form

```
u(k) = kp*e            (P term always uses the error)
     + ki*sum(e)*dt    (integral; stops accumulating when |e| > sep)
     + kd*D            (when kd != 0)

D raw term:
    error form :  d_raw  = (e(k) - e(k-1)) / dt
    measure form: d_raw  = (m(k-1) - m(k)) / dt
    first step :  D = 0 (seed step)

Anti-windup (stackable):
    CLAMP    : clamp the integral to [integral_min, integral_max] after the fact
    CONDITION: roll back this integral update when the unclamped output exceeds [out_min, out_max]
output = clamp(u, out_min, out_max)
```

### Incremental Form

```
Δu = kp*(e(k) - e(k-1))                 (P term always error form)
   + ki*e(k)*dt
   + kd*D

D raw term:
    error form :  d_raw = (e(k) - 2*e(k-1) + e(k-2)) / dt
    measure form: d_raw = (2*m(k-1) - m(k) - m(k-2)) / dt
    first step :  D = 0 (seed step)

Δu clamped to [delta_min, delta_max]
output = clamp(u(k-1) + Δu, out_min, out_max)   <- clamped value goes back as u(k-1), inherent anti-windup
```

Multiplying the I term by dt / dividing the D term by dt is deliberate dimensional consistency: all three gains share the "per second" dimension, so migrating parameters across different dt values changes only dt.

## Parameters

```c
elib_pid_params_t params = {
    .kp = 1.0f,            /* proportional gain */
    .ki = 1.0f,            /* integral gain */
    .kd = 0.1f,            /* derivative gain */
    .dt = 0.1f,            /* control period, must be > 0 */
    .dead_zone = 10.0f,    /* deadband width, >=0, symmetric; 0 = disabled */
    .out_min = 0.0f,       /* output lower limit */
    .out_max = 4000.0f,    /* output upper limit, must be > out_min */
    .delta_min = -500.0f,  /* incremental mode only: per-step negative delta lower limit */
    .delta_max = 500.0f,   /* incremental mode only: per-step positive delta upper limit */
    .d_on_meas = 1,        /* 1 = derivative on measurement (recommended), 0 = derivative on error */
    .d_filter_fn = NULL,   /* derivative filter callback, NULL = no filtering */
    .d_filter_ctx = NULL,  /* derivative filter user context */
};
```

**delta field guard rails** (incremental mode only; ignored by positional mode):

- both values are 0 → **both init and set_params reject**—`clamp(Δu, 0, 0)` would pin the controller at zero output with no error at all, which is one of the reasons this library exists
- to disable per-step clamping, write explicit large values (e.g. `±FLT_MAX`)
- cross (min > max) is rejected at init and at runtime updates alike

## Quick Start

### Positional Form

```c
elib_pid_pos_ctx_t ctx;
elib_pid_pos_init(&ctx, &params,
                  -50.0f, 50.0f,      /* integral clamp */
                  10.0f,              /* integral separation threshold */
                  ELIB_PID_POS_AW_CLAMP | ELIB_PID_POS_AW_CONDITION);

elib_pid_val_t output;
elib_pid_pos_compute(&ctx, setpoint, measurement, &output);
```

### Incremental Form

```c
elib_pid_inc_ctx_t ctx;
elib_pid_inc_init(&ctx, &params);

elib_pid_val_t output;
elib_pid_inc_compute(&ctx, setpoint, measurement, &output);
```

### Incomplete Differentiation (First-Order Low-Pass Example)

```c
static elib_pid_val_t my_d_filter(elib_pid_val_t raw_d, elib_pid_val_t dt,
                                  void *user_ctx) {
    static float alpha = 0.1f;
    float *prev = user_ctx;
    *prev = alpha * raw_d + (1.0f - alpha) * (*prev);
    return *prev;
}

float filter_state = 0.0f;
params.d_filter_fn = my_d_filter;
params.d_filter_ctx = &filter_state;
```

The seed step does not enter the filter—the filter state is not skewed by a 0 at startup (the test suite has a counter assertion: first step filter_calls == 0).

### Runtime Parameter Updates (Fuzzy PID)

```c
elib_pid_params_t new_params = params;
new_params.kp = fuzzy_kp;
new_params.ki = fuzzy_ki;
new_params.kd = fuzzy_kd;

elib_pid_pos_set_params(&ctx, &new_params);   /* validate everything, then copy in */
```

After switching the `d_on_meas` direction, it is recommended to follow immediately with a `reset` (the historical sequences of the two modes mean different things).

### Motor Speed Loop (parameters verified to converge with a numerical mirror)

Goal: speed converges within ±10 of the setpoint; PWM 0-4000, 100ms period, per-step clamp ±500.

```c
elib_pid_params_t params = {
    .kp = 1.0f,
    .ki = 1.0f,
    .kd = 0.1f,
    .dt = 0.1f,
    .dead_zone = 10.0f,      /* |e| <= 10 stops adjusting, avoiding jitter */
    .out_min = 0.0f,
    .out_max = 4000.0f,
    .delta_min = -500.0f,    /* per-step clamp = slew-rate limit */
    .delta_max = 500.0f,
    .d_on_meas = 1,          /* derivative on measurement */
    .d_filter_fn = NULL,
    .d_filter_ctx = NULL,
};

elib_pid_inc_ctx_t ctx;
elib_pid_inc_init(&ctx, &params);

while (1) {
    elib_pid_val_t pid_out;
    elib_pid_inc_compute(&ctx, setpoint, speed, &pid_out);
    set_pwm((uint16_t)pid_out);
    speed = read_encoder_speed();
    delay_ms(100);
}
```

> A note on these parameters: with too large a `kp` paired with an equally strong per-step clamp, one push and one stop form a limit cycle (the setpoint is never reached, and the error sits still at the hundreds). The set above (kp=1, ki=1, kd=0.1, dead_zone=10, d_on_meas=1) was recomputed step by step with a Python numerical mirror: final error at step 2000 is on the order of ±0.1 with no oscillation. Rule of thumb: **ki is the main force for eliminating steady-state error, kp only provides fast response, and kd must not outweigh ki's step contribution**.

## Semantics Notes (Pitfalls)

- **Incremental output starts from 0 after reset**: if you want to "resume seamlessly from the current output", do not reset; parameter updates during normal operation do not affect u(k-1)
- **The setpoint-change kick in positional mode exists only in the error form**: with `d_on_meas = 1` the D term differentiates the measurement, so however the setpoint jumps D does not move (but the P term responds as usual—that is responsiveness, not a defect)
- **The gated deadband at the edges of incremental mode**: when the error enters or leaves the deadband, the history of e jumps between 0 and non-zero, and the P term produces a one-time reverse kick; the per-step clamp catches it. This is expected behavior
- **I/D dimensions**: inside the library ki is used as `ki*e*dt` and kd as `kd*Δe/dt`—if you bring parameters tuned elsewhere with the textbook incremental form (no dt), convert them first

## API

Unified error codes: `ELIB_PID_OK / ELIB_PID_ERR_INVALID_PARAM / ELIB_PID_ERR_NOT_INITIALIZED`, shared by both modes.

| Positional | Incremental |
|--------|--------|
| `elib_pid_pos_init(ctx, params, int_min, int_max, sep, aw_mode)` | `elib_pid_inc_init(ctx, params)` |
| `elib_pid_pos_deinit(ctx)` | `elib_pid_inc_deinit(ctx)` |
| `elib_pid_pos_reset(ctx)` | `elib_pid_inc_reset(ctx)` |
| `elib_pid_pos_set_params(ctx, params)` | `elib_pid_inc_set_params(ctx, params)` |
| `elib_pid_pos_compute(ctx, sp, m, &out)` | `elib_pid_inc_compute(ctx, sp, m, &out)` |

Anti-windup bitmask (two constants, stackable): `ELIB_PID_POS_AW_CLAMP` / `ELIB_PID_POS_AW_CONDITION`; any bit outside the value range is rejected.

Value type: `ELIB_PID_VAL_T` defaults to `float`; override with `-D` to switch to fixed point / double (the library uses only the four basic arithmetic operations and comparisons, no function stubs needed).

## Building

No build system; add `include/` to the header path and `src/` to the project:

```bash
# positional mode only
arm-none-eabi-gcc -std=c99 -Wall -Wextra -Werror -Iinclude \
    -c src/elib_pid_pos.c

# incremental mode only
arm-none-eabi-gcc -std=c99 -Wall -Wextra -Werror -Iinclude \
    -c src/elib_pid_inc.c

# tests (touch internal helpers, so add one more -Isrc)
arm-none-eabi-gcc -std=c99 -Wall -Wextra -Werror -Iinclude -Isrc \
    -c test/test_elib_pid.c
```

## Directory Layout

```
include/elib_pid.h     the only public header (configuration/error codes/parameters/ctx+API for both modes)
src/elib_pid_util.h    internal helpers (deadband/clamping/validation)
src/elib_pid_pos.c     positional mode
src/elib_pid_inc.c     incremental mode
test/test_elib_pid.c   regression suite (ported cases + v2-added guard rails/derivative on measurement/seed step)
```

## Relationship to v1

The algorithm skeleton (P term in error form, I multiplied by dt, D term dimensions, separation/anti-windup semantics, incremental clamp feedback) matches v1; the API is not compatible (single umbrella header + unified error codes + renamed anti-windup enum). v2 fixes: the all-zero incremental clamp silent trap (an entire v1 test parameter set fell into it—under v1 semantics the official test's `test_inc_compute_first_step` could never produce the 10.025 it claims), the validation gap in runtime parameter updates, and the silent truncation of the anti-windup bitmask; it adds the derivative-on-measurement switch and the seed step.

## License

MIT License
