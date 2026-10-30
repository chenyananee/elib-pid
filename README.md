**简体中文** | [English](README.en.md)

# elib-pid

嵌入式 PID 控制库：位置式 + 增量式双模式。C99、零动态内存、无全局状态、
值类型可配置（float 默认，可覆盖为定点）。

## 特性

- **两种模式独立实现**：位置式（积分分离 + 双抗饱和）与增量式
  （天然抗饱和 + 增量限幅），可按需只编译其一
- **微分先行可切换**：D 项可对测量值微分（工程默认，设定值跳变无踢冲）
  或对误差微分（经典形），一个参数运行时切换
- **种子步**：init/reset 后首步 D 恒 0——没有"凭空微分"的开机冲量，
  用户微分滤波器状态也不被搅动
- **不完全微分**：微分滤波回调（如一阶低通）直通注册
- **配置即校验**：一切文档化的非法参数在 init 与运行时更新**都被拒绝**
  （增量限幅两零拒绝、抗饱和位掩码超界拒绝、参数交叉校验双向齐全）
- **死区 / 输出限幅 / 增量限幅**：庭院三件套，两模式共用一套参数结构
- **零动态内存**：ctx 全部由用户静态分配

## 数学

误差 `e(k) = setpoint − measurement`，先过门式**死区**（|e| ≤ dead_zone
时 e 记 0，dead_zone=0 关闭）。

### 位置式

```
u(k) = kp·e            （P 项恒对误差）
     + ki·Σe·dt        （积分；分离阈值 |e| > sep 时停积）
     + kd·D            （kd ≠ 0 时）

D 原始项：
    error 形 :  d_raw  = (e(k) − e(k−1)) / dt
    measure 形: d_raw  = (m(k−1) − m(k)) / dt
    首步     :  D = 0（种子步）

抗饱和（可叠加）：
    CLAMP    : 积分事后钳到 [integral_min, integral_max]
    CONDITION: 未钳输出越 [out_min, out_max] 时回滚本次积分
输出 = clamp(u, out_min, out_max)
```

### 增量式

```
Δu = kp·(e(k) − e(k−1))                 （P 项恒误差形）
   + ki·e(k)·dt
   + kd·D

D 原始项：
    error 形 :  d_raw = (e(k) − 2e(k−1) + e(k−2)) / dt
    measure 形: d_raw = (2m(k−1) − m(k) − m(k−2)) / dt
    首步     :  D = 0（种子步）

Δu 钳至 [delta_min, delta_max]
输出 = clamp(u(k−1) + Δu, out_min, out_max)   ← 钳后值回作 u(k−1)，天然抗饱和
```

I 项乘 dt / D 项除 dt 是刻意的量纲一致化：三个增益同为"每秒"量纲，
不同 dt 下参数迁移只改 dt 一项。

## 参数

```c
elib_pid_params_t params = {
    .kp = 1.0f,            /* 比例增益 */
    .ki = 1.0f,            /* 积分增益 */
    .kd = 0.1f,            /* 微分增益 */
    .dt = 0.1f,            /* 控制周期，必须 > 0 */
    .dead_zone = 10.0f,    /* 死区宽度，>=0，对称；0 = 关闭 */
    .out_min = 0.0f,       /* 输出下限 */
    .out_max = 4000.0f,    /* 输出上限，必须 > out_min */
    .delta_min = -500.0f,  /* 增量式专用：单步负增量下限 */
    .delta_max = 500.0f,   /* 增量式专用：单步正增量上限 */
    .d_on_meas = 1,        /* 1 = 微分先行（推荐），0 = 对误差微分 */
    .d_filter_fn = NULL,   /* 微分滤波回调，NULL = 不滤波 */
    .d_filter_ctx = NULL,  /* 微分滤波用户上下文 */
};
```

**delta 字段防线**（增量式专用，位置式忽略）：

- 两值同为 0 → **init 与 set_params 都拒绝**——`clamp(Δu, 0, 0)` 会把
  控制器钉死在零输出且无任何报错，这是本库存在的理由之一
- 禁用单步限幅请写显式大值（如 `±FLT_MAX`）
- 交叉（min > max）在 init 与运行时更新通通拒绝

## 快速上手

### 位置式

```c
elib_pid_pos_ctx_t ctx;
elib_pid_pos_init(&ctx, &params,
                  -50.0f, 50.0f,      /* 积分限幅 */
                  10.0f,              /* 积分分离阈值 */
                  ELIB_PID_POS_AW_CLAMP | ELIB_PID_POS_AW_CONDITION);

elib_pid_val_t output;
elib_pid_pos_compute(&ctx, setpoint, measurement, &output);
```

### 增量式

```c
elib_pid_inc_ctx_t ctx;
elib_pid_inc_init(&ctx, &params);

elib_pid_val_t output;
elib_pid_inc_compute(&ctx, setpoint, measurement, &output);
```

### 不完全微分（一阶低通示例）

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

种子步不进滤波器——滤波器状态不会在开机瞬间被一个 0 拖歪
（测试集里有 counter 断言：首步 filter_calls == 0）。

### 运行时更新参数（模糊 PID）

```c
elib_pid_params_t new_params = params;
new_params.kp = fuzzy_kp;
new_params.ki = fuzzy_ki;
new_params.kd = fuzzy_kd;

elib_pid_pos_set_params(&ctx, &new_params);   /* 全量校验再拷入 */
```

切换 `d_on_meas` 方向后建议紧跟一次 `reset`（两种模式的历史序列
含义不同）。

### 电机转速闭环（参数经数值镜像验证收敛）

目标：转速收敛到设定值 ±10 内；PWM 0–4000、100ms 周期、单步限幅 ±500。

```c
elib_pid_params_t params = {
    .kp = 1.0f,
    .ki = 1.0f,
    .kd = 0.1f,
    .dt = 0.1f,
    .dead_zone = 10.0f,      /* |e| ≤ 10 停止调整，避免抖动 */
    .out_min = 0.0f,
    .out_max = 4000.0f,
    .delta_min = -500.0f,    /* 单步限幅 = 斜率限制 */
    .delta_max = 500.0f,
    .d_on_meas = 1,          /* 微分先行 */
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

> 参数组的讲究：`kp` 过大配同等强弱的单步限幅时，一推一停会形成
> 极限环（设定值永远近不了身，误差停在数百量级不动）。上面这组
> （kp=1, ki=1, kd=0.1, dead_zone=10, d_on_meas=1）经 Python 数值镜像
> 逐步复算：2000 步终值误差 ±0.1 量级，无抖振。经验法则：**ki 提供
> 静差消除的主力，kp 只做快速响应，kd 别盖过 ki 的步进量**。

## 语义备忘（踩坑位）

- **增量式 reset 后输出从 0 起步**：要"从当前输出无缝续跑"就别 reset，
  normal 运行中参数更新不影响 u(k−1)
- **位置式换设定值的 kick 只存在于 error 形**：`d_on_meas = 1` 时
  D 项对测量值微分，设定值怎么跳 D 都不动（但 P 项照常响应——那是
  快速性，不是缺陷）
- **门式死区在增量式的边缘**：误差进出死区瞬间 e 的历史在 0 与非零间
  跳变，P 项产生一次性反向突冲，由单步限幅兜底，属预期行为
- **I/D 量纲**：库内 ki 按 `ki·e·dt`、kd 按 `kd·Δe/dt` 使用——若你从
  别处搬来的参数按教科书增量式（无 dt）整定，需先换算

## API

统一错误码：`ELIB_PID_OK / ELIB_PID_ERR_INVALID_PARAM /
ELIB_PID_ERR_NOT_INITIALIZED`，两模式共用。

| 位置式 | 增量式 |
|--------|--------|
| `elib_pid_pos_init(ctx, params, int_min, int_max, sep, aw_mode)` | `elib_pid_inc_init(ctx, params)` |
| `elib_pid_pos_deinit(ctx)` | `elib_pid_inc_deinit(ctx)` |
| `elib_pid_pos_reset(ctx)` | `elib_pid_inc_reset(ctx)` |
| `elib_pid_pos_set_params(ctx, params)` | `elib_pid_inc_set_params(ctx, params)` |
| `elib_pid_pos_compute(ctx, sp, m, &out)` | `elib_pid_inc_compute(ctx, sp, m, &out)` |

抗饱和位掩码（两常量，可叠加）：`ELIB_PID_POS_AW_CLAMP` /
`ELIB_PID_POS_AW_CONDITION`；值域外的位一律拒绝。

值类型：`ELIB_PID_VAL_T` 默认 `float`，`-D` 覆盖即可换定点/双精度
（库内只用四则运算与比较，无需函数桩）。

## 构建

无构建系统，将 `include/` 加入头文件路径、`src/` 加入工程：

```bash
# 只用位置式
arm-none-eabi-gcc -std=c99 -Wall -Wextra -Werror -Iinclude \
    -c src/elib_pid_pos.c

# 只用增量式
arm-none-eabi-gcc -std=c99 -Wall -Wextra -Werror -Iinclude \
    -c src/elib_pid_inc.c

# 测试（会摸到内部助手，多给一个 -Isrc）
arm-none-eabi-gcc -std=c99 -Wall -Wextra -Werror -Iinclude -Isrc \
    -c test/test_elib_pid.c
```

## 目录

```
include/elib_pid.h     唯一公共头（配置/错误码/参数/两模式 ctx+API）
src/elib_pid_util.h    内部助手（死区/钳制/校验）
src/elib_pid_pos.c     位置式
src/elib_pid_inc.c     增量式
test/test_elib_pid.c   回归套件（平移用例 + v2 新增防线/微分先行/种子步）
```

## 与 v1 的关系

算法骨架（P 项误差形、I 乘 dt、D 项量纲、分离/抗饱和语义、增量限幅
回代）与 v1 一致；API 不兼容（单头伞 + 统一错误码 + 抗饱和枚举改名）。
v2 修复：增量限幅全零静默陷阱（v1 一套测试参数都踩进去了——按 v1
语义，官方测试里的 `test_inc_compute_first_step` 根本跑不出它写的
10.025）、运行时参数更新的校验缺口、抗饱和位掩码静默截断；新增
微分先行开关与种子步。

## License

MIT License
