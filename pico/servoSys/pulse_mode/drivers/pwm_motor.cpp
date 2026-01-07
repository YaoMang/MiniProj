#include "pwm_motor.hpp"

#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "hardware/clocks.h"

#include <cmath>

// ============================================================
// Internal state (private to pwm_motor)
// ============================================================

namespace {

// 每个 slice 一份剩余步数（RP2040 一共 8 个 slice）
volatile uint32_t remaining_steps[8] = {0};

// 记录哪些 slice 被 pwm_motor 使用
volatile uint32_t active_slice_mask = 0;

// ------------------------------------------------------------
// IRQ handler
// ------------------------------------------------------------

void pwm_wrap_irq_handler() {
    uint32_t status = pwm_get_irq_status_mask();

    // 只处理被 pwm_motor 管理的 slice
    uint32_t mask = status & active_slice_mask;

    while (mask) {
        uint slice = __builtin_ctz(mask);
        mask &= ~(1u << slice);

        pwm_clear_irq(slice);

        if (remaining_steps[slice] > 0) {
            remaining_steps[slice]--;
            if (remaining_steps[slice] == 0) {
                pwm_set_enabled(slice, false);
                pwm_set_irq_enabled(slice, false);
                active_slice_mask &= ~(1u << slice);
            }
        }
    }
}

inline uint pwm_slice(uint pin) {
    return pwm_gpio_to_slice_num(pin);
}

inline uint pwm_channel(uint pin) {
    return pwm_gpio_to_channel(pin);
}

// ------------------------------------------------------------
// NEW: choose clk_div so that wrap <= 65535
// ------------------------------------------------------------
//
// wrap = sys_hz / (freq * clk_div) - 1
// require: wrap <= 65535
//
//inline float choose_clk_div(uint32_t sys_hz, uint32_t freq_hz) {
//    if (freq_hz == 0) return 1.0f;
//
//    float div =
//        (float)sys_hz / (float)(freq_hz * 65536ULL);
//
//    if (div < 1.0f)   div = 1.0f;
//    if (div > 256.0f) div = 256.0f;   // RP2040 PWM 上限
//
//    return div;
//}

//inline float choose_clk_div(uint32_t sys_hz, uint32_t freq_hz) {
//    if (freq_hz == 0) return 1.0f;
//
//    // ---------- tunable policy ----------
//    constexpr uint32_t WRAP_MIN = 400;    // 推荐下限（过小会影响分辨率/抖动）
//    constexpr uint32_t WRAP_MAX = 20000;  // 推荐上限（过大会影响响应/可用频率）
//
//    float best_div = 1.0f;
//    float best_err = 1e30f;
//
//    // clk_div is 8.4 fixed-point → step = 1/16
//    for (int i = 16; i <= 256 * 16; ++i) {
//        float div = i / 16.0f;
//
//        float wrap_f = (float)sys_hz / (div * (float)freq_hz) - 1.0f;
//        if (wrap_f < (float)WRAP_MIN || wrap_f > (float)WRAP_MAX)
//            continue;
//
//        uint32_t wrap = (uint32_t)(wrap_f + 0.5f);
//
//        // 实际频率
//        float real_freq = (float)sys_hz / (div * (float)(wrap + 1u));
//        float err = fabsf(real_freq - (float)freq_hz);
//
//        if (err < best_err) {
//            best_err = err;
//            best_div = div;
//        }
//    }
//
//    // fallback：保证不会返回非法值（wrap <= 65535）
//    if (best_err == 1e30f) {
//        float div = (float)sys_hz / ((float)freq_hz * 65536.0f);
//        if (div < 1.0f) div = 1.0f;
//        if (div > 256.0f) div = 256.0f;
//        best_div = div;
//    }
//
//    return best_div;
//}

inline float choose_clk_div(uint32_t sys_hz, uint32_t freq_hz) {
    if (freq_hz == 0) return 1.0f;

    // -------- tunable policy --------
    constexpr uint32_t WRAP_MIN = 400;
    constexpr uint32_t WRAP_MAX = 20000;

    constexpr float W_FREQ = 1.0f;
    constexpr float W_WRAP = 0.02f;

    float best_div   = 1.0f;
    float best_score = 1e30f;

    // clk_div is 8.4 fixed-point → step = 1/16
    for (int i = 16; i <= 256 * 16; ++i) {
        float div = i / 16.0f;

        // continuous wrap estimate
        float wrap_f = (float)sys_hz / (div * (float)freq_hz) - 1.0f;

        // hard legality check
        if (wrap_f < 2.0f || wrap_f > 65535.0f)
            continue;

        // nearest integer wrap (matches PWM reality)
        uint32_t wrap = (uint32_t)(wrap_f + 0.5f);

        // actual frequency
        float real_freq =
            (float)sys_hz / (div * (float)(wrap + 1u));

        // normalized frequency error
        float freq_err =
            fabsf(real_freq - (float)freq_hz) / (float)freq_hz;

        // wrap penalty (soft constraint)
        float wrap_penalty = 0.0f;
        if (wrap < WRAP_MIN) {
            wrap_penalty =
                (float)(WRAP_MIN - wrap) / (float)WRAP_MIN;
        } else if (wrap > WRAP_MAX) {
            wrap_penalty =
                (float)(wrap - WRAP_MAX) / (float)WRAP_MAX;
        }

        float score =
            W_FREQ * freq_err +
            W_WRAP * wrap_penalty;

        if (score < best_score) {
            best_score = score;
            best_div   = div;
        }
    }

    // Fallback: guarantee legality (never fail)
    if (best_score == 1e30f) {
        float div =
            (float)sys_hz / ((float)freq_hz * 65536.0f);
        if (div < 1.0f)   div = 1.0f;
        if (div > 256.0f) div = 256.0f;
        best_div = div;
    }

    return best_div;
}


} // namespace

// ============================================================
// Public API
// ============================================================

void pwm_motor_init(uint step_pin) {
    gpio_set_function(step_pin, GPIO_FUNC_PWM);

    uint slice = pwm_slice(step_pin);
    pwm_config cfg = pwm_get_default_config();

    // 默认 clk_div = 1，真正运行时会动态调整
    pwm_init(slice, &cfg, false);
    pwm_set_gpio_level(step_pin, 0);

    remaining_steps[slice] = 0;

    // 安装 IRQ handler（只需一次）
    static bool irq_installed = false;
    if (!irq_installed) {
        irq_set_exclusive_handler(PWM_IRQ_WRAP, pwm_wrap_irq_handler);
        irq_set_enabled(PWM_IRQ_WRAP, true);
        irq_installed = true;
    }
}

void pwm_motor_run(uint step_pin,
                   uint32_t freq_hz,
                   uint32_t steps) {
    if (freq_hz == 0 || steps == 0) return;

    uint slice = pwm_slice(step_pin);
    uint chan  = pwm_channel(step_pin);

    uint32_t sys_hz = clock_get_hz(clk_sys);

    // -------- NEW: dynamic clk_div --------
    float clk_div = choose_clk_div(sys_hz, freq_hz);

    uint32_t wrap =
        (uint32_t)((float)sys_hz / (clk_div * freq_hz)) - 1;

    if (wrap < 2) wrap = 2;
    if (wrap > 65535) wrap = 65535; // 防御性兜底

    pwm_set_enabled(slice, false);      // 确保安全修改
    pwm_set_clkdiv(slice, clk_div);
    pwm_set_wrap(slice, wrap);
    pwm_set_chan_level(slice, chan, wrap / 2);
    pwm_set_counter(slice, 0);

    remaining_steps[slice] = steps;
    active_slice_mask |= (1u << slice);

    pwm_clear_irq(slice);
    pwm_set_irq_enabled(slice, true);
    pwm_set_enabled(slice, true);
}

void pwm_motor_stop(uint step_pin) {
    uint slice = pwm_slice(step_pin);

    pwm_set_enabled(slice, false);
    pwm_set_irq_enabled(slice, false);

    remaining_steps[slice] = 0;
    active_slice_mask &= ~(1u << slice);
}
