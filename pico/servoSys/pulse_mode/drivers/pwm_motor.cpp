#include "pwm_motor.hpp"

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"

// ============================================================
// Internal state (private to pwm_motor)
// ============================================================

namespace {

// 每个 slice 一份剩余步数（RP2040 一共 8 个 slice）
volatile uint32_t remaining_steps[8] = {0};

// 记录哪些 slice 被 pwm_motor 使用
volatile uint32_t active_slice_mask = 0;

// NEW: 记录 slice 对应的 step_pin（用于 IRQ 结束时把 pin 拉低）
// 0xFF 表示未知/未绑定
volatile uint8_t slice_step_pin[8] = {
    0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF
};

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
inline float choose_clk_div(uint32_t sys_hz, uint32_t freq_hz) {
    if (freq_hz == 0) return 1.0f;

    // ---------- tunable policy ----------
    constexpr uint32_t WRAP_MIN = 400;    // 推荐下限
    constexpr uint32_t WRAP_MAX = 20000;  // 推荐上限

    float best_div = 1.0f;
    float best_err = 1e30f;

    // clk_div is 8.4 fixed-point → step = 1/16
    for (int i = 16; i <= 256 * 16; ++i) {
        float div = i / 16.0f;

        float wrap_f = (float)sys_hz / (div * freq_hz) - 1.0f;
        if (wrap_f < WRAP_MIN || wrap_f > WRAP_MAX)
            continue;

        uint32_t wrap = (uint32_t)(wrap_f + 0.5f);

        // 实际频率
        float real_freq =
            (float)sys_hz / (div * (wrap + 1));

        float err = fabsf(real_freq - freq_hz);

        if (err < best_err) {
            best_err = err;
            best_div = div;
        }
    }

    // fallback：保证不会返回非法值
    if (best_err == 1e30f) {
        float div = (float)sys_hz / (freq_hz * 65536.0f);
        if (div < 1.0f) div = 1.0f;
        if (div > 256.0f) div = 256.0f;
        best_div = div;
    }

    return best_div;
}


// ------------------------------------------------------------
// NEW: force idle level to LOW (release PWM mux + drive GPIO low)
// ------------------------------------------------------------
inline void force_idle_low_from_pin(uint pin) {
    gpio_set_function(pin, GPIO_FUNC_SIO);
    gpio_set_dir(pin, GPIO_OUT);
    gpio_put(pin, 0);
}

inline void force_idle_low_from_slice(uint slice) {
    uint8_t pin = slice_step_pin[slice];
    if (pin != 0xFF) {
        force_idle_low_from_pin((uint)pin);
    }
}

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
                // 1) stop PWM + disable IRQ
                pwm_set_enabled(slice, false);
                pwm_set_irq_enabled(slice, false);
                active_slice_mask &= ~(1u << slice);

                // 2) IMPORTANT: ensure STEP idle level is LOW
                // PWM disable does NOT guarantee pin low.
                force_idle_low_from_slice(slice);
            }
        }
    }
}

} // namespace

// ============================================================
// Public API
// ============================================================

void pwm_motor_init(uint step_pin) {
    // 先绑定 slice->pin（IRQ 需要）
    uint slice = pwm_slice(step_pin);
    slice_step_pin[slice] = (uint8_t)step_pin;

    // 让 PWM 具备驱动能力
    gpio_set_function(step_pin, GPIO_FUNC_PWM);

    pwm_config cfg = pwm_get_default_config();
    // 默认 clk_div=1，真正运行时会动态调整
    pwm_init(slice, &cfg, false);

    // 初始化为低（注意：这里仍是 PWM mux）
    pwm_set_gpio_level(step_pin, 0);

    remaining_steps[slice] = 0;

    // 安装 IRQ handler（只需一次）
    static bool irq_installed = false;
    if (!irq_installed) {
        irq_set_exclusive_handler(PWM_IRQ_WRAP, pwm_wrap_irq_handler);
        irq_set_enabled(PWM_IRQ_WRAP, true);
        irq_installed = true;
    }

    // 为了“闲时必为低”，我们把 pin 先切回 GPIO 低电平
    // （运行时 pwm_motor_run 会再切回 PWM）
    force_idle_low_from_pin(step_pin);
}

void pwm_motor_run(uint step_pin,
                   uint32_t freq_hz,
                   uint32_t steps) {
    if (freq_hz == 0 || steps == 0) return;

    uint slice = pwm_slice(step_pin);
    uint chan  = pwm_channel(step_pin);

    // 更新 slice->pin 映射（防御性：允许同 slice 换 pin 的极端情况）
    slice_step_pin[slice] = (uint8_t)step_pin;

    uint32_t sys_hz = clock_get_hz(clk_sys);

    // -------- dynamic clk_div --------
    float clk_div = choose_clk_div(sys_hz, freq_hz);

    // wrap = sys_hz / (clk_div * freq) - 1
    uint32_t wrap =
        (uint32_t)((float)sys_hz / (clk_div * (float)freq_hz)) - 1;

    if (wrap < 2) wrap = 2;
    if (wrap > 65535) wrap = 65535; // 防御性兜底

    // 切回 PWM mux（因为 init/stop/自然结束都会切回 SIO）
    gpio_set_function(step_pin, GPIO_FUNC_PWM);

    // 确保安全修改
    pwm_set_enabled(slice, false);

    pwm_set_clkdiv(slice, clk_div);
    pwm_set_wrap(slice, wrap);

    // 50% duty
    pwm_set_chan_level(slice, chan, wrap / 2);

    // 清计数器
    pwm_set_counter(slice, 0);

    // arm steps + IRQ
    remaining_steps[slice] = steps;
    active_slice_mask |= (1u << slice);

    pwm_clear_irq(slice);
    pwm_set_irq_enabled(slice, true);

    // start
    pwm_set_enabled(slice, true);
}

void pwm_motor_stop(uint step_pin) {
    uint slice = pwm_slice(step_pin);

    pwm_set_enabled(slice, false);
    pwm_set_irq_enabled(slice, false);

    remaining_steps[slice] = 0;
    active_slice_mask &= ~(1u << slice);

    // IMPORTANT: ensure idle is LOW
    force_idle_low_from_pin(step_pin);
}
