#include "pwm_motor.hpp"

#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "hardware/clocks.h"

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

} // namespace

// ============================================================
// Public API
// ============================================================

void pwm_motor_init(uint step_pin) {
    gpio_set_function(step_pin, GPIO_FUNC_PWM);

    uint slice = pwm_slice(step_pin);
    pwm_config cfg = pwm_get_default_config();

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

    // 一个 PWM 周期 = 一个 STEP 脉冲
    uint32_t wrap = sys_hz / freq_hz;
    if (wrap < 2) wrap = 2;
    wrap -= 1;

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
