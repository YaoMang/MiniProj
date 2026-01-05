#include "pio_exec.hpp"

#include "hardware/clocks.h"
#include "hardware/gpio.h"

#include "motor_exec.pio.h"
#include "radar_sync.pio.h"

#include <math.h>


void motor_exec_init(
    PIO pio,
    uint sm,
    uint offset,
    uint dir_pin,
    uint step_pin,
    float clk_div
) {
    // 配置 GPIO
    pio_gpio_init(pio, dir_pin);
    pio_gpio_init(pio, step_pin);

    // 设置为输出
    pio_sm_set_consecutive_pindirs(pio, sm, dir_pin, 1, true);
    pio_sm_set_consecutive_pindirs(pio, sm, step_pin, 1, true);

    // 获取默认配置
    pio_sm_config c = motor_exec_program_get_default_config(offset);

    // 设置 pin mapping
    sm_config_set_out_pins(&c, dir_pin, 1);
    sm_config_set_set_pins(&c, step_pin, 1);

    // 时钟分频
    sm_config_set_clkdiv(&c, clk_div);

    // 初始化 SM
    pio_sm_init(pio, sm, offset, &c);

    // 启动
    pio_sm_set_enabled(pio, sm, true);
}

static inline double pio_freq_hz() {
    return (double)clock_get_hz(clk_sys);
}

uint32_t hz_to_duty_period(double hz) {
    if (hz <= 0.0) return 0;

    double f_sys = pio_freq_hz();

    // duty = (f_sys / f - 3) / 2
    double d = (f_sys / hz - 3.0) * 0.5;

    if (d <= 0.0) return 1; // 最小合法 duty_period

    return (uint32_t)llround(d);
}

uint32_t period_to_duty_period(double period_s) {
    if (period_s <= 0.0) return 0;

    double f_sys = pio_freq_hz();
    double cycles = period_s * f_sys;

    double d = (cycles - 3.0) * 0.5;
    if (d <= 0.0) return 1;

    return (uint32_t)llround(d);
}

uint32_t rpm_to_duty_period(double rpm, uint32_t pulses_per_rev) {
    if (rpm <= 0.0 || pulses_per_rev == 0) return 0;

    double hz = (rpm / 60.0) * pulses_per_rev;
    return hz_to_duty_period(hz);
}

uint32_t duration_to_steps(double duration_s, double hz) {
    if (duration_s <= 0.0 || hz <= 0.0) return 0;
    return (uint32_t)llround(duration_s * hz);
}
