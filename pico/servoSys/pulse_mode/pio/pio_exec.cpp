#include "pio_exec.hpp"

#include "hardware/clocks.h"
#include "hardware/gpio.h"

#include "motor_exec.pio.h"
#include "radar_sync.pio.h"

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
