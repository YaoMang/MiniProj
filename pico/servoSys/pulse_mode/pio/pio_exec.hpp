#pragma once

#include "hardware/pio.h"

// 初始化 motor_exec PIO 程序
void motor_exec_init(
    PIO pio,
    uint sm,
    uint offset,
    uint dir_pin,
    uint step_pin,
    float clk_div
);

// 初始化 radar_sync PIO 程序
void radar_sync_init(
    PIO pio,
    uint sm,
    uint offset,
    uint sync_pin,
    uint step_input_pin,
    float clk_div
);
