#pragma once

#include "hardware/pio.h"
#include <stdint.h>

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

// ========= 基础换算 =========

// Hz → duty_period
uint32_t hz_to_duty_period(double hz);

// 周期（秒）→ duty_period
uint32_t period_to_duty_period(double period_s);

// rpm → duty_period（伺服场景）
uint32_t rpm_to_duty_period(
    double rpm,
    uint32_t pulses_per_rev
);

// 根据持续时间计算 steps
uint32_t duration_to_steps(
    double duration_s,
    double hz
);

// ========= 高层辅助 =========

// 生成一段恒速动作（Hz + 时间）
void emit_const_speed(
    uint32_t* dst,
    size_t& idx,
    double hz,
    double duration_s
);

// 生成 round 结束符
inline void emit_end(uint32_t* dst, size_t& idx) {
    dst[idx++] = 0; // duty_period = 0
    dst[idx++] = 0; // steps = 0
}