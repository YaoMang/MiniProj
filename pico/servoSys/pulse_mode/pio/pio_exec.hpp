#pragma once

#include "hardware/pio.h"
#include <stdint.h>
#include <stddef.h>

// =======================
// PIO init (STEP only)
// =======================
void motor_exec_init(
    PIO pio,
    uint sm,
    uint offset,
    uint step_pin,
    float clk_div
);

void motor_exec_run(
    PIO pio,
    uint sm,
    uint32_t duty_period,
    uint32_t steps
);

// ------------------------------------------------------------
// PIO program loader (shared, idempotent)
// ------------------------------------------------------------

// Ensure motor_exec_program is loaded into given PIO.
// Safe to call multiple times.
// Returns program offset inside the PIO instruction memory.
uint motor_exec_ensure_program(PIO pio);

// =======================
// DMA stream execution
// =======================

// 启动一次 DMA 指令流注入
// - words: uint32_t 指令流
// - count: word 数量
// - 返回 DMA channel（>=0），失败返回 -1
int motor_exec_stream_start(
    PIO pio,
    uint sm,
    const uint32_t* words,
    size_t count
);

// 中止一次 DMA 指令流
void motor_exec_stream_abort(int dma_chan);

// =======================
// Time model helpers
// =======================
uint32_t hz_to_duty_period(double hz);
uint32_t period_to_duty_period(double period_s);
uint32_t rpm_to_duty_period(double rpm, uint32_t pulses_per_rev);
uint32_t duration_to_steps(double duration_s, double hz);
