#include "pio_exec.hpp"

#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/dma.h"

#include "motor_exec.pio.h"

#include <math.h>

// ------------------------------------------------------------
// Internal program registry (per PIO)
// ------------------------------------------------------------

namespace {

// Pico 只有两个 PIO
static bool     program_loaded[2] = { false, false };
static uint     program_offset[2] = { 0, 0 };

static inline uint pio_index(PIO pio) {
    return (pio == pio0) ? 0u : 1u;
}

} // namespace

// ------------------------------------------------------------
// Public API
// ------------------------------------------------------------

uint motor_exec_ensure_program(PIO pio) {
    const uint idx = pio_index(pio);

    if (!program_loaded[idx]) {
        program_offset[idx] =
            pio_add_program(pio, &motor_exec_program);
        program_loaded[idx] = true;
    }

    return program_offset[idx];
}

// ============================================================
// PIO init (STEP-only)
// ============================================================

void motor_exec_init(
    PIO pio,
    uint sm,
    uint offset,
    uint step_pin,
    float clk_div
) {
    // STEP GPIO
    pio_gpio_init(pio, step_pin);
    pio_sm_set_consecutive_pindirs(pio, sm, step_pin, 1, true);

    pio_sm_config c = motor_exec_program_get_default_config(offset);

    // STEP mapped to SET pins
    sm_config_set_set_pins(&c, step_pin, 1);

    // Clock divider
    sm_config_set_clkdiv(&c, clk_div);

    // Init + enable
    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
}

void motor_exec_run(
    PIO pio,
    uint sm,
    uint32_t duty_period,
    uint32_t steps
) {
    // 阻塞写 FIFO，保证顺序与完整性
    pio_sm_put_blocking(pio, sm, duty_period);
    pio_sm_put_blocking(pio, sm, steps);
}

// ============================================================
// DMA stream execution (xE)
// ============================================================

int motor_exec_stream_start(
    PIO pio,
    uint sm,
    const uint32_t* words,
    size_t count
) {
    if (!words || count == 0) return -1;

    // ===== 1. 停止状态机 =====
    pio_sm_set_enabled(pio, sm, false);

    // ===== 2. 清空 FIFO（TX/RX）=====
    pio_sm_clear_fifos(pio, sm);

    // ===== 3. 重启状态机（清 PC / X / Y / ISR / OSR）=====
    pio_sm_restart(pio, sm);

    // ===== 4. 确保 STEP 初始为低电平 =====
    pio_sm_set_pins(pio, sm, 0);
    pio_sm_exec(pio, sm, pio_encode_set(pio_pins, 0));

    // ===== 5. 重新 enable =====
    pio_sm_set_enabled(pio, sm, true);

    // ===== 6. 配置 DMA =====
    int dma_chan = dma_claim_unused_channel(false);
    if (dma_chan < 0) {
        return -1;
    }

    dma_channel_config cfg = dma_channel_get_default_config(dma_chan);

    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_32);
    channel_config_set_read_increment(&cfg, true);
    channel_config_set_write_increment(&cfg, false);
    channel_config_set_dreq(&cfg, pio_get_dreq(pio, sm, true));

    dma_channel_configure(
        dma_chan,
        &cfg,
        &pio->txf[sm],   // 写入 PIO TX FIFO
        words,           // 从内存读取
        (uint32_t)count,
        true             // 立即启动
    );

    return dma_chan;
}

void motor_exec_stream_abort(int dma_chan) {
    if (dma_chan < 0) return;

    dma_channel_abort(dma_chan);
    dma_channel_unclaim(dma_chan);
}

// ============================================================
// Timing model
//   Tstep = (2*duty + 7) / f_sys
// ============================================================

static inline double pio_freq_hz() {
    return (double)clock_get_hz(clk_sys);
}

// Hz → duty_period
uint32_t hz_to_duty_period(double hz) {
    if (hz <= 0.0) return 0;

    double f_sys = pio_freq_hz();

    // duty = (f_sys / hz - 7) / 2
    double d = (f_sys / hz - 6.0) * 0.5;

    if (d < 1.0) return 1;
    return (uint32_t)llround(d);
}

// Period (s) → duty_period
uint32_t period_to_duty_period(double period_s) {
    if (period_s <= 0.0) return 0;

    double f_sys = pio_freq_hz();
    double cycles = period_s * f_sys;

    double d = (cycles - 7.0) * 0.5;
    if (d < 1.0) return 1;

    return (uint32_t)llround(d);
}

// RPM → duty_period
uint32_t rpm_to_duty_period(double rpm, uint32_t pulses_per_rev) {
    if (rpm <= 0.0 || pulses_per_rev == 0) return 0;

    double hz = (rpm / 60.0) * pulses_per_rev;
    return hz_to_duty_period(hz);
}

// Duration (s) → steps
uint32_t duration_to_steps(double duration_s, double hz) {
    if (duration_s <= 0.0 || hz <= 0.0) return 0;
    return (uint32_t)llround(duration_s * hz);
}
