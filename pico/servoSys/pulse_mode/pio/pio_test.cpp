// pio/pio_test.cpp

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"

#include "pio_exec.hpp"
#include "motor_exec.pio.h"

void run_pio_dma_test() {
    PIO pio = pio0;
    uint sm = 0;

    uint offset = pio_add_program(pio, &motor_exec_program);

    motor_exec_init(
        pio,
        sm,
        offset,
        2,      // DIR pin
        3,      // STEP pin
        1.0f    // clk_div = 1 → 125 MHz
    );

    // ===== 单条测试命令（4 words，完全匹配 PIO）=====
    uint32_t cmd[] = {
        1,          // DIR = 1
        50000,      // delay_count   ≈ 50000 * 8ns ≈ 400 µs
        0xFFFFFFFF,          // steps
        30000       // pulse_high    ≈ 30000 * 8ns ≈ 240 µs
    };

    int dma_chan = dma_claim_unused_channel(true);
    dma_channel_config cfg = dma_channel_get_default_config(dma_chan);

    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_32);
    channel_config_set_read_increment(&cfg, true);
    channel_config_set_write_increment(&cfg, false);
    channel_config_set_dreq(&cfg, pio_get_dreq(pio, sm, true));

    dma_channel_configure(
        dma_chan,
        &cfg,
        &pio->txf[sm],
        cmd,
        4,      // ⚠️ 注意：现在是 4 words
        true
    );

    dma_channel_wait_for_finish_blocking(dma_chan);
}
