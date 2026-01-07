#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"

#include "pio_exec.hpp"
#include "motor_exec.pio.h"

void run_pio_test() {
    PIO pio = pio0;
    uint sm = 0;

    uint offset = pio_add_program(pio, &motor_exec_program);

    motor_exec_init(
        pio,
        sm,
        offset,
        3,      // STEP pin
        1.0f    // clk_div
    );

    double hz = 5000.0;
    uint32_t duty = hz_to_duty_period(hz);
    uint32_t steps = 5;

    uint32_t cmd[] = {
        duty,
        steps
    };

    // ===== 1. 停止状态机 =====
    pio_sm_set_enabled(pio, sm, false);
    
    // ===== 2. 清空 FIFO（TX/RX）=====
    pio_sm_clear_fifos(pio, sm);
    
    // ===== 3. 重启状态机（清 PC / X / Y / ISR / OSR）=====
    pio_sm_restart(pio, sm);
    
    // ===== 4. 确保 STEP 初始为低电平（非常推荐）=====
    pio_sm_set_pins(pio, sm, 0);
    pio_sm_exec(pio, sm, pio_encode_set(pio_pins, 0));
    
    // ===== 5. 重新 enable =====
    pio_sm_set_enabled(pio, sm, true);


    dma_channel_config cfg;
    int dma_chan = dma_claim_unused_channel(true);
    cfg = dma_channel_get_default_config(dma_chan);

    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_32);
    channel_config_set_read_increment(&cfg, true);
    channel_config_set_write_increment(&cfg, false);
    channel_config_set_dreq(&cfg, pio_get_dreq(pio, sm, true));

    dma_channel_configure(
        dma_chan,
        &cfg,
        &pio->txf[sm],
        cmd,
        2,
        true
    );
}
