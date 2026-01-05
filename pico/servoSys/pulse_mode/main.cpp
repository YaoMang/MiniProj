#include "pico/stdlib.h"
#include "pio/pio_test.hpp"

int main() {
    stdio_init_all();
    sleep_ms(1000);

    run_pio_dma_test();

    while (true) {
        tight_loop_contents();
    }
}
