#include "pico/stdlib.h"
#include <stdio.h>
#include <cstring>

#include "pwm_motor.hpp"

static constexpr uint STEP_PIN = 3;

int main() {
    stdio_init_all();
    sleep_ms(2000);

    printf("\nPWM motor test ready.\n");
    printf("Commands:\n");
    printf("  run <hz> <steps>\n");
    printf("  stop\n");

    pwm_motor_init(STEP_PIN);

    char line[64];

    while (true) {
        if (fgets(line, sizeof(line), stdin)) {
            double hz;
            uint32_t steps;

            if (sscanf(line, "run %lf %u", &hz, &steps) == 2) {
                printf("PWM run: hz=%.1f steps=%u\n", hz, steps);
                pwm_motor_run(STEP_PIN,
                              static_cast<uint32_t>(hz),
                              steps);
            }
            else if (strncmp(line, "stop", 4) == 0) {
                printf("PWM stop\n");
                pwm_motor_stop(STEP_PIN);
            }
            else {
                printf("Unknown command\n");
            }
        }

        // 非阻塞验证点
        tight_loop_contents();
    }
}
