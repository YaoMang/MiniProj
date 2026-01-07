#include "pico/stdlib.h"
#include <stdio.h>
#include <string.h>

#include "drivers/ps100.hpp"
#include "pio/pio_exec.hpp"

// ------------------------------------------------------------
// configuration (adjust to your wiring)
// ------------------------------------------------------------

static constexpr uint STEP_PIN   = 3;
static constexpr uint DIR_PIN    = 4;
static constexpr uint ENABLE_PIN = static_cast<uint>(-1);

// ------------------------------------------------------------
// globals
// ------------------------------------------------------------

PS100_P* motor = nullptr;
PS100_P::Backend current_backend = PS100_P::Backend::PWM;

// ------------------------------------------------------------
// helpers
// ------------------------------------------------------------

static const char* backend_name(PS100_P::Backend b) {
    return (b == PS100_P::Backend::PWM) ? "PWM" : "PIO";
}

static const char* reason_name(PS100_P::CompletionReason r) {
    switch (r) {
        case PS100_P::CompletionReason::Completed:   return "Completed";
        case PS100_P::CompletionReason::Interrupted: return "Interrupted";
        case PS100_P::CompletionReason::Stopped:     return "Stopped";
        default: return "?";
    }
}

static void print_help() {
    printf(
        "\nCommands:\n"
        "  backend pwm|pio\n"
        "  run  <hz> <steps>\n"
        "  runv <hz> <ms>\n"
        "  stream <hz> <steps>\n"
        "  stop\n"
        "  status\n"
        "  dir <0|1>\n"
        "  help\n\n"
    );
}

// ------------------------------------------------------------
// main
// ------------------------------------------------------------

int main() {
    stdio_init_all();
    sleep_ms(2000);

    printf("\nPS100 acceptance test ready.\n");
    print_help();

    // -------- create PS100 --------

    PS100_P::Config cfg{};
    cfg.step_pin = STEP_PIN;
    cfg.dir_pin  = DIR_PIN;
    cfg.enable_pin = ENABLE_PIN;

    cfg.pio = pio0;
    cfg.sm  = 0;
    cfg.pio_clk_div = 1.0f;

    static PS100_P ps100(cfg);
    motor = &ps100;

    motor->init();
    motor->enable();

    // -------- command loop --------

    char line[128];

    while (true) {
        if (fgets(line, sizeof(line), stdin)) {
            // remove trailing newline
            line[strcspn(line, "\r\n")] = 0;

            // ------------------------------------------------
            // backend
            // ------------------------------------------------
            if (strncmp(line, "backend", 7) == 0) {
                char name[16];
                if (sscanf(line, "backend %15s", name) == 1) {
                    if (strcmp(name, "pwm") == 0) {
                        current_backend = PS100_P::Backend::PWM;
                        printf("Backend set to PWM\n");
                    } else if (strcmp(name, "pio") == 0) {
                        current_backend = PS100_P::Backend::PIO;
                        printf("Backend set to PIO\n");
                    } else {
                        printf("Unknown backend\n");
                    }
                }
            }
            // ------------------------------------------------
            // run
            // ------------------------------------------------
            else if (strncmp(line, "run ", 4) == 0) {
                uint32_t hz, steps;
                if (sscanf(line, "run %u %u", &hz, &steps) == 2) {
                    printf(
                        "run: hz=%u steps=%u backend=%s\n",
                        hz, steps, backend_name(current_backend)
                    );
                    motor->run_steps(steps, hz, current_backend);
                }
            }
            // ------------------------------------------------
            // runv
            // ------------------------------------------------
            else if (strncmp(line, "runv ", 5) == 0) {
                uint32_t hz, ms;
                if (sscanf(line, "runv %u %u", &hz, &ms) == 2) {
                    printf(
                        "runv: hz=%u duration=%ums backend=%s\n",
                        hz, ms, backend_name(current_backend)
                    );
                    motor->run_velocity(hz, ms, current_backend);
                }
            }
            // ------------------------------------------------
            // stream (PIO only)
            // ------------------------------------------------
            else if (strncmp(line, "stream ", 7) == 0) {
                uint32_t hz, steps;
                if (sscanf(line, "stream %u %u", &hz, &steps) == 2) {
                    if (!motor->supports_pio_stream()) {
                        printf("PIO stream not supported\n");
                        continue;
                    }

                    uint32_t duty = hz_to_duty_period(hz);

                    static uint32_t cmd[2];
                    cmd[0] = duty;
                    cmd[1] = steps;

                    uint64_t duration_us =
                        (static_cast<uint64_t>(steps) * 1000000ULL) / hz;

                    printf(
                        "stream: hz=%u steps=%u duration=%llu us\n",
                        hz, steps,
                        (unsigned long long)duration_us
                    );

                    motor->run_pio_stream(cmd, 2, duration_us);
                }
            }
            // ------------------------------------------------
            // stop
            // ------------------------------------------------
            else if (strcmp(line, "stop") == 0) {
                printf("stop\n");
                motor->stop();
            }
            // ------------------------------------------------
            // status
            // ------------------------------------------------
            else if (strcmp(line, "status") == 0) {
                printf(
                    "busy=%d last=%s\n",
                    motor->busy() ? 1 : 0,
                    reason_name(motor->last_completion())
                );
            }
            // ------------------------------------------------
            // dir
            // ------------------------------------------------
            else if (strncmp(line, "dir ", 4) == 0) {
                int d;
                if (sscanf(line, "dir %d", &d) == 1) {
                    motor->set_direction(d != 0);
                    printf("dir=%d\n", d);
                }
            }
            // ------------------------------------------------
            // help
            // ------------------------------------------------
            else if (strcmp(line, "help") == 0) {
                print_help();
            }
            // ------------------------------------------------
            else {
                printf("Unknown command. Type 'help'.\n");
            }
        }

        tight_loop_contents();
    }
}
