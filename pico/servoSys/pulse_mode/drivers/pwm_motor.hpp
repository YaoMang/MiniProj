#pragma once

#include "pico/stdlib.h"
#include <cstdint>

// ============================================================
// PWM motor backend (STEP only)
// ============================================================

// Initialize PWM on STEP pin
void pwm_motor_init(uint step_pin);

// Run fixed number of steps at fixed frequency (non-blocking)
// - freq_hz: step frequency
// - steps: number of pulses
void pwm_motor_run(uint step_pin,
                   uint32_t freq_hz,
                   uint32_t steps);

// Immediately stop PWM output
void pwm_motor_stop(uint step_pin);
