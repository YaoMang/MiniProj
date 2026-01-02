// RP2040 PIO timing model utilities

#pragma once
#include <cstdint>

// Convert desired STEP frequency (Hz) to PIO delay loop count.
// delay=0 is reserved and never returned for valid speed.
uint32_t speed_hz_to_delay(uint32_t speed_hz);

// Convert radar pulse width in microseconds to PIO loop count.
uint32_t pulse_us_to_radar_len(uint32_t pulse_us);
