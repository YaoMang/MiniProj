#pragma once

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include <cstdint>
#include <cstddef>

// ============================================================
// PS100_P
//   - Pulse-mode servo driver abstraction
//   - Owns exactly ONE execution context:
//       * STEP / DIR pins
//       * ONE PIO state machine (or PWM backend)
//   - DOES NOT own PIO program memory
// ============================================================

class PS100_P {
public:
    // ------------------------------------------------------------
    // Execution backend (API-level choice, NOT state-machine logic)
    // ------------------------------------------------------------
    enum class Backend : uint8_t {
        PWM,   // hardware PWM
        PIO    // PIO parameter mode (xF)
    };

    // ------------------------------------------------------------
    // How a command ended (COM1)
    // ------------------------------------------------------------
    enum class CompletionReason : uint8_t {
        Completed,    // reached t_end naturally
        Interrupted,  // overridden by a newer command
        Stopped       // explicit stop()
    };

    // ------------------------------------------------------------
    // State of the current command slot (COM2)
    // ------------------------------------------------------------
    enum class CommandState : uint8_t {
        Empty,
        Running
    };

    // ------------------------------------------------------------
    // Configuration (NO program ownership here)
    // ------------------------------------------------------------
    struct Config {
        // STEP / DIR pins
        uint step_pin;
        uint dir_pin;
        bool dir_invert = false;

        // Optional enable pin
        uint enable_pin = static_cast<uint>(-1);
        bool enable_invert = false;

        // -------- PIO execution context --------
        // Program MUST already be loaded by upper layer.
        PIO   pio;              // pio0 or pio1
        uint  sm;               // state machine index
        uint  program_offset;   // motor_exec program offset (REQUIRED)
        float pio_clk_div = 1.0f;
    };

public:
    explicit PS100_P(const Config& cfg);

    // ------------------------------------------------------------
    // lifecycle
    // ------------------------------------------------------------
    bool init();    // binds SM, configures pins, does NOT load program
    void deinit();

    // ------------------------------------------------------------
    // driver control
    // ------------------------------------------------------------
    void enable();
    void disable();
    void set_direction(bool forward);

    // ------------------------------------------------------------
    // Motion commands (last-command-wins)
    // ------------------------------------------------------------
    void run_steps(uint32_t steps,
                   uint32_t freq_hz,
                   Backend backend = Backend::PWM);

    void run_velocity(uint32_t freq_hz,
                      uint32_t duration_ms,
                      Backend backend = Backend::PWM);

    // PIO-only raw DMA stream (xE)
    void run_pio_stream(const uint32_t* words,
                        size_t count,
                        uint64_t estimated_duration_us);

    // Immediate termination (HAS real hardware side effects)
    void stop();

    // ------------------------------------------------------------
    // State query (update-on-read)
    // ------------------------------------------------------------
    bool busy();                         // true iff COM2 is Running
    CompletionReason last_completion();  // COM1 result

    // ------------------------------------------------------------
    // Capability query (pure observation)
    // ------------------------------------------------------------
    bool supports_pio_stream() const;

private:
    // ------------------------------------------------------------
    // Core state transition
    //   - Pure logic
    //   - No hardware side effects
    // ------------------------------------------------------------
    void update();

private:
    Config cfg_;

    // ------------------------------------------------------------
    // COM1: previous command (already finished)
    // ------------------------------------------------------------
    CompletionReason com1_reason_ = CompletionReason::Completed;

    // ------------------------------------------------------------
    // COM2: current command
    // ------------------------------------------------------------
    CommandState     com2_state_ = CommandState::Empty;
    absolute_time_t  com2_t_end_{};
};
