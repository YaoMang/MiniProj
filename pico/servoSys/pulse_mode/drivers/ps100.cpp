#include "ps100.hpp"

#include "hardware/gpio.h"
#include "hardware/pio.h"     // pio_sm_* helpers

#include "pwm_motor.hpp"
#include "pio/pio_exec.hpp"
#include "motor_exec.pio.h"

// ------------------------------------------------------------
// helpers
// ------------------------------------------------------------

static inline absolute_time_t now_ts() {
    return get_absolute_time();
}

static inline uint pio_index(PIO pio) {
    return (pio == pio0) ? 0u : 1u;
}

// PIO function select for a given PIO instance
static inline gpio_function_t pio_gpio_func(PIO pio) {
    return (pio == pio0) ? GPIO_FUNC_PIO0 : GPIO_FUNC_PIO1;
}

// ------------------------------------------------------------
// internal backend ownership / hard-stop helpers (file-local)
// ------------------------------------------------------------

namespace {

enum class ActiveBackend : uint8_t {
    None,
    PWM,
    PIO_PARAM,   // xF via PIO FIFO (motor_exec_run)
    PIO_STREAM   // xE via DMA stream (motor_exec_stream_start)
};

// Per-(PIO,SM) backend tracker to support multiple PS100_P instances.
static ActiveBackend g_backend[2][4] = {
    { ActiveBackend::None, ActiveBackend::None, ActiveBackend::None, ActiveBackend::None },
    { ActiveBackend::None, ActiveBackend::None, ActiveBackend::None, ActiveBackend::None }
};

static inline ActiveBackend& backend_ref(PIO pio, uint sm) {
    return g_backend[pio_index(pio)][sm & 0x3u];
}

// 兜底：把 STEP 设为 GPIO OUT LOW。
// 重要：只允许在 “无人占用 / init 安全态” 使用，
// 不要在 PWM/PIO backend 活跃或刚停止的瞬间滥用。
static inline void select_step_as_gpio_low(uint pin) {
    gpio_set_function(pin, GPIO_FUNC_SIO);
    gpio_set_dir(pin, GPIO_OUT);
    gpio_put(pin, 0);
}

static inline void select_step_for_pwm(uint pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
}

static inline void select_step_for_pio(uint pin, PIO pio) {
    gpio_set_function(pin, pio_gpio_func(pio));
}

// Hard stop PIO SM and try to leave STEP low (PIO side).
// 重要：这里不再“切回 GPIO 低电平”，避免异步截断 & 和上层策略冲突。
static inline void hard_stop_pio(PIO pio, uint sm) {
    // Disable SM immediately
    pio_sm_set_enabled(pio, sm, false);

    // Clear FIFOs and restart so any pending pulls/loops are discarded
    pio_sm_clear_fifos(pio, sm);
    pio_sm_restart(pio, sm);

    // Ensure pin low from PIO side (defensive)
    // 注：此时 pin 仍然挂在 PIO mux 上（如果上层没改 mux）
    pio_sm_exec(pio, sm, pio_encode_set(pio_pins, 0));
}

// Stop currently active backend safely.
// 重要：PS100_P 不再“替 backend 决定最终电平”。
// - PWM：交给 pwm_motor_stop() + pwm_motor_poll_cleanup()
// - PIO：hard_stop_pio() 仅在 PIO 侧 set pins=0，不切 SIO
static inline void terminate_hardware(PS100_P::Config const& cfg) {
    auto& b = backend_ref(cfg.pio, cfg.sm);

    switch (b) {
        case ActiveBackend::PWM:
            // pwm_motor_stop 已保证 idle=LOW（并可能需要 poll_cleanup 做 mux 收尾）
            pwm_motor_stop(cfg.step_pin);
            break;

        case ActiveBackend::PIO_PARAM:
        case ActiveBackend::PIO_STREAM:
            hard_stop_pio(cfg.pio, cfg.sm);
            break;

        case ActiveBackend::None:
        default:
            // truly idle fallback: keep it safely low
            select_step_as_gpio_low(cfg.step_pin);
            break;
    }

    b = ActiveBackend::None;
}

} // namespace

// ------------------------------------------------------------
// ctor
// ------------------------------------------------------------

PS100_P::PS100_P(const Config& cfg)
    : cfg_(cfg) {}

// ------------------------------------------------------------
// lifecycle
// ------------------------------------------------------------

bool PS100_P::init() {
    // STEP safe default: GPIO low
    select_step_as_gpio_low(cfg_.step_pin);

    // DIR
    gpio_init(cfg_.dir_pin);
    gpio_set_dir(cfg_.dir_pin, GPIO_OUT);
    set_direction(true);

    // ENABLE (optional)
    if (cfg_.enable_pin != static_cast<uint>(-1)) {
        gpio_init(cfg_.enable_pin);
        gpio_set_dir(cfg_.enable_pin, GPIO_OUT);
        disable();
    }

    // PWM backend init (always available)
    // pwm_motor_init 会做内部状态 + 并确保 idle low（在你的新实现中）
    pwm_motor_init(cfg_.step_pin);

    // ---- PIO program is NOT owned here ----
    // motor_exec_init will pio_gpio_init(step_pin) => steals pin mux.
    motor_exec_init(
        cfg_.pio,
        cfg_.sm,
        cfg_.program_offset,
        cfg_.step_pin,
        cfg_.pio_clk_div
    );

    // Keep SM disabled by default; enable only when running a PIO command.
    pio_sm_set_enabled(cfg_.pio, cfg_.sm, false);

    // 重要：init 结束把 STEP 留在“安全闲置态”
    // 这里我们选择 GPIO low 兜底（没有 active backend）
    select_step_as_gpio_low(cfg_.step_pin);

    // -------- state init (COM1/COM2) --------
    com1_reason_ = CompletionReason::Completed;
    com2_state_  = CommandState::Empty;
    com2_t_end_  = now_ts();

    backend_ref(cfg_.pio, cfg_.sm) = ActiveBackend::None;

    return true;
}

void PS100_P::deinit() {
    stop();
}

// ------------------------------------------------------------
// driver control
// ------------------------------------------------------------

void PS100_P::enable() {
    if (cfg_.enable_pin == static_cast<uint>(-1)) return;
    gpio_put(cfg_.enable_pin, !cfg_.enable_invert);
}

void PS100_P::disable() {
    if (cfg_.enable_pin == static_cast<uint>(-1)) return;
    gpio_put(cfg_.enable_pin, cfg_.enable_invert);
}

void PS100_P::set_direction(bool forward) {
    bool level = forward ^ cfg_.dir_invert;
    gpio_put(cfg_.dir_pin, level);
}

// ------------------------------------------------------------
// core state transition (pure logic, no side effects)
// ------------------------------------------------------------

void PS100_P::update() {
    if (com2_state_ != CommandState::Running) return;

    // Natural completion when time reached
    if (!time_reached(com2_t_end_)) return;

    // COM2 -> COM1 (Completed), COM2 becomes Empty
    com1_reason_ = CompletionReason::Completed;
    com2_state_  = CommandState::Empty;

    // no hardware stop here by design
    // （硬件在 backend 自己的自然结束语义中完成；PWM 的 mux 收尾靠 pwm_motor_poll_cleanup）
}

// ------------------------------------------------------------
// State query (update-on-read)
// ------------------------------------------------------------

bool PS100_P::busy() {
    update();
    return (com2_state_ == CommandState::Running);
}

PS100_P::CompletionReason PS100_P::last_completion() {
    update();
    return com1_reason_;
}

// ------------------------------------------------------------
// motion commands
// ------------------------------------------------------------

void PS100_P::run_steps(uint32_t steps,
                        uint32_t freq_hz,
                        Backend backend) {
    // settle natural completion first
    update();

    // If COM2 still running, interrupt it (physical stop + state shift)
    if (com2_state_ == CommandState::Running) {
        terminate_hardware(cfg_);
        com1_reason_ = CompletionReason::Interrupted;
        com2_state_  = CommandState::Empty;
    }

    if (steps == 0 || freq_hz == 0) {
        // no-op command: keep COM2 empty, COM1 becomes Completed
        com1_reason_ = CompletionReason::Completed;
        com2_state_  = CommandState::Empty;

        // 兜底：无人占用时保持安全低
        backend_ref(cfg_.pio, cfg_.sm) = ActiveBackend::None;
        select_step_as_gpio_low(cfg_.step_pin);
        return;
    }

    // start backend (non-blocking) + pin mux ownership
    if (backend == Backend::PWM) {
        // Ensure PIO SM is not running, give pin to PWM
        pio_sm_set_enabled(cfg_.pio, cfg_.sm, false);
        select_step_for_pwm(cfg_.step_pin);

        pwm_motor_run(cfg_.step_pin, freq_hz, steps);
        backend_ref(cfg_.pio, cfg_.sm) = ActiveBackend::PWM;
    } else {
        // Ensure PWM is stopped, give pin to PIO, and SM is clean
        // 注：pwm_motor_stop 自己会保证 idle low + mux 收尾策略
        pwm_motor_stop(cfg_.step_pin);

        select_step_for_pio(cfg_.step_pin, cfg_.pio);

        pio_sm_set_enabled(cfg_.pio, cfg_.sm, false);
        pio_sm_clear_fifos(cfg_.pio, cfg_.sm);
        pio_sm_restart(cfg_.pio, cfg_.sm);
        pio_sm_set_enabled(cfg_.pio, cfg_.sm, true);

        uint32_t duty = hz_to_duty_period(static_cast<double>(freq_hz));
        motor_exec_run(cfg_.pio, cfg_.sm, duty, steps);

        backend_ref(cfg_.pio, cfg_.sm) = ActiveBackend::PIO_PARAM;
    }

    // COM2 becomes Running with deterministic t_end
    uint64_t duration_us =
        (static_cast<uint64_t>(steps) * 1000000ULL) / freq_hz;

    com2_t_end_  = delayed_by_us(now_ts(), duration_us);
    com2_state_  = CommandState::Running;
}

void PS100_P::run_velocity(uint32_t freq_hz,
                           uint32_t duration_ms,
                           Backend backend) {
    if (freq_hz == 0 || duration_ms == 0) {
        // treat as no-op completion
        update();
        com1_reason_ = CompletionReason::Completed;

        backend_ref(cfg_.pio, cfg_.sm) = ActiveBackend::None;
        select_step_as_gpio_low(cfg_.step_pin);
        return;
    }

    const uint64_t duration_us = static_cast<uint64_t>(duration_ms) * 1000ULL;
    const double duration_s = static_cast<double>(duration_us) / 1e6;

    // steps = hz * duration_s
    const uint32_t steps = duration_to_steps(duration_s, static_cast<double>(freq_hz));

    run_steps(steps, freq_hz, backend);
}

void PS100_P::run_pio_stream(const uint32_t* words,
                             size_t count,
                             uint64_t estimated_duration_us) {
    if (!supports_pio_stream()) return;

    // settle natural completion first
    update();

    if (!words || count == 0 || estimated_duration_us == 0) {
        com1_reason_ = CompletionReason::Completed;
        com2_state_  = CommandState::Empty;

        backend_ref(cfg_.pio, cfg_.sm) = ActiveBackend::None;
        select_step_as_gpio_low(cfg_.step_pin);
        return;
    }

    // If COM2 still running, interrupt it (physical stop + state shift)
    if (com2_state_ == CommandState::Running) {
        terminate_hardware(cfg_);
        com1_reason_ = CompletionReason::Interrupted;
        com2_state_  = CommandState::Empty;
    }

    // Stop PWM, switch pin to PIO, clean SM, then start DMA stream
    pwm_motor_stop(cfg_.step_pin);
    select_step_for_pio(cfg_.step_pin, cfg_.pio);

    pio_sm_set_enabled(cfg_.pio, cfg_.sm, false);
    pio_sm_clear_fifos(cfg_.pio, cfg_.sm);
    pio_sm_restart(cfg_.pio, cfg_.sm);
    pio_sm_set_enabled(cfg_.pio, cfg_.sm, true);

    motor_exec_stream_start(cfg_.pio, cfg_.sm, words, count);

    backend_ref(cfg_.pio, cfg_.sm) = ActiveBackend::PIO_STREAM;

    com2_t_end_ = delayed_by_us(now_ts(), estimated_duration_us);
    com2_state_ = CommandState::Running;
}

// stop(): the only API that is allowed to have real hardware side effects
void PS100_P::stop() {
    // settle natural completion first (keeps semantics crisp)
    update();

    if (com2_state_ != CommandState::Running) {
        // still enforce safe hardware termination
        terminate_hardware(cfg_);
        com2_state_ = CommandState::Empty;

        // idle fallback
        select_step_as_gpio_low(cfg_.step_pin);
        return;
    }

    // Physical stop
    terminate_hardware(cfg_);

    // COM2 -> COM1 (Stopped), COM2 becomes Empty
    com1_reason_ = CompletionReason::Stopped;
    com2_state_  = CommandState::Empty;

    // idle fallback（停止后 backend=None）
    select_step_as_gpio_low(cfg_.step_pin);
}

// ------------------------------------------------------------
// capability
// ------------------------------------------------------------

bool PS100_P::supports_pio_stream() const {
    // Program ownership is external; we only require a valid execution context.
    // offset can legally be 0, so we cannot treat 0 as "not loaded".
    // As long as upper layer provided correct offset + pio_exec implements streaming, it's supported.
    return true;
}
