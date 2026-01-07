import math
import pandas as pd

# ============================================================
# Configuration
# ============================================================

CLK_SYS_HZ = 125_000_000   # RP2040 default
WRAP_MIN = 400
WRAP_MAX = 20000

W_FREQ = 1.0      # frequency error weight
W_WRAP = 0.005     # wrap penalty weight (tunable)

OUT_CSV = "pwm_div_error_compare_scored.csv"

# ============================================================
# Frequency sweep
# ============================================================

def make_freq_list(max_hz=100_000):
    freqs = []
    freqs += list(range(1, 101))                  # 1..100 Hz
    freqs += list(range(110, 1001, 10))           # 110..1k
    freqs += list(range(1100, 10_001, 100))       # 1.1k..10k
    freqs += list(range(11_000, max_hz + 1, 1000))# 11k..100k
    return freqs

# ============================================================
# PWM helpers (match C++ behavior)
# ============================================================

def quantize_div_8p4(div: float) -> float:
    div = round(div * 16.0) / 16.0
    return min(max(div, 1.0), 256.0)

def compute_real_freq(sys_hz, freq_hz, div):
    wrap = int(sys_hz / (div * freq_hz) - 1.0)
    wrap = max(2, min(wrap, 65535))
    real = sys_hz / (div * (wrap + 1))
    return wrap, real

# ============================================================
# Old choose_clk_div (legal only)
# ============================================================

def choose_div_old(sys_hz, freq_hz):
    if freq_hz == 0:
        return 1.0
    div = sys_hz / (freq_hz * 65536.0)
    return quantize_div_8p4(div)

# ============================================================
# New (hard wrap window)
# ============================================================

def choose_div_window(sys_hz, freq_hz):
    best_div = None
    best_err = float("inf")

    for i in range(16, 256 * 16 + 1):
        div = i / 16.0
        wrap_f = sys_hz / (div * freq_hz) - 1.0
        if wrap_f < WRAP_MIN or wrap_f > WRAP_MAX:
            continue

        wrap = int(wrap_f + 0.5)
        real = sys_hz / (div * (wrap + 1))
        err = abs(real - freq_hz)

        if err < best_err:
            best_err = err
            best_div = div

    if best_div is None:
        return choose_div_old(sys_hz, freq_hz)

    return best_div

# ============================================================
# Layered scoring version (RECOMMENDED)
# ============================================================

def wrap_penalty(wrap):
    if WRAP_MIN <= wrap <= WRAP_MAX:
        return 0.0
    if wrap < WRAP_MIN:
        return (WRAP_MIN - wrap) / WRAP_MIN
    return (wrap - WRAP_MAX) / WRAP_MAX

def choose_div_scored(sys_hz, freq_hz):
    best_div = None
    best_score = float("inf")

    for i in range(16, 256 * 16 + 1):
        div = i / 16.0
        wrap_f = sys_hz / (div * freq_hz) - 1.0
        if wrap_f < 2 or wrap_f > 65535:
            continue

        wrap = int(wrap_f + 0.5)
        real = sys_hz / (div * (wrap + 1))

        freq_err_norm = abs(real - freq_hz) / freq_hz
        score = (
            W_FREQ * freq_err_norm +
            W_WRAP * wrap_penalty(wrap)
        )

        if score < best_score:
            best_score = score
            best_div = div

    if best_div is None:
        return choose_div_old(sys_hz, freq_hz)

    return best_div

# ============================================================
# Main
# ============================================================

def main():
    rows = []

    for freq in make_freq_list():
        # old
        d_old = choose_div_old(CLK_SYS_HZ, freq)
        w_old, r_old = compute_real_freq(CLK_SYS_HZ, freq, d_old)

        # window
        d_win = choose_div_window(CLK_SYS_HZ, freq)
        w_win, r_win = compute_real_freq(CLK_SYS_HZ, freq, d_win)

        # scored
        d_sc = choose_div_scored(CLK_SYS_HZ, freq)
        w_sc, r_sc = compute_real_freq(CLK_SYS_HZ, freq, d_sc)

        rows.append({
            "target_hz": freq,

            "old_div": d_old,
            "old_wrap": w_old,
            "old_real_hz": r_old,
            "old_err_ppm": abs(r_old - freq) / freq * 1e6,

            "window_div": d_win,
            "window_wrap": w_win,
            "window_real_hz": r_win,
            "window_err_ppm": abs(r_win - freq) / freq * 1e6,

            "scored_div": d_sc,
            "scored_wrap": w_sc,
            "scored_real_hz": r_sc,
            "scored_err_ppm": abs(r_sc - freq) / freq * 1e6,
        })

    df = pd.DataFrame(rows)
    df.to_csv(OUT_CSV, index=False)

    print(f"[OK] exported: {OUT_CSV}")
    print(df.head(20))

if __name__ == "__main__":
    main()
