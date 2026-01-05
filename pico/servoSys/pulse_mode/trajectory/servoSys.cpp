#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#define PULSE_WIDTH 625   // ≈10us @ 125MHz PIO (2 cycles/loop)
#define PROFILE_SEGMENTS 32   // S 曲线段数（每侧）

// motor_exec FIFO format: [delay_count, steps]
typedef struct {
    uint32_t delay;
    uint32_t steps;
} pio_cmd_t;


// =======================================================
// CE trajectory discretization (STEP domain only)
// =======================================================
pio_cmd_t* ce_config_to_pio(uint32_t v_max,
                            uint32_t total_steps,
                            uint32_t ramp_steps_per_side,
                            uint32_t radar_ratio)
{
    (void)radar_ratio; // radar_sync consumes this elsewhere

    // ---------- 基本防护 ----------
    if (v_max == 0 || total_steps == 0) {
        return NULL;
    }

    // ---------- ramp 步数（STEP 空间） ----------
    uint32_t Sr_nominal = ramp_steps_per_side;
    uint32_t Sr = Sr_nominal;
    int has_cruise = 1;

    if (Sr_nominal == 0 || total_steps <= 2 * Sr_nominal) {
        // 短行程：对称 S 曲线，无匀速段
        Sr = total_steps / 2;
        has_cruise = 0;
    }

    uint32_t S_cruise = has_cruise ? (total_steps - 2 * Sr) : 0;

    // ---------- 分段数调整 ----------
    uint32_t M = PROFILE_SEGMENTS;
    if (Sr < M) {
        M = Sr;
        if (M < 4) M = Sr; // 极短行程退化
    }

    // ---------- 分配输出缓冲 ----------
    uint32_t max_cmds = 2 * M + (has_cruise ? 1 : 0) + 1;
    pio_cmd_t* cmds = calloc(max_cmds + 1, sizeof(pio_cmd_t));
    if (!cmds) return NULL;

    // ---------- 生成 S 型速度模板 ----------
    float weight_sum = 0.0f;
    float w[PROFILE_SEGMENTS];

    for (uint32_t i = 0; i < M; i++) {
        float u = ((float)i + 0.5f) / (float)M;
        float g = 6.0f * u * (1.0f - u);   // bell-shape
        w[i] = g;
        weight_sum += g;
    }

    if (weight_sum <= 0.0f) {
        free(cmds);
        return NULL;
    }

    // ---------- 短行程峰值缩放 ----------
    float alpha = 1.0f;
    if (!has_cruise && Sr_nominal > 0) {
        alpha = (float)Sr / (float)Sr_nominal;
        if (alpha > 1.0f) alpha = 1.0f;
    }

    // ---------- 分配加速段 steps ----------
    uint32_t steps_accum = 0;
    uint32_t steps_acc[PROFILE_SEGMENTS];
    uint32_t rem[PROFILE_SEGMENTS];

    for (uint32_t i = 0; i < M; i++) {
        float exact = (w[i] / weight_sum) * Sr;
        uint32_t s = (uint32_t)floorf(exact);
        steps_acc[i] = s;
        rem[i] = (uint32_t)((exact - s) * 1e6f);
        steps_accum += s;
    }

    // ---------- 余数补齐 ----------
    while (steps_accum < Sr) {
        uint32_t best = 0;
        for (uint32_t i = 1; i < M; i++) {
            if (rem[i] > rem[best]) best = i;
        }
        steps_acc[best]++;
        rem[best] = 0;
        steps_accum++;
    }

    // ---------- 输出指令 ----------
    uint32_t idx = 0;

    // --- 加速段 ---
    for (uint32_t i = 0; i < M; i++) {
        if (steps_acc[i] == 0) continue;

        uint32_t speed = (uint32_t)lroundf(v_max * alpha * w[i]);
        if (speed < 1) speed = 1;

        cmds[idx].delay = speed_hz_to_delay(speed);
        cmds[idx].steps = steps_acc[i];
        idx++;
    }

    // --- 匀速段 ---
    if (has_cruise && S_cruise > 0) {
        cmds[idx].delay = speed_hz_to_delay(v_max);
        cmds[idx].steps = S_cruise;
        idx++;
    }

    // --- 减速段 ---
    for (int32_t i = (int32_t)M - 1; i >= 0; i--) {
        if (steps_acc[i] == 0) continue;

        uint32_t speed = (uint32_t)lroundf(v_max * alpha * w[i]);
        if (speed < 1) speed = 1;

        cmds[idx].delay = speed_hz_to_delay(speed);
        cmds[idx].steps = steps_acc[i];
        idx++;
    }

    // --- End marker ---
    cmds[idx].delay = 0;
    cmds[idx].steps = 0;

    return cmds;
}
