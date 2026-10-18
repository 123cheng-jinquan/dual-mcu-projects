/**
 * @file    vibration_feat.c
 * @brief   振动特征提取实现（Goertzel 单频点能量 + 时域统计量）
 *
 * 三个特征分两组：
 *   时域 —— 均值、标准差、峰峰值、过零率（便宜，直接遍历窗口）
 *   频域 —— 主频、主频带能量占比（用 Goertzel 在 3~25Hz 内扫频）
 *
 * 为什么用 Goertzel 而不是 FFT：
 *   我们只关心 3~25Hz 这个窄带（采样率 50Hz，Nyquist 25Hz），
 *   在这个范围里按 0.5Hz 步进扫 45 个频点就够了。
 *   做 128 点 FFT 要算全部 64 个频点且复数运算更多 —— Goertzel 更省。
 *   实测 45 频点 × 100 点 × 6 轴 ≈ 2.7 万次浮点运算，F411 有硬件 FPU，
 *   实际耗时 1~2ms，占 2 秒窗口的千分之一。
 */

#include "vibration_feat.h"

#include <math.h>       /* cosf / sqrtf */

/* ---- 频域参数 ---------------------------------------------------------- */
#define FEAT_FS_HZ      50.0f    /* 采样率：与 task_acq_fast 的 20ms 周期一致 */
#define FEAT_FMIN_HZ     3.0f    /* 主频搜索下限（更低的多半是姿态漂移，不是振动） */
#define FEAT_FMAX_HZ    25.0f    /* 上限 = Nyquist（fs/2） */
#define FEAT_DF_HZ       0.5f    /* 频率分辨率 = fs/N = 50/100 */

int16_t vib_win_at(const int16_t *w, uint16_t k, uint16_t axis)
{
    return w[(uint16_t)(k * REG_WIN_AXES + axis)];
}

float vib_goertzel_pow(const int16_t *w, uint16_t axis, float freq)
{
    const float coeff = 2.0f * cosf(2.0f * 3.14159265f * freq / FEAT_FS_HZ);
    float s1 = 0.0f, s2 = 0.0f;
    uint16_t k;

    for (k = 0u; k < REG_WIN_SAMPLES; k++) {
        float s0 = (float)vib_win_at(w, k, axis) + coeff * s1 - s2;
        s2 = s1;
        s1 = s0;
    }
    return (s1 * s1) + (s2 * s2) - coeff * s1 * s2;
}

void vib_compute_features(const int16_t *w, int16_t out[VIB_FEAT_DIM])
{
    uint16_t axis, k;

    /* ---- 第一段：均值 / 标准差 / 峰峰值 / 过零率 ---- */
    for (axis = 0u; axis < REG_WIN_AXES; axis++) {
        int32_t  sum = 0;
        int16_t  vmin = 32767, vmax = -32768;
        float    mean, acc = 0.0f;
        uint16_t zc = 0u;
        int      prev_sign = 0;

        for (k = 0u; k < REG_WIN_SAMPLES; k++) {
            int16_t v = vib_win_at(w, k, axis);
            sum += v;
            if (v < vmin) { vmin = v; }
            if (v > vmax) { vmax = v; }
        }
        mean = (float)sum / (float)REG_WIN_SAMPLES;

        /* 过零率按"相对均值的符号变化"统计：直接数过 0 会被直流偏置骗到 */
        for (k = 0u; k < REG_WIN_SAMPLES; k++) {
            float d = (float)vib_win_at(w, k, axis) - mean;
            int   s;
            acc += d * d;
            s = (d > 0.0f) ? 1 : ((d < 0.0f) ? -1 : 0);
            if ((s != 0) && (prev_sign != 0) && (s != prev_sign)) { zc++; }
            if (s != 0) { prev_sign = s; }
        }

        out[0u * REG_WIN_AXES + axis] = (int16_t)mean;                              /* 均值 */
        out[1u * REG_WIN_AXES + axis] = (int16_t)sqrtf(acc / (float)REG_WIN_SAMPLES); /* 标准差 */
        out[2u * REG_WIN_AXES + axis] = (int16_t)(vmax - vmin);                     /* 峰峰值 */
        out[3u * REG_WIN_AXES + axis] = (int16_t)((uint32_t)zc * 100u / REG_WIN_SAMPLES); /* 过零率 ×100 */
    }

    /* ---- 第二段：主频 / 主频带能量占比（同样 per-feature）---- */
    for (axis = 0u; axis < REG_WIN_AXES; axis++) {
        float f, best_f = 0.0f, best_p = 0.0f, total = 0.0f;

        for (f = FEAT_FMIN_HZ; f <= (FEAT_FMAX_HZ + 0.01f); f += FEAT_DF_HZ) {
            float p = vib_goertzel_pow(w, axis, f);
            total += p;
            if (p > best_p) { best_p = p; best_f = f; }
        }
        out[4u * REG_WIN_AXES + axis] = (int16_t)(best_f * 10.0f);                    /* 主频 ×10 Hz */
        out[5u * REG_WIN_AXES + axis] = (int16_t)(1000.0f * best_p / (total + 1e-3f)); /* 能量占比 ×1000 */
    }
}
