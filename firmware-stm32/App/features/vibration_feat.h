/**
 * @file    vibration_feat.h
 * @brief   振动特征提取：100×6 采样窗口 → 36 维特征向量
 *
 * 为什么单独成模块：
 *   这是整个项目里**唯一的算法核心**，和驱动、协议、任务调度都没有关系 ——
 *   它只接受一个窗口数组，输出 36 个整数。放在 app_main.c 里的时候，
 *   想单独看/单独测这段算法，得先翻 900 行无关代码。
 *   抽出来之后，它在 PC 上可以直接编译单测（只依赖 math.h 和 regbank 的两个常量）。
 *
 * 输出布局：per-feature（严格）
 *   6 个轴 × 6 个特征，但**按特征分段写**，不是按轴循环写：
 *     out[0..5]   = 各轴均值
 *     out[6..11]  = 各轴标准差
 *     out[12..17] = 各轴峰峰值
 *     out[18..23] = 各轴过零率 ×100
 *     out[24..29] = 各轴主频 ×10 Hz
 *     out[30..35] = 各轴主频带能量占比 ×1000
 *
 *   ⚠️ 写成"按轴循环"（每轴 6 个特征连续放）是**转置关系**，维度同样是 36，
 *      程序不报错、模型照训照跑，但**准确率会掉到随机水平**且极难排查 ——
 *      这是本项目最隐蔽的坑，详见 docs/02-modbus-protocol.md §5-E。
 */

#ifndef VIBRATION_FEAT_H
#define VIBRATION_FEAT_H

#include <stdint.h>
#include "regbank.h"      /* REG_WIN_AXES / REG_WIN_SAMPLES */

#ifdef __cplusplus
extern "C" {
#endif

/** 特征维度（= 6 轴 × 6 个特征），与文档 E 区长度一致 */
#define VIB_FEAT_DIM   36

/**
 * @brief 从 100×6 的采样窗口算出 36 维特征
 *
 * @param w   窗口起始地址，布局为"每样本 6 轴"行优先
 *            （第 k 个样本的第 axis 个轴在 w[k*REG_WIN_AXES + axis]）
 * @param out 输出数组，至少 VIB_FEAT_DIM 个 int16
 *
 * @note 用的是 Goertzel 而不是整段 FFT —— 只算 3~25Hz 内 45 个频点，
 *       比做 128 点 FFT 便宜得多，且浮点写法在 PC 上能直接单测。
 */
void vib_compute_features(const int16_t *w, int16_t out[VIB_FEAT_DIM]);

/** 取窗口里第 k 个样本的第 axis 个轴（给需要自己遍历窗口的调用方用） */
int16_t vib_win_at(const int16_t *w, uint16_t k, uint16_t axis);

/** 单个频点上的能量（Goertzel）。主要给自测/诊断用，正常流程不直接调 */
float vib_goertzel_pow(const int16_t *w, uint16_t axis, float freq);

#ifdef __cplusplus
}
#endif

#endif /* VIBRATION_FEAT_H */
