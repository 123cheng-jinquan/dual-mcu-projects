/* =============================================================================
 *  mfcc.cpp — MFCC 特征提取实现
 * =============================================================================
 *  见 mfcc.h 顶部的算法说明。
 *
 *  实现选择说明（为什么这么写）
 *  ---------------------------------------------------------------------------
 *  ① **自己实现基-2 FFT，而不是用 ESP-DSP**
 *     理由：ESP-DSP 的 API/依赖会随 IDF 版本变，而这里 FFT 只有 256 点、
 *     代码不到 60 行，自己写**完全可控、可讲解、不引入版本风险**。
 *     面试时能一行行讲清楚蝶形运算是怎么做的，比"我调了个库"有价值。
 *
 *  ② **用 float 而不是纯定点**
 *     ESP32-S3 有硬件单精度 FPU，浮点算 256 点 FFT 只要几十微秒，
 *     完全够快；定点虽然更快但精度和可读性都差很多。
 *     （如果以后要移植到没有 FPU 的芯片，再换 Q15 版本。）
 *
 *  ③ **mel 滤波器和窗函数只算一次**，存在静态数组里
 *     每帧都重算是纯浪费 —— 这些参数只跟采样率/帧长有关，与音频无关。
 *
 *  ④ **幅度谱用 |X|²（功率谱）而不是 |X|**
 *     MFCC 传统上用功率谱；而且省一次开方。
 * ========================================================================== */

#include "mfcc.h"
#include <math.h>
#include <string.h>

/* ---- 静态参数（只算一次）----------------------------------------------- */
static float s_hamming[MFCC_FRAME_LEN];
static float s_melFb[MFCC_MEL_BANDS][MFCC_FFT_SIZE / 2 + 1];  /* 每个滤波器在每个 bin 上的权重 */
static float s_dct[MFCC_NUM_CEPS][MFCC_MEL_BANDS];
static bool  s_tablesReady = false;

/* 工作缓冲（静态，避免每帧在栈上分配）*/
static float s_re[MFCC_FFT_SIZE];
static float s_im[MFCC_FFT_SIZE];

/* 上一次计算的每维标准差 */
static float s_lastStd[MFCC_FEAT_DIM];

/* =============================================================================
 *  工具：Hz <-> Mel
 * ========================================================================== */
static inline float hz2mel(float hz)
{
    /* 常用公式（O'Shaughnessy）：mel = 2595 * log10(1 + hz/700) */
    return 2595.0f * log10f(1.0f + hz / 700.0f);
}

static inline float mel2hz(float mel)
{
    return 700.0f * (powf(10.0f, mel / 2595.0f) - 1.0f);
}

/* =============================================================================
 *  基-2 FFT（原地，迭代版 + 位反转重排）
 * =============================================================================
 *  为什么用迭代版而不是递归版：
 *    迭代版没有递归开销，且可以预先算好旋转因子，在 MCU 上快得多。
 *
 *  n 必须是 2 的幂。
 * ========================================================================== */
static void fftRadix2(float *re, float *im, int n, bool inverse)
{
    /* ---- 位反转重排 ----
     * FFT 的输入需要按"位反转"顺序排列（DSP 教科书里的 classic 步骤）。
     * 例：8 点时，索引 3(011) 要和 索引 6(110) 交换。 */
    for (int i = 1, j = 0; i < n; i++) {
        int bit = n >> 1;
        for (; j & bit; bit >>= 1) {
            j ^= bit;
        }
        j ^= bit;
        if (i < j) {
            float t = re[i]; re[i] = re[j]; re[j] = t;
            t = im[i]; im[i] = im[j]; im[j] = t;
        }
    }

    /* ---- 蝶形运算 ----
     * 逐级合并：len = 2, 4, 8, ... n
     * 每级用旋转因子 W = exp(-2πi·k/len)（逆变换用 +）。 */
    for (int len = 2; len <= n; len <<= 1) {
        const float ang = (inverse ? 2.0f : -2.0f) * (float)M_PI / (float)len;
        const float wRe = cosf(ang);
        const float wIm = sinf(ang);

        for (int i = 0; i < n; i += len) {
            /* 旋转因子递推：w = w * w_step（比每次调用 cos/sin 快很多）*/
            float curRe = 1.0f, curIm = 0.0f;
            for (int k = 0; k < len / 2; k++) {
                const int a = i + k;
                const int b = i + k + len / 2;

                const float uRe = re[a], uIm = im[a];
                const float vRe = re[b] * curRe - im[b] * curIm;
                const float vIm = re[b] * curIm + im[b] * curRe;

                re[a] = uRe + vRe;  im[a] = uIm + vIm;
                re[b] = uRe - vRe;  im[b] = uIm - vIm;

                const float nRe = curRe * wRe - curIm * wIm;
                curIm = curRe * wIm + curIm * wRe;
                curRe = nRe;
            }
        }
    }

    /* 逆变换要除以 n */
    if (inverse) {
        const float inv = 1.0f / (float)n;
        for (int i = 0; i < n; i++) { re[i] *= inv; im[i] *= inv; }
    }
}

/* =============================================================================
 *  预计算：Hamming 窗 / Mel 滤波器组 / DCT 矩阵
 * ========================================================================== */
static void buildTables(void)
{
    if (s_tablesReady) { return; }

    /* ---- ① Hamming 窗 ----
     * w[n] = 0.54 - 0.46*cos(2πn/(N-1))
     * 为什么用 Hamming 而不是矩形窗：矩形窗的旁瓣很高（-13dB），
     * 会让强频率成分"泄漏"到相邻 bin，污染 mel 能量。Hamming 旁瓣 -43dB。 */
    for (int i = 0; i < MFCC_FRAME_LEN; i++) {
        s_hamming[i] = 0.54f - 0.46f * cosf(2.0f * (float)M_PI * i
                                            / (float)(MFCC_FRAME_LEN - 1));
    }

    /* ---- ② Mel 滤波器组 ----
     * 在 mel 刻度上等间距放 MFCC_MEL_BANDS+2 个点，相邻三点构成一个三角滤波器。
     * 每个滤波器的输出 = 该频带内功率谱的加权和。 */
    const float fLow  = 0.0f;
    const float fHigh = (float)MFCC_SAMPLE_RATE / 2.0f;    /* 4000Hz */
    const float melLow  = hz2mel(fLow);
    const float melHigh = hz2mel(fHigh);

    const int nbins = MFCC_FFT_SIZE / 2 + 1;
    float binHz = (float)MFCC_SAMPLE_RATE / (float)MFCC_FFT_SIZE;  /* 每个 bin 多少 Hz */

    memset(s_melFb, 0, sizeof(s_melFb));
    for (int m = 0; m < MFCC_MEL_BANDS; m++) {
        const float melL = melLow + (melHigh - melLow) * (float)m       / (float)(MFCC_MEL_BANDS + 1);
        const float melC = melLow + (melHigh - melLow) * (float)(m + 1) / (float)(MFCC_MEL_BANDS + 1);
        const float melR = melLow + (melHigh - melLow) * (float)(m + 2) / (float)(MFCC_MEL_BANDS + 1);

        const float hzL = mel2hz(melL);
        const float hzC = mel2hz(melC);
        const float hzR = mel2hz(melR);

        for (int k = 0; k < nbins; k++) {
            const float f = (float)k * binHz;
            float w = 0.0f;
            if (f >= hzL && f <= hzC && hzC > hzL) {
                w = (f - hzL) / (hzC - hzL);          /* 上升沿 */
            } else if (f > hzC && f <= hzR && hzR > hzC) {
                w = (hzR - f) / (hzR - hzC);          /* 下降沿 */
            }
            s_melFb[m][k] = w;
        }
    }

    /* ---- ③ DCT-II 矩阵 ----
     * c[i] = sum_m  logE[m] * cos(π·i·(m+0.5)/M)
     * 把 M 个 mel 对数能量压缩成 MFCC_NUM_CEPS 个倒谱系数。
     * 只有第 0 个系数需要额外的归一化（乘 1/sqrt(2)），这是标准定义。 */
    for (int i = 0; i < MFCC_NUM_CEPS; i++) {
        for (int m = 0; m < MFCC_MEL_BANDS; m++) {
            s_dct[i][m] = cosf((float)M_PI * (float)i
                               * ((float)m + 0.5f) / (float)MFCC_MEL_BANDS);
        }
    }

    s_tablesReady = true;
}

/* =============================================================================
 *  主流程（静态 MFCC → CMN → Δ → ΔΔ → 帧平均）
 * =============================================================================
 *  ★ 为什么要 CMN（倒谱均值归一化）★
 *    同一支麦克风、同一个房间会给所有语音叠加**相同的通道频响**。
 *    这个成分对"区分谁在说话"毫无用处，却会**抬高不同人之间的相似度**
 *    （实测：本人 0.99、放一段别人说话的视频也 0.994，只差 0.005）。
 *    CMN 的做法：把整段语音每一维的**均值减掉**，只保留相对变化。
 *    这样就把"通道 + 说话人"的叠加变成了"纯粹说话人相对特征"。
 *
 *  ★ 为什么要 Δ / ΔΔ ★
 *    静态 MFCC 只描述"频谱长什么样"，而**发音动作的节奏**（音素过渡速度、
 *    协同发音模式）是很有辨识度的说话人特征。Δ 就是一阶时间差分，
 *    ΔΔ 是二阶。经典的回归公式（和 HTK 一致）：
 *        Δ[t] = Σ_{k=1..K} k·(c[t+k] - c[t-k]) / (2·Σ k²)
 *    这里取 K=2（用前后各两帧），是常用配置。
 * ========================================================================== */
int mfccCompute(const uint8_t *samples, uint32_t count, float *out)
{
    if (samples == nullptr || out == nullptr) { return 0; }
    if (count < MFCC_FRAME_LEN) { return 0; }

    buildTables();

    const int nFrames = (int)((count - MFCC_FRAME_LEN) / MFCC_FRAME_STEP) + 1;
    if (nFrames <= 0) { return 0; }

    /* 帧数上限保护：1 秒音频约 98 帧，给 4 倍余量 */
    static const int MAX_FRAMES = 400;
    if (nFrames > MAX_FRAMES) { return 0; }

    /* 存所有帧的静态特征，供 Δ 和统计量使用。
     * 400×14×4B = 22KB，放静态区（不占栈）。 */
    static float s_feat[MAX_FRAMES][MFCC_STATIC_DIM];

    /* Δ 的回归公式（K=2）：
     *   Δ[t] = Σ_{k=1..2} k·(c[t+k] - c[t-k]) / (2·(1²+2²))
     *        = [1·(c[t+1]-c[t-1]) + 2·(c[t+2]-c[t-2])] / 10
     * 边界用"复制端点"处理，避免越界。
     * 为什么用宏：ΔΔ 需要对 Δ 再算一次同样的回归，
     * 展开成宏比写两遍代码更不容易出错。 */
    #define DELTA_AT(idx, dim) ( \
        (1.0f * (s_feat[((idx)+1 < nFrames ? (idx)+1 : (idx))][dim] \
               - s_feat[((idx)-1 >= 0 ? (idx)-1 : (idx))][dim]) \
       + 2.0f * (s_feat[((idx)+2 < nFrames ? (idx)+2 : (idx))][dim] \
               - s_feat[((idx)-2 >= 0 ? (idx)-2 : (idx))][dim])) / 10.0f )

    const int nbins = MFCC_FFT_SIZE / 2 + 1;
    float prev = 0.0f;      /* 预加重用的上一拍样本 */

    /* ---- 第一遍：算出每一帧的静态特征 ------------------------------- */
    for (int f = 0; f < nFrames; f++) {
        const uint32_t off = (uint32_t)f * MFCC_FRAME_STEP;

        /* ① 预加重 + ③ 加窗，同时填充 FFT 输入 */
        memset(s_re, 0, sizeof(s_re));
        memset(s_im, 0, sizeof(s_im));
        for (int n = 0; n < MFCC_FRAME_LEN; n++) {
            /* 8bit 样本先还原到以 0 为中心：F103 发来的是 (12bit ADC >> 4)，
             * 即 0~255、中点约 128。减 128 等价于"去直流"，
             * 不做的话 FFT 的 0 号 bin 会巨大，污染所有 mel 能量。 */
            const float x = (float)samples[off + n] - 128.0f;
            const float emph = x - MFCC_PREEMPH * prev;
            prev = x;
            s_re[n] = emph * s_hamming[n];
        }

        /* ④ FFT */
        fftRadix2(s_re, s_im, MFCC_FFT_SIZE, false);

        /* ⑤ Mel 滤波器组（功率谱）+ ⑥ 取对数 */
        float melE[MFCC_MEL_BANDS];
        for (int m = 0; m < MFCC_MEL_BANDS; m++) {
            float sum = 0.0f;
            for (int k = 0; k < nbins; k++) {
                const float w = s_melFb[m][k];
                if (w > 0.0f) {
                    sum += w * (s_re[k] * s_re[k] + s_im[k] * s_im[k]);
                }
            }
            melE[m] = logf(sum + 1e-6f);
        }

        /* ⑦ DCT -> 静态 MFCC */
        float *dst = s_feat[f];
        float e = 0.0f;
        for (int m = 0; m < MFCC_MEL_BANDS; m++) { e += melE[m]; }
        dst[0] = e / (float)MFCC_MEL_BANDS;        /* 第 0 维：对数总能量 */

        for (int i = 1; i < MFCC_STATIC_DIM; i++) {
            const int ci = i - 1;
            float s = 0.0f;
            for (int m = 0; m < MFCC_MEL_BANDS; m++) {
                s += melE[m] * s_dct[ci][m];
            }
            if (ci == 0) { s *= 0.70710678f; }     /* 标准归一化 */
            dst[i] = s;
        }
    }

    /* ---- 第二遍：对 42 维动态特征求 均值 + 标准差 ---------------------
     * ★ 为什么不用 CMN（这里特意说明，避免以后又走回头路）★
     *   CMN 会把每一维在整段语音上的均值减成 0。这样一来，
     *   "对所有帧求平均"得到的向量几乎全是 0（只剩浮点噪声），
     *   拿噪声算相似度会得到随机的负值 —— 实测交叉检查出现 -0.76。
     *   ⇒ 结论：**要么用 CMN + 帧级对齐(DTW)，要么不用 CMN + 用统计量**。
     *     我们选后者（实现简单、无需对齐），所以这里**不做 CMN**。
     *
     * 保留静态特征的绝对幅度是**有意的**：
     *   一个人声道的整体频谱包络高低（比如低沉的嗓音 c1 偏大）
     *   本身就是很强的说话人线索，减均值会把它抹掉。 */
    double acc[MFCC_DYN_DIM];
    double accSq[MFCC_DYN_DIM];
    for (int i = 0; i < MFCC_DYN_DIM; i++) { acc[i] = 0.0; accSq[i] = 0.0; }

    for (int f = 0; f < nFrames; f++) {
        float feat[MFCC_DYN_DIM];

        for (int d = 0; d < MFCC_STATIC_DIM; d++) {
            /* 静态部分（保留绝对值） */
            feat[d] = s_feat[f][d];

            /* ① Δ（一阶差分）回归公式，K=2，边界复制端点 */
            const int fp1 = (f + 1 < nFrames) ? (f + 1) : f;
            const int fm1 = (f - 1 >= 0)        ? (f - 1) : f;
            const int fp2 = (f + 2 < nFrames) ? (f + 2) : f;
            const int fm2 = (f - 2 >= 0)        ? (f - 2) : f;

            const float d1 = (1.0f * (s_feat[fp1][d] - s_feat[fm1][d])
                            + 2.0f * (s_feat[fp2][d] - s_feat[fm2][d])) / 10.0f;
            feat[MFCC_STATIC_DIM + d] = d1;

            /* ② ΔΔ（对 Δ 再做一次同样的回归）*/
            const float d1p1 = DELTA_AT(fp1, d);
            const float d1m1 = DELTA_AT(fm1, d);
            const float d1p2 = DELTA_AT(fp2, d);
            const float d1m2 = DELTA_AT(fm2, d);
            feat[MFCC_STATIC_DIM * 2 + d] =
                (1.0f * (d1p1 - d1m1) + 2.0f * (d1p2 - d1m2)) / 10.0f;
        }

        for (int i = 0; i < MFCC_DYN_DIM; i++) {
            acc[i]   += feat[i];
            accSq[i] += feat[i] * feat[i];
        }
    }

    /* ---- 第三遍：输出「均值 | 标准差」拼接的 84 维 -------------------
     * 前 42 维 = 各维均值   （音色：频谱包络平均形状）
     * 后 42 维 = 各维标准差 （动态：包络起伏有多剧烈）
     * 为什么标准差也放进特征：它衡量"这段语音的起伏程度"，
     * 而不同人的发音力度/节奏差别很大，是很有效的说话人线索。 */
    for (int i = 0; i < MFCC_DYN_DIM; i++) {
        const double mean = acc[i] / (double)nFrames;
        out[i] = (float)mean;

        double var = accSq[i] / (double)nFrames - mean * mean;
        if (var < 0.0) { var = 0.0; }
        const float sd = (float)sqrt(var);
        out[MFCC_DYN_DIM + i] = sd;

        s_lastStd[i] = sd;
    }

    return nFrames;
}

#undef DELTA_AT

const float *mfccLastStd(void)
{
    return s_lastStd;
}

/* =============================================================================
 *  余弦相似度 / 归一化
 * ========================================================================== */
void mfccNormalize(float *v, int dim)
{
    if (v == nullptr || dim <= 0) { return; }
    double n2 = 0.0;
    for (int i = 0; i < dim; i++) { n2 += (double)v[i] * (double)v[i]; }
    const float n = (float)sqrt(n2);
    if (n < 1e-9f) { return; }
    for (int i = 0; i < dim; i++) { v[i] /= n; }
}

float mfccCosine(const float *a, const float *b, int dim)
{
    if (a == nullptr || b == nullptr || dim <= 0) { return 0.0f; }
    double dot = 0.0, na = 0.0, nb = 0.0;
    for (int i = 0; i < dim; i++) {
        dot += (double)a[i] * (double)b[i];
        na  += (double)a[i] * (double)a[i];
        nb  += (double)b[i] * (double)b[i];
    }
    if (na < 1e-12 || nb < 1e-12) { return 0.0f; }
    return (float)(dot / (sqrt(na) * sqrt(nb)));
}

/* =============================================================================
 *  自检：用合成的"元音"信号验证整条链路
 * =============================================================================
 *  为什么要离线自检：
 *    真实音频的 MFCC 没法"一眼看出对不对"。但合成信号可以：
 *    一个纯正弦经过 MFCC 后，能量应该集中在它对应的那个 mel 频带，
 *    倒谱系数也应该稳定。如果 FFT 或滤波器组写错了，这里立刻能看出来。
 * ========================================================================== */
void mfccSelfTest(void)
{
    buildTables();

    Serial.println("---- MFCC SELF TEST ----");

    /* 用 300Hz（接近男声基频）合成 0.5 秒信号，叠加一点噪声 */
    static uint8_t sig[MFCC_SAMPLE_RATE / 2];
    const int N = sizeof(sig);
    for (int i = 0; i < N; i++) {
        const float t = (float)i / (float)MFCC_SAMPLE_RATE;
        const float v = 60.0f * sinf(2.0f * (float)M_PI * 300.0f * t)
                      + 20.0f * sinf(2.0f * (float)M_PI * 1200.0f * t);
        int q = (int)(128.0f + v);
        if (q < 0) { q = 0; }
        if (q > 255) { q = 255; }
        sig[i] = (uint8_t)q;
    }
    Serial.printf("  synth: 300Hz + 1200Hz, %d samples\n", N);

    float feat[MFCC_FEAT_DIM];
    const int nf = mfccCompute(sig, N, feat);
    Serial.printf("  frames=%d\n", nf);
    if (nf <= 0) { Serial.println("  !! mfccCompute returned 0"); return; }

    Serial.print("  MFCC: ");
    for (int i = 0; i < MFCC_FEAT_DIM; i++) {
        Serial.printf("%.2f ", feat[i]);
    }
    Serial.println();

    /* 相似度自检：同一信号算两次应该几乎完全一致（≈1.0）*/
    float feat2[MFCC_FEAT_DIM];
    mfccCompute(sig, N, feat2);
    Serial.printf("  self cosine = %.4f (expect ~1.0000)\n",
                  mfccCosine(feat, feat2, MFCC_FEAT_DIM));

    /* 对照：换一个频率（800Hz），相似度应该明显下降 */
    for (int i = 0; i < N; i++) {
        const float t = (float)i / (float)MFCC_SAMPLE_RATE;
        const float v = 60.0f * sinf(2.0f * (float)M_PI * 800.0f * t);
        int q = (int)(128.0f + v);
        if (q < 0) { q = 0; }
        if (q > 255) { q = 255; }
        sig[i] = (uint8_t)q;
    }
    float feat3[MFCC_FEAT_DIM];
    mfccCompute(sig, N, feat3);
    Serial.printf("  diff cosine = %.4f (expect clearly < 1.0)\n",
                  mfccCosine(feat, feat3, MFCC_FEAT_DIM));

    Serial.println("------------------------");
}
