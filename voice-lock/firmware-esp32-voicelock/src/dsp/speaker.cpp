/* =============================================================================
 *  speaker.cpp — 声纹注册 / 验证实现
 * =============================================================================
 *  见 speaker.h 顶部的算法说明（特别是"为什么不用神经网络"和"相似度度量
 *  为什么必须标准化"这两段）。
 * ========================================================================== */

#include "speaker.h"
#include <math.h>
#include <string.h>
#include <Preferences.h>     /* ESP32 的 NVS，用来持久化模板 */

static SpkState s_st = {};

/* =============================================================================
 *  距离尺度 σ（自动标定，不问用户要参数）
 * =============================================================================
 *  ★ 为什么必须标定（这是第 5 版之后发现的问题）★
 *    高斯核 sim = exp(-d² / (2σ²)) 里的 σ 决定了"多近算像"。
 *    我最初拍脑袋设 σ² = 特征维数（84），结果实测：
 *        用户自己 3 段录音的相似度只有 **0.40**！
 *    反推 d² ≈ 350 —— 远大于 σ² = 84，所以"一点点距离"就把相似度打到很低，
 *    连自己都"不像自己"，阈值根本没法定。
 *
 *  ★ 正确思路：用**用户自己的注册样本**来标定 σ ★
 *    本人不同次录音之间的距离 d_self 就代表"正常的说话波动"。
 *    我们希望这种波动对应"较高的相似度"（比如 0.85），于是：
 *        exp(-d_self² / (2σ²)) = 0.85
 *        ⇒ 2σ² = d_self² / (-ln 0.85) = d_self² / 0.1625
 *    取所有模板两两距离的**中位数**作为 d_self，反解出 σ。
 *    这样阈值就变成了"相对于本人波动幅度"的相对值，**自适应**，
 *    不需要用户去调任何参数。
 * ========================================================================== */
static float s_sigma2 = 84.0f;   /* 默认值，注册后会被自动标定覆盖 */

/* 切比雪夫式保护：σ 不能太小（否则噪声主导）也不能太大（否则失去区分度） */
#define SIGMA2_MIN   15.0f
#define SIGMA2_MAX   2000.0f

/* NVS 命名空间与键 */
static Preferences s_nvs;
static const char *NVS_NS  = "voicelock";
static const char *NVS_KEY = "spk";

/* =============================================================================
 *  维度权重（84 维 = 42 维均值 + 42 维标准差）
 * =============================================================================
 *  为什么要加权：
 *    · 静态 MFCC 的第 0 维（对数能量）受音量/距离影响极大 ⇒ 权重压到最低
 *    · 高阶倒谱系数数值小、受内容影响大 ⇒ 权重递减
 *    · Δ / ΔΔ 反映**发音动作模式**（说话习惯），与内容关系小 ⇒ 权重给足
 *    · 标准差部分表示"起伏程度"，也是有效线索 ⇒ 中等权重
 *
 *  ⚠ 这些权重是按 MFCC 各维的**物理含义**定的先验值，
 *    **不是从 5 段样本里拟合出来的** —— 小样本拟合出的权重不可信。
 * ========================================================================== */
static const float s_dimWeight[MFCC_FEAT_DIM] = {
    /* ============ 前 42 维：动态特征的【均值】 ============ */
    /* -- 静态 MFCC 均值（0~13）-- */
    0.05f, 1.20f, 1.10f, 1.00f, 0.90f, 0.80f, 0.70f,
    0.60f, 0.50f, 0.40f, 0.35f, 0.30f, 0.25f, 0.20f,
    /* -- Δ 均值（14~27）：发音动作速度 -- */
    0.40f, 1.50f, 1.50f, 1.40f, 1.30f, 1.20f, 1.10f,
    1.00f, 0.90f, 0.80f, 0.70f, 0.60f, 0.50f, 0.45f,
    /* -- ΔΔ 均值（28~41）：发音加速度 -- */
    0.30f, 1.30f, 1.30f, 1.20f, 1.10f, 1.00f, 0.90f,
    0.80f, 0.70f, 0.60f, 0.50f, 0.45f, 0.40f, 0.35f,

    /* ============ 后 42 维：动态特征的【标准差】（起伏程度）============
     * 给中等偏下的权重：它是有用线索，但比"均值"更容易受语速/内容影响。 */
    0.05f, 0.80f, 0.75f, 0.70f, 0.65f, 0.60f, 0.55f,
    0.50f, 0.45f, 0.40f, 0.35f, 0.30f, 0.25f, 0.20f,
    0.30f, 1.00f, 1.00f, 0.95f, 0.90f, 0.85f, 0.80f,
    0.75f, 0.70f, 0.65f, 0.60f, 0.55f, 0.50f, 0.45f,
    0.25f, 0.90f, 0.90f, 0.85f, 0.80f, 0.75f, 0.70f,
    0.65f, 0.60f, 0.55f, 0.50f, 0.45f, 0.40f, 0.35f
};

/* =============================================================================
 *  相似度度量（本模块最关键的部分，改了三版才定下来）
 * =============================================================================
 *  ★ 版本 1：直接用原始 MFCC 算余弦
 *      结果：300Hz vs 800Hz 两段完全不同的信号相似度 0.73 —— 太高。
 *      原因：各维数值范围差 100 倍，**第 0 维（对数能量）压倒性主导**，
 *            只要"响度差不多"就像，音色信息被淹没。
 *
 *  ★ 版本 2：对每一维做 z-score（减自己的均值、除以自己的标准差）
 *      结果：**5 段自己的录音之间相似度变成 -0.8 ~ +0.5（负的！）**，更糟。
 *      原因（两个都在）：
 *        ① **MFCC 的绝对值本身就携带说话人信息**。
 *           一个人的声道形状决定了他频谱包络的整体高低 —— 这就是"音色"。
 *           减掉"自己样本的均值"等于**把本人最有辨识度的特征减掉了**。
 *        ② 只有 5 个样本估标准差，估计极不稳定，除以错误的数会导致**符号翻转**
 *           （正是因为减均值减出了"负相关"）。
 *
 *  ★ 版本 3（现在这版）：**不去均值，只做加权 + L2 归一**
 *      · 只做 L2 归一（除以向量长度）⇒ 消除整体幅度的缩放，
 *        但**保留各维之间的相对大小**（也就是保留音色）
 *      · 给每一维固定权重：压低最不稳的"能量维"，抬高最稳的低阶倒谱
 *      · 权重是**先验知识**（从 MFCC 的物理含义来），
 *        **不是从 5 个样本里"学"出来的** —— 小样本下学出来的东西不可信
 *
 *  这个"改了三版"的过程值得在面试时讲：它体现的是
 *  **"不要照搬教科书公式，要看公式在这个具体场景下破坏了什么信息"**。
 * ========================================================================== */

/* =============================================================================
 *  重算各维统计量（**只用于显示和诊断，不再用于相似度计算**）
 * ========================================================================== */
static void recomputeStats(void)
{
    if (s_st.count == 0) { return; }

    for (int d = 0; d < MFCC_FEAT_DIM; d++) {
        double sum = 0.0, sumSq = 0.0;
        for (int i = 0; i < s_st.count; i++) {
            const double v = s_st.feats[i][d];
            sum   += v;
            sumSq += v * v;
        }
        const double mean = sum / (double)s_st.count;
        s_st.mean[d] = (float)mean;

        double var = sumSq / (double)s_st.count - mean * mean;
        if (var < 0.0) { var = 0.0; }
        s_st.std[d] = (float)sqrt(var);
    }
}

/* =============================================================================
 *  相似度度量（第 5 版，最终采用）
 * =============================================================================
 *  ★ 为什么从"余弦相似度"改成"归一化欧氏距离 + 高斯核" ★
 *
 *  余弦相似度只看**向量方向**，会丢掉"幅度"这个信息。而我们的特征现在是
 *  「均值 | 标准差」拼接：
 *    · 均值部分  ：一个人嗓音高低（比如低沉嗓音的 c1 偏大）**就是幅度信息**，
 *                  这正是最强的说话人线索之一
 *    · 标准差部分：起伏程度同样是幅度量纲
 *  ⇒ 用余弦会把这两类最有用的信息都归一化掉，只剩"各维之间的相对比例"，
 *    实测就出现了"自己的录音之间相似度 -0.76"这种诡异结果。
 *
 *  ★ 正确做法：在"标准化空间"里算欧氏距离 ★
 *    ① 用**全球统计量**（所有注册样本汇总）对每一维做 z-score
 *       —— 注意：这里是**跨样本**的全局标准化，不是"减掉每段自己的均值"。
 *          后者（CMN）会把说话人信息一起减掉，那是之前失败的根源。
 *    ② 加权欧氏距离 d² = Σ w[d]² · (a'[d] - b'[d])²
 *    ③ 用高斯核把距离映射成 0~1 的相似度：
 *          sim = exp(-d² / (2σ²))
 *       为什么要非线性映射：距离本身没有"像不像"的直观含义，
 *       而且分布是长尾的。高斯核让"很接近"→ 接近 1，"差一点"→ 快速下降，
 *       比"1/(1+d)"这种映射更符合实际分布，也更容易定阈值。
 *       σ² 取维数级别（= MFCC_FEAT_DIM），使"每维平均差 1 个标准差"
 *       时相似度约 0.6，正好落在常用阈值附近。
 * ========================================================================== */
static float normCosine(const float *a, const float *b)
{
    double d2 = 0.0;

    for (int d = 0; d < MFCC_FEAT_DIM; d++) {
        /* ① 全局 z-score（s_st.mean/std 是**所有注册样本汇总**的统计量，
         *    不是每段自己的 —— 这一点至关重要，见上面说明）*/
        const float sd = (s_st.std[d] > 1e-6f) ? s_st.std[d] : 1.0f;
        const double va = ((double)a[d] - s_st.mean[d]) / sd;
        const double vb = ((double)b[d] - s_st.mean[d]) / sd;

        /* ② 加权欧氏距离 */
        const double diff = (va - vb) * s_dimWeight[d];
        d2 += diff * diff;
    }

    /* ③ 高斯核 → 0~1 相似度（σ 由注册样本自动标定）*/
    return (float)exp(-d2 / (2.0 * (double)s_sigma2));
}

/* 只算距离，不做核变换（标定 σ 时需要）*/
static double normDist2(const float *a, const float *b)
{
    double d2 = 0.0;
    for (int d = 0; d < MFCC_FEAT_DIM; d++) {
        const float sd = (s_st.std[d] > 1e-6f) ? s_st.std[d] : 1.0f;
        const double va = ((double)a[d] - s_st.mean[d]) / sd;
        const double vb = ((double)b[d] - s_st.mean[d]) / sd;
        const double diff = (va - vb) * s_dimWeight[d];
        d2 += diff * diff;
    }
    return d2;
}

/* =============================================================================
 *  自动标定 σ：让"本人的正常波动"对应较高相似度
 * =============================================================================
 *  步骤：
 *    ① 算出所有模板两两之间的 d²
 *    ② 取中位数 median（代表"典型的一次自身波动"）
 *    ③ 令 median 对应目标相似度 TARGET_SIM（0.85），反解 σ²：
 *          target = exp(-median / (2σ²))
 *          ⇒ 2σ² = median / (-ln target)
 *       ⇒ σ² = median / (-2·ln target)
 *  ⚠ 至少要有 2 个模板才有"两两距离"可算；只有 1 个时保留默认 σ。
 * ========================================================================== */
#define TARGET_SELF_SIM   0.85f

static void calibrateSigma(void)
{
    if (s_st.count < 2) {
        s_sigma2 = 84.0f;      /* 样本不够，用保守默认值 */
        return;
    }

    /* 收集所有两两距离（最多 5 个模板 ⇒ 10 对，够用）*/
    double dists[SPK_MAX_TEMPLATES * SPK_MAX_TEMPLATES];
    int n = 0;
    for (int i = 0; i < s_st.count; i++) {
        for (int j = i + 1; j < s_st.count; j++) {
            dists[n++] = normDist2(s_st.feats[i], s_st.feats[j]);
        }
    }
    if (n == 0) { s_sigma2 = 84.0f; return; }

    /* 取中位数（小数组直接插入排序，n 很小）*/
    for (int i = 1; i < n; i++) {
        const double key = dists[i];
        int j = i - 1;
        while (j >= 0 && dists[j] > key) { dists[j + 1] = dists[j]; j--; }
        dists[j + 1] = key;
    }
    const double median = dists[n / 2];

    /* 反解 σ²：median 对应 TARGET_SELF_SIM */
    const double negLnTarget = -log((double)TARGET_SELF_SIM);   /* ≈ 0.1625 */
    double sigma2 = median / (2.0 * negLnTarget);

    /* 限幅，避免极端值 */
    if (sigma2 < SIGMA2_MIN) { sigma2 = SIGMA2_MIN; }
    if (sigma2 > SIGMA2_MAX) { sigma2 = SIGMA2_MAX; }

    s_sigma2 = (float)sigma2;

    Serial.printf("[SPK] sigma calibrated: median d2 = %.1f -> sigma2 = %.1f\n",
                  median, sigma2);
}

/* =============================================================================
 *  对外接口
 * ========================================================================== */
void spkBegin(void)
{
    memset(&s_st, 0, sizeof(s_st));
    s_st.threshold = SPK_THRESHOLD_DEF;
    s_st.sigma2    = 84.0f;
    s_st.magic     = SPK_STATE_MAGIC;
    spkLoad();          /* 尝试从 NVS 恢复 */
}

bool spkEnroll(const float *feat)
{
    if (feat == nullptr) { return false; }
    if (s_st.count >= SPK_MAX_TEMPLATES) {
        Serial.printf("[SPK] templates full (%d). clear first.\n", SPK_MAX_TEMPLATES);
        return false;
    }

    memcpy(s_st.feats[s_st.count], feat, sizeof(float) * MFCC_FEAT_DIM);
    s_st.count++;
    s_st.enrolled = (s_st.count >= SPK_MIN_TEMPLATES);

    recomputeStats();
    calibrateSigma();       /* ★ 每次注册后重新标定距离尺度 ★ */
    s_st.sigma2 = s_sigma2;

    Serial.printf("[SPK] enrolled sample %d/%d%s\n",
                  s_st.count, SPK_MAX_TEMPLATES,
                  s_st.enrolled ? "  (verification ENABLED)" : "  (need more)");
    return true;
}

float spkVerify(const float *feat, bool *pass)
{
    if (feat == nullptr || s_st.count == 0) {
        if (pass) { *pass = false; }
        return 0.0f;
    }

    /* 与每一段注册模板比，取**最高**相似度。
     * 为什么取最高而不是平均：注册样本之间本身有差异（同一人不同次说话），
     * 只要和其中一段明显像，就应该认为"是本人"。
     * 取平均会把"最像的那次"抹平，反而降低通过率。 */
    float best = -1.0f;
    for (int i = 0; i < s_st.count; i++) {
        const float s = normCosine(feat, s_st.feats[i]);
        if (s > best) { best = s; }
    }

    if (pass) { *pass = (best >= s_st.threshold); }

    Serial.printf("[SPK] similarity = %.4f  (threshold %.2f)  => %s\n",
                  best, s_st.threshold, (best >= s_st.threshold) ? "PASS" : "FAIL");
    return best;
}

const SpkState &spkState(void)
{
    return s_st;
}

void spkClear(void)
{
    memset(&s_st, 0, sizeof(s_st));
    s_st.threshold = SPK_THRESHOLD_DEF;
    s_st.sigma2    = 84.0f;
    s_st.magic     = SPK_STATE_MAGIC;
    s_sigma2       = 84.0f;
    Serial.println("[SPK] all templates cleared");
}

/* =============================================================================
 *  NVS 持久化
 * ========================================================================== */
bool spkSave(void)
{
    if (!s_nvs.begin(NVS_NS, false)) {
        Serial.println("[SPK] !! NVS open failed");
        return false;
    }
    /* 直接把整个 SpkState 存成二进制块。
     * 为什么可以这么简单：结构体是纯 POD、大小固定、没有指针，
     * 存/取对称就行。（换编译器/改结构体时要注意版本兼容。） */
    const size_t n = s_nvs.putBytes(NVS_KEY, &s_st, sizeof(s_st));
    s_nvs.end();

    Serial.printf("[SPK] saved %u bytes to NVS\n", (unsigned)n);
    return (n == sizeof(s_st));
}

bool spkLoad(void)
{
    if (!s_nvs.begin(NVS_NS, true)) { return false; }   /* true = 只读 */
    const size_t len = s_nvs.getBytesLength(NVS_KEY);
    if (len != sizeof(s_st)) {
        s_nvs.end();
        return false;                                   /* 没存过，或版本不符 */
    }
    const size_t n = s_nvs.getBytes(NVS_KEY, &s_st, sizeof(s_st));
    s_nvs.end();

    if (n != sizeof(s_st)) { return false; }

    /* ★ magic 校验（很关键）★
     * 特征维度经过 14 → 42 → 84 的变化，旧 NVS 数据的**布局完全不同**。
     * 如果不校验就直接用，会读到错位的值 —— 表现是"相似度乱跳、时好时坏"，
     * 极难排查。加上 magic 后，版本不匹配就自动丢弃并提示重新注册。 */
    if (s_st.magic != SPK_STATE_MAGIC) {
        Serial.printf("[SPK] NVS data version mismatch (magic=0x%08lX, "
                      "expect 0x%08lX) -- discarded, please re-enroll\n",
                      (unsigned long)s_st.magic,
                      (unsigned long)SPK_STATE_MAGIC);
        memset(&s_st, 0, sizeof(s_st));
        s_st.threshold = SPK_THRESHOLD_DEF;
        s_st.sigma2    = 84.0f;
        s_st.magic     = SPK_STATE_MAGIC;
        return false;
    }

    if (s_st.threshold <= 0.0f || s_st.threshold > 1.0f) {
        s_st.threshold = SPK_THRESHOLD_DEF;
    }
    if (s_st.sigma2 < SIGMA2_MIN || s_st.sigma2 > SIGMA2_MAX) {
        s_st.sigma2 = 84.0f;
    }
    /* 恢复标定好的尺度到全局变量（normCosine 用它）*/
    s_sigma2 = s_st.sigma2;

    Serial.printf("[SPK] loaded %u templates from NVS, threshold=%.2f, "
                  "sigma2=%.1f\n",
                  (unsigned)s_st.count, s_st.threshold, s_st.sigma2);
    return true;
}

void spkDump(void)
{
    Serial.println("---- SPEAKER STATE ----");
    Serial.printf("  enrolled=%d  count=%u/%u  threshold=%.2f  sigma2=%.1f\n",
                  (int)s_st.enrolled, (unsigned)s_st.count,
                  SPK_MAX_TEMPLATES, s_st.threshold, s_st.sigma2);
    if (s_st.count == 0) {
        Serial.println("  (no templates)");
        Serial.println("-----------------------");
        return;
    }

    /* ★ 把每一段模板的**原始值**打出来 ★
     * 为什么重要：之前出现"自己的 5 段录音相似度为负"时，
     * 只看相似度矩阵根本猜不出原因。看到原始值才发现是
     * "减均值把本人特征减掉了"。所以这个 dump 是必备的诊断手段。 */
    Serial.println("  raw template values (first 6 dims):");
    Serial.print("        ");
    for (int d = 0; d < 6; d++) { Serial.printf("  dim%-6d", d); }
    Serial.println();
    for (int i = 0; i < s_st.count; i++) {
        Serial.printf("  #%-3d ", i);
        for (int d = 0; d < 6; d++) { Serial.printf("%9.2f ", s_st.feats[i][d]); }
        Serial.println();
    }

    Serial.println("  per-dim mean / std (for reference):");
    for (int d = 0; d < MFCC_FEAT_DIM; d++) {
        Serial.printf("  dim%2d: mean=%9.2f  std=%8.2f  w=%.2f\n",
                      d, s_st.mean[d], s_st.std[d], s_dimWeight[d]);
    }
    Serial.println("-----------------------");
}

/* =============================================================================
 *  交叉验证：用注册模板互相比对，打印相似度矩阵
 * =============================================================================
 *  这是**标定阈值**最实用的工具：
 *    · 矩阵对角线 = 自己和自己 = 应该 ≈ 1.0
 *    · 非对角线   = 同一人的不同次样本 = 应该明显高（> 0.7）
 *    · 再用另一个人的样本跑一次 → 应该明显低（< 0.5）
 *  把这两组数一比，就能定出"卡在中间"的阈值。
 * ========================================================================== */
void spkCrossCheck(void)
{
    if (s_st.count == 0) {
        Serial.println("[SPK] no templates to cross-check");
        return;
    }

    Serial.println("---- TEMPLATE CROSS CHECK ----");
    Serial.print("      ");
    for (int j = 0; j < s_st.count; j++) { Serial.printf("  #%d   ", j); }
    Serial.println();

    float minOff = 1.0f;
    for (int i = 0; i < s_st.count; i++) {
        Serial.printf("  #%d  ", i);
        for (int j = 0; j < s_st.count; j++) {
            const float s = normCosine(s_st.feats[i], s_st.feats[j]);
            Serial.printf("%6.3f ", s);
            if (i != j && s < minOff) { minOff = s; }
        }
        Serial.println();
    }

    if (s_st.count >= 2) {
        Serial.printf("  worst self-similarity (off-diagonal) = %.3f\n", minOff);
        Serial.printf("  => suggested threshold < %.2f\n", minOff * 0.9f);
    }
    Serial.println("------------------------------");
}
