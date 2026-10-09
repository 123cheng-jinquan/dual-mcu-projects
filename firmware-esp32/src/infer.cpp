/* =============================================================================
 *  infer.cpp — TFLite Micro 端侧推理实现
 * =============================================================================
 *  与 tools/train_mlp.py 的产出严格对齐（接口契约见 infer.h 顶部）：
 *    src/model_data.h    →  model_data[] / model_data_len（extern "C"，alignas(16)）
 *    src/norm_params.h   →  FEATURE_DIM / feature_mean[] / feature_std[]
 *                           + MODEL_INPUT_SCALE / ZERO_POINT（交叉校验用）
 *
 *  这个骨架**把"能跑起来"的部分写实，把"要等数据和硬件"的部分留 TODO**：
 *   [真] arena 分配（PSRAM + 内部 RAM 回退）、模型加载与算子注册、张量维度自检、
 *        标准化参数交叉校验、特征 → 标准化 → int8 量化 → Invoke → 反量化 全流程、
 *        已量化输入的直通路径（给 verify_tflite.py 对拍用）、耗时统计、结果发布。
 *   [TODO] 1D-CNN（原始 100×6 窗口）的第二个模型与输入张量填充；
 *          verify_tflite.py 的串口对拍协议（协议原文见 infer.h 末尾）；
 *          报警联动（置信度阈值 → 蜂鸣器 + MQTT）。
 * =============================================================================
 */

#include "infer.h"
#include "app_tasks.h"

#include <string.h>
#include <math.h>

#include <esp_heap_caps.h>

/* -----------------------------------------------------------------------------
 * 模型包：有就编进来，没有就降级（plan.md §8 的 A 表要求"模型可以晚两周上"）
 *   用 __has_include 而不是 __attribute__((weak))：
 *   因为 train_mlp.py 生成的头文件是 extern "C" 的，自己声明 extern 会变成
 *   C++ 链接 → 链接期报 undefined reference。必须真的把那个头文件 include 进来。
 * -------------------------------------------------------------------------- */
#if __has_include("model_data.h")
  #include "model_data.h"
  #define INFER_HAVE_MODEL 1
#else
  #define INFER_HAVE_MODEL 0
#endif

#if __has_include("norm_params.h")
  #include "norm_params.h"
  #define INFER_HAVE_NORM 1
  /* 训练侧导出的特征维度必须和 E 区长度（config.h 的 INFER_FEATURE_DIM）一致。
   * 对不上是最隐蔽的一类 bug：模型能跑、结果很"合理"、但全是错的。 */
  static_assert(FEATURE_DIM == INFER_FEATURE_DIM,
                "norm_params.h 的 FEATURE_DIM 与 config.h 的 INFER_FEATURE_DIM 不一致："
                "训练用的特征维度和 E 区 0x0400~0x0423 对不上，回去检查 train_mlp.py 的输入");
#else
  #define INFER_HAVE_NORM 0
  /* ---- 还没有训练出模型（norm_params.h 不存在）时的占位参数 ----
   * 给一组"恒等标准化"（mean=0 / std=1），让整条推理链路先编得过、跑得通，
   * 只是输出必然无意义（本来也没有模型）。tools/train_mlp.py 导出
   * norm_params.h 之后，这段会自动让位 ✗→✓。 */
  static const float feature_mean[INFER_FEATURE_DIM] = { 0 };
  static const float feature_std [INFER_FEATURE_DIM] = { 1 };
#endif

#define INFER_HAS_MODEL_PACKAGE (INFER_HAVE_MODEL && INFER_HAVE_NORM)

/* TFLM 头文件。⚠️ 关键教训：
 *   <TensorFlowLite_ESP32.h>（tanakamasayuki 发行版）在多数版本里**只是个空壳头** ✗，
 *   作用仅是让 PlatformIO 的 LDF 找到这个库；它**不提供 tflite:: 任何类型**。
 *   真正的 API 在 tensorflow/lite/micro/... 里。
 *   之前把空壳头排在第一个分支 → INFER_HAVE_TFLM=1 却没有任何 tflite 类型定义 ✗。
 *   正确做法：**优先真正的 micro_* 头**，且逐个单独判断（各版本头文件集合不一致）。 */
#if __has_include(<tensorflow/lite/micro/micro_interpreter.h>)
  #include <tensorflow/lite/micro/micro_interpreter.h>
  #define INFER_HAVE_TFLM 1
#endif
#if __has_include(<tensorflow/lite/micro/micro_mutable_op_resolver.h>)
  #include <tensorflow/lite/micro/micro_mutable_op_resolver.h>
#endif
#if __has_include(<tensorflow/lite/micro/micro_error_reporter.h>)
  #include <tensorflow/lite/micro/micro_error_reporter.h>
#endif
#if __has_include(<tensorflow/lite/schema/schema_generated.h>)
  #include <tensorflow/lite/schema/schema_generated.h>
#endif
#if __has_include(<TensorFlowLite_ESP32.h>)
  #include <TensorFlowLite_ESP32.h>   /* 某些发行版在这里面补了额外声明，包含无害 */
#endif
#ifndef INFER_HAVE_TFLM
  #define INFER_HAVE_TFLM 0
#endif

/* =============================================================================
 * §1  类别名
 * -----------------------------------------------------------------------------
 *  train_mlp.py **不把类别名导出到 C 头**，只在 model_info.json 里给
 *  "class_names"，而且用的是 `sorted(set(labels))`（字母序）。
 *  TODO: 数据集标签定了之后，把这里换成和 model_info.json 逐项一致的顺序。
 *        例如三类振动任务（字母序）：{ "loose", "normal", "unbalance" }
 *  infer_init() 会拿这里的条数和模型输出维度比，不一致就打 WARNING。
 * ============================================================================= */
static const char *const kClassLabels[] = {
    /* TODO: 与 tools/model_out/model_info.json 的 class_names 严格对齐（字母序） */
    "class0",
    "class1",
    "class2",
    "class3",
    "class4",
    "class5",
    "class6",
    "class7",
};
#define INFER_LABEL_COUNT ((int)(sizeof(kClassLabels) / sizeof(kClassLabels[0])))
static_assert(INFER_LABEL_COUNT == INFER_MAX_CLASSES,
              "kClassLabels[] 的条数必须等于 config.h 的 INFER_MAX_CLASSES");

/* =============================================================================
 * §2  常量与模块状态
 * ============================================================================= */

/* arena 大小（config.h 的 TFLM_ARENA_SIZE，48KB 起步） */
static constexpr size_t kArenaSize = TFLM_ARENA_SIZE;

/* 标准化用的中间缓冲 + 预计算的 1/std。
 * 为什么要有中间缓冲（而不是直接写张量内存）：
 *   · 张量内存可能在 PSRAM 里，逐元素穿越 PSRAM 写 36 次很慢，
 *     且 int8 写入是 byte 级操作、cache 命中率差；
 *   · 36 个 float = 144 字节，放**内部 RAM 的栈上**几乎零成本，
 *     先在内部 RAM 算完，再一次性拷进张量。
 * 这就是 infer.h 里说的"哪些缓冲区必须留内部 RAM"的答案。
 * 为什么预计算 1/std：train_mlp.py 导出的是 std（不是 1/std），
 * 而推理要跑成千上万次，把 36 次除法挪到 infer_init() 里做一次。 */
static float s_norm_buf[INFER_FEATURE_DIM];
static float s_inv_std[INFER_FEATURE_DIM];

static uint32_t s_last_us = 0;
static bool     s_ready    = false;
/* infer_init() 只做一次实际工作：setup() 会先调一次（为了尽早暴露内存问题），
 * task_infer 起来后还会调一次。没有这个标志的话失败路径会把 arena 反复分配。 */
static bool     s_init_attempted = false;

/* 「最新结果」的单写多读槽位 */
static infer_result_t s_latest;
static portMUX_TYPE   s_latest_mux   = portMUX_INITIALIZER_UNLOCKED;
static bool           s_latest_valid = false;

#if INFER_HAVE_TFLM
namespace {
/* error reporter：把 TFLM 内部的报错引到串口。用静态对象，避免堆分配。 */
tflite::MicroErrorReporter      s_err;
const tflite::Model            *s_model  = nullptr;
tflite::MicroInterpreter       *s_interp = nullptr;
TfLiteTensor                   *s_input  = nullptr;
TfLiteTensor                   *s_output = nullptr;
uint8_t                        *s_arena  = nullptr;
int                             s_n_out  = 0;

/* 算子注册器容量 8：MLP(36→32→N) + 量化模型常见的边界算子
 *   FullyConnected / Softmax / Reshape  = MLP 本体
 *   Relu                               = 隐层激活（若没被融合掉）
 *   Quantize / Dequantize              = 训练后 int8 全整数量化常留的边界算子
 *   Add / Mul                          = 特征缩放或残差（保险起见）
 * 容量不足的表现：AllocateTensors() 返回 kTfLiteError，
 * 日志里能看到 "Didn't find op for builtin opcode"。 */
tflite::MicroMutableOpResolver<8> s_resolver;

/* 输出张量的最后一个维度 = 类别数（[1,N] 或 [N] 都能取对） */
int tensor_last_dim(const TfLiteTensor *t)
{
    if (t == nullptr || t->dims == nullptr || t->dims->size == 0) {
        return 0;
    }
    return (int)t->dims->data[t->dims->size - 1];
}
}  // namespace
#endif  /* INFER_HAVE_TFLM */

/* =============================================================================
 * §3  初始化
 * ============================================================================= */
bool infer_init(void)
{
#if !INFER_HAVE_TFLM
    APP_LOGW("没有可用的 TFLite Micro 库（检查 platformio.ini 的 lib_deps），推理功能关闭");
    s_ready = false;
    return false;
#elif !INFER_HAS_MODEL_PACKAGE
    APP_LOGW("模型未部署：src/model_data.h 与/或 src/norm_params.h 不存在。");
    APP_LOGW("  生成方法：python tools/train_mlp.py --data data --split data/split.json "
             "--out model_out --epochs 200");
    APP_LOGW("  然后把 model_out/ 下的 model_data.h/.cc 与 norm_params.h/.cc 拷进 firmware-esp32/src/");
    APP_LOGW("  在此之前推理功能关闭，其余功能（总线/GUI/上云）不受影响。");
    s_ready = false;
    return false;
#else
    if (s_interp != nullptr) {
        return s_ready;
    }
    if (s_init_attempted) {
        return s_ready;         /* 已经试过（且失败），别重复分配 arena */
    }
    s_init_attempted = true;

    /* ---- 1) tensor arena ----
     * 为什么放 PSRAM（plan.md §7.2 的硬要求）：
     *   内部 RAM 的分食者太多 —— WiFi/LWIP 约 60KB、LVGL 绘制缓冲 25KB、
     *   7 个任务栈约 40KB、窗口池若退到内部还要 3.7KB。48KB 的 arena 挤进去
     *   必然导致任务创建失败或 WiFi 起不来。
     *   arena 的访问模式是"推理期间集中访问、平时闲置"，正好是 PSRAM 的舒适区。
     * 代价与对策：
     *   · PSRAM 访问延迟高，所以**标准化/量化的中间缓冲留在内部 RAM**
     *     （见 s_norm_buf 的说明），只在最后一次把 36 字节 memcpy 进去；
     *   · 若实测推理耗时明显超标，优先尝试把 arena 降到能用内部 RAM 的尺寸
     *     （用 arena_used_bytes() 量），而不是改模型结构。 */
    s_arena = (uint8_t *)heap_caps_malloc(kArenaSize, MALLOC_CAP_SPIRAM);
    if (s_arena != NULL) {
        APP_LOGI("tensor arena: %u 字节 @ PSRAM", (unsigned)kArenaSize);
    } else {
        APP_LOGW("PSRAM 分配 arena 失败，回退内部 RAM（会挤占 WiFi/LVGL，可能创建任务失败）");
        s_arena = (uint8_t *)heap_caps_malloc(kArenaSize, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (s_arena == NULL) {
            APP_LOGE("arena 分配彻底失败（%u 字节），推理关闭", (unsigned)kArenaSize);
            s_ready = false;
            return false;
        }
    }

    /* ---- 2) 加载 flatbuffer ---- */
    s_model = tflite::GetModel(model_data);
    if (s_model->version() != TFLITE_SCHEMA_VERSION) {
        APP_LOGE("模型 schema 版本 %d != 运行时要求的 %d（tflite 转换器与 TFLM 版本不匹配）",
                 (int)s_model->version(), (int)TFLITE_SCHEMA_VERSION);
        s_ready = false;
        return false;
    }

    /* ---- 3) 注册算子 ---- */
    s_resolver.AddFullyConnected();
    s_resolver.AddSoftmax();
    s_resolver.AddReshape();
    s_resolver.AddRelu();
    s_resolver.AddQuantize();
    s_resolver.AddDequantize();
    s_resolver.AddAdd();
    s_resolver.AddMul();

    /* ---- 4) 建解释器、分配张量 ----
     * MicroInterpreter 自带几百字节状态，放内部堆；大块（张量数据）在 arena 里。 */
    s_interp = new tflite::MicroInterpreter(s_model, s_resolver, s_arena, kArenaSize, &s_err);
    if (s_interp == nullptr) {
        APP_LOGE("MicroInterpreter 创建失败（内部堆不足）");
        s_ready = false;
        return false;
    }
    if (s_interp->AllocateTensors() != kTfLiteOk) {
        APP_LOGE("AllocateTensors 失败：arena 不够(%u 字节)或算子缺失（看上面的 opcode 报错）",
                 (unsigned)kArenaSize);
        s_ready = false;
        return false;
    }

    s_input  = s_interp->input(0);
    s_output = s_interp->output(0);

    /* ---- 5a) **特征布局**硬校验（这一条能救你一整天） ----
     * docs/02 §5-E 规定 E 区是「每个特征 6 轴连续」：
     *     0x0400~0x0405 6 轴均值 → 0x0406~0x040B 6 轴标准差 → … → 0x0423
     * 也就是 ax_mean, ay_mean, az_mean, gx_mean, gy_mean, gz_mean, ax_std, …
     *
     * 而 tools/train_mlp.py 的 **默认** --feat-layout 是 `per-axis`
     * （"每轴 6 个特征连续"：ax_mean, ax_std, ax_p2p, …, ay_mean, …），
     * 与 E 区的顺序**正好是转置关系**。
     *
     * 后果：模型照跑不误、维度也对得上，但 36 维输入被打乱 →
     *       准确率从 90% 掉到随机水平，且没有任何报错。这是最难查的一类 bug。
     * 所以这里做成**硬失败**：必须用
     *     python tools/train_mlp.py ... --feat-layout per-feature
     * 训练导出，才不会错位。
     *
     * ⚠ 如果你将来真的要用 per-axis（比如 F411 侧改了 E 区写入顺序），
     *   请在下面的标准化循环里**显式做一次重排**（并同步改 docs/02），
     *   不要只是把这段检查注释掉。 */
    if (strcmp(FEATURE_LAYOUT, "per-feature") != 0) {
        APP_LOGE("特征布局不匹配：norm_params.h 的 FEATURE_LAYOUT=\"%s\"，"
                 "但 E 区 0x0400~0x0423 是 per-feature（docs/02 §5-E）。"
                 "请重新训练导出："
                 "python tools/train_mlp.py --data data --out model_out --feat-layout per-feature",
                 FEATURE_LAYOUT);
        s_ready = false;
        return false;
    }

    /* ---- 5b) 维度自检：输入必须是 36 维（= E 区 6 轴 × 6 个统计量） ---- */
    const int in_dim = tensor_last_dim(s_input);
    if (in_dim != INFER_FEATURE_DIM) {
        APP_LOGE("模型输入维度 %d != E 区特征维度 %d（docs/02 §5-E），"
                 "检查 train_mlp.py 的输入与 0x0400~0x0423 的对应关系",
                 in_dim, INFER_FEATURE_DIM);
        s_ready = false;
        return false;
    }

    /* ---- 6) 类别数自检（用输出张量维度，不靠头文件） ---- */
    s_n_out = tensor_last_dim(s_output);
    if (s_n_out <= 0 || s_n_out > INFER_MAX_CLASSES) {
        APP_LOGE("模型输出类别数 %d 非法/超过 INFER_MAX_CLASSES(%d)，调大 config.h 的上界",
                 s_n_out, INFER_MAX_CLASSES);
        s_ready = false;
        return false;
    }
    if (s_n_out != INFER_LABEL_COUNT) {
        APP_LOGW("模型输出 %d 类，但 infer.cpp 的 kClassLabels[] 有 %d 条 —— "
                 "类别名会显示成 classN。请按 model_info.json 的 class_names（字母序）改表",
                 s_n_out, INFER_LABEL_COUNT);
    }

    /* ---- 7) 量化参数交叉校验 ----
     * 为什么值得查：norm_params.h 是"跟着哪次训练导出的"很关键。
     * 如果有人把 A 模型的 model_data.h 和 B 模型的 norm_params.h 混着拷了，
     * 张量里的 scale 和宏里的 scale 就会不一致 —— 这时量化必然错，但程序不崩、
     * 只是精度烂，极难发现。启动时对一次就能拦住。
     * ⚠ 只在 int8 张量上比（float 模型没有 scale）。 */
    if (s_input->type == kTfLiteInt8) {
        const float ds = fabsf(s_input->params.scale - (float)MODEL_INPUT_SCALE);
        if (ds > fabsf((float)MODEL_INPUT_SCALE) * 0.01f + 1e-9f ||
            s_input->params.zero_point != (int32_t)MODEL_INPUT_ZERO_POINT) {
            APP_LOGE("量化参数不匹配！张量 scale=%.8g zp=%d，"
                     "norm_params.h 里 scale=%g zp=%d —— "
                     "八成是把不同训练的 model_data.h 与 norm_params.h 混着拷了",
                     (double)s_input->params.scale, (int)s_input->params.zero_point,
                     (double)(float)MODEL_INPUT_SCALE, (int)MODEL_INPUT_ZERO_POINT);
            /* 不 return false：以张量里的值为准（它才是模型真正的参数），
             * 但错误必须显式暴露出来。 */
        }
    }

    /* ---- 8) 预计算 1/std（train_mlp.py 导出的是 std） ---- */
    for (int i = 0; i < INFER_FEATURE_DIM; i++) {
        if (feature_std[i] == 0.0f) {
            APP_LOGW("feature_std[%d] = 0（该维在训练集上是常数），这维标准化后恒为 0", i);
            s_inv_std[i] = 0.0f;
        } else {
            s_inv_std[i] = 1.0f / feature_std[i];
        }
    }

    APP_LOGI("推理就绪：输入 [1,%d] %s，输出 %d 类，arena 用量 %u 字节",
             in_dim,
             (s_input->type == kTfLiteInt8) ? "int8" : "float32",
             s_n_out,
             (unsigned)s_interp->arena_used_bytes());
    s_ready = true;
    return true;
#endif  /* 平台/模型可用性分支 */
}

void infer_deinit(void)
{
#if INFER_HAVE_TFLM
    /* TODO(v0.4 换模型/OTA 时才需要)：
     *   MicroInterpreter 是 new 出来的、arena 是 heap_caps_malloc 出来的，两者都要释放；
     *   并且必须保证此刻没有任务在调用 infer_run()（否则 use-after-free）。
     *   现在没有调用者，先留个骨架。 */
    if (s_interp != nullptr) {
        delete s_interp;
        s_interp = nullptr;
    }
    if (s_arena != nullptr) {
        heap_caps_free(s_arena);
        s_arena = nullptr;
    }
    s_input  = nullptr;
    s_output = nullptr;
    s_model  = nullptr;
    s_n_out  = 0;
#endif
    s_ready = false;
}

/* =============================================================================
 * §4  推理：标准化 → 量化 → Invoke → 反量化
 * ============================================================================= */
bool infer_run(const int16_t *features, float *probs, int *n_classes)
{
    if (features == NULL || probs == NULL || n_classes == NULL) {
        return false;
    }

#if !INFER_HAVE_TFLM
    (void)s_norm_buf;
    (void)s_inv_std;
    return false;
#else
    if (!s_ready || s_input == nullptr || s_output == nullptr) {
        return false;
    }

    /* ---- 步骤 1：标准化 ----
     * E 区的 36 个 int16 是**原始量纲**（均值/标准差是原始 LSB，过零率 ×100，
     * 主频 ×10Hz，能量占比 ×1000），直接用会让 loss 被大数量级维度主导。
     * 参数必须来自训练集统计（feature_mean / feature_std），不能用推理时的数据
     * 现算 —— 那是数据泄漏。
     *
     * 顺序保证：features[i] 与 feature_mean[i] 的 i 必须指同一个量。
     *   两端都是「每特征 6 轴连续」（docs/02 §5-E）：
     *     i = 0..5   → 6 个轴的均值
     *     i = 6..11  → 6 个轴的标准差
     *     i = 12..17 → 峰峰值   i = 18..23 → 过零率
     *     i = 24..29 → 主频     i = 30..35 → 主频带能量占比
     *   infer_init() 已经用 FEATURE_LAYOUT 硬校验过这件事，这里不再重复判断。 */
    for (int i = 0; i < INFER_FEATURE_DIM; i++) {
        s_norm_buf[i] = ((float)features[i] - feature_mean[i]) * s_inv_std[i];
    }

    /* ---- 步骤 2：写入输入张量 ----
     * int8 量化（train_mlp.py 第 645~650 行给的同一套公式）：
     *     q = round(x_norm / input_scale) + input_zero_point，裁进 [-128,127]
     * scale / zero_point **从张量里读**，宏只用来交叉校验
     * （它们由 TFLite 转换器按训练集动态范围算出，换一次模型就变一次）。 */
    if (s_input->type == kTfLiteInt8) {
        const float scale = s_input->params.scale;
        const int   zp    = (int)s_input->params.zero_point;
        int8_t     *dst   = s_input->data.int8;
        if (scale == 0.0f) {
            APP_LOGE("输入张量 scale 为 0，量化参数异常");
            return false;
        }
        for (int i = 0; i < INFER_FEATURE_DIM; i++) {
            float q = roundf(s_norm_buf[i] / scale) + (float)zp;
            if (q > 127.0f)  q = 127.0f;
            if (q < -128.0f) q = -128.0f;
            dst[i] = (int8_t)q;
        }
    } else if (s_input->type == kTfLiteFloat32) {
        /* 未量化的 float 模型：直接拷（plan.md §6.3 的"量化 vs 非量化"对比实验用） */
        memcpy(s_input->data.f, s_norm_buf, sizeof(float) * INFER_FEATURE_DIM);
    } else {
        APP_LOGE("不支持的输入张量类型 %d（只支持 int8 / float32）", (int)s_input->type);
        return false;
    }

    /* ---- 步骤 3：Invoke ---- */
    const uint32_t t0 = (uint32_t)micros();
    if (s_interp->Invoke() != kTfLiteOk) {
        APP_LOGE("Invoke() 失败");
        s_last_us = (uint32_t)micros() - t0;
        return false;
    }
    s_last_us = (uint32_t)micros() - t0;

    /* ---- 步骤 4：读输出、反量化 ----
     * 约定：模型最后一层是 Softmax（train_mlp.py 的 Dense(..., activation="softmax")），
     *       所以反量化出来就是概率。int8 输出用 (q - zp) * scale 还原。 */
    const int n = s_n_out;
    *n_classes = n;

    for (int i = 0; i < n && i < INFER_MAX_CLASSES; i++) {
        float p = 0.0f;
        if (s_output->type == kTfLiteInt8) {
            p = ((float)s_output->data.int8[i] - (float)s_output->params.zero_point) *
                s_output->params.scale;
        } else if (s_output->type == kTfLiteFloat32) {
            p = s_output->data.f[i];
        }
        if (!(p >= 0.0f)) {
            p = 0.0f;             /* 顺手挡掉 NaN（!(p>=0) 对 NaN 为真） */
        }
        if (p > 1.0f) {
            p = 1.0f;
        }
        probs[i] = p;
    }
    for (int i = n; i < INFER_MAX_CLASSES; i++) {
        probs[i] = 0.0f;
    }

    /* 归一化：只在和明显偏离 1 时做，避免把量化噪声放大 */
    float sum = 0.0f;
    for (int i = 0; i < n; i++) {
        sum += probs[i];
    }
    if (sum > 1e-6f && (sum < 0.98f || sum > 1.02f)) {
        for (int i = 0; i < n; i++) {
            probs[i] /= sum;
        }
    }
    return true;
#endif  /* INFER_HAVE_TFLM */
}

/* -----------------------------------------------------------------------------
 *  已量化 int8 直通：给 tools/verify_tflite.py 的 PC↔MCU 对拍用。
 *  PC 已经把 36 维特征标准化并量化好了（0xA5 0x01 0x24 <36 个 int8> <cksum>），
 *  这里**绝对不能**再做一次 (x-mean)/std，否则两边输入就不一样了
 *  （verify_tflite.py 第 255 行专门警告过这一点）。
 *  这个函数也是上面那条 TODO 协议要调用的唯一入口。
 * -------------------------------------------------------------------------- */
bool infer_run_quantized(const int8_t *input_i8, int n_bytes,
                         int8_t *out_i8, int *out_len)
{
    if (input_i8 == NULL || n_bytes <= 0) {
        return false;
    }
#if !INFER_HAVE_TFLM
    (void)out_i8;
    (void)out_len;
    return false;
#else
    if (!s_ready || s_input == nullptr || s_output == nullptr) {
        return false;
    }
    if (n_bytes != INFER_FEATURE_DIM) {
        APP_LOGE("对拍输入长度 %d != %d", n_bytes, INFER_FEATURE_DIM);
        return false;
    }
    if (s_input->type != kTfLiteInt8) {
        APP_LOGE("模型输入不是 int8，无法走已量化直通路径（用 infer_run() 那条路）");
        return false;
    }

    memcpy(s_input->data.int8, input_i8, (size_t)n_bytes);

    const uint32_t t0 = (uint32_t)micros();
    if (s_interp->Invoke() != kTfLiteOk) {
        APP_LOGE("Invoke() 失败（对拍路径）");
        return false;
    }
    s_last_us = (uint32_t)micros() - t0;

    if (out_i8 != NULL && out_len != NULL) {
        const int n = s_n_out;
        if (s_output->type == kTfLiteInt8) {
            for (int i = 0; i < n; i++) {
                out_i8[i] = s_output->data.int8[i];
            }
        } else {
            /* float 输出模型：按输出量化参数折回 int8，好让 PC 侧统一比较。
             * TODO: 如果真要给 float 模型做对拍，建议让 PC 侧直接用 --pc-output-json
             *       离线比，比在这里硬折回 int8 更干净。 */
            const float scale = (s_output->params.scale != 0.0f) ? s_output->params.scale : 1.0f;
            const int   zp    = (int)s_output->params.zero_point;
            for (int i = 0; i < n; i++) {
                float q = roundf(s_output->data.f[i] / scale) + (float)zp;
                if (q > 127.0f)  q = 127.0f;
                if (q < -128.0f) q = -128.0f;
                out_i8[i] = (int8_t)q;
            }
        }
        *out_len = n;
    }
    return true;
#endif
}

uint32_t infer_last_us(void)
{
    return s_last_us;
}

/* =============================================================================
 * §5  最新结果发布 / 类别名
 * ============================================================================= */
void infer_publish_latest(const infer_result_t *res)
{
    if (res == NULL) {
        return;
    }
    portENTER_CRITICAL(&s_latest_mux);
    s_latest = *res;
    s_latest_valid = true;
    portEXIT_CRITICAL(&s_latest_mux);
}

bool infer_get_latest(infer_result_t *out)
{
    if (out == NULL) {
        return false;
    }
    portENTER_CRITICAL(&s_latest_mux);
    *out = s_latest;
    const bool ok = s_latest_valid;
    portEXIT_CRITICAL(&s_latest_mux);
    return ok;
}

const char *infer_label(int index)
{
    if (index < 0 || index >= INFER_LABEL_COUNT) {
        return "n/a";
    }
    return kClassLabels[index];
}

/* =============================================================================
 * §6  推理任务
 * ============================================================================= */
void task_infer(void *arg)
{
    (void)arg;

    /* 初始化失败**不终止任务**：模型晚两周才部署（plan.md §8 的 A 表就是这样），
     * 这段时间任务空转，GUI 上把推理区域显示为"模型未部署"即可。 */
    if (!infer_init()) {
        APP_LOGW("推理任务以'无模型'模式运行（收到数据就丢弃）");
    }

    static bool warned_window_path = false;
    acq_item_t it;
    app_wdt_register();

    for (;;) {
        app_wdt_feed();

        if (xQueueReceive(g_acq_queue, &it, pdMS_TO_TICKS(500)) != pdTRUE) {
            continue;               /* 超时仅为了喂狗 */
        }

        infer_result_t res;
        memset(&res, 0, sizeof(res));
        res.tick    = xTaskGetTickCount();
        res.win_seq = it.win_seq;

        if (it.kind == ACQ_ITEM_FEATURES) {
            /* ---- 特征通道：MLP（36 维），v0.3 的主路径 ---- */
            int n = 0;
            if (infer_run(it.features, res.probs, &n)) {
                res.valid     = true;
                res.n_classes = n;
                res.best      = 0;
                for (int i = 1; i < n; i++) {
                    if (res.probs[i] > res.probs[res.best]) {
                        res.best = i;
                    }
                }
            }
            res.infer_us = infer_last_us();
#if INFER_HAVE_TFLM
            res.arena_used_bytes = (s_interp != nullptr)
                                 ? (uint32_t)s_interp->arena_used_bytes() : 0u;
#endif
            infer_publish_latest(&res);

            ui_msg_t m;
            memset(&m, 0, sizeof(m));
            m.kind    = UI_MSG_INFER;
            m.u.infer = res;
            (void)ui_post(&m, 0);

            /* TODO(报警联动): 若 res.best 对应"松动/不平衡"且置信度 > 阈值，
             *   往 g_cmd_queue 投 CMD_CTRL1_SET_BITS(CTRL1_BEEP) 并用
             *   net_publish_alarm() 发 MQTT_TOPIC_ALARM。
             *   阈值要和人一起定（工业场景宁可漏报也别误报），现在先不做。 */

        } else {
            /* ---- 原始窗口通道：1D-CNN，plan.md §6.1 的主任务 ----
             * TODO(1D-CNN 路径，步骤如下)：
             *   1) 再训一个模型（输入 [1,100,6] int8）。注意 train_mlp.py 目前
             *      走的是 FeatureDatasetExtractor / 36 维特征；窗口模型需要它支持
             *      --feature-source window（该参数已存在，见 parse_args），
             *      输入张量形状会是 [1,100,6]。
             *   2) 这里把 it.win->samples[]（行优先 100×6）按**6 轴各自的**均值/方差
             *      标准化（不是 36 维特征那一套），再量化填进张量；
             *   3) 复用 infer_run() 的 Invoke + 反量化部分 —— 为此需要把 infer_run()
             *      拆成 fill_input_mlp() / fill_input_cnn() + invoke_common() 三个函数
             *      （现在先不拆，等第二个模型真的到位再动，避免无谓的抽象）；
             *   4) 结果和特征通道的结果都进 g_ui_queue，GUI 上并排显示做对比实验
             *      （plan.md §6.3 的"原始数据 vs 特征向量"对比）。 */
            if (!warned_window_path) {
                warned_window_path = true;
                APP_LOGW("收到原始窗口（seq=%u）：1D-CNN 通道尚未实现"
                         "（见 task_infer 的 TODO），本次丢弃",
                         (unsigned)it.win_seq);
            }
        }
    }
}
