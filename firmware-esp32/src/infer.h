#pragma once
/* =============================================================================
 *  infer.h — TFLite Micro 端侧推理骨架
 * =============================================================================
 *  数据流（对应 plan.md §6.2 的流水线右侧）：
 *
 *    F411 E 区 0x0400~0x0423（36 个 int16 原始特征）
 *         │  mb_read_holding(0x0400, 36, ...)   ← modbus_master.cpp 快车道顺带读
 *         ▼
 *    acq_queue（ACQ_ITEM_FEATURES）
 *         │
 *         ▼  task_infer（core 0）
 *    ┌──────────────────────────────────────────────────────────────┐
 *    │ 1) 原值 → float：逐维 (raw - mean[i]) / std[i]                │
 *    │ 2) float → int8：q = round(x / input_scale) + input_zero_point│
 *    │    并钳位到 [-128, 127]（量化参数从输入张量里读，见下）        │
 *    │ 3) interpreter->Invoke()                                     │
 *    │ 4) 输出张量 → 反量化 → 概率（模型末尾带 softmax，故反量化即概率）│
 *    └──────────────────────────────────────────────────────────────┘
 *         │
 *         ▼  infer_result_t
 *    g_ui_queue（UI_MSG_INFER → 画标签 + 置信度） + net（MQTT 上报）
 *
 *  ===========================================================================
 *  模型包接口契约（**以 tools/train_mlp.py 的实际产出为准，不是猜的**）
 *  ===========================================================================
 *  生成命令（tools/README.md）：
 *      python train_mlp.py --data data --split data/split.json --out model_out --epochs 200
 *
 *  model_out/ 下会得到（见 train_mlp.py 的 export_c_array / export_norm_params）：
 *
 *    model_data.h / model_data.cc      → **拷进 firmware-esp32/src/**
 *        extern "C" {
 *          extern const unsigned char model_data[];      // alignas(16)
 *          extern const unsigned int  model_data_len;
 *        }
 *
 *    norm_params.h / norm_params.cc    → **拷进 firmware-esp32/src/**
 *        #define FEATURE_DIM               36
 *        #define FEATURE_LAYOUT            "raw_register"
 *        #define MODEL_INPUT_SCALE         <float>
 *        #define MODEL_INPUT_ZERO_POINT    <int>
 *        #define MODEL_OUTPUT_SCALE        <float>
 *        #define MODEL_OUTPUT_ZERO_POINT   <int>
 *        extern "C" {
 *          extern const float feature_mean[FEATURE_DIM];
 *          extern const float feature_std[FEATURE_DIM];   // 注意是 std，不是 1/std
 *        }
 *
 *    model.tflite / norm_params.py / model_info.json / confusion_matrix.txt
 *        → **留在 PC 侧**：norm_params.py 给 tools/verify_tflite.py 用；
 *          model_info.json 里的 "class_names" 是类别名的唯一出处
 *          （train_mlp.py 用的是 `sorted(set(labels))`，即**字母序**）。
 *
 *  ⚠⚠⚠ 必须加 `--feat-layout per-feature` —— 这是本工程最容易踩的坑：
 *    · docs/02 §5-E 规定 E 区是「**每个特征** 6 轴连续」：
 *        0x0400~0x0405 6 轴均值 → 0x0406~0x040B 6 轴标准差 → … → 0x0423
 *        （即 ax_mean, ay_mean, az_mean, gx_mean, gy_mean, gz_mean, ax_std, …）
 *    · tools/train_mlp.py 的**默认** `--feat-layout` 是 `per-axis`
 *        （「每**轴** 6 个特征连续」：ax_mean, ax_std, ax_p2p, …, ay_mean, …），
 *        与 E 区顺序是**转置**关系；
 *    · 两者维度都是 36，模型照跑、不报错，但输入被打乱 → 准确率掉到随机水平。
 *    正确命令：
 *        python tools/train_mlp.py --data data --split data/split.json \
 *               --out model_out --feat-layout per-feature --epochs 200
 *    infer_init() 会用 norm_params.h 的 `FEATURE_LAYOUT` 宏做**硬校验**，
 *    对不上直接拒绝启动推理（见 infer.cpp 步骤 5a）。
 *
 *  ⚠ 三条硬注意事项：
 *   1. 这两个头文件是 `extern "C"` 的，所以固件里**必须 `#include` 它们**。
 *      自己写 `extern const unsigned char model_data[];` 会变成 C++ 链接 → 链不上。
 *   2. train_mlp.py **不在 C 头里导出类别名/类别数**。所以：
 *      · n_classes 从**输出张量的维度**读（运行时的权威值）；
 *      · 类别名表放在 infer.cpp 的 kClassLabels[]，必须与 model_info.json 的
 *        class_names 逐项对齐；对不上时 infer_init() 会打 WARNING 提醒。
 *   3. 没有模型文件时固件**照样要能编、能烧、能跑**（plan.md §8 的 A 表就是
 *      "先交双机联调，模型晚两周"）。这里用 `__has_include` 兜底：
 *      头文件不在 → INFER_HAVE_MODEL/INFER_HAVE_NORM = 0，推理恒返回 false，
 *      GUI 上显示"模型未部署"。所以上面那两个头文件**不要**先建空壳占位。
 * =============================================================================
 */

#include <stdint.h>
#include <stdbool.h>

#include "config.h"

/* -----------------------------------------------------------------------------
 * 推理结果
 * -------------------------------------------------------------------------- */
typedef struct {
    bool     valid;                             /* false = 本次没有有效结果（模型未部署/推理失败） */
    int      n_classes;                         /* 本次模型的类别数（取自输出张量维度） */
    int      best;                              /* argmax 下标，用于查 infer_label() */
    float    probs[INFER_MAX_CLASSES];          /* 各类概率（已归一化，和 ≈ 1.0） */
    uint32_t infer_us;                          /* 本次 Invoke 耗时（微秒），GUI 诊断页显示 */
    uint32_t arena_used_bytes;                  /* tensor arena 实际占用，用来收紧 TFLM_ARENA_SIZE */
    uint16_t win_seq;                           /* 结果对应的窗口序号；0 = 来自特征通道 */
    uint32_t tick;                              /* 出结果的时刻 */
} infer_result_t;

/* -----------------------------------------------------------------------------
 * 初始化：分配 tensor arena（PSRAM）+ Include 模型 + 建 MicroInterpreter + 分配张量。
 * 返回 false 表示模型不可用（未部署 / arena 不够 / 算子缺失 / 维度不一致）——
 * **这不是致命错误**：固件继续跑，GUI 上把推理区置灰。
 * 内部做了幂等保护：重复调用只有第一次真正干活（setup() 与 task_infer 都会调）。
 * -------------------------------------------------------------------------- */
bool infer_init(void);

/* 反初始化（v0.4 换模型 / OTA 时用）。目前留空实现。 */
void infer_deinit(void);

/* -----------------------------------------------------------------------------
 * 单次推理（**核心 API**，签名即需求约定）
 *   features   : 36 维原始 int16 特征（直接来自 0x0400 区，**未标准化**）
 *   probs      : 输出，长度至少 INFER_MAX_CLASSES
 *   n_classes  : 输出，本次模型的类别数
 * 返回 true 表示 probs/n_classes 有效。
 * 内部完成「标准化 → int8 量化 → Invoke → 反量化」四步，耗时记在内部，
 * 通过 infer_last_us() 取。
 * -------------------------------------------------------------------------- */
bool infer_run(const int16_t *features, float *probs, int *n_classes);

/* -----------------------------------------------------------------------------
 * 第二种输入：**已量化好的 int8 向量**，直接灌进输入张量，不做标准化/量化。
 * 专为 tools/verify_tflite.py 的 PC↔MCU 对拍准备（PC 侧已经算好 int8，
 * 若 MCU 这边再 (x-mean)/std 就必然对不上 —— verify_tflite.py 第 255 行专门
 * 警告过这件事）。
 * 返回原始 int8 输出（out_i8 长度 ≥ INFER_MAX_CLASSES），供 PC 直接比较。
 * -------------------------------------------------------------------------- */
bool infer_run_quantized(const int8_t *input_i8, int n_bytes,
                         int8_t *out_i8, int *out_len);

/* 最近一次推理耗时（微秒）；0 = 还没跑过 */
uint32_t infer_last_us(void);

/* -----------------------------------------------------------------------------
 * 「最新结果」的跨任务发布（net 上报 / GUI 显示用）
 * 单写多读：task_infer 写，task_net / task_gui 读，内部用临界区保护。
 * -------------------------------------------------------------------------- */
void infer_publish_latest(const infer_result_t *res);
bool infer_get_latest(infer_result_t *out);

/* 类别名（查 infer.cpp 的 kClassLabels[]，必须与 model_info.json 的 class_names 对齐）。
 * index 越界或模型未部署时返回 "n/a"。 */
const char *infer_label(int index);

/* -----------------------------------------------------------------------------
 * 推理任务（定义在 infer.cpp）：core 0，优先级 TASK_PRIO_INFER。
 * 阻塞在 acq_queue 上，收到什么算什么：
 *   · ACQ_ITEM_FEATURES → infer_run() 直接出结果（36 维特征，MLP，v0.3 主路径）
 *   · ACQ_ITEM_WINDOW   → TODO: 1D-CNN 路径（输入是 100×6 的原始窗口，
 *                          需要第二个模型；plan.md §6.1 把 1D-CNN 作为主任务、
 *                          MLP(36 维特征) 作为对比实验，两条路都保留）。
 * 结果同时投 g_ui_queue（UI_MSG_INFER）和 infer_publish_latest()。
 * -------------------------------------------------------------------------- */
void task_infer(void *arg);

/* =============================================================================
 *  TODO：tools/verify_tflite.py 要求 MCU 侧实现的**串口对拍协议**
 * =============================================================================
 *  这是另一个工具（tools/verify_tflite.py，`--mcu-format int8|class`）已经定死的
 *  接口，**目前固件里还没有实现**。协议原文（该文件第 17~34 行）：
 *
 *    PC → MCU（请求）:  0xA5 0x01 <count> <input bytes...> <checksum>
 *        · 0xA5 同步字节；0x01 = 推理命令
 *        · count = 输入字节数（本项目 = 36）
 *        · input bytes = 36 个 int8，**就是模型输入张量的原始字节**
 *          （PC 侧已经按 norm_params.py 标准化并量化过了）
 *        · checksum = 前面所有字节的 uint8 累加和（不含自身）
 *
 *    MCU → PC（响应）：
 *        int8 模式（默认）：0x5A 0x01 0x24 <int8 输出 ×36> <checksum>
 *        class 模式        ：0x5A 0x02 0x03 <class_id> <conf_lo> <conf_hi> <checksum>
 *
 *  实现要点（补齐时照这个来）：
 *    1. 走**调试串口**（UART0/USB-CDC，也就是 Serial），不是 485 那条总线 ——
 *       所以不受"唯一总线持有者"约束，但要和 APP_LOGI 的日志输出分帧区分开
 *       （建议实现成 main.cpp 的 loop() 或一个低优先级任务里的状态机，
 *         日志统一加前缀，避免把十六进制当成帧头）；
 *    2. 收到 0x01 命令后调 infer_run_quantized()（**不要**调 infer_run()，
 *       否则会二次标准化）；
 *    3. 上层用 --mcu-format 选格式：默认 int8 模式。
 *    4. 没有这条协议时，verify_tflite.py 仍可用 --pc-output-json / --print-input
 *       做"离线对拍"（第 35~37 行），所以它不是阻塞项。
 * =============================================================================
 */
