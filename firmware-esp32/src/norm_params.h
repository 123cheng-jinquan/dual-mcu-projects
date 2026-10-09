// 自动生成，请勿手改：由 tools/train_mlp.py 生成
// 特征标准化参数 + tflite 量化参数。**必须与训练时使用同一套**，否则 MCU 与 PC 结果不一致。
#ifndef NORM_PARAMS_H_
#define NORM_PARAMS_H_

#ifdef __cplusplus
extern "C" {
#endif

#define FEATURE_DIM 36
#define FEATURE_LAYOUT "per-feature"

// 量化参数
#define MODEL_INPUT_SCALE       0.08751462399959564f
#define MODEL_INPUT_ZERO_POINT  -9
#define MODEL_OUTPUT_SCALE      0.00390625f
#define MODEL_OUTPUT_ZERO_POINT -128

// 标准化：x_norm = (raw - feature_mean[i]) / feature_std[i]
extern const float feature_mean[FEATURE_DIM];
extern const float feature_std[FEATURE_DIM];

#ifdef __cplusplus
}
#endif

#endif  // NORM_PARAMS_H_
