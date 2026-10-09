// 自动生成，请勿手改：由 tools/train_mlp.py 从 model.tflite 生成
// 用途：TFLite Micro 部署（ESP32-S3 侧）。tensor arena 建议用
//       heap_caps_malloc(..., MALLOC_CAP_SPIRAM) 放到 PSRAM（plan.md §7.2）。
#ifndef MODEL_DATA_H_
#define MODEL_DATA_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>

// 模型字节数：5584；对齐 16 字节（TFLite Micro 要求 16 字节对齐）
extern const unsigned char model_data[];
extern const unsigned int model_data_len;

#ifdef __cplusplus
}
#endif

#endif  // MODEL_DATA_H_
