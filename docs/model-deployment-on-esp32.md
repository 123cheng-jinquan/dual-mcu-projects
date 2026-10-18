# 模型是怎么部署到 ESP32 上的

> 面试高频追问。这里按"训练侧 → 部署侧 → 难点"讲一遍，
> 每一句都能在代码里找到对应位置。

---

## 一、一句话流程

```
PC 上训练 MLP  →  量化成 int8 的 .tflite（5.5 KB）
      →  转成 C 数组编进固件  →  ESP32 上用 TFLite Micro 解释执行
```

**没有"烧模型文件"这一步** —— 模型是**固件的一部分**，跟代码一起编译进去的。

---

## 二、训练侧（PC）：4 个产物

跑一次 `python tools/train_mlp.py --data data --out model_out`，会生成 4 个文件：

| 文件 | 大小 | 作用 |
|---|---|---|
| `model.tflite` | **5584 字节** | 量化后的模型 |
| `model_data.cc` / `.h` | 35 KB（文本形式）| 把 tflite 逐字节写成 C 数组 |
| `norm_params.cc` / `.h` | 2 KB | **归一化参数 + 量化参数** |
| `model_info.json` | 4 KB | 类别表、准确率、混淆矩阵（给人看的）|

### 关键动作 1：int8 全整数量化

```python
converter = tf.lite.TFLiteConverter.from_keras_model(model)
converter.optimizations = [tf.lite.Optimize.DEFAULT]
converter.representative_dataset = representative_dataset   # ← 校准
converter.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS_INT8]
converter.inference_input_type  = tf.int8
converter.inference_output_type = tf.int8
```

**为什么必须量化**：

| | float32 | int8 |
|---|---|---|
| 模型体积 | ~22 KB | **5.5 KB** |
| 是否需要 FPU | 需要 | **不需要** |
| ESP32 上速度 | 慢 | **快 3~4 倍** |

**"代表性数据集"是什么**：量化要把 float 权重压到 int8（−128~127），
需要知道输入数据的**实际取值范围**才能算好缩放系数。
所以得喂一批真实样本（就是训练集里抽的）进去"探路"，这叫**校准**。

### 关键动作 2：把 tflite 变成 C 数组

```c
// model_out/model_data.cc（自动生成，不要手改）
alignas(16) const unsigned char model_data[] = {
    0x1c, 0x00, 0x00, 0x00, 0x54, 0x46, 0x4c, 0x33, ...   // "TFL3" 魔数
};
const unsigned int model_data_len = 5584;
```

**三个细节**：

| 细节 | 为什么 |
|---|---|
| `alignas(16)` | TFLite Micro 要求模型数据 **16 字节对齐**，不对齐会读到错位的数据（不报错，只出错结果）|
| `extern "C"` | 生成的是 C 数组，C++ 里要包一层防止名字修饰 |
| 注释写"请勿手改" | 重跑训练脚本会覆盖 |

---

## 三、部署侧（ESP32）：三步初始化

### 第 1 步：分配 tensor arena（张量内存池）

```c
static constexpr size_t kArenaSize = TFLM_ARENA_SIZE;   // 48 KB
s_arena = (uint8_t *)heap_caps_malloc(kArenaSize, MALLOC_CAP_SPIRAM);
```

**TFLite Micro 不用 malloc** —— 它要求你**预先给一块连续内存**（arena），
所有中间张量都从这块地里切。这叫"静态内存分配"，是嵌入式的标准做法
（避免运行时碎片化）。

**为什么放 PSRAM**（这点很关键）：
- ESP32-S3 内部 RAM 只有 512 KB，而 7 个任务栈约 40 KB、LVGL 要几十 KB
- 48 KB 的 arena 挤进内部 RAM 会**导致 WiFi / LVGL 任务创建失败**
- arena 的访问模式是"推理期间集中访问、平时闲着" —— 正好适合 PSRAM

**但中间缓冲留在内部 RAM**：
```c
// 特征标准化 / 量化的中间缓冲
static float s_norm_buf[INFER_FEATURE_DIM];   // 普通静态变量 = 内部 RAM
```
因为 **PSRAM 访问延迟高**，逐元素写 36 次很慢，且 int8 是字节级操作、cache 命中差。

### 第 2 步：注册算子（裁剪的关键）

```c
tflite::MicroMutableOpResolver<8> s_resolver;
s_resolver.AddFullyConnected();
s_resolver.AddSoftmax();
s_resolver.AddReshape();
s_resolver.AddRelu();
s_resolver.AddQuantize();
s_resolver.AddDequantize();
s_resolver.AddAdd();
s_resolver.AddMul();
```

**为什么不用 `AllOpsResolver`**：那会把**全部 100+ 个算子**链进固件，
占几百 KB Flash。8 个够用就只注册 8 个 —— 这也是一种裁剪。

**如果注册漏了算子会怎样**：`AllocateTensors()` 返回 `kTfLiteError`，
**不会告诉你缺哪个** —— 只能靠猜，所以注释里写清楚了每个算子对应什么。

### 第 3 步：加载 + 分配张量

```c
tflite::MicroInterpreter interp(model_data, model_data_len, s_arena, kArenaSize, &s_resolver);
interp.AllocateTensors();          // 按模型结构在 arena 里切张量
```

---

## 四、推理时的数据流（4 步）

```c
// ① 从 E 区寄存器读到 36 维原始特征（int16）
// ② 标准化：用训练时的同一套均值和标准差
float x = ((float)raw[i] - feature_mean[i]) / feature_std[i];

// ③ 量化成 int8（scale / zero_point 来自 norm_params.h）
int8_t q = (int8_t)lrintf(x / MODEL_INPUT_SCALE) + MODEL_INPUT_ZERO_POINT;

// ④ 填输入张量 → Invoke() → 读输出 → 反量化
memcpy(input->data.int8, qbuf, 36);
interp.Invoke();
int8_t out = output->data.int8[class_id];
float prob = (out - MODEL_OUTPUT_ZERO_POINT) * MODEL_OUTPUT_SCALE;
```

**量化公式的直观理解**：
`scale` 是"1 个 int8 单位代表多少实际值"，`zero_point` 是"实际值 0 对应哪个 int8"。
所以 `int8 = 实际值 / scale + zero_point`。

---

## 五、真正的 5 个难点（这才是值得讲的部分）

### 难点 1：归一化参数必须和训练时一模一样

**训练时** PC 上做 `(x - mean) / std`，用的是训练集算出来的 36 个均值和 36 个标准差。

**推理时** MCU 上也做同样的事 —— 所以这 72 个数**必须原封不动导出到固件**。

`norm_params.h` 就是这个作用。如果导出错了、或者 MCU 上自己重算一遍（算出来必然不同），
**模型输出的概率就完全没意义了，但程序不会崩** —— 这是最阴的一类 bug。

### 难点 2：量化参数交叉校验

模型张量里存着 `scale` 和 `zero_point`，`norm_params.h` 里也有一份。
**两份必须一致**，否则量化必然错。

所以代码里加了一道自检：

```c
const float ds = fabsf(s_input->params.scale - (float)MODEL_INPUT_SCALE);
if (ds > fabsf((float)MODEL_INPUT_SCALE) * 0.01f + 1e-9f ||
    s_input->params.zero_point != (int32_t)MODEL_INPUT_ZERO_POINT) {
    APP_LOGE("量化参数不匹配！张量 scale=%.8g zp=%d，norm_params.h 里 scale=%g zp=%d",
             ...);
}
```

**为什么必须校验**：如果模型换了（重新训练）但头文件没更新，
或者反过来 —— **量化参数不一致，推理结果就是垃圾，但程序照常运行、不报错**。
不主动校验的话，你会花几天时间怀疑特征算错了。

### 难点 3：arena 大小要"够用但不浪费"

`kTfLiteError` 如果是因为 arena 不够，**它不会告诉你差多少**。

```c
// 唯一能拿到用量的办法
interp.arena_used_bytes();
```

注释里留了一句经验：**"若实测推理耗时明显超标，优先尝试把 arena 降到能用内部 RAM 的尺寸
（用 arena_used_bytes() 量），而不是改模型结构。"** —— 因为 PSRAM 比内部 RAM 慢，
能塞进内部 RAM 就塞。

### 难点 4：模型对齐

```c
alignas(16) const unsigned char model_data[] = { ... };
```

TFLite Micro 解析模型时按 16 字节边界读。
**如果不对齐**：不崩溃、不报错，但读出来的算子表是错位的 →
`AllocateTensors()` 失败或者推理结果离谱。这种 bug 靠猜是猜不到的。

### 难点 5：算子注册漏了不会明说

`MicroMutableOpResolver<N>` 的 `N` 是模板参数，**超了会编译报错**（这个是好的）。
但**漏注册**某个算子，只在运行期 `AllocateTensors()` 返回失败，
错误码是笼统的 `kTfLiteError`。

所以代码注释里把 8 个算子逐个标注了用途：
```
Quantize / Dequantize  = 训练后 int8 全整数量化常留的边界算子
Add / Mul              = 特征缩放或残差（保险起见）
```

---

## 六、怎么验证部署没错

### 验证 1：PC 端先跑通（不用板子）

`tools/verify_tflite.py` 用同一套 `norm_params.py` 参数，在 PC 上跑 tflite，
和训练时的 Keras 模型对拍 —— **两边结果一致才说明量化没破坏精度**。

### 验证 2：MCU 端启动自检

| 检查项 | 目的 |
|---|---|
| 张量维度自检 | 输入是不是 36 维、输出是不是 2 类 |
| 量化参数交叉校验 | scale / zero_point 一致（见难点 2）|
| 算子注册完整性 | `AllocateTensors()` 是否成功 |

### 验证 3：实测耗时

单次推理 **不到 400 微秒** —— 用 `esp_timer` 量的。

**这个数字的意义**：50Hz 采样周期是 20ms = 20000µs，
所以推理只占 **2%** 的时间片，完全不影响实时采集。

---

## 七、一句话总结（面试可以直接说）

> 模型在 PC 上用 TF 训练，然后做 **int8 全整数量化**把体积压到 5.5 KB，
> 转成 C 数组编进固件。ESP32 上用 **TFLite Micro** 跑，48 KB 的 tensor arena
> 放 PSRAM（避免挤占内部 RAM），只注册用到的 8 个算子。
> 推理前用训练导出的同一套均值和标准差做归一化，再量化成 int8 送进去。
> 另外加了两道自检：张量维度和量化参数，因为这两处出错**程序不会崩、只会结果错**，
> 是最难查的一类问题。
