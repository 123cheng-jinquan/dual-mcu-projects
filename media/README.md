# 演示素材（视频 + 截图）

> 这里放的是**一次性录制的原始演示素材**，用于证明两个项目都是**真实跑通的实物**，
> 不是仿真、不是纸上谈兵。

---

## 一、视频

| 文件 | 时长 | 内容 | 对应项目 |
|---|---|---|---|
| [`videos/01-hardware-demo.mp4`](videos/01-hardware-demo.mp4) | 1:12 | 桌面硬件全景：F103RET6 核心板 + ESP32-S3 + MAX9814 麦克风 + 接线，末尾手持 OLED 特写 | **项目2** 声纹门禁 |
| [`videos/02-serial-bootlog.mp4`](videos/02-serial-bootlog.mp4) | 0:50 | 串口终端：上电启动日志 `[BOOT]` / `[AUDIO] 8kHz capture started`，以及 `H` 命令的完整帮助列表 | **项目2** 声纹门禁 |
| [`videos/03-screen-verify.mp4`](videos/03-screen-verify.mp4) | 1:08 | 屏幕录制：两个串口窗口（F103 的 COM8 + ESP32 的 COM6）同步演示验证流程 | **项目2** 声纹门禁 |

**说明**：三支视频是**手机手持拍摄 + 屏幕录制**的原始素材，
没有剪辑、没有配音、没有字幕 —— 目的是**证明真实性**。
正式版演示视频请按 [`../voice-lock/docs/demo-video-script.md`](../voice-lock/docs/demo-video-script.md) 的分镜脚本重录。

**体积**：已用 H.264 CRF 30 + 960p 压缩，48 MB → **10.5 MB**（便于入库）。

---

## 二、截图（从视频中抽取）

| 文件 | 内容 |
|---|---|
| [`frames/01-hardware-overview.jpg`](frames/01-hardware-overview.jpg) | 硬件全景（两块板 + 麦克风 + 接线）|
| [`frames/02-hardware-closeup.jpg`](frames/02-hardware-closeup.jpg) | 接线细节（杜邦线走向）|
| [`frames/03-serial-boot.jpg`](frames/03-serial-boot.jpg) | 串口启动日志 |
| [`frames/04-verify-screen.jpg`](frames/04-verify-screen.jpg) | 双串口验证流程 |
| [`frames/05-oled-held.jpg`](frames/05-oled-held.jpg) | 手持 OLED 特写 |

**用途**：README 里插图、简历附件、答辩 PPT。

---

## 三、素材与项目的对应关系

**目前所有素材都是项目2（声纹门禁）的。**

项目1（Modbus 振动监测网关）的验证方式是**长时间跑数据**而不是拍视频：

| 证据类型 | 位置 |
|---|---|
| **4 小时长跑日志** | [`../logs/`](../logs/)（本地保留，不入库 —— 几十 MB）|
| **长跑结论** | [`../docs/06-decisions-and-results.md`](../docs/06-decisions-and-results.md) |
| **复现脚本** | [`../tools/soak_test.ps1`](../tools/soak_test.ps1) |

> **为什么项目1 不拍视频**：它的核心指标是"**47510 次事务零错误**"，
> 这靠**日志和脚本**证明比靠视频有力得多。
> 项目2 的核心是"**说句话 → 屏幕显示 PASS**"，这是个**交互过程**，视频才说得清。

---

## 四、录制环境

| 项 | 值 |
|---|---|
| 拍摄设备 | 手机（1080p / 30fps）|
| 录音 | 设备本机麦克风 |
| 光线 | 室内自然光 + 台灯 |
| 串口工具 | XCOM V2.6 |
| 分辨率 | 原始 1280×720 → 压缩后 960×540 |

---

## 五、如果要重录（推荐）

按 [`../voice-lock/docs/demo-video-script.md`](../voice-lock/docs/demo-video-script.md) 执行，那里面已经写好了：

- **7 个镜头**的逐条操作、画面、字幕、旁白
- **硬件清单 + 接线表**（含那根最容易漏的 `PB11 ← IO6`）
- **录制技巧**（拍 OLED 必须固定机位、串口字号调 16+、说话 25~30cm）
- **7 个翻车点速查**（`NO_RES` = 回传线没接、削顶导致相似度掉…）
- **面试追问的 7 个问答要点**

**重录后的成品视频建议放 `videos/final-demo.mp4`，原始素材可以删掉以控制仓库体积。**

---

## 六、关于仓库体积

| 项 | 大小 |
|---|---|
| 三支视频（压缩后）| **10.5 MB** |
| 五张截图 | 0.45 MB |
| **media/ 合计** | **约 11 MB** |

**为什么允许视频入库**：
- GitHub 单文件限制 100 MB，**10 MB 完全没问题**
- 视频是**最直接的"项目真实性"证据** —— 面试官/HR 打开仓库就能看到
- 相比"把视频放网盘、链接会失效"，**入库更可靠**

**如果以后要控制体积**，把 `videos/` 移到 Git LFS：
```bash
git lfs install
git lfs track "media/videos/*.mp4"
```
