/* =============================================================================
 *  main.cpp — ESP32-S3 声纹门禁网关
 * =============================================================================
 *  在整体架构里的位置（双片分工）
 *  ---------------------------------------------------------------------------
 *      STM32F103RET6（采集节点）            ESP32-S3（推理网关，本工程）
 *      ─────────────────────────            ─────────────────────────────
 *        MAX9814 麦克风 → 8kHz ADC+DMA
 *        帧能量 VAD 检测语音
 *        采 1 秒（8000 点）
 *        8bit 量化 → 二进制帧 + CRC  ──────→  收帧、校验
 *                                              转回 12bit 量程
 *                                              MFCC 特征提取
 *                                              与注册模板算余弦相似度
 *        舵机开锁 / OLED 显示        ←──────  回传 PASS/FAIL
 *
 *  为什么这样分工（和项目1 完全一致的思路）：
 *    F103 是 Cortex-M3 @72MHz **没有 FPU**，算一次 1 秒音频的 MFCC
 *    要几秒；ESP32-S3 是 240MHz 双核 + 向量指令，几十毫秒就能算完。
 *    所以"节点负责实时采集、网关负责重计算"是唯一合理的划分。
 *
 *  本文件当前阶段：**链路验证**
 *  ---------------------------------------------------------------------------
 *    先只做"收音频 + 打印统计"，确认硬件链路通了，再叠加 MFCC/识别。
 *    这是刻意的分层验证 —— 出问题时才分得清是"通信的锅"还是"算法的锅"。
 * ========================================================================== */

#include <Arduino.h>
#include "uart_link.h"
#include "mfcc.h"
#include "speaker.h"

/* ---- 接收缓冲 ---------------------------------------------------------- */
static AudioMeta s_meta;
static uint8_t  *s_audio = nullptr;      /* 8bit 量化样本（8000 点）*/
static uint32_t  s_audioCap = 0;
static uint32_t  s_audioLen = 0;         /* 已收字节数 */
static uint16_t  s_chunksGot = 0;
static uint32_t  s_rxStartMs = 0;
static uint32_t  s_lastChunkMs = 0;      /* 最后一块到达的时间（用于超时兜底）*/
static bool      s_receiving = false;

/* =============================================================================
 *  会话超时（"收不满也要处理"）
 * =============================================================================
 *  为什么需要：如果只按"收满 chunkCount 才处理"来判，一旦两边块数不一致
 *  （比如 F103 还是旧固件的 16 块、ESP32 等 32 块），就会**永远卡在等待**，
 *  而且没有任何输出 —— 完全看不出发生了什么。
 *
 *  更糟的是这个坑真实发生过：F103 改了协议但没重新烧写，
 *  ESP32 就一直 "waiting for audio"，让人以为是接线问题。
 *
 *  所以加一个"数据断了 500ms 就收工"的兜底：即使收不满也处理并报出来，
 *  让问题**可见**（打印 chunks=16/32 一眼就看出对不上）。
 * ========================================================================== */
#define RX_IDLE_TIMEOUT_MS   500

static uint32_t  s_sessions = 0;

/* 十六进制 dump 开关（'d' 命令切换）*/
static bool      s_hexDumpOn = false;

/* 最近一次提取的 MFCC 特征（注册/验证用）*/
static float     s_lastFeat[MFCC_FEAT_DIM];
static bool      s_haveFeat = false;

/* 当前模式：false = 验证，true = 注册 */
static bool      s_enrollMode = false;

/* =============================================================================
 *  收到 META：准备缓冲
 * ========================================================================== */
static void onAudioMeta(const AudioMeta &m)
{
    s_meta = m;

    Serial.printf("\n[META] rate=%lu pts=%lu chunks=%u rawbits=%u txbits=%u\n",
                  (unsigned long)m.sampleRate,
                  (unsigned long)m.sampleCount,
                  (unsigned)m.chunkCount,
                  (unsigned)m.rawBits,
                  (unsigned)m.txBits);

    /* 按声明的样本数准备缓冲。为什么用 malloc 而不是静态数组：
     *   1 秒 8kHz = 8000 字节；以后如果要采 2 秒就是 16000 字节，
     *   静态数组每次都得改（还容易忘）。动态分配按需来。
     *   ⚠ 用 PSRAM：N16R8 有 8MB PSRAM，音频缓冲放那里不占内部 RAM。 */
    const uint32_t need = m.sampleCount;
    if (need > s_audioCap) {
        if (s_audio != nullptr) { free(s_audio); s_audio = nullptr; }
        s_audio = (uint8_t *)ps_malloc(need);
        if (s_audio == nullptr) {
            Serial.printf("[META] !! ps_malloc(%lu) FAILED, fallback to malloc\n",
                          (unsigned long)need);
            s_audio = (uint8_t *)malloc(need);
        }
        s_audioCap = (s_audio != nullptr) ? need : 0;
        Serial.printf("[META] buffer = %lu bytes (%s)\n",
                      (unsigned long)s_audioCap,
                      (s_audio != nullptr) ? "ok" : "FAILED");
    }

    s_audioLen   = 0;
    s_chunksGot  = 0;
    s_rxStartMs  = millis();
    s_lastChunkMs = millis();
    s_receiving  = true;
}

/* =============================================================================
 *  收到音频块：追加到缓冲
 * ========================================================================== */
static void onAudioData(const uint8_t *data, uint16_t len)
{
    if (!s_receiving) {
        /* 没收到 META 就来数据 —— 可能是上次会话的残帧，或对端状态不同步。
         * 丢弃即可，但要计数（说明有问题）。 */
        return;
    }

    const uint32_t space = (s_audioCap > s_audioLen) ? (s_audioCap - s_audioLen) : 0;
    const uint32_t copy  = (len < space) ? len : space;

    if (s_audio != nullptr && copy > 0) {
        memcpy(&s_audio[s_audioLen], data, copy);
    }
    s_audioLen += copy;
    s_chunksGot++;
    s_lastChunkMs = millis();

    /* 每 8 块打一个进度点，避免刷屏；最后一块打完整信息 */
    if ((s_chunksGot % 8) == 0 || s_chunksGot >= s_meta.chunkCount) {
        Serial.printf("[RX] chunk %u/%u  (%lu/%lu bytes)\n",
                      (unsigned)s_chunksGot, (unsigned)s_meta.chunkCount,
                      (unsigned long)s_audioLen, (unsigned long)s_audioCap);
    }
}

/* =============================================================================
 *  一次会话收完后的处理（当前阶段只做统计验证）
 * ========================================================================== */
static void processSession(void)
{
    s_receiving = false;
    s_sessions++;

    const uint32_t rxMs = millis() - s_rxStartMs;
    const bool complete = (s_chunksGot >= s_meta.chunkCount)
                       && (s_audioLen >= s_meta.sampleCount);

    Serial.printf("\n========== SESSION #%lu %s ==========\n",
                  (unsigned long)s_sessions,
                  complete ? "COMPLETE" : "INCOMPLETE(timeout)");
    Serial.printf("[SUM] bytes=%lu/%lu  chunks=%u/%u  took=%lums\n",
                  (unsigned long)s_audioLen, (unsigned long)s_audioCap,
                  (unsigned)s_chunksGot, (unsigned)s_meta.chunkCount,
                  (unsigned long)rxMs);

    /* 收不满时明确报出来 —— 这通常意味着**两边的协议版本不一致**
     * （比如 F103 还是旧的 4kHz/16 块固件，而 ESP32 已按 8kHz/32 块来等）。 */
    if (!complete) {
        Serial.printf("[SUM] !! INCOMPLETE -- expected %u chunks / %lu bytes, "
                      "got %u / %lu\n",
                      (unsigned)s_meta.chunkCount,
                      (unsigned long)s_meta.sampleCount,
                      (unsigned)s_chunksGot, (unsigned long)s_audioLen);
        Serial.println("[SUM] !! hint: did you re-flash the F103 after "
                       "changing the protocol? (rebuild + download)");
    }

    const LinkStats &st = uartLinkStats();
    Serial.printf("[LINK] frames=%lu crcErr=%lu resync=%lu dropped=%lu\n",
                  (unsigned long)st.framesOk, (unsigned long)st.crcErrors,
                  (unsigned long)st.resyncs, (unsigned long)st.bytesDropped);

    if (s_audio == nullptr || s_audioLen == 0) {
        Serial.println("[SUM] !! no audio data");
        uartLinkSendStatus("NO_AUDIO");
        return;
    }

    /* ---- 基本统计：验证"收到的确实是有内容的音频" ----------------------
     * 这里只算最朴素的均值/峰峰值，因为目的只是确认链路通了。
     *   · mean 应该接近 F103 侧打印的 baseline >> 4（8bit 量化后约 95）
     *   · p2p 应该和 F103 侧的 p2p >> 4 大致吻合
     * 如果两边对不上，说明传输过程中有丢字节（CRC 会先报出来）。 */
    uint32_t sum = 0;
    uint8_t  lo = 255, hi = 0;
    for (uint32_t i = 0; i < s_audioLen; i++) {
        const uint8_t v = s_audio[i];
        sum += v;
        if (v < lo) { lo = v; }
        if (v > hi) { hi = v; }
    }
    const uint8_t mean = (uint8_t)(sum / s_audioLen);

    Serial.printf("[AUDIO] 8bit: mean=%u min=%u max=%u p2p=%u\n",
                  (unsigned)mean, (unsigned)lo, (unsigned)hi,
                  (unsigned)(hi - lo));
    /* 换算回 12bit 量程（左移 4 位）便于和 F103 侧打印对比 */
    Serial.printf("[AUDIO] ->12bit: mean=%u min=%u max=%u p2p=%u\n",
                  (unsigned)mean << 4, (unsigned)lo << 4,
                  (unsigned)hi << 4, (unsigned)(hi - lo) << 4);

    /* 传输速率（用于确认 115200 下是否合理；理论约 8000 字节/0.72s）*/
    if (rxMs > 0) {
        Serial.printf("[AUDIO] effective rate = %lu B/s\n",
                      (unsigned long)((uint64_t)s_audioLen * 1000ULL / rxMs));
    }

    /* =========================================================================
     *  ★ 核心：MFCC 特征提取 ★
     * =========================================================================
     *  这是把"1 秒波形"变成"14 维音色特征"的关键一步。
     *  之后所有识别都基于这个特征向量，而不是原始波形。
     * ======================================================================= */
    {
        const uint32_t t0 = millis();
        float feat[MFCC_FEAT_DIM];
        const int nf = mfccCompute(s_audio, s_audioLen, feat);
        const uint32_t dt = millis() - t0;

        if (nf <= 0) {
            Serial.println("[MFCC] !! extraction failed");
            uartLinkSendStatus("MFCC_FAIL");
            return;
        }

        Serial.printf("[MFCC] %d frames, took %lu ms\n",
                      nf, (unsigned long)dt);
        Serial.print("[MFCC] feat = ");
        for (int i = 0; i < MFCC_FEAT_DIM; i++) {
            Serial.printf("%.2f ", feat[i]);
        }
        Serial.println();

        Serial.print("[MFCC] std  = ");
        const float *sd = mfccLastStd();
        for (int i = 0; i < MFCC_FEAT_DIM; i++) {
            Serial.printf("%.2f ", sd[i]);
        }
        Serial.println();

        /* 存到全局，供注册/验证使用 */
        for (int i = 0; i < MFCC_FEAT_DIM; i++) { s_lastFeat[i] = feat[i]; }
        s_haveFeat = true;

        /* =====================================================================
         *  ★ 按当前模式做注册 或 验证 ★
         * =====================================================================
         *  e 模式：把这次的特征存进模板库（可连续录几段提升鲁棒性）
         *  v 模式：与已注册模板比对，判 PASS/FAIL 并回传 F103
         * =================================================================== */
        if (s_enrollMode) {
            spkEnroll(feat);
            spkSave();
            Serial.println("[SUM] enrollment done (switch to 'v' to verify)");
        } else if (spkState().enrolled) {
            bool pass = false;
            const float sim = spkVerify(feat, &pass);

            /* 回传 F103：PASS/FAIL + 置信度 + 相似度(千分比) + 耗时 */
            const uint8_t conf = (uint8_t)(pass ? (sim * 100.0f)
                                                : ((1.0f - sim) * 100.0f));
            const uint16_t simPermille = (uint16_t)((sim + 1.0f) * 500.0f);
            uartLinkSendResult(pass ? UART_VERDICT_PASS : UART_VERDICT_FAIL,
                               conf, simPermille, dt);

            Serial.printf("[SUM] %s  sim=%.3f  (sent to F103)\n",
                          pass ? "== PASS ==" : "== FAIL ==", sim);
        } else {
            Serial.printf("[SUM] only %u template(s) -- need %d. "
                          "press 'e' and speak to enroll.\n",
                          (unsigned)spkState().count, SPK_MIN_TEMPLATES);
        }
    }
}

/* =============================================================================
 *  setup / loop
 * ========================================================================== */
void setup(void)
{
    Serial.begin(115200);
    delay(300);          /* 等 USB/CH343 枚举完，否则开头几行日志会丢 */

    Serial.println();
    Serial.println("=====================================================");
    Serial.println(" ESP32-S3 VoiceLock Gateway (voiceprint access control)");
    Serial.println("=====================================================");
    Serial.printf(" chip=%s  cores=%d  cpu=%dMHz  flash=%luMB  psram=%luKB\n",
                  ESP.getChipModel(), ESP.getChipCores(),
                  ESP.getCpuFreqMHz(),
                  (unsigned long)(ESP.getFlashChipSize() / (1024 * 1024)),
                  (unsigned long)(ESP.getPsramSize() / 1024));

    uartLinkBegin();
    spkBegin();       /* 从 NVS 载入已注册的声纹模板（如果有）*/

    Serial.println("[BOOT] waiting for audio from STM32F103RET6 ...");
    Serial.println("[BOOT] on F103: press 'u' then speak");
    Serial.printf("[BOOT] speaker: %u template(s), threshold=%.2f, mode=%s\n",
                  (unsigned)spkState().count,
                  spkState().threshold,
                  s_enrollMode ? "ENROLL" : "VERIFY");
    if (spkState().count < SPK_MIN_TEMPLATES) {
        Serial.println("[BOOT] !! not enough templates -- press 'e' then "
                       "'u' on F103 to enroll (2~3 times)");
    }
    Serial.println("-----------------------------------------------------");
}

void loop(void)
{
    uartLinkPoll(onAudioMeta, onAudioData);
    uartLinkPollAlt();          /* 对照实验（未开启时是空操作）*/

    /* ---- 链路"有没有数据"监视（每 2 秒报一次）-------------------------
     * 为什么需要这个：
     *   出"一直 waiting"时，最想知道的是"RX 线上到底有没有字节"。
     *   光看"没收到帧"分不清是接线问题还是协议问题。
     *   这里把裸字节速率打出来，一眼就能判断：
     *     0 字节/秒      → 接线问题（TX/RX 接反、没接、没共地）
     *     有字节但收不到帧 → 波特率/电平/干扰
     *     有字节且有帧    → 正常 */
    static uint32_t lastReportMs = 0;
    static uint32_t lastFramesOk = 0;
    if ((millis() - lastReportMs) >= 2000) {
        lastReportMs = millis();
        const uint32_t rate  = uartLinkLastSecondBytes();
        const LinkStats &st  = uartLinkStats();
        const bool gotFrames = (st.framesOk != lastFramesOk);
        lastFramesOk = st.framesOk;

        if (rate == 0 && !gotFrames && s_sessions == 0) {
            /* 静默期：只在"从来没有数据"时提示，避免正常运行时也刷屏 */
            static bool warned = false;
            if (!warned) {
                warned = true;
                Serial.println("[LINK] still no data on RX -- check wiring:");
                Serial.println("       F103 PB10(TX) -> ESP32 GPIO5(RX)");
                Serial.println("       F103 PB11(RX) <- ESP32 GPIO4(TX)");
                Serial.println("       GND  <->  GND   (must be common!)");
                Serial.println("       press 'i' for details, 'd' to hex-dump");
            }
        } else if (rate > 0) {
            Serial.printf("[LINK] rx %lu B/s  frames=%lu crcErr=%lu resync=%lu\n",
                          (unsigned long)rate,
                          (unsigned long)st.framesOk,
                          (unsigned long)st.crcErrors,
                          (unsigned long)st.resyncs);
        }
    }

    if (s_receiving && s_meta.chunkCount > 0) {
        /* 收满（正常路径）或数据断了 500ms（兜底路径）都处理。
         * 为什么要兜底：见 RX_IDLE_TIMEOUT_MS 的说明 —— 收不满时如果什么都不做，
         * 现象是"永远 waiting，毫无线索"。 */
        const bool full     = (s_chunksGot >= s_meta.chunkCount);
        const bool idleDone = (s_chunksGot > 0)
                           && ((millis() - s_lastChunkMs) > RX_IDLE_TIMEOUT_MS);
        if (full || idleDone) {
            processSession();
        }
    }

    /* 串口命令（人工触发，方便调试）
     * ---------------------------------------------------------------------
     * ⚠ 命令统一**转成大写再分发**，目的是让大小写都不敏感。
     *   踩过的坑：最初 'X'（交叉检查）只认大写，用户按小写 x 就没反应，
     *   还以为是程序卡了 —— 这种"命令没大小写兼容"的设计很浪费时间。
     *   所以这里把输入统一 upper()，switch 里只写大写字母即可。
     * ------------------------------------------------------------------- */
    if (Serial.available() > 0) {
        int c = Serial.read();
        if (c >= 'a' && c <= 'z') { c = c - 'a' + 'A'; }   /* 统一大写 */
        switch (c) {
        case 'I': {
            const LinkStats &st = uartLinkStats();
            Serial.printf("[INFO] rawBytes=%lu  lastSec=%lu B/s\n",
                          (unsigned long)uartLinkRawBytes(),
                          (unsigned long)uartLinkLastSecondBytes());
            Serial.printf("[INFO] frames=%lu crcErr=%lu resync=%lu dropped=%lu "
                          "sessions=%lu\n",
                          (unsigned long)st.framesOk, (unsigned long)st.crcErrors,
                          (unsigned long)st.resyncs, (unsigned long)st.bytesDropped,
                          (unsigned long)s_sessions);
            if (uartLinkRawBytes() == 0) {
                Serial.println("[INFO] !! no raw bytes at all -> RX line is silent");
                Serial.println("[INFO]    check: PB10(TX)->GPIO5(RX), "
                               "PB11(RX)<-GPIO4(TX), GND<->GND");
            }
            break;
        }
        case 'D':
            s_hexDumpOn = !s_hexDumpOn;
            uartLinkSetHexDump(s_hexDumpOn);
            break;
        case 'Y':   /* ESP32 UART 硬件诊断（读引脚实际电平）*/
            uartLinkDiag();
            break;
        case 'L':   /* UART 回环测试：把 TX 用线接到 RX 后运行 */
            uartLinkLoopbackTest();
            break;
        case 'M':   /* MFCC 算法离线自检（合成信号，不需要 F103）*/
            mfccSelfTest();
            break;
        case 'E':   /* 进入注册模式：下一次收到音频就存为模板 */
            s_enrollMode = true;
            Serial.println("[MODE] ENROLL -- now on F103 press 'u' and speak");
            break;
        case 'V':   /* 进入验证模式 */
            s_enrollMode = false;
            Serial.println("[MODE] VERIFY -- now on F103 press 'u' and speak");
            break;
        case 'S':   /* 查看声纹状态 */
            spkDump();
            break;
        case 'C':   /* 清空模板 */
            spkClear();
            spkSave();
            break;
        case 'X':   /* 模板交叉验证（标定阈值用）*/
            spkCrossCheck();
            break;
        case 'T': {  /* 调整阈值：T<数值>，例如 T0.65 */
            /* 简化处理：直接读下一段数字 */
            String num = Serial.readStringUntil('\n');
            num.trim();
            const float t = num.toFloat();
            if (t > 0.0f && t <= 1.0f) {
                /* threshold 是 const 成员，用 const_cast 修改（内部调试用）*/
                const_cast<SpkState &>(spkState()).threshold = t;
                spkSave();
                Serial.printf("[SPK] threshold set to %.3f\n", t);
            } else {
                Serial.printf("[SPK] bad value '%s' (use e.g. T0.65)\n",
                              num.c_str());
            }
            break;
        }
        case 'H':   /* 帮助（大小写都可以，内部已统一大写）*/
            Serial.println("======== ESP32 VoiceLock commands ========");
            Serial.println("  E : enter ENROLL mode (next audio -> template)");
            Serial.println("  V : enter VERIFY mode (next audio -> PASS/FAIL)");
            Serial.println("  X : cross-check templates (for threshold tuning)");
            Serial.println("  S : show speaker state (mean/std/threshold)");
            Serial.println("  C : clear all templates");
            Serial.println("  T<v> : set threshold, e.g. T0.65");
            Serial.println("  M : MFCC algorithm self-test (no F103 needed)");
            Serial.println("  ---- link diagnostics ----");
            Serial.println("  I : link stats        Y : pin levels");
            Serial.println("  L : UART loopback     D : hex dump");
            Serial.println("  A/Z/B : alt-UART experiment / stop / compare");
            Serial.println("  P : send status       R : fake PASS   F : fake FAIL");
            Serial.println("==========================================");
            break;
        case 'A':   /* 对照实验：UART1 也挂到同一对引脚上听 */
            uartLinkAltBegin();
            break;
        case 'Z':   /* 停止对照实验 */
            uartLinkAltEnd();
            break;
        case 'B': {  /* 对照实验结论 */
            const uint32_t u2 = uartLinkLastSecondBytes();
            const uint32_t u1 = uartLinkAltBytes();
            Serial.println("---- COMPARE ----");
            Serial.printf("  UART2 (IO%d RX) : %lu B/s\n",
                          F103_PIN_RX, (unsigned long)u2);
            Serial.printf("  UART1 (IO%d RX) : %lu B/s   (alt experiment)\n",
                          F103_PIN_RX, (unsigned long)u1);
            if (u1 == 0 && u2 == 0) {
                Serial.println("  => NO SIGNAL on the pin at all.");
                Serial.println("     pin has no data: wrong pin / wire / no common GND");
            } else if (u1 > 0 && u2 == 0) {
                Serial.println("  => UART1 sees data but UART2 does not.");
                Serial.println("     => try changing F103_UART_NUM to 1 in uart_link.h");
            } else if (u2 > 0 && u1 == 0) {
                Serial.println("  => UART2 works fine.");
            } else {
                Serial.println("  => both see data (fine).");
            }
            break;
        }
        case 'P':
            uartLinkSendStatus("PING_FROM_ESP32");
            Serial.println("[INFO] status sent to F103");
            break;
        case 'R':   /* 模拟一次 PASS，验证回传链路 */
            uartLinkSendResult(UART_VERDICT_PASS, 90, 850, 42);
            Serial.println("[INFO] fake PASS result sent to F103");
            break;
        case 'F':   /* 模拟一次 FAIL */
            uartLinkSendResult(UART_VERDICT_FAIL, 20, 120, 41);
            Serial.println("[INFO] fake FAIL result sent to F103");
            break;
        default:
            break;
        }
    }
}
