/* =============================================================================
 *  main.cpp — ESP32-S3 上位机入口
 * =============================================================================
 *
 *  ┌───────────────────────────────────────────────────────────────────────────┐
 *  │                        任务与队列的数据流（plan.md §7.2）                  │
 *  └───────────────────────────────────────────────────────────────────────────┘
 *
 *   F411 (从机 0x01)                                                     云端
 *        ▲   RS485 两线 8N1 115200                          ┌──────────────┐
 *        │                                                    │ MQTT broker  │
 *        │  FC03/04/06/10                                     └──────┬───────┘
 *        │                                                           │ WiFi
 *  ┌─────┴──────────────── core 0 ─────────────────┐   ┌─────────────┴────────┐
 *  │                                              │   │  task_net  (P3)      │
 *  │  task_modbus_bus (P6)  ★唯一总线持有者★        │   │  telemetry/ 上行     │
 *  │    ← g_mb_req_queue ── 串行收发 + 重试 + 统计  │   │  cmd/ 下行 ──┐       │
 *  └─────▲───────────────▲────────────────▲───────┘   └──────────────┼───────┘
 *        │               │                │                          │
 *   mb_read_holding  mb_read_holding  mb_write_*                    │
 *        │               │                │                          │
 *  ┌─────┴──────┐  ┌─────┴──────┐   ┌─────┴──────┐                   │
 *  │task_modbus │  │task_modbus │   │ task_cmd   │◀──────────────────┘
 *  │_slow  (P5) │  │_fast  (P5) │   │   (P5)     │◀──── g_cmd_queue ──┐
 *  │ 500ms      │  │ 2s         │   │ 事件驱动    │                    │
 *  └──┬──────┬──┘  └──┬──────┬──┘   └────────────┘                    │
 *     │      │        │      │                                        │
 *     │      │        │      └────► g_acq_queue ──► task_infer (P3) ──┤
 *     │      │        │             (窗口/特征)      TFLite Micro      │
 *     │      │        │                                   │           │
 *     │      │        └────► g_ui_queue ───────────────────┤           │
 *     │      └─────────────► g_ui_queue ───────────────────┤           │
 *     │                                                     │           │
 *     │  ┌──────────── core 1 ────────────┐                 │           │
 *     └─►│  task_gui (P4)  LVGL + 触摸     │◀────────────────┘           │
 *        │  gui_tick(): lv_timer_handler() │                             │
 *        │  + 排空 g_ui_queue              │──── 按钮 ──► g_cmd_queue ───┘
 *        └─────────────────────────────────┘
 *
 *  队列一览（深度见 config.h）：
 *    g_acq_queue  acq_item_t  窗口 / 36 维特征 → 推理
 *    g_ui_queue   ui_msg_t    环境快照 / 窗口 / 推理结果 / 在线状态 → GUI
 *    g_cmd_queue  cmd_msg_t   GUI 按钮 / MQTT 下行 → 写车道
 *  不经队列的共享数据：
 *    慢车道寄存器快照（seqlock 双缓冲）、窗口三缓冲池（PSRAM）、mb_stats_t（临界区）
 *
 *  ★硬约束★：只有 task_modbus_bus 可以碰 RS485 与 DE 引脚；
 *            其它任务一律通过 g_mb_req_queue 或同步 API 提交请求。
 *            详见 modbus_master.h 顶部。
 * =============================================================================
 */

#include <Arduino.h>

#include "config.h"
#include "modbus_master.h"
#include "app_tasks.h"
#include "gui.h"
#include "net.h"
#include "infer.h"
#include "oled.h"

/* 板级/编译期信息，排错时第一眼要看的东西 */
static void print_boot_banner(void)
{
    APP_LOGI("==========================================================");
    APP_LOGI(" Modbus 边缘智能网关 · 上位机 (ESP32-S3)");
    APP_LOGI(" 从机地址 0x%02X  波特率 %d 8N1  慢车道 %dms  快车道 %dms",
             MODBUS_SLAVE_ADDR, MODBUS_BAUD, MODBUS_SLOW_PERIOD_MS, MODBUS_FAST_PERIOD_MS);
    APP_LOGI(" 窗口：0x%04X~0x%04X（%d 个寄存器，分 %d 块 %d/%d/%d/%d/%d）",
             MODBUS_WIN_DATA_START, (unsigned)(MODBUS_WIN_DATA_START + MODBUS_WIN_DATA_REGS - 1),
             MODBUS_WIN_DATA_REGS, MODBUS_WIN_CHUNK_COUNT, 125, 125, 125, 125, 100);
    APP_LOGI(" 特征：0x%04X ~ 0x%04X（%d 维）",
             MODBUS_FEAT_START, (unsigned)(MODBUS_FEAT_START + MODBUS_FEAT_COUNT - 1),
             INFER_FEATURE_DIM);
#if USE_SIM_SLAVE
    APP_LOGI(" 模式：**仿真从机**（USE_SIM_SLAVE=1）→ 连 PC 上的 tools/modbus_slave_sim.py");
#else
    APP_LOGI(" 模式：真机（USE_SIM_SLAVE=0）");
#endif
    APP_LOGI(" PSRAM: %s", psramFound() ? "有" : "无（内存预算会爆，见 platformio.ini）");
    if (psramFound()) {
        APP_LOGI(" PSRAM 空闲: %u 字节 / 内部 RAM 空闲: %u 字节",
                 (unsigned)ESP.getFreePsram(), (unsigned)ESP.getFreeHeap());
    }
    APP_LOGI("==========================================================");
}

void setup()
{
    /* ---- 0) 调试串口：第一个初始化，后面的日志才有地方去 ---- */
    Serial.begin(115200);
    delay(50);                      /* 给 USB/CH340 枚举一点时间，别丢掉前几行 */
    print_boot_banner();

    /* ---- 1) 看门狗：先起来，后面每个任务自己登记 ---- */
    app_wdt_init();

    /* ---- 2) 队列 + 窗口池（PSRAM） ----
     * 必须在 mb_init() 之前：总线初始化会打自检日志，也可能往 UI 队列投消息。 */
    app_queues_init();

    /* ---- 3) Modbus 主站：UART1 + DE + 请求队列 + 帧格式自检 ----
     * 这里的自检会把 docs/02 §3.1 的 CRC 向量与 §6 的示例帧逐字节比一遍，
     * 失败会打 ERROR —— 见到 ERROR 就先修协议层，别往下调联调。 */
    mb_init();

    /* ---- 3.5) 本地 OLED（SSD1306，I2C）----
     * 为什么必须在这里（而不是在某个任务里）初始化 I2C 总线：
     *   总线初始化本身不是线程安全的（要配 GPIO 矩阵 + 装 ISR），
     *   在单线程的 setup() 里做完最省心；之后只有 task_oled 碰这条 I2C。
     * 失败（没插屏/接错线）不是致命错误：返回 false，由 task_oled 周期重试，
     * 总线/推理/网络全都不受影响 —— 显示是"可选的观测手段"，不是必需品。 */
    (void)oled_init();

    /* ---- 4) 推理：分配 arena、加载模型 ----
     * 失败不致命（模型可能还没训出来），GUI 上会显示"模型未部署"。 */
    (void)infer_init();

    /* ---- 5) 建 7 个任务（核 / 优先级 / 栈 见 config.h §2） ---- */
    if (!app_tasks_start()) {
        APP_LOGE("任务创建失败，系统不会正常工作（先看是不是 PSRAM 没配上）");
    }

    /* ---- 6) 网络：最后起（WiFi 会吃掉大量内部 RAM 和 CPU，
     *         先让总线/GUI 把内存占稳，避免创建任务时才发现不够） ---- */
    net_init();

    APP_LOGI("setup() 完成，进入 loop()（只剩 vTaskDelay）");
}

void loop()
{
    /* loop() 故意什么都不做：
     *   · 所有工作都在 setup() 建的 7 个任务里；
     *   · 这里如果塞逻辑，就会占着 core 1（Arduino 的 loopTask 跑在 core 1）
     *     和 GUI 任务抢时间片，表现为界面刷新不稳。
     *   · 让出时间片即可。 */
    vTaskDelay(pdMS_TO_TICKS(1000));
}
