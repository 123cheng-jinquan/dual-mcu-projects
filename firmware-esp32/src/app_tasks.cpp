/* =============================================================================
 *  app_tasks.cpp — 任务创建、队列、缓冲池、看门狗、写车道
 * =============================================================================
 *  这个文件是"接线盒"：它不实现业务，只负责把 modbus_master / infer / gui / net
 *  四个模块按 plan.md §7.2 的核绑定与优先级接起来。
 * =============================================================================
 */

#include "app_tasks.h"
#include "gui.h"
#include "net.h"
#include "oled.h"

#include <string.h>

#include <esp_heap_caps.h>

/* -----------------------------------------------------------------------------
 * 内核版本适配：esp_task_wdt_init() 的签名在 arduino-esp32 3.x 变了
 *   · 2.x : esp_task_wdt_init(uint32_t timeout_s, bool panic)
 *   · 3.x : esp_task_wdt_init(const esp_task_wdt_config_t *)
 * 用 ESP_ARDUINO_VERSION_MAJOR 区分（esp_arduino_version.h 在 2.0.3+ 都有，
 * 万一没有就按 2.x 处理，2.x 的写法在 3.x 上会编译报错——所以这里必须判准）。 */
#if __has_include(<esp_arduino_version.h>)
#include <esp_arduino_version.h>
#endif
#ifndef ESP_ARDUINO_VERSION_MAJOR
#define ESP_ARDUINO_VERSION_MAJOR 2
#endif
#include <esp_task_wdt.h>

/* =============================================================================
 * §1  队列
 * ============================================================================= */
QueueHandle_t g_acq_queue = NULL;   /* acq_item_t → task_infer */
QueueHandle_t g_ui_queue  = NULL;   /* ui_msg_t   → task_gui   */
QueueHandle_t g_cmd_queue = NULL;   /* cmd_msg_t  → task_cmd   */

/* =============================================================================
 * §2  慢车道寄存器快照（seqlock 双缓冲，单写多读）
 * ============================================================================= */
static slow_regs_t       s_slow_db;
static volatile uint32_t s_slow_seq = 0;
static portMUX_TYPE      s_slow_mux = portMUX_INITIALIZER_UNLOCKED;

void slow_publish(const slow_regs_t *src)
{
    if (src == NULL) {
        return;
    }
    portENTER_CRITICAL(&s_slow_mux);
    s_slow_seq++;                 /* 变奇数 = 正在写，读者请重试 */
    s_slow_db = *src;
    s_slow_seq++;                 /* 变偶数 = 写完 */
    portEXIT_CRITICAL(&s_slow_mux);
}

bool slow_get_latest(slow_regs_t *out)
{
    if (out == NULL) {
        return false;
    }
    /* 重试 8 次：写者只写 60 字节（几十个周期），读者几乎不可能连撞 8 次。
     * 撞满 8 次说明 CPU 被更高优先级任务长期占着，那就让调用方用旧值 —— 
     * 显示一帧旧数据远好于在这里自旋。 */
    for (int i = 0; i < 8; i++) {
        const uint32_t s0 = s_slow_seq;
        if ((s0 & 1u) != 0u) {
            continue;             /* 写者正在写 */
        }
        *out = s_slow_db;
        if (s_slow_seq == s0) {
            return true;
        }
    }
    return false;
}

/* =============================================================================
 * §3  窗口三缓冲池（PSRAM）
 * ============================================================================= */
static acq_window_t *s_win_pool = NULL;
static uint8_t       s_win_next = 0;

void window_pool_init(void)
{
    if (s_win_pool != NULL) {
        return;
    }
    const size_t bytes = sizeof(acq_window_t) * WINDOW_POOL_SLOTS;

    /* 放 PSRAM 的理由：3 × 1200B ≈ 3.7KB，看起来不多，但内部 RAM 里：
     *   WiFi/LWIP 静态约 60KB + LVGL 缓冲 25KB + TFLite arena(若退到内部) 48KB
     *   + 7 个任务栈约 40KB —— 每一 KB 都要抢。窗口缓冲是"写一次读两次"的
     *   大块数据，正是 PSRAM 该承担的。
     * 代价：PSRAM 走 cache，访问比内部 RAM 慢 3~5 倍，所以推理时不要反复穿越
     * 这个大数组（infer.cpp 会先把需要的部分拷进内部 RAM 的小缓冲）。 */
    s_win_pool = (acq_window_t *)heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM);
    if (s_win_pool == NULL) {
        /* 没有 PSRAM（配错 memory_type 或板子是无 PSRAM 版本）时不要直接崩，
         * 退到内部 RAM 让功能先跑起来，同时大声告警。 */
        APP_LOGW("PSRAM 分配窗口池失败（%u 字节），退到内部 RAM —— 请检查板子是否有 PSRAM", (unsigned)bytes);
        s_win_pool = (acq_window_t *)malloc(bytes);
    }
    if (s_win_pool == NULL) {
        APP_LOGE("窗口池分配彻底失败，快车道将无法工作");
        return;
    }
    memset(s_win_pool, 0, bytes);
    s_win_next = 0;
    APP_LOGI("窗口池就绪：%d 槽 × %u 字节", WINDOW_POOL_SLOTS, (unsigned)sizeof(acq_window_t));
}

acq_window_t *window_pool_next(void)
{
    if (s_win_pool == NULL) {
        return NULL;
    }
    /* 轮转取下一块。不做"引用计数/回收"，因为消费者（infer / gui）只读不持有：
     * 3 个槽位意味着同一块被覆盖之前，至少隔了 2 个窗口周期（4s），
     * 而推理 <20ms、GUI 拷贝 <1ms，不可能追不上。
     * 若将来推理变慢到这个量级，就要改成"生产者等消费者释放"的信号量模型。 */
    acq_window_t *w = &s_win_pool[s_win_next];
    s_win_next = (uint8_t)((s_win_next + 1) % WINDOW_POOL_SLOTS);
    return w;
}

/* =============================================================================
 * §4  消息投递
 * ============================================================================= */
bool ui_post(const ui_msg_t *m, TickType_t wait_ticks)
{
    if (g_ui_queue == NULL || m == NULL) {
        return false;
    }
    return xQueueSend(g_ui_queue, m, wait_ticks) == pdTRUE;
}

bool cmd_post(const cmd_msg_t *c, TickType_t wait_ticks)
{
    if (g_cmd_queue == NULL || c == NULL) {
        return false;
    }
    return xQueueSend(g_cmd_queue, c, wait_ticks) == pdTRUE;
}

/* =============================================================================
 * §5  看门狗
 * ============================================================================= */
static bool s_wdt_on = false;

bool app_wdt_init(void)
{
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    esp_task_wdt_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.timeout_ms     = (uint32_t)WDT_TIMEOUT_S * 1000u;
    cfg.idle_core_mask = 0;         /* 不监控 idle 任务（否则高负载时会误触发） */
    cfg.trigger_panic  = true;      /* 超时直接 panic 重启，而不是只打条日志 */
    esp_err_t err = esp_task_wdt_init(&cfg);
    if (err == ESP_ERR_INVALID_STATE) {
        /* Arduino 内核在启动时可能已经初始化过 TWDT，改配置即可 */
        err = esp_task_wdt_reconfigure(&cfg);
    }
#else
    esp_err_t err = esp_task_wdt_init((uint32_t)WDT_TIMEOUT_S, true);
    if (err == ESP_ERR_INVALID_STATE) {
        err = ESP_OK;               /* 已初始化：复用 */
    }
#endif

    if (err != ESP_OK) {
        APP_LOGE("看门狗初始化失败：%d（继续跑，但失去自恢复能力）", (int)err);
        s_wdt_on = false;
        return false;
    }

    /* 把各工作任务注册进 TWDT。
     * 为什么不注册 loopTask：loop() 里只有 vTaskDelay，永远不会卡，
     * 监控它等于没监控。真正要看的 bus/slow/fast/infer/gui/net/cmd。
     * ⚠ 注意：注册过的任务必须周期调用 esp_task_wdt_reset()，否则 5s 后被复位——
     *   这是故意的，"任务卡死 → 自动重启"正是我们要的。 */
    s_wdt_on = true;
    APP_LOGI("看门狗已启动：%d 秒超时，panic 复位", WDT_TIMEOUT_S);
    return true;
}

void app_wdt_register(void)
{
    if (s_wdt_on) {
        /* 登记当前任务。重复登记会返回 ESP_ERR_INVALID_ARG，忽略即可。 */
        const esp_err_t e = esp_task_wdt_add(NULL);
        if (e == ESP_OK) {
            APP_LOGI("[wdt] 任务 %s 已登记", pcTaskGetTaskName(NULL));
        } else if (e != ESP_ERR_INVALID_ARG && e != ESP_ERR_INVALID_STATE) {
            APP_LOGW("[wdt] 任务 %s 登记失败：%d", pcTaskGetTaskName(NULL), (int)e);
        }
    }
}

void app_wdt_feed(void)
{
    if (s_wdt_on) {
        const esp_err_t e = esp_task_wdt_reset();
        if (e != ESP_OK) {
            /* 只报一次，避免刷屏；这条出现就说明"任务以为在喂狗，其实没喂上" */
            static bool warned = false;
            if (!warned) {
                warned = true;
                APP_LOGW("[wdt] 任务 %s 喂狗失败：%d（没登记？被删了？）",
                         pcTaskGetTaskName(NULL), (int)e);
            }
        }
    }
}

/* =============================================================================
 * §6  写车道：消费 cmd_queue → 调用 modbus_master 的写 API
 * ============================================================================= */

/* 读-改-写控制字 1（0x0020）的某几位，然后**一次 FC06** 写回。
 * 为什么要 RMW：控制字是位域，单独写一位会覆盖掉其它位（比如开蜂鸣器把风扇使能清零）。
 * 为什么可以这么做：当前 F411 侧控制字只有主机一个写者，用慢车道刚读回来的快照
 * 做基准就够了。
 * TODO: 如果将来加了本地按键/OLED 改控制字，这里就有竞争了。届时应该让从机
 *       提供一个"置位/清位"命令（可以占用 0x002F 命令寄存器的一个新魔数），
 *       或者让主机改用 FC10 连写"完整控制字 + 期望位"，彻底避免 RMW。 */
static bool ctrl1_rmw_write(uint16_t set_mask, uint16_t clr_mask)
{
    /* ★ 2026-10 修正：不能拿"慢车道快照"当 RMW 的基准值 ★
     * ---------------------------------------------------------------------
     * 原实现是 `slow_u16(&snap, REG_CTRL1)`，而这里有个**必错的越界**：
     *   · 慢车道只读 0x0000~0x001B（MODBUS_SLOW_START + MODBUS_SLOW_REG_COUNT = 28）
     *   · 而 REG_CTRL1 = **0x0020**，**不在这个范围里**
     *   · slow_u16() 对越界地址直接 `return 0`
     * ⇒ cur 永远是 0 ⇒ nv = **只有 mask** ⇒ **整个控制字被覆盖，原有位全丢**。
     *   实测：执行 `beep` 之后，从机 ctrl 从 0x0020 变成 0x0001/0x0008 ——
     *   bit5（允许本地自治）被无声清掉了。
     *
     * 而且**即使范围覆盖到了，用快照做 RMW 在设计上也是错的**：
     * 快照是慢车道每 1s 采一次的缓存，可能已过期近 1 秒；
     * 用它算新值会把这一秒内发生的改动**无声地覆盖掉**。
     *
     * ⇒ 正确做法：**RMW 之前实时读一次从机的真实值**。
     *   这也符合 docs/02 §7 的"读-改-写"约定：从机侧没有原子的位操作功能码，
     *   FC06 是写整字，所以基准值必须是"刚读到的真值"。 */
    uint16_t cur = 0;
    if (mb_read_holding(REG_CTRL1, 1, &cur,
                        MODBUS_TIMEOUT_MS_WRITE, MODBUS_RETRY_WRITE) != MB_OK) {
        APP_LOGW("控制字 RMW 失败：读 0x%04X 无响应（从机离线？）", REG_CTRL1);
        return false;
    }
    const uint16_t nv = (uint16_t)((cur | set_mask) & (uint16_t)~clr_mask);
    APP_LOGI("控制字 RMW：读回 0x%04X，置 0x%04X 清 0x%04X → 写 0x%04X",
             cur, set_mask, clr_mask, nv);
    return mb_write_single(REG_CTRL1, nv,
                           MODBUS_TIMEOUT_MS_WRITE, MODBUS_RETRY_WRITE) == MB_OK;
}

/* 风扇：**一次 FC10 原子写** [控制字|风扇使能, 风扇PWM]。
 * 为什么不用两条 FC06：两条之间如果被别的请求插进来，就会出现
 * "使能开了但 PWM 还是旧的" 或反之的中间态，风扇会抖一下。
 * REG_CTRL1(0x0020) 与 REG_FAN_PWM(0x0021) 地址连续，正好能一帧写完，
 * 这也正是 docs/02 §6 ⑥ 给出的示例。
 *
 * ★ 同样改为"实时读真值"（理由见 ctrl1_rmw_write）：一次 FC03 读
 *   0x0020~0x0021 拿基准，再一次 FC10 原子写回 —— 读写各一次，中间窗口最小。 */
static bool fan_set(uint16_t pwm)
{
    if (pwm > 1000) {
        pwm = 1000;                 /* docs/02 §5-C：风扇 PWM 范围 0~1000 */
    }
    uint16_t cur[2] = { 0, 0 };
    if (mb_read_holding(REG_CTRL1, 2, cur,
                        MODBUS_TIMEOUT_MS_WRITE, MODBUS_RETRY_WRITE) != MB_OK) {
        APP_LOGW("风扇设置失败：读 0x%04X~0x%04X 无响应", REG_CTRL1, REG_FAN_PWM);
        return false;
    }
    const uint16_t vals[2] = {
        (uint16_t)(cur[0] | CTRL1_FAN_EN),   /* 保留其它使能位 */
        pwm
    };
    APP_LOGI("风扇设置：控制字 0x%04X → 0x%04X，PWM = %u",
             cur[0], vals[0], (unsigned)pwm);
    return mb_write_multiple(REG_CTRL1, 2, vals,
                             MODBUS_TIMEOUT_MS_WRITE, MODBUS_RETRY_WRITE) == MB_OK;
}

void task_cmd(void *arg)
{
    (void)arg;
    cmd_msg_t c;
    app_wdt_register();

    for (;;) {
        app_wdt_feed();
        if (xQueueReceive(g_cmd_queue, &c, pdMS_TO_TICKS(500)) != pdTRUE) {
            continue;               /* 超时只是为了让看门狗有机会被喂 */
        }

        /* 丢弃过期命令（见 config.h CMD_MAX_AGE_MS 的说明） */
        if (c.tick != 0 && (millis() - c.tick) > CMD_MAX_AGE_MS) {
            APP_LOGW("丢弃过期命令 kind=%d（已排队 %u ms）",
                     (int)c.kind, (unsigned)(millis() - c.tick));
            continue;
        }

        mb_status_t st = MB_ERR_ARG;
        switch (c.kind) {
            case CMD_WRITE_SINGLE:
                st = mb_write_single(c.start, c.values[0],
                                     MODBUS_TIMEOUT_MS_WRITE, MODBUS_RETRY_WRITE);
                break;

            case CMD_WRITE_MULTIPLE: {
                const uint16_t cnt = (c.count > CMD_MAX_VALUES) ? CMD_MAX_VALUES : c.count;
                /* docs/02 §9："写重试要幂等：只重试整块写，避免半写"。
                 * mb_write_multiple 的重试就是整帧重发，符合这条要求。 */
                st = mb_write_multiple(c.start, cnt, c.values,
                                       MODBUS_TIMEOUT_MS_WRITE, MODBUS_RETRY_WRITE);
                break;
            }

            case CMD_CTRL1_SET_BITS:
                st = ctrl1_rmw_write(c.values[0], 0) ? MB_OK : MB_ERR_TIMEOUT;
                break;

            case CMD_CTRL1_CLR_BITS:
                st = ctrl1_rmw_write(0, c.values[0]) ? MB_OK : MB_ERR_TIMEOUT;
                break;

            case CMD_SAFE_SHUTDOWN:
                st = ctrl1_rmw_write(CTRL1_SAFE_SHUTDOWN, 0) ? MB_OK : MB_ERR_TIMEOUT;
                break;

            case CMD_FORCE_WINDOW:
                st = ctrl1_rmw_write(CTRL1_FORCE_WIN, 0) ? MB_OK : MB_ERR_TIMEOUT;
                break;

            case CMD_FAN_SET:
                st = fan_set(c.values[0]) ? MB_OK : MB_ERR_TIMEOUT;
                break;

            default:
                APP_LOGW("未知命令 kind=%d", (int)c.kind);
                break;
        }

        if (st != MB_OK) {
            APP_LOGW("命令下发失败 kind=%d addr=0x%04X -> %d", (int)c.kind, c.start, (int)st);
        }
        /* TODO: 下发失败要回给 GUI（做按钮回弹 + 红字提示），
         *       现在只打日志。做法：往 g_ui_queue 投一条 UI_MSG_ONLINE 之外的
         *       新消息类型，或者在 cmd_msg_t 里加一个结果回传字段。 */
    }
}

/* =============================================================================
 * §7  初始化 + 任务创建
 * ============================================================================= */

void app_queues_init(void)
{
    if (g_acq_queue == NULL) {
        g_acq_queue = xQueueCreate(QLEN_ACQ, sizeof(acq_item_t));
    }
    if (g_ui_queue == NULL) {
        g_ui_queue = xQueueCreate(QLEN_UI, sizeof(ui_msg_t));
    }
    if (g_cmd_queue == NULL) {
        g_cmd_queue = xQueueCreate(QLEN_CMD, sizeof(cmd_msg_t));
    }
    window_pool_init();

    APP_LOGI("队列就绪：acq=%d ui=%d cmd=%d",
             QLEN_ACQ, QLEN_UI, QLEN_CMD);
}

bool app_tasks_start(void)
{
    BaseType_t r = pdPASS;

    /* 总线持有者：core 0，最高优先级（见 modbus_master.h 顶部的硬约束） */
    r &= xTaskCreatePinnedToCore(task_modbus_bus,  "mb_bus",  TASK_STACK_BUS,
                                 NULL, TASK_PRIO_BUS,   NULL, CORE_BUS);
    /* 慢车道 / 快车道：生产者，和总线同核，靠队列 + 通知串行化 */
    r &= xTaskCreatePinnedToCore(task_modbus_slow, "mb_slow", TASK_STACK_SLOW,
                                 NULL, TASK_PRIO_SLOW,  NULL, CORE_BUS);
    r &= xTaskCreatePinnedToCore(task_modbus_fast, "mb_fast", TASK_STACK_FAST,
                                 NULL, TASK_PRIO_FAST,  NULL, CORE_BUS);
    /* 写车道 */
    r &= xTaskCreatePinnedToCore(task_cmd,         "cmd",     TASK_STACK_CMD,
                                 NULL, TASK_PRIO_CMD,   NULL, CORE_BUS);
    /* 推理：core 0，优先级低于总线（总线有硬实时要求，推理有 2s 的余量） */
    r &= xTaskCreatePinnedToCore(task_infer,       "infer",   TASK_STACK_INFER,
                                 NULL, TASK_PRIO_INFER, NULL, CORE_INFER);
    /* GUI：core 1 独占，避免被 WiFi 的射频任务打断造成刷新抖动 */
    r &= xTaskCreatePinnedToCore(task_gui,         "gui",     TASK_STACK_GUI,
                                 NULL, TASK_PRIO_GUI,   NULL, CORE_GUI);
    /* 网络：core 0，优先级最低（WiFi 内部任务优先级远高于我们，别去抢） */
    r &= xTaskCreatePinnedToCore(task_net,         "net",     TASK_STACK_NET,
                                 NULL, TASK_PRIO_NET,   NULL, CORE_NET);
    /* 本地 OLED（路径 C）：core 0 + 最低优先级。
     * 为什么不放 core1 和 GUI 同核：刷一帧 1024 字节 @400kHz ≈ 25ms，
     * 放在 core1 会把 LVGL 的刷新节拍拖出 25ms 的坑；放 core0 最低优先级，
     * 它只会在总线/推理都空闲时才跑，天然不抢实时性。 */
#if OLED_ENABLE
    if (!oled_task_start()) {
        r &= pdFAIL;
    }
#endif

    /* 调试串口命令行：core 0 + 最低优先级。
     * 为什么需要：本项目**没有触摸屏也没有按键**，三页界面原本切不了。
     * 命令行提供 page/next/prev/auto/status 等命令，是硬件补齐前唯一的
     * 直接操作手段。它对实时性零要求，放最低优先级、有输入才干活。 */
#if CONSOLE_ENABLE
    if (!console_task_start()) {
        r &= pdFAIL;
    }
#endif

    if (r != pdPASS) {
        APP_LOGE("有任务创建失败（内部 RAM 不足？把 LVGL 缓冲/TFLM arena 挪到 PSRAM）");
        return false;
    }
    APP_LOGI("任务已启动：mb_bus/mb_slow/mb_fast/cmd/infer/net@core0, gui@core1, "
             "oled@core0, console@core0");
    return true;
}
