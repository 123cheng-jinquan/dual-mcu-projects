/* =============================================================================
 *  console.cpp — 调试串口命令行（core 0，低优先级）
 * =============================================================================
 *  为什么需要它：
 *    本项目**没有触摸屏、也没有实体按键**，三页界面（数据/波形/诊断）
 *    原本切不了 —— 只能一直盯着第一页。触摸那 5 根线当初特意没接（省引脚、
 *    免校准），按键也还没焊，所以在硬件补齐之前，命令行是唯一的直接操作手段。
 *
 *  设计要点：
 *   1) **独立任务、core 0、最低优先级** —— 不占 core1（GUI 在那），
 *      也不和总线任务抢。串口读不到字节时 `Serial.read()` 立即返回 -1，
 *      配上一小段 delay 让出 CPU，几乎不耗资源。
 *   2) **绝不直接调用 LVGL** —— 所有动作都走 gui.h 的 gui_request_page() 等
 *      接口（它们只写原子变量，真正的 LVGL 调用在 core1 的 task_gui 里）。
 *   3) 用**行缓冲 + 非阻塞读**，不依赖 `Serial.readStringUntil()`：
 *      后者在没收到换行时会一直等，把任务卡住（也就没法喂看门狗）。
 *
 *  命令一览（大小写不敏感，`=` 和空格都能分隔）：
 *    page=0|1|2 | page 0       切到指定页（0=数据 1=波形 2=诊断）
 *    next | n                  下一页（循环）
 *    prev | p                  上一页（循环）
 *    auto=0|1 | auto on|off    开关自动轮播（默认 10s 一页，见 config.h）
 *    status                    打印当前页 / 轮播状态 / 总线心跳 / 内存
 *    help | ?                  打印命令帮助
 *
 *  用法：串口工具（115200 8N1，COM6）里直接敲 `next` 回车。
 *  ⚠ PlatformIO 的 `pio device monitor` 也能敲，但要注意它默认会**复位设备**；
 *    用 `--no-reset` 或换 putty/串口助手更顺手。
 * ========================================================================== */

#include <Arduino.h>

#include "config.h"
#include "app_tasks.h"
#include "gui.h"
#include "modbus_master.h"

#if CONSOLE_ENABLE

/* 一行命令的最大长度。超长直接丢弃并提示 —— 不要让它撑爆栈。 */
#define CONSOLE_LINE_MAX   64

static void print_help(void)
{
    Serial.println();
    Serial.println(F("──────── 串口命令（无触摸无按键时的操作手段）────────"));
    Serial.println(F("  page=0|1|2    切到指定页（0=数据 1=波形 2=诊断）"));
    Serial.println(F("  next / n      下一页（循环）"));
    Serial.println(F("  prev / p      上一页（循环）"));
    Serial.println(F("  auto=0|1      关/开自动轮播"));
    Serial.println(F("  status        当前页 / 轮播 / 总线 / 内存"));
    Serial.println(F("  help          显示这份帮助"));
    Serial.println(F("────────────────────────────────────────────────────"));
    Serial.println();
}

static void print_status(void)
{
    const int  page = gui_get_page();
    const bool auto_cyc = gui_get_auto_cycle();

    Serial.println();
    Serial.printf("  当前页   : %d（%s）%s\n", page,
                  (page == 0) ? "数据" : (page == 1) ? "波形" : "诊断",
                  auto_cyc ? "  [自动轮播中]" : "  [手动模式]");
    Serial.printf("  轮播周期 : %u ms（config.h 的 GUI_PAGE_CYCLE_MS）\n",
                  (unsigned)GUI_PAGE_CYCLE_MS);
    /* 诊断自动轮播为什么不动：把状态机内部值打出来 */
    Serial.printf("  [诊断] 距上次切页 %u ms，请求页 %d\n",
                  (unsigned)gui_debug_since_page_ms(), gui_debug_want());

    /* 总线统计是"只读快照"，不碰总线本身，随便调 */
    mb_stats_t st;
    mb_get_stats(&st);
    Serial.printf("  485 总线 : TX %u  RX %u  超时 %u  重试 %u  异常 %u  CRC错 %u  %s\n",
                  (unsigned)st.tx_count, (unsigned)st.rx_count,
                  (unsigned)st.timeout_count, (unsigned)st.retry_count,
                  (unsigned)st.exception_count, (unsigned)st.crc_err_count,
                  st.online ? "在线" : "离线");
    Serial.printf("  队列     : cmd 等待 %u  ui 等待 %u\n",
                  (unsigned)(g_cmd_queue ? uxQueueMessagesWaiting(g_cmd_queue) : 0),
                  (unsigned)(g_ui_queue  ? uxQueueMessagesWaiting(g_ui_queue)  : 0));
    Serial.printf("  内存     : 内部堆剩余 %u 字节，最小曾到 %u 字节\n",
                  (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMinFreeHeap());
    Serial.println();
}

/* 解析并执行一行命令。line 已在调用处去掉换行、并以 '\0' 结尾。 */
static void exec_line(char *line)
{
    /* 统一：去掉首尾空白、转小写、把 '=' 换成空格，这样
     * `PAGE = 2` / `page=2` / `page 2` 都能认。 */
    while (*line == ' ' || *line == '\t') {
        line++;
    }
    char *end = line + strlen(line);
    while (end > line && (end[-1] == ' ' || end[-1] == '\t' || end[-1] == '\r')) {
        *--end = '\0';
    }
    if (*line == '\0') {
        return;
    }
    for (char *q = line; *q; q++) {
        if (*q >= 'A' && *q <= 'Z') {
            *q = (char)(*q - 'A' + 'a');
        } else if (*q == '=') {
            *q = ' ';
        }
    }

    /* 拆成"命令 + 参数" */
    char *arg = strchr(line, ' ');
    if (arg != NULL) {
        *arg++ = '\0';
        while (*arg == ' ') {
            arg++;
        }
    }

    if (strcmp(line, "help") == 0 || strcmp(line, "?") == 0) {
        print_help();
        return;
    }
    if (strcmp(line, "status") == 0 || strcmp(line, "st") == 0) {
        print_status();
        return;
    }
    if (strcmp(line, "next") == 0 || strcmp(line, "n") == 0) {
        gui_request_page_step(+1);
        Serial.println(F("  → 下一页"));
        return;
    }
    if (strcmp(line, "prev") == 0 || strcmp(line, "p") == 0) {
        gui_request_page_step(-1);
        Serial.println(F("  → 上一页"));
        return;
    }
    if (strcmp(line, "page") == 0) {
        if (arg == NULL) {
            Serial.printf("  当前页 = %d\n", gui_get_page());
            return;
        }
        const int p = atoi(arg);
        if (p < 0 || p >= GUI_PAGE_COUNT) {
            Serial.printf("  ✗ 页码超范围（只能是 0~%d）\n", GUI_PAGE_COUNT - 1);
            return;
        }
        gui_request_page(p);
        Serial.printf("  → 切到第 %d 页\n", p);
        return;
    }
    if (strcmp(line, "auto") == 0) {
        if (arg == NULL) {
            Serial.printf("  自动轮播 = %s\n", gui_get_auto_cycle() ? "开" : "关");
            return;
        }
        const bool on = (strcmp(arg, "1") == 0 || strcmp(arg, "on") == 0 ||
                         strcmp(arg, "true") == 0 || strcmp(arg, "yes") == 0);
        const bool off = (strcmp(arg, "0") == 0 || strcmp(arg, "off") == 0 ||
                          strcmp(arg, "false") == 0 || strcmp(arg, "no") == 0);
        if (!on && !off) {
            Serial.println(F("  ✗ 参数只能是 0/1（或 on/off）"));
            return;
        }
        gui_set_auto_cycle(on);
        Serial.printf("  → 自动轮播已%s\n", on ? "打开" : "关闭");
        return;
    }

    Serial.printf("  ✗ 不认识的命令：`%s`（敲 help 看用法）\n", line);
}

static void task_console(void *arg)
{
    (void)arg;
    APP_LOGI("task_console 已启动（core0，低优先级）：串口敲 help 看命令");

    char   buf[CONSOLE_LINE_MAX];
    size_t len = 0;

    for (;;) {
        /* 非阻塞读：把当前缓冲区里能拿到的字节全拿走 */
        bool got = false;
        while (Serial.available() > 0) {
            const int ch = Serial.read();
            if (ch < 0) {
                break;
            }
            got = true;

            if (ch == '\r' || ch == '\n') {
                if (len > 0) {
                    buf[len] = '\0';
                    exec_line(buf);
                    len = 0;
                }
                continue;
            }
            /* 只收可打印字符；退格把它从缓冲里删掉 */
            if (ch == 8 || ch == 127) {
                if (len > 0) {
                    len--;
                }
                continue;
            }
            if (ch < 32 || ch > 126) {
                continue;
            }
            if (len < CONSOLE_LINE_MAX - 1) {
                buf[len++] = (char)ch;
            } else {
                /* 超长：丢弃整行，避免半个命令被执行 */
                len = 0;
                Serial.println(F("  ✗ 命令过长，已丢弃（上限 63 字符）"));
            }
        }

        /* 没有输入时让出 CPU：50ms 的节拍对人手输入完全够用，
         * 又能把这条任务的占用压到可以忽略。 */
        if (!got) {
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
}

bool console_task_start(void)
{
    const BaseType_t r = xTaskCreatePinnedToCore(
        task_console, "console", TASK_STACK_CONSOLE, NULL,
        TASK_PRIO_CONSOLE, NULL, 0);          /* core 0 */
    return r == pdPASS;
}

#endif  /* CONSOLE_ENABLE */
