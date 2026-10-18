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
 *  ── 下行控制（★ 2026-10 新增，用于验证"写方向"链路）──────────────
 *    beep [0|1]                开/关蜂鸣器（不带参数 = 响 1 秒后自动关）
 *    rgb R G B | rgb off       设 RGB（0~255，0x 开头按十六进制解析）
 *    fan 0~1000                风扇 PWM（带使能位一起原子写）
 *    w 寄存器 值                通用写一个寄存器（地址十进制或 0x 十六进制）
 *    cset 掩码 | cclr 掩码      控制字置位 / 清位（如 cset 0x20 = 允许本地自治）
 *
 *  为什么加这些：项目原先只能"读"（采集→显示），**写方向从未实际验证过**
 *  ——诊断页那几个按钮需要触摸屏，而触摸没接。这些命令走的是**和 MQTT 下行
 *  完全相同的路径**（cmd_post → g_cmd_queue → task_cmd → Modbus FC06/FC10），
 *  所以验证它们等于验证了整条下行链路，且不需要 broker、不需要额外硬件。
 * ========================================================================== */

#include <Arduino.h>

#include "config.h"
#include "app_tasks.h"
#include "gui.h"
#include "modbus_master.h"

#if CONSOLE_ENABLE

/* 一行命令的最大长度。超长直接丢弃并提示 —— 不要让它撑爆栈。 */
#define CONSOLE_LINE_MAX   64
/* 参数个数上限（beep / rgb / w 最多用 3 个） */
#define CONSOLE_ARG_MAX    4

/* ---- 参数解析小工具 ---------------------------------------------------- */

/** 解析一个整数：`0x`/`0X` 开头按十六进制，否则十进制。失败返回 false。 */
static bool parse_u16(const char *s, uint16_t *out)
{
    if (s == NULL || *s == '\0') {
        return false;
    }
    char *endp = NULL;
    const long v = strtol(s, &endp, 0);      /* base=0 → 自动识别 0x / 0 */
    if (endp == s) {
        return false;                        /* 一个数字都没解析出来 */
    }
    if (v < 0 || v > 0xFFFFL) {
        return false;
    }
    *out = (uint16_t)v;
    return true;
}

/** 把 `a,b,c` 或 `a b c` 拆成参数数组。返回实际个数。 */
static int split_args(char *s, char *argv[], int max_n)
{
    int n = 0;
    while (s != NULL && *s != '\0' && n < max_n) {
        while (*s == ' ' || *s == ',' || *s == '\t') {
            s++;
        }
        if (*s == '\0') {
            break;
        }
        argv[n++] = s;
        while (*s != '\0' && *s != ' ' && *s != ',' && *s != '\t') {
            s++;
        }
        if (*s != '\0') {
            *s++ = '\0';
        }
    }
    return n;
}

/** 投递一条命令到 g_cmd_queue。返回 false = 队列满。 */
static bool post(const cmd_msg_t *c)
{
    return cmd_post(c, pdMS_TO_TICKS(200));
}

/** 通用：写一个寄存器（FC06） */
static void do_write_reg(uint16_t reg, uint16_t val)
{
    cmd_msg_t c;
    memset(&c, 0, sizeof(c));
    c.kind      = CMD_WRITE_SINGLE;
    c.start     = reg;
    c.count     = 1;
    c.values[0] = val;
    c.tick      = xTaskGetTickCount();
    if (post(&c)) {
        Serial.printf("  → FC06 写 0x%04X = %u (0x%04X)\n", reg, val, val);
    } else {
        Serial.println(F("  ✗ 命令队列满，投递失败"));
    }
}

/** 控制字置位/清位（RMW，从机侧原子完成） */
static void do_ctrl_bits(uint16_t mask, bool set)
{
    cmd_msg_t c;
    memset(&c, 0, sizeof(c));
    c.kind      = set ? CMD_CTRL1_SET_BITS : CMD_CTRL1_CLR_BITS;
    c.start     = REG_CTRL1;
    c.count     = 1;
    c.values[0] = mask;
    c.tick      = xTaskGetTickCount();
    if (post(&c)) {
        Serial.printf("  → 控制字 0x%04X %s（掩码 0x%04X）\n",
                      REG_CTRL1, set ? "置位" : "清位", mask);
    } else {
        Serial.println(F("  ✗ 命令队列满，投递失败"));
    }
}

/** RGB：一次 FC10 原子写 R/G/B 三个寄存器，并同时置位 RGB 使能 */
static void do_rgb(uint16_t r, uint16_t g, uint16_t b)
{
    cmd_msg_t c;
    memset(&c, 0, sizeof(c));
    c.kind      = CMD_WRITE_MULTIPLE;
    c.start     = REG_RGB_R;
    c.count     = 3;
    c.values[0] = r;
    c.values[1] = g;
    c.values[2] = b;
    c.tick      = xTaskGetTickCount();
    if (!post(&c)) {
        Serial.println(F("  ✗ 命令队列满，投递失败"));
        return;
    }
    /* 使能位必须单独置位（它和控制字同寄存器，不能和 RGB 一起原子写） */
    do_ctrl_bits(CTRL1_RGB_EN, true);
    Serial.printf("  → RGB = (%u, %u, %u)，已置位 RGB 使能\n", r, g, b);
}

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
    Serial.println(F("  ── 下行控制（验证写方向）──"));
    Serial.println(F("  beep [0|1]    蜂鸣器（无参数 = 响 1 秒）"));
    Serial.println(F("  rgb R G B     设 RGB 颜色，0~255（rgb off 关闭）"));
    Serial.println(F("  fan 0~1000    风扇 PWM"));
    Serial.println(F("  w 寄存器 值    通用写寄存器（支持 0x 十六进制）"));
    Serial.println(F("  cset/cclr 掩码 控制字置位/清位（如 cset 0x28）"));
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
    if (st.exception_count > 0 || st.crc_err_count > 0) {
        Serial.printf("  ⚠ 最近异常码 = 0x%02X（0x01 功能码非法 / 0x02 地址非法 / 0x03 数值非法）\n",
                      (unsigned)st.last_exception_code);
    }
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

    /* ───────────────── 下行控制（验证写方向）───────────────────── */

    if (strcmp(line, "beep") == 0 || strcmp(line, "buzzer") == 0) {
        if (arg == NULL) {
            /* 不带参数 = 响 1 秒后自动关（省得忘了关一直响） */
            do_ctrl_bits(CTRL1_BEEP, true);
            Serial.println(F("  → 蜂鸣器响 1 秒…"));
            vTaskDelay(pdMS_TO_TICKS(1000));
            do_ctrl_bits(CTRL1_BEEP, false);
            Serial.println(F("  → 已关闭"));
            return;
        }
        uint16_t v;
        if (!parse_u16(arg, &v)) {
            Serial.println(F("  ✗ 参数只能是 0 或 1"));
            return;
        }
        do_ctrl_bits(CTRL1_BEEP, v != 0);
        return;
    }

    if (strcmp(line, "rgb") == 0) {
        if (arg == NULL) {
            Serial.println(F("  用法：rgb R G B（0~255）或 rgb off"));
            return;
        }
        char *argv[CONSOLE_ARG_MAX];
        const int n = split_args(arg, argv, CONSOLE_ARG_MAX);

        if (strcmp(argv[0], "off") == 0) {
            /* 关灯：先清使能位（从机侧会把三路 PWM 都置 0） */
            do_ctrl_bits(CTRL1_RGB_EN, false);
            Serial.println(F("  → RGB 已关闭"));
            return;
        }
        if (n < 3) {
            Serial.println(F("  ✗ 需要 3 个参数：rgb R G B（0~255）"));
            return;
        }
        uint16_t r, g, b;
        if (!parse_u16(argv[0], &r) || !parse_u16(argv[1], &g) || !parse_u16(argv[2], &b)) {
            Serial.println(F("  ✗ 颜色值解析失败（只能是 0~255 或 0x00~0xFF）"));
            return;
        }
        if (r > 255 || g > 255 || b > 255) {
            Serial.println(F("  ✗ 颜色值超范围（0~255）"));
            return;
        }
        do_rgb(r, g, b);
        return;
    }

    if (strcmp(line, "fan") == 0) {
        uint16_t v;
        if (arg == NULL || !parse_u16(arg, &v)) {
            Serial.println(F("  用法：fan 0~1000（0 = 停）"));
            return;
        }
        if (v > 1000) {
            Serial.println(F("  ✗ 风扇 PWM 范围 0~1000"));
            return;
        }
        /* 一次 FC10 原子写 [控制字|风扇使能, 风扇PWM]
         * —— 就是 docs/02 §6⑥ 的示例帧 `01 10 00 20 00 02 04 …` 的用法。
         * ⚠ 这里直接写控制字整字（不是 RMW），所以必须**先读出当前值再或上使能位**，
         *   否则会把别的使能位一起清掉。用 CMD_FAN_SET 让 task_cmd 处理这个细节。 */
        cmd_msg_t c;
        memset(&c, 0, sizeof(c));
        c.kind      = CMD_FAN_SET;
        c.start     = REG_FAN_PWM;
        c.count     = 1;
        c.values[0] = v;
        c.tick      = xTaskGetTickCount();
        if (post(&c)) {
            Serial.printf("  → 风扇 PWM = %u（%s）\n", v, (v == 0) ? "停" : "转");
        } else {
            Serial.println(F("  ✗ 命令队列满，投递失败"));
        }
        return;
    }

    if (strcmp(line, "w") == 0 || strcmp(line, "write") == 0) {
        char *argv[CONSOLE_ARG_MAX];
        const int n = (arg == NULL) ? 0 : split_args(arg, argv, CONSOLE_ARG_MAX);
        uint16_t reg, val;
        if (n < 2 || !parse_u16(argv[0], &reg) || !parse_u16(argv[1], &val)) {
            Serial.println(F("  用法：w 寄存器 值   （如 w 0x0027 450）"));
            return;
        }
        do_write_reg(reg, val);
        return;
    }

    if (strcmp(line, "cset") == 0 || strcmp(line, "cclr") == 0) {
        uint16_t mask;
        if (arg == NULL || !parse_u16(arg, &mask)) {
            Serial.println(F("  用法：cset 掩码 / cclr 掩码（如 cset 0x8 开 RGB 使能）"));
            return;
        }
        do_ctrl_bits(mask, strcmp(line, "cset") == 0);
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
