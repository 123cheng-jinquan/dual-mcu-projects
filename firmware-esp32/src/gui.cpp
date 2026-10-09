/* =============================================================================
 *  gui.cpp — LVGL 触摸 HMI 骨架（数据页 / 波形页 / 诊断页）+ OLED 本地显示
 * =============================================================================
 *  ⚠ 屏幕驱动 IC / 分辨率 / 是否电容触摸**尚未确定**（config.h [TBD-1]），
 *    所以 gui_init() 里显示与触摸的移植部分是三条路径：
 *      路径 A（GUI_USE_ESP_LVGL_PORT = 1）：一体板，走官方 esp_lvgl_port
 *      路径 B（GUI_USE_ESP_LVGL_PORT = 0）：分体屏，TFT_eSPI / LovyanGFX 自绘
 *      路径 C（OLED_ENABLE = 1）：SSD1306 128×64 I2C —— **不依赖 LVGL**，
 *                                 走 oled.cpp，与 A/B 互不冲突、可同时存在
 *    路径 C 是当前**唯一可用**的显示通道（A/B 都要等屏型号定下来）：
 *    它把「分类结果 / 振动 std / 485 在线状态」显示出来，串口之外还能用眼睛看。
 *    其余部分（三个界面的控件树、更新入口、命令下发）都是完整的：
 *    移植路径一补上，界面立刻能用。
 *
 *  所有字符串都用 snprintf + lv_label_set_text 而不是 lv_label_set_text_fmt：
 *    LVGL 内置的 lv_snprintf 默认不开浮点（LV_SPRINTF_USE_FLOAT=0），
 *    直接用 "%.1f" 会打出乱码；用 ESP32 newlib 的 snprintf 就没这个问题。
 * =============================================================================
 */

#include "gui.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

#include <esp_heap_caps.h>

#include "oled.h"

/* =============================================================================
 * §1  模块状态
 * ============================================================================= */
static bool       s_ui_ready    = false;
static lv_obj_t  *s_tabview     = NULL;

/* =============================================================================
 * 页面控制状态（2026-10）
 * -----------------------------------------------------------------------------
 * ★ 全部用**原子变量**，因为写它们的是 core0 的 console/net 任务，
 *   读它们的是 core1 的 task_gui。int 在这个平台上读写本身是原子的，
 *   但要显式用 __atomic_* 表达意图，也防止编译器把它优化到寄存器里
 *   （`volatile` 也能防优化，但 __atomic_* 语义更明确）。
 *
 * 分工：
 *   s_page_want — 请求的目标页（任意任务可写）
 *   s_page_cur  — 当前实际页（只有 task_gui 写，任意任务可读）
 *   s_auto_cycle— 自动轮播开关
 *   s_page_last_ms — 上次切页时刻（只有 task_gui 读写）
 * ========================================================================== */
static volatile int      s_page_want      = 0;
static volatile int      s_page_cur       = 0;
static volatile bool     s_auto_cycle     = true;   /* 默认开轮播：没人操作也能看到三页 */
static uint32_t          s_page_last_ms   = 0;

static const char *page_name(int p)
{
    switch (p) {
        case 0:  return "数据";
        case 1:  return "波形";
        case 2:  return "诊断";
        default: return "?";
    }
}
static uint32_t   s_diag_last_ms = 0;
static uint32_t   s_wave_last_ms = 0;

/* 最近一次算出的振动指标（三轴加速度 std / 峰峰值最大值）。
 * 显示走 oled_set_vibration()（见 §5 的 vib_stats_accel）；这两个只在诊断页显示，
 * 让人能对照着 OLED 一起读。 */
static uint16_t   s_vib_std_last = 0;
static uint16_t   s_vib_pp_last  = 0;

/* 数据页：10 张卡片。每张卡片绑定一个 A 区寄存器 + 一个标度。
 * 为什么做成表驱动而不是手写 10 遍控件创建：加一个传感器只改这张表一行。 */
#define DASH_CARD_COUNT 10

typedef struct {
    const char *title;      /* 卡片标题 */
    const char *unit;       /* 单位（拼在数值后面） */
    uint16_t    addr;       /* 寄存器地址（docs/02 §5-A） */
    bool        is_signed;  /* 温度是 int16，其余是 uint16 */
    float       div;        /* 显示值 = 原值 / div（×10 的标度就是 10.0） */
    uint8_t     decimals;   /* 显示小数位 */
    lv_obj_t   *val_lbl;    /* 数值标签句柄（运行时填） */
} dash_card_t;

static dash_card_t s_cards[DASH_CARD_COUNT] = {
    { "温度",     "C",   REG_TEMP,  true,  10.0f, 1, NULL },
    { "湿度",     "%RH", REG_HUMI,  false, 10.0f, 1, NULL },
    { "气压",     "hPa", REG_PRESS, false, 10.0f, 1, NULL },
    { "光照",     "",    REG_LIGHT, false,  1.0f, 0, NULL },
    { "烟雾",     "",    REG_SMOKE, false,  1.0f, 0, NULL },
    { "空气质量", "",    REG_AIR,   false,  1.0f, 0, NULL },
    { "距离",     "mm",  REG_DIST,  false,  1.0f, 0, NULL },
    { "人体感应", "",    REG_PIR,   false,  1.0f, 0, NULL },
    { "心率",     "bpm", REG_HR,    false,  1.0f, 0, NULL },
    { "血氧",     "%",   REG_SPO2,  false,  1.0f, 0, NULL },
};

/* 波形页 */
static lv_obj_t          *s_chart     = NULL;
static lv_chart_series_t *s_ser_ax    = NULL;
static lv_chart_series_t *s_ser_ay    = NULL;
static lv_chart_series_t *s_ser_az    = NULL;
static lv_obj_t          *s_wave_info = NULL;   /* 窗口序号 / 采样率 / 时间戳 */
static lv_obj_t          *s_infer_lbl = NULL;   /* 推理标签 + 置信度 */
static lv_obj_t          *s_conf_bar  = NULL;   /* 最高类置信度进度条 */

/* 诊断页 */
static lv_obj_t *s_diag_lbl   = NULL;
static lv_obj_t *s_sw_beep    = NULL;
static lv_obj_t *s_sl_fan     = NULL;
static lv_obj_t *s_lbl_fan    = NULL;
static lv_obj_t *s_sl_servo   = NULL;
static lv_obj_t *s_lbl_servo  = NULL;
static lv_obj_t *s_sl_rgb[3]  = { NULL, NULL, NULL };
static lv_obj_t *s_lbl_rgb[3] = { NULL, NULL, NULL };

/* =============================================================================
 * §2  小工具
 * ============================================================================= */

/* 数字格式化：统一走 newlib snprintf（见文件头注释） */
static void set_value_text(lv_obj_t *lbl, float v, int decimals, const char *unit)
{
    if (lbl == NULL) {
        return;
    }
    char buf[24];
    snprintf(buf, sizeof(buf), "%.*f%s", decimals, (double)v, unit);
    lv_label_set_text(lbl, buf);
}

/* 命令下发：GUI **只投队列**，绝不直接调 Modbus API（见 gui.h 的设计口径） */
static void post_cmd(cmd_kind_t kind, uint16_t start, uint16_t count, const uint16_t *vals)
{
    cmd_msg_t c;
    memset(&c, 0, sizeof(c));
    c.kind  = kind;
    c.start = start;
    c.count = count;
    c.tick  = millis();
    if (vals != NULL) {
        for (uint16_t i = 0; i < count && i < CMD_MAX_VALUES; i++) {
            c.values[i] = vals[i];
        }
    }
    if (!cmd_post(&c, 0)) {
        APP_LOGW("cmd_queue 满，本次按钮操作被丢弃（总线被窗口读占住了？）");
    }
}

/* 造一个"标题 + 控件"的小行，返回行容器（诊断页的控制区用它排版） */
static lv_obj_t *make_row(lv_obj_t *parent, const char *title)
{
    lv_obj_t *row = lv_obj_create(parent);
    lv_obj_set_size(row, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(row, 0, 0);
    lv_obj_set_style_pad_all(row, 2, 0);
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(row, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    lv_obj_t *lbl = lv_label_create(row);
    lv_label_set_text(lbl, title);
    lv_obj_set_width(lbl, 90);
    return row;
}

/* =============================================================================
 * §3  显示 / 触摸移植路径
 * -----------------------------------------------------------------------------
 *  三条路径的关系（**不是三选一**，C 与 A/B 可以同时存在）：
 *
 *   路径 A/B = LVGL 屏（主界面，触摸交互）—— 等 config.h [TBD-1] 定屏型
 *   路径 C   = SSD1306 OLED（本地观察窗，无触摸）—— 已实现，见 oled.cpp
 *
 *  为什么把 C 单独拿出来：它**不依赖 LVGL**，所以 A/B 一天没落地，
 *  "分类结果 / 振动 std / 485 在线状态"也一天不会看不到。
 *  代价是它只有 21×8 个 ASCII 字符格（中文字库不在这块屏的预算里）。
 * ============================================================================= */

/* ---- 路径 C：SSD1306 I2C OLED（见 oled.cpp / oled.h）-----------------------
 * 与 LVGL 完全解耦：自己的任务（task_oled，core0）、自己的 I2C 总线、
 * 只读共享数据不发总线请求。所以这里的返回 false **只表示 LVGL 屏不可用**，
 * 不代表"没有显示"—— OLED 可能好好的。
 * 返回值恒为 false（A/B 都还没实现），GUI 控件树因此不建，但：
 *   · gui_tick() 照常排空 g_ui_queue（否则队列满会让总线那边丢消息）；
 *   · gui_update_wave() 照常算振动 std 并喂给 OLED；
 *   · task_oled 照常刷屏。
 * -------------------------------------------------------------------------- */
#if OLED_ENABLE
static void gui_oled_note(void)
{
    APP_LOGI("显示路径 C（SSD1306 OLED）已启用：由 task_oled 独立驱动，"
             "与 LVGL 解耦、不占用总线");
}
#else
static void gui_oled_note(void) {}
#endif

#if GUI_USE_ESP_LVGL_PORT
/* ---- 路径 A：一体板（板载屏 + 官方 BSP）→ esp_lvgl_port -------------------
 * TODO([TBD-1] 拿到板子型号后补全)：
 *   1) const esp_lvgl_port_cfg_t port_cfg = ESP_LVGL_PORT_INIT_CONFIG();
 *      port_cfg.task_priority = TASK_PRIO_GUI;      // 和 config.h 对齐
 *      port_cfg.task_stack    = TASK_STACK_GUI;
 *      ESP_ERROR_CHECK(esp_lvgl_port_init(&port_cfg));
 *   2) 用 esp_lcd 建 panel handle（SPI 屏：esp_lcd_new_panel_io_spi +
 *      esp_lcd_new_panel_st7789/ili9341/...；RGB 屏走另一套），
 *      填 lvgl_port_display_cfg_t（分辨率、色深 16、buffer_size = 1/10 屏、
 *      flags.buff_dma / buff_spiram 按 config.h 的 LVGL_DRAW_BUF_IN_PSRAM 定），
 *      然后 lvgl_port_add_disp(&disp_cfg)。
 *   3) 触摸：esp_lcd_touch 建 handle → lvgl_port_add_touch(&touch_cfg)。
 *      ⚠ 电容触摸要额外确认 I2C 地址与中断脚（GT911/FT6236 各不相同）。
 *   4) **重要**：esp_lvgl_port 自己会起一个 LVGL 任务并持有互斥锁，
 *      所以接进来之后本文件的 gui_tick() 必须改成：
 *          if (lvgl_port_lock(0)) { lv_timer_handler(); ...改控件...; lvgl_port_unlock(); }
 *      否则会和 BSP 的任务同时碰 LVGL，表现为随机 HardFault。
 *      同时 task_gui 的职责要缩小为"排空 g_ui_queue 并更新控件"，
 *      不再自己调 lv_timer_handler（BSP 的任务在做）。
 */
static bool gui_port_init(void)
{
    gui_oled_note();
    APP_LOGW("显示移植路径 A（esp_lvgl_port）尚未实现：屏幕型号未确定，见 config.h [TBD-1]");
    return false;
}
#else
/* ---- 路径 B：自购 SPI 屏（LovyanGFX）★ 已实现 ★ ---------------------------
 * 屏幕：2.4" 320×240 SPI 模块，ILI9341 驱动（红色 PCB，背面丝印
 *       `2.4" TFT SPI 240X320 V1.3`），带 XPT2046 电阻触摸 + microSD
 *       （触摸那 5 根线本项目暂时不接，省引脚也不用校准）。
 * 引脚与接线表见 include/config.h §4.2；面板参数写在下面的 `class Lcd` 里。
 *
 * ★ 为什么用 LovyanGFX 而不是 TFT_eSPI（2026-10 实测换掉的）★
 *   TFT_eSPI 2.5.43 在本工程的核心版本（Arduino-ESP32 **2.0.17**）下，
 *   `init()` **必崩**：
 *       Guru Meditation Error: Core 1 panic'ed (StoreProhibited)
 *       EXCVADDR: 0x00000010
 *       Backtrace: writecommand → begin_tft_write → TFT_eSPI::init
 *   崩点是 `SET_BUS_WRITE_MODE`，展开成 `*_spi_user = SPI_USR_MOSI` ——
 *   该库为了速度**直接操作 SPI 寄存器**，而算出来的 `_spi_user` 地址是 0x10。
 *   已逐个排除：TFT_MISO 未定义、SPI.begin() 未调用、SPI_PORT 宏未解析、
 *   **以及"屏没接"（屏接上照崩）** → 结论是库与核心版本不兼容。
 *   LovyanGFX 自带 ESP32-S3 的 SPI 驱动（走 esp-idf 的 spi_master），
 *   **既不依赖 Arduino 的 SPIClass，也不碰寄存器宏**，从根上绕开这类问题；
 *   而且面板参数写在代码里，换屏只改一个类。
 *
 * 实现要点（每条对应一个经典坑）：
 *   1) flush_cb 里 pushImage 之后**必须**调 lv_disp_flush_ready()，
 *      否则 LVGL 等第一块等到天荒地老（"只画了最上面一条就不动了"）。
 *   2) 画缓冲放**内部 DMA 内存**（不是 PSRAM）：320×40×2B = 25.6KB，
 *      SPI DMA 从内部 RAM 搬更快，也不和 WiFi 抢 cache。
 *   3) 单缓冲先跑起来（双缓冲要 51KB，收益有限）。
 *   4) 背光用 LEDC（以后加"亮度"滑块就靠它）。
 *   5) 反色/红蓝互换/偏移：都改 `class Lcd` 里 `panel()` 的参数，
 *      **不用动 LVGL 的代码**。
 */
#define LGFX_USE_V1
#include <LovyanGFX.hpp>

/* ★ 屏幕朝向：**用库的旋转表**（读 LovyanGFX 源码确认，不是猜的）★
 * ---------------------------------------------------------------------------
 * 关键源码（Panel_LCD.inl）：
 *   ① `_internal_rotation = ((r + offset_rotation) & 3) | ((r & 4) ^ (offset_rotation & 4))`
 *      —— **bit2 是"翻转标志"，不参与横竖屏判断**
 *   ② 用 `_internal_rotation` 算 `_width/_height/_colstart/_rowstart`，再写 MADCTL
 *   ③ 画布是否交换由 **`_internal_rotation & 1`** 决定：
 *        偶数 → 竖屏画布（240×320）   奇数 → 横屏画布（320×240）
 *
 * ★ 本屏的定值推导（实机 8 组 MADCTL 穷举 + 源码核对）★
 *   实机测试（直接写寄存器，不含 BGR）结论：
 *     **0xA0 正确且竖屏**（用户选中的"编号 5"是 0x60 —— 两者只差 MX/MY 的
 *       极性写法，朝向等价；0x60 那组在库的表里没有对应项，0xA0 有）
 *     0xA8 = 0xA0 | BGR  ⇒ 库的 `madctl_table[4]` = `MV|MY|ML` ✓
 *   ⇒ 目标 **`_internal_rotation = 4`**
 *   ⇒ 取 **`TFT_ROTATION = 4`、`offset_rotation = 0`**：
 *        `(4+0)&3 = 0`，`(4&4)^(0&4) = 4` → `_internal_rotation = 4` ✓
 *        `4 & 1 = 0` ⇒ 画布不交换 ⇒ **240×320 竖屏** ✓（与 LVGL 一致）
 *
 * ⚠ 两个血教训（都踩过）：
 *   ① **不要"调完 setRotation 再自己覆写 MADCTL"**：setRotation 同时算
 *      _colstart/_rowstart/_width/_height，单独覆盖寄存器会让两者不一致
 *      → 画面**整体转 90° + 镜像**。用库的表就够了，不必绕过。
 *   ② **画布尺寸必须一起对上**：r=3/offset=2 时 `&3` 同样是 3，但 `&1=1`
 *      → 库把画布算成 320×240，与 LVGL 的 240×320 冲突 → 错位。
 *
 * 判据（唯一）：**白方块在左上、绿线在上、蓝线在左、数字正着读、2 列卡片**。 */
#define TFT_ROTATION        4

/* ---- 面板定义（换屏只改这一个类）--------------------------------------- */
class Lcd : public lgfx::LGFX_Device {
    lgfx::Panel_ILI9341 _panel;
    lgfx::Bus_SPI      _bus;

public:
    Lcd()
    {
        {
            /* --- SPI 总线 ---
             * spi_host = SPI2_HOST → 用 SPI2(FSPI)。避开 SPI0/SPI1（内部 Flash 占用）。
             * ⚠ 必须用 SPI2_HOST 这个枚举，不能写 1：LovyanGFX 的 cfg.spi_host 类型是
             *   `spi_host_device_t`，写 int 会报
             *   "invalid conversion from 'int' to 'spi_host_device_t'"。 */
            auto cfg = _bus.config();
            cfg.spi_host    = SPI2_HOST;
            cfg.spi_mode    = 0;
            cfg.freq_write  = TFT_SPI_WRITE_HZ;   /* 40MHz；花屏就降到 27MHz */
            cfg.freq_read   = 16000000;
            cfg.spi_3wire   = false;
            cfg.use_lock    = true;
            /* ★ 关掉 DMA（2026-10，为治"雪花"）★
             * 现象：屏幕上出现**会移动的花屏/雪花块**（时而左侧、时而顶部），
             * 位置不固定 —— 这是**传输过程被破坏**的典型特征，而不是数据算错
             * （算错的话位置是固定的）。
             * 已排除/已处理：SPI 时钟降到 10MHz（仍有雪花）、缓冲改成整屏
             * （仍有雪花）⇒ 剩下的嫌疑就是 **DMA 搬运**与刷屏节奏的抢跑。
             * 关掉 DMA 后 LovyanGFX 走**阻塞式发送**：慢一点（整屏约
             * 240×320×2B / 10MHz ≈ 154ms），但**每次发送都是原子的**，
             * 不会再和 LVGL 的绘制抢缓冲。
             * 对"每秒刷新几次的仪表界面"来说，这点速度完全够用。
             * ⚠ 以后要提速，先修好 DMA 路径（缓冲对齐 + 传输完成回调），
             *   再打开这里。 */
            cfg.dma_channel = 0;
            cfg.pin_sclk    = TFT_PIN_SCK;
            cfg.pin_mosi    = TFT_PIN_MOSI;
            cfg.pin_miso    = TFT_PIN_MISO;       /* 只写屏可不接，但必须给合法脚 */
            cfg.pin_dc      = TFT_PIN_DC;
            _bus.config(cfg);
            _panel.setBus(&_bus);
        }
        {
            /* --- 面板（参数按 ILI9341 数据手册 + 实机穷举结果）---------------
             * ★ 本屏的最终配置（实机 8 组 MADCTL 穷举定出来的）★
             *   目标 MADCTL = **0x68** = MV|MY|ML|BGR
             *     （用户测试选中"编号 5"时的值是 0x60；0x68 只多了 BGR 位，
             *       BGR 只影响红蓝顺序、不影响朝向，且多数模块需要它）
             *   它对应库 Panel_LCD.hpp 的 `madctl_table[3]` ⇒ `_internal_rotation = 3`
             *   由 `_internal_rotation = (r + offset_rotation) & 3` 取
             *     **offset_rotation = 2、setRotation(1)**（见文件顶部 TFT_ROTATION）
             *   此时 `_internal_rotation & 1 = 1` ⇒ 库交换画布 ⇒ **240×320 竖屏** ✓
             *
             * ⚠ 两个血的教训（都在这里踩过）：
             *   ① **不要"调完 setRotation 再自己覆写 MADCTL"**：
             *      setRotation 会同时算出 _colstart/_rowstart/_width/_height，
             *      单独覆盖寄存器会让两者不一致 → 画面**整体转 90° + 镜像**。
             *      用库的旋转表就够了（0x68 能在表里表达，无需绕过）。
             *   ② **画布尺寸也必须对上**：曾用 r=3/offset=0 试过，`&3` 同样是 3，
             *      但库算出的画布是 320×240，与 LVGL 的 240×320 不一致 → 错位。
             *
             * 面板几何按数据手册原生值给：240(列) × 320(行)。
             * ⚠ 这是"芯片视角"，不是"用户视角"；用户看到的宽高由 MV 位决定。 */
            auto cfg = _panel.config();
            cfg.pin_cs           = TFT_PIN_CS;
            cfg.pin_rst          = TFT_PIN_RST;
            cfg.pin_busy         = -1;
            /* ★★ 面板几何：**物理 320×240**（2026-10 实机照片定出来的）★★
             * 之前按 ILI9341"原生 240×320"配 → 内容只画到 240px 宽，
             * 右边空出 320-240 = **80px**（用户照片实测正是这个值）。
             * 那不是偏移，是**面板宽度配错了**。
             * 现在按真实硬件填 320(宽)×240(高)。
             *
             * ★ 注意区分两件事（我今天混淆了很久）★
             *   · **面板物理/显存几何** = 硬件的真实像素数 → 这里固定 320×240
             *   · **用户看到的横竖** = MADCTL 的 **MV 位**（行列交换）→ 由
             *     `setRotation(TFT_ROTATION)` 决定，见文件顶部说明
             *   把这两件事分开之后，"内容只占 240 宽"和"竖屏观感"才能同时满足。 */
            cfg.panel_width      = 320;          /* 硬件真实宽度 */
            cfg.panel_height     = 240;          /* 硬件真实高度 */
            cfg.memory_width     = 320;          /* 控制器显存宽度（=硬件） */
            cfg.memory_height    = 240;          /* 控制器显存高度 */
            cfg.offset_x         = 0;            /* 无偏移（内容从 0 开始） */
            cfg.offset_y         = 0;
            /* ★ offset_rotation = 0：配 `setRotation(4)` 得
             *   `_internal_rotation = ((4+0)&3) | ((4&4)^(0&4)) = 0 | 4 = 4`
             *   → 库写 MADCTL = `madctl_table[4] | BGR` = **0xA8**（竖屏、不镜像）
             *   → 且 `4 & 1 == 0` ⇒ **画布不交换** ⇒ 240×320 与 LVGL 一致 ✓
             * ⚠ 别和 setRotation 的实参搞混：最终朝向由两者**一起**决定，
             *   而且**画布尺寸也依赖这个结果**（偶数=竖屏画布、奇数=横屏画布）。 */
            cfg.offset_rotation  = 0;
            cfg.readable         = true;          /* init 时读一次 ID，确认是不是 ILI9341 */
            cfg.invert           = false;         /* 出现"底片效果"就改 true */
            cfg.rgb_order        = false;         /* **红蓝反了**就改 true */
            cfg.dlen_16bit       = false;
            cfg.bus_shared       = false;         /* 触摸没接，独占总线 */
            _panel.config(cfg);
        }
        setPanel(&_panel);
    }
};

static Lcd               *s_lcd       = NULL;
static lv_disp_draw_buf_t s_draw_buf;
static lv_color_t        *s_draw_buf1 = NULL;

/* =============================================================================
 * ★ MADCTL 直写工具（绕开库的旋转推断，直接按数据手册控制朝向）★
 * -----------------------------------------------------------------------------
 * ILI9341 的 0x36 (MADCTL) 寄存器位定义（数据手册 §8.2.29）：
 *   bit7 MY  : 行地址方向（1 = 从下往上）
 *   bit6 MX  : 列地址方向（1 = 从右往左）
 *   bit5 MV  : 行列交换（1 = 交换，即横屏；0 = 竖屏）
 *   bit4 ML  : 垂直刷新方向（扫描方向，只影响自刷新，一般 0）
 *   bit3 BGR : 1 = BGR 顺序（**大多数模块需要 1**，否则红蓝互换）
 *   bit2 MH  : 水平刷新方向（一般 0）
 * 所以"竖屏 + 不镜像"= MV=0, MX=0, MY=0，加 BGR = **0x08**。
 * 今天绕最久的原因：LovyanGFX 的 setRotation 会把它的旋转表和这块屏的
 * 实际极性叠加，4 个值全落在错误的那一半 —— 直接写寄存器才能跳出这个死循环。
 *
 * ★ 重要：写 MADCTL 之后必须重新设置地址窗口（CASET/PASET），
 *   否则库缓存的窗口还是旧方向的，会出现**雪花/错位**（今天的雪花就可能是它）。
 * ========================================================================== */
static void lcd_apply_madctl(uint8_t madctl)
{
    if (s_lcd == NULL) {
        return;
    }
    s_lcd->startWrite();
    s_lcd->writeCommand(0x36);          /* MADCTL */
    s_lcd->writeData(madctl);
    s_lcd->endWrite();
}

/* 刷屏统计（排障用，见 TFT_FLUSH_STATS）：
 *   flush 一直在涨 → LVGL 在刷、数据推给了 SPI → 问题在硬件/线序/驱动 IC
 *   flush 一直为 0 → LVGL 这边没动静 → 问题在定时器/控件树 */
static volatile uint32_t s_flush_calls = 0;
static volatile uint32_t s_flush_px    = 0;

/* ---- LVGL 刷屏回调 --------------------------------------------------------
 * LVGL 把"某块区域已画好"的像素交给我们（area 是**含端点**的闭区间）。
 * 注意 w/h 要 +1：少算一列会看到右边/下边有一条永远不刷新的缝。 */
static void gui_flush_cb(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p)
{
    if (s_lcd != NULL) {
        const uint32_t w = (uint32_t)(area->x2 - area->x1 + 1);
        const uint32_t h = (uint32_t)(area->y2 - area->y1 + 1);
        /* ★★ 缓冲区必须按 `uint16_t*` 传，**不要**转成 `lgfx::rgb565_t*` ★★
         * 踩过的坑（2026-10，整整改了一天）：
         *   LVGL 的 `lv_color_t` 在 16 位色深下是 2 字节的**位域结构体**
         *   （`{blue:5, green:6, red:5}`），配合 `LV_COLOR_16_SWAP=1`，
         *   它在内存里的字节序是"屏要的那种"。
         *   而 `lgfx::rgb565_t` 是 C++ 结构体，LovyanGFX 对它有**自己的**
         *   字节序处理。两者叠加 = **转换做了两遍**：
         *     现象就是**规则的横向/纵向条纹 + 颜色发白发绿**，
         *     而且看起来非常像"镜像""缩放""SPI 太快"——极易误判。
         *   ⇒ 正确做法：把 LVGL 的缓冲当**裸 uint16 像素数组**交给 LovyanGFX，
         *     让它只做一次转换。`LV_COLOR_16_SWAP=1` + `(uint16_t*)` 是
         *     ESP32 + SPI 屏 + LVGL v8 的标准组合。 */
        s_lcd->pushImage((int32_t)area->x1, (int32_t)area->y1,
                         (int32_t)w, (int32_t)h, (const uint16_t *)color_p);
        s_flush_calls++;
        s_flush_px += w * h;
    }
    lv_disp_flush_ready(disp_drv);          /* ★ 不能省，否则只画第一块 ★ */
}

/* ---- 背光 ---------------------------------------------------------------
 * 用 LEDC 而不是 digitalWrite：占空比可调 = 以后加"亮度"设置项。
 * ⚠ 不用 analogWrite()：它在 ESP32-S3 上依赖 LEDC 通道分配，这里显式占通道 7。 */
static void gui_backlight_init(void)
{
#if TFT_BL_USE_GPIO
    const int ch = 7;
    ledcSetup(ch, 5000, 8);
    ledcAttachPin(TFT_PIN_BL, ch);
    ledcWrite(ch, 255);                     /* 100% 亮度 */
    APP_LOGI("TFT 背光：GPIO%d（LEDC ch%d，100%%）", TFT_PIN_BL, ch);
#endif
}

static bool gui_port_init(void)
{
    gui_oled_note();

#if !TFT_ENABLE
    /* 屏支持按 config.h 的 TFT_ENABLE 关掉（留个一键回退的口子）。 */
    APP_LOGI("LVGL SPI 屏（路径 B）已在 config.h 里关闭（TFT_ENABLE=0）："
             "本地显示走 OLED（路径 C）");
    return false;
#else
    /* ---- 1) 画缓冲 ------------------------------------------------------
     * ★ 2026-10 改动：优先申请**整屏**缓冲（320×240 = 153600 字节）★
     * 为什么：原来用 40 行的小缓冲，LVGL 会把屏幕切成很多横条**分块刷**。
     * 实测屏幕上出现**左侧一条雪花带**——小缓冲 + 多次 DMA 传输的路径上
     * 容易出这种局部损坏。改成整屏缓冲后 LVGL 一次刷完整幅，
     *   ① 路径简单、不会出现"半幅是新半幅是旧"的撕裂；
     *   ② 配合下面的 `full_refresh = 1`，每帧都是整屏一致的数据。
     * 内存：ESP32-S3 内部 DRAM 够（本工程 RAM 用量约 23%，加这 150KB 也能放下）。
     * 拿不到整屏就依次退回：PSRAM → 40 行内部 DMA → 普通堆。 */
    size_t buf_pixels = (size_t)LVGL_HOR_RES * LVGL_VER_RES;   /* 先试整屏 */
    size_t buf_bytes  = buf_pixels * sizeof(lv_color_t);

#if TFT_DRAW_BUF_IN_PSRAM
    s_draw_buf1 = (lv_color_t *)heap_caps_malloc(buf_bytes, MALLOC_CAP_SPIRAM);
    if (s_draw_buf1 != NULL) {
        APP_LOGW("TFT 画缓冲 %u 字节 @ PSRAM（整屏）", (unsigned)buf_bytes);
    }
#endif
    if (s_draw_buf1 == NULL) {
        s_draw_buf1 = (lv_color_t *)heap_caps_malloc(buf_bytes,
                                                     MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    }
    if (s_draw_buf1 == NULL) {
        /* 退回 40 行的小缓冲（老路子，能跑但可能出现分块瑕疵） */
        buf_pixels = (size_t)LVGL_HOR_RES * LVGL_DRAW_BUF_LINES;
        buf_bytes  = buf_pixels * sizeof(lv_color_t);
        s_draw_buf1 = (lv_color_t *)heap_caps_malloc(buf_bytes,
                                                     MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
        if (s_draw_buf1 != NULL) {
            APP_LOGW("整屏缓冲申请失败，退回 %d 行小缓冲", LVGL_DRAW_BUF_LINES);
        }
    }
    if (s_draw_buf1 == NULL) {
        s_draw_buf1 = (lv_color_t *)malloc(buf_bytes);   /* 最后退普通堆 */
    }
    if (s_draw_buf1 == NULL) {
        APP_LOGE("TFT 画缓冲分配失败（%u 字节）——界面无法建立", (unsigned)buf_bytes);
        return false;
    }
    APP_LOGI("TFT 画缓冲：%u 像素 = %u 字节（%s）",
             (unsigned)buf_pixels, (unsigned)buf_bytes,
             (buf_pixels == (size_t)LVGL_HOR_RES * LVGL_VER_RES) ? "整屏" : "分块");

    /* ---- 2) 屏幕初始化 ------------------------------------------------ */
    s_lcd = new Lcd();
    if (s_lcd == NULL) {
        APP_LOGE("Lcd 对象创建失败（内部堆不足）");
        free(s_draw_buf1);
        s_draw_buf1 = NULL;
        return false;
    }
    /* 每步 flush 一次串口：万一还在 init 里崩，能一眼看出崩在哪一步
     * （这个习惯是 TFT_eSPI 那次换来的，留着不亏）。 */
    Serial.flush(); APP_LOGI("[tft] step1: Lcd 对象已创建");
    s_lcd->init();
    Serial.flush(); APP_LOGI("[tft] step2: init() 返回");

    /* ---- 朝向：**用库的旋转表**（见本文件顶部 TFT_ROTATION 的说明）----
     * ⚠ 千万不要在这里"调完 setRotation 再自己覆写 MADCTL"：
     *   setRotation 会同时算出 _colstart/_rowstart 等内部地址窗口，
     *   单独覆盖寄存器会让两者不一致 → 画面整体转 90° + 镜像（我踩过）。
     *   库的旋转表已经能表达我们需要的朝向（r=3 → MADCTL 0x68 = 0x60|BGR）。 */
    s_lcd->setRotation(TFT_ROTATION);
    Serial.flush();
    APP_LOGI("[tft] step3: setRotation(%d) → 报告尺寸 %dx%d",
             TFT_ROTATION, (int)s_lcd->width(), (int)s_lcd->height());

    s_lcd->fillScreen(TFT_BLACK);
    Serial.flush(); APP_LOGI("[tft] step4: fillScreen ok");
    gui_backlight_init();
    Serial.flush(); APP_LOGI("[tft] step5: 背光 ok");

    /* ---- 3) 注册 LVGL 显示驱动 ---------------------------------------- */
    lv_disp_draw_buf_init(&s_draw_buf, s_draw_buf1, NULL, buf_pixels);

    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res      = LVGL_HOR_RES;
    disp_drv.ver_res      = LVGL_VER_RES;
    disp_drv.flush_cb     = gui_flush_cb;
    disp_drv.draw_buf     = &s_draw_buf;
    /* full_refresh = 0 = 局部刷新（LVGL 只重画脏区域）。
     * ⚠ 不要设成 1：LVGL v8 要求 full_refresh 时缓冲必须是整屏的，
     *   而整屏缓冲在"内存紧张退回分块"的分支下会不满足，直接触发断言。
     *   局部刷新 + 整屏缓冲的组合最好：平时只刷脏区（快），
     *   需要整屏重画时 LVGL 也能一次画完（因为缓冲够大）。 */
    disp_drv.full_refresh = 0;
    lv_disp_drv_register(&disp_drv);

    APP_LOGI("TFT 就绪（LovyanGFX + ILI9341）：%dx%d  SCLK=%d MOSI=%d CS=%d DC=%d RST=%d BL=%d",
             LVGL_HOR_RES, LVGL_VER_RES,
             TFT_PIN_SCK, TFT_PIN_MOSI, TFT_PIN_CS, TFT_PIN_DC, TFT_PIN_RST, TFT_PIN_BL);
    /* ★ 一致性自检：LVGL 的 canvas 尺寸必须和**屏实际报告的尺寸**一致 ★
     * 不一致的后果：界面按一个方向的尺寸排版、屏按另一个方向显示 →
     *   看起来就是"内容整体转了 90° / 被拉伸 / 错位"，
     *   极容易被误判成"旋转没调对"（我今天就在这上面绕了很久）。
     * 这条日志把两个值并排打出来，一眼就能看出是否一致。 */
    APP_LOGI("[tft] 尺寸一致性：LVGL canvas = %dx%d，屏报告 = %dx%d  %s",
             LVGL_HOR_RES, LVGL_VER_RES,
             (int)s_lcd->width(), (int)s_lcd->height(),
             ((int)s_lcd->width() == LVGL_HOR_RES &&
              (int)s_lcd->height() == LVGL_VER_RES) ? "✓ 一致"
                                                   : "✗ **不一致，必然是这里的问题**");

    /* ---- 4) 触摸：本屏带 XPT2046，但**先不接**（省 5 个引脚，还要校准）。
     * 以后要加就在这里注册（LovyanGFX 内置 XPT2046）：
     *     // 用 s_lcd->getTouch(&x, &y) 读坐标，再缩放到 320×240 后
     *     // 交给 lv_indev_drv_register 注册的 gui_touch_read_cb。
     *     // 注意 XPT2046 和屏共用 SPI，要做片选仲裁（bus_shared = true）。 */

    return true;
#endif  /* TFT_ENABLE */
}
#endif  /* GUI_USE_ESP_LVGL_PORT */

/* =============================================================================
 * §4  数据页（dashboard）
 * ============================================================================= */

void gui_dashboard(lv_obj_t *parent)
{
    if (parent == NULL) {
        return;
    }
    /* 用 flex 自动换行排卡片：分辨率一变（[TBD-1]），卡片自动重排，不用改坐标 */
    lv_obj_set_flex_flow(parent, LV_FLEX_FLOW_ROW_WRAP);
    lv_obj_set_flex_align(parent, LV_FLEX_ALIGN_SPACE_EVENLY,
                          LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_all(parent, 4, 0);
    lv_obj_set_style_pad_row(parent, 4, 0);
    lv_obj_set_style_pad_column(parent, 4, 0);

    for (int i = 0; i < DASH_CARD_COUNT; i++) {
        dash_card_t *c = &s_cards[i];

        lv_obj_t *card = lv_obj_create(parent);
        /* 竖屏 240 宽：卡片宽度按"两列"算（240 - 边距）/2 ≈ 104，
         * 高度留够两行字（标题 12px + 数值 20px）。 */
        lv_obj_set_size(card, 104, 62);
        lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);
        /* ★ 白底黑字 ★
         * 只做**浅色微调**：白底 + 浅灰描边 + 圆角。文字用纯黑。
         * **不要**再设深色背景（之前设成 0x1E2A38 深蓝黑，把主题盖掉了）。 */
        lv_obj_set_style_bg_color(card, lv_color_white(), 0);
        lv_obj_set_style_border_color(card, lv_color_hex(0xC8D0D8), 0);
        lv_obj_set_style_border_width(card, 1, 0);
        lv_obj_set_style_radius(card, 6, 0);
        lv_obj_set_style_pad_all(card, 4, 0);
        lv_obj_set_flex_flow(card, LV_FLEX_FLOW_COLUMN);
        lv_obj_set_flex_align(card, LV_FLEX_ALIGN_CENTER,
                              LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

        lv_obj_t *title = lv_label_create(card);
        lv_label_set_text(title, c->title);
        lv_obj_set_style_text_color(title, lv_color_hex(0x505860), 0);   /* 深灰标题 */
        /* ★ 标题用 16px 中文字库 ★
         * 14px 的汉字在 320×240 屏上笔画会糊在一起、看起来像"小方块"
         * （用户实机反馈：标签页读得清、卡片标题是方框）。
         * 三档字库见 lv_conf.h：14 默认 / 16 标题 / 20 数值。 */
        lv_obj_set_style_text_font(title, &lv_font_cn_16, 0);

        c->val_lbl = lv_label_create(card);
        lv_label_set_text(c->val_lbl, "--");
        /* 数值用**纯黑 + 20px 中文字库**（含 ASCII，数字也走它，
         * 这样万一以后数值里带单位汉字也不会变方框）。 */
        lv_obj_set_style_text_color(c->val_lbl, lv_color_black(), 0);
        lv_obj_set_style_text_font(c->val_lbl, &lv_font_cn_20, 0);
    }
}

void gui_update_env(const slow_regs_t *env)
{
    if (!s_ui_ready || env == NULL) {
        return;
    }
    for (int i = 0; i < DASH_CARD_COUNT; i++) {
        dash_card_t *c = &s_cards[i];
        const uint16_t raw = slow_u16(env, c->addr);
        const float v = c->is_signed ? ((float)(int16_t)raw / c->div)
                                     : ((float)raw / c->div);
        set_value_text(c->val_lbl, v, c->decimals, c->unit);
    }
    /* TODO(卡片着色): 依据 B 区 0x001A 本地报警位 / 0x0027~0x0029 阈值
     * 把超限卡片染红。阈值判断放 GUI 还是放报警仲裁模块，等 v0.4 的报警联动
     * 一起定（plan.md §8 B 表 v0.4）。 */
}

/* =============================================================================
 * §5  波形页（MPU6050 三轴 + 推理结果）
 * ============================================================================= */

/* ---- 振动强度（给 OLED 的"振动 std"用）--------------------------------------
 * 为什么放在这里而不是 oled.cpp：算 std 要遍历一个窗口的 100×6 个样本，
 * 那是"窗口消费者"的活；显示模块只该拿到一个已经算好的数字。
 *
 * 指标定义（和 docs/03 阶段 6 的判读口径对齐，见 tools/precheck_quiet.py）：
 *   std = 该轴 100 个样本的标准差（原始 LSB）。
 *         MPU6050 ±2g 量程下 1g ≈ 16384 LSB；实测静止时 az std ≈ 20~60，
 *         有振动 ≈ 100~500，剧烈 ≈ >800。所以这个数**量级**就够判断了。
 *   pp  = 峰峰值（max-min），用来区分"小幅抖动"(std 高 pp 低) 和
 *         "大幅低频摆动"(两者都高) —— 单看 std 会把这两种混在一起。
 *
 * 取三轴加速度里最大的那个（哪一轴在振都算振）；陀螺仪不参与，
 * 因为它的 std 和"振动强度"没有可比量纲（而且噪声底噪大得多）。
 *
 * ⚠ 整数运算 + int64 中间量：100 个 int16 的平方和最大约 100×32768² ≈ 1.07e11，
 *   超过 int32（2.1e9）—— 用 int32 会静默溢出，得到一个看起来"合理"的错值。
 *   这里算 var = (n*sum2 - sum*sum)/n²，再对 var 开根号。
 * -------------------------------------------------------------------------- */
static void vib_stats_accel(const acq_window_t *win, uint16_t *out_std, uint16_t *out_pp)
{
    uint16_t best_std = 0;
    uint16_t best_pp  = 0;

    for (int axis = 0; axis < 3; axis++) {    /* 0=ax 1=ay 2=az */
        int32_t  vmin = 32767;
        int32_t  vmax = -32768;
        int64_t  sum  = 0;
        int64_t  sum2 = 0;

        for (int i = 0; i < WINDOW_SAMPLES; i++) {
            const int32_t v = (int32_t)win->samples[i * WINDOW_AXES + axis];
            sum  += v;
            sum2 += (int64_t)v * (int64_t)v;
            if (v < vmin) vmin = v;
            if (v > vmax) vmax = v;
        }

        const int64_t n  = (int64_t)WINDOW_SAMPLES;
        const int64_t nv = n * sum2 - sum * sum;     /* = n² · 方差（未归一化） */
        uint32_t std_raw = 0;
        if (nv > 0) {
            /* var = nv / n²  →  std = sqrt(var) = sqrt(nv) / n */
            const double std_d = sqrt((double)nv) / (double)n;
            std_raw = (uint32_t)(std_d + 0.5);
        }
        if (std_raw > 65535u) {
            std_raw = 65535u;                        /* 裁剪（正常绝不会到这里） */
        }
        const uint32_t pp = (uint32_t)(vmax - vmin);
        if (std_raw > best_std) best_std = (uint16_t)std_raw;
        if (pp > best_pp)       best_pp  = (uint16_t)pp;
    }

    if (out_std != NULL) *out_std = best_std;
    if (out_pp  != NULL) *out_pp  = best_pp;
}

void gui_waveform(lv_obj_t *parent)
{
    if (parent == NULL) {
        return;
    }
    lv_obj_set_flex_flow(parent, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(parent, 4, 0);
    lv_obj_set_style_pad_row(parent, 4, 0);

    s_chart = lv_chart_create(parent);
    lv_obj_set_size(s_chart, LV_PCT(100), LV_PCT(60));
    lv_chart_set_type(s_chart, LV_CHART_TYPE_LINE);
    lv_chart_set_point_count(s_chart, GUI_WAVE_POINTS);
    lv_chart_set_range(s_chart, LV_CHART_AXIS_PRIMARY_Y, GUI_WAVE_Y_MIN, GUI_WAVE_Y_MAX);
    lv_chart_set_div_line_count(s_chart, 3, 5);
    /* 三条曲线 = 加速度 ax/ay/az（振动任务主要看这三个；陀螺仪同理可加）
     * 颜色固定，图例由下面的 s_wave_info 文字说明 */
    s_ser_ax = lv_chart_add_series(s_chart, lv_color_hex(0xE74C3C), LV_CHART_AXIS_PRIMARY_Y);
    s_ser_ay = lv_chart_add_series(s_chart, lv_color_hex(0x2ECC71), LV_CHART_AXIS_PRIMARY_Y);
    s_ser_az = lv_chart_add_series(s_chart, lv_color_hex(0x3498DB), LV_CHART_AXIS_PRIMARY_Y);

    s_wave_info = lv_label_create(parent);
    lv_label_set_text(s_wave_info, "ax=red ay=green az=blue | 等待窗口...");

    s_infer_lbl = lv_label_create(parent);
    lv_label_set_text(s_infer_lbl, "推理: 模型未部署");

    s_conf_bar = lv_bar_create(parent);
    lv_obj_set_size(s_conf_bar, LV_PCT(60), 12);
    lv_bar_set_range(s_conf_bar, 0, 100);
    lv_bar_set_value(s_conf_bar, 0, LV_ANIM_OFF);
}

void gui_update_wave(const acq_window_t *win)
{
    if (win == NULL) {
        return;
    }

    /* ---- 先算振动指标喂给 OLED（路径 C），**不受下面 LVGL 节流影响** ----
     * 为什么顺序重要：LVGL 的节流是"屏幕画得别太勤"，而 OLED 那条线的数据
     * 只应该跟着窗口（2s 一个）走；如果先 return，OLED 的 std 就会跟着
     * LVGL 的节流一起被吃掉，表现为屏幕上长期停在某个旧值。 */
    uint16_t vstd = 0;
    uint16_t vpp  = 0;
    vib_stats_accel(win, &vstd, &vpp);
#if OLED_ENABLE
    oled_set_vibration(vstd, vpp, win->win_seq);
#endif
    s_vib_std_last = vstd;      /* 诊断页用（与 OLED 同一个数） */
    s_vib_pp_last  = vpp;

    if (!s_ui_ready || s_chart == NULL) {
        return;
    }
    /* 节流：窗口是 2s 一个，没必要每来一条队列消息都重画 */
    const uint32_t now = millis();
    if (s_wave_last_ms != 0 && (now - s_wave_last_ms) < GUI_WAVE_MIN_INTERVAL_MS) {
        return;
    }
    s_wave_last_ms = now;

    /* 抽点：100 个样本 → GUI_WAVE_POINTS(50) 个显示点。
     * samples[] 是行优先的 100×6（docs/02 §5-D），所以第 i 个样本的
     * ax/ay/az 在 samples[i*6 + 0/1/2]。 */
    const int step = (WINDOW_SAMPLES > GUI_WAVE_POINTS) ? (WINDOW_SAMPLES / GUI_WAVE_POINTS) : 1;

    /* ★★ 去均值 + 自适应 Y 轴（2026-10，第二次修正）★★
     * ---------------------------------------------------------------------
     * 为什么必须**去均值**（只画波动量）：
     *   加速度原始值里**重力占了大头**，而且三个轴的重力分量方向不同：
     *       ax ≈ -10700   ay ≈ -10500   az ≈ +6350
     *   三条线各占 Y 轴一端、**永不交叠**；而振动引起的波动只有 **±200**。
     *   如果直接把原始值画在共用的一根 Y 轴上，范围必须同时容纳三个均值
     *   （跨度 ≈17000），波动只占 1% —— 三条都是**平直线**（用户实测反馈"很平"）。
     *   ⇒ 显示 **v - 本轴均值**，三条线都围绕 0，波动被放大到占满屏高。
     *
     * 为什么均值按**阈值自动选**（而不是永远去均值）：
     *   · 静止/微振（波动远小于重力）→ 去均值才有意义，否则看不出东西
     *   · 剧烈振动（波动量与重力可比）→ 原始值本身就"有形状"，
     *     去均值反而会把它压扁成对称波形，丢失"哪个方向受力"的信息
     *   取 `span > GUI_WAVE_DEMEAN_SPAN`（默认 2000，约 0.12g）才算"剧烈"。
     * --------------------------------------------------------------------- */
    int32_t raw_min = 0, raw_max = 0;
    bool have_raw = false;
    int64_t sum[3] = { 0, 0, 0 };
    int     npts   = 0;

    for (int p = 0; p < GUI_WAVE_POINTS; p++) {
        const int si = p * step;
        if (si >= WINDOW_SAMPLES) {
            break;
        }
        npts++;
        for (int a = 0; a < 3; a++) {
            const int32_t v = (int32_t)win->samples[si * WINDOW_AXES + a];
            sum[a] += v;
            if (!have_raw) {
                raw_min = raw_max = v;
                have_raw = true;
            } else {
                if (v < raw_min) raw_min = v;
                if (v > raw_max) raw_max = v;
            }
        }
    }
    if (npts <= 0) {
        return;
    }

    const int32_t raw_span = raw_max - raw_min;
    const bool    demean   = (raw_span <= GUI_WAVE_DEMEAN_SPAN);
    int32_t       mean[3]  = { 0, 0, 0 };
    for (int a = 0; a < 3; a++) {
        mean[a] = (int32_t)(sum[a] / npts);
    }

    /* 算最终显示值的范围（用于自适应 Y 轴） */
    int32_t vmin = 0, vmax = 0;
    bool have_v = false;
    for (int p = 0; p < GUI_WAVE_POINTS; p++) {
        const int si = p * step;
        if (si >= WINDOW_SAMPLES) {
            break;
        }
        for (int a = 0; a < 3; a++) {
            int32_t v = (int32_t)win->samples[si * WINDOW_AXES + a];
            if (demean) {
                v -= mean[a];
            }
            if (!have_v) {
                vmin = vmax = v;
                have_v = true;
            } else {
                if (v < vmin) vmin = v;
                if (v > vmax) vmax = v;
            }
        }
    }
    if (have_v) {
        int32_t span = vmax - vmin;
        if (span < 100) {                  /* 最小跨度 100（counts），别被噪声抖花 */
            const int32_t mid = (vmax + vmin) / 2;
            vmin = mid - 50;
            vmax = mid + 50;
            span = 100;
        }
        const int32_t pad = span / 10 + 1; /* 上下各 10% 余量 */
        lv_chart_set_range(s_chart, LV_CHART_AXIS_PRIMARY_Y, vmin - pad, vmax + pad);
    }

    for (int p = 0; p < GUI_WAVE_POINTS; p++) {
        const int si = p * step;
        if (si >= WINDOW_SAMPLES) {
            break;
        }
        int32_t v[3];
        for (int a = 0; a < 3; a++) {
            v[a] = (int32_t)win->samples[si * WINDOW_AXES + a];
            if (demean) {
                v[a] -= mean[a];
            }
        }
        lv_chart_set_value_by_id(s_chart, s_ser_ax, (uint16_t)p, (lv_coord_t)v[0]);
        lv_chart_set_value_by_id(s_chart, s_ser_ay, (uint16_t)p, (lv_coord_t)v[1]);
        lv_chart_set_value_by_id(s_chart, s_ser_az, (uint16_t)p, (lv_coord_t)v[2]);
    }
    lv_chart_refresh(s_chart);

    if (s_wave_info != NULL) {
        char buf[80];
        snprintf(buf, sizeof(buf),
                 "seq=%u  rate=%.1fHz  ts=%ums  %s",
                 (unsigned)win->win_seq,
                 (double)win->rate_x10 / 10.0,
                 (unsigned)win->ts_10ms * 10u,
                 (win->win_head & WINDOW_HEAD_VALID_BIT) ? "valid" : "INVALID");
        lv_label_set_text(s_wave_info, buf);
    }
    /* TODO(波形增强)：
     *  · 加陀螺仪 gx/gy/gz 三条曲线（图表支持多 series，再加 3 个即可）；
     *  · 加"暂停/继续"按钮，方便截图；
     *  · 把 win->seq_mismatch_count > 0 的窗口标成黄色（说明发生过跨窗口拼接重读）。 */
}

void gui_update_infer(const infer_result_t *res)
{
    if (!s_ui_ready || res == NULL || s_infer_lbl == NULL) {
        return;
    }
    char buf[80];
    if (!res->valid) {
        snprintf(buf, sizeof(buf), "推理: 无结果（模型未部署或 %u us 超时）",
                 (unsigned)res->infer_us);
        lv_label_set_text(s_infer_lbl, buf);
        if (s_conf_bar != NULL) {
            lv_bar_set_value(s_conf_bar, 0, LV_ANIM_OFF);
        }
        return;
    }
    const char *label = infer_label(res->best);
    snprintf(buf, sizeof(buf), "推理: %s  %.1f%%   (%u us, seq=%u)",
             label,
             (double)res->probs[res->best] * 100.0,
             (unsigned)res->infer_us,
             (unsigned)res->win_seq);
    lv_label_set_text(s_infer_lbl, buf);
    if (s_conf_bar != NULL) {
        lv_bar_set_value(s_conf_bar, (int32_t)(res->probs[res->best] * 100.0f), LV_ANIM_OFF);
    }
    /* TODO(多类别显示)：用 LV_USE_TABLE 或一排 lv_bar 把各类概率都显示出来
     *  （对比实验要同时看"正常/不平衡/松动"三个值）。 */
}

/* =============================================================================
 * §6  诊断页（统计 + 控制按钮）
 * ============================================================================= */

/* ---- 控制按钮的回调：只投 g_cmd_queue，不做任何总线操作 ---- */

static void cb_beep(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) {
        return;
    }
    /* v8.3 的 lv_event_get_target() 返回 void*，显式转一下（v8.0/8.1 返回 lv_obj_t*，
     * 显式 C 风格转换对两种签名都成立） */
    lv_obj_t *sw = (lv_obj_t *)lv_event_get_target(e);
    const uint16_t mask = CTRL1_BEEP;   /* 只动 bit0；bit8 是"长鸣"，不用 */
    const bool on = lv_obj_has_state(sw, LV_STATE_CHECKED);
    post_cmd(on ? CMD_CTRL1_SET_BITS : CMD_CTRL1_CLR_BITS, REG_CTRL1, 1, &mask);
}

static void cb_fan(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_RELEASED) {
        return;
    }
    const uint16_t pwm = (uint16_t)lv_slider_get_value(s_sl_fan);
    if (s_lbl_fan != NULL) {
        char buf[24];
        snprintf(buf, sizeof(buf), "%u / 1000", (unsigned)pwm);
        lv_label_set_text(s_lbl_fan, buf);
    }
    /* 走 CMD_FAN_SET：一条 FC10 原子写 [控制字|风扇使能, PWM]（docs/02 §6⑥） */
    post_cmd(CMD_FAN_SET, REG_FAN_PWM, 1, &pwm);
}

static void cb_servo(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_RELEASED) {
        return;
    }
    const uint16_t ang = (uint16_t)lv_slider_get_value(s_sl_servo);
    if (s_lbl_servo != NULL) {
        char buf[24];
        snprintf(buf, sizeof(buf), "%u deg", (unsigned)ang);
        lv_label_set_text(s_lbl_servo, buf);
    }
    /* 两条命令：先使能舵机（控制字 bit4），再写角度。
     * TODO: 严格来说这两条不是原子的（中间可能被窗口读插进来）。
     *       若实测出现"角度没生效"，改成一条 FC10 连写 0x0020~0x0023。 */
    const uint16_t en = CTRL1_SERVO_EN;
    post_cmd(CMD_CTRL1_SET_BITS, REG_CTRL1, 1, &en);
    post_cmd(CMD_WRITE_SINGLE, REG_SERVO_ANGLE, 1, &ang);
}

static void cb_rgb_apply(lv_event_t *e)
{
    (void)e;
    const uint16_t rgb[3] = {
        (uint16_t)lv_slider_get_value(s_sl_rgb[0]),
        (uint16_t)lv_slider_get_value(s_sl_rgb[1]),
        (uint16_t)lv_slider_get_value(s_sl_rgb[2]),
    };
    for (int i = 0; i < 3; i++) {
        if (s_lbl_rgb[i] != NULL) {
            char buf[16];
            snprintf(buf, sizeof(buf), "%u", (unsigned)rgb[i]);
            lv_label_set_text(s_lbl_rgb[i], buf);
        }
    }
    const uint16_t en = CTRL1_RGB_EN;
    post_cmd(CMD_CTRL1_SET_BITS, REG_CTRL1, 1, &en);
    /* REG_RGB_R/G/B = 0x0024/25/26 连续，一次 FC10 写完（docs/02 §5-C） */
    post_cmd(CMD_WRITE_MULTIPLE, REG_RGB_R, 3, rgb);
}

static void cb_safe_shutdown(lv_event_t *e)
{
    (void)e;
    /* 控制字 bit9：全部输出安全关断（docs/02 §5-C）。
     * 这是安全相关的紧急按钮，所以单独成一个命令，不依赖控制字快照。 */
    const uint16_t dummy = 0;
    post_cmd(CMD_SAFE_SHUTDOWN, REG_CTRL1, 1, &dummy);
    APP_LOGW("GUI: 已下发【全部输出安全关断】");
}

static void cb_force_window(lv_event_t *e)
{
    (void)e;
    const uint16_t dummy = 0;
    post_cmd(CMD_FORCE_WINDOW, REG_CTRL1, 1, &dummy);
}

/* ---- 一个滑块行（标题 + 滑块 + 数值），返回滑块句柄 ---- */
static lv_obj_t *make_slider_row(lv_obj_t *parent, const char *title,
                                 int32_t min, int32_t max, int32_t init,
                                 lv_obj_t **out_lbl)
{
    lv_obj_t *row = make_row(parent, title);
    lv_obj_t *sl = lv_slider_create(row);
    lv_obj_set_flex_grow(sl, 1);
    lv_slider_set_range(sl, min, max);
    lv_slider_set_value(sl, init, LV_ANIM_OFF);

    lv_obj_t *lbl = lv_label_create(row);
    char buf[24];
    snprintf(buf, sizeof(buf), "%d", (int)init);
    lv_label_set_text(lbl, buf);
    lv_obj_set_width(lbl, 70);

    if (out_lbl != NULL) {
        *out_lbl = lbl;
    }
    return sl;
}

void gui_diag(lv_obj_t *parent)
{
    if (parent == NULL) {
        return;
    }
    lv_obj_set_flex_flow(parent, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(parent, 4, 0);
    lv_obj_set_style_pad_row(parent, 3, 0);

    /* ---- 统计文字块（一次多行刷完，比十几个 label 省事也更省 CPU） ---- */
    s_diag_lbl = lv_label_create(parent);
    lv_label_set_text(s_diag_lbl, "等待统计...");

    /* ---- 状态翻转信号（在线/离线灰显）---- */
    /* TODO: 离线时把整个 tabview 置灰 —— lv_obj_add_state(parent, LV_STATE_DISABLED)，
     *       并把 s_diag_lbl 的文字染红。docs/03 阶段 4 的验收要求
     *       "拔掉 485 后界面 2s 内显示节点离线"。 */

    /* ---- 控制按钮区 ---- */
    lv_obj_t *ctl = lv_obj_create(parent);
    lv_obj_set_size(ctl, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_clear_flag(ctl, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_opa(ctl, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(ctl, 0, 0);
    lv_obj_set_style_pad_all(ctl, 2, 0);
    lv_obj_set_flex_flow(ctl, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(ctl, 3, 0);

    /* 蜂鸣器：开关 */
    {
        lv_obj_t *row = make_row(ctl, "蜂鸣器");
        s_sw_beep = lv_switch_create(row);
        lv_obj_add_event_cb(s_sw_beep, cb_beep, LV_EVENT_VALUE_CHANGED, NULL);
    }

    /* 风扇 PWM：滑块 0~1000（docs/02 §5-C，FC10 原子写） */
    s_sl_fan = make_slider_row(ctl, "风扇PWM", 0, 1000, 0, &s_lbl_fan);
    lv_obj_add_event_cb(s_sl_fan, cb_fan, LV_EVENT_RELEASED, NULL);

    /* 舵机角度：滑块 0~180 */
    s_sl_servo = make_slider_row(ctl, "舵机角度", 0, 180, 90, &s_lbl_servo);
    lv_obj_add_event_cb(s_sl_servo, cb_servo, LV_EVENT_RELEASED, NULL);

    /* RGB：三个滑块 + 一个应用按钮（0~255，docs/02 §5-C） */
    {
        lv_obj_t *row = make_row(ctl, "RGB R/G/B");
        for (int i = 0; i < 3; i++) {
            s_sl_rgb[i] = lv_slider_create(row);
            lv_obj_set_flex_grow(s_sl_rgb[i], 1);
            lv_slider_set_range(s_sl_rgb[i], 0, 255);
            lv_slider_set_value(s_sl_rgb[i], 0, LV_ANIM_OFF);
        }
        lv_obj_t *apply = lv_btn_create(row);
        lv_obj_t *al = lv_label_create(apply);
        lv_label_set_text(al, "应用");
        lv_obj_add_event_cb(apply, cb_rgb_apply, LV_EVENT_CLICKED, NULL);
    }

    /* 危险/特殊动作 */
    {
        lv_obj_t *row = make_row(ctl, "动作");
        lv_obj_t *b1 = lv_btn_create(row);
        lv_obj_t *l1 = lv_label_create(b1);
        lv_label_set_text(l1, "安全关断");
        lv_obj_add_event_cb(b1, cb_safe_shutdown, LV_EVENT_CLICKED, NULL);

        lv_obj_t *b2 = lv_btn_create(row);
        lv_obj_t *l2 = lv_label_create(b2);
        lv_label_set_text(l2, "强制重采窗口");
        lv_obj_add_event_cb(b2, cb_force_window, LV_EVENT_CLICKED, NULL);
    }
}

void gui_update_diag(const mb_stats_t *st)
{
    if (!s_ui_ready || st == NULL || s_diag_lbl == NULL) {
        return;
    }
    slow_regs_t snap;
    const bool have_snap = slow_get_latest(&snap);
    const uint16_t win_seq = have_snap ? slow_u16(&snap, REG_WIN_STATUS) : 0;
    const uint16_t sample_seq = have_snap ? slow_u16(&snap, REG_SAMPLE_SEQ) : 0;
    const uint16_t mode = have_snap ? slow_u16(&snap, REG_MODE) : 0;
    const uint16_t alarm = have_snap ? slow_u16(&snap, REG_ALARM) : 0;

    uint32_t silent_ms = 0;
    if (st->last_rx_ms != 0) {
        silent_ms = millis() - st->last_rx_ms;
    }

    char buf[420];
    snprintf(buf, sizeof(buf),
             "节点: %s\n"
             "TX %u  RX %u  超时 %u  重试 %u\n"
             "CRC错 %u  异常 %u(码 %u)\n"
             "连续超时 %u / %d   静默 %ums\n"
             "窗口序号 0x%04X  采样序号 0x%04X\n"
             "模式 %s  报警位 0x%04X\n"
             "序号失配重读 %u 次\n"
             "振动 std %u  峰峰 %u  (三轴加速度最大值)\n"
             "推理耗时 %u us",
             st->online ? "在线" : "离线",
             (unsigned)st->tx_count, (unsigned)st->rx_count,
             (unsigned)st->timeout_count, (unsigned)st->retry_count,
             (unsigned)st->crc_err_count, (unsigned)st->exception_count,
             (unsigned)st->last_exception_code,
             (unsigned)st->consecutive_timeout, MODBUS_OFFLINE_TIMEOUTS,
             (unsigned)silent_ms,
             (unsigned)win_seq, (unsigned)sample_seq,
             (mode == 0) ? "本地自治" : "远程受控",
             (unsigned)alarm,
             (unsigned)st->seq_mismatch_count,
             (unsigned)s_vib_std_last, (unsigned)s_vib_pp_last,
             (unsigned)infer_last_us());

    lv_label_set_text(s_diag_lbl, buf);
    /* TODO(离线灰显): st->online == false 时
     *   lv_obj_add_state(控件, LV_STATE_DISABLED) + 文字染红。 */
}

void gui_set_online(bool online)
{
    /* 1) OLED（路径 C）：这条必须在 s_ui_ready 判断**之前**。
     *    理由就是本文件顶上那条"两条显示通道互相独立"的设计口径：
     *    LVGL 屏没落地的时候，OLED 恰恰是唯一能看到"485 掉线了"的地方，
     *    如果这里先被 s_ui_ready 挡住，OLED 上就会永远停在 ONLINE。 */
#if OLED_ENABLE
    oled_on_online_event(online);
#endif

    if (!s_ui_ready) {
        return;
    }
    /* LVGL 侧现在只打日志占位；真正的灰显在 gui_update_diag() 里读 st->online 即可
     * （诊断页 1s 刷一次，比事件驱动更省事，也不丢状态）。
     * 这里保留独立入口是因为 plan.md §7.2 把"报警仲裁"列成了单独职责。 */
    APP_LOGI("GUI: 节点 %s", online ? "上线" : "离线");
}

/* =============================================================================
 * §7  初始化与主循环
 * ============================================================================= */

static void gui_build_ui(void)
{
    lv_obj_t *scr = GUI_SCREEN_ACTIVE();
    /* ★ 配色：**白底黑字**（用户明确要求）★
     * 做法是"**不覆盖主题**"而不是"手动设成白色"：
     *   lv_conf.h 里 `LV_THEME_DEFAULT_DARK = 0`，LVGL 的 default 主题
     *   **本身就是浅色**（白底 + 黑字 + 浅灰控件）。
     *   之前这里写了一句 `lv_obj_set_style_bg_color(scr, 0x101820)`（深蓝黑），
     *   把主题盖掉了 —— 那是"仪表盘风格"的审美选择，不是必需的。
     *   现在删掉所有深色覆盖，只做**少量**浅色微调（卡片描边、圆角），
     *   界面自然就是白底黑字。
     * ⚠ 以后想改配色，**改主题或改这里的少量覆盖**，不要在每个控件上硬编码颜色。 */
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    s_tabview = lv_tabview_create(scr, LV_DIR_TOP, GUI_TAB_BAR_H);
    lv_obj_set_size(s_tabview, LV_PCT(100), LV_PCT(100));

    lv_obj_t *tab_data = lv_tabview_add_tab(s_tabview, "数据");
    lv_obj_t *tab_wave = lv_tabview_add_tab(s_tabview, "波形");
    lv_obj_t *tab_diag = lv_tabview_add_tab(s_tabview, "诊断");

    gui_dashboard(tab_data);
    gui_waveform(tab_wave);
    gui_diag(tab_diag);

    s_ui_ready = true;
    /* 轮播计时从"界面建好"这一刻起算，避免开机后立刻切页 */
    s_page_last_ms = millis();
    APP_LOGI("GUI: 三个界面已建立（数据 / 波形 / 诊断）");
    /* TODO(中文显示)：LVGL 默认字体只有 ASCII，中文会全部显示成方框。
     * 需要做一个 12/16px 的中文字库（LV_FONT_CUSTOM_DECLARE + lv_font_conv），
     * 把常用汉字（温度/湿度/离线/报警...）打进去，然后在 lv_conf.h 里
     * 设 LV_FONT_DEFAULT。字库放 Flash，别放 RAM。 */
}

bool gui_init(void)
{
    lv_init();

    if (!gui_port_init()) {
        /* 没有 LVGL 显示驱动就不要往下建控件：建了也刷不出来，还白占内存。
         * ⚠ 注意这里**只表示 LVGL 屏不可用**：
         *   路径 C（OLED，见 oled.cpp）在 setup() 里已经初始化好、由 task_oled
         *   自己刷屏，不受这个 false 影响；总线/网络/推理也照常运行。
         *   返回 false 的语义 = "控件树没建"，不是"没有显示"。 */
        s_ui_ready = false;
        APP_LOGW("LVGL 界面未建立（仅路径 C 的 OLED 可用）——控件树跳过，数据仍会喂给 OLED");
        return false;
    }

    gui_build_ui();
    return true;
}

/* =============================================================================
 * 页面控制接口（对外，见 gui.h 的详细说明）
 * -----------------------------------------------------------------------------
 * ⚠ 这四个函数**只写原子变量**，绝不调用任何 LVGL API —— 它们可能被
 *   core0 的任务调用，而 LVGL 只在 core1 的 task_gui 里碰。
 * ========================================================================== */
void gui_request_page(int page)
{
    if (page < 0) {
        page = 0;
    }
    if (page >= GUI_PAGE_COUNT) {
        page = GUI_PAGE_COUNT - 1;
    }
    __atomic_store_n(&s_page_want, page, __ATOMIC_RELAXED);
}

void gui_request_page_step(int delta)
{
    /* 基于"当前实际页"做相对移动（而不是 want），这样连按两次 next
     * 能连翻两页，不会因为 want 还没被消费而原地打转。 */
    const int cur = (int)__atomic_load_n(&s_page_cur, __ATOMIC_RELAXED);
    int next = cur + delta;
    while (next < 0) {
        next += GUI_PAGE_COUNT;
    }
    next %= GUI_PAGE_COUNT;
    __atomic_store_n(&s_page_want, next, __ATOMIC_RELAXED);
}

int gui_get_page(void)
{
    return (int)__atomic_load_n(&s_page_cur, __ATOMIC_RELAXED);
}

void gui_set_auto_cycle(bool on)
{
    __atomic_store_n(&s_auto_cycle, on, __ATOMIC_RELAXED);
}

bool gui_get_auto_cycle(void)
{
    return __atomic_load_n(&s_auto_cycle, __ATOMIC_RELAXED);
}

uint32_t gui_debug_since_page_ms(void)
{
    return millis() - s_page_last_ms;
}

int gui_debug_want(void)
{
    return (int)__atomic_load_n(&s_page_want, __ATOMIC_RELAXED);
}

void gui_tick(void)
{
    /* 1) LVGL 内部定时器（动画、重绘、输入扫描）。
     *    若走 esp_lvgl_port（路径 A），这里必须换成
     *    if (lvgl_port_lock(0)) { lv_timer_handler(); ...; lvgl_port_unlock(); } */
    if (s_ui_ready) {
        lv_timer_handler();
    }

    /* 2) 排空 UI 队列（非阻塞）。即使界面还没起来也要排空，
     *    否则队列满了会让 modbus 任务那边一直打印"队列满"。
     *
     *    ⚠ 这里**不能**在 switch 之前 `if (!s_ui_ready) continue;`：
     *      UI_MSG_WINDOW 除了画曲线，还要算振动 std 喂给 OLED（路径 C）。
     *      早期版本就是这么写的 → LVGL 屏没落地时，OLED 上的 std 永远是空的。
     *      所以把"s_ui_ready 判断"下放到每个 case 的 gui_update_xxx() 里面
     *      （那些函数自己会 return），而**与显示无关的副作用**在上层执行。 */
    ui_msg_t m;
    while (xQueueReceive(g_ui_queue, &m, 0) == pdTRUE) {
        switch (m.kind) {
            case UI_MSG_ENV:
                /* 慢车道快照：LVGL 卡片 + OLED 都从快照取，不需要额外动作
                 * （OLED 自己调 slow_get_latest）。 */
                gui_update_env(&m.u.env);
                break;

            case UI_MSG_WINDOW:
                /* 波形页重绘（LVGL）+ 振动 std 计算（OLED） */
                gui_update_wave(m.u.win);
                break;

            case UI_MSG_INFER:
                if (m.u.infer.valid) {
                    /* 每来一条有效结果打一行：这是"模型真的在跑"的唯一串口证据。
                     * 特征通道 2s 一次，不会刷屏。 */
                    APP_LOGI("推理：%s %.1f%%（%u us, seq=%u）",
                             infer_label(m.u.infer.best),
                             (double)(m.u.infer.probs[m.u.infer.best] * 100.0f),
                             (unsigned)m.u.infer.infer_us,
                             (unsigned)m.u.infer.win_seq);
                }
                gui_update_infer(&m.u.infer);
                break;

            case UI_MSG_ONLINE:
                /* 必须在这层调：gui_set_online() 内部先喂 OLED，再管 LVGL 控件 */
                gui_set_online(m.u.online != 0);
                break;

            default:
                break;
        }
    }

    /* 3) 诊断页按 1s 刷新（直接取快照，不走队列 —— 理由见 gui.h §更新入口） */
    if (s_ui_ready && (millis() - s_diag_last_ms) >= GUI_DIAG_REFRESH_MS) {
        s_diag_last_ms = millis();
        mb_stats_t st;
        mb_get_stats(&st);
        gui_update_diag(&st);
    }

    /* 4) 页面控制：处理手动请求 + 自动轮播（见 gui.h 的"页面控制"说明）
     *
     * ★ 为什么放在最后、并且只有这里碰 LVGL ★
     *   gui_request_page() 可能被 core0 的 console / task_net 调用，
     *   而 LVGL 不是线程安全的。所以那里**只写原子变量**，
     *   真正的 `lv_tabview_set_act()` 由**这条任务**（core1）执行 → 天然串行。 */
    if (s_ui_ready && s_tabview != NULL) {
        const int want = (int)__atomic_load_n(&s_page_want, __ATOMIC_RELAXED);
        const int cur  = (int)__atomic_load_n(&s_page_cur,  __ATOMIC_RELAXED);

        if (want != cur) {
            /* 手动请求：切过去，并**重置轮播计时**（否则刚切完就被轮播切走） */
            lv_tabview_set_act(s_tabview, (uint16_t)want, LV_ANIM_ON);
            __atomic_store_n(&s_page_cur,  want, __ATOMIC_RELAXED);
            /* ★ 必须把 want 也同步成同一个值 ★
             * 否则下一拍 `want != cur` 又成立，会被误判成"手动请求"，
             * 立刻切回 want —— 表现是**自动轮播永远停在那一页**
             * （2026-10 实测踩到：每 10 秒都打"切到页面 0"，永远回数据页）。 */
            __atomic_store_n(&s_page_want, want, __ATOMIC_RELAXED);
            s_page_last_ms = millis();
            APP_LOGI("[gui] 手动切到页面 %d（%s）", want, page_name(want));
        } else if (__atomic_load_n(&s_auto_cycle, __ATOMIC_RELAXED) &&
                   (millis() - s_page_last_ms) >= GUI_PAGE_CYCLE_MS) {
            /* 自动轮播：下一页（循环）。
             * ⚠ want 和 cur 要**一起**更新，理由见上面的注释。 */
            const int next = (cur + 1) % GUI_PAGE_COUNT;
            lv_tabview_set_act(s_tabview, (uint16_t)next, LV_ANIM_ON);
            __atomic_store_n(&s_page_cur,  next, __ATOMIC_RELAXED);
            __atomic_store_n(&s_page_want, next, __ATOMIC_RELAXED);
            s_page_last_ms = millis();
            APP_LOGI("[gui] 轮播到页面 %d（%s）", next, page_name(next));
        }
    }

    /* 5) 刷屏统计：**排障用的，平时关掉**（每 3 秒一条会刷满串口日志）。
     * 需要排查"背光亮点不亮/界面不动"时把 TFT_FLUSH_STATS 改成 1：
     *   flush 一直在涨 → LVGL 在刷、数据推给了 SPI → 问题在硬件/线序/驱动 IC
     *   flush 一直为 0 → LVGL 这边没动静 → 问题在定时器/控件树（不是硬件） */
#define TFT_FLUSH_STATS 0
#if TFT_ENABLE && TFT_FLUSH_STATS
    {
        static uint32_t last_rep = 0;
        if (s_ui_ready && (millis() - last_rep) >= 3000u) {
            last_rep = millis();
            APP_LOGI("[tft] LVGL 刷屏统计：flush 调用 %u 次，累计 %u 像素",
                     (unsigned)s_flush_calls, (unsigned)s_flush_px);
        }
    }
#endif
}

void task_gui(void *arg)
{
    (void)arg;
    if (!gui_init()) {
        APP_LOGW("GUI 未启动（LVGL 显示移植路径待补，见 gui.cpp 的 TODO）——"
                 "先靠串口日志 + OLED 验证总线与推理");
    }

    TickType_t last_wake = xTaskGetTickCount();
    app_wdt_register();
    for (;;) {
        app_wdt_feed();
        gui_tick();
        /* 固定 10ms 节拍：LVGL 要求 lv_timer_handler 每 5~30ms 被调一次 */
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(GUI_TICK_PERIOD_MS));
    }
}
