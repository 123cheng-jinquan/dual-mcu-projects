/* =============================================================================
 *  oled.cpp — SSD1306 128×64 I2C 驱动（手写，不引第三方库）
 * =============================================================================
 *  为什么手写而不是装 U8g2/Adafruit_SSD1306：
 *    · U8g2 带一堆无关字库，Flash 占用大；Adafruit 库默认带 1KB 全屏 RAM 缓冲
 *      且强制它自己的 GFX 抽象；
 *    · 这个驱动总共 ~450 行，没有依赖，编译时间和 Flash 占用都可控，
 *      而且"SSD1306 初始化序列 + I2C 分块写"这两件事在面试里是能讲清楚的点。
 *
 *  硬件：SSD1306 / SSD1315 0.96" 128×64 单色 OLED，I2C 7 位地址 0x3C
 *        VCC=3V3 GND=GND SDA=GPIO8 SCL=GPIO9（见 config.h §4.1）
 *
 *  I2C 帧格式（SSD1306 datasheet §8.1.1）：
 *    控制字节 | 数据...
 *      0x00 = 后面全是命令
 *      0x40 = 后面全是显存数据
 *    所以一次"写命令"= Wire.beginTransmission(addr); write(0x00); write(cmd...); endTransmission();
 *
 *  显存布局：128×64 = 8 页（page），每页 128 列、每列 8 个像素（bit0 在最上）。
 *    → buffer[page * 128 + x] = 该列这一页的 8 个像素
 *  一次全屏刷新 = 8 × (1 控制字节 + 128 数据字节) ≈ 1032 字节 → 400kHz 下约 25ms。
 * =============================================================================
 */

#include "oled.h"

#include <Wire.h>
#include <string.h>

#include "infer.h"

/* =============================================================================
 * §1  SSD1306 命令字（datasheet Table 9-1，只列用到的）
 * ============================================================================= */
#define SSD_CMD_DISPLAY_OFF         0xAE
#define SSD_CMD_DISPLAY_ON          0xAF
#define SSD_CMD_SET_CLK_DIV         0xD5    /* 后面跟 [freq<<4 | div] */
#define SSD_CMD_SET_MULTIPLEX       0xA8    /* 后面跟 [rows-1] */
#define SSD_CMD_SET_DISPLAY_OFFSET  0xD3    /* 后面跟 [offset] */
#define SSD_CMD_SET_START_LINE      0x40
#define SSD_CMD_SET_CHARGE_PUMP     0x8D    /* 后面跟 0x14 = 内部电荷泵开（3V3 供电必须开） */
#define SSD_CMD_SET_MEM_MODE        0x20    /* 后面跟 0x00 = 水平寻址 */
#define SSD_CMD_SET_SEG_REMAP       0xA1    /* 列地址 127→SEG0（左右镜像，和 0xC8 一起决定方向） */
#define SSD_CMD_SET_COM_SCAN_DEC    0xC8    /* 上下翻转 */
#define SSD_CMD_SET_COM_PINS        0xDA    /* 后面跟 0x12 = 交替行 */
#define SSD_CMD_SET_CONTRAST        0x81    /* 后面跟 [0~255] */
#define SSD_CMD_SET_PRECHARGE       0xD9    /* 后面跟 0xF1 */
#define SSD_CMD_SET_VCOM_DETECT     0xDB    /* 后面跟 0x40 */
#define SSD_CMD_DISPLAY_ALL_ON_RES  0xA4    /* 显示跟随显存（不是全部点亮） */
#define SSD_CMD_NORMAL_DISPLAY      0xA6    /* 不是反色 */
#define SSD_CMD_DEACTIVATE_SCROLL   0x2E

/* =============================================================================
 * §2  6×8 点阵字库（字符 0x20~0x7F，可打印 ASCII）
 * -----------------------------------------------------------------------------
 *  每个字符 5 字节，第 i 字节的 bit0~bit6 = 该列从上到下的 7 个像素（bit7 恒 0）。
 *  渲染时按 bit 展开成显存的 8 像素列 → 刚好一行 8 像素 = 1 页。
 *
 *  来源：Adafruit GLCD / 经典 5×7 ASCII 点阵表（public domain，出处已不可考，
 *        是 1980 年代起所有点阵屏共用的那张表）。
 *  ⚠ 5 列 + 1 列间距 = 6 像素宽 → OLED_TEXT_COLS = 128/6 = 21 列。
 * ============================================================================= */
static const uint8_t kFont5x7[96][5] = {
    { 0x00, 0x00, 0x00, 0x00, 0x00 }, /* 0x20 ' ' */
    { 0x00, 0x00, 0x5F, 0x00, 0x00 }, /* ! */
    { 0x00, 0x07, 0x00, 0x07, 0x00 }, /* " */
    { 0x14, 0x7F, 0x14, 0x7F, 0x14 }, /* # */
    { 0x24, 0x2A, 0x7F, 0x2A, 0x12 }, /* $ */
    { 0x23, 0x13, 0x08, 0x64, 0x62 }, /* % */
    { 0x36, 0x49, 0x55, 0x22, 0x50 }, /* & */
    { 0x00, 0x05, 0x03, 0x00, 0x00 }, /* ' */
    { 0x00, 0x1C, 0x22, 0x41, 0x00 }, /* ( */
    { 0x00, 0x41, 0x22, 0x1C, 0x00 }, /* ) */
    { 0x14, 0x08, 0x3E, 0x08, 0x14 }, /* * */
    { 0x08, 0x08, 0x3E, 0x08, 0x08 }, /* + */
    { 0x00, 0x50, 0x30, 0x00, 0x00 }, /* , */
    { 0x08, 0x08, 0x08, 0x08, 0x08 }, /* - */
    { 0x00, 0x60, 0x60, 0x00, 0x00 }, /* . */
    { 0x20, 0x10, 0x08, 0x04, 0x02 }, /* / */
    { 0x3E, 0x51, 0x49, 0x45, 0x3E }, /* 0 */
    { 0x00, 0x42, 0x7F, 0x40, 0x00 }, /* 1 */
    { 0x42, 0x61, 0x51, 0x49, 0x46 }, /* 2 */
    { 0x21, 0x41, 0x45, 0x4B, 0x31 }, /* 3 */
    { 0x18, 0x14, 0x12, 0x7F, 0x10 }, /* 4 */
    { 0x27, 0x45, 0x45, 0x45, 0x39 }, /* 5 */
    { 0x3C, 0x4A, 0x49, 0x49, 0x30 }, /* 6 */
    { 0x01, 0x71, 0x09, 0x05, 0x03 }, /* 7 */
    { 0x36, 0x49, 0x49, 0x49, 0x36 }, /* 8 */
    { 0x06, 0x49, 0x49, 0x29, 0x1E }, /* 9 */
    { 0x00, 0x36, 0x36, 0x00, 0x00 }, /* : */
    { 0x00, 0x56, 0x36, 0x00, 0x00 }, /* ; */
    { 0x08, 0x14, 0x22, 0x41, 0x00 }, /* < */
    { 0x14, 0x14, 0x14, 0x14, 0x14 }, /* = */
    { 0x00, 0x41, 0x22, 0x14, 0x08 }, /* > */
    { 0x02, 0x01, 0x51, 0x09, 0x06 }, /* ? */
    { 0x32, 0x49, 0x79, 0x41, 0x3E }, /* @ */
    { 0x7E, 0x11, 0x11, 0x11, 0x7E }, /* A */
    { 0x7F, 0x49, 0x49, 0x49, 0x36 }, /* B */
    { 0x3E, 0x41, 0x41, 0x41, 0x22 }, /* C */
    { 0x7F, 0x41, 0x41, 0x22, 0x1C }, /* D */
    { 0x7F, 0x49, 0x49, 0x49, 0x41 }, /* E */
    { 0x7F, 0x09, 0x09, 0x09, 0x01 }, /* F */
    { 0x3E, 0x41, 0x49, 0x49, 0x7A }, /* G */
    { 0x7F, 0x08, 0x08, 0x08, 0x7F }, /* H */
    { 0x00, 0x41, 0x7F, 0x41, 0x00 }, /* I */
    { 0x20, 0x40, 0x41, 0x3F, 0x01 }, /* J */
    { 0x7F, 0x08, 0x14, 0x22, 0x41 }, /* K */
    { 0x7F, 0x40, 0x40, 0x40, 0x40 }, /* L */
    { 0x7F, 0x02, 0x0C, 0x02, 0x7F }, /* M */
    { 0x7F, 0x04, 0x08, 0x10, 0x7F }, /* N */
    { 0x3E, 0x41, 0x41, 0x41, 0x3E }, /* O */
    { 0x7F, 0x09, 0x09, 0x09, 0x06 }, /* P */
    { 0x3E, 0x41, 0x51, 0x21, 0x5E }, /* Q */
    { 0x7F, 0x09, 0x19, 0x29, 0x46 }, /* R */
    { 0x46, 0x49, 0x49, 0x49, 0x31 }, /* S */
    { 0x01, 0x01, 0x7F, 0x01, 0x01 }, /* T */
    { 0x3F, 0x40, 0x40, 0x40, 0x3F }, /* U */
    { 0x1F, 0x20, 0x40, 0x20, 0x1F }, /* V */
    { 0x7F, 0x20, 0x18, 0x20, 0x7F }, /* W */
    { 0x63, 0x14, 0x08, 0x14, 0x63 }, /* X */
    { 0x03, 0x04, 0x78, 0x04, 0x03 }, /* Y */
    { 0x61, 0x51, 0x49, 0x45, 0x43 }, /* Z */
    { 0x00, 0x7F, 0x41, 0x41, 0x00 }, /* [ */
    { 0x02, 0x04, 0x08, 0x10, 0x20 }, /* \ */
    { 0x00, 0x41, 0x41, 0x7F, 0x00 }, /* ] */
    { 0x04, 0x02, 0x01, 0x02, 0x04 }, /* ^ */
    { 0x40, 0x40, 0x40, 0x40, 0x40 }, /* _ */
    { 0x00, 0x01, 0x02, 0x04, 0x00 }, /* ` */
    { 0x20, 0x54, 0x54, 0x54, 0x78 }, /* a */
    { 0x7F, 0x48, 0x44, 0x44, 0x38 }, /* b */
    { 0x38, 0x44, 0x44, 0x44, 0x20 }, /* c */
    { 0x38, 0x44, 0x44, 0x48, 0x7F }, /* d */
    { 0x38, 0x54, 0x54, 0x54, 0x18 }, /* e */
    { 0x08, 0x7E, 0x09, 0x01, 0x02 }, /* f */
    { 0x0C, 0x52, 0x52, 0x52, 0x3E }, /* g */
    { 0x7F, 0x08, 0x04, 0x04, 0x78 }, /* h */
    { 0x00, 0x44, 0x7D, 0x40, 0x00 }, /* i */
    { 0x20, 0x40, 0x44, 0x3D, 0x00 }, /* j */
    { 0x7F, 0x10, 0x28, 0x44, 0x00 }, /* k */
    { 0x00, 0x41, 0x7F, 0x40, 0x00 }, /* l */
    { 0x7C, 0x04, 0x18, 0x04, 0x78 }, /* m */
    { 0x7C, 0x08, 0x04, 0x04, 0x78 }, /* n */
    { 0x38, 0x44, 0x44, 0x44, 0x38 }, /* o */
    { 0x7C, 0x14, 0x14, 0x14, 0x08 }, /* p */
    { 0x08, 0x14, 0x14, 0x18, 0x7C }, /* q */
    { 0x7C, 0x08, 0x04, 0x04, 0x08 }, /* r */
    { 0x48, 0x54, 0x54, 0x54, 0x20 }, /* s */
    { 0x04, 0x3F, 0x44, 0x40, 0x20 }, /* t */
    { 0x3C, 0x40, 0x40, 0x20, 0x7C }, /* u */
    { 0x1C, 0x20, 0x40, 0x20, 0x1C }, /* v */
    { 0x3C, 0x40, 0x30, 0x40, 0x3C }, /* w */
    { 0x44, 0x28, 0x10, 0x28, 0x44 }, /* x */
    { 0x0C, 0x50, 0x50, 0x50, 0x3C }, /* y */
    { 0x44, 0x64, 0x54, 0x4C, 0x44 }, /* z */
    { 0x00, 0x08, 0x36, 0x41, 0x00 }, /* { */
    { 0x00, 0x00, 0x7F, 0x00, 0x00 }, /* | */
    { 0x00, 0x41, 0x36, 0x08, 0x00 }, /* } */
    { 0x08, 0x08, 0x2A, 0x1C, 0x08 }, /* ~ */
    { 0x08, 0x1C, 0x2A, 0x08, 0x08 }, /* 0x7F（DEL 位，画成箭头，不会用到） */
};

/* =============================================================================
 * §3  模块状态
 * ============================================================================= */
static uint8_t           s_buf[OLED_BUF_SIZE];      /* 显存（1KB，内部 RAM） */
static bool              s_ready        = false;    /* 屏已初始化且能写 */
static uint32_t          s_last_try_ms  = 0;
static volatile uint32_t s_err_count    = 0;

/* 写侧数据（被其他任务改，临界区保护） */
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static bool     s_online   = false;
static uint16_t s_vib_std  = 0;
static uint16_t s_vib_pp   = 0;
static uint16_t s_vib_seq  = 0;
static bool     s_vib_valid = false;

/* =============================================================================
 * §4  低层 I2C / SSD1306
 * ============================================================================= */

/* 写命令序列（第一个字节是控制字节 0x00 = 命令流） */
static bool ssd_cmd(const uint8_t *cmds, size_t n)
{
    Wire.beginTransmission(OLED_I2C_ADDR);
    Wire.write((uint8_t)0x00);
    for (size_t i = 0; i < n; i++) {
        Wire.write(cmds[i]);
    }
    return Wire.endTransmission() == 0;
}

static bool ssd_cmd1(uint8_t c)
{
    return ssd_cmd(&c, 1);
}

/* 把整个显存写进 SSD1306。
 * 分块理由：Arduino 的 TwoWire 默认有一个 I2C 发送缓冲（ESP32 上是 128 字节
 * 的动态缓冲），一次 write 1032 字节会失败/被截断 —— 典型症状是"只刷新了第一页"。
 * 所以按 OLED_WRITE_CHUNK 分块，每块都自带控制字节 0x40（数据流）。 */
static bool ssd_flush(void)
{
    const uint8_t ctrl = 0x40;
    size_t off = 0;
    while (off < OLED_BUF_SIZE) {
        const size_t n = (OLED_BUF_SIZE - off < OLED_WRITE_CHUNK)
                       ? (OLED_BUF_SIZE - off) : (size_t)OLED_WRITE_CHUNK;
        Wire.beginTransmission(OLED_I2C_ADDR);
        Wire.write(ctrl);
        Wire.write(&s_buf[off], (uint8_t)n);
        if (Wire.endTransmission() != 0) {
            return false;
        }
        off += n;
    }
    return true;
}

/* 设置绘制窗口为整屏（水平寻址模式）：页 0~7、列 0~127 */
static bool ssd_window_full(void)
{
    const uint8_t seq[] = {
        0x21, 0x00, (uint8_t)(OLED_HOR_RES - 1),        /* 列地址 0~127 */
        0x22, 0x00, (uint8_t)(OLED_PAGES - 1),          /* 页地址 0~7   */
    };
    return ssd_cmd(seq, sizeof(seq));
}

/* SSD1306 上电初始化序列（照 datasheet 的推荐值 + 0.96" 模块的通用配置） */
static bool ssd_init_sequence(void)
{
    /* 先软复位：屏幕可能上次被留在"显示关/睡眠"状态，直接下发初始化有时不生效 */
    uint8_t cmds[32];
    size_t  n = 0;
    cmds[n++] = SSD_CMD_DISPLAY_OFF;                    /* 初始化期间先关显示，避免花屏 */
    cmds[n++] = SSD_CMD_SET_CLK_DIV;      cmds[n++] = 0x80;
    cmds[n++] = SSD_CMD_SET_MULTIPLEX;    cmds[n++] = (uint8_t)(OLED_VER_RES - 1);
    cmds[n++] = SSD_CMD_SET_DISPLAY_OFFSET; cmds[n++] = 0x00;
    cmds[n++] = SSD_CMD_SET_START_LINE | 0x00;
    cmds[n++] = SSD_CMD_SET_CHARGE_PUMP;  cmds[n++] = 0x14;  /* 3V3 供电必须开内部电荷泵 */
    cmds[n++] = SSD_CMD_SET_MEM_MODE;     cmds[n++] = 0x00;  /* 水平寻址 */
    cmds[n++] = SSD_CMD_SET_SEG_REMAP;                       /* 列方向 */
    cmds[n++] = SSD_CMD_SET_COM_SCAN_DEC;                    /* 行方向 */
    cmds[n++] = SSD_CMD_SET_COM_PINS;     cmds[n++] = 0x12;
    cmds[n++] = SSD_CMD_SET_CONTRAST;     cmds[n++] = 0x7F;
    cmds[n++] = SSD_CMD_SET_PRECHARGE;    cmds[n++] = 0xF1;
    cmds[n++] = SSD_CMD_SET_VCOM_DETECT;  cmds[n++] = 0x40;
    cmds[n++] = SSD_CMD_DISPLAY_ALL_ON_RES;
    cmds[n++] = SSD_CMD_NORMAL_DISPLAY;
    cmds[n++] = SSD_CMD_DEACTIVATE_SCROLL;
    if (!ssd_cmd(cmds, n)) {
        return false;
    }
    if (!ssd_window_full()) {
        return false;
    }
    /* 全屏清零后再开显示 */
    memset(s_buf, 0, sizeof(s_buf));
    if (!ssd_flush()) {
        return false;
    }
    return ssd_cmd1(SSD_CMD_DISPLAY_ON);
}

/* =============================================================================
 * §5  绘制（全部只改 s_buf，不碰 I2C）
 * ============================================================================= */

static void draw_clear(void)
{
    memset(s_buf, 0, sizeof(s_buf));
}

/* 在 (col,page) 画一个字符。col 单位 = 字符（6 像素），page = 行（8 像素）。
 * size > 1 时按点放大（每个像素画成 size×size 的方块）。 */
static void draw_char(uint8_t col, uint8_t page, char ch, uint8_t size)
{
    if (size == 0) {
        size = 1;
    }
    const uint8_t w = (uint8_t)(6 * size);
    const uint8_t h = (uint8_t)(8 * size);
    const int x0 = (int)col * w;
    const int y0 = (int)page * h;
    if (x0 >= OLED_HOR_RES || y0 >= OLED_VER_RES) {
        return;
    }

    uint8_t c = (uint8_t)ch;
    if (c < 0x20 || c > 0x7F) {
        c = '?';                    /* 非 ASCII（比如中文）统一显示成 '?' —— 这个字体只有 ASCII */
    }
    const uint8_t *g = kFont5x7[c - 0x20];

    for (int cx = 0; cx < 5; cx++) {
        const uint8_t colbits = g[cx];
        for (int cy = 0; cy < 7; cy++) {
            if ((colbits & (1u << cy)) == 0u) {
                continue;
            }
            for (int dy = 0; dy < size; dy++) {
                for (int dx = 0; dx < size; dx++) {
                    const int px = x0 + cx * size + dx;
                    const int py = y0 + cy * size + dy;
                    if (px < 0 || px >= OLED_HOR_RES || py < 0 || py >= OLED_VER_RES) {
                        continue;
                    }
                    s_buf[(py / 8) * OLED_HOR_RES + px] |= (uint8_t)(1u << (py % 8));
                }
            }
        }
    }
}

/* 画一行文本（自动裁剪到屏宽，避免越界写） */
static void draw_text(uint8_t col, uint8_t page, const char *s, uint8_t size)
{
    if (s == NULL) {
        return;
    }
    if (size == 0) {
        size = 1;
    }
    const uint8_t cw = (uint8_t)(6 * size);
    uint8_t c = col;
    for (; *s != '\0'; s++) {
        if ((int)(c * cw) + cw > OLED_HOR_RES) {
            break;
        }
        draw_char(c, page, *s, size);
        c++;
    }
}

/* =============================================================================
 * §6  初始化 / 重试
 * ============================================================================= */
bool oled_init(void)
{
#if !OLED_ENABLE
    APP_LOGW("OLED 已在 config.h 里关闭（OLED_ENABLE=0）");
    return false;
#else
    if (s_ready) {
        return true;
    }
    /* I2C 只初始化一次；失败后重试时不要反复 Wire.begin()（会把总线状态搅乱） */
    static bool wire_begun = false;
    if (!wire_begun) {
        /* 400kHz 快速模式。GPIO8/9 是 ESP32-S3-DevKitC-1 的空闲脚
         * （避开 19/20 原生 USB 与 33~37 Octal PSRAM，见 config.h §4.1）。 */
        const bool ok = Wire.begin(OLED_PIN_SDA, OLED_PIN_SCL, OLED_I2C_CLOCK_HZ);
        Wire.setTimeOut(OLED_I2C_TIMEOUT_MS);
        if (!ok) {
            APP_LOGE("OLED: I2C 初始化失败（SDA=%d SCL=%d），本地显示不可用",
                     OLED_PIN_SDA, OLED_PIN_SCL);
            return false;
        }
        wire_begun = true;
    }

    /* 探测器件：能 ACK 说明屏在线（地址错/没插/线接反都会在这里失败） */
    Wire.beginTransmission(OLED_I2C_ADDR);
    if (Wire.endTransmission() != 0) {
        s_ready = false;
        s_last_try_ms = millis();
        return false;
    }

    if (!ssd_init_sequence()) {
        s_ready = false;
        s_last_try_ms = millis();
        s_err_count++;
        return false;
    }

    s_ready = true;
    s_last_try_ms = millis();
    APP_LOGI("OLED 就绪：SSD1306 %dx%d @0x%02X  SDA=%d SCL=%d  %dkHz",
             OLED_HOR_RES, OLED_VER_RES, OLED_I2C_ADDR,
             OLED_PIN_SDA, OLED_PIN_SCL, OLED_I2C_CLOCK_HZ / 1000);
    return true;
#endif
}

/* =============================================================================
 * §7  数据接口
 * ============================================================================= */
void oled_set_online(bool online)
{
    portENTER_CRITICAL(&s_mux);
    s_online = online;
    portEXIT_CRITICAL(&s_mux);
}

void oled_on_online_event(bool online)
{
    oled_set_online(online);
}

void oled_set_vibration(uint16_t std_raw, uint16_t pp_raw, uint16_t seq)
{
    portENTER_CRITICAL(&s_mux);
    s_vib_std   = std_raw;
    s_vib_pp    = pp_raw;
    s_vib_seq   = seq;
    s_vib_valid = true;
    portEXIT_CRITICAL(&s_mux);
}

uint32_t oled_error_count(void)
{
    return s_err_count;
}

/* =============================================================================
 * §8  画面组装
 * -----------------------------------------------------------------------------
 *  布局（8 行 × 21 列，行 1 用 2 倍字号当标题）：
 *    row0  OLED GW  (2x)          —— 标题，兼作"这块屏还活着"的心跳
 *    row2  CLS  : normal 87%      —— 分类结果（TFLite Micro 推理）
 *    row3  VIB  : std 42 pp 180   —— 振动 std / 峰峰值（加速度三轴最大值）
 *    row4  SEQ  : 1234            —— 窗口序号
 *    row5  485  : ONLINE          —— 485 在线状态（连续 3 次超时 → OFFLINE）
 *    row6  TX/RX/TO/CRC           —— 总线统计
 *    row7  ENV  : 25.3C 60%       —— 温度/湿度（慢车道快照，顺带确认数据在更新）
 * ============================================================================= */

/* 一条 line 的"是否新鲜"判断：stale 时在行尾打 ~，避免把旧值当实时值看 */
#define FRESH_MS(now, t)   (((now) - (t)) <= (uint32_t)OLED_STALE_MS)

static void render(const mb_stats_t *st, bool have_snap, const slow_regs_t *snap,
                   bool infer_ok, const infer_result_t *ir,
                   bool vib_valid, uint16_t vib_std, uint16_t vib_pp, uint16_t vib_seq)
{
    char line[OLED_TEXT_COLS + 1];
    const uint32_t now = millis();

    draw_clear();

    /* --- 标题（2 倍字号，只有 10 个字符宽度：128/12 = 10） --- */
    draw_text(0, 0, "GW-485", 2);

    /* --- 分类结果 --- */
    if (infer_ok && ir->valid) {
        const char *lbl = infer_label(ir->best);
        const float conf = ir->probs[ir->best] * 100.0f;
        snprintf(line, sizeof(line), "CLS:%-6.6s %3d%%%s",
                 lbl, (int)(conf + 0.5f),
                 FRESH_MS(now, ir->tick) ? "" : " ~");
    } else if (infer_ok) {
        snprintf(line, sizeof(line), "CLS: no result");
    } else {
        snprintf(line, sizeof(line), "CLS: no model");
    }
    draw_text(0, 2, line, 1);

    /* --- 振动 std / 峰峰值 --- */
    if (vib_valid) {
        snprintf(line, sizeof(line), "VIB std%5u pp%5u", (unsigned)vib_std, (unsigned)vib_pp);
    } else {
        snprintf(line, sizeof(line), "VIB std    - pp    -");
    }
    draw_text(0, 3, line, 1);

    /* --- 窗口序号 --- */
    snprintf(line, sizeof(line), "SEQ win%5u", (unsigned)vib_seq);
    draw_text(0, 4, line, 1);

    /* --- 485 在线状态（本屏最要紧的一行：它挂了后面几行就没意义了） ---
     * ⚠ 排版限制：一行只有 OLED_TEXT_COLS = 21 个字符格。
     *   "485:" (4) + 状态(7，用 %-7s) + " to=" (4) + 数字(2) = 17 ← 安全。
     *   早期写成 "485: %-8s to=%u" 是 4+1+8+4+2 = 19 看着够，但 consecutive_timeout
     *   到 3 位时就会挤到 20 再往上顶到 21 边界，某些字体参数下会被裁掉最后一位。 */
    snprintf(line, sizeof(line), "485:%-7s to=%u",
             st->online ? "ONLINE" : "OFFLINE", (unsigned)st->consecutive_timeout);
    draw_text(0, 5, line, 1);

    /* --- 总线统计 --- */
    snprintf(line, sizeof(line), "TX%5u RX%5u", (unsigned)st->tx_count, (unsigned)st->rx_count);
    draw_text(0, 6, line, 1);

    /* --- 环境量（顺带证明慢车道数据在更新；没有快照就是还没读到过一次） --- */
    if (have_snap) {
        const int   t = (int)slow_i16(snap, REG_TEMP);
        const float temp = (float)t / 10.0f;
        snprintf(line, sizeof(line), "ENV %5.1fC %4u%%",
                 (double)temp, (unsigned)slow_u16(snap, REG_HUMI) / 10u);
    } else {
        snprintf(line, sizeof(line), "ENV  no data yet");
    }
    draw_text(0, 7, line, 1);
}

/* =============================================================================
 * §9  任务：周期刷屏 + 失败重试
 * ============================================================================= */
void task_oled(void *arg)
{
    (void)arg;
    app_wdt_register();
    APP_LOGI("task_oled 已启动：%dms 刷一次 SSD1306（读共享数据，不碰总线）",
             OLED_REFRESH_PERIOD_MS);

    TickType_t last_wake = xTaskGetTickCount();
    uint32_t   frames    = 0;

    for (;;) {
        app_wdt_feed();

#if OLED_ENABLE
        if (!s_ready) {
            /* 没插屏 / 初始化失败：周期性重试（支持运行时插上）。
             * 重试失败会打一次日志（只打第一次 + 每 6 次一次，避免刷屏）。 */
            const uint32_t now = millis();
            if ((now - s_last_try_ms) >= OLED_RETRY_PERIOD_MS) {
                static uint16_t retry_n = 0;
                retry_n++;
                if (oled_init()) {
                    APP_LOGI("OLED 上线（第 %u 次尝试）", (unsigned)retry_n);
                } else if (retry_n == 1 || (retry_n % 6u) == 0u) {
                    APP_LOGW("OLED 未就绪（第 %u 次）：检查 VCC=3V3 / GND / SDA=GPIO%d / SCL=GPIO%d / 地址 0x%02X",
                             (unsigned)retry_n, OLED_PIN_SDA, OLED_PIN_SCL, OLED_I2C_ADDR);
                }
            }
        } else {
            /* ---- 取共享数据（全部是"拷贝一份"，不持锁渲染） ---- */
            mb_stats_t st;
            mb_get_stats(&st);

            slow_regs_t snap;
            const bool have_snap = slow_get_latest(&snap);

            /* 先清零再取：infer_get_latest() 返回 false 时不保证填过 ir，
             * 而下面 render() 的签名是 const 引用式传参 —— 留个干净的值，
             * 免得将来有人在 render() 里提前读了它。 */
            infer_result_t ir;
            memset(&ir, 0, sizeof(ir));
            const bool infer_ok = infer_get_latest(&ir);

            uint16_t vstd = 0, vpp = 0, vseq = 0;
            bool vib_valid = false;
            portENTER_CRITICAL(&s_mux);
            vstd = s_vib_std;
            vpp  = s_vib_pp;
            vseq = s_vib_seq;
            vib_valid = s_vib_valid;
            portEXIT_CRITICAL(&s_mux);

            render(&st, have_snap, &snap, infer_ok, &ir, vib_valid, vstd, vpp, vseq);

            if (!ssd_flush()) {
                s_err_count++;
                s_ready = false;        /* 掉线/被扯掉：下次循环重试初始化 */
                s_last_try_ms = millis();
                APP_LOGW("OLED 写入失败（第 %u 次）：屏可能被拔掉或 I2C 被干扰，转入重试",
                         (unsigned)s_err_count);
            } else if ((++frames % 40u) == 0u) {
                /* 每 20 秒一条：证明本地显示这条链路还活着（不影响总线） */
                APP_LOGI("OLED 刷新正常：%u 帧，错误 %u 次，485 %s",
                         (unsigned)frames, (unsigned)s_err_count,
                         st.online ? "在线" : "离线");
            }
        }
#endif  /* OLED_ENABLE */

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(OLED_REFRESH_PERIOD_MS));
    }
}

bool oled_task_start(void)
{
    const BaseType_t r = xTaskCreatePinnedToCore(task_oled, "oled", TASK_STACK_OLED,
                                                 NULL, TASK_PRIO_OLED, NULL, CORE_BUS);
    if (r != pdPASS) {
        APP_LOGE("OLED 任务创建失败（内部 RAM 不足）");
        return false;
    }
    return true;
}
