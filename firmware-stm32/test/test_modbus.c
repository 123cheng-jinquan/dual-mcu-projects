/**
 * @file    test_modbus.c
 * @brief   手写 Modbus 从机协议栈的 PC 端单元测试（不需要板子、不需要 485）
 *
 * 为什么要在 PC 上测：
 *   协议栈的 bug 大多在边界与异常路径（数量越界、地址越界、CRC 错、广播、部分写入），
 *   这些用 485 + Modbus Poll 手工点很难覆盖全。先把纯逻辑测干净，联调时只剩硬件问题。
 *
 * 编译运行：
 *   Windows(MinGW) : gcc -std=c99 -Wall -Wextra -I../App *.c ../App/modbus_crc.c
 *                    ../App/modbus_slave.c ../App/regbank.c -o test_modbus.exe && test_modbus.exe
 *   或直接运行同目录的 build_and_run.ps1
 */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "../App/modbus_crc.h"
#include "../App/modbus_slave.h"
#include "../App/regbank.h"

/* ---- 测试框架 ---------------------------------------------------------- */
static int g_pass = 0;
static int g_fail = 0;

#define CHECK(cond, msg)                                                       \
    do {                                                                       \
        if (cond) {                                                            \
            g_pass++;                                                          \
            printf("  [PASS] %s\n", (msg));                                    \
        } else {                                                               \
            g_fail++;                                                          \
            printf("  [FAIL] %s   (%s:%d)\n", (msg), __FILE__, __LINE__);      \
        }                                                                      \
    } while (0)

/* ---- 工具 -------------------------------------------------------------- */
static mb_slave_t g_slave;

static void slave_setup(void)
{
    uint8_t n = 0u;

    regbank_init();
    g_slave.slave_addr    = regbank_get_slave_addr();
    g_slave.areas         = regbank_areas(&n);
    g_slave.area_count    = n;
    g_slave.get           = regbank_get;
    g_slave.set           = regbank_set;
    g_slave.check         = regbank_check;
    g_slave.on_block_read = regbank_on_block_read;
    mb_slave_clear_stats();
}

/** 用 PDU（不含 CRC）组一帧，返回帧长 */
static uint16_t frame(uint8_t *buf, const uint8_t *pdu, uint16_t pdu_len)
{
    memcpy(buf, pdu, pdu_len);
    return mb_crc_append(buf, pdu_len, 64u);
}

static uint16_t make_read(uint8_t *buf, uint8_t dev, uint16_t start, uint16_t qty)
{
    uint8_t pdu[6];
    pdu[0] = dev;
    pdu[1] = MB_FC_READ_HOLDING;
    pdu[2] = (uint8_t)(start >> 8);
    pdu[3] = (uint8_t)(start & 0xFFu);
    pdu[4] = (uint8_t)(qty >> 8);
    pdu[5] = (uint8_t)(qty & 0xFFu);
    return frame(buf, pdu, 6u);
}

static uint16_t make_write1(uint8_t *buf, uint8_t dev, uint16_t addr, uint16_t val)
{
    uint8_t pdu[6];
    pdu[0] = dev;
    pdu[1] = MB_FC_WRITE_SINGLE;
    pdu[2] = (uint8_t)(addr >> 8);
    pdu[3] = (uint8_t)(addr & 0xFFu);
    pdu[4] = (uint8_t)(val >> 8);
    pdu[5] = (uint8_t)(val & 0xFFu);
    return frame(buf, pdu, 6u);
}

/* ---- 用例 -------------------------------------------------------------- */

static void test_crc_vectors(void)
{
    /* docs/02-modbus-protocol.md §3.1 的测试向量（CRC 先低字节后高字节） */
    static const struct { uint8_t pdu[6]; uint8_t len; uint16_t crc; } v[] = {
        { { 0x01, 0x03, 0x00, 0x00, 0x00, 0x01 }, 6, 0x0A84u },
        { { 0x01, 0x03, 0x00, 0x00, 0x00, 0x0E }, 6, 0x0EC4u },
        { { 0x01, 0x03, 0x00, 0x00, 0x00, 0x1C }, 6, 0x0344u },
        { { 0x01, 0x03, 0x01, 0x04, 0x00, 0x64 }, 6, 0x1C04u },
        { { 0x01, 0x06, 0x00, 0x20, 0x00, 0x01 }, 6, 0xC049u },
        { { 0x01, 0x06, 0x00, 0x21, 0x03, 0xE8 }, 6, 0x7ED9u },
        { { 0x02, 0x03, 0x00, 0x00, 0x00, 0x02 }, 6, 0x38C4u },
        { { 0x01, 0x03, 0x00, 0x4B, 0x02, 0x00 }, 6, 0xBC34u },
    };
    unsigned i;

    printf("\n[1] CRC16 测试向量\n");
    for (i = 0u; i < sizeof(v) / sizeof(v[0]); i++) {
        uint16_t crc = mb_crc16(v[i].pdu, v[i].len);
        char msg[96];
        sprintf(msg, "CRC(0x%02X 0x%02X ...) = 0x%04X", v[i].pdu[0], v[i].pdu[1], v[i].crc);
        CHECK(crc == v[i].crc, msg);
    }
    {
        /* 整帧残差为 0 的自检技巧 */
        uint8_t f[16];
        uint16_t n = make_read(f, 0x01u, 0x0000u, 0x0001u);
        CHECK(n == 8u, "读请求帧长度为 8");
        CHECK(f[6] == 0x84u && f[7] == 0x0Au, "01 03 00 00 00 01 的 CRC 线上字节序 = 84 0A");
        CHECK(mb_crc_check_frame(f, n), "整帧残差为 0 → CRC 校验通过");
    }
}

static void test_read_normal(void)
{
    uint8_t req[16], resp[300];
    uint16_t n;

    printf("\n[2] 正常读 A 区（0x0000~0x000D，14 个寄存器）\n");
    slave_setup();
    regbank_write_i16(REG_A_TEMP, 253);      /* 25.3 ℃ */
    regbank_write_internal(REG_A_HUMI, 600); /* 60.0 %RH */
    regbank_write_internal(REG_A_LIGHT, 1234);
    regbank_write_internal(REG_A_SEQ, 7);

    n = make_read(req, 0x01u, 0x0000u, 14u);
    n = mb_slave_process(&g_slave, req, n, resp, sizeof(resp));

    CHECK(n == (uint16_t)(3u + 14u * 2u + 2u), "响应长度 = 3 + 28 + 2 = 33");
    CHECK(resp[0] == 0x01u && resp[1] == 0x03u, "响应头 = 01 03");
    CHECK(resp[2] == 28u, "字节数为 28");
    CHECK(((uint16_t)resp[3] << 8 | resp[4]) == 253u, "温度 = 253");
    CHECK(((uint16_t)resp[5] << 8 | resp[6]) == 600u, "湿度 = 600");
    CHECK(((uint16_t)resp[7] << 8 | resp[8]) == 0u, "0x0002 气压未写入 → 0（验证寄存器偏移）");
    CHECK(((uint16_t)resp[9] << 8 | resp[10]) == 1234u, "0x0003 光照 = 1234");
    CHECK(mb_crc_check_frame(resp, n), "响应帧 CRC 正确");
    CHECK(g_mb_stats.ok_frames == 1u, "ok_frames = 1");
}

static void test_read_negative_i16(void)
{
    uint8_t req[16], resp[64];
    uint16_t n;

    printf("\n[3] int16 负值（加速度 -1234）以补码上线\n");
    slave_setup();
    regbank_write_i16(REG_A_TEMP, -1234);
    n = make_read(req, 0x01u, 0x0000u, 1u);
    n = mb_slave_process(&g_slave, req, n, resp, sizeof(resp));
    CHECK(n == 7u, "单寄存器读响应长度 = 3 + 2 + 2 = 7");
    CHECK(((int16_t)((uint16_t)resp[3] << 8 | resp[4])) == -1234, "读回 -1234");
}

static void test_read_bad_quantity(void)
{
    uint8_t req[16], resp[64];
    uint16_t n;

    printf("\n[4] 数量越界（126 > 125）→ 异常码 03\n");
    slave_setup();
    n = make_read(req, 0x01u, 0x0000u, 126u);
    n = mb_slave_process(&g_slave, req, n, resp, sizeof(resp));
    CHECK(n == 5u, "异常响应长度 = 5");
    CHECK(resp[1] == 0x83u, "功能码最高位置 1");
    CHECK(resp[2] == 0x03u, "异常码 = 03 非法数据值");
    CHECK(mb_crc_check_frame(resp, n), "异常帧 CRC 正确");

    printf("\n[4b] 数量 = 0 → 异常码 03\n");
    slave_setup();
    n = make_read(req, 0x01u, 0x0000u, 0u);
    n = mb_slave_process(&g_slave, req, n, resp, sizeof(resp));
    CHECK(n == 5u && resp[2] == 0x03u, "数量 0 被拒绝");
}

static void test_read_bad_address(void)
{
    uint8_t req[16], resp[64];
    uint16_t n;

    printf("\n[5] 地址未映射（0x0360）→ 异常码 02\n");
    slave_setup();
    n = make_read(req, 0x01u, 0x0360u, 1u);
    n = mb_slave_process(&g_slave, req, n, resp, sizeof(resp));
    CHECK(n == 5u && resp[2] == 0x02u, "异常码 = 02 非法数据地址");

    printf("\n[5b] 跨区读（A 区尾 + 未映射空洞）→ 异常码 02\n");
    slave_setup();
    n = make_read(req, 0x01u, 0x000Fu, 4u);   /* 0x000F 有效，0x0010 有效，0x0013 未定义? 实为 B 区 */
    n = mb_slave_process(&g_slave, req, n, resp, sizeof(resp));
    CHECK(n > 0u, "跨相邻区（A→B）允许读取");
    CHECK(n == (uint16_t)(3u + 4u * 2u + 2u), "跨区读返回 4 个寄存器");

    printf("\n[5c] 读窗口头部 0x0100~0x0103（跨 D 头）\n");
    slave_setup();
    n = make_read(req, 0x01u, 0x0100u, 4u);
    n = mb_slave_process(&g_slave, req, n, resp, sizeof(resp));
    CHECK(n == (uint16_t)(3u + 4u * 2u + 2u), "窗口头可读");
    CHECK((((uint16_t)resp[3] << 8) | resp[4]) == 0u, "无窗口时头 = 0x0000");
}

static void test_write_readonly(void)
{
    uint8_t req[16], resp[64];
    uint16_t n;

    printf("\n[6] 写只读区（0x0000）→ 异常码 02\n");
    slave_setup();
    n = make_write1(req, 0x01u, 0x0000u, 100u);
    n = mb_slave_process(&g_slave, req, n, resp, sizeof(resp));
    CHECK(n == 5u && resp[1] == 0x86u && resp[2] == 0x02u, "FC06 写只读区被拒绝");
}

static void test_write_ok_and_invalid_value(void)
{
    uint8_t req[16], resp[64];
    uint16_t n;

    printf("\n[7] 写 C 区：合法值 / 非法值\n");
    slave_setup();
    n = make_write1(req, 0x01u, REG_C_FAN_PWM, 1000u);
    n = mb_slave_process(&g_slave, req, n, resp, sizeof(resp));
    CHECK(n == 8u && memcmp(req, resp, 6u) == 0, "FC06 正常响应 = 原样回显");
    CHECK(regbank_get(REG_C_FAN_PWM) == 1000u, "风扇 PWM 已写入 1000");

    n = make_write1(req, 0x01u, REG_C_FAN_PWM, 1001u);
    n = mb_slave_process(&g_slave, req, n, resp, sizeof(resp));
    CHECK(n == 5u && resp[2] == 0x03u, "PWM 1001 超范围 → 异常 03");

    n = make_write1(req, 0x01u, REG_C_BAUD_CODE, 9u);
    n = mb_slave_process(&g_slave, req, n, resp, sizeof(resp));
    CHECK(n == 5u && resp[2] == 0x03u, "波特率代码 9 → 异常 03");

    n = make_write1(req, 0x01u, REG_C_SLAVE_ADDR, 248u);
    n = mb_slave_process(&g_slave, req, n, resp, sizeof(resp));
    CHECK(n == 5u && resp[2] == 0x03u, "从机地址 248 → 异常 03");
}

static void test_unsupported_fc(void)
{
    uint8_t req[16], resp[64];
    uint8_t pdu[4] = { 0x01u, 0x05u, 0x00u, 0x00u };
    uint16_t n;

    printf("\n[8] 不支持的功能码 0x05 → 异常码 01\n");
    slave_setup();
    n = frame(req, pdu, 4u);
    n = mb_slave_process(&g_slave, req, n, resp, sizeof(resp));
    CHECK(n == 5u && resp[1] == 0x85u && resp[2] == 0x01u, "异常码 = 01 非法功能");
}

static void test_silent_paths(void)
{
    uint8_t req[16], resp[64];
    uint16_t n;

    printf("\n[9] 静默路径：地址不匹配 / CRC 错 / 长度非法 / 广播读\n");
    slave_setup();
    n = make_read(req, 0x02u, 0x0000u, 1u);          /* 地址不匹配 */
    n = mb_slave_process(&g_slave, req, n, resp, sizeof(resp));
    CHECK(n == 0u, "地址 0x02 不匹配 → 不响应");
    CHECK(g_mb_stats.silent_drops == 1u, "silent_drops 计数 +1");

    slave_setup();
    n = make_read(req, 0x01u, 0x0000u, 1u);
    req[n - 1u] ^= 0xFFu;                           /* 破坏 CRC 高字节 */
    n = mb_slave_process(&g_slave, req, n, resp, sizeof(resp));
    CHECK(n == 0u, "CRC 错 → 静默丢弃（不能回异常）");
    CHECK(g_mb_stats.crc_errors == 1u, "crc_errors 计数 +1");

    slave_setup();
    n = mb_slave_process(&g_slave, req, 3u, resp, sizeof(resp));
    CHECK(n == 0u, "长度 < 4 → 静默丢弃");

    slave_setup();
    n = make_read(req, 0x00u, 0x0000u, 1u);          /* 广播读 */
    n = mb_slave_process(&g_slave, req, n, resp, sizeof(resp));
    CHECK(n == 0u, "广播读 → 不响应");
}

static void test_broadcast_write(void)
{
    uint8_t req[16], resp[64];
    uint16_t n;

    printf("\n[10] 广播写：执行但不响应\n");
    slave_setup();
    n = make_write1(req, 0x00u, REG_C_RGB_R, 200u);
    n = mb_slave_process(&g_slave, req, n, resp, sizeof(resp));
    CHECK(n == 0u, "广播写不产生响应");
    CHECK(regbank_get(REG_C_RGB_R) == 200u, "但值已写入");

    printf("\n[10b] 广播写非法值 → 仍然静默\n");
    slave_setup();
    n = make_write1(req, 0x00u, REG_C_FAN_PWM, 5000u);
    n = mb_slave_process(&g_slave, req, n, resp, sizeof(resp));
    CHECK(n == 0u, "广播异常不回异常帧（否则总线冲突）");
}

static void test_write_multi_atomic(void)
{
    uint8_t req[64], resp[64];
    uint8_t pdu[32];
    uint16_t n;

    printf("\n[11] FC10 连写 2 个寄存器（RGB R/G）\n");
    slave_setup();
    pdu[0] = 0x01u; pdu[1] = MB_FC_WRITE_MULTI;
    pdu[2] = 0x00u; pdu[3] = 0x24u;   /* 起始 0x0024 */
    pdu[4] = 0x00u; pdu[5] = 0x02u;   /* 数量 2 */
    pdu[6] = 0x04u;                   /* 字节数 4 */
    pdu[7] = 0x00u; pdu[8] = 0x0Au;   /* R = 10 */
    pdu[9] = 0x00u; pdu[10] = 0x14u;  /* G = 20 */
    n = frame(req, pdu, 11u);
    n = mb_slave_process(&g_slave, req, n, resp, sizeof(resp));
    CHECK(n == 8u, "FC10 响应长度 = 8");
    CHECK(resp[1] == 0x10u && resp[4] == 0x00u && resp[5] == 0x02u, "响应回显起始地址与数量");
    CHECK(regbank_get(REG_C_RGB_R) == 10u && regbank_get(REG_C_RGB_G) == 20u, "两个寄存器都写入");

    printf("\n[11b] FC10 部分非法 → 整块拒绝（不能写一半）\n");
    slave_setup();
    regbank_set(REG_C_FAN_PWM, 100u);
    pdu[2] = 0x00u; pdu[3] = 0x21u;   /* 起始 0x0021（PWM，合法） */
    pdu[4] = 0x00u; pdu[5] = 0x0Cu;   /* 数量 12 → 覆盖到 0x002C 从机地址 */
    pdu[6] = 24u;
    {
        uint8_t data[24];
        uint8_t i;
        memset(data, 0, sizeof(data));
        data[0] = 0x03u; data[1] = 0xE8u;             /* 0x0021 = 1000 合法 */
        /* 0x002B = 安全状态位图（合法）；0x002C 从机地址 = 0 非法 */
        data[22] = 0x00u; data[23] = 0x00u;          /* 0x002C = 0 → 非法 */
        memcpy(&pdu[7], data, 24u);
        (void)i;
    }
    n = frame(req, pdu, 31u);
    n = mb_slave_process(&g_slave, req, n, resp, sizeof(resp));
    CHECK(n == 5u && resp[2] == 0x03u, "整块被拒绝，异常 03");
    CHECK(regbank_get(REG_C_FAN_PWM) == 100u, "前置校验生效：PWM 未被改成 1000（无部分写入）");
}

static void test_window_double_buffer(void)
{
    uint8_t req[16], resp[300];
    int16_t s[6];
    uint16_t n;
    int i;

    printf("\n[12] 六轴窗口：双缓冲 + 序号 + 读取锁定\n");
    slave_setup();
    regbank_tick(0u);

    /* 填满一个窗口：第 i 个样本 ax = i, ay = -i, ... */
    for (i = 0; i < 100; i++) {
        s[0] = (int16_t)i; s[1] = (int16_t)(-i); s[2] = 100;
        s[3] = 200; s[4] = 300; s[5] = 400;
        regbank_push_sample(s);
    }
    CHECK(regbank_window_ready(), "填满 100 样本后窗口就绪");
    CHECK(regbank_window_seq() == 1u, "窗口序号 = 1");

    n = make_read(req, 0x01u, REG_D_HEADER, 1u);
    n = mb_slave_process(&g_slave, req, n, resp, sizeof(resp));
    CHECK(n == 7u && (((uint16_t)resp[3] << 8) | resp[4]) == 0x8064u, "窗口头 = 0x8064（就绪 + 100 样本）");

    n = make_read(req, 0x01u, REG_D_DATA, 12u);
    n = mb_slave_process(&g_slave, req, n, resp, sizeof(resp));
    CHECK(((int16_t)((uint16_t)resp[3] << 8 | resp[4])) == 0, "样本 0 的 ax = 0");
    CHECK(((int16_t)((uint16_t)resp[17] << 8 | resp[18])) == -1, "样本 1 的 ay = -1（行优先：寄存器 6k+j）");

    /* 读一块数据 → 窗口被锁 */
    printf("\n[12b] 读取期间锁定窗口，再写 150 个样本不应改变序号\n");
    n = make_read(req, 0x01u, REG_D_DATA, 100u);
    n = mb_slave_process(&g_slave, req, n, resp, sizeof(resp));
    CHECK(n > 0u && regbank_window_locked(), "读 D 区数据后窗口处于锁定状态");

    for (i = 0; i < 150; i++) {
        s[0] = 9999; s[1] = 9999; s[2] = 9999; s[3] = 9999; s[4] = 9999; s[5] = 9999;
        regbank_push_sample(s);
    }
    CHECK(regbank_window_seq() == 1u, "锁定期间窗口序号不变（不会跨窗口拼接）");
    CHECK(regbank_window_overrun(), "锁定期间样本溢出被标记");

    /* 读 0x0101 收尾校验 → 解锁 */
    printf("\n[12c] 主站读 0x0101 收尾 → 解锁，窗口恢复更新\n");
    n = make_read(req, 0x01u, REG_D_SEQ, 1u);
    n = mb_slave_process(&g_slave, req, n, resp, sizeof(resp));
    CHECK(!regbank_window_locked(), "读到窗口序号后自动解锁");

    for (i = 0; i < 100; i++) {
        s[0] = 7; s[1] = 7; s[2] = 7; s[3] = 7; s[4] = 7; s[5] = 7;
        regbank_push_sample(s);
    }
    CHECK(regbank_window_seq() > 1u, "解锁后窗口继续更新");
    CHECK(!regbank_window_overrun(), "新窗口提交后溢出标志被清除");

    printf("\n[12d] 锁定超时兜底（主站读一半崩了）\n");
    slave_setup();
    regbank_tick(0u);
    for (i = 0; i < 100; i++) { s[0]=1;s[1]=1;s[2]=1;s[3]=1;s[4]=1;s[5]=1; regbank_push_sample(s); }
    n = make_read(req, 0x01u, REG_D_DATA, 10u);
    (void)mb_slave_process(&g_slave, req, n, resp, sizeof(resp));
    CHECK(regbank_window_locked(), "已锁定");
    regbank_tick((uint32_t)(REG_WIN_LOCK_MS + 1u));
    CHECK(!regbank_window_locked(), "超过 500ms 未收尾 → 超时强制解锁（不会永久锁死）");
}

static void test_comm_watchdog(void)
{
    printf("\n[13] 通信看门狗：从没连过主机不报警；连过再掉线才报警\n");
    slave_setup();
    regbank_tick(0u);
    CHECK(!regbank_comm_watchdog_expired(), "从未收到主站请求 → 不进入报警（单板调试不误报）");
    CHECK(regbank_get(REG_B_MODE) == 0u, "未受控时模式 = 0 本地自治");

    regbank_notify_master_request(1000u);
    regbank_tick(1000u);
    CHECK(!regbank_comm_watchdog_expired(), "刚收到请求 → 未超时");
    CHECK(regbank_get(REG_B_MODE) == 1u, "模式切到 1 远程受控");

    regbank_tick(1000u + 30u * 100u + 1u);   /* 默认看门狗 30 × 100ms = 3s */
    CHECK(regbank_comm_watchdog_expired(), "超过 3s 无请求 → 看门狗超时");
    CHECK(regbank_get(REG_B_MODE) == 0u, "掉线 → 退回本地自治");
    CHECK(regbank_get(REG_B_WD_TIMEOUTS) == 1u, "超时次数计数 +1");

    /* 关闭看门狗 */
    regbank_set(REG_C_WD_TIMEOUT, 0u);
    regbank_tick(100000u);
    CHECK(!regbank_comm_watchdog_expired(), "看门狗配置为 0 → 禁用");
}

static void test_stats_and_command(void)
{
    uint8_t req[16], resp[16];
    uint16_t n;

    printf("\n[14] 统计同步与命令寄存器\n");
    slave_setup();
    n = make_read(req, 0x01u, 0x0000u, 1u);
    (void)mb_slave_process(&g_slave, req, n, resp, sizeof(resp));
    n = make_read(req, 0x02u, 0x0000u, 1u);   /* 地址不匹配 → silent */
    (void)mb_slave_process(&g_slave, req, n, resp, sizeof(resp));
    regbank_sync_stats();
    CHECK(regbank_get(REG_B_OK_CNT_LO) == 1u, "B 区正常帧计数 = 1");

    regbank_set(REG_C_FAN_PWM, 500u);
    CHECK(regbank_set(REG_C_COMMAND, 0xA5A5u) == MB_EXC_NONE, "恢复出厂命令被接受");
    CHECK(regbank_get(REG_C_FAN_PWM) == 0u, "恢复出厂后风扇 PWM 归零");

    CHECK(regbank_set(REG_C_COMMAND, 0x1234u) == MB_EXC_VALUE, "非法命令值被拒绝");
}

/**
 * §6 实战帧回归用例。
 *
 * 为什么要有这一组：文档与代码各写一套 CRC 迟早会漂移。本项目真的发生过一次——
 * docs/02 §6 示例② 第 1 行曾误把块地址 0x0100 的 CRC（84 17）抄到 0x0104 那一行，
 * 而 ESP32 侧照抄进 mb_selftest() 的常量后，开机自检直接失败、网关无法启动。
 * 这里用"整帧残差为 0"复核文档里的每一个帧，把文档变成可执行规格。
 */
static void test_doc_frames(void)
{
    static const struct { const char *name; uint8_t f[16]; uint8_t len; } v[] = {
        { "§6① 慢车道 读 A+B 区 28 个",     {0x01,0x03,0x00,0x00,0x00,0x1C,0x44,0x03}, 8 },
        { "§6② 窗口块1 0x0104 x125",        {0x01,0x03,0x01,0x04,0x00,0x7D,0xC5,0xD6}, 8 },
        { "§6② 窗口块2 0x0181 x125",        {0x01,0x03,0x01,0x81,0x00,0x7D,0xD4,0x3F}, 8 },
        { "§6② 窗口块3 0x01FE x125",        {0x01,0x03,0x01,0xFE,0x00,0x7D,0xE5,0xE7}, 8 },
        { "§6② 窗口块4 0x027B x125",        {0x01,0x03,0x02,0x7B,0x00,0x7D,0xF4,0x4A}, 8 },
        { "§6② 窗口块5 0x02F8 x100",        {0x01,0x03,0x02,0xF8,0x00,0x64,0xC4,0x68}, 8 },
        { "§6④ 开蜂鸣器 FC06",              {0x01,0x06,0x00,0x20,0x00,0x01,0x49,0xC0}, 8 },
        { "§6⑤ 风扇 PWM=1000 FC06",         {0x01,0x06,0x00,0x21,0x03,0xE8,0xD9,0x7E}, 8 },
        { "§6⑥ 连写控制字+PWM FC10",        {0x01,0x10,0x00,0x20,0x00,0x02,0x04,0x00,
                                              0x01,0x03,0xE8,0xA0,0xC9},              13 },
        { "§6⑦ 正常读响应 30.0℃/60.0%RH",   {0x01,0x03,0x04,0x01,0x2C,0x02,0x58,0x3A,0x9C}, 9 },
        { "§6⑧ 异常响应 非法数据值",         {0x01,0x83,0x03,0x01,0x31},               5 },
    };
    unsigned i;

    printf("\n[15] docs/02 §6 实战帧回归（防文档与代码漂移）\n");
    for (i = 0u; i < sizeof(v) / sizeof(v[0]); i++) {
        char msg[96];
        sprintf(msg, "%s  CRC 残差为 0", v[i].name);
        CHECK(mb_crc_check_frame(v[i].f, v[i].len), msg);
    }
}

int main(void)
{
    printf("==== 手写 Modbus 从机协议栈 · PC 单元测试 ====\n");

    test_crc_vectors();
    test_read_normal();
    test_read_negative_i16();
    test_read_bad_quantity();
    test_read_bad_address();
    test_write_readonly();
    test_write_ok_and_invalid_value();
    test_unsupported_fc();
    test_silent_paths();
    test_broadcast_write();
    test_write_multi_atomic();
    test_window_double_buffer();
    test_comm_watchdog();
    test_stats_and_command();
    test_doc_frames();

    printf("\n==== 结果：PASS %d / FAIL %d ====\n", g_pass, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
