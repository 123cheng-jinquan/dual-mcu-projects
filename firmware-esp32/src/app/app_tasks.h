#pragma once
/* =============================================================================
 *  app_tasks.h — 任务划分、队列、共享数据结构、缓冲池
 * =============================================================================
 *  数据流总览（ASCII 图在 main.cpp 顶部，这里是"谁拥有什么"的口径）：
 *
 *    生产者                          队列            消费者
 *    ─────────────────────────────  ──────────────  ──────────────────────────
 *    task_modbus_slow (core0)  ──▶  g_ui_queue  ──▶  task_gui (core1)
 *    task_modbus_fast (core0)  ──▶  g_ui_queue  ──▶  task_gui
 *    task_modbus_fast (core0)  ──▶  g_acq_queue ──▶  task_infer (core0)
 *    task_infer       (core0)  ──▶  g_ui_queue  ──▶  task_gui
 *    task_gui         (core1)  ──▶  g_cmd_queue ──▶  task_cmd  (core0)
 *    task_net         (core0)  ──▶  g_cmd_queue ──▶  task_cmd
 *    task_cmd         (core0)  ──▶  同步调用 modbus_master 的写 API
 *
 *  共享内存（不经队列的）：
 *    · g_slow_regs  —— 慢车道寄存器快照，seqlock 双缓冲，单写多读
 *    · mb_stats_t   —— 由 modbus_master 的临界区保护（mb_get_stats 拷贝）
 *    · 窗口三缓冲池 —— window_pool_next() 轮转，生产者写、消费者只读
 * =============================================================================
 */

#include <Arduino.h>
#include <stdint.h>
#include <stdbool.h>

#include "config.h"
#include "modbus_master.h"
#include "infer.h"

/* =============================================================================
 * §1  慢车道寄存器快照（0x0000~0x001B，28 个）
 * ============================================================================= */

typedef struct {
    uint16_t regs[MODBUS_SLOW_REG_COUNT];   /* 下标 = 地址 - MODBUS_SLOW_START */
    uint32_t tick;                          /* 本次读回来的 FreeRTOS tick */
} slow_regs_t;

/* 下标宏，避免上层到处写 0x0000+5 这种表达式 */
#define SLOW_IDX(addr)  ((addr) - MODBUS_SLOW_START)

/* seqlock 风格发布/获取：
 *   写者：seq 变奇数 → 写数据 → seq 变偶数
 *   读者：读到偶数且前后一致才认，否则重试
 * 为什么不用 mutex：读者是 GUI（core1），拿 mutex 会和 core0 的写者互相阻塞，
 * 而这份数据只有 60 字节，重试一两次的成本远小于跨核互斥的开销。 */
void slow_publish(const slow_regs_t *src);
bool slow_get_latest(slow_regs_t *out);

/* 语义化取字段的便捷函数（从快照里取，越界返回 0） */
static inline uint16_t slow_u16(const slow_regs_t *s, uint16_t addr)
{
    if (s == NULL || addr < MODBUS_SLOW_START ||
        addr >= (uint16_t)(MODBUS_SLOW_START + MODBUS_SLOW_REG_COUNT)) {
        return 0;
    }
    return s->regs[SLOW_IDX(addr)];
}
/* A 区里 int16 的（温度等）用这个取 */
static inline int16_t slow_i16(const slow_regs_t *s, uint16_t addr)
{
    return (int16_t)slow_u16(s, addr);
}

/* =============================================================================
 * §2  六轴采样窗口（D 区）与三缓冲池
 * ============================================================================= */

typedef struct {
    uint16_t win_head;                              /* 0x0100 原值（bit15 = 有效，低 8 位 = 样本数） */
    uint16_t win_seq;                               /* 0x0101 窗口序号 */
    uint16_t rate_x10;                              /* 0x0102 采样率 ×10 Hz */
    uint16_t ts_10ms;                               /* 0x0103 窗口时间戳，单位 10ms */
    uint32_t tick;                                  /* 主机读到的时刻 */
    uint32_t seq_mismatch_count;                    /* 读到这一窗时代价了多少次序号失配（诊断用） */
    /* 行优先：样本 0 的 ax,ay,az,gx,gy,gz 在最前（docs/02 §5-D 的填充顺序） */
    int16_t  samples[WINDOW_SAMPLES * WINDOW_AXES];
} acq_window_t;

/* 三缓冲池。
 * 为什么是"池 + 指针队列"而不是"把窗口按值塞进队列"：
 *   一个窗口是 100×6×2 = 1200 字节，FreeRTOS 队列的存储是**拷贝**语义，
 *   按值投递每 2s 要多做 1.2KB 的 memcpy，而且队列存储要占内部 RAM。
 *   改成 3 个静态槽位轮转 + 队列里只传指针，拷贝成本归零；
 *   3 个槽位保证"推理在算第 N 窗、GUI 在画第 N-1 窗、采集在写第 N+1 窗"互不打架。
 * 池子在 PSRAM 里（3 × 1.2KB ≈ 3.7KB；内部 RAM 要留给 WiFi/LVGL/栈）。 */
#define WINDOW_POOL_SLOTS   3

void          window_pool_init(void);
acq_window_t *window_pool_next(void);           /* 轮转到下一个可写槽位（不会返回 NULL，除非池未初始化） */

/* =============================================================================
 * §3  推理输入项（原始窗口 或 36 维特征，双通道，见 plan.md §5.3 第 4 条）
 * ============================================================================= */

typedef enum {
    ACQ_ITEM_WINDOW   = 0,      /* 原始 600 点窗口 → 1D-CNN 路径 */
    ACQ_ITEM_FEATURES = 1,      /* E 区 36 个特征寄存器原值 → MLP 路径 */
} acq_item_kind_t;

typedef struct {
    acq_item_kind_t      kind;
    const acq_window_t  *win;                       /* kind == ACQ_ITEM_WINDOW 时有效（只读，池子里不会释放） */
    int16_t              features[INFER_FEATURE_DIM]; /* kind == ACQ_ITEM_FEATURES 时有效 */
    uint16_t             win_seq;                   /* 窗口序号；特征通道填 0 */
    uint32_t             tick;
} acq_item_t;

/* =============================================================================
 * §4  UI 消息（→ task_gui）
 * ============================================================================= */

typedef enum {
    UI_MSG_ENV    = 0,          /* 慢车道 28 个寄存器快照（仪表盘卡片） */
    UI_MSG_WINDOW = 1,          /* 新窗口到达（波形页重绘） */
    UI_MSG_INFER  = 2,          /* 推理结果（波形页的标签 + 置信度） */
    UI_MSG_ONLINE = 3,          /* 在线/离线状态变化（诊断页灰显） */
} ui_msg_kind_t;

typedef struct {
    ui_msg_kind_t kind;
    union {
        slow_regs_t          env;      /* UI_MSG_ENV */
        const acq_window_t  *win;      /* UI_MSG_WINDOW（只读，池子里不会释放） */
        infer_result_t       infer;    /* UI_MSG_INFER */
        uint8_t              online;   /* UI_MSG_ONLINE：0/1 */
    } u;
} ui_msg_t;

/* 非阻塞/限时投递一条 UI 消息。返回 false = 队列满（GUI 跟不上，丢一条旧的没危害）。 */
bool ui_post(const ui_msg_t *m, TickType_t wait_ticks);

/* =============================================================================
 * §5  命令消息（GUI / MQTT → 总线写请求）
 * -----------------------------------------------------------------------------
 *  为什么不让 GUI 直接调 mb_write_single()：
 *    · GUI 任务在 core1、优先级要留给刷屏，一旦它阻塞在总线等待上，
 *      屏幕会卡住（docs/03 阶段 4 的坑）；
 *    · 而且"唯一总线持有者"这条约束的落地方式就是——所有写请求都收敛到
 *      task_cmd 这一个消费者，再由它串行调用同步 API。
 * ============================================================================= */

#define CMD_MAX_VALUES  8

typedef enum {
    CMD_WRITE_SINGLE    = 0,    /* FC06：values[0] → start */
    CMD_WRITE_MULTIPLE  = 1,    /* FC10：values[0..count-1] → start.. */
    CMD_CTRL1_SET_BITS  = 2,    /* 控制字 RMW 置位：values[0] = 掩码（CTRL1_xxx） */
    CMD_CTRL1_CLR_BITS  = 3,    /* 控制字 RMW 清位：values[0] = 掩码 */
    CMD_SAFE_SHUTDOWN   = 4,    /* = CTRL1_SET_BITS(CTRL1_SAFE_SHUTDOWN)，单独留名因为语义重要 */
    CMD_FORCE_WINDOW    = 5,    /* = CTRL1_SET_BITS(CTRL1_FORCE_WIN) */
    CMD_FAN_SET         = 6,    /* values[0] = 0~1000；
                                 * 用 **一次 FC10** 原子写 [控制字|风扇使能, 风扇PWM]
                                 * —— 就是 docs/02 §6 ⑥ 的示例帧
                                 * `01 10 00 20 00 02 04 00 01 03 E8 A0 C9` 的用法。 */
} cmd_kind_t;

typedef struct {
    cmd_kind_t kind;
    uint16_t   start;                   /* CMD_WRITE_SINGLE / CMD_WRITE_MULTIPLE 的目标地址 */
    uint16_t   count;                   /* CMD_WRITE_MULTIPLE 的寄存器个数（≤ CMD_MAX_VALUES） */
    uint16_t   values[CMD_MAX_VALUES];  /* 要写的值；CMD_WRITE_SINGLE 用 values[0] */
    uint32_t   tick;                    /* 入队时刻，用于丢弃过期命令（见 task_cmd 的实现） */
} cmd_msg_t;

/* 投递一条命令（GUI 按钮 / MQTT 下行都用它）。返回 false = 队列满。 */
bool cmd_post(const cmd_msg_t *c, TickType_t wait_ticks);

/* =============================================================================
 * §6  队列与初始化
 * ============================================================================= */

extern QueueHandle_t g_acq_queue;   /* acq_item_t  → task_infer */
extern QueueHandle_t g_ui_queue;    /* ui_msg_t    → task_gui   */
extern QueueHandle_t g_cmd_queue;   /* cmd_msg_t   → task_cmd   */

/* 创建三条队列 + 初始化窗口池（PSRAM）。
 * 必须在 mb_init() 之前调用：modbus_master 的初始化会往队列里发自检日志消息。 */
void app_queues_init(void);

/* 看门狗：app_wdt_init() 在 setup() 里调一次；
 * 每个工作任务的**循环入口**先调 app_wdt_register() 把自己登记进去，
 * 然后循环里周期调 app_wdt_feed()。
 *
 * 为什么要每个任务自己注册：ESP-IDF 的 esp_task_wdt_add(NULL) 只能登记
 * **调用它的那个任务**，没法从外部把别人加进去。
 * 为什么不用 Arduino 的 enableLoopWDT()：loop() 里只有 vTaskDelay，
 * 它永远不会卡住 —— 那个看门狗什么也监控不到（这是个很常见的假安全感）。
 * ⚠ 登记过的任务如果超过 WDT_TIMEOUT_S 没喂狗，会被直接 panic 复位。 */
bool app_wdt_init(void);
void app_wdt_register(void);
void app_wdt_feed(void);

/* 创建全部 7 个任务（按 config.h 的核/优先级/栈），返回 false = 有任务创建失败。 */
bool app_tasks_start(void);

/* =============================================================================
 * §7  任务入口原型（实现分散在各自的 .cpp）
 * ============================================================================= */
void task_cmd(void *arg);       /* app_tasks.cpp */
/* task_modbus_bus / task_modbus_slow / task_modbus_fast  → modbus_master.h */
/* task_infer                                            → infer.h          */
/* task_gui                                              → gui.h            */
/* task_net                                              → net.h            */
