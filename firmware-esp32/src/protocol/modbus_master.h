#pragma once
/* =============================================================================
 *  modbus_master.h — Modbus RTU 主站（**唯一总线持有者**）
 * =============================================================================
 *  本模块是 docs/02 §9 与 plan.md §5.4 那条硬约束的实现载体。
 *
 *  ┌───────────────────────────────────────────────────────────────────────┐
 *  │ 硬约束（违反 = 100% 出现偶发乱码，且极难排查）：                        │
 *  │                                                                       │
 *  │   1. **只有 task_modbus_bus（本模块的持有者任务）可以碰 RS485 串口和    │
 *  │      DE 引脚。** 其它任何任务（GUI / NET / INFER / 定时器回调）都不    │
 *  │      允许调用 Serial1.write()/read()、不允许 digitalWrite(DE)、        │
 *  │      不允许 delay() 等总线。                                          │
 *  │   2. 其它任务想要读/写从机寄存器，**只能**调用本文件暴露的             │
 *  │      mb_read_holding() / mb_read_input() / mb_write_single() /        │
 *  │      mb_write_multiple()，或者往 mb_req_queue 投一个 modbus_request_t。│
 *  │      这两条路都会把请求排队，由持有者任务串行执行。                    │
 *  │   3. 不许在 LVGL 的定时器回调里调上述 API（会拖慢刷新，docs/03 阶段 4）。│
 *  │      GUI 的任务是把命令投进 cmd_queue（见 app_tasks.h），由 task_cmd    │
 *  │      统一转成总线请求。                                               │
 *  └───────────────────────────────────────────────────────────────────────┘
 *
 *  对外暴露两层接口：
 *   · **同步便捷层**（90% 的场景用它）：mb_read_holding() 等，内部"入队 + 阻塞等通知"，
 *     返回值就是本次操作的结果。适合慢车道/快车道/写车道这种"我有时间等"的任务。
 *   · **异步层**：mb_enqueue()，投完就走，执行结果通过 *status_out 得到。
 *     适合将来要在一个任务里连续下发多条控制命令、不想被总线时延串起来的场景。
 *
 *  帧格式 / 功能码 / 异常码 / CRC 全部遵循 docs/02 §2~§6，未自行发明任何东西。
 * =============================================================================
 */

#include <Arduino.h>
#include <stdint.h>
#include <stdbool.h>

#include "config.h"

/* -----------------------------------------------------------------------------
 * 请求类型与结果码
 * -------------------------------------------------------------------------- */
typedef enum {
    MB_REQ_READ_HOLDING = 0,    /* FC 0x03 */
    MB_REQ_READ_INPUT   = 1,    /* FC 0x04（从机侧可复用 0x03 实现，docs/02 §4） */
    MB_REQ_WRITE_SINGLE = 2,    /* FC 0x06 */
    MB_REQ_WRITE_MULTI  = 3,    /* FC 0x10 */
} mb_req_kind_t;

typedef enum {
    MB_OK = 0,                  /* 正常响应，数据已写入调用方的缓冲 */
    MB_ERR_TIMEOUT,             /* 超时（无响应 / 响应被截断） */
    MB_ERR_CRC,                 /* 收到帧但 CRC 校验失败（多半是干扰或半双工回环没清干净） */
    MB_ERR_EXCEPTION,           /* 从机回了异常帧，具体码见 mb_stats_t.last_exception_code */
    MB_ERR_BAD_FRAME,           /* 收到帧但不是我们要的（地址/功能码/长度不对） */
    MB_ERR_ARG,                 /* 调用方给的参数越界（比如一次读 126 个寄存器） */
    MB_ERR_QUEUE_FULL,          /* 请求队列满，投不进去 */
    MB_ERR_NOT_INIT,            /* mb_init() 还没调用 */
} mb_status_t;

/* -----------------------------------------------------------------------------
 * 统计与在线判定（docs/02 §9：tx_count / timeout_count / retry_count / 离线）
 * 这些字段会被 GUI 诊断页和 MQTT 上报读，所以 getter 里做了临界区拷贝。
 * -------------------------------------------------------------------------- */
typedef struct {
    uint32_t tx_count;              /* 发出的请求帧总数（不含重试？含。重试也占总线） */
    uint32_t rx_count;              /* 收到的合法响应帧数 */
    uint32_t timeout_count;         /* 超时次数（单次尝试计一次，不是整个请求） */
    uint32_t crc_err_count;         /* CRC 错帧次数（从机侧静默丢弃，所以主机只能靠超时/坏帧发现） */
    uint32_t exception_count;       /* 从机回的异常帧次数 */
    uint32_t retry_count;           /* 触发重试的次数（第 2 次及以后的尝试都计入） */
    uint32_t seq_mismatch_count;    /* 快车道"读完发现窗口序号变了"→ 丢弃重读的次数（§7 的关键指标） */
    uint32_t consecutive_timeout;   /* 连续超时计数，达到 MODBUS_OFFLINE_TIMEOUTS 判离线并清零重计 */
    uint32_t last_ok_tick;          /* 最后一次成功响应的 FreeRTOS tick（mb_is_online 用） */
    uint32_t last_rx_ms;            /* 最后一次成功响应的 millis()，给 GUI 显示"多久没数据了" */
    uint8_t  last_exception_code;   /* 最近一次异常码（0x01/0x02/0x03），见 docs/02 §4 */
    uint8_t  last_request_kind;     /* 最近一次请求类型，排错时看"是哪条通道挂的" */
    bool     online;                /* 连续超时 < 3 时为 true */
} mb_stats_t;

/* -----------------------------------------------------------------------------
 * 请求描述符
 *
 * 生命周期约定（很重要）：
 *   · 同步 API：结构体是调用方（栈上）的，投递后调用方阻塞等通知，
 *     所以持有者任务读它的时候它一定还活着 —— 不需要动态分配；
 *   · 异步 API：调用方自己保证结构体（以及 buf/wdata 指向的内存）
 *     一直有效到 *status_out 被写入为止。
 * -------------------------------------------------------------------------- */
typedef struct modbus_request {
    mb_req_kind_t   kind;
    uint16_t        start;          /* 起始寄存器地址 */
    uint16_t        count;          /* 读：寄存器个数；FC10 写：寄存器个数 */
    uint16_t       *buf;            /* 读：响应数据写到这（调用方提供，长度 ≥ count） */
    const uint16_t *wdata;          /* FC10：要写的值（长度 ≥ count） */
    uint16_t        timeout_ms;     /* 单次尝试的超时 */
    uint8_t         retries;        /* 额外重试次数（总尝试次数 = retries + 1） */
    TaskHandle_t    waiter;         /* 非 NULL：执行完毕 xTaskNotifyGive(waiter) */
    volatile mb_status_t *status_out; /* 执行结果写回这里（waiter 通知后调用方才读） */
    /* 可选完成回调：由**持有者任务（core 0）**在写完 status_out、发完通知之后调用。
     * 用途：异步调用方不想开一个任务专门等结果时，用回调把"下一步"挂上去
     * （比如连续性下发命令的链式处理）。
     * ⚠ 回调在总线任务的上下文里跑，所以**必须短**：只允许投队列/置标志，
     *   绝对不允许在里面调 mb_read_holding() 之类的同步 API（会死锁：总线任务
     *   在等自己把请求处理完）。 */
    void          (*on_done)(struct modbus_request *self);
} modbus_request_t;

/* -----------------------------------------------------------------------------
 * 初始化
 *   · 打开 UART1（config.h 的波特率/引脚）、配置 DE 引脚为接收态
 *   · 创建请求队列 g_mb_req_queue
 *   · 跑一次 CRC + 帧构造自检（对着 docs/02 §3.1 与 §6 的字节逐条比），
 *     失败会打 ERROR 日志 —— 这是"帧格式没写错"的最廉价证据
 *  必须在 app_queues_init() 之后、任务启动之前调用。
 * -------------------------------------------------------------------------- */
void mb_init(void);

/* -----------------------------------------------------------------------------
 * 同步 API（推荐使用）
 *
 * 返回值 MB_OK 时数据才有效；其它值都表示本次操作失败（数据未定义）。
 * 行为：入队 → 阻塞等通知 → 返回结果。
 * 超时预算 = timeout_ms × (retries + 1) + MODBUS_NOTIFY_SLACK_MS。
 *
 * ⚠ 同一个任务不要并发调用（它会用自己的任务通知来等完成）。
 *   本项目里调用方是 task_modbus_slow / task_modbus_fast / task_cmd，各自串行。
 *
 * 参数约束（越界直接返回 MB_ERR_ARG，不占用总线）：
 *   FC03/FC04：count ∈ [1, 125]（docs/02 §4）
 *   FC06     ：无 count；reg 落在 C 区 0x0020~0x002F 之外会由从机回异常 0x02/0x03
 *   FC10     ：count ∈ [1, 123]（docs/02 §4）
 * -------------------------------------------------------------------------- */
mb_status_t mb_read_holding(uint16_t start, uint16_t count, uint16_t *out_buf,
                            uint32_t timeout_ms = MODBUS_TIMEOUT_MS_SLOW,
                            uint8_t  retries    = MODBUS_RETRY_READ);

mb_status_t mb_read_input(uint16_t start, uint16_t count, uint16_t *out_buf,
                          uint32_t timeout_ms = MODBUS_TIMEOUT_MS_SLOW,
                          uint8_t  retries    = MODBUS_RETRY_READ);

mb_status_t mb_write_single(uint16_t reg, uint16_t value,
                            uint32_t timeout_ms = MODBUS_TIMEOUT_MS_WRITE,
                            uint8_t  retries    = MODBUS_RETRY_WRITE);

mb_status_t mb_write_multiple(uint16_t start, uint16_t count, const uint16_t *values,
                              uint32_t timeout_ms = MODBUS_TIMEOUT_MS_WRITE,
                              uint8_t  retries    = MODBUS_RETRY_WRITE);

/* -----------------------------------------------------------------------------
 * 异步 API：把请求投进总线队列就返回，不阻塞。
 * 返回值：投递是否成功。真正的执行结果之后由持有者任务写入 *(req->status_out)，
 * 若 req->on_done 非空则随后调用它（见结构体注释里的限制）。
 * 调用方必须保证 req 及其指向的内存在结果写回前不被回收。
 * -------------------------------------------------------------------------- */
bool mb_enqueue(modbus_request_t *req, TickType_t wait_ticks);

/* -----------------------------------------------------------------------------
 * 状态查询
 * -------------------------------------------------------------------------- */
bool mb_is_online(void);                    /* 连续 3 次超时 → false；恢复后自动 true */
void mb_get_stats(mb_stats_t *out);         /* 临界区内的快照拷贝，任意任务可调 */

/* -----------------------------------------------------------------------------
 * CRC16/MODBUS（docs/02 §3 的算法，多项式 0xA001 反射）
 * 发送时**先低字节后高字节**。
 * 自检技巧（docs/02 §3）：把收到的整帧（含 CRC）送进来，结果应为 0x0000。
 * -------------------------------------------------------------------------- */
uint16_t mb_crc16(const uint8_t *buf, uint16_t len);

/* -----------------------------------------------------------------------------
 * 帧构造/CRC 自检：把 docs/02 §3.1 的 10 组测试向量、§6 表② 的 5 个分块请求帧、
 * 以及 §6 ④⑤⑥ 的写帧全部构造一遍逐字节比对。
 * mb_init() 内部会自动调用；也可以单独调用做单元测试。返回 true = 全部通过。
 * -------------------------------------------------------------------------- */
bool mb_selftest(void);

/* -----------------------------------------------------------------------------
 * 以下两个任务由 app_tasks_start() 创建，定义在 modbus_master.cpp：
 *
 *  task_modbus_bus   —— **唯一总线持有者**。只做一件事：
 *                       从 g_mb_req_queue 取请求 → 组帧 → 收发 → 校验 → 回状态。
 *                       不做任何耗时处理（解析出来的原始数据直接扔进缓冲）。
 *                       固定在 core 0，优先级 TASK_PRIO_BUS。
 *
 *  task_modbus_slow  —— 慢车道，固定 500ms：
 *                       一次读 0x0000~0x001B（28 个寄存器，docs/02 §6 ①）
 *                       → 发布到慢车道双缓冲（seqlock）
 *                       → 投 UI_MSG_ENV 给 GUI。
 *
 *  task_modbus_fast  —— 快车道，固定 2s（docs/02 §7 的顺序，**不能颠倒**）：
 *                       ① 读窗口头 0x0100~0x0103，记下 seq0
 *                       ② 按 §6 表② 分 5 块读 600 个寄存器（125/125/125/125/100）
 *                       ③ 再读一次 0x0101 得 seq1
 *                       ④ seq0 != seq1 → 本次作废、丢弃重读（seq_mismatch_count++）
 *                       → 窗口投 acq_queue（推理用）+ UI_MSG_WINDOW（画曲线用）
 *                       → 顺带读 E 区 36 个特征寄存器投 acq_queue（特征通道）
 * -------------------------------------------------------------------------- */
void task_modbus_bus(void *arg);
void task_modbus_slow(void *arg);
void task_modbus_fast(void *arg);

/* 总线请求队列（由 mb_init() 创建）。除本模块外不建议直接用，用同步 API 更安全。 */
extern QueueHandle_t g_mb_req_queue;
