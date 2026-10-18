#pragma once
/* =============================================================================
 *  gui.h — LVGL 触摸 HMI（三个界面 + 更新入口）
 * =============================================================================
 *  设计口径：
 *   · 本模块**不写死屏幕驱动**。gui_init() 里有两条互斥的移植路径，按
 *     config.h 的 GUI_USE_ESP_LVGL_PORT 选（见 gui.cpp 的 TODO）。
 *   · 每个界面只有三个东西：创建函数 gui_xxx(parent)、控件句柄、
 *     一个 gui_update_xxx() 更新入口。**业务逻辑一律不在这里**。
 *   · GUI 任务在 core 1，只做两件事：lv_timer_handler() + 排空 g_ui_queue。
 *     它**绝不**调用 Modbus 的同步 API（那会阻塞刷屏）——
 *     所有下发都走 g_cmd_queue（见 app_tasks.h §5）。
 *   · 三个界面共用一个 tabview；每个 tab 的 parent 由 gui_init() 传入。
 *
 *  LVGL 版本：代码按 **v8.3.x** 写。v9 的差异点集中在三处，已在代码里用宏隔离：
 *    1) lv_scr_act()            → v9: lv_screen_active()   [GUI_SCREEN_ACTIVE()]
 *    2) lv_disp_drv_t 注册       → v9: lv_display_create() + lv_display_set_flush_cb()
 *    3) lv_obj_set_style_xxx(obj, v, sel)：v8.0/8.1 用 LV_PART_MAIN|LV_STATE_DEFAULT，
 *       v8.2+ 与 v9 都是 lv_style_selector_t —— 本项目统一传 0，两版都能编过。
 * =============================================================================
 */

#include <Arduino.h>
#include <stdint.h>
#include <stdbool.h>

#include <lvgl.h>

#include "config.h"
#include "app_tasks.h"

/* LVGL v8 / v9 的屏幕取用差异，统一从这里走 */
#if defined(LVGL_VERSION_MAJOR) && (LVGL_VERSION_MAJOR >= 9)
  #define GUI_SCREEN_ACTIVE()   lv_screen_active()
#else
  #define GUI_SCREEN_ACTIVE()   lv_scr_act()
#endif

/* -----------------------------------------------------------------------------
 * 初始化：LVGL 内核 + 显示驱动 + 触摸输入 + 建三个界面。
 * 必须在 task_gui 里、创建任何控件之前调用。
 * 返回 false 表示显示驱动没起来（此时界面不可用，但总线/网络继续跑）。
 * -------------------------------------------------------------------------- */
bool gui_init(void);

/* -----------------------------------------------------------------------------
 * 三个界面的构建函数（只建控件，不填数据）。
 * parent = 由 gui_init() 提供的 tab 页容器。
 * -------------------------------------------------------------------------- */
void gui_dashboard(lv_obj_t *parent);   /* 数据页：实时数值卡片 */
void gui_waveform(lv_obj_t *parent);    /* 波形页：三轴时域曲线 + 推理结果 */
void gui_diag(lv_obj_t *parent);        /* 诊断页：统计 + 控制按钮 */

/* -----------------------------------------------------------------------------
 * 三个更新入口（**唯一的数据入口**，由 gui_tick() 从 g_ui_queue 里取到消息后调用）
 *
 * 数据来源标注：
 *   gui_update_env()   ←  ui_msg_t.u.env  （UI_MSG_ENV，由 task_modbus_slow 投递，
 *                                          内容 = 0x0000~0x001B 共 28 个寄存器）
 *   gui_update_wave()  ←  ui_msg_t.u.win  （UI_MSG_WINDOW，由 task_modbus_fast 投递，
 *                                          指向窗口池里的只读 acq_window_t）
 *   gui_update_infer() ←  ui_msg_t.u.infer（UI_MSG_INFER，由 task_infer 投递）
 *   gui_update_diag()  ←  mb_get_stats() + slow 快照（**不走队列**：
 *                          诊断页是 1s 刷一次的只读快照，直接取更省事；
 *                          真正的数据路径仍然是队列，见 plan.md §5.4）
 *   gui_set_online()   ←  ui_msg_t.u.online（UI_MSG_ONLINE，由 task_modbus_bus 在
 *                          在线状态翻转时投递一次）
 * -------------------------------------------------------------------------- */
void gui_update_env(const slow_regs_t *env);
void gui_update_wave(const acq_window_t *win);
void gui_update_infer(const infer_result_t *res);
void gui_update_diag(const mb_stats_t *st);
void gui_set_online(bool online);

/* -----------------------------------------------------------------------------
 * GUI 心跳：由 task_gui 周期调用。
 *   · lv_timer_handler()：LVGL 内部定时器（动画、重绘、输入扫描）
 *   · 排空 g_ui_queue（非阻塞）→ 分发到上面的 update 函数
 *   · 每 GUI_DIAG_REFRESH_MS 刷一次诊断页
 *   · 处理页面切换请求 + 自动轮播（见下面的页面控制）
 * -------------------------------------------------------------------------- */
void gui_tick(void);

/* =============================================================================
 * 页面控制（2026-10 新增）
 * -----------------------------------------------------------------------------
 * 背景：本项目**没有触摸屏也没有按键**，三页界面（数据/波形/诊断）原本切不了。
 * 解决办法：
 *   ① **自动轮播**：每 GUI_PAGE_CYCLE_MS 自动切下一页（默认 10 秒，见 config.h）
 *   ② **串口命令**：`page=1` / `next` / `prev` / `auto 0|1`（见 console.cpp）
 *   ③ **MQTT 下行**：`{"page":1}` / `{"page":"next"}`（见 net.cpp）
 *
 * ★ 线程安全设计（重要）★
 *   LVGL **不是线程安全的**，而上面三种入口分别来自 core0 的不同任务
 *   （console / task_net）。所以：
 *     · 这些入口**只调用 gui_request_page()**，它仅仅写一个**原子变量**；
 *     · 真正调用 `lv_tabview_set_act()` 的只有 `gui_tick()`，它跑在 core1
 *       的 task_gui 里 —— 和 LVGL 的其它调用在同一条任务上，天然串行。
 *   ⚠ **绝对不要**在 core0 的任务里直接调用 LVGL 的 API。
 * ========================================================================== */

/* 请求切到第 page 页（0=数据 1=波形 2=诊断）。
 * 越界会被夹到合法范围；本函数**不阻塞、可在任意任务调用**。
 * 调用后会**重置自动轮播计时**（手动切完不会马上又被轮播切走）。 */
void gui_request_page(int page);

/* 读当前页（0/1/2）。任意任务可调用。 */
int  gui_get_page(void);

/* 开/关自动轮播。任意任务可调用。 */
void gui_set_auto_cycle(bool on);
bool gui_get_auto_cycle(void);

/* 相对切页：delta = +1 下一页 / -1 上一页（循环）。任意任务可调用。 */
void gui_request_page_step(int delta);

/* 排障用（console 的 status 命令会打印）：
 *   gui_debug_since_page_ms() — 距上次切页过了多久（ms）
 *   gui_debug_want()          — 当前"请求的目标页"
 * 用来判断自动轮播为什么没动（是计时没到、还是 want 卡住没被消费）。 */
uint32_t gui_debug_since_page_ms(void);
int      gui_debug_want(void);

/* -----------------------------------------------------------------------------
 * GUI 任务入口（定义在 gui.cpp）：core 1，优先级 TASK_PRIO_GUI，节拍 GUI_TICK_PERIOD_MS。
 * -------------------------------------------------------------------------- */
void task_gui(void *arg);

/* -----------------------------------------------------------------------------
 * 调试串口命令行（定义在 console.cpp）：core 0，低优先级。
 * 提供 page / next / prev / auto / status / help 等命令，
 * 让"没有按键也没有触摸"的设备也能被操作。
 * -------------------------------------------------------------------------- */
bool console_task_start(void);
