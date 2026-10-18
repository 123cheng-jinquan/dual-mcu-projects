#pragma once
/* =============================================================================
 *  net.h — WiFi + MQTT（上报 telemetry / 订阅 cmd）
 * =============================================================================
 *  职责（plan.md §7.2 的 net 任务，core 0，最低优先级）：
 *   · 连 WiFi（断线自动重连，带退避，别把射频任务打死）
 *   · 连 MQTT broker，订阅 `cmd/{device_id}`
 *   · 每 NET_TELEMETRY_PERIOD_MS 上报一条 JSON 到 `telemetry/{device_id}`
 *   · 收到下行命令 → **只解析成 cmd_msg_t 投进 g_cmd_queue**，
 *     绝不在这里碰 Modbus（唯一总线持有者约束，见 modbus_master.h）
 *
 *  主题（docs/03 阶段 7）：
 *    telemetry/gw01   上行，环境量 + 状态字 + 推理结果
 *    cmd/gw01         下行，控制命令
 *    alarm/gw01       上行，报警（v0.4 报警联动时启用）
 *
 *  ⚠ v0.2 里程碑（7 天冲刺）**不包含 MQTT**（plan.md §8 明确不塞进来）。
 *    这里把骨架写全，让"接上去就能用"，但要清楚它现在不是验收项。
 * =============================================================================
 */

#include <Arduino.h>
#include <stdint.h>
#include <stdbool.h>

#include "config.h"
#include "app_tasks.h"

/* 网络状态（诊断页可显示，目前并入 MQTT 上报） */
typedef struct {
    bool     wifi_connected;
    bool     mqtt_connected;
    uint32_t wifi_reconnect_count;
    uint32_t mqtt_reconnect_count;
    uint32_t tx_count;              /* 已成功 publish 的条数 */
    uint32_t rx_cmd_count;          /* 已收到的下行命令条数 */
    uint32_t last_pub_ms;
    int      last_rssi;             /* WiFi RSSI dBm */
} net_stats_t;

/* 初始化：起 WiFi（非阻塞）+ 配 MQTT。可在 setup() 里先调，也可以在任务里调。
 * 返回 false = WiFi 参数还是占位值（config.h 的 YOUR_WIFI_SSID），此时只打告警不阻塞。 */
bool net_init(void);

/* 非阻塞主循环：内部做 WiFi 状态机 + MQTT 重连 + client.loop() + 到点上报。
 * 由 task_net 每 NET_LOOP_PERIOD_MS 调用一次。 */
void net_loop(void);

/* 立刻上报一条 telemetry（正常情况下由 net_loop 按周期触发） */
bool net_publish_telemetry(void);

/* 单独发一条报警（v0.4 报警联动用） */
bool net_publish_alarm(const char *kind, const char *detail);

void net_get_stats(net_stats_t *out);

/* 网络任务入口（定义在 net.cpp）：core 0，优先级 TASK_PRIO_NET */
void task_net(void *arg);
