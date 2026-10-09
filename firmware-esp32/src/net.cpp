/* =============================================================================
 *  net.cpp — WiFi + MQTT 骨架
 * =============================================================================
 *  数据流：
 *    上行：slow 快照 + infer 最新结果 → JSON → publish telemetry/gw01
 *    下行：subscribe cmd/gw01 → 解析 JSON → cmd_msg_t → g_cmd_queue → task_cmd
 *
 *  ⚠ 不要在这个模块里调用任何 Modbus API。下行走队列，理由见 modbus_master.h。
 * =============================================================================
 */

#include "net.h"
/* 需要 gui_request_page(): MQTT 下行支持 {"page":1} 切本地 UI 页面。
 * ⚠ 该接口内部只写原子变量，**不在 core0 直接碰 LVGL**（见 gui.h 的说明）。 */
#include "gui.h"

#include <string.h>
#include <stdio.h>

#include <WiFi.h>

#if __has_include(<PubSubClient.h>)
  #include <PubSubClient.h>
  #define NET_HAVE_MQTT 1
#else
  #define NET_HAVE_MQTT 0
#endif

/* ArduinoJson 用来组包/解包。没有它也能编过（降级成"只连 WiFi 不上报"），
 * 但 telemetry/cmd 就没法实现了 —— platformio.ini 里已经列为依赖。 */
#if __has_include(<ArduinoJson.h>)
  #include <ArduinoJson.h>
  #define NET_HAVE_JSON 1
#else
  #define NET_HAVE_JSON 0
#endif

/* =============================================================================
 * §1  模块状态
 * ============================================================================= */
static net_stats_t s_stats;
static uint32_t    s_last_pub_ms   = 0;
static uint32_t    s_last_wifi_try = 0;
static uint32_t    s_last_mqtt_try = 0;
static bool        s_inited        = false;

#if NET_HAVE_MQTT
static WiFiClient   s_wifi_client;
static PubSubClient s_mqtt(s_wifi_client);
#endif

/* =============================================================================
 * §2  下行命令：JSON → cmd_msg_t
 * -----------------------------------------------------------------------------
 *  报文约定（和 topics 一起定死，PC 侧 mosquitto_pub 就能测）：
 *
 *   {"reg":32,"value":1}                      → FC06 写 0x0020 = 1（开蜂鸣器）
 *   {"reg":33,"values":[1000]}                → FC10 写 0x0021 起 1 个
 *   {"fan":750}                               → CMD_FAN_SET（原子写 控制字+PWM）
 *   {"ctrl_set":1}                            → 控制字 bit0 置位（掩码直接用寄存器位定义）
 *   {"ctrl_clr":1}                            → 控制字 bit0 清位
 *   {"safe_shutdown":1}                       → CMD_SAFE_SHUTDOWN
 *   {"force_window":1}                        → CMD_FORCE_WINDOW
 *
 *  为什么允许"寄存器直写"和"语义命令"两种：前者调试期最灵活（不用改固件就能试
 *  任意寄存器），后者是给正式业务用的（比如 fan 要原子写两个寄存器）。
 * ============================================================================= */
static void handle_cmd_payload(const uint8_t *payload, unsigned int len)
{
    if (payload == NULL || len == 0) {
        return;
    }
#if !NET_HAVE_JSON
    APP_LOGW("收到下行命令但没编入 ArduinoJson，无法解析（len=%u）", len);
    (void)payload;
#else
    StaticJsonDocument<NET_JSON_BUF_SIZE> doc;
    const DeserializationError err = deserializeJson(doc, payload, len);
    if (err) {
        APP_LOGW("下行 JSON 解析失败：%s", err.c_str());
        return;
    }

    cmd_msg_t c;
    memset(&c, 0, sizeof(c));
    c.tick = millis();

    /* ★ 页面控制（2026-10）：`{"page":1}` / `{"page":"next"}` ★
     * 这是**本地 UI 操作**，不是写 485 寄存器 —— 所以它**不进 cmd_queue**，
     * 而是直接调 gui.h 的接口（内部只写原子变量，见 gui.h 的线程安全说明）。
     * 放在最前面：它和后面的"寄存器写"是两套完全不同的目标，
     * 一起写容易让人误以为 page 也会下发到 STM32。 */
    if (doc.containsKey("page")) {
        if (doc["page"].is<const char *>()) {
            const char *s = doc["page"].as<const char *>();
            if (s != NULL && (strcmp(s, "next") == 0 || strcmp(s, "n") == 0)) {
                gui_request_page_step(+1);
            } else if (s != NULL && (strcmp(s, "prev") == 0 || strcmp(s, "p") == 0)) {
                gui_request_page_step(-1);
            } else {
                APP_LOGW("page 名字无法识别（只支持 next/prev 或数字），忽略");
                return;
            }
            APP_LOGI("MQTT 下行：切页（%s）", s);
        } else {
            const int p = doc["page"].as<int>();
            if (p < 0 || p >= GUI_PAGE_COUNT) {
                APP_LOGW("page=%d 超范围（0~%d），忽略", p, GUI_PAGE_COUNT - 1);
                return;
            }
            gui_request_page(p);
            APP_LOGI("MQTT 下行：切到页面 %d", p);
        }
        s_stats.rx_cmd_count++;
        return;
    }

    if (doc.containsKey("safe_shutdown") && doc["safe_shutdown"].as<int>() != 0) {
        c.kind = CMD_SAFE_SHUTDOWN;
        APP_LOGW("MQTT 下行：全部输出安全关断");
    } else if (doc.containsKey("force_window") && doc["force_window"].as<int>() != 0) {
        c.kind = CMD_FORCE_WINDOW;
    } else if (doc.containsKey("fan")) {
        c.kind = CMD_FAN_SET;
        c.start = REG_FAN_PWM;
        c.count = 1;
        c.values[0] = (uint16_t)doc["fan"].as<int>();
    } else if (doc.containsKey("ctrl_set")) {
        c.kind = CMD_CTRL1_SET_BITS;
        c.start = REG_CTRL1;
        c.count = 1;
        c.values[0] = (uint16_t)doc["ctrl_set"].as<int>();
    } else if (doc.containsKey("ctrl_clr")) {
        c.kind = CMD_CTRL1_CLR_BITS;
        c.start = REG_CTRL1;
        c.count = 1;
        c.values[0] = (uint16_t)doc["ctrl_clr"].as<int>();
    } else if (doc.containsKey("reg")) {
        const uint16_t reg = (uint16_t)doc["reg"].as<int>();
        if (doc.containsKey("values") && doc["values"].is<JsonArray>()) {
            JsonArray arr = doc["values"].as<JsonArray>();
            c.kind  = CMD_WRITE_MULTIPLE;
            c.start = reg;
            c.count = 0;
            for (JsonVariant v : arr) {
                if (c.count >= CMD_MAX_VALUES) {
                    break;
                }
                c.values[c.count++] = (uint16_t)v.as<int>();
            }
            if (c.count == 0) {
                APP_LOGW("下行命令 values 为空，忽略");
                return;
            }
        } else if (doc.containsKey("value")) {
            c.kind = CMD_WRITE_SINGLE;
            c.start = reg;
            c.count = 1;
            c.values[0] = (uint16_t)doc["value"].as<int>();
        } else {
            APP_LOGW("下行命令缺少 value/values，忽略");
            return;
        }
    } else {
        APP_LOGW("下行命令无法识别（既没有语义键也没有 reg），忽略");
        return;
    }

    if (cmd_post(&c, 0)) {
        s_stats.rx_cmd_count++;
    } else {
        APP_LOGW("cmd_queue 满，下行命令被丢弃");
    }
#endif  /* NET_HAVE_JSON */
}

#if NET_HAVE_MQTT
static void mqtt_callback(char *topic, uint8_t *payload, unsigned int length)
{
    (void)topic;
    /* 这里运行在 PubSubClient 的 client.loop() 调用栈里（也就是 task_net 自己的
     * 上下文），所以只做"解析 + 投队列"这种快活，绝不阻塞。
     * **尤其不能调 mb_write_*()** —— 那会阻塞在总线上，把 MQTT 心跳拖超时。 */
    handle_cmd_payload(payload, length);
}
#endif

/* =============================================================================
 * §3  WiFi / MQTT 连接管理
 * ============================================================================= */

static void wifi_begin(void)
{
    /* 用占位 SSID 直接跳过联网：让"还没配 WiFi"的开发阶段也能正常跑
     * （否则每次上电都要等 15s 超时，调试很烦）。 */
    if (strcmp(WIFI_SSID, "YOUR_WIFI_SSID") == 0) {
        APP_LOGW("WiFi SSID 还是占位值，跳过联网（改 config.h 的 WIFI_SSID/WIFI_PASS）");
        return;
    }

    WiFi.mode(WIFI_STA);
    /* 关掉省电：ESP32 的 modem sleep 会让 MQTT 心跳周期性丢失，
     * 表现为"每隔几分钟断一次线"。网关是常供电设备，不需要省电。 */
    WiFi.setSleep(false);
    WiFi.setAutoReconnect(true);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    s_last_wifi_try = millis();
    APP_LOGI("WiFi 连接中：%s ...", WIFI_SSID);
}

static void wifi_loop(void)
{
    if (strcmp(WIFI_SSID, "YOUR_WIFI_SSID") == 0) {
        return;
    }
    const bool connected = (WiFi.status() == WL_CONNECTED);
    if (connected) {
        if (!s_stats.wifi_connected) {
            APP_LOGI("WiFi 已连接，IP=%s RSSI=%d", WiFi.localIP().toString().c_str(), WiFi.RSSI());
        }
        s_stats.wifi_connected = true;
        s_stats.last_rssi      = WiFi.RSSI();
        return;
    }

    s_stats.wifi_connected = false;
    /* 退避重连：不要在一个循环里疯狂 WiFi.begin()，那会把射频任务占满，
     * 连带的后果是 LVGL 卡顿 + Modbus 超时（看起来像两个无关的 bug）。 */
    if ((millis() - s_last_wifi_try) >= WIFI_RETRY_BACKOFF_MS) {
        s_last_wifi_try = millis();
        s_stats.wifi_reconnect_count++;
        APP_LOGW("WiFi 断开，第 %u 次重连...", (unsigned)s_stats.wifi_reconnect_count);
        WiFi.disconnect();
        WiFi.begin(WIFI_SSID, WIFI_PASS);
    }
}

#if NET_HAVE_MQTT
static void mqtt_loop(void)
{
    if (!s_stats.wifi_connected) {
        return;
    }
    if (s_mqtt.connected()) {
        s_stats.mqtt_connected = true;
        s_mqtt.loop();          /* 必须常调：收发 + 心跳都在这里面 */
        return;
    }

    s_stats.mqtt_connected = false;
    if ((millis() - s_last_mqtt_try) < NET_RECONNECT_BACKOFF_MS) {
        return;
    }
    s_last_mqtt_try = millis();
    s_stats.mqtt_reconnect_count++;

    s_mqtt.setServer(MQTT_BROKER, MQTT_PORT);
    s_mqtt.setKeepAlive(MQTT_KEEPALIVE_S);
    s_mqtt.setSocketTimeout(MQTT_SOCKET_TIMEOUT_S);
    s_mqtt.setBufferSize(NET_JSON_BUF_SIZE);   /* 默认 256 字节，telemetry 会超 */
    /* 遗嘱消息：网关掉线时 broker 主动发这条，云端才能区分"网关死了"和"没数据" */
    const bool ok = s_mqtt.connect(MQTT_CLIENT_ID,
                                   MQTT_USER[0] ? MQTT_USER : NULL,
                                   MQTT_PASS[0] ? MQTT_PASS : NULL,
                                   MQTT_TOPIC_ALARM, 1, true, "{\"gw\":\"offline\"}");
    if (ok) {
        APP_LOGI("MQTT 已连接 %s:%d", MQTT_BROKER, MQTT_PORT);
        s_mqtt.subscribe(MQTT_TOPIC_CMD);
        APP_LOGI("已订阅 %s", MQTT_TOPIC_CMD);
    } else {
        APP_LOGW("MQTT 连接失败 state=%d，%d ms 后重试", s_mqtt.state(), NET_RECONNECT_BACKOFF_MS);
    }
}
#endif  /* NET_HAVE_MQTT */

/* =============================================================================
 * §4  上行
 * ============================================================================= */
bool net_publish_telemetry(void)
{
#if !NET_HAVE_MQTT || !NET_HAVE_JSON
    (void)s_last_pub_ms;
    return false;
#else
    if (!s_mqtt.connected()) {
        return false;
    }

    slow_regs_t snap;
    const bool have_snap = slow_get_latest(&snap);

    StaticJsonDocument<NET_JSON_BUF_SIZE> doc;
    doc["id"]  = MQTT_DEVICE_ID;
    doc["ts"]  = millis();

    if (have_snap) {
        /* A 区（docs/02 §5-A）。除以 10 的标度在这里还原成人看的物理量，
         * 云端拿到的就是"25.3℃"而不是"253"。 */
        doc["temp"]  = (float)slow_i16(&snap, REG_TEMP) / 10.0f;
        doc["humi"]  = (float)slow_u16(&snap, REG_HUMI) / 10.0f;
        doc["press"] = (float)slow_u16(&snap, REG_PRESS) / 10.0f;
        doc["light"] = slow_u16(&snap, REG_LIGHT);
        doc["smoke"] = slow_u16(&snap, REG_SMOKE);
        doc["air"]   = slow_u16(&snap, REG_AIR);
        doc["dist"]  = slow_u16(&snap, REG_DIST);
        doc["pir"]   = slow_u16(&snap, REG_PIR);
        doc["hr"]    = slow_u16(&snap, REG_HR);
        doc["spo2"]  = slow_u16(&snap, REG_SPO2);
        /* B 区状态（诊断用） */
        doc["status1"] = slow_u16(&snap, REG_STATUS1);
        doc["status2"] = slow_u16(&snap, REG_STATUS2);
        doc["mode"]    = slow_u16(&snap, REG_MODE);
        doc["alarm"]   = slow_u16(&snap, REG_ALARM);
        doc["fok"]     = slow_u16(&snap, REG_FRAME_OK);
        doc["fcerr"]   = slow_u16(&snap, REG_FRAME_CRCERR);
        doc["uptime"]  = slow_u16(&snap, REG_UPTIME_MIN);
    }

    /* 主机侧统计（docs/02 §9 要求上报 tx/timeout/retry） */
    mb_stats_t st;
    mb_get_stats(&st);
    doc["mb_tx"]      = st.tx_count;
    doc["mb_to"]      = st.timeout_count;
    doc["mb_retry"]   = st.retry_count;
    doc["mb_crc"]     = st.crc_err_count;
    doc["mb_online"]  = st.online ? 1 : 0;
    doc["mb_seqmiss"] = st.seq_mismatch_count;

    /* 推理结果 */
    infer_result_t ir;
    if (infer_get_latest(&ir) && ir.valid) {
        doc["cls"]     = infer_label(ir.best);
        doc["conf"]    = ir.probs[ir.best];
        doc["infer_us"] = ir.infer_us;
    }

    char payload[NET_JSON_BUF_SIZE];
    const size_t n = serializeJson(doc, payload, sizeof(payload));
    if (n == 0) {
        APP_LOGW("telemetry JSON 序列化失败（缓冲 %d 字节不够？）", NET_JSON_BUF_SIZE);
        return false;
    }
    if (!s_mqtt.publish(MQTT_TOPIC_TELEMETRY, (const uint8_t *)payload, (unsigned int)n)) {
        APP_LOGW("telemetry publish 失败（MQTT 缓冲不够或已断开）");
        return false;
    }
    s_stats.tx_count++;
    s_stats.last_pub_ms = millis();
    return true;
#endif
}

bool net_publish_alarm(const char *kind, const char *detail)
{
#if !NET_HAVE_MQTT || !NET_HAVE_JSON
    (void)kind;
    (void)detail;
    return false;
#else
    if (!s_mqtt.connected()) {
        return false;
    }
    StaticJsonDocument<192> doc;
    doc["id"]     = MQTT_DEVICE_ID;
    doc["ts"]     = millis();
    doc["kind"]   = kind ? kind : "unknown";
    doc["detail"] = detail ? detail : "";
    char payload[192];
    const size_t n = serializeJson(doc, payload, sizeof(payload));
    /* PubSubClient 的载荷重载要的是 const uint8_t*，char* 不会隐式转换（C++） */
    return n > 0 && s_mqtt.publish(MQTT_TOPIC_ALARM, (const uint8_t *)payload, (unsigned int)n);
#endif
}

void net_get_stats(net_stats_t *out)
{
    if (out != NULL) {
        *out = s_stats;
    }
}

/* =============================================================================
 * §5  主循环 / 任务
 * ============================================================================= */
bool net_init(void)
{
    if (s_inited) {
        return true;            /* setup() 已调过一次，task_net 起来后再调是空操作 */
    }
    memset(&s_stats, 0, sizeof(s_stats));
#if NET_HAVE_MQTT
    s_mqtt.setCallback(mqtt_callback);
#else
    APP_LOGW("没有 PubSubClient，MQTT 功能关闭（检查 platformio.ini 的 lib_deps）");
#endif
    wifi_begin();
    s_inited = true;
    return true;
}

void net_loop(void)
{
    if (!s_inited) {
        return;
    }
    wifi_loop();
#if NET_HAVE_MQTT
    mqtt_loop();
#endif
    /* 到点上报：和快车道同拍（2s），一次把环境量 + 统计 + 推理结果发走 */
    if ((millis() - s_last_pub_ms) >= NET_TELEMETRY_PERIOD_MS) {
        (void)net_publish_telemetry();   /* 失败下次再发，不重排队（旧数据没价值） */
    }
}

void task_net(void *arg)
{
    (void)arg;
    net_init();

    TickType_t last_wake = xTaskGetTickCount();
    app_wdt_register();
    for (;;) {
        app_wdt_feed();
        net_loop();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(NET_LOOP_PERIOD_MS));
    }
}
