/**
 * @file    regbank.h
 * @brief   寄存器表 + 六轴采样窗口双缓冲（对应 docs/02-modbus-protocol.md 的 A/B/C/D/E 区）
 *
 * 职责划分：
 *   - regbank 负责"数据放在哪、怎么保证一致性"（临界区、双缓冲、窗口锁）
 *   - modbus_slave 负责"协议怎么解析"，通过 regbank_get/set 访问数据
 *   - 采集任务通过 regbank_write_internal() / regbank_push_sample() 写入
 *
 * 一致性机制（docs/02 §7）：采集任务以 50Hz 写 A 缓冲；满 100 样本后把 A 变成
 * "读缓冲"、B 变成"写缓冲"并让窗口序号 +1（双缓冲，读取期间无需拷贝）。
 * 主站开始读 D 区时锁定窗口（禁止切换），读到 0x0101 或超过 500ms 自动解锁，
 * 防止主站分块读到一半时缓冲被切换，导致数据跨窗口拼接。
 */
#ifndef REGBANK_H
#define REGBANK_H

#include <stdint.h>
#include <stdbool.h>
#include "modbus_slave.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 逻辑地址空间 ------------------------------------------------------ */
#define REGBANK_SIZE        0x0500u   /**< 影子寄存器数组大小，覆盖 0x0000~0x04FF */

/* A 区：环境量（只读） */
#define REG_A_TEMP          0x0000u   /**< 温度 ×10，int16 */
#define REG_A_HUMI          0x0001u   /**< 湿度 ×10 */
#define REG_A_PRESS         0x0002u   /**< 气压 ×10 hPa */
#define REG_A_LIGHT         0x0003u   /**< 光照 ADC */
#define REG_A_SMOKE         0x0004u   /**< 烟雾 MQ-2 ADC */
#define REG_A_AIR           0x0005u   /**< 空气质量 MQ-135 ADC */
#define REG_A_FLAME         0x0006u   /**< 火焰 ADC */
#define REG_A_SOIL          0x0007u   /**< 土壤湿度 */
#define REG_A_RAIN          0x0008u   /**< 雨量 ADC */
#define REG_A_DIST          0x0009u   /**< 距离 mm */
#define REG_A_PIR           0x000Au   /**< 人体感应 0/1 */
#define REG_A_RADAR         0x000Bu   /**< 人体雷达 0/1 */
#define REG_A_HR            0x000Cu   /**< 心率 bpm，0=无效 */
#define REG_A_SPO2          0x000Du   /**< 血氧 %，0=无效 */
#define REG_A_POT           0x000Eu   /**< 电位器 ADC */
#define REG_A_SEQ           0x000Fu   /**< 采样序号（主机判新旧） */

/* B 区：状态与统计（只读） */
#define REG_B_STATUS1       0x0010u   /**< 传感器在线位图 */
#define REG_B_STATUS2       0x0011u   /**< 系统状态位图 */
#define REG_B_VERSION       0x0012u   /**< 固件版本 0x0100 = v1.0 */
#define REG_B_OK_CNT_LO     0x0013u   /**< 正常帧计数低 16 位 */
#define REG_B_CRC_ERR       0x0014u   /**< CRC 错帧计数 */
#define REG_B_EXC_CNT       0x0015u   /**< 异常帧计数 */
#define REG_B_UPTIME_MIN    0x0016u   /**< 运行时间（分钟） */
#define REG_B_RESET_CAUSE   0x0017u   /**< 复位原因 */
#define REG_B_WD_TIMEOUTS   0x0018u   /**< 通信看门狗超时次数 */
#define REG_B_MODE          0x0019u   /**< 0=本地自治 1=远程受控 */
#define REG_B_ALARM_BITS    0x001Au   /**< 本地报警位图 */
#define REG_B_WIN_STATUS    0x001Bu   /**< 窗口状态（同 0x0100） */

/* C 区：控制与配置（读写） */
#define REG_C_CTRL          0x0020u   /**< 控制字 1，位定义见 docs/02 §5 C 区 */
#define REG_C_FAN_PWM       0x0021u   /**< 风扇 PWM 0~1000 */
#define REG_C_MOTOR_PWM     0x0022u   /**< 电机 PWM int16 -1000~+1000 */
#define REG_C_SERVO_ANGLE   0x0023u   /**< 舵机角度 0~180，0xFFFF=保持 */
#define REG_C_RGB_R         0x0024u
#define REG_C_RGB_G         0x0025u
#define REG_C_RGB_B         0x0026u
#define REG_C_TH_TEMP_MAX   0x0027u   /**< 温度报警上限 ×10 */
#define REG_C_TH_SMOKE_MAX  0x0028u   /**< 烟雾报警上限 */
#define REG_C_TH_HR_MIN     0x0029u   /**< 心率报警下限 */
#define REG_C_WD_TIMEOUT    0x002Au   /**< 通信看门狗超时 ×100ms，0=禁用 */
#define REG_C_SAFE_STATE    0x002Bu   /**< 安全状态位图 */
#define REG_C_SLAVE_ADDR    0x002Cu   /**< 从机地址 1~247 */
#define REG_C_BAUD_CODE     0x002Du   /**< 波特率代码 0~4 */
#define REG_C_ACQ_PERIOD    0x002Eu   /**< 采集周期 ms */
#define REG_C_COMMAND       0x002Fu   /**< 命令寄存器 0xA5A5/0x5A5A */

/* 控制字 0x0020 的位定义 */
#define CTRL_BIT_BUZZER     (1u << 0)
#define CTRL_BIT_FAN_EN     (1u << 1)
#define CTRL_BIT_MOTOR_EN   (1u << 2)
#define CTRL_BIT_RGB_EN     (1u << 3)
#define CTRL_BIT_SERVO_EN   (1u << 4)
#define CTRL_BIT_LOCAL_AUTO (1u << 5)   /**< 允许本地自治（网关离线时） */
#define CTRL_BIT_CLEAR_STAT (1u << 6)   /**< 清统计（自清位） */
#define CTRL_BIT_SAVE_CFG   (1u << 7)   /**< 保存配置到 Flash（应用层处理，自清位） */
#define CTRL_BIT_BUZZ_LONG  (1u << 8)
#define CTRL_BIT_SAFE_OFF   (1u << 9)   /**< 全部输出安全关断 */
#define CTRL_BIT_WIN_RECAP  (1u << 10)  /**< 强制重采窗口（自清位） */

/* D 区：六轴采样窗口（只读） */
#define REG_D_HEADER        0x0100u   /**< bit15=就绪，低 8 位=有效样本数 */
#define REG_D_SEQ           0x0101u   /**< 窗口序号，每完成一个窗口 +1 */
#define REG_D_RATE          0x0102u   /**< 采样率 ×10（500 = 50.0Hz） */
#define REG_D_TIMESTAMP     0x0103u   /**< 窗口时间戳低 16 位，单位 10ms */
#define REG_D_DATA          0x0104u   /**< 600 个寄存器：100 样本 × (ax,ay,az,gx,gy,gz) 行优先 */
#define REG_D_DATA_END      (REG_D_DATA + REG_WIN_SAMPLES * REG_WIN_AXES - 1u)

#define REG_WIN_SAMPLES     100u      /**< 每窗口样本数 */
#define REG_WIN_AXES        6u        /**< 每样本轴数 */
#define REG_WIN_LOCK_MS     500u      /**< 窗口锁最长保持时间（兜底，防止主站读一半崩了） */

/* E 区：特征向量（只读，36 个） */
#define REG_E_FEATURES      0x0400u   /**< 6 轴 × {均值,标准差,峰峰值,过零率,主频,主频带能量占比} */
#define REG_E_FEATURES_END  0x0423u

/* ---- 生命周期 ---------------------------------------------------------- */
/** 初始化：清寄存器、恢复出厂阈值、窗口复位 */
void regbank_init(void);

/* ---- 供 modbus_slave 使用的钩子（直接填进 mb_slave_t） ---------------- */
uint16_t regbank_get(uint16_t addr);
mb_exc_t regbank_set(uint16_t addr, uint16_t value);
mb_exc_t regbank_check(uint16_t addr, uint16_t value);
void     regbank_on_block_read(uint16_t start, uint16_t count);

/** 取寄存器区表（供 mb_slave_t 填充） */
const mb_area_t *regbank_areas(uint8_t *count_out);

/** 取本机从机地址（可从 C 区配置） */
uint16_t regbank_get_slave_addr(void);

/* ---- 采集任务写入接口 -------------------------------------------------- */
/**
 * @brief 内部写入（不做可写性校验），供采集/健康任务更新 A/B/E 区
 * @note  对 A 区这类多字节量，建议在调用处用临界区包住，或在采集任务里
 *        一次性写完后再更新 REG_A_SEQ（序号最后写，主站可据此判一致性）
 */
void regbank_write_internal(uint16_t addr, uint16_t value);

/** 写 int16（自动按补码转 uint16） */
void regbank_write_i16(uint16_t addr, int16_t value);

/**
 * @brief 推送一个 50Hz 六轴样本
 * @param six 6 个 int16：[ax, ay, az, gx, gy, gz]
 * @note  满 100 样本时自动切换缓冲、窗口序号 +1；若窗口被主站锁定则丢弃样本
 *        并置溢出标志（宁可丢样本也不要破坏正在被读的数据）
 */
void regbank_push_sample(const int16_t six[REG_WIN_AXES]);

/** 是否已有一个有效窗口（供健康任务/推理使用） */
bool regbank_window_ready(void);

/** 当前窗口序号 */
uint16_t regbank_window_seq(void);

/** 窗口是否被主站锁定 */
bool regbank_window_locked(void);

/** 窗口溢出标志（锁定期间丢过样本） */
bool regbank_window_overrun(void);

/* ---- 周期任务 ---------------------------------------------------------- */
/**
 * @brief 周期调用（建议 10ms 一次）
 * @param now_ms 单调递增毫秒计数（HAL_GetTick）
 * @note  处理：窗口锁超时释放、通信看门狗计时、运行时间累加
 */
void regbank_tick(uint32_t now_ms);

/** 通知收到一次合法主站请求（喂通信看门狗） */
void regbank_notify_master_request(uint32_t now_ms);

/** 通信看门狗是否已超时（应用层据此进入本地自治 + 安全状态） */
bool regbank_comm_watchdog_expired(void);

/**
 * @brief 把协议栈统计（g_mb_stats）同步到 B 区 0x0013~0x0015
 * @note  建议在 task_health 里每 200ms 调一次；不要放在中断里
 */
void regbank_sync_stats(void);

/**
 * B 区状态字 2（0x0011）位定义参考（应用层可用 regbank_write_internal 置位）：
 *   bit0 I2C 总线故障   bit1 SPI Flash 故障   bit2 窗口溢出
 *   bit3 处于本地自治   bit4 通信看门狗超时
 * 其中 bit2/bit3/bit4 由 regbank_tick() 自动维护。
 */
#define REGB_ST2_I2C_FAULT   (1u << 0)
#define REGB_ST2_SPI_FAULT   (1u << 1)
#define REGB_ST2_WIN_OVERRUN (1u << 2)
#define REGB_ST2_LOCAL_AUTO  (1u << 3)
#define REGB_ST2_COMM_WD     (1u << 4)

/* ---- 临界区适配 -------------------------------------------------------- */
/** 在 CubeMX 工程里定义 REGBANK_USE_FREERTOS 即可自动使用 taskENTER_CRITICAL */
#ifdef REGBANK_USE_FREERTOS
#include "FreeRTOS.h"
#include "task.h"
#define REGBANK_ENTER_CRITICAL() taskENTER_CRITICAL()
#define REGBANK_EXIT_CRITICAL()  taskEXIT_CRITICAL()
#else
#define REGBANK_ENTER_CRITICAL() do { } while (0)
#define REGBANK_EXIT_CRITICAL()  do { } while (0)
#endif

#ifdef __cplusplus
}
#endif
#endif /* REGBANK_H */
