// ============================================================================
//  Config.h —— 滤镜轮固件唯一需要用户修改的配置头
//
//  硬件：ESP32 + TMC2209（官方库 TMCStepper）+ MT6701 磁编码器
//  协议：Xagyl Filter Wheel 串口协议（见 docs/PROTOCOL.md）
// ============================================================================
#pragma once

#include <Arduino.h>

// ----------------------------------------------------------------------------
// 0. 机械与光学参数
// ----------------------------------------------------------------------------
// 滤镜位数。注意：Xagyl 协议通过 I8 把它上报给上位机，上位机会据此校验范围。
#ifndef FW_FILTER_SLOTS
#define FW_FILTER_SLOTS 5
#endif

// 槽位间距，单位为「编码器角度」。
// 0 = 自动取 360 / FW_FILTER_SLOTS（滤镜轮等分且编码器与轮同轴时正确）。
// 若编码器通过齿轮比多转，或槽位非等分，请在此填实际角度，
// 或用服务台 space / 自动校准（R6）逐槽修正。
// 本值只是出厂默认，运行期以 NVS 中的 slotSpacingDeg 为准。
#ifndef FW_SLOT_SPACING_DEG
#define FW_SLOT_SPACING_DEG 0.0
#endif

// 到位判定容差，单位「度」。
// MT6701 的积分非线性 INL 典型 ±1.0°、最大 ±1.5°，因此容差必须大于该量级，
// 否则每次换位都会触发纠正重试。自动校准会把各槽中心实测值写回配置表，
// 从而抵消 INL 误差，此时容差可以收紧。
#ifndef FW_POSITION_TOL_DEG
#define FW_POSITION_TOL_DEG 2.0f
#endif

// 空闲时对编码器角度做的静默复查。开启后若轮子被外力推动超出容差，
// 固件会自行走回原位（不产生任何串口输出）。
// 注意：预处理器 #if 无法比较浮点常量，所以「开关」用整数宏、
//       「阈值」用浮点宏，二者分开定义。
#ifndef FW_IDLE_RECHECK_ON
#define FW_IDLE_RECHECK_ON 0
#endif
#ifndef FW_IDLE_RECHECK_DEG
#define FW_IDLE_RECHECK_DEG 0.0f
#endif
#ifndef FW_IDLE_RECHECK_MS
#define FW_IDLE_RECHECK_MS 500
#endif

// ----------------------------------------------------------------------------
// 1. 步进电机 / TMC2209
// ----------------------------------------------------------------------------
// 电机整步数（1.8° 电机 = 200；0.9° 电机 = 400）
#ifndef FW_MOTOR_STEPS_PER_REV
#define FW_MOTOR_STEPS_PER_REV 200
#endif

// 微步细分：1 / 2 / 4 / 8 / 16 / 32 / 64 / 128 / 256
#ifndef FW_MICROSTEPS
#define FW_MICROSTEPS 64
#endif

// 运动引脚（STEP/DIR 模式）
#ifndef FW_PIN_STEP
#define FW_PIN_STEP 26
#endif
#ifndef FW_PIN_DIR
#define FW_PIN_DIR 27
#endif
// 使能引脚（低电平使能）。硬件已把 ENN 接低时填 -1 表示不使用。
#ifndef FW_PIN_EN
#define FW_PIN_EN -1
#endif

// 速度曲线
#ifndef FW_SPEED_MAX_HZ            // 巡航脉冲频率（步/秒）
#define FW_SPEED_MAX_HZ 3000
#endif
#ifndef FW_SPEED_MIN_HZ            // 起跳频率（梯形曲线最低速，见 Motion.cpp）
#define FW_SPEED_MIN_HZ 100
#endif
#ifndef FW_ACCEL_HZ_PER_S          // 加速度（步/秒²）
#define FW_ACCEL_HZ_PER_S 6000
#endif

// 一次移动允许的最大纠正重试次数（到位后编码器仍超差）
#ifndef FW_MOVE_MAX_RETRY
#define FW_MOVE_MAX_RETRY 2
#endif
// 单次移动超时，毫秒。超时后放弃并保持当前实际位置。
#ifndef FW_MOVE_TIMEOUT_MS
#define FW_MOVE_TIMEOUT_MS 8000
#endif
// 到位后需连续满足容差的时长，用于等待机械振荡衰减
#ifndef FW_SETTLE_MS
#define FW_SETTLE_MS 120
#endif

// ----------------------------------------------------------------------------
// 2. TMC2209 UART（官方库 TMCStepper）
// ----------------------------------------------------------------------------
// 是否启用 UART 配置链路。0 = 纯 STEP/DIR 硬件模式（依赖 MS1/MS2、VREF 等硬件设置）
#ifndef FW_ENABLE_TMC_UART
#define FW_ENABLE_TMC_UART 1
#endif

// 单线半双工：RX 与 TX 接同一个 GPIO（PDN_UART 经 1kΩ 串联 + 10kΩ 上拉）
#ifndef FW_TMC_HALF_DUPLEX
#define FW_TMC_HALF_DUPLEX 1
#endif

#ifndef FW_PIN_TMC_RX
#define FW_PIN_TMC_RX 16
#endif
#ifndef FW_PIN_TMC_TX
#define FW_PIN_TMC_TX 17          // 半双工时请与 RX 填同一个值
#endif

// TMC UART 波特率固定 115200（TMC2209 上电默认，本项目不改动）
#ifndef FW_TMC_BAUD
#define FW_TMC_BAUD 115200
#endif

// TMC 使用哪个硬件 UART 实例：0=Serial, 1=Serial1, 2=Serial2。
// 必须与下面的 FW_PROTO_SERIAL 不同，否则两套协议会抢同一个外设。
// 固件启动时会比对对象地址，冲突则自动降级为纯 STEP/DIR 并在服务台如实报告。
#ifndef FW_TMC_SERIAL_INDEX
#define FW_TMC_SERIAL_INDEX 1
#endif

// 从机地址：由 MS1/MS2 引脚电平决定，0~3
#ifndef FW_TMC_ADDR
#define FW_TMC_ADDR 0
#endif

// 采样电阻，单位 Ω。常见模块 0.11；内部检流用 TMC2209 不适用（那是 2208 特性）
#ifndef FW_TMC_RSENSE
#define FW_TMC_RSENSE 0.11f
#endif

// 电流（mA，RMS）与保持电流比例
#ifndef FW_TMC_RUN_MA
#define FW_TMC_RUN_MA 500
#endif
#ifndef FW_TMC_HOLD_RATIO
#define FW_TMC_HOLD_RATIO 0.30f
#endif

// 静音模式：1 = StealthChop（安静、力矩略小），0 = SpreadCycle（有力、有啸叫）
// 必须用 1/0 而不是 true/false：预处理器不认识 true，会把它当未定义标识符求值为 0，
// 导致 #if FW_TMC_STEALTHCHOP 永远为假、静音模式形同虚设。
#ifndef FW_TMC_STEALTHCHOP
#define FW_TMC_STEALTHCHOP 1
#endif

// 空闲后自动降流/断流（TPOWERDOWN 秒数，0 = 保持运行电流）
#ifndef FW_TMC_POWERDOWN_S
#define FW_TMC_POWERDOWN_S 20
#endif

// StallGuard 堵转检测（需要 spreadCycle 且电机在动才有效），默认关
#ifndef FW_TMC_STALLGUARD
#define FW_TMC_STALLGUARD 0
#endif
#ifndef FW_TMC_SGTHRS
#define FW_TMC_SGTHRS 60
#endif

// ----------------------------------------------------------------------------
// 3. MT6701 磁编码器
// ----------------------------------------------------------------------------
// 总线选择：1 = I2C（推荐，默认）  2 = SSI/SPI（只读，含磁场状态与 CRC）
#ifndef FW_ENCODER_BUS
#define FW_ENCODER_BUS 1
#endif
#define FW_ENCODER_BUS_I2C 1
#define FW_ENCODER_BUS_SPI 2

// --- I2C ---
#ifndef FW_PIN_I2C_SDA
#define FW_PIN_I2C_SDA 21
#endif
#ifndef FW_PIN_I2C_SCL
#define FW_PIN_I2C_SCL 22
#endif
// MT6701 出厂地址 0x06（处于 I2C 保留区，总线扫描扫不到，属正常），备选 0x46
#ifndef FW_MT6701_I2C_ADDR
#define FW_MT6701_I2C_ADDR 0x06
#endif
#ifndef FW_I2C_FREQ_HZ
#define FW_I2C_FREQ_HZ 400000
#endif

// --- SSI/SPI ---
#ifndef FW_PIN_SSI_SCK
#define FW_PIN_SSI_SCK 18
#endif
#ifndef FW_PIN_SSI_MISO
#define FW_PIN_SSI_MISO 19
#endif
#ifndef FW_PIN_SSI_CS
#define FW_PIN_SSI_CS 5
#endif
// ESP32 硬件 SPI 要求 MOSI 是有效 GPIO。SSI 是只读接口，MOSI 不接线即可，
// 但必须给一个与 SCK/MISO/CS 都不冲突的引脚号。
#ifndef FW_PIN_SSI_MOSI
#define FW_PIN_SSI_MOSI 23
#endif
#ifndef FW_SSI_FREQ_HZ
#define FW_SSI_FREQ_HZ 1000000     // 手册最高 10MHz；ESPHome 默认 1MHz Mode1
#endif
// 帧内位序：手册按「bit0 先出」编号（D0..D13 → Mg0..Mg3 → CRC0..CRC5），
// 即先收到的位是最低有效位。若你的芯片实测为 MSB 先行，改为 0。
#ifndef FW_SSI_LSB_FIRST
#define FW_SSI_LSB_FIRST 1
#endif
// CRC-6 校验。多项式 x^6+x+1（0x43），初值 0。
// ⚠ 位序/反射在公开资料中无法 100% 确认，故默认关闭。
//    上机后用服务台 `encdiag` 连续读几百帧，确认角度稳定无误码，再置 1。
#ifndef FW_SSI_CHECK_CRC
#define FW_SSI_CHECK_CRC 0
#endif

// ----------------------------------------------------------------------------
// 4. 串口端口分配
// ----------------------------------------------------------------------------
// 协议口：接天文台主控（Xagyl 驱动），波特率必须 9600 8N1
#ifndef FW_PROTO_BAUD
#define FW_PROTO_BAUD 9600
#endif
#ifndef FW_PROTO_SERIAL
#define FW_PROTO_SERIAL Serial2    // ESP32-S3/C3 只有 2~3 个 UART，必要时改为 Serial1
#endif
#ifndef FW_PIN_PROTO_RX
#define FW_PIN_PROTO_RX 4
#endif
#ifndef FW_PIN_PROTO_TX
#define FW_PIN_PROTO_TX 2
#endif

// 服务台口：给人用的调试命令行。默认 UART0（GPIO1 TX / GPIO3 RX），即多数 ESP32
// 开发板的板载 USB-串口桥。
// 若你的板子只有 USB-C 原生口（S3/C3 常见），把下面改成 SerialUSB 等价写法并
// 在 IDE 里开启 "USB CDC On Boot: Enabled"，同时确保 UART0 未被占用。
#ifndef FW_CONSOLE_BAUD
#define FW_CONSOLE_BAUD 115200
#endif
#ifndef FW_CONSOLE_SERIAL
#define FW_CONSOLE_SERIAL Serial
#endif

// ----------------------------------------------------------------------------
// 5. 协议行为开关（细节见 docs/PROTOCOL_NOTES.md）
// ----------------------------------------------------------------------------
// G / B / F 是否立即回一行状态。
// 1 = 回 "Moving"（推荐：兼容 xagyl.cpp 的 getSlotParams() 读取，多余字节会被
//     驱动的 purgeTxRx 丢弃，不会破坏 I2 轮询）
// 0 = 完全不应答
#ifndef FW_PROTO_REPLY_ON_MOVE
#define FW_PROTO_REPLY_ON_MOVE 1
#endif

// 是否实现 T0 的 "Sensors LL RR" 双值格式（Xagyl 专属，SBIG 老固件只回单值）
#ifndef FW_PROTO_T0_SENSORS
#define FW_PROTO_T0_SENSORS 1
#endif

// I1 上报的字符串。驱动会做「去掉多余小数点 + atof」归一化：
//   "4.2"   -> 4.2  → 启用 R7 恢复默认、S 命令 16 档调速
//   "3.1.5" -> 3.15 → ≤3.4 分支，换位期间用 bytesWaitingRx 探测
//   带 "FW" 前缀会被 atof 判成 0.0，不要加前缀。
#ifndef FW_PROTO_VERSION_STR
#define FW_PROTO_VERSION_STR "4.2"
#endif

// 真正的代码版本，只在服务台横幅里显示
#define FW_BUILD_VERSION "1.0.0"
#define FW_MODEL_STRING  "ESP32-Xagyl-FilterWheel"

// ----------------------------------------------------------------------------
// 6. 掉电保存
// ----------------------------------------------------------------------------
// 用 Preferences(NVS) 存一个配置块。以下参数可经服务台或协议命令改写并保存。
#ifndef FW_NVS_NAMESPACE
#define FW_NVS_NAMESPACE "fwheel"
#endif
#ifndef FW_NVS_KEY
#define FW_NVS_KEY "cfg"
#endif

// 偏移量单位：一次 `(0` / `)0` 命令改变多少度。
// Xagyl 协议只有 ±1 单位步进，没有「直接赋值」命令，故单位越小越精确但收敛越慢。
#ifndef FW_OFFSET_UNIT_DEG
#define FW_OFFSET_UNIT_DEG 0.25f
#endif
