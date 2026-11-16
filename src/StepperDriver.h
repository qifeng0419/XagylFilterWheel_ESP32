// ============================================================================
//  StepperDriver.h —— TMC2209 封装，基于官方/社区标准库 TMCStepper
//
//  按用户要求使用官方驱动库：本文件不自行实现 UART 报文与 CRC8，
//  全部寄存器读写交给 TMCStepper 库的 TMC2209Stepper 类完成。
//
//  依赖库：TMCStepper（Arduino Library Manager 搜索 "TMCStepper"，作者 teemuatlut）
//  若 FW_ENABLE_TMC_UART == 0，本类退化为「只配置 STEP/DIR/EN 引脚」的空实现，
//  细分、电流由硬件 MS1/MS2/VREF 决定。
// ============================================================================
#pragma once

#include <Arduino.h>
#include "Config.h"

class StepperDriver {
 public:
  enum Mode : uint8_t { MODE_PINS_ONLY, MODE_UART };

  void begin();
  Mode mode() const { return _mode; }
  bool uartOk() const { return _uartOk; }

  // 运行/保持电流，单位 mA(RMS)
  void setCurrentMa(uint16_t runMa, float holdRatio);
  uint16_t runMa() const { return _runMa; }

  // microsteps 为「实际细分数」：1/2/4/.../256，库内部换算 mres
  void setMicrosteps(uint16_t microsteps);

  void setStealthChop(bool on);
  void setPowerDown(uint8_t seconds);
  void setStallGuard(bool on, uint8_t threshold);
  uint16_t stallGuardResult();

  // 使能 / 断电（freewheel）
  void enable(bool on);
  bool enabled() const { return _enabled; }

  // 连接自检：读 IC 版本寄存器，失败说明 UART 没通
  bool testConnection();

  // 诊断输出用
  uint32_t chopconf();
  uint32_t drvStatus();
  bool     driverOverTemp();
  bool     driverShortToGround();
  bool     motorStandstill();
  uint8_t  csActual();

  // 电机 UART 与协议 UART 撞了同一个外设（配置错误，已自动降级）
  bool uartPortClash() const { return _uartPortClash; }
  const char *modeText() const {
    if (_mode == MODE_PINS_ONLY) return "STEP/DIR only";
    return _uartOk ? "STEP/DIR + TMC UART ok" : "STEP/DIR + TMC UART unreachable";
  }

 private:
  Mode   _mode = MODE_PINS_ONLY;
  bool   _uartOk = false;
  bool   _uartPortClash = false;
  bool   _enabled = false;
  uint16_t _runMa = FW_TMC_RUN_MA;
};

extern StepperDriver g_stepper;
