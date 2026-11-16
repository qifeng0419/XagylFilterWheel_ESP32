// ============================================================================
//  StepperDriver.cpp —— 基于官方库 TMCStepper 的 TMC2209 封装
//
//  库接口核对（TMCStepper v0.7.3，已读源码确认）：
//    TMC2209Stepper(Stream * SerialPort, float RS, uint8_t addr)
//    begin() / defaults() / push() / rms_current(mA) / hold_multiplier(f)
//    microsteps(uint16_t) / en_spreadCycle(bool) / pwm_autoscale(bool) / toff(uint8_t)
//    TPOWERDOWN(uint8_t) / SGTHRS(uint8_t) / TCOOLTHRS(uint32_t) / SG_RESULT()
//    DRV_STATUS() / CHOPCONF() / freewheel(uint8_t) / test_connection() / GSTAT()
//
//  ESP32 适配要点：
//    · 库头文件里 SW_CAPABLE_PLATFORM 只对 AVR/LPC1768/STM32 为真，ESP32 只能走
//      `TMC2209Stepper(Stream*, RS, addr)` 构造；且 beginSerial() 在 ESP32 上被 delete，
//      因此**必须由我们自己调用 HardwareSerial::begin()**，再调用 driver->begin()。
//    · TMC2209 UART 波特率固定 115200（芯片上电默认，本项目不改动）。
//    · 单线半双工时 RX/TX 填同一个 GPIO；ESP32 允许同引脚收发。
// ============================================================================
#include "StepperDriver.h"

#if FW_ENABLE_TMC_UART
#include <TMCStepper.h>
#endif

StepperDriver g_stepper;

#if FW_ENABLE_TMC_UART
namespace {
// 电机 UART 用独立的 HardwareSerial 实例，避免与协议口 FW_PROTO_SERIAL 抢同一个外设。
// 具体实例由 FW_TMC_SERIAL_INDEX 决定；begin() 里做一次对象地址比对，撞车时明确报错。
HardwareSerial &tmcPort() {
#if FW_TMC_SERIAL_INDEX == 0
  return Serial;
#elif FW_TMC_SERIAL_INDEX == 1
  return Serial1;
#else
  return Serial2;
#endif
}

TMC2209Stepper *tmcDrv() {
  static TMC2209Stepper *drv = new TMC2209Stepper(&tmcPort(),
                                                  (float)FW_TMC_RSENSE,
                                                  (uint8_t)FW_TMC_ADDR);
  return drv;
}
}  // namespace
#endif

void StepperDriver::begin() {
  // ---- 引脚部分（两种模式都要配） ----
  pinMode(FW_PIN_STEP, OUTPUT);
  pinMode(FW_PIN_DIR, OUTPUT);
  digitalWrite(FW_PIN_STEP, LOW);
  digitalWrite(FW_PIN_DIR, LOW);
#if FW_PIN_EN >= 0
  pinMode(FW_PIN_EN, OUTPUT);
  digitalWrite(FW_PIN_EN, LOW);          // ENN 低电平有效
  _enabled = true;
#else
  _enabled = true;                       // ENN 已硬件接地
#endif

#if FW_ENABLE_TMC_UART == 0
  _mode = MODE_PINS_ONLY;
  return;
#else
  _mode = MODE_UART;

  // 撞口检测：协议口与电机口若是同一个 UART 外设，双方都会失能
  if ((void *)&tmcPort() == (void *)&FW_PROTO_SERIAL) {
    // 不抛异常、不死机：直接降级为纯 STEP/DIR，并在服务台如实报告原因
    _uartPortClash = true;
    _mode = MODE_PINS_ONLY;
    return;
  }

#if FW_TMC_HALF_DUPLEX
  tmcPort().begin((uint32_t)FW_TMC_BAUD, SERIAL_8N1, FW_PIN_TMC_RX, FW_PIN_TMC_RX);
#else
  tmcPort().begin((uint32_t)FW_TMC_BAUD, SERIAL_8N1, FW_PIN_TMC_RX, FW_PIN_TMC_TX);
#endif
  delay(50);                             // 等驱动板 MCU 上电就绪

  TMC2209Stepper *d = tmcDrv();
  d->begin();

  _uartOk = testConnection();
  if (!_uartOk) {                        // UART 不通：退回硬件模式，电机仍可动
    _mode = MODE_PINS_ONLY;
    return;
  }

  d->defaults();                         // 先把库内寄存器刷成安全默认值

  // 电流：hold_multiplier 必须在 rms_current 之前设置，库内部按比例算 CS 值
  d->hold_multiplier((float)FW_TMC_HOLD_RATIO);
  d->rms_current((uint16_t)FW_TMC_RUN_MA);

  d->microsteps((uint16_t)FW_MICROSTEPS);
  d->toff(5);                            // TOFF=0 会关闭驱动输出，必须 >0
#if FW_TMC_STEALTHCHOP
  d->en_spreadCycle(false);
  d->pwm_autoscale(true);
  d->pwm_autograd(true);
#else
  d->en_spreadCycle(true);               // StallGuard 只在 spreadCycle 下有效
  d->pwm_autoscale(false);
#endif
  d->TPOWERDOWN((uint8_t)FW_TMC_POWERDOWN_S);
#if FW_TMC_STALLGUARD
  d->SGTHRS((uint8_t)FW_TMC_SGTHRS);
  d->TCOOLTHRS(0xFFFFF);
#else
  d->SGTHRS(0);
  d->TCOOLTHRS(0);
#endif
  d->push();                             // 一次性把缓存寄存器写到芯片

  (void)d->GSTAT();                      // 读一次，确认双向链路真的可用
#endif
}

bool StepperDriver::testConnection() {
#if FW_ENABLE_TMC_UART == 0
  return false;
#else
  if (_mode != MODE_UART) return false;
  // test_connection() 返回 IFCNT 自增校验结果，0 = 通信正常，>0 = 丢包/无响应
  return tmcDrv()->test_connection() == 0;
#endif
}

void StepperDriver::setCurrentMa(uint16_t runMa, float holdRatio) {
  _runMa = runMa;
#if FW_ENABLE_TMC_UART
  if (_mode != MODE_UART) return;
  tmcDrv()->hold_multiplier(holdRatio);
  tmcDrv()->rms_current(runMa);
#endif
}

void StepperDriver::setMicrosteps(uint16_t microsteps) {
#if FW_ENABLE_TMC_UART
  if (_mode != MODE_UART) return;
  tmcDrv()->microsteps(microsteps);
  tmcDrv()->push();
#else
  (void)microsteps;
#endif
}

void StepperDriver::setStealthChop(bool on) {
#if FW_ENABLE_TMC_UART
  if (_mode != MODE_UART) return;
  TMC2209Stepper *d = tmcDrv();
  d->en_spreadCycle(!on);
  d->pwm_autoscale(on);
  d->push();
#else
  (void)on;
#endif
}

void StepperDriver::setPowerDown(uint8_t seconds) {
#if FW_ENABLE_TMC_UART
  if (_mode != MODE_UART) return;
  tmcDrv()->TPOWERDOWN(seconds);
#else
  (void)seconds;
#endif
}

void StepperDriver::setStallGuard(bool on, uint8_t threshold) {
#if FW_ENABLE_TMC_UART
  if (_mode != MODE_UART) return;
  TMC2209Stepper *d = tmcDrv();
  d->SGTHRS(on ? threshold : 0);
  d->TCOOLTHRS(on ? 0xFFFFF : 0);
  d->push();
#else
  (void)on;
  (void)threshold;
#endif
}

uint16_t StepperDriver::stallGuardResult() {
#if FW_ENABLE_TMC_UART
  if (_mode != MODE_UART) return 0;
  return tmcDrv()->SG_RESULT();
#else
  return 0;
#endif
}

void StepperDriver::enable(bool on) {
  _enabled = on;
#if FW_PIN_EN >= 0
  digitalWrite(FW_PIN_EN, on ? LOW : HIGH);
#endif
#if FW_ENABLE_TMC_UART
  if (_mode != MODE_UART) return;
  TMC2209Stepper *d = tmcDrv();
  if (on) {
    d->toff(5);
    d->freewheel(0);                     // 正常驱动
  } else {
    d->freewheel(1);                     // 线圈断电自由滑行，语义比 toff(0) 明确
  }
  d->push();
#endif
}

uint32_t StepperDriver::chopconf() {
#if FW_ENABLE_TMC_UART
  if (_mode != MODE_UART) return 0;
  return tmcDrv()->CHOPCONF();
#else
  return 0;
#endif
}

uint32_t StepperDriver::drvStatus() {
#if FW_ENABLE_TMC_UART
  if (_mode != MODE_UART) return 0;
  return tmcDrv()->DRV_STATUS();
#else
  return 0;
#endif
}

bool StepperDriver::driverOverTemp() {
#if FW_ENABLE_TMC_UART
  if (_mode != MODE_UART) return false;
  return tmcDrv()->ot();
#else
  return false;
#endif
}

bool StepperDriver::driverShortToGround() {
#if FW_ENABLE_TMC_UART
  if (_mode != MODE_UART) return false;
  TMC2209Stepper *d = tmcDrv();
  return d->s2ga() || d->s2gb();
#else
  return false;
#endif
}

bool StepperDriver::motorStandstill() {
#if FW_ENABLE_TMC_UART
  if (_mode != MODE_UART) return false;
  return tmcDrv()->stst();
#else
  return false;
#endif
}

uint8_t StepperDriver::csActual() {
#if FW_ENABLE_TMC_UART
  if (_mode != MODE_UART) return 0;
  return (uint8_t)tmcDrv()->cs_actual();
#else
  return 0;
#endif
}
