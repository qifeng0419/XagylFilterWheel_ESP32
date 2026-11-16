// ============================================================================
//  main.cpp —— 固件入口（PlatformIO src/ 结构）
//
//  硬件      ：ESP32 + TMC2209（官方库 TMCStepper）+ MT6701 14 位磁编码器
//  协议      ：Xagyl Filter Wheel 串口协议（rpineau/XagylFilterWheel 的驱动契约）
//  核心特性  ：初始位置可调（home 角 / 槽间距 / 逐槽偏移 / 上电策略，全部掉电保存）
//
//  所有可调参数集中在 src/Config.h；上电后服务台（UART0）可现场修改并落盘。
//
//  Arduino 迁移说明：
//    从 .ino 改为 .cpp 后，两个关键差别必须处理 ——
//    1) 不再自动 #include <Arduino.h>，本文件显式包含；
//    2) 不再有自动函数原型，static 辅助函数定义在 setup() 之前即可（本文件已如此）。
//
//  接线速查（默认引脚，改配置请看 README）：
//    TMC2209 STEP -> GPIO26      TMC2209 DIR  -> GPIO27
//    TMC2209 PDN_UART -> GPIO16 (经 1k 串阻 + 10k 上拉到 3.3V)
//    MT6701 SDA -> GPIO21  SCL -> GPIO22   （I2C 后端，默认）
//    协议口 RX -> GPIO4    TX -> GPIO2     （9600 8N1，接天文台主控）
//    服务台 UART0 RX/TX -> GPIO3/GPIO1     （115200，接板载 USB-串口桥）
// ============================================================================
#include <Arduino.h>
#include <math.h>

#include "Config.h"
#include "Settings.h"
#include "Encoder.h"
#include "StepperDriver.h"
#include "Motion.h"
#include "Protocol.h"
#include "Console.h"
#include "Calibrate.h"

namespace {

// 不考虑容差，返回离 deg 最近的槽位（1..N）
int nearestSlot(float deg) {
  int best = 1;
  float bd = 1e9f;
  for (int i = 1; i <= FW_FILTER_SLOTS; ++i) {
    const float d = fabsf(Encoder::signedDelta(deg, g_settings.slotAngle(i)));
    if (d < bd) { bd = d; best = i; }
  }
  return best;
}

void runBootPolicy() {
  // 绝对值编码器的优势：位置不需要靠归零找回，只需要「吸附到最近的槽中心」。
  if (!g_encoder.healthy()) {
    // 编码器还没读通：先重试若干次，仍失败就直接走槽 1（最保守的安全位置）
    for (int i = 0; i < 5 && !g_encoder.healthy(); ++i) g_encoder.update();
    FW_CONSOLE_SERIAL.println("boot: encoder unhealthy, falling back to slot 1");
    g_motion.replanToSlot(1);
    return;
  }

  switch (g_settings.s().bootMode) {
    case BOOT_TO_SLOT_1:
      g_motion.replanToSlot(1);
      FW_CONSOLE_SERIAL.println("boot: moving to slot 1");
      break;

    case BOOT_RESTORE_LAST: {
      const int s = g_settings.s().lastSlot;
      if (s >= 1 && s <= FW_FILTER_SLOTS) {
        g_motion.replanToSlot(s);
        FW_CONSOLE_SERIAL.printf("boot: restoring last slot %d\r\n", s);
      } else {
        g_motion.replanToSlot(1);
        FW_CONSOLE_SERIAL.println("boot: no valid last slot, going to slot 1");
      }
      break;
    }

    case BOOT_FROM_ENCODER:
    default: {
      const float a = g_encoder.angleDeg();
      const int s = g_settings.slotForAngle(a);
      if (s > 0) {
        FW_CONSOLE_SERIAL.printf("boot: on slot %d at %8.3f deg, no move needed\r\n", s, (double)a);
      } else {
        // 停在两个槽之间（被人扳动过、或掉电时正在走位）-> 吸附到最近的槽中心
        const int n = nearestSlot(a);
        FW_CONSOLE_SERIAL.printf("boot: between slots at %8.3f deg, snapping to slot %d\r\n",
                                 (double)a, n);
        g_motion.replanToSlot(n);
      }
      break;
    }
  }
}

}  // namespace

void setup() {
  delay(200);                       // 等 USB/串口桥稳定，避免丢横幅

  g_settings.begin();
  g_encoder.begin();
  g_stepper.begin();
  g_motion.begin();

  // 把掉电前保存的运行参数真正下发到驱动器
  g_motion.setMicrosteps(g_settings.s().microsteps);
  g_motion.setSpeedHz(g_settings.s().speedMaxHz);
  g_stepper.setCurrentMa(g_settings.s().runMa, (float)FW_TMC_HOLD_RATIO);

  g_protocol.begin();
  g_console.begin();
  g_console.banner();

  if (FW_FILTER_SLOTS < 1 || FW_FILTER_SLOTS > SETTINGS_MAX_SLOTS) {
    FW_CONSOLE_SERIAL.printf("!! FW_FILTER_SLOTS=%d out of range [1,%d] -> using 1\r\n",
                            (int)FW_FILTER_SLOTS, SETTINGS_MAX_SLOTS);
  }

  runBootPolicy();
}

void loop() {
  static bool wasMoving = false;
  static uint32_t lastEncPollMs = 0;
#if FW_IDLE_RECHECK_ON
  static uint32_t lastIdleCheckMs = 0;
#endif

  // 1) 运动层：发脉冲 / 沉降 / 验位（内部自带单次时长上限）
  g_motion.tick();

  // 2) 协议层：收命令、回应答、推进校准
  g_protocol.tick();

  // 3) 服务台：只在有输入时才消耗时间
  g_console.tick();

  // 4) 走位结束结算：记录掉电记忆槽位
  const bool moving = g_motion.isMoving();
  if (wasMoving && !moving) g_protocol.noteMotionComplete();
  wasMoving = moving;

  // 5) 静止时保持编码器新鲜，让 I2 轮询拿到的是当下真实位置。
  //    30 ms 一次：MT6701 角度延时 <5 µs，I2C 400k 读一次约 0.1 ms，开销可忽略。
  const uint32_t now = millis();
  if (!moving && (now - lastEncPollMs) >= 30) {
    lastEncPollMs = now;
    g_encoder.update();
  }

  // 6) 静默复查（可选，默认关闭）：轮子被外力推动后自行走回原位。
  //    注意这会产生「上位机没发命令却自己在动」的行为，只在明确需要时才开。
#if FW_IDLE_RECHECK_ON
  if (!moving && !g_protocol.calibrating() && g_encoder.healthy() &&
      (now - lastIdleCheckMs) >= (unsigned long)FW_IDLE_RECHECK_MS) {
    lastIdleCheckMs = now;
    const int s = g_settings.slotForAngle(g_encoder.angleDeg());
    if (s > 0) {
      const float err = fabsf(Encoder::signedDelta(g_encoder.angleDeg(), g_settings.slotAngle(s)));
      const float tol = g_settings.s().positionTolDeg;
      if (err > tol + (float)FW_IDLE_RECHECK_DEG) g_motion.replanToSlot(s);
    } else {
      g_motion.replanToSlot(nearestSlot(g_encoder.angleDeg()));
    }
  }
#endif

  // 7) 配置落盘兜底：服务台改过的参数若还没写下去，在这里安静补写
  if (!moving && g_settings.dirty()) g_settings.save();
}
