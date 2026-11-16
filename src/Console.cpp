// ============================================================================
//  Console.cpp —— 服务台命令行（UART0 / USB CDC）
//
//  这里输出的每一行都不属于 Xagyl 协议，只给人看。
//  `proto <XY>` 是唯一例外：它把命令注入协议状态机，走完整的设备端解析与应答路径，
//  因此可以在不另接 USB-TTL 的情况下自测协议实现。
// ============================================================================
#include "Console.h"
#include "Config.h"
#include "Encoder.h"
#include "Settings.h"
#include "Motion.h"
#include "StepperDriver.h"
#include "Protocol.h"
#include "Calibrate.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <math.h>

Console g_console;

void Console::begin() {
  FW_CONSOLE_SERIAL.begin((uint32_t)FW_CONSOLE_BAUD);
  _len = 0;
}

void Console::banner() {
  FW_CONSOLE_SERIAL.println();
  FW_CONSOLE_SERIAL.println("=== Xagyl Filter Wheel | ESP32 + TMC2209 + MT6701 ===");
  FW_CONSOLE_SERIAL.printf("firmware %s | protocol id %s | slots %d\r\n",
                           FW_BUILD_VERSION, FW_PROTO_VERSION_STR, (int)FW_FILTER_SLOTS);
  FW_CONSOLE_SERIAL.printf("motor %d/rev x micro %u = %lu pulse/rev | cruise %u Hz (%d%%) accel %d Hz/s\r\n",
                           (int)FW_MOTOR_STEPS_PER_REV, (unsigned)g_motion.microsteps(),
                           (unsigned long)g_motion.stepsPerRev(),
                           (unsigned)g_motion.speedHz(), g_motion.pctSpeed(),
                           (int)FW_ACCEL_HZ_PER_S);
  FW_CONSOLE_SERIAL.printf("encoder %s | stepper %s\r\n",
#if FW_ENCODER_BUS == FW_ENCODER_BUS_I2C
                           "MT6701 I2C @0x06",
#else
                           "MT6701 SSI/SPI",
#endif
                           g_stepper.modeText());
  if (g_stepper.uartPortClash()) {
    FW_CONSOLE_SERIAL.println("!! FW_TMC_SERIAL_INDEX clashes with the protocol port -> TMC UART disabled");
  }
  if (g_stepper.mode() == StepperDriver::MODE_PINS_ONLY && FW_ENABLE_TMC_UART) {
    FW_CONSOLE_SERIAL.println("!! TMC UART not responding -> current/microsteps come from hardware (VREF, MS1/MS2)");
  }
  FW_CONSOLE_SERIAL.println("type `help`");
}

void Console::helpText() {
  FW_CONSOLE_SERIAL.println(F("pos                 one-line status"));
  FW_CONSOLE_SERIAL.println(F("diag                full diagnostics"));
  FW_CONSOLE_SERIAL.println(F("slots               list every slot center angle"));
  FW_CONSOLE_SERIAL.println(F("go <n>              move to slot n"));
  FW_CONSOLE_SERIAL.println(F("rel <deg>           move relative to current angle"));
  FW_CONSOLE_SERIAL.println(F("abs <deg>           move to an absolute encoder angle"));
  FW_CONSOLE_SERIAL.println(F("home [<deg>]        read/set slot-1 reference angle (INITIAL POSITION)"));
  FW_CONSOLE_SERIAL.println(F("space [<deg>]       read/set slot spacing (0 = auto 360/N)"));
  FW_CONSOLE_SERIAL.println(F("set <n>             teach: current angle becomes slot n center"));
  FW_CONSOLE_SERIAL.println(F("offset <n> <units>  write one slot's offset (unit = offsetDeg)"));
  FW_CONSOLE_SERIAL.println(F("tol <deg>           in-position tolerance"));
  FW_CONSOLE_SERIAL.println(F("speed <Hz>          cruise pulse frequency"));
  FW_CONSOLE_SERIAL.println(F("micro <n>           microstepping, power of two <= 256"));
  FW_CONSOLE_SERIAL.println(F("current <mA>        run current (needs TMC UART)"));
  FW_CONSOLE_SERIAL.println(F("dirinvert [0|1]     direction mapping (encoder angle vs DIR level)"));
  FW_CONSOLE_SERIAL.println(F("boot <0|1|2>        0=from encoder 1=go slot1 2=restore last slot"));
  FW_CONSOLE_SERIAL.println(F("cal                 run per-slot offset calibration"));
  FW_CONSOLE_SERIAL.println(F("proto <XY>          inject one raw protocol command (e.g. proto G3)"));
  FW_CONSOLE_SERIAL.println(F("wheeldiag           protocol packet counters"));
  FW_CONSOLE_SERIAL.println(F("reset               restore defaults and save"));
  FW_CONSOLE_SERIAL.println(F("save                flush settings to flash"));
}

void Console::printStatus() {
  FW_CONSOLE_SERIAL.printf("slot=%d angle=%8.3f raw=%5u target=%8.3f err=%+6.3f state=%s\r\n",
                           g_settings.slotForAngle(g_encoder.angleDeg()),
                           (double)g_encoder.angleDeg(),
                           (unsigned)g_encoder.status().lastRaw,
                           (double)g_motion.targetDeg(),
                           (double)g_motion.lastErrorDeg(),
                           g_motion.stateText());
  FW_CONSOLE_SERIAL.printf("home=%7.3f spacing=%7.3f tol=%5.2f dirInv=%u boot=%u offsets:",
                           (double)g_settings.s().homeAngleDeg,
                           (double)g_settings.spacingDeg(),
                           (double)g_settings.s().positionTolDeg,
                           (unsigned)g_settings.s().dirInvert,
                           (unsigned)g_settings.s().bootMode);
  for (int i = 1; i <= FW_FILTER_SLOTS; ++i) {
    FW_CONSOLE_SERIAL.printf(" %+d", (int)g_settings.s().offsetCnt[i - 1]);
  }
  FW_CONSOLE_SERIAL.println();
}

void Console::printDiag() {
  printStatus();
  const EncoderStatus &st = g_encoder.status();
  FW_CONSOLE_SERIAL.printf("enc  ok=%d errs=%u reads=%lu/%lu field=%u crcChk=%d crcBad=%d\r\n",
                           st.ok ? 1 : 0, (unsigned)st.consecutiveErrors,
                           (unsigned long)st.readsTotal, (unsigned long)st.readsFailed,
                           (unsigned)st.fieldCode, st.crcChecked ? 1 : 0, st.crcBad ? 1 : 0);
#if FW_ENCODER_BUS == FW_ENCODER_BUS_SPI
  FW_CONSOLE_SERIAL.println("     if the angle looks mirrored, rebuild with FW_SSI_LSB_FIRST flipped");
#endif
  FW_CONSOLE_SERIAL.printf("step emitted=%lu planned=%lu hz=%u retry=%u/%u\r\n",
                           (unsigned long)g_motion.stepsEmitted(),
                           (unsigned long)g_motion.stepsPlanned(),
                           (unsigned)g_motion.currentHz(),
                           (unsigned)g_motion.retryCount(),
                           (unsigned)g_motion.retryBudget());
  if (g_motion.isFault()) FW_CONSOLE_SERIAL.printf("     FAULT: %s\r\n", g_motion.faultReason());
  FW_CONSOLE_SERIAL.printf("cal  %s step %d/%d worst=%+.3f deg",
                           g_calibrate.phaseText(), g_calibrate.currentStep(),
                           g_calibrate.totalSteps(), (double)g_calibrate.worstResidualDeg());
  if (g_calibrate.phase() == Calibrate::FAILED) FW_CONSOLE_SERIAL.printf(" reason=%s", g_calibrate.failReason());
  FW_CONSOLE_SERIAL.println();
  FW_CONSOLE_SERIAL.printf("tmc  %s DRV_STATUS=0x%08lX CHOPCONF=0x%08lX cs=%u sg=%u ot=%d s2g=%d stst=%d\r\n",
                           g_stepper.modeText(),
                           (unsigned long)g_stepper.drvStatus(),
                           (unsigned long)g_stepper.chopconf(),
                           (unsigned)g_stepper.csActual(),
                           (unsigned)g_stepper.stallGuardResult(),
                           g_stepper.driverOverTemp() ? 1 : 0,
                           g_stepper.driverShortToGround() ? 1 : 0,
                           g_stepper.motorStandstill() ? 1 : 0);
}

void Console::printSlots() {
  FW_CONSOLE_SERIAL.printf("spacing=%7.3f  home=%7.3f  offsetUnit=%5.2f deg\r\n",
                           (double)g_settings.spacingDeg(),
                           (double)g_settings.s().homeAngleDeg,
                           (double)FW_OFFSET_UNIT_DEG);
  for (int i = 1; i <= FW_FILTER_SLOTS; ++i) {
    FW_CONSOLE_SERIAL.printf("  slot %d  center=%8.3f  offset=%+d (%+0.3f deg)\r\n",
                             i, (double)g_settings.slotAngle(i),
                             (int)g_settings.s().offsetCnt[i - 1],
                             (double)g_settings.offsetDeg(i));
  }
}

// ---------------------------------------------------------------------------
// 示教式标定：把「当前实测角」定为槽位 n 的中心，反解出 home 角
// ---------------------------------------------------------------------------
void Console::teach(int slot) {
  if (slot < 1 || slot > FW_FILTER_SLOTS) { FW_CONSOLE_SERIAL.println("slot out of range"); return; }
  if (!g_encoder.healthy())               { FW_CONSOLE_SERIAL.println("encoder unhealthy, refusing to teach"); return; }
  if (g_motion.isMoving())                { FW_CONSOLE_SERIAL.println("busy, wait for movement to stop"); return; }

  const float a = g_encoder.angleDeg();
  float home = a - (float)(slot - 1) * g_settings.spacingDeg() - g_settings.offsetDeg(slot);
  home = fmodf(home, 360.0f);
  if (home < 0.0f) home += 360.0f;

  g_settings.s().homeAngleDeg = home;
  g_settings.markDirty();
  g_settings.save();
  FW_CONSOLE_SERIAL.printf("slot %d taught at %8.3f -> home=%8.3f\r\n", slot, (double)a, (double)home);
}

void Console::manualMove(int argc, char **argv) {
  if (argc < 2) { FW_CONSOLE_SERIAL.println("missing angle"); return; }
  const float v = (float)atof(argv[1]);
  const bool isAbs = (argv[0][0] == 'a' || argv[0][0] == 'A');
  const bool ok = isAbs ? g_motion.startMoveToAngle(v) : g_motion.startMoveRelative(v);
  if (ok) FW_CONSOLE_SERIAL.printf("moving (%s %+0.3f deg)\r\n", isAbs ? "to abs" : "relative", (double)v);
  else FW_CONSOLE_SERIAL.println("cannot start: busy");
}

void Console::execute(char *line) {
  char *argv[4];
  int argc = 0;
  char *p = strtok(line, " \t\r\n");
  while (p && argc < 4) { argv[argc++] = p; p = strtok(nullptr, " \t\r\n"); }
  if (argc == 0) return;
  const char *c = argv[0];

  if (!strcasecmp(c, "help") || !strcmp(c, "?"))            { helpText(); return; }
  if (!strcasecmp(c, "pos") || !strcasecmp(c, "status"))    { printStatus(); return; }
  if (!strcasecmp(c, "diag"))                               { printDiag(); return; }
  if (!strcasecmp(c, "slots"))                              { printSlots(); return; }
  if (!strcasecmp(c, "wheeldiag")) {
    FW_CONSOLE_SERIAL.printf("proto cmds=%lu bytes=%lu bad=%lu replies=%lu calibrating=%d\r\n",
                             (unsigned long)g_protocol.cmdsReceived(),
                             (unsigned long)g_protocol.bytesIn(),
                             (unsigned long)g_protocol.badFrames(),
                             (unsigned long)g_protocol.repliesSent(),
                             g_protocol.calibrating() ? 1 : 0);
    return;
  }
  if (!strcasecmp(c, "go")) {
    if (argc < 2) { FW_CONSOLE_SERIAL.println("usage: go <n>"); return; }
    FW_CONSOLE_SERIAL.println(g_motion.replanToSlot(atoi(argv[1])) ? "ok" : "bad slot");
    return;
  }
  if (!strcasecmp(c, "rel") || !strcasecmp(c, "abs"))       { manualMove(argc, argv); return; }
  if (!strcasecmp(c, "set")) {
    if (argc < 2) { FW_CONSOLE_SERIAL.println("usage: set <n>"); return; }
    teach(atoi(argv[1]));
    return;
  }
  if (!strcasecmp(c, "home")) {
    if (argc < 2) { FW_CONSOLE_SERIAL.printf("home=%8.3f\r\n", (double)g_settings.s().homeAngleDeg); return; }
    float h = (float)fmod(atof(argv[1]), 360.0);
    if (h < 0.0f) h += 360.0f;
    g_settings.s().homeAngleDeg = h;
    g_settings.markDirty();
    g_settings.save();
    FW_CONSOLE_SERIAL.printf("home=%8.3f saved\r\n", (double)h);
    printSlots();
    return;
  }
  if (!strcasecmp(c, "space")) {
    if (argc < 2) { FW_CONSOLE_SERIAL.printf("spacing=%7.3f (effective %7.3f)\r\n",
                                             (double)g_settings.s().slotSpacingDeg,
                                             (double)g_settings.spacingDeg()); return; }
    float sp = (float)atof(argv[1]);
    if (sp < 0.0f || sp > 360.0f) { FW_CONSOLE_SERIAL.println("spacing must be 0..360"); return; }
    g_settings.s().slotSpacingDeg = sp;
    g_settings.markDirty();
    g_settings.save();
    FW_CONSOLE_SERIAL.printf("spacing=%7.3f (effective %7.3f)\r\n", (double)sp, (double)g_settings.spacingDeg());
    printSlots();
    return;
  }
  if (!strcasecmp(c, "offset")) {
    if (argc < 3) { FW_CONSOLE_SERIAL.println("usage: offset <n> <units>"); return; }
    const int n = atoi(argv[1]);
    const int v = g_settings.setOffsetUnits(n, atoi(argv[2]));
    if (n < 1 || n > FW_FILTER_SLOTS) { FW_CONSOLE_SERIAL.println("bad slot"); return; }
    g_settings.save();
    FW_CONSOLE_SERIAL.printf("slot%d offset=%+d unit = %+0.3f deg\r\n", n, v, (double)g_settings.offsetDeg(n));
    return;
  }
  if (!strcasecmp(c, "tol")) {
    if (argc < 2) { FW_CONSOLE_SERIAL.printf("tol=%5.2f deg\r\n", (double)g_settings.s().positionTolDeg); return; }
    float t = (float)atof(argv[1]);
    if (t < 0.1f || t > 30.0f) { FW_CONSOLE_SERIAL.println("tol must be 0.1..30"); return; }
    g_settings.s().positionTolDeg = t;
    g_settings.markDirty();
    g_settings.save();
    FW_CONSOLE_SERIAL.printf("tol=%5.2f\r\n", (double)t);
    return;
  }
  if (!strcasecmp(c, "speed")) {
    if (argc < 2) { FW_CONSOLE_SERIAL.printf("cruise=%u Hz (%d%%)\r\n", (unsigned)g_motion.speedHz(), g_motion.pctSpeed()); return; }
    g_motion.setSpeedHz((uint16_t)atoi(argv[1]));
    g_settings.save();
    FW_CONSOLE_SERIAL.printf("cruise=%u Hz (%d%%)\r\n", (unsigned)g_motion.speedHz(), g_motion.pctSpeed());
    return;
  }
  if (!strcasecmp(c, "micro")) {
    if (argc < 2) { FW_CONSOLE_SERIAL.printf("microsteps=%u -> %lu pulse/rev\r\n",
                                             (unsigned)g_motion.microsteps(),
                                             (unsigned long)g_motion.stepsPerRev()); return; }
    g_motion.setMicrosteps((uint16_t)atoi(argv[1]));
    FW_CONSOLE_SERIAL.printf("microsteps=%u -> %lu pulse/rev (verify actual current/micro via `diag`)\r\n",
                             (unsigned)g_motion.microsteps(), (unsigned long)g_motion.stepsPerRev());
    return;
  }
  if (!strcasecmp(c, "current")) {
    if (argc < 2) { FW_CONSOLE_SERIAL.println("current is read back via DRV_STATUS cs_actual in `diag`"); return; }
    const uint16_t ma = (uint16_t)atoi(argv[1]);
    g_settings.s().runMa = ma;
    g_settings.markDirty();
    g_stepper.setCurrentMa(ma, (float)FW_TMC_HOLD_RATIO);
    FW_CONSOLE_SERIAL.printf("run current set %u mA (%s)\r\n", (unsigned)ma,
                             g_stepper.uartOk() ? "ok" : "ignored, no TMC UART");
    g_settings.save();
    return;
  }
  if (!strcasecmp(c, "dirinvert")) {
    uint8_t v = (argc < 2) ? (uint8_t)(g_settings.s().dirInvert ^ 1) : (uint8_t)(atoi(argv[1]) ? 1 : 0);
    g_settings.s().dirInvert = v;
    g_settings.markDirty();
    g_settings.save();
    FW_CONSOLE_SERIAL.printf("dirInvert=%u (%s)\r\n", (unsigned)v,
                             v ? "encoder angle increases when DIR is LOW"
                               : "encoder angle increases when DIR is HIGH");
    return;
  }
  if (!strcasecmp(c, "boot")) {
    if (argc < 2) { FW_CONSOLE_SERIAL.printf("bootMode=%u\r\n", (unsigned)g_settings.s().bootMode); return; }
    const int b = atoi(argv[1]);
    if (b < 0 || b > 2) { FW_CONSOLE_SERIAL.println("boot is 0, 1 or 2"); return; }
    g_settings.s().bootMode = (BootMode)b;
    g_settings.markDirty();
    g_settings.save();
    FW_CONSOLE_SERIAL.println("boot mode saved (applies at next power-on)");
    return;
  }
  if (!strcasecmp(c, "cal")) {
    if (g_calibrate.running()) { FW_CONSOLE_SERIAL.println("already running"); return; }
    if (g_calibrate.start()) FW_CONSOLE_SERIAL.println("calibration started, follow with `diag`");
    else FW_CONSOLE_SERIAL.printf("calibration refused: %s\r\n", g_calibrate.failReason());
    return;
  }
  if (!strcasecmp(c, "proto")) {
    if (argc < 2 || strlen(argv[1]) != 2) { FW_CONSOLE_SERIAL.println("usage: proto <2 chars, e.g. proto G3>"); return; }
    g_protocol.injectCommand(argv[1]);
    FW_CONSOLE_SERIAL.printf("injected '%c%c' -> reply left on the protocol port (see `wheeldiag`)\r\n",
                             argv[1][0], argv[1][1]);
    return;
  }
  if (!strcasecmp(c, "save")) { g_settings.markDirty(); g_settings.save(); FW_CONSOLE_SERIAL.println("saved"); return; }
  if (!strcasecmp(c, "reset")) {
    g_settings.setDefault();
    g_settings.markDirty();
    g_settings.save();
    g_motion.setMicrosteps(g_settings.s().microsteps);
    g_motion.setSpeedHz(g_settings.s().speedMaxHz);
    FW_CONSOLE_SERIAL.println("defaults restored and saved");
    return;
  }

  FW_CONSOLE_SERIAL.printf("unknown command: %s (try `help`)\r\n", c);
}

void Console::tick() {
  while (FW_CONSOLE_SERIAL.available() > 0) {
    const int ch = FW_CONSOLE_SERIAL.read();
    if (ch < 0) break;

    if (ch == '\r' || ch == '\n') {
      if (_len > 0) {
        _line[_len] = 0;
        execute(_line);
        _len = 0;
      }
      continue;
    }
    if (ch == 8 || ch == 127) {                 // 退格
      if (_len > 0) _len--;
      continue;
    }
    if (_len < sizeof(_line) - 1) _line[_len++] = (char)ch;
    else _len = 0;                               // 超长整行丢弃，避免拼出错误命令
  }
}
