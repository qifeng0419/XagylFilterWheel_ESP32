// ============================================================================
//  Protocol.cpp —— Xagyl 协议设备端实现
//
//  实现原则（对应 docs/PROTOCOL.md）：
//    1) 应答严格匹配驱动的 sscanf 模板；文本 + CRLF；文本长度 ≤ 30 字符
//       （驱动 readResponse 最多读 32 字节，其中 2 字节留给 CR LF）。
//    2) G / B / F 不阻塞：立即回 "Moving"，靠 I2 轮询收敛。
//       坚持回一行而不是完全静默：驱动的 getSlotParams() 发 G 后会调用
//       readResponse()，无应答会白等 1 秒超时。
//    3) 位置一律由编码器实测角反推（currentSlot），不回报「命令里期望的值」。
//       这是与两份 SBIG 系固件最关键的行为差别 —— 它们走位失败仍回目标位号。
// ============================================================================
#include "Protocol.h"
#include "Encoder.h"
#include "Settings.h"
#include "Motion.h"
#include "StepperDriver.h"
#include "Calibrate.h"
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <math.h>

Protocol g_protocol;

namespace {
constexpr uint32_t CALIB_DONE_LEAD_MS = 80;   // Calibrating 与 Done 之间的最小间隔
constexpr size_t   REPLY_MAX_TEXT = 30;       // 见文件头说明
}

void Protocol::begin() {
  FW_PROTO_SERIAL.begin((uint32_t)FW_PROTO_BAUD, SERIAL_8N1,
                        FW_PIN_PROTO_RX, FW_PIN_PROTO_TX);
  _rxState = 0;
}

// ---------------------------------------------------------------------------
// 应答输出：统一补 CRLF，并限制长度防止破坏驱动的定长缓冲解析
// ---------------------------------------------------------------------------
void Protocol::reply(const char *line) {
  if (!line) return;
  char buf[REPLY_MAX_TEXT + 3];
  strncpy(buf, line, REPLY_MAX_TEXT);
  buf[REPLY_MAX_TEXT] = 0;
  FW_PROTO_SERIAL.print(buf);
  FW_PROTO_SERIAL.print("\r\n");
  _replies++;
}

void Protocol::notePositionChanged(int slot) {
  if (slot >= 0 && slot <= FW_FILTER_SLOTS) {
    if ((int)g_settings.s().lastSlot != slot) {
      g_settings.s().lastSlot = (uint8_t)slot;
      g_settings.markDirty();
    }
  }
}

// 运动层每次从 MOVING 走到终态（IDLE 或 FAULT）时调用一次，结算掉电记忆槽位。
// 只有真正到位才记录；FAULT 时记 0，下次上电 bootMode=2 会退回槽 1 而不是回到坏位置。
void Protocol::noteMotionComplete() {
  if (g_motion.isFault()) { notePositionChanged(0); g_settings.save(); return; }
  const int s = currentSlot();
  notePositionChanged(s);
  g_settings.save();
}

// ---------------------------------------------------------------------------
// 服务台注入：等价于上位机发来一条命令，便于上机自检
// ---------------------------------------------------------------------------
void Protocol::injectCommand(const char *twoChars) {
  if (!twoChars || strlen(twoChars) != 2) return;
  handle(twoChars[0], twoChars[1]);
}

// ---------------------------------------------------------------------------
// 当前槽位：由编码器实测角反推。传感器不可信时返回 0（未知）。
// ---------------------------------------------------------------------------
int Protocol::currentSlot() const {
  if (!g_encoder.healthy()) return 0;
  return g_settings.slotForAngle(g_encoder.angleDeg());
}

int Protocol::offsetUnits(int slot) const {
  if (slot < 1 || slot > FW_FILTER_SLOTS) return 0;
  return (int)g_settings.s().offsetCnt[slot - 1];
}

void Protocol::tick() {
  // ---- 1. 接收：2 字节定长命令，无结束符 ----
  while (FW_PROTO_SERIAL.available() > 0) {
    const int c = FW_PROTO_SERIAL.read();
    if (c < 0) break;
    _bytesIn++;

    if (_rxState == 0) {
      if (c <= 32) continue;                 // 丢弃 CR/LF/空格等控制字符
      _c1 = (char)c;
      _rxState = 1;
      continue;
    }
    // 第二字节
    if (c <= 32) {                           // 参数位来了控制字符 -> 整帧作废
      _badFrames++;
      _rxState = 0;
      continue;
    }
    _rxState = 0;
    handle(_c1, (char)c);
  }

  // ---- 2. 校准状态机推进 + 异步 Done 上报 ----
  if (_calibrating) {
    g_calibrate.tick();
    if (!g_calibrate.running() && millis() - _calStartMs >= CALIB_DONE_LEAD_MS) {
      pushCalibrationDone();
    }
  }
}

void Protocol::pushCalibrationDone() {
  _calibrating = false;
  // 驱动靠 strstr(szResp,"Done") 判定结束，并把当前位置硬记成槽 1。
  // 失败时同样必须含 Done —— 这是协议里唯一的结束信号，不发就等于让上位机永久等待。
  // 失败原因不靠这条报文传达，改由后续 I2/I6 的真实数值体现，不编造成功。
  if (g_calibrate.ok()) reply("Calibration Done");
  else reply("Calibration Done (failed)");
  g_settings.save();
}

// ---------------------------------------------------------------------------
// 命令分发
// ---------------------------------------------------------------------------
void Protocol::handle(char c1, char c2) {
  _cmds++;
  const char cmd = (char)toupper((unsigned char)c1);

  // S 的参数字符允许 0-9 / A-F（驱动用 %X 格式化），其余命令参数必须是数字
  if (cmd == 'S') { cmdSpeed(c2); return; }

  if (!isdigit((unsigned char)c2)) {
    _badFrames++;
    reply("Command Unknown");
    return;
  }
  const int number = c2 - '0';

  switch (cmd) {
    case 'G': cmdGo(number); break;
    case 'F': cmdRelativeMove(number, true); break;
    case 'B': cmdRelativeMove(number, false); break;
    case 'I': cmdInfo(number); break;
    case 'O': cmdOffsetQuery(number); break;
    case 'T': cmdSensors(number); break;
    case '(': cmdOffset(number, true); break;
    case ')': cmdOffset(number, false); break;
    case ']': cmdJitter(true); break;
    case '[': cmdJitter(false); break;
    case '}': cmdThreshold(true); break;
    case '{': cmdThreshold(false); break;
    case 'M': cmdPulseWidth(true); break;
    case 'N': cmdPulseWidth(false); break;
    case 'R': cmdReset(number); break;
    default:
      _badFrames++;
      reply("Command Unknown");
      break;
  }
}

// ---------------------------------------------------------------------------
// 信息查询 I0..I9
// ---------------------------------------------------------------------------
void Protocol::cmdInfo(int n) {
  char buf[40];
  const int slot = currentSlot();

  switch (n) {
    case 0: reply(FW_MODEL_STRING); break;
    case 1: reply(FW_PROTO_VERSION_STR); break;

    case 2:
      // 驱动语义： strstr(resp,"Moving") -> 仍在走位；否则 sscanf("P%d")
      if (g_motion.isMoving()) { reply("Moving"); break; }
      if (g_motion.isFault())  { reply("Moving error"); break; }   // 让驱动继续轮询并最终超时
      snprintf(buf, sizeof(buf), "P%d", slot);
      reply(buf);
      break;

    case 3: reply("S/N:FW0001"); break;

    case 4:
      // 驱动模板 "MaxSpeed %d"，中间无空格。SBIG 固件写 "Max Speed 400" 会解析失败。
      snprintf(buf, sizeof(buf), "MaxSpeed %d", g_motion.pctSpeed());
      reply(buf);
      break;

    case 5:
      snprintf(buf, sizeof(buf), "Jitter %d", _jitter);
      reply(buf);
      break;

    case 6: {
      const int s = slot ? slot : 1;
      snprintf(buf, sizeof(buf), "P%d Offset %d", s, offsetUnits(s));
      reply(buf);
      break;
    }

    case 7:
      snprintf(buf, sizeof(buf), "Threshold %d", _threshold);
      reply(buf);
      break;

    case 8:
      snprintf(buf, sizeof(buf), "FilterSlots %d", (int)FW_FILTER_SLOTS);
      reply(buf);
      break;

    case 9:
      // 直驱方案没有「位置脉宽」这一概念。报 0 使驱动 hasPulseWidthControl()=false，
      // 于是它永远不会发 M0/N0 —— 用能力协商代替虚假实现，比回一个假数字更诚实。
      reply("Pulse Width 0mS");
      break;

    default: reply("Command Unknown"); break;
  }
}

void Protocol::cmdOffsetQuery(int n) {
  char buf[40];
  if (n >= 1 && n <= FW_FILTER_SLOTS) {
    snprintf(buf, sizeof(buf), "P%d Offset %d", n, offsetUnits(n));
    reply(buf);
  }
  // n 越界时不回应答：驱动同样不解析，多余字节只会被下一次 purgeTxRx 丢弃
}

// ---------------------------------------------------------------------------
// 定位
// ---------------------------------------------------------------------------
void Protocol::cmdGo(int n) {
  // 走位启动前先结算上一次的位置记账
  notePositionChanged(currentSlot());

  if (n < 1 || n > FW_FILTER_SLOTS) {
    // 越界：回当前真实位置。驱动会按「还没到」继续轮询，25 秒后自行报超时。
    // 不回英文错误串（那会让等待 P 的主机无所适从），也不伪造 P<n>。
    char buf[16];
    snprintf(buf, sizeof(buf), "P%d", currentSlot());
    reply(buf);
    return;
  }
  if (!g_motion.replanToSlot(n)) {
    char buf[16];
    snprintf(buf, sizeof(buf), "P%d", currentSlot());
    reply(buf);
    return;
  }
#if FW_PROTO_REPLY_ON_MOVE
  reply("Moving");
#endif
}

void Protocol::cmdRelativeMove(int n, bool forward) {
  // B/F 是 ASCOM 风格的相对进退。Xagyl 驱动本身不发这两条，
  // 保留实现是为了兼容共用同一台设备的 ASCOM/INDI 通用滤镜轮驱动。
  int cur = currentSlot();
  if (cur == 0) cur = 1;                       // 位置未知时以槽 1 为基准
  int target = cur;
  const int steps = n > 4 * FW_FILTER_SLOTS ? 4 * FW_FILTER_SLOTS : n;  // 防呆，避免死循环
  for (int i = 0; i < steps; ++i) {
    target = forward ? (target >= FW_FILTER_SLOTS ? 1 : target + 1)
                     : (target <= 1 ? FW_FILTER_SLOTS : target - 1);
  }
  cmdGo(target);
}

// ---------------------------------------------------------------------------
// 偏移微调：一次 ±1 单位，并立刻重新对中
// ---------------------------------------------------------------------------
void Protocol::cmdOffset(int n, bool increase) {
  char buf[40];
  (void)n;   // 协议语义是「当前槽位」，数字只是占位符

  int slot = currentSlot();
  if (slot == 0) slot = g_motion.targetSlot();
  if (slot < 1 || slot > FW_FILTER_SLOTS) slot = 1;

  const int v = g_settings.addOffsetUnits(slot, increase ? 1 : -1);
  g_settings.save();                       // 天文台使用习惯：校准值必须掉电不丢

  // 偏移变了就必须重新对中，否则设定值与实际位置脱节
  g_motion.replanToSlot(slot);

  snprintf(buf, sizeof(buf), "P%d Offset %d", slot, v);
  reply(buf);
}

// ---------------------------------------------------------------------------
// jitter / threshold：在 SBIG 固件里只是被读出来的字符串。
// 这里给了可验证的真实语义，映射关系写进文档，避免「看起来能调其实没作用」。
//   threshold  每 +1 放宽 0.25°，基准 1 = FW_POSITION_TOL_DEG
//   jitter     每 +1 多给一次纠错重试，基准 1 = FW_MOVE_MAX_RETRY
// ---------------------------------------------------------------------------
void Protocol::cmdJitter(bool increase) {
  char buf[24];
  int v = (int)_jitter + (increase ? 1 : -1);
  if (v < 0) v = 0;
  if (v > 8) v = 8;
  _jitter = (int8_t)v;
  g_motion.setRetryBudget((uint8_t)(FW_MOVE_MAX_RETRY + (_jitter > 1 ? _jitter - 1 : 0)));
  snprintf(buf, sizeof(buf), "Jitter %d", _jitter);
  reply(buf);
}

void Protocol::cmdThreshold(bool increase) {
  char buf[24];
  int v = (int)_threshold + (increase ? 1 : -1);
  if (v < 1) v = 1;
  if (v > 40) v = 40;
  _threshold = (int8_t)v;
  float tol = (float)FW_POSITION_TOL_DEG + 0.25f * (float)(v - 1);
  if (tol > 30.0f) tol = 30.0f;
  g_settings.s().positionTolDeg = tol;
  g_settings.markDirty();
  g_settings.save();
  snprintf(buf, sizeof(buf), "Threshold %d", _threshold);
  reply(buf);
}

void Protocol::cmdPulseWidth(bool increase) {
  // 能力协商：本设备不支持脉宽调节（I9 报 0）。
  // 若旧驱动或手工终端仍发来，保持幂等并回同一个值，明确告知数值不变。
  (void)increase;
  reply("Pulse Width 0mS");
}

// ---------------------------------------------------------------------------
// 转速 S<hex>：本协议中唯一参数可为字母的命令
// ---------------------------------------------------------------------------
void Protocol::cmdSpeed(char paramChar) {
  int nib = -1;
  if (paramChar >= '0' && paramChar <= '9') nib = paramChar - '0';
  else {
    const char u = (char)toupper((unsigned char)paramChar);
    if (u >= 'A' && u <= 'F') nib = 10 + (u - 'A');
  }
  if (nib < 0) { _badFrames++; reply("Command Unknown"); return; }

  // 按固件版本 ≥4.2 的口径解释为 16 档（0..F ↔ 0..100%）
  const int pct = (int)lroundf((float)nib / 15.0f * 100.0f);
  const uint32_t lo = (uint32_t)FW_SPEED_MIN_HZ;
  const uint32_t hi = (uint32_t)FW_SPEED_MAX_HZ;
  const uint16_t hz = (uint16_t)(lo + (hi - lo) * (uint32_t)pct / 100UL);
  g_motion.setSpeedHz(hz);
  g_settings.save();

  char buf[24];
  // 回读驱动模板 "MaxSpeed %d"：驱动据此判断是否达成，未达成会按新公式重发一次
  snprintf(buf, sizeof(buf), "MaxSpeed %d", g_motion.pctSpeed());
  reply(buf);
}

// ---------------------------------------------------------------------------
// T0：传感器原始值，格式 "Sensors %d %d"
//
// 诚实说明：SBIG 原机有一对光学槽位传感器。本方案只有一只磁编码器，
// 因此两个字段取「编码器 14 位原始计数」与「磁场状态码」——
// 都是有物理意义的诊断量，不是凑数的假数据。文档里必须讲清这个差异。
// ---------------------------------------------------------------------------
void Protocol::cmdSensors(int n) {
  char buf[40];
  const EncoderStatus &st = g_encoder.status();
#if FW_PROTO_T0_SENSORS
  (void)n;
  snprintf(buf, sizeof(buf), "Sensors %d %d", (int)st.lastRaw, (int)st.fieldCode);
#else
  (void)n;
  snprintf(buf, sizeof(buf), "%d", (int)st.lastRaw);
#endif
  reply(buf);
}

// ---------------------------------------------------------------------------
// 复位族 R0..R7
// ---------------------------------------------------------------------------
void Protocol::cmdReset(int n) {
  char buf[40];
  switch (n) {
    case 0:
    case 1:
      // 真实复位（SBIG 系固件只回字符串、不复位）。
      // ESP32 重启约几百毫秒，先回应答再延时，给主机一次干净的重连窗口。
      reply("Reset");
      delay(120);
      _rxState = 0;
      g_settings.save();
      ESP.restart();
      break;

    case 2: {                                     // 清全部偏移并回槽 1
      for (int i = 1; i <= FW_FILTER_SLOTS; ++i) g_settings.setOffsetUnits(i, 0);
      g_settings.save();
      g_motion.replanToSlot(1);
      reply("Calibration Removed");
      break;
    }

    case 3:
      snprintf(buf, sizeof(buf), "Jitter %d", _jitter);
      reply(buf);
      break;

    case 4:
      snprintf(buf, sizeof(buf), "MaxSpeed %d", g_motion.pctSpeed());
      reply(buf);
      break;

    case 5:
      snprintf(buf, sizeof(buf), "Threshold %d", _threshold);
      reply(buf);
      break;

    case 6:                                       // R6 = 启动校准（Xagyl 语义）
      cmdStartCalibration();
      break;

    case 7: {                                     // R7 = 恢复出厂默认（驱动仅在版本 ≥4.2 才发）
      g_settings.setDefault();
      g_settings.markDirty();
      g_settings.save();
      g_motion.setMicrosteps(g_settings.s().microsteps);
      g_motion.setSpeedHz(g_settings.s().speedMaxHz);
      _jitter = 1;
      _threshold = 1;
      g_motion.setRetryBudget(FW_MOVE_MAX_RETRY);
      reply("Defaults Restored");
      break;
    }

    default: reply("Command Unknown"); break;
  }
}

void Protocol::cmdStartCalibration() {
  if (_calibrating) { reply("Calibrating"); return; }   // 重复触发保持幂等

  if (g_motion.isMoving()) g_motion.holdNow();          // 校准要求静止起步

  if (!g_calibrate.start()) {
    reply("Calibration Failed");      // 不含 "Calibrating" -> 驱动判命令失败，符合预期
    return;
  }
  _calibrating = true;
  _calStartMs = millis();
  reply("Calibrating");
}
