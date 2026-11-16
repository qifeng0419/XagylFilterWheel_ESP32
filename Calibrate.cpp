// ============================================================================
//  Calibrate.cpp —— 逐槽偏差测量与补偿（非阻塞状态机）
//
//  与运动层的分工：
//    运动层负责「走到并验位」，本模块只负责「逐槽推进 + 记录残差 + 写补偿」。
//    因此校准过程天然继承了闭环纠错与超时保护，不需要重复实现。
// ============================================================================
#include "Calibrate.h"
#include "Motion.h"
#include "Encoder.h"
#include "Settings.h"
#include <math.h>

Calibrate g_calibrate;

const char *Calibrate::phaseText() const {
  switch (_phase) {
    case IDLE:     return "idle";
    case TRAVEL:   return "travel";
    case MEASURE:  return "measure";
    case RETURN:   return "return";
    case FINISHED: return "finished";
    case FAILED:   return "failed";
  }
  return "?";
}

bool Calibrate::start() {
  if (running()) return true;

  if (!g_encoder.healthy()) { _phase = FAILED; _failReason = "encoder not healthy"; return false; }
  if (FW_FILTER_SLOTS < 1 || FW_FILTER_SLOTS > SETTINGS_MAX_SLOTS) {
    _phase = FAILED; _failReason = "invalid slot count"; return false;
  }

  _step = 1;
  _total = FW_FILTER_SLOTS;
  _worst = 0.0f;
  _failReason = "";
  _phase = TRAVEL;
  if (!g_motion.replanToSlot(_step)) { finish(false, "slot 1 unreachable"); return false; }
  return true;
}

void Calibrate::tick() {
  if (_phase == IDLE || _phase == FINISHED || _phase == FAILED) return;

  if (g_motion.isMoving()) return;                 // 等当前这一步走完

  if (_phase == RETURN) {
    _phase = FINISHED;
    _failReason = "";
    g_settings.save();
    return;
  }

  if (_phase == TRAVEL) {
    if (g_motion.isFault()) { finish(false, g_motion.faultReason()); return; }
    _phase = MEASURE;
    // 落到下面立即测量，省一次循环
  }

  // ---- MEASURE ----
  if (_phase != MEASURE) return;

  if (!g_encoder.update() || !g_encoder.healthy()) { finish(false, "encoder read failed"); return; }

  const int slot = _step;
  const float nominal = g_settings.slotAngle(slot);
  const float actual = g_encoder.angleDeg();
  const float dev = Encoder::signedDelta(actual, nominal);

  if (fabsf(dev) > fabsf(_worst)) _worst = dev;

  // 残差 -> 偏移单位数（addOffsetUnits 内部做半间距限幅）
  const int addUnits = (int)lroundf(dev / FW_OFFSET_UNIT_DEG);

  if (addUnits != 0) {
    g_settings.addOffsetUnits(slot, addUnits);
    // 补偿后重走一次确认收敛。若仍不收敛，第二次 dev 会更小，逐步逼近；
    // 收敛判据是 addUnits == 0，不会无限循环（偏移被限幅在半个槽距内）。
    _phase = TRAVEL;
    if (!g_motion.replanToSlot(slot)) { finish(false, "re-plan failed"); return; }
    return;
  }

  if (_step < _total) {
    _step++;
    _phase = TRAVEL;
    if (!g_motion.replanToSlot(_step)) { finish(false, "slot out of range"); return; }
  } else {
    _phase = RETURN;
    if (!g_motion.replanToSlot(1)) { finish(false, "return failed"); return; }
  }
}

void Calibrate::finish(bool success, const char *why) {
  _phase = success ? FINISHED : FAILED;
  _failReason = success ? "" : (why && why[0] ? why : "failed");
}
