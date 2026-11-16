// ============================================================================
//  Motion.cpp —— 梯形脉冲规划 + 绝对角闭环校验
//
//  闭环策略：
//    1) 起步前读编码器实测角作为起点（找回丢步/被扳动的关键，不能只信上次命令值）
//    2) 角度差 -> 脉冲数，走梯形曲线；长移动中周期复查驱动故障位
//    3) SETTLING 等待机械振荡衰减
//    4) VERIFYING 读编码器，|误差| > tol 则低速补走残差，最多 FW_MOVE_MAX_RETRY 次
//    5) 仍超差 -> FAULT。绝不谎报到位。
// ============================================================================
#include "Motion.h"
#include "Encoder.h"
#include "Settings.h"
#include "StepperDriver.h"
#include <math.h>

Motion g_motion;

namespace {
// STEP 高电平脉宽。TMC2209 在 dedge=0 时要求 ≥ 125 ns（tSW），开 multistep_filt 后
// 建议 ≥ 1 µs；这里取 2 µs。
constexpr uint16_t STEP_PULSE_US = 2;
// 单次 tick() 最多占用时间。9600 波特一个字符 ≈ 1.04 ms，
// 只要远小于它就不会丢协议字节。
constexpr uint32_t MAX_TICK_BUDGET_US = 250;
constexpr float    MIN_FREQ_HZ = 12.0f;

inline float clampF(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

// 梯形频率 -> 周期（µs）。i = 已走步数，r = 剩余步数。
inline uint32_t periodFor(uint32_t i, uint32_t r, float fMin, float fMax, float accelHzS) {
  const float fMin2 = fMin * fMin;
  const float fa = fMin2 + 2.0f * accelHzS * (float)i;
  const float fb = fMin2 + 2.0f * accelHzS * (float)r;
  float f2 = (fa < fb) ? fa : fb;
  const float fMax2 = fMax * fMax;
  if (f2 > fMax2) f2 = fMax2;
  float f = sqrtf(f2);
  if (f < MIN_FREQ_HZ) f = MIN_FREQ_HZ;
  float p = 1000000.0f / f;
  const float pMin = (float)(2 * STEP_PULSE_US + 1);
  if (p < pMin) p = pMin;
  return (uint32_t)p;
}

uint16_t nearestPow2(uint16_t v) {
  uint16_t ms = 1;
  while (ms < 256 && (uint32_t)ms * 2 <= v) ms = (uint16_t)(ms * 2);
  return ms;
}
}  // namespace

const char *Motion::stateText() const {
  switch (_state) {
    case IDLE:      return "IDLE";
    case MOVING:    return "MOVING";
    case SETTLING:  return "SETTLING";
    case VERIFYING: return "VERIFYING";
    case FAULT:     return "FAULT";
  }
  return "?";
}

int32_t Motion::degToSteps(float deg) const {
  const float s = deg * (float)stepsPerRev() / 360.0f;
  return (int32_t)lroundf(s);
}

void Motion::begin() {
  _microsteps = g_settings.s().microsteps ? g_settings.s().microsteps : (uint16_t)FW_MICROSTEPS;
  _speedMaxHz = g_settings.s().speedMaxHz ? g_settings.s().speedMaxHz : (uint16_t)FW_SPEED_MAX_HZ;
  _retryBudget = FW_MOVE_MAX_RETRY;
  _targetSlot = 0;
  _targetDeg = 0.0f;
  _state = IDLE;
}

void Motion::setMicrosteps(uint16_t microsteps) {
  const uint16_t ms = nearestPow2(microsteps);
  if (ms == _microsteps) return;
  _microsteps = ms;
  g_stepper.setMicrosteps(ms);
  g_settings.s().microsteps = ms;
  g_settings.markDirty();
  g_settings.save();
}

void Motion::setSpeedHz(uint16_t hz) {
  _speedMaxHz = (uint16_t)clampF((float)hz, 50.0f, 20000.0f);
  g_settings.s().speedMaxHz = _speedMaxHz;
  g_settings.markDirty();
}

// ---------------------------------------------------------------------------
// 规划：以编码器实测角为起点
// ---------------------------------------------------------------------------
void Motion::planMove(float targetAbsDeg, int slot) {
  const float measured = g_encoder.healthy() ? g_encoder.angleDeg() : _targetDeg;
  _startDeg = measured;
  _targetDeg = Encoder::unwrap(targetAbsDeg, measured);
  _targetSlot = slot;
  _retry = 0;
  _lastErrDeg = 0.0f;
  _faultReason = "";
  startRampFor(degToSteps(Encoder::signedDelta(_targetDeg, _startDeg)));
}

void Motion::startRampFor(int32_t deltaSteps) {
  _deltaSteps = deltaSteps;
  _stepsToGo = (uint32_t)labs(deltaSteps);
  _stepsEmitted = 0;
  _curHz = 0;

  if (_stepsToGo == 0) {              // 本就在目标附近（不足半步），直接验位
    beginSettle();
    return;
  }

  // 方向：dirInvert=0 时 DIR 高 = 编码器角增大方向
  const bool dirHigh = (deltaSteps > 0) ? (g_settings.s().dirInvert == 0)
                                        : (g_settings.s().dirInvert != 0);
  digitalWrite(FW_PIN_DIR, dirHigh ? HIGH : LOW);
  delayMicroseconds(10);              // DIR -> STEP 建立时间，留足余量

  _state = MOVING;
  _moveStartMs = millis();
  _nextPulseUs = micros();
}

void Motion::tick() {
  if (_state == MOVING) {
    const float fMin = clampF((float)FW_SPEED_MIN_HZ, MIN_FREQ_HZ, (float)FW_SPEED_MAX_HZ);
    const float fMax = clampF((float)_speedMaxHz, fMin + 1.0f, 20000.0f);
    const float accel = clampF((float)FW_ACCEL_HZ_PER_S, 100.0f, 200000.0f);

    const uint32_t tBegin = micros();
    while (_stepsEmitted < _stepsToGo) {
      uint32_t now = micros();
      if ((int32_t)(now - _nextPulseUs) < 0) break;    // 还没到下一脉冲时刻

      digitalWrite(FW_PIN_STEP, HIGH);
      delayMicroseconds(STEP_PULSE_US);
      digitalWrite(FW_PIN_STEP, LOW);
      _stepsEmitted++;

      const uint32_t r = _stepsToGo - _stepsEmitted;
      const uint32_t period = periodFor(_stepsEmitted, r, fMin, fMax, accel);
      _curHz = (uint16_t)(1000000UL / (period ? period : 1UL));
      _nextPulseUs += period;

      // 每 256 步复查一次：编码器健康 + 驱动故障位
      if ((_stepsEmitted & 255u) == 0u) {
        g_encoder.update();
        if (g_stepper.driverOverTemp())      { abort("TMC over-temperature"); return; }
        if (g_stepper.driverShortToGround()) { abort("TMC short-to-ground"); return; }
      }

      if ((uint32_t)(micros() - tBegin) >= MAX_TICK_BUDGET_US) break;   // 让出 CPU 给串口
    }

    if (_stepsEmitted >= _stepsToGo) { beginSettle(); return; }

    if (millis() - _moveStartMs > (unsigned long)FW_MOVE_TIMEOUT_MS) {
      abort("move timeout");
    }
    return;
  }

  if (_state == SETTLING) {
    if (millis() - _settleStartMs >= (unsigned long)FW_SETTLE_MS) verify();
    return;
  }
}

void Motion::beginSettle() {
  _state = SETTLING;
  _settleStartMs = millis();
  _curHz = 0;
  g_encoder.update();
}

void Motion::verify() {
  g_encoder.update();
  if (!g_encoder.healthy()) {
    _state = FAULT;
    _faultReason = "encoder read failed";
    return;
  }

  const float actual = g_encoder.angleDeg();
  const float err = Encoder::signedDelta(actual, _targetDeg);
  _lastErrDeg = err;

  const float tol = g_settings.s().positionTolDeg > 0.0f ? g_settings.s().positionTolDeg
                                                          : (float)FW_POSITION_TOL_DEG;
  if (fabsf(err) <= tol) {
    _state = IDLE;
    return;
  }

  if (_retry < _retryBudget) {
    _retry++;
    const int32_t need = degToSteps(-err);
    if (need == 0) {                  // 残差不足一步：已是最优可达位置
      _state = IDLE;
      return;
    }
    // 补走残差。_targetDeg 保持原值，只把起点重设到实测角，
    // 避免误差累积到下一次判断。
    _startDeg = actual;
    _targetDeg = Encoder::unwrap(_targetDeg, actual);
    startRampFor(need);
    return;
  }

  _state = FAULT;
  _faultReason = "position not reached within tolerance";
}

void Motion::abort(const char *reason) {
  _state = FAULT;
  _faultReason = reason ? reason : "aborted";
  _stepsToGo = _stepsEmitted;
  _curHz = 0;
  g_stepper.enable(false);            // 断电，避免继续发热
  delay(2);
  g_stepper.enable(true);
}

float Motion::holdNow() {
  if (_state == MOVING) _stepsToGo = _stepsEmitted;   // 让 tick 收尾进 SETTLING
  g_encoder.update();
  return g_encoder.angleDeg();
}

bool Motion::startMoveToSlot(int slot) {
  if (slot < 1 || slot > FW_FILTER_SLOTS) return false;
  if (isMoving()) return false;
  planMove(g_settings.slotAngle(slot), slot);
  return true;
}

bool Motion::replanToSlot(int slot) {
  if (slot < 1 || slot > FW_FILTER_SLOTS) return false;
  // 正在走位时也允许重新规划：以当前实测角为新起点重新起跳。
  // Xagyl 的 G 命令语义允许打断上一次走位（驱动靠轮询 I2 收敛，不等命令应答）。
  planMove(g_settings.slotAngle(slot), slot);
  return true;
}

int Motion::pctSpeed() const {
  const uint32_t lo = (uint32_t)FW_SPEED_MIN_HZ;
  const uint32_t hi = (uint32_t)FW_SPEED_MAX_HZ;
  if (hi <= lo) return 100;
  uint32_t v = _speedMaxHz;
  if (v < lo) v = lo;
  if (v > hi) v = hi;
  return (int)((v - lo) * 100UL / (hi - lo));
}

bool Motion::startMoveToAngle(float targetDeg) {
  if (isMoving()) return false;
  float a = fmodf(targetDeg, 360.0f);
  if (a < 0.0f) a += 360.0f;
  planMove(a, 0);                     // slot 0 = 非标准槽位（服务台/校准）
  return true;
}

bool Motion::startMoveRelative(float deltaDeg) {
  if (isMoving()) return false;
  const float base = g_encoder.healthy() ? g_encoder.angleDeg() : _targetDeg;
  planMove(base + deltaDeg, 0);
  return true;
}
