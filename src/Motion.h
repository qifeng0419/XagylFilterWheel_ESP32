// ============================================================================
//  Motion.h —— 开环走位 + 绝对编码器闭环校验的运动层
//
//  为什么不做连续 PID 闭环：
//    滤镜轮是「点对点」定位机构，绝大多数时间在静止。周期性跑 PID 收益极低，
//    还会引入积分漂移。这里采用更常见的分段做法：
//      以编码器实测角为起点开环走位 -> 沉降 -> 编码器验位
//      -> 超差则以低速补走残差 -> 仍超差进 FAULT
//    步进丢步（机械卡阻、碰撞、人为扳动）能被检出，并在同一次移动内纠正。
//
//  脉冲调度：
//    tick() 内用 micros() 逐脉冲计算周期，并受「单次调用最长占用时间」约束，
//    保证 9600 波特率的协议口不会因为发脉冲而丢字节。
//    恒加速段的频率-位移关系：f(i)^2 = f_min^2 + 2*a*x，
//    起步用「已走步数」、停止用「剩余步数」，两者取小再与 f_max 取小，
//    一个表达式同时覆盖加速 / 巡航 / 减速。
//    需要 >20 kHz 脉冲频率时请改用 LEDC/RMT 外设，见 README「进阶」。
//
//  状态机：IDLE -> MOVING -> SETTLING -> VERIFYING -> IDLE / FAULT
// ============================================================================
#pragma once

#include <Arduino.h>
#include "Config.h"

class Motion {
 public:
  enum State : uint8_t { IDLE, MOVING, SETTLING, VERIFYING, FAULT };

  void begin();

  // 移动到槽位 slot（1..FW_FILTER_SLOTS），目标角来自 Settings 角度表
  bool startMoveToSlot(int slot);
  // 移动到任意绝对角（服务台、自动校准用）
  bool startMoveToAngle(float targetDeg);
  // 相对当前实测角转 deltaDeg（正 = 编码器角增大方向）
  bool startMoveRelative(float deltaDeg);

  // 协议专用：正在走位时也重新规划到目标槽。
  // Xagyl 的 G 命令语义允许打断上一次走位（驱动靠轮询 I2 收敛，不依赖命令应答），
  // 返回 false 表示槽位号非法。
  bool replanToSlot(int slot);

  // 主循环每轮调用；内部自行限时长
  void tick();

  bool  isMoving() const { return _state == MOVING || _state == SETTLING || _state == VERIFYING; }
  bool  isFault()  const { return _state == FAULT; }
  State state() const { return _state; }
  const char *stateText() const;
  const char *faultReason() const { return _faultReason; }

  void abort(const char *reason);

  int    targetSlot() const { return _targetSlot; }
  float  targetDeg() const { return _targetDeg; }
  float  lastErrorDeg() const { return _lastErrDeg; }
  uint32_t stepsEmitted() const { return _stepsEmitted; }
  uint32_t stepsPlanned() const { return _stepsToGo; }
  uint16_t currentHz() const { return _curHz; }
  uint8_t  retryCount() const { return _retry; }

  void     setSpeedHz(uint16_t hz);
  uint16_t speedHz() const { return _speedMaxHz; }
  // 把当前巡航频率折算回协议的 0..100（量程 = FW_SPEED_MIN_HZ..FW_SPEED_MAX_HZ）
  int      pctSpeed() const;

  // 协议 ]0 / [0 会调大/调小纠错重试预算
  void     setRetryBudget(uint8_t n) { _retryBudget = n; }
  uint8_t  retryBudget() const { return _retryBudget; }

  // 微步改变会改变每圈脉冲数，必须同步给驱动与运动层
  void      setMicrosteps(uint16_t microsteps);
  uint16_t  microsteps() const { return _microsteps; }

  uint32_t stepsPerRev() const {
    return (uint32_t)FW_MOTOR_STEPS_PER_REV * (uint32_t)_microsteps;
  }
  int32_t degToSteps(float deg) const;
  float   stepsToDeg(int32_t steps) const {
    return ((float)steps / (float)stepsPerRev()) * 360.0f;
  }

  // 立即收尾（校准流程逐步逼近时用到）：停止发脉冲并进入验位
  float holdNow();

 private:
  void planMove(float targetAbsDeg, int slot);
  void startRampFor(int32_t deltaSteps);
  void beginSettle();
  void verify();

  State   _state = IDLE;
  int     _targetSlot = 0;
  float   _targetDeg = 0.0f;    // 连续量目标角（相对起点做过 unwrap）
  float   _startDeg = 0.0f;

  int32_t  _deltaSteps = 0;
  uint32_t _stepsToGo = 0;
  uint32_t _stepsEmitted = 0;
  uint32_t _nextPulseUs = 0;
  uint16_t _curHz = 0;

  uint32_t _moveStartMs = 0;
  uint32_t _settleStartMs = 0;
  uint8_t  _retry = 0;
  uint8_t  _retryBudget = FW_MOVE_MAX_RETRY;

  float    _lastErrDeg = 0.0f;
  uint16_t _speedMaxHz = FW_SPEED_MAX_HZ;
  uint16_t _microsteps = FW_MICROSTEPS;   // 256 无法用 uint8_t 表示

  const char *_faultReason = "";
};

extern Motion g_motion;
