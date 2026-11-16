// ============================================================================
//  Calibrate.h —— R6 自动校准（逐槽偏差测量与补偿）
//
//  校准做什么：
//    对每个槽位 i：
//      1) 走到位（走位本身含闭环验位）
//      2) 读编码器实测角 actual，计算残差 dev = signedDelta(actual, nominal)
//      3) 把 dev 累加进该槽 offsetCnt，即 offset_new = offset_old + dev
//         —— 消化齿轮间隙、装配偏心、磁体安装偏差造成的系统性到位误差
//    全部槽走完后回到槽 1。
//
//  校准不做什么（必须如实说明）：
//    · 本方案只有一个磁编码器、没有第二套槽位识别传感器（SBIG 原厂靠轮上的
//      磁铁+霍尔），因此校准**无法自行发现「哪个角度是槽中心」**。
//      槽中心仍由 homeAngleDeg + slotSpacingDeg 决定，需先用服务台
//      `sethome` / `setspace` 或 `teach` 流程标定一次。
//    · 重复执行是安全的：若上次已补偿，本次 dev≈0，不会发散。
//      想彻底重来先做 R2（清偏移）。
// ============================================================================
#pragma once

#include <Arduino.h>

class Calibrate {
 public:
  enum Phase : uint8_t { IDLE, TRAVEL, MEASURE, RETURN, FINISHED, FAILED };

  // 启动。返回 false 表示前置条件不满足（编码器不可信 / 槽位配置非法）
  bool start();

  // 主循环调用，非阻塞推进状态机
  void tick();

  bool running() const { return _phase == TRAVEL || _phase == MEASURE || _phase == RETURN; }
  bool ok() const { return _phase == FINISHED; }
  Phase phase() const { return _phase; }
  int  currentStep() const { return _step; }
  int  totalSteps() const { return _total; }
  float worstResidualDeg() const { return _worst; }
  const char *phaseText() const;
  const char *failReason() const { return _failReason; }

 private:
  void finish(bool success, const char *why);

  Phase  _phase = IDLE;
  int    _step = 0;
  int    _total = 0;
  float  _worst = 0.0f;
  const char *_failReason = "";
};

extern Calibrate g_calibrate;
