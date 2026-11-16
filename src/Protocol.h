// ============================================================================
//  Protocol.h —— Xagyl 滤镜轮串口协议设备端实现
//
//  协议契约（逐条对齐 xagyl.cpp 的 sscanf 模板，详见 docs/PROTOCOL.md）：
//    · 链路：9600 8N1，DTR 置位
//    · 命令：2 字节 ASCII，**无结束符**，必须靠定长判帧
//    · 应答：文本 + CR LF。驱动读到 LF 后固定砍掉末尾 2 字节，
//            只发 LF 会吃掉最后一个有效字符 -> 必须发完整 CRLF
//    · 应答文本 ≤ 30 字符（驱动 SERIAL_BUFFER_SIZE = 32）
//
//  与 SBIG 系固件的四处刻意不同（都是驱动侧要求，SBIG 固件其实不合规）：
//    1) I4 回报 "MaxSpeed 100"（无空格）。SBIG 的 "Max Speed 400" 会让
//       驱动 sscanf("MaxSpeed %d") 失配，握手在 connect 第 5 步直接中断。
//    2) I9 回报 "Pulse Width 0mS"。直驱方案没有脉宽概念，报 0 让驱动
//       hasPulseWidthControl()=false，从而永不发 M0/N0 —— 用能力协商代替假实现。
//    3) R6 是「启动校准」，不是 SBIG 的「存 EEPROM」：须立即回含 Calibrating 的文本，
//       结束后自行推送含 Done 的报文 —— 这是本协议唯一的设备主动上报。
//    4) S 命令参数是十六进制（可 A~F）。SBIG 固件的 isDigit 校验会直接拒收 SA..SF。
//    5) T0 回 "Sensors %d %d" 双值。SBIG 固件只回单个数字，驱动解析必然失败。
// ============================================================================
#pragma once

#include <Arduino.h>
#include "Config.h"

class Protocol {
 public:
  void begin();

  // 主循环调用：解析命令 + 推进校准状态机 + 处理异步上报
  void tick();

  // 供服务台复用：发一行协议格式应答
  void reply(const char *line);

  // 供服务台复用：注入一条 2 字符命令，等价于上位机发来
  void injectCommand(const char *twoChars);

  // 运动层完成一次走位后调用，用于掉电记忆槽位
  void noteMotionComplete();

  uint32_t cmdsReceived() const { return _cmds; }
  uint32_t bytesIn() const { return _bytesIn; }
  uint32_t badFrames() const { return _badFrames; }
  uint32_t repliesSent() const { return _replies; }
  bool     calibrating() const { return _calibrating; }

 private:
  void handle(char c1, char c2);

  void cmdInfo(int n);
  void cmdOffsetQuery(int n);
  void cmdGo(int n);
  void cmdRelativeMove(int n, bool forward);
  void cmdOffset(int n, bool increase);
  void cmdJitter(bool increase);
  void cmdThreshold(bool increase);
  void cmdPulseWidth(bool increase);
  void cmdSpeed(char paramChar);
  void cmdSensors(int n);
  void cmdReset(int n);
  void cmdStartCalibration();
  void pushCalibrationDone();

  // 由编码器实测角反推当前槽位，返回 0 表示未知/不在槽位上
  int currentSlot() const;
  int offsetUnits(int slot) const;
  void notePositionChanged(int slot);

  uint8_t _rxState = 0;              // 0=等第 1 字节，1=等第 2 字节
  char    _c1 = 0;

  uint32_t _cmds = 0;
  uint32_t _bytesIn = 0;
  uint32_t _badFrames = 0;
  uint32_t _replies = 0;

  bool     _calibrating = false;
  uint32_t _calStartMs = 0;

  // jitter / threshold 有真实语义（映射见 docs/PROTOCOL.md 第 6 节）
  int8_t _jitter = 1;
  int8_t _threshold = 1;
};

extern Protocol g_protocol;
