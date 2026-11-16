// ============================================================================
//  Encoder.h —— MT6701 14 位磁编码器抽象层（I2C / SSI 双后端）
//
//  设计要点：
//    · 绝对值编码器，掉电不丢位置 —— 「初始位置可调」因此不必每次上电归零，
//      只需维护一份可编辑的「槽位角度表」。
//    · 角度计算（归一化、跨 0 点展开、有符号角差、SSI 帧解析与 CRC）
//      全部委托给 lib/xagyl_geom，本文件只负责「从总线取到 14 位计数」。
//      这样单元测试能在 PC 上验证同一份几何实现，不是验证副本。
//    · 连续 5 次读失败才判定硬件故障，单次误码由内部重试吸收。
//    · 不做臆断：SSI 的 4 位磁场状态原样上报为 fieldCode，
//      不在固件里猜它的高低/强弱编码表。
// ============================================================================
#pragma once

#include <Arduino.h>
#include "Config.h"
#include "xagyl_geom.h"

struct EncoderStatus {
  bool     ok;                  // 最近一次读取是否成功
  bool     crcChecked;          // 本次构建是否启用了 SSI CRC 校验
  bool     crcBad;              // 最近一次 SSI 帧 CRC 不匹配
  uint8_t  fieldCode;           // SSI 帧 4 位磁场状态码（I2C 后端恒 0）
  uint8_t  consecutiveErrors;   // 连续失败次数
  uint16_t lastRaw;             // 最近一次有效 14 位计数（0..16383）
  uint32_t readsTotal;
  uint32_t readsFailed;
};

class Encoder {
 public:
  void begin();

  // 读一次绝对角。失败返回 false，但 angleDeg()/raw() 保留上一次有效值。
  bool update();

  float    angleDeg() const { return _lastDeg; }
  uint16_t raw() const { return _st.lastRaw; }
  bool     healthy() const { return _st.ok && _st.consecutiveErrors < 5; }
  const EncoderStatus &status() const { return _st; }

  // ---- 几何计算转发到 xagyl_geom，保留成员名以便既有调用点不用改 ----
  static float    unwrap(float deg, float reference) { return xagyl_geom::unwrap(deg, reference); }
  static float    signedDelta(float a, float b) { return xagyl_geom::signedDelta(a, b); }
  static float    normalizeDeg(float deg) { return xagyl_geom::normalizeDeg(deg); }
  static float    countToDeg(uint16_t count) { return xagyl_geom::countToDeg(count); }
  // SSI CRC-6：只需整帧 24 位，位序无关（见 lib 注释）
  static uint8_t  crc6(uint32_t frame24) { return xagyl_geom::ssiCrc6(frame24); }

 private:
  bool readI2C(uint16_t &count);
  bool readSSI(uint16_t &count, uint8_t &fieldCode, bool &crcOk);

  void commit(uint16_t count, uint8_t fieldCode);

  float          _lastDeg = 0.0f;
  EncoderStatus  _st = {false,
                        (FW_ENCODER_BUS == FW_ENCODER_BUS_SPI) && (FW_SSI_CHECK_CRC != 0),
                        false, 0, 0, 0, 0, 0};
};

extern Encoder g_encoder;
