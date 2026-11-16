// ============================================================================
//  Encoder.h —— MT6701 14 位磁编码器抽象层（I2C / SSI 双后端）
//
//  设计要点：
//    · 绝对值编码器，掉电不丢位置 —— 「初始位置可调」因此不必每次上电归零，
//      只需维护一份可编辑的「槽位角度表」。
//    · 角度统一归一到 [0, 360)。跨 0 点由 unwrap()/signedDelta() 处理，
//      否则 359.5° -> 0.5° 会被算成倒转 359°。
//    · 连续 5 次读失败才判定硬件故障（与 ESPHome 的 MAX_CONSECUTIVE_ERRORS 一致），
//      单次误码由内部重试吸收，不影响协议实时性。
//    · 未上机验证的信息不做臆断：SSI 帧的 4 位磁场状态原样上报为 fieldCode，
//      不在固件里猜它的高度/强度编码表。
// ============================================================================
#pragma once

#include <Arduino.h>
#include "Config.h"

struct EncoderStatus {
  bool     ok;                  // 最近一次读取是否成功
  bool     crcChecked;          // 本次构建是否启用了 SSI CRC 校验
  bool     crcBad;              // 最近一次 SSI 帧 CRC 不匹配
  uint8_t  fieldCode;           // SSI 帧 bits14-17 原始磁场状态码（I2C 后端恒 0）
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

  // 以 reference 为起点把 deg 展开成连续量，满足 |结果 - reference| <= 180
  static float unwrap(float deg, float reference);
  // 有符号角差 (a - b)，落在 (-180, +180]
  static float signedDelta(float a, float b);

  // CRC-6，多项式 x^6+x+1（生成多项式低位形式 0x03），初值 0，无前/后置反。
  // frame24 为整帧 24 位；lsbFirst 选择「手册 bit0 优先」还是「线上首位=bit23」的解读。
  static uint8_t crc6(uint32_t frame24, bool lsbFirst);

 private:
  bool readI2C(uint16_t &count);
  bool readSSI(uint16_t &count, uint8_t &fieldCode, bool &crcOk);

  void commit(uint16_t count, uint8_t fieldCode);

  float          _lastDeg = 0.0f;
  EncoderStatus  _st = {false, (FW_ENCODER_BUS == FW_ENCODER_BUS_SPI) && FW_SSI_CHECK_CRC,
                        false, 0, 0, 0, 0, 0};
};

extern Encoder g_encoder;
