// ============================================================================
//  Encoder.cpp —— MT6701 读取实现
//
//  取数依据（均来自公开数据手册与上游驱动，逐条对应）：
//    · 分辨率 14 位，16384 counts / 圈，单圈绝对值，掉电不丢失。
//    · I2C：角度拆在两个寄存器，0x03 = Angle[13:6]，0x04 = Angle[5:0]（占高 6 位）。
//      故 count = ((r03 << 8) | r04) >> 2。
//      从机地址默认 0x06，处于 I2C 保留地址区，总线扫描扫不到属正常；备选 0x46。
//    · SSI：只读，24 位/帧 = 位 0-13 角度、位 14-17 磁场状态、位 18-23 CRC-6。
//      数据在 SCK 上升沿变化，建议在下降沿采样 → SPI Mode 1；ESPHome 亦默认 Mode1 / 1 MHz。
//      CRC-6 多项式 x^6+x+1，初值 0。
//
//  ⚠ 位序说明（上机需确认一次，用固件自检即可判定）：
//    手册把「最先传出的位」编号为 bit0，而 SPI 硬件把最先收到的位放在收到的最高位。
//    两种编号习惯会导致角度呈现位反转（镜像）差异。本文件用 FW_SSI_LSB_FIRST 一个开关
//    覆盖两种解读，并让 CRC 与角度取自同一位序，因此：
//      置 1 和置 0 各编译一次，看哪个能让 CRC 稳定通过 —— 通过的那个就是正确位序。
//    这是把「无法从文档确认的信息」转成「上机一次可判定」的处理方式，不做臆断。
// ============================================================================
#include "Encoder.h"

#if FW_ENCODER_BUS == FW_ENCODER_BUS_I2C
#include <Wire.h>
#endif
#if FW_ENCODER_BUS == FW_ENCODER_BUS_SPI
#include <SPI.h>
#endif

Encoder g_encoder;

namespace {
constexpr uint8_t  REG_ANGLE_H = 0x03;   // Angle[13:6]
constexpr uint8_t  REG_ANGLE_L = 0x04;   // Angle[5:0] 左对齐在高 6 位
constexpr uint16_t ENC_COUNTS   = 16384;  // 14 bit
constexpr uint8_t  ENC_MAX_RETRY = 3;
constexpr uint8_t  ENC_UNHEALTHY = 5;     // 连续失败阈值

// 反转 n 位。n 最大 14，故用 uint32_t 承载，避免 8 位截断。
uint32_t reverseBits(uint32_t v, uint8_t n) {
  uint32_t r = 0;
  for (uint8_t i = 0; i < n; ++i) {
    r = (uint32_t)((r << 1) | ((v >> i) & 1u));
  }
  return r;
}
}  // namespace

// ---------------------------------------------------------------------------
// CRC-6 / x^6+x+1，逐位处理帧中「最先传出的 18 个位」
// ---------------------------------------------------------------------------
uint8_t Encoder::crc6(uint32_t frame24, bool lsbFirst) {
  uint8_t crc = 0;
  for (uint8_t idx = 0; idx < 18; ++idx) {
    // idx 表示「第 idx 个传出的位」
    uint8_t in;
    if (lsbFirst) {
      // 手册位序：bit0 最先传出 → 它落在 SPI 收到的最高位
      in = (uint8_t)((frame24 >> (23 - idx)) & 1u);
    } else {
      // 直接按位号 idx 取（bit0 为最后传出的位）
      in = (uint8_t)((frame24 >> idx) & 1u);
    }
    const uint8_t feedback = (uint8_t)(((crc >> 5) & 1u) ^ in);
    crc = (uint8_t)(((crc << 1) & 0x3Fu) ^ (feedback ? 0x03u : 0x00u));
  }
  return (uint8_t)(crc & 0x3Fu);
}

float Encoder::unwrap(float deg, float reference) {
  float d = deg - reference;
  while (d >= 180.0f) d -= 360.0f;
  while (d < -180.0f) d += 360.0f;
  return reference + d;
}

float Encoder::signedDelta(float a, float b) {
  float d = a - b;
  while (d >= 180.0f) d -= 360.0f;
  while (d < -180.0f) d += 360.0f;
  return d;
}

void Encoder::begin() {
#if FW_ENCODER_BUS == FW_ENCODER_BUS_I2C
  Wire.begin(FW_PIN_I2C_SDA, FW_PIN_I2C_SCL, (uint32_t)FW_I2C_FREQ_HZ);
  delay(20);                       // MT6701 上电到首次访问留稳定时间
#elif FW_ENCODER_BUS == FW_ENCODER_BUS_SPI
  pinMode(FW_PIN_SSI_CS, OUTPUT);
  digitalWrite(FW_PIN_SSI_CS, HIGH);
  // SSI 只需 SCK + MISO + CS，MOSI 传 0；ESP32 硬 SPI 要求 MOSI 有效，
  // 用一个与 SCK 不同的引脚占位即可（不接线）。
  SPI.begin(FW_PIN_SSI_SCK, FW_PIN_SSI_MISO, FW_PIN_SSI_MOSI, FW_PIN_SSI_CS);
#endif
  // 首次读取失败不阻断启动：健康状态会在后续 update() 中持续反映
  update();
}

void Encoder::commit(uint16_t count, uint8_t fieldCode) {
  _st.lastRaw = count;
  _st.fieldCode = fieldCode;
  _st.ok = true;
  _st.crcBad = false;
  _st.consecutiveErrors = 0;
  _lastDeg = ((float)count / (float)ENC_COUNTS) * 360.0f;
  if (_lastDeg >= 360.0f) _lastDeg -= 360.0f;
}

#if FW_ENCODER_BUS == FW_ENCODER_BUS_I2C
bool Encoder::readI2C(uint16_t &count) {
  // 分两次「写寄存器地址 → 重复起始 → 读 1 字节」，不依赖芯片的地址自增行为
  uint8_t hi = 0, lo = 0;

  Wire.beginTransmission((uint8_t)FW_MT6701_I2C_ADDR);
  Wire.write(REG_ANGLE_H);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)FW_MT6701_I2C_ADDR, (size_t)1) != 1) return false;
  hi = (uint8_t)Wire.read();

  Wire.beginTransmission((uint8_t)FW_MT6701_I2C_ADDR);
  Wire.write(REG_ANGLE_L);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)FW_MT6701_I2C_ADDR, (size_t)1) != 1) return false;
  lo = (uint8_t)Wire.read();

  count = (uint16_t)((((uint16_t)hi << 8) | (uint16_t)lo) >> 2);
  return true;
}
#endif

#if FW_ENCODER_BUS == FW_ENCODER_BUS_SPI
bool Encoder::readSSI(uint16_t &count, uint8_t &fieldCode, bool &crcOk) {
  // 24 位帧：分 3 次 8 位传输，首字节最先收到 → 落在 frame 的 bit23..bit16
  uint32_t frame = 0;
  SPI.beginTransaction(SPISettings((uint32_t)FW_SSI_FREQ_HZ, MSBFIRST, SPI_MODE1));
  digitalWrite(FW_PIN_SSI_CS, LOW);
  delayMicroseconds(2);
  uint8_t b0 = SPI.transfer(0x00);
  uint8_t b1 = SPI.transfer(0x00);
  uint8_t b2 = SPI.transfer(0x00);
  delayMicroseconds(2);
  digitalWrite(FW_PIN_SSI_CS, HIGH);
  SPI.endTransaction();

  frame = ((uint32_t)b0 << 16) | ((uint32_t)b1 << 8) | (uint32_t)b2;

  const bool lsb = (FW_SSI_LSB_FIRST != 0);
  uint32_t angleRaw;
  uint32_t f4;
  uint8_t  gotCrc;

  if (!lsb) {
    // 线上首位 = bit23：角度占 bit23..bit10，磁场状态 bit9..bit6，CRC bit5..bit0
    angleRaw = (frame >> 10) & 0x3FFFu;
    f4       = (frame >> 6) & 0x0Fu;
    gotCrc   = (uint8_t)(frame & 0x3Fu);
  } else {
    // 手册位序（bit0 最先传出）：需要把各字段位反转还原
    angleRaw = reverseBits((frame >> 10) & 0x3FFFu, 14);
    f4       = reverseBits((frame >> 6) & 0x0Fu, 4);
    gotCrc   = (uint8_t)reverseBits(frame & 0x3Fu, 6);
  }

  count = (uint16_t)angleRaw;
  fieldCode = (uint8_t)f4;

#if FW_SSI_CHECK_CRC
  crcOk = (crc6(frame, lsb) == gotCrc);
#else
  (void)gotCrc;
  crcOk = true;
#endif
  _st.crcChecked = (FW_SSI_CHECK_CRC != 0);
  return crcOk;
}
#endif

bool Encoder::update() {
  _st.readsTotal++;

  uint16_t count = _st.lastRaw;
  uint8_t  fieldCode = 0;
  bool     crcOk = true;
  bool     got = false;

  for (uint8_t attempt = 0; attempt < ENC_MAX_RETRY && !got; ++attempt) {
#if FW_ENCODER_BUS == FW_ENCODER_BUS_I2C
    fieldCode = 0;
    crcOk = true;
    got = readI2C(count);
#elif FW_ENCODER_BUS == FW_ENCODER_BUS_SPI
    got = readSSI(count, fieldCode, crcOk);
    // CRC 不通过时继续重试；重试次数用尽则计入失败
#endif
    if (!got) delayMicroseconds(60);
  }

  if (!got) {
    _st.ok = false;
    _st.crcBad = !crcOk;
    if (_st.consecutiveErrors < 250) _st.consecutiveErrors++;
    _st.readsFailed++;
    return false;
  }

  commit(count, fieldCode);
  (void)fieldCode;
  return true;
}
