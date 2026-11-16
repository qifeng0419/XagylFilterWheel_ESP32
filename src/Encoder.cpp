// ============================================================================
//  Encoder.cpp —— MT6701 读取实现（只负责取数，几何计算在 lib/xagyl_geom）
//
//  取数依据（公开数据手册与上游驱动）：
//    · 14 位，16384 counts / 圈，单圈绝对值，掉电不丢失。
//    · I2C：0x03 = Angle[13:6]，0x04 = Angle[5:0]（左对齐高 6 位），
//      故 count = ((r03 << 8) | r04) >> 2。
//      从机地址默认 0x06，处于 I2C 保留地址区，总线扫描扫不到属正常；备选 0x46。
//    · SSI：只读，24 位/帧 = 14 位角度 + 4 位磁场状态 + 6 位 CRC-6。
//      数据在 SCK 上升沿变化，应在下降沿采样 → SPI Mode 1（ESPHome 亦默认 Mode1/1MHz）。
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
constexpr uint8_t REG_ANGLE_H  = 0x03;   // Angle[13:6]
constexpr uint8_t REG_ANGLE_L  = 0x04;   // Angle[5:0] 左对齐
constexpr uint8_t ENC_MAX_RETRY = 3;
}  // namespace

void Encoder::begin() {
#if FW_ENCODER_BUS == FW_ENCODER_BUS_I2C
  Wire.begin(FW_PIN_I2C_SDA, FW_PIN_I2C_SCL, (uint32_t)FW_I2C_FREQ_HZ);
  delay(20);                       // MT6701 上电到首次访问留稳定时间
#elif FW_ENCODER_BUS == FW_ENCODER_BUS_SPI
  pinMode(FW_PIN_SSI_CS, OUTPUT);
  digitalWrite(FW_PIN_SSI_CS, HIGH);
  // SSI 只用 SCK + MISO + CS；ESP32 硬 SPI 要求 MOSI 是有效 GPIO，不接线即可。
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
  _lastDeg = xagyl_geom::countToDeg(count);
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
  // 24 位帧 = 3 次 8 位传输；最先收到的字节是帧的 bit23..bit16
  SPI.beginTransaction(SPISettings((uint32_t)FW_SSI_FREQ_HZ, MSBFIRST, SPI_MODE1));
  digitalWrite(FW_PIN_SSI_CS, LOW);
  delayMicroseconds(2);
  const uint32_t b0 = SPI.transfer(0x00);
  const uint32_t b1 = SPI.transfer(0x00);
  const uint32_t b2 = SPI.transfer(0x00);
  delayMicroseconds(2);
  digitalWrite(FW_PIN_SSI_CS, HIGH);
  SPI.endTransaction();

  const uint32_t frame = (b0 << 16) | (b1 << 8) | b2;
  const bool lsb = (FW_SSI_LSB_FIRST != 0);

  uint8_t gotCrc = 0;
  xagyl_geom::ssiUnpack(frame, lsb, &count, &fieldCode, &gotCrc);

  _st.crcChecked = (FW_SSI_CHECK_CRC != 0);
#if FW_SSI_CHECK_CRC
  crcOk = (xagyl_geom::ssiCrc6(frame) == gotCrc);
#else
  (void)gotCrc;
  crcOk = true;
#endif
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
    got = readSSI(count, fieldCode, crcOk);   // CRC 不通过则继续重试
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
  return true;
}
