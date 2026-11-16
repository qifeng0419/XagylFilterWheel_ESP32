// ============================================================================
//  xagyl_geom.cpp —— 纯计算实现（无硬件依赖）
// ============================================================================
#include "xagyl_geom.h"
#include <math.h>

namespace xagyl_geom {
namespace {

// 反转 n 位（n ≤ 32）
uint32_t reverseBits(uint32_t v, uint8_t n) {
  uint32_t r = 0;
  for (uint8_t i = 0; i < n; ++i) {
    r = (r << 1) | ((v >> i) & 1u);
  }
  return r;
}

}  // namespace

float normalizeDeg(float deg) {
  float a = fmodf(deg, 360.0f);
  if (a < 0.0f) a += 360.0f;
  return a;
}

float countToDeg(uint16_t count) {
  return normalizeDeg(((float)count / (float)ENC_COUNTS) * 360.0f);
}

float unwrap(float deg, float reference) {
  float d = deg - reference;
  while (d >= 180.0f) d -= 360.0f;
  while (d < -180.0f) d += 360.0f;
  return reference + d;
}

float signedDelta(float a, float b) {
  float d = a - b;
  while (d >= 180.0f) d -= 360.0f;
  while (d < -180.0f) d += 360.0f;
  return d;
}

float slotAngleDeg(int slot, float homeDeg, float spacingDeg, float offsetDeg) {
  const float a = homeDeg + (float)(slot - 1) * spacingDeg + offsetDeg;
  return normalizeDeg(a);
}

int nearestSlot(float deg, int nSlots, float homeDeg, float spacingDeg,
                const int16_t *offsetCnt, float offsetUnitDeg, float toleranceDeg) {
  if (nSlots < 1 || spacingDeg <= 0.0f) return 0;
  const float half = spacingDeg * 0.5f;
  const float limit = (toleranceDeg > 0.0f && toleranceDeg < half) ? toleranceDeg : half;

  int best = 0;
  float bestDist = 1e9f;
  for (int i = 1; i <= nSlots; ++i) {
    const float off = (offsetCnt != nullptr) ? (float)offsetCnt[i - 1] * offsetUnitDeg : 0.0f;
    const float center = slotAngleDeg(i, homeDeg, spacingDeg, off);
    const float d = fabsf(signedDelta(deg, center));
    if (d < bestDist) { bestDist = d; best = i; }
  }
  return (bestDist <= limit) ? best : 0;
}

// ---------------------------------------------------------------------------
// SSI CRC-6 / 多项式 x^6+x+1（低位形式 0x03），初值 0，无前/后置反。
//
// 输入按「物理线上顺序」处理：最先收到的位 = 帧的 bit23（SPI 硬件固定如此），
// 前 18 位（bit23..bit6）参与运算。这个顺序与手册用哪种位编号无关，
// 所以两个候选解读共用同一个 CRC 计算，差异只体现在字段拆解（ssiUnpack）里。
// ---------------------------------------------------------------------------
uint8_t ssiCrc6(uint32_t frame24) {
  uint8_t crc = 0;
  for (uint8_t idx = 0; idx < 18; ++idx) {
    const uint8_t in = (uint8_t)((frame24 >> (23 - idx)) & 1u);
    const uint8_t feedback = (uint8_t)(((crc >> 5) & 1u) ^ in);
    crc = (uint8_t)(((crc << 1) & 0x3Fu) ^ (feedback ? 0x03u : 0x00u));
  }
  return (uint8_t)(crc & 0x3Fu);
}

// ---------------------------------------------------------------------------
// 字段拆解的两种候选解读（上机确认一次即可，方法见 README 已知限制 1）：
//   lsbFirst = false：线上首位（bit23）是角度最高位 —— 标准 MSB-first 直觉。
//                     角度 = frame[23:10]，场 = frame[9:6]，CRC = frame[5:0]。
//   lsbFirst = true ：线上首位是手册编号的 bit0（角度最低位 / CRC0）。
//                     各字段需位反转还原：角度 = reverse(frame[23:10]) 等。
// ---------------------------------------------------------------------------
void ssiUnpack(uint32_t frame24, bool lsbFirst,
               uint16_t *angleCount, uint8_t *fieldCode, uint8_t *crc) {
  if (!lsbFirst) {
    if (angleCount) *angleCount = (uint16_t)((frame24 >> 10) & 0x3FFFu);
    if (fieldCode)  *fieldCode  = (uint8_t)((frame24 >> 6) & 0x0Fu);
    if (crc)        *crc        = (uint8_t)(frame24 & 0x3Fu);
  } else {
    if (angleCount) *angleCount = (uint16_t)reverseBits((frame24 >> 10) & 0x3FFFu, 14);
    if (fieldCode)  *fieldCode  = (uint8_t)reverseBits((frame24 >> 6) & 0x0Fu, 4);
    if (crc)        *crc        = (uint8_t)reverseBits(frame24 & 0x3Fu, 6);
  }
}

}  // namespace xagyl_geom
