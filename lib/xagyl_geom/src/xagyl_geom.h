// ============================================================================
//  xagyl_geom —— 滤镜轮的纯几何/编码计算库（不依赖 Arduino，可在 PC 上单测）
//
//  为什么单独成库：
//    「初始位置可调」的正确性完全押在这几个函数上 —— 跨 0° 的角度展开、
//    有符号角差、槽中心角表、SSI 帧 CRC。它们一旦算错，表现为
//    「轮子倒转 359°」或「报错误槽位」这类难以定位的现象。
//    放在 lib/ 里可以脱离硬件在 PC 上跑单元测试，固件与测试共用同一份实现，
//    不存在「测的是副本」的问题。
//
//  本库只使用 <stdint.h> / <math.h>，可编译进任何平台。
// ============================================================================
#pragma once

#include <stdint.h>

namespace xagyl_geom {

// 14 位绝对编码器每圈计数（MT6701 / MT68xx 系列）
constexpr uint16_t ENC_COUNTS = 16384;

// 把 raw 计数换算成 [0,360) 的角度
float countToDeg(uint16_t count);

// 把任意角度归一到 [0,360)
float normalizeDeg(float deg);

// 以 reference 为起点把 deg 展开成连续量，满足 |结果 - reference| <= 180。
// 换位路径规划全靠它：没有这一步，359.5° -> 0.5° 会被当成「倒转 359°」。
float unwrap(float deg, float reference);

// 有符号角差 (a - b)，落在 [-180, +180)。
// 恰好相差半圈是退化情况（顺逆时针等距），本实现取负值一侧。
float signedDelta(float a, float b);

// 槽中心角表：slotAngleDeg(slot) = home + (slot-1)*spacing + offset
// slot 从 1 起算；结果归一到 [0,360)。
// 越界的 slot 返回 home，调用方应自行保证 slot 合法。
float slotAngleDeg(int slot, float homeDeg, float spacingDeg, float offsetDeg);

// 由实测角反推最近的槽位（1..nSlots）；
// 距所有槽中心都超过 halfSpacing 时返回 0，表示「不在任何槽位上」。
// toleranceDeg <= 0 时按 halfSpacing 判定。
int nearestSlot(float deg, int nSlots, float homeDeg, float spacingDeg,
                const int16_t *offsetCnt, float offsetUnitDeg, float toleranceDeg);

// SSI 帧 CRC-6，多项式 x^6+x+1（低位形式 0x03），初值 0，无前/后置反。
// 只校验帧的前 18 位（14 位角度 + 4 位磁场状态），按物理线上顺序处理：
// 最先收到的位 = 帧的 bit23（SPI 硬件固定如此）。
// 该顺序与手册采用哪种位编号无关，因此两种候选解读共用同一个 CRC 计算，
// 位序分歧只体现在 ssiUnpack() 的字段拆解上。
uint8_t ssiCrc6(uint32_t frame24);

// 从 24 位 SSI 帧中按指定位序解出 14 位角度、4 位磁场状态、6 位 CRC。
// 输出参数可为 nullptr。
void ssiUnpack(uint32_t frame24, bool lsbFirst,
               uint16_t *angleCount, uint8_t *fieldCode, uint8_t *crc);

}  // namespace xagyl_geom
