// ============================================================================
//  test/test_xagyl_geom —— 滤镜轮几何计算的单元测试（native 平台，无需硬件）
//
//  运行： pio test -e native
//
//  为什么重点测这些：
//    「初始位置可调」出问题时最典型的表现是跨 0° 的角度算反
//    （359.5° -> 0.5° 被当成倒转 359°），以及槽中心反查落到错误的槽位。
//    这两类错误在真机上很难一眼看出，在这里是几行断言。
//
//  断言的取舍：
//    只写「由算法性质保证成立」的断言，不写「碰巧 observed」的断言。
//    例如 CRC 的位翻转检出性：本实现初值为 0 且线性，
//    因此任意单比特错误必然改变结果（单个 1 的多项式次数 ≤ 18 < 被 g(x) 整除所需次数），
//    于是从全零帧出发翻转区域内任一位，CRC 必须非零。这是可证明的。
//    反过来，「两个随机帧 CRC 不同」无法保证，就不写成断言。
// ============================================================================
#include <unity.h>
#include <math.h>
#include "xagyl_geom.h"

using namespace xagyl_geom;

#define EPS 1e-4f

// Unity 的 RUN_TEST 会在每个用例前后调用 setUp/tearDown，必须提供定义，
// 否则链接期报 undefined reference。这里用例彼此独立，留空即可。
void setUp(void) {}
void tearDown(void) {}

// ---------------------------------------------------------------------------
// 角度归一化
// ---------------------------------------------------------------------------
void test_normalize(void) {
  TEST_ASSERT_FLOAT_WITHIN(EPS, 0.0f,   normalizeDeg(0.0f));
  TEST_ASSERT_FLOAT_WITHIN(EPS, 0.0f,   normalizeDeg(360.0f));
  TEST_ASSERT_FLOAT_WITHIN(EPS, 10.0f,  normalizeDeg(10.0f));
  TEST_ASSERT_FLOAT_WITHIN(EPS, 350.0f, normalizeDeg(-10.0f));
  TEST_ASSERT_FLOAT_WITHIN(EPS, 45.0f,  normalizeDeg(405.0f));
  TEST_ASSERT_FLOAT_WITHIN(EPS, 270.0f, normalizeDeg(-450.0f));
}

// ---------------------------------------------------------------------------
// 计数 -> 角度
// ---------------------------------------------------------------------------
void test_count_to_deg(void) {
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f,   countToDeg(0));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 90.0f,  countToDeg(4096));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 180.0f, countToDeg(8192));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 270.0f, countToDeg(12288));
  const float top = countToDeg(16383);       // 最大计数必须仍落在 [0,360)
  TEST_ASSERT_TRUE(top > 359.9f && top < 360.0f);
}

// ---------------------------------------------------------------------------
// 有符号角差：跨 0 点是重点
// ---------------------------------------------------------------------------
void test_signed_delta_wrap(void) {
  TEST_ASSERT_FLOAT_WITHIN(EPS, 0.0f,  signedDelta(10.0f, 10.0f));
  TEST_ASSERT_FLOAT_WITHIN(EPS, 5.0f,  signedDelta(15.0f, 10.0f));
  TEST_ASSERT_FLOAT_WITHIN(EPS, -5.0f, signedDelta(10.0f, 15.0f));
  // 359 -> 1 实际是 +2，不是 -358
  TEST_ASSERT_FLOAT_WITHIN(EPS, 2.0f,  signedDelta(1.0f, 359.0f));
  TEST_ASSERT_FLOAT_WITHIN(EPS, -2.0f, signedDelta(359.0f, 1.0f));
  // 滤镜轮最常见的边界：0.5 与 359.5
  TEST_ASSERT_FLOAT_WITHIN(EPS, 1.0f,  signedDelta(0.5f, 359.5f));
  TEST_ASSERT_FLOAT_WITHIN(EPS, -1.0f, signedDelta(359.5f, 0.5f));
  // 恰好相差半圈是退化情况，本实现取负值一侧（值域 [-180, +180)）
  TEST_ASSERT_FLOAT_WITHIN(EPS, -180.0f, signedDelta(180.0f, 0.0f));
  TEST_ASSERT_FLOAT_WITHIN(EPS, -180.0f, signedDelta(0.0f, 180.0f));
}

// ---------------------------------------------------------------------------
// 连续量展开
// ---------------------------------------------------------------------------
void test_unwrap(void) {
  TEST_ASSERT_FLOAT_WITHIN(EPS, 361.0f, unwrap(1.0f, 359.0f));   // 正向跨过 0 -> 361
  TEST_ASSERT_FLOAT_WITHIN(EPS, -1.0f,  unwrap(359.0f, 1.0f));   // 反向跨过 0 -> -1
  TEST_ASSERT_FLOAT_WITHIN(EPS, 361.5f, unwrap(1.5f, 359.5f));
  TEST_ASSERT_FLOAT_WITHIN(EPS, 72.0f,  unwrap(72.0f, 0.0f));
  TEST_ASSERT_FLOAT_WITHIN(EPS, -72.0f, unwrap(288.0f, 0.0f));   // 等价写法：-72 而非 288

  // 性质：展开结果与参考点距离不超过 180，且归一化后与原始角一致
  for (float r = 0.0f; r < 360.0f; r += 7.0f) {
    for (float d = 0.0f; d < 360.0f; d += 11.0f) {
      const float u = unwrap(d, r);
      TEST_ASSERT_TRUE(fabsf(u - r) <= 180.001f);
      TEST_ASSERT_FLOAT_WITHIN(0.05f, normalizeDeg(u), normalizeDeg(d));
    }
  }
}

// ---------------------------------------------------------------------------
// 槽中心角表 —— 初始位置可调的核心
// ---------------------------------------------------------------------------
void test_slot_table_5way(void) {
  const float home = 0.0f, spacing = 72.0f;
  TEST_ASSERT_FLOAT_WITHIN(EPS, 0.0f,   slotAngleDeg(1, home, spacing, 0.0f));
  TEST_ASSERT_FLOAT_WITHIN(EPS, 72.0f,  slotAngleDeg(2, home, spacing, 0.0f));
  TEST_ASSERT_FLOAT_WITHIN(EPS, 144.0f, slotAngleDeg(3, home, spacing, 0.0f));
  TEST_ASSERT_FLOAT_WITHIN(EPS, 216.0f, slotAngleDeg(4, home, spacing, 0.0f));
  TEST_ASSERT_FLOAT_WITHIN(EPS, 288.0f, slotAngleDeg(5, home, spacing, 0.0f));

  // home 平移：整体跟着移，且仍归一化
  TEST_ASSERT_FLOAT_WITHIN(EPS, 10.0f,  slotAngleDeg(1, 10.0f, spacing, 0.0f));
  TEST_ASSERT_FLOAT_WITHIN(EPS, 298.0f, slotAngleDeg(5, 10.0f, spacing, 0.0f));

  // 偏移生效且不破坏归一化
  TEST_ASSERT_FLOAT_WITHIN(EPS, 1.0f,   slotAngleDeg(1, 0.0f, spacing, 1.0f));
  TEST_ASSERT_FLOAT_WITHIN(EPS, 359.0f, slotAngleDeg(1, 0.0f, spacing, -1.0f));
}

void test_slot_table_wrap(void) {
  // home = 350，间距 72：槽 2 起跨过 0°
  const float home = 350.0f, spacing = 72.0f;
  TEST_ASSERT_FLOAT_WITHIN(EPS, 350.0f, slotAngleDeg(1, home, spacing, 0.0f));
  TEST_ASSERT_FLOAT_WITHIN(EPS, 62.0f,  slotAngleDeg(2, home, spacing, 0.0f));
  TEST_ASSERT_FLOAT_WITHIN(EPS, 134.0f, slotAngleDeg(3, home, spacing, 0.0f));
  // 从槽 1 走到槽 2 应只需 +72°，绝不能变成 -288°
  TEST_ASSERT_FLOAT_WITHIN(EPS, 72.0f, signedDelta(slotAngleDeg(2, home, spacing, 0.0f),
                                                   slotAngleDeg(1, home, spacing, 0.0f)));
  // 相邻槽位之间的步距恒等于 spacing（遍历一整圈回到起点）
  for (int i = 1; i < 5; ++i) {
    TEST_ASSERT_FLOAT_WITHIN(0.01f, spacing,
                             signedDelta(slotAngleDeg(i + 1, home, spacing, 0.0f),
                                         slotAngleDeg(i, home, spacing, 0.0f)));
  }
}

void test_slot_table_7way(void) {
  // 7 位轮：360/7 = 51.4286°，非整数间距最容易暴露归一化与浮点累计误差
  const float spacing = 360.0f / 7.0f;
  for (int i = 1; i <= 7; ++i) {
    const float a = slotAngleDeg(i, 0.0f, spacing, 0.0f);
    TEST_ASSERT_TRUE(a >= 0.0f && a < 360.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, (float)(i - 1) * spacing, a);
  }
}

// ---------------------------------------------------------------------------
// 由实测角反推槽位
// ---------------------------------------------------------------------------
void test_nearest_slot(void) {
  const float home = 0.0f, spacing = 72.0f, unit = 0.25f;
  const int16_t off[8] = {0, 0, 0, 0, 0, 0, 0, 0};

  TEST_ASSERT_EQUAL(1, nearestSlot(0.0f,   5, home, spacing, off, unit, 2.0f));
  TEST_ASSERT_EQUAL(3, nearestSlot(144.2f, 5, home, spacing, off, unit, 2.0f));
  TEST_ASSERT_EQUAL(5, nearestSlot(287.9f, 5, home, spacing, off, unit, 2.0f));
  // 跨 0°：359.8° 离槽 1(0°) 只有 0.2°，必须落到槽 1 而不是槽 5
  TEST_ASSERT_EQUAL(1, nearestSlot(359.8f, 5, home, spacing, off, unit, 2.0f));
  TEST_ASSERT_EQUAL(1, nearestSlot(0.2f,   5, home, spacing, off, unit, 2.0f));
  // 两槽正中间（36°）超出 2° 容差 -> 判定为「不在槽位上」
  TEST_ASSERT_EQUAL(0, nearestSlot(36.0f, 5, home, spacing, off, unit, 2.0f));
  // 容差请求值大于半槽距时被夹到半槽距，此时 36° 恰好可被认领
  TEST_ASSERT_EQUAL(1, nearestSlot(36.0f, 5, home, spacing, off, unit, 40.0f));
}

void test_nearest_slot_with_offset(void) {
  const float home = 0.0f, spacing = 72.0f, unit = 0.25f;
  const int16_t off[8] = {0, 0, 20, 0, 0, 0, 0, 0};   // 槽 3 偏移 +20 单位 = +5°

  TEST_ASSERT_FLOAT_WITHIN(EPS, 149.0f, slotAngleDeg(3, home, spacing, 20.0f * unit));
  TEST_ASSERT_EQUAL(3, nearestSlot(149.0f, 5, home, spacing, off, unit, 2.0f));
  // 144° 原本是槽 3 中心，加偏移后偏离 5°（> 容差 2°）-> 不再认领为槽 3
  TEST_ASSERT_TRUE(nearestSlot(144.0f, 5, home, spacing, off, unit, 2.0f) != 3);
}

void test_nearest_slot_guard(void) {
  const int16_t off[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  TEST_ASSERT_EQUAL(0, nearestSlot(10.0f, 0, 0.0f, 72.0f, off, 0.25f, 0.0f));   // 槽数非法
  TEST_ASSERT_EQUAL(0, nearestSlot(10.0f, 5, 0.0f, 0.0f,  off, 0.25f, 0.0f));   // 间距为 0
  TEST_ASSERT_EQUAL(0, nearestSlot(10.0f, 5, 0.0f, -1.0f, off, 0.25f, 0.0f));   // 负间距
  // offsetCnt 为空时按零偏移处理，不得崩溃
  TEST_ASSERT_EQUAL(2, nearestSlot(72.0f, 5, 0.0f, 72.0f, nullptr, 0.25f, 2.0f));
}

// ---------------------------------------------------------------------------
// SSI CRC-6（x^6+x+1，初值 0）
//
// 这些向量是「算法自洽性」校验，防止后续重构把实现改坏；
// 不能证明该多项式与位序就是 MT6701 的口径 —— 那必须上机确认，
// 方法见 README 已知限制 1（用 FW_SSI_CHECK_CRC=1 连读几百帧看是否稳定通过）。
// ---------------------------------------------------------------------------
void test_ssi_crc_zero_and_determinism(void) {
  // 初值 0 且全零输入 -> 输出必为 0
  TEST_ASSERT_EQUAL_UINT8(0, ssiCrc6(0x000000u));
  TEST_ASSERT_EQUAL_UINT8(ssiCrc6(0x123456u), ssiCrc6(0x123456u));
}

void test_ssi_crc_ignores_crc_field_bits(void) {
  // bit5..bit0 是帧自带的 CRC 字段，不参与计算。
  // 下面两帧只有低 6 位不同，高 18 位完全相同 -> CRC 必须相同
  const uint8_t a = ssiCrc6(0x2BC540u);
  const uint8_t b = ssiCrc6(0x2BC57Fu);
  TEST_ASSERT_EQUAL_UINT8(a, b);
  // 而改动 bit6（参与计算区域的最低位）必须得到不同结果
  TEST_ASSERT_NOT_EQUAL(a, ssiCrc6(0x2BC500u));
}

void test_ssi_crc_detects_all_single_bit_errors(void) {
  // 线性 CRC + 初值 0：从全零帧出发，翻转参与计算区域内任意一位，结果必非零。
  // 区域 = bit6..bit23（共 18 位）
  for (uint8_t bit = 6; bit < 24; ++bit) {
    const uint32_t frame = 1u << bit;
    TEST_ASSERT_NOT_EQUAL_MESSAGE(0, ssiCrc6(frame), "single-bit error undetected");
  }
}

void test_ssi_crc_output_range(void) {
  // 结果始终落在 6 位内，且实现不是「恒返回同一个值」的废掉状态
  uint8_t seen[64] = {0};
  int distinct = 0;
  for (uint32_t v = 0; v < 4096; ++v) {
    const uint8_t c = ssiCrc6(v * 0x9E3779B9u & 0x00FFFFFFu);
    TEST_ASSERT_TRUE(c <= 0x3F);
    if (!seen[c]) { seen[c] = 1; distinct++; }
  }
  TEST_ASSERT_TRUE_MESSAGE(distinct > 16, "CRC looks degenerate");
}

// ---------------------------------------------------------------------------
// SSI 字段拆解
// ---------------------------------------------------------------------------
void test_ssi_unpack_msb_first(void) {
  const uint32_t angle = 0x2BC5u, field = 0xAu, crc = 0x2Fu;   // angle=11205
  const uint32_t frame = (angle << 10) | (field << 6) | crc;
  uint16_t a = 0; uint8_t f = 0, c = 0;
  ssiUnpack(frame, false, &a, &f, &c);
  TEST_ASSERT_EQUAL_UINT16(angle, a);
  TEST_ASSERT_EQUAL_UINT8(field, f);
  TEST_ASSERT_EQUAL_UINT8(crc, c);
  ssiUnpack(frame, false, nullptr, nullptr, nullptr);   // 空指针安全
}

void test_ssi_unpack_lsb_first_is_mirror(void) {
  // 取一个只有最高位为 1 的角度：位反转后应只剩最低位为 1
  const uint32_t angleHi = 0x2000u;                 // 14 位里的最高位
  const uint32_t frame = angleHi << 10;             // 磁场状态与 CRC 字段留空
  uint16_t a1 = 0, a2 = 0;
  ssiUnpack(frame, false, &a1, nullptr, nullptr);
  ssiUnpack(frame, true,  &a2, nullptr, nullptr);
  TEST_ASSERT_EQUAL_UINT16(0x2000u, a1);            // MSB 解读：原样
  TEST_ASSERT_EQUAL_UINT16(0x0001u, a2);            // LSB 解读：镜像到最低位

  // 反转是对合运算：把镜像值再当帧头解一次，必须回到原值
  uint16_t back = 0;
  ssiUnpack(((uint32_t)a2 << 10), true, &back, nullptr, nullptr);
  TEST_ASSERT_EQUAL_UINT16(angleHi, back);

  // 两种解读在本帧上确实不同
  TEST_ASSERT_NOT_EQUAL(a1, a2);

  // 字段位宽在两种解读下都必须合法
  uint8_t f = 0, c = 0;
  const uint32_t full = (0x2BC5u << 10) | (0x0Au << 6) | 0x2Fu;
  ssiUnpack(full, false, &a1, &f, &c);
  TEST_ASSERT_TRUE(f <= 0x0F && c <= 0x3F);
  ssiUnpack(full, true, &a2, &f, &c);
  TEST_ASSERT_TRUE(a2 <= 0x3FFF && f <= 0x0F && c <= 0x3F);
}

void test_ssi_roundtrip_all_angles(void) {
  // MSB 解读下遍历角度取值，拆解必须无损，且与 countToDeg 一致
  for (uint32_t angle = 0; angle < 16384; angle += 137u) {
    const uint32_t frame = (angle << 10) | (0x5u << 6);
    uint16_t a = 0; uint8_t f = 0, c = 0;
    ssiUnpack(frame, false, &a, &f, &c);
    TEST_ASSERT_EQUAL_UINT16(angle, a);
    TEST_ASSERT_EQUAL_UINT8(0x5, f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, countToDeg((uint16_t)angle), countToDeg(a));
  }
}

// ---------------------------------------------------------------------------
int main(void) {
  UNITY_BEGIN();

  RUN_TEST(test_normalize);
  RUN_TEST(test_count_to_deg);
  RUN_TEST(test_signed_delta_wrap);
  RUN_TEST(test_unwrap);
  RUN_TEST(test_slot_table_5way);
  RUN_TEST(test_slot_table_wrap);
  RUN_TEST(test_slot_table_7way);
  RUN_TEST(test_nearest_slot);
  RUN_TEST(test_nearest_slot_with_offset);
  RUN_TEST(test_nearest_slot_guard);
  RUN_TEST(test_ssi_crc_zero_and_determinism);
  RUN_TEST(test_ssi_crc_ignores_crc_field_bits);
  RUN_TEST(test_ssi_crc_detects_all_single_bit_errors);
  RUN_TEST(test_ssi_crc_output_range);
  RUN_TEST(test_ssi_unpack_msb_first);
  RUN_TEST(test_ssi_unpack_lsb_first_is_mirror);
  RUN_TEST(test_ssi_roundtrip_all_angles);

  return UNITY_END();
}
