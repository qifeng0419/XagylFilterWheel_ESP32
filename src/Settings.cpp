// ============================================================================
//  Settings.cpp —— NVS 持久化
//
//  落盘格式带 magic + version 头：固件升级导致结构体布局变化时，
//  旧数据会被识别为不兼容并自动回落到默认值，而不是按错误偏移量解析成乱码。
//
//  槽中心角与槽位反查直接调用 lib/xagyl_geom ——
//  生产代码与 `pio test -e native` 跑的是同一份实现，不存在「测的是副本」。
// ============================================================================
#include "Settings.h"
#include "xagyl_geom.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

SettingsStore g_settings;

namespace {
constexpr uint32_t CFG_MAGIC   = 0x46574831UL;   // "FWH1"
constexpr uint16_t CFG_VERSION = 1;

struct Blob {
  uint32_t magic;
  uint16_t version;
  Settings st;
};

bool isPow2InRange(uint16_t v) {
  return v >= 1 && v <= 256 && (v & (v - 1)) == 0;
}

// 偏移允许的最大单位数：半个槽距。超过它相邻槽位会互相穿插。
float offsetLimitUnits(float spacingDeg) {
  return spacingDeg * 0.5f / (float)FW_OFFSET_UNIT_DEG;
}
}  // namespace

void SettingsStore::setDefault() {
  memset(&_cur, 0, sizeof(_cur));
  _cur.homeAngleDeg    = 0.0f;          // 槽位 1 默认对齐编码器 0°
  _cur.slotSpacingDeg  = (float)FW_SLOT_SPACING_DEG;   // 0 = 自动 360/N
  _cur.bootMode        = BOOT_FROM_ENCODER;
  _cur.microsteps      = FW_MICROSTEPS;
  _cur.runMa           = FW_TMC_RUN_MA;
  _cur.speedMaxHz      = FW_SPEED_MAX_HZ;
  _cur.positionTolDeg  = FW_POSITION_TOL_DEG;
  _cur.dirInvert       = 0;
  _cur.lastSlot        = 0;
  _cur.valid           = true;
}

bool SettingsStore::sane() const {
  if (!isfinite(_cur.homeAngleDeg) || _cur.homeAngleDeg < 0.0f || _cur.homeAngleDeg >= 360.0f) return false;
  if (!isfinite(_cur.slotSpacingDeg) || _cur.slotSpacingDeg < 0.0f || _cur.slotSpacingDeg > 360.0f) return false;
  if (!isPow2InRange(_cur.microsteps)) return false;
  if (_cur.runMa < 50 || _cur.runMa > 2000) return false;
  if (_cur.speedMaxHz < 50 || _cur.speedMaxHz > 20000) return false;
  if (!isfinite(_cur.positionTolDeg) || _cur.positionTolDeg < 0.1f || _cur.positionTolDeg > 30.0f) return false;
  if ((uint8_t)_cur.bootMode > (uint8_t)BOOT_RESTORE_LAST) return false;
  if (_cur.dirInvert > 1) return false;
  if (_cur.lastSlot > FW_FILTER_SLOTS) return false;
  if (FW_FILTER_SLOTS < 1 || FW_FILTER_SLOTS > SETTINGS_MAX_SLOTS) return false;

  const float lim = offsetLimitUnits(spacingDeg());
  for (int i = 0; i < SETTINGS_MAX_SLOTS; ++i) {
    if ((float)abs((int)_cur.offsetCnt[i]) > lim) return false;
  }
  return true;
}

void SettingsStore::begin() {
  _prefs.begin(FW_NVS_NAMESPACE, false);
  load();
}

void SettingsStore::load() {
  Blob blob;
  memset(&blob, 0, sizeof(blob));
  const size_t got = _prefs.getBytes(FW_NVS_KEY, &blob, sizeof(blob));

  const bool headerOk = (got == sizeof(blob)) &&
                        (blob.magic == CFG_MAGIC) &&
                        (blob.version == CFG_VERSION);
  _cur = blob.st;

  if (headerOk && sane()) {
    _cur.valid = true;
    _dirty = false;
    return;
  }

  // 首次运行 / 布局或版本不匹配 / 数值越界 -> 回默认并立即落盘
  setDefault();
  _dirty = true;
  save();
}

void SettingsStore::save() {
  if (!_dirty) return;
  Blob blob;
  blob.magic   = CFG_MAGIC;
  blob.version = CFG_VERSION;
  blob.st      = _cur;
  _prefs.putBytes(FW_NVS_KEY, &blob, sizeof(blob));
  _dirty = false;
}

// ---------------------------------------------------------------------------
// 槽位 n 的目标绝对角 = home + (n-1)*spacing + offset[n]，归一到 [0,360)
// 越界的 slot 返回 home（调用方应自行校验，这里只是保证不返回垃圾值）
// ---------------------------------------------------------------------------
float SettingsStore::slotAngle(int slot) const {
  if (slot < 1 || slot > FW_FILTER_SLOTS) return _cur.homeAngleDeg;
  return xagyl_geom::slotAngleDeg(slot, _cur.homeAngleDeg, spacingDeg(), offsetDeg(slot));
}

// ---------------------------------------------------------------------------
// 由实测角反推最近槽位。超出判定范围返回 0 —— 报告真实测到的位置，
// 而不是「命令里期望的位置」。容差直接用到位容差，与运动层的判据保持一致。
// ---------------------------------------------------------------------------
int SettingsStore::slotForAngle(float deg) const {
  return xagyl_geom::nearestSlot(deg, FW_FILTER_SLOTS, _cur.homeAngleDeg, spacingDeg(),
                                 _cur.offsetCnt, (float)FW_OFFSET_UNIT_DEG,
                                 _cur.positionTolDeg);
}

int SettingsStore::setOffsetUnits(int slot, int units) {
  if (slot < 1 || slot > FW_FILTER_SLOTS) return 0;
  const float lim = offsetLimitUnits(spacingDeg());
  int v = units;
  if ((float)v > lim)  v = (int)lim;
  if ((float)v < -lim) v = -(int)lim;
  _cur.offsetCnt[slot - 1] = (int16_t)v;
  markDirty();
  return v;
}

int SettingsStore::addOffsetUnits(int slot, int delta) {
  if (slot < 1 || slot > FW_FILTER_SLOTS) return 0;
  return setOffsetUnits(slot, (int)_cur.offsetCnt[slot - 1] + delta);
}
