// ============================================================================
//  Settings.h —— 可持久化配置（掉电保存，基于 Preferences/NVS）
//
//  「初始位置可调」落在四个字段上：
//    · homeAngleDeg   ：槽位 1 中心对应的绝对编码器角度（0~360）
//    · slotSpacingDeg ：相邻槽位的角间距（0 = 自动用 360/N）
//    · offsetCnt[n]   ：各槽位微调偏移，单位 = FW_OFFSET_UNIT_DEG
//    · bootMode       ：上电如何确定当前位置（绝对值编码器可直接吸附，无需归零）
//  另有 microsteps / speedMaxHz / positionTolDeg / dirInvert / lastSlot。
// ============================================================================
#pragma once

#include <Arduino.h>
#include <Preferences.h>
#include "Config.h"

enum BootMode : uint8_t {
  BOOT_FROM_ENCODER = 0,   // 读绝对角就近吸附 —— 绝对值编码器的天然优势，永不需要归零
  BOOT_TO_SLOT_1    = 1,   // 上电主动走到槽位 1
  BOOT_RESTORE_LAST = 2,   // 恢复到掉电前记录的槽位，若已被移动则走回去
};

// 槽位上限 8：配置数组按最大设计，实际位数由 FW_FILTER_SLOTS 决定并经 I8 上报
#define SETTINGS_MAX_SLOTS 8

struct Settings {
  float    homeAngleDeg;
  float    slotSpacingDeg;               // 0 = 自动 360/N
  int16_t  offsetCnt[SETTINGS_MAX_SLOTS];
  BootMode bootMode;
  uint16_t microsteps;                   // 实际细分数 1..256；uint8_t 装不下 256
  uint16_t runMa;
  uint16_t speedMaxHz;
  float    positionTolDeg;
  uint8_t  dirInvert;                    // 1 = 编码器角增大方向对应 DIR 低电平
  uint8_t  lastSlot;                     // 掉电前所在槽位，0 = 未知
  bool     valid;                        // 运行时标志，不落盘
};

class SettingsStore {
 public:
  void begin();
  void load();
  void save();                          // 只有 dirty 时才写 NVS
  bool dirty() const { return _dirty; }
  void markDirty() { _dirty = true; }

  Settings &s() { return _cur; }
  const Settings &s() const { return _cur; }

  // 某槽位的目标绝对角（含偏移），归一到 [0,360)
  float slotAngle(int slot) const;
  float spacingDeg() const {
    return _cur.slotSpacingDeg > 0.01f ? _cur.slotSpacingDeg
                                       : 360.0f / (float)FW_FILTER_SLOTS;
  }
  float offsetDeg(int slot) const {
    if (slot < 1 || slot > SETTINGS_MAX_SLOTS) return 0.0f;
    return (float)_cur.offsetCnt[slot - 1] * FW_OFFSET_UNIT_DEG;
  }

  // 由实测角反推最近槽位（1..N）；距所有槽中心都超过半间距时返回 0（不在槽位上）
  int slotForAngle(float deg) const;

  // 带限幅写入偏移，返回写入后的单位数
  int addOffsetUnits(int slot, int delta);
  int setOffsetUnits(int slot, int units);

  void setDefault();

 private:
  bool sane() const;

  Preferences _prefs;
  Settings    _cur;
  bool        _dirty = false;
};

extern SettingsStore g_settings;
