#pragma once
#include <Arduino.h>

// ─── タイヤ状態 ─────────────────────────────────────────────
// Continental TPMS センサー 1 本分のデコード済みデータ。
// LCD スロット (0〜3) で管理。

struct TireState {
    uint32_t sensorId;          // 32-bit Continental sensor ID
    float    psi;               // 圧力 [PSI]
    float    bar;               // 圧力 [bar]
    float    kPa;               // 圧力 [kPa]
    float    temperatureC;      // 温度 [°C]
    char     label[8];          // "FL"/"FR"/"RL"/"RR"
    uint32_t lastUpdateMs;      // millis() 更新時刻
    bool     valid;             // 有効データがあるか
};

static const int LCD_SENSOR_COUNT = 4;

// 各スロットの最新状態（lcd_display.cpp に実体、main.cpp から更新）
extern TireState g_tireState[LCD_SENSOR_COUNT];

// ─── 車両レイアウト (2LCD 左右分割) ──────────────────────────
// 左LCD: 左側タイヤ   右LCD: 右側タイヤ
//
//   ┌──────────┐  ┌──────────┐
//   │ slot 0   │  │ slot 2   │  ← 上段 (Front)
//   │ FL       │  │ FR       │
//   │──────────│  │──────────│
//   │ slot 1   │  │ slot 3   │  ← 下段 (Rear)
//   │ RL       │  │ RR       │
//   └──────────┘  └──────────┘
//
// センサーは検出順に slot 0→1→2→3 へ自動割当

// ─── 公開 API ─────────────────────────────────────────────────

// LCD 初期化（setup() 内で呼ぶ）
void lcdBegin();

// Continental TPMS デコード結果を TireState に書き込み、描画フラグを立てる
// lcdSlot       : LCD 表示スロット (0..3)
// sensorId      : 32-bit Continental sensor ID
// psi/bar/kPa   : 圧力値
// temperatureC  : 温度 [°C]
void lcdUpdateTire(int lcdSlot, uint32_t sensorId, float psi, float bar, float kPa, float temperatureC);

// 変更のあった象限を再描画（loop() 内で周期的に呼ぶ）
void lcdRefresh();
