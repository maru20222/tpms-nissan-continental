#include "lcd_display.h"
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>

// ─── ピン定義 ────────────────────────────────────────────────
// doc/index.md の LCD 結線表より
// SPI バス共有
#define LCD_SCK   17
#define LCD_MOSI  18
#define LCD_BL     7    // バックライト（全LCD共用、Pch MOSFET 高側スイッチ: LOW=ON）

// 第1LCD（左側、既存）
#define LCD1_RST   4
#define LCD1_DC    5
#define LCD1_CS    6

// 第2LCD（右側、追加）
#define LCD2_RST  19
#define LCD2_DC   21
#define LCD2_CS   20

// ─── 色定数 (RGB565) ─────────────────────────────────────────
#define COLOR_BLACK      0x0000u
#define COLOR_WHITE      0xFFFFu
#define COLOR_RED        0xF800u
#define COLOR_GREEN      0x07E0u
#define COLOR_YELLOW     0xFFE0u
#define COLOR_ORANGE     0xFC00u
#define COLOR_CYAN       0x07FFu
#define COLOR_DARKGREY   0x7BEFu
#define COLOR_LIGHTGREY  0xC618u

// ─── Adafruit_ST7789 インスタンス ─────────────────────────────
// ESP32-S3 で CC1101(HSPI=SPI3) と競合しないよう FSPI(SPI2) を専用バスとして使う。
// 2台のLCDが同一SPIバスを共有し、CS/DC/RST は個別。
static SPIClass        lcdSpi(FSPI);
static Adafruit_ST7789 tftL(&lcdSpi, LCD1_CS, LCD1_DC, LCD1_RST); // 左LCD
static Adafruit_ST7789 tftR(&lcdSpi, LCD2_CS, LCD2_DC, LCD2_RST); // 右LCD

// ─── グローバル状態（extern 宣言は lcd_display.h に有り）───────
TireState g_tireState[LCD_SENSOR_COUNT];

// 再描画フラグ
static bool g_dirty[LCD_SENSOR_COUNT] = { true, true, true, true };

// ─── スロットレイアウト ──────────────────────────────────────
// 各スロット: 240×120 (上半分 y=0, 下半分 y=120)
// slot 0 = 左LCD上 (F.L A)  slot 1 = 左LCD下 (R.L D)
// slot 2 = 右LCD上 (F.R B)  slot 3 = 右LCD下 (R.R C)
static const int SLOT_Y[4] = { 0, 120, 0, 120 };

static Adafruit_ST7789* slotTft(int slot) {
    return (slot < 2) ? &tftL : &tftR;
}

// スロットラベル
static const char* SLOT_LABEL[4] = { "FL", "RL", "FR", "RR" };

// ─── ヘルパー ─────────────────────────────────────────────────

static uint16_t pressureColor(float kPa, bool valid) {
    if (!valid)           return COLOR_DARKGREY;
    if (kPa < 140.0f)    return COLOR_RED;       // 低圧警告
    if (kPa < 180.0f)    return COLOR_YELLOW;    // やや低め
    if (kPa > 310.0f)    return COLOR_ORANGE;    // 高圧注意
    return COLOR_GREEN;
}

// 最終受信からの経過秒 (1〜999s にクランプ)
static uint16_t slotAgeSec(const TireState& ts) {
    uint32_t sec = (millis() - ts.lastUpdateMs) / 1000;
    if (sec < 1)   sec = 1;
    if (sec > 999) sec = 999;
    return (uint16_t)sec;
}

// 経過秒だけを再描画（kPa と温度の間の領域）
static const int AGE_X = 100;
static const int AGE_W = 56;

static void drawSlotAge(int slot) {
    if (slot < 0 || slot >= 4) return;
    const TireState& ts = g_tireState[slot];
    if (!ts.valid) return;

    Adafruit_ST7789* tft = slotTft(slot);
    const int sy = SLOT_Y[slot];

    char buf[8];
    snprintf(buf, sizeof(buf), "%3us", slotAgeSec(ts));

    tft->fillRect(AGE_X, sy + 84, AGE_W, 16, COLOR_BLACK);
    tft->setTextColor(COLOR_LIGHTGREY);
    tft->setTextSize(2);
    tft->setCursor(AGE_X, sy + 84);
    tft->print(buf);
}

// ─── 1 スロット描画 (240×120) ────────────────────────────────
static void drawSlot(int slot) {
    if (slot < 0 || slot >= 4) return;

    Adafruit_ST7789* tft = slotTft(slot);
    const int sy = SLOT_Y[slot];

    const TireState& ts = g_tireState[slot];
    const bool  valid   = ts.valid;
    uint16_t colText = pressureColor(ts.kPa, valid);

    // 背景クリア (240×119、分割線1px分を除く)
    tft->fillRect(0, sy, 240, 119, COLOR_BLACK);

    // ── ラベル行 (位置名 + センサーID) ───────────────────────
    tft->setTextColor(COLOR_WHITE);
    tft->setTextSize(2);
    tft->setCursor(4, sy + 4);
    tft->print(SLOT_LABEL[slot]);

    if (valid) {
        // センサーID表示（左寄せ）
        char idBuf[14];
        snprintf(idBuf, sizeof(idBuf), "ID:%08X", ts.sensorId);
        tft->setTextColor(COLOR_LIGHTGREY);
        tft->setTextSize(2);
        tft->setCursor(40, sy + 4);
        tft->print(idBuf);
    }

    if (!valid) {
        tft->setTextColor(COLOR_DARKGREY);
        tft->setTextSize(4);
        tft->setCursor(84, sy + 36);
        tft->print("---");
        tft->setTextSize(2);
        tft->setCursor(78, sy + 80);
        tft->print("NO DATA");
        return;
    }

    // ── 圧力 bar (大表示・中央) ──────────────────────────────
    char bufBar[12];
    dtostrf(ts.bar, 4, 2, bufBar);
    const char* pBar = bufBar;
    while (*pBar == ' ') pBar++;

    tft->setTextColor(colText);
    tft->setTextSize(5);
    // 中央寄せ: 1文字=30px幅(size5)、文字数に応じて調整
    int barLen = (int)strlen(pBar);
    int barX = (240 - barLen * 30) / 2;
    if (barX < 4) barX = 4;
    tft->setCursor(barX, sy + 28);
    tft->print(pBar);

    tft->setTextSize(2);
    tft->setCursor(barX + barLen * 30 + 4, sy + 52);
    tft->print("bar");

    // ── kPa / 温度 (補助表示) ────────────────────────────────
    char bufKpa[16];
    snprintf(bufKpa, sizeof(bufKpa), "%.0fkPa", ts.kPa);
    tft->setTextColor(COLOR_CYAN);
    tft->setTextSize(2);
    tft->setCursor(8, sy + 84);
    tft->print(bufKpa);

    // 最終受信からの経過秒
    char bufAge[8];
    snprintf(bufAge, sizeof(bufAge), "%3us", slotAgeSec(ts));
    tft->setTextColor(COLOR_LIGHTGREY);
    tft->setCursor(AGE_X, sy + 84);
    tft->print(bufAge);

    // 温度表示
    char bufTemp[12];
    snprintf(bufTemp, sizeof(bufTemp), "%3.0fC", ts.temperatureC);
    tft->setTextColor(COLOR_LIGHTGREY);
    tft->setCursor(160, sy + 84);
    tft->print(bufTemp);

    // ── ステータス ────────────────────────────────────────────
    const char* statusDisp = "OK";
    uint16_t    statusCol  = COLOR_GREEN;
    if      (ts.kPa < 140.0f) { statusDisp = "LOW!";  statusCol = COLOR_RED; }
    else if (ts.kPa < 180.0f) { statusDisp = "LOW";   statusCol = COLOR_YELLOW; }
    else if (ts.kPa > 310.0f) { statusDisp = "HIGH";  statusCol = COLOR_ORANGE; }

    tft->setTextColor(statusCol);
    tft->setTextSize(2);
    tft->setCursor(200, sy + 28);
    tft->print(statusDisp);
}

// ─── 分割線描画 ───────────────────────────────────────────────
static void drawDividers() {
    tftL.drawFastHLine(0, 119, 240, COLOR_WHITE);
    tftR.drawFastHLine(0, 119, 240, COLOR_WHITE);
}

// ─── 公開 API 実装 ─────────────────────────────────────────────

void lcdBegin() {
    // バックライト ON（全LCD共用）
    // Pチャネル MOSFET (ZVP2106A) による高側スイッチのため LOW=点灯。
    pinMode(LCD_BL, OUTPUT);
    digitalWrite(LCD_BL, LOW);

    // FSPI(SPI2) を LCD ピンで初期化。CC1101 の HSPI(SPI3) と別ホストなので衝突なし。
    lcdSpi.begin(LCD_SCK, /*MISO*/ -1, LCD_MOSI, LCD1_CS);

    // 左LCD (ST7789 240×240)
    tftL.init(240, 240, SPI_MODE3);
    tftL.setRotation(0);
    tftL.fillScreen(COLOR_BLACK);

    // 右LCD (ST7789 240×240)
    tftR.init(240, 240, SPI_MODE3);
    tftR.setRotation(0);
    tftR.fillScreen(COLOR_BLACK);

    // 初期状態設定
    const char* initialLabels[LCD_SENSOR_COUNT] = {
        "FL",     // slot 0 (Left top)
        "RL",     // slot 1 (Left bottom)
        "FR",     // slot 2 (Right top)
        "RR",     // slot 3 (Right bottom)
    };
    for (int i = 0; i < LCD_SENSOR_COUNT; i++) {
        memset(&g_tireState[i], 0, sizeof(TireState));
        strncpy(g_tireState[i].label, initialLabels[i], sizeof(g_tireState[i].label) - 1);
        g_tireState[i].valid = false;
        g_dirty[i] = true;
    }

    for (int s = 0; s < 4; s++) drawSlot(s);
    drawDividers();

    Serial.println("[LCD] dual init OK (L+R)");
}

// ─── 致命エラー画面 ───────────────────────────────────────────
static const int FATAL_CD_Y = 150;

void lcdShowFatal(const char* title, const char* detail) {
    Adafruit_ST7789* tfts[2] = { &tftL, &tftR };
    for (int i = 0; i < 2; i++) {
        Adafruit_ST7789* tft = tfts[i];
        tft->fillScreen(COLOR_BLACK);
        tft->setTextColor(COLOR_RED);
        tft->setTextSize(4);
        tft->setCursor(8, 50);
        tft->print(title);
        tft->setTextColor(COLOR_WHITE);
        tft->setTextSize(2);
        tft->setCursor(8, 110);
        tft->print(detail);
    }
}

void lcdShowFatalCountdown(int secLeft) {
    if (secLeft < 0) secLeft = 0;
    char buf[24];
    snprintf(buf, sizeof(buf), "REBOOT in %2ds", secLeft);
    lcdShowFatalNote(buf);
}

void lcdShowFatalNote(const char* msg) {
    Adafruit_ST7789* tfts[2] = { &tftL, &tftR };
    for (int i = 0; i < 2; i++) {
        Adafruit_ST7789* tft = tfts[i];
        tft->fillRect(8, FATAL_CD_Y, 232, 20, COLOR_BLACK);
        tft->setTextColor(COLOR_YELLOW);
        tft->setTextSize(2);
        tft->setCursor(8, FATAL_CD_Y);
        tft->print(msg);
    }
}

void lcdForceRedraw() {
    tftL.fillScreen(COLOR_BLACK);
    tftR.fillScreen(COLOR_BLACK);
    for (int i = 0; i < LCD_SENSOR_COUNT; i++) g_dirty[i] = false;
    for (int s = 0; s < LCD_SENSOR_COUNT; s++) drawSlot(s);
    drawDividers();
}

void lcdUpdateTire(int lcdSlot, uint32_t sensorId, float psi, float bar, float kPa, float temperatureC) {
    if (lcdSlot < 0 || lcdSlot >= LCD_SENSOR_COUNT) return;

    TireState& ts   = g_tireState[lcdSlot];
    ts.sensorId     = sensorId;
    ts.psi          = psi;
    ts.bar          = bar;
    ts.kPa          = kPa;
    ts.temperatureC = temperatureC;
    ts.lastUpdateMs = millis();
    ts.valid        = true;

    g_dirty[lcdSlot] = true;
}

void lcdRefresh() {
    bool anyDirty = false;
    for (int i = 0; i < LCD_SENSOR_COUNT; i++) {
        if (g_dirty[i]) { anyDirty = true; break; }
    }

    for (int slot = 0; slot < LCD_SENSOR_COUNT; slot++) {
        if (!g_dirty[slot]) continue;
        drawSlot(slot);
        g_dirty[slot] = false;
    }
    if (anyDirty) drawDividers();

    // 経過秒は毎秒その領域だけ更新（全面再描画によるちらつきを避ける）
    static uint32_t lastAgeMs = 0;
    if (millis() - lastAgeMs >= 1000) {
        lastAgeMs = millis();
        for (int slot = 0; slot < LCD_SENSOR_COUNT; slot++) drawSlotAge(slot);
    }
}
