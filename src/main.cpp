// ============================================================
// Continental / Nissan TPMS Receiver  (v3)
// CC1101 (315 MHz FSK) + ESP32-S3 + ST7789 LCD x2
//
// Sensor: Continental S180052353E  (FCC: KR5S180052015B)
//   Nissan part: 40700-4GA0B
//   Build env switch: TPMS_ENV_DEV (see below)
//   PROD IDs (Autel MX-Sensor):
//     FL:11111111  FR:22222222  RL:33333333  RR:44444444
//   DEV IDs (original OEM sensors):
//     FL:AE5C32C8  FR:AC4ACC28  RL:AE58E836  RR:AC4CCF67
//
// Protocol (reverse-engineered from captures + BMW Gen5 reference):
//   Modulation  : FSK, Manchester coded (G.E. Thomas)
//   Half-bit    : 122us (park mode) / ~52us measured (drive mode, nominal 61us)
//   Preamble    : Alternating 0/1 run (~28+ half-bits)
//   Data (8 bytes = 64 bits, at bit offset 1 from Manchester decode):
//     Byte 0     : Brand/Manufacturer (8h)  -- 0xA8 for this sensor
//     Bytes 1-4  : Sensor ID (32h)
//     Byte 5     : Pressure (8h)  -- PSI = raw / 4
//     Byte 6     : Temperature (8h) -- C = raw - 52
//     Byte 7     : CRC-8 (poly=0x07, init=0xAA) over bytes 0-6
//   After CRC   : optional flags/sequence byte(s)
// ============================================================

#include <Arduino.h>
#include <SPI.h>
#include <driver/gpio.h>
#include <RadioLib.h>
#include <Preferences.h>
#include <math.h>
#include "lcd_display.h"

// ====== Build environment: dev / prod ======
// TPMS_ENV_DEV = 1 : dev  (original OEM sensors: AE5C32C8 ...)
// TPMS_ENV_DEV = 0 : prod (Autel MX-Sensors:     11111111 ...)
// Override from platformio.ini with e.g. build_flags = -DTPMS_ENV_DEV=0
#ifndef ENABLE_DESK_TEST
#define ENABLE_DESK_TEST false
#endif

#ifndef TPMS_ENV_DEV
#define TPMS_ENV_DEV false
#endif

#if TPMS_ENV_DEV
static const char* const TPMS_ENV_NAME = "DEV (OEM sensors)";
#else
static const char* const TPMS_ENV_NAME = "PROD (Autel MX-Sensor)";
#endif

#ifndef ENABLE_DETAILED_LOG
#define ENABLE_DETAILED_LOG false
#endif

#ifndef ENABLE_OTHER_SENSOR_ID_LOG
#define ENABLE_OTHER_SENSOR_ID_LOG false
#endif

// ====== Pin wiring ======
static const int PIN_CS   = 10;
static const int PIN_SCK  = 12;
static const int PIN_MOSI = 11;
static const int PIN_MISO = 13;
static const int PIN_GDO0 = 16;   // Carrier Sense
static const int PIN_GDO2 = 15;   // Async Data

// ====== Known sensor table ======
struct KnownSensor {
  uint32_t fullId;
  int      lcdSlot;  // 0=FL 1=RL 2=FR 3=RR
  const char* partNo;
};

#if TPMS_ENV_DEV
// dev: original OEM sensors
static const KnownSensor KNOWN_SENSORS[] = {
  { 0xAE5C32C8, 0, "FL:40700-4GA0B" },  // FL
  { 0xAC4ACC28, 2, "FR:40700-4GA0B" },  // FR
  { 0xAE58E836, 1, "RL:40700-4GA0B" },  // RL
  { 0xAC4CCF67, 3, "RR:40700-4GA0B" },  // RR
};
#else
// prod: Autel MX-Sensors
static const KnownSensor KNOWN_SENSORS[] = {
  { 0x11111111, 0, "FL:40700-4GA0B" },  // FL
  { 0x22222222, 2, "FR:40700-4GA0B" },  // FR
  { 0x33333333, 1, "RL:40700-4GA0B" },  // RL
  { 0x44444444, 3, "RR:40700-4GA0B" },  // RR
};
#endif
static const int KNOWN_SENSOR_COUNT = (int)(sizeof(KNOWN_SENSORS) / sizeof(KNOWN_SENSORS[0]));

// ====== Persistent LCD state (NVS) ======
static const uint32_t TPMS_STATE_MAGIC = 0x54504D53;  // 'TPMS'
static const uint16_t TPMS_STATE_VERSION = 1;
static const uint32_t TPMS_STATE_SAVE_INTERVAL_MS = 60000;

struct PersistTireSlot {
  uint32_t sensorId;
  float    psi;
  float    bar;
  float    kPa;
  float    temperatureC;
  uint8_t  valid;
  uint8_t  reserved[3];
};

struct PersistDisplayState {
  uint32_t magic;
  uint16_t version;
  uint8_t  envDev;
  uint8_t  slotCount;
  uint32_t sensorTableHash;
  uint32_t savedAtSec;
  PersistTireSlot slots[LCD_SENSOR_COUNT];
  uint32_t checksum;
};

static PersistDisplayState g_lastSavedState = {};
static bool g_hasLastSavedState = false;
static bool g_displayStateDirty = false;
static uint32_t g_lastPersistAttemptMs = 0;

static uint32_t fnv1a32(const uint8_t* data, size_t len) {
  uint32_t h = 2166136261u;
  for (size_t i = 0; i < len; i++) {
    h ^= data[i];
    h *= 16777619u;
  }
  return h;
}

static uint32_t calcKnownSensorsFingerprint() {
  uint32_t h = 2166136261u;
  for (int i = 0; i < KNOWN_SENSOR_COUNT; i++) {
    h ^= KNOWN_SENSORS[i].fullId; h *= 16777619u;
    h ^= (uint32_t)(KNOWN_SENSORS[i].lcdSlot & 0xFF); h *= 16777619u;
  }
  h ^= (uint32_t)KNOWN_SENSOR_COUNT; h *= 16777619u;
  return h;
}

static const char* getStateNamespace() {
  return TPMS_ENV_DEV ? "tpms_dev" : "tpms_prod";
}

static bool sameSlotForSave(const PersistTireSlot& a, const PersistTireSlot& b) {
  if (a.valid != b.valid) return false;
  if (!a.valid) return true;
  if (a.sensorId != b.sensorId) return false;
  if (fabsf(a.psi - b.psi) > 0.05f) return false;
  if (fabsf(a.bar - b.bar) > 0.01f) return false;
  if (fabsf(a.kPa - b.kPa) > 0.5f) return false;
  if (fabsf(a.temperatureC - b.temperatureC) > 0.5f) return false;
  return true;
}

static bool sameStateForSave(const PersistDisplayState& a, const PersistDisplayState& b) {
  if (a.magic != b.magic || a.version != b.version ||
      a.envDev != b.envDev || a.slotCount != b.slotCount ||
      a.sensorTableHash != b.sensorTableHash) {
    return false;
  }
  for (int i = 0; i < LCD_SENSOR_COUNT; i++) {
    if (!sameSlotForSave(a.slots[i], b.slots[i])) return false;
  }
  return true;
}

static void buildStateFromRam(PersistDisplayState* out) {
  memset(out, 0, sizeof(*out));
  out->magic = TPMS_STATE_MAGIC;
  out->version = TPMS_STATE_VERSION;
  out->envDev = TPMS_ENV_DEV ? 1 : 0;
  out->slotCount = LCD_SENSOR_COUNT;
  out->sensorTableHash = calcKnownSensorsFingerprint();
  out->savedAtSec = millis() / 1000;

  for (int i = 0; i < LCD_SENSOR_COUNT; i++) {
    out->slots[i].sensorId = g_tireState[i].sensorId;
    out->slots[i].psi = g_tireState[i].psi;
    out->slots[i].bar = g_tireState[i].bar;
    out->slots[i].kPa = g_tireState[i].kPa;
    out->slots[i].temperatureC = g_tireState[i].temperatureC;
    out->slots[i].valid = g_tireState[i].valid ? 1 : 0;
  }

  out->checksum = fnv1a32((const uint8_t*)out, sizeof(*out) - sizeof(out->checksum));
}

static bool isStateHeaderValid(const PersistDisplayState& s) {
  if (s.magic != TPMS_STATE_MAGIC) return false;
  if (s.version != TPMS_STATE_VERSION) return false;
  if (s.envDev != (TPMS_ENV_DEV ? 1 : 0)) return false;
  if (s.slotCount != LCD_SENSOR_COUNT) return false;
  if (s.sensorTableHash != calcKnownSensorsFingerprint()) return false;
  uint32_t calc = fnv1a32((const uint8_t*)&s, sizeof(s) - sizeof(s.checksum));
  if (calc != s.checksum) return false;
  return true;
}

static void restoreDisplayStateFromPersist(const PersistDisplayState& s) {
  static const uint32_t RESTORE_DISPLAY_AGE_MS = 999000UL;
  int restored = 0;
  for (int i = 0; i < LCD_SENSOR_COUNT; i++) {
    if (!s.slots[i].valid) continue;
    lcdUpdateTire(i,
                  s.slots[i].sensorId,
                  s.slots[i].psi,
                  s.slots[i].bar,
                  s.slots[i].kPa,
                  s.slots[i].temperatureC);
    // 復帰直後は「保存データを読み出しただけ」で、最新受信ではないことを
    // ひと目で分かるように 999s 表示へ寄せる。
    // uint32_t の差分演算なので、millis() ベースでも問題なく扱える。
    g_tireState[i].lastUpdateMs = millis() - RESTORE_DISPLAY_AGE_MS;
    restored++;
  }
  Serial.printf("[PERSIST] Restored %d tire slots from NVS (%s)\n", restored, getStateNamespace());
}

static void loadPersistedDisplayState() {
  Preferences prefs;
  if (!prefs.begin(getStateNamespace(), true)) {
    Serial.printf("[PERSIST] open(read) failed: ns=%s\n", getStateNamespace());
    return;
  }

  size_t len = prefs.getBytesLength("lcd_state");
  if (len != sizeof(PersistDisplayState)) {
    if (len > 0) {
      Serial.printf("[PERSIST] size mismatch: stored=%u expected=%u\n",
                    (unsigned)len, (unsigned)sizeof(PersistDisplayState));
    }
    prefs.end();
    return;
  }

  PersistDisplayState loaded = {};
  size_t rd = prefs.getBytes("lcd_state", &loaded, sizeof(loaded));
  prefs.end();
  if (rd != sizeof(loaded)) {
    Serial.printf("[PERSIST] read failed: %u/%u\n", (unsigned)rd, (unsigned)sizeof(loaded));
    return;
  }
  if (!isStateHeaderValid(loaded)) {
    Serial.println("[PERSIST] ignored: header/checksum/env/sensor-table mismatch");
    return;
  }

  restoreDisplayStateFromPersist(loaded);
  g_lastSavedState = loaded;
  g_hasLastSavedState = true;
  g_displayStateDirty = false;
}

static void maybeSavePersistedDisplayState(bool force = false) {
  uint32_t now = millis();
  if (!force) {
    if (!g_displayStateDirty) return;
    if ((uint32_t)(now - g_lastPersistAttemptMs) < TPMS_STATE_SAVE_INTERVAL_MS) return;
  }

  PersistDisplayState cur = {};
  buildStateFromRam(&cur);
  if (g_hasLastSavedState && sameStateForSave(cur, g_lastSavedState)) {
    g_displayStateDirty = false;
    g_lastPersistAttemptMs = now;
    return;
  }

  Preferences prefs;
  if (!prefs.begin(getStateNamespace(), false)) {
    Serial.printf("[PERSIST] open(write) failed: ns=%s\n", getStateNamespace());
    g_lastPersistAttemptMs = now;
    return;
  }

  size_t wr = prefs.putBytes("lcd_state", &cur, sizeof(cur));
  prefs.end();
  g_lastPersistAttemptMs = now;

  if (wr == sizeof(cur)) {
    g_lastSavedState = cur;
    g_hasLastSavedState = true;
    g_displayStateDirty = false;
    Serial.printf("[PERSIST] saved (%s)\n", getStateNamespace());
  } else {
    Serial.printf("[PERSIST] write failed: %u/%u\n", (unsigned)wr, (unsigned)sizeof(cur));
  }
}

// 【自車センサーID完全一致チェック関数】
bool isMyCarSensor(uint32_t sensorId) {
  int numSensors = sizeof(KNOWN_SENSORS) / sizeof(KNOWN_SENSORS[0]);
  for (int s = 0; s < numSensors; s++) {
    if (sensorId == KNOWN_SENSORS[s].fullId) {
      return true; // 自車の本物センサーを発見！
    }
  }
  return false; // リストにない他車の電波、またはノイズファントム
}

static int findKnownSensorSlot(uint32_t sensorId) {
  for (int i = 0; i < KNOWN_SENSOR_COUNT; i++) {
    if (sensorId == KNOWN_SENSORS[i].fullId) {
      Serial.printf("  ** Known sensor: %08X -> %s (%s) **\n",
                    sensorId,
                    (KNOWN_SENSORS[i].lcdSlot == 0) ? "FL" :
                    (KNOWN_SENSORS[i].lcdSlot == 1) ? "RL" :
                    (KNOWN_SENSORS[i].lcdSlot == 2) ? "FR" : "RR",
                    KNOWN_SENSORS[i].partNo);
      return KNOWN_SENSORS[i].lcdSlot;
    }
  }
  return -1;
}

// ====== CC1101 SPI ======
// 車載(ダッシュボード)は配線が長くノイズも多いので、RadioLib既定の2MHzでは
// SPIが化けて ERR_CHIP_NOT_FOUND(-2) になる。500kHzまで落とす。
static const SPISettings CC_SPI_SETTINGS(500000, MSBFIRST, SPI_MODE0);
static SPIClass cc1101Spi(HSPI);
CC1101 radio = new Module(PIN_CS, -1, -1, -1, cc1101Spi, CC_SPI_SETTINGS);

// ====== CC1101 register helpers ======
static const uint8_t READ_SINGLE = 0x80;

void ccWrite(uint8_t addr, uint8_t val) {
  digitalWrite(PIN_CS, LOW);
  cc1101Spi.transfer(addr);
  cc1101Spi.transfer(val);
  digitalWrite(PIN_CS, HIGH);
}

uint8_t ccRead(uint8_t addr) {
  digitalWrite(PIN_CS, LOW);
  cc1101Spi.transfer(addr | READ_SINGLE);
  uint8_t v = cc1101Spi.transfer(0);
  digitalWrite(PIN_CS, HIGH);
  return v;
}

// Status registers (e.g. RSSI 0x34) require READ+BURST (0xC0) header.
uint8_t ccReadStatus(uint8_t addr) {
  digitalWrite(PIN_CS, LOW);
  cc1101Spi.transfer(addr | 0xC0);
  uint8_t v = cc1101Spi.transfer(0);
  digitalWrite(PIN_CS, HIGH);
  return v;
}

// CC1101 RSSI in dBm (offset ~74 dB for these settings).
int ccRssiDbm() {
  uint8_t raw = ccReadStatus(RADIOLIB_CC1101_REG_RSSI);
  int r = (raw >= 128) ? (raw - 256) : raw;
  return (r / 2) - 74;
}

// CC1101 データシート 19.1 の手動パワーオンリセット。
// 車載時は5V/3V3の立ち上がりが遅く、自動PORが完了する前にSPIを叩いて
// -2(CHIP_NOT_FOUND)になる。毎回これを先に入れて確実にリセットさせる。
void ccPowerOnReset() {
  cc1101Spi.beginTransaction(CC_SPI_SETTINGS);
  digitalWrite(PIN_CS, HIGH); delayMicroseconds(10);
  digitalWrite(PIN_CS, LOW);  delayMicroseconds(10);
  digitalWrite(PIN_CS, HIGH); delayMicroseconds(45);
  digitalWrite(PIN_CS, LOW);
  uint32_t t0 = millis();
  while (digitalRead(PIN_MISO) == HIGH && (millis() - t0) < 50) { }  // wait SO low
  cc1101Spi.transfer(RADIOLIB_CC1101_CMD_RESET);                     // SRES strobe
  t0 = millis();
  while (digitalRead(PIN_MISO) == HIGH && (millis() - t0) < 50) { }  // wait reset done
  digitalWrite(PIN_CS, HIGH);
  cc1101Spi.endTransaction();
  delay(5);
}

// CC1101 が全く応答しない(PARTNUM/VERSION=0x00)ときに、電源断線か SPI 配線かを切り分ける。
// CC1101 の SO は CSn=L かつ XOSC 安定で L を出す。CSn=H では Hi-Z。
void ccDiagPins() {
  // --- SPI 経路テスト（バスを落とす前に実施） ---
  // ヘッダ転送中に SO へ出るステータスバイトが取れるか = SCK が生きているか。
  // SYNC1 への書き戻しが通るか = MOSI が生きているか。
  // ここは再初期化直前にしか呼ばないので SYNC1 を壊して構わない。
  digitalWrite(PIN_CS, LOW);
  uint8_t stat = cc1101Spi.transfer(0x31 | 0xC0);
  uint8_t ver  = cc1101Spi.transfer(0x00);
  digitalWrite(PIN_CS, HIGH);
  ccWrite(0x04, 0x5A);
  uint8_t back = ccRead(0x04);
  Serial.printf("[DIAG] SPI status=0x%02X ver=0x%02X  wr0x5A->rd0x%02X\n", stat, ver, back);
  // status: bit7=CHIP_RDYn(0=ready), bit6..4=state(0..5 が正常値)
  bool statPlausible = ((stat & 0x80) == 0) && (((stat >> 4) & 0x07) <= 0x05);
  bool verOk = (ver == 0x14 || ver == 0x04 || ver == 0x17);
  if (stat == 0x00 && ver == 0x00 && back == 0x00) {
    Serial.printf("[DIAG]   no clock reaches chip -> SCK(GPIO%d) OPEN\n", PIN_SCK);
  } else if (!statPlausible || !verOk) {
    Serial.printf("[DIAG]   garbled/shifted bytes -> SCK(GPIO%d) BOUNCING (bad contact)\n", PIN_SCK);
  } else if (back != 0x5A) {
    Serial.printf("[DIAG]   clock+status OK but write lost -> MOSI(GPIO%d) suspect\n", PIN_MOSI);
  }

  cc1101Spi.end();

  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  pinMode(PIN_MISO, INPUT_PULLUP);   delay(2);
  int csHighPu = digitalRead(PIN_MISO);
  pinMode(PIN_MISO, INPUT_PULLDOWN); delay(2);
  int csHighPd = digitalRead(PIN_MISO);

  digitalWrite(PIN_CS, LOW);
  pinMode(PIN_MISO, INPUT_PULLUP);   delay(2);
  int csLowPu = digitalRead(PIN_MISO);
  digitalWrite(PIN_CS, HIGH);

  Serial.printf("[DIAG] MISO(GPIO%d) CS=H pu=%d pd=%d | CS=L pu=%d | GDO0=%d GDO2=%d\n",
                PIN_MISO, csHighPu, csHighPd, csLowPu,
                digitalRead(PIN_GDO0), digitalRead(PIN_GDO2));
  if (csLowPu == 0 && csHighPu == 1) {
    Serial.println("[DIAG] SO driven LOW at CS=L -> chip ALIVE. Suspect SCK/MOSI/CS wiring.");
  } else if (csHighPu == 1 && csHighPd == 0 && csLowPu == 1) {
    Serial.println("[DIAG] MISO FLOATING -> CC1101 unpowered (VDD/GND) or MISO wire open.");
  } else if (csHighPu == 0) {
    Serial.println("[DIAG] MISO stuck LOW with pull-up -> shorted to GND or wrong pin.");
  }

  pinMode(PIN_MISO, INPUT);
  cc1101Spi.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);
  gpio_pullup_en((gpio_num_t)PIN_MISO);
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
}

void ccSetPktFormat(uint8_t fmt) {
  uint8_t v = ccRead(RADIOLIB_CC1101_REG_PKTCTRL0);  // PKTCTRL0
  v = (uint8_t)((v & ~0x30) | ((fmt & 0x03) << 4));
  ccWrite(RADIOLIB_CC1101_REG_PKTCTRL0, v);
}

void ccEnableAsyncOnGDO2() {
  ccSetPktFormat(3);           // async serial
  ccWrite(RADIOLIB_CC1101_REG_IOCFG2, 0x0D);        // IOCFG2 = async data out
  ccWrite(RADIOLIB_CC1101_REG_IOCFG0, 0x0E);        // IOCFG0 = Carrier Sense



  if (ENABLE_DESK_TEST) {
    // 【机上テスト：パケット判定の最適化】
    // 本来は AGCCTRL2 狙いだったが、アドレスのズレ（0x07）により PKTCTRL1 に 0x01 を書き込み。
    // 静かな机上環境において、プリアンブルの品質ゲート（PQI閾値）を適正化し、
    // 浮遊ノイズを最初の頭の段階ではじくセーフティとして奇跡的に100点満点で機能していました。
    ccWrite(RADIOLIB_CC1101_REG_PKTCTRL1, 0x01);  // RADIOLIB_CC1101_REG_PKTCTRL1

    // 【机上テスト：パケット長の完全固定】
    // 本来は AGCCTRL1 狙いだったが、アドレスのズレ（0x06）により PKTLEN に 0x40（10進数で64）を書き込み。
    // 日産純正/Autelの64ビット（8バイト）データ構造にジャストフィットする最大パケット長フィルターとなり、
    // 机上でパケットのサイズを綺麗に揃えてデコーダーへ渡す効果を生んでいました。これを維持。
    ccWrite(RADIOLIB_CC1101_REG_PKTLEN, 0x40);  // RADIOLIB_CC1101_REG_PKTLEN: Packet Length = 64 bits (8 bytes)

    // 【机上テスト：同期ワードの下位バイト】
    // 本来は AGCCTRL0 狙いだったが、アドレスのズレ（0x05）により SYNC0 に 0x0F を書き込み。
    ccWrite(RADIOLIB_CC1101_REG_SYNC0, 0x0F);  // RADIOLIB_CC1101_REG_SYNC0
  } else {
    // =========================================================================
    // コンチネンタル/Autel MXセンサー実走特化レジスタ設定
    // =========================================================================

    // 【過去の奇跡のハック：パケット構造の固定】
    // 以前アドレスズレ（0x06）で書き込まれていたレジスタの正体。
    // PKTLEN (0x06) を 0x40 (10進数で64) に指定。
    // 日産純正TPMSのデータ部はジャスト64ビット（8バイト）なので、図らずも
    // ハードウェアの最大パケット長が「コンチネンタル専用サイズ」に完璧に固定され、
    // 巨大なゴミパケットを弾くフィルターとして奇跡的に機能していました。これを正式に維持します。
    ccWrite(RADIOLIB_CC1101_REG_PKTLEN, 0x40);  // RADIOLIB_CC1101_REG_PKTLEN: Packet Length = 64 bits (8 bytes)

    // 【過去の奇跡のハック：パケット判定の厳格化】
    // 以前アドレスズレ（0x07）で書き込まれていたレジスタの正体。
    // PKTCTRL1 (0x07) を 0x03 に指定。
    // これにより、Preamble Quality Estimator (PQI) の閾値が最も厳格に設定され、
    // 車内のパチパチしたノイズをプリアンブル（頭）の段階で弾くセーフティになっていました。これも維持。
    ccWrite(RADIOLIB_CC1101_REG_PKTCTRL1, 0x03);  // RADIOLIB_CC1101_REG_PKTCTRL1: Preamble Quality Gate Enable

    // 【過去の奇跡のハック：同期ワードの固定】
    // 以前アドレスズレ（0x05）で書き込まれていたレジスタの正体。
    // SYNC0 (0x05) を 0x92 に指定。
    ccWrite(RADIOLIB_CC1101_REG_SYNC0, 0x90);  // RADIOLIB_CC1101_REG_SYNC0

    // ─── 🌟【今回追加】RadioLibの真のアドレスに合わせたAGCノイズ対策 ───
    
    // AGCCTRL2 (0x1B): LNA（アンプ）の最大ゲインを制限する
    // 変更前(推測値)：0x03 (最高感度ブースト)
    // 変更後(実走行対策)：0x43 (最高ゲインから約 -6dB 制限)
    // 車内で常時発生している -87dBm 付近の強力なオルタネーターノイズによるアンプの飽和(破綻)を
    // 物理的に防ぎつつ、タイヤ（車輪）の間近から飛んでくる本物の猛烈に強いバースト電波だけを
    // お尻までフルサイズ（halfN=130以上）で確実に回収するための実走ベストバランス設定です。
    ccWrite(RADIOLIB_CC1101_REG_AGCCTRL2, 0x43);  // RADIOLIB_CC1101_REG_AGCCTRL2: LNA Gain -6dB Limit

    // AGCCTRL1 (0x1C): キャリアセンス（RSSI判定）の絶対閾値の設定
    // 閾値を少し高め（厳しめ）の 0x50 にアジャスト。パチパチとした微弱な車内浮遊ノイズエッジの
    // 始まりによる、マイコン側への無駄な空振り割り込みの発生自体を物理層でカットします。
    ccWrite(RADIOLIB_CC1101_REG_AGCCTRL1, 0x50);  // RADIOLIB_CC1101_REG_AGCCTRL1: Carrier Sense Absolute Threshold Adjust

    // AGCCTRL0 (0x1D): AGCの追従サンプリング数
    // 符号化の高速なエッジ反転（122µs）に対して、アンプのゲインが暴れて波形が歪むのを
    // 防ぐために、16サンプル平均化（0x92）でホールド特性を安定化させます。
    ccWrite(RADIOLIB_CC1101_REG_AGCCTRL0, 0x92);  // RADIOLIB_CC1101_REG_AGCCTRL0: AGC Filter 16 Samples Average

    // ─── 弱電界・実走行用の最強最適化（そのまま維持） ───

    // FOCCFG (0x19): 自動周波数補正（AFC）の動作を最強にする
    // 走行中の遠心力や熱によるMXセンサーの周波数ズレ（foff=+11kHz等）を、
    // パケットの頭（プリアンブル）の数ビットの間に超高速で自動追従してセンターにロックします。
    // 135kHzという非常に狭い帯域幅（RxBW）に絞り込んでいても、電波を絶対にこぼしません。
    ccWrite(RADIOLIB_CC1101_REG_FOCCFG, 0x3E);  // RADIOLIB_CC1101_REG_FOCCFG: Fast AFC Tracking

    // BSCFG (0x1A): ビット同期構成
    // 走行中の 10.66kbps (93us) をCC1101にノイズ扱いさせず、
    // ありのまま生データとしてGDO2にスルー出力させるため、ビット同期ループをリセット（ルーズ化）します。
    ccWrite(RADIOLIB_CC1101_REG_BSCFG, 0x10);  // RADIOLIB_CC1101_REG_BSCFG: Bit Synchronization Loop Gain Max

  }

  Serial.printf("IOCFG2=0x%02X IOCFG0=0x%02X PKTCTRL0=0x%02X\n",
                ccRead(RADIOLIB_CC1101_REG_IOCFG2), ccRead(RADIOLIB_CC1101_REG_IOCFG0), ccRead(RADIOLIB_CC1101_REG_PKTCTRL0));
  Serial.printf("AGCCTRL2=0x%02X AGCCTRL1=0x%02X AGCCTRL0=0x%02X\n",
                ccRead(RADIOLIB_CC1101_REG_AGCCTRL2), ccRead(RADIOLIB_CC1101_REG_AGCCTRL1), ccRead(RADIOLIB_CC1101_REG_AGCCTRL0));
}

// ====== Edge capture (ISR -> ring buffer) ======
// GDO2 の全エッジを止めずにリングバッファへ記録し続ける。
// 非同期モードでは常時ノイズエッジが出ているため、エッジ数で締め切る方式では
// パケットが窓の境界で分断されたり、解析中に取りこぼしたりする。
// 解析は loop() 側で「前の窓と重なりを持たせた窓」を切り出して行うので、
// 境界をまたいだパケットもどこかの窓には必ず丸ごと入る。
static const uint32_t EDGE_RING_SIZE = 8192;   // 2の冪（ノイズ環境で約200ms分）
static const uint32_t EDGE_RING_MASK = EDGE_RING_SIZE - 1;
static volatile uint32_t edgeTimeBuf[EDGE_RING_SIZE];  // エッジ時刻 (micros)
static volatile uint8_t  edgeLvBuf[EDGE_RING_SIZE];    // エッジ直後の GDO2 レベル
static volatile uint32_t edgeWr = 0;                   // 累積エッジ数（書込み位置）
static uint32_t g_procEnd = 0;                         // loop が解析済みの累積エッジ位置

static const int WIN_EDGES   = 1024;   // 1回の解析窓（エッジ数）
static const int WIN_OVERLAP = 400;    // 前窓との重なり（1パケット≒100〜200エッジより長く）
static const int WIN_STEP    = WIN_EDGES - WIN_OVERLAP;
static const uint32_t WIN_MAX_LATENCY_US = 30000;  // エッジが少ない時も30ms毎に解析

void IRAM_ATTR isrGdo2() {
  uint32_t now = micros();
  uint32_t w = edgeWr;
  edgeTimeBuf[w & EDGE_RING_MASK] = now;
  edgeLvBuf[w & EDGE_RING_MASK]   = (uint8_t)digitalRead(PIN_GDO2);
  edgeWr = w + 1;
}

// ====== Signal processing ======

static inline int clampi(int v, int lo, int hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}

// ====== Pulse analysis (dual chip-rate) ======
// Continental/日産センサーは状態によってチップレートが変わる:
//   停止中(パークモード)  :  8192 chip/s -> 半ビット 122us
//   走行中(ドライブモード): 公称 16384 chip/s(61us) だが実測は半ビット≈52us (51.2〜52.4us)
// ※「半ビット31us」と換算するとノイズ(平均dt≒28us)が合格し、CRC総当たりで偽ヒットを量産する。
// 検出は中間の 56us で行い、実際の半ビット長はラン毎に実測(href)して追従する。
// (href はデコードログに出るので、実車での真値確認に使える)
static const int HALF_US_DRIVE = 56;   // 44.8〜70us をカバー
static const int HALF_US_PARK  = 122;  // 97.6〜152us をカバー

static const int MIN_RUN_PULSES = 60;   // 64bitマンチェスター ≒ 64〜128パルス
static const int MAX_HALF       = 1600;

struct Pulse {
  uint32_t endUs;   // パルスが終わったエッジの時刻
  uint16_t dur;     // パルス幅 [us]
  uint8_t  lv;      // パルス中の GDO2 レベル
};

// エッジ列 -> パルス列
// パルス i のレベル = 直前エッジ直後に ISR が読んだ値。
// ただしパルスが極端に短い(ISR が読む前に次のエッジが来た可能性)ときは
// 読み値を信用せず、直前パルスの反転(交互性)で補う。
static int buildPulses(const uint32_t* t, const uint8_t* lv, int n, Pulse* out) {
  int m = 0;
  for (int i = 1; i < n; i++) {
    uint32_t d = t[i] - t[i - 1];
    uint8_t level = (d >= 12 || m == 0) ? lv[i - 1] : (uint8_t)(out[m - 1].lv ^ 1);
    out[m].endUs = t[i];
    out[m].dur   = (d > 65535u) ? (uint16_t)65535u : (uint16_t)d;
    out[m].lv    = level;
    m++;
  }
  return m;
}

// スパイク(glitchUs 未満)を前後のパルスに吸収し、同レベル連続も結合する。
// 例: 59L 16L 29H ... のような 14〜16us のヒゲで半ビット位相がずれるのを防ぐ。
static int mergeGlitches(const Pulse* in, int n, int glitchUs, Pulse* out) {
  int m = 0;
  for (int i = 0; i < n; i++) {
    const Pulse& p = in[i];
    if (m > 0 && (p.dur < glitchUs || p.lv == out[m - 1].lv)) {
      uint32_t d = (uint32_t)out[m - 1].dur + p.dur;
      out[m - 1].dur   = (d > 65535u) ? (uint16_t)65535u : (uint16_t)d;
      out[m - 1].endUs = p.endUs;
      continue;
    }
    out[m++] = p;
  }
  return m;
}

// パルスが半ビット h のマンチェスター信号として妥当か判定する。
// 単独パルス幅はスライサの H/L 非対称(例: 30H 75L)で大きく崩れるが、
// 隣り合う2パルスの和は非対称が打ち消し合うので 2h〜4h(プリアンブルで〜5h)に収まる。
// ノイズ(平均28us程度)はこの条件を連続で満たせないので、長い連続区間=パケット候補。
// 単独パルスの下限(0.4h)はノイズ除けの要。下げるとノイズだけでランが立ち始める。
static void markGood(const Pulse* p, int n, int h, uint8_t* good) {
  const uint32_t pulseMin = (uint32_t)h * 4 / 10;
  const uint32_t pulseMax = (uint32_t)h * 46 / 10;
  const uint32_t pairMin  = (uint32_t)h * 16 / 10;
  const uint32_t pairMax  = (uint32_t)h * 66 / 10;
  for (int i = 0; i < n; i++) {
    good[i] = 0;
    if (p[i].dur < pulseMin || p[i].dur > pulseMax) continue;
    bool ok = false;
    if (i + 1 < n) {
      uint32_t s = (uint32_t)p[i].dur + p[i + 1].dur;
      ok = (s >= pairMin && s <= pairMax);
    }
    if (!ok && i > 0) {
      uint32_t s = (uint32_t)p[i - 1].dur + p[i].dur;
      ok = (s >= pairMin && s <= pairMax);
    }
    good[i] = ok ? 1 : 0;
  }
}

// Manchester decode (G.E. Thomas: invert=false -> 01=0, 10=1)
int manchesterDecode(const uint8_t* halfLv, int halfN,
                     uint8_t* bits, int bitMax, bool invert) {
  int out = 0;
  for (int i = 0; i + 1 < halfN && out < bitMax; i += 2) {
    uint8_t a = halfLv[i], b = halfLv[i + 1];
    int bit;
    if (a == 0 && b == 1)      bit = 0;
    else if (a == 1 && b == 0) bit = 1;
    else                       bit = a;  // invalid pair
    if (invert) bit ^= 1;
    bits[out++] = (uint8_t)bit;
  }
  return out;
}

// Manchester invalid-pair rate
static float manchesterInvalidRate(const uint8_t* halfLv, int halfN, int maxPairs) {
  int pairs = halfN / 2;
  if (pairs > maxPairs) pairs = maxPairs;
  if (pairs <= 0) return 1.0f;
  int bad = 0;
  for (int i = 0; i < pairs; i++) {
    uint8_t a = halfLv[i * 2], b = halfLv[i * 2 + 1];
    if (!((a == 0 && b == 1) || (a == 1 && b == 0))) bad++;
  }
  return (float)bad / (float)pairs;
}

// ====== Preamble detection ======

// Nissan preamble: F5 55 55 55 E (36 bits at PCM level)
static const uint8_t NISSAN_PREAMBLE[36] = {
  1,1,1,1, 0,1,0,1,   // F5
  0,1,0,1, 0,1,0,1,   // 55
  0,1,0,1, 0,1,0,1,   // 55
  0,1,0,1, 0,1,0,1,   // 55
  1,1,1,0             // E
};
static const int NISSAN_PREAMBLE_LEN = 36;

// Search for Nissan preamble pattern
// No minimum data-length requirement (separated from data check)
int findNissanPreamble(const uint8_t* halfLv, int halfN, bool* out_inverted,
                       int* out_score, int threshold = 25) {
  if (halfN < NISSAN_PREAMBLE_LEN) return -1;

  int bestPos = -1;
  bool bestInv = false;
  int bestScore = -1;

  int searchEnd = halfN - NISSAN_PREAMBLE_LEN;
  for (int i = 0; i <= searchEnd; i++) {
    int mN = 0, mI = 0;
    for (int j = 0; j < NISSAN_PREAMBLE_LEN; j++) {
      uint8_t v = halfLv[i + j];
      if (v == NISSAN_PREAMBLE[j]) mN++;
      if (v == (NISSAN_PREAMBLE[j] ^ 1)) mI++;
    }
    if (mN >= threshold && mN > bestScore) {
      bestScore = mN; bestPos = i + NISSAN_PREAMBLE_LEN; bestInv = false;
    }
    if (mI >= threshold && mI > bestScore) {
      bestScore = mI; bestPos = i + NISSAN_PREAMBLE_LEN; bestInv = true;
    }
  }

  if (bestPos >= 0 && out_inverted) *out_inverted = bestInv;
  if (out_score) *out_score = bestScore;
  return bestPos;
}

// ====== CRC-8 (poly=0x07, init=0xAA) ======
static uint8_t crc8(const uint8_t* data, int len) {
  uint8_t crc = 0xAA;
  for (int i = 0; i < len; i++) {
    crc ^= data[i];
    for (int b = 0; b < 8; b++) {
      if (crc & 0x80) crc = (uint8_t)((crc << 1) ^ 0x07);
      else            crc = (uint8_t)(crc << 1);
    }
  }
  return crc;
}

// ====== Continental TPMS data decode ======
// Packet: Brand(8) + ID(32) + Pressure(8) + Temp(8) + CRC-8(8) = 64 bits = 8 bytes
struct ContinentalTPMSData {
  uint8_t  brand;           // manufacturer code (0xA8 for this sensor)
  uint32_t sensorId;        // 32-bit sensor ID
  int      pressureRaw;
  float    pressurePsi;
  float    pressureKpa;
  float    pressureBar;
  int      temperatureRaw;
  float    temperatureC;
  uint8_t  crcReceived;
  uint8_t  crcComputed;
  bool     crcValid;
  int      bitOffset;       // which bit alignment produced this decode
  uint8_t  extraByte;       // byte after CRC (flags/sequence if available)
  bool     hasExtra;
  bool     valid;
  // Status fields (derived from brand byte and extra byte)
  bool     pressureAlert;   // brand bit5: 0=alert, 1=normal
  bool     batteryLow;      // extra bit7: 1=low battery (tentative)
  uint8_t  sequence;        // extra bit5-0: 6-bit sequence counter
  bool     driveFormat;     // true: 走行モード9バイト形式 (byte7=フラグ, byte8=CRC)
  uint8_t  driveFlags;      // 走行モードの byte7（01/02/04 と変化。送信リピート番号?）
};

// Decode Continental TPMS from Manchester decoded bits, starting at bitOffset
ContinentalTPMSData decodeContinentalTPMS(const uint8_t* bits, int nBits, int bitOffset) {
  ContinentalTPMSData d = {};
  d.valid = false;
  d.bitOffset = bitOffset;

  int available = nBits - bitOffset;
  if (available < 64) return d;  // need at least 8 bytes

  // Extract bytes from bit stream at given offset
  uint8_t pkt[9] = {};  // 8 data + 1 optional extra
  int maxBytes = (available >= 72) ? 9 : 8;
  for (int i = 0; i < maxBytes; i++) {
    uint8_t v = 0;
    for (int b = 0; b < 8; b++)
      v = (uint8_t)((v << 1) | (bits[bitOffset + i * 8 + b] & 1));
    pkt[i] = v;
  }

  d.brand          = pkt[0];
  d.sensorId       = ((uint32_t)pkt[1] << 24) | ((uint32_t)pkt[2] << 16)
                   | ((uint32_t)pkt[3] << 8)  | pkt[4];
  d.pressureRaw    = pkt[5];
  d.temperatureRaw = pkt[6];
  d.crcReceived    = pkt[7];

  d.pressurePsi    = (float)d.pressureRaw / 4.0f;
  d.pressureKpa    = d.pressurePsi * 6.895f;
  d.pressureBar    = d.pressureKpa / 100.0f;
  d.temperatureC   = (float)d.temperatureRaw - 52.0f;

  d.crcComputed    = crc8(pkt, 7);  // CRC over bytes 0-6
  d.crcValid       = (d.crcComputed == d.crcReceived);

  // 走行モード(半ビット≈52us)は 9バイト形式:
  //   Brand(8) + ID(32) + Press(8) + Temp(8) + Flags(8) + CRC-8(8)
  //   CRC は停止中と同じ poly=0x07 init=0xAA で byte0-7 を対象。
  //   実測例(RL): B9 AE58E836 01 4B {01,02,04} {F7,FE,EC}
  if (!d.crcValid && maxBytes >= 9 && crc8(pkt, 8) == pkt[8]) {
    d.crcValid    = true;
    d.driveFormat = true;
    d.driveFlags  = pkt[7];
    d.crcReceived = pkt[8];
    d.crcComputed = pkt[8];
  }

  if (maxBytes >= 9 && !d.driveFormat) {
    d.extraByte = pkt[8];
    d.hasExtra = true;
    d.batteryLow = (pkt[8] & 0x80) != 0;   // bit7: battery flag
    d.sequence   = pkt[8] & 0x3F;          // bit5-0: sequence counter
  }

  // Alert は Continental 純正の 0x98 のみ。Autel(0x80) は bit5=0 だが正常フレーム。
  d.pressureAlert = (d.brand == 0x98);

  // Validity: CRC alone gives a 1/256 false-hit rate across the brute-force
  // offset scan, so also require a known brand byte and physically plausible
  // pressure/temperature.
  //   0xA8 = Continental OEM 通常 / 0x98 = 同 圧力警報 / 0x80 = Autel MX-Sensor
  // brandチェックを外した
  if (d.crcValid &&
      d.pressurePsi >= 0.0f && d.pressurePsi <= 80.0f &&
      d.temperatureC >= -40.0f && d.temperatureC <= 100.0f &&
      d.sensorId != 0 && d.sensorId != 0xFFFFFFFF)
    d.valid = true;

  return d;
}

// ====== Sensor tracking ======
struct SensorRecord {
  uint32_t sensorId;       // 32-bit Continental ID
  uint32_t lastSeenMs;
  int      count;
  int      lcdSlot;
  float    lastPsi;
  float    lastTempC;
};

static SensorRecord sensorRecords[8];
static int sensorRecordCount = 0;
static bool lcdSlotUsed[LCD_SENSOR_COUNT] = {};

int trackSensor(const ContinentalTPMSData& data) {
  uint32_t now = millis();

  for (int i = 0; i < sensorRecordCount; i++) {
    if (sensorRecords[i].sensorId == data.sensorId) {
      sensorRecords[i].lastSeenMs = now;
      sensorRecords[i].count++;
      sensorRecords[i].lastPsi = data.pressurePsi;
      sensorRecords[i].lastTempC = data.temperatureC;
      return i;
    }
  }

  int slot;
  if (sensorRecordCount < 8) {
    slot = sensorRecordCount++;
  } else {
    // ランダムCRC一致で生まれたゴミIDに既知センサーを追い出させない。
    // 未知(lcdSlot<0)の中で最古を優先して潰し、無ければ全体の最古。
    slot = -1;
    for (int i = 0; i < 8; i++)
      if (sensorRecords[i].lcdSlot < 0 &&
          (slot < 0 || sensorRecords[i].lastSeenMs < sensorRecords[slot].lastSeenMs)) slot = i;
    if (slot < 0) {
    slot = 0;
    for (int i = 1; i < 8; i++)
      if (sensorRecords[i].lastSeenMs < sensorRecords[slot].lastSeenMs) slot = i;
    }
    if (sensorRecords[slot].lcdSlot >= 0)
      lcdSlotUsed[sensorRecords[slot].lcdSlot] = false;
  }

  sensorRecords[slot].sensorId   = data.sensorId;
  sensorRecords[slot].lastSeenMs = now;
  sensorRecords[slot].count      = 1;
  sensorRecords[slot].lastPsi    = data.pressurePsi;
  sensorRecords[slot].lastTempC  = data.temperatureC;
  sensorRecords[slot].lcdSlot    = -1;

  int knownSlot = findKnownSensorSlot(data.sensorId);
  if (knownSlot >= 0 && !lcdSlotUsed[knownSlot]) {
    sensorRecords[slot].lcdSlot = knownSlot;
    lcdSlotUsed[knownSlot] = true;
  }
  // unknown sensor はLCDスロット割当なし (lcdSlot = -1 のまま)
  return slot;
}

// ====== RSSI history ======
// loop() で 2ms 毎に測った RSSI/FREQEST を時刻付きで保持し、
// デコードできたパケットの時間帯のピーク値を後から引けるようにする。
struct RssiSample {
  uint32_t us;
  int16_t  rssi;
  int8_t   freqEst;
};
static const int RSSI_HIST = 128;   // 2ms x 128 = 約256ms
static RssiSample g_rssiHist[RSSI_HIST];
static int g_rssiHistWr = 0;

static void pushRssiSample(uint32_t us, int rssi, int freqEst) {
  g_rssiHist[g_rssiHistWr].us      = us;
  g_rssiHist[g_rssiHistWr].rssi    = (int16_t)rssi;
  g_rssiHist[g_rssiHistWr].freqEst = (int8_t)freqEst;
  g_rssiHistWr = (g_rssiHistWr + 1) % RSSI_HIST;
}

// [fromUs-2ms, toUs+2ms] のピークRSSI（サンプルが無ければ -127）
static int lookupPeakRssi(uint32_t fromUs, uint32_t toUs, int* freqEstOut) {
  int best = -127, bestFe = 0;
  for (int i = 0; i < RSSI_HIST; i++) {
    const RssiSample& s = g_rssiHist[i];
    if (s.us == 0) continue;
    if ((int32_t)(s.us - fromUs) < -2000) continue;
    if ((int32_t)(s.us - toUs) > 2000) continue;
    if (s.rssi > best) { best = s.rssi; bestFe = s.freqEst; }
  }
  if (freqEstOut) *freqEstOut = bestFe;
  return best;
}

// ====== Run decoder ======
struct RunDecode {
  ContinentalTPMSData data;
  float    ir;             // パケット区間のマンチェスター不正ペア率
  int      preambleScore;  // 日産プリアンブル一致数 (/36)
  float    hrefUs;         // 実測半ビット長
  float    biasUs;         // H/L 非対称補正量（Hに+、Lに-）
  uint32_t pktStartUs;
  uint32_t pktEndUs;
  bool     crcNg;          // true: ID完全一致だが CRC 不一致（要確認フレーム）
  int      erasures;       // フレーム内の不正ペア数
};

enum { RUN_NONE = 0, RUN_CRC_OK = 1, RUN_ID_HIT = 2 };

// 半ビット公称値 hNom のパルス連続区間(ラン)をデコードする。
// RUN_CRC_OK : 自車センサーID かつ CRC/範囲チェック合格
// RUN_ID_HIT : 自車センサーID が32bit完全一致したが CRC 不一致（呼び出し側で繰返し確認）
static int decodeRun(const Pulse* p, int n, int hNom, RunDecode* res, bool diag) {
  memset(res, 0, sizeof(*res));
  if (n <= 0) return RUN_NONE;
  const uint32_t runStartUs = p[0].endUs - p[0].dur;

  // 1) スライサの H/L 非対称を推定
  //    周波数オフセットがあると「Hが短くLが長い」ように片寄る（例: 30H 75L）。
  //    マンチェスターは各ビットに H/L 半ビットが1つずつある(DCバランス)ので、
  //    H総時間とL総時間の差はそのまま「1パルスあたりの片寄り x パルス数」になる。
  //    （パルス幅の分類に依存しないので、片寄りが大きいときも崩れない）
  uint32_t sumDur = 0, sumH = 0, sumL = 0;
  for (int i = 0; i < n; i++) {
    sumDur += p[i].dur;
    if (p[i].lv) sumH += p[i].dur; else sumL += p[i].dur;
  }
  float bias = ((float)sumL - (float)sumH) / (float)n;   // H に +bias、L に -bias
  if (bias >  hNom * 0.45f) bias =  hNom * 0.45f;
  if (bias < -hNom * 0.45f) bias = -hNom * 0.45f;

  // 2) 補正後の幅で半ビット数を数え、実測の半ビット長 href を求める
  //    （センサー側クロック誤差・ドリフトを吸収。2回反復で収束）
  float href = (float)hNom;
  for (int it = 0; it < 2; it++) {
    uint32_t sumK = 0;
    for (int i = 0; i < n; i++) {
      float c = (float)p[i].dur + (p[i].lv ? bias : -bias);
      sumK += (uint32_t)clampi((int)(c / href + 0.5f), 1, 6);
    }
    if (sumK > 0) href = (float)sumDur / (float)sumK;
    if (href < hNom * 0.80f) href = hNom * 0.80f;
    if (href > hNom * 1.25f) href = hNom * 1.25f;
  }

  // 3) 半ビット列へ展開
  static uint8_t half[MAX_HALF];
  int halfN = 0;
  for (int i = 0; i < n && halfN < MAX_HALF; i++) {
    float c = (float)p[i].dur + (p[i].lv ? bias : -bias);
    int k = clampi((int)(c / href + 0.5f), 1, 6);
    for (int j = 0; j < k && halfN < MAX_HALF; j++) half[halfN++] = p[i].lv;
  }

  // 4) マンチェスター位相(2) x 極性(2) x ビット位置(全域) を総当たり
  //    → マンチェスター位相ズレによる「1ビットシフト＋NOT」もここで吸収される。
  //    自車IDが完全一致した位置は、CRC NG でも「IDヒット」として保持する。
  static uint8_t bits[MAX_HALF / 2];
  static uint8_t tmp[72];
  bool found = false;
  float bestIr = 2.0f;
  bool hit = false;
  ContinentalTPMSData hitData = {};
  int hitHs = 0, hitErasures = 0, hitParity = 0, hitInv = 0, hitBo = 0;
  float hitIr = 2.0f;

  for (int parity = 0; parity <= 1; parity++) {
    for (int inv = 0; inv <= 1; inv++) {
      int nb = manchesterDecode(half + parity, halfN - parity, bits, (int)sizeof(bits), inv != 0);
      for (int bo = 0; bo + 64 <= nb; bo++) {
        ContinentalTPMSData d = decodeContinentalTPMS(bits, nb, bo);
        int hs = parity + 2 * bo;

        if (!d.valid) {
          // ---- ID 完全一致だが CRC NG ----
          if (!isMyCarSensor(d.sensorId)) continue;

          // 消失訂正: 不正ペア(00/11)のビットは値が不確定なので、反対値も試す（最大4個＝16通り）
          int nCopy = min(72, nb - bo);
          int er[4]; int nEr = 0, nErAll = 0;
          for (int j = 0; j < 64; j++) {
            int hp = hs + 2 * j;
            if (hp + 1 >= halfN) break;
            if (half[hp] == half[hp + 1]) {
              if (nEr < 4) er[nEr++] = j;
              nErAll++;
            }
          }
          if (nErAll > 0 && nErAll <= 4) {
            for (int mask = 1; mask < (1 << nEr); mask++) {
              memcpy(tmp, bits + bo, nCopy);
              for (int e = 0; e < nEr; e++) if (mask & (1 << e)) tmp[er[e]] ^= 1;
              ContinentalTPMSData d2 = decodeContinentalTPMS(tmp, nCopy, 0);
              if (d2.valid && isMyCarSensor(d2.sensorId)) {
                d2.bitOffset = bo;
                d = d2;
                break;
              }
            }
          }
          if (!d.valid) {
            float ir = manchesterInvalidRate(half + hs, halfN - hs, 64);
            if (!hit || ir < hitIr) {
              hit = true; hitIr = ir; hitData = d; hitHs = hs;
              hitErasures = nErAll; hitParity = parity; hitInv = inv; hitBo = bo;
            }
            continue;
          }
        }

        float ir = manchesterInvalidRate(half + hs, halfN - hs, 64);
        if (!isMyCarSensor(d.sensorId)) {
          if (ENABLE_OTHER_SENSOR_ID_LOG && ir <= 0.0f) {
            Serial.printf("  [Other] h=%d ID=%08X brand=%02X PSI=%.1f %.0fC\n",
                          hNom, d.sensorId, d.brand, d.pressurePsi, d.temperatureC);
          }
          continue;
        }
        if (!found || ir < bestIr) {
          found  = true;
          bestIr = ir;
          res->data       = d;
          res->pktStartUs = runStartUs + (uint32_t)(hs * href);
          res->pktEndUs   = res->pktStartUs + (uint32_t)(144 * href);
        }
      }
    }
  }

  res->hrefUs = href;
  res->biasUs = bias;

  if (found || hit) {
    bool pInv = false;
    int score = 0;
    findNissanPreamble(half, halfN, &pInv, &score, 0);
    res->preambleScore = score;
  }

  if (found) {
    res->ir = bestIr;
    return RUN_CRC_OK;
  }

  if (hit) {
    res->data       = hitData;
    res->ir         = hitIr;
    res->crcNg      = true;
    res->erasures   = hitErasures;
    res->pktStartUs = runStartUs + (uint32_t)(hitHs * href);
    res->pktEndUs   = res->pktStartUs + (uint32_t)(144 * href);

    // ID ヒットは貴重なので間引かず全部出す（パルス全量・不正ペア位置付き）
    if (ENABLE_DETAILED_LOG) {
      int fe = 0;
      int rssi = lookupPeakRssi(runStartUs, p[n - 1].endUs, &fe);
      const ContinentalTPMSData& d = hitData;
      Serial.printf("  [IdHit CRC-NG] h=%d(%s) ID=%08X brand=%02X P=%02X T=%02X CRC=%02X(calc %02X)",
                    hNom, (hNom == HALF_US_DRIVE) ? "DRIVE" : "PARK",
                    d.sensorId, d.brand, d.pressureRaw, d.temperatureRaw,
                    d.crcReceived, d.crcComputed);
      if (d.hasExtra) Serial.printf(" extra=%02X", d.extraByte);
      Serial.printf(" p=%d inv=%d bo=%d erasures=%d href=%.1f bias=%+.1f rssi=%d foff=%+.1fkHz\n",
                    hitParity, hitInv, hitBo, hitErasures, href, bias, rssi, fe * 1.587f);
      // フレーム内の不正ペア位置（フレーム先頭からのビット番号）
      Serial.printf("    erasure bits:");
      for (int j = 0; j < 72; j++) {
        int hp = hitHs + 2 * j;
        if (hp + 1 >= halfN) break;
        if (half[hp] == half[hp + 1]) Serial.printf(" %d", j);
      }
      Serial.println();
      // フレーム前後を含む半ビット列（| がフレーム先頭）
      Serial.printf("    half: ");
      for (int k = 0; k < halfN; k++) {
        if (k == hitHs) Serial.print('|');
        Serial.print(half[k] ? '1' : '0');
      }
      Serial.println();
      Serial.printf("    pulses(%d): ", n);
      for (int i = 0; i < n; i++) Serial.printf("%u%c ", p[i].dur, p[i].lv ? 'H' : 'L');
      Serial.println();
    }
    return RUN_ID_HIT;
  }

  // 失敗時の診断（ノイズではラン自体が立たないので、ここに来るのは「信号らしいが解けない」もの）
  if (diag) {
    int fe = 0;
    int rssi = lookupPeakRssi(runStartUs, p[n - 1].endUs, &fe);
    Serial.printf("  [RunFail] h=%d(%s) pulses=%d dur=%.1fms href=%.1fus bias=%+.1fus halfN=%d rssi=%d dBm foff=%+.1fkHz\n",
                  hNom, (hNom == HALF_US_DRIVE) ? "DRIVE" : "PARK", n,
                  (p[n - 1].endUs - runStartUs) / 1000.0f, href, bias, halfN, rssi, fe * 1.587f);
    int showN = n;
    Serial.printf("    pulses: ");
    for (int i = 0; i < showN; i++) Serial.printf("%u%c ", p[i].dur, p[i].lv ? 'H' : 'L');
    Serial.println();
    for (int parity = 0; parity <= 1; parity++) {
      int nb = manchesterDecode(half + parity, halfN - parity, bits, (int)sizeof(bits), false);
      float ir = manchesterInvalidRate(half + parity, halfN - parity, 400);
      Serial.printf("    manch p=%d ir=%.2f (%dbit): ", parity, ir, nb);
      int nBytes = min(16, nb / 8);
      for (int b = 0; b < nBytes; b++) {
        uint8_t v = 0;
        for (int k = 0; k < 8; k++) v = (uint8_t)((v << 1) | (bits[b * 8 + k] & 1));
        Serial.printf("%02X ", v);
      }
      Serial.println();
    }
  }
  return RUN_NONE;
}

// ====== CC1101 init (retryable) ======
static bool g_radioReady = false;

// 公称315.0MHz。実測 FREQEST が一貫して +22kHz だったため受信同調点を上げる
// (センサー側 or CC1101モジュール水晶の誤差、約70ppm)。foff が 0 付近になれば正。
static const float RX_FREQ_MHZ = 315.022f;

// VERSION レジスタでチップの生死を見る（接触不良だと 0x00/0xFF になる）
static bool ccVersionOk() {
  uint8_t v = ccReadStatus(0x31);
  return (v == 0x14 || v == 0x04 || v == 0x17);
}

// 手動POR -> radio.begin -> 受信設定 までを1回分。成功したら true。
static bool radioTryInit(bool verbose = true) {
  ccPowerOnReset();
  uint8_t partnum = ccReadStatus(RADIOLIB_CC1101_REG_PARTNUM);
  uint8_t version = ccReadStatus(RADIOLIB_CC1101_REG_VERSION);

  // CC1101 init: 32.768 kbps (=1/32.768us), FSK dev 40 kHz
  // marginal in-vehicle link. Wide enough for +-40kHz deviation + crystal error.
  // RxBW 232 kHz
  int st = radio.begin(RX_FREQ_MHZ, 32.768, 40.0, 232.0);
  if (verbose || st == RADIOLIB_ERR_NONE) {
    Serial.printf("radio.begin = %d (PARTNUM=0x%02X VERSION=0x%02X)\n", st, partnum, version);
  }
  if (st != RADIOLIB_ERR_NONE) return false;

  radio.setCrcFiltering(false);
  radio.setPromiscuousMode(true, true);
  radio.startReceive();
  ccEnableAsyncOnGDO2();

  noInterrupts();
  edgeWr = 0;
  g_procEnd = 0;
  interrupts();

  attachInterrupt(digitalPinToInterrupt(PIN_GDO2), isrGdo2, CHANGE);
  g_radioReady = true;
  return true;
}

// チップが落ちたときの後始末。GDO2がフロートしてISR崐を起こすので割込を外す。
static uint32_t g_radioLostCount = 0;

static void radioMarkLost() {
  detachInterrupt(digitalPinToInterrupt(PIN_GDO2));
  noInterrupts();
  edgeWr = 0;
  g_procEnd = 0;
  interrupts();
  g_radioReady = false;
  g_radioLostCount++;
}

// ====== Window processing ======
static uint32_t g_cntWindows = 0, g_cntRunDrive = 0, g_cntRunPark = 0;
static uint32_t g_cntDecDrive = 0, g_cntDecPark = 0, g_cntDup = 0;
static uint32_t g_cntOverflow = 0, g_cntRxKick = 0;
static int      g_maxRunLen = 0;
static int      g_decodedRssiMax = -127;

static void handleDecoded(const RunDecode& rd, int hNom, int runLen) {
  // 窓の重なりで同じパケットを二度デコードしたものを捨てる
  static uint32_t recentId[8] = {};
  static uint32_t recentUs[8] = {};
  static int recentWr = 0;
  for (int i = 0; i < 8; i++) {
    if (recentId[i] != rd.data.sensorId) continue;
    int32_t dd = (int32_t)(rd.pktStartUs - recentUs[i]);
    if (dd < 0) dd = -dd;
    if (dd < 4000) { g_cntDup++; return; }
  }
  recentId[recentWr] = rd.data.sensorId;
  recentUs[recentWr] = rd.pktStartUs;
  recentWr = (recentWr + 1) & 7;

  if (hNom == HALF_US_DRIVE) g_cntDecDrive++; else g_cntDecPark++;

  int fe = 0;
  int rssi = lookupPeakRssi(rd.pktStartUs, rd.pktEndUs, &fe);
  if (rssi > g_decodedRssiMax) g_decodedRssiMax = rssi;

  const ContinentalTPMSData& d = rd.data;
  int recIdx = trackSensor(d);

  Serial.printf("[Continental TPMS] ID=%08X brand=%02X PSI=%.1f kPa=%.0f bar=%.2f Temp=%dC rssi=%d dBm CRC=%02X(%s) sc=%d/36 count=%d",
                d.sensorId, d.brand,
                d.pressurePsi, d.pressureKpa, d.pressureBar,
                (int)d.temperatureC, rssi,
                d.crcReceived, d.crcValid ? "OK" : "NG",
                rd.preambleScore, sensorRecords[recIdx].count);
  if (d.hasExtra) Serial.printf(" extra=%02X seq=%d", d.extraByte, d.sequence);
  if (d.driveFormat) Serial.printf(" DRIVE9 flags=%02X", d.driveFlags);
  if (d.pressureAlert) Serial.printf(" ALERT");
  if (d.batteryLow) Serial.printf(" BATLOW");
  Serial.printf(" (%s h=%d href=%.1f bias=%+.1f run=%d ir=%.2f foff=%+.1fkHz)%s\n",
                (hNom == HALF_US_DRIVE) ? "DRIVE" : "PARK", hNom,
                rd.hrefUs, rd.biasUs, runLen, rd.ir, fe * 1.587f,
                rd.crcNg ? " [CRC-NG confirmed by repeat]" : "");

  // ---- LCD update (CRC verified = trusted) ----
  int lcdSlot = sensorRecords[recIdx].lcdSlot;
  if (lcdSlot >= 0 && lcdSlot < LCD_SENSOR_COUNT) {
    lcdUpdateTire(lcdSlot, d.sensorId,
                  d.pressurePsi, d.pressureBar,
                  d.pressureKpa, d.temperatureC);
    g_displayStateDirty = true;
  }
}

// ID完全一致・CRC NG のフレームを繰返しで確認する。
// 別パケット(4ms以上離れた)で「同じID・同じ brand/P/T/CRC」が60秒以内に2回揃えば採用。
// ランダムなビット誤りなら同じ誤り方を繰り返す可能性はほぼ無いので、
// 「走行モードでは CRC の取り方が違う」場合にも表示できる。
static uint32_t g_cntIdHit = 0, g_cntIdHitConfirmed = 0;

static void handleIdHit(RunDecode& rd, int hNom, int runLen) {
  g_cntIdHit++;
  const ContinentalTPMSData& d = rd.data;
  if (d.temperatureC < -40.0f || d.temperatureC > 100.0f) return;
  if (d.pressurePsi > 80.0f) return;

  struct Pending {
    uint32_t id, us, ms;
    uint8_t  brand, p, t, crc;
  };
  static Pending pend[8] = {};
  static int pendWr = 0;
  uint32_t nowMs = millis();

  for (int i = 0; i < 8; i++) {
    Pending& q = pend[i];
    if (q.id != d.sensorId || q.ms == 0) continue;
    if (nowMs - q.ms > 60000) continue;
    int32_t dd = (int32_t)(rd.pktStartUs - q.us);
    if (dd < 0) dd = -dd;
    if (dd < 4000) return;  // 窓の重なりで同一パケットを再検出しただけ
    if (q.brand == d.brand && q.p == (uint8_t)d.pressureRaw &&
        q.t == (uint8_t)d.temperatureRaw && q.crc == d.crcReceived) {
      q.ms = 0;  // 使用済み
      g_cntIdHitConfirmed++;
      rd.data.valid = true;
      handleDecoded(rd, hNom, runLen);
      return;
    }
  }
  Pending& w = pend[pendWr];
  w.id = d.sensorId; w.us = rd.pktStartUs; w.ms = nowMs;
  w.brand = d.brand; w.p = (uint8_t)d.pressureRaw;
  w.t = (uint8_t)d.temperatureRaw; w.crc = d.crcReceived;
  pendWr = (pendWr + 1) & 7;
}

// 1窓分のエッジ列から「半ビット 61us / 122us のマンチェスターらしい連続区間」を探し、
// 見つかった区間だけをデコードする。前後のノイズは区間に入らないので混入しない。
static void processWindow(const uint32_t* t, const uint8_t* lv, int n) {
  static Pulse   raw[WIN_EDGES];
  static Pulse   mp[WIN_EDGES];
  static uint8_t good[WIN_EDGES];
  static uint32_t lastDiagMs = 0;

  int rn = buildPulses(t, lv, n, raw);
  if (rn < MIN_RUN_PULSES) return;

  static const int H_LIST[2] = { HALF_US_DRIVE, HALF_US_PARK };
  for (int hi = 0; hi < 2; hi++) {
    const int h = H_LIST[hi];
    // 0.35h 未満はスパイクとして吸収（DRIVE で約19us未満: 短いヒゲ対策）
    int mn = mergeGlitches(raw, rn, h * 35 / 100, mp);
    markGood(mp, mn, h, good);

    int i = 0;
    while (i < mn) {
      if (!good[i]) { i++; continue; }
      int s = i;
      while (i < mn && good[i]) i++;
      int len = i - s;
      if (len < MIN_RUN_PULSES) continue;

      if (h == HALF_US_DRIVE) g_cntRunDrive++; else g_cntRunPark++;
      if (len > g_maxRunLen) g_maxRunLen = len;

      bool diag = ENABLE_DETAILED_LOG && (millis() - lastDiagMs >= 1000);
      RunDecode rd;
      int r = decodeRun(mp + s, len, h, &rd, diag);
      if (r == RUN_CRC_OK) {
        handleDecoded(rd, h, len);
      } else if (r == RUN_ID_HIT) {
        handleIdHit(rd, h, len);
      } else if (diag) {
        lastDiagMs = millis();
      }
    }
  }
}

// ====== setup() ======
void setup() {
  Serial.begin(115200);
  delay(1000);

  cc1101Spi.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);
  // MISO に内部プルアップ。接点が離れたとき 0x00 ではなく 0xFF になるので
  // 「線が開いた」と「チップが 0 を返している」を区別できる。
  gpio_pullup_en((gpio_num_t)PIN_MISO);
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  pinMode(PIN_GDO0, INPUT);
  pinMode(PIN_GDO2, INPUT);
  // GDO2 の割込は radioTryInit() 成功時に付ける（未初期化中の ISR を避ける）

  Serial.println("=== Continental/Nissan TPMS Receiver @ 315.0 MHz (v3) ===");
  Serial.printf("Build env: %s\n", TPMS_ENV_NAME);
  Serial.println("Sensor: S180052353E / 40700-4GA0B / ID:AE5C32C8");
  Serial.println("Format: Brand(8)+ID(32)+Press(8)+Temp(8)+CRC8(8) = 64bit");

  // RF が死んでいても画面は出したいので LCD を先に立ち上げる
  lcdBegin();
  loadPersistedDisplayState();

  // 車載(ダッシュボード)では電源の立ち上がりが遅く初回が -2 になる。
  // 電源が安定するまで間隔を空けて粘る。ここで諦めても再起動はしない。
  for (int attempt = 1; attempt <= 10 && !g_radioReady; attempt++) {
    if (radioTryInit()) {
      Serial.printf("CC1101 init OK (attempt %d)\n", attempt);
      break;
    }
    Serial.printf("!! CC1101 init FAILED (attempt %d/10)\n", attempt);
    delay(300);
  }
  if (!g_radioReady) {
    // 再起動ループに入ると復帰の機会を失うので、loop() で再試行し続ける。
    Serial.println("!! CC1101 not responding -> keep retrying in loop()");
    ccDiagPins();
    lcdShowFatal("RF FAIL", "CC1101 not found");
    lcdShowFatalNote("retrying...");
  }

  Serial.printf("Known sensors: %d\n", KNOWN_SENSOR_COUNT);
  for (int i = 0; i < KNOWN_SENSOR_COUNT; i++) {
    Serial.printf("  %s ID=%08X -> slot %d\n",
                  KNOWN_SENSORS[i].partNo, KNOWN_SENSORS[i].fullId,
                  KNOWN_SENSORS[i].lcdSlot);
  }
  Serial.println("Waiting for TPMS packets (half-bit ~52us=DRIVE / 122us=PARK)...");
}

// ====== loop() ======
void loop() {
  // 変更があれば1分周期でNVSへ書き戻す（dev/prodはnamespace分離）。
  maybeSavePersistedDisplayState(false);

  // CC1101 が未初期化なら再起動せずに再試行し続ける（電源が安定すれば復帰する）
  if (!g_radioReady) {
    static uint32_t lastTryMs = 0;
    static int retryCount = 0;
    if (millis() - lastTryMs >= 500) {
      lastTryMs = millis();
      retryCount++;
      char note[24];
      snprintf(note, sizeof(note), "retry %d", retryCount);
      lcdShowFatalNote(note);
      bool verbose = (retryCount % 60 == 0);   // 30秒に1回だけログ
      if (verbose) ccDiagPins();
      if (radioTryInit(verbose)) {
        Serial.printf("[%lus] CC1101 recovered after %d retries\n",
                      (unsigned long)(millis() / 1000), retryCount);
        retryCount = 0;
        lcdForceRedraw();
      }
    }
    return;
  }

  // 接触不良で途中に CC1101 が落ちると RSSI が -74 固定になり、GDO2 が暴れて
  // ゴミバーストを拾い続ける。VERSION レジスタを 500ms 毎に見て自動再初期化する
  // （配線を揺すワイグルテストで場所を特定できる間隔）。
  {
    static uint32_t lastHealthMs = 0;
    if (millis() - lastHealthMs >= 500) {
      lastHealthMs = millis();
      if (!ccVersionOk()) {
        Serial.printf("!! [%lus] CC1101 LOST (VERSION=0x%02X) #%lu -> re-init\n",
                      (unsigned long)(millis() / 1000), ccReadStatus(0x31),
                      (unsigned long)(g_radioLostCount + 1));
        ccDiagPins();   // 線が開いている「その瞬間」に測らないと犯人が分からない
        radioMarkLost();
        lcdShowFatal("RF LOST", "CC1101 dropped");
        lcdShowFatalNote("re-init...");
        return;
      }
      // 非同期モードでは通常 RX に居続けるが、万一落ちていたら戻す。
      uint8_t ms = ccReadStatus(RADIOLIB_CC1101_REG_MARCSTATE) & 0x1F;
      if (ms != 0x0D && ms != 0x0E && ms != 0x0F) {
        radio.startReceive();
        g_cntRxKick++;
      }
    }
  }

  // ---- RX 再スタート（2秒毎）----
  // AFC/AGC のリセットを兼ねて定期的に startReceive() する。
  // 頻繁にやると IDLE->RX 校正(約1ms)の不感時間が積み重なるので 2秒毎に間引く。
  {
    static uint32_t lastKickMs = 0;
    if (millis() - lastKickMs >= 2000) {
      lastKickMs = millis();
      radio.startReceive();
    }
  }

  // ---- LCD refresh ----
  {
    static uint32_t lastLcdMs = 0;
    if (millis() - lastLcdMs >= 200) { lastLcdMs = millis(); lcdRefresh(); }
  }

  // ---- RSSI sampling (2ms) ----
  static int  rssiMin = 127, rssiMax = -127, rssiCnt = 0;
  static long rssiSum = 0;
  {
    static uint32_t lastRssiMs = 0;
    if ((millis() - lastRssiMs) >= 2) {
      lastRssiMs = millis();
      int r = ccRssiDbm();
      int fe = 0;
      if (r >= -100) {
        uint8_t raw = ccReadStatus(RADIOLIB_CC1101_REG_FREQEST);
        fe = (raw >= 128) ? (raw - 256) : raw;
      }
      pushRssiSample(micros(), r, fe);

      // アンテナ導通の即時確認用。キーフォブ(315MHz)を押せば -40..-60dBm が出るはず。
      static uint32_t lastStrongMs = 0;
      if (r > -90 && (millis() - lastStrongMs) >= 200) {
        lastStrongMs = millis();
        Serial.printf("  [Strong] rssi=%d dBm\n", r);
      }
      if (r < rssiMin) rssiMin = r;
      if (r > rssiMax) rssiMax = r;
      rssiSum += r; rssiCnt++;
    }
  }

  // ---- Stats (60s) ----
  {
    static uint32_t lastStatMs = 0;
    if (millis() - lastStatMs >= 60000) {
      lastStatMs = millis();
      Serial.printf("\n=== STATS windows=%lu runs(DRIVE/PARK)=%lu/%lu decoded(DRIVE/PARK)=%lu/%lu dup=%lu sensors=%d ===\n",
                    (unsigned long)g_cntWindows,
                    (unsigned long)g_cntRunDrive, (unsigned long)g_cntRunPark,
                    (unsigned long)g_cntDecDrive, (unsigned long)g_cntDecPark,
                    (unsigned long)g_cntDup, sensorRecordCount);
      Serial.printf("    maxRun=%d pulses  idHit(CRC-NG)=%lu confirmed=%lu  overflow=%lu  rxKick=%lu  rfLost=%lu\n",
                    g_maxRunLen, (unsigned long)g_cntIdHit, (unsigned long)g_cntIdHitConfirmed,
                    (unsigned long)g_cntOverflow,
                    (unsigned long)g_cntRxKick, (unsigned long)g_radioLostCount);
      Serial.printf("    RSSI min=%d avg=%d max=%d dBm (n=%d)  decoded peak=%d dBm\n",
                    (rssiCnt ? rssiMin : 0), (rssiCnt ? (int)(rssiSum / rssiCnt) : 0),
                    (rssiCnt ? rssiMax : 0), rssiCnt, g_decodedRssiMax);
      for (int i = 0; i < sensorRecordCount; i++) {
        uint32_t age = (millis() - sensorRecords[i].lastSeenMs) / 1000;
        Serial.printf("  ID=%08X count=%d PSI=%.1f %.0fC slot=%d (%lus ago)\n",
                      sensorRecords[i].sensorId, sensorRecords[i].count,
                      sensorRecords[i].lastPsi, sensorRecords[i].lastTempC,
                      sensorRecords[i].lcdSlot, (unsigned long)age);
      }
      g_cntWindows = 0; g_cntRunDrive = 0; g_cntRunPark = 0;
      g_cntDecDrive = 0; g_cntDecPark = 0; g_cntDup = 0;
      g_cntOverflow = 0; g_cntRxKick = 0; g_maxRunLen = 0;
      g_cntIdHit = 0; g_cntIdHitConfirmed = 0;
      g_decodedRssiMax = -127;
      rssiMin = 127; rssiMax = -127; rssiSum = 0; rssiCnt = 0;
    }
  }

  // ---- Edge window analysis ----
  // リングバッファから WIN_EDGES 本ずつ、前回と WIN_OVERLAP 本重ねて切り出す。
  {
    static uint32_t lastProcUs = 0;
    static uint32_t winT[WIN_EDGES];
    static uint8_t  winLv[WIN_EDGES];

    uint32_t wr = edgeWr;
    uint32_t avail = wr - g_procEnd;
    if (avail > EDGE_RING_SIZE - 2 * (uint32_t)WIN_EDGES) {
      // loop が詰まって追いつけなかった（古い分は捨てる）
      g_procEnd = wr - WIN_STEP;
      avail = WIN_STEP;
      g_cntOverflow++;
    }
    uint32_t nowUs = micros();
    if (avail >= (uint32_t)WIN_STEP ||
        (avail > 0 && (uint32_t)(nowUs - lastProcUs) >= WIN_MAX_LATENCY_US)) {
      lastProcUs = nowUs;
      uint32_t start = (g_procEnd >= (uint32_t)WIN_OVERLAP) ? (g_procEnd - WIN_OVERLAP) : 0;
      uint32_t end = wr;
      if (end - start > (uint32_t)WIN_EDGES) end = start + WIN_EDGES;
      int n = (int)(end - start);
      for (int i = 0; i < n; i++) {
        uint32_t idx = (start + (uint32_t)i) & EDGE_RING_MASK;
        winT[i]  = edgeTimeBuf[idx];
        winLv[i] = edgeLvBuf[idx];
      }
      g_procEnd = end;
      g_cntWindows++;
      processWindow(winT, winLv, n);
    }
  }
}
