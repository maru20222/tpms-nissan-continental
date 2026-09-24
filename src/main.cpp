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
//   Half-bit    : ~122us  (Continental short=122)
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

#ifndef ENABLE_BIGDROP_DETAILED_LOG
#define ENABLE_BIGDROP_DETAILED_LOG false
#endif

// ====== receive wait time (ms) ======
static const int RECEIVE_WAIT_TIME_MS = 50;

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

// ====== Burst capture (ISR) ======
static const int MAX_EDGES = 4000;
static const int MIN_EDGES = 40;

volatile uint16_t dtBuf[MAX_EDGES];
volatile uint8_t  lvBuf[MAX_EDGES];
volatile int edgeN = 0;
volatile uint32_t lastEdgeUs = 0;
volatile uint32_t burstStartUs = 0;
volatile uint32_t burstEndUs = 0;
volatile bool burstReady = false;

void IRAM_ATTR isrGdo2() {
  // 【セーフティガード】すでにloop()側のデコード処理が完了していない（前回のデータを処理中）なら、
  // 新しいエッジがきてもバッファを上書きしないよう、即座に何もしないでリターンする。
  if (burstReady) return;
  uint32_t now = micros();
  uint32_t dt = now - lastEdgeUs;
  lastEdgeUs = now;

  // グリッチノイズ（dtが0〜10usの範囲）を吸い込まないようにするため、ここで即座にリターンする。
  if (dt > 0 && dt < 10) return; 

  // バースト（パケット）の一番最初のエッジが届いた瞬間、その時刻を記録しておく。
  if (edgeN == 0) burstStartUs = now;

  // 250エッジを超えたら、即座に loop() 側に引き渡す（バッファ汚染を防ぐ）
  if (edgeN >= 250) {
    burstEndUs = now - dt;
    burstReady = true; // 250本で「1パケット完成」として即座に loop() に引き渡す
    return;            // 251本目以降のノイズエッジは完全シャットアウト（バッファ汚染を防ぐ）
  }

  // =========================================================================
  // 【データ格納エリア】最大4,000本までエッジを配列に保存し続ける
  // =========================================================================
  // 250本ガードがない状態だと、2.5msの隙間が空かない限り、ノイズを吸い込み続け、
  // loop()側の古いセーフティ（576本付近）で無理やり止められるまで、配列にゴミを格納し続けていました。
  if (edgeN < MAX_EDGES) {
    // CC1101のタイマー仕様に合わせ、dtが65.5ms（uint16_tの上限）を超えた場合は飽和（カンスト）させる
    dtBuf[edgeN] = (dt > 65535 ? 65535 : (uint16_t)dt);
    // その瞬間のGDO2ピンのLo/Hi状態をそのまま保存する（マンチェスター復調用）
    lvBuf[edgeN] = (uint8_t)digitalRead(PIN_GDO2);
    edgeN++;
  }
}

// ====== Signal processing ======

static inline int clampi(int v, int lo, int hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}

// Estimate half-bit period from dt histogram (5usウィンドウ・走行ジッター対応型)
int estimateHalfBitUs(const uint16_t* dts, int n, float* peakFrac = nullptr) {
  // 5us刻みのビンを用意 (0〜300us を 60個のビンで管理)
  static uint16_t bins[61];
  memset(bins, 0, sizeof(bins));
  int validCnt = 0;

  for (int i = 0; i < n; i++) {
    int dt = dts[i];
    if (dt >= 8 && dt <= 300) {
      int b = dt / 5;
      if (b >= 0 && b < 61) { bins[b]++; validCnt++; }
    }
  }

  // 🌟【走行時対策】最もパルスが集中している「5us幅の山」を探す
  int bestBin = 24; // デフォルトは 120us 付近 (24 * 5)
  uint16_t bestCnt = 0;
  
  // 15us から 200us の範囲をスキャン
  for (int b = 3; b <= 40; b++) {
    // 自身のビンとその前後1つの計3ビン（計15us幅）の合計で評価する
    // これにより、走行中に 60us, 61us, 62us にブレて分散したパルスを1つの「大きな山」として正しく捕捉できます
    uint16_t sum = bins[b-1] + bins[b] + bins[b+1];
    if (sum > bestCnt) { bestCnt = sum; bestBin = b; }
  }

  // 代表値（中央の値）を決定
  int bestDt = bestBin * 5 + 2;

  if (peakFrac) {
    int inPeak = 0;
    // 本物の Continental センサーの「1倍の山(61us)」と「2倍の山(122us)」
    // この2つの本物のエリアだけにパルスがどれだけ集中しているかを「厳格に」評価します
    // ノイズ（46usなど）に騙されて peakFrac が高得点を出すバグを根絶します
    for (int i = 0; i < n; i++) {
      int dt = dts[i];
      // 61us付近（50〜72us）または 122us付近（105〜135us）
      if ((dt >= 50 && dt <= 72) || (dt >= 105 && dt <= 135)) {
        inPeak++;
      }
    }
    *peakFrac = (validCnt > 0) ? (float)inPeak / validCnt : 0.0f;
  }
  
  return bestDt;
}

// Expand edge timings to half-bit level array
// 【走行時対策】固定の halfUs ではなく、変動する currentHalfUs を基準に何マス分か計算
int expandToHalfbits(const uint16_t* dts, const uint8_t* lvs, int n,
                     int halfUs, uint8_t* halfLv, int halfMax) {
  int out = 0;
  
  // 引数で渡された基準値を初期値として、動的に長さを伸縮させる「動的な物差し」
  float currentHalfUs = halfUs;
  // 追従の感度（0.02 = パルス幅のズレを約2%ずつ次の基準にフィードバックする）
  // 走行時の緩やかなパルスの伸び縮みにピッタリ吸い付きます
  const float trackingRate = 0.02f;

  for (int i = 0; i < n; i++) {
    int dt = dts[i];
    if (dt < 8) continue;
    if (dt > 5000) break;
    uint8_t prevLv = (uint8_t)(lvs[i] ^ 1);

    // 【走行時対策】固定の halfUs ではなく、変動する currentHalfUs を基準に何マス分か計算
    float estimatedK = (float)dt / currentHalfUs;
    int k = clampi((int)(estimatedK + 0.5f), 1, 20); // 四捨五入

    // ノイズでなければ、今回の実際のパルス幅から「物差しの長さ」を微修正（フィードバック）
    // 1パルスあたりのハーフビット数が多すぎる（ノイズや長い無通信）場合は追従をスキップ
    if (k >= 1 && k <= 4) {
      float actualHalfUs = (float)dt / k;
      currentHalfUs = currentHalfUs + (actualHalfUs - currentHalfUs) * trackingRate;
      
      // ガード：引数で指定された本来の速度（30, 61, 122us）から±30%以上は離れないように縛る
      // これにより、激しいノイズを吸い込んでも物差しが明当違いな値に壊れるのを防ぎます
      float minBound = halfUs * 0.85f;
      float maxBound = halfUs * 1.15f;
      if (currentHalfUs < minBound) currentHalfUs = minBound;
      if (currentHalfUs > maxBound) currentHalfUs = maxBound;
    }

    // ハーフビット配列への展開（外側のロジックと完全に同一）
    for (int j = 0; j < k && out < halfMax; j++)
      halfLv[out++] = prevLv;
  }
  return out;
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

// Find longest alternating-bit run (01 or 10 repeating)
int findAlternatingRun(const uint8_t* halfLv, int halfN, int* out_pos, int* out_endPos) {
  int bestRun = 0, bestPos = 0;
  int i = 0;
  while (i < halfN - 1) {
    int start = i;
    int runLen = 0;
    while (i + 1 < halfN && halfLv[i] != halfLv[i + 1]) {
      runLen++; i++;
    }
    if (runLen > 0) runLen++;
    if (runLen > bestRun) { bestRun = runLen; bestPos = start; }
    i++;
  }
  if (out_pos) *out_pos = bestPos;
  if (out_endPos) *out_endPos = bestPos + bestRun;
  return bestRun;
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

  if (maxBytes >= 9) {
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

// ====== DIAG output ======
// Returns valid ContinentalTPMSData if CRC match found during DIAG scan
ContinentalTPMSData printDiag(const uint16_t* dts, const uint8_t* lvs, int n,
               const uint8_t* halfLv, int halfN, int halfUs,
               uint32_t durMs, float pf,
               int preambleScore, bool isInv, int altRun, int altEnd,
               int dataStart) {
  ContinentalTPMSData diagResult = {};
  
  if(ENABLE_DETAILED_LOG)
  {
  Serial.printf("\n==== [DIAG] sc=%d/36 altRun=%d edges=%d dur=%lums halfUs=%d pf=%.2f halfN=%d inv=%d ====\n",
                preambleScore, altRun, n, (unsigned long)durMs, halfUs, pf, halfN, (int)isInv);
  }
  // dt histogram
  {
    static uint16_t dtBins[61];
    memset(dtBins, 0, sizeof(dtBins));
    for (int i = 0; i < n; i++) {
      int bin = dts[i] / 5;
      if (bin >= 0 && bin < 61) dtBins[bin]++;
    }
    if(ENABLE_DETAILED_LOG) {Serial.printf("  dt-hist: ");}
    for (int top = 0; top < 5; top++) {
      int bestBin = -1; uint16_t bestCntB = 0;
      for (int b = 0; b < 61; b++)
        if (dtBins[b] > bestCntB) { bestCntB = dtBins[b]; bestBin = b; }
      if (bestBin < 0 || bestCntB == 0) break;
      if(ENABLE_DETAILED_LOG) {Serial.printf("%d-%dus(%d) ", bestBin * 5, bestBin * 5 + 4, bestCntB);}
      dtBins[bestBin] = 0;
    }
    if(ENABLE_DETAILED_LOG) {Serial.println();}
  }

  // Edge timings (first 30)
  {
    int showN = min(n, 30);
    if(ENABLE_DETAILED_LOG) {Serial.printf("  edges[0..%d]: ", showN - 1);}
    for (int i = 0; i < showN; i++)
      if(ENABLE_DETAILED_LOG) {Serial.printf("%u%c ", dts[i], lvs[i] ? 'H' : 'L');}
    if(ENABLE_DETAILED_LOG) {Serial.println();}
  }

  // Raw bytes at estimated halfUs and at 122us
  {
    static uint8_t tmpHalf[2000];
    int rates[2] = { 122, 61 };
    int rateCount = 2;
    for (int r = 0; r < rateCount; r++) {
      int rate = rates[r];
      memset(tmpHalf, 0, sizeof(tmpHalf));
      int tmpN = expandToHalfbits(dts, lvs, n, rate, tmpHalf, (int)sizeof(tmpHalf));
      int maxBytes = min(20, tmpN / 8);
      if (maxBytes < 2) continue;
      if(ENABLE_DETAILED_LOG) {Serial.printf("  raw@%dus(N=%d): ", rate, tmpN);}
      for (int b = 0; b < maxBytes; b++) {
        uint8_t v = 0;
        for (int bit = 0; bit < 8; bit++)
          v = (uint8_t)((v << 1) | (tmpHalf[b * 8 + bit] & 1));
        if(ENABLE_DETAILED_LOG) {Serial.printf("%02X ", v);}
      }
      if(ENABLE_DETAILED_LOG) {Serial.println();}
    }
  }

  // Manchester decoded data + Continental alignment check
  {
    int dp = (dataStart >= 0) ? dataStart : altEnd;
    if (dp >= 0 && dp < halfN - 4) {
      int remaining = halfN - dp;
      if(ENABLE_DETAILED_LOG) {Serial.printf("  data@%d (%d half-bits = %d manch-bits):\n", dp, remaining, remaining / 2);}
      for (int inv = 0; inv <= 1; inv++) {
        static uint8_t dbBits[400];
        memset(dbBits, 0, sizeof(dbBits));
        int dbN = manchesterDecode(halfLv + dp, remaining,
                                   dbBits, (int)sizeof(dbBits), inv != 0);
        if (dbN < 3) continue;
        int dbBytes = min(15, (dbN + 7) / 8);
        if(ENABLE_DETAILED_LOG) {Serial.printf("    manchester inv=%d (%dbit): ", inv, dbN);}
        for (int b = 0; b < dbBytes; b++) {
          uint8_t v = 0;
          for (int bit = 0; bit < 8 && (b * 8 + bit) < dbN; bit++)
            v = (uint8_t)((v << 1) | (dbBits[b * 8 + bit] & 1));
          if(ENABLE_DETAILED_LOG) {Serial.printf("%02X ", v);}
        }
        if(ENABLE_DETAILED_LOG) {Serial.println();}

        // Try Continental decode at bit offsets 0-15 (up to 2 bytes of misalignment)
        for (int bo = 0; bo <= 15 && bo + 64 <= dbN; bo++) {
          ContinentalTPMSData trial = decodeContinentalTPMS(dbBits, dbN, bo);
          if (trial.crcValid) {
            Serial.printf("    --> CRC OK @ bitOff=%d inv=%d: brand=%02X ID=%08X PSI=%.1f %dC\n",
                          bo, inv, trial.brand, trial.sensorId,
                          trial.pressurePsi, (int)trial.temperatureC);
            if (trial.valid && !diagResult.valid)
              diagResult = trial;
          }
        }
      }
    }
  }

  if(ENABLE_DETAILED_LOG) {Serial.println("====");}
  return diagResult;
}

// Diagnostic decode for packet-sized bursts that failed the halfUs filter.
// Tries fixed half-bit candidates (independent of the broken estimate) and dumps
// Manchester bytes + CRC scan, to identify the true bit period and confirm the
// packet is decodable vs. front-end noise.
void diagBigBurst(const uint16_t* dts, const uint8_t* lvs, int n, uint32_t durMs) {
  static uint8_t bigHalf[8000];
  memset(bigHalf, 0, sizeof(bigHalf));
  // Cross-burst repeat table: a real sensor repeats the same ID; random CRC
  // collisions do not. Only IDs seen in >=2 separate bursts are trustworthy.
  static uint32_t seenId[32];
  static uint8_t  seenCnt[32];
  static int      seenN = 0;

  int hu = 122;
  int halfN = expandToHalfbits(dts, lvs, n, hu, bigHalf, (int)sizeof(bigHalf));
  Serial.printf("    [BigDiag] half=%dus halfN=%d\n", hu, halfN);
  if (halfN < 80) return;
  for (int start = 0; start <= 16; start++) {
    for (int inv = 0; inv <= 1; inv++) {
      static uint8_t bits[400];
      memset(bits, 0, sizeof(bits));
      int sn = manchesterDecode(bigHalf + start, halfN - start, bits, (int)sizeof(bits), inv != 0);
      if (sn < 64) continue;

      // Try Continental decode at bit offsets 0-15 (up to 2 bytes of misalignment)
      for (int bo = 0; bo <= 15 && bo + 64 <= sn; bo++) {
        ContinentalTPMSData t = decodeContinentalTPMS(bits, sn, bo);

        // Only trust hits that look like a real packet: known brand byte and a
        // plausible pressure. Kills random CRC collisions.
        if (t.crcValid  && isMyCarSensor(t.sensorId) &&
            t.pressurePsi >= 0.1f && t.pressurePsi <= 60.0f) {
          Serial.printf("      --> CRC OK half=%d inv=%d start=%d bo=%d: brand=%02X ID=%08X PSI=%.1f %dC\n",
                        hu, inv, start, bo, t.brand, t.sensorId,
                        t.pressurePsi, (int)t.temperatureC);
          return;  // only show first hit
        }
      }
    }
  }
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

  // CC1101 init: 8.192 kbps (=1/122us), FSK dev 40 kHz
  // RxBW narrowed 325->162kHz to cut noise bandwidth (~+4dB sensitivity) for the
  // marginal in-vehicle link. Wide enough for +-40kHz deviation + crystal error.
  // MX-Sensor (Autel) は 162kHzのほうがよさそう
  int st = radio.begin(RX_FREQ_MHZ, 8.192, 40.0, 162.0);
  if (verbose || st == RADIOLIB_ERR_NONE) {
    Serial.printf("radio.begin = %d (PARTNUM=0x%02X VERSION=0x%02X)\n", st, partnum, version);
  }
  if (st != RADIOLIB_ERR_NONE) return false;

  radio.setCrcFiltering(false);
  radio.setPromiscuousMode(true, true);
  radio.startReceive();
  ccEnableAsyncOnGDO2();

  noInterrupts();
  edgeN = 0;
  burstReady = false;
  lastEdgeUs = micros();
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
  edgeN = 0;
  burstReady = false;
  interrupts();
  g_radioReady = false;
  g_radioLostCount++;
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
  Serial.println("Waiting for TPMS packets (halfUs 100-150us filter)...");
}

// ====== loop() ======
void loop() {
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
    }
  }

  static uint32_t lastKickMs = 0;
  static uint32_t cntBurst = 0, cntInRange = 0;
  static uint32_t cntPreamble = 0, cntDecoded = 0;
  // Packet-sized bursts (edges >= BIG_BURST_EDGES): a real Continental packet
  // needs ~150-260 edges. Tracks whether such bursts arrive but get filtered out.
  static const int BIG_BURST_EDGES = 120;
  static uint32_t cntBig = 0, cntBigDropped = 0;
  static int      maxEdgesSeen = 0;
  // Ambient RSSI (noise floor): distinguishes a noisy vehicle RF environment
  // from a weak-signal/antenna problem. Sampled when idle (no active burst).
  static int      rssiMin = 127, rssiMax = -127, rssiCnt = 0;
  static long     rssiSum = 0;
  static uint32_t lastRssiMs = 0;
  // Peak RSSI sampled DURING a burst -> how far the burst rises above the floor.
  static int      curBurstRssiMax = -127;   // resets per burst
  static int      bigRssiMax = -127;         // peak among big bursts (STATS)
  // FREQEST at the burst peak: receiver-vs-sensor carrier offset. A large offset
  // means we are losing sensitivity and 315.0MHz should be retuned.
  static int      curBurstFreqEst = 0;

  uint32_t nowUs = micros();

 // ============================================================
  // Force-finalize burst (どれか1つでも満たしたら即座に処理へ回す)
  // ============================================================
  if (!burstReady) {
    // 【大元のバッファ上限セーフティ】（クラッシュ防止リミッター）
    // 配列の最大サイズ（MAX_EDGES=4000）を突き抜けてメモリ破壊・ハングアップを起こすのを
    // 絶対に防ぐための、システム全体の最下層の鉄壁リミッターです。そのまま残します。
    if (edgeN >= (MAX_EDGES - 10)) {
      noInterrupts(); burstReady = true; burstEndUs = micros(); interrupts();
    }
    // 【長時間タイムアウトセーフティ】
    // エッジ数が250に達しないような中途半端なノイズが、延々とバッファに
    // 居座り続けるのを防ぐため、40ms 経過したら強制リセットして次へ回します。
    if (edgeN > 30 && (nowUs - burstStartUs > 40000)) {
      noInterrupts(); burstReady = true; burstEndUs = micros(); interrupts();
    }
  }

  // LCD refresh
  {
    static uint32_t lastLcdMs = 0;
    if (millis() - lastLcdMs >= 200) { lastLcdMs = millis(); lcdRefresh(); }
  }

  // RSSI sampling: noise floor when idle, peak signal while a burst is captured
  if ((millis() - lastRssiMs) >= 2) {
    lastRssiMs = millis();
    int r = ccRssiDbm();
    // アンテナ導通の即時確認用。キーフォブ(315MHz)を押せば -40..-60dBm が出るはず。
    // 何も出ない = アンテナ未接続を疑う。
    {
      static uint32_t lastStrongMs = 0;
      if (r > -90 && (millis() - lastStrongMs) >= 200) {
        lastStrongMs = millis();
        Serial.printf("  [Strong] rssi=%d dBm\n", r);
      }
    }
    if (edgeN == 0) {
      if (r < rssiMin) rssiMin = r;
      if (r > rssiMax) rssiMax = r;
      rssiSum += r; rssiCnt++;
    } else if (!burstReady) {
      if (r > curBurstRssiMax) {
        curBurstRssiMax = r;
        uint8_t fe = ccReadStatus(RADIOLIB_CC1101_REG_FREQEST);
        curBurstFreqEst = (fe >= 128) ? (fe - 256) : fe;
      }
    }
  }

  // Stats (60s)
  {
    static uint32_t lastStatMs = 0;
    if (millis() - lastStatMs >= 60000) {
      lastStatMs = millis();
      Serial.printf("\n=== STATS bursts=%lu inRange=%lu preamble=%lu decoded=%lu sensors=%d ===\n",
                    (unsigned long)cntBurst, (unsigned long)cntInRange,
                    (unsigned long)cntPreamble, (unsigned long)cntDecoded,
                    sensorRecordCount);
      Serial.printf("    big(>=%d edges)=%lu dropped=%lu maxEdges=%d\n",
                    BIG_BURST_EDGES, (unsigned long)cntBig,
                    (unsigned long)cntBigDropped, maxEdgesSeen);
      Serial.printf("    noiseFloor RSSI min=%d avg=%d max=%d dBm (n=%d)\n",
                    (rssiCnt ? rssiMin : 0), (rssiCnt ? (int)(rssiSum / rssiCnt) : 0),
                    (rssiCnt ? rssiMax : 0), rssiCnt);
      Serial.printf("    bigBurst peak RSSI max=%d dBm  rfLost=%lu\n",
                    bigRssiMax, (unsigned long)g_radioLostCount);
      for (int i = 0; i < sensorRecordCount; i++) {
        uint32_t age = (millis() - sensorRecords[i].lastSeenMs) / 1000;
        Serial.printf("  ID=%08X count=%d PSI=%.1f %.0fC slot=%d (%lus ago)\n",
                      sensorRecords[i].sensorId, sensorRecords[i].count,
                      sensorRecords[i].lastPsi, sensorRecords[i].lastTempC,
                      sensorRecords[i].lcdSlot, (unsigned long)age);
      }
      cntBurst = 0; cntInRange = 0; cntPreamble = 0; cntDecoded = 0;
      cntBig = 0; cntBigDropped = 0; maxEdgesSeen = 0;
      rssiMin = 127; rssiMax = -127; rssiSum = 0; rssiCnt = 0;
      bigRssiMax = -127;
    }
  }

  if (!burstReady) return;

  // ---- Copy ISR buffer ----
  static uint16_t dts[MAX_EDGES];
  static uint8_t  lvs[MAX_EDGES];
  int n;
  uint32_t bStart, bEnd;

  noInterrupts(); // 一時的に新しい割り込みを止めて、データが書き換わるのを防ぐ
  n = edgeN;      // 溜まったエッジ数（250本など）をコピー
  if (n > MAX_EDGES) n = MAX_EDGES;
    // ここで dtBuf（割り込みが溜めた生データ）を、dts（loop側で解析するための配列）へ吸い出す！
  for (int i = 0; i < n; i++) { dts[i] = dtBuf[i]; lvs[i] = lvBuf[i]; }
  bStart = burstStartUs;
  bEnd   = burstEndUs;
  edgeN = 0;
  burstReady = false;
  interrupts();

  uint32_t dur = bEnd - bStart;
  cntBurst++;
  if (n > maxEdgesSeen) maxEdgesSeen = n;
  bool bigBurst = (n >= BIG_BURST_EDGES);
  if (bigBurst) cntBig++;
  int burstRssi = curBurstRssiMax;   // peak RSSI captured during this burst
  curBurstRssiMax = -127;
  int burstFreqEst = curBurstFreqEst;
  curBurstFreqEst = 0;
  if (bigBurst && burstRssi > bigRssiMax) bigRssiMax = burstRssi;

  // ---- Basic filters ----
  if (dur < 3000 || dur > 300000 || n < MIN_EDGES) {
    if (bigBurst) cntBigDropped++;
    radio.startReceive();
    return;
  }

  // ---- Estimate half-bit period ----
  float peakFrac = 0.0f;
  int halfUs = estimateHalfBitUs(dts, n, &peakFrac);

  // ─── 【マルチパス・サチュレーション救済ゲート】───
  // RSSI が -70dBm 以上に跳ね上がり、パルスが 37us や 47us に
  // ズタズタに潰されている場合、これは自車センサーの超強力パケットの悲鳴です。
  // 問答無用で halfUs = 122 に強制書き換えして合格ゲートを突破させます！

  if (burstRssi >= -75 && (halfUs < 100 || halfUs > 150)) {
    halfUs = 122; // 強制的に Continental 基準値に上書きして下のデコーダーへ叩き込む！
  }

  // ============================================================
  // KEY FILTER: halfUs must be 100-150us (Continental/Nissan TPMS)
  // Eliminates false triggers from noise at h=46, 52, 64 etc.
  // ============================================================
  if (halfUs < 100 || halfUs > 150) {
    // A packet-sized burst rejected here may be a real packet with a skewed
    // halfUs estimate -> dump raw timing (throttled) to read its true bit period.
    if (bigBurst && ENABLE_DETAILED_LOG && (ENABLE_BIGDROP_DETAILED_LOG || (burstRssi >= -89))) {
      cntBigDropped++;
      static uint32_t lastBigMs = 0;
      if (millis() - lastBigMs >= 2000) {
        lastBigMs = millis();
        Serial.printf("  [BigDrop halfUs] n=%d dur=%lums halfUs=%d pf=%.2f rssi=%d dBm\n",
                      n, (unsigned long)(dur / 1000), halfUs, peakFrac, burstRssi);
        // dt-histogram: top 6 peaks (5us bins) to reveal the real half-bit period
        static uint16_t dtBins[61];
        memset(dtBins, 0, sizeof(dtBins));
        for (int i = 0; i < n; i++) {
          int bin = dts[i] / 5;
          if (bin >= 0 && bin < 61) dtBins[bin]++;
        }
        Serial.printf("    dt-hist: ");
        for (int top = 0; top < 6; top++) {
          int bestBin = -1; uint16_t bestCntB = 0;
          for (int b = 0; b < 61; b++)
            if (dtBins[b] > bestCntB) { bestCntB = dtBins[b]; bestBin = b; }
          if (bestBin < 0 || bestCntB == 0) break;
          Serial.printf("%d-%dus(%d) ", bestBin * 5, bestBin * 5 + 4, bestCntB);
          dtBins[bestBin] = 0;
        }
        Serial.println();
        int showN = min(n, 48);
        Serial.printf("    edges[0..%d]: ", showN - 1);
        for (int i = 0; i < showN; i++)
          Serial.printf("%u%c ", dts[i], lvs[i] ? 'H' : 'L');
        Serial.println();
        // Attempt decode at fixed half-bit candidates (61/122us) to find the packet
        diagBigBurst(dts, lvs, n, dur / 1000);
      }
    }
    if(!ENABLE_DETAILED_LOG){delay(RECEIVE_WAIT_TIME_MS);}
    radio.startReceive();
    return;
  }

  if (peakFrac < 0.15f  && ENABLE_DETAILED_LOG) {
    if (bigBurst) {
      cntBigDropped++;
      static uint32_t lastBigMs = 0;
      if (millis() - lastBigMs >= 1000) {
        lastBigMs = millis();
        Serial.printf("  [BigDrop pf] n=%d dur=%lums halfUs=%d pf=%.2f\n",
                      n, (unsigned long)(dur / 1000), halfUs, peakFrac);
      }
    }
    if(!ENABLE_DETAILED_LOG){delay(RECEIVE_WAIT_TIME_MS);}
    radio.startReceive();
    return;
  }

  cntInRange++;

  // Per-burst signal strength for the candidate TPMS fragments (throttled).
  {
    static uint32_t lastIrMs = 0;
    if (millis() - lastIrMs >= 1000) {
      lastIrMs = millis();
      if(ENABLE_DETAILED_LOG) {Serial.printf("  [InRange] n=%d dur=%lums halfUs=%d pf=%.2f rssi=%d dBm\n",
                    n, (unsigned long)(dur / 1000), halfUs, peakFrac, burstRssi);}
    }
  }

  // ─── 走行ノイズによる物差しのブレを、本物の 122us に叩き直す ───
  // 100〜150us（Continentalゲート）を合格したものは、ノイズで多少ブレていようが、
  // 物理的な真値は100%「122us（データレート8.192kbps）」です。
  // ここで 122us 固定にしてから下の expandToHalfbits に引き渡すことで、
  // 先ほど中身を書き換えた「動的追従（PLL）」が最高精度で綺麗にロックオンを始めます！
  halfUs = 122;

  // ---- Expand to half-bits ----
  static uint8_t halfLv[8000];
  memset(halfLv, 0, sizeof(halfLv));
  int halfN = expandToHalfbits(dts, lvs, n, halfUs, halfLv, (int)sizeof(halfLv));

  // ---- Dual preamble detection ----
  bool isInverted = false;
  int preambleScore = 0;
  int dataStart = findNissanPreamble(halfLv, halfN, &isInverted, &preambleScore, 25);

  int altPos = 0, altEnd = 0;
  int altRun = findAlternatingRun(halfLv, halfN, &altPos, &altEnd);

  // ---- Should we process this burst? ----
  bool hasNissanPreamble = (preambleScore >= 28 && dataStart >= 0);
  bool hasGoodAltRun = (altRun >= 20 && peakFrac >= 0.40f);
  bool diagWorthy = (preambleScore >= 25) || hasGoodAltRun;

  if (!hasNissanPreamble && !diagWorthy) {
    radio.startReceive();
    return;
  }

  // ---- Determine decode start position ----
  int decodeStart = -1;
  if (hasNissanPreamble) {
    decodeStart = dataStart;
  } else if (hasGoodAltRun && altEnd < halfN) {
    decodeStart = altEnd;  // try from end of alternating run
  }

  // ---- DIAG output + full burst CRC scan ----
  ContinentalTPMSData diagFound = printDiag(dts, lvs, n, halfLv, halfN, halfUs,
            dur / 1000, peakFrac,
            preambleScore, isInverted, altRun, altEnd, decodeStart);

  // ---- Full burst scan: try Manchester decode from multiple positions ----
  // This catches packets where preamble was partially captured
  if (!diagFound.valid && halfN >= 130) {
    for (int scanStart = 0; scanStart <= halfN - 130; scanStart += 2) {
      float ir = manchesterInvalidRate(halfLv + scanStart, halfN - scanStart, 40);
      if (ir > 0.25f) continue;
      for (int invMode = 0; invMode <= 1; invMode++) {
        static uint8_t scanBits[400];
        memset(scanBits, 0, sizeof(scanBits));
        int scanN = manchesterDecode(halfLv + scanStart, halfN - scanStart,
                                     scanBits, (int)sizeof(scanBits), invMode != 0);
        if (scanN < 65) continue;
        for (int bo = 0; bo <= 15 && bo + 64 <= scanN; bo++) {
          ContinentalTPMSData trial = decodeContinentalTPMS(scanBits, scanN, bo);
          if (trial.valid && isMyCarSensor(trial.sensorId)) {
            diagFound = trial;
            goto scanDone;
          }
        }
      }
    }
    scanDone:;
  }

  // ---- Check if enough data for normal decode ----
  // decodeStart はプリアンブル終端の推定値で、数ハーフビット遅すぎることがある。
  // 前方にも振るので、判定もその分だけ緩める。
  static const int HALF_OFF_MIN = -8;
  int remaining = (decodeStart >= 0) ? halfN - decodeStart : 0;
  if (remaining < (130 + HALF_OFF_MIN) && !diagFound.valid ) {
    if(ENABLE_DETAILED_LOG) {
      Serial.printf("  [NoData] halfN=%d decodeStart=%d remaining=%d (need ~130 half-bits for 64-bit Continental)\n",
                    halfN, decodeStart, remaining);
    } else {
      delay(RECEIVE_WAIT_TIME_MS);
    }
    
    cntPreamble++;
    radio.startReceive();
    return;
  }

  cntPreamble++;

  // ---- Continental TPMS decode ----
  // Manchester decode at even half-bit offsets (0 and 2),
  // both inversions, then try bit alignments 0-7 in decoded stream.
  ContinentalTPMSData bestData = {};
  float bestInvRate = 1.0f;

  for (int invMode = 0; invMode <= 1; invMode++) {
    for (int halfOff = HALF_OFF_MIN; halfOff <= 2; halfOff += 2) {  // even offsets
      int start = decodeStart + halfOff;
      if (start < 0 || start + 130 > halfN) continue;

      float invRate = manchesterInvalidRate(halfLv + start, halfN - start, 40);
      if (invRate > 0.30f) continue;  // too many invalid pairs

      static uint8_t trialBits[400];
      memset(trialBits, 0, sizeof(trialBits));
      int trialN = manchesterDecode(halfLv + start, halfN - start,
                                    trialBits, (int)sizeof(trialBits), invMode != 0);
      if (trialN < 65) continue;  // need at least 1 skip + 64 data bits

      // 実走時の激しいズレ（bitOff=12など）を跨ぎきるため、探索幅を 15（1.5バイト分）へ拡張！
      // これにより、大元のプリアンブル検索を]すり抜けた走行フレームをここで1本残らず完璧に仕留めます。
      for (int bitOff = 0; bitOff <= 15 && bitOff + 64 <= trialN; bitOff++) {
        ContinentalTPMSData trial = decodeContinentalTPMS(trialBits, trialN, bitOff);

        if (trial.valid && isMyCarSensor(trial.sensorId)) {
          bool isBetter = false;
          if (!bestData.valid)               isBetter = true;
          else if (invRate < bestInvRate)     isBetter = true;
          if (isBetter) {
            bestData = trial;
            bestInvRate = invRate;
          }
        } else if (trial.crcValid &&
                   trial.temperatureC >= -40.0f && trial.temperatureC <= 100.0f &&
                   trial.sensorId != 0 && trial.sensorId != 0xFFFFFFFF) {
          // CRC は通ったが brand が 0xA8/0x98 以外。トリガーツール応答など
          // 別ファンクションのフレームを取りこぼしていないか見るため出力する。
          static uint32_t lastRejMs = 0;
          if (millis() - lastRejMs >= 2000) {
            lastRejMs = millis();
            Serial.printf("  [CRCok-Rejected] brand=0x%02X ID=%08X PSI=%.1f %.0fC\n",
                          trial.brand, trial.sensorId,
                          trial.pressurePsi, trial.temperatureC);
          }
        }
      }
    }
  }

  if (!bestData.valid) {
    // Use DIAG/scan result as fallback
    if (diagFound.valid) {
      bestData = diagFound;
      bestInvRate = 0.0f;
    } else {
      static uint32_t lastFailMs = 0;
      if (millis() - lastFailMs >= 3000) {
        lastFailMs = millis();
        if (ENABLE_DETAILED_LOG) {
          Serial.printf("  [DecodeFail] No valid CRC-8 match found (halfUs=%d)\n", halfUs);
        }
      }
      if(!ENABLE_DETAILED_LOG){delay(RECEIVE_WAIT_TIME_MS);}
      radio.startReceive();
      return;
    }
  }

  cntDecoded++;

  // ---- Sensor tracking ----
  int recIdx = trackSensor(bestData);

  // ---- Serial output ----
  Serial.printf("[Continental TPMS] ID=%08X brand=%02X PSI=%.1f kPa=%.0f bar=%.2f Temp=%dC rssi=%d dBm CRC=%02X(%s) sc=%d/36 count=%d",
                bestData.sensorId, bestData.brand,
                bestData.pressurePsi, bestData.pressureKpa, bestData.pressureBar,
                (int)bestData.temperatureC,
                burstRssi,
                bestData.crcReceived, bestData.crcValid ? "OK" : "NG",
                preambleScore,
                sensorRecords[recIdx].count);
  if (bestData.bitOffset != 0) Serial.printf(" bitOff=%d", bestData.bitOffset);
  if (bestData.hasExtra) Serial.printf(" extra=%02X seq=%d", bestData.extraByte, bestData.sequence);
  if (bestData.pressureAlert) Serial.printf(" ALERT");
  if (bestData.batteryLow) Serial.printf(" BATLOW");
  Serial.printf(" (h=%d pf=%.2f ir=%.2f foff=%+.1fkHz)\n",
                halfUs, peakFrac, bestInvRate, burstFreqEst * 1.587f);

  // ---- LCD update (CRC verified = trusted) ----
  int lcdSlot = sensorRecords[recIdx].lcdSlot;
  if (lcdSlot >= 0 && lcdSlot < LCD_SENSOR_COUNT)
    lcdUpdateTire(lcdSlot, bestData.sensorId,
                  bestData.pressurePsi, bestData.pressureBar,
                  bestData.pressureKpa, bestData.temperatureC);
  if(!ENABLE_DETAILED_LOG){delay(RECEIVE_WAIT_TIME_MS);}
  radio.startReceive();

  if (millis() - lastKickMs > 3000) {
    lastKickMs = millis();
    radio.startReceive();
  }
}
