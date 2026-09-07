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
#include <RadioLib.h>
#include "lcd_display.h"

// ====== Build environment: dev / prod ======
// TPMS_ENV_DEV = 1 : dev  (original OEM sensors: AE5C32C8 ...)
// TPMS_ENV_DEV = 0 : prod (Autel MX-Sensors:     11111111 ...)
// Override from platformio.ini with e.g. build_flags = -DTPMS_ENV_DEV=0
#ifndef TPMS_ENV_DEV
#define TPMS_ENV_DEV 0
#endif

#if TPMS_ENV_DEV
static const char* const TPMS_ENV_NAME = "DEV (OEM sensors)";
#else
static const char* const TPMS_ENV_NAME = "PROD (Autel MX-Sensor)";
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
static SPIClass cc1101Spi(HSPI);
CC1101 radio = new Module(PIN_CS, -1, -1, -1, cc1101Spi);

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
  uint8_t raw = ccReadStatus(0x34);
  int r = (raw >= 128) ? (raw - 256) : raw;
  return (r / 2) - 74;
}

void ccSetPktFormat(uint8_t fmt) {
  uint8_t v = ccRead(0x08);  // PKTCTRL0
  v = (uint8_t)((v & ~0x30) | ((fmt & 0x03) << 4));
  ccWrite(0x08, v);
}

void ccEnableAsyncOnGDO2() {
  ccSetPktFormat(3);           // async serial
  ccWrite(0x00, 0x0D);        // IOCFG2 = async data out
  ccWrite(0x02, 0x0E);        // IOCFG0 = Carrier Sense

  // AGC: maximum sensitivity for 315MHz (bench-proven config; the in-vehicle
  // issue is SNR/noise-floor, not gain, so keep the known-good sensitivity)
  ccWrite(0x07, 0x01);  // AGCCTRL2: MAX_LNA_GAIN=000, MAGN_TARGET=001
  ccWrite(0x06, 0x40);  // AGCCTRL1: AGC_LNA_PRIORITY=1
  ccWrite(0x05, 0x0F);  // AGCCTRL0: HYST=00, WAIT=11, FILTER=11

  Serial.printf("IOCFG2=0x%02X IOCFG0=0x%02X PKTCTRL0=0x%02X\n",
                ccRead(0x00), ccRead(0x02), ccRead(0x08));
  Serial.printf("AGCCTRL2=0x%02X AGCCTRL1=0x%02X AGCCTRL0=0x%02X\n",
                ccRead(0x07), ccRead(0x06), ccRead(0x05));
}

// ====== Burst capture (ISR) ======
static const uint32_t BURST_GAP_US = 20000;   // 20ms gap = end of burst
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
  if (burstReady) return;
  uint32_t now = micros();
  uint32_t dt = now - lastEdgeUs;
  lastEdgeUs = now;

  if (dt > 0 && dt < 10) return;  // glitch

  if (dt > BURST_GAP_US && edgeN > 0) {
    if (edgeN >= MIN_EDGES) {
      burstEndUs = now - dt;
      burstReady = true;
      return;
    }
    edgeN = 0;
    burstStartUs = now;
  }

  if (edgeN == 0) burstStartUs = now;

  if (edgeN < MAX_EDGES) {
    dtBuf[edgeN] = (dt > 65535 ? 65535 : (uint16_t)dt);
    lvBuf[edgeN] = (uint8_t)digitalRead(PIN_GDO2);
    edgeN++;
  }
}

// ====== Signal processing ======

static inline int clampi(int v, int lo, int hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}

// Estimate half-bit period from dt histogram
int estimateHalfBitUs(const uint16_t* dts, int n, float* peakFrac = nullptr) {
  static uint16_t hist[301];
  memset(hist, 0, sizeof(hist));
  int validCnt = 0;
  for (int i = 0; i < n; i++) {
    int dt = dts[i];
    if (dt >= 8 && dt <= 300) { hist[dt]++; validCnt++; }
  }

  int bestDt = 122;
  uint16_t bestCnt = 0;
  for (int dt = 15; dt <= 200; dt++) {
    if (hist[dt] > bestCnt) { bestCnt = hist[dt]; bestDt = dt; }
  }

  if (peakFrac) {
    int inPeak = 0;
    int lo1 = bestDt - 8, hi1 = bestDt + 8;
    int lo2 = bestDt * 2 - 12, hi2 = bestDt * 2 + 12;
    for (int dt = 8; dt <= 300; dt++) {
      if ((dt >= lo1 && dt <= hi1) || (dt >= lo2 && dt <= hi2))
        inPeak += hist[dt];
    }
    *peakFrac = (validCnt > 0) ? (float)inPeak / validCnt : 0.0f;
  }
  return bestDt;
}

// Expand edge timings to half-bit level array
int expandToHalfbits(const uint16_t* dts, const uint8_t* lvs, int n,
                     int halfUs, uint8_t* halfLv, int halfMax) {
  int out = 0;
  for (int i = 0; i < n; i++) {
    int dt = dts[i];
    if (dt < 8) continue;
    if (dt > 5000) break;
    uint8_t prevLv = (uint8_t)(lvs[i] ^ 1);
    int k = clampi((dt + halfUs / 2) / halfUs, 1, 20);
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

  // Brand byte status: bit5 = 0 → pressure alert
  d.pressureAlert = (d.brand & 0x20) == 0;

  // Validity: CRC alone gives a 1/256 false-hit rate across the brute-force
  // offset scan, so also require a real Continental brand byte (0xA8 normal /
  // 0x98 pressure-alert) and physically plausible pressure/temperature.
  if (d.crcValid &&
      (d.brand == 0xA8 || d.brand == 0x98) &&
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
    slot = 0;
    for (int i = 1; i < 8; i++)
      if (sensorRecords[i].lastSeenMs < sensorRecords[slot].lastSeenMs) slot = i;
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

  Serial.printf("\n==== [DIAG] sc=%d/36 altRun=%d edges=%d dur=%lums halfUs=%d pf=%.2f halfN=%d inv=%d ====\n",
                preambleScore, altRun, n, (unsigned long)durMs, halfUs, pf, halfN, (int)isInv);

  // dt histogram
  {
    static uint16_t dtBins[61];
    memset(dtBins, 0, sizeof(dtBins));
    for (int i = 0; i < n; i++) {
      int bin = dts[i] / 5;
      if (bin >= 0 && bin < 61) dtBins[bin]++;
    }
    Serial.printf("  dt-hist: ");
    for (int top = 0; top < 5; top++) {
      int bestBin = -1; uint16_t bestCntB = 0;
      for (int b = 0; b < 61; b++)
        if (dtBins[b] > bestCntB) { bestCntB = dtBins[b]; bestBin = b; }
      if (bestBin < 0 || bestCntB == 0) break;
      Serial.printf("%d-%dus(%d) ", bestBin * 5, bestBin * 5 + 4, bestCntB);
      dtBins[bestBin] = 0;
    }
    Serial.println();
  }

  // Edge timings (first 30)
  {
    int showN = min(n, 30);
    Serial.printf("  edges[0..%d]: ", showN - 1);
    for (int i = 0; i < showN; i++)
      Serial.printf("%u%c ", dts[i], lvs[i] ? 'H' : 'L');
    Serial.println();
  }

  // Raw bytes at estimated halfUs and at 122us
  {
    static uint8_t tmpHalf[2000];
    int rates[2] = { halfUs, 122 };
    int rateCount = (abs(halfUs - 122) <= 3) ? 1 : 2;
    for (int r = 0; r < rateCount; r++) {
      int rate = rates[r];
      int tmpN = expandToHalfbits(dts, lvs, n, rate, tmpHalf, (int)sizeof(tmpHalf));
      int maxBytes = min(20, tmpN / 8);
      if (maxBytes < 2) continue;
      Serial.printf("  raw@%dus(N=%d): ", rate, tmpN);
      for (int b = 0; b < maxBytes; b++) {
        uint8_t v = 0;
        for (int bit = 0; bit < 8; bit++)
          v = (uint8_t)((v << 1) | (tmpHalf[b * 8 + bit] & 1));
        Serial.printf("%02X ", v);
      }
      Serial.println();
    }
  }

  // Manchester decoded data + Continental alignment check
  {
    int dp = (dataStart >= 0) ? dataStart : altEnd;
    if (dp >= 0 && dp < halfN - 4) {
      int remaining = halfN - dp;
      Serial.printf("  data@%d (%d half-bits = %d manch-bits):\n", dp, remaining, remaining / 2);
      for (int inv = 0; inv <= 1; inv++) {
        static uint8_t dbBits[400];
        int dbN = manchesterDecode(halfLv + dp, remaining,
                                   dbBits, (int)sizeof(dbBits), inv != 0);
        if (dbN < 3) continue;
        int dbBytes = min(15, (dbN + 7) / 8);
        Serial.printf("    manchester inv=%d (%dbit): ", inv, dbN);
        for (int b = 0; b < dbBytes; b++) {
          uint8_t v = 0;
          for (int bit = 0; bit < 8 && (b * 8 + bit) < dbN; bit++)
            v = (uint8_t)((v << 1) | (dbBits[b * 8 + bit] & 1));
          Serial.printf("%02X ", v);
        }
        Serial.println();

        // Try Continental decode at bit offsets 0-7
        for (int bo = 0; bo <= 7 && bo + 64 <= dbN; bo++) {
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

  Serial.println("====");
  return diagResult;
}

// Diagnostic decode for packet-sized bursts that failed the halfUs filter.
// Tries fixed half-bit candidates (independent of the broken estimate) and dumps
// Manchester bytes + CRC scan, to identify the true bit period and confirm the
// packet is decodable vs. front-end noise.
void diagBigBurst(const uint16_t* dts, const uint8_t* lvs, int n, uint32_t durMs) {
  static const int cand[] = { 30, 61, 122 };
  static uint8_t bigHalf[8000];
  // Cross-burst repeat table: a real sensor repeats the same ID; random CRC
  // collisions do not. Only IDs seen in >=2 separate bursts are trustworthy.
  static uint32_t seenId[32];
  static uint8_t  seenCnt[32];
  static int      seenN = 0;

  for (int ci = 0; ci < (int)(sizeof(cand) / sizeof(cand[0])); ci++) {
    int hu = cand[ci];
    int halfN = expandToHalfbits(dts, lvs, n, hu, bigHalf, (int)sizeof(bigHalf));
    Serial.printf("    [BigDiag] half=%dus halfN=%d\n", hu, halfN);
    if (halfN < 80) continue;
    for (int inv = 0; inv <= 1; inv++) {
      static uint8_t bits[400];
      int nb = manchesterDecode(bigHalf, halfN, bits, (int)sizeof(bits), inv != 0);
      if (nb < 65) continue;
      int nbytes = min(12, nb / 8);
      Serial.printf("      manch inv=%d (%dbit): ", inv, nb);
      for (int b = 0; b < nbytes; b++) {
        uint8_t v = 0;
        for (int bit = 0; bit < 8; bit++)
          v = (uint8_t)((v << 1) | (bits[b * 8 + bit] & 1));
        Serial.printf("%02X ", v);
      }
      Serial.println();
      // Scan start offset (0-3 half-bits) x bit alignment (0-7) for a valid CRC
      for (int start = 0; start <= 3; start++) {
        int sn = manchesterDecode(bigHalf + start, halfN - start, bits, (int)sizeof(bits), inv != 0);
        for (int bo = 0; bo <= 7 && bo + 64 <= sn; bo++) {
          ContinentalTPMSData t = decodeContinentalTPMS(bits, sn, bo);
          // Only trust hits that look like a real Continental packet:
          // brand==0xA8 and a plausible pressure. Kills random CRC collisions.
          if (t.crcValid && t.brand == 0xA8 &&
              t.pressurePsi >= 5.0f && t.pressurePsi <= 60.0f) {
            int slot = -1;
            for (int s = 0; s < seenN; s++)
              if (seenId[s] == t.sensorId) { slot = s; break; }
            if (slot < 0 && seenN < 32) { slot = seenN++; seenId[slot] = t.sensorId; seenCnt[slot] = 0; }
            if (slot >= 0 && seenCnt[slot] < 255) seenCnt[slot]++;
            Serial.printf("      --> CRC OK half=%d inv=%d start=%d bo=%d: brand=A8 ID=%08X PSI=%.1f %dC seen=%d%s\n",
                          hu, inv, start, bo, t.sensorId,
                          t.pressurePsi, (int)t.temperatureC,
                          (slot >= 0) ? seenCnt[slot] : 0,
                          (slot >= 0 && seenCnt[slot] >= 2) ? "  ** REPEAT (real!) **" : "");
          }
        }
      }
    }
  }
}

// ====== setup() ======
void setup() {
  Serial.begin(115200);
  delay(1000);

  cc1101Spi.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  pinMode(PIN_GDO0, INPUT);
  pinMode(PIN_GDO2, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_GDO2), isrGdo2, CHANGE);

  Serial.println("=== Continental/Nissan TPMS Receiver @ 315.0 MHz (v3) ===");
  Serial.printf("Build env: %s\n", TPMS_ENV_NAME);
  Serial.println("Sensor: S180052353E / 40700-4GA0B / ID:AE5C32C8");
  Serial.println("Format: Brand(8)+ID(32)+Press(8)+Temp(8)+CRC8(8) = 64bit");

  // CC1101 init: 315.0 MHz, 8.192 kbps (=1/122us), FSK dev 40 kHz
  // RxBW narrowed 325->135kHz to cut noise bandwidth (~+4dB sensitivity) for the
  // marginal in-vehicle link. Wide enough for +-40kHz deviation + crystal error.
  int st = radio.begin(315.0, 8.192, 40.0, 135.0);
  Serial.printf("radio.begin = %d\n", st);
  if (st != RADIOLIB_ERR_NONE) {
    // 車載時のみ発生（電源投入直後の電圧変動でCC1101がまだ応答しない）。
    // 電源が安定するのを待って1回だけリトライする。
    Serial.printf("!! CC1101 init FAILED (code=%d) -> retry in 500ms\n", st);
    delay(500);
    st = radio.begin(315.0, 8.192, 40.0, 135.0);
    Serial.printf("radio.begin (retry) = %d\n", st);
  }
  if (st != RADIOLIB_ERR_NONE) {
    // 無線が死んだまま走り続けても意味がないので、LCDに10秒表示してから再起動。
    Serial.printf("!! CC1101 init FAILED (code=%d) -> reboot in 10s\n", st);
    lcdBegin();
    char detail[32];
    snprintf(detail, sizeof(detail), "CC1101 code=%d", st);
    lcdShowFatal("RF FAIL", detail);
    for (int s = 10; s > 0; s--) { lcdShowFatalCountdown(s); delay(1000); }
    Serial.println("Rebooting...");
    Serial.flush();
    ESP.restart();
  }

  radio.setCrcFiltering(false);
  radio.setPromiscuousMode(true, true);
  radio.startReceive();
  ccEnableAsyncOnGDO2();

  lastEdgeUs = micros();
  lcdBegin();

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

  uint32_t nowUs = micros();

  // Force-finalize burst
  if (!burstReady && edgeN >= (MAX_EDGES - 10)) {
    noInterrupts(); burstReady = true; burstEndUs = micros(); interrupts();
  }
  if (!burstReady && edgeN > 30 && (nowUs - burstStartUs > 200000)) {
    noInterrupts(); burstReady = true; burstEndUs = micros(); interrupts();
  }
  if (!burstReady && edgeN >= MIN_EDGES && (nowUs - lastEdgeUs > BURST_GAP_US)) {
    noInterrupts(); burstReady = true; burstEndUs = lastEdgeUs; interrupts();
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
    if (edgeN == 0) {
      if (r < rssiMin) rssiMin = r;
      if (r > rssiMax) rssiMax = r;
      rssiSum += r; rssiCnt++;
    } else if (!burstReady) {
      if (r > curBurstRssiMax) curBurstRssiMax = r;
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
      Serial.printf("    bigBurst peak RSSI max=%d dBm\n", bigRssiMax);
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

  noInterrupts();
  n = edgeN;
  if (n > MAX_EDGES) n = MAX_EDGES;
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

  // ============================================================
  // KEY FILTER: halfUs must be 100-150us (Continental/Nissan TPMS)
  // Eliminates false triggers from noise at h=46, 52, 64 etc.
  // ============================================================
  if (halfUs < 100 || halfUs > 150) {
    // A packet-sized burst rejected here may be a real packet with a skewed
    // halfUs estimate -> dump raw timing (throttled) to read its true bit period.
    if (bigBurst) {
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
    radio.startReceive();
    return;
  }

  if (peakFrac < 0.15f) {
    if (bigBurst) {
      cntBigDropped++;
      static uint32_t lastBigMs = 0;
      if (millis() - lastBigMs >= 1000) {
        lastBigMs = millis();
        Serial.printf("  [BigDrop pf] n=%d dur=%lums halfUs=%d pf=%.2f\n",
                      n, (unsigned long)(dur / 1000), halfUs, peakFrac);
      }
    }
    radio.startReceive();
    return;
  }

  cntInRange++;

  // Per-burst signal strength for the candidate TPMS fragments (throttled).
  {
    static uint32_t lastIrMs = 0;
    if (millis() - lastIrMs >= 1000) {
      lastIrMs = millis();
      Serial.printf("  [InRange] n=%d dur=%lums halfUs=%d pf=%.2f rssi=%d dBm\n",
                    n, (unsigned long)(dur / 1000), halfUs, peakFrac, burstRssi);
    }
  }

  // ---- Expand to half-bits ----
  static uint8_t halfLv[8000];
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
        int scanN = manchesterDecode(halfLv + scanStart, halfN - scanStart,
                                     scanBits, (int)sizeof(scanBits), invMode != 0);
        if (scanN < 65) continue;
        for (int bo = 0; bo <= 7 && bo + 64 <= scanN; bo++) {
          ContinentalTPMSData trial = decodeContinentalTPMS(scanBits, scanN, bo);
          if (trial.valid) {
            diagFound = trial;
            goto scanDone;
          }
        }
      }
    }
    scanDone:;
  }

  // ---- Check if enough data for normal decode ----
  int remaining = (decodeStart >= 0) ? halfN - decodeStart : 0;
  if (remaining < 130 && !diagFound.valid) {
    Serial.printf("  [NoData] dataStart=%d remaining=%d (need ~130 half-bits for 64-bit Continental)\n",
                  decodeStart, remaining);
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
    for (int halfOff = 0; halfOff <= 2; halfOff += 2) {  // 0 and 2 (even offsets)
      int start = decodeStart + halfOff;
      if (start < 0 || start + 130 > halfN) continue;

      float invRate = manchesterInvalidRate(halfLv + start, halfN - start, 40);
      if (invRate > 0.30f) continue;  // too many invalid pairs

      static uint8_t trialBits[400];
      int trialN = manchesterDecode(halfLv + start, halfN - start,
                                    trialBits, (int)sizeof(trialBits), invMode != 0);
      if (trialN < 65) continue;  // need at least 1 skip + 64 data bits

      // Try bit alignments 0-7
      for (int bitOff = 0; bitOff <= 7 && bitOff + 64 <= trialN; bitOff++) {
        ContinentalTPMSData trial = decodeContinentalTPMS(trialBits, trialN, bitOff);

        if (trial.valid) {
          bool isBetter = false;
          if (!bestData.valid)               isBetter = true;
          else if (invRate < bestInvRate)     isBetter = true;
          if (isBetter) {
            bestData = trial;
            bestInvRate = invRate;
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
        Serial.printf("  [DecodeFail] No valid CRC-8 match found (halfUs=%d)\n", halfUs);
      }
      radio.startReceive();
      return;
    }
  }

  cntDecoded++;

  // ---- Sensor tracking ----
  int recIdx = trackSensor(bestData);

  // ---- Serial output ----
  Serial.printf("[Continental TPMS] ID=%08X brand=%02X PSI=%.1f kPa=%.0f bar=%.2f Temp=%dC CRC=%02X(%s) sc=%d/36 count=%d",
                bestData.sensorId, bestData.brand,
                bestData.pressurePsi, bestData.pressureKpa, bestData.pressureBar,
                (int)bestData.temperatureC,
                bestData.crcReceived, bestData.crcValid ? "OK" : "NG",
                preambleScore,
                sensorRecords[recIdx].count);
  if (bestData.bitOffset != 0) Serial.printf(" bitOff=%d", bestData.bitOffset);
  if (bestData.hasExtra) Serial.printf(" extra=%02X seq=%d", bestData.extraByte, bestData.sequence);
  if (bestData.pressureAlert) Serial.printf(" ALERT");
  if (bestData.batteryLow) Serial.printf(" BATLOW");
  Serial.printf(" (h=%d pf=%.2f ir=%.2f)\n",
                halfUs, peakFrac, bestInvRate);

  // ---- LCD update (CRC verified = trusted) ----
  int lcdSlot = sensorRecords[recIdx].lcdSlot;
  if (lcdSlot >= 0 && lcdSlot < LCD_SENSOR_COUNT)
    lcdUpdateTire(lcdSlot, bestData.sensorId,
                  bestData.pressurePsi, bestData.pressureBar,
                  bestData.pressureKpa, bestData.temperatureC);

  radio.startReceive();

  if (millis() - lastKickMs > 3000) {
    lastKickMs = millis();
    radio.startReceive();
  }
}
