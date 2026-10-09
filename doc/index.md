# TPMS受信機 ハードウェア構成（Nissan / Continental）

## 回路図（Schemdraw）

![TPMS回路図](image/tpms_circuit_schemdraw.svg)

※ 回路図の生成スクリプト: `doc/image/generate_tpms_circuit.py`

## CC1101とESP32-S3-WROOM-2 N32R16V ピン対応表

### SPI接続

| CC1101 ピン番号 | CC1101 機能 | ESP32-S3 GPIO | 説明 |
|----------------|------------|---------------|------|
| 1 | GND | GND | グランド |
| 2 | VCC | 3.3V | 電源（3.3V） |
| 3 | GDO0 | GPIO16 | 汎用デジタル出力0（キャリア検出等に使用可能） |
| 4 | CSN | GPIO10 | SPI チップセレクト |
| 5 | SCK | GPIO12 | SPI クロック |
| 6 | MOSI (SI) | GPIO11 | SPI データ入力 (Master Out Slave In) |
| 7 | MISO (SO) | GPIO13 | SPI データ出力 (Master In Slave Out) |
| 8 | GDO2 | GPIO15 | 非同期データ出力（割り込み用）、GND線を平行に配置しAsync Data線のループ面積縮小＋擬似シールド |

### 備考

- **ESP32-S3-WROOM-2 N32R16V** はOctal Flash + Octal PSRAMを搭載
- **CC1101** は315MHz/433MHz/868MHz/915MHz帯のサブGHz無線トランシーバ
- **GDO0 (GPIO16)** : 初期化時にキャリアセンス出力（IOCFG0=0x0E）を設定するが、現行ソフトでは未使用（配線診断で読むのみ）。
  定期的な `radio.startReceive()` で RadioLib が IOCFG0 を書き換えるため、常時キャリアセンスとは限らない
- **GDO2 (GPIO15)** : CC1101から非同期データを受信し、エッジ検出割り込みで処理
- **電源** : CC1101は3.3V動作、ESP32-S3から直接供給可能

### ソフトウェア設定

```cpp
// SPI ピン定義 (src/main.cpp)
static const int PIN_CS   = 10;   // CC1101 pin4 CSN
static const int PIN_SCK  = 12;   // CC1101 pin5 SCK
static const int PIN_MOSI = 11;   // CC1101 pin6 MOSI
static const int PIN_MISO = 13;   // CC1101 pin7 MISO
static const int PIN_GDO0 = 16;   // CC1101 pin3 GDO0 (Carrier Sense)
static const int PIN_GDO2 = 15;   // CC1101 pin8 GDO2 (Async Data)
```
### ピン接続（ESP32-S3 ← → M154-240240-RGB）

CC1101 とは**独立した SPI3 バス**を使用（速度設定の切り替え不要、干渉なし）。

| M154 ピン | 機能 | ESP32-S3 GPIO | 備考 |
|-----------|------|---------------|------|
| VCC | 電源 | 3.3V | |
| GND | グランド | GND | |
| SCL | SPI クロック | GPIO17 | SPI3 専用（全LCD共有）|
| SDA | SPI MOSI | GPIO18 | SPI3 専用（全LCD共有）|
| RES | ハードリセット | GPIO4 | 第1LCD（左側、既存）|
| DC | データ/コマンド | GPIO5 | 第1LCD（左側、既存）|
| CS | チップセレクト | GPIO6 | 第1LCD（左側、既存）|
| BLK | バックライト | GPIO7 | 全LCD共用、Pch MOSFET(ZVP2106A) 高側スイッチで **LOW=ON**（PWM 調光可、起動時 OFF）|

右側用に追加する第2台LCDの制御ピン：

| 信号 | 第2LCD（右側） 推奨 GPIO | 備考 |
|------|------------------------|------|
| RES2 | GPIO19 | 第2LCD（右側）のハードリセット |
| DC2  | GPIO21 | 第2LCD（右側）の D/C |
| CS2  | GPIO20 | 第2LCD（右側）の CS |


### ソフトウェア設定

```cpp
#define LCD_SCK   17   // SPI3 専用
#define LCD_MOSI  18   // SPI3 専用
#define LCD_RST    4    // 第1LCD RST
#define LCD_DC     5    // 第1LCD DC
#define LCD_CS     6    // 第1LCD CS
#define LCD_BLK    7    // バックライト（全LCD共用）

// 追加LCD用（推奨）
#define LCD2_RST   19   // 第2LCD RST
#define LCD2_DC    21   // 第2LCD DC
#define LCD2_CS   20    // 第2LCD CS
```

### 動作モード

このプロジェクトでは、CC1101を**非同期モード (Async Serial Mode)** で使用し、
GDO2ピンから出力される生データをESP32-S3で直接キャプチャして、
Manchester符号化されたContinental TPMSセンサーデータをデコードします。

#### 受信パラメータ

| 項目 | 設定値 | 備考 |
|------|--------|------|
| 周波数 | 315.022 MHz | 公称315.0MHz。実測 FREQEST が +22kHz 前後だったため同調点を補正 |
| データレート | 32.768 kbps | 受信機側の設定値（センサーの速度ではない）。非同期モードのため名目値で、ビット判定はESP32側で半ビット長（PARK 122µs / DRIVE 約52µs）により行う |
| FSK偏差 | 40 kHz | |
| 受信帯域幅 | 232 kHz | |
| IOCFG2 | 0x0D | GDO2 = 非同期データ出力 |
| PKTCTRL0 | 0x31 | 非同期シリアルモード |
| AGCCTRL2 / 1 / 0 | 0x43 / 0x50 / 0x92 | LNA最大ゲイン制限・キャリアセンス閾値・AGCフィルタ |
| FOCCFG | 0x3E | AFC 高速追従 |
| BSCFG | 0x10 | ビット同期ループ（非同期モードでは実質無関係） |

#### GDO2エッジ取得とパケット検出

1. **エッジ記録（ISR）**
   GDO2 の CHANGE 割り込みで、全エッジの時刻（`micros()`）とエッジ直後のレベルを
   リングバッファ（8192エッジ）へ止めずに記録する。
2. **窓切り出し（loop）**
   1024エッジずつ、前の窓と400エッジ重ねて切り出す（エッジが少ない時も30ms毎に解析）。
   重なりがあるので、窓の境界をまたいだパケットも必ずどこかの窓に丸ごと入る。
3. **パルス化・ヒゲ除去**
   エッジ列をパルス（幅＋レベル）列に変換し、0.35h 未満の短いパルスを前後に吸収する。
4. **ラン検出**
   「単独パルス幅 0.4h〜4.6h かつ 隣接2パルスの和が 1.6h〜6.6h」を満たすパルスが
   60個以上連続する区間をパケット候補（ラン）とする。
   - DRIVE: h=56µs（約45〜70µsをカバー）
   - PARK : h=122µs
   ノイズ（平均28µs程度）はこの条件を連続で満たせないため、ラン自体が立たない。
5. **ラン単位のデコード**
   - H/L 総時間の差からスライサの片寄り（bias）を推定して補正
   - 補正後の幅から実際の半ビット長（href）を実測
   - Manchester の位相2通り × 極性2通り × 全ビット位置で CRC を総当たり
   - 自車センサーID かつ CRC/範囲チェック合格のものだけ採用
6. **重複除去**
   窓の重なりで同じパケットを二度デコードした場合（同一ID・開始時刻差4ms未満）は捨てる。

その他の周期処理:

- RSSI / FREQEST を2ms毎にサンプリングし、デコード成功時に該当時間帯のピーク値をログ出力
- 500ms毎に VERSION レジスタでチップ生存確認、MARCSTATE が RX 以外なら RX に戻す
- 2秒毎に `radio.startReceive()` で RX を再スタート

### フレーム構造（Continental / Nissan TPMS）

センサーは状態によって **PARK（停止中）** と **DRIVE（走行中）** の2種類のフレームを送信する。
両者は半ビット長・フレーム長・CRC位置が異なるため、別フォーマットとして扱う。

#### PARK / DRIVE 比較

| 項目 | PARK（停止中） | DRIVE（走行中） |
|------|----------------|-----------------|
| 半ビット長 | 約122µs | 約52µs（実測 51.2〜52.4µs） |
| チップレート | 8192 chip/s | 約19.2 kchip/s |
| ビットレート | 4096 bps | 約9.6 kbps（9.5〜9.8 kbps） |
| プリアンブル | `F5 55 55 55 E`（36 half-bits） | bit0 × 15以上 ＋ 同期bit1 |
| フレーム長 | 8バイト（64ビット）＋任意の追加1バイト | 9バイト（72ビット） |
| Byte 7 | CRC-8 | Flags（繰り返し番号 01〜04） |
| Byte 8 | 追加バイト（BAT?/シーケンス） | CRC-8 |
| CRC対象 | Byte 0〜6 | Byte 0〜7 |
| Brand（実測） | 0xA8 / 0x98 / 0x80 | 0xB9 |
| 1回の送信 | 未整理 | 同一内容を4回（Flags=01→04） |

**速度表記の定義**（本書・コードはこの用語で統一）

- **半ビット長**: Manchester の半ビット（チップ）1個の幅。実測値でありすべての基準
- **チップレート [chip/s]** = 1 ÷ 半ビット長
- **ビットレート [bps]** = チップレート ÷ 2（Manchester は2チップで1データビット）
- CC1101 の「データレート 32.768 kbps」は受信機側の設定値で、センサーの速度ではない（非同期モードでは名目値）
- 外部情報の「16384bps」は実測と一致しないため使用しない

#### 共通事項

- 周波数: 315 MHz、FSK
- 符号化: Manchester（G.E. Thomas、half-bit単位）
  - bit 0 → `01`（Low→High）
  - bit 1 → `10`（High→Low）
- Sensor ID（Byte 1〜4）・Press（Byte 5）・Temp（Byte 6）の位置は両モード共通
- CRC-8: poly=0x07, init=0xAA（対象範囲のみモードで異なる）
- デコード式

  ```
  PSI = pressureRaw / 4.0
  kPa = PSI × 6.895
  bar = kPa / 100
  ℃  = temperatureRaw − 52
  ```

- センサーID一覧

  ```
  FL : AE5C32C8   FR : AC4ACC28
  RL : AE58E836   RR : AC4CCF67
  ```

#### PARK フレーム（停止中）

```
物理信号 (GDO2 raw edge stream)
──────────────────────────────────────────────────────────────────

[ プリアンブル                    ][ データ部 (8バイト = 64ビット)    ]
  F5 55 55 55 E (36 half-bits)     Brand + ID + Press + Temp + CRC

  Half-bit period: ~122µs (= 1/8192 chip/s)

──────────────────────────────────────────────────────────────────
パケット構造 (8バイト = 64ビット)

  +--------+--------+--------+--------+--------+--------+--------+--------+
  | Byte 0 | Byte 1 | Byte 2 | Byte 3 | Byte 4 | Byte 5 | Byte 6 | Byte 7 |
  +--------+--------+--------+--------+--------+--------+--------+--------+
  | Brand  | -------- Sensor ID (32bit) --------| Press  |  Temp  | CRC-8  |
  | (8bit) | MSB                            LSB | (8bit) | (8bit) | (8bit) |
  +--------+--------+--------+--------+--------+--------+--------+--------+

  Brand    : メーカーコード（0xA8 = Continental）
             bit5 でステータス判定（下記参照）
  SensorID : 32ビットセンサー固有ID
  Press    : 圧力 raw値 → PSI = raw / 4
  Temp     : 温度 raw値 → ℃ = raw − 52
  CRC-8    : poly=0x07, init=0xAA（Byte 0〜6 に対して計算）

──────────────────────────────────────────────────────────────────
Byte 8（CRC後の追加バイト、存在する場合）

  +----+----+----+----+----+----+----+----+
  | b7 | b6 | b5 | b4 | b3 | b2 | b1 | b0 |
  +----+----+----+----+----+----+----+----+
  |BAT?| ?  |     6-bit Sequence Counter   |
  +----+----+----+----+----+----+----+----+

  bit 7    : バッテリーフラグ？（0=正常, 1=低電圧 ※推定）
  bit 6    : 不明（既知センサーでは常に 1）
  bit 5-0  : シーケンスカウンタ（0〜63、送信ごとに変化）

  ※ 実測された既知センサーの extra 値は 0x40〜0x5F に集中
    (bit7=0, bit6=1, seq=0〜31)

──────────────────────────────────────────────────────────────────
Brand バイトのステータス情報

  Brand = 0xA8 (1010_1000) : Normal mode（安定圧力時）
  Brand = 0x98 (1001_1000) : Pressure Alert mode（圧力変動時）
  Brand = 0x80 (1000_0000) : 125kHz LF トリガー時

  判定: bit 5 = 1 → Normal / bit 5 = 0 → Pressure Alert

  実測データ:
    brand=0xA8 (93回) : PSI 32.0〜50.2 の安定範囲のみ
    brand=0x98 (52回) : PSI 11.0〜63.0 の広範囲（空気注入中等）

  参考: Microchip AN00238C (SP37/Schrader系) では
    bit 1-0 が Operating State を示す:
      00=Initial/Storage, 01=Normal, 10=Pressure Alert, 11=Temp Alert
    Continental ではビット配置が異なるが機能は類似
```

#### DRIVE フレーム（走行中）

```
物理信号 (GDO2 raw edge stream)
──────────────────────────────────────────────────────────────────

[ プリアンブル          ][同期][ データ部 (9バイト = 72ビット)              ]
  bit 0 × 15以上          bit 1  Brand + ID + Press + Temp + Flags + CRC
  (half-bit: 0101…01)     (10)

  Half-bit period: ~52µs（実測 51.2〜52.4µs、≒ 1/19200 chip/s）
  1フレーム長     : 約9〜10ms（約150パルス）

  ※ プリアンブルの bit0 個数は観測値（先頭が欠けている可能性あり）
  ※ 同期bit1 → Brand の境目で half-bit "0110" となり、
     プリアンブル直後に 2half-bit 幅(約104µs) のパルスが現れる

──────────────────────────────────────────────────────────────────
パケット構造 (9バイト = 72ビット)

  +--------+--------+--------+--------+--------+--------+--------+--------+--------+
  | Byte 0 | Byte 1 | Byte 2 | Byte 3 | Byte 4 | Byte 5 | Byte 6 | Byte 7 | Byte 8 |
  +--------+--------+--------+--------+--------+--------+--------+--------+--------+
  | Brand  | -------- Sensor ID (32bit) --------| Press  |  Temp  | Flags  | CRC-8  |
  | (8bit) | MSB                            LSB | (8bit) | (8bit) | (8bit) | (8bit) |
  +--------+--------+--------+--------+--------+--------+--------+--------+--------+

  Brand    : 0xB9 (1011_1001) を観測
             PARK の 0xA8 (1010_1000) と比べ bit4 / bit0 が 1。
             走行状態を示すビットと推定（未確定）
  SensorID : PARK と同じ
  Press    : PARK と同じ（PSI = raw / 4）※ 加圧状態での確認は未実施
  Temp     : PARK と同じ（℃ = raw − 52）
  Flags    : 1回の送信内の繰り返し番号。01 → 02 → 03 → 04 と変化
  CRC-8    : poly=0x07, init=0xAA（Byte 0〜7 に対して計算）
             ※ PARK と同じアルゴリズムだが対象が Flags まで含む

──────────────────────────────────────────────────────────────────
送信パターン

  1回の送信で同一データを4フレーム連続送信（Flags だけが 01〜04 で変化）
  フレーム間隔・送信周期は未計測

──────────────────────────────────────────────────────────────────
実測データ（2026-10-07, serial_20261007200329.log）

  条件: RL センサーを 700c ロードバイクリムに貼付けて回転
        大気圧（ゲージ圧 0）、室温

  B9 | AE 58 E8 36 | 01 | 4B | 01 | F7
  B9 | AE 58 E8 36 | 01 | 4B | 02 | FE
  B9 | AE 58 E8 36 | 01 | 4B | 03 | F9
  B9 | AE 58 E8 36 | 01 | 4B | 04 | EC
  Brand  ID=RL      P=0.25psi T=23℃ Flags  CRC

  RSSI -47〜-63 dBm、Manchester不正ペア率 0.00、2回の送信で8/8フレーム受信

──────────────────────────────────────────────────────────────────
過去の誤認（記録）

  ・RTR-SDR blog v4 ＋ SDR++ で走行モードの電波を録音し、Universal Radio Hackerで61シンボル/Manchester IIで、ID 32bitのうち31bitが読めたため、16384bpsと判断した
  ・16384bpsから half-bit を 31µs（bpsとして換算）/ 61µs（chip/sとして換算）
    と想定していたが、実測は約52µs（約19.2k chip/s、約9.6 kbps）。31µs 想定ではノイズが大量に通過し、
    CRC総当たりで他車IDの偽ヒットを量産していた。
  ・PARK と同じ 8バイト形式（Byte 7 = CRC）で検証していたため、
    ID は一致するのに CRC NG となっていた。
    実際は Byte 7 = Flags、Byte 8 = CRC（Byte 0〜7 対象）。
```

#### 受信側の判別方法

- PARK / DRIVE はラン検出の半ビット長で振り分ける（詳細は「GDO2エッジ取得とパケット検出」参照）
- CRC は PARK形式（Byte 0〜6 → Byte 7）を先に試し、不一致なら DRIVE形式（Byte 0〜7 → Byte 8）を試す
- DRIVE形式で一致したフレームはログに `DRIVE9 flags=xx` と表示される

#### 録音時の記録
![SDR](image/Record_SDR.jpg)
![URH1](image/Record_URH1.png)
![URH2](image/Record_URH2.png)

#### 参考

```
  BMW Gen5 Continental TPMS (rtl_433 Issue #2821) は同系統で
  88ビット（11バイト）のより長いフォーマットだが、
  本センサーは PARK 64ビット / DRIVE 72ビットの短縮版を使用。
  CRC多項式も異なる（BMW: poly=0x2F / Nissan: poly=0x07）。
```


---
## CC1101とESP32-DevKitC-32E ピン対応表（本番機・案）

### SPI接続

CC1101 は **HSPI（SPI2）** を GPIO マトリクス経由で使用（SPI クロック 500kHz なので速度制約なし）。
DevKitC-32E の**左列（J1）にまとめて配置**する。

| CC1101 ピン番号 | CC1101 機能 | ESP32 GPIO | J1 位置 | 説明 |
|----------------|------------|------------|---------|------|
| 1 | GND | GND | J1-14 | グランド |
| 2 | VCC | — | — | 外部LDO（NJU7223F33）の 3.3V から供給 |
| 3 | GDO0 | GPIO32 | J1-7 | キャリアセンス（現行ソフトでは配線診断のみ） |
| 4 | CSN | GPIO27 | J1-11 | SPI チップセレクト。**10kΩ で 3.3V へプルアップ**推奨 |
| 5 | SCK | GPIO14 | J1-12 | SPI クロック |
| 6 | MOSI (SI) | GPIO26 | J1-10 | SPI データ (Master Out Slave In) |
| 7 | MISO (SO) | GPIO25 | J1-9 | SPI データ (Master In Slave Out)、内部プルアップ使用 |
| 8 | GDO2 | GPIO33 | J1-8 | 非同期データ出力（CHANGE 割り込み） |

### 備考

- **使用禁止ピン**
  - GPIO6〜11 : SPI Flash 接続（DevKitC の SD0〜SD3 / CMD / CLK）
  - GPIO1 / 3 : UART0（USB シリアルログ用）
- **使用を避けたピン（ストラッピング）**
  - GPIO12 (MTDI) : 起動時 High だと Flash 電圧が 1.8V になり起動不能。
    S3 版の割り当てを流用すると HSPI デフォルトの MISO=GPIO12 になり、
    現行ソフトの MISO 内部プルアップで**起動しなくなる**ため不可
  - GPIO0 / 2 / 15 : ブートモード・ログ出力選択
  - GPIO5 は LCD の CS1 にのみ使用（出力専用、内部プルアップのまま起動に影響なし）
- **割り当ての考え方**
  - CC1101 の出力ピン（GDO0 / GDO2 / SO）は、起動時に ESP32 側が信号を出す可能性のある GPIO14 を避ける。
    GPIO14 は ESP32 → CC1101 方向の SCK に充てる（起動中は CSN=High なので CC1101 は無視する）
  - CSN は ESP32 リセット中〜初期化前にフローティングになるため、外付けプルアップで非選択に固定する
  - GDO2 は割り込みを使うので GPIO36 / 39 を避ける（Wi-Fi/ADC 動作時に偽エッジが出るエラッタあり）
  - アナログ入力は ADC1（GPIO32〜39）のみ使う（ADC2 は Wi-Fi 使用中に読めない）
  - GPIO16 / 17 は WROOM-32E では使用可（WROVER-E では PSRAM に使われるため不可）
- **電源** : CC1101・LCD とも外部LDO（NJU7223F33）の 3.3V から供給。ESP32 の 3V3 ピンとは接続しない（GND は共通）

#### ピン割り当て全体（案）

LCD・照度センサーを含めた全体の割り当て。CC1101 は左列（J1）、LCD は右列（J3）に分けて配線の交差を避ける。
LCD は **VSPI（SPI3）の IO_MUX 直結ピン**（SCK=18 / MOSI=23）を使い、最大 80MHz まで出せるようにする。

左列 J1（アンテナ側から）

| J1 | ピン | 割り当て | 備考 |
|----|------|----------|------|
| 1 | 3V3 | — | 未使用（外部LDO系とは接続しない） |
| 2 | EN | — | |
| 3 | GPIO36 (SVP) | 空き | 入力専用・ADC1 |
| 4 | GPIO39 (SVN) | 空き | 入力専用・ADC1 |
| 5 | GPIO34 | 照度センサー（NJL7502L） | 入力専用・ADC1_CH6 |
| 6 | GPIO35 | 空き（予備: 車載電源電圧監視など） | 入力専用・ADC1_CH7 |
| 7 | GPIO32 | CC1101 GDO0 | |
| 8 | GPIO33 | CC1101 GDO2 | 割り込み |
| 9 | GPIO25 | CC1101 MISO | |
| 10 | GPIO26 | CC1101 MOSI | |
| 11 | GPIO27 | CC1101 CSN | 10kΩ プルアップ |
| 12 | GPIO14 | CC1101 SCK | |
| 13 | GPIO12 | 使用しない | ストラッピング |
| 14 | GND | CC1101 GND | |
| 15 | GPIO13 | 空き | |
| 16〜18 | SD2 / SD3 / CMD | 使用禁止 | Flash |
| 19 | 5V | 電源入力 | ショットキー経由で DCDC 5V |

右列 J3（アンテナ側から）

| J3 | ピン | 割り当て | 備考 |
|----|------|----------|------|
| 1 | GND | LCD GND | |
| 2 | GPIO23 | LCD MOSI (SDA) | VSPI IO_MUX、全LCD共有 |
| 3 | GPIO22 | LCD2 RST | 第2LCD（右側） |
| 4 | TXD0 | 使用しない | USB シリアル |
| 5 | RXD0 | 使用しない | USB シリアル |
| 6 | GPIO21 | LCD2 CS | 第2LCD（右側） |
| 7 | GND | — | |
| 8 | GPIO19 | LCD2 DC | 第2LCD（右側） |
| 9 | GPIO18 | LCD SCK (SCL) | VSPI IO_MUX、全LCD共有 |
| 10 | GPIO5 | LCD CS | 第1LCD（左側） |
| 11 | GPIO17 | LCD DC | 第1LCD（左側） |
| 12 | GPIO16 | LCD RST | 第1LCD（左側） |
| 13 | GPIO4 | LCD BLK | Pch MOSFET ゲート（LOW=ON）。ゲートの 10kΩ プルアップで起動時 OFF を保証 |
| 14 | GPIO0 | 使用しない | BOOT ボタン |
| 15 | GPIO2 | 使用しない | ストラッピング |
| 16 | GPIO15 | 使用しない | ストラッピング |
| 17〜19 | SD1 / SD0 / CLK | 使用禁止 | Flash |

- LCD 信号は 9本（SCK / MOSI / RST / DC / CS / BLK / RST2 / DC2 / CS2）＋ GND で、10P ボックスヘッダーにちょうど収まる
- CC1101 信号は 6本。14P ボックスヘッダーで信号線の間に GND を挟む
- 空き: GPIO13, GPIO35, GPIO36, GPIO39（36 / 39 は割り込み用途には使わない）

### ボックスヘッダー ピン配置（案）

#### 前提

- 両端コネクター付リボンケーブルは **1番ピン同士がつながる（ストレート）** 前提。
  ケーブルの赤線（▼マーク）側が1番。**組立前にテスターで 1-1 / 2-2 を確認**すること
- そのため、メイン基板側と子基板側は**同じ番号に同じ信号**を割り当てる
- リボンケーブル上で隣り合う線は「番号が連続するピン」（1-2-3-4…）。
  クロストーク対策はこの並びで考える
- 電源（3.3V）はヘッダーに通さず、各基板へ XH コネクターで別供給（[本番用部品.md](../本番用部品.md) のとおり）
- 下図は**部品面から見た図**（切り欠き上）。ユニバーサル基板の**はんだ面から配線するときは左右反転**する

#### CC1101 用 14P（2×7）: メイン基板側 / CC1101基板側 共通

奇数ピンを全部 GND にして、ケーブル上で全信号線の両隣を GND にする。
信号の並びは ESP32 の J1 の並び（GDO0→GDO2→MISO→MOSI→CSN→SCK）と同じにして、メイン基板上の配線が交差しないようにする。
クロック（SCK）は最も敏感な GDO2 から最も遠い位置にする。

部品面から見た図

```
           【 切り欠き (上) 】
  1:GND    3:GND    5:GND    7:GND    9:GND   11:GND   13:GND
  2:GDO0   4:GDO2   6:MISO   8:MOSI  10:CSN   12:SCK   14:GND
```

はんだ面から見た図（左右反転）

```
           【 切り欠き (上) 】
 13:GND   11:GND    9:GND    7:GND    5:GND    3:GND    1:GND
 14:GND   12:SCK   10:CSN    8:MOSI   6:MISO   4:GDO2   2:GDO0
```

| ピン | 信号 | メイン基板側（ESP32） | CC1101基板側（CC1101 ピン） | ケーブル上の両隣 |
|------|------|----------------------|----------------------------|------------------|
| 1 | GND | GND（J1-14） | 1 GND | — / GDO0 |
| 2 | GDO0 | GPIO32（J1-7） | 3 GDO0 | GND / GND |
| 3 | GND | GND | 1 GND | GDO0 / GDO2 |
| 4 | GDO2 | GPIO33（J1-8） | 8 GDO2 | GND / GND |
| 5 | GND | GND | 1 GND | GDO2 / MISO |
| 6 | MISO | GPIO25（J1-9） | 7 SO | GND / GND |
| 7 | GND | GND | 1 GND | MISO / MOSI |
| 8 | MOSI | GPIO26（J1-10） | 6 SI | GND / GND |
| 9 | GND | GND | 1 GND | MOSI / CSN |
| 10 | CSN | GPIO27（J1-11）＋10kΩプルアップ | 4 CSN | GND / GND |
| 11 | GND | GND | 1 GND | CSN / SCK |
| 12 | SCK | GPIO14（J1-12） | 5 SCK | GND / GND |
| 13 | GND | GND | 1 GND | SCK / GND |
| 14 | GND | GND | 1 GND | GND / — |

- 奇数列（上段）が全部 GND なので、基板上は上段を1本の GND バスでつなげばよい
- CSN の 10kΩ プルアップは**CC1101基板側**（LDO 3.3V〜CSN 間）に置く。ケーブルが抜けていても CC1101 を非選択に保てる
- CC1101 の VCC 直近に 0.1µF パスコン

#### LCD 用 10P（2×5）: メイン基板側 / 液晶基板側 共通

信号 9本で GND は 1本しか取れないため、次の方針で並べる。

- **SCK は GND（1番）の隣**の 2番に置き、反対隣は RST にする
  （ST7789 のリセットは 10µs 以上の Low が必要なので、短いクロストークでは誤リセットしない）
- 第1LCD（RST / MOSI / DC / CS）→ 第2LCD（RST2 / CS2 / DC2）→ BLK の順にまとめる
- PWM で切り替わる BLK は端の 10番に置き、隣は DC2 にする（CS が High の間は DC が揺れても影響なし）

部品面から見た図

```
         【 切り欠き (上) 】
  1:GND    3:RST    5:DC     7:RST2   9:DC2
  2:SCK    4:MOSI   6:CS     8:CS2   10:BLK
```

はんだ面から見た図（左右反転）

```
         【 切り欠き (上) 】
  9:DC2    7:RST2   5:DC     3:RST    1:GND
 10:BLK    8:CS2    6:CS     4:MOSI   2:SCK
```

| ピン | 信号 | メイン基板側（ESP32） | 液晶基板側 | ケーブル上の両隣 |
|------|------|----------------------|------------|------------------|
| 1 | GND | GND（J3-1） | LCD1 / LCD2 GND | — / SCK |
| 2 | SCK | GPIO18（J3-9） | LCD1 / LCD2 SCL | GND / RST |
| 3 | RST | GPIO16（J3-12） | LCD1 RES | SCK / MOSI |
| 4 | MOSI | GPIO23（J3-2） | LCD1 / LCD2 SDA | RST / DC |
| 5 | DC | GPIO17（J3-11） | LCD1 DC | MOSI / CS |
| 6 | CS | GPIO5（J3-10） | LCD1 CS | DC / RST2 |
| 7 | RST2 | GPIO22（J3-3） | LCD2 RES | CS / CS2 |
| 8 | CS2 | GPIO21（J3-6） | LCD2 CS | RST2 / DC2 |
| 9 | DC2 | GPIO19（J3-8） | LCD2 DC | CS2 / BLK |
| 10 | BLK | GPIO4（J3-13） | Pch MOSFET 経由で LCD1 / LCD2 BLK | DC2 / — |

- J3 の並びとヘッダーの並びは一致しないため、メイン基板上で数本は交差する（AWG30/32 のジャンパーで逃がす）
- Pch MOSFET（ZVP2106A）・ゲート抵抗 100Ω・ゲートプルアップ 10kΩ は**液晶基板側**に置く想定。
  ケーブルが抜けてもバックライトが OFF のままになる
- GND が1本しかないので、ケーブルはできるだけ短くし、SPI クロックは 40MHz から始めて表示が乱れたら下げる
- GND を増やしたい場合は、RST と RST2 を1本にまとめて（両LCDを同時リセット）空いたピンを GND にする

### ソフトウェア設定

```cpp
// CC1101 (src/main.cpp)  HSPI を GPIO マトリクス経由で使用
static SPIClass cc1101Spi(HSPI);
static const int PIN_CS   = 27;   // CC1101 pin4 CSN（外付け10kΩプルアップ）
static const int PIN_SCK  = 14;   // CC1101 pin5 SCK
static const int PIN_MOSI = 26;   // CC1101 pin6 MOSI
static const int PIN_MISO = 25;   // CC1101 pin7 MISO
static const int PIN_GDO0 = 32;   // CC1101 pin3 GDO0 (Carrier Sense)
static const int PIN_GDO2 = 33;   // CC1101 pin8 GDO2 (Async Data)

// LCD  VSPI (IO_MUX 直結ピン)
#define LCD_SCK   18   // VSPI SCK
#define LCD_MOSI  23   // VSPI MOSI
#define LCD_RST   16   // 第1LCD RST
#define LCD_DC    17   // 第1LCD DC
#define LCD_CS     5   // 第1LCD CS
#define LCD_BLK    4   // バックライト（全LCD共用、LOW=ON）
#define LCD2_RST  22   // 第2LCD RST
#define LCD2_DC   19   // 第2LCD DC
#define LCD2_CS   21   // 第2LCD CS

// 照度センサー
#define PIN_LIGHT 34   // NJL7502L（ADC1_CH6）
```

---

## LCD ハードウェア構成

### 採用モジュール

| 項目 | 内容 |
|------|------|
| 型番 | 秋月電子 M154-240240-RGB |
| コントローラ | ST7789 |
| 解像度 | 240 × 240 px |
| 接続方式 | SPI（4線式, 書き込み専用）|
| 電源 | 3.3V |
| 購入日 | 2026-04-04 |


### ライブラリ: Adafruit ST7789 Library, GFX Library

platformio.ini に追加：

```ini
lib_deps =
  adafruit/Adafruit ST7735 and ST7789 Library
  adafruit/Adafruit GFX Library
```

### 参考資料

- [CC1101 Datasheet](https://www.ti.com/lit/ds/symlink/cc1101.pdf)
- [ESP32-S3-DevKitC-1 User Guide](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32s3/esp32-s3-devkitc-1/user_guide_v1.0.html)
- [RadioLib Documentation](https://github.com/jgromes/RadioLib)
- [Arduino TPMS Tyre Pressure Display](https://www.hackster.io/jsmsolns/arduino-tpms-tyre-pressure-display-b6e544)
- [reddit Need help decoding TPMS sensor](https://www.reddit.com/r/RTLSDR/comments/v0hqqf/need_help_decoding_tpms_sensor/)
- [Printables ESP32 with CC1101 Case](https://www.printables.com/model/1241571-esp32-with-cc1101-case)
- [rtl_433/BMW Gen5 TPMS support](https://github.com/merbanan/rtl_433/issues/2821)
