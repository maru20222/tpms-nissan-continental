"""TPMS 回路図: まずは各デバイスをピン付きシンボルとして配置する。

サンプル画像のスタイルに合わせ、配線はまだ引かず、
doc/index.md のピン割当を各デバイスのピンとして表現する。

ESP32 は ESP32-S3-DevKitC-1 の公式ピンレイアウト (J1/J3) の
全ヘッダピンを表現する。
参考: https://docs.espressif.com/projects/esp-dev-kits/en/latest/
      esp32s3/_images/ESP32-S3_DevKitC-1_pinlayout.jpg
"""

import itertools
from pathlib import Path

import matplotlib

# ウィンドウを一切表示しない (非対話 Agg バックエンドに固定)。
matplotlib.use('Agg')

import schemdraw
import schemdraw.elements as elm
from schemdraw.elements import Ic, IcPin


def _side_pins(items, side):
    """上から下へ並ぶピン定義を IcPin のリストへ変換する。

    schemdraw では slot 番号が大きいほど上側に配置されるため、
    先頭(上)を total、末尾(下)を 1 に割り当てる。

    items の各要素は文字列 (ピン名) か、
    (ピン名, アンカー名) のタプルを受け付ける。
    アンカー名を指定すると結線時に ``ic.<アンカー名>`` で参照できる。
    """
    total = len(items)
    pins = []
    for i, item in enumerate(items):
        if isinstance(item, tuple):
            name, anchor = item
        else:
            name, anchor = item, None
        slot = f'{total - i}/{total}'
        kwargs = {'anchorname': anchor} if anchor else {}
        pins.append(IcPin(name=name, side=side, slot=slot, **kwargs))
    return pins


def _hv_intersect(h, v):
    """水平セグメント h と垂直セグメント v が交差するか判定。"""
    hx1, hx2, hy = h
    vx, vy1, vy2 = v
    x1, x2 = sorted((hx1, hx2))
    y1, y2 = sorted((vy1, vy2))
    return x1 < vx < x2 and y1 < hy < y2


def _wire_segments(cc_edge_x, esp_edge_x, cc_y, esp_y, bus_x):
    """'c'(ↄ字) 配線の水平/垂直セグメントを返す。"""
    return {
        'h': [(cc_edge_x, bus_x, cc_y), (bus_x, esp_edge_x, esp_y)],
        'v': [(bus_x, cc_y, esp_y)],
    }


def _count_crossings(segs):
    """全配線ペアの直交交差数を数える。"""
    total = 0
    for a, b in itertools.combinations(segs, 2):
        for h in a['h']:
            for v in b['v']:
                if _hv_intersect(h, v):
                    total += 1
        for h in b['h']:
            for v in a['v']:
                if _hv_intersect(h, v):
                    total += 1
    return total


def _solve_bus_order(coords, cc_edge_x, esp_edge_x, span=(0.0, 1.0)):
    """各配線の (cc_y, esp_y) から交差が最小になる縦バス順を解く。

    2 IC 間 (channel) のうち span=(lo, hi) で指定した区間を等間隔に
    分割した bus 位置へ各線を割り当て、交差数が最小 (可能なら 0) に
    なる順列を総当たりで求める。span を左寄り (例 hi<1.0) にすると
    折れ場所(縦バス)を全体的に左へ寄せられる。
    ピン数が小さい前提 (n! 探索)。返り値は各線に割り当てる bus 位置。
    """
    n = len(coords)
    channel = esp_edge_x - cc_edge_x
    lo, hi = span
    lo_x = cc_edge_x + channel * lo
    band = channel * (hi - lo)
    slots = [lo_x + band * (r + 1) / (n + 1) for r in range(n)]

    best_perm = tuple(range(n))
    best_cross = None
    for perm in itertools.permutations(range(n)):
        segs = [
            _wire_segments(cc_edge_x, esp_edge_x, cy, ey, slots[perm[i]])
            for i, (cy, ey) in enumerate(coords)
        ]
        c = _count_crossings(segs)
        if best_cross is None or c < best_cross:
            best_cross, best_perm = c, perm
            if c == 0:
                break
    return [slots[best_perm[i]] for i in range(n)], best_cross


def _over_top_segments(start, end, riser_x, drop_x, top_y):
    """LCD → ESP を「上から回り込む」直角配線のセグメントを返す。

    経路: start(左向き) → 左へ riser_x → 上へ top_y → 左へ drop_x
          → 下へ end.y → 右へ end(左向きピン先端)。
    riser_x は ESP 本体の右側 (body と LCD の間)、
    drop_x は ESP 本体の左側に置くことで本体・ラベルを避ける。
    """
    sx, sy = start.x, start.y
    ex, ey = end.x, end.y
    h = [(sx, riser_x, sy), (riser_x, drop_x, top_y), (drop_x, ex, ey)]
    v = [(riser_x, sy, top_y), (drop_x, top_y, ey)]
    return {'h': h, 'v': v}


def _solve_over_top(coords, riser0, riser_step, drop0, drop_step, top0, top_step):
    """上回り配線の交差を最小化する (riser/drop/top を独立に決める)。

    coords: [(start_anchor, end_anchor), ...]
    - riser_x (LCD 側の縦線): start.y が高い線ほど LCD に近い(右)に置く
      → LCD 側の入口で線が交差しない。
    - drop_x (ESP 側の縦線): end.y が高い線ほど ESP に近い(右)に置く
      → ESP 側の出口で線が交差しない。
    - top_y (天面の横レーン): 総当たりで割り当て、残る交差を最小化。
      (LCD 順と ESP 順のズレにより天面で不可避に生じる交差のみ残る)
    返り値: 各ネットの (riser_x, drop_x, top_y) と交差数。
    """
    n = len(coords)

    # riser: start.y 降順(上から)に、LCD へ近い順(右=大きい riser_x)
    start_rank = {i: r for r, i in enumerate(
        sorted(range(n), key=lambda i: -coords[i][0].y))}
    riser_of = {i: riser0 + (n - 1 - start_rank[i]) * riser_step for i in range(n)}

    # drop: end.y 降順(上から)に、ESP へ近い順(右=大きい drop_x=drop0)
    end_rank = {i: r for r, i in enumerate(
        sorted(range(n), key=lambda i: -coords[i][1].y))}
    drop_of = {i: drop0 - end_rank[i] * drop_step for i in range(n)}

    tops = [top0 + r * top_step for r in range(n)]
    best_perm, best_cross = tuple(range(n)), None
    for perm in itertools.permutations(range(n)):
        segs = [
            _over_top_segments(s, e, riser_of[i], drop_of[i], tops[perm[i]])
            for i, (s, e) in enumerate(coords)
        ]
        c = _count_crossings(segs)
        if best_cross is None or c < best_cross:
            best_cross, best_perm = c, perm
            if c == 0:
                break
    params = [
        (riser_of[i], drop_of[i], tops[best_perm[i]])
        for i in range(n)
    ]
    return params, best_cross


def build_tpms_diagram(output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)

    with schemdraw.Drawing(show=False) as d:
        d.config(unit=2.0, lw=1.6, fontsize=9)

        # ------------------------------------------------------------------
        # U1: ESP32-S3-DevKitC-1 (ESP32-S3-WROOM-2 N32R16V)
        #   公式ピンレイアウト J1(左) / J3(右) の全ヘッダピンを表現。
        #   () 内はこのプロジェクトでの用途 (doc/index.md 準拠)。
        # ------------------------------------------------------------------
        j1_left = [
            ('3V3', 'ESP_3V3'),
            ('3V3', 'ESP_3V3_2'),
            'RST',
            ('4 (LCD1_RST)', 'ESP_LCD1_RST'),
            ('5 (LCD1_DC)', 'ESP_LCD1_DC'),
            ('6 (LCD1_CS)', 'ESP_LCD1_CS'),
            ('7 (LCD_BLK)', 'ESP_LCD_BLK'),
            ('15 (GDO2)', 'ESP_GDO2'),
            ('16 (GDO0)', 'ESP_GDO0'),
            ('17 (LCD_SCK)', 'ESP_LCD_SCK'),
            ('18 (LCD_MOSI)', 'ESP_LCD_MOSI'),
            '8',
            '3',
            '46',
            '9',
            ('10 (CC_CS)', 'ESP_CC_CS'),
            ('11 (CC_MOSI)', 'ESP_CC_MOSI'),
            ('12 (CC_SCK)', 'ESP_CC_SCK'),
            ('13 (CC_MISO)', 'ESP_CC_MISO'),
            '14',
            '5V',
            ('GND', 'ESP_GND'),
        ]
        j3_right = [
            ('GND', 'ESP_GND_TR'),
            'TX',
            'RX',
            '1',
            '2',
            '42',
            '41',
            '40',
            '39',
            '38',
            '37',
            '36',
            '35',
            '0',
            '45',
            '48',
            '47',
            ('21 (LCD2_DC)', 'ESP_LCD2_DC'),
            ('20 (LCD2_CS)', 'ESP_LCD2_CS'),
            ('19 (LCD2_RST)', 'ESP_LCD2_RST'),
            'GND',
            'GND',
        ]
        esp = Ic(
            pins=_side_pins(j1_left, 'left') + _side_pins(j3_right, 'right'),
            edgepadW=1.5,
            edgepadH=0.5,
            pinspacing=1.0,
            leadlen=0.9,
        )
        esp.label('U1  ESP32-S3 DevKitC-1 (WROOM-2)', loc='bottom', ofst=0.4)
        d.add(esp)

        # ------------------------------------------------------------------
        # U2: CC1101  (315MHz サブGHz トランシーバ, 8ピン)
        #   ESP32 が右側にあるため、結線する全ピンを右側に集約して
        #   右方向へまっすぐ配線する。並び順は ESP32 側のピン位置に
        #   近づけて交差を減らす。
        # ------------------------------------------------------------------
        cc = Ic(
            pins=[
                IcPin(name='2 VCC', side='right', slot='8/8', anchorname='CC_VCC'),
                IcPin(name='8 GDO2', side='right', slot='7/8', anchorname='CC_GDO2'),
                IcPin(name='3 GDO0', side='right', slot='6/8', anchorname='CC_GDO0'),
                IcPin(name='4 CSN', side='right', slot='5/8', anchorname='CC_CSN'),
                IcPin(name='6 MOSI', side='right', slot='4/8', anchorname='CC_MOSI'),
                IcPin(name='5 SCK', side='right', slot='3/8', anchorname='CC_SCK'),
                IcPin(name='7 MISO', side='right', slot='2/8', anchorname='CC_MISO'),
                IcPin(name='1 GND', side='right', slot='1/8', anchorname='CC_GND'),
            ],
            edgepadW=1.0,
            leadlen=0.9,
        )
        cc.label('U2  CC1101 315MHz', loc='bottom', ofst=0.4)
        d.add(cc.at((-13.0, 4.0)))

        # ------------------------------------------------------------------
        # Q1: Pチャネル MOSFET (AO3401 等) — LCD バックライト駆動 (高側スイッチ)
        #   単一 BLK ピンの LCD は低側 Nch では切れないため高側 Pch を使う。
        #   S→+3.3V, D→BLK, G←GPIO7(100Ω), G→10k で +3.3V プルアップ。
        #   GPIO7=LOW で導通・点灯、PWM 調光可、起動時は自動 OFF。
        #   backlight_drive_ascii.svg の意図(高側 Pch)に部品を合わせた。
        #   anchors: source / drain / gate
        # ------------------------------------------------------------------
        q1 = elm.PFet()
        q1.label('Q1  ZVP2106A (TO-92, Pch)', loc='bottom', ofst=0.4)
        q1.label('D', loc='drain', ofst=(0.1, 0.0), halign='left')
        q1.label('G', loc='gate', ofst=(0.0, 0.15), valign='bottom')
        q1.label('S', loc='source', ofst=(0.1, 0.0), halign='left')
        d.add(q1.at((-13.0, 20.0)))

        # 注: Q1 のゲート配線(左向き Line)はここに置くと後続の LCD
        #   配置が描画方向(theta=180)を継承して 180° 回転してしまう。
        #   そのため全 IC 配置後の末尾(配線セクション)で行う。

        # ------------------------------------------------------------------
        # LCD1: M154-240240-RGB (ST7789, 240x240) 左側
        # ------------------------------------------------------------------
        lcd1 = Ic(
            pins=[
                IcPin(name='VCC', side='left', slot='8/8', anchorname='L1_VCC'),
                IcPin(name='GND', side='left', slot='7/8', anchorname='L1_GND'),
                IcPin(name='SCL', side='left', slot='6/8', anchorname='L1_SCL'),
                IcPin(name='SDA', side='left', slot='5/8', anchorname='L1_SDA'),
                IcPin(name='RES', side='left', slot='4/8', anchorname='L1_RES'),
                IcPin(name='DC', side='left', slot='3/8', anchorname='L1_DC'),
                IcPin(name='CS', side='left', slot='2/8', anchorname='L1_CS'),
                IcPin(name='BLK', side='left', slot='1/8', anchorname='L1_BLK'),
            ],
            edgepadW=1.0,
            leadlen=0.9,
        )
        lcd1.label('LCD1  ST7789 240x240', loc='bottom', ofst=0.4)
        d.add(lcd1.at((20.0, 5.0)))

        # ------------------------------------------------------------------
        # LCD2: M154-240240-RGB (ST7789, 240x240) 右側
        # ------------------------------------------------------------------
        lcd2 = Ic(
            pins=[
                IcPin(name='VCC', side='left', slot='8/8', anchorname='L2_VCC'),
                IcPin(name='GND', side='left', slot='7/8', anchorname='L2_GND'),
                IcPin(name='SCL', side='left', slot='6/8', anchorname='L2_SCL'),
                IcPin(name='SDA', side='left', slot='5/8', anchorname='L2_SDA'),
                IcPin(name='RES', side='left', slot='4/8', anchorname='L2_RES'),
                IcPin(name='DC', side='left', slot='3/8', anchorname='L2_DC'),
                IcPin(name='CS', side='left', slot='2/8', anchorname='L2_CS'),
                IcPin(name='BLK', side='left', slot='1/8', anchorname='L2_BLK'),
            ],
            edgepadW=1.0,
            leadlen=0.9,
        )
        lcd2.label('LCD2  ST7789 240x240', loc='bottom', ofst=0.4)
        d.add(lcd2.at((20.0, -4.0)))

        # ------------------------------------------------------------------
        # 結線: ESP32 <-> CC1101
        #   CC1101 の全ピンを右側(ESP32 側)に集約し、'c'(ↄ字) の
        #   直角配線で結ぶ。縦バスの位置(k)は始点からの絶対水平距離な
        #   ため、単調に増減させると交差(ジグザグ)や突き抜けが起きる。
        #   → 実際のピン座標から交差ゼロになる bus 順を解いて割り当てる。
        # ------------------------------------------------------------------
        cc_links = [
            # (CC アンカー, ESP アンカー, 色)
            (cc.CC_VCC, esp.ESP_3V3, 'red'),
            (cc.CC_GDO2, esp.ESP_GDO2, 'black'),
            (cc.CC_GDO0, esp.ESP_GDO0, 'black'),
            (cc.CC_CSN, esp.ESP_CC_CS, 'black'),
            (cc.CC_MOSI, esp.ESP_CC_MOSI, 'black'),
            (cc.CC_SCK, esp.ESP_CC_SCK, 'black'),
            (cc.CC_MISO, esp.ESP_CC_MISO, 'black'),
            (cc.CC_GND, esp.ESP_GND, 'black'),
        ]
        cc_edge_x = cc.CC_VCC.x      # CC 右側ピン先端の X (全線共通)
        esp_edge_x = esp.ESP_3V3.x   # ESP 左側ピン先端の X (全線共通)
        coords = [(cp.y, ep.y) for cp, ep, _ in cc_links]
        # 折れ場所(縦バス)を channel の左寄り区間に収めて左へ寄せる。
        bus_xs, _ = _solve_bus_order(coords, cc_edge_x, esp_edge_x, span=(0.05, 0.5))
        # VCC(最上段, 赤)は端子直上から立ち上げる(k=0)。
        bus_xs[0] = cc_edge_x

        for (cc_pin, esp_pin, col), bus_x in zip(cc_links, bus_xs):
            k = bus_x - cc_edge_x   # 'c' シェイプの最初の水平長 = 始点→縦バス
            d.add(
                elm.Wire('c', k=k)
                .at(cc_pin)
                .to(esp_pin)
                .color(col)
            )

        # ------------------------------------------------------------------
        # デカップリング: CC1101 pin2(VCC) - pin1(GND) 間に 0.1µF
        #   電源直近のバイパスコンデンサ。信号線(pin3〜8)と交差しない
        #   よう本体左側(x=-14.5)へ縦置きし、VCC は本体上・GND は本体下
        #   を回して接続する。両ピン先端(タップ点)に Dot を打つ。
        # ------------------------------------------------------------------
        cc_cap_x = -14.5
        cc_cap_top, cc_cap_bot = 7.6, 5.6
        # VCC(pin2): 本体上(y=9.8)を回って左の縦置きコンデンサ上端へ
        cc_vcc_pts = [
            (cc.CC_VCC.x, cc.CC_VCC.y),
            (cc.CC_VCC.x, 9.8),
            (cc_cap_x, 9.8),
            (cc_cap_x, cc_cap_top),
        ]
        for a, b in zip(cc_vcc_pts[:-1], cc_vcc_pts[1:]):
            d.add(elm.Line().at(a).to(b).color('red'))
        # GND(pin1): 本体下(y=3.0)を回って左の縦置きコンデンサ下端へ
        cc_gnd_pts = [
            (cc.CC_GND.x, cc.CC_GND.y),
            (cc.CC_GND.x, 3.0),
            (cc_cap_x, 3.0),
            (cc_cap_x, cc_cap_bot),
        ]
        for a, b in zip(cc_gnd_pts[:-1], cc_gnd_pts[1:]):
            d.add(elm.Line().at(a).to(b).color('black'))
        cccap = elm.Capacitor().at((cc_cap_x, cc_cap_top)).to((cc_cap_x, cc_cap_bot))
        cccap.label('0.1µF', loc='bottom', ofst=0.3)
        d.add(cccap)
        d.add(elm.Dot(radius=0.12).at((cc.CC_VCC.x, cc.CC_VCC.y)).color('red').fill('red'))
        d.add(elm.Dot(radius=0.12).at((cc.CC_GND.x, cc.CC_GND.y)).color('black').fill('black'))

        # ------------------------------------------------------------------
        # 結線: ESP32 <-> LCD1
        #   LCD は物理ピン順(VCC,GND,SCL,SDA,RES,DC,CS,BLK)固定。
        #   ・GND : ESP32 右上 GND を利用し、LCD 右側から上へ回して接続。
        #   ・VCC : LCD VCC から "垂直に" 上げ、最上部を横断して 3V3 へ。
        #           (VCC を分離すると残り線の交差が減る)
        #   ・制御6本 : ESP32 左側(J1)ピンへ「上から回り込む」アーチ。
        #           折れ点(riser)を LCD 寄り(右)に置き、ESP 付近の
        #           めり込みを避ける。ピン順が違うため交差は許容。
        # ------------------------------------------------------------------
        # GND: LCD1 → ESP32 右上 GND (右側で上へ回す)
        g_riser_x = 15.1   # 共通タップ(SCL riser=14.5)と重ならないよう右寄せ
        gnd_top_y = esp.ESP_GND_TR.y
        gnd_pts = [
            (lcd1.L1_GND.x, lcd1.L1_GND.y),
            (g_riser_x, lcd1.L1_GND.y),
            (g_riser_x, gnd_top_y),
            (esp.ESP_GND_TR.x, gnd_top_y),
        ]
        for a, b in zip(gnd_pts[:-1], gnd_pts[1:]):
            d.add(elm.Line().at(a).to(b).color('black'))

        # LCD2 が共有(タップ)する LCD1 側の合流点を記録する。
        #   {信号: (x, y, 色)} 各点は LCD1 の折れ点(縦線の下端付近)。
        tap = {}
        tap['GND'] = (g_riser_x, lcd1.L1_GND.y, 'black')

        # VCC: LCD1 VCC から垂直に上げ、最上レーンを横断して 3V3 へ
        #   制御アーチの top 帯(23.0〜26.0)より上に置く必要があるため
        #   26.5 が安全な最小値(これ未満だとアーチと交差する)。
        vcc_top_y = 26.5    # 全アーチより高い最上レーン
        vcc_drop_x = -2.2   # ESP 左リード線先端より外側
        vcc_pts = [
            (lcd1.L1_VCC.x, lcd1.L1_VCC.y),
            (lcd1.L1_VCC.x, vcc_top_y),          # 垂直に上昇
            (vcc_drop_x, vcc_top_y),             # 最上部を横断
            (vcc_drop_x, esp.ESP_3V3.y),         # 左端で下降
            (esp.ESP_3V3.x, esp.ESP_3V3.y),      # 3V3 へ
        ]
        for a, b in zip(vcc_pts[:-1], vcc_pts[1:]):
            d.add(elm.Line().at(a).to(b).color('red'))
        tap['VCC'] = (lcd1.L1_VCC.x, lcd1.L1_VCC.y, 'red')

        # CC1101 の 3V3 線(水平 y=ESP_3V3.y)と LCD の 3V3 縦線
        # (x=vcc_drop_x)の交点に赤丸(接続点)を打つ。
        d.add(elm.Dot(radius=0.12)
              .at((vcc_drop_x, esp.ESP_3V3.y))
              .color('red').fill('red'))

        # 制御5本: 上から回り込む入れ子アーチ
        #   BLK は GPIO7 直結をやめ(MOSFET 経由へ変更)、ここでは
        #   制御 5 本(SCL/SDA/RES/DC/CS)のみを回り込ませる。
        lcd1_links = [
            # (LCD1 アンカー, ESP アンカー, 色, 信号名)
            (lcd1.L1_SCL, esp.ESP_LCD_SCK, 'black', 'SCL'),
            (lcd1.L1_SDA, esp.ESP_LCD_MOSI, 'black', 'SDA'),
            (lcd1.L1_RES, esp.ESP_LCD1_RST, 'black', 'RES'),
            (lcd1.L1_DC, esp.ESP_LCD1_DC, 'black', 'DC'),
            (lcd1.L1_CS, esp.ESP_LCD1_CS, 'black', 'CS'),
        ]
        l1_coords = [(s, e) for s, e, _, _ in lcd1_links]
        ctrl_riser = {}   # LCD1 の RES/DC/CS riser X (LCD2 と揃える)
        # riser(LCD 側の折れ点)を LCD 寄り(右)に配置して ESP 付近の
        # めり込みを回避。drop は ESP 左リード線先端より外側、
        # top は ESP 天面(21.5)より十分上に入れ子で置く。
        l1_params, _ = _solve_over_top(
            l1_coords,
            riser0=10.5, riser_step=0.8,
            drop0=-1.4, drop_step=0.55,
            top0=23.0, top_step=0.6,
        )
        for (s, e, col, sig), (riser_x, drop_x, top_y) in zip(lcd1_links, l1_params):
            pts = [
                (s.x, s.y),
                (riser_x, s.y),
                (riser_x, top_y),
                (drop_x, top_y),
                (drop_x, e.y),
                (e.x, e.y),
            ]
            for a, b in zip(pts[:-1], pts[1:]):
                d.add(elm.Line().at(a).to(b).color(col))
            # LCD2 と共有する信号は LCD1 の折れ点(riser 下端)を記録
            if sig in ('SCL', 'SDA'):
                tap[sig] = (riser_x, s.y, col)
            # RES/DC/CS は LCD2 の縦線 X を LCD1 と揃えるため記録
            if sig in ('RES', 'DC', 'CS'):
                ctrl_riser[sig] = riser_x

        # BLK は GPIO7 直結を切ったので、LCD1↔LCD2 の共有ノード
        # として扱う。共有の縦線 X は GND リサー(15.3)より左に
        # 寄せた 15.0 で一元管理し、LCD2 側と D への配線で共用する。
        blk_share_x = 14.5
        tap['BLK'] = (blk_share_x, lcd1.L1_BLK.y, 'black')

        # ------------------------------------------------------------------
        # 結線: LCD2
        #   ・共通信号(VCC/GND/SCL/SDA/BLK)は LCD1 の折れ点(tap)に合流。
        #     合流点(交点)に Dot を打つ(VCC=赤, その他=黒)。
        #   ・固有信号(RES/DC/CS)は ESP32 右側(J3)の 19/21/20 へ配線。
        # ------------------------------------------------------------------
        # 共通信号: LCD2 ピン → 左へ tap_x → 上へ tap_y(LCD1 折れ点)へ合流
        lcd2_shared = [
            (lcd2.L2_VCC, 'VCC'),
            (lcd2.L2_GND, 'GND'),
            (lcd2.L2_SCL, 'SCL'),
            (lcd2.L2_SDA, 'SDA'),
            (lcd2.L2_BLK, 'BLK'),
        ]
        for pin, sig in lcd2_shared:
            tx, ty, col = tap[sig]
            pts = [
                (pin.x, pin.y),
                (tx, pin.y),   # 左へ水平移動して LCD1 折れ点の X へ
                (tx, ty),      # 上へ垂直移動して合流点へ
            ]
            for a, b in zip(pts[:-1], pts[1:]):
                d.add(elm.Line().at(a).to(b).color(col))
            # 合流点(交点)に Dot
            d.add(elm.Dot(radius=0.12).at((tx, ty)).color(col).fill(col))

        # BLK 共有ノード(LCD1&LCD2) → Q1 ドレイン(D)
        #   over-top で配線。既存の制御アーチ(23.0〜25.4)・VCC レーン
        #   (26.5)より上の y=27.5 を横断レーンにする。VCC の縦線
        #   (x=17.9)と重ならないよう BLK ライザーは x=16 に取り、Q1 の
        #   左(x=-14.5)で下降してドレインへ入る。
        blk_node = (lcd1.L1_BLK.x, lcd1.L1_BLK.y)   # (17.9, 4.5)
        drain = q1.drain                            # (-13.0, 18.5)
        blk_riser_x = blk_share_x   # LCD2 共有縦線と同じ X(15.0)
        blk_top_y = 27.5
        blk_drop_x = -14.5
        blk_pts = [
            (blk_node[0], blk_node[1]),
            (blk_riser_x, blk_node[1]),
            (blk_riser_x, blk_top_y),
            (blk_drop_x, blk_top_y),
            (blk_drop_x, drain.y),
            (drain.x, drain.y),
        ]
        for a, b in zip(blk_pts[:-1], blk_pts[1:]):
            d.add(elm.Line().at(a).to(b).color('black'))

        # 固有信号: LCD2 RES/DC/CS → ESP32 J3(19/21/20)
        lcd2_ctrl = [
            (lcd2.L2_RES, esp.ESP_LCD2_RST, 'black', 'RES'),
            (lcd2.L2_DC, esp.ESP_LCD2_DC, 'black', 'DC'),
            (lcd2.L2_CS, esp.ESP_LCD2_CS, 'black', 'CS'),
        ]
        # 縦線 X は LCD1 の同信号 riser に揃える(RES/DC/CS で整列)。
        for (cp, ep, col, sig) in lcd2_ctrl:
            bx = ctrl_riser[sig]
            pts = [
                (cp.x, cp.y),
                (bx, cp.y),
                (bx, ep.y),
                (ep.x, ep.y),
            ]
            for a, b in zip(pts[:-1], pts[1:]):
                d.add(elm.Line().at(a).to(b).color(col))

        # ------------------------------------------------------------------
        # Q1 ゲート駆動: GPIO7 → 100Ω → Gate
        #   BLK は MOSFET ドレインへ回すため、GPIO7 はゲートへ 100Ω
        #   直列で接続する(LOW=ON)。左向き Line を含むため、LCD 等の
        #   IC 配置がすべて済んだこの末尾で描く(描画方向の継承を回避)。
        # ------------------------------------------------------------------
        g7 = esp.ESP_LCD_BLK          # GPIO7 先端 (-0.9, 15.5)
        gate = q1.gate                # Q1 ゲート (-11.63, 19.25)
        r_r_x, r_l_x = -4.3, -6.3     # 100Ω の右端/左端 X (y=g7.y)
        gate_riser_x = -10.5          # CC 縦バス(-9.5..-5.5)より左に退避
        d.add(elm.Line().at((g7.x, g7.y)).to((r_r_x, g7.y)).color('black'))
        rg = elm.Resistor().at((r_r_x, g7.y)).to((r_l_x, g7.y))
        rg.label('100Ω', loc='top', ofst=0.15)
        d.add(rg)
        gate_pts = [
            (r_l_x, g7.y),
            (gate_riser_x, g7.y),
            (gate_riser_x, gate.y),
            (gate.x, gate.y),
        ]
        for a, b in zip(gate_pts[:-1], gate_pts[1:]):
            d.add(elm.Line().at(a).to(b).color('black'))

        # Q1 ソース(S) → ESP32 左 2 番目の 3V3 (赤)
        #   S(-13,20) から真っ直ぐ左へ引き、3V3_2 の手前(外側)で
        #   下げて 3V3 先端へ入る。
        src = q1.source                 # (-13.0, 20.0)
        v33 = esp.ESP_3V3_2             # ESP 左側 2 番目の 3V3 先端
        s_drop_x = -3.4                 # ESP 左リード線先端より外側
        src_pts = [
            (src.x, src.y),
            (s_drop_x, src.y),
            (s_drop_x, v33.y),
            (v33.x, v33.y),
        ]
        for a, b in zip(src_pts[:-1], src_pts[1:]):
            d.add(elm.Line().at(a).to(b).color('red'))

        # 10kΩ プルアップ: ゲート ⇔ ソース(+3.3V) 間
        #   ゲート水平線(y=g7.y, x=r_l_x..gate_riser_x)と
        #   ソース水平線(y=src.y)の双方が通る x=-8.0 に縦向きで配置。
        #   起動時に確実に OFF(Pch は Vgs=0 で遮断)にする。
        pu_x = -8.0
        rp = elm.Resistor().at((pu_x, g7.y)).to((pu_x, src.y))
        rp.label('10kΩ', loc='bottom', ofst=0.15)
        d.add(rp)
        # 両端の接続点に Dot(ゲート側=黒, ソース側=赤)
        d.add(elm.Dot(radius=0.12).at((pu_x, g7.y)).color('black').fill('black'))
        d.add(elm.Dot(radius=0.12).at((pu_x, src.y)).color('red').fill('red'))

        # デカップリング: 各 LCD の VCC-GND 間に 0.1µF と 1µF を並列
        #   VCC/GND ピン先端(x=19.1)のすぐ左に 2 本縦置する。
        #   cx1=0.1µF(X7R), cx2=1µF(X7R) を同一箱所に近接して並列。
        for vcc, gnd in [(lcd1.L1_VCC, lcd1.L1_GND),
                         (lcd2.L2_VCC, lcd2.L2_GND)]:
            for cx, cval, cloc in [(18.6, '0.1µF', 'right'),
                                   (17.6, '1µF', 'left')]:
                d.add(elm.Line().at((vcc.x, vcc.y)).to((cx, vcc.y)).color('red'))
                d.add(elm.Line().at((gnd.x, gnd.y)).to((cx, gnd.y)).color('black'))
                cap = elm.Capacitor().at((cx, vcc.y)).to((cx, gnd.y))
                cap.label(cval, loc=cloc, ofst=0.15)
                d.add(cap)

        # 22µF バルク(電解/ポリマー): 電源レール間に 1 個
        #   GND 線の角(g_riser_x, gnd_top_y)=GND 側と、その水平線を
        #   延長した LCD VCC 縦線(L1_VCC.x)との交点=VCC 側の間に
        #   y=gnd_top_y に水平配置する。
        bulk_gnd_x = g_riser_x          # 15.1 (GND 角)
        bulk_vcc_x = lcd1.L1_VCC.x      # 19.1 (VCC 縦線)
        bulk_y = gnd_top_y              # 21.5
        bulk_l = bulk_gnd_x + 1.3       # キャパシタ左端(GND側)
        bulk_r = bulk_vcc_x - 1.3       # キャパシタ右端(VCC側)
        d.add(elm.Line().at((bulk_gnd_x, bulk_y)).to((bulk_l, bulk_y)).color('black'))
        # 有極性: + を VCC(高電位)側にするため VCC 側から GND 側へ描く。
        bcap = elm.Capacitor(polar=True).at((bulk_r, bulk_y)).to((bulk_l, bulk_y))
        bcap.label('22µF', loc='top', ofst=0.15)
        d.add(bcap)
        d.add(elm.Line().at((bulk_r, bulk_y)).to((bulk_vcc_x, bulk_y)).color('red'))
        # 両端の接続点に Dot(GND 角=黒, VCC 交点=赤)
        d.add(elm.Dot(radius=0.12).at((bulk_gnd_x, bulk_y)).color('black').fill('black'))
        d.add(elm.Dot(radius=0.12).at((bulk_vcc_x, bulk_y)).color('red').fill('red'))

        d.save(str(output_dir / 'tpms_circuit_schemdraw.svg'), transparent=False)
        d.save(str(output_dir / 'tpms_circuit_schemdraw.png'), transparent=False)


if __name__ == '__main__':
    build_tpms_diagram(Path(__file__).resolve().parent)
