# iconcard 部品選定メモ

基板サイズ: 約30mm x 30mm x 10mm
実装: JLCPCB PCBA (SMD実装)

## IC（フットプリント割り当て済み）

| Ref | 部品 | パッケージ | 備考 |
|-----|------|-----------|------|
| U1 | STM32G030F6Px | TSSOP-20 | MCU |
| U2 | KXTJ3-1057 | LGA-12 (2x2mm) | 加速度計 |
| U3 | MCP73831-2-OT | SOT-23-5 | LiPo充電IC |
| U4 | AP2112K-3.3 | SOT-23-5 | 3.3V LDOレギュレータ |

## コネクタ・スイッチ（JLCPCB部品選定済み）

| Ref | 部品 | JLCPCB# | 型式 | 備考 |
|-----|------|---------|------|------|
| J4 | USB-Cレセプタクル 6P | C456012 | SHOU HAN TYPE-C 6P | 充電専用。シンボル: USB_C_Receptacle_PowerOnly_6P |
| Power | スライドスイッチ SPDT | C49023767 | MST-12D18G4 | 電源ON/OFF。シンボルをSPDTに変更済み |
| J1 | SHコネクタ 4P (1.0mm) | C7527619 | XDWF-0910-04P | デバッグ/UART用。常設 |

## 未選定の部品

### コネクタ

| Ref | 種類 | 仕様 | メモ |
|-----|------|------|------|
| J2 | ピンソケット 2.54mm 7pin | OLED接続用 | **PCBA除外・手はんだ**。フットプリントのみ基板に配置 |
| J3 | JST-PH 2pin (2.0mm) | C968533 | BOOMELE PH-2P SMD横型 | LiPoバッテリー用。**極性注意** |

### スイッチ

| Ref | 種類 | JLCPCB# | 型式 | メモ |
|-----|------|---------|------|------|
| RST | タクトスイッチ | C273519 | E-Switch TL3301AF160QG | 6x6x1.45mm SMD |
| BOOT | タクトスイッチ | C273519 | E-Switch TL3301AF160QG | 同上 |

### 抵抗（全て UNI-ROYAL 0402 ±1% Basic part）

| Ref | 値 | JLCPCB# | 型式 | 用途 |
|-----|-----|---------|------|------|
| R1 | 10K | C25744 | 0402WGF1002TCE | BOOT0プルダウン |
| R2 | 2K | C4109 | 0402WGF2001TCE | LED電流制限 |
| R3 | 2.2K | C25879 | 0402WGF2201TCE | I2Cプルアップ |
| R4 | 2.2K | C25879 | 0402WGF2201TCE | I2Cプルアップ |
| R5 | 5.1K | C25905 | 0402WGF5101TCE | USB-C CC1プルダウン |
| R6 | 5.1K | C25905 | 0402WGF5101TCE | USB-C CC2プルダウン |
| R7 | 4.99K | C25903 | 0402WGF4991TCE | MCP73831 PROG (充電≒200mA) |
| R8 | 2K | C4109 | 0402WGF2001TCE | LED電流制限 |

### コンデンサ

| Ref | 値 | サイズ | JLCPCB# | 型式 | 分類 | 用途 |
|-----|-----|--------|---------|------|------|------|
| C1 | 0.1uF | 0402 | C309458 | YAGEO CC0402KRX7R6BB104 (10V X7R) | Extended | デカップリング |
| C2 | 4.7uF | 0603 | C19666 | Samsung CL10A475KO8NNNC (16V X5R) | Basic | MCP73831 VBAT側 |
| C3 | 0.1uF | 0402 | C309458 | (同上) | Extended | デカップリング |
| C4 | 0.1uF | 0402 | C309458 | (同上) | Extended | デカップリング |
| C5 | 10uF | 0805 | C380332 | CCTC TCC0805X5R106K160FT (16V X5R) | Extended | デカップリング |
| C6 | 10uF | 0805 | C380332 | (同上) | Extended | MCP73831 VDD側 |
| C7 | 0.1uF | 0402 | C309458 | (同上) | Extended | KXTJ3デカップリング |
| C8 | 10uF | 0805 | C380332 | (同上) | Extended | AP2112K入力 |
| C9 | 10uF | 0805 | C380332 | (同上) | Extended | AP2112K出力 |

### LED（0603）

| Ref | 色 | JLCPCB# | 型式 | Vf | メモ |
|-----|-----|---------|------|-----|------|
| CHARGE_STATUS | 赤 | C2286 | KT-0603R (Hubei KENTO) | 2.4V | Basic part。R8=2K → 約0.45mA |
| STM32_STATUS | 緑 | C12624 | KT-0603G (Hubei KENTO) | 3.1V | Extended。R2=2K → 約0.1mA（暗め。必要ならR値下げ検討） |

## フットプリント割り当て

全部品のフットプリント割り当て完了。カスタムフットプリントは `custom_footprints/` に格納:
- `MST-12D18G4.pretty` — スライドスイッチ (C49023767)
- `TL3301AF160QG.pretty` — タクトスイッチ (C273519) ※パッド名を左列=1, 右列=2にリネーム済み
- `TYPE-C-6P.pretty` — USB-C 6P (C456012) ※シールドパッド名を7→S1にリネーム済み

## 回路図修正履歴

- [x] MCP73831 VDD/STAT ピンスワップ修正
- [x] AP2112K-3.3 LDO追加（U4）
- [x] パワースイッチをスライドスイッチに変更（DPDT → SPDT）
- [x] AP2112K ENピンをVINに接続
- [x] スイッチ未使用ピンにno_connectマーカー追加
- [x] USB-CシンボルをUSB_C_Receptacle_PowerOnly_6Pに変更（J4）
