# ROS2-Seeed-Studio-USB-CAN-Analyzer-Driver

Seeed Studio USB-CAN Analyzer の ROS 2（Kilted 想定）向けドライバです。USB シリアル I/O には Boost.Asio を使用します。

## 選択テキストの要約（日本語）

### 背景
- USB-CAN Analyzer は安価で実用的な CAN 解析手段だが、公式のホスト通信仕様が十分に公開されていない。
- そのため Linux（SocketCAN など）向けの標準的なソフトウェア対応が乏しく、独自実装が必要になりやすい。
- 解析結果として、USB 側シリアルプロトコルの実用上重要な情報が整理されている。

### 既存公開資料との差分
- 以前公開された「USB (Serial port) to CAN protocol defines」の説明は、実機挙動と一致しない点がある。
- 実装側では高ボーレート・高負荷時の問題対策として、固定 20 バイトではなく可変長フレーム（6〜16 バイト）へ最適化した可能性が高い。

### CAN データフレーム（送受信共通）
- 先頭は `0xAA`（開始）、末尾は `0x55`（終了）。
- 2 バイト目はフレーム情報：
  - Bit7/6 は常に 1
  - Bit5: 0=標準ID(11bit), 1=拡張ID(29bit)
  - Bit4: 0=データフレーム, 1=リモートフレーム
  - Bit3..0: DLC(0〜8)
- 標準IDは ID を 2 バイト、拡張IDは ID を 4 バイト（LSB→MSB）で格納。
- 受信/送信データ部は DLC に応じた長さ。
- データフレームにはチェックサムがない（転送量と処理時間削減のためと推定）。

### CAN 初期化フレーム
- 固定 20 バイト。
- 主な項目：開始バイト列、メッセージID `0x12`、CAN ボーレート、送信ID形式、Filter/Mask、動作モード、チェックサム。
- チェックサムは byte 2〜18 を対象に処理。

### CAN ボーレート定義
- `0x01`=1000k, `0x02`=800k, `0x03`=500k, `0x04`=400k, `0x05`=250k, `0x06`=200k,
  `0x07`=125k, `0x08`=100k, `0x09`=50k, `0x0A`=20k, `0x0B`=10k, `0x0C`=5k。

### 動作モード
- `0x00`=Normal
- `0x01`=Loopback
- `0x02`=Silent (Listen-Only)
- `0x03`=Loopback + Silent

### CAN コントローラ状態フレーム
- 要求フレームと応答フレームは同一フォーマット。
- ステータス要求時は byte 3〜18 が `0x00`。
- メッセージID は `0x04`、byte 3/4 は受信/送信エラーカウンタ。

## ビルド

```bash
colcon build --packages-select seeed_usb_can_analyzer_driver
```

## 実行

```bash
ros2 run seeed_usb_can_analyzer_driver usb_can_analyzer_node
```
