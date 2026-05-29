# ROS2-Seeed-Studio-USB-CAN-Analyzer-Driver

Seeed Studio USB-CAN Analyzer 向けの ROS 2（Kilted 想定）ドライバです。  
ROS2依存のノード層と、ROS2非依存のシリアル/プロトコル層を分離しています。

## 構成

- `src/usb_can_analyzer_node.cpp`:
  - ROS2ノード実装
  - サブスクライバー`/ssuca/transmit` と パブリッシャー`/ssuca/receive` の実装
- `src/serial_protocol.cpp`
- `include/seeed_usb_can_analyzer_driver/serial_protocol.hpp`:
  - ROS2非依存の USBシリアルI/O（Boost.Asio + POSIX termios/ioctl）
  - USB-CANプロトコルのフレーム生成/解析
- `msg/CanFrame.msg`:
  - 独自メッセージ定義

## 独自メッセージ `CanFrame`

`seeed_usb_can_analyzer_driver/msg/CanFrame` は、CAN 1フレームをROS2トピックで受け渡しするためのメッセージです。

- `id` (`uint32`): CAN ID（標準/拡張どちらも格納）
- `extended` (`bool`): `false`=標準ID(11bit), `true`=拡張ID(29bit)
- `remote` (`bool`): `false`=データフレーム, `true`=リモートフレーム
- `dlc` (`uint8`): データ長コード（0〜8）
- `data` (`uint8[]`): データペイロード（0〜8バイト）

このドライバでは `/ssuca/transmit` 受信時に `dlc` と `data` 長が一致しないフレームを送信しません。

## ビルド

```bash
colcon build --packages-select seeed_usb_can_analyzer_driver
source install/setup.bash
```

## 実行

### デフォルト値で実行

```bash
ros2 run seeed_usb_can_analyzer_driver usb_can_analyzer_node
```

### USB path と bitrate を指定して実行

```bash
ros2 run seeed_usb_can_analyzer_driver usb_can_analyzer_node --ros-args \
  -p usb_path:=/dev/ttyUSB0 \
  -p bitrate:=500000 \
  -p operation_mode:=0
```

### launch ファイルで実行

```bash
ros2 launch seeed_usb_can_analyzer_driver usb_can_analyzer.launch.py
```

既定のパラメータは `config/usb_can_analyzer.yaml` にあります。別の YAML を使う場合:

```bash
ros2 launch seeed_usb_can_analyzer_driver usb_can_analyzer.launch.py \
  config_file:=/path/to/usb_can_analyzer.yaml
```

## パラメータ

- `usb_path` (string): USBシリアルデバイスパス（例: `/dev/ttyUSB0`）
- `bitrate` (int): CAN bitrate
  - 有効値: `1000000, 800000, 500000, 400000, 250000, 200000, 125000, 100000, 50000, 20000, 10000, 5000`
- `serial_baud` (int): USBシリアル通信速度（既定: `2000000`）
- `tx_extended` (bool): 送信IDフォーマット（false=STD, true=EXT）
- `filter_id` (int): 受信フィルタID
- `mask_id` (int): 受信マスクID
- `operation_mode` (int): CANコントローラ動作モード
  - `0=Normal`, `1=Loopback`, `2=Silent`, `3=Loopback+Silent`

## Topic I/O

- Subscribe: `/ssuca/transmit` (`seeed_usb_can_analyzer_driver/msg/CanFrame`)
- Publish: `/ssuca/receive` (`seeed_usb_can_analyzer_driver/msg/CanFrame`)

### 送信例

```bash
ros2 topic pub /ssuca/transmit seeed_usb_can_analyzer_driver/msg/CanFrame \
  '{id: 291, extended: false, remote: false, dlc: 2, data: [1, 2]}'
```

### 受信確認例

```bash
ros2 topic echo /ssuca/receive
```

## 参考文献

- https://copperhilltech.com/blog/usbcan-analyzer-usb-to-can-bus-serial-protocol-definition/
