# delta
delta robot controller

## Usage
オンラインティーチング
```
# python ./delta_teaching_online.py
```
オフラインティーチング
```
# python ./delta_teaching_offline.py
```
ダイレクトティーチング
```
# python ./delta_teaching_direct.py
```
オンラインリプレイ
```
# python ./delta_player_online <filename>
```
オフラインリプレイ
```
# python ./delta_player_offline <filename>
```

## Configuration
```python
config = {
    "arm_length": {
        "A":  55.0, #mm
        "B":  87.0, #mm
        "C": 171.0, #mm
        "D":  47.0  #mm
    },
    "move_min": [-40.0, -40.0, -200.0],     #[x, y, z]
    "move_max": [ 40.0,  40.0, -120.0],     #[x, y, z]
    "maxspeed": 60.0, #mm/sec
    "plot_range": {
        "x": [-200.0, 200.0], #[min, max]
        "y": [-200.0, 200.0], #[min, max]
        "z": [-300.0, 100.0]  #[min, max]
    },
    "interval": 100 , #msec
    # "tty": "/dev/tty.usbserial-A5052NCY"
    "tty": "COM4"
}
```

| パラメータ名        | 説明| 備考 |
| --------------- |:---------------|:----| 
| arm_length |ロボットのアームの長さ  | 変更不要 |
| move_min |ロボットの移動可能範囲  | 変更不要 |
| move_max |ロボットの移動可能範囲  | 変更不要 |
| maxspeed |ロボットの最大移動速度  | 60mm/secまで動作確認済み。速くしすぎると壊れるかも。 |
| plot_range |シミュレータ上の描画領域  | 変更不要 |
| interval |シミュレータの描画更新間隔及び、サーボモータへの位置情報の送信間隔  | 100msecまで確認済み。短くしすぎると動作しないかも。|
| tty |サーボモータとのシリアル接続ポート  | オフラインモードの場合は設定不要。WindowsはCOM*、Linux、MacOSは/dev/****。 |


