MZ-1500 Emulator for Raspberry Pi Pico
---
[screenshot](/pictures/screenshot00.jpg)
SHARP MZ-700/1500 のエミュレータです。
以下の機能を実装しています。

- メイン RAM(64KB)
- VRAM
- PCG (MZ-1500)
- テープ
- QD(読み込みのみ)
- Beep
- PSG
- RAM ファイル (MZ-1R28)

速度は調整していませんが、単純にCPU の速度は実機より数割増し、描画は少し遅めなのでほぼトントンと思われます。

---
# 配線など

BML3 や FM-7 エミュレータと同じです

- GPIO0 VGA:H-SYNC
- GPIO1 VGA:V-SYNC
- GPIO2 VGA:Blue
- GPIO3 VGA:Red
- GPIO4 VGA:Green
- GPIO6 Audio

VGA、Audio の　GND に Pico の　GND を接続してください。

---
# キーボード

Pico の USB 端子に、OTG ケーブルなどを介して USB キーボードを接続します。
USB キーボードに存在しないキーは以下のように割り当てています。

- 英数 → Tab
- カナ → カタカナ・ひらがな
- GRAPH → ALT

また F12 でメニュー画面に移ります。
QD イメージや MZT イメージの操作ができます。

---
# VGA

いつもの VGA 出力ですが、データ量節約のため 320x400 ドットで描画しています。
PCG の変更があるたびに、キャラクタ単位で書き換えているので、グラフィック命令の速度が遅いです。

---
# テープ

いつもと違って、UART 入出力に対応していません。
LittleFS 上の MZT 形式のファイルをロード・セーブに用います。

---
# QD

QDF / MZT 形式のファイルの読み込みのみ対応しています。
littleFS 上においてください。

littleFS の作り方、イメージの書き込み方は、
https://mkusunoki.net/?p=8196 等を参考にしてください。
(SWD経由で書き込める装置が必要です)

---
# ROM など

いつものように純正ROM が必要です。
`mzipl` `mzext` `mzfont` にそれぞれ、IPL、拡張 ROM、フォントを入れてください。

なお MZ-700 として使えるように、互換 ROM & FONT を同梱しています。(拡張 ROM がないので、QD は使えません)

MZ-NEWMON
http://mzakd.cool.coocan.jp/mz-memories/mz700win.html##6

漢字 ROM は互換 FONT で動作確認していますが、辞書 ROM の動作は未テストです。

---
# 制限事項

- 最初に起動する際に LittleFS のフォーマットで固まることがあります。(リセットでOK)
- 辞書ROMの動作確認をしていません。

---
# ライセンスなど

このエミュレータは以下のライブラリを使用しています。

[libz80](https://github.com/ggambetta/libz80/tree/master)
[VGA ライブラリ(一部改変)](https://github.com/vha3/Hunter-Adams-RP2040-Demos/tree/master/VGA_Graphics)
[LittleFS](https://github.com/littlefs-project/littlefs)

---
# Gallary

[S-BASIC](/pictures/screenshot01.jpg)