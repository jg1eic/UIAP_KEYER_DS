# KEYER DS - Powered by UIAPduino

KEYER DS は、UIAPduino / CH32V003F4P6 向けのアマチュア無線 CW キーヤー用ファームウェアです。

ハードウェアの組立方法や基本的な使い方は別ページで扱う前提とし、この README ではソフトウェア機能、内部構成、ポート割り当て、ビルド方法、変更履歴、ライセンス情報をまとめます。

## ソフトウェア機能

- CH32V003F4P6 + `ch32v003fun` 環境で動作
- Iambic パドル入力による CW キーイング
- ストレートキー入力対応
- ADC 入力による速度調整: 5 - 40 WPM
- サイドトーン出力
- KEYOUT 出力による無線機キーイング
- SSD1306 128x64 OLED 表示
- 4本のメモリメッセージ送信
- メモリメッセージの本体操作による編集
- Flash EEPROM エミュレーションによるメモリ保存
- 送信中メッセージと手動キーイングの CW デコード表示
- 起動時システムメッセージ `OK` の送信
- システムメッセージ送信中は KEYOUT を出さない保護動作
- メモリ再生中のパドル、ストレートキー、スイッチ操作による中断
- 無操作時のスタンバイ移行と、スイッチ / パドル / ストレートキー入力による復帰

## 動作モード

| モード | 内容 |
| --- | --- |
| `KEYER` | 通常の手動キーイング、WPM 表示、CW デコード表示 |
| `PLAY` | メモリメッセージまたはシステムメッセージの自動送信 |
| `EDIT_SELECT` | 編集するメモリ番号の選択 |
| `EDIT` | メモリメッセージの文字編集 |
| `SETUP` | 将来拡張用。現時点では未実装 |

## 操作仕様

通常モードでは、SW1 - SW4 のクリックでメモリ 1 - 4 を再生します。再生中にいずれかのスイッチ、パドル、ストレートキーを操作すると再生を停止して `KEYER` モードへ戻ります。

SW1 + SW2 の同時操作で `EDIT_SELECT` に入り、SW1 - SW4 で編集対象のメモリを選択します。`EDIT_SELECT` 中に SW1 + SW2 を同時操作すると `KEYER` へ戻ります。

`EDIT` モードの割り当ては次の通りです。

| 操作 | 動作 |
| --- | --- |
| DOT パドル | 現在文字を前の文字へ変更。長押しでリピート |
| DASH パドル | 現在文字を次の文字へ変更。長押しでリピート |
| SW1 クリック | 現在文字を前の文字へ変更 |
| SW2 クリック | 現在文字を次の文字へ変更 |
| SW3 クリック | カーソルを左へ移動 |
| SW4 クリック | カーソルを右へ移動 |
| SW1 長押し | カーソル位置以降を削除 |
| SW4 長押し | Flash に保存して編集終了 |

編集可能文字は次のテーブルです。

```text
 ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789/?.=+-@
```

## ソフトウェア構成

| ファイル / ディレクトリ | 内容 |
| --- | --- |
| `src/main.cpp` | キーヤー本体。状態管理、CW 生成、CW デコード、OLED 表示、メモリ編集、Flash 保存、スタンバイ制御 |
| `src/keyer_hal.h` | GPIO ピン割り当てと HAL 関数プロトタイプ |
| `src/keyer_hal.cpp` | GPIO、TIM1、TIM2 PWM 関連の初期化 |
| `src/funconfig.h` | `ch32v003fun` 向け設定 |
| `src/ch32v003_GPIO_branchless.h` | GPIO ヘルパ |
| `lib/ssd1306` | SSD1306 OLED 表示ライブラリ |
| `lib/flash_eep` | Flash EEPROM エミュレーション |
| `make_firmware_bin.py` | PlatformIO ビルド後に `.bin` を生成する post action |
| `tools/flash.bat` | `tools/minichlink.exe` で `firmware.bin` を書き込む補助バッチ |

主な定数は `src/main.cpp` に集約されています。

| 定数 | 値 | 内容 |
| --- | ---: | --- |
| `WPM_MIN` | 5 | 最小速度 |
| `WPM_MAX` | 40 | 最大速度 |
| `MSG_NUM` | 4 | メモリ数 |
| `MSG_LEN` | 64 | 1メモリあたりのバッファ長 |
| `DISP_COLS` | 10 | 編集画面に表示する文字数 |
| `CW_BUF_SIZE` | 64 | CW デコード用リングバッファ |
| `MORSE_BUF_LEN` | 8 | 1文字分のモールス符号バッファ |
| `SW_SCAN_DIV` | 20 | スイッチスキャン周期。TIM1 256 us tick x 20 |
| `SW_PUSH_TH` | 5 | クリック判定しきい値 |
| `SW_PRESS_TH` | 127 | 長押し判定しきい値 |

デフォルトメッセージは `src/main.cpp` の `default_msgs` で定義されています。

```cpp
"CQ TEST JO1YGK"
"5NN 13M BK"
"TEST MESSAGE 3"
"TEST MESSAGE 4"
```

## ポート割り当て

ピン定義は `src/keyer_hal.h` にあります。

| 論理名 | MCU ピン | 方向 | 用途 |
| --- | --- | --- | --- |
| `PIN_DOT` | PC5 | 入力 Pull-up | DOT パドル |
| `PIN_DASH` | PC6 | 入力 Pull-up | DASH パドル |
| `PIN_ST` | PA1 | 入力 Pull-up | ストレートキー |
| `PIN_SPEED` | PA2 / AIN0 | アナログ入力 | WPM 調整 |
| `PIN_SW1` | PD0 | 入力 Pull-up | SW1 / メモリ1 |
| `PIN_SW2` | PC3 | 入力 Pull-up | SW2 / メモリ2 |
| `PIN_SW3` | PD2 | 入力 Pull-up | SW3 / メモリ3 |
| `PIN_SW4` | PC4 | 入力 Pull-up | SW4 / メモリ4 |
| `PIN_TONE` | PC7 | 出力 Push-pull | サイドトーン |
| `PIN_KEYOUT` | PC0 | 出力 Push-pull | 無線機キーイング |

OLED は `ssd1306_i2c` ライブラリで初期化されます。I2C の実ピンは使用している `ch32v003fun` / SSD1306 ライブラリ側の設定に従います。

## ビルド

PlatformIO を使用します。

```bash
platformio run
```

ターゲット環境は `platformio.ini` の `ch32v003f4p6_evt_r0` です。

```ini
platform = ch32v
framework = ch32v003fun
board = ch32v003f4p6_evt_r0
monitor_speed = 115200
build_flags = -lc -Os
extra_scripts = post:make_firmware_bin.py
```

ビルド後、PlatformIO のビルドディレクトリに `.bin` が生成されます。`tools/flash.bat` は `tools/firmware.bin` を `minichlink.exe` で書き込む簡易スクリプトです。必要に応じて生成された `.bin` を `tools/firmware.bin` として配置して使用します。

## 実装メモ

- CW 生成は half-dit 単位の状態機械で行います。
- 通常送信時は `PIN_TONE` と `PIN_KEYOUT` を同時に制御します。
- システムメッセージ送信時は `keyout_enabled = false` とし、サイドトーンのみ出力します。
- CW デコードはキー状態の ON/OFF 時間から DOT / DASH / 文字間 / 単語間イベントを作り、リングバッファ経由でメインループ側が表示します。
- メモリ再生中の自動送信内容はデコードバッファに入れず、再生文字を直接 OLED に表示します。
- Flash に保存されたメッセージは起動時に妥当性チェックされ、未保存または破損時はデフォルトメッセージを使用します。
- 無操作が約10秒続くとスタンバイへ移行します。復帰時はシステムリセットで再初期化します。

## 変更履歴

| 日付 | バージョン / コミット | 内容 |
| --- | --- | --- |
| 2026-05-24 | V0.1 | 電波文化祭6向け初回リリース |

## ライセンス

このリポジトリで追加・変更したコードは MIT License とします。詳細は `LICENSE` を参照してください。

ベースソフトウェアおよび同梱・利用ライブラリには、それぞれの作者のライセンス条件が適用されます。本ファームウェアは `ch32v003fun`、SSD1306 表示ライブラリ、Flash EEPROM エミュレーションコードを利用しているため、公開、配布、改変版の再配布を行う場合は各ライブラリのライセンス条件も確認してください。
