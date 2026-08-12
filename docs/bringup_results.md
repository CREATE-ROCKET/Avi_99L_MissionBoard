# C-99L Mission Board bring-up結果

## 1. 現在の識別情報

- 記録日時: 2026-08-11T01:50:00+09:00
- 作業開始時Mission Board HEAD: `31cadbc`
- 現在のMission Board HEAD: `48d3ee3f7161b9bb44fcc3840108baed1af618f9`
- branch: `main`
- HEAD変化: 作業中に外部の`git pull`で`31cadbc`から`48d3ee3`へ進んだ。今回の作業によるcommitではない
- Avi_ESP_Libs branch: `refactor`
- Avi_ESP_Libs取得元HEAD: `0701132deb1d662732059a505001d0e238bc0d13`
- submodule状態: 上記SHAを基点とし、CAN診断ID対応の未commit変更あり
- serial port: `/dev/ttyACM0`。このpathは今回の観測結果であり、設定へ固定しない
- MCU確認: PlatformIO付属esptoolでESP32-S3 revision 0.2を確認
- commit: 実施していない

## 2. 現時点の要約

| test | status | 要約 |
|---|---|---|
| 作業前状態・docs確認 | PASS | 未commit変更を保持し、branch/HEADとrepository内docsを確認 |
| 変更前empty firmware build | FAIL | 旧環境名のstale sdkconfigとESP-IDF 6.0.1の組合せで`kconfgen`が`AttributeError` |
| Avi_ESP_Libs submodule取得 | PASS | `lib/Avi_ESP_Libs`、`refactor`、SHA `0701132...` |
| Avi_ESP_Libs ESP-IDF smoke build | PASS | CAN修正後を含めlibrary側build成功 |
| project-local board foundation build | PASS | ESP32-S3、16 MiB Flash、8 MiB PSRAM用envで`pio run`成功 |
| upload/boot | PASS | `/dev/ttyACM0`へuploadしboot完了 |
| safe output初期化 | PASS | boot先頭のGPIO38/39/40/44 LOW設定が`ESP_OK`、意図しないactuator動作なし |
| Flash/PSRAM runtime確認 | PASS | Flash 16 MiB、PSRAM 8 MiB initialized、QIO 80 MHzを確認 |
| custom partition boot確認 | PASS | NVS 64 KiB、PHY 4 KiB、factory 4 MiB、flightlog 2 MiBのtableを確認 |
| host capture parser self-test | PASS | CRC、分割frame、破損frame、sequence gap、encoder/IMU/ADC decode、CSV出力を確認 |
| SPICREATE/AS5047D/ICM42688 | PENDING | 実機API試験とdata captureは未実施 |
| CANCREATE | PENDING | 実機ACK/load/bus-off/recovery試験は未実施 |
| I2CCREATE no-device | PENDING | 100回の有限timeout/hang/resource確認は未実施 |
| LPS25HB/SSCDRRN005PD2A5 sensor値 | SKIP | 差圧系が未接続。接続後に再試験 |
| STSCREATE/STS3215 | PENDING | 意図しない動作、telemetry、free/hold/small moveは未実施 |
| microSD/flightlog Flash/ADC | PENDING | 実機read/write/calibration試験は未実施 |
| calibration | PENDING | gyro/gravityとrepeatabilityの取得は未実施 |
| motor/combined identification | PENDING | 安全上、前段driver検証完了後に実施 |

`PENDING`はPASSでもFAILでもありません。差圧sensor値以外を未接続扱いで`SKIP`にはしていません。

## 3. 実施記録

### BR-000: 変更前empty firmware build

- date/time: 2026-08-11、作業開始直後
- Mission Board git HEAD: `31cadbc`
- Avi_ESP_Libs submodule HEAD: 未追加
- test name: 変更前empty firmware `pio run`
- firmware build: 既存`4d_systems_esp32s3_gen4_r8n16` env
- serial port: 使用なし
- duration: 未記録
- sample count: 0
- error count: 1 build failure
- max latency: 対象外
- observed behavior: ESP-IDF 6.0.1のconfig生成中に`kconfgen`が`AttributeError`で停止
- result: FAIL
- cause: tracked `sdkconfig.4d_systems_esp32s3_gen4_r8n16`が旧ESP-IDFで生成されたstale設定で、新しいconfig modelと整合しなかった
- library fix: なし
- remaining TODO: 新しいproject-local envと`sdkconfig.defaults`から生成するbuildを基準にする

### BR-001: Avi_ESP_Libs取得とlibrary build

- date/time: 2026-08-11
- Mission Board git HEAD: `48d3ee3f7161b9bb44fcc3840108baed1af618f9`
- Avi_ESP_Libs submodule HEAD: `0701132deb1d662732059a505001d0e238bc0d13`
- test name: submodule状態確認、ESP-IDF smoke build
- firmware build: `lib/Avi_ESP_Libs/test_apps/platformio_espidf`
- serial port: 使用なし
- duration: 未記録
- sample count: 対象外
- error count: 最終build 0
- max latency: 対象外
- observed behavior: `refactor` branchの上記SHAを取得し、CAN変更後にlibrary test appがbuild成功
- result: PASS
- cause: 最初のCAN変更ではidentifier helperを`static_assert`から呼べない定義にしてbuildが失敗した
- library fix: helperを`constexpr`化。加えて次項の診断ID opt-inを実装
- remaining TODO: library実機test後に修正を正式commitへ固定する。今回はcommit禁止のためsubmoduleはdirty

### BR-002: project-local board build

- date/time: 2026-08-11T01:43:40+09:00
- Mission Board git HEAD: `48d3ee3f7161b9bb44fcc3840108baed1af618f9`
- Avi_ESP_Libs submodule HEAD: `0701132deb1d662732059a505001d0e238bc0d13` + 未commit変更
- test name: foundation `pio run`
- firmware build: env `avi_99l_missionboard`、artifact `.pio/build/avi_99l_missionboard/firmware.bin`
- serial port: 使用なし
- duration: 未記録
- sample count: 対象外
- error count: 0
- max latency: 対象外
- observed behavior: local board manifest、custom partition、ESP-IDF C++17 foundationがbuild成功。firmware.binは約188 KiB
- result: PASS
- cause: なし
- library fix: CAN診断ID修正を含む状態でlink成功
- remaining TODO: 全bring-up module統合後に再度`pio run`

### BR-003: chip確認、upload、boot安全状態

- date/time: 2026-08-11
- Mission Board git HEAD: `48d3ee3f7161b9bb44fcc3840108baed1af618f9`
- Avi_ESP_Libs submodule HEAD: `0701132deb1d662732059a505001d0e238bc0d13` + 未commit変更
- test name: esptool chip確認、foundation upload、runtime config、安全出力
- firmware build: BR-002 foundation build
- serial port: `/dev/ttyACM0`
- duration: boot観察のみ。連続試験ではない
- sample count: 対象外
- error count: 0
- max latency: 未測定
- observed behavior: ESP32-S3 revision 0.2、USB Serial/JTAGを確認。upload成功。Flash 16 MiB、PSRAM 8 MiB initialized、QIO 80 MHz、custom partitionを確認。`safe_outputs=ESP_OK`で、boot時にmotor/paraの意図しない動作なし
- result: PASS
- cause: なし
- library fix: なし
- remaining TODO: boot安全状態を各actuator test失敗後にも再確認する

### BR-004: host capture parser self-test

- date/time: 2026-08-11T01:50:00+09:00
- Mission Board git HEAD: `48d3ee3f7161b9bb44fcc3840108baed1af618f9`
- Avi_ESP_Libs submodule HEAD: `0701132deb1d662732059a505001d0e238bc0d13` + 未commit変更
- test name: `python3 tools/capture_bringup.py --self-test`
- firmware build: 使用なし
- serial port: 使用なし
- duration: 0.1秒未満
- sample count: valid frame 2、意図的な破損frame 1
- error count: 想定外error 0
- max latency: 対象外
- observed behavior: CRC known vector、分割input、magic再同期、CRC破損除外、uint32 wrapを跨ぐsequence gap、encoder/IMU/ADC decode、CSV file生成がassertを通過
- result: PASS
- cause: なし
- library fix: なし
- remaining TODO: 実機binary streamでencoder/IMU/ADC schema、throughput、gapを検証する。motor/calibrationはproducer確定後にschemaを追加する

## 4. Avi_ESP_Libsで発見した問題

### CAN bring-up専用IDを公開APIで送信できない

- 症状: libraryには診断用standard ID範囲`0x400..0x7FE`の概念がある一方、public `write`/`read` validationがMission protocol範囲だけを許可し、bring-up専用ID `0x7FE`を送受信できなかった
- 原因: 診断frameを明示的に許可するpublic Configが無く、共通identifier validationが診断範囲を拒否していた
- 修正: `CANCREATE::Config`へdefault `false`の`allow_diagnostic_frames`を追加し、opt-in時だけ`0x400..0x7FE`を許可。内部test予約ID `0x7FF`は引き続きapplicationから拒否
- API影響: additiveでdefault-off。既存利用側の動作は変わらない。破壊変更ではない
- 検証: library ESP-IDF smoke buildとMission Board foundation buildはPASS。実CAN busでの再試験はPENDING

## 5. 取得data

- firmware runtime確認: serial consoleで観察。保存raw fileなし
- host parser self-test: temporary directoryだけを使用し、終了時に削除
- sensor/motor実測data: 未取得
- 今後の保存先: `data/bringup/`。raw/CSV/summaryはGit管理外とし、このfileにsummaryだけを記録する

## 6. 実測から分かった値

| 項目 | 値 | 根拠 |
|---|---:|---|
| MCU | ESP32-S3 revision 0.2 | PlatformIO付属esptool |
| USB interface | USB Serial/JTAG | device enumeration/boot console |
| physical Flash | 16 MiB | runtime確認 |
| PSRAM | 8 MiB、initialized | runtime確認 |
| Flash mode/frequency | QIO、80 MHz | boot/build設定確認 |
| serial port | `/dev/ttyACM0` | 今回の接続時だけの観測値 |

encoder jitter、IMU bias/noise/drift、motor polarity/dead-zone/time response、ADC電圧、CAN/STS latency、SD速度はまだ実測していません。

## 7. 残っているTODO(SIMULATION)

- ICM連続欠落許容sample数をSpicaで決定する
- alpha/betaをSpica + 実encoder logで確定する

## 8. 残っているTODO(HW_TEST)

- 400 Hz AirData取得時のI2C operation timeout
- STS3215 tx/response timeout、para電源安定待ち、bring-up torque/speed/acceleration
- bring-up motor速度停止上限
- battery present threshold/debounce
- 初期calibration時間
- FlightMotorA/SpareMotorB実測profile
- 動翼・stopper組付後のfin software limit
- Flash logging rateとRealtimeへの影響
- LPS25HB/SSCDRRN005PD2A5接続後のsensor値
- encoder、IMU、CAN、STS、storage、ADC、motorの各実機長時間試験

## 9. Mission実装へ進むblocker

- SPICREATE、AS5047D、ICM42688、CANCREATE、I2CCREATE、STSCREATE/STS3215、storage、ADC、motorの要求された実機試験が未完了
- AS5047D 60秒1 kHz、ICM 60秒/5分、CAN 100 Hz 60秒、no-device I2C連続試験のdataが未取得
- actuator試験前のnon-actuator PASS条件を満たしていない
- CAN library修正が未commitで、検証済みsubmodule SHAとして固定できていない
- motor polarity/profile、filter係数、欠落sample許容値、fin limitが未確定

## 10. 追記template

各試験ごとに次を複製し、未測定値を空欄にせず`未測定`と記入します。

```text
### BR-XXX: test名
- date/time:
- Mission Board git HEAD:
- Avi_ESP_Libs submodule HEAD:
- test name:
- firmware build:
- serial port:
- duration:
- sample count:
- error count:
- max latency:
- observed behavior:
- result: PASS / FAIL / SKIP
- cause:
- library fix:
- remaining TODO:
- data files:
```
