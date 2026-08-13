# C-99L Mission Board bring-up結果

## 1. 現在の識別情報

- 最新検証日時: 2026-08-13
- 今回変更のbase Mission Board HEAD: `442adb8a74451de71eeb91e9d211ab15a5811d7c`
- branch: `main`
- Avi_ESP_Libs branch: `refactor`
- Avi_ESP_Libs作業開始時HEAD: `9bd7a365057b3f532f25166a2620b45c20df0783`
- Avi_ESP_Libs検証済みHEAD: `43a185ab2a570d4e1b889a4a534984fd19b2194f`
- submodule状態: clean。親repositoryのgitlinkを上記検証済みrevisionへ固定
- serial port: `/dev/ttyACM0`（serial `44:1B:F6:D1:DC:A8`）
- MCU確認: PlatformIO付属esptoolでESP32-S3 revision 0.2を確認
- 実際のESP-IDF: 6.0.1。build log、build metadata、`esp_idf_version.h`で確認
- local TWAI source: `/home/hotaru/.platformio/packages/framework-espidf/components/esp_driver_twai/esp_twai_onchip.c`、SHA-256 `40a53f7a2fc14a6d1045bd9e209239a2cc9d2e13056f9348b442e1ca174297dd`
- upstream fix: `_node_flush_pended_set_bits`相当なし。issue [#18803](https://github.com/espressif/esp-idf/issues/18803)の修正`6e0d480b2a630419456a04e3eb71d1a4062063ae`未適用
- commit: Avi_ESP_Libsを上記revisionへcommitし、親repositoryは本記録を含むcommitでgitlinkを固定

## 2. 現時点の要約

| test | status | 要約 |
|---|---|---|
| 作業前状態・docs確認 | PASS | 未commit変更を保持し、branch/HEADとrepository内docsを確認 |
| 変更前empty firmware build | FAIL | 旧環境名のstale sdkconfigとESP-IDF 6.0.1の組合せで`kconfgen`が`AttributeError` |
| Avi_ESP_Libs submodule | PASS | `refactor`、開始SHA `9bd7a365...`。ESP-IDF 6以上を既知safe release確認まで`CANCREATE::test()`で安全拒否し、library smoke buildとMission実機で確認 |
| production/bring-up/CAN診断 build | PASS | 実際のESP-IDF 6.0.1でlibrary guardと最終phase log追加後に3 environmentを再build |
| upload/boot | PASS | serial `44:1B:F6:D1:DC:A8`を再識別し、最新bring-upをflash/boot |
| safe output初期化 | PASS | boot先頭のGPIO38/39/40/44 LOW設定が`ESP_OK`、意図しないactuator動作なし |
| Flash/PSRAM runtime確認 | PASS | Flash 16 MiB、PSRAM 8 MiB initialized、QIO 80 MHzを確認 |
| custom partition boot確認 | PASS | NVS 64 KiB、PHY 4 KiB、factory 4 MiB、flightlog 2 MiBのtableを確認 |
| host capture parser self-test | PASS | CRC、分割frame、破損frame、sequence gap、encoder/IMU/ADC decode、CSV出力を確認 |
| ICM42688 | PASS (静置/通信) | self-test 10/10、1 kHz 5分で300,000 sample、欠落・FIFO fault・CRC error 0 |
| AS5047D hardware | PASS (最終再試験) | 先行試験ではall-zeroを検出したが、最終`encoder-test`はraw 6447、offset done、AGC 61、magnitude 4616、pipelineを含めPASS |
| AS5047D production startup gate | PASS | 正常statusでpipeline開始を確認。一時診断buildのall-zeroでは`ESP_ERR_INVALID_RESPONSE`、pipeline未開始、panic/resetなしを確認 |
| MissionRealtimeTask startup | PASS | 43,024-byte frameを6,144-byte stackへ置いたoverflowを修正。大型task専有storageをstatic化後のframeは2,352 byte、1 kHz loop開始前までの実機最小空き2,504 byte |
| CANCREATE begin/end lifecycle | PASS | TXなしの`can-lifecycle-test 100`を100/100完走、panicなし |
| raw ESP-IDF TWAI lifecycle | PASS | public APIだけの単発1回と続く20回反復を全て完走、panicなし |
| 修正前`CANCREATE::test()` | FAIL (PANIC) | 1回目で`xEventGroupSetBits`経由のassert/panic。backtraceはissue #18803と一致 |
| library fail-closed guard | NOT_SUPPORTED (意図どおり) | 実機`can-test`はnode再生成前に`ESP_ERR_NOT_SUPPORTED`、state/restored未評価、続くstatus/read/endは正常、panicなし |
| CAN peer round-trip/load | BLOCKED | 通信基板未接続。normal TX ACKと`can-load-test`は評価していない |
| I2CCREATE address probe | FAIL (intermittent) | 100 ms以上安定後も未使用LPS high addressで1/3,200 false ACK。原因未確定 |
| LPS25HB transport timing | MEASURED | config/read/end、400/400 read成功、最大transaction latency 710 us。値・実効ODRは未評価 |
| LPS25HB pressure/temperature | NOT EVALUATED | 現在個体の物理値妥当性を保証できないため、値決め・判定へ不使用 |
| SSCDRRN005PD2A5 sensor値 | SKIP | 未接続。Unavailable条件だけ確認 |
| STSCREATE/STS3215 | PASS (read-only) | persistent UARTでprobe/read、50 sample、torque OFF、Para電源OFFを確認。hold/small moveは未実施 |
| microSD | PASS (bring-up I/O) | 1 MiB write/read/CRC/unmountを6/6成功。production logger throughputは未評価 |
| flightlog Flash | PASS (bring-up sector) | 既知test sectorのwrite/read/CRC/reboot/eraseを確認。production appendは未評価 |
| ADC | PARTIAL | motor source 10.17 V相当、logic senseは0 V。bench基準との校正と閾値決定は未実施 |
| calibration | PARTIAL | 3秒×10回、IMU 10/10成功、SSC 0/10。stationary acceptance値は未決定 |
| motor/combined identification | BLOCKED | 今回の安全条件に従いmotorをarmせず、駆動試験を実施していない |

`PARTIAL`、`BLOCKED`、`MEASURED`、`NOT_SUPPORTED`、`NOT EVALUATED`、`PENDING`はPASSではありません。PANICはFAILです。LPS25HBのtransaction timing取得は、address probe、pressure/temperature、実効ODR、freshness、離床・頂点判定のPASSを意味しません。

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
- remaining TODO: 当時の未commit変更は後続の実機検証を経て、現在の検証済みsubmodule revisionへ固定済み

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
- 検証: library ESP-IDF smoke buildとMission Board foundation buildはPASS。通信基板未接続のためpeerを含む実CAN bus再試験はBLOCKED

### 影響を受けるESP-IDFで`CANCREATE::test()`がdriver panicを誘発する

- 症状: ESP-IDF 6.0.1で`CANCREATE::test()`を1回実行すると、self-test TX後のnode削除に続いてFreeRTOS timer taskが`xEventGroupSetBits`でassert/panicした
- 切り分け: CANCREATEのTXなしbegin/endを100回完走し、同じpin/configを使うraw public TWAI lifecycleも単発1回と20回反復を完走した。一方、panic stackとlocal driver sourceはEspressif issue #18803のdeferred event-group pointer use-after-freeに一致した
- 原因分類: 主因はupstream ESP-IDF TWAI driver bug。CANCREATEのbackend/callback lifetimeやdisable/delete/destroy順序に別のlibrary bugを示す証拠は得られなかった。Mission bring-up側の一律拒否では3層を比較できなかったため、専用unsafe環境とlayer別commandへ修正した
- library修正: public `CANCREATE::test()`自身をESP-IDF 6以上で`ESP_ERR_NOT_SUPPORTED`にするfail-closed guardを追加。既知safe releaseをsourceで確認するまで将来versionをsafe扱いしない
- API影響: signature、Config、Frame、Status、TestResult、通常の`begin`/`end`/`read`/`write`は変更なし。該当driverで危険な診断だけを安全拒否する
- 検証: guard追加後のlibrary ESP-IDF smoke build、Mission 3 environment build、実機`can-test`で`ESP_ERR_NOT_SUPPORTED`、panicなしを確認

## 5. 取得data

- ICM 60秒/5分、ADC 10秒、calibration 10回のraw/CSV/summary: `data/bringup/hwtest_20260813/`（Git管理外）
- STS、I2C、SD、Flash、AS5047D: serial consoleで観察
- CAN診断raw: `/tmp/avi_can_diag.tsEKwA/bringup_20260813T193506_868398.raw`（修正前lifecycle 100）、`bringup_20260813T193526_910188.raw`（raw TWAI単発）、`bringup_20260813T193547_806618.raw`（修正前CANCREATE panic）。guard後は`/tmp/avi_can_guard.i2pGEM/bringup_20260813T211106_901752.raw`（lifecycle 100）、`bringup_20260813T211122_851128.raw`（raw TWAI）、`bringup_20260813T211138_758945.raw`（CANCREATE NOT_SUPPORTED）。一時fileでGit管理外
- production boot: `/tmp/avi_can_guard.i2pGEM/production_boot_phases.log`に修正前TG1WDT、`/tmp/avi-production-gate2-tj5qflvh/production.log`に正常status、`/tmp/avi-production-zero-7ckc41rw/production_zero.log`に一時all-zero診断を記録
- host parser self-test: temporary directoryだけを使用し、終了時に削除
- motor実測data: 今回の安全条件に従いmotorをarmせず未取得。先行all-zeroの原因も未確定
- raw/CSV/summaryはGit管理外とし、このfileにsummaryだけを記録する

## 6. 実測から分かった値

| 項目 | 値 | 根拠 |
|---|---:|---|
| MCU | ESP32-S3 revision 0.2 | PlatformIO付属esptool |
| USB interface | USB Serial/JTAG | device enumeration/boot console |
| physical Flash | 16 MiB | runtime確認 |
| PSRAM | 8 MiB、initialized | runtime確認 |
| Flash mode/frequency | QIO、80 MHz | boot/build設定確認 |
| compile対象ESP-IDF | 6.0.1 | build log、build metadata、header |
| local TWAI #18803 fix | なし | `esp_twai_onchip.c`実ファイル確認 |
| CANCREATE TXなしlifecycle | 100/100 PASS | `can-lifecycle-test 100` |
| raw TWAI node lifecycle | 単発PASS、反復20/20 PASS | `can-idf-lifecycle-test`、panic 0 |
| Mission USB serial | `44:1B:F6:D1:DC:A8` | udev VID/PIDとserial |
| ICM FIFO read latency | 5分試験最大231 us（全試験最大259 us） | 300,000 sample |
| ICM 60秒gyro平均 X/Y/Z | 0.00038 / 0.11117 / 0.07467 dps | 静置capture。正式bias値ではない |
| ICM 60秒gyro標準偏差 X/Y/Z | 0.04838 / 0.04474 / 0.04490 dps | 標本標準偏差 |
| STS cold-start応答 | 約826 ms、100/100 ms設定で7回目 | 同一電源cycleの再PING |
| STS ready後PING / begin | 最大448 us / 約3.2 ms | read-only反復 |
| STS telemetry read | 最大907 us（別試行938 us） | 50 sample、error 0 |
| LPS I2C read latency | 最大710 us | 400 transaction。物理値・sample cadenceは未評価 |
| motor電源ADC | 平均10.174 V相当 | 10秒、1,000 sample。bench基準未比較 |
| logic電源ADC | 0 V相当 | 接続条件との矛盾があり未解決 |
| microSD throughput | write 0.550 MiB/s、read 1.760 MiB/s | 1 MiB bring-up file |
| Flash test block | 最大9,221 us | 4 KiB test sector |

LPS25HBのpressure/temperature読値は現在個体の妥当性を保証できないため、この表とparameter判断から除外しました。

## 7. 残っているTODO(SIMULATION)

- ICM連続欠落許容sample数をSpicaで決定する
- alpha/betaをSpica + 実encoder logで確定する

## 8. 残っているTODO(HW_TEST)

- 400 Hz AirData取得時のI2C operation timeout
- LPS25HBの物理値は正常性を確認した個体と基準圧力/温度で別途検証する。今回のtransport latencyから値を決めない
- STS3215 tx/response timeout、起動deadline、bring-up torque/speed/acceleration
- bring-up motor速度停止上限
- battery present threshold/debounce
- 初期calibration時間
- FlightMotorA/SpareMotorB実測profile
- 動翼・stopper組付後のfin software limit
- Flash logging rateとRealtimeへの影響
- SSCDRRN005PD2A5接続後の400 Hz通信、zero、sensor値
- encoder、CAN、STS motion、production storage、ADC校正、motorの各実機長時間試験

## 9. Mission実装へ進むblocker

- AS5047Dの最終再試験は正常だが、先行all-zeroの原因は未確定。再発時はmotor試験をfail-closedで停止する
- CANCREATE自己診断はESP-IDF 6以上を既知safe release確認まで禁止中。upstream修正版IDFとComBoard復旧後に再試験が必要
- LPS25HBの物理値は未検証で、pressure trend、離床・頂点判定の根拠にできない
- SSC未接続のため400 Hz AirData、zero、airspeed、Control gateを完了できない
- logic電源ADCが0の原因、battery threshold/debounce、production loggingのRealtime影響が未解決
- STS motion/open/stall/retryは安全監視・fixtureなしで実施していない
- motor polarity/profile、filter係数、欠落sample許容値、fin limitが未確定

## 10. 追加実測記録

### BR-005: STS3215 persistent UARTと起動待ち

- date/time: 2026-08-13
- Mission Board git HEAD: `5ff403309fb2eb1611d0f8e663993552f3d76941` + working tree変更
- Avi_ESP_Libs submodule HEAD: `9bd7a365`
- test name: `status`、`sts-probe`、`sts-read`、2 command間のUART再利用
- firmware build: env `avi_99l_missionboard_bringup`
- serial port: `/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_44:1B:F6:D1:DC:A8-if00`
- duration: 各commandでPara電源ONから最大1.5秒の起動待ち
- sample count: 100/100 ms設定のread 50 sample、5/10 ms設定のcold start 10 cycleとread 50 sample、追加のpower cycle反復
- error count: ready後のtelemetry error/timeout/fault/device error 0
- max latency: ready後PING 448 us、STS begin約3.2 ms、telemetry response 907 us（別試行938 us）
- observed behavior: Para電源ON後100 msでは応答せず、100/100 ms設定では電源を維持した7回目、約826 msで応答。5/10 ms設定でも1秒OFFを挟む10/10 cold cycleが24回目で成功し、read 50/50が成功した。timeout値そのものではなくservo起動時間が初期timeoutの原因。2 command目でもUART1を再beginせず、GPIO42再利用errorなし。worker stackをESP-IDFのbyte単位で16 KiBへ修正後、`sts-read`後の最小空きは8,196 byte
- result: PASS (read-only)
- cause: timeoutごとにPara電源をOFFにすると、servoの約0.8秒の起動期間を毎回先頭からやり直していた。加えて従来のworker stack指定は4,096 byteしかなく、telemetry bufferを持つ`sts-read`で不足していた
- library fix: なし
- remaining TODO: final設定は100/100 ms、1.5秒deadlineを維持する。温度、電源電圧、別個体を含むworst caseで短縮可否を判断する。`sts-hold`/`sts-small-move`、Open、stall/retryは安全確認後に別途実施する
- data files: serial consoleで確認。raw fileなし

### BR-006: ICM42688静置取得と初期calibration再現性

- date/time: 2026-08-13
- Mission Board git HEAD: `5ff403309fb2eb1611d0f8e663993552f3d76941` + working tree変更
- test name: `imu-selftest` 10回、`imu-stream 60`、`imu-static 300`、`calibration-repeat 10`
- sample count: IMU 60,000＋300,000、calibration 3,000 sample×10回
- error count: CRC/decode/sequence gap/FIFO lost/full/fault/timestamp nonmonotonic 0。calibration IMU error 0、SSC valid 0/10
- max latency: 5分試験231 us、全試験259 us
- observed behavior: self-test 10/10、60秒stream 60,000/60,000、5分static 300,000/300,000、calibration IMU 10/10。5分試験はfirmware/host双方で欠落・FIFO fault・timestamp非単調0。5分gyro平均X/Y/Z=-0.00383/0.10522/0.05128 dps、標準偏差=0.04416/0.04505/0.04401 dps。5分推定変化X/Y/Z=-0.00323/-0.00138/+0.00144 dps。正式bias値ではなく、calibration間のbias変動とstationary判定基準は未確定
- result: PASS (静置通信/FIFO)、PARTIAL (calibration parameter)
- remaining TODO: vibration/motor動作中のFIFO margin、既知姿勢でのbody axis、stationary acceptance、calibration時間を決定する。SSC zeroは未接続のため未評価
- data files: `data/bringup/hwtest_20260813/bringup_20260813T161215_659232_*`、`bringup_20260813T164409_968103_*`、`bringup_20260813T161750_390495_*`

### BR-007: I2C/LPS transport、SSC未接続、ADC

- date/time: 2026-08-13
- test name: AUX5V 100 ms以上安定後の`i2c-probe` 10 power cycle＋6同一power run、LPS transport read、`adc-stream 10`
- sample count: 未接続address probe 3,200、LPS read 400、ADC 1,000
- error count: 未使用LPS high address false ACK 1、LPS read error 0、ADC CRC/decode/sequence gap 0
- max latency: LPS I2C read 710 us、ADC処理419 us
- observed behavior: 通電直後に加え、100 ms以上安定後も未使用`0x5D`の100回中72回目に1度だけfalse ACKを観測し、続く1,000 probeでは再発しなかった。LPS low側のconfig/read/endは成功。motor sourceは平均10.174 V相当、logic senseは0 V相当
- result: FAIL (intermittent address probe)、MEASURED (LPS transaction timing only)、SKIP (SSC)、NOT EVALUATED (LPS physical values)、PARTIAL (ADC)
- limitation: 現在のLPS25HB個体はpressure/temperatureの妥当性を保証できない。物理値、実効ODR、sample更新、freshness、離床・頂点判定の判断には使用しない
- remaining TODO: logic analyzerでfalse ACK時のSDA/SCLと実ACKを確認し、IC/board/driverを切り分ける。SSC接続後の400 Hz timing/zero、正常性を確認したLPS個体と基準器による物理値試験、logic sense 0の原因、bench DMM/supplyによるADC校正と閾値決定
- data files: `data/bringup/hwtest_20260813/bringup_20260813T161638_245786_*`。I2C console rawなし

### BR-008: microSDとflightlog test sector

- date/time: 2026-08-13
- test name: `sd-test` 6回、`flash-test`、esptoolによる試験前後4 KiB確認
- sample count: microSD 1 MiB×6、Flash 4 KiB 1 sector
- error count: read/write/CRC/unmount error 0
- max latency: SD block 712,668 us、Flash operation 9,221 us
- observed behavior: SD CRC `D0F275EB`一致、write 0.550 MiB/s、read 1.760 MiB/s。flightlog先頭に既存bring-up magicを確認してから既知test sectorだけを試験し、reboot verification後に全`0xFF`へeraseされたことを確認
- result: PASS (bring-up I/O)
- remaining TODO: production append logger、50 Hz候補、1 kHz Realtime同時動作、queue overflow、耐久性は未評価。SDの大きなblocking時間はowner task分離を必須とする
- data files: serial console、試験後4 KiB readback。Git管理対象rawなし

### BR-009: AS5047D all-zero fail-closed

- date/time: 2026-08-13
- test name: `encoder-test`、共通health判定とproduction startup gate
- observed behavior: driver beginとangle readは`ESP_OK`だが、offset_done=0、AGC=0、magnitude=0、DIAG fault bitも全0、angle raw=0。MISO stuck-low等でもparityを通る全0応答を正常扱いしないようbring-upの全status gateを`ESP_ERR_INVALID_RESPONSE`にした
- implementation: `sensors::as5047d_health::{statusResponseAllZero,statusFaulted,validateStatus}`へ共通化し、encoder/motor bring-upとproduction `MissionRealtimeTask`から使用。productionはbegin成功後にstatusを検証し、成功時だけpipelineを開始する
- follow-up: 最終bring-up再試験ではraw 6447、offset_done=1、AGC=61、magnitude=4616、direct/pipeline readを含めPASS。all-zeroは再現しなかった
- production bring-up bug: `MissionRealtimeTask`の関数frameは43,024 byteで、ESP-IDFのbyte単位6,144-byte task stackを入口で破壊していた。38,408-byteのgyro historyと2,256-byteのIMU wrapperをtask専有static storageへ移し、frameを2,352 byteへ削減した。これはAS5047D startup gateへの実機到達を阻害したroot-cause bugの修正であり、一般refactorではない
- production validation: 正常statusではbegin/status/pipelineが全て`ESP_OK`。一時診断buildで取得済みstatusをall-zeroへ置換するとstatusは`ESP_ERR_INVALID_RESPONSE`、pipelineは`ESP_ERR_INVALID_STATE`となり、8秒間reset/panicなし。診断macro/environmentは検証後に削除した
- result: PASS (最終hardware再試験)、PASS (共通fail-closed実装/build)、PASS (production正常/all-zero経路実機確認)
- fail-safe: motor characterization前のstatus gateで停止し、coast/disarmを維持する。productionもstatus不良時はpipelineを開始せず、fin angle/rateをunavailableにする。raw angle 0単独は正常値として拒否しない
- scope: startupと明示reinitializeだけを保証する。1 kHz loopへ周期的なpipeline stop/DIAG/restartは追加していない
- remaining TODO: 先行all-zeroの再発条件を電源、CS/SCLK/MISO/MOSI、磁石、実波形で切り分ける。飛行中にMISO stuck-lowへ遷移した場合を既存read error以外で検出する方法を別途設計する
- data files: `/tmp/avi_can_guard.i2pGEM/`内の最終`encoder-test` raw、`/tmp/avi-production-gate2-tj5qflvh/production.log`、`/tmp/avi-production-zero-7ckc41rw/production_zero.log`。Git管理対象外

### BR-010: ESP-IDF 6.0.1 TWAI diagnostic lifecycle

- date/time: 2026-08-13
- test name: `can-lifecycle-test 100`、raw `can-idf-lifecycle-test`、修正前`can-test`、root cause確認
- firmware build: `avi_99l_missionboard_can_diag`、ESP-IDF 6.0.1
- serial port: `/dev/ttyACM0`
- sample count: CANCREATE TXなしlifecycle 100/100、raw TWAI単発1回＋反復20/20、修正前`CANCREATE::test()` 1回
- observed behavior: TXなしCANCREATE lifecycleは100回完走し、raw public TWAI lifecycleも全反復をpanicなしで完走した。修正前`CANCREATE::test()`は最初の1回で`assert failed: spinlock_acquire spinlock.h:142`となり、symbolicateしたstackは`xEventGroupSetBits`、`vEventGroupSetBitsCallback`、timer taskだった。local driverにはpending set-bits flushが無く、Espressif issue #18803と修正`6e0d480b2a630419456a04e3eb71d1a4062063ae`に一致した
- result: PASS (`can-lifecycle-test` 100/100、guard前後)、PASS (raw TWAI 20/20とguard後単発、panicなし)、FAIL/PANIC (guard前`CANCREATE::test()`)、NOT_SUPPORTED (guard後`CANCREATE::test()`、panicなし)、BLOCKED (peer round-trip/load)
- cause: upstream ESP-IDF TWAI node-delete use-after-freeが主因。CANCREATE backend lifetimeの別bugを示す証拠はなく、Mission bring-up側の問題は一律拒否でlibrary/raw/Missionを切り分けられなかったこと
- library fix: `CANCREATE::test()`はESP-IDF 6以上を既知safe release確認まで`ESP_ERR_NOT_SUPPORTED`でfail-closedにする。公開APIの破壊変更なし
- fail-safe: private API、delay、task priorityによる不確実な回避は行わない。guard後のNOT_SUPPORTEDはPASS扱いしない
- final safety: 通常bring-up firmwareへ再flashし、safe=yes、motor disarmed、IN1/IN2/AUX5V/PARAすべて0を確認
- remaining TODO: upstream修正を含むIDFへ更新後、peerを接続して100 Hz 60秒、bus-off/recoveryを再実施する。task lifetime中nodeを保持するproduction CAN ownerは今回のcreate/delete raceとは別lifecycle
- data files: guard前は`/tmp/avi_can_diag.tsEKwA/`、guard後は`/tmp/avi_can_guard.i2pGEM/`。Git管理対象外

### 追記template

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
- result: 判定label（PASS / FAIL / BLOCKED / NOT_SUPPORTED / PARTIAL / MEASURED / NOT EVALUATED / SKIP / PENDING）
- cause:
- library fix:
- remaining TODO:
- data files:
```
