# ICM42688 INTによる1 kHz Mission heartbeat

## 1. 目的

MissionRealtimeTaskの1 kHz周期をFreeRTOS tickだけで作らず、ICM42688の実sample生成に同期させる。

第五版搭載計器審査書の表2ではICM42688 sampling rateが1000 Hzとされているため、productionのaccel/gyro ODRは1000 Hzのまま維持する。2 kHz化は本変更では行わない。

## 2. ICM42688設定

productionでは既存の設定を維持する。

- SPI: 8 MHz
- accel: ±16 g
- gyro: ±2000 deg/s
- accel ODR: 1000 Hz
- gyro ODR: 1000 Hz
- filter: `odr_div4`
- INT: GPIO15
- FIFO: enabled
- FIFO watermark: 1 record

Avi_ESP_LibsのICM42688 driverはFIFO threshold INTのGPIO ISRで固定counterを更新し、semaphoreを`FromISR`で通知する。ISR内でSPI、heap、logging、blocking処理は行わない。

## 3. MissionRealtimeTaskの周期

通常production environmentだけ、PlatformIO linker option `--wrap=xTaskDelayUntil`を使用する。

`MissionRealtimeTask`は同task内でFIFO付き`ImuBringup::begin()`を行うため、そのinstanceとtask handleがheartbeat sourceとして登録される。既存の

```cpp
vTaskDelayUntil(&wake, 1);
```

はFreeRTOS内部で`xTaskDelayUntil()`を呼ぶため、productionではwrapperへ入り、登録owner taskだけ次の処理になる。

```text
ICM42688 1000 Hz sample
        ↓
FIFO watermark = 1
        ↓
GPIO15 INT
        ↓
Avi_ESP_Libs GPIO ISR
        ↓
FIFO semaphore
        ↓
MissionRealtimeTask wake
        ↓
FIFO read / attitude / mission tick / control / motor apply
```

登録sourceが存在しないtaskでは従来の`xTaskDelayUntil()`をそのまま呼ぶ。そのためSafetyTask等の周期処理は変更しない。

## 4. 故障時

heartbeat waitは3 msを上限とする。

3 ms以内にFIFO thresholdが来ない、またはFIFO waitがerrorを返した場合、追加の1 ms delayを挟まずMissionRealtimeTaskを再開する。runtime本体が既に持つIMU stale、FIFO data loss、`end()`→`begin()` recoveryを直ちに評価する。

`ImuBringup::end()`でICMが終了した場合はheartbeat sourceを解除する。再初期化成功時に同じowner taskへ再登録する。初期化前や再初期化中はwrapperが通常のFreeRTOS周期へfallbackする。

## 5. Characterizationとの分離

AS5047D encoder-only rate-checkはICM42688へ依存させない。

characterizationではAS5047D raw sampleを1 ms epochの境界ではなく各slot中央へ配置する。

```text
1 kHz: 500 us
2 kHz: 250, 750 us
5 kHz: 100, 300, 500, 700, 900 us
```

Characterizationのsample clockとconsumer clockはESP-IDF 6.0.1のGPTimerを1 MHzで使用する。GPTimerを起動した後にraw counterを`esp_timer_get_time()`へ同期し、最初のalarm countをslot中心またはepoch終端の絶対時刻へ設定する。その後はhardware auto-reloadで周期を維持する。

この構成では、相対`esp_timer_start_periodic()`を呼び出せた時刻のscheduler jitterをsampling phaseへ含めない。ESP-IDF 6.0.1には`esp_timer_start_periodic_at()`が存在しないため、将来版のAPIを前提にせず6.0.1の公開GPTimer APIのみを使用する。

GPTimer ISRは固定lifetimeのtaskへ`vTaskNotifyGiveFromISR()`相当の通知を行うだけで、SPI、SD I/O、motor操作、heap allocationを行わない。AS5047D SPI readは従来どおりpriority 23のsampler taskで実施する。

これにより高priority encoder taskと1 kHz consumer releaseをepoch終端で同時起床させない。固定1 ms epoch、capture timestampによる半開区間割当、future sampleを借りない規則、startup/steady incompleteの分離は変更しない。

## 6. 検証

### Characterization

motor電源OFF、`hardware-approved=false`で1/2/5 kHz rate-checkを行い、少なくとも次を確認する。

```text
command-deadline=0
release-deadline=0
schedule-miss=0
steady-incomplete=0
trigger-coalesced=0
```

1000 Hzは必須。2/5 kHzは取得結果に基づきaccepted/unsupportedを判断する。

GPTimerと`esp_timer_get_time()`の同期に25 usを超えるoffsetが残る場合、capture開始前に`ESP_ERR_TIMEOUT`として失敗させる。測定中にtimestampを補正・捏造してdatasetを成立させることはしない。

### Production

`avi_99l_missionboard` environmentでbuildし、起動後にICM FIFO/INTが正常な状態でMissionRealtimeTaskが約1 kHzで継続することを実機計測する。

TODO(HW_TEST): GPIO15 INT発生からMissionRealtimeTask再開までのlatency分布、99.9 percentile、最大値を取得する。

TODO(HW_TEST): heartbeat timeout時にmotorが危険な継続commandを保持せず、既存のIMU stale/recovery pathへ遷移することをfault injectionで確認する。

TODO(SIMULATION): 1000 Hz ICM sample timestampと1 kHz control updateの位相差をSpicaへ反映し、旧tick駆動との差を比較する。

2 kHz ICMは第五版審査書の1000 Hz記載と異なるため、審査書・Vault仕様の変更、filter/検出器/姿勢推定の再評価、実機検証なしにproductionへ有効化しない。

## 7. Rate-check timing診断

rate-checkはmotorをarmせずCoastを維持するため、100 us consumer deadlineを1回超えただけでは診断runを打ち切らない。notificationが1回だけで、releaseが次epoch終端より前に収まっている場合は、そのepochへ`EpochDeadline`を残したままrunを継続する。最終判定では従来どおり1回でもdeadline missがあればunsupportedであり、正常扱いにはしない。

notification coalescing、1 epoch以上のrelease遅延、queue overflow、sensor transport/error、writer/power faultは固定epoch順序または証拠保全を壊すため継続対象にしない。full profileでは従来どおり最初のdeadline/incompleteを安全停止条件とする。

rate-check終了時は既存`CHAR_RATE_RESULT`とは別に`CHAR_RATE_TIMING`を出力する。V5 wire layoutは変更しない。

```text
consumer-alarm-late-max-us
consumer-isr-task-max-us
consumer-work-max-us
consumer-wait-late-max-us
release-first-us
release-max-us
release-steady-max-us
encoder-alarm-late-max-us
encoder-isr-task-max-us
encoder-capture-late-max-us
diagnostic-continued
```

`consumer-alarm-late-max-us`はGPTimer alarm時刻からISR callback実行まで、`consumer-isr-task-max-us`はISRから`char_runtime`再開まで、`consumer-work-max-us`は前epoch releaseから次のnotification待機へ入るまでの最大処理時間を表す。encoder側もalarm、task wake、実captureまでを分離して記録し、5 kHzのslot越境原因を切り分ける。

ESP-IDF 6.0.1のGPTimerはauto-reload時、alarm後にcounterをreload値へ戻した後の値をISRでcaptureする。そのため`AbsolutePeriodicTimer`は利用側callbackへ渡すeventを正規化し、`count_value - alarm_value`が1周期未満のalarm→ISR遅延を表すようにする。1周期以上ISRが遅れた場合は既存のnotification/schedule validationでqualification失敗とする。

consumerの1 ms処理時間を切り分けるため、rate-checkではさらに`CHAR_RATE_STAGE`を出力する。

```text
power-latest-max-us
encoder-drain-max-us
assembler-release-max-us
angle-convert-max-us
record-validate-max-us
writer-enqueue-max-us
encoder-read-max-us
```

`encoder-read-max-us`はpriority 23のencoder taskで`readPipelined()`に要した最大wall-timeであり、sampler停止後にconsumer側diagnosticへ転記する。その他は`char_runtime`単独所有のdiagnostic stateへ記録するため、ISR共有lockを追加しない。stage profilerはrate-check時だけ有効で、full profileのrealtime pathには追加計測を入れない。

`CHAR_RATE_STAGE`は診断用であり、`CHAR_RATE_RESULT`、V5 record/footer、100 us deadline、fixed epoch、capture timestampによるslot所属判定を変更しない。