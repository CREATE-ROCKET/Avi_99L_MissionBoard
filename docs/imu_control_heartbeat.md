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

### Production

`avi_99l_missionboard` environmentでbuildし、起動後にICM FIFO/INTが正常な状態でMissionRealtimeTaskが約1 kHzで継続することを実機計測する。

TODO(HW_TEST): GPIO15 INT発生からMissionRealtimeTask再開までのlatency分布、99.9 percentile、最大値を取得する。

TODO(HW_TEST): heartbeat timeout時にmotorが危険な継続commandを保持せず、既存のIMU stale/recovery pathへ遷移することをfault injectionで確認する。

TODO(SIMULATION): 1000 Hz ICM sample timestampと1 kHz control updateの位相差をSpicaへ反映し、旧tick駆動との差を比較する。

2 kHz ICMは第五版審査書の1000 Hz記載と異なるため、審査書・Vault仕様の変更、filter/検出器/姿勢推定の再評価、実機検証なしにproductionへ有効化しない。
