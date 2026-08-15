# AS5047D取得レート制約

## 1. 優先順位

本書は2026-08-16時点の99L AS5047D取得レート決定であり、既存の`docs/05_実装仕様.md`、`docs/characterization_campaign.md`、`docs/imu_control_heartbeat.md`等に残る5 kHz / 10 kHz取得記述と矛盾する場合は本書を優先する。

AS5047Dは最大更新レート2.5 kHzであるため、99Lでは仕様上限ぎりぎりを使わず、新規取得を次の2レートだけに限定する。

- 1000 Hz
- 2000 Hz

5000 Hz / 10000 Hzで新規AS5047D captureを行わない。

## 2. Characterization

characterizationのrate-checkおよびfull captureで実際にAS5047D samplerを開始できるのは1000 Hz / 2000 Hzだけとする。

`EncoderRate::Hz5000`はV5の旧ログ・旧campaignとの互換用enumとして当面残すが、`EncoderSampler::begin()`は5000 Hzを`ESP_ERR_INVALID_ARG`で拒否し、SPI初期化やpipelined captureを開始しない。

Stage完了条件は1000 Hz / 2000 Hzのrate-check解決と1000 Hz full完了だけを見る。旧5 kHz campaign slotはStage完了条件に含めない。

1000 Hzは必須レートとする。2000 Hzはrate-check結果に基づいてaccepted / unsupportedを決定する。2000 Hzがunsupportedでも1000 Hz fullが正常であればcampaignは継続可能とする。

## 3. Production

productionのAS5047D capture候補は2000 Hzのみとし、1 kHz estimator/controlへ渡す。5 kHz captureをproduction候補から除外する。

飛行中にdiagnostic目的でpipelined readを周期停止しない方針は変更しない。

## 4. 旧ログ互換

V5 wire layout、`EncoderRate::Hz5000`の数値、`kMaximumEncoderSamplesPerEpoch=5`は過去ログのdecode互換を壊さないため直ちには削除しない。

これは5 kHz acquisitionを現在サポートする意味ではない。新規実機取得の許可条件と旧wire valueの認識を分離して扱う。

## 5. Realtime capture timing

PRECHECK7.5では1 kHz raw capture自体はcompleteだった一方、consumerが前epoch処理を終える前に次alarmへ到達してrelease deadlineを落としていた。このためconsumerのqueue drainは「空になるまでproducerを追う」方式を禁止し、drain開始時にSPSC ringのproducer headをsnapshotして、その時点で公開済みだったsampleだけを有限個処理する。snapshot後にproducerがpublishしたsampleは次consumer cycleへ残す。future sampleを現在epochへ借りることはなく、epoch所属は引き続きactual capture timestampで決める。

V5 recordのfull validationは`char_runtime`では実施せず、immutable recordをwriter queueへ値copyした後、`char_writer`がwire encode直前に`validateRecordStrict()`を1回だけ実施する。motor safety、position guard、deadline、queue overflow、encoder transport/status errorは従来どおりrealtime側で直接監視し、writer validation failureも既存failure notificationで安全停止へ伝播させる。native/offline validationは従来どおりstrictである。

既存`CHAR_RATE_STAGE`の`record-validate-max-us`はrealtime pathからvalidationを外した後は0を基本とする。strict validationはwriter側へ移動したため、このfieldをwriter CPU時間として再解釈しない。

### 5.1. Trigger phase

1 kHzは従来どおりepoch内500 usでtriggerする。

2 kHzはPRECHECK7.5でscheduled→actual capture最大347 usが観測され、250/750 us triggerでは500 us slot境界を越えた。このため2 kHzのprovisional triggerを120/620 usへ前倒しする。120 usは100 us command deadlineより後に置き、encoder taskがcommand apply deadline内を直接奪わないようにする。

この120/620 usは採用済みflight定数ではなく実機qualification用のprovisional phaseである。actual capture timestampが各half-open slot内へ入り、repeated/skipped/deadlineが0になることをrate-checkで確認できない場合、2 kHzはunsupportedのままとする。deadlineやslot境界を緩めて成立扱いにしてはならない。

### 5.2. Task affinityとwriter throughput

PRECHECK7.8では`char_runtime`をCore 0へ分離したことで、1 kHzの`consumer-work-max-us`は644 us、`consumer-wait-late-max-us`は0まで改善し、2 kHzでもraw epochはcompleteになった。一方、1000/2000 Hzとも約233 recordでwriter queueがoverflowした。

characterizationではtask affinityを次のように固定する。

- Core 0: `char_runtime` priority 21、`char_vbus` priority 12、`char_console` priority 5
- Core 1: `char_encoder` priority 23、`char_writer` priority 10

`char_encoder`は同Coreのwriterより常に優先し、AS5047D captureをSD書込みより優先する。writer queue depthを増やして持続throughput不足を隠すことはしない。

rate-check終了時は`CHAR_WRITER_TIMING`を出力する。

```text
queue-high-water
max-batch-records
validate-max-us
encode-max-us
fwrite-max-us
records-written
```

`queue-high-water`が継続的に増えて128へ到達する場合はwriterの持続処理速度不足とする。`fwrite-max-us`が支配的ならSD/FAT側、`validate-max-us`または`encode-max-us`が支配的ならCPU側として切り分ける。

## 6. Characterization build最適化

characterization firmwareはtiming qualification専用buildとして、ESP-IDF側の`CONFIG_COMPILER_OPTIMIZATION_PERF=y`を有効にし、PlatformIO compile flagで`-O3`を明示する。さらにESP-IDFのcompile-time/link-time LTOを有効にする。

`-Ofast`およびfast-math系optionは使用しない。`std::isfinite`、NaN/Inf、浮動小数点比較等の安全判定の意味論を維持するためである。

ESP-IDFの正式なperformance optimizationは`-O2`であり、`-O3`はcustom optimization levelである。このため最適化変更後のbinaryは別firmware SHAとして扱い、1/2 kHz rate-check、full 1 kHz、writer throughput、position guard、安全停止を再qualificationする。deadlineやacceptance基準は最適化のために緩和しない。
