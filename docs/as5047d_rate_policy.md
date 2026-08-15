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
