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

飛行中に診断目的でpipelined readを周期停止しない方針は変更しない。

## 4. 旧ログ互換

V5 wire layout、`EncoderRate::Hz5000`の数値、`kMaximumEncoderSamplesPerEpoch=5`は過去ログのdecode互換を壊さないため直ちには削除しない。

これは5 kHz acquisitionを現在サポートする意味ではない。新規実機取得の許可条件と旧wire valueの認識を分離して扱う。
