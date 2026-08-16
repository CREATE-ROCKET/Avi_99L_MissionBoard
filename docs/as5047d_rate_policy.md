# AS5047D取得レート制約

## 1. 優先順位

本書は2026-08-16時点の99L AS5047D取得レート決定であり、`docs/05_実装仕様.md`、`docs/characterization_campaign.md`、`docs/imu_control_heartbeat.md`等に残る旧取得レート記述と矛盾する場合は本書を優先する。

**新規のAS5047D acquisitionはcharacterization / productionとも1000 Hzだけとする。**

2000 Hz / 5000 Hz / 10000 Hzで新規実機captureを行わない。

## 2. 判断根拠

PRECHECK8.4では1000 Hzで10秒・10000 epochについて、次を確認した。

- `valid=10000/10000`
- `repeated=0`
- `skipped=0`
- `invalid=0`
- `steady-incomplete=0`
- `raw-queue-overflow=0`
- `writer-queue-overflow=0`
- `encoder-transport=0`
- `encoder-status=0`
- `command-deadline=0`
- `records-written=10000`

同じ試験の2000 Hzでは`repeated=3`、`skipped=3`、`steady-incomplete=6`が発生した。motor system identificationおよびproduction controlは1000 Hzで成立させられるため、2000 Hzの追加qualificationを継続しない。

## 3. Production

`MissionRealtimeTask`は1 ms周期で動作し、各周期に`EncoderBringup::readPipelined()`を1回だけ実行する。AS5047D angle、fin velocity estimator、controlへ渡すencoder updateは1000 Hzとする。

飛行中にdiagnostic目的でpipelined readを周期停止しない。詳細status/ERRFL確認はCommandReceiveまたはbring-upで行う。

## 4. Motor system identification

新規motor-ID campaignで許可するrate-check/full captureは1000 Hzだけとする。

Stage完了条件は次の2点だけを見る。

1. 1000 Hz motor-ID rate-checkがnormal
2. 1000 Hz full captureがnormal

2000 Hz / 5000 Hz slotはV5旧campaignとのdecode互換のため型として残してよいが、新規campaignでは最初からunsupported扱いとし、operatorへ実行を要求しない。

### 4.1. 100 us targetと1 ms hard deadline

motor-IDとflight realtime qualificationを分離する。

productionの100 us realtime requirementは変更しない。一方、motor-ID full captureでは100 usをcommand applyの診断targetとし、必要なcommand applyが100 usを超えても、次の全条件を満たす場合はrunを継続する。

- command applyが当該1 ms epoch内で完了する、すなわち`command_apply_timestamp_us < epoch_start_us + 1000 us`
- command generationの順序が維持される
- actual `command_apply_timestamp_us`が保存される
- notification coalescingやepoch順序の欠落がない
- encoder sampleのepoch/slot所属がactual capture timestampで確定できる

100 us超過を隠して100 us requirementを満たした扱いにしてはならない。offline解析では`command_apply_timestamp_us - epoch_start_us`を使ってtarget missを診断する。

command apply開始または完了が次epochへ達した場合、すなわち`>= epoch_start_us + 1000 us`では因果関係を同一epochへ保持できないためfatalとする。遅れたDrive/Brakeをまだ適用していない場合は出力せずCoastへ倒し、適用完了後に境界超過が判明した場合も直ちにCoastへ倒して`AbortReason::Deadline`でrunを停止する。

consumer releaseがepoch終端から100 usを超えても、次の全条件を満たす場合はmotor-IDのtimestamp診断としてrunを継続できる。

- notificationがexactly 1
- releaseが次epoch終端より前、すなわち1 epoch未満の遅延
- raw sampleのactual capture timestampが正しいepoch/slot内
- repeated/skipped/invalid/steady incompleteがない

このrelease-only遅延は`consumer_lateness_us`、`release-deadline`、`CHAR_RATE_TIMING`へ残すが、新規motor-ID recordでは`EpochDeadline`へ昇格しない。

notification coalescing、1 epoch以上のrelease遅延、1 epoch以上のcommand apply遅延、encoder error、raw/writer queue overflow、Vbus invalid、position guard、overshoot等は従来どおりfatalである。

**motor-IDで100 us超過を診断値として許容することは、production MissionRealtimeTaskが100 us realtime requirementを満たしたことを意味しない。** production timing qualificationは別試験とする。

### 4.2. TB67H standby復帰時間

`command_apply_timestamp_us`はMCUがTB67H入力へcommandを適用した時刻として扱い、TB67H内部でstandby解除後にOUTが有効になるまでの最大30 usは含めない。

長いCoast後のDriveではstandby復帰遅延が生じ得るため、motor-ID解析では`command_apply_timestamp_us`以後のplant-side latency uncertaintyとして扱う。最大30 usを固定値としてtimestampへ加算してはならない。

## 5. V5旧ログ互換

V5 schema、320 byte record、`EncoderRate::Hz2000` / `Hz5000`の数値、5 raw-sample slotは過去ログdecode互換のため残す。

旧V5 captureでrelease遅延に`EpochDeadline`が立っている形式もvalidatorは引き続き受理する。旧契約で100 us超過commandに`EpochDeadline`と`AbortReason::Deadline`が記録されているfull captureもdecode可能とする。

新しいmotor-ID full captureでは、100 us超過かつ1 ms未満のcommand applyについて`EpochDeadline`やabortを要求しない。actual `command_apply_timestamp_us`を証拠として保持する。1 ms以上のcommand applyでは`EpochDeadline`と`AbortReason::Deadline`を必須とする。

`tools/verify_characterization.py`をV5 timing契約の正本validatorとし、`tools/verify_motor_id_characterization.py`は同じvalidatorを呼び出す互換entry pointとする。schema、record size、CRC chainおよび既存golden fixtureのbyte互換性は変更しない。

## 6. Storage

writer queueは512 records、writer batchは最大64 recordsを維持する。run前にplanned file sizeを`ftruncate()`/`fsync()`で事前確保し、終了時にactual footer終端へtruncateする。

PRECHECK8.4では1000 Hzで`queue-high-water=132/512`、`records-written=10000`、`writer-queue-overflow=0`で完走しているため、motor-IDのstorage pathは1000 Hz用途で継続使用する。

## 7. Packaging

Spica引渡しpackageの新規必須captureは各stageについて次だけとする。

- 1000 Hz rate-check normal
- 1000 Hz full normal

FV / FH+ / FH- / M0の全stageで2000 Hz evidenceを要求しない。V5旧ログの2/5 kHz decode能力自体は削除しない。
