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

### 4.1. 100 us deadlineの扱い

motor-IDとflight realtime qualificationを分離する。

100 us以内に必要なmotor command applyが完了しない場合は、motor-IDでも従来どおりfatalとする。遅れたDrive/Brakeを出力せず、Coastへ倒してrunを停止する。

一方、consumer releaseがepoch終端から100 usを超えても、次の全条件を満たす場合はmotor-IDのtimestamp診断としてrunを継続できる。

- notificationがexactly 1
- releaseが次epoch終端より前、すなわち1 epoch未満の遅延
- raw sampleのactual capture timestampが正しいepoch/slot内
- repeated/skipped/invalid/steady incompleteがない

このrelease-only遅延は`consumer_lateness_us`、`release-deadline`、`CHAR_RATE_TIMING`へ残すが、新規motor-ID recordでは`EpochDeadline`へ昇格しない。

notification coalescing、1 epoch以上のrelease遅延、command deadline miss、encoder error、raw/writer queue overflow、Vbus invalid、position guard、overshoot等は従来どおりfatalである。

**このmotor-IDでrelease-only遅延を許容することは、production MissionRealtimeTaskが100 us realtime requirementを満たしたことを意味しない。** production timing qualificationは別試験とする。

## 5. V5旧ログ互換

V5 schema、320 byte record、`EncoderRate::Hz2000` / `Hz5000`の数値、5 raw-sample slotは過去ログdecode互換のため残す。

旧V5 captureでrelease遅延に`EpochDeadline`が立っている形式もvalidatorは引き続き受理する。新しいmotor-ID captureではrelease-only遅延についてflagを省略し、数値timestampを証拠として保持する。

通常の`tools/verify_characterization.py`は旧strict契約を保持する。新しいmotor-ID packageでは`tools/verify_motor_id_characterization.py`を使用し、release-only遅延のflag省略だけを追加で許可する。late commandのflag/abort証拠は緩和しない。

## 6. Storage

writer queueは512 records、writer batchは最大64 recordsを維持する。run前にplanned file sizeを`ftruncate()`/`fsync()`で事前確保し、終了時にactual footer終端へtruncateする。

PRECHECK8.4では1000 Hzで`queue-high-water=132/512`、`records-written=10000`、`writer-queue-overflow=0`で完走しているため、motor-IDのstorage pathは1000 Hz用途で継続使用する。

## 7. Packaging

Spica引渡しpackageの新規必須captureは各stageについて次だけとする。

- 1000 Hz rate-check normal
- 1000 Hz full normal

FV / FH+ / FH- / M0の全stageで2000 Hz evidenceを要求しない。V5旧ログの2/5 kHz decode能力自体は削除しない。
