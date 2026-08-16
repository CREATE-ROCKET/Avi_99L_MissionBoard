# 99L FV → FH+ → FH− → M0 motor system identification campaign

このfirmwareは、Spicaへ渡すmotor system identification用の実機data取得、V5 integrity検証、package作成までを担当します。新規AS5047D acquisitionは**1000 Hzだけ**です。

2 kHz / 5 kHz / 10 kHzは新規campaignで使用しません。V5の旧rate値は過去log decode互換のため残します。

motor profileは起動時に自動実行されません。`char arm`と`char run`はoperatorが明示実行します。

## 1. 極性とmechanical safety

実機ZeroHoldで確認済みの物理極性は次です。

```text
IN1駆動 -> AS5047D積算角が増加
IN2駆動 -> AS5047D積算角が減少
```

V5 wire規約は後方互換のため次を維持します。

```text
command > 0  -> DriveIn2
command < 0  -> DriveIn1
command == 0 -> Coast または Brake
```

characterizationでは`AVI_99L_CHARACTERIZATION_COMMAND_TO_FIN_SIGN=-1`を使用します。

mechanical contract:

- total reduction: `176.175:1`
- physical stop: `±15 deg`
- routine envelope: `±8 deg`
- hard abort: `±10 deg`
- backlash full width: `0.344 deg`

通常profileで物理stopperへ接触させません。

## 2. Build

```sh
git submodule update --init --recursive
git status --short
git submodule status

pio test -e native
python3 tools/capture_characterization.py --self-test
python3 tools/verify_characterization.py --self-test
python3 tools/package_spica_characterization.py --self-test

AVI_99L_CHARACTERIZATION_HARDWARE_APPROVED=0 \
  pio run -e avi_99l_missionboard_characterization
```

正式motor runでは実機構成、30 kHz PWM、配線、回転方向を確認した後だけ`AVI_99L_CHARACTERIZATION_HARDWARE_APPROVED=1`を使用します。

## 3. 1 kHz acquisition

AS5047Dは各1 ms epochの中央、500 usで1回triggerします。raw sampleはscheduled timeではなく**actual capture timestamp**で半開区間へ割り当てます。

```text
epoch n = [epoch_zero_us + n*1000,
           epoch_zero_us + (n+1)*1000)
```

各epochの期待sample数は1です。future sampleで欠落を穴埋めしません。

次は個別に計数し、raw dataをrepairしません。

- repeated
- skipped
- invalid
- startup incomplete
- steady-state incomplete
- raw/writer queue overflow
- encoder transport/status error

## 4. Motor-ID用deadline契約

motor system identificationとflight realtime qualificationを分離します。

**100 us command apply deadlineはmotor-IDでもfatalです。** 必要なDrive/Brake変更が100 usを超えた場合は遅れた指令を出さずCoastへ倒し、runを停止します。

consumer releaseがepoch終端から100 usを超えても、notificationがexactly 1で、releaseが1 epoch未満の遅延であり、raw sampleのepoch所属が壊れていない場合はmotor-ID診断として継続します。

release-only遅延は`consumer_lateness_us`、`CHAR_RATE_RESULT release-deadline`、`CHAR_RATE_TIMING`へ残します。新規motor-ID V5 recordではrelease-only遅延を`EpochDeadline`へ昇格しません。

次は引き続きfatalです。

- command apply deadline miss
- notification coalescing
- 1 epoch以上のconsumer release遅延
- encoder invalid / transport / status error
- steady-state incomplete / repeated / skipped
- raw/writer queue overflow
- Vbus invalid/stale/future timestamp
- position guard / hard abort / overshoot
- motor apply error
- SD write/sync/close error

この契約はflight controlの100 us realtime qualificationではありません。

## 5. Stage順序

FV、FH+、FH−は同じboot、同じsession ID、同じprofile seed、同じcommon zeroを使用します。FH+/FH−でzeroを取り直しません。

FV開始例:

```text
char new-session <session-id> <seed>
char confirm-stage FV ORIENTATION_CONFIRMED
char zero-capture common
char rate-check 1000 10
char arm <session-id>
char run full 1000
char complete-stage
```

FH+:

```text
char confirm-stage FH+ ORIENTATION_CONFIRMED
char rate-check 1000 10
char arm <session-id>
char run full 1000
char complete-stage
```

FH−:

```text
char confirm-stage FH- ORIENTATION_CONFIRMED
char rate-check 1000 10
char arm <session-id>
char run full 1000
char complete-stage
char prepare-m0
```

2 kHz / 5 kHzのrate-check/fullは実行しません。

## 6. M0 handoff

`char prepare-m0`後はsampling停止、queue drain、`fsync`、footer、close、handoff保存まで完了させ、完全に電源を切ります。

USB、logic battery、motor battery、外部電源をすべて外し、保持電荷が無くなった後にfinを取り外します。software resetやreset buttonはM0 cold handoffとして扱いません。

cold power-on後:

```text
char resume-m0 <session-id> FIN_REMOVED
char zero-capture m0
char rate-check 1000 10
char arm <session-id>
char run full 1000
char complete-stage
```

M0はfull-fin共通zeroを使わず、M0 local zeroを取得します。

## 7. Full profile

全stageで同じepisode順序とseedを使います。

| episode | 主な励振 |
|---|---|
| stationary baseline | Coast 1 s |
| zero approach + | `+1.0 -> ... -> 0.0 deg` |
| zero approach - | `-1.0 -> ... -> 0.0 deg` |
| polarity evidence | 正負短pulse |
| breakaway sweep | 10..30%を正負で評価 |
| sustained motion | 30% kick後に低dutyへ移行 |
| bounded pulse grid | ±10/15/20/25/30% |
| Coast | spin-up後のfree decay |
| Short Brake | spin-up後の短Brake |
| reversal | 正負反転 |
| bounded PRBS | ±25% |
| band-limited noise | ±25%以内 |
| chirp | ±25% |
| recenter | 0 degへ復帰 |
| final baseline | Coast 1 s |

zero approach/recenterでは±1.0 degから0.1 deg刻みで0へ接近し、overshoot、非単調接近、timeoutをabortします。

## 8. Storage

V5 binaryが正本です。

- header: 256 byte
- record: 320 byte / 1 ms
- footer: 192 byte
- writer queue: 512 records
- writer batch: 最大64 records
- run前にplanned fileをpreallocate
- 終了時にactual footer終端へtruncate

PRECHECK8.4の1000 Hzでは10秒・10000 recordを`writer-queue-overflow=0`、`queue-high-water=132/512`で保存できています。

## 9. Validation

旧V5 strict contractの確認には従来toolを残します。

```sh
python3 tools/verify_characterization.py --self-test
```

新しいmotor-ID captureはrelease-only遅延のflag省略を許す専用wrapperで検証します。

```sh
python3 tools/verify_motor_id_characterization.py capture.bin \
  --integrity integrity.json
```

このwrapperはrelease-onlyの`deadline flag tears`だけを追加で許し、CRC、sequence、timestamp、raw sample、command apply deadline、mode、Vbus、footer等の既存検査を再利用します。

## 10. Spica package

各stageで必要なbinaryは次だけです。

- 1000 Hz rate-check normal
- 1000 Hz full normal

packageは2000 Hz evidenceを要求しません。

```sh
python3 tools/package_spica_characterization.py captures \
  "99l_characterization_<session-id>" \
  --conditions conditions.json \
  --operator-label "<operator-label>" \
  --uart FV=FV_uart.log \
  --uart FH_positive=FH_positive_uart.log \
  --uart FH_negative=FH_negative_uart.log \
  --uart M0=M0_uart.log \
  --csv
```

binaryを正本とし、CSVは確認用です。
