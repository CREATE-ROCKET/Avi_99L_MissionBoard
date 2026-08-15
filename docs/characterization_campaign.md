# 99L FV → FH+ → FH− → M0 characterization campaign

このfirmwareは、Spicaへ渡す実機dataの取得、V5 integrity検証、package作成までを担当します。電流・motor torque・外力を測定せず、firmware自身では効率、摩擦torque、絶対慣性、制御器係数を推定しません。後段のSpica解析では審査書source値を含む既知parameterと本captureを組み合わせます。

motor profileは起動時に自動実行されません。`char arm`と`char run`はoperatorが明示実行します。

## 1. 実施前の停止条件と極性

2026-08-15時点で、搭載計器審査書の表記と現行Vault/実装にはencoder型式・PWM carrierのprovenance差が残っています。正式campaignでは、実機に搭載した部品と採用sourceをoperatorが確認してからhardware approvalを与えます。この確認をsource codeのdirty変更で表現しません。

実機ZeroHoldで確認済みの物理極性は次です。

```text
IN1駆動 -> AS5047D積算角が増加
IN2駆動 -> AS5047D積算角が減少
```

V5 wire上の抽象規約は後方互換のため次を維持します。

```text
command > 0  -> DriveIn2
command < 0  -> DriveIn1
command == 0 -> Coast または Brake
```

このためcharacterizationでは`AVI_99L_CHARACTERIZATION_COMMAND_TO_FIN_SIGN=-1`を固定します。これは「正のfin誤差を直すには負のV5 command、すなわちIN1を駆動する」というmappingです。V5の符号規約を物理GPIO極性へ読み替えてはいけません。

hardware approvalはbuild時環境変数で与えます。未指定時は0で、`char arm`を拒否します。

```sh
AVI_99L_CHARACTERIZATION_HARDWARE_APPROVED=0 \
  pio run -e avi_99l_missionboard_characterization
```

実機構成、30 kHz使用、配線、回転方向を人間が確認した正式runだけ、clean checkoutのまま次を使用します。

```sh
AVI_99L_CHARACTERIZATION_HARDWARE_APPROVED=1 \
  pio run -e avi_99l_missionboard_characterization
```

`tools/pio_characterization_provenance.py`はbuild時にMissionBoard HEADと`lib/Avi_ESP_Libs` HEADを取得してmacroへ埋め込みます。tracked fileがdirty、submoduleがdirty、またはsubmodule HEADがMissionBoardのgitlinkと一致しない場合はcharacterization buildを失敗させます。untracked capture fileはsource provenanceへ影響しないため許容します。

構造metadataは次を記録します。

- total reduction `176.175:1`
- physical limit `±15 deg`
- backlash full width `0.344 deg`
- backlash half width `0.172 deg`
- routine envelope `±8 deg`
- hard abort `±10 deg`

## 2. buildとhost検証

submoduleを固定revisionへ揃えます。

```sh
git submodule update --init --recursive

git status --short
git submodule status
```

非駆動build/test:

```sh
pio project config
pio run
pio run -e avi_99l_missionboard_bringup
pio test -e native

python3 tools/capture_characterization.py --self-test
python3 tools/verify_characterization.py --self-test
python3 tools/package_spica_characterization.py --self-test

AVI_99L_CHARACTERIZATION_HARDWARE_APPROVED=0 \
  pio run -e avi_99l_missionboard_characterization
```

uploadは対象portを人間が確認した後だけ実行します。最初はmotor電源を物理遮断し、safe output、AS5047D、SD、console、1/2/5 kHz rate-checkを確認します。

```sh
export MISSION_PORT=/dev/ttyACM0
AVI_99L_CHARACTERIZATION_HARDWARE_APPROVED=0 \
  pio run -e avi_99l_missionboard_characterization -t upload \
  --upload-port "$MISSION_PORT"
```

正式profile用firmwareはhardware確認後にapproval=1でbuild/uploadします。

## 3. logger atomicity

過去の2 kHz fullでは、`command=0`と`DriveIn2`が同一recordへ現れたstrict rejectがありました。V5ではmotor stateをwriter taskが個別に読みません。

1 kHz realtime ownerがcommand適用後に、次を一つの`ImmutableCommandEvidence`へ値copyします。

```text
command_generation
requested_command_permille
requested_motor_mode
applied_command_permille
applied_motor_mode
apply_result_code
command_apply_timestamp_us
logger_snapshot_timestamp_us
```

writer taskは完成済み`ImmutableLogRecord`のみをqueueから受け取り、live motor/profile stateへアクセスしません。同じ`command_generation`のrecordでcommand/mode/apply timestampが変化した場合、V5 strict validatorはrejectします。

`command=0 / DriveIn1|DriveIn2`、符号とmodeの不一致もrepairせずrejectします。

## 4. AS5047D取得と固定1 ms epoch

AS5047D producerは1/2/5 kHzで動作し、1 kHz consumerとは分離します。raw sampleは**capture timestamp**だけで次の半開区間へ割り当てます。

```text
epoch n = [epoch_zero_us + n*1000,
           epoch_zero_us + (n+1)*1000)
```

期待sample数は1/2/5 kHzで1/2/5です。

- sample欠落時はそのepochを`incomplete`とする
- 次epochのsampleで穴埋めしない
- `repeated`、`skipped`、`invalid`を別計数する
- raw capture timestampとepoch release timestampを分離する
- consumer deadline missを別計数する
- epoch 0のstartup incompleteとReady後のsteady-state incompleteを分離する
- 5 kHzで4 sampleしか無いepochへ次windowの1 sampleを借りない

rate-checkで2/5 kHzが`unsupported`になった場合、値を補正せず理由とcounterを保存します。1000 Hzは全stageで必須です。

## 5. operator commandとstage順序

```text
char new-session <session-id> <seed>
char status
char confirm-stage FV ORIENTATION_CONFIRMED
char zero-capture common
char rate-check 1000 10
char rate-check 2000 10
char rate-check 5000 10
char arm <session-id>
char run full 1000
char stop
char complete-stage
```

FV、FH+、FH−は同じboot、同じsession ID、同じprofile seed、同じcommon encoder zeroで続けます。FH+/FH−でzeroを取り直しません。

各stageでfixture姿勢、固定、配線、mechanical envelopeを人間が確認した後だけ、次を入力します。

```text
char confirm-stage <FV|FH+|FH-> ORIENTATION_CONFIRMED
```

2/5 kHz fullは、そのstageのrate-checkが`accepted`の場合だけ実行します。1000 Hz fullは必須です。

## 6. full profile

全stageで同じepisode順序とseedを使います。30 kHzで過去に得たmotor-only thresholdを跨げるよう、10--30% dutyを使用します。characterization buildのdriver上限は35%ですが、現在のprofileが自動要求する最大値は30%です。production flight buildの暫定15%上限は変更しません。

| episode | 主な励振 |
|---|---|
| stationary baseline | Coast 1 s |
| zero approach + | `+1.0 -> +0.9 -> ... -> 0.0 deg` |
| zero approach - | `-1.0 -> -0.9 -> ... -> 0.0 deg` |
| polarity evidence | ±20%, 各100 ms |
| breakaway | ±10, 12.5, 15, 17.5, 20, 22.5, 25, 27.5, 30%; 各80 ms |
| sustained motion | 30% kick後、30%から10%へ段階的に低下、正負別 |
| bounded pulse grid | ±10/15/20/25/30%, 各60 ms、repeat |
| Coast | ±30% 100 ms spin-up後、約1 s Coast |
| Short Brake | ±30% 100 ms spin-up後、250 ms Brake |
| reversal | +25%→-25%、-25%→+25% |
| bounded PRBS | ±25%, 20 ms更新 |
| band-limited noise | ±25%以内、50 ms更新 |
| chirp | ±25% |
| recenter | position feedbackで0 degへ戻す |
| final baseline | Coast 1 s |

zero approach/recenterは動作済みZeroHoldを基礎にした位置feedbackです。30 kHz用の同定値として20 kHz ZeroHold gainを採用するのではなく、接近専用の保守的command生成にだけ利用します。

- Kp相当: `500 permille/deg`
- Kd相当: `25 permille/(deg/s)`
- active command: `160..300 permille`
- 目標近傍250 mdeg以内: 20 ms周期のうち8 msだけactive、残りCoast
- settle deadband: 20 mdeg
- 各branch timeout: 18 s

software guardは通常`±8 deg`、hard abort `±10 deg`です。物理stopper `±15 deg`へ通常profileで意図的に接触させません。

## 7. abort

次のいずれかでretryせず即Coast/disarmし、そのrunを失敗として閉じます。

- `char stop`
- zero approach timeout、overshoot、非単調接近
- encoder invalid、transport/status error、unwrap不整合
- position guard違反
- Vbus invalid/stale/future timestamp
- consumer deadline miss
- raw/writer queue overflow
- command/mode/generation snapshot不整合
- motor apply error
- SD open/write/sync/close error

異音、衝突、過熱、緩み、配線異常を人間が認めた場合は`char stop`に加えて物理電源を遮断します。software stopを物理非常停止の代用にしません。

## 8. FH−からM0への移行

FH−完了後:

```text
char prepare-m0
```

firmwareはCoast、disarm、sampling停止、queue drain、`fsync`、footer、close、NVS handoff保存を行い、`power_cycle_required`へ遷移します。

その後、USB、logic battery、motor battery、外部電源をすべて外します。LED/UARTが消え、保持電荷が無くなった後にfinを取り外します。reset button、software reset、watchdog、panic、brownout、片側電源だけの切断はM0移行として認めません。

cold boot後:

```text
char resume-m0 <session-id> FIN_REMOVED
char zero-capture m0
char rate-check 1000 10
char rate-check 2000 10
char rate-check 5000 10
char arm <session-id>
char run full 1000
char complete-stage
```

`FIN_REMOVED`は人間の作業確認でありsensor判定ではありません。

## 9. V5 binaryとstrict validation

正本は`.bin`です。

- header: 256 byte
- 1 ms epoch record: 320 byte
- footer: 192 byte
- little-endian固定
- header/record/footer CRC-32
- footer file CRC
- truncation/trailing byte reject

```sh
python3 tools/verify_characterization.py capture.bin \
  --integrity integrity.json
```

validatorは値を補正しません。CRC、sequence、timestamp、fixed epoch、raw count、command generation/mode、Vbus、footer counterのいずれかが矛盾すれば非0終了します。

## 10. UART logとSpica package

```sh
python3 tools/capture_characterization.py "$MISSION_PORT" FV_uart.log \
  --command "char status" \
  --interactive
```

host toolは`arm`と`run`を自動送信しません。operatorがinteractive consoleから入力します。

全stage回収後:

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

正式packageへ入れるのはstrict validationを通ったcaptureだけです。CSVは確認用で、binaryを正本とします。

Spica側はV5をlossless importし、model fitとparameter採用は別工程で行います。

```matlab
r = runtests("tests/Test99LCharacterizationCaptureV5.m");
assertSuccess(r)
run_99l_characterization_import("99l_characterization_<session-id>")
```

## 11. 残るHW_TEST

profile実装後も次は実測で確認します。

- 30% profileを含む実機安全性と停止距離
- predicted stopping horizon/model
- fixture orientation tolerance
- 2/5 kHz steady-state qualification
- fin組付状態でのtemperature/creep

このrepository変更だけではboard upload、motor motion、SD実機I/O、2/5 kHz steady-state qualityを実証していません。正式campaignではrevision、port、UART log、生成binary、validator結果を保存します。
