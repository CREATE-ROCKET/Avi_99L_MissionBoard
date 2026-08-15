# 99L FV → FH+ → FH− → M0 characterization campaign

このfirmwareは、実機dataの取得、V5 integrity検証、Spica引渡しpackageの作成だけを行います。物理torque・電流・効率・絶対慣性の推定、model fit、制御器更新、本番parameter採用は行いません。motor profileは起動時に自動実行されません。

## 1. 実施前の停止条件

2026-08-15時点で、次のprovenance衝突は未解決です。

| 対象 | 現行code・実機記録 | 最新搭載計器審査書 | 扱い |
|---|---|---|---|
| encoder | AS5047D、SPI2、8 MHz | AS5048A | `UNRESOLVED_CONFLICT` |
| PWM carrier | 30 kHz | 20 kHz | `UNRESOLVED_CONFLICT` |
| 正commandの極性 | characterization契約はIN2 | production候補はIN1、未実測 | `TODO(HW_TEST)` |

審査書との対応が確認され、operatorが採用値を承認するまで、`char arm`と`char run full`を実行してはいけません。build、host test、console、safe output、静止encoder rate-check、SD open/sync/closeは非駆動確認として実施できます。

drive gateは既定で`AVI_99L_CHARACTERIZATION_HARDWARE_APPROVED=0`かつ`AVI_99L_CHARACTERIZATION_COMMAND_TO_FIN_SIGN=0`です。実機の低出力極性確認と上記衝突の承認後だけ、専用environmentでapprovalを`1`、正commandでfin角が増える場合はsignを`1`、減る場合は`-1`へ設定します。sign `0`のままではapprovalを変更してもarmを拒否します。

構造metadataは次のように記録します。

- total reduction `176.175:1`: 構造系審査書の歯数から算出した`DERIVED_EXACT`
- physical limit `±15 deg`: 構造系審査書の`SOURCE_FIXED`
- backlash full width `0.344 deg`: 構造系審査書の`MEASURED`
- backlash half width `0.172 deg`: full widthを2分した`DERIVED`
- routine envelope `±8 deg`、hard abort `±10 deg`: 今回の`TASK_POLICY`

## 2. buildとhost検証

```sh
pio project config
pio run -e avi_99l_missionboard_characterization
pio run
pio run -e avi_99l_missionboard_bringup
pio test -e native

python3 tools/capture_characterization.py --self-test
python3 tools/verify_characterization.py --self-test
python3 tools/package_spica_characterization.py --self-test
```

characterization environmentだけが`AVI_99L_CHARACTERIZATION=1`を定義し、専用`app_main`を選択します。production、bring-up、CAN診断へcharacterization consoleやprofileを混入させません。

uploadは対象boardとportを確認し、人間が明示許可した場合だけ行います。最初のbootではmotor電源を物理遮断し、safe output、build/Avi revision表示、AS5047D status、静止1/2/5 kHz rate-check、SD close、console parserだけを確認します。

```sh
pio run -e avi_99l_missionboard_characterization -t upload \
  --upload-port "$MISSION_PORT"
```

## 3. taskとownership

- sampling task: AS5047D pipelineの唯一owner。timer callbackはtask notificationだけを行い、SPI readを行いません。通知がcoalesceしても1回だけ取得し、catch-up readで架空sampleを作りません。
- 1 kHz realtime task: profile、requested/applied motor command、position guard、epoch release、immutable record完成の唯一ownerです。
- writer task: static queueから値コピー済み`ImmutableLogRecord`だけを読み、little-endianでserializeします。motor/profileのlive stateを読みません。
- console task: operator commandを受け、長時間run中も`char stop`を受理します。

raw sampleはcapture timestampにより、次の半開区間へ所属します。

```text
epoch n = [epoch_zero_us + n*1000, epoch_zero_us + (n+1)*1000)
```

期待slot数は1/2/5 kHzでそれぞれ1/2/5です。欠落slotを次epochから借りません。startup incomplete、steady-state incomplete、repeated、skipped、invalid、late、deadline missを別counterへ保存します。

## 4. operator command

各commandは`CHAR_OK <verb> ...`または`CHAR_ERROR <verb> ...`で完了します。

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

FV、FH+、FH−は同じboot、同じsession ID、同じprofile seedで続けます。各stageでfixture姿勢、固定、配線、mechanical envelopeを人間が実物確認した後だけ、`char confirm-stage <FV|FH+|FH-> ORIENTATION_CONFIRMED`を入力します。このliteralはoperatorの確認証跡であり、firmwareが姿勢をsensor検出した結果ではありません。mechanical zero確認、arm、full開始も人間が1回ずつ実行します。2/5 kHz fullはrate-checkがacceptedの場合だけ実行できます。1000 Hz fullは全stageで必須です。

FH−完了後は次を実行します。

```text
char prepare-m0
```

firmwareは、coast、disarm、sampling停止、realtime queue drain、writer queue drain、`fsync`、footer、close、NVS handoff保存の順で終了し、`power_cycle_required`へ遷移します。この状態では全motor commandを拒否します。

USB、logic battery、motor battery、外部電源をすべて外し、UARTとLEDが完全に消えて保持電荷がなくなるまで待ってからfinを取り外します。reset button、software reset、watchdog、panic、brownout、片側電源だけの切断はM0移行ではありません。

次のcold bootで、`ESP_RST_POWERON`、NVS schema/session/seed/checksum、RTC no-init cookie消失を確認後、人間が同じsession IDとliteralを入力します。

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

firmwareはfin取り外しをsensorで確認できません。`FIN_REMOVED`は人間の作業確認であり、自動検出結果ではありません。

## 5. profileとabort

全stageで同じseedとphase順序を使用し、episode IDを保存します。stationary baseline、正負zero approach、polarity check、breakaway sweep、sustained-motion sweep、bounded pulse grid、coast、short brake、両方向reversal、bounded PRBS、band-limited noise、chirp、recenterを含みます。

zero approachのtargetは次のとおりです。共通geometric zeroをbranchごとに取り直しません。

```text
FromPositive: +1.0, +0.9, ..., +0.1, 0.0 deg
FromNegative: -1.0, -0.9, ..., -0.1, 0.0 deg
```

次のいずれかで、retryせず即coast/disarmしてrunを失敗させます。

- `char stop`、timeout、overshoot、非単調approach
- encoder invalid、parity/framing/sensor status error、14-bit unwrap不整合
- `±8 deg`から外向きcommand、`±10 deg`到達、predicted stopping limit違反
- Vbus invalid、stale、future timestamp
- consumer deadline miss、raw/writer queue overflow
- motor apply error、command/mode mismatch、record validation error
- SD open/write/sync/close error

異音、衝突、過熱、緩み、配線異常、角度異常を人間が認めた場合は`char stop`と物理電源遮断を実行します。software stopを物理非常停止の代用にしません。

次は低出力実機試験で確定するまで`TODO(HW_TEST)`です。

- approach gain、settle angle/rate tolerance、dwell、timeout、maximum duty
- command-to-fin polarity
- predicted stopping horizon/model
- fixture orientation tolerance
- 2/5 kHz steady-state qualification threshold

## 6. V5 binaryとstrict validation

正本は`.bin`です。V5はheader 256 byte、1 ms epoch record 320 byte、footer 192 byteの固定長little-endian formatです。magicは`99LMCV5\0`、record markerは`EPV5`、footer magicは`99LEND5\0`です。C++ ABI layoutは保存しません。

header、各record、footerはCRC-32/ISO-HDLCを持ち、footerのfile CRCはheaderと全recordのserialized byte列を対象にします。footerの`total_records`からfile長を一意に決め、truncationとtrailing byteを拒否します。符号規約は全readerで共通です。

```text
command > 0  -> DriveIn2
command < 0  -> DriveIn1
command == 0 -> Coast または Brake
```

検証は値を補正しません。

```sh
python3 tools/verify_characterization.py capture.bin \
  --integrity integrity.json
```

CRC、size、enum、sequence、timestamp、fixed epoch、raw count、command generation/mode、Vbus、footer counterのいずれかが矛盾すれば非0終了します。rate-checkで正しく記録された`unsupported`はrepair対象ではなく、理由と分母を保持した構造的に有効な結果です。

## 7. UART取得とpackage

host capture toolは`arm`と`run`の自動送信を拒否します。これらは監視中の人間がconsoleから入力します。

```sh
python3 tools/capture_characterization.py "$MISSION_PORT" FV_uart.log \
  --command "char status" \
  --interactive
```

`--interactive`はstdinがTTYの場合だけ開始し、UART受信を画面とtimestamp付きlogの両方へ残します。operatorはこの画面で`char arm`と`char run`を入力します。Ctrl-C、stdin EOF、timeout、host errorのいずれでもhostはbest-effortで`char stop`を送りますが、物理非常停止の代用にはしません。

全stageのraw `.bin`、UART log、条件JSONを回収後、packageを作ります。

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

出力は次の形です。

```text
99l_characterization_<session-id>/
  manifest.json
  conditions.json
  integrity.json
  SHA256SUMS
  FV/{1000Hz,2000Hz,5000Hz}/*.bin
  FV/uart.log
  FH_positive/...
  FH_negative/...
  M0/...
  optional_csv/{epochs,raw_samples,events}.csv
```

`manifest.json`はpath、SHA-256、stage、rate、profile、start/end、firmware/Avi revision、operator labelを保持します。CSVを生成してもbinaryを削除しません。CSVは確認用であり、手編集してimportを通してはいけません。

strict validationに失敗したrawは入力側へそのまま残し、validatorが生成した`strict_pass=false`のreject integrityと一緒に保管します。不正captureは正式packageへ混ぜません。

## 8. Spica

Spicaではpackage directoryまたは`.bin`をlossless importし、acquisition integrityだけを判定します。

```matlab
r = runtests("tests/Test99LCharacterizationCaptureV5.m");
assertSuccess(r)
run_99l_characterization_import("99l_characterization_<session-id>")
```

V5 readerはV2/V4 branchを残し、unknown enum、CRC、sequence gap、command/mode tearing、truncation、trailing byteをfail-fastします。`events`はepochのphase、episode、abort reason変化からlosslessに派生します。model fitとparameter採用は別作業です。

## 9. 実機確認状態

この実装作業ではboard upload、静止capture、SD実機I/O、motor motionを実行していません。実機確認を行った場合は、両repository revision、binary schema、port、command、UART log、生成file、判定を別artifactとして保存してください。
