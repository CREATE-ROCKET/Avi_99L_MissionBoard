# C-99L Mission Board

ESP32-S3-WROOM-1-N16R8を搭載した99L Mission Board firmwareです。Vault 00〜07/04aに基づくMission FSM、CAN codec、command lifecycle、sensor continuity、制御pipeline、パラシュート安全coreをproduction moduleとして実装し、既存bring-up firmwareも明示選択できる形で維持しています。

既定production buildは、未確定値へ`TODO(HW_TEST)`または`TODO(SIMULATION)`を残した暫定設定で`StartSequence`を受理し、ZeroHold/Roll制御からTB67H450 PWMまでの動翼経路と、STS3215の現在位置Hold/Open/retry/電源遮断経路を実行します。これはbench/HILで一連の動作を検証するための実装であり、飛行認定済みfirmwareではありません。`StartSequence`後は実際にmotorとパラシュートサーボが動作し得ます。

現在の99L source contractは[`docs/99l_source_contract.json`](docs/99l_source_contract.json)に固定しています。Control rollは離床相対の連続・unwrapped推定値からentry時に一度だけreferenceをcaptureし、full-turn差をそのまま制御・0x10A telemetryへ渡します。

対象基板は`temp_new.kicad_sch` / `temp_new.kicad_pcb`です。現行`board_config.hpp`のGPIO38/39/40/41/42/44はこの基板と一致しています。

詳細な試験順序は[docs/bringup.md](docs/bringup.md)、実施結果は[docs/bringup_results.md](docs/bringup_results.md)を参照してください。

2026-08-14にMission `/dev/ttyACM1`、ComBoard `/dev/ttyACM0`、Ground `/dev/ttyUSB0`を同時接続し、bring-upとproductionを実機確認しました。Mission productionはICM42688、AS5047D health gate/pipeline、CAN、AirData、Parachuteの各taskを起動し、panic/reset loopなしで動作しました。CommandReceive中のproduction CANは、修正後30秒で`0x100/0x109`を各3,000 frame、`0x108`を750 frame、`0x102`を301 frame、`0x103/0x107`を各300 frame、`0x012`を30 frame受信側で確認し、sequence gapとdecode errorは0でした。安全なGround commandとActuatorEmergencyのend-to-end結果も確認済みです。数値と判定は[docs/bringup_results.md](docs/bringup_results.md#br-012-3基板production-canと安全command-round-trip)に記録しています。

## architecture

- `src/protocol`: Classic CAN 11-bit/125 kbit/sのlittle-endian codec、04a量子化、ID別sequence/schedule
- `src/mission`: 5状態FSM、flight epoch、reset/Deep Sleep recovery、16件transaction cacheとreplay
- `src/sensors`: liftoff/apex detector、freshness、ICM timestamp continuity、1 sample欠落補間
- `src/control`: Roll/ZeroHold、60〜180 m/s gain interpolation、quadratic N=3、TorqueMapper、stopper判定
- `src/actuators`: power cutoff latch、パラシュートstall retry/global deadline
- `src/runtime`: fixed task/queue allocationとhardware owner。FSMは`MissionRealtimeTask`だけが更新し、CAN/CommandWorkerからmessageで遷移要求を受けます。

優先順はSafety→Parachute→MissionRealtime→AirData→CAN→persistenceです。SPI2/SPI3とmotorはMissionRealtime、I2CはAirData、CAN controllerはCanTask、STS UARTとパラシュート設定はParachuteTask、NVSはInternalFlashTaskがそれぞれ唯一ownerです。SafetyTaskはflight epoch/離床時刻を独立監視し、離床+25秒でUART taskに依存せずGPIO40/44を遮断します。

ICM42688のfreshnessは取得時の`host_timestamp_us`と`esp_timer_get_time()`を同じhost clock domainで比較します。ICM sensor timestampはhistory/姿勢計算用に保持しますが、clock driftを含むsensor時刻とhost時刻を直接比較して再初期化を繰り返してはいけません。CAN送信はTX完了callbackまで1枠を占有するため、production telemetryは5 msの有限timeoutで直列化します。Emergencyのstate判定はCanTaskのmutex noWait snapshotに依存せず、nonzero transactionをlatchしてMissionRealtimeTaskへ渡します。

## 暫定flight設定

未確定値を空欄のまま残すと、状態機械だけが進み、実出力は常時coastするという検証不能な状態になります。そこで、暫定値を`src/config/flight_config.hpp`と`src/config/board_config.hpp`へ集約し、値の根拠がsimulationかhardware testかをTODOで区別しています。

| 対象 | 暫定値 | 確定方法 |
|---|---:|---|
| motor極性 | 正torqueでIN1 PWM | `TODO(HW_TEST)`で回転方向を確認 |
| motor抵抗 / torque定数 / 無負荷速度 | 3.48 Ω / 0.00855 N·m/A / 1120 rpm | `TODO(HW_TEST)`で個体同定 |
| drivetrain効率 / motor電流 / 出力torque上限 | 0.60 / 2.0 A / 1.21208 N·m | `TODO(HW_TEST)`で効率・温度・機構負荷を確認 |
| 用途別requested torque | HoldPosition 0.30 / ZeroHold 0.80 / Roll Gentle 1.21208 / High-authority comparator 3.0 N·m | generic limitとして共用せず、実機同定後に再評価 |
| motor bus / PWM duty上限 | 9.0 V / 15% | `TODO(HW_TEST)`でADC実測と安全上限を確定 |
| fin software limit | ±14° | `TODO(HW_TEST)`でstopper余裕を確認 |
| fin zero | production起動後の最初の有効AS5047D角を0°とする | `TODO(HW_TEST)`で機械zero取得手順へ置換 |
| para Open / Close / 許容差 | NVSに保存するoptionalな1回転絶対角 / ±2° | Open/Closeは`SetParaOpen` / `SetParaClose`で実機位置を設定し、許容差は`TODO(HW_TEST)`で確定 |
| para速度 / 加速度 / torque | 180°/s / 360°/s² / 20% | `TODO(HW_TEST)`で開放時間とstall余裕を確認 |
| para電源安定 / 初期化目安 / retry | 100 ms / 1.5 s / 20 ms | `TODO(HW_TEST)`で電源立上り分布を確認 |
| 差圧zero / 平均窓 / 負圧許容 | 400 sample / 8 sample / 5 Pa | `TODO(SIMULATION)`と`TODO(HW_TEST)`でfilterと実測noiseを確認 |
| Saint-Venant係数 | firmware assumed 0.92 / robustness true range 0.60〜1.20 | true/assumedを共用せず`TODO(SIMULATION/AERO_VALIDATION)`で照合 |
| encoder pipeline | acquisition 1 kHz / consumer 1 kHzの暫定config | 1/2 kHzの最終選択はsystem ID後にconfig差替え |
| Roll gain | 60〜180 m/sの全点で`{0.08, 2.32, 0.04, 0.296}` | `TODO(SIMULATION)`でSpica/HIL同定値へ置換 |

`config_flags`はbit0=MotorProfile、bit1=fin zero取得、bit2=para Open/Closeの両方設定済み、bit3=SSC zero取得を示します。bit7は暫定値を含みflight qualification未完了であることを常時示します。SSC zeroはCommandReceive中の400 sampleを自動取得する暫定実装です。`StartSequence`自体はbit3未取得でも受理されますが、そのflightではControl gateを通過できないため、bench/HILではbit3を確認してから開始します。再度CommandReceiveへ戻った場合は前flightのzeroを破棄して取り直します。

Control遷移時には、同じtickで最新かつ未来時刻ではないunwrapped roll角を一度だけ基準角として取得します。角度偏差は最短角へwrapしません。flight epoch更新、CancelSequence、DisableFinControl、LiftoffDetectionEmergencyStop、reset/recovery rollbackで基準角を無効化します。Control input喪失後の再entryは禁止し、保持した基準角を再取得しません。

## 安全上の注意

- 起動直後、他のdriverやtaskより先にGPIO38、39、40、44をLOWへ設定します。TB67H450は停止、+5 Vとpara電源はOFFです。
- **production buildでは`StartSequence`によりactuatorが動作します。** 機体へ接続する前に、motor電源とpara電源を独立して物理遮断できるfixture上で確認してください。暫定値のまま飛行へ使用してはいけません。
- bring-up buildのactuator試験は自動実行しません。USB consoleから対応commandを明示的に入力した場合だけ実行します。
- production motorはcommand_receive、reset/recovery無効化、ActuatorEmergency、sensor/config不正時にcoastまたはbrakeへ移ります。±14°のsoftware limitでstopper方向またはzero torqueが要求された場合は、back-EMF補償PWMを残さず明示brakeとし、中心方向のtorqueだけを許可します。ActuatorEmergencyではmotorをcoast、Paraをtorque OFFかつGPIO44 OFFのFreeへ移し、差圧系GPIO40は維持します。driver APIが失敗した場合はmotor unavailableをlatchし、coastへ退避します。
- production paraは通常`StartSequence`でfreshな現在位置をHoldし、Close位置へは移動しません。Open/Closeを検証して成功時にflight RAM snapshotへfreezeし、Descent/OpenとretryはそのsnapshotのOpenだけを使用します。0.5秒間隔のretryを含む5秒の全体deadlineは延長せず、成功・通信不能のどちらでもOpen試行終了時にGPIO44のPara電源だけを遮断します。SafetyTaskは離床+25秒でGPIO40/44の両方をUART taskと独立に遮断し、再投入不能にラッチします。
- bring-up motor PWMは`motor-arm`後だけ許可されます。arm状態はRAMだけに保持され、resetで必ず解除されます。試験中でも`motor-disarm`を受理し、通常は即時、出力lock競合時も5 ms以内に安全停止を再試行します。production経路はbring-upのarm状態を使用せず、Mission FSMと安全gateで出力を管理します。
- bring-upでmotor単体を接続する場合は動翼・stopperが無いため、飛行用software limitを使いません。bring-up専用上限はduty 15%、速度100 rad/s、command時間45秒です。速度上限は実測前の暫定値で、`combined-motor-imu-test`の実行時間は41秒です。
- bring-upのSTS UART1はshell初期化時、Para電源OFFのまま一度だけopenし、shell lifetime中は保持します。`sts-*` command時だけPara電源をONにして100 ms待機し、起動中のtimeoutだけを1.5秒のdeadline内で再PINGしてからSTS3215を初期化します。終了時はtorqueを無効化してPara電源をOFFにしますが、UARTはcloseしません。最初の移動は`sts-small-move`で絶対値3度以下に限定します。
- `aux5v-on`はGPIO40をHIGHにし、`aux5v-off`またはresetまで+5 Vを保持します。必要な試験中だけ明示的にONにし、終了後は必ずOFFへ戻します。
- `pio device monitor`とbinary captureを同時に開かないでください。同じUSB deviceを同時利用できません。
- motor capture中に`Ctrl-C`を入力するとhost toolはportを閉じる前に`motor-disarm`を送ります。ただしUSB通信不能時には届かないため、物理resetまたはmotor電源遮断を直ちに行える状態を必ず確保してください。
- CAN診断前に`status`でmotor disarm、IN1/IN2 LOW、AUX5V OFF、Para電源OFFを確認します。CAN診断中はmotor arm/test、STS hold/move、AUX5V ON、actuator出力commandを実行しません。

## 初期取得とsubmodule

新規cloneでは次を実行します。

```sh
git clone --recurse-submodules https://github.com/CREATE-ROCKET/Avi_99L_MissionBoard.git
cd Avi_99L_MissionBoard
```

既存cloneでは次を実行します。

```sh
git submodule sync --recursive
git submodule update --init --recursive
git submodule status
git -C lib/Avi_ESP_Libs status --short
git -C lib/Avi_ESP_Libs branch --show-current
git -C lib/Avi_ESP_Libs rev-parse HEAD
```

`lib/Avi_ESP_Libs`は`refactor` branchから取得し、親repositoryのgitlinkでrevisionを固定します。Mission側へdriverをvendor化しません。

## build

PlatformIOと`espressif32@7.0.1`、指定されたtool packageを初回だけnetwork経由で取得します。

```sh
pio pkg install -e avi_99l_missionboard
pio run
```

既定は`MISSION_BRINGUP_SHELL=0`のproduction runtimeです。既存bring-up shellは`pio run -e avi_99l_missionboard_bringup`でbuildします。既知のdriver panicを再現し得るCAN比較診断は、専用environmentだけに隔離しています。

```sh
pio run -e avi_99l_missionboard_can_diag
```

`avi_99l_missionboard_can_diag`だけが`MISSION_CAN_UNSAFE_DIAG=1`を定義します。通常のproduction/bring-up buildでは`can-test`とraw ESP-IDF lifecycle試験を`ESP_ERR_NOT_SUPPORTED`で拒否します。2026-08-13のbuild logで実際にcompileされたESP-IDFは6.0.1であり、PlatformIO platform versionから推測した値ではありません。既定production buildは暫定flight設定を有効化しているため、`StartSequence`後にactuator出力を実行します。

host testはworkspace外の実行可能なbuild directoryを指定します。

```sh
cmake -S host_test -B /tmp/avi-99l-mission-host
cmake --build /tmp/avi-99l-mission-host --parallel
ctest --test-dir /tmp/avi-99l-mission-host --output-on-failure
python3 tools/capture_bringup.py --self-test
```

共通golden vectorは`testdata/99l_protocol_golden_vectors.txt`です。Mission/ComBoard/Groundでbyte-identicalに保ちます。

次の条件を満たせば、通常の`pio run`にnetworkは不要です。

- `lib/Avi_ESP_Libs`をsubmoduleとして取得済み
- `platformio.ini`で指定するplatform/toolchain/ESP-IDF/esptool packageをPlatformIO cacheへ取得済み
- remote `lib_deps`に依存していない

project-local board manifestは`boards/avi_99l_missionboard.json`、partition tableは`partitions.csv`です。設定はESP32-S3、Flash 16 MiB、Octal PSRAM 8 MiB、QIO 80 MHzです。

## FV / FH+ / FH− / M0 characterization

実機data取得専用buildはproductionと別entry pointです。起動時は必ずmotor coast/disarmで、profileを自動開始しません。

```sh
pio run -e avi_99l_missionboard_characterization
pio test -e native
python3 tools/capture_characterization.py --self-test
python3 tools/verify_characterization.py --self-test
python3 tools/package_spica_characterization.py --self-test
```

対象board・portを確認し、人間がcharacterization firmwareのuploadを明示許可した場合に限り、専用environmentを指定します。

```sh
pio run -e avi_99l_missionboard_characterization -t upload \
  --upload-port "$MISSION_PORT"
```

campaign順序、console command、M0のcold-power gate、abort条件、V5 wire contract、取得・検証・package手順は[docs/characterization_campaign.md](docs/characterization_campaign.md)を参照してください。1000 Hz fullは全stageで必須、2/5 kHzは先に静止rate-checkを行い、unsupportedの場合も理由と分母をraw logへ残します。

2026-08-15時点では、搭載計器審査書のAS5048A/20 kHzと現行code・実機記録のAS5047D/30 kHzが衝突し、motor極性も未確定です。buildと非駆動確認はできますが、この衝突をoperatorが解消するまで`char arm`と`char run full`を実行してはいけません。upload、motor arm、stage変更、fin取り外しは自動化しません。

drive gateの既定値はhardware approval `0`、command-to-fin sign `0`です。両方を実機確認済みの値へ明示設定しない限り、firmwareはarmを拒否します。

raw `.bin`を検証してから、全stageのUART logとconditionsをまとめます。

```sh
python3 tools/verify_characterization.py capture.bin --integrity integrity.json
python3 tools/package_spica_characterization.py captures \
  "99l_characterization_<session-id>" \
  --conditions conditions.json --operator-label "<operator-label>" \
  --uart FV=FV_uart.log --uart FH_positive=FH_positive_uart.log \
  --uart FH_negative=FH_negative_uart.log --uart M0=M0_uart.log --csv
```

binaryが正本で、CSVはlosslessな確認用です。validator/importerがrejectした値を手修正せず、writerまたはreaderの契約違反を直して新しいartifactを取得します。model fitとparameter採用は今回の範囲外です。

## port確認、upload、monitor

`/dev/ttyACM<N>`の`N`を推測せず、接続のたびに候補とchipを確認します。以下の`MISSION_PORT`には、その場で確認したdevice pathを入力してください。

```sh
pio device list
ls -l /dev/ttyACM*
read -r MISSION_PORT
test -c "$MISSION_PORT"
pio pkg exec -p tool-esptoolpy -c "esptool.py --chip esp32s3 --port $MISSION_PORT chip_id"
```

確認後に同じ変数を使います。`upload_port`はrepositoryへ固定しません。次はproduction専用であり、characterization firmwareには使いません。

```sh
pio run -t upload --upload-port "$MISSION_PORT"
```

次はhardware reviewとoperatorの明示許可後だけ使うcharacterization専用uploadです。

```sh
pio run -e avi_99l_missionboard_characterization -t upload \
  --upload-port "$MISSION_PORT"
```

uploadしたenvironmentに対応するmonitorを開始します。

```sh
pio device monitor --port "$MISSION_PORT" --baud 115200 --eol LF
```

## BringupShell command

以下は`MISSION_BRINGUP_SHELL=1`でのみ有効です。production runtimeのUSB console commandではありません。

全commandは1行ずつ送ります。不正な引数、未初期化、timeout、driver error、別testの実行中をshellが明示します。

| command | 内容 |
|---|---|
| `help` | command一覧を表示 |
| `status` | 初期化、busy、arm、安全出力、stream状態を表示 |
| `spi-test` | encoder用SPI2とIMU用SPI3のbegin/device count/endを確認 |
| `encoder-test` | AS5047Dのstatus、通常read、pipeline、error flagを確認 |
| `encoder-stream <seconds>` | AS5047Dを1 kHzでbinary stream出力 |
| `imu-selftest` | FIFO無効状態でICM42688 self-testを実行 |
| `imu-stream <seconds>` | ICM42688 FIFOを1 kHzでbinary stream出力 |
| `imu-static <seconds>` | stationary raw dataをbinary stream出力 |
| `can-lifecycle-test <count>` | TXせずCANCREATEの生成/begin/status/no-data read/end/破棄を1〜1000回確認 |
| `can-test` | CANCREATEのACK/self-loopback診断。`can_diag` buildだけで許可されるが、Avi_ESP_LibsはESP-IDF 6以上を既知safe release確認まで安全拒否 |
| `can-idf-lifecycle-test` | CANCREATEを使わずpublic ESP-IDF TWAI APIだけでself-test TX/node削除を比較。`can_diag` build専用 |
| `can-load-test <hz> <seconds>` | test ID `0x7FE`の負荷試験。ESP-IDF 6以上では既知safe release確認まで実行拒否 |
| `sts-probe` | persistent UARTで明示PING、servo begin、設定read後に安全cleanup |
| `sts-read` | STS3215 telemetryを約50 Hzで取得 |
| `sts-free` | torqueを無効化し、終了時にpara電源OFF |
| `sts-hold` | 現在位置保持を明示実行 |
| `sts-small-move <deg>` | 低速・低torqueで相対移動。`abs(deg) <= 3` |
| `i2c-probe` | `0x28`、`0x5C`、`0x5D`の有限timeoutとLPS transport latencyを確認。物理値は評価しない |
| `sd-test` | SDMMC 4-bitで1 MiB write/flush/fsync/readback/CRC/unmount |
| `flash-test` | label検証済み`flightlog`の試験範囲をwrite/read/再起動検証/erase |
| `adc-stream <seconds>` | logic/motor電圧をcalibrated ADCで100 Hz取得 |
| `aux5v-on` | +5 V enableをON。`aux5v-off`またはresetまで保持 |
| `aux5v-off` | +5 V enableをOFF |
| `calibrate` | 最新attemptのgyro bias、gravity、tilt、差圧zero validityを更新 |
| `calibration-repeat <count>` | calibrationを指定回数繰り返す |
| `motor-arm` | motor試験をRAM上でarm |
| `motor-disarm` | PWM=0、IN1/IN2 LOWへ戻しdisarm |
| `motor-polarity` | 2、4、6、8、10%の短いpulseで回転方向を実測 |
| `motor-step` | 正負5、10、15%の短いstep responseを取得 |
| `motor-prbs` | 固定seed、band-limited PRBSで同定dataを取得 |
| `motor-coast` | 加速後にHi-Zとし速度減衰を取得 |
| `motor-brake-test` | 加速後にshort brakeとし速度減衰を取得 |
| `combined-motor-imu-test` | 30秒motor-off baseline、10秒±5% PRBS、1秒coastを1回で同時取得 |

## binary data capture

host toolはPython標準libraryだけを使います。まずparserとCRC、sequence gap処理のself-testを実行します。

```sh
python3 tools/capture_bringup.py --self-test
```

取得時間にはfirmware側commandより数秒の余裕を持たせます。`pio device monitor`を閉じてから実行してください。

```sh
python3 tools/capture_bringup.py "$MISSION_PORT" data/bringup \
  --duration 65 --command "encoder-stream 60"

python3 tools/capture_bringup.py "$MISSION_PORT" data/bringup \
  --duration 305 --command "imu-static 300"
```

record type 1〜5をすべてCSVへ変換します。

| type | CSV | payload size |
|---:|---|---:|
| 1 | encoder | 23 byte |
| 2 | IMU | 69 byte |
| 3 | ADC | 41 byte |
| 4 | motor identification | 78 byte |
| 5 | calibration | 57 byte |

出力はbyte列をそのまま保存する`.raw`、capture統計とsequence gapを記録する`_summary.json`、record種別ごとのCSVです。IMU recordについてはgyroの平均・標本標準偏差・時間drift、accelerationの平均・norm・標本標準偏差、1/3/10秒window biasのばらつきもsummaryへ逐次集計します。

`Ctrl-C`時は`motor-disarm`送信を最大0.5秒試行し、成否を`emergency_disarm_attempted`と`emergency_disarm_error`へ記録します。これは物理resetまたは電源遮断の代替ではありません。`data/bringup/`は`.gitignore`対象で、結果の要約だけを`docs/bringup_results.md`へ残します。詳細なwire formatは[docs/bringup.md](docs/bringup.md#binary-stream-protocol)を参照してください。

## protocol

CANはstandard 11-bit、125 kbit/s、DLC 8以下、multi-byteはlittle-endianです。`0x001/002` Emergency、`0x008/105` Recovery、`0x010/011` generic request/result、`0x012/013` Time、`0x020` MissionEvent、`0x100..109` telemetryを実装します。Emergencyはgeneric commandへ統合しません。transaction ID 0は禁止、pending ID再利用禁止、同一requestはreplay、同一ID異payloadは`ProtocolError`です。

100 HzはKinematics/Control/AirspeedとDescent中のDescentCore、25 HzはLPS、10 HzはMissionStatus/PowerTime/Tiltです。AirData power cutoff後はLPS/Airspeedを停止します。未取得物理値は04aのsemantic error rawを送信し、0として偽装しません。

## production MotorProfile build

active MotorProfileはruntime/NVSでは選択しません。`AVI_99L_MOTOR_PROFILE_ID`をbuild時に必ず明示し、未指定または未知IDはcompile errorにします。例えばFlightMotorA候補を選ぶ場合は次のようにbuildします。

```sh
PLATFORMIO_BUILD_FLAGS="-DMISSION_BRINGUP_SHELL=0 -DAVI_99L_MOTOR_PROFILE_ID=1" pio run -e avi_99l_missionboard
```

ID 1は現在も`TODO(HW_TEST)`のqualification未完了なので、選択してbuildできても`MotorProfileValid=false`です。通常`StartSequence`ではreadiness bit3がmissingになり、`ForceStartSequence`でも状態をvalidへ書き換えません。

## hardware assumptions

- ICM42688とAS5047Dは各独立SPI busで1 kHz acquisition候補
- LPS25HB/SSCは外部pull-upのI2C0 300 kHz。SSC未接続は正常な起動条件で、Controlだけをinhibitします。
- 現在のLPS25HB個体はpressure/temperatureの妥当性を保証できません。bring-upではtransport/config/read/cleanupとread latencyだけを評価し、物理値、実効ODR、freshness、離床・頂点判定を合格扱いしません。
- 2026-08-13のaddress probeでは100 ms以上安定後も未使用`0x5D`へ1/3,200のintermittent false ACKがありました。logic analyzerでwire ACK/NACKを確認するまでI2C address検出も未解決です。
- STS3215とactive/flightパラシュート設定は`ParachuteTask`が唯一ownerです。Open/Closeは0..4095 countの1回転絶対角として独立にNVSへ保存し、方向は保存しません。絶対Open/Closeは共通の最短円周経路を使い、exact 180 deg（2048 count）は方向を選ばず拒否します。signed相対角をそのまま使うのは`ParaMoveRelative`だけです。
- bring-upのSTS tx/response timeout 100 msと起動deadline 1.5秒は`TODO(HW_TEST)`暫定値です。2026-08-13の実機ではPara電源ON後の最初の6回がtimeoutし、7回目のPING、STS3215初期化、50回のtelemetry read、cleanupが成功しました。個体差・電源条件を測定後に安全に短縮します。
- MotorProfile polarity/個体値、fin software limit、para位置・速度・torqueは暫定値を設定し、`TODO(HW_TEST)`を保持します。
- Roll gain table、freshness/debounce、quadratic estimator、差圧filter、torque scaleは暫定値を設定し、`TODO(SIMULATION)`を保持します。

## known limitations

- NVS para設定は`InternalFlashTask`を唯一ownerとして接続済みです。`SetParaOpen` / `SetParaClose`はfreshなSTS絶対角を保存・commit・readback検証した後だけactive設定を更新します。flight中はOpen target取得のためにNVSを読みません。Internal Flash append logとSD production loggerはowner taskの安全stubまでで、flight pathへ未接続です。software/watchdog reset用checkpointはversion/CRC付きRTC memoryで復旧し、POR後の絶対時刻復旧は未接続のため安全側に飛行再開しません。
- production runtimeは暫定flight設定で有効ですが、値はsimulation/HIL/実機で未確定です。bit7が立っているbuildを飛行認定済みとして扱ってはいけません。
- ICM history/replay、AS5047D unwrap、quadratic fin rate、ZeroHold/Roll、TorqueMapper、TB67H450 PWMは接続済みです。起動時AS5047D角をzeroとするため、電源投入時の翼角がずれていれば、そのずれを0°として制御します。
- productionの手動Para commandは接続済みです。`SetParaOpen` / `SetParaClose`は全argumentを0とし、現在位置だけを保存します。通常`StartSequence`は7項目readinessを同一snapshotで評価し、`ForceStartSequence 0x04`はそのmissing maskだけをbypassします。Open/Closeはflight snapshotでも独立optionalのまま保持し、actual current→targetがexact half-turnの場合だけ移動しません。Open失敗または5秒retry期限後もSTSはHold要求と再接続を続け、電源は離床+25秒の絶対cutoffまで維持します。明示`RunPreflightCalibration`はgyro bias、gravity reference、SSC zeroの最新attemptを更新し、Force時もinvalid値をvalidへ偽装しません。
- ADCによるmotor/logic電圧監視とbattery present threshold/debounceはproduction flight gateへ未接続です。
- AS5047DではDIAG/AGC/magnitudeが全て0の応答を一度確認しましたが、最終再試験では`offset_done=1`、AGC 61、magnitude 4616で正常化しました。bring-upとproduction startupは共通health判定でall-zeroを`ESP_ERR_INVALID_RESPONSE`とし、productionはpipelineを開始せずfin angle/rateをunavailable、motorをcoastに保ちます。角度0度そのものは故障条件ではありません。実機productionでは正常status時のpipeline開始と、一時診断buildでall-zeroを注入した際のstatus拒否/pipeline未開始を確認しました。確認を妨げていたMissionRealtimeTaskのstack overflowは、task専有の大型history/FIFO wrapperをstatic storageへ移して解消し、startup時点の最小空き2,504 byteを実測しています。このgateはstartupと明示reinitialize時だけで、1 kHz loopへ周期的なpipeline停止を追加していません。飛行中にMISOがstuck-lowへ遷移した場合のDIAG検出は残TODOです。
- ESP-IDF 6.0.1のlocal `esp_twai_onchip.c`にはEspressif issue [#18803](https://github.com/espressif/esp-idf/issues/18803)の修正がありません。TX完了通知のevent-group更新がtimer taskへ遅延したままnodeを削除するuse-after-freeを実機で再現したため、Avi_ESP_Libsの`CANCREATE::test()`はESP-IDF 6以上を既知safe release確認まで`ESP_ERR_NOT_SUPPORTED`で拒否します。常駐ownerを使うproduction CANとはlifecycleが異なります。
- Deep SleepはRTC marker/wake causeを検証した専用task subsetで10秒周期wakeします。Internal Flash/SD log reader未接続のためlog dump要求は`SourceUnavailable`です。2秒command windowは`TODO(HW_TEST)`です。
- 3基板実機でMission→ComBoard CAN、ComBoard↔Ground LoRa、Ground→Missionの安全なcommand round-tripは確認済みです。今回接続したproduction motor motion、STS収納保持/Open、差圧zero、Control遷移、5秒/25秒cutoffは未検証であり、成功扱いしません。
- Mission基板上のmicroSDはbring-up `sd-test`で1 MiB write/read/CRCをPASSしていますが、Mission production loggerは未接続です。今回BLOCKEDとなった`CAN.CSV` logging/readbackはComBoard側microSDの初期化問題であり、Missionのbring-up microSD PASSを取り消す結果ではありません。
