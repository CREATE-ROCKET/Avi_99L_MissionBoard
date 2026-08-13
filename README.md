# C-99L Mission Board

ESP32-S3-WROOM-1-N16R8を搭載した99L Mission Board firmwareです。Vault 00〜07/04aに基づくMission FSM、CAN codec、command lifecycle、sensor continuity、制御pipeline、パラシュート安全coreをproduction moduleとして実装し、既存bring-up firmwareも明示選択できる形で維持しています。

現時点ではMotorProfile、fin software limit、パラシュートOpen/Close永続設定が未確定・未接続です。既定production buildは各owner taskとsensor/CANを起動しますが、`StartSequence`を`NotConfigured`で拒否し、motor coast/パラシュート電源OFFを維持します。飛行可能firmwareを意味しません。

詳細な試験順序は[docs/bringup.md](docs/bringup.md)、実施結果は[docs/bringup_results.md](docs/bringup_results.md)を参照してください。

## architecture

- `src/protocol`: Classic CAN 11-bit/125 kbit/sのlittle-endian codec、04a量子化、ID別sequence/schedule
- `src/mission`: 5状態FSM、flight epoch、reset/Deep Sleep recovery、16件transaction cacheとreplay
- `src/sensors`: liftoff/apex detector、freshness、ICM timestamp continuity、1 sample欠落補間
- `src/control`: Roll/ZeroHold、60〜180 m/s gain interpolation、quadratic N=3、TorqueMapper、stopper判定
- `src/actuators`: power cutoff latch、パラシュートstall retry/global deadline
- `src/runtime`: fixed task/queue allocationとhardware owner。FSMは`MissionRealtimeTask`だけが更新し、CAN/CommandWorkerからmessageで遷移要求を受けます。

優先順はSafety→Parachute→MissionRealtime→AirData→CAN→persistenceです。SPI2/SPI3とmotorはMissionRealtime、I2CはAirData、CAN controllerはCanTask、STS UARTはParachuteTaskがそれぞれ唯一ownerです。SafetyTaskはflight epoch/離床時刻を独立監視し、離床+25秒でUART taskに依存せずGPIO40/44を遮断します。

## 安全上の注意

- 起動直後、他のdriverやtaskより先にGPIO38、39、40、44をLOWへ設定します。TB67H450は停止、+5 Vとpara電源はOFFです。
- actuatorを動かす試験は自動実行しません。USB consoleから対応commandを明示的に入力した場合だけ実行します。
- motor PWMは`motor-arm`後だけ許可されます。arm状態はRAMだけに保持され、resetで必ず解除されます。試験中でも`motor-disarm`を受理し、通常は即時、出力lock競合時も5 ms以内に安全停止を再試行します。
- motorには動翼・stopperが無いため飛行用±15度limitは使いません。bring-up専用上限はduty 15%、速度100 rad/s、command時間45秒です。速度上限は実測前の暫定値で、`combined-motor-imu-test`の実行時間は41秒です。
- STS3215は`sts-*` command実行中だけpara電源をONにし、終了時にtorqueを無効化して電源をOFFにします。最初の移動は`sts-small-move`で絶対値3度以下に限定します。
- `aux5v-on`はGPIO40をHIGHにし、`aux5v-off`またはresetまで+5 Vを保持します。必要な試験中だけ明示的にONにし、終了後は必ずOFFへ戻します。
- `pio device monitor`とbinary captureを同時に開かないでください。同じUSB deviceを同時利用できません。
- motor capture中に`Ctrl-C`を入力するとhost toolはportを閉じる前に`motor-disarm`を送ります。ただしUSB通信不能時には届かないため、物理resetまたはmotor電源遮断を直ちに行える状態を必ず確保してください。

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

既定は`MISSION_BRINGUP_SHELL=0`のproduction runtimeです。既存bring-up shellは`pio run -e avi_99l_missionboard_bringup`でbuildします。flight設定が未確定の既定buildではactuator commandを自動実行しません。

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

## port確認、upload、monitor

`/dev/ttyACM<N>`の`N`を推測せず、接続のたびに候補とchipを確認します。以下の`MISSION_PORT`には、その場で確認したdevice pathを入力してください。

```sh
pio device list
ls -l /dev/ttyACM*
read -r MISSION_PORT
test -c "$MISSION_PORT"
pio pkg exec -p tool-esptoolpy -c "esptool.py --chip esp32s3 --port $MISSION_PORT chip_id"
```

確認後に同じ変数を使います。`upload_port`はrepositoryへ固定しません。

```sh
pio run -t upload --upload-port "$MISSION_PORT"
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
| `can-test` | CANCREATEのACK/self-loopback診断、status、Config復元を確認 |
| `can-load-test <hz> <seconds>` | test ID `0x7FE`、sequence付きframeで負荷、bus-off/recoveryを確認 |
| `sts-probe` | para電源ON、bus/servo begin、設定read後に安全cleanup |
| `sts-read` | STS3215 telemetryを約50 Hzで取得 |
| `sts-free` | torqueを無効化し、終了時にpara電源OFF |
| `sts-hold` | 現在位置保持を明示実行 |
| `sts-small-move <deg>` | 低速・低torqueで相対移動。`abs(deg) <= 3` |
| `i2c-probe` | `0x28`、`0x5C`、`0x5D`を含むno-device有限timeout試験 |
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

## hardware assumptions

- ICM42688とAS5047Dは各独立SPI busで1 kHz acquisition候補
- LPS25HB/SSCは外部pull-upのI2C0 300 kHz。SSC未接続は正常な起動条件で、Controlだけをinhibitします。
- STS3215はUART1唯一owner。Open位置未復元時は動作せずpower cutoffします。
- MotorProfile polarity/個体値、fin software limitは`TODO(HW_TEST)`のまま未設定です。
- Roll gain table、freshness/debounce、quadratic estimator、torque scaleは`TODO(SIMULATION)`を保持します。

## known limitations

- NVS para設定、Internal Flash append log、SD production loggerはowner taskの安全stubまでで、flight pathへ未接続です。software/watchdog reset用checkpointはversion/CRC付きRTC memoryで復旧し、POR後の絶対時刻復旧は未接続のため安全側に飛行再開しません。
- Roll gain tableとmotor/fin configuration未確定のためproduction runtimeはflight-disabledです。
- ICM history/replay、AS5047D unwrap、quadratic fin rateまではruntimeへ接続済みですが、Roll/ZeroHold/TorqueMapperからmotor出力への経路はprofile確定まで無効で、常にcoastします。
- ParachuteTaskはSTS唯一ownerですが、Open位置永続loader未接続のためflight Openを安全拒否します。
- Deep SleepはRTC marker/wake causeを検証した専用task subsetで10秒周期wakeします。Internal Flash/SD log reader未接続のためlog dump要求は`SourceUnavailable`です。2秒command windowは`TODO(HW_TEST)`です。
- 実機flash、CAN/LoRa round-trip、actuator試験はREADMEのbuild/test完了とは別に記録します。未実施を成功扱いしません。
