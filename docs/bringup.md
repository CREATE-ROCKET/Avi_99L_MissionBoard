# C-99L Mission Board bring-up手順

## 1. 目的と範囲

この手順は次の順序を守ります。

1. `Avi_ESP_Libs`の公開APIをMission Board実機で検証する
2. 制御・推定・filter係数を決めるためのraw dataを取得する
3. PASSしたdriverだけを後段のMission実装へ引き渡す

完全なMission FSM、LQR、ZeroHold gain、TorqueMapper tuning、LoRa protocol、ComBoard、ground stationは対象外です。物理方向または係数が未確定の場合は推測せず`Unconfigured`のまま扱います。

## 2. 試験前check

1. 作業tree、branch、HEAD、submoduleを記録する。

   ```sh
   git status --short
   git branch --show-current
   git rev-parse HEAD
   git submodule status
   git -C lib/Avi_ESP_Libs status --short
   git -C lib/Avi_ESP_Libs branch --show-current
   git -C lib/Avi_ESP_Libs rev-parse HEAD
   ```

2. buildする。

   ```sh
   pio run
   ```

3. portを列挙し、確認したpathを入力する。

   ```sh
   pio device list
   ls -l /dev/ttyACM*
   read -r MISSION_PORT
   test -c "$MISSION_PORT"
   pio pkg exec -p tool-esptoolpy -c "esptool.py --chip esp32s3 --port $MISSION_PORT chip_id"
   ```

4. uploadしてmonitorを開く。

   ```sh
   pio run -t upload --upload-port "$MISSION_PORT"
   pio device monitor --port "$MISSION_PORT" --baud 115200 --eol LF
   ```

5. boot logで次を確認する。

   - safe output初期化が`ESP_OK`
   - GPIO38、39、40、44がLOWで、motor、+5 V、para電源が意図せず動作していない
   - physical Flashが16 MiB
   - PSRAMが初期化済みで8 MiB
   - custom partition tableが選択されている
   - `status`でmotorがdisarm

6. `python3 tools/capture_bringup.py --self-test`をPASSさせる。

各試験の前後で`status`を確認します。同じdeviceを同時に操作するcommandを実行せず、1つの試験が終了してから次へ進みます。失敗時は次のactuator試験へ進みません。

## 3. 非actuator試験

### 3.1 SPI transport

`spi-test`を実行します。

- encoder bus: SPI2_HOST、MOSI GPIO4、MISO GPIO5、SCLK GPIO6
- IMU bus: SPI3_HOST、MOSI GPIO17、MISO GPIO16、SCLK GPIO18
- 両busの`begin`と`end`が成功する

`spi-test`が直接確認するのは両hostの`begin`、device count、`end`です。transport errorの注入は行わないため、このcommandだけでerror/timeout pathをPASSにしません。sensor commandの有限timeoutまたは専用fault injectionを別途実施し、hangしないことを記録します。

### 3.2 AS5047D

1. `encoder-test`で`begin`、`getStatus`、`read`、pipeline開始/停止、error flag read/clear、`end`を確認する。
2. monitorを閉じ、60秒streamを取得する。

   ```sh
   python3 tools/capture_bringup.py "$MISSION_PORT" data/bringup \
     --duration 65 --command "encoder-stream 60"
   ```

3. sensorを手で1回転し、0/360度境界を跨ぐことを確認する。motor回転中の確認はmotor試験段階で行う。

PASS条件は1 kHzで60秒、`angle_raw`が0..16383、degree/radian変換が整合、sequence gapなし、parity/framing errorなし、sensor error flagなしです。jitter、AGC、magnitude、磁界low/high、CORDIC overflow、sample数、error数、最大read latencyを記録します。pipelineがerrorで解除された場合は、その事実と再初期化結果を記録します。

### 3.3 ICM42688

1. `imu-selftest`を先に実行する。self-test時はFIFOを有効化しない。
2. monitorを閉じ、60秒streamを取得する。

   ```sh
   python3 tools/capture_bringup.py "$MISSION_PORT" data/bringup \
     --duration 65 --command "imu-stream 60"
   ```

3. 60秒試験がPASSした後、5分stationary logを取得する。

   ```sh
   python3 tools/capture_bringup.py "$MISSION_PORT" data/bringup \
     --duration 305 --command "imu-static 300"
   ```

`timestamp_ticks`、`timestamp_us`、timestamp単調性/wrap、accel/gyro validity、ODR change、FIFO full、lost packet、FIFO fault、sample数、read latencyを確認します。`body_z = -sensor_z`だけが確定済みで、X/Yは変換しません。通常動作のlost packetを先にゼロへすることを優先し、1 sample補間候補の評価はraw data取得後に行います。

### 3.4 I2C/AirData no-device

`i2c-probe`を実行します。I2C_NUM_0、SDA GPIO47、SCL GPIO48、300 kHz、internal pull-upなし、lock noWait、有限operation timeoutを使います。

現在の基板ではSSCは未接続、LPS25HBは接続済みです。`aux5v-on`を明示実行し、100 ms以上待ってから`i2c-probe`を実行します。終了後は`aux5v-off`で必ずOFFへ戻します。`aux5v-on`は自動cleanupされず、OFF commandまたはresetまでGPIO40をHIGHに保持します。

- SSC `0x28`
- LPS `0x5C`
- LPS `0x5D`

SSC `0x28`が見つからない結果は正常です。各未接続addressへの100回のaccessが有限時間で戻り、bus hangとresource leakが無いことをPASS条件とします。LPS25HBはODRを25 Hzへ設定して25回readし、I2C transactionの成功数、error数、最大latency、cleanupだけを評価します。45 ms間隔のpollは実効25 Hzを証明しません。

現在のLPS25HB個体はpressure/temperatureの妥当性を保証できないため、物理値を表示・合否判定・parameter決定へ使いません。実効ODR、sample更新、averaging delay、freshness、離床・頂点判定も本試験では`NOT EVALUATED`です。

### 3.5 CAN

CAN配線とpeerのtest ID許可を確認してから実施します。ただし現在のESP-IDF 6.0.xでは、TWAI node削除後にdeferred callbackが解放済みevent groupへ触れ得る既知問題があるため、`can-test`と`can-load-test`は`ESP_ERR_NOT_SUPPORTED`で安全拒否します。

1. upstream修正を含むESP-IDFへ更新後、`can-test`を実行する。`CANCREATE::test`は標準ID `0x7FF`をnormal/single-shotで送り、ACKが無い場合はno-ack/self-loopbackでcontrollerとpeer不在を切り分け、最後に元Configを復元する。このcommandは`recover`を呼ばない。
2. 同じ修正後、peer接続時に`can-load-test 100 60`を実行する。bring-up専用標準ID `0x7FE`、payload内sequence counterを使う。bus-offまたはrecoveringを検出した場合の`recover`はこの負荷試験で確認する。
3. requested、driver queue投入成功、write error、RX error、bus error、dropped RX、bus-off回数、recovery成否、平均/最大write latencyを記録する。

`write == ESP_OK`はdriver queueへの投入成功であり、物理ACK完了数ではありません。100 Hzで問題が出た場合はlibrary、Mission Board、配線、peerを切り分け、peer処理能力またはbus loadが原因と確認できた場合だけ10 Hzを比較します。

### 3.6 storage

`sd-test`はSDMMC 4-bitで実行し、SPI modeへ変更しません。1 MiBの決定的patternについてmount、create、sequential write、flush/fsync、close、reopen、readback、CRC/内容一致、unmountを確認します。速度と最大blocking時間を記録し、失敗してもapplicationをabortしません。

`flash-test`は次の二段階です。

1. partition type/subtype/label/address/sizeを確認してから、`flightlog`先頭の試験範囲だけをerase/write/readbackする。
2. `reboot_required`表示後にresetし、もう一度`flash-test`を実行する。再起動後readback/CRCを確認して試験範囲をeraseする。

NVS、PHY、factory appをeraseしてはいけません。`flightlog`内に必要なdataがある状態では実行しません。

### 3.7 ADC

monitorを閉じ、標準試験として10秒取得します。

```sh
python3 tools/capture_bringup.py "$MISSION_PORT" data/bringup \
  --duration 15 --command "adc-stream 10"
```

GPIO1 logicとGPIO2 motorを100 Hzで取得します。ESP-IDF calibration APIによるpin電圧と、`V_source = V_pin * 5.7`によるsource電圧を記録します。`raw * 3.3 / 4095`だけの換算は禁止です。battery present thresholdはこの試験では決定しません。

### 3.8 calibration

`calibrate`を実行し、gyro biasとgravity vectorを取得します。SSC未接続時は差圧zeroだけ`valid=false`でよく、attempt全体を失敗扱いにしません。`calibration-repeat <count>`で再現性を確認します。

採用するのは常に最新attemptだけです。最新attemptで失敗した項目を過去の値へrollbackしません。gravity vectorはtilt判定に使い、gravityだけでroll zeroを決めません。

## 4. STS3215試験

周囲の安全を確認し、servoを目視できる状態で次の順に1 commandずつ実行します。

1. `sts-probe`
2. `sts-read`
3. `sts-free`を実行し、可能な範囲でhand-movable状態を確認
4. `sts-hold`
5. faultが無い場合だけ`sts-small-move 3`または`sts-small-move -3`

STSCREATE UART1はbring-up shell初期化時、Para電源OFFのまま一度だけbeginし、shell lifetime中はopenしたまま保持します。各commandはPara電源ON、100 ms安定待ち、起動中のtimeoutまたは不完全応答だけを1.5秒のdeadline内で再PING、STS3215 begin、必要なoperation、disableTorque、STS3215 end、Para電源OFFの順で処理します。STSCREATEはcommand終了時にendしません。`PING`、`STS begin`、cleanupはそれぞれresult、latency、device errorを個別に記録します。

tx/response timeoutは切り分け用に100 ms / 100 msとしますが、起動deadlineとともに`TODO(HW_TEST)`の暫定値です。2026-08-13の実機では最初の6回がtimeoutし、同じ電源cycleの7回目でPINGに成功しました。続くSTS3215 begin、`sts-read`の50 sample、cleanupが成功し、2 command目でも`GPIO 42 is not usable`は再発していません。PINGが1.5秒間すべてtimeoutする場合は物理UART/RS-485を、PING成功後のSTS beginだけ失敗する場合はconfiguration readを次段階で調査します。

response latency、timeout数、`lastDeviceError`、overload、overcurrent、overtemperature、encoder/voltage fault、position、speed、current、voltage、temperatureを記録します。最初の移動は絶対値3度以下、低speed、低torqueです。

## 5. motor試験

### 5.1 共通安全条件

- 動翼とstopperが無く、軸が自由に回ることを再確認する
- 人、配線、工具を回転部から離す
- `status`でdisarmを確認してから`motor-arm`を明示実行する
- PWM carrierは30 kHz、IN1 GPIO39、IN2 GPIO38
- duty絶対値は15%以下、1 commandの時間は45秒以下。`combined-motor-imu-test`は41秒
- encoderから得る速度がbring-up上限を超えた場合は直ちに停止する
- test終了、error、timeoutのすべてでPWM=0、IN1/IN2 LOWへ戻す
- 飛行用fin software limitをこの試験へ流用しない
- binary capture中の`Ctrl-C`はportを閉じる前に`motor-disarm`を最大0.5秒試行する
- USB通信不能時に備え、物理resetまたはmotor電源遮断を直ちに実行できる状態にする

### 5.2 実施順序

1. `motor-polarity`: 正負方向について2、4、6、8、10%の短いpulseを実行し、AS5047Dの符号からpolarityを記録する。polarityを事前に推測しない。この段階のlevel列をdead-zone候補にも使う。
2. `motor-step`: 正負5、10、15%の短いstepを取得する。
3. `motor-coast`: 一定速度後にHi-Zとして減衰を取得する。
4. `motor-brake-test`: 一定速度後にshort brakeとして減衰を取得する。
5. `motor-prbs`: 固定seedで最初の10秒は正負5%、sensor/error/速度条件を満たした場合だけ後半10秒を正負10%で実行する。15% PRBSは現commandでは実装しない。
6. `combined-motor-imu-test`を1回だけ実行する。先頭30秒はmotor-off baseline、続く10秒は正負5% PRBS、最後の1秒はcoastで、AS5047DとICM42688を1 kHz同時取得する。baseline用とexcitation用に二重実行しない。
7. `motor-disarm`を実行し、`status`でdisarmと安全出力を確認する。

binary dataを残す場合、monitorを閉じてhost toolからcommandを送ります。armは最初の短いcaptureで行い、各testのhost durationにはfirmware実行時間より数秒の余裕を持たせます。次はcombined試験の例です。

```sh
python3 tools/capture_bringup.py "$MISSION_PORT" data/bringup \
  --duration 2 --command "motor-arm"
python3 tools/capture_bringup.py "$MISSION_PORT" data/bringup \
  --duration 46 --command "combined-motor-imu-test"
python3 tools/capture_bringup.py "$MISSION_PORT" data/bringup \
  --duration 2 --command "motor-disarm"
```

`Ctrl-C`を受けたhost toolは`motor-disarm`を送信してからportを閉じ、summaryへ試行結果を残します。送信失敗または反応不明時は、待たずに物理resetまたはmotor電源遮断を行います。

高回転、異音、過電流、sensor error、stream overflow、timeoutを検出したら即中止します。無制限Gaussian white noiseはPWMへ入力しません。現在の個体をMotorProfile候補Aとして記録し、同じcommandとcapture形式を予備motor Bにも再利用します。

記録対象はtimestamp、command duty/direction、motor bus電圧、encoder raw/unwrapped angle/角速度/status、IMU raw accel/gyro/timestamp/FIFO status/errorです。

## 6. binary stream protocol

1 kHz dataを大量に`printf`せず、固定長queueを介したexplicit little-endian frameとして送ります。C++ structのpaddingやhost endianには依存しません。

| offset | size | field | 内容 |
|---:|---:|---|---|
| 0 | 2 | magic | `A5 5A` |
| 2 | 1 | version | 現在は`1` |
| 3 | 1 | record type | `1=encoder`, `2=imu`, `3=adc`, `4=motor`, `5=calibration` |
| 4 | 2 | payload length | little-endian、最大96 byte |
| 6 | 4 | sequence | little-endian uint32、capture開始時に0へ戻る |
| 10 | 可変 | payload | fieldごとに明示serialize |
| 10+N | 2 | CRC | CRC-16/CCITT-FALSE、little-endian |

CRC対象はversionからpayload末尾までで、magicを含みません。parameterはpolynomial `0x1021`、initial `0xFFFF`です。queue depthは64で、enqueueはnoWaitです。overflow時は黙って捨てずdrop counterを増やし、送信前にsequenceが進むためhost側でもgapとして検出できます。

`tools/capture_bringup.py`は明示された`/dev/ttyACM<N>`だけを開き、raw byte保存、magic再同期、length上限、CRC、32 bit sequence wrap、gap/duplicate/out-of-orderを検査します。ASCII shell outputが混在してもmagicまで読み飛ばします。type 1〜5をすべてdecodeし、payload sizeはencoder 23 byte、IMU 69 byte、ADC 41 byte、motor 78 byte、calibration 57 byteです。未知typeはpayloadを失わず`unknown` CSVへ残し、既知typeの長さ不一致は`unknown` CSVとsummaryのdecode errorへ残します。

IMU recordはCSV変換と同時に、gyro 3軸の平均・標本標準偏差・時間に対する最小二乗drift、acceleration 3軸の平均・標本標準偏差・norm、1/3/10秒の非重複windowごとのgyro biasばらつきをonline集計し、`_summary.json`の`imu_statistics`へ保存します。valid flagがfalseまたはNaN/Infのsampleは統計から除外し、除外数を残します。

capture中の`Ctrl-C`では`motor-disarm`送信を最大0.5秒試行します。`_summary.json`の`emergency_disarm_attempted`と`emergency_disarm_error`を確認し、送信失敗またはUSB通信断時は物理resetまたは電源遮断を行います。

## 7. PASS/FAIL判定と記録

- `PASS`: 要求されたdurationとsample数を満たし、driver error、sensor fault、予期しないsequence gap、hangが無い
- `FAIL`: API/build/upload/verifyが失敗した、errorが非ゼロ、無限待ち相当の挙動、安全cleanupに失敗した
- `SKIP`: 接続されていないdeviceのsensor値検証など、既知の試験条件不足
- `PENDING`: 実施前。`SKIP`と混同しない

各試験後、[bringup_results.md](bringup_results.md)へdate/time、両HEAD、firmware build、port、duration、sample/error数、最大latency、観察、判定、原因、library修正、残TODOを記録します。library責務のbugはMission Board側へworkaroundせず、submodule側を修正してlibrary build、Mission Board build、実機再試験の順に確認します。
