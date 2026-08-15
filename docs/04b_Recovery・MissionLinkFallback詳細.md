# 99L Recovery / MissionLinkFallback 詳細仕様

## 1. 適用範囲と優先順位

本書は、Mission Board、通信基板、Ground Station間のRecoveryBeacon移行責務と、Mission Boardからのtelemetryが失われた場合のMissionLinkFallbackを定義する。

本書のRecoveryBeacon移行責務は、以下の既存記述より優先する。

- `00_共通仕様.md` のRecovery責務に関する記述
- `02_飛行シーケンス.md` の「RecoveryBeacon移行後」
- `04_通信仕様.md` 9章/13章のRecoveryBeacon移行記述
- `08_実装仮定・検証待ち一覧.md` の旧`ForceRecoveryBeacon`/Recovery entry仮定

上記文書は順次本仕様へ同期する。矛盾時は本書を正とする。

## 2. 状態所有権

- `MissionState`はMission Boardだけが所有する。
- RecoveryBeaconはMissionStateではなく通信基板のPower/communication modeである。
- RecoveryBeaconへの**移行判断もMission Boardが所有する**。
- 通信基板はMissionStateを推測しない。
- 通信基板は離床後経過時間、last-known MissionState、MissionStatus/CAN timeout、GNSS状態等からRecoveryBeaconへ自律移行してはならない。
- Ground Stationは受信したMission telemetryだけをcurrent MissionStateとして表示する。

RecoveryBeacon中もMissionStateは`Descent`のままとする。Mission Board側ではRecovery移行済みlatch/RTC markerをMissionStateとは別に保持する。

## 3. RecoveryBeacon移行条件

### 3.1 自動移行

自動移行の初期条件は、Mission Boardが以下を満たすこととする。

- current MissionStateが`Descent`
- 離床時刻から120秒以上経過

120秒はRecovery battery life、通信要求、回収運用を含む実機確認で最終調整してよいが、条件評価主体は常にMission Boardとする。

### 3.2 地上局からの早期移行要求

手動Recovery移行は通信基板local commandで直接行わない。

MissionGeneric commandとして`EnterRecovery`を追加する。

- command code: `0x33`
- unused args: すべて0
- 受理可能state: `Descent`のみ
- 追加Safety条件: `DeploymentPowerCutoffDone=true`

条件不成立時は`InvalidState`または`SafetyInterlock`で拒否する。

Ground Stationまたは通信基板がMission Boardを経由せず通信基板だけをRecoveryBeaconへ切り替えてはならない。

旧ComBoard local `ForceRecoveryBeacon`相当commandは廃止し、受信時は`NotSupported`とする。

## 4. Mission BoardのRecovery移行手順

Mission BoardがRecovery移行を決定した場合は、次の順序を基本とする。

1. finをStop/Hi-Z相当へ安全化する。
2. パラシュートサーボおよび差圧系基板の電源遮断済みであることを確認する。
3. Mission microSD/Internal Flashのflushをbounded best-effortで要求し、無期限には待たない。
4. RTC Recovery markerを設定する。
5. `RecoveryModeCommand::EnterRecoveryBeacon`を通信基板へ送る。
6. Mission BoardはDeep Sleepへ入る。

RecoveryBeaconへの移行を通信完了待ちで無期限に止めない。`RecoveryModeCommand`のCAN送信は初期候補として最大1秒のbounded retryを許可する。

`RecoveryModeCommand`を1回も正常送信できなかった場合、Mission Boardは通信基板がRecoveryBeaconへ入ったと仮定しない。Mission BoardはDeep Sleepへ入り、Recovery専用periodic wake時に再度送信する。

## 5. RecoveryModeCommand

Mission→ComのCAN messageとして`RecoveryModeCommand`を追加する。

- CAN ID: `0x014`
- Direction: Mission→Com
- DLC: 3

| byte | 内容 |
|---:|---|
| 0 | sequence u8、wrap可 |
| 1 | mode |
| 2 | reason |

mode:

- `1`: `EnterRecoveryBeacon`
- `0`, `2..255`: reserved

reason:

- `0`: `AutoElapsed120`
- `1`: `GroundRequested`
- `2`: `RecoveryWakeRetry`
- `3`: `ResetRecovery`
- `4..255`: reserved

同一modeの再受信はidempotentに扱う。すでにRecoveryBeacon中なら状態を維持し、副作用を重複実行しない。

通信基板は有効な`EnterRecoveryBeacon`を受信した時点でRecoveryBeacon modeをlatchedし、A5を約10秒周期で送信する。

## 6. RecoveryControl `0x008`の責務変更

`RecoveryControl 0x008`はCom→MissionのRecovery log/wake制御専用とする。

旧opcode 0 `EnterRecovery`は廃止し、reservedとして`NotSupported`を返す。

opcode:

- `0`: reserved / legacy EnterRecovery / `NotSupported`
- `1`: `Wake`
- `2`: `StartLogDump`
- `3`: `StopLogDump`

RecoveryBeaconへの移行に`RecoveryControl 0x008`を使用しない。

## 7. Recovery Deep Sleep

Mission BoardはRecovery移行後、Light Sleep fallbackを持たずDeep Sleepを使用する。

Deep SleepからのwakeではRTC Recovery markerとwake causeを確認してRecovery専用boot pathへ入り、通常flight bootへ戻らない。

初期候補として10秒periodic wakeを使用する。

wake時には以下を行う。

- ADC/CAN/Recovery status更新
- `RecoveryModeCommand::EnterRecoveryBeacon`再送
- 短時間command window
- 要求がなければ再Deep Sleep

wake周期、command window、CAN再initializationは`TODO(HW_TEST)`とする。

通信基板がresetしてRecoveryBeacon latchを失った場合も、次回Mission wakeの再送でRecoveryBeaconへ復帰させる。

## 8. MissionLinkFallback communication mode

Mission Boardからのtelemetryが失われても、通信基板、GNSS、LoRa、通信基板microSDが継続動作している状態を、MissionStateとは別の`MissionLinkFallback` communication modeとして定義する。

MissionState `Unknown`をA0 CommandReceiveへ丸めてはならない。

### 8.1 Normal → Fallback

- validな`MissionStatusTelemetry`のageが300 msを超えたらA8へ切り替える。
- 300 ms以上1.0 s未満はGround上で`MISSION STATUS LATE`として扱う。
- 1.0 s以上MissionStatusが無く、他Mission periodic CANが1.0 s以内なら`MISSION_STATUS_TIMEOUT`。
- 全Mission periodic CANが1.0 s以上無ければ`NO_MISSION_TRAFFIC`。
- ComBoard起動後にMissionStatusを一度も受信していなければ`STARTUP_WAITING`。
- CAN bus-off/recovering/controller errorは時間条件を待たずA8理由へ反映してよい。

### 8.2 Fallback → Normal

ID、DLC、reserved bit、MissionState enumがvalidな`MissionStatusTelemetry`を**1回受信した時点で即復帰**する。

連続3回等のdebounceは要求しない。

Fallback中に受けたMission向けcommandを復帰後へqueueして実行してはならない。

### 8.3 Recoveryとの関係

Mission Boardの明示指示によりRecoveryBeacon modeがすでにlatchedされている場合はA5を優先する。

Mission Boardが停止・通信断になるまでに明示的な`EnterRecoveryBeacon`を受信していなければ、通信基板はA8を継続する。

**Mission link loss中に120秒等の経過時間を理由としてA5へ自律移行してはならない。**

## 9. MissionLinkFallbackTelemetry `A8`

- Header: `0xA8`
- schema version: `0x01`
- application packet length: 24 byte
- E220 fixed prefix 3 byteとappend RSSIは含めない

| byte | field | encoding |
|---:|---|---|
| 0 | Header | `0xA8` |
| 1 | schema version | `0x01` |
| 2 | sequence | u8 wrap |
| 3 | primary loss reason | enum |
| 4..5 | status flags | u16 LE |
| 6 | last valid MissionState | 0..4、未受信=`0xFF` |
| 7 | GNSS state | enum |
| 8..9 | MissionStatus age | 0.1 s/LSB、`0xFFFF=never/unavailable` |
| 10..11 | any Mission periodic CAN age | 0.1 s/LSB、`0xFFFF=never/unavailable` |
| 12..13 | last PowerTimeTelemetry age | 0.1 s/LSB、`0xFFFF=never/unavailable` |
| 14..15 | GNSS East | 既存signed16/error raw |
| 16..17 | GNSS North | 既存signed16/error raw |
| 18..19 | GNSS absolute height | 既存9 bit raw、upper 7 bit=0 |
| 20 | last logic voltage | 既存battery raw、liveではない |
| 21 | last motor voltage | 既存battery raw、liveではない |
| 22 | CAN health | enum |
| 23 | XOR | byte 0..22 |

### 9.1 primary loss reason

- `0`: `STARTUP_WAITING`
- `1`: `MISSION_STATUS_TIMEOUT`
- `2`: `NO_MISSION_TRAFFIC`
- `3`: `CAN_BUS_OFF`
- `4`: `CAN_RECOVERING`
- `5`: `MISSION_STATUS_INVALID`
- `6`: `FORCED_TEST`
- `7`: `UNKNOWN`
- `8..255`: reserved

### 9.2 status flags

- bit0: valid MissionStatusをboot後に1回以上受信
- bit1: valid Mission periodic CANをboot後に1回以上受信
- bit2: MissionStatus age < 1.0 s
- bit3: any Mission periodic CAN age < 1.0 s
- bit4: ComBoard microSD healthy
- bit5: logging requested
- bit6: logging active
- bit7: unflushed dataあり
- bit8: GNSS enabled
- bit9: GNSS valid numeric fix
- bit10: GNSS stale
- bit11: last PowerTimeTelemetryあり
- bit12: last MissionState valid
- bit13: CAN controller active
- bit14: CAN runtime error
- bit15: reserved=0

### 9.3 GNSS state

- `0`: OFF
- `1`: STARTING
- `2`: RECEIVER_DETECTED
- `3`: CONFIGURATION_FAILED
- `4`: RECEIVER_ERROR
- `5`: NO_FIX
- `6`: VALID_FIX
- `7`: INVALID_SAMPLE
- `8`: STALE
- `9..255`: reserved

### 9.4 CAN health

- `0`: UNKNOWN
- `1`: ACTIVE
- `2`: WARNING
- `3`: PASSIVE
- `4`: BUS_OFF
- `5`: RECOVERING
- `6`: CONTROLLER_ERROR
- `7..255`: reserved

A8中のlogic/motor voltageは最後にMission Boardから受信した値でありlive値ではない。Groundは`PowerTime age`を併記し、A8中はbattery remaining-time推定へこのstale値を使用しない。

A8はmode entry、reason/GNSS/CAN health変化時に即時送信し、その後は約0.5秒周期を基本とする。

## 10. Ground Station表示

A8受信時のcurrent MissionStateは`UNKNOWN`とする。最後に有効受信したMissionStateは`LAST KNOWN`としてのみ表示する。

Top bar例:

- `MISSION STATE: UNKNOWN`
- `LAST STATE: CONTROL / 1.4 s ago`
- `COMM MODE: MISSION LINK FALLBACK`
- `MISSION LINK: LOST`
- `COMBOARD: OK`
- `GNSS: VALID FIX`
- `LORA: ACTIVE / RSSI`
- `COM SD: RECORDING`

ICM42688/LPS25HB/SSCDRRN005PD2A5/AS5047D/STS3215は個別FAULTではなく`NOT REPORTED / MISSION LINK LOST`とする。

liveのまま継続する表示:

- GNSS position/fix
- East/North/absolute height
- LoRa RSSI/RX interval
- Ground側last RX
- ComBoard microSD/logging
- A8 reason
- CAN controller health

freeze/gray化する表示:

- 3D姿勢
- roll/roll rate/tilt
- fin angle/fin rate
- airspeed/pressure/temperature
- requested torque
- parachute angle
- Mission IC Health
- logic/motor voltage
- battery remaining time

3D predictionとMission由来graph補間はA8受信時に停止する。graphには`MISSION LINK LOST`のvertical markerを入れ、それ以降のMission由来lineを延長しない。

## 11. Fallback中のcommand policy

ComBoard自身に閉じたlogging/GNSS/wake/log dump commandは使用できる。

Mission generic commandは通常GUIではdisabledにする。

Emergency commandをbest-effortで送信する場合も復帰後へqueueせず単発送信とし、B0またはtimeoutまで成功表示しない。

RecoveryBeaconへ直接遷移するComBoard local commandは使用しない。

## 12. 必須試験

1. Mission未接続bootでA8 `STARTUP_WAITING`になりA0を偽装しない。
2. 正常通信後にMission電源断し、300 msでA8へ切り替わる。
3. MissionStatusだけdropし、他Mission CANありで`MISSION_STATUS_TIMEOUT`。
4. 全Mission CAN dropで`NO_MISSION_TRAFFIC`。
5. CAN bus-off/recovery reasonをA8へ反映する。
6. valid MissionStatus 1 frameでNormalへ即復帰する。
7. Missionから0x014未受信のまま120秒を超えてもComBoardがA5へ入らない。
8. Missionから0x014受信後はA5を優先する。
9. ComBoard reset後、Mission Recovery wakeの0x014再送でA5へ復帰する。
10. A8受信中にGroundの3D predictionとMission graph補間が停止する。
11. A7/A8 golden vectorをComBoard、Ground board、Ground Stationで一致させる。
