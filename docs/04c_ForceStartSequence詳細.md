# 99L ForceStartSequence 詳細仕様

## 1. 目的と優先順位

本書は、既知の故障、未設定、未検証項目、またはpreflight判定実装そのものの不具合が存在する状況で、operatorがriskを認識した上でMission sequence開始を優先するための`ForceStartSequence`を定義する。

通常の`StartSequence`を無条件化しない。通常開始はflight-readiness条件をfail-closedで評価し、`ForceStartSequence`だけを明示的なbypass経路とする。

本書の`ForceStartSequence`に関する記述は、以下と矛盾する場合に優先する。

- `01_コマンド受信モード.md` のSequence Start条件
- `04_通信仕様.md` のGeneric command一覧
- `05_実装仕様.md` のflight enable gate
- `06_初期実装・検証仕様.md` / `07_実装前最終確認・シミュレーション仕様.md` のpreflight gate
- Vault `08_実装仮定・検証待ち一覧.md` の`MISSION-006`

Vault `08`の「未確定設定では飛行sequenceを実行できない」という記述は、**通常の`StartSequence`については維持し、`ForceStartSequence`については本書で上書きする**。

## 2. Generic command

`ForceStartSequence`はMission generic commandとする。

- command code: `0x04`
- request: `GenericCommandRequest 0x010`
- args0..5: すべて0
- 受理可能MissionState: `CommandReceive`のみ
- command domain: sequence
- transaction/replay/cache規則: 他のgeneric commandと同一

unused argsが非0の場合は`InvalidArgument`とする。

通常の`StartSequence 0x01`、`CancelSequence 0x02`、`DisableFinControl 0x03`の意味は変更しない。

## 3. StartSequenceとの違い

### 3.1 StartSequence

通常の`StartSequence`はflight-readiness条件を確認する。

少なくとも以下が満たされない場合は`NotConfigured`または対応するreasonで拒否する。

- `FinZeroConfigured`
- `ParachuteOpenConfigured`
- `ParachuteCloseConfigured`
- runtimeで追加された通常flight用configuration gate

Calibration、actuator command、storage operation、bring-up motor test等との競合は`Busy`とする。

### 3.2 ForceStartSequence

`ForceStartSequence`は**flight-readiness / preflight条件を開始拒否理由にしない**。

Force対象には少なくとも以下を含む。

- Fin zero未設定または未qualification
- Para Open/Close未設定、永続化未完了、またはprovisional fallback使用
- MotorProfile未qualification
- motor polarity / fin software limit / Roll gain / ZeroHold等の`TODO(HW_TEST)` / `TODO(SIMULATION)`残存
- PreflightCalibration未実施または一部invalid
- ICM42688 / LPS25HB / SSCDRRN005PD2A5 / AS5047D / STS3215 Health異常
- storage、CAN、LoRa等の非flight-critical health異常
- 通常preflight checkerがsoftware bug等により成立しないとoperatorが判断した場合
- 将来追加されるflight-readiness check

Forceは「全状態を正常とみなす」操作ではなく、「現在の異常状態を保持したままsequence transitionだけを許可する」操作である。

## 4. Forceでもbypassしてはならない条件

以下はflight-readinessではなく、software execution safetyまたはmission safetyそのものなのでForce対象外とする。

1. current MissionStateが`CommandReceive`であること
2. transaction ID、DLC、argument、command code等のprotocol validity
3. CommandReceive→LiftoffDetectionのmemory freeze / resource preallocationが完了していること
4. Task、queue、mutex、driver owner、power arbiter等のruntime invariantが成立していること
5. Calibration、actuator operation、Flash export/erase、bring-up motor test等との排他が成立していること
6. actuator outputのNaN/Inf、未初期化memory、範囲外index等を防ぐ最低限のnumeric/runtime validation
7. SafetyTaskによるdeployment条件監視
8. 離床+17秒deployment fallback
9. 離床+25秒絶対power cutoff
10. `ActuatorEmergencyStop`、`LiftoffDetectionEmergencyStop`等のEmergency処理

これらが成立しない場合はForceでも`Busy`、`InvalidState`、`InvalidArgument`、`InternalError`等として拒否または安全側停止する。

## 5. Health・Control・後段fail-safe

Force開始時にHealth、freshness、configuration、calibration stateを正常値へ変更してはならない。

例としてSSCがUnavailableのまま`ForceStartSequence`を受理した場合、次の挙動とする。

1. `CommandReceive -> LiftoffDetection`は許可する。
2. 離床検知後は`EngineBurn`へ進む。
3. SSC availabilityを必要とするControl gateは通常どおりfalseとなる。
4. Controlへ入らない、またはControl中にUnavailableとなれば通常どおりEngineBurnへ戻り`control_reentry_inhibited`を立てる。
5. deployment条件監視は継続し、pressure条件が使えなければ+17秒fallbackを使用する。
6. +25秒絶対power cutoffは必ず実行する。

Force開始したことを理由に後段のControl/sensor/deployment fail-safeを緩和しない。

## 6. Provisional fallback値

Force開始の目的は、設定保存やpreflight checkerに不具合があってもoperator判断で飛行sequenceを開始できるようにすることである。そのため、actuator command生成に必須の値が通常設定sourceから得られない場合は、compile-timeのprovisional fallbackを使用してよい。

provisional fallbackには必ず`TODO(HW_TEST)`または`TODO(SIMULATION)`を付け、通常のconfigured値と区別する。

対象例:

- MotorProfile
- MotorPolarity
- fin software limit
- Roll gain table
- ZeroHold係数
- Para Open / Close位置および方向
- Para speed / acceleration / torque
- AirData filter / coefficient

未初期化memory、NaN/Inf、暗黙の0、前flightのstale値をfallbackとして使用してはならない。

Force開始時にどの値がnormal configで、どの値がprovisional fallbackであったかをMission logへ記録する。

## 7. Forced-start stateの記録

各flight attemptへ`forced_start`相当のlatchを持たせる。

- normal `StartSequence`: `forced_start=false`
- `ForceStartSequence`: `forced_start=true`
- `CancelSequence`後の新しいattemptでは再設定する
- reset recovery時は可能な範囲でcheckpoint/logから引き継ぐ

少なくとも以下へ記録する。

- Mission Board full-rate / event log
- Ground Station session log
- command transaction IDと`CommandResult`
- bypassしたpreflight項目のsnapshot
- 使用したprovisional fallback一覧

wire telemetryのbit割当を追加する場合は既存v1 fieldを再解釈せず、reserved fieldまたはversioned extensionで行う。bit不足を理由に既存意味を変更しない。

## 8. Ground Station GUI

GUIでは通常StartとForce Startを同じbuttonへ統合しない。

`ForceStartSequence`操作時は最低限以下を満たす。

1. 通常Startとは視覚的に明確に異なる危険操作として表示する。
2. 現在取得できるpreflight異常・未設定・stale・unavailable項目を一覧表示する。
3. 「これらを無視してsequenceを開始する」ことを明示する。
4. 二段階確認を要求する。
5. 確認後にのみ`0x04 ForceStartSequence`を送る。
6. `Accepted`だけで成功表示せず、終端`Completed`を確認する。
7. Force開始後はsession中、`FORCED START / PREFLIGHT BYPASSED`相当の警告を継続表示する。
8. session logへoperator操作時刻、transaction ID、bypass一覧を記録する。

GUIに表示できない未知の異常が存在し得るため、表示一覧を「安全が確認できた項目一覧」と誤解させない。Forceはoperatorが不明なriskも含めて受容する操作である。

## 9. Command console / 非GUI経路

raw command consoleから`ForceStartSequence`を送信できる構成は許容する。ただしGUIの確認dialogを通らないことを理由にMission Board側でForceを通常Startへ変換しない。

console/manual送信も`0x04`として明確に識別され、Mission/Ground logにforced startとして残ることを要求する。

通信基板は`ForceStartSequence`を通常generic commandとして透過転送し、独自にpreflight判定を追加しない。

## 10. 必須試験

少なくとも以下を自動testまたはHILで確認する。

1. normal `StartSequence`は必須設定不足時に`NotConfigured`。
2. 同一条件で`ForceStartSequence`はCommandReceiveからLiftoffDetectionへ遷移する。
3. Forceしてもinvalid Health/config/calibrationがvalidへ変化しない。
4. SSC/LPS unavailableでForceしてもControl gateが偽成立しない。
5. Force後も+17秒deployment fallbackが動作する。
6. Force後も+25秒absolute cutoffが最優先で動作する。
7. Calibration/actuator/storage競合中はForceでも`Busy`。
8. CommandReceive以外ではForceを`InvalidState`で拒否する。
9. malformed args / transaction ID 0は通常protocol規則どおり拒否する。
10. Force開始時のprovisional fallback使用箇所がlogへ残る。
11. Ground GUIで二段階確認なしにForce commandを送信しない。
12. `Accepted`後に最終`Completed/Failed`を追跡する。
13. normal startとforced startをflight後のMission/Ground logから一意に区別できる。

## 11. 飛行認定との関係

`ForceStartSequence`が存在すること、またはForceでシーケンスが完走したことをflight qualificationの根拠にしてはならない。

Forceは、operatorが既知・未知のriskを受容してでもsequence開始を優先するための運用上の非常口である。通常flight readinessの検証、`TODO(HW_TEST)` / `TODO(SIMULATION)`の解消、審査書要求への適合確認は別途必要である。
