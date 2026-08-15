# 99L ForceStartSequence・パラシュート Stage 2 確定仕様

## 1. 優先順位

本仕様は、パラシュートの絶対角・optional・flight snapshotを実装したStage 1以後の決定である。

Vault内および本repository内の旧記述と矛盾する場合、本仕様を優先する。Stage 1で使用していたOpen/Close間の保存済み方向関係や、Force時のprovisional endpoint生成は使用しない。

## 2. Open/Close endpoint

Open/Closeは1回転内の絶対角であり、それぞれ独立したoptionalとして保持する。

- NVS keyなしは未設定とする。
- CRC、schema、size、endpoint、reserved、count範囲等が不正なendpointも、運用上は未設定と同じ`nullopt`として扱う。
- 破損理由はHealth、MissionEvent、内部logへ残す。
- 破損値を剰余演算、clamp、既定角で補正しない。
- ForceStart時にmissing/corruptをvalidへ偽装しない。

通常StartはOpen/Closeの両方を要求する。ForceStartではOpen/Closeを独立optionalのままflight snapshotへfreezeできる。

## 3. half-turn判定

half-turnは、保存済みOpenとCloseの相互関係ではなく、実際に移動するときのfreshな現在角とtargetとの関係で判定する。

- OpenとCloseが2048 count離れていても、それ自体をStart拒否理由にしない。
- Open動作直前に`current -> open`を検証する。
- Close動作直前に`current -> close`を検証する。
- retry時もfresh currentから同じsnapshot targetへの変位を再計算する。
- 2048 countちょうどなら任意方向へ動かさずfailureとする。
- 通常Startでは、fresh currentを取得できた場合に`current -> open`を事前検証する。ただしdeployment時にも再検証する。
- ForceStartはSTS/currentが取得不能でも遷移できる。ForceStart時点のhalf-turn判定は必須とせず、実際のdeployment時に判定する。

## 4. StartSequenceのcommand phase

通常StartおよびForceStartは、protocol/state/Busy/resource条件を通過した後に`Accepted`を返し、ParachuteTaskで非同期preparationを実行する。

通常StartでOpen/Closeを含む7項目のpreflight不足が判明した場合は、terminal resultを次とする。

- phase: `Failed`
- reason: `NotConfigured`
- detail: 7項目のpreflight missing bitmask

Groundはterminal phaseが`Rejected`または`Failed`のどちらであっても、reasonが`NotConfigured`かつdetail bit0..6が非0の場合に限りForce操作を提示できる。

path half-turnは7項目missing maskへ混ぜず、`SafetyInterlock`と専用detailで表す。

## 5. ForceStartSequence

`ForceStartSequence`はcommand code `0x04`、args0..5全て0、CommandReceiveのみ受理する。

Forceがbypassするのは、通常Startの次の7項目のpreflight `NotConfigured` gateだけである。

- bit0: `FinZeroConfigured`
- bit1: `ParachuteOpenConfigured`
- bit2: `ParachuteCloseConfigured`
- bit3: `MotorProfileValid`
- bit4: `GyroBiasValid`
- bit5: `GravityReferenceValid`
- bit6: `SscZeroValid`

Forceでもprotocol validity、MissionState、Busy、queue/mutex/task等のruntime invariant、resources preallocationはbypassしない。

### 5.1 STS unavailable

ForceStartでは、STS初期化、fresh angle取得、現在位置Holdをbest effortで試す。

次の失敗があっても、他の非bypass条件が成立していればLiftoffDetectionへ遷移する。

- STS未接続
- STS initialization failure
- fresh angle read timeout/failure
- current Hold failure

失敗状態を正常へ偽装しない。

- STS HealthはUnavailable/Faultのまま
- HoldEstablishedはfalseのまま
- current angleを0等で生成しない
- Open/Close endpointを生成しない

ParachuteTaskはpower-on要求とboundedな再接続を継続し、STSが利用可能になった時点でfresh currentを取得して現在位置Holdを試す。

### 5.2 snapshot

ForceStartのflight snapshotはOpen/Closeを独立optionalとして保持する。

- Open valid / Close valid: 両方をfreeze
- Open valid / Close missing/corrupt: Openだけをfreeze
- Open missing/corrupt / Close valid: Closeだけをfreeze
- 両方missing/corrupt: 両方`nullopt`

Openだけ存在する場合でも、deployment時にfresh current angleを取得でき、`current -> open`がexact half-turnでなければOpen動作を行ってよい。Closeはdeploymentの必須条件ではない。

### 5.3 completed detail

`ForceStartSequence`のterminal `Completed.detail`には、Force受理時に実際にbypassした7項目missing maskを格納する。Accepted時のdetailは0でよい。

Force preparation中にruntime failureをterminal failureとして返す場合は、missing maskではなく該当failure detailを優先する。ただしSTS unavailableだけではForceを失敗させない。

## 6. forced_start latch

flight attemptごとに次を保持する。

- normal Start成功: `forced_start=false`
- ForceStart成功: `forced_start=true`
- `CancelSequence`: clear
- 次のnormal/Force Start: 新しい値で上書き
- `LiftoffDetectionEmergencyStop`: 同じflight attempt由来として維持
- software/watchdog reset recovery: RTC checkpointから復元
- checkpointが検証不能な場合: trueを推測しない

Force受理時の7項目snapshot/missing maskもRTC/full-rate logへ保持する。

## 7. 電源・Hold方針

起動直後は従来どおりGPIOを安全化してパラシュート電源をOFFにする。

Task/driver/queue初期化後の通常CommandReceiveでは、明示的なEmergencyまたはpower-off状態でない限り、STS電源を可能な限りONに維持する。

- STSが利用可能なら現在位置Holdを維持する。
- 未接続/故障中もpower-on要求とboundedな再初期化を継続する。
- `ParaFree`は原則としてtorqueをOFFにするが、通常操作では電源まで落とす必要はない。
- `ActuatorEmergencyStop`および絶対cutoff等、既存の安全処理は電源OFFを許可する。

LiftoffDetection以降、離床+25秒より前はSTS電源を基本的にONとする。

- Hold要求中は現在位置Hold
- Open成功後はOpen位置Hold
- Open failure後は取得できた現在位置Hold
- STS unavailable中はrequested modeをHoldのまま維持し、Healthをfaultとして再接続を続ける
- 離床+25秒で状態にかかわらず絶対遮断

## 8. deployment

### 8.1 Openあり

flight snapshotにOpenが存在する場合:

1. fresh current angleを読む
2. `shortestParachuteDisplacement(current, open)`を計算
3. exact half-turnなら動かさない
4. path validならshortest pathでOpen指令
5. retryでも同じsnapshot Openを使う
6. retryのたびにfresh currentから変位を再計算
7. NVS/CommandReceive active configは参照しない

### 8.2 Openなし

Openがmissing/corruptの場合:

- MissionStateはDescentへ遷移する
- Open moveを発行しない
- `ParachuteOpenNotConfigured`相当のfailureを一度記録
- 現在位置Holdを継続
- 電源を早期遮断しない
- 離床+25秒で絶対遮断

### 8.3 current angleなし

Openは存在するがfresh current angleを取得できない場合:

- target angleから移動方向を推測しない
- Open moveを発行しない
- STS再接続とfresh angle取得を継続
- Open再試行期限内に取得できればOpenを試す
- 取得不能のまま期限を迎えたらfailureを記録し、requested Holdへ戻る
- 電源は離床+25秒まで維持する

### 8.4 5秒deadline

約5秒のglobal Open retry deadlineは維持する。ただし、このdeadlineは「Open move/retryを終了する期限」であり、「パラシュート電源を切る期限」ではない。

5秒経過時:

- 新しいOpen retryを停止
- retry deadlineを延長しない
- failureを記録
- fresh currentを取得できる場合はその位置をHold
- STS電源はONのまま
- 離床+25秒でのみ絶対遮断

Open成功時もOpen位置をHoldし、離床+25秒まで電源を維持する。

## 9. persistence load状態

次を区別する。

- key missing: endpoint `nullopt`、通常の未設定
- blob corrupt: endpoint `nullopt`、persistence faultを記録
- load in progress: Start/ForceStartは`Busy`
- NVS owner/queue/init/open/read自体が利用不能: `PersistenceError`または`InternalError`
- load完了状態が不明: 未設定へ丸めず`Busy`/`InternalError`

Forceはmissing/corrupt endpointをbypassできるが、load未完了やNVS ownerのruntime failureを単なる未設定としてbypassしない。

## 10. readiness snapshot

7項目と`resources_preallocated`は、一貫した`PreflightReadinessSnapshot`として取得する。

少なくとも以下を保持する。

- generation
- captured_at_us
- 7項目bool
- missing mask
- resources_preallocated

Force/normal Startの判定とlogには同じsnapshotを使用する。

## 11. failure/event

内部には最低限次を区別する。

- `open_not_configured`
- `current_angle_unavailable`
- `ambiguous_half_turn`
- `move_command_failed`
- `retry_exhausted`
- `hold_failed`
- `persistence_corrupt`

自律deployment failureはCommandResultだけでは表せないため、full-rate logとMissionEvent/Descent statusの既存予約領域を使用して一度だけ記録する。既存wire layoutは変更せず、予約bit/detailの意味を通信仕様へ明記する。

## 12. 必須試験

- corrupt endpointがoperationally `nullopt`になる
- corrupt理由はpersistence faultとして残る
- OpenのみのForce snapshot
- CloseのみのForce snapshot
- 両方なしのForce snapshot
- Openのみでもfresh currentがあればdeployment可能
- Open/Close相互が2048 countでもStartを拒否しない
- actual current/targetが2048 countならmoveしない
- normal Startはcurrent/Open half-turnを拒否
- ForceStartはSTS unavailableでもLiftoffDetectionへ遷移
- STS unavailableをvalidへ偽装しない
- STS復旧後に現在位置Holdへ入る
- OpenなしのdeploymentでHold継続
- Open成功後も+25秒までHold/power-on
- retry exhausted後も+25秒までHold/power-on
- +25秒で必ず絶対遮断
- 5秒deadlineがretryで延長されない
- Force Completed.detailが実際のmissing mask
- Cancelで`forced_start` clear
- Emergency rollbackで`forced_start`維持
- software/watchdog resetで`forced_start`とoptional snapshotを復元
- NVS load中のForceはBusy
- NVS runtime failureをmissingとしてbypassしない
