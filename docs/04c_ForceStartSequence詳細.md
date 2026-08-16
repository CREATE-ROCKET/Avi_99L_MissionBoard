# 99L ForceStartSequence・パラシュート簡素化 詳細仕様

本書は[[08e_20260816_パラシュート簡素化|08e パラシュート簡素化仕様]]を前提とする。Open/Close絶対角endpoint、NVS endpoint、flight parachute snapshot、`ParaFree`、自動Open retryを前提とする旧記述は廃止する。

## 1. Start / ForceStart

通常`StartSequence`のpreflight readinessは次の5 bitとする。

| bit | 項目 |
|---:|---|
| 0 | `FinZeroConfigured` |
| 1 | `MotorProfileValid` |
| 2 | `GyroBiasValid` |
| 3 | `GravityReferenceValid` |
| 4 | `SscZeroValid` |

`ForceStartSequence (0x04)`はこの5項目の`NotConfigured` gateだけをbypassする。protocol validity、MissionState、Busy、resource preallocation、fin overtravel等の安全interlockはbypassしない。terminal `Completed.detail`には実際にbypassした5 bit missing maskを格納する。

STS3215がStart時にUnavailableでもendpoint不足としてStartを拒否しない。ParachuteTaskはbounded reconnectを継続し、利用可能になった時点で現在位置Holdをbest effortで確立する。

## 2. パラシュートcommand

CommandReceiveで有効なパラシュートgeneric commandは次の2つだけとする。

- `ParaOpen (0x25)`: 反時計回り130度の固定相対move
- `ParaClose (0x26)`: 時計回り130度の固定相対move

args0..5はすべて0とする。`0x20..0x24`はdeprecated reserved codeであり、Mission Boardは常に副作用なしの`Rejected / NotSupported`とする。

## 3. STS3215 Hold / relative move

1. relative move前に現在位置Holdを成立させ、Torque ONを保証する。
2. 固定130度relative moveを1回だけ発行する。
3. move停止確認後は現在位置Holdへ戻る。
4. UART response timeout等でcommand実行有無が曖昧な場合、同じ130度moveを自動再送しない。
5. Open/Close target計算にNVS値や絶対角を使用しない。現在角はtelemetry観測用にのみ使用してよい。

STS runtime Direction設定は既知状態へ固定し、実機でOpenが反時計回り、Closeが時計回りとなる符号を飛行前に確認する。

## 4. 自動deployment / cutoff

- pressure apex条件は離床+10秒以降に使用してよい。
- pressure条件が成立しなくても離床+17秒でtimer fallbackを成立させる。
- CAN、LoRa、LPS、SSC、loggingの故障を+17秒timer fallbackの成立条件にしない。
- deployment requestはflight epochごとに1回だけrelative Openへ変換する。
- move command失敗/確認timeoutでも130度relative moveを自動再送しない。
- Open完了後も離床+25秒までHold/Torque ONを維持する。
- +25秒では事前`disableTorque()`なしでパラシュート電源を物理的にOFFする。
- `ActuatorEmergencyStop`では動翼は既定Emergency処理へ移し、パラシュートはFree/power-offせずHoldを要求する。

飛行中resetで`deployment_started=true`を復元した場合はOpen relative moveを再発行せず、現在位置Holdだけを再確立して+25秒cutoffを継続する。

## 5. persistence / telemetry

パラシュートOpen/Close endpoint用NVS key/blob/CRC、configured flag、flight snapshot、RTC endpoint checkpointを使用しない。旧値がFlashに残っていても読み込まない。

既存wire frame layoutは維持し、旧endpoint persistence用bitはreserved 0へ戻してよい。`ParaMode::opening_or_retrying`というwire名を残しても、自動retryを意味しない。
