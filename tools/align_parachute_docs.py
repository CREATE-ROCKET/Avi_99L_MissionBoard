from pathlib import Path
import re

root = Path("docs")
latest_name = "08e_20260816_パラシュート簡素化"
latest_link = f"[[{latest_name}|08e パラシュート簡素化仕様]]"
notice = (
    f"> **2026-08-16更新:** パラシュートのOpen/Close、STS3215 Hold、NVS、readiness、"
    f"廃止commandについては{latest_link}を優先する。本書に残る絶対角endpoint、NVS、"
    "Free、Open retryの旧記述は当該範囲では廃止。\n\n"
)

for name in [
    "00_共通仕様.md",
    "01_コマンド受信モード.md",
    "02_飛行シーケンス.md",
    "04_通信仕様.md",
    "04c_ForceStartSequence詳細.md",
    "05_実装仕様.md",
    "06_初期実装・検証仕様.md",
    "07_実装前最終確認・シミュレーション仕様.md",
    "08b_20260816実装判断.md",
]:
    path = root / name
    text = path.read_text()
    if latest_link not in text:
        first_nl = text.find("\n")
        if first_nl < 0:
            raise RuntimeError(f"title missing: {name}")
        text = text[: first_nl + 1] + "\n" + notice + text[first_nl + 1 :]
    path.write_text(text)

# CommandReceive一次仕様を新契約へ直接更新。
path = root / "01_コマンド受信モード.md"
s = path.read_text()
s = s.replace("7項目", "5項目")
s = s.replace("7 bit missing mask", "5 bit missing mask")
s = s.replace(
    "actuatorの再接続時に初期Holdへ勝手に戻さず、現在要求されている`FinMode` / `ParaMode`を維持する。特に`ActuatorEmergencyStop`後はrequested modeがFreeであるため、deviceが再接続されても自動HoldしてEmergency Stopの効果を暗黙解除しない。専用解除commandは不要で、その後の通常commandがrequested modeを変更する。",
    "actuatorの再接続時は現在の安全要求を維持する。動翼はrequested `FinMode`を維持する。パラシュートはFreeを使用せず、`ActuatorEmergencyStop`後を含め、電源遮断済みでなければ現在位置Holdをbest effortで再確立する。",
)
old_commands = """- `StartSequence`
- `ForceStartSequence`
- `FinFree`
- `SetFinZero`
- `StartFinZeroHold`
- `FinMoveRelative`
- `ParaFree`
- `ParaHold`
- `ParaMoveRelative`
- `SetParaOpen`
- `SetParaClose`
- `ParaOpen`
- `ParaClose`
- `RunPreflightCalibration`
- `ExportFlashLogToSdAndErase`"""
new_commands = """- `StartSequence`
- `ForceStartSequence`
- `FinFree`
- `SetFinZero`
- `StartFinZeroHold`
- `FinMoveRelative`
- `ParaOpen`
- `ParaClose`
- `RunPreflightCalibration`
- `ExportFlashLogToSdAndErase`

パラシュートの`0x20..0x24`は旧wire codeとして予約するが、Mission Boardは常に`Rejected / NotSupported`とし、副作用を発生させない。"""
if old_commands in s:
    s = s.replace(old_commands, new_commands, 1)
s = s.replace(
    "受理した場合は実行中の動翼・パラシュート操作へ割り込み、動翼とパラシュートをどちらも手で動かせるFree状態へ移行する。\n\nこの操作は専用Emergency latchを追加するものではない。解除commandは設けず、requested Fin/Para modeをFreeとして維持し、その後の通常commandにより変更する。",
    "受理した場合は実行中の動翼・パラシュート操作へ割り込む。動翼は既定のEmergency安全状態であるFree/Coastへ移行する。パラシュートはFreeまたはpower-offへ移行せず、電源遮断済みでなければ現在位置Holdを要求する。\n\nこの操作は専用Emergency latchを追加するものではない。動翼はその後の通常commandでrequested modeを変更できる。パラシュートは`ParaOpen` / `ParaClose`を受理するまでHoldを維持し、飛行中は離床+25秒absolute cutoffまでHold/Torque ONを維持する。",
)
section11 = """## 11. パラシュート試験操作

CommandReceiveで使用できるパラシュート駆動commandは`ParaOpen`と`ParaClose`だけとする。

- `ParaOpen (0x25)`: 現在位置から物理的な反時計回りへ130度相対移動し、到達後は現在位置Holdへ戻る。
- `ParaClose (0x26)`: 現在位置から物理的な時計回りへ130度相対移動し、到達後は現在位置Holdへ戻る。
- args0..5はすべて0とし、角度・方向をwireから指定しない。
- `ParaFree`、`ParaHold`、`ParaMoveRelative`、`SetParaOpen`、`SetParaClose`は廃止し、常に`NotSupported`とする。
- Open/Close絶対角をNVSへ保存しない。Open/Close configured flagも持たない。
- relative move発行前にHold/Torque ONを確立し、move後もHoldを維持する。
- command実行有無が曖昧な場合、同じ130度moveを自動再送しない。

詳細は""" + latest_link + """に従う。
"""
s, n = re.subn(r"## 11\. パラシュート試験操作\n.*?(?=\n## 12\.)", section11.rstrip(), s, count=1, flags=re.S)
if n not in (0, 1):
    raise RuntimeError(f"01 section11 replacements={n}")
path.write_text(s)

# Flight sequenceのOpen処理を固定相対one-shotへ更新。
path = root / "02_飛行シーケンス.md"
s = path.read_text()
section9 = """## 9. パラシュートOpen処理

Descent移行時はNVS endpointや絶対角targetを参照せず、flight epochごとに**反時計回り130度の相対moveを1回だけ**要求する。

- Open move直前にSTS3215の現在位置Holdを成立させ、Torque ONを保証する。
- `moveRelativeDegrees(-130 deg)`相当を1回だけ発行する。実機方向が反時計回りとなる固定符号を使用する。
- 正常にmove停止を確認した後は、その位置をHoldする。
- UART response timeout等でcommand実行有無が曖昧な場合、二重130度移動を避けるため同一flight epochで自動再送しない。
- move失敗、確認timeout、Hold失敗はfailureとしてtelemetry/logへ残すが、離床+25秒absolute cutoff clockを止めない。
- LPS、SSC、CAN、LoRa、logging故障は離床+17秒timer fallbackの成立条件にしない。
- Open完了後も離床+25秒までHold/Torque ONを維持する。

飛行中reset後、checkpoint上ですでに`deployment_started=true`なら、reset前に130度moveが実行済みか一意に判定できないためOpen相対moveを再発行しない。利用可能なら現在位置Holdだけを再確立し、+25秒absolute cutoffを継続する。
"""
s, n = re.subn(r"## 9\. パラシュートOpen処理\n.*?(?=\n## 10\.)", section9.rstrip(), s, count=1, flags=re.S)
if n not in (0, 1):
    raise RuntimeError(f"02 section9 replacements={n}")
path.write_text(s)

# 通信仕様のcommand表と旧endpoint semanticsを更新。
path = root / "04_通信仕様.md"
s = path.read_text()
s = s.replace("7項目", "5項目")
for code, name in [("0x20", "ParaFree"), ("0x21", "ParaHold"), ("0x22", "ParaMoveRelative"), ("0x23", "SetParaOpen"), ("0x24", "SetParaClose")]:
    s = re.sub(rf"\| `{code}` \| `{name}` \|[^\n]*", f"| `{code}` | reserved: retired `{name}` (`NotSupported`) | args ignored |", s)
anchor = "`ParaOpen` / `ParaClose`"
if "固定相対移動" not in s:
    insert_at = s.find("## 7.")
    paragraph = (
        "\n`0x20..0x24`はdeprecated reserved commandとし、Mission Boardは引数/stateに依存せず副作用なく`Rejected / NotSupported`を返す。"
        "`ParaOpen (0x25)` / `ParaClose (0x26)`はargs0..5をすべて0とし、Openは反時計回り130度、Closeは時計回り130度の固定相対移動とする。"
        "Open/Close endpointをNVSへ保存しない。詳細は" + latest_link + "に従う。\n\n"
    )
    if insert_at >= 0:
        s = s[:insert_at] + paragraph + s[insert_at:]
path.write_text(s)

# ForceStart詳細は旧endpoint設計を残さず簡潔な最終契約へ置換。
path = root / "04c_ForceStartSequence詳細.md"
path.write_text("""# 99L ForceStartSequence・パラシュート簡素化 詳細仕様

本書は""" + latest_link + """を前提とする。Open/Close絶対角endpoint、NVS endpoint、flight parachute snapshot、`ParaFree`、自動Open retryを前提とする旧記述は廃止する。

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
""")

# 飛行前チェックの旧endpoint作業を置換。
path = root / "08b_20260816実装判断.md"
s = path.read_text()
s = s.replace(
    "- パラシュートOpen/Close endpointを実機で設定し、Open方向・到達・Free/torque-offと離床+25秒absolute cutoffを確認する。",
    "- パラシュート`ParaOpen`が反時計回り130度、`ParaClose`が時計回り130度で動作し、移動後Holdを維持することを実機確認する。endpoint/NVS設定は行わない。",
)
s = s.replace(
    "- Emergency commandが動翼/パラシュートを期待する安全状態へ移すこと。",
    "- `ActuatorEmergencyStop`で動翼は既定Emergency安全状態へ移り、パラシュートはFree/power-offせず現在位置Holdを維持すること。",
)
path.write_text(s)

# MissionBoard repo内の最終優先仕様。
(root / f"{latest_name}.md").write_text("""# 99L Mission Board パラシュート簡素化仕様 2026-08-16

本書はNatsu-B/Vaultの2026-08-16パラシュート簡素化仕様をMissionBoard repo側へ同期した要約である。パラシュート関連で既存docsと矛盾する場合、本書を優先する。

## 1. Generic command

- `ParaOpen (0x25)`: 反時計回り130度の固定相対move後にHold
- `ParaClose (0x26)`: 時計回り130度の固定相対move後にHold
- args0..5は全0
- `0x20..0x24`は常に`Rejected / NotSupported`、副作用なし

## 2. STS3215

- step/relative modeを使用する。
- relative move前に現在位置Holdを成立させ、Torque ONを保証する。
- command実行有無が曖昧な場合、同じ130度moveを自動再送しない。
- Open/Close完了後もHoldを維持する。
- 通常flight pathで`disableTorque()`/Freeを使用しない。
- `ActuatorEmergencyStop`でもパラシュートはHoldを維持する。
- MissionBoardの`lib/Avi_ESP_Libs`は`43a185ab2a570d4e1b889a4a534984fd19b2194f`へ固定する。

## 3. 自動deployment

- pressure apexは離床+10秒以降にOpen条件として使用可能。
- pressure条件がなくても離床+17秒でtimer fallback Open。
- +17秒fallbackをCAN/LoRa/LPS/SSC/logging故障に依存させない。
- relative Openはflight epochごとに1回だけ発行する。
- `deployment_started=true`をreset復元した場合はrelative Openを再発行せずHoldだけを再確立する。
- 離床+25秒で物理的にパラシュート電源OFF。cutoff直前までHold/Torque ON。

## 4. persistence / readiness

Open/Close endpoint用NVS、blob/CRC、configured flag、flight snapshot、RTC endpoint checkpointを廃止する。

通常Startのreadinessは5 bitとする。

| bit | 項目 |
|---:|---|
| 0 | `FinZeroConfigured` |
| 1 | `MotorProfileValid` |
| 2 | `GyroBiasValid` |
| 3 | `GravityReferenceValid` |
| 4 | `SscZeroValid` |

`ForceStartSequence`がbypassできるのはこの5項目の`NotConfigured` gateだけである。

## 5. 必須実機確認

1. boot後に現在位置Holdへ入る。
2. `ParaOpen`が物理的に反時計回り130度で動き、その位置をHoldする。
3. `ParaClose`が物理的に時計回り130度で動き、その位置をHoldする。
4. `0x20..0x24`が`NotSupported`で出力を変えない。
5. `ActuatorEmergencyStop`でもパラシュートHoldを維持する。
6. +17秒timer fallbackが通信・AirData故障と独立して進む。
7. +25秒で物理電源がOFFになる。

Open/Closeの固定符号は実機機構で最終確認すること。現在の実装は`Direction::normal`でOpen=-130度、Close=+130度としている。
""")

# 明らかな一次仕様の旧表現だけは残さない。
checks = {
    "01_コマンド受信モード.md": ["7 bit missing mask", "動翼とパラシュートをどちらも手で動かせるFree状態"],
    "04c_ForceStartSequence詳細.md": ["ParachuteOpenConfigured", "SetParaOpen", "SetParaClose"],
}
for name, forbidden_list in checks.items():
    text = (root / name).read_text()
    for forbidden in forbidden_list:
        if forbidden in text:
            raise RuntimeError(f"{name}: obsolete wording remains: {forbidden}")
