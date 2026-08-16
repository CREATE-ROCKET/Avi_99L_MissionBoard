# 99L MotorProfile ビルド時固定詳細仕様

## 1. 目的と優先順位

本書はMission Boardの飛行用`MotorProfile`選択方法を正式に定義する。

MotorProfileは**runtime設定ではなくfirmware build時に固定する**。地上局、CAN、LoRa、NVS、内部Flashその他のruntime経路からactive MotorProfileを変更してはならない。

本書のMotorProfile選択・保存・通信commandに関する記述は、以下と矛盾する場合に優先する。

- `00_共通仕様.md`
- `01_コマンド受信モード.md`
- `03_制御仕様.md`
- `04_通信仕様.md`
- `04c_ForceStartSequence詳細.md`
- `05_実装仕様.md`
- `06_初期実装・検証仕様.md`
- `07_実装前最終確認・シミュレーション仕様.md`
- `08_実装仮定・検証待ち一覧.md`

特に過去記述に存在する「MotorProfileをCommandReceiveで選択する」「MotorProfileをNVSへ永続化する」「保存成功後にactive profileを更新する」という仕様は本書により廃止する。

## 2. Build-time selector

飛行firmwareはcompile definition `AVI_99L_MOTOR_PROFILE_ID`を必須とする。

sourceは少なくとも次のfail-closed構造を持つ。

```cpp
#ifndef AVI_99L_MOTOR_PROFILE_ID
#error "AVI_99L_MOTOR_PROFILE_ID must be selected at build time"
#endif

#if AVI_99L_MOTOR_PROFILE_ID == 1
constexpr const auto &kActiveMotorProfile = board::kFlightMotorA;
#elif AVI_99L_MOTOR_PROFILE_ID == 2
constexpr const auto &kActiveMotorProfile = board::kSpareMotorB;
#else
#error "Unsupported AVI_99L_MOTOR_PROFILE_ID"
#endif
```

実際のsymbol配置は実装都合で変更してよいが、以下のsemanticを変えてはならない。

1. macro未指定のbuildは失敗する。
2. 未知のprofile IDを指定したbuildは失敗する。
3. firmware binaryには一つのactive MotorProfileだけをcompile-timeで選択する。
4. runtime中にactive profileへのpointer/reference/IDを別profileへ変更しない。

PlatformIOではenvironmentまたはbuild flagから明示的にIDを指定する。repositoryのdefaultとして暗黙にID=1を注入して「未指定でもbuild成功」としてはならない。

## 3. Profile catalogとID

各MotorProfileはstableなprofile IDを持つ。

現時点のcatalog:

| ID | Symbol | Status |
|---:|---|---|
| 1 | `kFlightMotorA` | parameters候補あり。最終qualificationは各`TODO(HW_TEST)`に従う |
| 2 | `kSpareMotorB` | 未同定/未qualificationの場合は`MotorProfileValid=false` |

profile IDは一度割り当てた意味を別motorへ再利用しない。新しいmotor個体を追加する場合は新しいIDとprofile entryを追加する。

profileには少なくとも以下を含める。

- profile ID
- motor electrical parameters
- MotorPolarity
- drivetrain efficiency
- current / output torque limits
- ZeroHoldに必要なmotor依存parameter
- 必要ならControl/TorqueMapper用のmotor依存parameter
- qualification / parameters-valid state

## 4. Runtime immutable

boot後のactive MotorProfileはimmutableとする。

禁止事項:

- NVSからMotorProfile IDを読み込みactive profileを変更する
- NVS/Flashへactive MotorProfile IDを書き込む
- CAN/LoRa/USB commandでprofileを変更する
- Ground GUIでprofile selectorを提供する
- flight中またはCommandReceive中に別profileへswitchする
- unknown/unselected profileを暗黙にID=1へfallbackする

MotorProfile変更が必要な場合は、`AVI_99L_MOTOR_PROFILE_ID`を変更してfirmwareを再buildし、Mission Boardへ再書込みする。

再書込み/reset後は通常のreset規則に従い`FinZeroConfigured=false`となるため、runtime profile変更専用のFin zero invalidation処理は不要である。

## 5. MotorProfileValid

`MotorProfileValid`は「profileが選択された」だけではtrueにしない。

少なくとも以下を満たす場合のみtrueとする。

- compile-time selectorが既知profileを選択している
- profileのrequired parametersがvalid
- MotorPolarityが有効
- profile固有のflight-critical limit/configurationが構造上valid
- 当該profileが必要とするqualification flagが有効

compile-timeでprofile entry自体が存在していても、個体同定やpolarity等が未確定なら`MotorProfileValid=false`としてよい。

未qualification profileを意図的に選択してbuildすることはbring-up/検証のため許容する。ただし通常`StartSequence`は`MotorProfileValid=false`を`Rejected / NotConfigured`として扱う。

## 6. ForceStartSequenceとの関係

`MotorProfileValid`は`ForceStartSequence`がbypass可能な7項目のbit3である。

通常Start:

```text
MotorProfileValid=false
  -> Rejected / NotConfigured
  -> CommandResult.detail bit3 = 1
```

ForceStartSequence:

- bit3を開始拒否理由から除外してLiftoffDetectionへの遷移を試みてよい。
- active profileを別profileへ変更しない。
- invalid profileをvalidへ書き換えない。
- missing parameterを暗黙値、0、前build値、NVS値で補わない。
- 後段Control/TorqueMapperが必要parameterを安全に使用できない場合は既存のunavailable/fail-safe handlingへ従う。

## 7. Generic command `0x32`

旧`SelectMotorProfile`は廃止する。

- `0x32`はprotocol互換性のためreserved codeとして保持する。
- 新しい別機能へ再利用しない。
- Ground/ComBoardは`0x32`を通常操作として生成しない。
- Mission Boardが`0x32`を受信した場合は副作用なく`CommandResult(Rejected, NotSupported)`を返す。
- Ground GUIの通常画面およびcommand matrixにMotorProfile変更操作を表示しない。

Developer raw consoleから旧`0x32`を送信されても、runtime profileを変更してはならない。

## 8. Telemetryとlogging

Groundからは少なくとも`MotorProfileValid`を確認可能とする。

可能ならMission full-rate/event logには以下も残す。

- compile-time `AVI_99L_MOTOR_PROFILE_ID`
- active profile ID
- profile validity
- profile/build識別情報

ただし既存wire packetの意味を変更してprofile IDを無理に詰め込まない。profile IDのwire telemetry追加が必要ならversioned/additiveなfieldとして別途定義する。

## 9. Build/test要件

最低限、次をCI/host testで確認する。

1. `AVI_99L_MOTOR_PROFILE_ID`未指定buildがcompile errorになる。
2. 未知ID指定buildがcompile errorになる。
3. ID=1指定時に`kFlightMotorA`だけがactive profileになる。
4. ID=2指定時に`kSpareMotorB`だけがactive profileになる。
5. unqualified profileを選んだ場合`MotorProfileValid=false`となる。
6. runtimeにprofile switch API/stateが存在しない。
7. NVS recordにactive MotorProfile selectionを持たない。
8. Generic command `0x32`が`Rejected / NotSupported`となり副作用がない。
9. Ground GUI/typed command catalogが`SelectMotorProfile`を生成しない。
10. normal Startでinvalid profileはNotConfigured detail bit3を立てる。
11. ForceStartSequenceはbit3をbypassしてもprofile validity自体を変更しない。

## 10. 運用

使用motorを交換した場合は、該当MotorProfile IDをbuild設定へ明示し、firmwareを再build・再書込みした上で、そのmotor用のbring-up/qualificationとFin zero設定をやり直す。

一つのbinaryを複数motor個体へ共用しruntime選択する運用は禁止する。build artifactと搭載motor個体の対応はflight記録から追跡可能にする。