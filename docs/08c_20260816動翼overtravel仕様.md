# 99L Mission Board 動翼overtravel仕様

## 1. 位置付け

本仕様は2026-08-16時点の動翼角overtravel判定と復帰条件を定義する。既存文書にある「飛行時±15 degを絶対限界とし、超過を不可逆な異常とする」記述と矛盾する場合、runtimeのovertravel fault判定・復帰条件については本仕様を優先する。

ただし、構造審査で使用しているstopper角、characterization試験の安全limit、通常の動翼指令可能範囲、CAN telemetryの量子化rangeを本仕様だけで20 degへ拡張してはならない。これらはovertravel fault閾値とは別の値として管理する。

## 2. overtravel fault閾値

動翼の論理角`fin_angle_rad`について、暫定fault閾値を絶対値20 degとする。

- `|fin_angle| <= 20 deg`: overtravel faultではない。
- `|fin_angle| > 20 deg`: `FinOvertravelFault`を成立させる。
- 正負両方向を同じ条件で判定する。
- NaN / Inf、encoder invalid、zero未設定は「20 deg以内へ戻った」とみなさない。これらは既存の別fault/invalid条件で扱う。

```text
TODO(HW_TEST): 実機のたわみ、backlash、stopper位置、encoder zero誤差を含め、20 degの最終fault閾値を確定する。
```

## 3. 通常指令limitとの分離

`FinOvertravelFault`の20 deg閾値は、Roll Controller、ZeroHold、`FinMoveRelative`等が指令してよい角度範囲を意味しない。

通常の`FinSoftwareLimits`は独立したcommand/drive limitとして保持し、20 degへ自動拡張しない。TorqueMapperは通常software limitの外側でさらに外向きとなるtorqueを禁止し、内向きに安全側へ戻すtorqueの扱いは既存規則に従う。

characterization buildの試験limitも本fault閾値とは独立とし、本仕様を根拠に拡張しない。

## 4. 飛行中の扱い

`LiftoffDetection`、`EngineBurn`、`Control`、`Descent`等の飛行sequence中に`FinOvertravelFault`が成立した場合、動翼制御に必要な状態を失ったものとして扱う。

- Control中ならControlを停止し、安全側のBrakeへ移行する。
- そのflight epochでは`control_reentry_inhibited=true`とし、20 deg以内へ戻ってもControlへ再entryしない。
- fault成立後にencoder値が20 deg以内へ戻ったことだけを理由に、飛行中のno-reentry latchを解除しない。
- パラシュート開放等、動翼以外の自律sequenceは継続する。

## 5. CommandReceiveでの復帰

CommandReceiveでは`FinOvertravelFault`を永久latchとしない。

以下のいずれかでovertravel faultだけを解除できる。

1. encoderがvalid/freshでzero設定済みであり、有限な`fin_angle`が`|fin_angle| <= 20 deg`へ戻った場合。
2. `SetFinZero`が正常完了し、現在位置を新しい論理0 degとして設定した場合。

`SetFinZero`失敗時にはfaultを解除しない。encoder transport fault、timestamp fault、motor fault等の別faultをovertravel復帰処理で解除してはならない。

fault中でもCommandReceiveの`FinFree`は使用可能とし、手動で安全範囲へ戻せるようにする。`StartFinZeroHold`や`FinMoveRelative`等の駆動commandは、overtravel faultが解除されるまで開始しない。

`SetFinZero`はoperatorが現在位置を新しい基準として明示的に採用する操作であるため、overtravel中でもencoderが利用可能なら実行を許可する。正常完了した場合だけovertravel faultを解除する。

## 6. telemetry

現行のFinAngle telemetryは8 bit、0.125 deg/LSBで`-15..+15 deg`をnumeric rangeとして使用しているため、本変更だけではwire formatを変更しない。

- runtime内部の20 deg判定は量子化前の`fin_angle_rad`で行う。
- `|fin_angle| > 15 deg`では現行FinAngle telemetryがerror codeとなってもよい。
- 15..20 degの実角を地上局へ数値表示する必要が生じた場合は、protocol revisionとして別途range/bit幅を変更する。

telemetryが15 deg超を数値表現できないことと、20 deg overtravel fault判定を混同しない。

## 7. 必須テスト

最低限、以下をhost testで固定する。

- `+20.0 deg` / `-20.0 deg`はovertravel faultにならない。
- `+20.1 deg` / `-20.1 deg`はovertravel faultになる。
- 飛行中のfault成立でControlを停止し、そのflight epochでは20 deg以内へ戻ってもControl再entryしない。
- CommandReceiveではvalid/freshな角度が20 deg以内へ戻るとfaultを解除する。
- CommandReceiveの`SetFinZero`成功でfaultを解除する。
- `SetFinZero`失敗、encoder invalid/stale、NaN/Infではfaultを解除しない。
- overtravel faultの解除で別のdevice/transport faultを解除しない。
- fault中も`FinFree`を受理できる。
- fault中の`StartFinZeroHold` / `FinMoveRelative`は拒否する。
