from pathlib import Path


def replace_once(path: Path, old: str, new: str, label: str) -> None:
    text = path.read_text()
    if new in text:
        return
    count = text.count(old)
    if count != 1:
        raise SystemExit(f"{label}: target count={count}, expected=1")
    path.write_text(text.replace(old, new, 1))


header = Path("src/actuators/parachute_configuration.hpp")
replace_once(
    header,
    """constexpr double kParachuteDegreesPerCount =\n    360.0 / static_cast<double>(kParachuteCountsPerRevolution);\n\nclass AbsoluteParachuteAngle {\n""",
    """constexpr double kParachuteDegreesPerCount =\n    360.0 / static_cast<double>(kParachuteCountsPerRevolution);\n\n// STS3215 multi-turn position feedbackのbit15=sign、bit0..14=magnitudeを\n// 連続したsigned countへ変換する。保存済みendpointには使用しない。\n[[nodiscard]] constexpr int32_t\ndecodeStsSignedMagnitudePositionCount(uint16_t raw_count) {\n  const int32_t magnitude = static_cast<int32_t>(raw_count & 0x7FFFU);\n  return (raw_count & 0x8000U) != 0U ? -magnitude : magnitude;\n}\n\nclass AbsoluteParachuteAngle {\n""",
    "parachute signed position helper",
)
replace_once(
    header,
    """  // STS3215のcurrent position rawはbit15=sign、bit0..14=magnitudeであり、\n  // step/multi-turn動作では1回転を越える。機構の絶対角として使う際は\n  // signed positionを1回転内の0..4095へwrapする。\n  // 保存済みendpoint/checkpointの検証にはfromCanonicalCount()を使うこと。\n  [[nodiscard]] static constexpr std::optional<AbsoluteParachuteAngle>\n  fromCount(uint16_t raw_count) {\n    const int32_t magnitude = static_cast<int32_t>(raw_count & 0x7FFFU);\n    const int32_t signed_count =\n        (raw_count & 0x8000U) != 0U ? -magnitude : magnitude;\n    int32_t wrapped = signed_count % kParachuteCountsPerRevolution;\n""",
    """  // STS3215のposition+multi-turn current positionを1回転内の\n  // 0..4095へwrapする。保存済みendpoint/checkpointの検証には\n  // fromCanonicalCount()を使うこと。\n  [[nodiscard]] static constexpr std::optional<AbsoluteParachuteAngle>\n  fromCount(uint16_t raw_count) {\n    const int32_t signed_count =\n        decodeStsSignedMagnitudePositionCount(raw_count);\n    int32_t wrapped = signed_count % kParachuteCountsPerRevolution;\n""",
    "parachute wrapped position decoder",
)

runtime = Path("src/runtime/production_runtime.cpp")
replace_once(
    runtime,
    """  uint64_t last_position_read_attempt_us = 0;\n  uint64_t last_position_valid_us = 0;\n  bool power_enabled = false;\n""",
    """  uint64_t last_position_read_attempt_us = 0;\n  uint64_t last_position_valid_us = 0;\n  // ParachuteTaskだけが更新するfreshなSTS multi-turn物理位置。\n  // move commandは直前のreadCurrent成功後にだけこの値を使用する。\n  double last_position_unwrapped_degrees = 0.0;\n  bool power_enabled = false;\n""",
    "runtime unwrapped position state",
)
replace_once(
    runtime,
    """    if (!mode_prepared) {\n      last_initialization_error =\n          servo.verifyOperatingMode(STS3215::OperatingMode::step);\n      if (last_initialization_error != ESP_OK)\n        last_initialization_error = servo.configureStepMode(\n            STS3215::Persistence::volatile_only);\n      if (last_initialization_error == ESP_OK)\n        last_initialization_error = servo.setFeedbackMode(\n            STS3215::FeedbackMode::single_turn,\n            STS3215::Persistence::volatile_only);\n      if (last_initialization_error != ESP_OK)\n        return false;\n      mode_prepared =\n          servo.verifyOperatingMode(STS3215::OperatingMode::step) ==\n          ESP_OK;\n    }\n""",
    """    if (!mode_prepared) {\n      // Step modeでは停止後のcurrent positionが相対残量へ戻るfirmwareが\n      // あるため、productionでは絶対position + multi-turn feedbackを使う。\n      // configurePositionMode()は過去のStep設定で0になったposition limitも戻す。\n      last_initialization_error = servo.configurePositionMode(\n          STS3215::Persistence::volatile_only);\n      if (last_initialization_error == ESP_OK)\n        last_initialization_error = servo.setFeedbackMode(\n            STS3215::FeedbackMode::multi_turn,\n            STS3215::Persistence::volatile_only);\n      if (last_initialization_error != ESP_OK)\n        return false;\n      mode_prepared =\n          servo.verifyOperatingMode(STS3215::OperatingMode::position) ==\n          ESP_OK;\n    }\n""",
    "runtime STS mode initialization",
)
replace_once(
    runtime,
    """    const auto current =\n        actuators::AbsoluteParachuteAngle::fromCount(data.position);\n""",
    """    const int32_t unwrapped_count =\n        actuators::decodeStsSignedMagnitudePositionCount(data.position);\n    const auto current =\n        actuators::AbsoluteParachuteAngle::fromCount(data.position);\n""",
    "runtime STS position decode",
)
replace_once(
    runtime,
    """    angle = *current;\n    moving = data.moving;\n    const double degrees = static_cast<double>(current->count()) *\n                           actuators::kParachuteDegreesPerCount;\n""",
    """    angle = *current;\n    moving = data.moving;\n    last_position_unwrapped_degrees =\n        static_cast<double>(unwrapped_count) *\n        actuators::kParachuteDegreesPerCount;\n    const double degrees = static_cast<double>(current->count()) *\n                           actuators::kParachuteDegreesPerCount;\n""",
    "runtime publish unwrapped position",
)
replace_once(
    runtime,
    """  auto moveToAbsolute = [&](actuators::AbsoluteParachuteAngle current,\n                            actuators::AbsoluteParachuteAngle target) {\n    const auto displacement =\n        actuators::shortestParachuteDisplacement(current, target);\n    if (!displacement.valid())\n      return ESP_ERR_INVALID_ARG;\n    return servo.moveRelativeDegrees(\n        static_cast<float>(displacement.degrees()), motion());\n  };\n""",
    """  auto moveToAbsolute = [&](actuators::AbsoluteParachuteAngle current,\n                            actuators::AbsoluteParachuteAngle target) {\n    const auto displacement =\n        actuators::shortestParachuteDisplacement(current, target);\n    if (!displacement.valid())\n      return ESP_ERR_INVALID_ARG;\n    // wrapped currentから求めた最短変位だけをfreshなmulti-turn位置へ足す。\n    // これにより360度境界を跨いでも回転方向は常に180度未満になる。\n    const double target_unwrapped_degrees =\n        last_position_unwrapped_degrees + displacement.degrees();\n    return servo.moveAbsoluteDegrees(\n        static_cast<float>(target_unwrapped_degrees), motion());\n  };\n""",
    "runtime absolute endpoint move",
)
replace_once(
    runtime,
    """            const int16_t tenths = static_cast<int16_t>(raw);\n            const esp_err_t move = servo.moveRelativeDegrees(\n                static_cast<float>(tenths) * 0.1F, motion());\n""",
    """            const int16_t tenths = static_cast<int16_t>(raw);\n            const double target_unwrapped_degrees =\n                last_position_unwrapped_degrees +\n                static_cast<double>(tenths) * 0.1;\n            const esp_err_t move = servo.moveAbsoluteDegrees(\n                static_cast<float>(target_unwrapped_degrees), motion());\n""",
    "runtime ParaMoveRelative absolute target",
)

host_test = Path("host_test/parachute_configuration_tests.cpp")
replace_once(
    host_test,
    """  require(!AbsoluteParachuteAngle::fromCanonicalCount(4096).has_value(),\n          \"canonical count 4096 must remain invalid\");\n\n  const auto one_turn = AbsoluteParachuteAngle::fromCount(4096);\n""",
    """  require(!AbsoluteParachuteAngle::fromCanonicalCount(4096).has_value(),\n          \"canonical count 4096 must remain invalid\");\n  require(actuators::decodeStsSignedMagnitudePositionCount(0x0000U) == 0,\n          \"STS zero raw must decode to zero\");\n  require(actuators::decodeStsSignedMagnitudePositionCount(0x1000U) == 4096,\n          \"STS positive one-turn raw must remain unwrapped\");\n  require(actuators::decodeStsSignedMagnitudePositionCount(0x8001U) == -1,\n          \"STS negative one raw must decode signed magnitude\");\n  require(actuators::decodeStsSignedMagnitudePositionCount(0x9000U) == -4096,\n          \"STS negative one-turn raw must remain unwrapped\");\n\n  const auto one_turn = AbsoluteParachuteAngle::fromCount(4096);\n""",
    "host test STS unwrapped decoder",
)

for relative in [
    Path("docs/04c_ForceStartSequence詳細.md"),
]:
    replace_once(
        relative,
        """通常StartはOpen/Closeの両方を要求する。ForceStartではOpen/Closeを独立optionalのままflight snapshotへfreezeできる。\n\n## 3. half-turn判定\n""",
        """通常StartはOpen/Closeの両方を要求する。ForceStartではOpen/Closeを独立optionalのままflight snapshotへfreezeできる。\n\n### 2.1 STS3215 current/target座標系\n\nproductionのパラシュートサーボは`OperatingMode::position`と`FeedbackMode::multi_turn`を使用する。`OperatingMode::step`は相対移動command向けであり、停止後のcurrent position feedbackを物理絶対角として扱えないfirmwareがあるため、productionのcurrent angle sourceには使用しない。\n\n- STS3215のfresh current positionはsigned multi-turn countとして取得する。\n- GUI/CAN/LoRaへ送る`parachute angle`とOpen/Close endpoint比較では、fresh multi-turn countを1回転内`0..4095`へwrapした物理絶対角を使う。\n- `ParaMoveRelative`はfresh multi-turn currentに要求変位を加えたabsolute multi-turn targetを`position` modeへ送る。要求変位は従来どおり`abs(delta) < 180 deg`とする。\n- `ParaOpen`/`ParaClose`はwrapped currentとendpointから`shortestParachuteDisplacement`を求め、その`(-180, +180) deg`の変位をfresh multi-turn currentへ加えたabsolute multi-turn targetを送る。exact 180 degは従来どおり拒否する。\n- Holdはfresh current position自体をabsolute targetとして保持する。\n- driver/deviceのmulti-turn有効範囲を越えるtargetはalternate long pathへ置換せずfailureとする。\n- 電源再投入後に過去のturn countをNVSから推測しない。毎回STSのfresh position feedbackをsource of truthとし、永続化するOpen/Closeは1回転内絶対角だけとする。\n\n## 3. half-turn判定\n""",
        "Mission docs 04c absolute position mode",
    )

replace_once(
    Path("docs/04a_量子化・エラーコード詳細.md"),
    """## 15. parachute angle\n\n8 bit、1.5 deg/LSB。raw0..240=0..360 deg。\n""",
    """## 15. parachute angle\n\n本fieldはSTS3215を`OperatingMode::position` + `FeedbackMode::multi_turn`で取得したfresh physical current positionを、1回転内へwrapした絶対角である。Step modeの相対残量、targetまでの残差、commanded displacementを送ってはならない。\n\n8 bit、1.5 deg/LSB。raw0..240=0..360 deg。\n""",
    "Mission docs 04a parachute angle semantics",
)

print("MissionBoard parachute position-mode patch applied")
