from pathlib import Path


def replace_once(path: Path, old: str, new: str, label: str) -> None:
    text = path.read_text()
    if new in text:
        return
    count = text.count(old)
    if count != 1:
        raise SystemExit(f"{label}: target count={count}, expected=1")
    path.write_text(text.replace(old, new, 1))


runtime = Path("src/runtime/production_runtime.cpp")
replace_once(
    runtime,
    """      // Step modeでは停止後のcurrent positionが相対残量へ戻るfirmwareが\n      // あるため、productionでは絶対position + multi-turn feedbackを使う。\n      // configurePositionMode()は過去のStep設定で0になったposition limitも戻す。\n      last_initialization_error = servo.configurePositionMode(\n          STS3215::Persistence::volatile_only);\n      if (last_initialization_error == ESP_OK)\n        last_initialization_error = servo.setFeedbackMode(\n            STS3215::FeedbackMode::multi_turn,\n            STS3215::Persistence::volatile_only);\n      if (last_initialization_error != ESP_OK)\n        return false;\n      mode_prepared =\n          servo.verifyOperatingMode(STS3215::OperatingMode::position) ==\n          ESP_OK;\n""",
    """      // Step modeでは停止後のcurrent positionが相対残量へ戻るfirmwareが\n      // あるため、productionでは絶対position + multi-turn feedbackを使う。\n      // multi-turn absolute controlではangle limitを0/0にする必要があるため、\n      // 単回転用の0..4095 configurePositionMode()は使用しない。\n      last_initialization_error = servo.configureMultiTurnPositionMode(\n          STS3215::Persistence::volatile_only);\n      if (last_initialization_error != ESP_OK)\n        return false;\n      mode_prepared =\n          servo.verifyOperatingMode(STS3215::OperatingMode::position) ==\n          ESP_OK;\n""",
    "production runtime STS mode configuration",
)

doc = Path("docs/04c_ForceStartSequence詳細.md")
replace_once(
    doc,
    """productionのパラシュートサーボは`OperatingMode::position`と`FeedbackMode::multi_turn`を使用する。`OperatingMode::step`は相対移動command向けであり、停止後のcurrent position feedbackを物理絶対角として扱えないfirmwareがあるため、productionのcurrent angle sourceには使用しない。\n\n- STS3215のfresh current positionはsigned multi-turn countとして取得する。\n""",
    """productionのパラシュートサーボは`OperatingMode::position`と`FeedbackMode::multi_turn`を使用する。`OperatingMode::step`は相対移動command向けであり、停止後のcurrent position feedbackを物理絶対角として扱えないfirmwareがあるため、productionのcurrent angle sourceには使用しない。\n\nSTS3215のmulti-turn absolute position controlではminimum/maximum angle limitをともに`0`へ設定する。`0..4095`は単回転position mode用であり、multi-turn production設定には使用しない。\n\n- STS3215のfresh current positionはsigned multi-turn countとして取得する。\n""",
    "04c multi-turn limit requirement",
)

print("MissionBoard STS multi-turn limit patch applied")
