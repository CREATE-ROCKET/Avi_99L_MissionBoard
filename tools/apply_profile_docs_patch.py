from pathlib import Path


def replace_exact(path: str, old: str, new: str, expected: int = 1) -> None:
    target = Path(path)
    text = target.read_text(encoding="utf-8")
    count = text.count(old)
    if count != expected:
        raise SystemExit(
            f"{path}: 置換対象数が不正です: expected={expected} actual={count}\n{old[:200]}"
        )
    target.write_text(text.replace(old, new), encoding="utf-8")


replace_exact(
    "docs/04_通信仕様.md",
    "| `0x32` | `SelectMotorProfile` |",
    "| `0x32` | reserved / `NotSupported`（旧`SelectMotorProfile`） |",
)
replace_exact(
    "docs/04_通信仕様.md",
    "`SelectMotorProfile`はargs0をprofile IDとする。",
    "`0x32`は旧`SelectMotorProfile`のreserved codeであり、受信時は副作用なく`Rejected / NotSupported`とする。MotorProfileは`AVI_99L_MOTOR_PROFILE_ID`でbuild時に固定し、runtime commandでは変更しない。",
)

replace_exact(
    "docs/01_コマンド受信モード.md",
    "- `SelectMotorProfile`\n",
    "",
)
replace_exact(
    "docs/01_コマンド受信モード.md",
    """## 9. MotorProfile

動翼motorには予備個体があるため、motor依存値をcontrollerやTorqueMapperへ直接埋め込まない。

`MotorProfile`相当の設定として、少なくともprofile ID、motor電気定数、駆動系効率、current/torque limit、ZeroHold tuning、必要に応じたControl tuning/tableを差し替え可能にする。

`SelectMotorProfile`はCommandReceiveのみで受理し、選択profileをNVSへ永続化する。profile変更時は`FinZeroConfigured`を無効化する。

MotorPolarityはruntime profile選択とは別にコード上の静的設定とし、実機低duty試験で確定する。
""",
    """## 9. MotorProfile

動翼motorには予備個体があるため、motor依存値をcontrollerやTorqueMapperへ直接埋め込まない。各個体はstableなprofile IDを持ち、motor電気定数、MotorPolarity、駆動系効率、current/torque limit、ZeroHold tuning、必要に応じたControl/TorqueMapper用parameterを`MotorProfile`へまとめる。

飛行firmwareのactive profileはcompile definition `AVI_99L_MOTOR_PROFILE_ID`で**build時に一つだけ明示選択**する。未指定または未知IDのbuildは失敗させ、NVS、内部Flash、CAN、LoRa、USB commandからactive profileを変更しない。

現行catalogはID=1の`kFlightMotorA`とID=2の`kSpareMotorB`である。profile entryが存在してもrequired parameterやMotorPolarity、qualification条件が満たされない場合は`MotorProfileValid=false`とする。

旧`SelectMotorProfile` command `0x32`はreserved / `NotSupported`であり、副作用なく拒否する。motor個体を変更する場合はprofile IDを明示して再build・再書込みし、通常のreset規則に従ってFin zero等のflight設定をやり直す。
""",
)

replace_exact(
    "README.md",
    """```sh
pio pkg install -e avi_99l_missionboard
pio run
```

既定は`MISSION_BRINGUP_SHELL=0`のproduction runtimeです。既存bring-up shellは`pio run -e avi_99l_missionboard_bringup`でbuildします。既知のdriver panicを再現し得るCAN比較診断は、専用environmentだけに隔離しています。
""",
    """```sh
AVI_99L_MOTOR_PROFILE_ID=1 pio pkg install -e avi_99l_missionboard
AVI_99L_MOTOR_PROFILE_ID=1 pio run -e avi_99l_missionboard
```

`AVI_99L_MOTOR_PROFILE_ID`はbuildごとに明示指定必須です。現行catalogは`1=kFlightMotorA`、`2=kSpareMotorB`で、未指定または未知IDではbuildを失敗させます。active profileはruntime/NVS/CAN/LoRa/USBから変更できません。

既定environmentは`MISSION_BRINGUP_SHELL=0`のproduction runtimeです。既存bring-up shellは`AVI_99L_MOTOR_PROFILE_ID=1 pio run -e avi_99l_missionboard_bringup`でbuildします。既知のdriver panicを再現し得るCAN比較診断は、専用environmentだけに隔離しています。
""",
)
replace_exact(
    "README.md",
    "pio run -e avi_99l_missionboard_can_diag",
    "AVI_99L_MOTOR_PROFILE_ID=1 pio run -e avi_99l_missionboard_can_diag",
)
replace_exact(
    "README.md",
    "pio run -e avi_99l_missionboard_characterization",
    "AVI_99L_MOTOR_PROFILE_ID=1 pio run -e avi_99l_missionboard_characterization",
    expected=3,
)
