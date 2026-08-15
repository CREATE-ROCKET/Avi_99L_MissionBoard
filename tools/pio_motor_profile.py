Import("env")

if env.IsIntegrationDump():
    Return()

import os


value = os.environ.get("AVI_99L_MOTOR_PROFILE_ID")
if value not in ("1", "2"):
    raise RuntimeError(
        "AVI_99L_MOTOR_PROFILE_ID must be explicitly set to 1 or 2 before building"
    )

profile_id = int(value)
env.Append(CPPDEFINES=[("AVI_99L_MOTOR_PROFILE_ID", profile_id)])
print(f"MissionBoard motor profile: AVI_99L_MOTOR_PROFILE_ID={profile_id}")
