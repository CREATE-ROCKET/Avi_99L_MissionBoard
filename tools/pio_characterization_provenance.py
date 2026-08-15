Import("env")

if env.IsIntegrationDump():
    Return()

from pathlib import Path
import re
import subprocess


_SHA_RE = re.compile(r"^[0-9a-f]{40}$")


def _git(path: Path, *args: str) -> str:
    try:
        return subprocess.check_output(
            ["git", "-C", str(path), *args],
            stderr=subprocess.STDOUT,
            text=True,
        ).strip()
    except (OSError, subprocess.CalledProcessError) as error:
        output = getattr(error, "output", "")
        raise RuntimeError(
            f"characterization provenance: git command failed in {path}: "
            f"git {' '.join(args)} {output}"
        ) from error


def _sha(path: Path) -> str:
    value = _git(path, "rev-parse", "HEAD").lower()
    if _SHA_RE.fullmatch(value) is None:
        raise RuntimeError(
            f"characterization provenance: invalid git SHA for {path}: {value!r}"
        )
    return value


def _require_clean(path: Path, label: str) -> None:
    # untracked capture/data files do not alter the compiled source. Tracked changes do.
    status = _git(path, "status", "--porcelain", "--untracked-files=no")
    if status:
        raise RuntimeError(
            f"characterization provenance: {label} has tracked modifications; "
            "commit/stash them before building a formal capture firmware"
        )


project = Path(env.subst("$PROJECT_DIR")).resolve()
avi_libs = project / "lib" / "Avi_ESP_Libs"
if not avi_libs.is_dir():
    raise RuntimeError(
        "characterization provenance: lib/Avi_ESP_Libs is missing; "
        "run git submodule update --init --recursive"
    )

_require_clean(project, "MissionBoard repository")
_require_clean(avi_libs, "Avi_ESP_Libs submodule")
firmware_sha = _sha(project)
avi_sha = _sha(avi_libs)
expected_avi_sha = _git(project, "rev-parse", "HEAD:lib/Avi_ESP_Libs").lower()
if expected_avi_sha != avi_sha:
    raise RuntimeError(
        "characterization provenance: Avi_ESP_Libs checkout does not match "
        f"the MissionBoard gitlink (expected {expected_avi_sha}, actual {avi_sha})"
    )

env.Append(
    CPPDEFINES=[
        ("AVI_FIRMWARE_GIT_SHA", env.StringifyMacro(firmware_sha)),
        ("AVI_ESP_LIBS_GIT_SHA", env.StringifyMacro(avi_sha)),
    ]
)

print(
    "characterization provenance: "
    f"MissionBoard={firmware_sha} Avi_ESP_Libs={avi_sha}"
)
