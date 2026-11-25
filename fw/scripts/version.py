# Firmware version taken from git at build time (PlatformIO pre-script, see platformio.ini).
#
#   on a release tag v1.0.2             -> "1.0.2"
#   3 commits after v1.0.2              -> "1.0.2-3-gabc1234"   (abc1234 = commit)
#   with local changes not committed    -> "...-dirty"
#   no tag yet / no git                 -> "0.0.0-gabc1234" / "0.0.0-unknown"
#
# The value reaches the code as the macro FW_VERSION, printed in "[fw] name=g2fw version=...".
import subprocess

Import("env")


def git_version():
    try:
        out = subprocess.check_output(
            ["git", "describe", "--tags", "--match", "v[0-9]*", "--always", "--dirty"],
            cwd=env.subst("$PROJECT_DIR"),
            stderr=subprocess.DEVNULL,
            text=True,
        ).strip()
    except Exception:
        return "0.0.0-unknown"
    if out.startswith("v"):
        return out[1:]
    return "0.0.0-g" + out  # no release tag yet: only the commit


version = git_version()
env.Append(CPPDEFINES=[("FW_VERSION", env.StringifyMacro(version))])
print("g2fw version: " + version)
