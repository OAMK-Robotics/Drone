"""Reorganize the tracked *manual, non-flying* camera-axis calibration script.

Run from repository root:
    python -B migrate_camera_calibration.py

Checks the source against Git HEAD, preserves the original entry command,
runs only the explicitly offline --self-test paths, and rolls back on failure.
Does NOT import djitellopy or contact the drone.
"""
from __future__ import annotations

import ast
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parent
NAME = "calibrate_camera_axes_marker23.py"
SOURCE = ROOT / NAME
DEST_DIR = ROOT / "calibration"
DEST = DEST_DIR / NAME

LAUNCHER = '''"""Compatibility launcher for manual camera-axis calibration.

Actual implementation: calibration/calibrate_camera_axes_marker23.py.
Normal invocation CONNECTS to Tello for camera observation; no motor commands.
For offline checks use --self-test.
"""
from pathlib import Path
from runpy import run_path

if __name__ == "__main__":
    run_path(str(Path(__file__).resolve().parent / "calibration" / "calibrate_camera_axes_marker23.py"), run_name="__main__")
'''


def git(*arguments: str) -> subprocess.CompletedProcess[bytes]:
    return subprocess.run(
        ["git", *arguments], cwd=ROOT, stdout=subprocess.PIPE,
        stderr=subprocess.PIPE, check=True,
    )


def validate_source(source: bytes) -> None:
    tree = ast.parse(source.decode("utf-8-sig"), filename=NAME)
    top_level = [n for n in tree.body if isinstance(n, ast.If)]
    guarded_main = any(
        isinstance(n.test, ast.Compare)
        and isinstance(n.test.left, ast.Name)
        and n.test.left.id == "__name__"
        and any(isinstance(c, ast.Constant) and c.value == "__main__" for c in n.test.comparators)
        for n in top_level
    )
    if not guarded_main or b"--self-test" not in source:
        raise RuntimeError("STOP: expected guarded --self-test entry point missing")
    if not any(isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef)) and n.name == "main" for n in tree.body):
        raise RuntimeError("STOP: expected main function missing")


def offline_check(filename: Path) -> None:
    command = [sys.executable, "-B", str(filename), "--self-test"]
    print("RUNNING OFFLINE:", " ".join(command[1:]), flush=True)
    result = subprocess.run(command, cwd=ROOT, capture_output=True, text=True, timeout=30)
    print(result.stdout, end="")
    if result.returncode != 0:
        print(result.stderr, end="", file=sys.stderr)
        raise RuntimeError(f"STOP: offline self-test failed ({filename.name})")
    if "PASS:" not in result.stdout:
        raise RuntimeError(f"STOP: missing expected PASS output ({filename.name})")


def main() -> None:
    if Path.cwd().resolve() != ROOT:
        raise RuntimeError(f"STOP: run from repository root: {ROOT}")
    if not SOURCE.is_file() or DEST.exists():
        raise RuntimeError("STOP: source missing or destination already exists")
    if DEST_DIR.exists() and not DEST_DIR.is_dir():
        raise RuntimeError("STOP: calibration path is not a directory")
    if git("ls-files", "--error-unmatch", "--", NAME).stdout.strip() != NAME.encode():
        raise RuntimeError("STOP: source is not Git-tracked")
    current = SOURCE.read_bytes()
    committed = git("show", f"HEAD:{NAME}").stdout
    if current.replace(b"\r\n", b"\n") != committed.replace(b"\r\n", b"\n"):
        raise RuntimeError("STOP: local source differs from Git HEAD")
    if git("diff", "--cached", "--name-only", "HEAD", "--", NAME).stdout.strip():
        raise RuntimeError("STOP: source has staged modifications")
    validate_source(current)
    print("PRECHECK PASS: tracked camera-only source unchanged; no destination collision")
    offline_check(SOURCE)

    created_dir = not DEST_DIR.exists()
    migrated = False
    try:
        DEST_DIR.mkdir(exist_ok=True)
        shutil.move(str(SOURCE), str(DEST))
        migrated = True
        with SOURCE.open("w", encoding="utf-8", newline="\n") as handle:
            handle.write(LAUNCHER)

        if DEST.read_bytes() != current:
            raise RuntimeError("STOP: moved implementation bytes changed")
        offline_check(SOURCE)
        offline_check(DEST)
        print("SUCCESS: implementation moved to calibration/ unchanged")
        print("SUCCESS: original launch command remains usable via compatibility launcher")
        print("NO DRONE CONNECTION, NO TAKEOFF, NO MOTOR COMMANDS. DO NOT COMMIT YET.")
    except BaseException:
        if migrated:
            if SOURCE.is_file() and SOURCE.read_text(encoding="utf-8") == LAUNCHER:
                SOURCE.unlink()
            elif SOURCE.exists():
                raise RuntimeError("ROLLBACK STOP: launcher modified unexpectedly; recover manually")
            shutil.move(str(DEST), str(SOURCE))
            print("ROLLBACK PASS: restored original source")
        if created_dir and DEST_DIR.is_dir() and not any(DEST_DIR.iterdir()):
            DEST_DIR.rmdir()
        raise


if __name__ == "__main__":
    main()
