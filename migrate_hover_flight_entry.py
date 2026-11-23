"""Safely move the guarded Tello hover-capture implementation into flight/.

No flight, network, or motor commands. Execute from Drone repository root:
    python -B migrate_hover_flight_entry.py

Retains the existing `python -B test_hover_capture.py` entry point and updates
source inspection in the mocked tests. Rolls back on any test failure.
"""
from __future__ import annotations

import ast
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parent
SOURCE = ROOT / "test_hover_capture.py"
MOCK = ROOT / "tests" / "test_hover_capture_mocked.py"
DEST_DIR = ROOT / "flight"
DEST = DEST_DIR / "test_hover_capture.py"
INIT = DEST_DIR / "__init__.py"

WRAPPER = '''"""Backwards-compatible entry point for the guarded camera-only Tello hover.

CAUTION: Executing this script can command a REAL Tello takeoff after FLY.
Original implementation: flight/test_hover_capture.py.
"""
if __name__ == "__main__":
    from runpy import run_module
    run_module("flight.test_hover_capture", run_name="__main__")
'''


def git(*args: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(["git", *args], cwd=ROOT, capture_output=True, text=True, check=False)


def preflight() -> tuple[bytes, bytes]:
    if not (ROOT / ".git").is_dir():
        raise RuntimeError("STOP: script must be placed in the Git repository root")
    branch = git("branch", "--show-current")
    if branch.returncode != 0 or branch.stdout.strip() != "feature/vision-navigation":
        raise RuntimeError("STOP: expected Git branch feature/vision-navigation")
    for file in (SOURCE, MOCK):
        if not file.is_file():
            raise RuntimeError(f"STOP: missing {file.relative_to(ROOT)}")
        rel = file.relative_to(ROOT).as_posix()
        tracked = git("ls-files", "--error-unmatch", "--", rel)
        if tracked.returncode:
            raise RuntimeError(f"STOP: {rel} is not tracked by Git")
    clean = git("status", "--porcelain", "--", "test_hover_capture.py", "tests/test_hover_capture_mocked.py", "flight/")
    if clean.returncode or clean.stdout.strip():
        raise RuntimeError(f"STOP: files to migrate have local changes:\n{clean.stdout}")
    if DEST_DIR.exists():
        raise RuntimeError("STOP: flight/ already exists; no files modified")
    original_source, original_mock = SOURCE.read_bytes(), MOCK.read_bytes()
    if b'from djitellopy import Tello' not in original_source:
        raise RuntimeError("STOP: current capture source no longer matches reviewed program")
    if b'from junction_hover_observer import MotorDisabledObserver' not in original_source:
        raise RuntimeError("STOP: observer integration is missing")
    if b'observe_tello_rgb' not in original_source:
        raise RuntimeError("STOP: observer API is missing")
    if b'SCRIPT = ROOT / "test_hover_capture.py"' not in original_mock:
        raise RuntimeError("STOP: mock script path differs from expected")
    ast.parse(original_source.decode("utf-8-sig"), filename=str(SOURCE))
    ast.parse(original_mock.decode("utf-8-sig"), filename=str(MOCK))
    return original_source, original_mock


def patch_mock(original: bytes) -> bytes:
    # Byte-based substitutions preserve preexisting LF or CRLF line endings.
    newline = b"\r\n" if b"\r\n" in original else b"\n"
    replacements = [
        (b'SCRIPT = ROOT / "test_hover_capture.py"',
         b'SCRIPT = ROOT / "test_hover_capture.py"' + newline +
         b'SOURCE = ROOT / "flight" / "test_hover_capture.py"'),
        (b'    if not SCRIPT.is_file():', b'    if not SOURCE.is_file():'),
        (b'f"STOP: capture source missing: {SCRIPT}"', b'f"STOP: capture source missing: {SOURCE}"'),
        (b'    source = SCRIPT.read_text(encoding="utf-8-sig")',
         b'    source = SOURCE.read_text(encoding="utf-8-sig")'),
        (b'    tree = ast.parse(source, filename=str(SCRIPT))',
         b'    tree = ast.parse(source, filename=str(SOURCE))'),
    ]
    current = original
    for needle, replacement in replacements:
        if current.count(needle) != 1:
            raise RuntimeError(f"STOP: mock test changed; expected one occurrence of: {needle!r}")
        current = current.replace(needle, replacement)
    ast.parse(current.decode("utf-8-sig"), filename=str(MOCK))
    return current


def run_safe_tests() -> None:
    commands = [
        [sys.executable, "-B", "-m", "tests.test_hover_capture_mocked"],
        [sys.executable, "-B", "junction_hover_observer.py", "--self-test"],
        [sys.executable, "-B", "-m", "tests.test_junction_hover_integration"],
    ]
    for command in commands:
        print("RUNNING:", " ".join(command[1:]), flush=True)
        completed = subprocess.run(command, cwd=ROOT, check=False)
        if completed.returncode:
            raise RuntimeError(f"STOP: offline regression failed (exit {completed.returncode})")


def main() -> None:
    _, old_mock = preflight()
    new_mock = patch_mock(old_mock)
    print("PRECHECK PASS: tracked source and mock test clean; no destination collisions", flush=True)
    created_root_wrapper = False
    moved = False
    created_init = False
    try:
        DEST_DIR.mkdir()
        SOURCE.rename(DEST)
        moved = True
        SOURCE.write_text(WRAPPER, encoding="utf-8", newline="\n")
        created_root_wrapper = True
        INIT.write_text('"""Real-drone flight entry points. Importing must never auto-connect."""\n',
                        encoding="utf-8", newline="\n")
        created_init = True
        MOCK.write_bytes(new_mock)
        ast.parse(SOURCE.read_text(encoding="utf-8"), filename=str(SOURCE))
        ast.parse(DEST.read_text(encoding="utf-8-sig"), filename=str(DEST))
        run_safe_tests()
    except BaseException:
        # Restore only the specific files we changed; do not touch user datasets.
        print("ROLLBACK: restoring original root capture and mock test", flush=True)
        MOCK.write_bytes(old_mock)
        if created_root_wrapper and SOURCE.exists():
            SOURCE.unlink()
        if moved and DEST.exists():
            DEST.rename(SOURCE)
        if created_init and INIT.exists():
            INIT.unlink()
        try:
            DEST_DIR.rmdir()  # succeeds only when empty
        except OSError:
            print("WARNING: flight/ not empty; left intact", flush=True)
        raise
    print("SUCCESS: flight/test_hover_capture.py contains unchanged flight implementation", flush=True)
    print("SUCCESS: root test_hover_capture.py remains the same launch command", flush=True)
    print("SUCCESS: mocked tests check implementation and execute root launcher", flush=True)
    print("NO REAL TELLO. NO WIFI. NO MOTOR COMMANDS. DO NOT COMMIT YET.", flush=True)


if __name__ == "__main__":
    try:
        main()
    except (RuntimeError, OSError, UnicodeError, SyntaxError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr, flush=True)
        raise SystemExit(1)
