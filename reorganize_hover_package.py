"""Safely package the four Marker 23 hover modules without breaking old imports.

Run ONLY from the Drone repository root:
    python -B reorganize_hover_package.py

Move code into hover/, leave minimal compatibility wrappers in the root,
execute offline/mocked regressions, and roll back this move if any check fails.
Never imports DJITelloPy, opens a network connection, or sends motor commands.
"""

import ast
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parent
PACKAGE = ROOT / "hover"
FILES = (
    "junction_marker_detector.py",
    "junction_hover_controller.py",
    "junction_hover_runtime_offline.py",
    "junction_hover_observer.py",
)
IMPORT_REPLACEMENTS = {
    "junction_hover_controller.py": (
        ("from junction_marker_detector import detect_junction_marker",
         "from .junction_marker_detector import detect_junction_marker"),
    ),
    "junction_hover_runtime_offline.py": (
        ("from junction_hover_controller import JunctionHoverController",
         "from .junction_hover_controller import JunctionHoverController"),
    ),
    "junction_hover_observer.py": (
        ("from junction_hover_controller import JunctionHoverController",
         "from .junction_hover_controller import JunctionHoverController"),
        ("from junction_marker_detector import detect_junction_marker",
         "from .junction_marker_detector import detect_junction_marker"),
    ),
}
TEST_COMMANDS = (
    ("junction_hover_controller.py", "--self-test"),
    ("junction_hover_runtime_offline.py", "--self-test"),
    ("junction_hover_observer.py", "--self-test"),
    ("test_junction_hover_integration.py",),
    ("test_hover_capture_mocked.py",),
)


def run(args, *, capture=False):
    return subprocess.run(args, cwd=ROOT, check=True,
                          capture_output=capture, text=capture)


def main():
    if Path.cwd().resolve() != ROOT.resolve():
        raise SystemExit("STOP: run from C:\\python\\Drone\\Drone")
    if PACKAGE.exists():
        raise SystemExit("STOP: hover/ already exists; no changes made")

    snapshots = {}
    transformed = {}
    for filename in FILES:
        source_path = ROOT / filename
        if not source_path.is_file():
            raise SystemExit(f"STOP: missing {filename}; no changes made")
        # Do not overwrite or silently migrate unfinished edits.
        tracked = subprocess.run(
            ["git", "ls-files", "--error-unmatch", "--", filename],
            cwd=ROOT, capture_output=True, text=True,
        )
        if tracked.returncode != 0:
            raise SystemExit(f"STOP: {filename} is not tracked by git")
        snapshots[filename] = source_path.read_bytes()
        content = snapshots[filename].decode("utf-8-sig")
        for old, new in IMPORT_REPLACEMENTS.get(filename, ()):
            if content.count(old) != 1:
                raise SystemExit(f"STOP: import contract changed in {filename}: {old!r}")
            content = content.replace(old, new, 1)
        ast.parse(content, filename=f"hover/{filename}")
        transformed[filename] = content

    changed = subprocess.run(
        ["git", "status", "--porcelain", "--", *FILES],
        cwd=ROOT, check=True, capture_output=True, text=True,
    ).stdout.strip()
    if changed:
        raise SystemExit(f"STOP: hover source has uncommitted edits:\n{changed}")
    for cmd in TEST_COMMANDS:
        if not (ROOT / cmd[0]).is_file():
            raise SystemExit(f"STOP: missing offline test {cmd[0]}")

    print("PRECHECK PASS: four tracked hover modules unchanged; no destination collision", flush=True)
    created_package = False
    try:
        PACKAGE.mkdir()
        created_package = True
        (PACKAGE / "__init__.py").write_text(
            '"""Marker 23 vision, stabilization, and offline observation."""\n',
            encoding="utf-8",
        )
        for filename in FILES:
            (PACKAGE / filename).write_text(transformed[filename], encoding="utf-8")
            module = Path(filename).stem
            wrapper = (
                '"""Compatibility entry point; implementation lives in hover/."""\n'
                'if __name__ == "__main__":\n'
                '    from runpy import run_module\n'
                f'    run_module("hover.{module}", run_name="__main__")\n'
                'else:\n'
                f'    from hover.{module} import *\n'
            )
            ast.parse(wrapper)
            (ROOT / filename).write_text(wrapper, encoding="utf-8")

        for cmd in TEST_COMMANDS:
            print(f"RUNNING: python -B {' '.join(cmd)}", flush=True)
            run([sys.executable, "-B", *cmd])
        # Also prove that the historical import names and package imports resolve
        # to the same underlying objects.
        check = (
            'from junction_hover_controller import JunctionHoverController as A; '
            'from hover.junction_hover_controller import JunctionHoverController as B; '
            'from junction_hover_observer import MotorDisabledObserver as C; '
            'from hover.junction_hover_observer import MotorDisabledObserver as D; '
            'from junction_marker_detector import detect_junction_marker as E; '
            'from hover.junction_marker_detector import detect_junction_marker as F; '
            'assert A is B and C is D and E is F; '
            'print("PASS: legacy and package imports share classes/functions")'
        )
        run([sys.executable, "-B", "-c", check])
    except BaseException as exc:
        for filename, original in snapshots.items():
            (ROOT / filename).write_bytes(original)
        if created_package:
            shutil.rmtree(PACKAGE)
        print(f"ROLLED BACK: {type(exc).__name__}: {exc}", file=sys.stderr)
        raise

    print("SUCCESS: four implementations moved to hover/ with root compatibility entry points")
    print("PASS: all five offline/mocked checks and import compatibility")
    print("NO DRONE CONNECTION OR MOTOR COMMANDS BY THIS MIGRATION SCRIPT")
    print("DO NOT COMMIT YET: review changes and git status first")


if __name__ == "__main__":
    main()
