"""One-time safe edit of two UNTRACKED experimental Tello calibration sources.

- Does not import or execute either flight script.
- Updates the Marker 23 import to the hover package.
- Makes landing attempt independent of a neutral-RC failure.
- Saves backups in archive/backups and restores sources on test failure.
- Installs an AST-only mock safety test under tests/.

Run from repository root: python -B patch_motor_calibration_safety.py
"""
from __future__ import annotations

import ast
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path.cwd()
SOURCE_DIR = ROOT / 'calibration' / 'experiments'
BACKUP_DIR = ROOT / 'archive' / 'backups'
TEST_DEST = ROOT / 'tests' / 'test_motor_calibration_safety.py'
TEST_CONTENT = r'''"""Offline AST-based checks of experimental Tello motor-calibration cleanup.

Run ONLY as: python -B -m tests.test_motor_calibration_safety
Never imports or executes a live calibration script.
"""
from __future__ import annotations

import ast
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
NAMES = ('calibrate_tello_right_axis.py', 'calibrate_tello_forward_axis.py')


class FakeTello:
    def __init__(self, *, neutral_error=False, land_error=False):
        self.calls = []
        self.neutral_error = neutral_error
        self.land_error = land_error

    def send_rc_control(self, *args):
        self.calls.append(('neutral', args))
        assert args == (0, 0, 0, 0), 'Unexpected RC parameters in cleanup'
        if self.neutral_error:
            raise RuntimeError('MOCK neutral failure')

    def land(self):
        self.calls.append(('land',))
        if self.land_error:
            raise RuntimeError('MOCK land failure')

    def streamoff(self):
        self.calls.append(('streamoff',))

    def end(self):
        self.calls.append(('end',))


def extract_cleanup(path: Path):
    source = path.read_text(encoding='utf-8-sig')
    assert source.count('from hover.junction_marker_detector import detect_junction_marker') == 1, (
        f'{path.name}: expected new package import'
    )
    tree = ast.parse(source, filename=str(path))
    flight = [node for node in tree.body if isinstance(node, ast.Try) and any(
        isinstance(sub, ast.Call) and isinstance(sub.func, ast.Attribute)
        and sub.func.attr == 'takeoff'
        for statement in node.body for sub in ast.walk(statement)
    )]
    assert len(flight) == 1, f'{path.name}: unexpected flight structure'
    cleanup_ast = ast.fix_missing_locations(ast.Module(body=flight[0].finalbody, type_ignores=[]))
    return compile(cleanup_ast, str(path) + ':finally-only', 'exec')


def run_case(code, label, *, attempted=True, neutral_error=False, land_error=False):
    fake = FakeTello(neutral_error=neutral_error, land_error=land_error)
    messages = []
    ctx = {
        't': fake, 'takeoff_attempted': attempted, 'streaming': True, 'connected': True,
        'time': SimpleNamespace(sleep=lambda _: None),
        'print': lambda *args, **kwargs: messages.append(' '.join(map(str, args))),
    }
    exec(code, ctx)
    actions = [c[0] for c in fake.calls]
    expected = (['neutral', 'land'] if attempted else []) + ['streamoff', 'end']
    assert actions == expected, f'{label}: cleanup order {actions}, expected {expected}'
    if attempted:
        assert fake.calls[0] == ('neutral', (0, 0, 0, 0))
        if land_error:
            assert any('CRITICAL' in m for m in messages), f'{label}: no landing failure warning'
    print('PASS:', label)


def main():
    for name in NAMES:
        path = ROOT / 'calibration' / 'experiments' / name
        if not path.is_file():
            raise SystemExit(f'Missing file: {path}')
        code = extract_cleanup(path)
        for scenario, params in (
            ('normal', {}),
            ('neutral_failure', {'neutral_error': True}),
            ('landing_failure', {'land_error': True}),
            ('neutral_and_landing_failure', {'neutral_error': True, 'land_error': True}),
            ('no_takeoff', {'attempted': False}),
        ):
            run_case(code, name + ' / ' + scenario, **params)
    print('PASS: 10 offline cleanup scenarios across both calibration scripts')
    print('AST FINALLY-BLOCK ONLY. NO IMPORT OF LIVE SCRIPTS. NO TELLO. NO NETWORK. NO MOTOR COMMANDS.')
    print('Flight behavior, motor-axis mapping, physical landing remain UNVERIFIED.')


if __name__ == '__main__':
    main()
'''
NAMES = ('calibrate_tello_right_axis.py', 'calibrate_tello_forward_axis.py')
OLD_IMPORT = 'from junction_marker_detector import detect_junction_marker'
NEW_IMPORT = 'from hover.junction_marker_detector import detect_junction_marker'
OLD_CLEANUP = '''    if takeoff_attempted:
        try:
            t.send_rc_control(0, 0, 0, 0)
            t.land()
            print("Landing acknowledged; verify touchdown")
        except Exception as e:
            print("CRITICAL landing error:", e)
        time.sleep(2)
'''
NEW_CLEANUP = '''    if takeoff_attempted:
        try:
            t.send_rc_control(0, 0, 0, 0)
        except Exception as e:
            print("WARNING: Neutral RC command failed:", e)
        try:
            t.land()
            print("Landing acknowledged; verify touchdown")
        except Exception as e:
            print("CRITICAL landing error:", e)
        time.sleep(2)
'''


def main():
    if not (ROOT / '.git').exists() or not (ROOT / 'hover' / '__init__.py').is_file():
        raise SystemExit('STOP: Run from the Drone repo root, after hover package migration')
    if TEST_DEST.exists():
        raise SystemExit(f'STOP: Will not replace existing test: {TEST_DEST}')
    if not (ROOT / 'tests' / '__init__.py').exists():
        raise SystemExit('STOP: tests package not found')

    changed = {}
    original = {}
    backups = {}
    for name in NAMES:
        path = SOURCE_DIR / name
        backup = BACKUP_DIR / (name + '.bak_before_safety_patch')
        if not path.is_file():
            raise SystemExit(f'STOP: Missing {path}')
        if backup.exists():
            raise SystemExit(f'STOP: Backup already exists: {backup}')
        before = path.read_bytes()
        # Do not change unexpected files or silently alter a previously patched script.
        data = before.decode('utf-8-sig')
        lf = data.replace('\r\n', '\n')
        if lf.count(OLD_IMPORT) != 1 or lf.count(OLD_CLEANUP) != 1:
            raise SystemExit(f'STOP: Unexpected source layout in {name}; no edits made')
        updated = lf.replace(OLD_IMPORT, NEW_IMPORT).replace(OLD_CLEANUP, NEW_CLEANUP)
        ast.parse(updated, filename=name)
        original[path] = before
        changed[path] = updated.encode('utf-8')
        backups[path] = backup

    # Verify all inputs before modifying any file.
    test_bytes = TEST_CONTENT.encode('utf-8')
    ast.parse(TEST_CONTENT, filename=str(TEST_DEST))
    print('PRECHECK PASS: both sources exactly matched expected patterns; syntax valid')
    BACKUP_DIR.mkdir(parents=True, exist_ok=True)
    try:
        for path, backup in backups.items():
            backup.write_bytes(original[path])
            print('BACKUP:', backup.relative_to(ROOT))
        for path, data in changed.items():
            path.write_bytes(data)
            print('PATCHED:', path.relative_to(ROOT))
        TEST_DEST.write_bytes(test_bytes)
        print('INSTALLED:', TEST_DEST.relative_to(ROOT))
        print('RUNNING: python -B -m tests.test_motor_calibration_safety')
        result = subprocess.run([sys.executable, '-B', '-m', 'tests.test_motor_calibration_safety'], cwd=ROOT)
        if result.returncode != 0:
            raise RuntimeError(f'Mocked checks failed: exit {result.returncode}')
    except BaseException:
        for path, data in original.items():
            path.write_bytes(data)
        TEST_DEST.unlink(missing_ok=True)
        print('ROLLBACK: original motor calibration sources restored; tests not retained')
        raise
    print('SUCCESS: imports fixed and independent landing attempt installed in both experimental scripts')
    print('NO TELLO. NO WIFI. NO MOTOR COMMANDS. DO NOT EXECUTE EXPERIMENTS OR COMMIT YET.')


if __name__ == '__main__':
    main()
