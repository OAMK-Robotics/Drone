"""Offline AST-based checks of experimental Tello motor-calibration cleanup.

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
