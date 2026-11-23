"""Fix camera recorder printing while waiting for post-landing confirmation.

Offline source patch. Does not import or run the flight script or connect to Tello.
Run from repository root:
    python -B patch_ascent_recording_shutdown.py
"""
from __future__ import annotations

import ast
from pathlib import Path

SOURCE = Path('record_marker23_ascent.py')
BACKUP = Path('archive/backups/record_marker23_ascent.py.bak_recording_shutdown')

OLD = '''            # Keep connection and camera up while operator verifies actual touchdown.
            try:
                input('After VISUALLY confirming actual touchdown, press Enter: ')
            except (EOFError,KeyboardInterrupt):
                print('WARNING: operator confirmation unavailable; verify physical landing before leaving.',flush=True)
        stop.set()
        if thread is not None:
            thread.join(timeout=4)
            if thread.is_alive():
                print('WARNING: capture thread did not stop within 4 seconds.',flush=True)
'''

NEW = '''        # Stop recording BEFORE asking the operator to confirm touchdown.
        # Otherwise capture_worker keeps printing while input() is blocked.
        stop.set()
        if thread is not None:
            thread.join(timeout=4)
            if thread.is_alive():
                print('WARNING: capture thread did not stop within 4 seconds.',flush=True)
        if takeoff_attempted:
            # Physical touchdown must still be verified visually; land ACK is not proof.
            try:
                input('After VISUALLY confirming actual touchdown, press Enter: ')
            except (EOFError,KeyboardInterrupt):
                print('WARNING: operator confirmation unavailable; verify physical landing before leaving.',flush=True)
'''


def main():
    if not SOURCE.is_file():
        raise SystemExit(f'STOP: missing {SOURCE}; save this patch in the Drone project root')
    original_bytes = SOURCE.read_bytes()
    original = original_bytes.decode('utf-8-sig')
    normalized = original.replace('\r\n', '\n')
    if normalized.count(OLD) != 1:
        raise SystemExit('STOP: expected shutdown code not found exactly once. No changes made.')
    if BACKUP.exists():
        raise SystemExit(f'STOP: backup already exists: {BACKUP}. No changes made.')

    modified = normalized.replace(OLD, NEW, 1)
    ast.parse(modified, filename=str(SOURCE))  # syntax validation, no imports
    assert modified.count('stop.set()') >= 1
    assert modified.count("input('After VISUALLY confirming actual touchdown") == 1
    assert modified.index('        # Stop recording BEFORE asking') < modified.index("input('After VISUALLY confirming actual touchdown")
    assert modified.index("log_event('LANDING_ACKNOWLEDGED')") < modified.index('        # Stop recording BEFORE asking')
    assert modified.count('drone.land()') == normalized.count('drone.land()')
    # Code outside the specifically reviewed block must remain identical.
    assert modified.replace(NEW, OLD, 1) == normalized

    BACKUP.parent.mkdir(parents=True, exist_ok=True)
    BACKUP.write_bytes(original_bytes)
    try:
        SOURCE.write_bytes(modified.encode('utf-8'))
        assert SOURCE.read_bytes().decode('utf-8') == modified
    except BaseException:
        SOURCE.write_bytes(original_bytes)
        raise
    print('PASS: shutdown order corrected: land -> stop recording -> join thread -> prompt')
    print('PASS: source syntax and unchanged flight-control sections verified')
    print('BACKUP:', BACKUP)
    print('OFFLINE PATCH ONLY. NO DJITELLOPY IMPORT, WIFI, FLIGHT OR MOTOR COMMANDS.')
    print('Physical landing and recording reliability remain unverified.')


if __name__ == '__main__':
    main()
