import ast
from pathlib import Path
from unittest.mock import MagicMock, call

source = Path("Drone table detection.py").read_text(encoding="utf-8-sig")
tree = ast.parse(source)

# Find the actual flight mission
mission = next(
    node for node in tree.body
    if isinstance(node, ast.Try)
    and any(
        isinstance(item, ast.For)
        for item in node.body
    )
)

# Extract search and return-to-START code
start = next(
    i for i, node in enumerate(mission.body)
    if isinstance(node, ast.Assign)
    and any(
        isinstance(t, ast.Name) and t.id == "scan_positions"
        for t in node.targets
    )
)

end = next(
    i for i in range(start, len(mission.body))
    if isinstance(mission.body[i], ast.Expr)
    and isinstance(mission.body[i].value, ast.Call)
    and isinstance(mission.body[i].value.func, ast.Attribute)
    and mission.body[i].value.func.attr == "land"
)

module = ast.fix_missing_locations(
    ast.Module(body=mission.body[start:end + 1], type_ignores=[])
)
program = compile(module, "<offline-search>", "exec")

scenarios = [
    ("Pad 1", [-1, 1], 1, [50], [50]),
    ("Pad 3", [-1, -1, 3], 3, [50, 50], [100]),
    ("No pad", [-1, -1, -1], None, [50, 50], [100]),
]

for name, readings, expected_pad, rights, lefts in scenarios:
    drone = MagicMock()
    drone.get_mission_pad_id.side_effect = readings
    publish = MagicMock()

    context = {
        "tello": drone,
        "time": MagicMock(),
        "net_displacement": 0,
        "target_found": False,
        "publish_table_event": publish,
    }

    exec(program, context)

    assert drone.move_right.call_args_list == [
        call(d) for d in rights
    ], name

    assert drone.move_left.call_args_list == [
        call(d) for d in lefts
    ], name

    assert drone.get_mission_pad_id.call_count == len(readings)
    assert context["target_found"] == (expected_pad is not None)
    assert context["net_displacement"] == 0
    drone.land.assert_called_once_with()

    if expected_pad is None:
        publish.assert_not_called()
    else:
        publish.assert_called_once_with(expected_pad)

    print("PASS:", name)

print("Offline search tests: 3/3 PASSED")
