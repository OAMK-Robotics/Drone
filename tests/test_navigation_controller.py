from navigation_controller import NavigationController


def test_scenario(name, steps, expected):
    controller = NavigationController()
    actual = [
        controller.update(vision, pad)
        for vision, pad in steps
    ]

    assert actual == expected, (
        f"{name}: Expected {expected}, got {actual}"
    )
    print("PASS:", name)


test_scenario(
    "Table A active",
    [("JUNCTION", None), ("TABLE_ZONE", 1)],
    ["SELECT_TABLE_A", "PUBLISH_TABLE_1"]
)

test_scenario(
    "Table A inactive, then Table B active",
    [
        ("JUNCTION", None),
        ("TABLE_ZONE", 7),
        ("PATH", None),
        ("MAIN_JUNCTION", None),
        ("TABLE_ZONE", 3)
    ],
    [
        "SELECT_TABLE_A",
        "RETURN_TO_JUNCTION",
        "RETURN_TO_JUNCTION",
        "SELECT_TABLE_B",
        "PUBLISH_TABLE_3"
    ]
)

test_scenario(
    "Neither table active",
    [
        ("JUNCTION", None),
        ("TABLE_ZONE", 7),
        ("MAIN_JUNCTION", None),
        ("TABLE_ZONE", 7)
    ],
    [
        "SELECT_TABLE_A",
        "RETURN_TO_JUNCTION",
        "SELECT_TABLE_B",
        "NO_REQUEST"
    ]
)

test_scenario(
    "Unknown vision stops navigation",
    [("UNKNOWN", None), ("JUNCTION", None)],
    ["STOP", "STOP"]
)

test_scenario(
    "Unreadable pad triggers retry",
    [("JUNCTION", None), ("TABLE_ZONE", -1)],
    ["SELECT_TABLE_A", "RETRY_PAD"]
)

test_scenario(
    "No junction at start",
    [("PATH", None), ("JUNCTION", None)],
    ["WAIT_FOR_JUNCTION", "SELECT_TABLE_A"]
)

print("Navigation tests: 6/6 PASSED")
