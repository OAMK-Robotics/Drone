from junction_marker_detector import detect_junction_marker


def resolve_vision_label(frame, base_label):
    """Use ArUco ID 23 to distinguish the main junction."""

    marker = detect_junction_marker(frame)

    if marker is not None and marker["id"] == 23:
        return "MAIN_JUNCTION"

    return base_label
