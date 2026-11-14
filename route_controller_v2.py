class RouteController:
    """Offline mission decisions only. No drone commands."""

    def __init__(self):
        self.state = "START"
        self.request_found = False

    def update(self, event, pad_id=None, request_active=None):
        if self.state == "SAFE_STOP":
            return "STOP"

        if event not in ("HOME", "PATH", "TABLE_ZONE"):
            self.state = "SAFE_STOP"
            return "STOP"

        if self.state == "START":
            if event != "HOME":
                return "WAIT_HOME"
            self.state = "TO_B"
            return "SELECT_B"

        if self.state == "TO_B":
            if event == "TABLE_ZONE":
                if pad_id != 3:
                    return "HOLD_VERIFY_PAD"
                if type(request_active) is not bool:
                    return "HOLD_VERIFY_REQUEST"

                self.request_found = request_active
                self.state = "RETURN_B"

                return (
                    "REPORT_REQUEST_B"
                    if request_active else "RETURN_ON_B"
                )
            return "FOLLOW_B"

        if self.state == "RETURN_B":
            if event == "HOME":
                if self.request_found:
                    self.state = "COMPLETE"
                    return "READY_FOR_LANDING"

                self.state = "TO_A"
                return "SELECT_A"

            return "RETURN_ON_B"

        if self.state == "TO_A":
            if event == "TABLE_ZONE":
                if pad_id != 1:
                    return "HOLD_VERIFY_PAD"
                if type(request_active) is not bool:
                    return "HOLD_VERIFY_REQUEST"

                self.request_found = request_active
                self.state = "RETURN_A"

                return (
                    "REPORT_REQUEST_A"
                    if request_active else "RETURN_ON_A"
                )
            return "FOLLOW_A"

        if self.state == "RETURN_A":
            if event == "HOME":
                self.state = "COMPLETE"
                return "READY_FOR_LANDING"

            return "RETURN_ON_A"

        return "STOP"


