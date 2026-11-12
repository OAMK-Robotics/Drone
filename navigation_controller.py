class NavigationController:
    def __init__(self):
        self.state = "START"
        self.target = "TABLE_A"

    def update(self, vision, pad=None):

        # Unknown vision must not cause navigation.
        if self.state == "SAFE_STOP":
            return "STOP"

        if vision == "UNKNOWN":
            self.state = "SAFE_STOP"
            return "STOP"

        if self.state == "START":
            if vision != "JUNCTION":
                return "WAIT_FOR_JUNCTION"

            self.state = "SEARCH_TABLE_A"
            return "SELECT_TABLE_A"

        if self.state == "SEARCH_TABLE_A":
            if vision == "TABLE_ZONE":

                if pad == 1:
                    self.state = "RETURN"
                    return "PUBLISH_TABLE_1"

                if pad is not None and pad >= 0:
                    self.state = "RETURN_TO_JUNCTION"
                    return "RETURN_TO_JUNCTION"

                return "RETRY_PAD"

            return "FOLLOW_TABLE_A"

        if self.state == "RETURN_TO_JUNCTION":

            if vision == "MAIN_JUNCTION":
                self.state = "SEARCH_TABLE_B"
                self.target = "TABLE_B"
                return "SELECT_TABLE_B"

            return "RETURN_TO_JUNCTION"

        if self.state == "SEARCH_TABLE_B":
            if vision == "TABLE_ZONE":

                if pad == 3:
                    self.state = "RETURN"
                    return "PUBLISH_TABLE_3"

                if pad is not None and pad >= 0:
                    self.state = "RETURN"
                    return "NO_REQUEST"

                return "RETRY_PAD"

            return "FOLLOW_TABLE_B"

        if self.state == "RETURN":
            return "RETURN_TO_START"

        return "STOP"



