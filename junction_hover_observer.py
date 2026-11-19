"""Compatibility entry point; implementation lives in hover/."""
if __name__ == "__main__":
    from runpy import run_module
    run_module("hover.junction_hover_observer", run_name="__main__")
else:
    from hover.junction_hover_observer import *
