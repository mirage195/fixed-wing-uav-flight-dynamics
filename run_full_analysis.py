"""Backward-compatible launcher for the installed command-line interface."""

from fixed_wing_uav.cli import main

if __name__ == "__main__":
    raise SystemExit(main())
