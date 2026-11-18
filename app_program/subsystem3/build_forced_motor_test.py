#!/usr/bin/env python3
"""Build a single-pulse Subsystem 2 motor test with no motion gate."""

import sys

sys.dont_write_bytecode = True

from build_team_ir import main


if __name__ == "__main__":
    main(force_motor_test=True)
