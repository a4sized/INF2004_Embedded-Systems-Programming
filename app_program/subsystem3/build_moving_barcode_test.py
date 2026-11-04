#!/usr/bin/env python3
"""Build the USB-armed integrated line-following/barcode diagnostic image."""

import sys

sys.dont_write_bytecode = True

from build_team_ir import main


if __name__ == "__main__":
    main(moving_test=True)
