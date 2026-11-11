#!/usr/bin/env python3
"""Build a Wi-Fi connection test with only Wi-Fi status on USB serial."""

import sys

sys.dont_write_bytecode = True

from build_team_ir import main


if __name__ == "__main__":
    main(wifi_test_only=True)
