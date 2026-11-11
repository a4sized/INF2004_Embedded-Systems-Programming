#!/usr/bin/env python3
"""Build the team image with Wi-Fi, lwIP and MQTT enabled."""

import sys

sys.dont_write_bytecode = True

from build_team_ir import main


if __name__ == "__main__":
    main(network=True, mqtt=True)
