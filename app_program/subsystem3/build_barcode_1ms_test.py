#!/usr/bin/env python3
"""Networked barcode trial: 1 ms raw capture, 5 ms line publication."""
import sys
sys.dont_write_bytecode = True
from build_team_ir import main

if __name__ == "__main__":
    main(network=True, mqtt=True, barcode_1ms_test=True)
