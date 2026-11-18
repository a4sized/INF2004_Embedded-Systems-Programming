#!/usr/bin/env python3
"""USB one-shot pivot calibration; default effective track is 129 mm."""
import argparse
import sys
sys.dont_write_bytecode = True
from build_team_ir import main

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--track-mm", type=int, default=129)
    args = parser.parse_args()
    main(uturn_calibration=True, uturn_track_mm=args.track_mm)
