#!/usr/bin/env python3
"""Compile and execute portable calibration checks on the host."""

from pathlib import Path
import os
import subprocess
import sys

ROOT = Path(__file__).resolve().parent
subprocess.run([sys.executable, str(ROOT / "check_style.py")], check=True)
(ROOT / "build").mkdir(exist_ok=True)
executable = ROOT / "build" / "ir_calib_host_test"
subprocess.run([os.environ.get("CC", "cc"), "-std=c11", "-Wall", "-Wextra",
                "-Werror", "-Wconversion", "-Wshadow", "-Wstrict-prototypes",
                "-fsanitize=address,undefined", "-g", "-I", str(ROOT / "src"),
                str(ROOT / "src" / "ir_calib.c"),
                str(ROOT / "tests" / "ir_check_main.c"),
                "-o", str(executable)], check=True)
subprocess.run([str(executable)], check=True)
print("Host tests passed with AddressSanitizer and UndefinedBehaviorSanitizer.")

