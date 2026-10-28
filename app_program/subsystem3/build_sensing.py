#!/usr/bin/env python3
"""Build the isolated sensing UF2 with Buddy 1's fault Gatekeeper."""

from pathlib import Path
import os
import re
import shutil
import subprocess
import tempfile

SUBSYSTEM = Path(__file__).resolve().parent
REPO = SUBSYSTEM.parents[1]
IMAGE = "buddy3_ir_sensing_smp1_usb_cdc_wifi_cyw43"
SENSING_TICK_MS = 5
SOURCES = (
    "drv_ir.c", "drv_ir.h", "ir_sensing.c", "ir_sensing.h",
    "ir_line_filter.c", "ir_line_filter.h",
    "ir_line_state.c", "ir_line_state.h",
    "ir_time.c", "ir_time.h",
    "ir_barcode_capture.c", "ir_barcode_capture.h",
    "ir_barcode_gpio.c", "ir_barcode_gpio.h",
    "ir_barcode_window.c", "ir_barcode_window.h",
    "ir_barcode_segments.c", "ir_barcode_segments.h",
    "ir_fault_status.c", "ir_fault_status.h",
    "ir_line_snapshot.c", "ir_line_snapshot.h",
    "ir_line_snapshot_shared.c", "ir_line_snapshot_shared.h",
    "ir_sensing_main.c", "ir_sensing_main.h",
)


def stage_source(source, destination):
    """Link only the named source path into the temporary build tree."""
    destination.symlink_to(source, target_is_directory=source.is_dir())


def main():
    if not os.environ.get("PICO_SDK_PATH"):
        raise RuntimeError("Set PICO_SDK_PATH to the Pico SDK directory.")
    if not shutil.which("arm-none-eabi-gcc"):
        raise RuntimeError("Add arm-none-eabi-gcc to PATH.")

    with tempfile.TemporaryDirectory(prefix="buddy3-ir-sensing-") as temp:
        stage = Path(temp)
        build = stage / "build_make"
        app = stage / "app_program"
        build.mkdir()
        (build / "tools").mkdir()
        app.mkdir()
        (app / "subsystem1").mkdir()
        (app / "subsystem3").mkdir()

        for name in ("config", "device", "etc", "include", "kernel", "lib",
                     "tools"):
            if name == "config":
                shutil.copytree(REPO / name, stage / name)
            else:
                stage_source(REPO / name, stage / name)
        config = stage / "config" / "config.h"
        configured, count = re.subn(
            r"(?m)^(#define\s+CNF_TIMER_PERIOD\s+)\d+\b",
            lambda match: f"{match.group(1)}{SENSING_TICK_MS}",
            config.read_text(),
        )
        if count != 1:
            raise RuntimeError("Expected one kernel timer setting in config.h.")
        config.write_text(configured)
        for name in ("app.h", "systemFaults.h"):
            stage_source(REPO / "app_program" / name, app / name)
        for name in ("subsystem1.c", "subsystem1.h", "task_led.c"):
            stage_source(REPO / "app_program" / "subsystem1" / name,
                         app / "subsystem1" / name)
        for name in SOURCES:
            stage_source(SUBSYSTEM / name, app / "subsystem3" / name)

        recipes = REPO / "build_make"
        for source in recipes.rglob("*.mk"):
            destination = build / source.relative_to(recipes)
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(source, destination)

        makefile = (recipes / "makefile").read_text()
        makefile += "\nmtkernel_3/app_program/subsystem3/%.o: CFLAGS += "
        makefile += "-DBUDDY3_SENSING_STANDALONE=1 "
        makefile += "-Wall -Wextra -Werror -Wconversion -Wshadow\n"
        (build / "makefile").write_text(makefile)

        command = ["make", "-j4", "SMP=1", "CONSOLE=usb_cdc", "WIFI=cyw43",
                   "NET=none", "MQTT=0", "CRASH_REPORT=0", "TEST=0",
                   f"EXE_FILE={IMAGE}"]
        completed = subprocess.run(command, cwd=build, text=True,
                                   stdout=subprocess.PIPE,
                                   stderr=subprocess.STDOUT)
        if completed.returncode:
            print(completed.stdout[-12000:])
            raise RuntimeError("Buddy 3 sensing build failed.")

        destination = REPO / "build_make" / f"{IMAGE}.uf2"
        shutil.copyfile(build / f"{IMAGE}.uf2", destination)
        print(f"Built: {destination}")


if __name__ == "__main__":
    main()
