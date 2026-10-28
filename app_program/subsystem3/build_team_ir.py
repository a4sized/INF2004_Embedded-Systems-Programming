#!/usr/bin/env python3
"""Build a team image with the IR service and a staged 5 ms kernel tick."""

from pathlib import Path
import os
import re
import shutil
import subprocess
import tempfile

SUBSYSTEM = Path(__file__).resolve().parent
REPO = SUBSYSTEM.parents[1]
IMAGE = "team_ir_handoff_smp1_usb_cdc_wifi_cyw43"


def main():
    if not os.environ.get("PICO_SDK_PATH"):
        raise RuntimeError("Set PICO_SDK_PATH to the Pico SDK directory.")
    if not shutil.which("arm-none-eabi-gcc"):
        raise RuntimeError("Add arm-none-eabi-gcc to PATH.")

    with tempfile.TemporaryDirectory(prefix="team-ir-handoff-") as temp:
        stage = Path(temp)
        build = stage / "build_make"
        build.mkdir()
        (build / "tools").mkdir()
        for name in ("app_program", "device", "etc", "include", "kernel",
                     "lib", "tools"):
            (stage / name).symlink_to(REPO / name, target_is_directory=True)
        shutil.copytree(REPO / "config", stage / "config")
        config = stage / "config" / "config.h"
        configured, count = re.subn(
            r"(?m)^(#define\s+CNF_TIMER_PERIOD\s+)\d+\b",
            lambda match: f"{match.group(1)}5",
            config.read_text(),
        )
        if count != 1:
            raise RuntimeError("Expected one kernel timer setting in config.h.")
        config.write_text(configured)

        recipes = REPO / "build_make"
        for source in recipes.rglob("*.mk"):
            destination = build / source.relative_to(recipes)
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(source, destination)
        makefile = (recipes / "makefile").read_text()
        makefile += "\nmtkernel_3/app_program/%.o: CFLAGS += -DBUDDY3_TEAM_IR=1\n"
        (build / "makefile").write_text(makefile)

        command = ["make", "-j4", "SMP=1", "CONSOLE=usb_cdc", "WIFI=cyw43",
                   "NET=none", "MQTT=0", "CRASH_REPORT=0", "TEST=0",
                   f"EXE_FILE={IMAGE}"]
        completed = subprocess.run(command, cwd=build, text=True,
                                   stdout=subprocess.PIPE,
                                   stderr=subprocess.STDOUT)
        if completed.returncode:
            print(completed.stdout[-12000:])
            raise RuntimeError("Team IR handoff build failed.")

        symbols = subprocess.run(
            ["arm-none-eabi-nm", str(build / f"{IMAGE}.elf")],
            text=True, stdout=subprocess.PIPE, check=True,
        ).stdout
        for symbol in ("ir_sensing_service_init", "ir_motion_consumer_task",
                       "s2_set_ir_velocity"):
            if not any(line.endswith(f" {symbol}") for line in symbols.splitlines()):
                raise RuntimeError(f"Integrated image is missing {symbol}.")

        destination = REPO / "build_make" / f"{IMAGE}.uf2"
        shutil.copyfile(build / f"{IMAGE}.uf2", destination)
        print(f"Built: {destination}")


if __name__ == "__main__":
    main()
