#!/usr/bin/env python3
"""Build an isolated Buddy 3 sensing UF2 without changing team examples."""

from pathlib import Path
import os
import shutil
import subprocess
import tempfile

SUBSYSTEM = Path(__file__).resolve().parent
REPO = SUBSYSTEM.parents[1]
IMAGE = "buddy3_ir_sensing_smp1_usb_cdc_wifi_none"
SOURCES = (
    "drv_ir.c", "drv_ir.h", "ir_sensing.c", "ir_sensing.h",
    "ir_line_filter.c", "ir_line_filter.h",
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

        for name in ("config", "device", "etc", "include", "kernel", "lib",
                     "tools"):
            stage_source(REPO / name, stage / name)
        for name in SOURCES:
            stage_source(SUBSYSTEM / name, app / name)

        recipes = REPO / "build_make"
        for source in recipes.rglob("*.mk"):
            destination = build / source.relative_to(recipes)
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(source, destination)

        makefile = (recipes / "makefile").read_text()
        makefile += "\nmtkernel_3/app_program/%.o: CFLAGS += "
        makefile += "-DBUDDY3_SENSING_STANDALONE=1 "
        makefile += "-Wall -Wextra -Werror -Wconversion -Wshadow\n"
        (build / "makefile").write_text(makefile)

        command = ["make", "-j4", "SMP=1", "CONSOLE=usb_cdc", "WIFI=none",
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
