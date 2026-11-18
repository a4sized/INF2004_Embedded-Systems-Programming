#!/usr/bin/env python3
"""Build an isolated bench image with the professor's unchanged RTOS sources."""

from pathlib import Path
import os
import shutil
import subprocess
import sys

BENCH = Path(__file__).resolve().parent
REPO = BENCH.parents[1]
STAGE = BENCH / "build" / "stage"
BUILD = STAGE / "build_make"
IMAGE = "ir_calibration_smp1_usb_cdc_wifi_none"


def link_source(source, destination):
    """Refuse to overwrite anything other than the expected source link."""
    if destination.is_symlink() and destination.resolve() == source.resolve():
        return
    if destination.exists() or destination.is_symlink():
        raise RuntimeError(f"Unexpected existing path: {destination}")
    destination.symlink_to(source, target_is_directory=source.is_dir())


def main():
    subprocess.run([sys.executable, str(BENCH / "check_style.py")], check=True)
    if not os.environ.get("PICO_SDK_PATH"):
        raise RuntimeError("Set PICO_SDK_PATH to your Pico SDK directory.")
    if not shutil.which("arm-none-eabi-gcc"):
        raise RuntimeError("Add arm-none-eabi-gcc's bin directory to PATH.")

    BUILD.mkdir(parents=True, exist_ok=True)
    (BUILD / "tools").mkdir(exist_ok=True)
    for name in ("config", "device", "etc", "include", "kernel", "lib", "tools"):
        link_source(REPO / name, STAGE / name)
    application = STAGE / "app_program"
    application.mkdir(exist_ok=True)
    for source in sorted((BENCH / "src").iterdir()):
        link_source(source, application / source.name)

    # Copy build recipes only. Never reuse or clean the team's object tree.
    recipes = REPO / "build_make"
    for source in recipes.rglob("*.mk"):
        destination = BUILD / source.relative_to(recipes)
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source, destination)
    makefile = (recipes / "makefile").read_text()
    makefile += "\nmtkernel_3/app_program/%.o: CFLAGS += "
    makefile += "-Wall -Wextra -Werror -Wconversion -Wshadow\n"
    (BUILD / "makefile").write_text(makefile)

    command = ["make", "-j4", "SMP=1", "CONSOLE=usb_cdc", "WIFI=none",
               "NET=none", "MQTT=0", "CRASH_REPORT=0", "TEST=0",
               f"EXE_FILE={IMAGE}"]
    with (BENCH / "build" / "build.log").open("w") as log:
        log.write(f"Pico SDK: {os.environ['PICO_SDK_PATH']}\n")
        log.write("Build command: " + " ".join(command) + "\n")
        log.flush()
        subprocess.run(["arm-none-eabi-gcc", "--version"], stdout=log,
                       stderr=subprocess.STDOUT, check=True)
        completed = subprocess.run(command, cwd=BUILD, stdout=log,
                                   stderr=subprocess.STDOUT)
    if completed.returncode:
        print((BENCH / "build" / "build.log").read_text()[-12000:])
        raise RuntimeError("Build failed. See build/build.log.")
    for suffix in ("uf2", "elf", "map"):
        shutil.copyfile(BUILD / f"{IMAGE}.{suffix}",
                        BENCH / "build" / f"{IMAGE}.{suffix}")
    print(f"Built: {BENCH / 'build' / (IMAGE + '.uf2')}")
    print(f"Log: {BENCH / 'build' / 'build.log'}")


if __name__ == "__main__":
    main()
