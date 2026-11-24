#!/usr/bin/env python3
"""Build team firmware profiles; default kernel tick is 5 ms."""

from pathlib import Path
import os
import re
import shutil
import subprocess
import tempfile

SUBSYSTEM = Path(__file__).resolve().parent
REPO = SUBSYSTEM.parents[1]
IMAGE = "team_ir_handoff_smp1_usb_cdc_wifi_cyw43"
WIFI_IMAGE = "team_ir_handoff_smp1_usb_cdc_wifi_lwip"
NETWORK_IMAGE = "team_ir_handoff_smp1_usb_cdc_wifi_lwip_mqtt"


def main(network=False, mqtt=False):
    if mqtt and not network:
        raise ValueError("MQTT needs the lwIP network profile.")
    if not os.environ.get("PICO_SDK_PATH"):
        raise RuntimeError("Set PICO_SDK_PATH to the Pico SDK directory.")
    if not shutil.which("arm-none-eabi-gcc"):
        raise RuntimeError("Add arm-none-eabi-gcc to PATH.")

    with tempfile.TemporaryDirectory(prefix="team-ir-handoff-") as temp:
        stage = Path(temp)
        build = stage / "build_make"
        build.mkdir()
        (build / "tools").mkdir()
        for name in ("device", "etc", "include", "kernel", "lib", "tools"):
            (stage / name).symlink_to(REPO / name, target_is_directory=True)
        (stage / "app_program").symlink_to(
            REPO / "app_program", target_is_directory=True)
        shutil.copytree(REPO / "config", stage / "config")
        config = stage / "config" / "config.h"
        configured, count = re.subn(
            r"(?m)^(#define\s+CNF_TIMER_PERIOD\s+)\d+\b",
            r"\g<1>5",
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
        if mqtt:
            makefile += ("\nmtkernel_3/app_program/%.o: CFLAGS += "
                         "-DBUDDY3_BARCODE_DEBUG=1\n")
        (build / "makefile").write_text(makefile)
        image = NETWORK_IMAGE if mqtt else WIFI_IMAGE if network else IMAGE
        command = ["make", "-j4", "SMP=1", "CONSOLE=usb_cdc", "WIFI=cyw43",
                   "NET=lwip" if network else "NET=none",
                   "MQTT=1" if mqtt else "MQTT=0",
                   "CRASH_REPORT=0", "TEST=0",
                   f"EXE_FILE={image}"]
        completed = subprocess.run(command, cwd=build, text=True,
                                   stdout=subprocess.PIPE,
                                   stderr=subprocess.STDOUT)
        if completed.returncode:
            print(completed.stdout[-12000:])
            raise RuntimeError("Team IR handoff build failed.")

        symbols = subprocess.run(
            ["arm-none-eabi-nm", str(build / f"{image}.elf")],
            text=True, stdout=subprocess.PIPE, check=True,
        ).stdout
        required = ("ir_sensing_service_init", "ir_motion_consumer_task",
                    "s2_set_ir_velocity")
        for symbol in required:
            if not any(line.endswith(f" {symbol}") for line in symbols.splitlines()):
                raise RuntimeError(f"Integrated image is missing {symbol}.")
        if network:
            executable = (build / f"{image}.elf").read_bytes()
            markers = [b"wifi: joining", b"wifi: link up"]
            if mqtt:
                markers.append(b"mqtt: connected to")
                markers.extend((b"IR_BARCODE_SAMPLE", b"IR_BARCODE_EDGE",
                                b"IR_BARCODE_WINDOW"))
            for marker in markers:
                if marker not in executable:
                    raise RuntimeError(f"Network image is missing {marker!r}.")
        destination = REPO / "build_make" / f"{image}.uf2"
        shutil.copyfile(build / f"{image}.uf2", destination)
        print(f"Built: {destination}")


if __name__ == "__main__":
    main()
