#!/usr/bin/env python3
"""Host unit tests (S1-U11) verifying Subsystem 1's 8-byte binary telemetry
wire layout, fault definitions, and scripts/telemetry_decoder.py decoding
without hardware.
"""

from contextlib import redirect_stdout
import io
from pathlib import Path
import re
import shutil
import struct
import subprocess
import sys
import tempfile
import types
import unittest

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.dont_write_bytecode = True
sys.path.insert(0, str(REPO_ROOT / "scripts"))
try:
    import telemetry_decoder
except ModuleNotFoundError as exc:
    if exc.name and exc.name.startswith("paho"):
        paho_mod = types.ModuleType("paho")
        mqtt_mod = types.ModuleType("paho.mqtt")
        client_mod = types.ModuleType("paho.mqtt.client")
        sys.modules["paho"] = paho_mod
        sys.modules["paho.mqtt"] = mqtt_mod
        sys.modules["paho.mqtt.client"] = client_mod
        import telemetry_decoder
    else:
        raise


def pack_telemetry(
    *,
    seq: int = 0,
    hump_height_mm: int = 0,
    obstacle_cm: int = 100,
    car_state: int = 0,
    hump_state: int = 0,
    pitch_deg: int = 0,
    line_mask: int = 0,
    barcode_id: int = 0,
    fault_mask: int = 0,
) -> bytes:
    w0 = (
        (seq & 0xFF)
        | ((hump_height_mm & 0x3FF) << 8)
        | ((obstacle_cm & 0x1FF) << 18)
        | ((car_state & 0x07) << 27)
        | ((hump_state & 0x03) << 30)
    )
    w1 = (
        (pitch_deg & 0xFF)
        | ((line_mask & 0x1F) << 8)
        | ((barcode_id & 0x7F) << 13)
        | ((fault_mask & 0xFFF) << 20)
    )
    return struct.pack("<II", w0, w1)


class TelemetryDecoderTests(unittest.TestCase):
    def test_header_bitfields_match_64_bits_and_decoder_order(self):
        header = (REPO_ROOT / "app_program" / "subsystem1" / "subsystem1.h").read_text(
            encoding="utf-8"
        )
        match = re.search(
            r"typedef struct __attribute__\(\(packed\)\)\s*\{(.*?)\}\s*S1_TELEMETRY_PKT;",
            header,
            re.DOTALL,
        )
        self.assertIsNotNone(match)
        fields = re.findall(r"(\w+)\s*:\s*(\d+)\s*;", match.group(1))
        expected = [
            ("seq", "8"),
            ("hump_height_mm", "10"),
            ("obstacle_cm", "9"),
            ("car_state", "3"),
            ("hump_state", "2"),
            ("pitch_deg", "8"),
            ("line_mask", "5"),
            ("barcode_id", "7"),
            ("fault_mask", "12"),
        ]
        self.assertEqual(fields, expected)
        self.assertEqual(sum(int(bits) for _, bits in fields), 64)

    def test_fault_names_match_system_faults_header(self):
        header = (REPO_ROOT / "app_program" / "systemFaults.h").read_text(
            encoding="utf-8"
        )
        defines = {
            name: (1 << int(shift))
            for name, shift in re.findall(
                r"#define\s+(FAULT_\w+)\s+\(1U\s*<<\s*(\d+)\)", header
            )
        }
        self.assertEqual(
            {val: name for name, val in defines.items()},
            telemetry_decoder.FAULT_NAMES,
        )

    def test_decode_sample_telemetry_and_negative_pitch(self):
        payload = pack_telemetry(
            seq=42,
            hump_height_mm=320,
            obstacle_cm=511,
            car_state=2,
            hump_state=3,
            pitch_deg=-18,
            line_mask=0b10110,
            barcode_id=ord("A"),
            fault_mask=(1 << 0) | (1 << 5),
        )
        msg = types.SimpleNamespace(payload=payload)
        buf = io.StringIO()
        with redirect_stdout(buf):
            telemetry_decoder.on_message(None, None, msg)
        out = buf.getvalue()

        self.assertIn("[SEQ #042]", out)
        self.assertIn("MOTOR_STEADY_DRIVE (2)", out)
        self.assertIn("HUMP_PEAK (3)", out)
        self.assertIn("Peak Hump Height = 320 mm", out)
        self.assertIn("Pitch = -18°", out)
        self.assertIn("Obstacle = 511 cm", out)
        self.assertIn("Line IR = [0b10110]", out)
        self.assertIn("Barcode = 'A'", out)
        self.assertIn("FAULT_ESTOP | FAULT_WIFI", out)

    def test_decode_zero_barcode_and_clear_faults(self):
        payload = pack_telemetry(seq=1, obstacle_cm=25, barcode_id=0, fault_mask=0)
        msg = types.SimpleNamespace(payload=payload)
        buf = io.StringIO()
        with redirect_stdout(buf):
            telemetry_decoder.on_message(None, None, msg)
        out = buf.getvalue()

        self.assertIn("Barcode = '0'", out)
        self.assertIn("NONE (ALL CLEAR)", out)

    def test_reject_non_8_byte_payload(self):
        msg = types.SimpleNamespace(payload=b"\x01\x02\x03")
        buf = io.StringIO()
        with redirect_stdout(buf):
            telemetry_decoder.on_message(None, None, msg)
        self.assertIn("Non-8-byte payload received (3 bytes)", buf.getvalue())

    def test_matches_real_c_struct_layout_when_gcc_available(self):
        gcc = shutil.which("gcc")
        if not gcc:
            self.skipTest("gcc not found on PATH")

        c_src = r"""
#include <stdio.h>
#include <stdint.h>
#include "subsystem1/subsystem1.h"

int main(void) {
    S1_TELEMETRY_PKT pkt = {0};
    pkt.seq = 255;
    pkt.hump_height_mm = 789;
    pkt.obstacle_cm = 511;
    pkt.car_state = 4;
    pkt.hump_state = 1;
    pkt.pitch_deg = -45;
    pkt.line_mask = 0x1A;
    pkt.barcode_id = 'Z';
    pkt.fault_mask = (1u << 2) | (1u << 6);

    const uint8_t *b = (const uint8_t *)&pkt;
    printf("%zu:%zu:", sizeof(S1_TELEMETRY_PKT), sizeof(S1_CMD_PKT));
    for (size_t i = 0; i < sizeof(pkt); i++) {
        printf("%02X", b[i]);
    }
    putchar('\n');
    return 0;
}
"""
        with tempfile.TemporaryDirectory(prefix="s1_host_") as tmpdir:
            src_path = Path(tmpdir) / "check_layout.c"
            exe_name = "check_layout.exe" if sys.platform == "win32" else "check_layout"
            exe_path = Path(tmpdir) / exe_name
            src_path.write_text(c_src, encoding="utf-8")

            cmd = [
                gcc,
                "-std=gnu11",
                "-mno-ms-bitfields",
                f"-I{REPO_ROOT / 'testing' / 'subsystem4' / 'host_test' / 'sim'}",
                f"-I{REPO_ROOT / 'app_program'}",
                str(src_path),
                "-o",
                str(exe_path),
            ]
            res = subprocess.run(cmd, text=True, capture_output=True)
            if res.returncode != 0:
                self.skipTest(f"host gcc unavailable in current shell: {res.stderr.strip()}")
            out_line = subprocess.check_output([str(exe_path)], text=True).strip()

        size_pkt, size_cmd, hex_bytes = out_line.split(":")
        self.assertEqual(int(size_pkt), 8)
        self.assertEqual(int(size_cmd), 1)

        raw = bytes.fromhex(hex_bytes)
        msg = types.SimpleNamespace(payload=raw)
        buf = io.StringIO()
        with redirect_stdout(buf):
            telemetry_decoder.on_message(None, None, msg)
        decoded = buf.getvalue()

        self.assertIn("[SEQ #255]", decoded)
        self.assertIn("MOTOR_SAFETY_HALT (4)", decoded)
        self.assertIn("RAMP_UP (1)", decoded)
        self.assertIn("Peak Hump Height = 789 mm", decoded)
        self.assertIn("Pitch = -45°", decoded)
        self.assertIn("Obstacle = 511 cm", decoded)
        self.assertIn("Line IR = [0b11010]", decoded)
        self.assertIn("Barcode = 'Z'", decoded)
        self.assertIn("FAULT_IR_SENS | FAULT_CMD_REJECTED", decoded)


if __name__ == "__main__":
    unittest.main(verbosity=2)
