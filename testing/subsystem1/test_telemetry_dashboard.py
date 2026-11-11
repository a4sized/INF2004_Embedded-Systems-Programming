#!/usr/bin/env python3
"""Host unit & HTTP/SSE integration tests for scripts/telemetry_dashboard.py."""

from contextlib import redirect_stdout
import http.client
import io
import json
from pathlib import Path
import sys
import threading
import time
import types
import unittest
import urllib.error
import urllib.request

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.dont_write_bytecode = True
sys.path.insert(0, str(REPO_ROOT / "scripts"))

import telemetry_dashboard  # noqa: E402
import telemetry_decoder  # noqa: E402


class TelemetryDashboardUnitTests(unittest.TestCase):
    def test_decode_matches_telemetry_decoder_on_negative_pitch_and_faults(self):
        payload = telemetry_dashboard.encode_telemetry_payload(
            seq=42,
            hump_height_mm=320,
            obstacle_cm=511,
            car_state=2,
            hump_state=3,
            pitch_deg=-18,
            line_mask=0b10110,
            barcode_id=ord("W"),
            fault_mask=(1 << 0) | (1 << 3) | (1 << 5),
        )
        decoded = telemetry_dashboard.decode_telemetry_payload(payload)
        self.assertEqual(decoded["seq"], 42)
        self.assertEqual(decoded["hump_height_mm"], 320)
        self.assertEqual(decoded["obstacle_cm"], 511)
        self.assertTrue(decoded["obstacle_clear"])
        self.assertEqual(decoded["car_state"], 2)
        self.assertEqual(decoded["car_state_name"], "MOTOR_STEADY_DRIVE")
        self.assertEqual(decoded["hump_state"], 3)
        self.assertEqual(decoded["hump_state_name"], "HUMP_PEAK")
        self.assertEqual(decoded["pitch_deg"], -18)
        self.assertEqual(decoded["line_mask"], 0b10110)
        self.assertEqual(decoded["line_bin"], "0b10110")
        self.assertEqual(decoded["line_bits"], [1, 0, 1, 1, 0])
        self.assertEqual(decoded["barcode_display"], "W")
        self.assertEqual(decoded["fault_hex"], "0x029")
        self.assertTrue(decoded["tier1_fault"])
        self.assertTrue(decoded["tier2_fault"])
        self.assertIn("FAULT_ESTOP", decoded["fault_summary"])
        self.assertIn("FAULT_US_SENS", decoded["fault_summary"])
        self.assertIn("FAULT_WIFI", decoded["fault_summary"])

        buf = io.StringIO()
        with redirect_stdout(buf):
            telemetry_decoder.on_message(None, None, types.SimpleNamespace(payload=payload))
        cli_out = buf.getvalue()
        self.assertIn(f"[SEQ #{decoded['seq']:03d}] Raw HEX: [{decoded['raw_hex']}]", cli_out)
        self.assertIn(decoded["fault_summary"], cli_out)

    def test_decode_zero_barcode_and_invalid_length(self):
        payload = telemetry_dashboard.encode_telemetry_payload(seq=7, barcode_id=0, obstacle_cm=15)
        decoded = telemetry_dashboard.decode_telemetry_payload(payload)
        self.assertEqual(decoded["barcode_id"], 0)
        self.assertEqual(decoded["barcode_display"], "NONE")
        self.assertFalse(decoded["obstacle_clear"])

        with self.assertRaises(ValueError):
            telemetry_dashboard.decode_telemetry_payload(b"\x01\x02\x03")

    def test_encode_command_payload_modes(self):
        raw, desc = telemetry_dashboard.encode_command_payload("estop", mode="binary")
        self.assertEqual(raw, b"\x03")
        self.assertIn("0x03", desc)

        raw_ascii, _ = telemetry_dashboard.encode_command_payload("INVALID", mode="ascii")
        self.assertEqual(raw_ascii, b"INVALID")

        with self.assertRaises(ValueError):
            telemetry_dashboard.encode_command_payload("INVALID", mode="binary")
        with self.assertRaises(ValueError):
            telemetry_dashboard.encode_command_payload("   ", mode="ascii")

    def test_hub_tracks_drops_max_hump_demo_overrides_and_reset(self):
        hub = telemetry_dashboard.TelemetryHub(demo=True, enable_mqtt=False)

        hub.ingest_payload(telemetry_dashboard.encode_telemetry_payload(seq=10, hump_height_mm=22))
        hub.ingest_payload(telemetry_dashboard.encode_telemetry_payload(seq=13, hump_height_mm=45))
        hub.ingest_payload(b"\xFF")

        snap = hub.snapshot()
        self.assertEqual(snap["meta"]["rx_count"], 2)
        self.assertEqual(snap["meta"]["seq_drops"], 2)
        self.assertEqual(snap["meta"]["invalid_count"], 1)
        self.assertEqual(snap["meta"]["session_max_hump_mm"], 45)

        res = hub.send_command("ESTOP", mode="binary")
        self.assertTrue(res["ok"])
        pkt = hub.emit_demo_step()
        self.assertEqual(pkt["car_state"], 4)
        self.assertTrue(pkt["fault_mask"] & (1 << 0))

        hub.send_command("CLEAR", mode="ascii")
        hub.reset_session()
        after_reset = hub.snapshot()
        self.assertEqual(after_reset["meta"]["rx_count"], 0)
        self.assertEqual(after_reset["meta"]["seq_drops"], 0)
        self.assertEqual(after_reset["meta"]["session_max_hump_mm"], 0)
        self.assertIsNone(after_reset["latest"])


class TelemetryDashboardHttpTests(unittest.TestCase):
    def setUp(self) -> None:
        self.hub = telemetry_dashboard.TelemetryHub(demo=False, enable_mqtt=False)
        self.hub.start()
        self.server = telemetry_dashboard.QuietThreadingHTTPServer(
            ("127.0.0.1", 0), telemetry_dashboard.make_handler(self.hub)
        )
        self.port = self.server.server_address[1]
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    def tearDown(self) -> None:
        self.hub.stop()
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=2.0)

    def _request(self, method: str, path: str, body: dict | None = None) -> tuple[int, bytes]:
        url = f"http://127.0.0.1:{self.port}{path}"
        data = None if body is None else json.dumps(body).encode("utf-8")
        headers = {"Content-Type": "application/json"} if data is not None else {}
        req = urllib.request.Request(url, data=data, headers=headers, method=method)
        try:
            with urllib.request.urlopen(req, timeout=3.0) as resp:
                return resp.status, resp.read()
        except urllib.error.HTTPError as exc:
            try:
                return exc.code, exc.read()
            finally:
                exc.close()

    def test_static_assets_and_path_traversal_guard(self):
        code, html = self._request("GET", "/")
        self.assertEqual(code, 200)
        self.assertIn(b"/styles.css", html)
        self.assertIn(b"/app.js", html)

        code_css, css = self._request("GET", "/styles.css")
        self.assertEqual(code_css, 200)
        self.assertIn(b"--bg:", css)

        code_js, js = self._request("GET", "/app.js")
        self.assertEqual(code_js, 200)
        self.assertIn(b"EventSource", js)

        code_fav, _ = self._request("GET", "/favicon.ico")
        self.assertEqual(code_fav, 204)

        conn = http.client.HTTPConnection("127.0.0.1", self.port, timeout=3.0)
        try:
            conn.request("GET", "/../telemetry_decoder.py")
            resp = conn.getresponse()
            resp.read()
            self.assertIn(resp.status, (403, 404))
        finally:
            conn.close()

    def test_api_endpoints_and_live_sse_stream(self):
        conn = http.client.HTTPConnection("127.0.0.1", self.port, timeout=3.0)
        try:
            conn.request("GET", "/events")
            resp = conn.getresponse()
            self.assertEqual(resp.status, 200)
            self.assertIn("text/event-stream", resp.getheader("Content-Type", ""))

            first_lines = [resp.fp.readline().decode("utf-8") for _ in range(2)]
            self.assertEqual(first_lines[0].strip(), "event: init")
            self.assertTrue(first_lines[1].startswith("data: "))
            resp.fp.readline()  # trailing blank line

            code_demo, demo_raw = self._request("POST", "/api/demo", {"enabled": True})
            self.assertEqual(code_demo, 200)
            self.assertTrue(json.loads(demo_raw)["meta"]["demo_mode"])

            # Inject a deterministic frame and verify it arrives on the open SSE stream.
            self.hub.ingest_payload(
                telemetry_dashboard.encode_telemetry_payload(seq=99, pitch_deg=-7, barcode_id=ord("Z")),
                source="mqtt",
            )
            seen_seq99 = False
            for _ in range(30):
                line = resp.fp.readline().decode("utf-8")
                if '"seq":99' in line:
                    seen_seq99 = True
                    break
            self.assertTrue(seen_seq99)
        finally:
            conn.close()

        code_cmd, cmd_raw = self._request("POST", "/api/cmd", {"command": "PING", "mode": "binary"})
        self.assertEqual(code_cmd, 200)
        self.assertEqual(json.loads(cmd_raw)["entry"]["hex"], "05")

        code_bad, _ = self._request("POST", "/api/cmd", {"command": "NOPE", "mode": "binary"})
        self.assertEqual(code_bad, 400)

        code_brk_bad, _ = self._request("POST", "/api/broker", {"broker_ip": "127.0.0.1", "broker_port": 70000})
        self.assertEqual(code_brk_bad, 400)

        code_brk, brk_raw = self._request("POST", "/api/broker", {"broker_ip": "192.168.1.50", "broker_port": 1883})
        self.assertEqual(code_brk, 200)
        self.assertEqual(json.loads(brk_raw)["meta"]["broker_ip"], "192.168.1.50")

    def test_live_demo_thread_and_reset_endpoint(self):
        code, _ = self._request("POST", "/api/demo", {"enabled": True})
        self.assertEqual(code, 200)

        snap = {}
        for _ in range(30):
            code_state, raw = self._request("GET", "/api/state")
            self.assertEqual(code_state, 200)
            snap = json.loads(raw)
            if snap["meta"]["rx_count"] >= 1 and snap.get("latest", {}).get("source") == "demo":
                break
            time.sleep(0.05)

        self.assertGreaterEqual(snap["meta"]["rx_count"], 1)
        self.assertEqual(snap["latest"]["source"], "demo")

        code_demo_off, _ = self._request("POST", "/api/demo", {"enabled": False})
        self.assertEqual(code_demo_off, 200)

        code_rst, rst_raw = self._request("POST", "/api/reset", {})
        self.assertEqual(code_rst, 200)
        rst = json.loads(rst_raw)
        self.assertTrue(rst["ok"])
        self.assertEqual(rst["meta"]["rx_count"], 0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
