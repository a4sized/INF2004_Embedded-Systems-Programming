#!/usr/bin/env python3
"""
telemetry_dashboard.py
Local Web Ground Station Dashboard for Robot Car (INF2004 Team 15)

Bridges the pico's 8-byte binary MQTT telemetry (decoded identically to
scripts/telemetry_decoder.py) and inbound MQTT commands into a zero-dependency
offline web UI served over localhost (HTTP + Server-Sent Events).

Usage:
    python scripts/telemetry_dashboard.py [BROKER_IP]
    python scripts/telemetry_dashboard.py --demo
    python scripts/telemetry_dashboard.py 10.33.147.245 --http-port 8080
"""

from __future__ import annotations

import argparse
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import mimetypes
import os
from pathlib import Path
import queue
import struct
import sys
import threading
import time
from typing import Any
import urllib.parse
import webbrowser

SCRIPT_DIR = Path(__file__).resolve().parent
STATIC_DIR = SCRIPT_DIR / "dashboard"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

try:
    import paho.mqtt.client as mqtt  # type: ignore
except ModuleNotFoundError:
    mqtt = None  # Allows --demo mode and unit tests without paho-mqtt.
    import types

    paho_mod = types.ModuleType("paho")
    mqtt_mod = types.ModuleType("paho.mqtt")
    client_mod = types.ModuleType("paho.mqtt.client")
    sys.modules.setdefault("paho", paho_mod)
    sys.modules.setdefault("paho.mqtt", mqtt_mod)
    sys.modules.setdefault("paho.mqtt.client", client_mod)
    import telemetry_decoder  # type: ignore
else:
    import telemetry_decoder  # type: ignore


BROKER_IP_DEFAULT: str = telemetry_decoder.BROKER_IP
BROKER_PORT_DEFAULT: int = telemetry_decoder.BROKER_PORT
TOPIC_TELEM_DEFAULT: str = telemetry_decoder.TOPIC_TELEM
TOPIC_CMD_DEFAULT: str = "robot/inf2004team15/cmd"

FAULT_NAMES: dict[int, str] = dict(telemetry_decoder.FAULT_NAMES)
STATE_NAMES: dict[int, str] = dict(telemetry_decoder.STATE_NAMES)
HUMP_NAMES: dict[int, str] = dict(telemetry_decoder.HUMP_NAMES)

TIER1_FATAL_MASK = (1 << 0) | (1 << 1) | (1 << 2) | (1 << 4) | (1 << 7)
TIER2_HAZARD_MASK = 1 << 3

CMD_NAME_TO_ID: dict[str, int] = {
    "START": 1,
    "STOP": 2,
    "ESTOP": 3,
    "CLEAR": 4,
    "PING": 5,
}


def decode_telemetry_payload(payload: bytes) -> dict[str, Any]:
    """Unpack S1_TELEMETRY_PKT (8-byte little-endian <II) using decoder maps."""
    raw_hex = " ".join(f"{b:02X}" for b in payload)
    if len(payload) != 8:
        raise ValueError(f"Expected 8-byte payload, got {len(payload)} bytes [{raw_hex}]")

    w0, w1 = struct.unpack("<II", payload)

    seq = w0 & 0xFF
    hump_height_mm = (w0 >> 8) & 0x3FF
    obstacle_cm = (w0 >> 18) & 0x1FF
    car_state = (w0 >> 27) & 0x07
    hump_state = (w0 >> 30) & 0x03

    pitch_raw = w1 & 0xFF
    pitch_deg = pitch_raw if pitch_raw < 128 else pitch_raw - 256
    line_mask = (w1 >> 8) & 0x1F
    barcode_id = (w1 >> 13) & 0x7F
    fault_mask = (w1 >> 20) & 0xFFF

    state_name = STATE_NAMES.get(car_state, f"STATE_{car_state}")
    hump_name = HUMP_NAMES.get(hump_state, f"HUMP_{hump_state}")
    fault_str = telemetry_decoder.decode_faults(fault_mask)

    active_faults: list[dict[str, Any]] = []
    for bit, name in sorted(FAULT_NAMES.items()):
        if fault_mask & bit:
            tier = "tier1" if (bit & TIER1_FATAL_MASK) else ("tier2" if (bit & TIER2_HAZARD_MASK) else "info")
            active_faults.append({"bit": bit, "hex": f"0x{bit:03X}", "name": name, "tier": tier})

    known_mask = sum(FAULT_NAMES.keys())
    unknown = fault_mask & ~known_mask
    if unknown:
        active_faults.append({"bit": unknown, "hex": f"0x{unknown:03X}", "name": f"UNKNOWN(0x{unknown:X})", "tier": "tier1"})

    barcode_char = chr(barcode_id) if 32 <= barcode_id <= 126 else ""

    return {
        "seq": seq,
        "raw_hex": raw_hex,
        "w0_hex": f"0x{w0:08X}",
        "w1_hex": f"0x{w1:08X}",
        "car_state": car_state,
        "car_state_name": state_name,
        "hump_state": hump_state,
        "hump_state_name": hump_name,
        "hump_height_mm": hump_height_mm,
        "pitch_deg": pitch_deg,
        "obstacle_cm": obstacle_cm,
        "obstacle_clear": obstacle_cm == 511,
        "line_mask": line_mask,
        "line_bin": f"0b{line_mask:05b}",
        "line_bits": [(line_mask >> i) & 1 for i in range(4, -1, -1)],
        "barcode_id": barcode_id,
        "barcode_char": barcode_char,
        "barcode_display": barcode_char if barcode_char else ("NONE" if barcode_id == 0 else str(barcode_id)),
        "fault_mask": fault_mask,
        "fault_hex": f"0x{fault_mask:03X}",
        "fault_summary": fault_str,
        "active_faults": active_faults,
        "tier1_fault": bool(fault_mask & TIER1_FATAL_MASK),
        "tier2_fault": bool(fault_mask & TIER2_HAZARD_MASK),
    }


def encode_telemetry_payload(
    *,
    seq: int = 0,
    hump_height_mm: int = 0,
    obstacle_cm: int = 511,
    car_state: int = 0,
    hump_state: int = 0,
    pitch_deg: int = 0,
    line_mask: int = 0,
    barcode_id: int = 0,
    fault_mask: int = 0,
) -> bytes:
    """Pack telemetry fields into 8 little-endian bytes matching S1_TELEMETRY_PKT."""
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


def encode_command_payload(command: str, mode: str = "ascii") -> tuple[bytes, str]:
    """Encode an inbound command payload (ASCII string or 1-byte S1_CMD_PKT)."""
    cleaned = command.strip()
    if not cleaned:
        raise ValueError("Command payload cannot be empty.")

    upper = cleaned.upper()
    if mode == "binary":
        if upper in CMD_NAME_TO_ID:
            cmd_id = CMD_NAME_TO_ID[upper]
            return bytes([cmd_id]), f"BIN 0x{cmd_id:02X} ({upper})"
        if cleaned.isdigit() and 1 <= int(cleaned) <= 5:
            cmd_id = int(cleaned)
            return bytes([cmd_id]), f"BIN 0x{cmd_id:02X}"
        raise ValueError(f"Binary mode supports START/STOP/ESTOP/CLEAR/PING (or 1..5), got '{cleaned}'.")

    return cleaned.encode("utf-8"), f"ASCII '{cleaned}'"


@dataclass
class DashboardState:
    broker_ip: str = BROKER_IP_DEFAULT
    broker_port: int = BROKER_PORT_DEFAULT
    topic_telem: str = TOPIC_TELEM_DEFAULT
    topic_cmd: str = TOPIC_CMD_DEFAULT
    mqtt_status: str = "disconnected"
    mqtt_error: str = ""
    demo_mode: bool = False
    rx_count: int = 0
    invalid_count: int = 0
    seq_drops: int = 0
    session_max_hump_mm: int = 0
    last_seq: int | None = None
    last_rx_epoch: float | None = None
    packets: deque[dict[str, Any]] = field(default_factory=lambda: deque(maxlen=240))
    events: deque[dict[str, Any]] = field(default_factory=lambda: deque(maxlen=160))
    commands: deque[dict[str, Any]] = field(default_factory=lambda: deque(maxlen=60))
    barcodes: deque[dict[str, Any]] = field(default_factory=lambda: deque(maxlen=40))


class TelemetryHub:
    """Thread-safe hub managing MQTT, optional Demo generator, and SSE subscribers."""

    def __init__(
        self,
        *,
        broker_ip: str = BROKER_IP_DEFAULT,
        broker_port: int = BROKER_PORT_DEFAULT,
        topic_telem: str = TOPIC_TELEM_DEFAULT,
        topic_cmd: str = TOPIC_CMD_DEFAULT,
        demo: bool = False,
        enable_mqtt: bool = True,
    ) -> None:
        self._lock = threading.Lock()
        initial_status = "demo" if demo else ("connecting" if (enable_mqtt and mqtt is not None) else "disconnected")
        self._state = DashboardState(
            broker_ip=broker_ip,
            broker_port=broker_port,
            topic_telem=topic_telem,
            topic_cmd=topic_cmd,
            mqtt_status=initial_status,
            demo_mode=demo,
        )
        self._subscribers: set[queue.Queue[str | None]] = set()
        self._mqtt_client: Any = None
        self._enable_mqtt = enable_mqtt and (mqtt is not None)
        self._stop_event = threading.Event()
        self._demo_wakeup = threading.Event()
        self._demo_step = 0
        self._demo_override_fault = 0
        self._demo_override_state: int | None = None
        self._demo_thread: threading.Thread | None = None
        self._last_car_state: int | None = None
        self._last_fault_mask: int | None = None
        self._last_barcode_id: int | None = None

        if not self._enable_mqtt and not demo:
            self._state.mqtt_error = "paho-mqtt is not installed; use --demo mode or install paho-mqtt."

    def start(self) -> None:
        if self._enable_mqtt and not self._state.demo_mode:
            self._start_mqtt()
        self._demo_thread = threading.Thread(target=self._demo_loop, name="telemetry-demo", daemon=True)
        self._demo_thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        self._demo_wakeup.set()
        self._stop_mqtt()
        with self._lock:
            subs = list(self._subscribers)
        for q in subs:
            try:
                q.put_nowait(None)
            except Exception:
                pass

    def _broadcast(self, event_name: str, data: dict[str, Any]) -> None:
        frame = f"event: {event_name}\ndata: {json.dumps(data, separators=(',', ':'))}\n\n"
        with self._lock:
            dead: list[queue.Queue[str | None]] = []
            for q in self._subscribers:
                try:
                    q.put_nowait(frame)
                except queue.Full:
                    try:
                        q.get_nowait()
                        q.put_nowait(frame)
                    except Exception:
                        dead.append(q)
            for q in dead:
                self._subscribers.discard(q)

    def subscribe(self) -> queue.Queue[str | None]:
        q: queue.Queue[str | None] = queue.Queue(maxsize=128)
        with self._lock:
            self._subscribers.add(q)
        return q

    def unsubscribe(self, q: queue.Queue[str | None]) -> None:
        with self._lock:
            self._subscribers.discard(q)

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            latest = self._state.packets[-1] if self._state.packets else None
            return {
                "meta": self._meta_dict_locked(),
                "latest": latest,
                "history": list(self._state.packets)[-90:],
                "events": list(self._state.events)[-80:],
                "commands": list(self._state.commands)[-25:],
                "barcodes": list(self._state.barcodes)[-15:],
                "dictionaries": {
                    "states": STATE_NAMES,
                    "humps": HUMP_NAMES,
                    "faults": [
                        {
                            "bit": bit,
                            "hex": f"0x{bit:03X}",
                            "name": name,
                            "tier": "tier1" if (bit & TIER1_FATAL_MASK) else ("tier2" if (bit & TIER2_HAZARD_MASK) else "info"),
                        }
                        for bit, name in sorted(FAULT_NAMES.items())
                    ],
                },
            }

    def _meta_dict_locked(self) -> dict[str, Any]:
        return {
            "broker_ip": self._state.broker_ip,
            "broker_port": self._state.broker_port,
            "topic_telem": self._state.topic_telem,
            "topic_cmd": self._state.topic_cmd,
            "mqtt_status": self._state.mqtt_status,
            "mqtt_error": self._state.mqtt_error,
            "demo_mode": self._state.demo_mode,
            "rx_count": self._state.rx_count,
            "invalid_count": self._state.invalid_count,
            "seq_drops": self._state.seq_drops,
            "session_max_hump_mm": self._state.session_max_hump_mm,
            "last_rx_epoch": self._state.last_rx_epoch,
            "paho_available": mqtt is not None,
        }

    def _push_event_locked(self, kind: str, message: str, level: str = "info") -> dict[str, Any]:
        entry = {
            "time": datetime.now().strftime("%H:%M:%S.%f")[:-3],
            "epoch": time.time(),
            "kind": kind,
            "level": level,
            "message": message,
        }
        self._state.events.append(entry)
        return entry

    def ingest_payload(self, payload: bytes, *, source: str = "mqtt") -> dict[str, Any] | None:
        now_epoch = time.time()
        ts_str = datetime.fromtimestamp(now_epoch).strftime("%H:%M:%S.%f")[:-3]
        try:
            decoded = decode_telemetry_payload(payload)
        except ValueError as exc:
            with self._lock:
                self._state.invalid_count += 1
                ev = self._push_event_locked("WARN", str(exc), "warn")
                meta = self._meta_dict_locked()
            self._broadcast("status", {"meta": meta, "event": ev})
            return None

        decoded["rx_time"] = ts_str
        decoded["rx_epoch"] = now_epoch
        decoded["source"] = source

        new_events: list[dict[str, Any]] = []
        with self._lock:
            self._state.rx_count += 1
            self._state.last_rx_epoch = now_epoch
            if decoded["hump_height_mm"] > self._state.session_max_hump_mm:
                self._state.session_max_hump_mm = decoded["hump_height_mm"]

            if self._state.last_seq is not None:
                expected = (self._state.last_seq + 1) & 0xFF
                if decoded["seq"] != expected:
                    gap = (decoded["seq"] - expected) & 0xFF
                    if gap > 0:
                        self._state.seq_drops += gap
                        new_events.append(
                            self._push_event_locked(
                                "SEQ_GAP",
                                f"Expected #{expected:03d}, got #{decoded['seq']:03d} (+{gap} dropped)",
                                "warn",
                            )
                        )
            self._state.last_seq = decoded["seq"]

            if self._last_car_state is None or decoded["car_state"] != self._last_car_state:
                prev = STATE_NAMES.get(self._last_car_state, "INIT") if self._last_car_state is not None else "NONE"
                new_events.append(
                    self._push_event_locked(
                        "STATE",
                        f"{prev} -> {decoded['car_state_name']} ({decoded['car_state']})",
                        "alert" if decoded["car_state"] in (4, 6, 7) else "info",
                    )
                )
                self._last_car_state = decoded["car_state"]

            if self._last_fault_mask is None or decoded["fault_mask"] != self._last_fault_mask:
                if decoded["fault_mask"] != 0:
                    lvl = "alert" if decoded["tier1_fault"] else "warn"
                    new_events.append(
                        self._push_event_locked(
                            "FAULT",
                            f"[{decoded['fault_hex']}] {decoded['fault_summary']}",
                            lvl,
                        )
                    )
                elif self._last_fault_mask not in (None, 0):
                    new_events.append(self._push_event_locked("FAULT_CLEAR", "[0x000] ALL CLEAR", "ok"))
                self._last_fault_mask = decoded["fault_mask"]

            barcode_id = decoded["barcode_id"]
            if barcode_id != 0 and barcode_id != self._last_barcode_id:
                label = decoded["barcode_display"]
                self._state.barcodes.append(
                    {
                        "seq": decoded["seq"],
                        "time": ts_str,
                        "id": barcode_id,
                        "char": label,
                    }
                )
                new_events.append(
                    self._push_event_locked("BARCODE", f"Decoded barcode '{label}' (seq #{decoded['seq']:03d})", "ok")
                )
            self._last_barcode_id = barcode_id

            self._state.packets.append(decoded)
            meta = self._meta_dict_locked()
            barcodes = list(self._state.barcodes)[-15:]

        self._broadcast(
            "telemetry",
            {
                "packet": decoded,
                "meta": meta,
                "events": new_events,
                "barcodes": barcodes,
            },
        )
        return decoded

    def reset_session(self) -> dict[str, Any]:
        with self._lock:
            self._state.rx_count = 0
            self._state.invalid_count = 0
            self._state.seq_drops = 0
            self._state.session_max_hump_mm = 0
            self._state.last_seq = None
            self._state.last_rx_epoch = None
            self._state.packets.clear()
            self._state.events.clear()
            self._state.barcodes.clear()
            self._last_car_state = None
            self._last_fault_mask = None
            self._last_barcode_id = None
            self._demo_step = 0
            self._demo_override_fault = 0
            self._demo_override_state = None
            ev = self._push_event_locked("SESSION", "Dashboard counters & buffers cleared", "info")
            meta = self._meta_dict_locked()
        self._broadcast("reset", {"meta": meta, "event": ev})
        return {"ok": True, "meta": meta}

    def set_demo_mode(self, enabled: bool) -> dict[str, Any]:
        with self._lock:
            self._state.demo_mode = bool(enabled)
            if enabled:
                self._state.mqtt_status = "demo"
                self._state.mqtt_error = ""
                ev = self._push_event_locked("DEMO", "Switched to built-in telemetry simulator", "info")
            else:
                self._demo_override_fault = 0
                self._demo_override_state = None
                ev = self._push_event_locked("MQTT", f"Connecting to {self._state.broker_ip}:{self._state.broker_port}...", "info")
            meta = self._meta_dict_locked()

        if enabled:
            self._stop_mqtt()
            self._demo_wakeup.set()
        elif self._enable_mqtt:
            self._start_mqtt()
        else:
            with self._lock:
                self._state.mqtt_status = "disconnected"
                self._state.mqtt_error = "paho-mqtt is not installed on this Python environment."
                meta = self._meta_dict_locked()

        self._broadcast("status", {"meta": meta, "event": ev})
        return {"ok": True, "meta": meta}

    def configure_broker(self, broker_ip: str, broker_port: int | None = None) -> dict[str, Any]:
        cleaned_ip = broker_ip.strip()
        if not cleaned_ip:
            raise ValueError("Broker IP/hostname cannot be empty.")
        port = int(broker_port) if broker_port is not None else self._state.broker_port
        if not (1 <= port <= 65535):
            raise ValueError("Broker port must be 1..65535.")

        with self._lock:
            self._state.broker_ip = cleaned_ip
            self._state.broker_port = port
            self._state.demo_mode = False
            self._state.mqtt_status = "connecting"
            self._state.mqtt_error = ""
            ev = self._push_event_locked("BROKER", f"Target set to {cleaned_ip}:{port}", "info")
            meta = self._meta_dict_locked()

        self._stop_mqtt()
        if self._enable_mqtt:
            self._start_mqtt()
        else:
            with self._lock:
                self._state.mqtt_status = "disconnected"
                self._state.mqtt_error = "paho-mqtt is not installed on this Python environment."
                meta = self._meta_dict_locked()
        self._broadcast("status", {"meta": meta, "event": ev})
        return {"ok": True, "meta": meta}

    def send_command(self, command: str, mode: str = "ascii") -> dict[str, Any]:
        raw_bytes, display = encode_command_payload(command, mode=mode)
        ts_str = datetime.now().strftime("%H:%M:%S.%f")[:-3]

        with self._lock:
            demo_mode = self._state.demo_mode
            topic_cmd = self._state.topic_cmd
            client = self._mqtt_client
            mqtt_status = self._state.mqtt_status

        if demo_mode:
            entry = {
                "time": ts_str,
                "topic": topic_cmd,
                "display": f"{display} [DEMO]",
                "hex": raw_bytes.hex(" ").upper(),
                "status": "simulated",
            }
            with self._lock:
                self._state.commands.append(entry)
                ev = self._push_event_locked("CMD_TX", f"DEMO {display} -> {topic_cmd}", "ok")
                cmds = list(self._state.commands)[-25:]
                meta = self._meta_dict_locked()
            self._apply_demo_command(command.strip().upper())
            self._broadcast("command", {"command": entry, "commands": cmds, "meta": meta, "event": ev})
            return {"ok": True, "entry": entry}

        if not self._enable_mqtt or client is None or mqtt_status != "connected":
            raise RuntimeError("MQTT broker is not connected (connect Mosquitto or switch to Demo Mode).")

        info = client.publish(topic_cmd, payload=raw_bytes, qos=0, retain=False)
        ok = getattr(info, "rc", 0) == 0
        entry = {
            "time": ts_str,
            "topic": topic_cmd,
            "display": display,
            "hex": raw_bytes.hex(" ").upper(),
            "status": "sent" if ok else f"rc={getattr(info, 'rc', '?')}",
        }
        with self._lock:
            self._state.commands.append(entry)
            ev = self._push_event_locked("CMD_TX", f"Published {display} -> {topic_cmd}", "ok" if ok else "warn")
            cmds = list(self._state.commands)[-25:]
            meta = self._meta_dict_locked()
        self._broadcast("command", {"command": entry, "commands": cmds, "meta": meta, "event": ev})
        return {"ok": ok, "entry": entry}

    def _start_mqtt(self) -> None:
        self._stop_mqtt()
        if not self._enable_mqtt or mqtt is None:
            return

        with self._lock:
            broker_ip = self._state.broker_ip
            broker_port = self._state.broker_port
            topic_telem = self._state.topic_telem
            self._state.mqtt_status = "connecting"
            self._state.mqtt_error = ""

        try:
            client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        except AttributeError:
            client = mqtt.Client()

        def on_connect(cli: Any, userdata: Any, flags: Any, rc: Any, properties: Any = None) -> None:
            code = getattr(rc, "value", rc)
            if code == 0:
                cli.subscribe(topic_telem)
                with self._lock:
                    self._state.mqtt_status = "connected"
                    self._state.mqtt_error = ""
                    ev = self._push_event_locked("MQTT", f"Connected to {broker_ip}:{broker_port}", "ok")
                    meta = self._meta_dict_locked()
                self._broadcast("status", {"meta": meta, "event": ev})
            else:
                with self._lock:
                    self._state.mqtt_status = "error"
                    self._state.mqtt_error = f"Broker rejected connection (rc={rc})"
                    ev = self._push_event_locked("MQTT", self._state.mqtt_error, "alert")
                    meta = self._meta_dict_locked()
                self._broadcast("status", {"meta": meta, "event": ev})

        def on_disconnect(cli: Any, userdata: Any, *args: Any) -> None:
            with self._lock:
                if self._state.demo_mode:
                    return
                self._state.mqtt_status = "disconnected"
                ev = self._push_event_locked("MQTT", "Disconnected from broker (retrying...)", "warn")
                meta = self._meta_dict_locked()
            self._broadcast("status", {"meta": meta, "event": ev})

        def on_message(cli: Any, userdata: Any, msg: Any) -> None:
            self.ingest_payload(bytes(msg.payload), source="mqtt")

        def on_connect_fail(cli: Any, userdata: Any) -> None:
            with self._lock:
                if self._state.demo_mode:
                    return
                self._state.mqtt_status = "disconnected"
                self._state.mqtt_error = (
                    f"Waiting for Mosquitto at {broker_ip}:{broker_port} "
                    "(start Mosquitto, update IP, or toggle Demo Sim)."
                )
                meta = self._meta_dict_locked()
            self._broadcast("status", {"meta": meta})

        client.on_connect = on_connect
        client.on_disconnect = on_disconnect
        client.on_message = on_message
        client.on_connect_fail = on_connect_fail
        self._mqtt_client = client

        try:
            client.connect_async(broker_ip, broker_port, 60)
            client.loop_start()
        except Exception as exc:
            with self._lock:
                self._state.mqtt_status = "error"
                self._state.mqtt_error = str(exc)
                ev = self._push_event_locked("MQTT", f"Connect error: {exc}", "alert")
                meta = self._meta_dict_locked()
            self._broadcast("status", {"meta": meta, "event": ev})

    def _stop_mqtt(self) -> None:
        client = self._mqtt_client
        self._mqtt_client = None
        if client is not None:
            try:
                client.loop_stop()
            except Exception:
                pass
            try:
                client.disconnect()
            except Exception:
                pass

    def _apply_demo_command(self, upper_cmd: str) -> None:
        with self._lock:
            if upper_cmd == "ESTOP":
                self._demo_override_fault = 1 << 0
                self._demo_override_state = 4
            elif upper_cmd == "STOP":
                self._demo_override_state = 0
            elif upper_cmd == "CLEAR":
                self._demo_override_fault = 0
                self._demo_override_state = None
            elif upper_cmd == "START":
                self._demo_override_fault = 0
                self._demo_override_state = 2
            elif upper_cmd == "INVALID":
                self._demo_override_fault = 1 << 6

    _DEMO_TIMELINE = [
        # (car_state, hump_state, hump_mm, pitch_deg, obstacle_cm, line_mask, barcode_id, fault_mask)
        (0, 0, 0, 0, 511, 0b01100, 0, 0),
        (1, 0, 0, 2, 511, 0b01100, 0, 0),
        (2, 0, 0, 1, 48, 0b01100, 0, 0),
        (2, 1, 14, 11, 511, 0b01100, 0, 0),
        (2, 3, 36, 3, 511, 0b01100, 0, 0),
        (5, 2, 36, -12, 511, 0b01100, 0, 0),
        (2, 0, 36, 0, 511, 0b01100, ord("W"), 0),
        (2, 0, 36, -1, 511, 0b10000, 0, 0),
        (2, 0, 36, 0, 28, 0b01100, 0, 0),
        (0, 0, 36, 0, 10, 0b00000, 0, 0),
        (2, 0, 36, 1, 22, 0b00110, ord("Z"), 0),
        (6, 0, 36, 24, 511, 0b00000, 0, 1 << 4),
        (0, 0, 36, 0, 511, 0b01100, 0, 0),
    ]

    def emit_demo_step(self) -> dict[str, Any] | None:
        with self._lock:
            step = self._demo_step
            row = self._DEMO_TIMELINE[step % len(self._DEMO_TIMELINE)]
            car_state, hump_state, hump_mm, pitch_deg, obstacle_cm, line_mask, barcode_id, fault_mask = row
            if self._demo_override_state is not None:
                car_state = self._demo_override_state
            if self._demo_override_fault:
                fault_mask |= self._demo_override_fault
            self._demo_step = (step + 1) & 0xFF

        payload = encode_telemetry_payload(
            seq=step & 0xFF,
            hump_height_mm=hump_mm,
            obstacle_cm=obstacle_cm,
            car_state=car_state,
            hump_state=hump_state,
            pitch_deg=pitch_deg,
            line_mask=line_mask,
            barcode_id=barcode_id,
            fault_mask=fault_mask,
        )
        return self.ingest_payload(payload, source="demo")

    def _demo_loop(self) -> None:
        while not self._stop_event.is_set():
            self._demo_wakeup.clear()
            with self._lock:
                active = self._state.demo_mode
            if active:
                self.emit_demo_step()
                if self._stop_event.wait(1.0):
                    break
            else:
                self._demo_wakeup.wait(5.0)


class QuietThreadingHTTPServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def handle_error(self, request: Any, client_address: Any) -> None:
        exc = sys.exc_info()[1]
        if isinstance(exc, (ConnectionResetError, ConnectionAbortedError, BrokenPipeError)):
            return
        super().handle_error(request, client_address)


def make_handler(hub: TelemetryHub):
    class DashboardHandler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, format: str, *args: Any) -> None:
            return

        def _send_json(self, code: int, body: dict[str, Any]) -> None:
            raw = json.dumps(body, separators=(",", ":")).encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(raw)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(raw)

        def _read_json_body(self) -> dict[str, Any]:
            length = int(self.headers.get("Content-Length", "0") or "0")
            if length > 16384:
                raise ValueError("Request body too large.")
            raw = self.rfile.read(length) if length > 0 else b"{}"
            parsed = json.loads(raw.decode("utf-8"))
            if not isinstance(parsed, dict):
                raise ValueError("JSON object required.")
            return parsed

        def do_GET(self) -> None:  # noqa: N802
            parsed = urllib.parse.urlparse(self.path)
            path = parsed.path

            if path == "/favicon.ico":
                self.send_response(HTTPStatus.NO_CONTENT)
                self.send_header("Content-Length", "0")
                self.end_headers()
                return

            if path == "/api/state":
                self._send_json(HTTPStatus.OK, hub.snapshot())
                return

            if path == "/events":
                q = hub.subscribe()
                try:
                    self.send_response(HTTPStatus.OK)
                    self.send_header("Content-Type", "text/event-stream; charset=utf-8")
                    self.send_header("Cache-Control", "no-cache")
                    self.send_header("Connection", "keep-alive")
                    self.end_headers()

                    init_frame = f"event: init\ndata: {json.dumps(hub.snapshot(), separators=(',', ':'))}\n\n"
                    self.wfile.write(init_frame.encode("utf-8"))
                    self.wfile.flush()

                    while not hub._stop_event.is_set():
                        try:
                            msg = q.get(timeout=15.0)
                            if msg is None:
                                break
                            self.wfile.write(msg.encode("utf-8"))
                            self.wfile.flush()
                        except queue.Empty:
                            self.wfile.write(b": keepalive\n\n")
                            self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, OSError):
                    pass
                finally:
                    hub.unsubscribe(q)
                return

            rel = "index.html" if path in ("", "/") else path.lstrip("/")
            candidate = (STATIC_DIR / rel).resolve()
            try:
                candidate.relative_to(STATIC_DIR.resolve())
            except ValueError:
                self.send_error(HTTPStatus.FORBIDDEN)
                return

            if not candidate.is_file():
                self.send_error(HTTPStatus.NOT_FOUND)
                return

            mime, _ = mimetypes.guess_type(str(candidate))
            content = candidate.read_bytes()
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", f"{mime or 'application/octet-stream'}; charset=utf-8")
            self.send_header("Content-Length", str(len(content)))
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            self.wfile.write(content)

        def do_POST(self) -> None:  # noqa: N802
            parsed = urllib.parse.urlparse(self.path)
            try:
                body = self._read_json_body()
                if parsed.path == "/api/cmd":
                    res = hub.send_command(str(body.get("command", "")), mode=str(body.get("mode", "ascii")))
                    self._send_json(HTTPStatus.OK, res)
                elif parsed.path == "/api/demo":
                    res = hub.set_demo_mode(bool(body.get("enabled", False)))
                    self._send_json(HTTPStatus.OK, res)
                elif parsed.path == "/api/broker":
                    res = hub.configure_broker(
                        str(body.get("broker_ip", "")),
                        body.get("broker_port"),
                    )
                    self._send_json(HTTPStatus.OK, res)
                elif parsed.path == "/api/reset":
                    self._send_json(HTTPStatus.OK, hub.reset_session())
                else:
                    self._send_json(HTTPStatus.NOT_FOUND, {"ok": False, "error": "Unknown endpoint"})
            except ValueError as exc:
                self._send_json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": str(exc)})
            except RuntimeError as exc:
                self._send_json(HTTPStatus.CONFLICT, {"ok": False, "error": str(exc)})
            except Exception as exc:
                self._send_json(HTTPStatus.INTERNAL_SERVER_ERROR, {"ok": False, "error": str(exc)})

    return DashboardHandler


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="INF2004 Team 15 Ground Station Web Dashboard")
    parser.add_argument("broker_ip", nargs="?", default=os.environ.get("MQTT_BROKER_IP", BROKER_IP_DEFAULT),
                        help=f"Mosquitto broker IP (default: {BROKER_IP_DEFAULT})")
    parser.add_argument("--mqtt-port", type=int, default=BROKER_PORT_DEFAULT, help="Mosquitto TCP port (default: 1883)")
    parser.add_argument("--http-port", type=int, default=8080, help="Local dashboard web port (default: 8080)")
    parser.add_argument("--host", default="127.0.0.1", help="HTTP bind address (default: 127.0.0.1)")
    parser.add_argument("--demo", action="store_true", help="Start immediately with simulated telemetry packets")
    parser.add_argument("--no-browser", action="store_true", help="Do not auto-open browser window")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if not STATIC_DIR.is_dir():
        print(f"[ERROR] Missing dashboard static folder: {STATIC_DIR}", file=sys.stderr)
        return 1

    hub = TelemetryHub(
        broker_ip=args.broker_ip,
        broker_port=args.mqtt_port,
        demo=args.demo,
    )
    hub.start()
    server = QuietThreadingHTTPServer((args.host, args.http_port), make_handler(hub))
    port = server.server_address[1]
    url = f"http://{args.host}:{port}/"

    print("=" * 72)
    print(" INF2004 TEAM 15 GROUND STATION DASHBOARD")
    print(f"   Web UI:      {url}")
    print(f"   MQTT Target: {args.broker_ip}:{args.mqtt_port} ({TOPIC_TELEM_DEFAULT})")
    print(f"   Mode:        {'DEMO SIMULATOR' if args.demo else 'LIVE MQTT (toggle Demo inside UI)'}")
    print("=" * 72)

    if not args.no_browser:
        threading.Timer(0.6, lambda: webbrowser.open(url)).start()

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping Ground Station dashboard...")
    finally:
        server.shutdown()
        server.server_close()
        hub.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
