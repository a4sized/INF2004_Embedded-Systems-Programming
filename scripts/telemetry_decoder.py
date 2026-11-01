#!/usr/bin/env python3
"""
telemetry_decoder.py
Ground Station MQTT Decoder for Robot Car (Team 15)

Receives the compact 8-byte binary telemetry payload defined in subsystem1.h
and decodes all packed bit-fields into human-readable engineering units and
individual fault flags in real-time.
"""

import struct
import sys
import paho.mqtt.client as mqtt

BROKER_IP = "10.33.147.245"
BROKER_PORT = 1883
TOPIC_TELEM = "robot/inf2004team15/telemetry"

# Bit definitions matching systemFaults.h
FAULT_NAMES = {
    (1 << 0): "FAULT_ESTOP",
    (1 << 1): "FAULT_MOTOR",
    (1 << 2): "FAULT_IR_SENS",
    (1 << 3): "FAULT_US_SENS",
    (1 << 4): "FAULT_IMU_SENS",
    (1 << 5): "FAULT_WIFI",
    (1 << 6): "FAULT_CMD_REJECTED",
}

# State machine names matching subsystem2.h
STATE_NAMES = {
    0: "MOTOR_STOPPED",
    1: "MOTOR_RAMPING",
    2: "MOTOR_STEADY_DRIVE",
    3: "MOTOR_REVERSING",
    4: "MOTOR_SAFETY_HALT",
}

# Hump / terrain names
HUMP_NAMES = {
    0: "FLAT",
    1: "RAMP_UP",
    2: "RAMP_DOWN",
    3: "HUMP_PEAK",
}

def decode_faults(mask):
    if mask == 0:
        return "NONE (ALL CLEAR)"
    active = []
    for bit, name in FAULT_NAMES.items():
        if mask & bit:
            active.append(name)
    # Check for unmapped bits
    known_mask = sum(FAULT_NAMES.keys())
    unknown = mask & ~known_mask
    if unknown:
        active.append(f"UNKNOWN(0x{unknown:X})")
    return " | ".join(active)

def on_connect(client, userdata, flags, rc, properties=None):
    if rc == 0:
        print("\n" + "=" * 80)
        print(f" [OK] CONNECTED TO BROKER ({BROKER_IP}:{BROKER_PORT})")
        print(f" [OK] LISTENING FOR 8-BYTE PACKETS ON: {TOPIC_TELEM}")
        print("=" * 80 + "\n")
        client.subscribe(TOPIC_TELEM)
    else:
        print(f"[!] Connect failed with return code {rc}")

def on_message(client, userdata, msg):
    payload = msg.payload
    raw_hex = " ".join(f"{b:02X}" for b in payload)

    if len(payload) != 8:
        print(f"[WARN] Non-8-byte payload received ({len(payload)} bytes): {raw_hex}")
        return

    # Unpack two 32-bit little-endian words (matching ARM Cortex-M0+ packing)
    w0, w1 = struct.unpack("<II", payload)

    # Word 0 Bitfields:
    # Bits  0..7  ( 8 bits): seq counter (0..255)
    # Bits  8..17 (10 bits): speed_mm_s signed (-512..+511)
    # Bits 18..26 ( 9 bits): obstacle_cm (0..511)
    # Bits 27..29 ( 3 bits): car_state (0..7)
    # Bits 30..31 ( 2 bits): hump_state (0..3)
    seq         = w0 & 0xFF
    speed_raw   = (w0 >> 8) & 0x3FF
    speed_mm_s  = speed_raw if speed_raw < 512 else speed_raw - 1024
    obstacle_cm = (w0 >> 18) & 0x1FF
    car_state   = (w0 >> 27) & 0x07
    hump_state  = (w0 >> 30) & 0x03

    # Word 1 Bitfields:
    # Bits  0..7  ( 8 bits): pitch_deg signed (-128..+127)
    # Bits  8..12 ( 5 bits): line_mask (5-bit IR sensor)
    # Bits 13..19 ( 7 bits): barcode_id (0..127)
    # Bits 20..31 (12 bits): fault_mask (0..4095)
    pitch_raw   = w1 & 0xFF
    pitch_deg   = pitch_raw if pitch_raw < 128 else pitch_raw - 256
    line_mask   = (w1 >> 8) & 0x1F
    barcode_id  = (w1 >> 13) & 0x7F
    fault_mask  = (w1 >> 20) & 0xFFF

    state_str = STATE_NAMES.get(car_state, f"STATE_{car_state}")
    hump_str  = HUMP_NAMES.get(hump_state, f"HUMP_{hump_state}")
    fault_str = decode_faults(fault_mask)

    print(f"[SEQ #{seq:03d}] Raw HEX: [{raw_hex}] (8 bytes)")
    print(f"  ├─ State:     {state_str} ({car_state}) | Terrain: {hump_str} ({hump_state})")
    print(f"  ├─ Motion:    Speed = {speed_mm_s:+4d} mm/s | Obstacle = {obstacle_cm:3d} cm | Pitch = {pitch_deg:+3d}°")
    print(f"  ├─ Sensors:   Line IR = [0b{line_mask:05b}] | Barcode = '{chr(barcode_id) if 32 <= barcode_id <= 126 else barcode_id}'")
    print(f"  └─ Faults:    [0x{fault_mask:03X}] -> {fault_str}")
    print("-" * 80)

def main():
    # Support paho-mqtt v2 and v1
    try:
        client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    except AttributeError:
        client = mqtt.Client()

    client.on_connect = on_connect
    client.on_message = on_message

    print(f"Connecting to Mosquitto at {BROKER_IP}:{BROKER_PORT}...")
    try:
        client.connect(BROKER_IP, BROKER_PORT, 60)
        client.loop_forever()
    except KeyboardInterrupt:
        print("\nGround Station monitor stopped.")
    except Exception as e:
        print(f"\n[ERROR] Connection failed: {e}")
        print("Make sure Mosquitto is running on your PC with mosquitto_local.conf!")

if __name__ == "__main__":
    main()
