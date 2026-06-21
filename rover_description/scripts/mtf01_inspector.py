#!/usr/bin/env python3
"""MSP packet inspector for MTF-01 optical flow sensor.

Run directly (not as a ROS node):
    python3 mtf01_inspector.py

Controlled test protocol:
    1. Rover static, sensor aimed at floor — baseline values
    2. Move forward 0.5m slowly         — identify flow_x or flow_y change
    3. Move laterally 0.5m              — identify the other flow axis
    4. Lift sensor 10cm                 — identify distance field
    5. Cover lens                       — quality/confidence field drops to 0
    6. Rotate sensor in place           — rotational coupling in flow fields

Output lets you map: distance, flow_x, flow_y, quality, status bytes
"""

import serial
import struct
from datetime import datetime

PORT = "/dev/ttyUSB0"
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=1)

print("\n=== RAW MSP SENSOR INSPECTOR ===\n")

packet_count = 0


def hexdump(data):
    return ' '.join(f'{b:02X}' for b in data)


while True:
    if ser.read(1) != b'$':
        continue
    if ser.read(1) != b'X':
        continue

    direction = ser.read(1)
    if direction not in [b'<', b'>']:
        continue

    try:
        flag        = ser.read(1)[0]
        func_id     = struct.unpack('<H', ser.read(2))[0]
        payload_len = struct.unpack('<H', ser.read(2))[0]
        payload     = ser.read(payload_len)
        crc         = ser.read(1)[0]
    except Exception:
        continue

    packet_count += 1

    print("\n" + "=" * 60)
    print(f"PACKET #{packet_count}")
    print("=" * 60)
    print(f"Timestamp        : {datetime.now()}")
    print(f"Direction        : {direction.decode(errors='ignore')}")
    print(f"Flag             : {flag}")
    print(f"Function ID      : {func_id} (0x{func_id:04X})")
    print(f"Payload Length   : {payload_len}")
    print(f"CRC              : 0x{crc:02X}")

    print("\nRAW PAYLOAD")
    print("-" * 40)
    print(hexdump(payload))

    print("\nBYTE VIEW")
    print("-" * 40)
    for i, b in enumerate(payload):
        print(f"Byte[{i:02}] = {b:4} | 0x{b:02X}")

    print("\nINTERPRETATIONS")
    print("-" * 40)

    if payload_len >= 2:
        vals = struct.unpack('<' + 'h' * (payload_len // 2),
                             payload[:(payload_len // 2) * 2])
        print("\nAs int16:")
        for i, v in enumerate(vals):
            print(f"  int16[{i}] = {v:6d}")

    if payload_len >= 4:
        vals = struct.unpack('<' + 'i' * (payload_len // 4),
                             payload[:(payload_len // 4) * 4])
        print("\nAs int32:")
        for i, v in enumerate(vals):
            print(f"  int32[{i}] = {v:10d}")

    if payload_len >= 4:
        vals = struct.unpack('<' + 'f' * (payload_len // 4),
                             payload[:(payload_len // 4) * 4])
        print("\nAs float32:")
        for i, v in enumerate(vals):
            print(f"  float[{i}] = {v:.4f}")

    # If payload is 20 bytes, try Micolink layout as cross-check
    if payload_len == 20:
        try:
            (sys_t, dist_mm, d_str, d_prec, d_stat, _r1,
             fx, fy, fq, f_stat, _r2) = struct.unpack('<IIBBBBhhBBH', payload)
            print("\nAs Micolink layout (if this is the right protocol):")
            print(f"  sys_time_ms      = {sys_t}")
            print(f"  distance_mm      = {dist_mm}  ({dist_mm/1000:.3f} m)  status={d_stat}")
            print(f"  dist_strength    = {d_str}  precision={d_prec}")
            print(f"  flow_vel_x       = {fx}  cm/s@1m")
            print(f"  flow_vel_y       = {fy}  cm/s@1m")
            print(f"  flow_quality     = {fq}  status={f_stat}")
        except Exception:
            pass
