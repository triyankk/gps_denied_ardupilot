#!/usr/bin/env python3
# Run:
#   python3 jt16_serial_probe.py --port /dev/ttyUSB0

import argparse
import os
import select
import statistics
import termios
import time
from dataclasses import dataclass


POINT_PACKET_LEN = 80
IMU_PACKET_LEN = 34
FAULT_PACKET_LEN = 41
POINT_HEADER = b"\xee\xff"
FAULT_HEADER = b"\xee\xdd"


@dataclass
class PacketStats:
    point_packets: int = 0
    imu_packets: int = 0
    fault_packets: int = 0
    unknown_headers: int = 0
    last_azimuth_deg: float = 0.0
    last_min_distance_m: float = 0.0
    last_median_distance_m: float = 0.0
    last_max_distance_m: float = 0.0
    last_reflectivity: int = 0


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Read raw Hesai JT16 RS485 packets from a USB-RS485 adapter and print "
            "basic packet/distance diagnostics."
        )
    )
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=3000000)
    parser.add_argument("--duration", type=float, default=0.0, help="Seconds to run; 0 means forever.")
    parser.add_argument("--status-rate", type=float, default=2.0)
    return parser.parse_args()


def baud_constant(baud: int):
    name = f"B{baud}"
    if not hasattr(termios, name):
        raise SystemExit(f"Your system termios does not expose {name}; use a USB-RS485 adapter/driver that supports {baud}.")
    return getattr(termios, name)


def open_raw_serial(path: str, baud: int):
    fd = os.open(path, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
    attrs = termios.tcgetattr(fd)
    speed = baud_constant(baud)

    attrs[0] = 0
    attrs[1] = 0
    attrs[2] = termios.CLOCAL | termios.CREAD | termios.CS8
    attrs[3] = 0
    attrs[4] = speed
    attrs[5] = speed
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIFLUSH)
    return fd


def update_point_stats(packet: bytes, stats: PacketStats):
    body_offset = 16
    azimuth_raw = int.from_bytes(packet[body_offset : body_offset + 2], "little")
    stats.last_azimuth_deg = azimuth_raw * 0.01

    distances_m = []
    reflectivities = []
    channel_offset = body_offset + 2
    for channel in range(16):
        offset = channel_offset + channel * 3
        distance_raw = int.from_bytes(packet[offset : offset + 2], "little")
        reflectivity = packet[offset + 2]
        distance_m = distance_raw * 0.004
        if distance_m > 0.0:
            distances_m.append(distance_m)
        reflectivities.append(reflectivity)

    stats.point_packets += 1
    if distances_m:
        stats.last_min_distance_m = min(distances_m)
        stats.last_median_distance_m = statistics.median(distances_m)
        stats.last_max_distance_m = max(distances_m)
    stats.last_reflectivity = max(reflectivities) if reflectivities else 0


def consume_packets(buffer: bytearray, stats: PacketStats):
    while len(buffer) >= IMU_PACKET_LEN:
        if buffer[:2] == POINT_HEADER:
            if len(buffer) < 6:
                return
            data_type = buffer[5]
            if data_type == 0:
                if len(buffer) < POINT_PACKET_LEN:
                    return
                packet = bytes(buffer[:POINT_PACKET_LEN])
                del buffer[:POINT_PACKET_LEN]
                update_point_stats(packet, stats)
                continue
            if data_type == 1:
                if len(buffer) < IMU_PACKET_LEN:
                    return
                del buffer[:IMU_PACKET_LEN]
                stats.imu_packets += 1
                continue

        if buffer[:2] == FAULT_HEADER:
            if len(buffer) < FAULT_PACKET_LEN:
                return
            del buffer[:FAULT_PACKET_LEN]
            stats.fault_packets += 1
            continue

        next_point = buffer.find(POINT_HEADER, 1)
        next_fault = buffer.find(FAULT_HEADER, 1)
        candidates = [idx for idx in (next_point, next_fault) if idx >= 0]
        if not candidates:
            del buffer[:-1]
            stats.unknown_headers += 1
            return
        del buffer[: min(candidates)]
        stats.unknown_headers += 1


def print_status(stats: PacketStats, start_s: float):
    elapsed_s = max(time.time() - start_s, 1e-6)
    print(
        "JT16:"
        f" point={stats.point_packets} ({stats.point_packets / elapsed_s:.1f}/s)"
        f" imu={stats.imu_packets} ({stats.imu_packets / elapsed_s:.1f}/s)"
        f" fault={stats.fault_packets}"
        f" az={stats.last_azimuth_deg:.2f}deg"
        f" dist_min/med/max={stats.last_min_distance_m:.2f}/"
        f"{stats.last_median_distance_m:.2f}/{stats.last_max_distance_m:.2f}m"
        f" refl_max={stats.last_reflectivity}"
        f" sync_loss={stats.unknown_headers}"
    )


def main():
    args = parse_args()
    if not os.path.exists(args.port):
        raise SystemExit(f"{args.port} does not exist. Check USB-RS485 adapter with: ls -l /dev/ttyUSB*")

    fd = open_raw_serial(args.port, args.baud)
    buffer = bytearray()
    stats = PacketStats()
    start_s = time.time()
    next_status_s = start_s
    deadline_s = None if args.duration <= 0 else start_s + args.duration

    print(f"Reading JT16 on {args.port} at {args.baud} baud. Press Ctrl+C to stop.")
    try:
        while deadline_s is None or time.time() <= deadline_s:
            readable, _, _ = select.select([fd], [], [], 0.2)
            if readable:
                chunk = os.read(fd, 8192)
                if chunk:
                    buffer.extend(chunk)
                    consume_packets(buffer, stats)

            now_s = time.time()
            if now_s >= next_status_s:
                print_status(stats, start_s)
                next_status_s = now_s + 1.0 / max(args.status_rate, 0.1)
    finally:
        os.close(fd)


if __name__ == "__main__":
    main()
