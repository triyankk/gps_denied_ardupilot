# Run:
#   python3 external_pose_to_cube.py \
#     --demo hold \
#     --ports /dev/ttyACM0 /dev/ttyACM1 /dev/ttyTHS1 \
#     --origin-lat 12.9715987 \
#     --origin-lon 77.5945627 \
#     --origin-alt 900

import argparse
import json
import math
import socket
import time
from typing import Dict, Optional, Tuple

from pymavlink import mavutil

from gps_denied.mavlink_helpers import (
    DEFAULT_PORTS,
    TelemetryState,
    connect_to_cube,
    drain_messages,
    request_message_interval,
    send_statustext,
    set_origin_and_home,
)


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Send external-vision pose/velocity into ArduPilot for GPS-denied "
            "bench tests or integration with a future ROS/VIO pipeline."
        )
    )
    parser.add_argument(
        "--ports",
        nargs="+",
        default=DEFAULT_PORTS,
        help="Cube serial ports to try in order.",
    )
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument(
        "--vision-rate",
        type=float,
        default=20.0,
        help="Maximum outbound vision message rate in Hz.",
    )
    parser.add_argument(
        "--status-rate",
        type=float,
        default=2.0,
        help="Console status print rate in Hz.",
    )
    parser.add_argument(
        "--udp-host",
        default="0.0.0.0",
        help="Host/IP to bind for incoming JSON pose packets.",
    )
    parser.add_argument(
        "--udp-port",
        type=int,
        default=25100,
        help="UDP port for incoming JSON pose packets.",
    )
    parser.add_argument(
        "--demo",
        choices=["none", "hold", "circle"],
        default="none",
        help="Use a synthetic pose source instead of UDP input.",
    )
    parser.add_argument(
        "--demo-radius",
        type=float,
        default=0.6,
        help="Circle radius in meters for demo mode.",
    )
    parser.add_argument(
        "--demo-speed",
        type=float,
        default=0.25,
        help="Angular speed in rad/s for circle demo mode.",
    )
    parser.add_argument(
        "--demo-z",
        type=float,
        default=0.0,
        help="Synthetic vision Z in meters (NED down positive).",
    )
    parser.add_argument(
        "--demo-yaw-mode",
        choices=["fixed", "velocity"],
        default="fixed",
        help=(
            "Yaw behavior for synthetic demo poses. 'fixed' keeps heading constant; "
            "'velocity' points the nose along the synthetic velocity vector."
        ),
    )
    parser.add_argument(
        "--demo-yaw-deg",
        type=float,
        default=0.0,
        help="Fixed synthetic yaw in degrees when --demo-yaw-mode=fixed.",
    )
    parser.add_argument("--origin-lat", type=float)
    parser.add_argument("--origin-lon", type=float)
    parser.add_argument("--origin-alt", type=float, default=0.0)
    return parser.parse_args()


def bind_udp_socket(host: str, port: int):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((host, port))
    sock.setblocking(False)
    return sock


def parse_vector3(packet: Dict, key: str, fallback=(0.0, 0.0, 0.0)) -> Tuple[float, float, float]:
    value = packet.get(key)
    if value is None:
        return fallback
    if isinstance(value, dict):
        return (
            float(value.get("x", 0.0)),
            float(value.get("y", 0.0)),
            float(value.get("z", 0.0)),
        )
    if isinstance(value, (list, tuple)) and len(value) >= 3:
        return float(value[0]), float(value[1]), float(value[2])
    raise ValueError(f"Invalid {key} payload: {value!r}")


def read_udp_pose(sock) -> Tuple[Optional[Dict], Optional[str]]:
    newest_packet = None
    source = None
    while True:
        try:
            payload, address = sock.recvfrom(65535)
        except BlockingIOError:
            break
        newest_packet = json.loads(payload.decode("utf-8"))
        source = f"{address[0]}:{address[1]}"
    return newest_packet, source


def build_demo_pose(args, started_at: float, now: float) -> Dict:
    fixed_yaw = math.radians(args.demo_yaw_deg)

    if args.demo == "hold":
        return {
            "position": [0.0, 0.0, args.demo_z],
            "velocity": [0.0, 0.0, 0.0],
            "attitude": [0.0, 0.0, fixed_yaw],
        }

    phase = (now - started_at) * args.demo_speed
    x = args.demo_radius * math.cos(phase)
    y = args.demo_radius * math.sin(phase)
    vx = -args.demo_radius * args.demo_speed * math.sin(phase)
    vy = args.demo_radius * args.demo_speed * math.cos(phase)
    if args.demo_yaw_mode == "velocity":
        yaw = math.atan2(vy, vx) if abs(vx) + abs(vy) > 1e-6 else fixed_yaw
    else:
        yaw = fixed_yaw
    return {
        "position": [x, y, args.demo_z],
        "velocity": [vx, vy, 0.0],
        "attitude": [0.0, 0.0, yaw],
    }


def send_vision(master, packet: Dict, now: float):
    position = parse_vector3(packet, "position")
    velocity = parse_vector3(packet, "velocity")
    attitude = parse_vector3(packet, "attitude")
    timestamp_us = int(packet.get("t_usec", now * 1e6))

    master.mav.vision_position_estimate_send(
        timestamp_us,
        position[0],
        position[1],
        position[2],
        attitude[0],
        attitude[1],
        attitude[2],
    )
    master.mav.vision_speed_estimate_send(
        timestamp_us,
        velocity[0],
        velocity[1],
        velocity[2],
    )
    return position, velocity, attitude


def main():
    args = parse_args()

    if (args.origin_lat is None) != (args.origin_lon is None):
        raise SystemExit("Provide both --origin-lat and --origin-lon together")

    master = connect_to_cube(
        args.ports,
        args.baud,
        source_component=mavutil.mavlink.MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY,
    )
    state = TelemetryState()

    request_message_interval(
        master, mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 5
    )
    request_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 5)
    request_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_STATUSTEXT, 2)
    request_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT, 2)

    if args.origin_lat is not None and args.origin_lon is not None:
        set_origin_and_home(master, args.origin_lat, args.origin_lon, args.origin_alt)
        time.sleep(1.0)
    else:
        print("Origin/home not sent. Some position-hold modes may still reject arming.")

    sock = None
    if args.demo == "none":
        sock = bind_udp_socket(args.udp_host, args.udp_port)
        print(f"Listening for JSON pose packets on udp://{args.udp_host}:{args.udp_port}")
    else:
        print(f"Using synthetic pose demo source: {args.demo}")

    send_period = 1.0 / args.vision_rate
    status_period = 1.0 / args.status_rate
    next_send = time.time()
    next_status = time.time()
    started_at = time.time()
    last_packet_at = 0.0
    last_packet_source = "demo"
    stale_warning_sent = False
    last_position = (0.0, 0.0, 0.0)
    last_velocity = (0.0, 0.0, 0.0)

    try:
        while True:
            now = time.time()
            packet = None

            if args.demo == "none":
                packet, source = read_udp_pose(sock)
                if packet is not None:
                    last_packet_at = now
                    last_packet_source = source or "udp"
                    stale_warning_sent = False
            else:
                packet = build_demo_pose(args, started_at, now)
                last_packet_at = now

            if packet is not None and now >= next_send:
                last_position, last_velocity, _ = send_vision(master, packet, now)
                next_send = now + send_period

            if args.demo == "none" and last_packet_at > 0.0:
                age = now - last_packet_at
                if age > 1.0 and not stale_warning_sent:
                    send_statustext(
                        master,
                        f"External vision stale {age:.1f}s",
                        mavutil.mavlink.MAV_SEVERITY_WARNING,
                    )
                    print(f"Pose input is stale: {age:.1f}s since last UDP packet")
                    stale_warning_sent = True

            drain_messages(master, state)

            if now >= next_status:
                summary = [
                    f"source={last_packet_source}",
                    f"vision_xyz=({last_position[0]:+.2f},{last_position[1]:+.2f},{last_position[2]:+.2f}) m",
                    f"vision_v=({last_velocity[0]:+.2f},{last_velocity[1]:+.2f},{last_velocity[2]:+.2f}) m/s",
                ]
                if last_packet_at > 0.0:
                    summary.append(f"input_age={now - last_packet_at:.2f}s")
                if state.local_position is not None:
                    summary.append(
                        f"cube_xy=({state.local_position.x:+.2f},{state.local_position.y:+.2f}) m"
                    )
                if state.ekf_flags is not None:
                    summary.append(f"ekf_flags=0x{state.ekf_flags:x}")
                if state.status_text:
                    summary.append(f"status={state.status_text}")
                print(" | ".join(summary))
                next_status = now + status_period

            time.sleep(0.01)
    finally:
        if sock is not None:
            sock.close()


if __name__ == "__main__":
    main()
