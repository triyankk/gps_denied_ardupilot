# Run:
#   python3 scan_mavlink_ports.py --ports /dev/ttyACM1 /dev/ttyACM0 --bauds 115200 921600 57600

import argparse

from pymavlink import mavutil

from gps_denied.mavlink_helpers import wait_for_vehicle_heartbeat


def parse_args():
    parser = argparse.ArgumentParser(
        description="Scan candidate serial ports/bauds for an ArduPilot vehicle heartbeat."
    )
    parser.add_argument(
        "--ports",
        nargs="+",
        default=["/dev/ttyACM1", "/dev/ttyACM0"],
        help="Candidate ports to test.",
    )
    parser.add_argument(
        "--bauds",
        nargs="+",
        type=int,
        default=[115200, 921600, 57600],
        help="Candidate serial bauds to test.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=4.0,
        help="Seconds to wait for a vehicle heartbeat per port/baud.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    found = []

    for port in args.ports:
        for baud in args.bauds:
            print(f"Testing {port} @ {baud}...")
            master = None
            try:
                master = mavutil.mavlink_connection(
                    port,
                    baud=baud,
                    source_system=255,
                    source_component=mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER,
                )
                heartbeat = wait_for_vehicle_heartbeat(
                    master,
                    source_system=255,
                    timeout_s=args.timeout,
                )
                print(
                    "FOUND:"
                    f" port={port}"
                    f" baud={baud}"
                    f" sys={master.target_system}"
                    f" comp={master.target_component}"
                    f" type={getattr(heartbeat, 'type', 'unknown')}"
                    f" autopilot={getattr(heartbeat, 'autopilot', 'unknown')}"
                )
                found.append((port, baud))
            except Exception as exc:
                print(f"  no vehicle heartbeat: {exc}")
            finally:
                if master is not None:
                    try:
                        master.close()
                    except Exception:
                        pass

    if not found:
        print("No usable vehicle MAVLink serial endpoint found.")
        raise SystemExit(1)

    print("Usable MAVLink endpoints:")
    for port, baud in found:
        print(f"  {port} @ {baud}")


if __name__ == "__main__":
    main()
