# Imported by:
#   python3 external_pose_to_cube.py --demo hold --ports /dev/ttyACM0 --origin-lat 12.9715987 --origin-lon 77.5945627 --origin-alt 900
#   python3 realsense_optical_flow_to_cube.py --ports /dev/ttyACM1 /dev/ttyACM0

import os
import subprocess
import time
from dataclasses import dataclass
from typing import Iterable, Optional

from pymavlink import mavutil

try:
    from pymavlink.dialects.v20 import common as mavlink_common_v20
except Exception:
    mavlink_common_v20 = None


DEFAULT_PORTS = ["/dev/ttyACM1", "/dev/ttyACM0"]
LUA_STATUS_PARAM = "SCR_USER1"
LUA_SOURCE_SET_PARAM = "SCR_USER2"


@dataclass
class TelemetryState:
    local_position: Optional[object] = None
    attitude: Optional[object] = None
    ekf_flags: Optional[int] = None
    status_text: str = ""


@dataclass
class GlobalFix:
    lat_deg: float
    lon_deg: float
    alt_m: float
    source: str
    fix_type: Optional[int] = None
    satellites_visible: Optional[int] = None


def connect_to_cube(
    ports: Iterable[str],
    baud: int,
    source_system: int = 255,
    source_component: int = mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER,
):
    last_error = None
    for port in ports:
        if not os.path.exists(port):
            last_error = FileNotFoundError(port)
            print(f"Skipping {port}: device does not exist")
            continue
        if serial_port_in_use(port):
            last_error = RuntimeError(f"{port} is already in use")
            print(f"Skipping {port}: already in use by another process")
            continue

        master = None
        try:
            print(f"Trying MAVLink on {port} @ {baud}...")
            master = mavutil.mavlink_connection(
                port,
                baud=baud,
                source_system=source_system,
                source_component=source_component,
            )
            heartbeat = wait_for_vehicle_heartbeat(master, source_system, timeout_s=6.0)
            print(
                "Connected to Cube:"
                f" sys={master.target_system} comp={master.target_component}"
                f" type={getattr(heartbeat, 'type', 'unknown')}"
                f" autopilot={getattr(heartbeat, 'autopilot', 'unknown')}"
            )
            return master
        except Exception as exc:
            if master is not None:
                try:
                    master.close()
                except Exception:
                    pass
            last_error = exc
            print(f"Failed on {port}: {exc}")

    if last_error is None:
        raise RuntimeError("No MAVLink ports were provided")
    raise RuntimeError(f"Unable to connect to Cube: {last_error}") from last_error


def serial_port_in_use(port: str) -> bool:
    """Best-effort local check so the bridge does not fight QGC for a USB port."""
    try:
        result = subprocess.run(
            ["fuser", port],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )
    except Exception:
        return False
    return result.returncode == 0


def wait_for_vehicle_heartbeat(master, source_system: int, timeout_s: float = 6.0):
    deadline = time.time() + timeout_s
    ignored = []

    while time.time() <= deadline:
        msg = master.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
        if msg is None:
            continue

        sys_id = int(msg.get_srcSystem())
        comp_id = int(msg.get_srcComponent())
        mav_type = int(getattr(msg, "type", -1))
        autopilot = int(getattr(msg, "autopilot", -1))
        ignored.append(f"sys={sys_id} comp={comp_id} type={mav_type} autopilot={autopilot}")

        if sys_id <= 0 or comp_id <= 0 or sys_id == source_system:
            continue
        if mav_type == getattr(mavutil.mavlink, "MAV_TYPE_GCS", 6):
            continue
        if mav_type == getattr(mavutil.mavlink, "MAV_TYPE_ONBOARD_CONTROLLER", 18):
            continue
        if autopilot == getattr(mavutil.mavlink, "MAV_AUTOPILOT_INVALID", 8):
            continue

        master.target_system = sys_id
        master.target_component = comp_id
        return msg

    if ignored:
        recent = "; ".join(ignored[-5:])
        raise RuntimeError(
            "No valid vehicle heartbeat found."
            f" Ignored recent heartbeats: {recent}"
        )
    raise RuntimeError("No vehicle heartbeat received")


def request_message_interval(master, message_id: int, frequency_hz: float):
    if frequency_hz <= 0:
        interval_us = -1
    else:
        interval_us = int(1e6 / max(frequency_hz, 0.1))
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        message_id,
        interval_us,
        0,
        0,
        0,
        0,
        0,
    )


def _normalize_param_id(param_id) -> str:
    if isinstance(param_id, bytes):
        return param_id.decode("ascii", errors="ignore").rstrip("\x00")
    return str(param_id).rstrip("\x00")


def request_parameter(master, name: str, timeout_s: float = 2.0) -> Optional[float]:
    master.mav.param_request_read_send(
        master.target_system,
        master.target_component,
        name.encode("ascii"),
        -1,
    )

    deadline = time.time() + max(timeout_s, 0.0)
    while time.time() <= deadline:
        msg = master.recv_match(blocking=True, timeout=0.5)
        if msg is None or msg.get_type() != "PARAM_VALUE":
            continue
        if _normalize_param_id(getattr(msg, "param_id", "")) != name:
            continue
        return float(getattr(msg, "param_value", 0.0))

    return None


def set_parameter(master, name: str, value: float, timeout_s: float = 3.0) -> Optional[bool]:
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        name.encode("ascii"),
        float(value),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
    )

    deadline = time.time() + max(timeout_s, 0.0)
    while time.time() <= deadline:
        msg = master.recv_match(blocking=True, timeout=0.5)
        if msg is None or msg.get_type() != "PARAM_VALUE":
            continue
        if _normalize_param_id(getattr(msg, "param_id", "")) != name:
            continue
        return abs(float(getattr(msg, "param_value", 0.0)) - float(value)) < 0.01

    return None


def set_ekf_source_set(master, source_set_id: int, timeout_s: float = 3.0) -> Optional[bool]:
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_SET_EKF_SOURCE_SET,
        0,
        source_set_id,
        0,
        0,
        0,
        0,
        0,
        0,
    )

    deadline = time.time() + max(timeout_s, 0.0)
    while time.time() <= deadline:
        msg = master.recv_match(blocking=True, timeout=0.5)
        if msg is None or msg.get_type() != "COMMAND_ACK":
            continue
        if int(getattr(msg, "command", -1)) != mavutil.mavlink.MAV_CMD_SET_EKF_SOURCE_SET:
            continue

        result = int(getattr(msg, "result", -1))
        if result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            return True
        return False

    return None


def publish_lua_status(master, status_code: int, source_set_id: Optional[int] = None) -> bool:
    """Publish a low-rate state code for an FC-side Lua relay script.

    This intentionally uses the reserved SCR_USER parameters and should only be
    called on state transitions, not in a tight loop.
    """
    if source_set_id is not None:
        source_applied = set_parameter(master, LUA_SOURCE_SET_PARAM, float(source_set_id))
        if source_applied is not True:
            print(
                "Unable to publish Lua source-set id"
                f" {source_set_id} to {LUA_SOURCE_SET_PARAM}."
            )
            return False

    applied = set_parameter(master, LUA_STATUS_PARAM, float(status_code))
    if applied is not True:
        print(f"Unable to publish Lua status code {status_code} to {LUA_STATUS_PARAM}.")
        return False

    return True


def wait_for_gps_home(
    master,
    timeout_s: float,
    min_fix_type: int = 3,
    min_sats: int = 6,
    status_rate_hz: float = 1.0,
    gcs_status_interval_s: float = 5.0,
    message_callback=None,
    idle_callback=None,
) -> Optional[GlobalFix]:
    wait_forever = timeout_s <= 0
    deadline = None if wait_forever else time.time() + timeout_s
    status_period = 1.0 / max(status_rate_hz, 0.1)
    next_status = time.time()
    next_gcs_status = time.time()
    last_source = "waiting"
    last_fix_type: Optional[int] = None
    last_sats: Optional[int] = None
    last_lat_deg: Optional[float] = None
    last_lon_deg: Optional[float] = None
    global_fallback: Optional[GlobalFix] = None

    def build_gps_status_text() -> str:
        if last_fix_type is not None or last_sats is not None:
            return f"GPS status fix={last_fix_type or 0} sats={last_sats or 0}"
        if last_source == "GLOBAL_POSITION_INT":
            return "GPS position seen, waiting raw fix"
        return "Trying to get home position from GPS"

    while wait_forever or time.time() <= deadline:
        if idle_callback is not None:
            idle_callback()
        msg = master.recv_match(blocking=True, timeout=0.2)
        now = time.time()
        if idle_callback is not None:
            idle_callback()
        if msg is None:
            if now >= next_status:
                print("Waiting for GPS home: no GPS telemetry yet")
                next_status = now + status_period
            if now >= next_gcs_status:
                send_statustext(
                    master,
                    "GPS waiting for telemetry",
                    mavutil.mavlink.MAV_SEVERITY_NOTICE,
                )
                next_gcs_status = now + max(gcs_status_interval_s, 1.0)
            continue

        if message_callback is not None:
            message_callback(msg)

        msg_type = msg.get_type()
        if msg_type == "HOME_POSITION":
            lat = int(getattr(msg, "latitude", 0) or 0)
            lon = int(getattr(msg, "longitude", 0) or 0)
            alt = int(getattr(msg, "altitude", 0) or 0)
            if lat != 0 and lon != 0:
                last_source = msg_type
                last_lat_deg = lat / 1e7
                last_lon_deg = lon / 1e7
                send_statustext(
                    master,
                    "GPS home position received",
                    mavutil.mavlink.MAV_SEVERITY_NOTICE,
                )
                return GlobalFix(
                    lat_deg=last_lat_deg,
                    lon_deg=last_lon_deg,
                    alt_m=alt / 1000.0,
                    source=msg_type,
                    fix_type=last_fix_type,
                    satellites_visible=last_sats,
                )
        if msg_type == "GPS_RAW_INT":
            last_source = msg_type
            last_fix_type = int(getattr(msg, "fix_type", 0) or 0)
            last_sats = int(getattr(msg, "satellites_visible", 0) or 0)
            lat = int(getattr(msg, "lat", 0) or 0)
            lon = int(getattr(msg, "lon", 0) or 0)
            alt = int(getattr(msg, "alt", 0) or 0)
            if lat != 0 and lon != 0:
                last_lat_deg = lat / 1e7
                last_lon_deg = lon / 1e7
            if (
                lat != 0
                and lon != 0
                and last_fix_type >= min_fix_type
                and last_sats >= min_sats
            ):
                send_statustext(
                    master,
                    f"GPS ready fix={last_fix_type} sats={last_sats}",
                    mavutil.mavlink.MAV_SEVERITY_NOTICE,
                )
                return GlobalFix(
                    lat_deg=lat / 1e7,
                    lon_deg=lon / 1e7,
                    alt_m=alt / 1000.0,
                    source=msg_type,
                    fix_type=last_fix_type,
                    satellites_visible=last_sats,
                )
        elif msg_type == "GLOBAL_POSITION_INT":
            lat = int(getattr(msg, "lat", 0) or 0)
            lon = int(getattr(msg, "lon", 0) or 0)
            alt = int(getattr(msg, "alt", 0) or 0)
            if lat != 0 and lon != 0:
                last_source = msg_type
                last_lat_deg = lat / 1e7
                last_lon_deg = lon / 1e7
                global_fallback = GlobalFix(
                    lat_deg=last_lat_deg,
                    lon_deg=last_lon_deg,
                    alt_m=alt / 1000.0,
                    source=msg_type,
                    fix_type=last_fix_type,
                    satellites_visible=last_sats,
                )

        if now >= next_status:
            summary = [f"source={last_source}"]
            if last_fix_type is not None:
                summary.append(f"fix={last_fix_type}")
            if last_sats is not None:
                summary.append(f"sats={last_sats}")
            if last_lat_deg is not None and last_lon_deg is not None:
                summary.append(f"lat={last_lat_deg:.7f}")
                summary.append(f"lon={last_lon_deg:.7f}")
            print("Waiting for GPS home: " + " ".join(summary))
            next_status = now + status_period

        if now >= next_gcs_status:
            send_statustext(
                master,
                build_gps_status_text(),
                mavutil.mavlink.MAV_SEVERITY_NOTICE,
            )
            next_gcs_status = now + max(gcs_status_interval_s, 1.0)

    if global_fallback is not None:
        send_statustext(
            master,
            "GPS fallback using global position",
            mavutil.mavlink.MAV_SEVERITY_WARNING,
        )
        print(
            "GPS_RAW_INT did not reach the requested fix threshold in time."
            f" Falling back to {global_fallback.source} for origin/home."
        )
        return global_fallback

    if wait_forever:
        return None

    return None


def set_origin_and_home(master, lat_deg: float, lon_deg: float, alt_m: float):
    lat_int = int(lat_deg * 1e7)
    lon_int = int(lon_deg * 1e7)
    alt_mm = int(alt_m * 1000)

    master.mav.set_gps_global_origin_send(
        master.target_system,
        lat_int,
        lon_int,
        alt_mm,
        int(time.time() * 1e6),
    )
    master.mav.command_int_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL,
        mavutil.mavlink.MAV_CMD_DO_SET_HOME,
        0,
        0,
        0,
        0,
        0,
        0,
        lat_int,
        lon_int,
        alt_m,
    )
    print(
        "Sent EKF origin/home:"
        f" lat={lat_deg:.7f}, lon={lon_deg:.7f}, alt={alt_m:.1f} m"
    )


def send_statustext(master, text: str, severity=mavutil.mavlink.MAV_SEVERITY_INFO):
    master.mav.statustext_send(
        severity,
        text[:50].encode("utf-8", errors="ignore"),
    )


def play_tune(master, tune: str) -> bool:
    if mavlink_common_v20 is None or not hasattr(
        mavlink_common_v20, "MAVLink_play_tune_message"
    ):
        print("PLAY_TUNE is not available in this pymavlink build.")
        return False

    try:
        message = mavlink_common_v20.MAVLink_play_tune_message(
            master.target_system,
            master.target_component,
            tune.encode("ascii", errors="ignore"),
            b"",
        )
        master.mav.send(message)
        return True
    except Exception as exc:
        print(f"Unable to send PLAY_TUNE message: {exc}")
        return False


def drain_messages(master, state: TelemetryState):
    while True:
        msg = master.recv_match(blocking=False)
        if msg is None:
            break

        msg_type = msg.get_type()
        if msg_type == "LOCAL_POSITION_NED":
            state.local_position = msg
        elif msg_type == "ATTITUDE":
            state.attitude = msg
        elif msg_type == "EKF_STATUS_REPORT":
            state.ekf_flags = msg.flags
        elif msg_type == "STATUSTEXT":
            text = getattr(msg, "text", "")
            if isinstance(text, bytes):
                text = text.decode("utf-8", errors="ignore")
            state.status_text = str(text).strip()
