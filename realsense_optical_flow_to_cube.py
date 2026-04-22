# Run:
#   python3 realsense_optical_flow_to_cube.py \
#     --ports /dev/ttyACM1 /dev/ttyACM0
#   This waits for a real Cube home/origin once, then streams/logs RealSense optical
#   flow. PosHold can use the no-GPS source set when flow/range health is good.

import argparse
import csv
import math
import os
import socket
import time
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np
import pyrealsense2 as rs
from pymavlink import mavutil

from gps_denied.mavlink_helpers import (
    DEFAULT_PORTS,
    connect_to_cube,
    play_tune,
    publish_lua_status,
    request_parameter,
    request_message_interval,
    send_statustext,
    set_parameter,
    set_ekf_source_set,
    set_origin_and_home,
    wait_for_gps_home,
)


GPS_HOME_LOCK_TUNE = "MFT200O4L1CR4C"
SCRIPT_INIT_TUNE = "MFT255O5L16CR16CR16C"
SCRIPT_START_TUNE = "MFT240O5L16CR16C"
GPS_HOME_LOCK_TUNE_GUARD_S = 3.5
JETSON_BOOT_STATUS = "Jetson Nano detected running no GPS script"
GPS_STARTUP_SOURCE_PROFILE = {
    "POSXY": 3,
    "VELXY": 3,
    "POSZ": 1,
    "VELZ": 3,
    "YAW": 1,
}
NO_GPS_OPTICAL_FLOW_VELOCITY_SOURCE_PROFILE = {
    "POSXY": 0,
    "VELXY": 5,
    "POSZ": 1,
    "VELZ": 0,
    "YAW": 1,
}
NO_GPS_FLOW_EXTERNAL_NAV_SOURCE_PROFILE = {
    "POSXY": 6,
    "VELXY": 6,
    "POSZ": 1,
    "VELZ": 0,
    "YAW": 1,
}
NO_GPS_FLOW_GPS_INPUT_SOURCE_PROFILE = {
    "POSXY": 3,
    "VELXY": 3,
    "POSZ": 1,
    "VELZ": 0,
    "YAW": 1,
}
LUA_STATE_JETSON_BOOT = 10
LUA_STATE_GPS_ASSIST_ACTIVE = 20
LUA_STATE_ORIGIN_LOCKED = 30
LUA_STATE_FLOW_STARTED = 40
LUA_STATE_SOURCE_SET_ACTIVE = 50
LUA_STATE_GPS_SOURCE_ACTIVE = 51
LUA_STATE_NO_GPS_SOURCE_ACTIVE = 52
LUA_STATE_GPS_LESS_FLIGHT_ACTIVE = 53
LUA_STATE_MANUAL_ORIGIN = 60
LUA_STATE_CONFIG_FAILED = 80
LUA_STATE_GPS_TIMEOUT = 81
LUA_STATE_SOURCE_SWITCH_FAILED = 82
LUA_STATE_SOURCE_SWITCH_NO_ACK = 83
COPTER_MODE_NAMES = {
    0: "STABILIZE",
    1: "ACRO",
    2: "ALTHOLD",
    3: "AUTO",
    4: "GUIDED",
    5: "LOITER",
    6: "RTL",
    7: "CIRCLE",
    9: "LAND",
    11: "DRIFT",
    13: "SPORT",
    14: "FLIP",
    15: "AUTOTUNE",
    16: "POSHOLD",
    17: "BRAKE",
    18: "THROW",
    19: "AVOID_ADSB",
    20: "GUIDED_NOGPS",
    21: "SMART_RTL",
    22: "FLOWHOLD",
    23: "FOLLOW",
    24: "ZIGZAG",
    25: "SYSTEMID",
    26: "AUTOROTATE",
    27: "AUTO_RTL",
    28: "TURTLE",
}


@dataclass
class MonitorState:
    ekf_flags: Optional[int] = None
    ekf_status: Optional[object] = None
    local_position: Optional[object] = None
    attitude: Optional[object] = None
    rangefinder_distance_m: float = -1.0
    rangefinder_last_seen_s: float = 0.0
    rangefinder_sensor_id: Optional[int] = None
    rangefinder_orientation: Optional[int] = None
    flight_mode: str = "UNKNOWN"
    custom_mode: Optional[int] = None
    armed: bool = False
    status_text: str = ""


@dataclass
class FlowMeasurement:
    dx_px: float
    dy_px: float
    tracks: int
    quality: int


@dataclass
class FlowIntrinsics:
    fx_px: float
    fy_px: float


@dataclass
class SentFlow:
    x: float
    y: float
    label: str
    rate_x_rad_s: Optional[float] = None
    rate_y_rad_s: Optional[float] = None


@dataclass
class InertialFlowCheck:
    ok: bool
    reason: str
    implied_speed_m_s: float = 0.0
    lean_accel_m_s2: float = 0.0
    roll_deg: float = 0.0
    pitch_deg: float = 0.0


@dataclass
class FlowExternalNavState:
    north_m: float = 0.0
    east_m: float = 0.0
    last_timestamp_us: Optional[int] = None
    last_vn_m_s: float = 0.0
    last_ve_m_s: float = 0.0
    last_sent: bool = False
    last_reason: str = "disabled"
    last_lat_deg: Optional[float] = None
    last_lon_deg: Optional[float] = None
    last_alt_m: Optional[float] = None

    def reset(self, timestamp_us: Optional[int] = None):
        self.north_m = 0.0
        self.east_m = 0.0
        self.last_timestamp_us = timestamp_us
        self.last_vn_m_s = 0.0
        self.last_ve_m_s = 0.0
        self.last_sent = False
        self.last_reason = "reset"
        self.last_lat_deg = None
        self.last_lon_deg = None
        self.last_alt_m = None


@dataclass
class MavlinkUdpForwarder:
    static_targets: list
    bind_port: int
    sock: socket.socket
    learned_targets: set
    vehicle_packets: int = 0
    qgc_packets: int = 0
    last_error: str = ""

    @property
    def targets(self):
        return list(dict.fromkeys([*self.static_targets, *self.learned_targets]))

    def forward_vehicle_message(self, msg):
        try:
            payload = msg.get_msgbuf()
        except Exception as exc:
            self.last_error = f"vehicle forward failed: {exc}"
            return
        if not payload:
            return

        sent = False
        for target in self.targets:
            try:
                self.sock.sendto(payload, target)
                sent = True
            except OSError as exc:
                self.last_error = f"UDP send to {target[0]}:{target[1]} failed: {exc}"
        if sent:
            self.vehicle_packets += 1

    def drain_qgc_to_cube(self, master):
        while True:
            try:
                payload, address = self.sock.recvfrom(4096)
            except BlockingIOError:
                break
            except OSError as exc:
                self.last_error = f"UDP receive failed: {exc}"
                break

            if not payload:
                continue
            self.learned_targets.add((address[0], int(address[1])))
            try:
                master.write(payload)
                self.qgc_packets += 1
            except Exception as exc:
                self.last_error = f"Cube serial write from QGC failed: {exc}"
                break


FLOW_CSV_FIELDS = [
    "run_id",
    "unix_s",
    "run_s",
    "timestamp_us",
    "mode",
    "armed",
    "ekf_source",
    "flow_locked",
    "gps_fallback",
    "distance_m",
    "range_source",
    "range_age_s",
    "raw_dx_px",
    "raw_dy_px",
    "sent_label",
    "sent_x",
    "sent_y",
    "rate_x_rad_s",
    "rate_y_rad_s",
    "flow_limited",
    "dt_ms",
    "tracks",
    "quality",
    "flow_health",
    "flow_reason",
    "healthy_sends",
    "bad_flow_s",
    "inertial_ok",
    "inertial_reason",
    "of_speed_m_s",
    "lean_accel_m_s2",
    "roll_deg",
    "pitch_deg",
    "speed_xy_m_s",
    "local_x_m",
    "local_y_m",
    "local_vx_m_s",
    "local_vy_m_s",
    "ekf_flags",
    "ekf_vel_var",
    "ekf_pos_h_var",
    "ekf_pos_v_var",
    "ekf_terrain_var",
    "vision_sent",
    "vision_reason",
    "vision_n_m",
    "vision_e_m",
    "vision_vn_m_s",
    "vision_ve_m_s",
    "gps_input_sent",
    "gps_input_reason",
    "gps_input_lat",
    "gps_input_lon",
    "gps_input_alt_m",
    "gps_primary",
]


class FlowTracker:
    def __init__(
        self,
        max_features: int,
        min_tracks: int,
        lk_window: int,
        lk_levels: int,
        max_fb_error_px: float,
        ransac_reproj_threshold_px: float,
    ):
        self.max_features = max_features
        self.min_tracks = min_tracks
        self.max_fb_error_px = max_fb_error_px
        self.ransac_reproj_threshold_px = ransac_reproj_threshold_px
        self.last_previous_points: Optional[np.ndarray] = None
        self.last_current_points: Optional[np.ndarray] = None
        self.lk_params = dict(
            winSize=(lk_window, lk_window),
            maxLevel=lk_levels,
            criteria=(
                cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT,
                20,
                0.03,
            ),
        )
        self.prev_gray: Optional[np.ndarray] = None
        self.prev_points: Optional[np.ndarray] = None

    def update(self, gray: np.ndarray) -> Optional[FlowMeasurement]:
        next_points = self._detect(gray)

        if self.prev_gray is None or self.prev_points is None or len(self.prev_points) == 0:
            self.prev_gray = gray
            self.prev_points = next_points
            self.last_previous_points = None
            self.last_current_points = None
            return None

        previous_points = self.prev_points
        tracked_points, status, _ = cv2.calcOpticalFlowPyrLK(
            self.prev_gray,
            gray,
            previous_points,
            None,
            **self.lk_params,
        )
        if tracked_points is None or status is None:
            self.prev_gray = gray
            self.prev_points = next_points
            self.last_previous_points = None
            self.last_current_points = None
            return FlowMeasurement(0.0, 0.0, 0, 0)

        good_mask = status.reshape(-1) == 1
        if self.max_fb_error_px > 0.0 and np.any(good_mask):
            back_points, back_status, _ = cv2.calcOpticalFlowPyrLK(
                gray,
                self.prev_gray,
                tracked_points,
                None,
                **self.lk_params,
            )
            if back_points is None or back_status is None:
                good_mask[:] = False
            else:
                previous_flat = previous_points.reshape(-1, 2)
                back_flat = back_points.reshape(-1, 2)
                fb_error_px = np.linalg.norm(back_flat - previous_flat, axis=1)
                good_mask &= back_status.reshape(-1) == 1
                good_mask &= fb_error_px <= self.max_fb_error_px

        self.prev_gray = gray
        self.prev_points = next_points

        if not np.any(good_mask):
            self.last_previous_points = None
            self.last_current_points = None
            return FlowMeasurement(0.0, 0.0, 0, 0)

        previous = self.prev_points_for_measurement(previous_points, good_mask)
        current = self.prev_points_for_measurement(tracked_points, good_mask)
        if previous.size == 0 or current.size == 0:
            self.last_previous_points = None
            self.last_current_points = None
            return FlowMeasurement(0.0, 0.0, 0, 0)

        if self.ransac_reproj_threshold_px > 0.0 and len(previous) >= 3:
            _, inliers = cv2.estimateAffinePartial2D(
                previous,
                current,
                method=cv2.RANSAC,
                ransacReprojThreshold=self.ransac_reproj_threshold_px,
                maxIters=2000,
                confidence=0.99,
            )
            if inliers is not None:
                inlier_mask = inliers.reshape(-1) == 1
                if np.any(inlier_mask):
                    previous = previous[inlier_mask]
                    current = current[inlier_mask]

        self.last_previous_points = previous
        self.last_current_points = current

        delta = current - previous
        dx_px = float(np.median(delta[:, 0]))
        dy_px = float(np.median(delta[:, 1]))
        tracks = int(len(delta))

        if tracks < self.min_tracks:
            return FlowMeasurement(0.0, 0.0, tracks, 0)

        quality_ratio = min(1.0, tracks / max(self.max_features * 0.6, self.min_tracks))
        quality = int(max(0, min(255, round(255.0 * quality_ratio))))
        return FlowMeasurement(dx_px, dy_px, tracks, quality)

    @staticmethod
    def prev_points_for_measurement(points: np.ndarray, mask: np.ndarray) -> np.ndarray:
        return points.reshape(-1, 2)[mask]

    def _detect(self, gray: np.ndarray) -> Optional[np.ndarray]:
        points = cv2.goodFeaturesToTrack(
            gray,
            maxCorners=self.max_features,
            qualityLevel=0.01,
            minDistance=8,
            blockSize=7,
        )
        return points


class FlowSmoother:
    def __init__(
        self,
        alpha: float,
        max_frame_px: float,
        max_step_px: float,
        deadband_px: float,
    ):
        self.alpha = alpha
        self.max_frame_px = max_frame_px
        self.max_step_px = max_step_px
        self.deadband_px = deadband_px
        self.dx_px: Optional[float] = None
        self.dy_px: Optional[float] = None
        self.last_raw_dx_px = 0.0
        self.last_raw_dy_px = 0.0
        self.last_limited = False

    def reset(self):
        self.dx_px = None
        self.dy_px = None

    def update(self, measurement: FlowMeasurement) -> FlowMeasurement:
        self.last_raw_dx_px = measurement.dx_px
        self.last_raw_dy_px = measurement.dy_px
        self.last_limited = False

        if measurement.quality <= 0:
            self.reset()
            return measurement

        dx_px, dy_px = self._limit_vector(
            measurement.dx_px,
            measurement.dy_px,
            self.max_frame_px,
        )

        if self.dx_px is None or self.dy_px is None:
            smoothed_dx = dx_px
            smoothed_dy = dy_px
        else:
            dx_px, dy_px = self._limit_step(dx_px, dy_px)
            smoothed_dx = self.alpha * dx_px + (1.0 - self.alpha) * self.dx_px
            smoothed_dy = self.alpha * dy_px + (1.0 - self.alpha) * self.dy_px

        if abs(smoothed_dx) < self.deadband_px:
            smoothed_dx = 0.0
        if abs(smoothed_dy) < self.deadband_px:
            smoothed_dy = 0.0

        self.dx_px = smoothed_dx
        self.dy_px = smoothed_dy
        return FlowMeasurement(
            smoothed_dx,
            smoothed_dy,
            measurement.tracks,
            measurement.quality,
        )

    def _limit_vector(self, dx_px: float, dy_px: float, limit_px: float):
        if limit_px <= 0.0:
            return dx_px, dy_px
        magnitude = math.hypot(dx_px, dy_px)
        if magnitude <= limit_px:
            return dx_px, dy_px
        scale = limit_px / magnitude
        self.last_limited = True
        return dx_px * scale, dy_px * scale

    def _limit_step(self, dx_px: float, dy_px: float):
        if self.max_step_px <= 0.0 or self.dx_px is None or self.dy_px is None:
            return dx_px, dy_px
        step_dx = dx_px - self.dx_px
        step_dy = dy_px - self.dy_px
        step_magnitude = math.hypot(step_dx, step_dy)
        if step_magnitude <= self.max_step_px:
            return dx_px, dy_px
        scale = self.max_step_px / step_magnitude
        self.last_limited = True
        return self.dx_px + step_dx * scale, self.dy_px + step_dy * scale


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Estimate optical flow from a downward-facing RealSense stream and send "
            "MAVLink OPTICAL_FLOW_RAD to the Cube using external lidar or RealSense "
            "depth for range."
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
        "--flow-source",
        choices=["infra1", "depth"],
        default="infra1",
        help="RealSense stream to use for optical-flow tracking.",
    )
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument(
        "--send-rate",
        type=float,
        default=15.0,
        help="Maximum MAVLink flow send rate in Hz.",
    )
    parser.add_argument(
        "--status-rate",
        type=float,
        default=2.0,
        help="Console status print rate in Hz.",
    )
    parser.add_argument(
        "--flow-csv-log",
        default="realsense_optical_flow_to_cube_of.csv",
        help=(
            "Append companion-side optical-flow diagnostics to this CSV file. "
            "Use this because ArduPilot .bin logs may not include raw OPTICAL_FLOW_RAD."
        ),
    )
    parser.add_argument(
        "--disable-flow-csv-log",
        action="store_true",
        help="Disable companion-side optical-flow CSV diagnostics.",
    )
    parser.add_argument(
        "--qgc-udp-forward",
        action="append",
        default=None,
        metavar="HOST:PORT",
        help=(
            "Forward Cube MAVLink telemetry to this QGC UDP endpoint while the "
            "bridge owns the serial port. Can be repeated. Default is "
            "127.0.0.1:14550 for local QGC."
        ),
    )
    parser.add_argument(
        "--qgc-udp-bind-port",
        type=int,
        default=14555,
        help=(
            "UDP port the bridge listens on for QGC uplink packets. Remote QGC can "
            "connect to the Jetson IP on this port."
        ),
    )
    parser.add_argument(
        "--disable-qgc-udp-forward",
        action="store_true",
        help="Disable the built-in MAVLink UDP pass-through for QGC.",
    )
    parser.add_argument(
        "--crop-fraction",
        type=float,
        default=0.8,
        help="Center crop fraction used before tracking.",
    )
    parser.add_argument(
        "--downscale",
        type=int,
        default=1,
        help="Integer downscale factor applied after the center crop.",
    )
    parser.add_argument(
        "--rotate",
        type=int,
        choices=[0, 90, 180, 270],
        default=0,
        help="Rotate the processed flow image so its top points toward vehicle front.",
    )
    parser.add_argument(
        "--flow-message",
        choices=["rad", "pixel"],
        default="rad",
        help=(
            "MAVLink optical-flow message format. 'rad' sends OPTICAL_FLOW_RAD "
            "using RealSense intrinsics and is the flight-safe default. 'pixel' "
            "keeps the older deprecated OPTICAL_FLOW path for diagnostics only."
        ),
    )
    parser.add_argument(
        "--flow-scale",
        type=float,
        default=1.0,
        help="Multiplier applied after pixel-to-radian conversion for tuning.",
    )
    parser.add_argument(
        "--flow-max-rate-rad-s",
        type=float,
        default=0.8,
        help=(
            "Clamp the sent angular flow-rate magnitude in rad/s. This prevents "
            "single-frame tracker glitches from creating violent EKF corrections. "
            "Use 0 to disable."
        ),
    )
    parser.add_argument(
        "--flow-swap-xy",
        action="store_true",
        help="Swap flow X/Y axes after image rotation, for camera-orientation calibration.",
    )
    parser.add_argument(
        "--flow-invert-x",
        action="store_true",
        help="Invert the sent flow X axis after rotation/swap calibration.",
    )
    parser.add_argument(
        "--flow-invert-y",
        action="store_true",
        help="Invert the sent flow Y axis after rotation/swap calibration.",
    )
    parser.add_argument(
        "--external-nav-from-flow",
        action="store_true",
        default=False,
        help=(
            "Experimental GPS-denied PosHold path: integrate calibrated optical-flow "
            "rates plus range into local X/Y and send MAVLink VISION_POSITION_ESTIMATE "
            "and VISION_SPEED_ESTIMATE. This uses ArduPilot's VisualOdom backend and "
            "is disabled by default because some FCs report VisOdom out-of-memory."
        ),
    )
    parser.add_argument(
        "--disable-external-nav-from-flow",
        action="store_false",
        dest="external_nav_from_flow",
        help=(
            "Disable flow-derived external navigation and use the legacy velocity-only "
            "optical-flow source profile for diagnostics."
        ),
    )
    parser.add_argument(
        "--gps-input-from-flow",
        action="store_true",
        default=True,
        help=(
            "Default GPS-denied PosHold path: integrate calibrated optical-flow rates "
            "plus range and send MAVLink GPS_INPUT as GPS2. This avoids the VisualOdom "
            "backend and its FC memory use."
        ),
    )
    parser.add_argument(
        "--disable-gps-input-from-flow",
        action="store_false",
        dest="gps_input_from_flow",
        help="Disable flow-derived MAVLink GPS_INPUT output.",
    )
    parser.add_argument(
        "--gps-input-id",
        type=int,
        default=1,
        help=(
            "GPS_INPUT gps_id to publish. gps_id=1 maps to GPS2 on ArduPilot and "
            "lets GPS1 remain the real receiver."
        ),
    )
    parser.add_argument(
        "--gps-input-sats",
        type=int,
        default=10,
        help="satellites_visible value advertised in flow-derived GPS_INPUT.",
    )
    parser.add_argument(
        "--gps-input-hacc-m",
        type=float,
        default=0.35,
        help="Horizontal accuracy advertised in flow-derived GPS_INPUT, in meters.",
    )
    parser.add_argument(
        "--gps-input-vacc-m",
        type=float,
        default=1.5,
        help="Vertical accuracy advertised in flow-derived GPS_INPUT, in meters.",
    )
    parser.add_argument(
        "--gps-input-speed-acc-m-s",
        type=float,
        default=0.2,
        help="Speed accuracy advertised in flow-derived GPS_INPUT, in m/s.",
    )
    parser.add_argument(
        "--flow-odom-forward-scale",
        type=float,
        default=1.0,
        help=(
            "Scale/sign mapping from calibrated flow Y rate to vehicle forward velocity "
            "for flow-derived navigation. Use -1 if the forward axis is reversed."
        ),
    )
    parser.add_argument(
        "--flow-odom-right-scale",
        type=float,
        default=-1.0,
        help=(
            "Scale/sign mapping from calibrated flow X rate to vehicle right velocity "
            "for flow-derived navigation. Default -1 matches a downward camera where "
            "rightward vehicle motion moves ground features left in the image."
        ),
    )
    parser.add_argument(
        "--disable-inertial-flow-gate",
        action="store_true",
        help=(
            "Disable the IMU/attitude plausibility gate. By default, large optical-flow "
            "velocities are suppressed if the vehicle attitude does not make that "
            "motion physically plausible."
        ),
    )
    parser.add_argument(
        "--inertial-flow-gate-speed-m-s",
        type=float,
        default=1.2,
        help=(
            "Only run the inertial plausibility gate when optical flow implies at "
            "least this much horizontal speed."
        ),
    )
    parser.add_argument(
        "--inertial-flow-gate-lean-deg",
        type=float,
        default=4.0,
        help=(
            "Minimum roll or pitch angle that makes a large optical-flow velocity "
            "plausible without suppressing it."
        ),
    )
    parser.add_argument(
        "--inertial-flow-gate-accel-m-s2",
        type=float,
        default=0.7,
        help=(
            "Minimum lateral acceleration implied by roll/pitch that makes a large "
            "optical-flow velocity plausible."
        ),
    )
    parser.add_argument(
        "--max-depth-m",
        type=float,
        default=4.0,
        help="Clip depth frames to this range before normalizing for tracking.",
    )
    parser.add_argument("--max-features", type=int, default=300)
    parser.add_argument("--min-tracks", type=int, default=50)
    parser.add_argument("--lk-window", type=int, default=21)
    parser.add_argument("--lk-levels", type=int, default=3)
    parser.add_argument(
        "--lk-fb-max-error-px",
        type=float,
        default=1.5,
        help=(
            "Reject Lucas-Kanade tracks whose forward/backward reprojection error "
            "exceeds this many processed pixels. Use 0 to disable."
        ),
    )
    parser.add_argument(
        "--ransac-reproj-threshold-px",
        type=float,
        default=3.0,
        help=(
            "Reject optical-flow outliers using a RANSAC affine fit with this "
            "processed-pixel reprojection threshold. Use 0 to disable."
        ),
    )
    parser.add_argument(
        "--range-source",
        choices=["external", "realsense", "none"],
        default="external",
        help=(
            "Range scale for optical flow. Use 'external' for a Cube-connected "
            "down-facing lidar, 'realsense' to send RealSense depth as "
            "DISTANCE_SENSOR, or 'none' for diagnostics only."
        ),
    )
    parser.add_argument(
        "--external-range-min-m",
        type=float,
        default=0.2,
        help="Minimum valid external lidar distance in meters.",
    )
    parser.add_argument(
        "--external-range-max-m",
        type=float,
        default=300.0,
        help="Maximum valid external lidar distance in meters.",
    )
    parser.add_argument(
        "--external-range-timeout",
        type=float,
        default=1.0,
        help="Seconds before external lidar data is considered stale.",
    )
    parser.add_argument(
        "--external-range-sensor-id",
        type=int,
        default=-1,
        help="Only accept this DISTANCE_SENSOR id for external lidar. Use -1 for any id.",
    )
    parser.add_argument(
        "--external-range-any-orientation",
        action="store_true",
        help=(
            "Accept external DISTANCE_SENSOR messages with any orientation. "
            "Leave off for flight so only the configured down-facing orientation is used."
        ),
    )
    parser.add_argument(
        "--range-alt-consistency-max-m",
        type=float,
        default=2.0,
        help=(
            "Reject optical-flow navigation when the selected range differs from "
            "EKF local altitude by more than this many meters. Use 0 to disable."
        ),
    )
    parser.add_argument(
        "--range-alt-consistency-min-alt-m",
        type=float,
        default=1.5,
        help=(
            "Only run the range/EKF-altitude consistency check above this local "
            "altitude, in meters. Use 0 to check whenever local altitude is available."
        ),
    )
    parser.add_argument(
        "--flow-smoothing-alpha",
        type=float,
        default=0.45,
        help=(
            "EMA alpha for optical-flow pixel deltas. Use 1.0 for no smoothing; "
            "lower values are smoother but add lag."
        ),
    )
    parser.add_argument(
        "--flow-max-frame-px",
        type=float,
        default=16.0,
        help=(
            "Clamp one-frame optical-flow vector magnitude in processed pixels. "
            "Use 0 to disable this clamp."
        ),
    )
    parser.add_argument(
        "--flow-max-step-px",
        type=float,
        default=12.0,
        help=(
            "Clamp how fast the smoothed flow vector may change between sends. "
            "Use 0 to disable this clamp."
        ),
    )
    parser.add_argument(
        "--flow-deadband-px",
        type=float,
        default=0.03,
        help="Zero very small smoothed optical-flow values in processed pixels.",
    )
    parser.add_argument(
        "--lean-threshold-deg",
        type=float,
        default=3.0,
        help="Minimum roll/pitch angle to treat as active correction.",
    )
    parser.add_argument(
        "--speed-threshold",
        type=float,
        default=0.15,
        help="Minimum horizontal speed in m/s to treat as active correction.",
    )
    parser.add_argument(
        "--correction-quality-min",
        type=int,
        default=80,
        help="Minimum optical-flow quality before correction messages are trusted.",
    )
    parser.add_argument(
        "--range-window",
        type=int,
        default=5,
        help="Center window size in pixels for median depth sampling.",
    )
    parser.add_argument("--min-cm", type=int, default=20)
    parser.add_argument("--max-cm", type=int, default=1200)
    parser.add_argument("--sensor-id", type=int, default=1)
    parser.add_argument(
        "--orientation",
        type=int,
        default=mavutil.mavlink.MAV_SENSOR_ROTATION_PITCH_270,
        help="MAVLink distance sensor orientation enum.",
    )
    parser.add_argument("--origin-lat", type=float)
    parser.add_argument("--origin-lon", type=float)
    parser.add_argument("--origin-alt", type=float, default=0.0)
    parser.add_argument(
        "--flow-health-test",
        action="store_true",
        help=(
            "Prop-less/bench mode: stream optical flow/range data without waiting "
            "for GPS home/origin and without switching EKF sources."
        ),
    )
    parser.add_argument(
        "--gps-home-timeout",
        type=float,
        default=0.0,
        help=(
            "When --origin-* is omitted, wait this many seconds for a one-time GPS "
            "home/origin fix from the Cube. Use 0 to wait indefinitely."
        ),
    )
    parser.add_argument(
        "--gps-min-fix",
        type=int,
        default=3,
        help="Minimum GPS fix type required before using GPS for home/origin.",
    )
    parser.add_argument(
        "--gps-min-sats",
        type=int,
        default=6,
        help="Minimum visible satellites required before using GPS for home/origin.",
    )
    parser.add_argument(
        "--gps-startup-ekf-source-set",
        type=int,
        choices=[1, 2, 3],
        default=1,
        help="EKF source set used for GPS startup and home acquisition.",
    )
    parser.add_argument(
        "--post-home-ekf-source-set",
        type=int,
        choices=[0, 1, 2, 3],
        default=2,
        help=(
            "No-GPS EKF source set selected after home/origin is set and RealSense "
            "flow is healthy. Default 2 lets PosHold use the no-GPS lane while "
            "non-PosHold modes stay on the startup/GPS lane. Use 0 for logging only."
        ),
    )
    parser.add_argument(
        "--ekf-source-switch-after-sends",
        type=int,
        default=30,
        help=(
            "Number of consecutive healthy optical-flow sends required before "
            "selecting the no-GPS EKF source."
        ),
    )
    parser.add_argument(
        "--disable-flow-failsafe",
        action="store_true",
        help=(
            "Disable automatic GPS fallback when optical flow/range becomes unhealthy. "
            "Not recommended for outdoor flight."
        ),
    )
    parser.add_argument(
        "--flow-failsafe-quality-min",
        type=int,
        default=80,
        help="Minimum flow quality before the optical-flow source is considered healthy.",
    )
    parser.add_argument(
        "--flow-failsafe-min-tracks",
        type=int,
        default=50,
        help="Minimum tracked features before the optical-flow source is considered healthy.",
    )
    parser.add_argument(
        "--flow-failsafe-min-height-m",
        type=float,
        default=0.6,
        help=(
            "Minimum height for no-GPS optical-flow source use. Below this, the "
            "script stays on GPS because image motion is too large/noisy near the floor."
        ),
    )
    parser.add_argument(
        "--flow-failsafe-max-height-m",
        type=float,
        default=6.5,
        help=(
            "Maximum height for no-GPS optical-flow source use. Above this, the script "
            "falls back to GPS. Use 0 to disable the height ceiling."
        ),
    )
    parser.add_argument(
        "--flow-failsafe-bad-seconds",
        type=float,
        default=0.5,
        help="Seconds of bad optical-flow/range health before switching back to GPS.",
    )
    parser.add_argument(
        "--flow-failsafe-land-seconds",
        type=float,
        default=0.0,
        help=(
            "Seconds of persistent bad optical-flow/range health while armed before "
            "commanding LAND. Default 0 disables automatic LAND."
        ),
    )
    parser.add_argument(
        "--disable-flow-failsafe-land",
        action="store_true",
        help="Disable automatic LAND after persistent bad optical-flow/range health.",
    )
    parser.add_argument(
        "--poshold-failsafe-mode",
        choices=["LOITER", "GPS_ONLY"],
        default="LOITER",
        help=(
            "Action when PosHold optical-flow/range health is bad. LOITER switches "
            "back to GPS and commands Loiter; GPS_ONLY only switches the EKF source."
        ),
    )
    parser.add_argument(
        "--poshold-failsafe-bad-seconds",
        type=float,
        default=0.5,
        help=(
            "Seconds of bad PosHold optical-flow/range health before the companion "
            "falls back to GPS/Loiter."
        ),
    )
    parser.add_argument(
        "--poshold-ekf-variance-max",
        type=float,
        default=0.6,
        help=(
            "Treat PosHold as unhealthy when live EKF variance telemetry reaches "
            "this value. Use 0 to disable this pre-failsafe guard."
        ),
    )
    parser.add_argument(
        "--mode-source-switch",
        action="store_true",
        help=(
            "Compatibility flag. Flight-mode source switching is now enabled by default: "
            "PosHold uses optical flow, other modes use GPS."
        ),
    )
    parser.add_argument(
        "--disable-mode-source-switch",
        action="store_true",
        help=(
            "Disable mode-based EKF source switching and use the older one-time "
            "optical-flow lock behavior."
        ),
    )
    return parser.parse_args()


def parse_udp_target(value: str):
    if ":" not in value:
        raise ValueError("expected HOST:PORT")
    host, port_text = value.rsplit(":", 1)
    host = host.strip()
    port = int(port_text)
    if not host or port <= 0 or port > 65535:
        raise ValueError("expected HOST:PORT with a valid UDP port")
    return host, port


def create_qgc_udp_forwarder(args) -> Optional[MavlinkUdpForwarder]:
    if args.disable_qgc_udp_forward:
        return None
    if args.qgc_udp_bind_port <= 0 or args.qgc_udp_bind_port > 65535:
        raise SystemExit("--qgc-udp-bind-port must be between 1 and 65535")

    target_values = args.qgc_udp_forward or ["127.0.0.1:14550"]
    targets = []
    for value in target_values:
        try:
            targets.append(parse_udp_target(value))
        except ValueError as exc:
            raise SystemExit(f"Invalid --qgc-udp-forward value '{value}': {exc}") from exc

    try:
        udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        udp_sock.bind(("0.0.0.0", args.qgc_udp_bind_port))
        udp_sock.setblocking(False)
    except OSError as exc:
        raise SystemExit(
            f"Unable to open QGC UDP forwarder on port {args.qgc_udp_bind_port}: {exc}"
        ) from exc

    return MavlinkUdpForwarder(
        static_targets=targets,
        bind_port=args.qgc_udp_bind_port,
        sock=udp_sock,
        learned_targets=set(),
    )


def enable_streams(config, args):
    config.enable_stream(
        rs.stream.depth,
        args.width,
        args.height,
        rs.format.z16,
        args.fps,
    )
    if args.flow_source == "infra1":
        config.enable_stream(
            rs.stream.infrared,
            1,
            args.width,
            args.height,
            rs.format.y8,
            args.fps,
        )


def read_flow_frame(frames, flow_source: str):
    if flow_source == "infra1":
        return frames.get_infrared_frame(1)
    return frames.get_depth_frame()


def median_distance_m(depth_frame, window: int) -> float:
    width = depth_frame.get_width()
    height = depth_frame.get_height()
    center_x = width // 2
    center_y = height // 2
    half = max(window // 2, 0)

    samples = []
    for y in range(center_y - half, center_y + half + 1):
        for x in range(center_x - half, center_x + half + 1):
            distance_m = depth_frame.get_distance(x, y)
            if distance_m > 0.0:
                samples.append(distance_m)

    if not samples:
        return -1.0
    return float(np.median(np.asarray(samples, dtype=np.float32)))


def prepare_flow_image(frame, args) -> np.ndarray:
    image = np.asanyarray(frame.get_data())

    if args.flow_source == "depth":
        max_depth_mm = max(1, int(args.max_depth_m * 1000.0))
        image = np.clip(image, 0, max_depth_mm)
        image = cv2.convertScaleAbs(image, alpha=255.0 / max_depth_mm)
    else:
        if image.ndim == 3:
            image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    image = crop_center(image, args.crop_fraction)
    if args.downscale > 1:
        resized_width = max(32, image.shape[1] // args.downscale)
        resized_height = max(24, image.shape[0] // args.downscale)
        image = cv2.resize(
            image,
            (resized_width, resized_height),
            interpolation=cv2.INTER_AREA,
        )

    image = rotate_image(image, args.rotate)
    image = cv2.equalizeHist(image)
    image = cv2.GaussianBlur(image, (5, 5), 0)
    return image


def crop_center(image: np.ndarray, crop_fraction: float) -> np.ndarray:
    crop_fraction = min(max(crop_fraction, 0.1), 1.0)
    height, width = image.shape[:2]
    crop_width = max(32, int(width * crop_fraction))
    crop_height = max(24, int(height * crop_fraction))
    start_x = max(0, (width - crop_width) // 2)
    start_y = max(0, (height - crop_height) // 2)
    return image[start_y : start_y + crop_height, start_x : start_x + crop_width]


def rotate_image(image: np.ndarray, rotate: int) -> np.ndarray:
    if rotate == 90:
        return cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)
    if rotate == 180:
        return cv2.rotate(image, cv2.ROTATE_180)
    if rotate == 270:
        return cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)
    return image


def send_distance_sensor(master, args, distance_m: float, boot_ms: int):
    if distance_m < 0.0:
        return
    distance_cm = int(round(distance_m * 100.0))
    master.mav.distance_sensor_send(
        boot_ms,
        args.min_cm,
        args.max_cm,
        distance_cm,
        mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER,
        args.sensor_id,
        args.orientation,
        0,
    )


def update_external_range_from_distance_sensor(
    monitor: MonitorState,
    msg,
    args,
    now_s: float,
):
    sensor_id = int(getattr(msg, "id", -1) or -1)
    if args.external_range_sensor_id >= 0 and sensor_id != args.external_range_sensor_id:
        return

    orientation = int(getattr(msg, "orientation", -1) or -1)
    if (
        not args.external_range_any_orientation
        and orientation >= 0
        and orientation != int(args.orientation)
    ):
        return

    distance_cm = int(getattr(msg, "current_distance", 0) or 0)
    if distance_cm <= 0:
        return

    distance_m = distance_cm / 100.0
    if distance_m < args.external_range_min_m or distance_m > args.external_range_max_m:
        return

    monitor.rangefinder_distance_m = distance_m
    monitor.rangefinder_last_seen_s = now_s
    monitor.rangefinder_sensor_id = sensor_id
    monitor.rangefinder_orientation = orientation


def external_range_is_fresh(monitor: MonitorState, args, now_s: float) -> bool:
    if monitor.rangefinder_distance_m <= 0.0:
        return False
    return (now_s - monitor.rangefinder_last_seen_s) <= args.external_range_timeout


def selected_range_m(monitor: MonitorState, depth_frame, args, now_s: float) -> float:
    if args.range_source == "external":
        if external_range_is_fresh(monitor, args, now_s):
            return monitor.rangefinder_distance_m
        return -1.0
    if args.range_source == "realsense":
        return median_distance_m(depth_frame, args.range_window)
    return -1.0


def local_altitude_m(monitor: MonitorState) -> Optional[float]:
    if monitor.local_position is None:
        return None

    local_down_m = getattr(monitor.local_position, "z", None)
    if local_down_m is None:
        return None
    try:
        altitude_m = -float(local_down_m)
    except (TypeError, ValueError):
        return None

    if not math.isfinite(altitude_m) or altitude_m < 0.0:
        return None
    return altitude_m


def range_altitude_mismatch_reason(
    monitor: MonitorState,
    distance_m: float,
    args,
) -> Optional[str]:
    if args.range_alt_consistency_max_m <= 0.0 or distance_m <= 0.0:
        return None

    altitude_m = local_altitude_m(monitor)
    if altitude_m is None or altitude_m < args.range_alt_consistency_min_alt_m:
        return None

    difference_m = abs(altitude_m - distance_m)
    if difference_m <= args.range_alt_consistency_max_m:
        return None

    return (
        "range/alt mismatch"
        f" range={distance_m:.1f}m ekf_alt={altitude_m:.1f}m"
    )


def flow_health_ok(flow: FlowMeasurement, distance_m: float, monitor: MonitorState, args):
    reasons = []
    critical = False
    if flow.quality < args.flow_failsafe_quality_min:
        reasons.append(f"quality {flow.quality}<{args.flow_failsafe_quality_min}")
        critical = True
    if flow.tracks < args.flow_failsafe_min_tracks:
        reasons.append(f"tracks {flow.tracks}<{args.flow_failsafe_min_tracks}")
        critical = True
    if distance_m <= 0.0:
        reasons.append("range unavailable")
        critical = True
    elif (
        args.flow_failsafe_min_height_m > 0.0
        and distance_m < args.flow_failsafe_min_height_m
    ):
        reasons.append(
            f"height {distance_m:.1f}m<{args.flow_failsafe_min_height_m:.1f}m"
        )
    elif (
        args.flow_failsafe_max_height_m > 0.0
        and distance_m > args.flow_failsafe_max_height_m
    ):
        reasons.append(
            f"height {distance_m:.1f}m>{args.flow_failsafe_max_height_m:.1f}m"
        )
    mismatch_reason = range_altitude_mismatch_reason(monitor, distance_m, args)
    if mismatch_reason is not None:
        reasons.append(mismatch_reason)
        critical = True
    return len(reasons) == 0, ", ".join(reasons) if reasons else "healthy", critical


def ekf_variance_failure_reason(monitor: MonitorState, args) -> Optional[str]:
    if args.poshold_ekf_variance_max <= 0.0 or monitor.ekf_status is None:
        return None

    checks = [
        ("vel", "velocity_variance"),
        ("pos_h", "pos_horiz_variance"),
        ("pos_v", "pos_vert_variance"),
        ("terrain", "terrain_alt_variance"),
    ]
    failed = []
    for label, field_name in checks:
        raw_value = getattr(monitor.ekf_status, field_name, None)
        if raw_value is None:
            continue
        try:
            value = float(raw_value)
        except (TypeError, ValueError):
            continue
        if math.isfinite(value) and value >= args.poshold_ekf_variance_max:
            failed.append(f"{label}={value:.2f}")

    if not failed:
        return None
    return (
        "ekf variance high:"
        f" {' '.join(failed)}"
        f">={args.poshold_ekf_variance_max:.2f}"
    )


def csv_value(value):
    if value is None:
        return ""
    if isinstance(value, float):
        if not math.isfinite(value):
            return ""
        return f"{value:.6f}"
    return value


def open_flow_csv_log(path: str):
    write_header = not os.path.exists(path) or os.path.getsize(path) == 0
    if not write_header:
        try:
            with open(path, newline="") as existing_file:
                existing_header = next(csv.reader(existing_file), [])
            write_header = existing_header != FLOW_CSV_FIELDS
        except OSError:
            write_header = True
    log_file = open(path, "a", newline="", buffering=1)
    writer = csv.DictWriter(log_file, fieldnames=FLOW_CSV_FIELDS)
    if write_header:
        writer.writeheader()
    return log_file, writer


def command_flight_mode(master, mode_name: str) -> bool:
    normalized_mode = mode_name.upper()
    mode_id = None
    mode_mapping = master.mode_mapping()
    if mode_mapping:
        mode_id = mode_mapping.get(normalized_mode)
    if mode_id is None:
        for candidate_id, candidate_name in COPTER_MODE_NAMES.items():
            if candidate_name == normalized_mode:
                mode_id = candidate_id
                break
    if mode_id is None:
        print(f"Unable to command {normalized_mode} mode: unknown mode id")
        return False

    try:
        master.mav.set_mode_send(
            master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            int(mode_id),
        )
        return True
    except Exception as exc:
        print(f"Unable to command {normalized_mode} mode: {exc}")
        return False


def command_land(master) -> bool:
    return command_flight_mode(master, "LAND")


def clamp_int16(value: int) -> int:
    return max(-32768, min(32767, value))


def flow_intrinsics_from_frame(frame, args) -> FlowIntrinsics:
    video_profile = frame.profile.as_video_stream_profile()
    intrinsics = video_profile.get_intrinsics()
    downscale = max(args.downscale, 1)
    fx_px = float(intrinsics.fx) / downscale
    fy_px = float(intrinsics.fy) / downscale
    if args.rotate in (90, 270):
        fx_px, fy_px = fy_px, fx_px
    if fx_px <= 0.0 or fy_px <= 0.0:
        raise RuntimeError("RealSense returned invalid optical-flow intrinsics")
    return FlowIntrinsics(fx_px=fx_px, fy_px=fy_px)


def transform_flow_pixels(flow: FlowMeasurement, args):
    dx_px = flow.dx_px
    dy_px = flow.dy_px
    if args.flow_swap_xy:
        dx_px, dy_px = dy_px, dx_px
    if args.flow_invert_x:
        dx_px = -dx_px
    if args.flow_invert_y:
        dy_px = -dy_px
    return dx_px, dy_px


def clamp_flow_rate(integrated_x: float, integrated_y: float, dt_us: int, args):
    if args.flow_max_rate_rad_s <= 0.0 or dt_us <= 0:
        return integrated_x, integrated_y
    dt_s = dt_us / 1_000_000.0
    max_integrated = args.flow_max_rate_rad_s * dt_s
    magnitude = math.hypot(integrated_x, integrated_y)
    if magnitude <= max_integrated or magnitude <= 0.0:
        return integrated_x, integrated_y
    scale = max_integrated / magnitude
    return integrated_x * scale, integrated_y * scale


def implied_flow_speed_m_s(
    flow: FlowMeasurement,
    args,
    dt_us: int,
    distance_m: float,
    intrinsics: Optional[FlowIntrinsics],
) -> float:
    if (
        args.flow_message != "rad"
        or intrinsics is None
        or dt_us <= 0
        or distance_m <= 0.0
    ):
        return 0.0
    dx_px, dy_px = transform_flow_pixels(flow, args)
    dt_s = dt_us / 1_000_000.0
    rate_x_rad_s = args.flow_scale * dx_px / intrinsics.fx_px / dt_s
    rate_y_rad_s = args.flow_scale * dy_px / intrinsics.fy_px / dt_s
    return distance_m * math.hypot(rate_x_rad_s, rate_y_rad_s)


def inertial_flow_check(
    flow: FlowMeasurement,
    args,
    monitor: MonitorState,
    dt_us: int,
    distance_m: float,
    intrinsics: Optional[FlowIntrinsics],
) -> InertialFlowCheck:
    if args.disable_inertial_flow_gate:
        return InertialFlowCheck(True, "disabled")
    if monitor.attitude is None:
        return InertialFlowCheck(True, "waiting_attitude")

    implied_speed = implied_flow_speed_m_s(flow, args, dt_us, distance_m, intrinsics)
    if implied_speed < args.inertial_flow_gate_speed_m_s:
        return InertialFlowCheck(True, "small_flow", implied_speed_m_s=implied_speed)

    roll_rad = float(monitor.attitude.roll)
    pitch_rad = float(monitor.attitude.pitch)
    roll_deg = math.degrees(roll_rad)
    pitch_deg = math.degrees(pitch_rad)
    lean_accel = 9.80665 * math.hypot(math.sin(roll_rad), math.sin(pitch_rad))

    if (
        max(abs(roll_deg), abs(pitch_deg)) >= args.inertial_flow_gate_lean_deg
        or lean_accel >= args.inertial_flow_gate_accel_m_s2
    ):
        return InertialFlowCheck(
            True,
            "inertial_plausible",
            implied_speed_m_s=implied_speed,
            lean_accel_m_s2=lean_accel,
            roll_deg=roll_deg,
            pitch_deg=pitch_deg,
        )

    return InertialFlowCheck(
        False,
        "large_flow_without_inertial_motion",
        implied_speed_m_s=implied_speed,
        lean_accel_m_s2=lean_accel,
        roll_deg=roll_deg,
        pitch_deg=pitch_deg,
    )


def apply_inertial_flow_gate(
    flow: FlowMeasurement,
    args,
    monitor: MonitorState,
    dt_us: int,
    distance_m: float,
    intrinsics: Optional[FlowIntrinsics],
):
    check = inertial_flow_check(flow, args, monitor, dt_us, distance_m, intrinsics)
    if check.ok:
        return flow, check
    return FlowMeasurement(0.0, 0.0, flow.tracks, 0), check


def send_optical_flow(
    master,
    args,
    timestamp_us: int,
    dt_us: int,
    flow: FlowMeasurement,
    distance_m: float,
    intrinsics: Optional[FlowIntrinsics],
) -> SentFlow:
    dx_px, dy_px = transform_flow_pixels(flow, args)
    ground_distance = distance_m if distance_m >= 0.0 else -1.0

    if args.flow_message == "pixel":
        flow_x = clamp_int16(int(round(dx_px * 10.0)))
        flow_y = clamp_int16(int(round(dy_px * 10.0)))
        master.mav.optical_flow_send(
            timestamp_us,
            args.sensor_id,
            flow_x,
            flow_y,
            0.0,
            0.0,
            flow.quality,
            ground_distance,
        )
        return SentFlow(float(flow_x), float(flow_y), "dpix")

    if intrinsics is None:
        raise RuntimeError("OPTICAL_FLOW_RAD requires RealSense intrinsics")

    integrated_x = args.flow_scale * dx_px / intrinsics.fx_px
    integrated_y = args.flow_scale * dy_px / intrinsics.fy_px
    integrated_x, integrated_y = clamp_flow_rate(integrated_x, integrated_y, dt_us, args)
    dt_s = max(dt_us / 1_000_000.0, 1e-6)

    master.mav.optical_flow_rad_send(
        timestamp_us,
        args.sensor_id,
        max(dt_us, 1),
        integrated_x,
        integrated_y,
        0.0,
        0.0,
        0.0,
        0,
        flow.quality,
        max(dt_us, 0),
        ground_distance,
    )
    return SentFlow(
        integrated_x,
        integrated_y,
        "rad",
        rate_x_rad_s=integrated_x / dt_s,
        rate_y_rad_s=integrated_y / dt_s,
    )


def flow_external_nav_velocity_ned(
    sent_flow: SentFlow,
    distance_m: float,
    monitor: MonitorState,
    args,
):
    if (
        sent_flow.rate_x_rad_s is None
        or sent_flow.rate_y_rad_s is None
        or distance_m <= 0.0
        or monitor.attitude is None
    ):
        return None

    forward_m_s = args.flow_odom_forward_scale * distance_m * sent_flow.rate_y_rad_s
    right_m_s = args.flow_odom_right_scale * distance_m * sent_flow.rate_x_rad_s
    yaw_rad = float(monitor.attitude.yaw)
    cos_yaw = math.cos(yaw_rad)
    sin_yaw = math.sin(yaw_rad)
    north_m_s = cos_yaw * forward_m_s - sin_yaw * right_m_s
    east_m_s = sin_yaw * forward_m_s + cos_yaw * right_m_s
    return north_m_s, east_m_s


def offset_m_to_lat_lon(lat0_deg: float, lon0_deg: float, north_m: float, east_m: float):
    earth_radius_m = 6378137.0
    lat0_rad = math.radians(lat0_deg)
    lat_deg = lat0_deg + math.degrees(north_m / earth_radius_m)
    cos_lat = max(0.01, abs(math.cos(lat0_rad)))
    lon_deg = lon0_deg + math.degrees(east_m / (earth_radius_m * cos_lat))
    return lat_deg, lon_deg


def gps_input_ignore_flags():
    return 0


def send_flow_gps_input(
    master,
    args,
    timestamp_us: int,
    state: FlowExternalNavState,
    origin_lat_deg: Optional[float],
    origin_lon_deg: Optional[float],
    origin_alt_m: Optional[float],
):
    if origin_lat_deg is None or origin_lon_deg is None or origin_alt_m is None:
        state.last_sent = False
        state.last_reason = "waiting_origin"
        return

    lat_deg, lon_deg = offset_m_to_lat_lon(
        origin_lat_deg,
        origin_lon_deg,
        state.north_m,
        state.east_m,
    )
    state.last_lat_deg = lat_deg
    state.last_lon_deg = lon_deg
    state.last_alt_m = origin_alt_m
    master.mav.gps_input_send(
        timestamp_us,
        args.gps_input_id,
        gps_input_ignore_flags(),
        0,
        0,
        3,
        int(round(lat_deg * 1e7)),
        int(round(lon_deg * 1e7)),
        float(origin_alt_m),
        0.8,
        1.5,
        state.last_vn_m_s,
        state.last_ve_m_s,
        0.0,
        args.gps_input_speed_acc_m_s,
        args.gps_input_hacc_m,
        args.gps_input_vacc_m,
        args.gps_input_sats,
    )
    state.last_sent = True


def send_flow_external_nav(
    master,
    timestamp_us: int,
    state: FlowExternalNavState,
    monitor: MonitorState,
):
    if monitor.attitude is None:
        return

    roll_rad = float(monitor.attitude.roll)
    pitch_rad = float(monitor.attitude.pitch)
    yaw_rad = float(monitor.attitude.yaw)
    master.mav.vision_position_estimate_send(
        timestamp_us,
        state.north_m,
        state.east_m,
        0.0,
        roll_rad,
        pitch_rad,
        yaw_rad,
    )
    master.mav.vision_speed_estimate_send(
        timestamp_us,
        state.last_vn_m_s,
        state.last_ve_m_s,
        0.0,
    )
    state.last_sent = True


def update_flow_external_nav(
    master,
    args,
    timestamp_us: int,
    sent_flow: SentFlow,
    distance_m: float,
    monitor: MonitorState,
    flow_health_is_ok: bool,
    flow_health_reason: str,
    state: FlowExternalNavState,
    origin_lat_deg: Optional[float],
    origin_lon_deg: Optional[float],
    origin_alt_m: Optional[float],
):
    if not args.external_nav_from_flow and not args.gps_input_from_flow:
        state.last_sent = False
        state.last_reason = "disabled"
        return

    if args.flow_message != "rad":
        state.last_sent = False
        state.last_reason = "requires_optical_flow_rad"
        return

    if monitor.attitude is None:
        state.last_sent = False
        state.last_reason = "waiting_attitude"
        return

    if not monitor.armed:
        state.reset(timestamp_us)
        state.last_reason = "standby_disarmed"
        if args.external_nav_from_flow:
            send_flow_external_nav(master, timestamp_us, state, monitor)
        if args.gps_input_from_flow:
            send_flow_gps_input(
                master,
                args,
                timestamp_us,
                state,
                origin_lat_deg,
                origin_lon_deg,
                origin_alt_m,
            )
        return

    velocity_ned = flow_external_nav_velocity_ned(sent_flow, distance_m, monitor, args)
    if velocity_ned is None:
        state.last_timestamp_us = None
        state.last_sent = False
        state.last_reason = "waiting_flow_range"
        return

    if not flow_health_is_ok:
        state.last_timestamp_us = None
        state.last_sent = False
        state.last_reason = f"flow_unhealthy:{flow_health_reason}"
        return

    if state.last_timestamp_us is None:
        state.last_timestamp_us = timestamp_us
        state.last_vn_m_s = 0.0
        state.last_ve_m_s = 0.0
        state.last_reason = "priming"
        if args.external_nav_from_flow:
            send_flow_external_nav(master, timestamp_us, state, monitor)
        if args.gps_input_from_flow:
            send_flow_gps_input(
                master,
                args,
                timestamp_us,
                state,
                origin_lat_deg,
                origin_lon_deg,
                origin_alt_m,
            )
        return

    dt_s = max((timestamp_us - state.last_timestamp_us) / 1_000_000.0, 1e-6)
    state.last_timestamp_us = timestamp_us
    state.last_vn_m_s, state.last_ve_m_s = velocity_ned
    state.north_m += state.last_vn_m_s * dt_s
    state.east_m += state.last_ve_m_s * dt_s
    state.last_reason = "flow_integrated"
    if args.external_nav_from_flow:
        send_flow_external_nav(master, timestamp_us, state, monitor)
    if args.gps_input_from_flow:
        send_flow_gps_input(
            master,
            args,
            timestamp_us,
            state,
            origin_lat_deg,
            origin_lon_deg,
            origin_alt_m,
        )


def estimate_correction_state(monitor: MonitorState, flow_quality: int, args):
    if (
        flow_quality < args.correction_quality_min
        or monitor.local_position is None
        or monitor.attitude is None
    ):
        return "waiting", 0.0, 0.0, 0.0

    roll_deg = math.degrees(float(monitor.attitude.roll))
    pitch_deg = math.degrees(float(monitor.attitude.pitch))
    speed_xy = math.hypot(
        float(monitor.local_position.vx),
        float(monitor.local_position.vy),
    )
    correcting = (
        abs(roll_deg) >= args.lean_threshold_deg
        or abs(pitch_deg) >= args.lean_threshold_deg
        or speed_xy >= args.speed_threshold
    )
    return ("active" if correcting else "steady"), roll_deg, pitch_deg, speed_xy


def vehicle_mode_name(msg) -> str:
    custom_mode = int(getattr(msg, "custom_mode", -1))
    if custom_mode in COPTER_MODE_NAMES:
        return COPTER_MODE_NAMES[custom_mode]
    try:
        mode = mavutil.mode_string_v10(msg)
        if mode:
            return str(mode).upper().replace(" ", "_")
    except Exception:
        pass
    return f"MODE_{custom_mode}"


def is_vehicle_heartbeat(master, msg) -> bool:
    if int(msg.get_srcSystem()) != int(master.target_system):
        return False
    mav_type = int(getattr(msg, "type", -1))
    autopilot = int(getattr(msg, "autopilot", -1))
    if mav_type in (
        getattr(mavutil.mavlink, "MAV_TYPE_GCS", 6),
        getattr(mavutil.mavlink, "MAV_TYPE_ONBOARD_CONTROLLER", 18),
    ):
        return False
    return autopilot != getattr(mavutil.mavlink, "MAV_AUTOPILOT_INVALID", 8)


def drain_messages(master, monitor: MonitorState, args, qgc_forwarder=None):
    now_s = time.time()
    if qgc_forwarder is not None:
        qgc_forwarder.drain_qgc_to_cube(master)
    while True:
        msg = master.recv_match(blocking=False)
        if msg is None:
            break

        if qgc_forwarder is not None:
            qgc_forwarder.forward_vehicle_message(msg)

        msg_type = msg.get_type()
        if msg_type == "LOCAL_POSITION_NED":
            monitor.local_position = msg
        elif msg_type == "ATTITUDE":
            monitor.attitude = msg
        elif msg_type == "EKF_STATUS_REPORT":
            monitor.ekf_status = msg
            monitor.ekf_flags = msg.flags
        elif msg_type == "DISTANCE_SENSOR":
            update_external_range_from_distance_sensor(monitor, msg, args, now_s)
        elif msg_type == "HEARTBEAT" and is_vehicle_heartbeat(master, msg):
            monitor.custom_mode = int(getattr(msg, "custom_mode", -1))
            monitor.flight_mode = vehicle_mode_name(msg)
            monitor.armed = bool(
                int(getattr(msg, "base_mode", 0) or 0)
                & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            )
        elif msg_type == "STATUSTEXT":
            text = getattr(msg, "text", "")
            if isinstance(text, bytes):
                text = text.decode("utf-8", errors="ignore")
            monitor.status_text = str(text).strip()
    if qgc_forwarder is not None:
        qgc_forwarder.drain_qgc_to_cube(master)


def announce_status(
    master,
    text: str,
    severity=mavutil.mavlink.MAV_SEVERITY_NOTICE,
    also_print: bool = True,
):
    send_statustext(master, text, severity)
    if also_print:
        print(text)


def source_set_parameters(source_set_id: int, profile: dict) -> dict:
    return {
        f"EK3_SRC{source_set_id}_{suffix}": value
        for suffix, value in profile.items()
    }


def ensure_parameter_value(master, name: str, value: float) -> bool:
    current_value = request_parameter(master, name)
    if current_value is None:
        raise RuntimeError(f"Unable to read {name} from flight controller")
    if abs(current_value - float(value)) < 0.01:
        return False

    applied = set_parameter(master, name, value)
    if applied is not True:
        raise RuntimeError(f"Unable to set {name} to {value}")
    return True


def enforce_gps_to_flow_transition(
    master,
    startup_set_id: int,
    flow_set_id: int,
    require_gps: bool,
    use_flow_external_nav: bool,
    use_flow_gps_input: bool,
):
    changed_params = []
    reboot_required_params = []

    if require_gps:
        gps1_type = request_parameter(master, "GPS1_TYPE")
        if gps1_type is None:
            raise RuntimeError("Unable to read GPS1_TYPE from flight controller")
        if int(round(gps1_type)) == 0:
            raise RuntimeError(
                "GPS1_TYPE=0 disables GPS. Set it to 2 or 9 before auto-home."
            )

    flow_type = request_parameter(master, "FLOW_TYPE")
    if flow_type is None:
        raise RuntimeError("Unable to read FLOW_TYPE from flight controller")

    flow_type_changed = False
    if int(round(flow_type)) != 5:
        applied = set_parameter(master, "FLOW_TYPE", 5)
        if applied is not True:
            raise RuntimeError("Unable to set FLOW_TYPE=5 for MAVLink optical flow")
        flow_type_changed = True
        changed_params.append("FLOW_TYPE")
        reboot_required_params.append("FLOW_TYPE")

    required_params = {
        "EK3_SRC_OPTIONS": 0,
        "EK3_OGN_HGT_MASK": 0,
    }
    if require_gps:
        required_params.update(source_set_parameters(startup_set_id, GPS_STARTUP_SOURCE_PROFILE))
    if flow_set_id > 0:
        if use_flow_gps_input:
            flow_profile = NO_GPS_FLOW_GPS_INPUT_SOURCE_PROFILE
        elif use_flow_external_nav:
            flow_profile = NO_GPS_FLOW_EXTERNAL_NAV_SOURCE_PROFILE
        else:
            flow_profile = NO_GPS_OPTICAL_FLOW_VELOCITY_SOURCE_PROFILE
        required_params.update(source_set_parameters(flow_set_id, flow_profile))
        if use_flow_gps_input:
            required_params.update(
                {
                    "GPS2_TYPE": 14,
                    "GPS_AUTO_SWITCH": 0,
                    "GPS_PRIMARY": 0,
                    "VISO_TYPE": 0,
                }
            )
        if use_flow_external_nav:
            required_params["VISO_TYPE"] = 3
        elif not use_flow_gps_input:
            required_params["VISO_TYPE"] = 0

    for param_name, desired_value in required_params.items():
        if ensure_parameter_value(master, param_name, desired_value):
            changed_params.append(param_name)
            if param_name in ("EK3_OGN_HGT_MASK", "VISO_TYPE", "GPS2_TYPE"):
                reboot_required_params.append(param_name)

    return flow_type_changed, changed_params, reboot_required_params


def desired_source_for_mode(mode: str, gps_source_set: int, flow_source_set: int) -> int:
    if mode == "POSHOLD" and flow_source_set > 0:
        return flow_source_set
    return gps_source_set


def select_ekf_source_set(
    master,
    target_source_set: int,
    args,
    reason: str,
    active_source_set: Optional[int],
) -> Optional[int]:
    if target_source_set <= 0 or target_source_set == active_source_set:
        return active_source_set

    switch_result = set_ekf_source_set(master, target_source_set)
    if switch_result is True:
        if target_source_set == args.post_home_ekf_source_set:
            publish_lua_status(master, LUA_STATE_NO_GPS_SOURCE_ACTIVE, target_source_set)
            announce_status(master, f"No-GPS source set {target_source_set} active")
            print(
                "EKF source selected:"
                f" {reason}; optical-flow source set {target_source_set} is active"
            )
        else:
            publish_lua_status(master, LUA_STATE_GPS_SOURCE_ACTIVE, target_source_set)
            announce_status(master, f"GPS source set {target_source_set} active")
            print(
                "EKF source selected:"
                f" {reason}; GPS source set {target_source_set} is active"
            )
        return target_source_set

    if switch_result is False:
        publish_lua_status(master, LUA_STATE_SOURCE_SWITCH_FAILED, target_source_set)
        announce_status(
            master,
            f"EKF source set {target_source_set} switch failed",
            mavutil.mavlink.MAV_SEVERITY_WARNING,
        )
        print(
            "EKF source switch was rejected:"
            f" wanted source set {target_source_set} for {reason}"
        )
        return active_source_set

    publish_lua_status(master, LUA_STATE_SOURCE_SWITCH_NO_ACK, target_source_set)
    announce_status(
        master,
        f"EKF source set {target_source_set} switch no ack",
        mavutil.mavlink.MAV_SEVERITY_WARNING,
    )
    print(
        "No COMMAND_ACK received for EKF source switch:"
        f" wanted source set {target_source_set} for {reason}"
    )
    return active_source_set


def select_gps_primary(
    master,
    target_primary: int,
    reason: str,
    active_primary: Optional[int],
) -> Optional[int]:
    if active_primary == target_primary:
        return active_primary
    applied = set_parameter(master, "GPS_PRIMARY", target_primary)
    if applied is True:
        print(
            "GPS primary selected:"
            f" GPS{target_primary + 1} for {reason}"
        )
        return target_primary
    print(
        "GPS primary switch was rejected:"
        f" wanted GPS{target_primary + 1} for {reason}"
    )
    return active_primary


def main():
    args = parse_args()
    if args.flow_health_test:
        args.external_nav_from_flow = False
        args.gps_input_from_flow = False

    if (args.origin_lat is None) != (args.origin_lon is None):
        raise SystemExit("Provide both --origin-lat and --origin-lon together")
    if args.send_rate <= 0:
        raise SystemExit("--send-rate must be > 0")
    if args.status_rate <= 0:
        raise SystemExit("--status-rate must be > 0")
    if args.gps_home_timeout < 0:
        raise SystemExit("--gps-home-timeout must be >= 0")
    if args.ekf_source_switch_after_sends < 1:
        raise SystemExit("--ekf-source-switch-after-sends must be >= 1")
    if not 0.0 < args.flow_smoothing_alpha <= 1.0:
        raise SystemExit("--flow-smoothing-alpha must be > 0 and <= 1")
    if args.flow_max_frame_px < 0:
        raise SystemExit("--flow-max-frame-px must be >= 0")
    if args.flow_max_step_px < 0:
        raise SystemExit("--flow-max-step-px must be >= 0")
    if args.flow_deadband_px < 0:
        raise SystemExit("--flow-deadband-px must be >= 0")
    if args.flow_scale <= 0:
        raise SystemExit("--flow-scale must be > 0")
    if args.flow_max_rate_rad_s < 0:
        raise SystemExit("--flow-max-rate-rad-s must be >= 0")
    if args.external_nav_from_flow and args.flow_message != "rad":
        raise SystemExit("--external-nav-from-flow requires --flow-message rad")
    if args.gps_input_from_flow and args.flow_message != "rad":
        raise SystemExit("--gps-input-from-flow requires --flow-message rad")
    if args.external_nav_from_flow and args.post_home_ekf_source_set <= 0:
        raise SystemExit("--external-nav-from-flow requires --post-home-ekf-source-set > 0")
    if args.gps_input_from_flow and args.post_home_ekf_source_set <= 0:
        raise SystemExit("--gps-input-from-flow requires --post-home-ekf-source-set > 0")
    if args.external_nav_from_flow and args.range_source == "none":
        raise SystemExit("--external-nav-from-flow requires a valid range source")
    if args.gps_input_from_flow and args.range_source == "none":
        raise SystemExit("--gps-input-from-flow requires a valid range source")
    if args.external_nav_from_flow and args.gps_input_from_flow:
        raise SystemExit(
            "Use either --gps-input-from-flow or --external-nav-from-flow, not both"
        )
    if args.gps_input_id < 0:
        raise SystemExit("--gps-input-id must be >= 0")
    if args.gps_input_sats < 0:
        raise SystemExit("--gps-input-sats must be >= 0")
    if args.gps_input_hacc_m <= 0:
        raise SystemExit("--gps-input-hacc-m must be > 0")
    if args.gps_input_vacc_m <= 0:
        raise SystemExit("--gps-input-vacc-m must be > 0")
    if args.gps_input_speed_acc_m_s <= 0:
        raise SystemExit("--gps-input-speed-acc-m-s must be > 0")
    if args.inertial_flow_gate_speed_m_s < 0:
        raise SystemExit("--inertial-flow-gate-speed-m-s must be >= 0")
    if args.inertial_flow_gate_lean_deg < 0:
        raise SystemExit("--inertial-flow-gate-lean-deg must be >= 0")
    if args.inertial_flow_gate_accel_m_s2 < 0:
        raise SystemExit("--inertial-flow-gate-accel-m-s2 must be >= 0")
    if args.lk_fb_max_error_px < 0:
        raise SystemExit("--lk-fb-max-error-px must be >= 0")
    if args.ransac_reproj_threshold_px < 0:
        raise SystemExit("--ransac-reproj-threshold-px must be >= 0")
    if args.external_range_min_m < 0:
        raise SystemExit("--external-range-min-m must be >= 0")
    if args.external_range_max_m <= args.external_range_min_m:
        raise SystemExit("--external-range-max-m must be greater than --external-range-min-m")
    if args.external_range_timeout <= 0:
        raise SystemExit("--external-range-timeout must be > 0")
    if args.range_alt_consistency_max_m < 0:
        raise SystemExit("--range-alt-consistency-max-m must be >= 0")
    if args.range_alt_consistency_min_alt_m < 0:
        raise SystemExit("--range-alt-consistency-min-alt-m must be >= 0")
    if args.flow_failsafe_quality_min < 0 or args.flow_failsafe_quality_min > 255:
        raise SystemExit("--flow-failsafe-quality-min must be between 0 and 255")
    if args.flow_failsafe_min_tracks < 0:
        raise SystemExit("--flow-failsafe-min-tracks must be >= 0")
    if args.flow_failsafe_min_height_m < 0:
        raise SystemExit("--flow-failsafe-min-height-m must be >= 0")
    if args.flow_failsafe_max_height_m < 0:
        raise SystemExit("--flow-failsafe-max-height-m must be >= 0")
    if (
        args.flow_failsafe_min_height_m > 0.0
        and args.flow_failsafe_max_height_m > 0.0
        and args.flow_failsafe_min_height_m >= args.flow_failsafe_max_height_m
    ):
        raise SystemExit(
            "--flow-failsafe-min-height-m must be less than --flow-failsafe-max-height-m"
        )
    if args.flow_failsafe_bad_seconds <= 0:
        raise SystemExit("--flow-failsafe-bad-seconds must be > 0")
    if args.flow_failsafe_land_seconds < 0:
        raise SystemExit("--flow-failsafe-land-seconds must be >= 0")
    if args.poshold_failsafe_bad_seconds <= 0:
        raise SystemExit("--poshold-failsafe-bad-seconds must be > 0")
    if args.poshold_ekf_variance_max < 0:
        raise SystemExit("--poshold-ekf-variance-max must be >= 0")
    mode_source_switch_enabled = (
        not args.disable_mode_source_switch
        and args.post_home_ekf_source_set > 0
        and not args.flow_health_test
    )
    flow_failsafe_enabled = (
        not args.disable_flow_failsafe
        and not args.flow_health_test
        and args.post_home_ekf_source_set > 0
    )
    flow_failsafe_land_enabled = (
        flow_failsafe_enabled
        and not args.disable_flow_failsafe_land
        and args.flow_failsafe_land_seconds > 0.0
    )
    one_time_flow_source_switch_enabled = (
        not args.flow_health_test
        and not mode_source_switch_enabled
        and args.post_home_ekf_source_set > 0
    )
    if (
        not args.flow_health_test
        and args.origin_lat is None
        and args.post_home_ekf_source_set > 0
        and args.post_home_ekf_source_set == args.gps_startup_ekf_source_set
    ):
        raise SystemExit(
            "Auto-GPS home requires --gps-startup-ekf-source-set and "
            "--post-home-ekf-source-set to be different."
        )

    source_component = (
        getattr(mavutil.mavlink, "MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY", 197)
        if args.external_nav_from_flow
        else mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER
    )
    qgc_forwarder = create_qgc_udp_forwarder(args)
    if qgc_forwarder is not None:
        static_targets = ", ".join(
            f"{host}:{port}" for host, port in qgc_forwarder.static_targets
        )
        print(
            "QGC UDP bridge enabled:"
            f" forwarding vehicle telemetry to {static_targets};"
            f" QGC uplink listens on UDP port {qgc_forwarder.bind_port}"
        )
    else:
        print("QGC UDP bridge disabled")

    master = connect_to_cube(
        args.ports,
        args.baud,
        source_component=source_component,
    )
    monitor = MonitorState()
    active_ekf_source_set: Optional[int] = None
    active_gps_primary: Optional[int] = None
    origin_lat_deg: Optional[float] = None
    origin_lon_deg: Optional[float] = None
    origin_alt_m: Optional[float] = None

    request_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 5)
    request_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 5)
    request_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_STATUSTEXT, 2)
    request_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT, 2)
    if args.range_source == "external":
        distance_sensor_msg_id = getattr(mavutil.mavlink, "MAVLINK_MSG_ID_DISTANCE_SENSOR", 132)
        request_message_interval(master, distance_sensor_msg_id, 10)
    time.sleep(0.5)
    announce_status(master, JETSON_BOOT_STATUS)
    publish_lua_status(master, LUA_STATE_JETSON_BOOT, args.post_home_ekf_source_set)
    played_init_tune = play_tune(master, SCRIPT_INIT_TUNE)
    if played_init_tune:
        print("Played script-init beep sequence: 3 short beeps")
    else:
        print("Script-init beep sequence was not available on this link.")

    try:
        flow_type_changed, changed_params, reboot_required_params = enforce_gps_to_flow_transition(
            master,
            startup_set_id=args.gps_startup_ekf_source_set,
            flow_set_id=0 if args.flow_health_test else args.post_home_ekf_source_set,
            require_gps=args.origin_lat is None and not args.flow_health_test,
            use_flow_external_nav=args.external_nav_from_flow,
            use_flow_gps_input=args.gps_input_from_flow,
        )
    except RuntimeError as exc:
        publish_lua_status(master, LUA_STATE_CONFIG_FAILED, args.post_home_ekf_source_set)
        announce_status(
            master,
            "EKF transition config failed",
            mavutil.mavlink.MAV_SEVERITY_WARNING,
        )
        print(exc)
        raise SystemExit(1) from exc

    if changed_params:
        announce_status(master, "EKF transition config applied")
        print("Configured EKF transition params: " + ", ".join(changed_params))

    if reboot_required_params:
        announce_status(
            master,
            "Reboot FC to finish no-GPS transition setup",
            mavutil.mavlink.MAV_SEVERITY_WARNING,
        )
        print(
            "These parameters were changed and require a flight-controller reboot before"
            " the no-GPS transition can be trusted: "
            + ", ".join(reboot_required_params)
        )
        raise SystemExit(1)

    if not args.flow_health_test and args.gps_startup_ekf_source_set > 0:
        active_ekf_source_set = select_ekf_source_set(
            master,
            args.gps_startup_ekf_source_set,
            args,
            "startup GPS safety before optical-flow lock",
            active_ekf_source_set,
        )

    if args.flow_health_test:
        announce_status(master, "Flow health test: skipping GPS home/origin")
        announce_status(
            master,
            "Flow health test: EKF source switching disabled",
            mavutil.mavlink.MAV_SEVERITY_WARNING,
        )
        print(
            "Flow health test mode is for prop-less/in-hand validation only."
            " It streams optical flow and DISTANCE_SENSOR without selecting"
            " the no-GPS EKF source."
        )
        time.sleep(1.0)
    elif args.origin_lat is not None and args.origin_lon is not None:
        set_origin_and_home(master, args.origin_lat, args.origin_lon, args.origin_alt)
        origin_lat_deg = args.origin_lat
        origin_lon_deg = args.origin_lon
        origin_alt_m = args.origin_alt
        publish_lua_status(master, LUA_STATE_MANUAL_ORIGIN, args.post_home_ekf_source_set)
        announce_status(
            master,
            "Manual origin set; optical-flow source will lock after flow starts",
        )
        time.sleep(1.0)
    else:
        home_position_msg_id = getattr(mavutil.mavlink, "MAVLINK_MSG_ID_HOME_POSITION", 242)
        request_message_interval(master, home_position_msg_id, 1)
        request_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 2)
        request_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 2)
        publish_lua_status(master, LUA_STATE_GPS_ASSIST_ACTIVE, args.post_home_ekf_source_set)
        announce_status(master, "GPS assist active, locking home position")
        announce_status(master, "Trying to get home position from GPS", also_print=False)
        if args.gps_home_timeout == 0:
            print(
                "Origin/home not provided."
                " Waiting indefinitely for GPS home lock before starting optical flow."
            )
            announce_status(master, "Waiting for GPS home before flow start")
        else:
            print(
                "Origin/home not provided."
                f" Waiting up to {args.gps_home_timeout:.0f}s for a one-time GPS lock"
                " before starting optical flow."
            )
            announce_status(master, "Waiting for GPS home before flow start")
        gps_home = wait_for_gps_home(
            master,
            timeout_s=args.gps_home_timeout,
            min_fix_type=args.gps_min_fix,
            min_sats=args.gps_min_sats,
            message_callback=(
                qgc_forwarder.forward_vehicle_message
                if qgc_forwarder is not None
                else None
            ),
            idle_callback=(
                lambda: qgc_forwarder.drain_qgc_to_cube(master)
                if qgc_forwarder is not None
                else None
            ),
        )
        request_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, -1)
        request_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, -1)
        request_message_interval(master, home_position_msg_id, -1)

        if gps_home is not None:
            set_origin_and_home(
                master,
                gps_home.lat_deg,
                gps_home.lon_deg,
                gps_home.alt_m,
            )
            origin_lat_deg = gps_home.lat_deg
            origin_lon_deg = gps_home.lon_deg
            origin_alt_m = gps_home.alt_m
            time.sleep(1.0)
            publish_lua_status(master, LUA_STATE_ORIGIN_LOCKED, args.post_home_ekf_source_set)
            announce_status(master, "Origin locked from GPS")
            played_lock_tune = play_tune(master, GPS_HOME_LOCK_TUNE)
            announce_status(master, "Companion GPS requests disabled", also_print=False)
            announce_status(
                master,
                "RealSense flow ready for PosHold source switching",
            )
            print(
                "Locked origin/home from GPS once:"
                f" source={gps_home.source}"
                f" lat={gps_home.lat_deg:.7f}"
                f" lon={gps_home.lon_deg:.7f}"
                f" alt={gps_home.alt_m:.1f}m"
            )
            print(
                "Stopped requesting GPS telemetry from the Cube for the rest of this run."
                " PosHold will use optical flow when healthy; other modes stay on GPS."
            )
            if played_lock_tune:
                print("Played GPS-home lock beep sequence: 2 long beeps")
                time.sleep(GPS_HOME_LOCK_TUNE_GUARD_S)
            else:
                print("GPS-home lock beep sequence was not available on this link.")
        else:
            publish_lua_status(master, LUA_STATE_GPS_TIMEOUT, args.post_home_ekf_source_set)
            announce_status(
                master,
                "GPS home lock timeout",
                mavutil.mavlink.MAV_SEVERITY_WARNING,
            )
            announce_status(
                master,
                "No-GPS flow waiting for origin",
                mavutil.mavlink.MAV_SEVERITY_WARNING,
                also_print=False,
            )
            print(
                "GPS home lock was not acquired in time."
                " Origin/home was not sent."
                " Optical flow will not start without a valid home/origin."
            )
            raise SystemExit(1)

    pipeline = rs.pipeline()
    config = rs.config()
    enable_streams(config, args)
    pipeline.start(config)
    publish_lua_status(
        master,
        LUA_STATE_FLOW_STARTED,
        0 if args.flow_health_test else args.post_home_ekf_source_set,
    )
    if args.flow_health_test:
        announce_status(master, "Optical-flow health stream started")
    else:
        announce_status(master, "No-GPS flow bridge started")
    if args.range_source == "external":
        announce_status(master, "Using FC down lidar for optical-flow range")
        print(
            "Range source: external DISTANCE_SENSOR from the flight controller"
            f" min={args.external_range_min_m:.1f}m"
            f" max={args.external_range_max_m:.1f}m"
            f" timeout={args.external_range_timeout:.1f}s"
        )
    elif args.range_source == "realsense":
        announce_status(master, "Using RealSense depth for optical-flow range")
        print(
            "Range source: RealSense depth DISTANCE_SENSOR"
            f" min={args.min_cm / 100.0:.1f}m max={args.max_cm / 100.0:.1f}m"
        )
    else:
        announce_status(
            master,
            "No range source: optical flow diagnostics only",
            mavutil.mavlink.MAV_SEVERITY_WARNING,
        )
    played_start_tune = play_tune(master, SCRIPT_START_TUNE)
    if played_start_tune:
        print("Played script-start beep sequence: 2 short beeps")
    else:
        print("Script-start beep sequence was not available on this link.")

    tracker = FlowTracker(
        max_features=args.max_features,
        min_tracks=args.min_tracks,
        lk_window=args.lk_window,
        lk_levels=args.lk_levels,
        max_fb_error_px=args.lk_fb_max_error_px,
        ransac_reproj_threshold_px=args.ransac_reproj_threshold_px,
    )
    smoother = FlowSmoother(
        alpha=args.flow_smoothing_alpha,
        max_frame_px=args.flow_max_frame_px,
        max_step_px=args.flow_max_step_px,
        deadband_px=args.flow_deadband_px,
    )
    flow_intrinsics: Optional[FlowIntrinsics] = None

    send_period = 1.0 / args.send_rate
    status_period = 1.0 / args.status_rate
    next_send = time.time()
    next_status = time.time()
    start_time = time.time()

    last_sent_flow = SentFlow(0.0, 0.0, "rad")
    last_raw_flow_dpix = (0, 0)
    last_distance_m = -1.0
    last_tracks = 0
    last_quality = 0
    last_dt_us = 0
    last_send_us: Optional[int] = None
    last_correction_state = "waiting"
    healthy_flow_sends = 0
    last_flow_limited = False
    last_range_age_s = -1.0
    last_range_state = "unknown"
    bad_flow_since: Optional[float] = None
    bad_flow_duration_s = 0.0
    last_flow_health_state = "unknown"
    last_flow_health_reason = "unknown"
    last_inertial_check = InertialFlowCheck(True, "waiting")
    flow_external_nav_state = FlowExternalNavState()
    gps_fallback_active = False
    land_command_sent = False
    poshold_failsafe_mode_sent = False
    next_source_switch_retry = 0.0
    next_gps_less_active_ping = 0.0
    flow_source_locked = False
    last_announced_mode = monitor.flight_mode
    last_armed_state = monitor.armed
    run_id = time.strftime("%Y%m%d-%H%M%S")
    flow_csv_file = None
    flow_csv_writer = None
    next_csv_flush = time.time() + 1.0
    if not args.disable_flow_csv_log:
        try:
            flow_csv_file, flow_csv_writer = open_flow_csv_log(args.flow_csv_log)
            print(f"Companion optical-flow CSV log: {args.flow_csv_log}")
        except OSError as exc:
            print(f"Unable to open optical-flow CSV log {args.flow_csv_log}: {exc}")

    if mode_source_switch_enabled:
        active_ekf_source_set = select_ekf_source_set(
            master,
            args.gps_startup_ekf_source_set,
            args,
            "startup default for Loiter/Land",
            active_ekf_source_set,
        )
    elif one_time_flow_source_switch_enabled:
        print(
            "EKF source switching mode: one-time optical-flow lock after"
            f" {args.ekf_source_switch_after_sends} consecutive healthy flow sends"
        )
    else:
        print("EKF source switching mode: off; streaming sensors only")

    if flow_failsafe_enabled:
        announce_status(master, "Optical-flow GPS fallback armed")
        print(
            "Optical-flow failsafe:"
            f" GPS fallback after {args.flow_failsafe_bad_seconds:.1f}s bad health;"
            f" land_after={args.flow_failsafe_land_seconds:.1f}s"
            f" poshold_fallback={args.poshold_failsafe_mode}"
            f" after={args.poshold_failsafe_bad_seconds:.1f}s"
            f" min_flow_height={args.flow_failsafe_min_height_m:.1f}m"
            f" max_flow_height={args.flow_failsafe_max_height_m:.1f}m"
            f" min_quality={args.flow_failsafe_quality_min}"
            f" min_tracks={args.flow_failsafe_min_tracks}"
        )
    else:
        print("Optical-flow failsafe: disabled")

    if args.disable_inertial_flow_gate:
        print("Inertial flow plausibility gate: disabled")
    else:
        print(
            "Inertial flow plausibility gate:"
            f" speed>={args.inertial_flow_gate_speed_m_s:.1f}m/s"
            f" lean>={args.inertial_flow_gate_lean_deg:.1f}deg"
            f" accel>={args.inertial_flow_gate_accel_m_s2:.1f}m/s^2"
        )

    if args.gps_input_from_flow:
        announce_status(master, "Flow-derived GPS_INPUT enabled")
        print(
            "Flow-derived GPS_INPUT:"
            f" sends MAVLink GPS_INPUT gps_id={args.gps_input_id}"
            " from optical flow plus lidar height"
            f" forward_scale={args.flow_odom_forward_scale:+.2f}"
            f" right_scale={args.flow_odom_right_scale:+.2f}"
            " source_profile=GPS position+velocity"
        )
    elif args.external_nav_from_flow:
        announce_status(master, "Flow-derived external nav enabled")
        print(
            "Flow-derived external nav:"
            " sends VISION_POSITION_ESTIMATE/VISION_SPEED_ESTIMATE from optical flow"
            f" forward_scale={args.flow_odom_forward_scale:+.2f}"
            f" right_scale={args.flow_odom_right_scale:+.2f}"
            " source_profile=external-nav position+velocity"
        )
    else:
        print(
            "Flow-derived navigation: disabled;"
            " no-GPS source set uses optical-flow velocity only"
        )

    print(
        "Streaming RealSense optical flow to Cube:"
        f" source={args.flow_source}"
        f" flow_message={args.flow_message}"
        f" send_rate={args.send_rate:.1f}Hz"
        f" ekf_switch="
        f"{'mode' if mode_source_switch_enabled else ('one-time-flow' if one_time_flow_source_switch_enabled else 'off')}"
    )

    try:
        while True:
            frames = pipeline.wait_for_frames(timeout_ms=5000)
            now = time.time()
            drain_messages(master, monitor, args, qgc_forwarder)
            if (
                (args.external_nav_from_flow or args.gps_input_from_flow)
                and monitor.armed != last_armed_state
            ):
                flow_external_nav_state.reset()
                last_armed_state = monitor.armed
                if monitor.armed:
                    announce_status(master, "Flow odometry reset on arm")
                else:
                    announce_status(master, "Flow odometry reset on disarm")

            if now < next_send:
                continue

            depth_frame = frames.get_depth_frame()
            flow_frame = read_flow_frame(frames, args.flow_source)
            if not depth_frame or not flow_frame:
                continue

            distance_m = selected_range_m(monitor, depth_frame, args, now)
            gray = prepare_flow_image(flow_frame, args)
            if args.flow_message == "rad" and flow_intrinsics is None:
                flow_intrinsics = flow_intrinsics_from_frame(flow_frame, args)
                print(
                    "RealSense flow intrinsics:"
                    f" fx={flow_intrinsics.fx_px:.1f}px"
                    f" fy={flow_intrinsics.fy_px:.1f}px"
                    f" rotate={args.rotate}"
                    f" scale={args.flow_scale:.2f}"
                    f" max_rate={args.flow_max_rate_rad_s:.2f}rad/s"
                )

            timestamp_us = int(now * 1e6)
            dt_us = 0 if last_send_us is None else timestamp_us - last_send_us
            last_send_us = timestamp_us
            next_send = now + send_period

            raw_flow = tracker.update(gray)
            if raw_flow is None or dt_us <= 0:
                last_distance_m = distance_m
                continue

            flow = smoother.update(raw_flow)
            last_raw_flow_dpix = (
                clamp_int16(int(round(raw_flow.dx_px * 10.0))),
                clamp_int16(int(round(raw_flow.dy_px * 10.0))),
            )
            flow, last_inertial_check = apply_inertial_flow_gate(
                flow,
                args,
                monitor,
                dt_us,
                distance_m,
                flow_intrinsics,
            )
            last_sent_flow = send_optical_flow(
                master,
                args,
                timestamp_us,
                dt_us,
                flow,
                distance_m,
                flow_intrinsics,
            )
            if args.range_source == "realsense":
                send_distance_sensor(
                    master,
                    args,
                    distance_m,
                    int((now - start_time) * 1000.0),
                )

            last_distance_m = distance_m
            last_tracks = flow.tracks
            last_quality = flow.quality
            last_dt_us = dt_us
            last_flow_limited = smoother.last_limited
            last_range_age_s = (
                now - monitor.rangefinder_last_seen_s
                if args.range_source == "external" and monitor.rangefinder_last_seen_s > 0.0
                else -1.0
            )
            (
                current_flow_health_ok,
                flow_health_reason,
                flow_health_critical,
            ) = flow_health_ok(flow, distance_m, monitor, args)
            if not last_inertial_check.ok:
                current_flow_health_ok = False
                flow_health_critical = True
                flow_health_reason = (
                    "inertial gate rejected flow:"
                    f" {last_inertial_check.implied_speed_m_s:.1f}m/s"
                    f" with lean_accel={last_inertial_check.lean_accel_m_s2:.1f}m/s2"
                )
            if monitor.flight_mode == "POSHOLD":
                ekf_variance_reason = ekf_variance_failure_reason(monitor, args)
                if ekf_variance_reason is not None:
                    current_flow_health_ok = False
                    flow_health_critical = True
                    if flow_health_reason == "healthy":
                        flow_health_reason = ekf_variance_reason
                    else:
                        flow_health_reason = f"{flow_health_reason}, {ekf_variance_reason}"
            update_flow_external_nav(
                master,
                args,
                timestamp_us,
                last_sent_flow,
                distance_m,
                monitor,
                current_flow_health_ok,
                flow_health_reason,
                flow_external_nav_state,
                origin_lat_deg,
                origin_lon_deg,
                origin_alt_m,
            )
            if (
                (args.external_nav_from_flow or args.gps_input_from_flow)
                and current_flow_health_ok
                and not flow_external_nav_state.last_sent
            ):
                current_flow_health_ok = False
                flow_health_critical = True
                flow_health_reason = (
                    "flow nav not sent:"
                    f" {flow_external_nav_state.last_reason}"
                )
            last_flow_health_reason = flow_health_reason
            if current_flow_health_ok:
                healthy_flow_sends += 1
                bad_flow_since = None
                bad_flow_duration_s = 0.0
            else:
                healthy_flow_sends = 0
                if bad_flow_since is None:
                    bad_flow_since = now
                bad_flow_duration_s = now - bad_flow_since

            flow_health_state = "healthy" if current_flow_health_ok else "bad"
            if flow_health_state != last_flow_health_state:
                if current_flow_health_ok:
                    announce_status(master, "Optical-flow health recovered")
                else:
                    announce_status(
                        master,
                        f"Optical-flow health bad: {flow_health_reason}",
                        mavutil.mavlink.MAV_SEVERITY_WARNING,
                    )
                last_flow_health_state = flow_health_state

            if args.range_source == "external":
                range_state = "healthy" if distance_m > 0.0 else "waiting"
                if range_state != last_range_state:
                    if range_state == "healthy":
                        announce_status(master, "External lidar range healthy")
                    else:
                        announce_status(
                            master,
                            "External lidar range waiting/stale",
                            mavutil.mavlink.MAV_SEVERITY_WARNING,
                        )
                    last_range_state = range_state
            mode_changed = monitor.flight_mode != last_announced_mode

            desired_mode_source_set = None
            if mode_source_switch_enabled:
                desired_mode_source_set = desired_source_for_mode(
                    monitor.flight_mode,
                    args.gps_startup_ekf_source_set,
                    args.post_home_ekf_source_set,
                )
                if (
                    desired_mode_source_set == args.gps_startup_ekf_source_set
                    and (
                        active_ekf_source_set != desired_mode_source_set
                        or flow_source_locked
                        or mode_changed
                    )
                ):
                    if args.gps_input_from_flow:
                        active_gps_primary = select_gps_primary(
                            master,
                            0,
                            f"flight mode {monitor.flight_mode}; real GPS for non-PosHold",
                            active_gps_primary,
                        )
                    active_ekf_source_set = select_ekf_source_set(
                        master,
                        desired_mode_source_set,
                        args,
                        f"flight mode {monitor.flight_mode}; GPS for non-PosHold",
                        None if mode_changed else active_ekf_source_set,
                    )
                    if active_ekf_source_set == args.gps_startup_ekf_source_set:
                        flow_source_locked = False
                        gps_fallback_active = False

            using_flow_source = (
                active_ekf_source_set == args.post_home_ekf_source_set
                or flow_source_locked
            )
            gps_less_poshold_active = (
                monitor.flight_mode == "POSHOLD"
                and using_flow_source
                and not gps_fallback_active
            )
            if gps_less_poshold_active and now >= next_gps_less_active_ping:
                publish_lua_status(
                    master,
                    LUA_STATE_GPS_LESS_FLIGHT_ACTIVE,
                    args.post_home_ekf_source_set,
                )
                announce_status(
                    master,
                    "GPS-Less flight active",
                    mavutil.mavlink.MAV_SEVERITY_NOTICE,
                )
                next_gps_less_active_ping = now + 6.0
            elif not gps_less_poshold_active:
                next_gps_less_active_ping = now
            if monitor.flight_mode != "POSHOLD":
                poshold_failsafe_mode_sent = False

            poshold_flow_failed = (
                flow_failsafe_enabled
                and mode_source_switch_enabled
                and monitor.armed
                and monitor.flight_mode == "POSHOLD"
                and not current_flow_health_ok
                and bad_flow_duration_s >= args.poshold_failsafe_bad_seconds
                and not poshold_failsafe_mode_sent
            )
            if poshold_flow_failed:
                announce_status(
                    master,
                    "PosHold flow unhealthy: returning to GPS Loiter",
                    mavutil.mavlink.MAV_SEVERITY_WARNING,
                )
                active_ekf_source_set = select_ekf_source_set(
                    master,
                    args.gps_startup_ekf_source_set,
                    args,
                    f"PosHold flow guard: {flow_health_reason}",
                    active_ekf_source_set,
                )
                if active_ekf_source_set == args.gps_startup_ekf_source_set:
                    if args.gps_input_from_flow:
                        active_gps_primary = select_gps_primary(
                            master,
                            0,
                            "PosHold flow guard: real GPS fallback",
                            active_gps_primary,
                        )
                    gps_fallback_active = True
                    flow_source_locked = False
                if args.poshold_failsafe_mode == "LOITER":
                    if command_flight_mode(master, "LOITER"):
                        monitor.flight_mode = "LOITER"
                        poshold_failsafe_mode_sent = True
                        announce_status(
                            master,
                            "LOITER commanded by PosHold flow guard",
                            mavutil.mavlink.MAV_SEVERITY_WARNING,
                        )
                    else:
                        poshold_failsafe_mode_sent = True
                        announce_status(
                            master,
                            "Unable to command LOITER; GPS fallback attempted",
                            mavutil.mavlink.MAV_SEVERITY_CRITICAL,
                        )
                else:
                    poshold_failsafe_mode_sent = True

            if (
                flow_failsafe_enabled
                and not current_flow_health_ok
                and bad_flow_duration_s >= args.flow_failsafe_bad_seconds
                and using_flow_source
                and not gps_fallback_active
            ):
                announce_status(
                    master,
                    "Flow unhealthy: switching back to GPS",
                    mavutil.mavlink.MAV_SEVERITY_WARNING,
                )
                active_ekf_source_set = select_ekf_source_set(
                    master,
                    args.gps_startup_ekf_source_set,
                    args,
                    f"optical-flow failsafe: {flow_health_reason}",
                    active_ekf_source_set,
                )
                if active_ekf_source_set == args.gps_startup_ekf_source_set:
                    if args.gps_input_from_flow:
                        active_gps_primary = select_gps_primary(
                            master,
                            0,
                            "optical-flow failsafe: real GPS fallback",
                            active_gps_primary,
                        )
                    gps_fallback_active = True
                    flow_source_locked = False
                    announce_status(master, "GPS fallback active")
                else:
                    if flow_failsafe_land_enabled and monitor.armed and not land_command_sent:
                        announce_status(
                            master,
                            "GPS fallback failed; commanding LAND",
                            mavutil.mavlink.MAV_SEVERITY_CRITICAL,
                        )
                        if command_land(master):
                            land_command_sent = True
                            monitor.flight_mode = "LAND"
                            announce_status(
                                master,
                                "LAND commanded after GPS fallback failure",
                                mavutil.mavlink.MAV_SEVERITY_CRITICAL,
                            )
                    else:
                        announce_status(
                            master,
                            "GPS fallback failed; pilot action required",
                            mavutil.mavlink.MAV_SEVERITY_CRITICAL,
                        )

            should_land_for_bad_flow = (
                flow_failsafe_land_enabled
                and monitor.armed
                and not land_command_sent
                and flow_health_critical
                and bad_flow_duration_s >= args.flow_failsafe_land_seconds
                and (gps_fallback_active or using_flow_source)
            )
            if should_land_for_bad_flow:
                announce_status(
                    master,
                    "Persistent bad flow/range: commanding LAND",
                    mavutil.mavlink.MAV_SEVERITY_CRITICAL,
                )
                if command_land(master):
                    land_command_sent = True
                    monitor.flight_mode = "LAND"
                    announce_status(
                        master,
                        "LAND commanded by optical-flow failsafe",
                        mavutil.mavlink.MAV_SEVERITY_CRITICAL,
                    )

            if healthy_flow_sends >= args.ekf_source_switch_after_sends:
                if (
                    one_time_flow_source_switch_enabled
                    and not flow_source_locked
                    and not gps_fallback_active
                    and now >= next_source_switch_retry
                ):
                    next_source_switch_retry = now + 2.0
                    if args.gps_input_from_flow:
                        active_gps_primary = select_gps_primary(
                            master,
                            args.gps_input_id,
                            "healthy flow GPS_INPUT source",
                            active_gps_primary,
                        )
                    active_ekf_source_set = select_ekf_source_set(
                        master,
                        args.post_home_ekf_source_set,
                        args,
                        "RealSense optical flow is healthy; locking GPS-free PosHold lane",
                        active_ekf_source_set,
                    )
                    if active_ekf_source_set == args.post_home_ekf_source_set:
                        flow_source_locked = True
                        if flow_failsafe_enabled:
                            announce_status(
                                master,
                                "No-GPS optical-flow source locked; GPS fallback armed",
                            )
                        else:
                            announce_status(
                                master,
                                "No-GPS optical-flow source locked; no automatic GPS switching",
                            )
                elif (
                    mode_source_switch_enabled
                    and desired_mode_source_set == args.post_home_ekf_source_set
                    and now >= next_source_switch_retry
                ):
                    if desired_mode_source_set != active_ekf_source_set or mode_changed:
                        if args.gps_input_from_flow:
                            active_gps_primary = select_gps_primary(
                                master,
                                args.gps_input_id,
                                f"flight mode {monitor.flight_mode}; flow GPS_INPUT",
                                active_gps_primary,
                            )
                        active_ekf_source_set = select_ekf_source_set(
                            master,
                            desired_mode_source_set,
                            args,
                            f"flight mode {monitor.flight_mode}",
                            None if mode_changed else active_ekf_source_set,
                        )
                        next_source_switch_retry = now + 2.0
                        if active_ekf_source_set == args.post_home_ekf_source_set:
                            flow_source_locked = True
                            gps_fallback_active = False

            if mode_changed:
                print(
                    "Vehicle mode update:"
                    f" {monitor.flight_mode};"
                    f" active_source_set={active_ekf_source_set or 'unknown'}"
                    f" flow_locked={'yes' if flow_source_locked else 'no'}"
                )
                last_announced_mode = monitor.flight_mode

            correction_state, roll_deg, pitch_deg, speed_xy = estimate_correction_state(
                monitor,
                last_quality,
                args,
            )
            if correction_state != last_correction_state:
                if correction_state == "active":
                    print(
                        "Optical-flow correction active:"
                        f" roll={roll_deg:+.1f}deg pitch={pitch_deg:+.1f}deg"
                        f" speed_xy={speed_xy:.2f}m/s quality={last_quality}"
                    )
                elif correction_state == "steady":
                    print(
                        "Optical-flow hold steady:"
                        f" roll={roll_deg:+.1f}deg pitch={pitch_deg:+.1f}deg"
                        f" speed_xy={speed_xy:.2f}m/s quality={last_quality}"
                    )
                else:
                    print(
                        "Optical-flow correction waiting:"
                        " need attitude/local-position telemetry and stable flow quality"
                    )
                last_correction_state = correction_state

            if flow_csv_writer is not None:
                local_position = monitor.local_position
                ekf_status = monitor.ekf_status
                try:
                    flow_csv_writer.writerow(
                        {
                            "run_id": run_id,
                            "unix_s": csv_value(now),
                            "run_s": csv_value(now - start_time),
                            "timestamp_us": timestamp_us,
                            "mode": monitor.flight_mode,
                            "armed": int(monitor.armed),
                            "ekf_source": active_ekf_source_set or "",
                            "flow_locked": int(flow_source_locked),
                            "gps_fallback": int(gps_fallback_active),
                            "distance_m": csv_value(last_distance_m),
                            "range_source": args.range_source,
                            "range_age_s": csv_value(last_range_age_s),
                            "raw_dx_px": last_raw_flow_dpix[0],
                            "raw_dy_px": last_raw_flow_dpix[1],
                            "sent_label": last_sent_flow.label,
                            "sent_x": csv_value(last_sent_flow.x),
                            "sent_y": csv_value(last_sent_flow.y),
                            "rate_x_rad_s": csv_value(last_sent_flow.rate_x_rad_s),
                            "rate_y_rad_s": csv_value(last_sent_flow.rate_y_rad_s),
                            "flow_limited": int(last_flow_limited),
                            "dt_ms": csv_value(last_dt_us / 1000.0),
                            "tracks": last_tracks,
                            "quality": last_quality,
                            "flow_health": last_flow_health_state,
                            "flow_reason": last_flow_health_reason,
                            "healthy_sends": healthy_flow_sends,
                            "bad_flow_s": csv_value(bad_flow_duration_s),
                            "inertial_ok": int(last_inertial_check.ok),
                            "inertial_reason": last_inertial_check.reason,
                            "of_speed_m_s": csv_value(last_inertial_check.implied_speed_m_s),
                            "lean_accel_m_s2": csv_value(last_inertial_check.lean_accel_m_s2),
                            "roll_deg": csv_value(roll_deg),
                            "pitch_deg": csv_value(pitch_deg),
                            "speed_xy_m_s": csv_value(speed_xy),
                            "local_x_m": csv_value(
                                getattr(local_position, "x", None)
                                if local_position is not None
                                else None
                            ),
                            "local_y_m": csv_value(
                                getattr(local_position, "y", None)
                                if local_position is not None
                                else None
                            ),
                            "local_vx_m_s": csv_value(
                                getattr(local_position, "vx", None)
                                if local_position is not None
                                else None
                            ),
                            "local_vy_m_s": csv_value(
                                getattr(local_position, "vy", None)
                                if local_position is not None
                                else None
                            ),
                            "ekf_flags": (
                                f"0x{monitor.ekf_flags:x}"
                                if monitor.ekf_flags is not None
                                else ""
                            ),
                            "ekf_vel_var": csv_value(
                                getattr(ekf_status, "velocity_variance", None)
                                if ekf_status is not None
                                else None
                            ),
                            "ekf_pos_h_var": csv_value(
                                getattr(ekf_status, "pos_horiz_variance", None)
                                if ekf_status is not None
                                else None
                            ),
                            "ekf_pos_v_var": csv_value(
                                getattr(ekf_status, "pos_vert_variance", None)
                                if ekf_status is not None
                                else None
                            ),
                            "ekf_terrain_var": csv_value(
                                getattr(ekf_status, "terrain_alt_variance", None)
                                if ekf_status is not None
                                else None
                            ),
                            "vision_sent": int(flow_external_nav_state.last_sent),
                            "vision_reason": flow_external_nav_state.last_reason,
                            "vision_n_m": csv_value(flow_external_nav_state.north_m),
                            "vision_e_m": csv_value(flow_external_nav_state.east_m),
                            "vision_vn_m_s": csv_value(
                                flow_external_nav_state.last_vn_m_s
                            ),
                            "vision_ve_m_s": csv_value(
                                flow_external_nav_state.last_ve_m_s
                            ),
                            "gps_input_sent": int(
                                args.gps_input_from_flow
                                and flow_external_nav_state.last_sent
                            ),
                            "gps_input_reason": (
                                flow_external_nav_state.last_reason
                                if args.gps_input_from_flow
                                else ""
                            ),
                            "gps_input_lat": csv_value(
                                flow_external_nav_state.last_lat_deg
                            ),
                            "gps_input_lon": csv_value(
                                flow_external_nav_state.last_lon_deg
                            ),
                            "gps_input_alt_m": csv_value(
                                flow_external_nav_state.last_alt_m
                            ),
                            "gps_primary": active_gps_primary or 0,
                        }
                    )
                    if flow_csv_file is not None and now >= next_csv_flush:
                        flow_csv_file.flush()
                        next_csv_flush = now + 1.0
                except OSError as exc:
                    print(f"Optical-flow CSV logging disabled after write error: {exc}")
                    try:
                        if flow_csv_file is not None:
                            flow_csv_file.close()
                    finally:
                        flow_csv_file = None
                        flow_csv_writer = None

            if now >= next_status:
                summary = [
                    f"source={args.flow_source}",
                    f"range_source={args.range_source}",
                    f"distance={last_distance_m:.2f}m" if last_distance_m >= 0.0 else "distance=unknown",
                    f"raw_dpix=({last_raw_flow_dpix[0]:+d},{last_raw_flow_dpix[1]:+d})",
                    (
                        f"flow_rate=({last_sent_flow.rate_x_rad_s:+.3f},"
                        f"{last_sent_flow.rate_y_rad_s:+.3f})rad/s"
                        if last_sent_flow.label == "rad"
                        else f"flow_dpix=({int(last_sent_flow.x):+d},{int(last_sent_flow.y):+d})"
                    ),
                    f"flow_limited={'yes' if last_flow_limited else 'no'}",
                    f"dt={last_dt_us / 1000.0:.1f}ms",
                    f"tracks={last_tracks}",
                    f"quality={last_quality}",
                    f"inertial_gate={'pass' if last_inertial_check.ok else 'reject'}",
                    f"inertial_reason={last_inertial_check.reason}",
                    f"of_speed={last_inertial_check.implied_speed_m_s:.2f}m/s",
                    f"lean_accel={last_inertial_check.lean_accel_m_s2:.2f}m/s2",
                    f"flow_health={last_flow_health_state}",
                    f"bad_flow={bad_flow_duration_s:.1f}s",
                    (
                        f"flow_gps=sent:{flow_external_nav_state.last_reason}"
                        f" ll=({flow_external_nav_state.last_lat_deg or 0.0:+.7f},"
                        f"{flow_external_nav_state.last_lon_deg or 0.0:+.7f})"
                        f" v=({flow_external_nav_state.last_vn_m_s:+.2f},"
                        f"{flow_external_nav_state.last_ve_m_s:+.2f})m/s"
                        if args.gps_input_from_flow
                        else (
                            f"vision=sent:{flow_external_nav_state.last_reason}"
                            f" xy=({flow_external_nav_state.north_m:+.2f},"
                            f"{flow_external_nav_state.east_m:+.2f})m"
                            f" v=({flow_external_nav_state.last_vn_m_s:+.2f},"
                            f"{flow_external_nav_state.last_ve_m_s:+.2f})m/s"
                            if args.external_nav_from_flow
                            else "flow_nav=off"
                        )
                    ),
                    (
                        f"gps_primary={active_gps_primary + 1}"
                        if active_gps_primary is not None
                        else "gps_primary=unknown"
                    ),
                    (
                        f"qgc_udp=veh:{qgc_forwarder.vehicle_packets}"
                        f" qgc:{qgc_forwarder.qgc_packets}"
                        if qgc_forwarder is not None
                        else "qgc_udp=off"
                    ),
                    f"gps_fallback={'yes' if gps_fallback_active else 'no'}",
                    f"land_sent={'yes' if land_command_sent else 'no'}",
                    f"healthy_sends={healthy_flow_sends}",
                    f"mode={monitor.flight_mode}",
                    f"armed={'yes' if monitor.armed else 'no'}",
                    f"ekf_source={active_ekf_source_set or 'unknown'}",
                    f"flow_locked={'yes' if flow_source_locked else 'no'}",
                    f"correction={correction_state}",
                    f"roll={roll_deg:+.1f}deg",
                    f"pitch={pitch_deg:+.1f}deg",
                    f"speed_xy={speed_xy:.2f}m/s",
                ]
                if args.range_source == "external":
                    if last_range_age_s >= 0.0:
                        summary.append(f"range_age={last_range_age_s:.2f}s")
                    if monitor.rangefinder_sensor_id is not None:
                        summary.append(f"range_id={monitor.rangefinder_sensor_id}")
                if last_flow_health_state == "bad":
                    summary.append(f"flow_reason={last_flow_health_reason}")
                if monitor.local_position is not None:
                    summary.append(
                        f"cube_xy=({monitor.local_position.x:+.2f},{monitor.local_position.y:+.2f})m"
                    )
                if monitor.ekf_flags is not None:
                    summary.append(f"ekf_flags=0x{monitor.ekf_flags:x}")
                if qgc_forwarder is not None and qgc_forwarder.last_error:
                    summary.append(f"qgc_error={qgc_forwarder.last_error}")
                if monitor.status_text:
                    summary.append(f"status={monitor.status_text}")
                print(" | ".join(summary))
                next_status = now + status_period
    finally:
        if flow_csv_file is not None:
            flow_csv_file.close()
        pipeline.stop()


if __name__ == "__main__":
    main()
