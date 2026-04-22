# Run:
#   python3 realsense_optical_flow_preview.py \
#     --flow-source infra1 \
#     --width 640 \
#     --height 480

import argparse
import time
from typing import List, Optional, Tuple

import cv2
import numpy as np
import pyrealsense2 as rs

from realsense_optical_flow_to_cube import (
    FlowTracker,
    clamp_int16,
    enable_streams,
    median_distance_m,
    prepare_flow_image,
    read_flow_frame,
)


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Preview RealSense RGB, depth, and optical-flow tracking live without "
            "sending anything to the Cube. Useful for checking what the drone would "
            "receive before flight tests."
        )
    )
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
        help="Preview the outgoing MAVLink payload at this rate in Hz.",
    )
    parser.add_argument(
        "--crop-fraction",
        type=float,
        default=0.6,
        help="Center crop fraction used before tracking.",
    )
    parser.add_argument(
        "--downscale",
        type=int,
        default=2,
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
        "--max-depth-m",
        type=float,
        default=4.0,
        help="Clip depth frames to this range before normalizing for tracking.",
    )
    parser.add_argument("--max-features", type=int, default=160)
    parser.add_argument("--min-tracks", type=int, default=30)
    parser.add_argument("--lk-window", type=int, default=21)
    parser.add_argument("--lk-levels", type=int, default=3)
    parser.add_argument(
        "--range-window",
        type=int,
        default=5,
        help="Center window size in pixels for median depth sampling.",
    )
    parser.add_argument(
        "--grid-cols",
        type=int,
        default=12,
        help="How many dot-matrix columns to draw in the visual overlays.",
    )
    parser.add_argument(
        "--grid-rows",
        type=int,
        default=9,
        help="How many dot-matrix rows to draw in the visual overlays.",
    )
    parser.add_argument(
        "--display-width",
        type=int,
        default=640,
        help="Display width for each video panel.",
    )
    parser.add_argument(
        "--quality-preview-min",
        type=int,
        default=80,
        help="Minimum flow quality before the correction hint becomes actionable.",
    )
    return parser.parse_args()


def build_depth_colormap(depth_frame, max_depth_m: float) -> np.ndarray:
    depth_image = np.asanyarray(depth_frame.get_data())
    max_depth_mm = max(1, int(max_depth_m * 1000.0))
    depth_image = np.clip(depth_image, 0, max_depth_mm)
    depth_8u = cv2.convertScaleAbs(depth_image, alpha=255.0 / max_depth_mm)
    return cv2.applyColorMap(depth_8u, cv2.COLORMAP_JET)


def draw_dot_matrix(image: np.ndarray, cols: int, rows: int, color, radius: int):
    height, width = image.shape[:2]
    xs = np.linspace(width * 0.1, width * 0.9, max(cols, 2))
    ys = np.linspace(height * 0.1, height * 0.9, max(rows, 2))
    for x in xs:
        for y in ys:
            cv2.circle(image, (int(x), int(y)), radius, color, -1, lineType=cv2.LINE_AA)


def draw_center_marker(image: np.ndarray, text: str):
    height, width = image.shape[:2]
    center = (width // 2, height // 2)
    cv2.circle(image, center, 6, (0, 0, 255), -1, lineType=cv2.LINE_AA)
    cv2.putText(
        image,
        text,
        (20, 32),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )


def draw_tracks(
    image: np.ndarray,
    previous_points: Optional[np.ndarray],
    current_points: Optional[np.ndarray],
    flow_dx: float,
    flow_dy: float,
):
    if previous_points is None or current_points is None:
        return

    for previous, current in zip(previous_points, current_points):
        start = (int(previous[0]), int(previous[1]))
        end = (int(current[0]), int(current[1]))
        cv2.line(image, start, end, (0, 180, 255), 1, cv2.LINE_AA)
        cv2.circle(image, end, 2, (0, 255, 0), -1, cv2.LINE_AA)

    height, width = image.shape[:2]
    center = (width // 2, height // 2)
    arrow_tip = (
        int(center[0] + flow_dx * 8.0),
        int(center[1] + flow_dy * 8.0),
    )
    cv2.arrowedLine(
        image,
        center,
        arrow_tip,
        (0, 0, 255),
        2,
        cv2.LINE_AA,
        tipLength=0.25,
    )


def resize_panel(image: np.ndarray, display_width: int) -> np.ndarray:
    height, width = image.shape[:2]
    scale = display_width / float(width)
    display_height = max(120, int(round(height * scale)))
    return cv2.resize(image, (display_width, display_height), interpolation=cv2.INTER_AREA)


def make_payload(flow_dx: float, flow_dy: float, quality: int, distance_m: float):
    return {
        "flow_x": clamp_int16(int(round(flow_dx * 10.0))),
        "flow_y": clamp_int16(int(round(flow_dy * 10.0))),
        "quality": quality,
        "ground_distance_m": distance_m,
        "distance_sensor_cm": max(0, int(round(distance_m * 100.0))),
    }


def correction_hint(payload, quality_preview_min: int) -> str:
    if payload["quality"] < quality_preview_min:
        return "Waiting for stronger optical-flow quality"

    hints: List[str] = []
    if payload["flow_x"] > 3:
        hints.append("counter image drift in -X")
    elif payload["flow_x"] < -3:
        hints.append("counter image drift in +X")

    if payload["flow_y"] > 3:
        hints.append("counter image drift in -Y")
    elif payload["flow_y"] < -3:
        hints.append("counter image drift in +Y")

    if not hints:
        return "Ground image looks steady"
    return ", ".join(hints)


def make_info_panel(
    width: int,
    height: int,
    payload,
    flow_dx: float,
    flow_dy: float,
    tracks: int,
    correction_text: str,
    packet_age_ms: float,
) -> np.ndarray:
    panel = np.full((height, width, 3), 16, dtype=np.uint8)
    title_color = (90, 220, 255)
    text_color = (230, 230, 230)
    subdued = (160, 160, 160)

    lines = [
        ("Drone Input Preview", title_color, 0.9),
        ("OPTICAL_FLOW", title_color, 0.7),
        (f"flow_x={payload['flow_x']:+d}", text_color, 0.65),
        (f"flow_y={payload['flow_y']:+d}", text_color, 0.65),
        (f"quality={payload['quality']}", text_color, 0.65),
        (f"ground_distance={payload['ground_distance_m']:.2f} m", text_color, 0.65),
        ("DISTANCE_SENSOR", title_color, 0.7),
        (f"current_distance={payload['distance_sensor_cm']} cm", text_color, 0.65),
        ("Tracking", title_color, 0.7),
        (f"median_dx={flow_dx:+.2f} px", text_color, 0.65),
        (f"median_dy={flow_dy:+.2f} px", text_color, 0.65),
        (f"tracks={tracks}", text_color, 0.65),
        (f"packet_age={packet_age_ms:.0f} ms", subdued, 0.6),
        ("Hold Preview", title_color, 0.7),
        (correction_text, text_color, 0.6),
        ("Keys: q quit | s snapshot", subdued, 0.55),
    ]

    y = 30
    for text, color, scale in lines:
        cv2.putText(
            panel,
            text,
            (20, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            scale,
            color,
            2 if scale >= 0.7 else 1,
            cv2.LINE_AA,
        )
        y += 28 if scale >= 0.7 else 24

    return panel


def save_snapshot(canvas: np.ndarray):
    stamp = time.strftime("%Y%m%d_%H%M%S")
    path = f"flow_preview_{stamp}.png"
    cv2.imwrite(path, canvas)
    print(f"Saved snapshot to {path}")


def main():
    args = parse_args()
    if args.send_rate <= 0:
        raise SystemExit("--send-rate must be > 0")

    pipeline = rs.pipeline()
    config = rs.config()
    enable_streams(config, args)
    config.enable_stream(
        rs.stream.color,
        args.width,
        args.height,
        rs.format.bgr8,
        args.fps,
    )
    pipeline.start(config)
    align_to_color = rs.align(rs.stream.color)

    tracker = FlowTracker(
        max_features=args.max_features,
        min_tracks=args.min_tracks,
        lk_window=args.lk_window,
        lk_levels=args.lk_levels,
    )

    send_period = 1.0 / args.send_rate
    next_send = time.time()
    last_payload = make_payload(0.0, 0.0, 0, -1.0)
    last_packet_time = time.time()
    last_flow_dx = 0.0
    last_flow_dy = 0.0
    last_tracks = 0

    print(
        "Previewing RealSense optical flow:"
        f" flow_source={args.flow_source}"
        f" send_rate={args.send_rate:.1f}Hz"
        " | press q to quit, s to save a snapshot"
    )

    try:
        while True:
            frames = pipeline.wait_for_frames(timeout_ms=5000)
            aligned_frames = align_to_color.process(frames)
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()
            flow_frame = depth_frame if args.flow_source == "depth" else read_flow_frame(frames, args.flow_source)

            if not color_frame or not depth_frame or not flow_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data()).copy()
            depth_colormap = build_depth_colormap(depth_frame, args.max_depth_m)
            processed_flow = prepare_flow_image(flow_frame, args)
            tracking_view = cv2.cvtColor(processed_flow, cv2.COLOR_GRAY2BGR)

            distance_m = median_distance_m(depth_frame, args.range_window)
            flow = tracker.update(processed_flow)

            if flow is not None and time.time() >= next_send:
                last_flow_dx = flow.dx_px
                last_flow_dy = flow.dy_px
                last_tracks = flow.tracks
                last_payload = make_payload(
                    flow.dx_px,
                    flow.dy_px,
                    flow.quality,
                    distance_m,
                )
                last_packet_time = time.time()
                next_send = last_packet_time + send_period

            draw_dot_matrix(color_image, args.grid_cols, args.grid_rows, (255, 220, 40), 2)
            draw_dot_matrix(depth_colormap, args.grid_cols, args.grid_rows, (255, 255, 255), 2)
            draw_dot_matrix(tracking_view, args.grid_cols, args.grid_rows, (255, 160, 0), 1)
            draw_center_marker(color_image, f"RGB center depth {distance_m:.2f} m")
            draw_center_marker(depth_colormap, "Depth map")
            draw_tracks(
                tracking_view,
                tracker.last_previous_points,
                tracker.last_current_points,
                last_flow_dx,
                last_flow_dy,
            )
            draw_center_marker(
                tracking_view,
                f"Tracking dx={last_flow_dx:+.2f}px dy={last_flow_dy:+.2f}px",
            )

            rgb_panel = resize_panel(color_image, args.display_width)
            depth_panel = resize_panel(depth_colormap, args.display_width)
            tracking_panel = resize_panel(tracking_view, args.display_width)
            info_panel = make_info_panel(
                args.display_width,
                tracking_panel.shape[0],
                last_payload,
                last_flow_dx,
                last_flow_dy,
                last_tracks,
                correction_hint(last_payload, args.quality_preview_min),
                (time.time() - last_packet_time) * 1000.0,
            )

            top_row = cv2.hconcat([rgb_panel, depth_panel])
            bottom_row = cv2.hconcat([tracking_panel, info_panel])
            canvas = cv2.vconcat([top_row, bottom_row])

            cv2.imshow("RealSense Optical Flow Preview", canvas)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord("s"):
                save_snapshot(canvas)
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
