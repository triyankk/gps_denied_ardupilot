# GPS-Denied PosHold RealSense Flow Bridge

This workspace is trimmed down to the active flight path plus a few safe diagnostics:

- `realsense_optical_flow_to_cube.py`: active boot/flight bridge for RealSense optical flow, Cube lidar range, one-time home/origin startup, experimental flow-derived external navigation for GPS-denied `PosHold`, and companion-side optical-flow logging.
- `install_realsense_autostart.sh`: installs the boot service that starts the bridge automatically after Jetson boot.
- `systemd/realsense_optical_flow_to_cube.service.template`: service template used by the installer.
- `gps_denied/mavlink_helpers.py`: shared MAVLink connection, parameter, source-set, status, tune, and home/origin helpers.
- `ardupilot_lua/jetson_nogps_status.lua`: optional FC-side Lua relay for clearer GCS status messages.
- `realsense_optical_flow_preview.py`: camera-only preview for RGB/depth/tracking checks without talking to the Cube.
- `scan_mavlink_ports.py`: diagnostic tool for finding the Cube MAVLink port/baud.
- `external_pose_to_cube.py`: MAVLink external-vision pose bridge for a real VIO/SLAM source or a bench/recovery synthetic pose demo.
- `show_realsense_status_terminal.sh`: optional fullscreen status terminal for the service log.
- `install_x11vnc_boot_service.sh` and `systemd/x11vnc.service`: optional VNC boot-service helpers.

## Which Script Should You Run?

For the prop-less `PosHold` demo, run:

```bash
python3 external_pose_to_cube.py \
  --demo hold \
  --ports /dev/ttyACM0 /dev/ttyACM1 /dev/ttyTHS1 \
  --origin-lat 12.9715987 \
  --origin-lon 77.5945627 \
  --origin-alt 900
```

That script is retained for bench/recovery testing only.

With the Cube already configured for GPS-denied external vision, running this command should clear the external-navigation pre-arm messages and leave `PosHold` ready to arm.

For the downward-camera optical-flow path, run:

```bash
python3 realsense_optical_flow_to_cube.py \
  --ports /dev/ttyACM1 /dev/ttyACM0
```

That script sends `OPTICAL_FLOW_RAD` from a downward-facing RealSense feed and, by default, uses the Cube's own down-facing lidar `DISTANCE_SENSOR` telemetry as the height/range scale.

For the boot-service workflow, QGC gets the first minute after power-up to connect and download vehicle data. After that, the Jetson bridge tries `/dev/ttyACM1` first and then `/dev/ttyACM0`. The script skips any candidate port that is already in use by another local process, so if QGC is using ACM0 on the Jetson, the bridge will not fight it.

You can scan the candidate USB ports manually with:

```bash
python3 scan_mavlink_ports.py \
  --ports /dev/ttyACM1 /dev/ttyACM0 \
  --bauds 115200 921600 57600
```

Use the `FOUND: port=... baud=...` result for manual testing. If QGC already owns ACM0, stop QGC or let the boot service skip that port.

If you omit `--origin-lat`, `--origin-lon`, and `--origin-alt`, the script waits indefinitely by default for real home data from the Cube. It accepts the Cube's existing `HOME_POSITION` if home is already set, otherwise it waits for a one-time GPS fix, copies that real latitude/longitude/altitude into EKF origin/home, and only then starts the optical-flow bridge. The boot-service path then keeps normal modes on source set `1` and lets only `PosHold` switch to source set `2` after flow/range/GPS_INPUT health is good:

- startup/normal lane defaults to EKF source set `1` with GPS horizontal position/velocity, baro vertical position, GPS vertical velocity, and compass yaw
- no-GPS `PosHold` lane defaults to EKF source set `2` with MAVLink `GPS_INPUT` horizontal position/velocity generated from flow odometry, baro vertical position, and compass yaw
- `FLOW_TYPE=5`
- `GPS2_TYPE=14`, `GPS_AUTO_SWITCH=0`, `GPS_PRIMARY=0`, and `VISO_TYPE=0` so the VisualOdom backend is not loaded
- `EK3_SRC_OPTIONS=0`
- `EK3_OGN_HGT_MASK=0`

After home/origin is set, the default behavior is mode-based source switching: `PosHold` can use no-GPS source set `2` after healthy flow/range/GPS_INPUT samples; `Loiter`, `Land`, `RTL`, `AltHold`, `Stabilize`, and other modes stay on startup/GPS source set `1`.

The RealSense flow output is filtered and smoothed before it is sent to the Cube. The defaults are conservative for the latest flight log: `--flow-message rad`, `--flow-max-rate-rad-s 0.8`, `--lk-fb-max-error-px 1.5`, `--ransac-reproj-threshold-px 3`, `--flow-smoothing-alpha 0.45`, `--flow-max-frame-px 16`, `--flow-max-step-px 12`, and `--flow-deadband-px 0.03`. The runtime log prints `raw_dpix` plus the sent angular `flow_rate`; if `flow_limited=yes` appears constantly during a steady hover, the clamp is too tight, but occasional `flow_limited=yes` is expected when rejecting spikes.

The companion also writes every optical-flow sample to `realsense_optical_flow_to_cube_of.csv`. Use this CSV for flow diagnosis because ArduPilot `.bin` logs may not include raw incoming `OPTICAL_FLOW_RAD` records even when the companion is sending them.

An inertial plausibility gate is enabled by default. It does not replace ArduPilot's EKF; it only suppresses optical-flow samples if the camera suddenly implies a large horizontal speed while the IMU-derived attitude is nearly level. The defaults are `--inertial-flow-gate-speed-m-s 1.2`, `--inertial-flow-gate-lean-deg 4.0`, and `--inertial-flow-gate-accel-m-s2 0.7`. If the runtime log shows `inertial_gate=reject`, the CSV will mark optical flow unhealthy and `PosHold` will not switch to the no-GPS lane.

It also stops requesting GPS telemetry from the companion side after home lock, sends `STATUSTEXT` notifications for source changes and failsafe states, plays `3 short beeps` when the Jetson script has initiated successfully after boot delay, plays `2 long beeps` when GPS home/origin is locked successfully, and plays `2 short beeps` when the flow bridge has started successfully if `PLAY_TUNE` is available on the link.

The flow-derived `GPS_INPUT` mode is enabled by default because your flight controller reports `PreArm: VisOdom: out of memory` when `VISO_TYPE=3` is used. It is not true VIO/SLAM; it is an experimental visual-odometry-lite bridge that integrates optical flow plus lidar height into a local GPS-like position. It can drift over time, but it avoids the memory-heavy VisualOdom backend and fixes the previous velocity-only problem where `EK3_SRC2_POSXY=0` left `PosHold` without a horizontal position source.

The bridge owns the Cube serial port while it is running, so QGC should not try to open the same `/dev/ttyACM*` port directly at the same time. The script now includes a lightweight MAVLink UDP pass-through for QGC:

- local QGC on the Jetson can listen on the usual UDP `14550`
- remote QGC can connect a UDP link to the Jetson IP on port `14555`
- if you need direct USB/QGC access for parameter setup, stop the service first with `sudo systemctl stop realsense_optical_flow_to_cube.service`

If the script has to change a reboot-required parameter such as `FLOW_TYPE` or `EK3_OGN_HGT_MASK`, it will stop and require a flight-controller reboot before the transition can be trusted.

On connection it now also sends `Jetson Nano detected running no GPS script`, then reports the current phase to the GCS:

- `GPS assist active, locking home position`
- `Waiting for GPS home before flow start`
- `Trying to get home position from GPS`
- `GPS status fix=... sats=...`
- `Origin locked from GPS`
- `RealSense flow ready for PosHold source switching`
- `No-GPS source set 2 active`
- `GPS-Less flight active` every 6 seconds while `PosHold` is actually using the no-GPS flow lane
- `Flow unhealthy: switching back to GPS`
- `GPS fallback active`
- `GPS source set 1 active`

If you want those status updates to appear more reliably in the telemetry-linked GCS message pane, install the FC-side Lua relay too:

```bash
# Upload this file to the flight controller SD card as:
#   APM/scripts/jetson_nogps_status.lua
/home/atas/vscode/intellisense_cam/ardupilot_lua/jetson_nogps_status.lua
```

The Lua relay watches `SCR_USER1` and `SCR_USER2`, then re-emits Jetson startup and handoff states through the flight controller with `gcs:send_text(...)`. Before uploading it, make sure:

- `SCR_ENABLE=1`
- the flight controller has an SD card with an `APM/scripts/` folder
- the file is uploaded as `APM/scripts/jetson_nogps_status.lua`

After a reboot, you should see `Jetson no-GPS relay Lua loaded` from the FC, followed by the companion-driven state messages such as:

- `Jetson no-GPS bridge detected`
- `GPS assist active, locking home`
- `Home/origin locked from GPS`
- `No-GPS flow bridge started`
- `No-GPS EKF source set 2 active`

If you provide `--origin-*` manually, it skips GPS assist and reports `Manual origin set; optical-flow source will lock after flow starts`.

To install, enable, and start the boot service automatically, run:

```bash
sudo bash /home/atas/vscode/intellisense_cam/install_realsense_autostart.sh
```

That enables and starts the `systemd` service, so the bridge starts automatically every time the Jetson boots with the drone.

By default, the installer does not add a desktop autostart terminal, so it will not open a fullscreen window when a VNC desktop session starts.

To open the status terminal manually:

```bash
/home/atas/vscode/intellisense_cam/show_realsense_status_terminal.sh
```

To install the bridge service and opt into the desktop status terminal:

```bash
sudo bash /home/atas/vscode/intellisense_cam/install_realsense_autostart.sh --status-terminal
```

To install the service file without enabling or starting it, use:

```bash
sudo bash /home/atas/vscode/intellisense_cam/install_realsense_autostart.sh --install-only
```

To reinstall VNC as a background boot service, run:

```bash
sudo bash /home/atas/vscode/intellisense_cam/install_x11vnc_boot_service.sh
```

That installer backs up the current `x11vnc.service`, installs `systemd/x11vnc.service`, enables it for `graphical.target`, restarts it, and disables any duplicate desktop `x11vnc.desktop` autostart entry.

The boot service waits `60 seconds` before launching the script so QGC has time to connect and download vehicle data first. After the wait, the Jetson tries `/dev/ttyACM1` and then `/dev/ttyACM0`, using whichever available port provides a real Cube heartbeat. The installed service does not inject a fixed/manual home position. It waits for the Cube to report a real GPS home once, copies that real lat/lon/alt into EKF origin/home, and only then starts the RealSense optical-flow bridge.

The service command is intentionally tuned for the no-GPS PosHold path:

- real GPS home first, no fixed/manual origin in the boot command
- range source: external Cube down-facing lidar with `--range-source external --external-range-max-m 300`
- range sanity gate: no-GPS flow is rejected if selected range and EKF local altitude differ by more than `2.0m` above `1.5m` local altitude
- optical-flow message: `OPTICAL_FLOW_RAD` with RealSense intrinsics using `--flow-message rad --flow-max-rate-rad-s 0.8`
- GPS_INPUT bridge: `--gps-input-from-flow` integrates optical flow plus lidar height and sends MAVLink `GPS_INPUT` as GPS2, avoiding the VisualOdom backend
- mode behavior: only `PosHold` uses no-GPS source set `2`; `Loiter`, `Land`, `RTL`, and other modes are switched back to startup/GPS source set `1`
- height behavior: the rangefinder is mandatory for optical-flow scaling, but EKF vertical position stays on baro with `EK3_SRC*_POSZ=1`; this follows ArduPilot optical-flow guidance and avoids trusting bad range data as the aircraft altitude
- outdoor failsafe: GPS fallback and `Loiter` after `0.5s` bad PosHold optical-flow/range/EKF-variance health; automatic LAND is disabled by default with `--flow-failsafe-land-seconds 0`
- optical-flow envelope: no-GPS flow source only from `--flow-failsafe-min-height-m 0.6` to `--flow-failsafe-max-height-m 6.5`
- RealSense tracking crop/resolution: `--crop-fraction 0.8 --downscale 1`
- more tracking features: `--max-features 300 --min-tracks 50`
- optical-flow outlier rejection: `--lk-fb-max-error-px 1.5 --ransac-reproj-threshold-px 3`
- companion OF CSV: `--flow-csv-log __WORKDIR__/realsense_optical_flow_to_cube_of.csv`
- PosHold-only no-GPS EKF source switching after `30` consecutive healthy optical-flow/range/GPS_INPUT samples

If you want to test the script manually instead of through `systemd`, stop the boot service first and then run one of these:

```bash
sudo systemctl stop realsense_optical_flow_to_cube.service
```

```bash
python3 /home/atas/vscode/intellisense_cam/realsense_optical_flow_to_cube.py \
  --ports /dev/ttyACM1 /dev/ttyACM0
```

That command now waits indefinitely for GPS home/origin before starting the flow bridge.

For an indoor prop-less optical-flow health check, do not wait for GPS. Use health-test mode, which streams sensor data without setting home/origin and without changing EKF source sets:

```bash
python3 /home/atas/vscode/intellisense_cam/realsense_optical_flow_to_cube.py \
  --ports /dev/ttyACM1 /dev/ttyACM0 \
  --flow-health-test
```

In this mode, Mission Planner should stop showing `Bad OptFlow Health` once live RealSense flow messages are arriving. It is for prop-less/in-hand validation only, not for PosHold flight.

By default, only `PosHold` uses no-GPS source set `2`; all other modes use startup/GPS source set `1`. If your no-GPS lane is on a different set, change it with `--post-home-ekf-source-set`. If you only want to stream sensor data without commanding any EKF source change, use `--post-home-ekf-source-set 0 --disable-gps-input-from-flow`.

```bash
python3 /home/atas/vscode/intellisense_cam/realsense_optical_flow_to_cube.py \
  --ports /dev/ttyACM1 /dev/ttyACM0 \
  --post-home-ekf-source-set 0 \
  --disable-gps-input-from-flow
```

That last command is only for streaming RealSense data without letting the script command the no-GPS EKF source change. It still waits for real GPS home first unless you explicitly provide `--origin-*` for a bench-only/manual-origin test.

Useful boot-service check commands:

```bash
systemctl status --no-pager realsense_optical_flow_to_cube.service
```

```bash
journalctl -u realsense_optical_flow_to_cube.service -n 100 --no-pager
```

```bash
journalctl -u realsense_optical_flow_to_cube.service -f
```

```bash
tail -n 100 /home/atas/vscode/intellisense_cam/realsense_optical_flow_to_cube.log
```

```bash
tail -n 20 /home/atas/vscode/intellisense_cam/realsense_optical_flow_to_cube_of.csv
```

Useful boot-service control commands:

```bash
sudo systemctl restart realsense_optical_flow_to_cube.service
```

```bash
sudo systemctl stop realsense_optical_flow_to_cube.service
```

```bash
sudo systemctl start realsense_optical_flow_to_cube.service
```

```bash
sudo systemctl disable realsense_optical_flow_to_cube.service
```

Before flight, the minimum good signs are:

- the GCS shows `Jetson Nano detected running no GPS script`
- the GCS or log shows `No-GPS flow bridge started`
- the GCS or log shows `No-GPS source set 2 active`
- the runtime log shows `healthy_sends` increasing before the no-GPS source lock
- the GCS/log does not repeatedly alternate between `GPS source set 1 active` and `No-GPS source set 2 active`
- there is no `PreArm: AHRS: waiting for home`
- the runtime log does not stay stuck at `distance=unknown` if you are expecting optical-flow position hold

If you prefer a finite GPS wait for testing, you can still override it, for example:

```bash
python3 /home/atas/vscode/intellisense_cam/realsense_optical_flow_to_cube.py \
  --ports /dev/ttyACM1 /dev/ttyACM0 \
  --gps-home-timeout 60
```

## Workspace Layout

### Active files

- `realsense_optical_flow_to_cube.py`: active flight bridge.
- `gps_denied/mavlink_helpers.py`: MAVLink helpers used by the bridge and diagnostics.
- `install_realsense_autostart.sh`: installs/enables the boot service.
- `systemd/realsense_optical_flow_to_cube.service.template`: boot service template.
- `ardupilot_lua/jetson_nogps_status.lua`: optional FC-side GCS status relay.
- `realsense_optical_flow_preview.py`: camera-only preview and dot-tracking diagnostic.
- `scan_mavlink_ports.py`: Cube MAVLink port scanner.
- `external_pose_to_cube.py`: bench/recovery synthetic pose demo.
- `show_realsense_status_terminal.sh`: optional log-view terminal launcher.
- `install_x11vnc_boot_service.sh` and `systemd/x11vnc.service`: optional VNC boot helpers.

Removed clutter:

- `legacy/`
- `__pycache__/`
- old `.tlog` telemetry files
- saved `mav.parm`
- old standalone RealSense depth bridge
- old parameter-check helper

## Before The Prop-Less PosHold Demo

This is the clean sequence to follow before you arm in `PosHold` with no props.

### 1. Confirm the Cube is already configured

On the live Cube, confirm your indoor-navigation parameters are already set in Mission Planner or QGroundControl.

### 2. Remove props and power everything

- Remove all propellers.
- Power the Cube, Jetson, and connected sensors.
- Connect the Jetson to the Cube on the serial port you intend to use for MAVLink vision.
- Open Mission Planner or QGroundControl so you can watch messages and mode changes.

### 3. Start the external-vision demo script

Run:

```bash
python3 external_pose_to_cube.py \
  --demo hold \
  --ports /dev/ttyACM0 /dev/ttyACM1 /dev/ttyTHS1 \
  --origin-lat 12.9715987 \
  --origin-lon 77.5945627 \
  --origin-alt 900
```

This script sends a synthetic stationary pose to the Cube and sets the EKF origin/home from the supplied coordinates. On a correctly configured vehicle, that should clear the external-navigation pre-arm messages and leave `PosHold` ready to arm before you move on to real optical flow or VIO.

### 4. Watch for the correct signs

In the script output, look for:

- `vision_xyz=(...)`
- `cube_xy=(...)`
- `ekf_flags=...`
- no repeating `status=` warnings

In the ground station, look for:

- no external-navigation pre-arm errors
- `PosHold` can be selected and is ready to arm
- the EKF looks healthy

If you see `EKF compass variance`, your yaw estimate is still being rejected by the magnetometer path. For this bench demo, first check for local magnetic interference, then recalibrate or reduce compass dependence before expecting `PosHold` to arm cleanly.

ArduPilot's official external-navigation setup also expects the live flight controller to be configured roughly like this before MAVLink vision data will fuse cleanly:

- `VISO_TYPE=3`
- `EK3_SRC1_POSXY=6`
- `EK3_SRC1_VELXY=6` or `0`
- `EK3_SRC1_POSZ=1` or `6`
- `EK3_SRC1_VELZ=6` or `0`
- `EK3_SRC1_YAW=6` only if your companion is sending a real yaw estimate, otherwise `1`

The synthetic `--demo hold` and `--demo circle` modes are useful for exercising position/velocity inputs, but they do not create a real heading sensor. If your live setup still has `EKF compass variance`, changing the demo script alone will not clear that warning.

### 5. Arm in PosHold with no props

Only after the above looks clean:

- switch to `PosHold`
- arm the vehicle
- leave it stationary on the bench
- verify no new EKF or navigation warnings appear

This is the full prop-less demo. At this stage you are only proving estimator acceptance and mode readiness, not real hover performance.

## What Comes After The Prop-Less Demo

Once the bench demo is clean, the next real step is to replace `--demo hold` with a real motion source from the Jetson:

- `--gps-input-from-flow` from the downward-facing RealSense optical flow plus lidar height
- a real VIO/SLAM output converted to external vision

Your 3D lidar should stay dedicated to obstacle detection. The down-facing lidar/rangefinder remains the altitude scale for optical flow.

## RealSense Optical Flow

For the new downward-facing optical-flow bridge, run:

```bash
python3 realsense_optical_flow_to_cube.py \
  --ports /dev/ttyACM1 /dev/ttyACM0
```

Notes:

- It uses the RealSense infrared stream by default for optical-flow tracking.
- By default, it uses the Cube's external down-facing lidar `DISTANCE_SENSOR` data for range scaling: `--range-source external --external-range-max-m 300`.
- It also checks selected range against EKF local altitude. If the range says something like `0.8m` while EKF altitude is around `4m`, the no-GPS flow lane is rejected and the vehicle falls back to GPS/Loiter.
- If you want to use RealSense depth as the range source instead, run with `--range-source realsense`. That is for low-altitude testing, not a 300 m outdoor range.
- It sends `OPTICAL_FLOW_RAD` over the MAVLink link to the Cube, converting tracked pixels into radians using RealSense intrinsics. With the default `--gps-input-from-flow`, it also sends flow-derived `GPS_INPUT` as GPS2.
- Outdoor safety is enabled by default: only `PosHold` uses no-GPS source set `2`; all other modes switch to startup/GPS source set `1`. If flow quality/tracks/range or live EKF variance become unhealthy for `0.5s` in PosHold, the script switches back to source set `1` and commands `Loiter`.
- Automatic LAND from the companion failsafe is disabled by default because it can be too aggressive. If you want it later, set `--flow-failsafe-land-seconds 6` or another deliberate value.
- The default no-GPS optical-flow envelope is `0.6m` to `6.5m`. Below `0.6m`, GPS is used because image motion near the floor is too large/noisy; above `6.5m`, GPS is used because the current outdoor setup has not been validated beyond that height.
- If `--origin-*` is omitted, it will try to seed EKF origin/home from the Cube's GPS once at startup, then stop requesting GPS telemetry from the companion side.
- If the top of the camera image does not point toward vehicle front, rotate the processed image with `--rotate 90`, `180`, or `270`.
- If infrared texture is poor on your surface, try `--flow-source depth`.

Before the first powered hover test, let the script configure the live flight controller or verify the same source-set shape manually:

- `FLOW_TYPE=5`
- `GPS2_TYPE=14`
- `GPS_AUTO_SWITCH=0`
- `GPS_PRIMARY=0`
- `VISO_TYPE=0`
- `EK3_SRC_OPTIONS=0`
- `EK3_SRC1_POSXY=3`
- `EK3_SRC1_VELXY=3`
- `EK3_SRC1_POSZ=1`
- `EK3_SRC1_VELZ=3`
- `EK3_SRC1_YAW=1`
- `EK3_SRC2_POSXY=3`
- `EK3_SRC2_VELXY=3`
- `EK3_SRC2_POSZ=1`
- `EK3_SRC2_VELZ=0`
- `EK3_SRC2_YAW=1`

If you get `PreArm: VisOdom: out of memory`, the vehicle is still trying to load VisualOdom. Re-check that `VISO_TYPE=0` after the script applies parameters, then reboot the flight controller. The GPS_INPUT path does not need VisualOdom.

For first flight validation, ArduPilot recommends checking the sensor first in `Stabilize` or `AltHold` and only then moving into a position-hold mode once the logs show sane flow and range behavior.

## RealSense Preview

If you want a safe camera-only preview before talking to the Cube, run:

```bash
python3 realsense_optical_flow_preview.py \
  --flow-source infra1 \
  --width 640 \
  --height 480
```

This opens a live 2x2 view with:

- RGB footage
- depth colormap
- tracked-dot optical-flow view with motion arrows
- a payload panel showing the `OPTICAL_FLOW` and `DISTANCE_SENSOR` values the flight controller would receive

Press `q` to quit or `s` to save a snapshot.

## Synthetic Circle Demo

If you want the synthetic position to move in a circle while keeping heading fixed, run:

```bash
python3 external_pose_to_cube.py \
  --demo circle \
  --demo-radius 0.4 \
  --demo-speed 0.2
```

That now keeps yaw fixed at `0` degrees by default. If you want the synthetic heading to rotate with the motion direction instead, add:

```bash
--demo-yaw-mode velocity
```
