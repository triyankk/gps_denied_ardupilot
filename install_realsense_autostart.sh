#!/usr/bin/env bash
# Run:
#   sudo bash /home/atas/vscode/intellisense_cam/install_realsense_autostart.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TEMPLATE_PATH="${SCRIPT_DIR}/systemd/realsense_optical_flow_to_cube.service.template"
SERVICE_NAME="realsense_optical_flow_to_cube.service"
SERVICE_PATH="/etc/systemd/system/${SERVICE_NAME}"
TMP_SERVICE="$(mktemp)"
RUN_USER="${SUDO_USER:-$(id -un)}"
ENABLE_NOW=1
STATUS_TERMINAL=0

while [[ "$#" -gt 0 ]]; do
  arg="$1"
  case "${arg}" in
    --enable-now)
      ENABLE_NOW=1
      shift
      ;;
    --install-only)
      ENABLE_NOW=0
      shift
      ;;
    --status-terminal)
      STATUS_TERMINAL=1
      shift
      ;;
    --no-status-terminal)
      STATUS_TERMINAL=0
      shift
      ;;
    *)
      echo "Unknown argument: ${arg}" >&2
      echo "Usage: sudo bash ${0} [--enable-now|--install-only] [--status-terminal|--no-status-terminal]" >&2
      exit 2
      ;;
  esac
done

if [[ ! -f "${TEMPLATE_PATH}" ]]; then
  echo "Missing template: ${TEMPLATE_PATH}" >&2
  exit 1
fi

install_status_terminal_autostart() {
  local user_home
  local user_group
  local autostart_dir
  local desktop_file

  user_home="$(getent passwd "${RUN_USER}" | cut -d: -f6)"
  user_group="$(id -gn "${RUN_USER}")"
  autostart_dir="${user_home}/.config/autostart"
  desktop_file="${autostart_dir}/realsense-status-terminal.desktop"

  install -d -m 0755 -o "${RUN_USER}" -g "${user_group}" "${autostart_dir}"

  cat > "${desktop_file}" <<EOF
[Desktop Entry]
Type=Application
Name=Jetson No-GPS Bridge Status
Comment=Open fullscreen terminal with RealSense no-GPS bridge events
Exec=${SCRIPT_DIR}/show_realsense_status_terminal.sh
Terminal=false
X-GNOME-Autostart-enabled=true
EOF
  chown "${RUN_USER}:${user_group}" "${desktop_file}"
  chmod 0644 "${desktop_file}"
  echo "Installed desktop status terminal autostart: ${desktop_file}"
}

sed \
  -e "s|__USER__|${RUN_USER}|g" \
  -e "s|__WORKDIR__|${SCRIPT_DIR}|g" \
  "${TEMPLATE_PATH}" > "${TMP_SERVICE}"

install -m 0644 "${TMP_SERVICE}" "${SERVICE_PATH}"
rm -f "${TMP_SERVICE}"

systemctl daemon-reload
chmod 0755 "${SCRIPT_DIR}/show_realsense_status_terminal.sh"
if [[ "${STATUS_TERMINAL}" -eq 1 ]]; then
  install_status_terminal_autostart
else
  echo "Skipped desktop status terminal autostart."
fi
if [[ "${ENABLE_NOW}" -eq 1 ]]; then
  systemctl enable "${SERVICE_NAME}"
  systemctl restart "${SERVICE_NAME}"
  echo "Installed, enabled, and started ${SERVICE_NAME}"
  echo "The service waits 60 seconds, then tries /dev/ttyACM1 and /dev/ttyACM0."
  echo "It waits for real GPS home once, then starts optical flow and locks no-GPS source set 2."
else
  systemctl disable "${SERVICE_NAME}" >/dev/null 2>&1 || true
  systemctl stop "${SERVICE_NAME}" >/dev/null 2>&1 || true
  echo "Installed ${SERVICE_NAME} but left it disabled/stopped."
  echo "Run again without --install-only to enable automatic startup."
fi
echo "Check status with: systemctl status ${SERVICE_NAME}"
echo "Watch logs with: journalctl -u ${SERVICE_NAME} -f"
