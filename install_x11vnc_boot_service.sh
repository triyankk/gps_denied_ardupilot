#!/usr/bin/env bash
# Run:
#   sudo bash /home/atas/vscode/intellisense_cam/install_x11vnc_boot_service.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SERVICE_NAME="x11vnc.service"
SOURCE_PATH="${SCRIPT_DIR}/systemd/${SERVICE_NAME}"
SERVICE_PATH="/etc/systemd/system/${SERVICE_NAME}"
RUN_USER="${SUDO_USER:-$(id -un)}"

if [[ "${EUID}" -ne 0 ]]; then
  echo "Run this installer with sudo." >&2
  exit 1
fi

if [[ ! -f "${SOURCE_PATH}" ]]; then
  echo "Missing service file: ${SOURCE_PATH}" >&2
  exit 1
fi

if [[ -f "${SERVICE_PATH}" ]]; then
  BACKUP_PATH="${SERVICE_PATH}.bak.$(date +%Y%m%d-%H%M%S)"
  cp -a "${SERVICE_PATH}" "${BACKUP_PATH}"
  echo "Backed up current unit: ${BACKUP_PATH}"
fi

install -m 0644 "${SOURCE_PATH}" "${SERVICE_PATH}"
systemctl daemon-reload
systemctl enable "${SERVICE_NAME}"
systemctl restart "${SERVICE_NAME}"

USER_HOME="$(getent passwd "${RUN_USER}" | cut -d: -f6)"
DESKTOP_AUTOSTART="${USER_HOME}/.config/autostart/x11vnc.desktop"
if [[ -f "${DESKTOP_AUTOSTART}" ]]; then
  mv -n "${DESKTOP_AUTOSTART}" "${DESKTOP_AUTOSTART}.disabled"
  echo "Disabled duplicate desktop autostart: ${DESKTOP_AUTOSTART}.disabled"
fi

echo "Installed and restarted ${SERVICE_NAME}"
systemctl status --no-pager --lines=20 "${SERVICE_NAME}"
