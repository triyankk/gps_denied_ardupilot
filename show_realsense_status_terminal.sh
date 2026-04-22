#!/usr/bin/env bash
# Run:
#   /home/atas/vscode/intellisense_cam/show_realsense_status_terminal.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SERVICE_NAME="realsense_optical_flow_to_cube.service"
LOG_FILE="${SCRIPT_DIR}/realsense_optical_flow_to_cube.log"
TITLE="Jetson No-GPS Bridge Status"
LOCK_FILE="/tmp/realsense_status_terminal.lock"

inside_terminal=0
if [[ "${1:-}" == "--inside-terminal" ]]; then
  inside_terminal=1
fi

shell_quote() {
  printf "%q" "$1"
}

run_inside_terminal() {
  exec 9>"${LOCK_FILE}"
  if ! flock -n 9; then
    echo "A Jetson no-GPS status terminal is already running."
    sleep 3
    return 0
  fi

  printf '\033]0;%s\007' "${TITLE}"
  clear || true
  echo "============================================================"
  echo " ${TITLE}"
  echo "============================================================"
  echo "Started: $(date)"
  echo
  echo "Meaning of beep sequences:"
  echo "  3 short beeps = script initiated"
  echo "  2 long beeps  = GPS home/origin locked"
  echo "  2 short beeps = optical-flow bridge started"
  echo
  echo "Service status:"
  systemctl status --no-pager --lines=12 "${SERVICE_NAME}" || true
  echo
  echo "Live updates from ${LOG_FILE}"
  echo "Press Ctrl+C to close this window."
  echo "============================================================"
  echo

  mkdir -p "$(dirname "${LOG_FILE}")"
  touch "${LOG_FILE}" 2>/dev/null || true
  tail -n 200 -F "${LOG_FILE}"
}

open_terminal_window() {
  local command
  command="$(shell_quote "${BASH_SOURCE[0]}") --inside-terminal"

  if command -v gnome-terminal >/dev/null 2>&1; then
    exec gnome-terminal --full-screen --title="${TITLE}" -- bash -lc "${command}"
  fi

  if command -v mate-terminal >/dev/null 2>&1; then
    exec mate-terminal --full-screen --title="${TITLE}" -- bash -lc "${command}"
  fi

  if command -v xfce4-terminal >/dev/null 2>&1; then
    exec xfce4-terminal --fullscreen --title="${TITLE}" --command "bash -lc '${command}'"
  fi

  if command -v konsole >/dev/null 2>&1; then
    exec konsole --fullscreen -p tabtitle="${TITLE}" -e bash -lc "${command}"
  fi

  if command -v lxterminal >/dev/null 2>&1; then
    exec lxterminal --title="${TITLE}" -e bash -lc "${command}"
  fi

  if command -v xterm >/dev/null 2>&1; then
    exec xterm -fullscreen -T "${TITLE}" -e bash -lc "${command}"
  fi

  if command -v x-terminal-emulator >/dev/null 2>&1; then
    exec x-terminal-emulator -e bash -lc "${command}"
  fi

  echo "No supported terminal emulator found."
  echo "Install gnome-terminal, mate-terminal, xfce4-terminal, konsole, lxterminal, or xterm."
  exit 1
}

if [[ "${inside_terminal}" -eq 1 ]]; then
  run_inside_terminal
else
  open_terminal_window
fi
