#!/usr/bin/env bash
# Install only host USB reliability configuration. No ROS node or robot
# command path is started by this script.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
apply=false

usage() {
  cat <<'EOF'
Usage: sudo bash install_go2_host_config.sh --apply

Without --apply this prints the exact files that would be installed. It never
installs TinyNav, starts navigation, changes allow_goal, or sends a Go2 command.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --apply) apply=true; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

cat <<EOF
Host configuration plan:
  $SCRIPT_DIR/99-realsense-usb-power.rules
    -> /etc/udev/rules.d/99-realsense-usb-power.rules
  $SCRIPT_DIR/set_realsense_power.sh
    -> /usr/local/sbin/focus-set-realsense-power
  $SCRIPT_DIR/systemd/usbfs-memory-fix.service
    -> /etc/systemd/system/usbfs-memory-fix.service
  $SCRIPT_DIR/systemd/focus-realsense-power.service
    -> /etc/systemd/system/focus-realsense-power.service
EOF

if [[ "$apply" != true ]]; then
  echo "Dry run only. Re-run as root with --apply after reviewing the plan."
  exit 0
fi
if [[ "$(id -u)" -ne 0 ]]; then
  echo "--apply requires root (use sudo)." >&2
  exit 2
fi

install -D -m 0644 "$SCRIPT_DIR/99-realsense-usb-power.rules" \
  /etc/udev/rules.d/99-realsense-usb-power.rules
install -D -m 0755 "$SCRIPT_DIR/set_realsense_power.sh" \
  /usr/local/sbin/focus-set-realsense-power
install -D -m 0644 "$SCRIPT_DIR/systemd/usbfs-memory-fix.service" \
  /etc/systemd/system/usbfs-memory-fix.service
install -D -m 0644 "$SCRIPT_DIR/systemd/focus-realsense-power.service" \
  /etc/systemd/system/focus-realsense-power.service

systemctl daemon-reload
udevadm control --reload-rules
systemctl enable --now usbfs-memory-fix.service
systemctl enable --now focus-realsense-power.service

echo "Installed. A reboot is not required; verify with:"
echo "  bash $SCRIPT_DIR/verify_go2.sh --hardware"
