#!/usr/bin/env bash
# Recover the read-only Odin1 driver before a new board calibration epoch.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UNIT="focus-yunji-odin1-driver.service"
VERIFY="$SCRIPT_DIR/verify_odin1.sh"

[[ -x "$VERIFY" ]] || {
  echo "Missing executable Odin verifier: $VERIFY" >&2
  exit 1
}
lsusb -d 2207:0019 >/dev/null 2>&1 || {
  echo "Odin1 USB device 2207:0019 is not connected." >&2
  exit 1
}

# The driver contains no WATER or robot-motion endpoint. Enabling it here
# makes a subsequent NUC reboot restore only the sensor/SLAM data plane.
if ! systemctl is-enabled --quiet "$UNIT"; then
  sudo -n systemctl enable "$UNIT" >/dev/null
fi
if ! systemctl is-active --quiet "$UNIT"; then
  sudo -n systemctl reset-failed "$UNIT" >/dev/null 2>&1 || true
  sudo -n systemctl start "$UNIT"
fi

deadline=$((SECONDS + 45))
while ! systemctl is-active --quiet "$UNIT"; do
  if (( SECONDS >= deadline )); then
    systemctl status "$UNIT" --no-pager -l >&2 || true
    journalctl -u "$UNIT" -n 120 --no-pager >&2 || true
    exit 1
  fi
  sleep 1
done

verification_log="$(mktemp)"
trap 'unlink "$verification_log" >/dev/null 2>&1 || true' EXIT
for attempt in 1 2 3; do
  if bash "$VERIFY" --hardware >"$verification_log" 2>&1; then
    cat "$verification_log"
    echo "Yunji Odin1 calibration driver ready; no robot command was issued."
    exit 0
  fi
  if (( attempt < 3 )); then
    sleep 3
  fi
done

cat "$verification_log" >&2
echo "Odin1 stayed active but its required live topics were not ready." >&2
exit 1
