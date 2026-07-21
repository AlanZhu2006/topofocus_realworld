#!/usr/bin/env bash
# Set USB runtime power policy only for the verified D435i and its USB3 hub.
set -euo pipefail

if [[ "$(id -u)" -ne 0 ]]; then
  echo "Run as root; this script writes matching USB sysfs power controls." >&2
  exit 2
fi

matched=0
for device in /sys/bus/usb/devices/*; do
  [[ -r "$device/idVendor" && -r "$device/idProduct" ]] || continue
  vendor="$(<"$device/idVendor")"
  product="$(<"$device/idProduct")"
  case "$vendor:$product" in
    8086:0b3a|05e3:0625)
      [[ -w "$device/power/control" ]] || {
        echo "Not writable: $device/power/control" >&2
        exit 1
      }
      printf 'on\n' > "$device/power/control"
      actual="$(<"$device/power/control")"
      [[ "$actual" == "on" ]] || {
        echo "Power policy did not stick for $device ($vendor:$product): $actual" >&2
        exit 1
      }
      echo "$device $vendor:$product power/control=on"
      matched=$((matched + 1))
      ;;
  esac
done

if [[ "$matched" -eq 0 ]]; then
  echo "No matching D435i/USB3 hub is currently attached; nothing changed."
fi
