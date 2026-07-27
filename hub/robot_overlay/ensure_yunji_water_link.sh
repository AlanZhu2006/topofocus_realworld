#!/usr/bin/env bash
# Recover and verify the NUC-to-WATER Ethernet data path without robot motion.
set -euo pipefail

ROBOT_HOST="${FOCUS_YUNJI_WATER_HOST:-192.168.10.10}"
ROBOT_PORT="${FOCUS_YUNJI_WATER_PORT:-31001}"
PROFILE="${FOCUS_YUNJI_WATER_NM_PROFILE:-Yunji-Robot}"
EXPECTED_MAC="${FOCUS_YUNJI_WATER_NIC_MAC:-48:21:0b:6e:1f:bd}"
CONNECT_TIMEOUT_S="${FOCUS_YUNJI_WATER_CONNECT_TIMEOUT_S:-3}"

[[ "$ROBOT_HOST" =~ ^[A-Za-z0-9.-]+$ ]] || {
  echo "Invalid FOCUS_YUNJI_WATER_HOST." >&2
  exit 2
}
[[ "$ROBOT_PORT" =~ ^[1-9][0-9]{0,4}$ ]] || {
  echo "Invalid FOCUS_YUNJI_WATER_PORT." >&2
  exit 2
}
[[ "$CONNECT_TIMEOUT_S" =~ ^[1-9][0-9]*$ ]] || {
  echo "Invalid FOCUS_YUNJI_WATER_CONNECT_TIMEOUT_S." >&2
  exit 2
}

expected_mac="$(tr '[:upper:]' '[:lower:]' <<<"$EXPECTED_MAC")"
interface=""
for address_file in /sys/class/net/*/address; do
  [[ -r "$address_file" ]] || continue
  candidate="$(tr '[:upper:]' '[:lower:]' <"$address_file")"
  if [[ "$candidate" == "$expected_mac" ]]; then
    interface="$(basename "$(dirname "$address_file")")"
    break
  fi
done
[[ -n "$interface" ]] || {
  echo "YUNJI_WATER_NIC_MISSING: expected_mac=$expected_mac" >&2
  exit 1
}

carrier="$(cat "/sys/class/net/$interface/carrier" 2>/dev/null || true)"
[[ "$carrier" == 1 ]] || {
  echo "YUNJI_WATER_LINK_NO_CARRIER: interface=$interface expected_peer=$ROBOT_HOST" >&2
  echo "Check the NUC-to-WATER Ethernet cable and WATER chassis power." >&2
  exit 1
}

route_matches() {
  local route
  route="$(ip -4 route get "$ROBOT_HOST" 2>/dev/null || true)"
  [[ " $route " == *" dev $interface "* \
      && " $route " == *" src 192.168.10."* ]]
}

if ! route_matches; then
  profile_mac="$(
    nmcli --escape no -g 802-3-ethernet.mac-address \
      connection show "$PROFILE" 2>/dev/null \
      | tr '[:upper:]' '[:lower:]'
  )"
  [[ "$profile_mac" == "$expected_mac" ]] || {
    echo "YUNJI_WATER_PROFILE_MISMATCH: profile=$PROFILE expected_mac=$expected_mac observed_mac=${profile_mac:-missing}" >&2
    exit 1
  }
  echo "Recovering NetworkManager profile $PROFILE on $interface (network only; no robot command)."
  nmcli connection up id "$PROFILE" ifname "$interface" >/dev/null 2>&1 \
    || sudo -n nmcli connection up id "$PROFILE" ifname "$interface" \
      >/dev/null
  deadline=$((SECONDS + 10))
  until route_matches; do
    (( SECONDS < deadline )) || {
      echo "YUNJI_WATER_ROUTE_NOT_READY: host=$ROBOT_HOST interface=$interface" >&2
      exit 1
    }
    sleep 1
  done
fi

if ! timeout "$CONNECT_TIMEOUT_S" bash -c \
    'exec 3<>/dev/tcp/"$1"/"$2"' _ "$ROBOT_HOST" "$ROBOT_PORT" \
    2>/dev/null; then
  echo "YUNJI_WATER_TCP_UNREACHABLE: $ROBOT_HOST:$ROBOT_PORT via $interface" >&2
  exit 1
fi

echo "YUNJI_WATER_LINK_READY: host=$ROBOT_HOST port=$ROBOT_PORT interface=$interface"
echo "Safety: network readiness only; no WATER move or velocity request was issued."
