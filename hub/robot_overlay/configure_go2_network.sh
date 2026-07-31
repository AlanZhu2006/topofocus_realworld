#!/usr/bin/env bash
# Configure the Jetson-side Unitree LAN without opening DDS or contacting Go2.
# The default is a read-only plan; --apply changes only the selected host NIC.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HUB_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
LOCK_FILE="$HUB_DIR/config/deployments/robot0_cleanroom_sources_v1.json"

apply=false
interface="${UNITREE_NET_IF:-}"
host_cidr="${UNITREE_HOST_CIDR:-}"
robot_address="${UNITREE_ROBOT_ADDRESS:-}"

usage() {
  cat <<'EOF'
Usage:
  bash hub/robot_overlay/configure_go2_network.sh [options]

Options:
  --apply               configure the selected host interface
  --interface NAME      dedicated Unitree Ethernet interface
  --host-cidr CIDR      Jetson address/prefix (for example 192.168.123.100/24)
  --robot-address IP    expected Go2 address (for route verification only)

Without --apply, the script prints the resolved plan and changes nothing.
The apply path does not ping Go2, initialize DDS, or send a robot command.
It is intentionally transient: use the host network manager separately if a
persistent interface configuration is desired.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --apply) apply=true; shift ;;
    --interface) interface="$2"; shift 2 ;;
    --host-cidr) host_cidr="$2"; shift 2 ;;
    --robot-address) robot_address="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ -r "$LOCK_FILE" ]] || {
  echo "Missing clean-room source lock: $LOCK_FILE" >&2
  exit 1
}
python_bin="$(command -v python3.10 || command -v python3 || true)"
[[ -n "$python_bin" ]] || {
  echo "Python 3 is required to read the source lock." >&2
  exit 1
}
lock_value() {
  "$python_bin" - "$LOCK_FILE" "$1" <<'PY'
import json
import sys

value = json.load(open(sys.argv[1], encoding="utf-8"))
for part in sys.argv[2].split("."):
    value = value[part]
print(value)
PY
}

interface="${interface:-$(lock_value network_defaults.unitree_interface)}"
host_cidr="${host_cidr:-$(lock_value network_defaults.host_address)}"
robot_address="${robot_address:-$(lock_value network_defaults.robot_address)}"

[[ "$interface" =~ ^[a-zA-Z0-9_.:-]+$ ]] || {
  echo "Invalid interface name: $interface" >&2
  exit 2
}
"$python_bin" - "$host_cidr" "$robot_address" <<'PY'
import ipaddress
import sys

host = ipaddress.ip_interface(sys.argv[1])
robot = ipaddress.ip_address(sys.argv[2])
if host.version != 4 or robot.version != 4:
    raise SystemExit("Unitree reference network requires IPv4")
if robot not in host.network:
    raise SystemExit(f"{robot} is outside {host.network}")
if robot == host.ip:
    raise SystemExit("host and robot addresses must differ")
PY

cat <<EOF
Go2 host-network plan
  interface:       $interface
  host address:    $host_cidr
  robot address:   $robot_address
  persistence:     none (transient host configuration)
  robot contacted: no
EOF

if [[ "$apply" != true ]]; then
  echo "PLAN_ONLY=true"
  exit 0
fi

[[ -d "/sys/class/net/$interface" ]] || {
  echo "Interface does not exist: $interface" >&2
  exit 1
}
if pgrep -af \
  'go2_cmd_bridge|cmd_vel_control|planning_node.py|v2_wsj_receiver|v2_robot0_receiver|focus_guarded_cmd_vel' \
  >/dev/null 2>&1; then
  echo "Refusing network changes while a motion/planning process exists." >&2
  exit 1
fi

sudo -v
sudo ip link set dev "$interface" up
sudo ip address replace "$host_cidr" dev "$interface"

address_output="$(ip -o -4 address show dev "$interface")"
route_output="$(ip route get "$robot_address")"
grep -Fq "$host_cidr" <<<"$address_output" || {
  echo "Interface verification failed: $host_cidr is absent." >&2
  exit 1
}
grep -Fq "dev $interface" <<<"$route_output" || {
  echo "Route verification failed: $robot_address does not use $interface." >&2
  exit 1
}

echo "$address_output"
echo "$route_output"
echo "GO2_HOST_NETWORK_CONFIGURED=true"
echo "No DDS participant was created and no robot command was sent."
