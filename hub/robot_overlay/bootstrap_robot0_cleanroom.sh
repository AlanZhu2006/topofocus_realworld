#!/usr/bin/env bash
# Build the Robot 0 software environment on a freshly flashed JetPack host.
# The default is a read-only plan. --apply installs/builds software but never
# starts ROS, TinyNav, the receiver, or the Unitree command bridge.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HUB_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
REPOSITORY_ROOT="$(cd "$HUB_DIR/.." && pwd)"
LOCK_FILE="$HUB_DIR/config/deployments/robot0_cleanroom_sources_v1.json"

apply=false
skip_system_packages=false
jobs="${FOCUS_BUILD_JOBS:-2}"
install_root="${FOCUS_ROBOT0_INSTALL_ROOT:-${XDG_DATA_HOME:-$HOME/.local/share}/topofocus/robot-0}"
config_root="${FOCUS_ROBOT0_CONFIG_ROOT:-${XDG_CONFIG_HOME:-$HOME/.config}/topofocus}"
state_root="${FOCUS_ROBOT0_STATE_ROOT:-${XDG_STATE_HOME:-$HOME/.local/state}/topofocus}"

usage() {
  cat <<'EOF'
Usage:
  bash hub/robot_overlay/bootstrap_robot0_cleanroom.sh [options]

Options:
  --apply                  install and build the environment
  --install-root DIR       user-owned install prefix
  --config-root DIR        generated configuration directory
  --state-root DIR         runtime state directory
  --jobs N                 bounded source-build parallelism (default: 2)
  --skip-system-packages   do not configure ROS or install apt packages

Without --apply, the script prints the complete plan and changes nothing.
It assumes the Jetson has already been flashed with the recorded JetPack
6.2.1/L4T 36.4.7 base image. It never starts ROS or sends a robot command.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --apply) apply=true; shift ;;
    --install-root) install_root="$2"; shift 2 ;;
    --config-root) config_root="$2"; shift 2 ;;
    --state-root) state_root="$2"; shift 2 ;;
    --jobs) jobs="$2"; shift 2 ;;
    --skip-system-packages) skip_system_packages=true; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ "$jobs" =~ ^[1-9][0-9]*$ ]] || {
  echo "--jobs must be a positive integer." >&2
  exit 2
}
for path_value in "$install_root" "$config_root" "$state_root"; do
  [[ -n "$path_value" && "$path_value" == /* && "$path_value" != / ]] || {
    echo "Deployment paths must be absolute, non-root paths: $path_value" >&2
    exit 2
  }
done
[[ -r "$LOCK_FILE" ]] || {
  echo "Missing clean-room source lock: $LOCK_FILE" >&2
  exit 1
}

python_bin="$(command -v python3.10 || command -v python3 || true)"
[[ -n "$python_bin" ]] || {
  echo "Python 3 is required to read the clean-room lock." >&2
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

tinynav_commit="$(lock_value sources.tinynav.reconstructed_commit)"
tinynav_tree="$(lock_value sources.tinynav.reconstructed_tree)"
cyclonedds_url="$(lock_value sources.cyclonedds.url)"
cyclonedds_commit="$(lock_value sources.cyclonedds.commit)"
gtsam_url="$(lock_value sources.gtsam.url)"
gtsam_commit="$(lock_value sources.gtsam.commit)"
librealsense_url="$(lock_value sources.librealsense.url)"
librealsense_commit="$(lock_value sources.librealsense.commit)"
realsense_ros_url="$(lock_value sources.realsense_ros.url)"
realsense_ros_commit="$(lock_value sources.realsense_ros.commit)"
message_filters_url="$(lock_value sources.message_filters.url)"
message_filters_commit="$(lock_value sources.message_filters.commit)"
uv_version="$(lock_value tools.uv)"
python_version="$(lock_value tools.python)"
l4t_version_expected="$(lock_value base_platform.l4t_package)"
ros_base_version="$(lock_value host_packages.exact_reference_versions.ros-humble-ros-base)"
rmw_cyclonedds_version="$(lock_value host_packages.exact_reference_versions.ros-humble-rmw-cyclonedds-cpp)"
unitree_interface="$(lock_value network_defaults.unitree_interface)"
unitree_host_cidr="$(lock_value network_defaults.host_address)"
unitree_robot_address="$(lock_value network_defaults.robot_address)"
hub_endpoint="$(lock_value network_defaults.hub_endpoint)"

src_root="$install_root/src"
build_root="$install_root/build"
workspace_root="$install_root/workspaces"
tinynav_root="$workspace_root/tinynav"
cyclonedds_root="$src_root/cyclonedds"
gtsam_root="$src_root/gtsam"
librealsense_root="$src_root/librealsense"
realsense_ws="$workspace_root/realsense_ws"
message_filters_ws="$workspace_root/message_filters_ws"
setup_file="$config_root/robot-0-setup.bash"
env_file="$config_root/robot-0.env"
provenance_file="$state_root/robot-0-cleanroom-provenance.json"

cat <<EOF
Robot 0 clean-room plan
  repository:       $REPOSITORY_ROOT
  source lock:      $LOCK_FILE
  install root:     $install_root
  config root:      $config_root
  state root:       $state_root
  TinyNav commit:   $tinynav_commit
  TinyNav tree:     $tinynav_tree
  system packages:  $([[ "$skip_system_packages" == true ]] && echo skip || echo install)
  TensorRT engines: build all four runtime plans

Phases:
  1. validate JetPack/Ubuntu/aarch64 and absence of motion processes
  2. install ROS Humble and build prerequisites
  3. build pinned CycloneDDS, GTSAM, librealsense, realsense-ros and message_filters
  4. reconstruct the pinned TopoFocus TinyNav tree
  5. fetch and SHA-256 verify the five Git-LFS ONNX models
  6. create the locked TinyNav Python environment and TensorRT plans
  7. generate username-independent config and a machine provenance record

No ROS, planner, receiver, camera, or Unitree process is started.
EOF

if [[ "$apply" != true ]]; then
  echo "PLAN_ONLY=true"
  exit 0
fi

observed_python_version="$("$python_bin" -c 'import platform; print(platform.python_version())')"
[[ "$observed_python_version" == "$python_version" ]] || {
  echo "Expected Python $python_version; found $observed_python_version." >&2
  exit 1
}
[[ "$(uname -m)" == aarch64 ]] || {
  echo "Robot 0 clean-room install requires aarch64; found $(uname -m)." >&2
  exit 1
}
grep -q 'Ubuntu 22.04' /etc/os-release || {
  echo "Robot 0 reference install requires Ubuntu 22.04." >&2
  exit 1
}
l4t_version="$(dpkg-query -W -f='${Version}' nvidia-l4t-core 2>/dev/null || true)"
[[ "$l4t_version" == "$l4t_version_expected" ]] || {
  echo "Expected nvidia-l4t-core $l4t_version_expected; found '${l4t_version:-missing}'." >&2
  exit 1
}
if pgrep -af \
  'go2_cmd_bridge|cmd_vel_control|planning_node.py|v2_wsj_receiver|focus_guarded_cmd_vel' \
  >/dev/null 2>&1; then
  echo "Refusing environment installation while a motion/planning process exists." >&2
  exit 1
fi

if [[ "$skip_system_packages" != true ]]; then
  sudo -v
  sudo apt-get update
  sudo apt-get install -y --no-install-recommends \
    ca-certificates curl gnupg locales software-properties-common
  sudo add-apt-repository -y universe
  sudo locale-gen en_US en_US.UTF-8
  sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
  ros_key="$(mktemp)"
  trap 'rm -f "$ros_key"' EXIT
  curl -fsSL \
    https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o "$ros_key"
  printf '%s  %s\n' \
    4a91c49af0d6f0016108b93698782b596c27ccd836937e18e0e36c3347dc602f \
    "$ros_key" | sha256sum -c -
  sudo install -m 0644 "$ros_key" /usr/share/keyrings/ros-archive-keyring.gpg
  printf '%s\n' \
    "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
  sudo apt-get update
  sudo apt-get install -y --no-install-recommends \
    build-essential cmake ninja-build pkg-config git git-lfs \
    python3.10 python3.10-dev python3.10-venv python3-pip \
    python3-colcon-common-extensions python3-rosdep \
    python3-numpy python3-pybind11 python3-pyparsing pybind11-dev \
    libboost-all-dev libeigen3-dev libtbb-dev libmetis-dev \
    libsuitesparse-dev libceres-dev libudev-dev libusb-1.0-0-dev \
    libglfw3-dev libssl-dev libgl1-mesa-dev libglu1-mesa-dev \
    libgtk-3-dev tmux jq rsync iproute2 \
    "ros-humble-ros-base=$ros_base_version" \
    "ros-humble-rmw-cyclonedds-cpp=$rmw_cyclonedds_version" \
    ros-humble-cv-bridge ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    ros-humble-compressed-image-transport \
    ros-humble-rviz2 ros-humble-rosbag2 \
    ros-humble-rosbag2-storage-default-plugins \
    ros-humble-tf2-ros ros-humble-tf2-geometry-msgs \
    ros-humble-vision-msgs ros-humble-foxglove-bridge
  sudo rosdep init 2>/dev/null || true
  rosdep update
  git lfs install --skip-repo
  trap - EXIT
  rm -f "$ros_key"
fi

source /opt/ros/humble/setup.bash
mkdir -p "$src_root" "$build_root" "$workspace_root" \
  "$config_root" "$state_root"

ensure_checkout() {
  local url="$1" commit="$2" destination="$3"
  if [[ ! -e "$destination" ]]; then
    git clone --filter=blob:none --no-checkout "$url" "$destination"
  fi
  [[ -d "$destination/.git" ]] || {
    echo "Existing source path is not a Git checkout: $destination" >&2
    exit 1
  }
  [[ -z "$(git -C "$destination" status --porcelain)" ]] || {
    echo "Refusing dirty source checkout: $destination" >&2
    exit 1
  }
  if ! git -C "$destination" cat-file -e "$commit^{commit}" 2>/dev/null; then
    git -C "$destination" fetch origin "$commit"
  fi
  git -C "$destination" switch --detach "$commit"
  [[ "$(git -C "$destination" rev-parse HEAD)" == "$commit" ]]
}

ensure_checkout "$cyclonedds_url" "$cyclonedds_commit" "$cyclonedds_root"
cmake -S "$cyclonedds_root" -B "$build_root/cyclonedds" \
  -G Ninja -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$install_root/cyclonedds"
cmake --build "$build_root/cyclonedds" --parallel "$jobs"
cmake --install "$build_root/cyclonedds"

ensure_checkout "$gtsam_url" "$gtsam_commit" "$gtsam_root"
cmake -S "$gtsam_root" -B "$build_root/gtsam" \
  -DCMAKE_BUILD_TYPE=Release \
  -DGTSAM_BUILD_PYTHON=ON \
  -DGTSAM_PYTHON_VERSION=3.10 \
  -DGTSAM_BUILD_TESTS=OFF \
  -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
  -DGTSAM_BUILD_UNSTABLE=OFF \
  -DGTSAM_THROW_CHEIRALITY_EXCEPTION=OFF
cmake --build "$build_root/gtsam" --parallel "$jobs"

ensure_checkout \
  "$librealsense_url" "$librealsense_commit" "$librealsense_root"
cmake -S "$librealsense_root" -B "$build_root/librealsense" \
  -DCMAKE_BUILD_TYPE=Release -DFORCE_RSUSB_BACKEND=true \
  -DBUILD_EXAMPLES=false -DBUILD_GRAPHICAL_EXAMPLES=false \
  -DBUILD_TOOLS=true \
  -DCHECK_FOR_UPDATES=OFF \
  -DCMAKE_INSTALL_PREFIX="$install_root/librealsense"
cmake --build "$build_root/librealsense" --parallel "$jobs"
cmake --install "$build_root/librealsense"

mkdir -p "$realsense_ws/src"
ensure_checkout \
  "$realsense_ros_url" "$realsense_ros_commit" \
  "$realsense_ws/src/realsense-ros"
(
  cd "$realsense_ws"
  rosdep install -i --from-path src --rosdistro humble \
    --skip-keys=librealsense2 -y
  CMAKE_PREFIX_PATH="$install_root/librealsense${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}" \
  LD_LIBRARY_PATH="$install_root/librealsense/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" \
    colcon build --merge-install --cmake-args \
      "-DCMAKE_PREFIX_PATH=$install_root/librealsense"
)

mkdir -p "$message_filters_ws/src"
ensure_checkout \
  "$message_filters_url" "$message_filters_commit" \
  "$message_filters_ws/src/message_filters"
(
  cd "$message_filters_ws"
  colcon build --merge-install --packages-select message_filters \
    --allow-overriding message_filters
)

TINYNAV_PATCHED_ROOT="$tinynav_root" \
  bash "$SCRIPT_DIR/bootstrap_go2.sh" --destination "$tinynav_root"
[[ "$(git -C "$tinynav_root" rev-parse HEAD)" == "$tinynav_commit" ]]
[[ "$(git -C "$tinynav_root" rev-parse HEAD^{tree})" == "$tinynav_tree" ]]
printf '%s  %s\n' \
  "$(lock_value sources.tinynav.pyproject_sha256)" \
  "$tinynav_root/pyproject.toml" | sha256sum -c -
printf '%s  %s\n' \
  "$(lock_value sources.tinynav.uv_lock_sha256)" \
  "$tinynav_root/uv.lock" | sha256sum -c -
git -C "$tinynav_root" lfs pull --include='tinynav/models/*.onnx'

"$python_bin" - "$LOCK_FILE" "$tinynav_root" <<'PY'
import hashlib
import json
from pathlib import Path
import sys

manifest = json.load(open(sys.argv[1], encoding="utf-8"))
root = Path(sys.argv[2])
for record in manifest["tinynav_onnx_artifacts"]:
    path = root / record["path"]
    if not path.is_file():
        raise SystemExit(f"missing ONNX artifact: {path}")
    if path.stat().st_size != record["bytes"]:
        raise SystemExit(f"ONNX size mismatch: {path}")
    digest_builder = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest_builder.update(block)
    digest = digest_builder.hexdigest()
    if digest != record["sha256"]:
        raise SystemExit(f"ONNX SHA-256 mismatch: {path}")
print("TINYNAV_ONNX_VERIFIED=5")
PY

uv_venv="$install_root/uv"
if [[ ! -x "$uv_venv/bin/uv" ]]; then
  "$python_bin" -m venv "$uv_venv"
  "$uv_venv/bin/python" -m pip install "uv==$uv_version"
fi
uv_bin="$uv_venv/bin/uv"
[[ "$("$uv_bin" --version)" == "uv $uv_version"* ]] || {
  echo "uv version does not match the source lock." >&2
  exit 1
}
tinynav_python="$tinynav_root/.venv/bin/python"
if [[ ! -x "$tinynav_python" ]]; then
  "$uv_bin" venv --python "$python_bin" \
    --system-site-packages "$tinynav_root/.venv"
fi
export CYCLONEDDS_HOME="$install_root/cyclonedds"
export CMAKE_PREFIX_PATH="$install_root/cyclonedds${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
export LD_LIBRARY_PATH="$install_root/cyclonedds/lib:$install_root/librealsense/lib:$build_root/gtsam/gtsam${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export PYTHONPATH="$build_root/gtsam/python:$tinynav_root${PYTHONPATH:+:$PYTHONPATH}"
UV_PROJECT_ENVIRONMENT="$tinynav_root/.venv" \
  "$uv_bin" sync --project "$tinynav_root" --frozen --no-dev --extra unitree

trtexec_bin="$(command -v trtexec || true)"
[[ -n "$trtexec_bin" ]] || {
  echo "TensorRT trtexec is missing from the JetPack installation." >&2
  exit 1
}
make -C "$tinynav_root/tinynav/models" \
  TRTEXEC="$trtexec_bin" -j1 all

compatibility_link=/tinynav
if [[ -e "$compatibility_link" || -L "$compatibility_link" ]]; then
  [[ "$(readlink -f "$compatibility_link")" == "$tinynav_root" ]] || {
    echo "$compatibility_link exists and points elsewhere." >&2
    exit 1
  }
else
  sudo ln -s "$tinynav_root" "$compatibility_link"
fi

tmp_setup="${setup_file}.tmp.$$"
{
  printf '%s\n' '#!/usr/bin/env bash'
  printf 'source %q\n' /opt/ros/humble/setup.bash
  printf 'source %q\n' "$realsense_ws/install/setup.bash"
  printf 'source %q\n' "$message_filters_ws/install/setup.bash"
  printf 'export CYCLONEDDS_HOME=%q\n' "$install_root/cyclonedds"
  printf 'export CMAKE_PREFIX_PATH=%q${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}\n' \
    "$install_root/cyclonedds"
  printf 'export LD_LIBRARY_PATH=%q:%q:%q${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}\n' \
    "$install_root/cyclonedds/lib" "$install_root/librealsense/lib" \
    "$build_root/gtsam/gtsam"
  printf 'export PYTHONPATH=%q:%q${PYTHONPATH:+:${PYTHONPATH}}\n' \
    "$build_root/gtsam/python" "$tinynav_root"
  printf 'export PATH=%q:%q:%q${PATH:+:${PATH}}\n' \
    "$tinynav_root/.venv/bin" "$install_root/librealsense/bin" \
    "$(dirname "$trtexec_bin")"
  printf '%s\n' 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp'
} >"$tmp_setup"
chmod 0644 "$tmp_setup"
mv "$tmp_setup" "$setup_file"

tmp_env="${env_file}.tmp.$$"
{
  printf 'TINYNAV_WORKSPACE=%q\n' "$workspace_root"
  printf 'TINYNAV_ROOT=%q\n' "$tinynav_root"
  printf 'TINYNAV_PATCHED_ROOT=%q\n' "$tinynav_root"
  printf 'TINYNAV_PERCEPTION_PATCHED_ROOT=%q\n' "$tinynav_root"
  printf 'TINYNAV_PERCEPTION_PATCHED_COMMIT=%q\n' "$tinynav_commit"
  printf '%s\n' \
    'TINYNAV_PERCEPTION_PATCHED_SHA256=3a695d5210d60ea1f721549ca7458ba89e7bf32db5178cd1c312c633aef1c3b3'
  printf 'TINYNAV_SETUP=%q\n' "$setup_file"
  printf 'TINYNAV_PYTHON=%q\n' "$tinynav_python"
  printf 'FOCUS_ROBOT_STATE_DIR=%q\n' "$state_root"
  printf '%s\n' 'FOCUS_ROBOT_ID=robot-0'
  printf 'FOCUS_HUB_BASE_URL=%q\n' "$hub_endpoint"
  printf 'UNITREE_NET_IF=%q\n' "$unitree_interface"
  printf 'UNITREE_HOST_CIDR=%q\n' "$unitree_host_cidr"
  printf 'UNITREE_ROBOT_ADDRESS=%q\n' "$unitree_robot_address"
} >"$tmp_env"
chmod 0600 "$tmp_env"
mv "$tmp_env" "$env_file"

sudo bash "$SCRIPT_DIR/install_go2_host_config.sh" --apply

"$python_bin" "$SCRIPT_DIR/verify_robot0_cleanroom.py" \
  --lock "$LOCK_FILE" \
  --install-root "$install_root" \
  --config-root "$config_root" \
  --state-root "$state_root" \
  --level host \
  --write-provenance "$provenance_file"

echo "ROBOT0_CLEANROOM_INSTALL_COMPLETE=true"
echo "Configuration: $env_file"
echo "Provenance:    $provenance_file"
echo "Next read-only hardware gate:"
echo "  source '$setup_file'"
echo "  '$python_bin' '$SCRIPT_DIR/verify_robot0_cleanroom.py' --level hardware"
echo "No ROS process or physical command was started."
