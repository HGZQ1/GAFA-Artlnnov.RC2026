#!/usr/bin/env bash
# Install RC2026 bare-metal dependencies on Ubuntu 22.04 without Docker.
# RealSense dependencies are included.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
APT_DEPS_FILE="${1:-$SCRIPT_DIR/host_apt_deps.txt}"
PIP_INDEX_URL="${PIP_INDEX_URL:-https://pypi.tuna.tsinghua.edu.cn/simple}"

info() {
  printf '[INFO] %s\n' "$1"
}

warn() {
  printf '[WARN] %s\n' "$1" >&2
}

die() {
  printf '[ERROR] %s\n' "$1" >&2
  exit 1
}

if [[ ! -f "$APT_DEPS_FILE" ]]; then
  die "Apt dependency file not found: $APT_DEPS_FILE"
fi

if [[ ! -f /etc/os-release ]]; then
  die "/etc/os-release not found; this script expects Ubuntu 22.04."
fi

# shellcheck disable=SC1091
source /etc/os-release
if [[ "${ID:-}" != "ubuntu" || "${VERSION_CODENAME:-}" != "jammy" ]]; then
  die "This script targets Ubuntu 22.04 jammy. Current: ${PRETTY_NAME:-unknown}"
fi

info "Installing bootstrap tools and enabling Ubuntu universe repository..."
sudo apt update
sudo apt install -y software-properties-common curl gnupg lsb-release ca-certificates
sudo add-apt-repository universe -y

info "Adding ROS2 Humble apt repository..."
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${VERSION_CODENAME} main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null

info "Adding Intel RealSense apt repository..."
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || \
sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo ${VERSION_CODENAME} main" -y

info "Reading apt dependencies from: $APT_DEPS_FILE"
mapfile -t APT_PACKAGES < <(grep -Ev '^[[:space:]]*(#|$)' "$APT_DEPS_FILE")
if [[ "${#APT_PACKAGES[@]}" -eq 0 ]]; then
  die "No apt packages found in $APT_DEPS_FILE"
fi

info "Installing ${#APT_PACKAGES[@]} apt packages..."
sudo apt update
sudo apt install -y --no-install-recommends "${APT_PACKAGES[@]}"

info "Installing Python dependencies from docker/requirements.txt..."
python3 -m pip install --user --upgrade pip setuptools wheel
python3 -m pip install --user -r "$SCRIPT_DIR/requirements.txt" -i "$PIP_INDEX_URL"

info "Installing bare-metal Python extras: open3d, ttkbootstrap..."
python3 -m pip install --user open3d ttkbootstrap -i "$PIP_INDEX_URL"

info "Initializing rosdep..."
sudo rosdep init 2>/dev/null || true
rosdep update

ROSDEP_PATHS=()
[[ -d "$PROJECT_DIR/ros2_ws/src" ]] && ROSDEP_PATHS+=("$PROJECT_DIR/ros2_ws/src")
[[ -d "$PROJECT_DIR/sim_ws/src" ]] && ROSDEP_PATHS+=("$PROJECT_DIR/sim_ws/src")

if [[ "${#ROSDEP_PATHS[@]}" -gt 0 ]]; then
  info "Running rosdep install for project workspaces..."
  if ! rosdep install --from-paths "${ROSDEP_PATHS[@]}" --ignore-src -r -y; then
    warn "rosdep install reported unresolved dependencies. Continuing because key dependencies are installed from $APT_DEPS_FILE and pip."
  fi
fi

info "Adding current user to hardware access groups..."
sudo usermod -aG video,dialout,plugdev "$USER"

if ! grep -qxF "source /opt/ros/humble/setup.bash" "$HOME/.bashrc"; then
  echo "source /opt/ros/humble/setup.bash" >> "$HOME/.bashrc"
fi

cat <<EOF

[OK] RC2026 bare-metal dependencies installed.

RealSense dependencies are included:
  - ros-humble-realsense2-camera
  - ros-humble-librealsense2
  - librealsense2*
Next:
  1. Log out and log back in, or reboot, so group changes take effect.
  2. Source ROS2:
       source /opt/ros/humble/setup.bash
  3. Build project workspaces manually.

External packages still required for full robot launch:
  - livox_ros_driver2
  - fast_lio
EOF
