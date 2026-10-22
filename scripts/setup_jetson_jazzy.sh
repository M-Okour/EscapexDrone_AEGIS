#!/usr/bin/env bash
# Jetson setup for JetPack 7.2 (L4T R39, Ubuntu 24.04 "noble") -> ROS 2 Jazzy.
# The repo docs target Humble/JetPack 6; on Ubuntu 24.04 the matching ROS 2 is Jazzy.
#
#   cd ~/Documents/EscapexDrone_Vortex
#   bash scripts/setup_jetson_jazzy.sh            # install + build + unit tests + smoke test
#   SKIP_SMOKE=1 bash scripts/setup_jetson_jazzy.sh
#
# Safe to re-run: steps that are already done are skipped.
set -eo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DISTRO=jazzy
step() { echo; echo "=== $* ==="; }

. /etc/os-release
if [[ "$VERSION_CODENAME" != "noble" ]]; then
  echo "This script is for Ubuntu 24.04 (noble); found $VERSION_CODENAME." >&2
  exit 1
fi

# 1. ROS 2 Jazzy
if [[ ! -f /opt/ros/$DISTRO/setup.bash ]]; then
  step "Installing ROS 2 $DISTRO"
  sudo apt-get update
  sudo apt-get install -y software-properties-common curl locales
  sudo locale-gen en_US en_US.UTF-8
  sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
  sudo add-apt-repository -y universe

  # Official ROS apt source package (sets up key + repository)
  ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
    | grep -F '"tag_name"' | awk -F'"' '{print $4}')
  curl -L -o /tmp/ros2-apt-source.deb \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${VERSION_CODENAME}_all.deb"
  sudo dpkg -i /tmp/ros2-apt-source.deb

  sudo apt-get update
  sudo apt-get install -y ros-$DISTRO-ros-base ros-dev-tools
else
  step "ROS 2 $DISTRO already installed"
fi

# 2. Extra tools the repo uses (same set as docker/Dockerfile, Jazzy names)
step "Installing extra tools"
sudo apt-get install -y python3-opencv python3-pytest \
  ros-$DISTRO-rviz2 ros-$DISTRO-rqt-robot-monitor ros-$DISTRO-rqt-image-view

# 3. rosdep
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  step "Initialising rosdep"
  sudo rosdep init
fi
rosdep update --rosdistro $DISTRO

# 4. Shell setup
if ! grep -q "/opt/ros/$DISTRO/setup.bash" ~/.bashrc; then
  step "Adding ROS to ~/.bashrc"
  {
    echo ""
    echo "# ROS 2 $DISTRO + Escape-x-Drone workspace"
    echo "source /opt/ros/$DISTRO/setup.bash"
    echo "[ -f $WS/install/setup.bash ] && source $WS/install/setup.bash"
    echo "export ROS_DOMAIN_ID=\${ROS_DOMAIN_ID:-42}   # must match the ground laptop"
  } >> ~/.bashrc
fi

# ROS setup scripts reference unset variables, so no `set -u` here.
# shellcheck disable=SC1090
source /opt/ros/$DISTRO/setup.bash
cd "$WS"

# 5. Dependencies + build
step "rosdep install"
rosdep install --from-paths src --ignore-src -y --rosdistro $DISTRO

step "colcon build"
colcon build --symlink-install --event-handlers console_cohesion+
source install/setup.bash

# 6. Tests
step "Unit tests"
colcon test --event-handlers console_cohesion+ || true
colcon test-result --verbose

if [[ -z "${SKIP_SMOKE:-}" ]]; then
  step "Smoke test (full stack in sim)"
  ./scripts/smoke_test.sh
fi

echo
echo "Done. Open a new terminal (or 'source ~/.bashrc'), then:"
echo "  ros2 launch exd_bringup exd.launch.py auto_start:=true"
