# drone-swarm-autonomy — ROS2 cross-boundary environment (WSL side)
# Source this in every WSL shell that talks to Isaac Sim on Windows:
#   source scripts/ros-env.sh
#
# HOW THE BRIDGE WORKS (do not regress this — hard-won 2026-07-29):
# Mirrored WSL networking gives Windows and WSL the SAME IP, so default Fast DDS
# (a) treats the other side as same-host and tries shared-memory transport, which
# cannot cross the OS boundary, and (b) announces own-IP locators that route back
# to the sender. Raw UDP over 127.0.0.1 DOES cross in both directions, so both
# sides force a loopback-only UDP profile (config/fastdds-loopback.xml): UDP
# transport only (no SHM), interface whitelist 127.0.0.1, peer discovery via
# initial peers at 127.0.0.1 (plain RTPS — no multicast, no discovery server,
# humble<->jazzy compatible). Isaac side gets the same XML via run_scene.bat.
#
# NOTE: do NOT set ROS_SUPER_CLIENT=true — it crashes the Humble ros2 daemon
# (`!rclpy.ok()` on every CLI call). If the CLI misbehaves, prefer
# `ros2 topic list --no-daemon` and `ros2 topic echo <topic> <type>`.

# zsh and bash need different setup files (you use zsh).
if [ -n "${ZSH_VERSION:-}" ]; then
  source /opt/ros/humble/setup.zsh
else
  source /opt/ros/humble/setup.bash
fi

export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
unset ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT

_ML_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]:-${(%):-%x}}")/.." && pwd)"
export FASTRTPS_DEFAULT_PROFILES_FILE="$_ML_ROOT/config/fastdds-loopback.xml"

echo "[ros-env] ROS_DISTRO=$ROS_DISTRO DOMAIN=$ROS_DOMAIN_ID RMW=$RMW_IMPLEMENTATION profile=$FASTRTPS_DEFAULT_PROFILES_FILE"
