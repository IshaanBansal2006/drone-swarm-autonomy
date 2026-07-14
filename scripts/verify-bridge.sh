#!/usr/bin/env bash
# mini-lattice — verify the Isaac Sim (Windows) <-> WSL ROS2 bridge.
# Usage:  bash scripts/verify-bridge.sh
#
# This does NOT test any mini-lattice logic. It only confirms that DDS
# discovery crosses the WSL<->Windows boundary in both directions.
set -eo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$HERE/ros-env.sh"

echo
echo "== 1. WSL IP (mirrored mode should match your Windows IPv4) =="
ip -4 addr show | awk '/inet /{print "   " $2 "  ("$NF")"}'

echo
echo "== 2. Topics visible from WSL (Isaac Sim must be running + publishing) =="
echo "   Waiting 5s for discovery..."
timeout 6 ros2 topic list 2>/dev/null | sed 's/^/   /' || echo "   (no topics — is Isaac Sim running with the ROS2 bridge enabled?)"

echo
echo "== 3. Round-trip hints =="
echo "   WSL -> Windows:  ros2 topic pub /wsl_ping std_msgs/msg/String \"{data: hi}\""
echo "   Windows -> WSL:  echo a topic from Isaac Sim (e.g. /clock) into 'ros2 topic echo'"
echo
echo "If step 2 shows Isaac's topics, the bridge is up."
