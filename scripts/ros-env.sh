# mini-lattice — ROS2 cross-boundary environment (WSL side)
# Source this in every WSL shell that talks to Isaac Sim on Windows:
#   source scripts/ros-env.sh
#
# Both sides (this WSL shell AND the Isaac Sim ROS2 bridge on Windows) must
# agree on ROS_DOMAIN_ID and use the same RMW. Isaac Sim 4.x bundles Humble +
# Fast DDS, so we pin Fast DDS here to match.

source /opt/ros/humble/setup.bash

export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Mirrored networking shares the Windows network stack, so multicast discovery
# works without a discovery server. Leave ROS_LOCALHOST_ONLY unset (0) so
# discovery is not restricted to loopback.
export ROS_LOCALHOST_ONLY=0

echo "[ros-env] ROS_DISTRO=$ROS_DISTRO DOMAIN=$ROS_DOMAIN_ID RMW=$RMW_IMPLEMENTATION"
