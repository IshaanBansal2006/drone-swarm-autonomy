#!/usr/bin/env bash
# Fast DDS Discovery Server for the Isaac (Windows) <-> WSL ROS2 bridge.
#
# Unicast discovery — bypasses the WSL mirrored-networking multicast wall
# (WSL and Isaac never meet on multicast; they meet on this server instead).
# Leave this RUNNING whenever you want the bridge up.
#
# Server id 0, listens on all interfaces (so Isaac reaches it via loopback or the
# shared IP), port 7500 (inside the firewall-allowed 7400-7700 range).
set -eo pipefail

if [ -n "${ZSH_VERSION:-}" ]; then
  source /opt/ros/humble/setup.zsh
else
  source /opt/ros/humble/setup.bash
fi

echo "[discovery-server] starting Fast DDS discovery server 0 on 0.0.0.0:7500 ..."
exec fastdds discovery -i 0 -l 0.0.0.0 -p 7500
