#!/bin/bash
# Live RViz against the real AMR, from the desktop.
#
# RViz runs inside a ROS 2 HUMBLE container on purpose. The desktop's own
# Jazzy must never join the robot's domain: rmw_dds_common/ParticipantEntitiesInfo
# differs between Humble and Jazzy, and a Jazzy participant breaks the
# discovery of every Humble node on that domain -- measured, 2026-10-18.
#
# Discovery is unicast because the AP drops multicast between wireless clients.
SP="$(cd "$(dirname "$0")" && pwd)"
xhost +local:docker >/dev/null 2>&1
exec docker run --rm -it --net=host \
  -e DISPLAY="$DISPLAY" \
  -e ROS_DOMAIN_ID=30 \
  -e FASTRTPS_DEFAULT_PROFILES_FILE=/cfg/fastdds_desktop.xml \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$SP":/cfg:ro \
  osrf/ros:humble-desktop \
  bash -lc 'source /opt/ros/humble/setup.bash && rviz2 -d /cfg/mapping.rviz'
