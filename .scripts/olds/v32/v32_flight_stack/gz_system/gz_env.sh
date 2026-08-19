#!/bin/bash
# gz_env.sh - single source of truth for the KURSAD40 gz-transport environment.
#
# WHY THIS EXISTS (macOS port finding):
# gz-transport's DEFAULT partition is "<hostname>:<username>". On this Mac the
# hostname is DHCP/reverse-DNS derived (e.g. "172-2-2-144.lightspeed...") and
# changes when the network changes. The simulator was started under one
# hostname and every process launched afterwards computed a DIFFERENT default
# partition, so they never discovered each other: `gz topic -l` returned
# nothing and camera_service.py logged "no live camera topic found ... will
# likely produce zero frames" while Gazebo was publishing 1280x960 @30Hz the
# whole time. Pinning an explicit, fixed partition removes the hostname from
# the equation entirely.
#
# Every process that talks to Gazebo (sim, mission, camera) MUST agree on
# these two values. Source this file; do not re-declare them elsewhere.
#
#   source "<...>/v32_flight_stack/gz_system/gz_env.sh"
#
# NOTE: this is unrelated to PX4's own
# src/modules/simulation/gz_bridge/gz_env.sh.in (resource paths), which is
# PX4 source and is deliberately left untouched.

export GZ_PARTITION="${GZ_PARTITION:-kursad40}"
export GZ_IP="${GZ_IP:-127.0.0.1}"
