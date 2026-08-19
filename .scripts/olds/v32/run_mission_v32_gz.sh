#!/bin/bash
# run_mission_v32_gz - Mission Executor Launcher (Gazebo Simulator)

cd "$(dirname "$0")"
clear
echo "====================================="
echo " KURSAD40 V32 GAZEBO SIMULATION"
echo " Mission Executor"
echo "====================================="
echo ""
export PYTHONPATH=$(pwd)/v32_flight_stack

# Shared gz-transport env (GZ_PARTITION/GZ_IP) -- must match the sim launcher,
# otherwise gz-transport discovery silently yields zero camera frames.
source "$(pwd)/v32_flight_stack/gz_system/gz_env.sh"

source "$(dirname "$0")/resolve_python.sh"

"$PYTHON_BIN" -u v32_flight_stack/gz_system/main_gz.py "$@"
