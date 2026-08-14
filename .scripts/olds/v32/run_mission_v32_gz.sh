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
../../.venv/bin/python -u v32_flight_stack/gz_system/main_gz.py "$@"
