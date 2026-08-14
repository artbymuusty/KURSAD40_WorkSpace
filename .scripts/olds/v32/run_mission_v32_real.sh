#!/bin/bash
# run_mission_v32_real - Mission Executor Launcher (Gerçek Uçuş)

cd "$(dirname "$0")"
clear
echo "====================================="
echo " KURSAD40 V32 GERÇEK UÇUŞ"
echo " Mission Executor"
echo "====================================="
echo ""
export PYTHONPATH=$(pwd)/v32_flight_stack
../../.venv/bin/python -u v32_flight_stack/real_system/main_real.py "$@"
