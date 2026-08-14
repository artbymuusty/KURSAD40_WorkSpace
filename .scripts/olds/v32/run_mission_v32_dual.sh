#!/bin/bash
# run_mission_v32_dual - Mission Executor Launcher (Eşzamanlı Simülasyon + Gerçek)

cd "$(dirname "$0")"
clear
echo "====================================="
echo " KURSAD40 V32 DUAL MODE (Gölge Test)"
echo " Mission Executor"
echo "====================================="
echo ""
export PYTHONPATH=$(pwd)/v32_flight_stack
../../.venv/bin/python -u v32_flight_stack/dual_system/main_dual.py "$@"
