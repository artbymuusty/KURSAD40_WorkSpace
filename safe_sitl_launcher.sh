#!/bin/bash
# safe_sitl_launcher.sh
# Deterministic PX4 SITL + Gazebo Orchestration Wrapper
# Enforces the safety invariants defined in the orchestration contract.

echo "==========================================================="
echo "[ORCHESTRATOR] Initializing pre-flight state validation..."
echo "==========================================================="

# ---------------------------------------------------------
# 1. ENVIRONMENT NEUTRALITY
# ---------------------------------------------------------
echo "[ORCHESTRATOR] 1/4 Scrubbing environment variables..."
unset PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION
unset PYTHONPATH

# GZ_IP pins gz-transport's discovery/advertise address. process_manager.py,
# camera_service.py and unpause/verify_gazebo_ready() all force GZ_IP=127.0.0.1
# for every Python-side tool that talks to Gazebo -- but this launcher (the
# side that actually starts PX4+Gazebo) never set it, leaving it to whatever
# gz-transport auto-selects on this host. On a machine with more than one
# active network interface that can put the simulator and the Python tooling
# on different discovery paths, so gz_bridge/camera_service never see each
# other's topics even though both are genuinely running. Pin it here too so
# every process in the chain agrees.
export GZ_IP=127.0.0.1

# ---------------------------------------------------------
# 2. PRE-LAUNCH STATE PURGE (Process-level enforcement)
# ---------------------------------------------------------
echo "[ORCHESTRATOR] 2/4 Terminating existing/orphaned Gazebo and PX4 processes..."

# Kill any orphaned PX4 SITL processes to ensure no conflicting sessions
pkill -9 -f "px4_sitl" 2>/dev/null
pkill -9 -f "px4" 2>/dev/null

# Kill any Gazebo server processes or zombie transport listeners
#
# BUG FIX: these patterns used to say "gz-sim" (with a hyphen). The actual
# running process is "gz" (the CLI dispatcher) invoked with "sim" as a
# subcommand -- its real /proc/<pid>/cmdline is "gz sim --verbose=1 -r -s
# <world>.sdf" (a space, not a hyphen), so `pkill -9 -f "gz-sim"` has never
# matched it. Confirmed directly: a gz-sim server PID survived multiple
# consecutive runs of this script, each one printing "Pre-launch invariants
# met" while that same already-running (and, in one investigation, already
# crashed/toppled) world and vehicle model kept being reused underneath every
# supposedly-fresh PX4 relaunch. "gz sim" (space) matches the real dispatcher
# invocation; "gz sim -g" (the GUI client) matches the same pattern too.
pkill -9 -f "gz-transport-topic" 2>/dev/null
pkill -9 -f "gz sim" 2>/dev/null
pkill -9 -f "ruby-mri" 2>/dev/null

# Give OS a moment to reap processes
sleep 2

# ---------------------------------------------------------
# 3. VERIFY IDLE STATE
# ---------------------------------------------------------
echo "[ORCHESTRATOR] 3/4 Verifying process null state..."

if pgrep -f "px4|gz sim|gz-transport-topic" > /dev/null; then
    echo "[ORCHESTRATOR] FATAL: Failed to clear orphaned simulation processes."
    echo "[ORCHESTRATOR] Manual intervention required. Processes still alive:"
    pgrep -l -f "px4|gz sim|gz-transport-topic"
    exit 1
fi

echo "  -> [OK] Pre-launch invariants met. System is in a clean idle state."

# ---------------------------------------------------------
# 4. CONTROLLED SITL BOOTSTRAP
# ---------------------------------------------------------
echo "[ORCHESTRATOR] 4/4 Bootstrapping PX4 SITL natively..."
echo "==========================================================="

# Navigate to the PX4 root directory (assuming script is in root or .scripts)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"
if [[ "$SCRIPT_DIR" == *".scripts"* ]]; then
    cd "$SCRIPT_DIR/../.." || exit 1
else
    cd "$SCRIPT_DIR" || exit 1
fi

# Pass control to PX4
# PX4 will now correctly detect a clean topology and launch exactly
# one unified PX4 + Gazebo simulation authority.
# NOTE: the payload drop mechanism (PayloadDropSystem plugin + payload_blue/red
# dummy links) is built directly into Tools/simulation/gz/models/x500_mono_cam_down
# now -- there is no separate "_payload" model variant anymore (it existed
# briefly, then was consolidated into the base model; a stale reference to it
# here previously pointed at a make target/model directory that no longer
# exists, which would have failed to spawn the vehicle at all).
make px4_sitl gz_x500_mono_cam_down
