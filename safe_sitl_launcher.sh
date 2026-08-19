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
#
# GZ_PARTITION matters for the same reason and was the actual macOS blocker:
# gz-transport's DEFAULT partition is "<hostname>:<username>", and this Mac's
# hostname is DHCP/reverse-DNS derived, so it changes with the network. The
# sim and anything launched later ended up in different partitions and never
# discovered each other. Both values now come from one shared file used by the
# sim, mission and camera launchers alike.
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)/.scripts/olds/v32/v32_flight_stack/gz_system/gz_env.sh"
echo "[ORCHESTRATOR] gz-transport env: GZ_PARTITION=$GZ_PARTITION GZ_IP=$GZ_IP"

# ---------------------------------------------------------
# 2. PRE-LAUNCH STATE PURGE (Process-level enforcement)
# ---------------------------------------------------------
echo "[ORCHESTRATOR] 2/4 Terminating existing/orphaned Gazebo and PX4 processes..."

# Kill any orphaned PX4 SITL processes to ensure no conflicting sessions
# ADR-010 R3: ANCHORED patterns. These used to be `pkill -9 -f "px4"`,
# which matches ANY process whose full command line merely mentions px4 --
# including unrelated shells, editors, and tooling. That is not
# hypothetical: it kills the caller's own helper shells, and the
# verify-idle gate below (same unanchored pattern) then reports FATAL and
# exits WITHOUT starting the simulator, which is the most likely reason
# several "relaunch" attempts on 2026-08-17 left no PX4 running and no
# ULog on disk at all. Anchoring to the real binary path and the real
# `gz sim` invocation matches the processes we actually mean.
pkill -9 -f "px4_sitl_default/bin/px4$" 2>/dev/null
pkill -9 -f "bin/px4$" 2>/dev/null

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

# ADR-010 R3: anchored for the same reason as the pkill patterns above --
# an unanchored "px4" here made the gate trip on any shell that merely
# mentioned px4, aborting the launch before it began.
_ORPHANS="$(pgrep -f 'bin/px4$'; pgrep -f 'gz sim'; pgrep -f 'gz-transport-topic')"
if [ -n "$_ORPHANS" ]; then
    echo "[ORCHESTRATOR] FATAL: Failed to clear orphaned simulation processes."
    echo "[ORCHESTRATOR] Manual intervention required. Processes still alive:"
    echo "$_ORPHANS" | while read -r _p; do ps -p "$_p" -o pid,args= 2>/dev/null; done
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

# ---------------------------------------------------------
# 5. POST-LAUNCH: CLEAR A STUCK LAND MODE  (ADR-010 R3)
# ---------------------------------------------------------
# After a mission lands, PX4 stays in flight_mode=LAND even once disarmed
# and ON_GROUND, and refuses to arm from there (is_armable False while
# every individual pre-arm check passes). Three runs on 2026-08-17 died on
# this, each after ~3 minutes of futile waiting; commanding HOLD clears it
# in about 2 seconds. This lives here, in the launcher/hygiene layer,
# deliberately: it is a rig-reset concern, not mission logic, and the
# mission must never quietly re-arm a vehicle an operator left in LAND.
_HYGIENE="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)/.scripts/olds/v32/clear_land_mode.py"
if [ -f "$_HYGIENE" ]; then
    echo "[ORCHESTRATOR] 5/5 Clearing any stuck LAND mode before arming..."
    source "$(dirname "${BASH_SOURCE[0]}")/.scripts/olds/v32/resolve_python.sh" 2>/dev/null
    "${PYTHON_BIN:-python3}" "$_HYGIENE" || true
fi
