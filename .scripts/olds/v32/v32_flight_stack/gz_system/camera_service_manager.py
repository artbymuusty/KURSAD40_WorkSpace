"""
Ported from V30/V31 (.scripts/olds/v32/camera_manager.py), adapted to:
  - accept topic/zmq_addr as parameters (the original hardcoded them via a
    flat config.py import) so GzCameraSource can pass through whatever
    gz_system.yaml configures, and
  - anchor PID/log file paths to this module's own directory instead of the
    caller's cwd, since v32_flight_stack's entrypoints are invoked from
    v32/ (via run_mission_v32_*.sh's `cd "$(dirname "$0")"`), not from
    inside gz_system/.

Owns starting/stopping the camera_service.py subprocess (isolated Gazebo/
protobuf dependencies -- see that module's docstring) and tracking it via a
PID file so repeated mission runs don't leave orphaned or duplicate
processes bound to the same ZMQ port.
"""
import os
import sys
import subprocess
import time
import traceback

_DIR = os.path.dirname(os.path.abspath(__file__))
PID_FILE = os.path.join(_DIR, ".camera_service.pid")
LOG_FILE = os.path.join(_DIR, ".camera_service.log")
SERVICE_SCRIPT = os.path.join(_DIR, "camera_service.py")


def is_running(pid: int) -> bool:
    """Check if a process with the given PID is currently running and is our camera_service."""
    try:
        os.kill(pid, 0)
    except OSError:
        return False

    try:
        with open(f"/proc/{pid}/cmdline", "r") as f:
            cmdline = f.read().replace('\x00', ' ')
            if "camera_service.py" in cmdline:
                return True
    except FileNotFoundError:
        pass

    return False


def get_pid():
    if not os.path.exists(PID_FILE):
        return None
    try:
        with open(PID_FILE, "r") as f:
            return int(f.read().strip())
    except ValueError:
        return None


def start(topic: str, zmq_addr: str) -> None:
    pid = get_pid()

    if pid and is_running(pid):
        print(f"[CAMERA_SERVICE_MANAGER] Already running (PID: {pid}).")
        return

    if pid:
        print("[CAMERA_SERVICE_MANAGER] Stale PID file detected. Cleaning up...")
        os.remove(PID_FILE)

    print(f"[CAMERA_SERVICE_MANAGER] Starting camera_service.py (topic={topic}, zmq={zmq_addr})...")

    with open(LOG_FILE, "w") as out:
        # Inject system dist-packages into PYTHONPATH strictly for this
        # subprocess: solves the gz-transport ModuleNotFoundError without
        # polluting the venv or requiring the bindings to be pip-installed
        # into it (they're apt-installed system-side, tied to Gazebo).
        env = os.environ.copy()
        system_paths = "/usr/lib/python3/dist-packages"
        env["PYTHONPATH"] = f"{system_paths}:{env['PYTHONPATH']}" if "PYTHONPATH" in env else system_paths

        try:
            p = subprocess.Popen(
                [sys.executable, "-u", SERVICE_SCRIPT, "--topic", topic, "--zmq-addr", zmq_addr],
                stdout=out,
                stderr=out,
                env=env,
                start_new_session=True,  # detach from terminal completely
            )
        except Exception:
            traceback.print_exc()
            raise

    with open(PID_FILE, "w") as f:
        f.write(str(p.pid))

    time.sleep(1.0)  # give it a moment to fail fast if it's going to
    if is_running(p.pid):
        print(f"[CAMERA_SERVICE_MANAGER] Started (PID: {p.pid}).")
    else:
        print(f"[CAMERA_SERVICE_MANAGER] FAILED to start -- check {LOG_FILE} for details.")


def stop() -> None:
    pid = get_pid()

    if not pid or not is_running(pid):
        print("[CAMERA_SERVICE_MANAGER] Not running.")
        if os.path.exists(PID_FILE):
            os.remove(PID_FILE)
        return

    print(f"[CAMERA_SERVICE_MANAGER] Stopping (PID: {pid})...")
    try:
        os.kill(pid, 15)  # SIGTERM
        for _ in range(50):
            if not is_running(pid):
                break
            time.sleep(0.1)
        if is_running(pid):
            print(f"[CAMERA_SERVICE_MANAGER] Did not exit cleanly, sending SIGKILL to PID {pid}...")
            os.kill(pid, 9)
        print("[CAMERA_SERVICE_MANAGER] Stopped.")
    except OSError as e:
        print(f"[CAMERA_SERVICE_MANAGER] Failed to kill process: {e}")

    if os.path.exists(PID_FILE):
        os.remove(PID_FILE)
