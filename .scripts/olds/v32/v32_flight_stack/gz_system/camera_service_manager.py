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

try:  # normal import when v32_flight_stack is on PYTHONPATH (mission entrypoints)
    from gz_system.gz_env import apply_gz_env, describe_gz_env
except ImportError:  # running from inside gz_system/ (standalone/legacy callers)
    from gz_env import apply_gz_env, describe_gz_env

_DIR = os.path.dirname(os.path.abspath(__file__))
PID_FILE = os.path.join(_DIR, ".camera_service.pid")
LOG_FILE = os.path.join(_DIR, ".camera_service.log")
SERVICE_SCRIPT = os.path.join(_DIR, "camera_service.py")


def is_running(pid: int) -> bool:
    """Check if a process with the given PID is currently running and is our camera_service.

    Uses `ps` rather than reading /proc/{pid}/cmdline (Linux-only -- there is
    no /proc filesystem on macOS) so this works unmodified on both platforms.
    """
    try:
        os.kill(pid, 0)
    except OSError:
        return False

    try:
        result = subprocess.run(
            ["ps", "-p", str(pid), "-o", "command="],
            capture_output=True, text=True, timeout=2,
        )
        if "camera_service.py" in result.stdout:
            return True
    except Exception:
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
    print(f"[CAMERA_SERVICE_MANAGER] gz-transport env for child: {describe_gz_env(apply_gz_env(os.environ.copy()))}")

    with open(LOG_FILE, "w") as out:
        # Inject the system Gazebo Python bindings into PYTHONPATH strictly
        # for this subprocess: solves the gz-transport ModuleNotFoundError
        # without polluting the venv or requiring the bindings to be
        # pip-installed into it (they're tied to the system Gazebo install --
        # apt-installed on Linux, Homebrew-installed on macOS).
        env = os.environ.copy()
        if sys.platform == "darwin":
            brew_prefix = os.environ.get("HOMEBREW_PREFIX", "/opt/homebrew")
            system_paths = f"{brew_prefix}/lib/python{sys.version_info.major}.{sys.version_info.minor}/site-packages"
        else:
            system_paths = "/usr/lib/python3/dist-packages"
        env["PYTHONPATH"] = f"{system_paths}:{env['PYTHONPATH']}" if "PYTHONPATH" in env else system_paths

        # Pass the gz-transport settings EXPLICITLY rather than relying on
        # inheritance: if this process was started without them (e.g. launched
        # straight from an IDE or a shell that never sourced gz_env.sh), the
        # child would fall back to gz-transport's default
        # "<hostname>:<username>" partition, land in a different partition
        # from the simulator, and silently receive zero frames.
        apply_gz_env(env)

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
