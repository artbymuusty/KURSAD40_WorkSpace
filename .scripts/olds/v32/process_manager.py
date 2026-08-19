import os
import sys
import subprocess
import time
import signal
import traceback
import importlib.util
import threading

LOGS_DIR = "logs"
PID_CAMERA = "camera_service.pid"
PID_MISSION = "mission.pid"
PID_VIEWER = "viewer.pid"

def _gz_child_env():
    """Environment for a child process that talks to Gazebo.

    Replaces the previous hardcoded "/usr/lib/python3/dist-packages" (Debian
    apt layout) with the location appropriate to the running platform, and
    pins GZ_PARTITION/GZ_IP so the child lands in the SAME gz-transport
    partition as the simulator -- gz-transport's default partition is
    "<hostname>:<username>", and a DHCP-derived hostname changing mid-session
    silently put processes in different partitions, yielding zero frames.
    """
    env = os.environ.copy()
    if sys.platform == "darwin":
        brew_prefix = env.get("HOMEBREW_PREFIX", "/opt/homebrew")
        sys_paths = f"{brew_prefix}/lib/python{sys.version_info.major}.{sys.version_info.minor}/site-packages"
    else:
        sys_paths = "/usr/lib/python3/dist-packages"
    env["PYTHONPATH"] = f"{sys_paths}:{env.get('PYTHONPATH', '')}"
    env.setdefault("GZ_PARTITION", "kursad40")
    env.setdefault("GZ_IP", "127.0.0.1")
    return env


def _ensure_logs_dir():
    os.makedirs(LOGS_DIR, exist_ok=True)

def _rotate_log(log_filename):
    _ensure_logs_dir()
    filepath = os.path.join(LOGS_DIR, log_filename)
    if os.path.exists(filepath):
        backup1 = f"{filepath}.1"
        backup2 = f"{filepath}.2"
        if os.path.exists(backup1):
            if os.path.exists(backup2):
                os.remove(backup2)
            os.rename(backup1, backup2)
        os.rename(filepath, backup1)

def is_running(pid, script_name=None):
    try:
        os.kill(pid, 0)
    except OSError:
        return False
        
    if script_name:
        try:
            with open(f"/proc/{pid}/cmdline", "r") as f:
                cmdline = f.read().replace('\x00', ' ')
                if script_name in cmdline:
                    return True
        except FileNotFoundError:
            pass
        return False
    return True

def get_pid(pid_file):
    if not os.path.exists(pid_file):
        return None
    try:
        with open(pid_file, "r") as f:
            return int(f.read().strip())
    except ValueError:
        return None

def write_pid(pid_file, pid):
    with open(pid_file, "w") as f:
        f.write(str(pid))

def clear_pid(pid_file):
    if os.path.exists(pid_file):
        os.remove(pid_file)

def unpause_gazebo():
    print("[ INFO ] Unpausing Gazebo simulation...")
    env = os.environ.copy()
    env["GZ_IP"] = "127.0.0.1"
    try:
        # Don't block forever if Gazebo isn't up
        result = subprocess.run(
            ["gz", "service", "-s", "/world/default/control",
             "--reqtype", "gz.msgs.WorldControl",
             "--reptype", "gz.msgs.Boolean",
             "--timeout", "2000",
             "--req", "pause: false"],
            env=env, capture_output=True, text=True
        )
        if "data: true" in result.stdout or result.returncode == 0:
            print("  [ OK ] Gazebo unpaused successfully.")
        else:
            print(f"  [ WARN ] Could not unpause Gazebo. Output: {result.stdout.strip()}")
    except Exception as e:
        print(f"  [ WARN ] Error unpausing Gazebo: {e}")

def verify_gazebo_ready(timeout_s: float = 60.0) -> bool:
    """
    Confirms PX4 SITL + Gazebo are actually up and publishing the exact camera
    topic camera_service.py will subscribe to, BEFORE spawning camera_service
    and main.py against a simulator that may not exist or may be running the
    wrong world/model variant.

    Without this check, run_mission_v30 previously proceeded unconditionally:
    camera_service.py would subscribe successfully (subscribe() returns True
    even if nothing is publishing) and then spin forever with zero frames,
    and main.py's own output is captured to logs/mission.log rather than the
    terminal (see start_mission()) -- from the terminal this looked exactly
    like a silent hang ("Launching Mission (main.py)... " then nothing),
    with no indication of what was actually wrong.
    """
    from config import DEFAULT_GZ_TOPIC

    print("[ INFO ] Verifying Gazebo camera topic is live...")
    env = os.environ.copy()
    env["GZ_IP"] = "127.0.0.1"

    # Match by suffix, not the full hardcoded path: the model-instance name
    # segment depends on which world/model make target launched PX4 SITL
    # (e.g. 'x500_mono_cam_down' vs 'x500_mono_cam_down_payload'), and
    # camera_service.py's own resolve_camera_topic() does the same fallback --
    # requiring an exact match here would report "not ready" even when a
    # perfectly live camera topic exists under a different model name.
    camera_suffix = "/link/camera_link/sensor/camera/image"

    t0 = time.time()
    last_topics = ""
    last_progress_print = 0.0
    try:
        while time.time() - t0 < timeout_s:
            try:
                result = subprocess.run(
                    ["gz", "topic", "-l"],
                    env=env, capture_output=True, text=True, timeout=5.0
                )
                last_topics = result.stdout
                topics = last_topics.splitlines()
                if DEFAULT_GZ_TOPIC in topics:
                    print(f"  [ OK ] Camera topic is live: {DEFAULT_GZ_TOPIC}")
                    return True
                matches = [t for t in topics if t.strip().endswith(camera_suffix)]
                if matches:
                    print(f"  [ OK ] Camera topic is live (different model variant than configured): {matches[0]}")
                    return True
            except Exception:
                pass

            elapsed = time.time() - t0
            if elapsed - last_progress_print >= 5.0:
                print(f"  [ ... ] Still waiting for camera topic ({elapsed:.0f}s / {timeout_s:.0f}s)."
                      f" Gazebo cold-start (mesh/physics/render init) can take a while,"
                      f" especially without GPU acceleration.")
                last_progress_print = elapsed
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\n[ INFO ] Cancelled while waiting for Gazebo.")
        sys.exit(130)

    print("\n--- GAZEBO NOT READY ---")
    print(f"  Expected camera topic was never seen: {DEFAULT_GZ_TOPIC}")
    print("  This means PX4 SITL + Gazebo is either not running, or is running")
    print("  a different world/model than the one this mission expects.")
    print("  Fix: start the simulator first in its own terminal, then re-run:")
    print("    ./safe_sitl_launcher.sh")
    print("  (it launches 'make px4_sitl gz_x500_mono_cam_down_payload')")
    if last_topics:
        print("\n  Topics that ARE currently advertised (for comparison):")
        for line in last_topics.splitlines():
            if "camera" in line or "image" in line:
                print(f"    {line}")
    print("------------------------\n")
    return False

def verify_vehicle_armable(timeout_s: float = 12.0) -> bool:
    """Checks PX4's telemetry.health().is_armable via check_armable.py.

    This is the same flag PX4's own health_and_arming_checks module drives (the
    one printing 'Preflight Fail: Attitude failure (pitch)' etc. to its console).
    A crashed/toppled airframe left over in a Gazebo world that was never reset
    keeps publishing its camera topic fine (verify_gazebo_ready() alone can't see
    the problem), but PX4 will still correctly refuse to arm against it -- this
    catches that case before wasting a whole mission attempt on it.

    Run as a subprocess (check_armable.py), not imported in-process: by the time
    this runs, check_health() above has already imported gz.transport13 into
    this interpreter, and mavsdk's protobuf-generated stubs cannot coexist with
    gz-transport13's protobuf runtime in the same process (see check_armable.py
    launched as separate subprocesses rather than imported directly.
    """
    print("[ INFO ] Checking PX4 arming health (detecting a stale/crashed world)...")
    env = os.environ.copy()
    env.pop("PYTHONPATH", None)

    try:
        proc = subprocess.run(
            [sys.executable, "-u", "check_armable.py", str(timeout_s)],
            capture_output=True, text=True, timeout=timeout_s + 5, env=env
        )
        output = proc.stdout.strip()
    except Exception as e:
        print(f"  [ WARN ] Could not run arming-health probe ({e}). Proceeding anyway.")
        return True

    if "ARMABLE=True" in output:
        print("  [ OK ] PX4 reports vehicle is armable.")
        return True

    if "ARMABLE=False" in output:
        print("\n--- STALE / CRASHED WORLD DETECTED ---")
        print("  PX4 reports the vehicle is NOT armable (telemetry.health().is_armable == False).")
        print("  This almost always means the airframe in the current Gazebo world is already")
        print("  crashed/toppled from a previous run, and PX4 is correctly refusing to arm")
        print("  against it (PX4's own console would show e.g.")
        print("  'Preflight Fail: Attitude failure (pitch)' / 'Arming denied: Resolve system")
        print("  health failures first'). Launching main.py here would just repeat that failure.")
        print("  Fix: reset the simulator before retrying:")
        print("    ./safe_sitl_launcher.sh")
        print("---------------------------------------\n")
        return False

    print(f"  [ WARN ] Could not confirm PX4 arming health ({output or proc.stderr.strip()}). Proceeding anyway.")
    return True

def check_health():
    print("[1/2] Executing Pre-flight Health Checks...")
    errors = []

    # 1. Python Venv
    if not hasattr(sys, 'real_prefix') and sys.base_prefix == sys.prefix:
        errors.append("✓ Python environment: FAILED\n  Why: Virtual environment is not active.\n  Fix: Run 'source .venv/bin/activate'")
    else:
        print("  ✓ Python Virtual Environment")

    # 2. Required pip packages
    for pkg in ["cv2", "zmq", "mavsdk", "ultralytics"]:
        if importlib.util.find_spec(pkg) is None:
            errors.append(f"✓ {pkg} module: FAILED\n  Why: Not installed.\n  Fix: Run 'pip install {pkg}'")
    if not errors:
        print("  ✓ Python dependencies (cv2, zmq, mavsdk, ultralytics)")

    # 2b. YOLO weights file must actually be present (ultralytics will otherwise
    # try to auto-download it, which silently fails/hangs without internet access
    # and previously left the detector permanently returning zero detections).
    from config import YOLO_WEIGHTS_PATH
    if not os.path.exists(YOLO_WEIGHTS_PATH):
        errors.append(
            f"✓ YOLO weights: FAILED\n  Why: '{YOLO_WEIGHTS_PATH}' not found in {os.getcwd()}.\n"
            f"  Fix: place the trained weights file there, or run once with internet access "
            f"to let ultralytics download the stock model."
        )
    else:
        print(f"  ✓ YOLO weights file present ({YOLO_WEIGHTS_PATH})")

    # 3. Gazebo Bindings
    #
    # Ask the interpreter that will actually run the pipeline whether it can
    # import the bindings, instead of testing for a fixed distro path. The old
    # check appended /usr/lib/python3/dist-packages (Debian/apt layout) and
    # reported failure in terms of apt packages, which is meaningless on macOS
    # where Homebrew installs them elsewhere -- it failed even when the
    # bindings were present and working.
    probe = subprocess.run(
        [sys.executable, "-c", "import gz.transport13, gz.msgs10"],
        capture_output=True, text=True, env=os.environ.copy(),
    )
    if probe.returncode == 0:
        print("  ✓ Gazebo Python Bindings (gz.transport13, gz.msgs10)")
    else:
        errors.append(
            "✓ Gazebo bindings: FAILED\n"
            f"  Why: '{sys.executable}' cannot import gz.transport13 / gz.msgs10.\n"
            f"  Detail: {probe.stderr.strip().splitlines()[-1] if probe.stderr.strip() else 'no error output'}\n"
            "  Fix: ensure PYTHONPATH includes the system Gazebo Python bindings\n"
            "       for this interpreter (they ship with the Gazebo install, not pip),\n"
            "       and that you are running inside the KURSAD40 environment."
        )

    if errors:
        print("\n--- PRE-FLIGHT CHECK FAILED ---")
        for err in errors:
            print(err)
        sys.exit(1)
    
    print("[ OK ] Health checks passed.\n")

def print_status():
    print("=====================================")
    print(" KURSAD40 V30 - Service Status")
    print("=====================================")
    
    services = [
        ("Camera Service", PID_CAMERA, "camera_service.py"),
        ("Camera Viewer", PID_VIEWER, "viewer.py"),
        ("Mission (main.py)", PID_MISSION, "main.py")
    ]
    
    for name, pid_file, script in services:
        pid = get_pid(pid_file)
        if pid and is_running(pid, script):
            print(f"  [ RUNNING ] {name:<20} (PID: {pid})")
        else:
            print(f"  [ STOPPED ] {name:<20}")
            
    print("=====================================")

def start_camera_service():
    pid = get_pid(PID_CAMERA)
    if pid and is_running(pid, "camera_service.py"):
        print(f"[ OK ] Camera Service is already running (PID: {pid}). Reusing it.")
        return
    elif pid:
        print("[ INFO ] Cleaning up stale Camera Service PID.")
        clear_pid(PID_CAMERA)

    print("[ INFO ] Starting Camera Service...")
    _rotate_log("camera_service.log")
    
    env = _gz_child_env()
    
    out = open(os.path.join(LOGS_DIR, "camera_service.log"), "w")
    print(f"[DEBUG] process_manager starting camera_service with PYTHONPATH={env['PYTHONPATH']} GZ_IP={env.get('GZ_IP')} PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION={env.get('PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION', 'None')}")
    try:
        p = subprocess.Popen(
            [sys.executable, "-u", "camera_service.py"],
            stdout=out, stderr=out,
            start_new_session=True,
            env=env
        )
    except Exception as e:
        print(f"[DEBUG] Exception in subprocess.Popen: {e}")
        traceback.print_exc()
        raise
    write_pid(PID_CAMERA, p.pid)
    time.sleep(1)
    if is_running(p.pid, "camera_service.py"):
        print(f"[ OK ] Camera Service started (PID: {p.pid})")
    else:
        print("[ FAIL ] Camera Service failed to start. Check logs/camera_service.log")
        sys.exit(1)


def kill_process(pid_file, script_name, friendly_name):
    pid = get_pid(pid_file)
    if not pid or not is_running(pid, script_name):
        clear_pid(pid_file)
        return False

    print(f"[ INFO ] Stopping {friendly_name} (PID: {pid})...")
    try:
        os.kill(pid, 15)
        for _ in range(50):
            if not is_running(pid, script_name):
                break
            time.sleep(0.1)
        if is_running(pid, script_name):
            print(f"[ WARN ] {friendly_name} didn't stop. Sending SIGKILL...")
            os.kill(pid, 9)
    except OSError:
        pass
    
    clear_pid(pid_file)
    print(f"[ OK ] {friendly_name} stopped.")
    return True

def start_cam():
    check_health()
    _rotate_log("startup.log")
    unpause_gazebo()

    if not verify_gazebo_ready():
        print("[ ERROR ] Aborting: Gazebo/PX4 SITL is not ready.")
        sys.exit(1)

    start_camera_service()
    
    pid = get_pid(PID_VIEWER)
    if pid and is_running(pid, "viewer.py"):
        print(f"[ OK ] Viewer is already running (PID: {pid}).")
        sys.exit(0)
    elif pid:
        clear_pid(PID_VIEWER)
        
    print("[ INFO ] Launching Camera Viewer...")
    _rotate_log("viewer.log")
    
    out = open(os.path.join(LOGS_DIR, "viewer.log"), "w")
    p = subprocess.Popen(
        [sys.executable, "-u", "viewer.py"],
        stdout=out, stderr=out
    )
    write_pid(PID_VIEWER, p.pid)

def stream_logs(pipe, prefix):
    for line in iter(pipe.readline, ''):
        if not line:
            break
        print(f"[{prefix}] {line}", end='')
    pipe.close()

def start_foreground_cam():
    check_health()
    _rotate_log("startup.log")
    unpause_gazebo()

    if not verify_gazebo_ready():
        print("[ ERROR ] Aborting: Gazebo/PX4 SITL is not ready.")
        sys.exit(1)

    print("[ INFO ] Starting Camera Service in foreground...")
    
    pid_cam = get_pid(PID_CAMERA)
    if pid_cam and is_running(pid_cam, "camera_service.py"):
        print("[ WARN ] Camera Service is already running in background. Stopping it first...")
        kill_process(PID_CAMERA, "camera_service.py", "Camera Service")
        
    pid_view = get_pid(PID_VIEWER)
    if pid_view and is_running(pid_view, "viewer.py"):
        print("[ WARN ] Viewer is already running in background. Stopping it first...")
        kill_process(PID_VIEWER, "viewer.py", "Viewer")

    env = _gz_child_env()

    cam_p = subprocess.Popen(
        [sys.executable, "-u", "camera_service.py"],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        env=env, text=True, bufsize=1
    )
    write_pid(PID_CAMERA, cam_p.pid)

    viewer_p = subprocess.Popen(
        [sys.executable, "-u", "viewer.py"],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, bufsize=1
    )
    write_pid(PID_VIEWER, viewer_p.pid)

    cam_thread = threading.Thread(target=stream_logs, args=(cam_p.stdout, "CAMERA"), daemon=True)
    viewer_thread = threading.Thread(target=stream_logs, args=(viewer_p.stdout, "VIEWER"), daemon=True)
    cam_thread.start()
    viewer_thread.start()

    print("[ INFO ] Services started. Press Ctrl+C to stop.")

    def shutdown_gracefully(signum, frame):
        print("\n[ INFO ] Received stop signal. Shutting down gracefully...")
        # Restore default handlers to prevent recursive signals
        signal.signal(signal.SIGINT, signal.SIG_DFL)
        signal.signal(signal.SIGTERM, signal.SIG_DFL)
        
        for p, name in [(viewer_p, "Viewer"), (cam_p, "Camera Service")]:
            if p.poll() is None:
                print(f"[ INFO ] Stopping {name} (PID: {p.pid})...")
                p.terminate()

        for p, name in [(viewer_p, "Viewer"), (cam_p, "Camera Service")]:
            try:
                p.wait(timeout=5.0)
                print(f"[ OK ] {name} stopped.")
            except subprocess.TimeoutExpired:
                print(f"[ WARN ] {name} did not stop gracefully. Sending SIGKILL...")
                p.kill()
                p.wait()
                
        clear_pid(PID_VIEWER)
        clear_pid(PID_CAMERA)
        print("[ OK ] Shutdown complete.")
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown_gracefully)
    signal.signal(signal.SIGTERM, shutdown_gracefully)

    while cam_p.poll() is None or viewer_p.poll() is None:
        try:
            time.sleep(1)
        except KeyboardInterrupt:
            shutdown_gracefully(signal.SIGINT, None)

def stop_cam():
    kill_process(PID_VIEWER, "viewer.py", "Viewer")
    
    mission_pid = get_pid(PID_MISSION)
    if mission_pid and is_running(mission_pid, "main.py"):
        print("[ INFO ] Mission is currently running. Skipping Camera Service shutdown.")
    else:
        kill_process(PID_CAMERA, "camera_service.py", "Camera Service")

def start_mission():
    check_health()
    _rotate_log("startup.log")
    unpause_gazebo()

    if not verify_gazebo_ready():
        print("[ ERROR ] Aborting mission launch: Gazebo/PX4 SITL is not ready.")
        sys.exit(1)

    if not verify_vehicle_armable():
        print("[ ERROR ] Aborting mission launch: PX4 will not arm against this world.")
        sys.exit(1)

    start_camera_service()

    pid = get_pid(PID_MISSION)
    if pid and is_running(pid, "main.py"):
        print(f"[ ERROR ] Mission is already running (PID: {pid}). Aborting.")
        sys.exit(1)
    elif pid:
        clear_pid(PID_MISSION)

    print("[ INFO ] Launching Mission (main.py)...")
    _rotate_log("mission.log")

    # Tee main.py's output to both the log file AND this terminal. Previously
    # this only went to logs/mission.log, so the terminal running
    # run_mission_v30 showed nothing at all after "Launching Mission..." even
    # while main.py was fully alive and doing real work -- indistinguishable
    # from an actual hang. See stream_logs() below (same pattern already used
    # by start_foreground_cam()).
    out = open(os.path.join(LOGS_DIR, "mission.log"), "w")
    p = subprocess.Popen(
        [sys.executable, "-u", "main.py"] + sys.argv[2:],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1
    )
    write_pid(PID_MISSION, p.pid)

    def _tee_loop():
        for line in iter(p.stdout.readline, ''):
            if not line:
                break
            print(line, end='')
            out.write(line)
            out.flush()
        p.stdout.close()

    tee_thread = threading.Thread(target=_tee_loop, daemon=True)
    tee_thread.start()

    print("[ INFO ] Mission running. Press Ctrl+C to stop everything.")

    # Wait for the mission process to exit
    try:
        p.wait()
    except KeyboardInterrupt:
        print("\n[ INFO ] Keyboard interrupt detected. Sending stop signal...")
        os.kill(p.pid, 15)
        p.wait()
    tee_thread.join(timeout=2.0)
    out.close()
    clear_pid(PID_MISSION)

    # started with start_new_session=True (a separate process group), which
    # is exactly why the terminal's own Ctrl+C (SIGINT to the foreground
    # process group) never reaches them -- they were always meant to be torn
    # down explicitly here instead. That teardown call was missing (only
    # main.py's PID was ever killed above), so both services were left
    # running as orphans after every mission exit, whether via Ctrl+C or
    # normal completion. The next `./run_mission_v30` would then start a
    # one still holds; when that collision causes the new one to fail to
    # whole process_manager.py run before main.py is ever launched -- which
    # is what made the launcher look like it had stopped behaving as a
    # foreground application (it exits in ~1s with no mission log output).
    # kill_process() already existed and is used correctly elsewhere
    # (start_foreground_cam's shutdown_gracefully, stop_mission) -- this was
    # simply never wired up for the mission's own exit path.
    viewer_pid = get_pid(PID_VIEWER)
    if viewer_pid and is_running(viewer_pid, "viewer.py"):
        kill_process(PID_VIEWER, "viewer.py", "Viewer")
    kill_process(PID_CAMERA, "camera_service.py", "Camera Service")
    print("[ OK ] All mission services stopped.")

def stop_mission():
    kill_process(PID_MISSION, "main.py", "Mission")

    viewer_pid = get_pid(PID_VIEWER)
    if viewer_pid and is_running(viewer_pid, "viewer.py"):
        print("[ INFO ] Viewer is currently running. Skipping Camera Service shutdown.")
    else:
        kill_process(PID_CAMERA, "camera_service.py", "Camera Service")

def start_camera_test():
    print("[ INFO ] Launching Camera Diagnostics...")
    env = _gz_child_env()
    subprocess.call([sys.executable, "-u", "camera_test.py"], env=env)

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python process_manager.py [start_cam|foreground_cam|stop_cam|start_mission|stop_mission|start_test]")
        sys.exit(1)
        
    # Change working directory to ensure relative paths resolve
    script_dir = os.path.dirname(os.path.abspath(__file__))
    os.chdir(script_dir)
    
    cmd = sys.argv[1].lower()
    
    if cmd == "start_cam":
        start_cam()
    elif cmd == "foreground_cam":
        start_foreground_cam()
    elif cmd == "stop_cam":
        stop_cam()
    elif cmd == "start_mission":
        start_mission()
    elif cmd == "stop_mission":
        stop_mission()
    elif cmd == "start_test":
        start_camera_test()
    elif cmd == "status":
        print_status()
    elif cmd == "health":
        check_health()
    else:
        print(f"Unknown command: {cmd}")
        sys.exit(1)
