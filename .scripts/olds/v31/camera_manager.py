import os
import sys
import subprocess
import time
import traceback

PID_FILE = "camera_service.pid"
LOG_FILE = "camera_service.log"
SERVICE_SCRIPT = "camera_service.py"

def is_running(pid):
    """Check if a process with the given PID is currently running and is the camera_service."""
    try:
        os.kill(pid, 0)
    except OSError:
        return False
        
    # Extra check to ensure it's not a recycled PID for a different process
    try:
        with open(f"/proc/{pid}/cmdline", "r") as f:
            cmdline = f.read().replace('\x00', ' ')
            if SERVICE_SCRIPT in cmdline:
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

def start():
    print("[1/4] Checking Camera Service...")
    pid = get_pid()
    
    if pid and is_running(pid):
        print(f"[ OK ] Camera Service Running (PID: {pid})")
        return

    if pid:
        # PID exists but process is dead. Clean up.
        print("      Stale PID file detected. Cleaning up...")
        os.remove(PID_FILE)

    print("      Starting Camera Service...")
    
    # Open log file in write mode to truncate it (prevents infinite growth)
    with open(LOG_FILE, "w") as out:
        # Inject system dist-packages into PYTHONPATH strictly for CameraService.
        # This solves the ModuleNotFoundError without polluting the venv or 
        # requiring hardcoded paths inside the application source code.
        env = os.environ.copy()
        system_paths = "/usr/lib/python3/dist-packages"
        if "PYTHONPATH" in env:
            env["PYTHONPATH"] = f"{system_paths}:{env['PYTHONPATH']}"
        else:
            env["PYTHONPATH"] = system_paths
            
        print(f"[DEBUG] camera_manager starting camera_service with PYTHONPATH={env.get('PYTHONPATH', 'None')} GZ_IP={env.get('GZ_IP', 'None')} PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION={env.get('PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION', 'None')}")
        
        try:
            p = subprocess.Popen(
                [sys.executable, "-u", SERVICE_SCRIPT],
                stdout=out, 
                stderr=out,
                env=env,
                start_new_session=True, # Detach from terminal completely
            )
        except Exception as e:
            print(f"[DEBUG] Exception in subprocess.Popen (camera_manager): {e}")
            traceback.print_exc()
            raise
        
    with open(PID_FILE, "w") as f:
        f.write(str(p.pid))
        
    # Wait briefly to ensure it doesn't immediately crash
    time.sleep(1.0)
    if is_running(p.pid):
        print(f"[ OK ] Camera Service Started (PID: {p.pid})")
    else:
        print(f"[FAIL] Camera Service failed to start. Check {LOG_FILE} for details.")
        sys.exit(1)

def stop():
    print("[1/2] Stopping Camera Service...")
    pid = get_pid()
    
    if not pid or not is_running(pid):
        print("[ OK ] Camera Service is not running.")
        if os.path.exists(PID_FILE):
            os.remove(PID_FILE)
        return

    print(f"      Sending SIGTERM to PID {pid}...")
    try:
        os.kill(pid, 15) # SIGTERM
        
        # Wait up to 5 seconds for it to exit
        for _ in range(50):
            if not is_running(pid):
                break
            time.sleep(0.1)
            
        if is_running(pid):
            print(f"      Process did not exit cleanly. Sending SIGKILL to PID {pid}...")
            os.kill(pid, 9) # SIGKILL
            
        print("[ OK ] Camera Service Stopped.")
    except OSError as e:
        print(f"[FAIL] Failed to kill process: {e}")
        
    if os.path.exists(PID_FILE):
        os.remove(PID_FILE)

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python camera_manager.py [start|stop]")
        sys.exit(1)
        
    cmd = sys.argv[1].lower()
    
    # Change working directory to the directory of this script to ensure relative paths work
    script_dir = os.path.dirname(os.path.abspath(__file__))
    os.chdir(script_dir)
    
    if cmd == "start":
        start()
    elif cmd == "stop":
        stop()
    else:
        print(f"Unknown command: {cmd}")
        sys.exit(1)
