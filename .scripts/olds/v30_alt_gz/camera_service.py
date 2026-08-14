import threading
import numpy as np
import cv2
import os
import time
import zmq
import argparse

# Ensure protobuf python implementation to avoid descriptor collisions with other libraries
os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")

import sys
import traceback

print("\n--- VALIDATION START ---")
print(f"[DEBUG] camera_service process starts. PID: {os.getpid()}, Thread ID: {threading.get_ident()}")
print(f"[DEBUG] Executable: {sys.executable}")
print(f"[DEBUG] Python version: {sys.version.split()[0]}")
print(f"[DEBUG] PYTHONPATH: {os.environ.get('PYTHONPATH', 'None')}")
print(f"[DEBUG] GZ_IP: {os.environ.get('GZ_IP', 'None')}")
print(f"[DEBUG] PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION: {os.environ.get('PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION', 'None')}")
print(f"[DEBUG] Environment variables dump: {dict(os.environ)}")
print(f"[DEBUG] sys.path: {sys.path}")

try:
    from gz.transport13 import Node
    from gz.msgs11.image_pb2 import Image as ImageMsg
    print("[DEBUG] Gazebo imports successful: from gz.transport13 import Node, from gz.msgs11.image_pb2 import Image")
except ImportError as e:
    # Previously this printed a traceback and fell through: the module kept
    # executing, and the CameraService class body below references ImageMsg
    # as a type hint, which then crashed with an unrelated "NameError:
    # ImageMsg is not defined" -- masking the real cause (missing gz bindings
    # / wrong PYTHONPATH) behind a confusing secondary error.
    print(f"[CAMERA_SERVICE] FATAL: Gazebo Python bindings not importable: {e}")
    print("[CAMERA_SERVICE] Fix: ensure PYTHONPATH includes /usr/lib/python3/dist-packages "
          "(process_manager.py sets this automatically; running camera_service.py directly "
          "without it will hit this exact error).")
    traceback.print_exc()
    sys.exit(1)
except Exception as e:
    print(f"[CAMERA_SERVICE] FATAL: Unexpected error importing Gazebo bindings: {e}")
    traceback.print_exc()
    sys.exit(1)
print("--- VALIDATION END ---\n")

from config import DEFAULT_GZ_TOPIC, CAMERA_ZMQ_ADDRESS

def resolve_camera_topic(preferred_topic: str, timeout_s: float = 10.0) -> str:
    """
    Auto-discovers the live Gazebo mono-camera image topic via `gz topic -l`,
    falling back to the configured DEFAULT_GZ_TOPIC if discovery is inconclusive.

    The topic path is model-instance-specific
    (.../model/<model_name>_0/link/camera_link/sensor/camera/image), and the
    exact <model_name> depends on which world/model make target was used to
    launch PX4 SITL (e.g. 'x500_mono_cam_down' vs
    'x500_mono_cam_down_payload'). Hardcoding one variant in config.py means
    the camera silently produces zero frames the moment someone launches the
    other variant -- subscribe() still returns True even when nothing is
    publishing. Matching by suffix instead of the full hardcoded path makes
    this resilient to which variant is actually running.
    """
    import subprocess
    env = os.environ.copy()
    env.setdefault("GZ_IP", "127.0.0.1")
    try:
        result = subprocess.run(["gz", "topic", "-l"], env=env, capture_output=True, text=True, timeout=timeout_s)
        topics = [t.strip() for t in result.stdout.splitlines() if t.strip()]
    except Exception as e:
        print(f"[CAMERA_SERVICE] WARNING: 'gz topic -l' failed ({e}); using configured topic as-is.")
        return preferred_topic

    if preferred_topic in topics:
        return preferred_topic

    suffix = "/link/camera_link/sensor/camera/image"
    candidates = [t for t in topics if t.endswith(suffix)]
    if len(candidates) == 1:
        print(f"[CAMERA_SERVICE] Configured topic not live ({preferred_topic}). "
              f"Auto-discovered live camera topic instead: {candidates[0]}")
        return candidates[0]
    if len(candidates) > 1:
        print(f"[CAMERA_SERVICE] WARNING: multiple live camera topics found {candidates}; "
              f"using the first one: {candidates[0]}")
        return candidates[0]

    print(f"[CAMERA_SERVICE] WARNING: no live camera topic found via 'gz topic -l'. "
          f"Falling back to configured topic (will likely produce zero frames): {preferred_topic}")
    return preferred_topic

class CameraService:
    """
    Subscribes to Gazebo Harmonic camera topic and publishes frames over ZeroMQ.
    This runs as a standalone process to isolate Gazebo/protobuf dependencies.
    """
    def __init__(self, topic: str, zmq_addr: str):
        self.topic = topic
        self.zmq_addr = zmq_addr
        
        print(f"[DEBUG] Creating Node()... Thread ID: {threading.get_ident()}")
        try:
            self.node = Node()
            print(f"[DEBUG] Node object repr: {repr(self.node)}")
        except Exception as e:
            print(f"[DEBUG] Exception creating Node(): {e}")
            traceback.print_exc()
            raise
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        
        # Setup ZeroMQ Publisher
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        
        # Optional: set high water mark to drop old frames if not consumed fast enough
        self.socket.setsockopt(zmq.SNDHWM, 2)
        
        self.frame_count = 0
        self.last_fps_time = time.time()
        self.running = False

    def _cb(self, msg: ImageMsg):
        try:
            now_time = time.time()
            if not hasattr(self, 'cb_count'):
                self.cb_count = 0
                self.cb_start_time = now_time
            self.cb_count += 1
            if now_time - self.cb_start_time >= 1.0:
                fps = self.cb_count / (now_time - self.cb_start_time)
                print(f"[CAMERA_SERVICE] Gazebo CB at {fps:.1f} FPS | ts: {now_time:.2f}")
                self.cb_count = 0
                self.cb_start_time = now_time

            if not msg.data or msg.width == 0 or msg.height == 0:
                print("[DEBUG] Empty frame ignored.")
                return
            
            W, H = msg.width, msg.height
            step = msg.step if getattr(msg, "step", 0) else W * 3
            rowb = W * 3
            
            buf = np.frombuffer(msg.data, dtype=np.uint8)
            if buf.size < H * step:
                print(f"[CAMERA_SERVICE] ERROR: Incomplete buffer received. Expected {H*step}, got {buf.size}")
                return
                
            img_step = buf.reshape((H, step))
            rgb = img_step[:, :rowb].reshape((H, W, 3))
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            
            with self.frame_lock:
                self.latest_frame = bgr
                
        except Exception as e:
            print(f"[CAMERA_SERVICE] ERROR in frame callback: {e}")
            traceback.print_exc()

    def start(self):
        print(f"[DEBUG] start() called. Thread ID: {threading.get_ident()}")
        print(f"[CAMERA_SERVICE] Binding ZMQ publisher to {self.zmq_addr}...")
        try:
            print(f"[DEBUG] Calling socket.bind() on {self.zmq_addr}...")
            self.socket.bind(self.zmq_addr)
            print("[DEBUG] socket.bind() successful.")
        except Exception as e:
            print(f"[CAMERA_SERVICE] FATAL ERROR binding ZMQ: {e}")
            traceback.print_exc()
            raise

        print(f"[CAMERA_SERVICE] Subscribing to Gazebo topic: {self.topic}...")
        print("[DEBUG] Calling subscribe()...")
        try:
            sub_res = self.node.subscribe(ImageMsg, self.topic, self._cb)
            print(f"[DEBUG] subscribe return value: {sub_res}")
        except Exception as e:
            print(f"[DEBUG] Exception during subscribe(): {e}")
            traceback.print_exc()
            raise

        if not sub_res:
            print(f"[CAMERA_SERVICE] FATAL ERROR: Subscribe failed for topic {self.topic}.")
            print("[CAMERA_SERVICE] Please check if Gazebo is running and topic exists ('gz topic -l').")
            raise RuntimeError("Gazebo subscription failed.")
            
        self.running = True
        print("[CAMERA_SERVICE] Service is running.")
        
        # Main publisher loop
        try:
            while self.running:
                frame_to_send = None
                with self.frame_lock:
                    if self.latest_frame is not None:
                        frame_to_send = self.latest_frame.copy()
                        self.latest_frame = None  # Consume frame
                        
                if frame_to_send is not None:
                    # Encode frame as JPEG to save bandwidth, or send raw bytes. 
                    # For local IPC, raw bytes are very fast.
                    _, buffer = cv2.imencode('.jpg', frame_to_send, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
                    self.socket.send(buffer.tobytes())
                    
                    self.frame_count += 1
                    print(f"[DEBUG] frame transmitted. count: {self.frame_count}")
                    now = time.time()
                    if now - self.last_fps_time >= 1.0:
                        fps = self.frame_count / (now - self.last_fps_time)
                        print(f"[CAMERA_SERVICE] Publishing at {fps:.1f} FPS | Shape: {frame_to_send.shape}")
                        self.frame_count = 0
                        self.last_fps_time = now
                
                time.sleep(0.01) # Small sleep to prevent CPU spin
                if self.frame_count % 100 == 0 and self.frame_count > 0:
                    print("[DEBUG] publisher loop alive")
                
        except KeyboardInterrupt:
            print("\n[CAMERA_SERVICE] Keyboard interrupt received.")
        except Exception as e:
            print(f"[CAMERA_SERVICE] Unexpected error in publisher loop: {e}")
            traceback.print_exc()
        finally:
            self.stop()

    def stop(self):
        print("[CAMERA_SERVICE] Stopping service...")
        self.running = False
        self.socket.close()
        self.context.term()
        print("[CAMERA_SERVICE] Shutdown complete.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Standalone Gazebo Camera Service")
    parser.add_argument("--topic", type=str, default=DEFAULT_GZ_TOPIC, help="Gazebo image topic")
    parser.add_argument("--zmq-addr", type=str, default=CAMERA_ZMQ_ADDRESS, help="ZMQ Publish Address")
    parser.add_argument("--no-watchdog", action="store_true", help="Disable auto-reconnect watchdog loop")
    args = parser.parse_args()

    resolved_topic = resolve_camera_topic(args.topic)
    service = CameraService(resolved_topic, args.zmq_addr)
    
    if args.no_watchdog:
        try:
            service.start()
        except Exception as e:
            print(f"[DEBUG] Exception in main loop (watchdog disabled): {e}")
            traceback.print_exc()
            sys.exit(1)
    else:
        # Auto-reconnect loop wrapper for Gazebo restarts
        while True:
            try:
                service.start()
            except RuntimeError as e:
                print(f"[CAMERA_SERVICE] Retrying Gazebo connection in 5 seconds... {e}")
                time.sleep(5.0)
            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"[DEBUG] Exception in main loop: {e}")
                traceback.print_exc()
                sys.exit(1)
