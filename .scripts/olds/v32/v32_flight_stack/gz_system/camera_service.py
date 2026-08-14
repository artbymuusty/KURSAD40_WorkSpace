"""
Ported from V30/V31 (.scripts/olds/v32/camera_service.py, byte-identical
back to v31_3rd_mission) -- the previous v32_flight_stack GzCameraSource
never actually subscribed to Gazebo at all; it just set a one-time
np.zeros() black frame in start() and never updated it again, which is why
the Mission Operations Center's camera panel showed solid black instead of
the real simulated feed.

Runs as a SEPARATE OS process, deliberately: gz-transport's protobuf
bindings can collide with other protobuf usage in the main mission process
(MAVSDK's gRPC client, EventStore's own serialization, etc.) -- isolating
Gazebo's Python bindings here and bridging frames out over ZeroMQ avoids
that entirely, and also survives a Gazebo restart without taking the
mission process down with it. camera_service_manager.py owns launching and
supervising this process; GzCameraSource (gz_camera_source.py) is the
ZMQ-consuming client side (via camera_client.CameraClient).
"""
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

DEFAULT_GZ_TOPIC = "/world/default/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image"
DEFAULT_ZMQ_ADDRESS = "tcp://127.0.0.1:5555"

try:
    from gz.transport13 import Node
    from gz.msgs11.image_pb2 import Image as ImageMsg
except ImportError as e:
    print(f"[CAMERA_SERVICE] FATAL: Gazebo Python bindings not importable: {e}")
    print("[CAMERA_SERVICE] Fix: ensure PYTHONPATH includes /usr/lib/python3/dist-packages "
          "(camera_service_manager.py sets this automatically; running camera_service.py "
          "directly without it will hit this exact error).")
    traceback.print_exc()
    sys.exit(1)
except Exception as e:
    print(f"[CAMERA_SERVICE] FATAL: Unexpected error importing Gazebo bindings: {e}")
    traceback.print_exc()
    sys.exit(1)


def resolve_camera_topic(preferred_topic: str, timeout_s: float = 10.0) -> str:
    """
    Auto-discovers the live Gazebo mono-camera image topic via `gz topic -l`,
    falling back to the configured topic if discovery is inconclusive.

    The topic path is model-instance-specific
    (.../model/<model_name>_0/link/camera_link/sensor/camera/image), and the
    exact <model_name> depends on which world/model make target was used to
    launch PX4 SITL. Hardcoding one variant means the camera silently
    produces zero frames the moment someone launches a different variant --
    matching by suffix instead makes this resilient to which one is
    actually running.
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
    """Subscribes to a Gazebo camera topic and republishes frames over ZeroMQ."""

    def __init__(self, topic: str, zmq_addr: str):
        self.topic = topic
        self.zmq_addr = zmq_addr

        self.node = Node()
        self.latest_frame = None
        self.frame_lock = threading.Lock()

        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.setsockopt(zmq.SNDHWM, 2)  # drop old frames if not consumed fast enough

        self.frame_count = 0
        self.last_fps_time = time.time()
        self.running = False

    def _cb(self, msg: "ImageMsg"):
        try:
            if not msg.data or msg.width == 0 or msg.height == 0:
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
        print(f"[CAMERA_SERVICE] Binding ZMQ publisher to {self.zmq_addr}...")
        try:
            self.socket.bind(self.zmq_addr)
        except Exception as e:
            print(f"[CAMERA_SERVICE] FATAL ERROR binding ZMQ: {e}")
            traceback.print_exc()
            raise

        print(f"[CAMERA_SERVICE] Subscribing to Gazebo topic: {self.topic}...")
        try:
            sub_res = self.node.subscribe(ImageMsg, self.topic, self._cb)
        except Exception as e:
            print(f"[CAMERA_SERVICE] Exception during subscribe(): {e}")
            traceback.print_exc()
            raise

        if not sub_res:
            print(f"[CAMERA_SERVICE] FATAL ERROR: Subscribe failed for topic {self.topic}.")
            print("[CAMERA_SERVICE] Please check if Gazebo is running and topic exists ('gz topic -l').")
            raise RuntimeError("Gazebo subscription failed.")

        self.running = True
        print("[CAMERA_SERVICE] Service is running.")

        try:
            while self.running:
                frame_to_send = None
                with self.frame_lock:
                    if self.latest_frame is not None:
                        frame_to_send = self.latest_frame.copy()
                        self.latest_frame = None  # consume

                if frame_to_send is not None:
                    _, buffer = cv2.imencode('.jpg', frame_to_send, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
                    self.socket.send(buffer.tobytes())

                    self.frame_count += 1
                    now = time.time()
                    if now - self.last_fps_time >= 1.0:
                        fps = self.frame_count / (now - self.last_fps_time)
                        print(f"[CAMERA_SERVICE] Publishing at {fps:.1f} FPS | Shape: {frame_to_send.shape}")
                        self.frame_count = 0
                        self.last_fps_time = now

                time.sleep(0.01)  # small sleep to prevent CPU spin
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
    parser.add_argument("--zmq-addr", type=str, default=DEFAULT_ZMQ_ADDRESS, help="ZMQ Publish Address")
    parser.add_argument("--no-watchdog", action="store_true", help="Disable auto-reconnect watchdog loop")
    args = parser.parse_args()

    resolved_topic = resolve_camera_topic(args.topic)
    service = CameraService(resolved_topic, args.zmq_addr)

    if args.no_watchdog:
        try:
            service.start()
        except Exception:
            traceback.print_exc()
            sys.exit(1)
    else:
        # Auto-reconnect loop wrapper for Gazebo restarts.
        while True:
            try:
                service.start()
            except RuntimeError as e:
                print(f"[CAMERA_SERVICE] Retrying Gazebo connection in 5 seconds... {e}")
                time.sleep(5.0)
            except KeyboardInterrupt:
                break
            except Exception:
                traceback.print_exc()
                sys.exit(1)
