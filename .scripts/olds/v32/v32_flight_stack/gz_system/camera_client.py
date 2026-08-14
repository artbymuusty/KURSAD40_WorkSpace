"""
Ported from V30/V31 (.scripts/olds/v32/camera_client.py) largely unchanged --
this is proven, working code. Subscribes to camera_service.py over ZeroMQ
and maintains the latest decoded frame, thread-safe. Completely independent
of Gazebo/protobuf itself, so it can live in the main mission process
without the descriptor-collision problems camera_service.py's module
docstring describes.
"""
import threading
import time
import zmq
import cv2
import numpy as np
import traceback


class CameraClient:
    def __init__(self, zmq_addr: str):
        self.zmq_addr = zmq_addr

        self.latest_frame = None
        self.frame_lock = threading.Lock()

        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.setsockopt(zmq.RCVHWM, 2)
        self.socket.setsockopt(zmq.SUBSCRIBE, b"")
        try:
            self.socket.setsockopt(zmq.CONFLATE, 1)  # keep only the latest message, zero lag
        except AttributeError:
            pass  # older pyzmq versions might not have CONFLATE

        self.running = False
        self.thread = None
        self.last_receive_time = time.time()
        self.reconnect_requested = False

    def _receive_loop(self):
        print(f"[CAMERA_CLIENT] Connecting to ZMQ publisher at {self.zmq_addr}...")
        try:
            self.socket.connect(self.zmq_addr)
        except Exception as e:
            print(f"[CAMERA_CLIENT] FATAL ERROR connecting to ZMQ: {e}")
            self.running = False
            return

        print("[CAMERA_CLIENT] Connected. Waiting for frames...")

        while self.running:
            if self.reconnect_requested:
                print(f"[CAMERA_CLIENT] Recreating ZMQ socket for {self.zmq_addr}...")
                try:
                    self.socket.close()
                    self.socket = self.context.socket(zmq.SUB)
                    self.socket.setsockopt(zmq.RCVHWM, 2)
                    self.socket.setsockopt(zmq.SUBSCRIBE, b"")
                    try:
                        self.socket.setsockopt(zmq.CONFLATE, 1)
                    except AttributeError:
                        pass
                    self.socket.connect(self.zmq_addr)
                    self.reconnect_requested = False
                    self.last_receive_time = time.time()
                    print("[CAMERA_CLIENT] Reconnection successful.")
                except Exception as e:
                    print(f"[CAMERA_CLIENT] Reconnection failed: {e}")
                    time.sleep(1.0)
                    continue

            try:
                if self.socket.poll(100):
                    raw_msg = self.socket.recv()
                    np_arr = np.frombuffer(raw_msg, np.uint8)
                    frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

                    if frame is not None:
                        with self.frame_lock:
                            self.latest_frame = frame
                            self.last_receive_time = time.time()
                    else:
                        print("[CAMERA_CLIENT] Warning: Failed to decode received frame.")
            except zmq.ZMQError as e:
                print(f"[CAMERA_CLIENT] ZMQ Error: {e}")
                traceback.print_exc()
                time.sleep(1.0)
            except Exception as e:
                print(f"[CAMERA_CLIENT] Unexpected error in receive loop: {e}")
                traceback.print_exc()
                time.sleep(0.1)

    def start(self):
        self.running = True
        self.last_receive_time = time.time()  # avoid an immediate reconnect
        self.thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.thread.start()

    def stop(self):
        print("[CAMERA_CLIENT] Stopping client...")
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=2.0)
        try:
            self.socket.close()
            self.context.term()
        except Exception:
            pass
        print("[CAMERA_CLIENT] Shutdown complete.")

    def get_frame(self):
        """Thread-safe copy of the latest frame, or None if none received
        yet / the stream has gone stale (>2s since last frame)."""
        with self.frame_lock:
            if time.time() - self.last_receive_time > 2.0:
                if not self.reconnect_requested:
                    print("[CAMERA_CLIENT] WARNING: Frame stream is stale. Reconnecting...")
                    self.reconnect_requested = True
                self.latest_frame = None
                return None

            if self.latest_frame is None:
                return None

            return self.latest_frame.copy()
