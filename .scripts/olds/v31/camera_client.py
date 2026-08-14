import threading
import time
import zmq
import cv2
import numpy as np
import traceback

class CameraClient:
    """
    Subscribes to the CameraService over ZeroMQ and maintains the latest frame.
    This class is completely independent of Gazebo and Protobuf, solving all 
    descriptor collision issues and avoiding crashes when Gazebo restarts.
    """
    def __init__(self, zmq_addr: str):
        self.zmq_addr = zmq_addr
        
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        # Set receive high water mark to keep latency low
        self.socket.setsockopt(zmq.RCVHWM, 2)
        # Subscribe to all messages (empty filter)
        self.socket.setsockopt(zmq.SUBSCRIBE, b"")
        
        # Conflate options to only keep the very last message in buffer (zero lag)
        try:
            self.socket.setsockopt(zmq.CONFLATE, 1)
        except AttributeError:
            pass # older pyzmq versions might not have CONFLATE

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
                # Use polling with timeout to allow clean shutdown
                if self.socket.poll(100):
                    raw_msg = self.socket.recv()
                    
                    # Decode jpeg byte string back to BGR numpy array
                    np_arr = np.frombuffer(raw_msg, np.uint8)
                    frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                    
                    if frame is not None:
                        with self.frame_lock:
                            self.latest_frame = frame
                            self.last_receive_time = time.time()
                            
                        if not hasattr(self, 'recv_count'):
                            self.recv_count = 0
                            self.recv_start_time = time.time()
                        self.recv_count += 1
                        now_time = time.time()
                        if now_time - self.recv_start_time >= 1.0:
                            fps = self.recv_count / (now_time - self.recv_start_time)
                            print(f"[CAMERA_CLIENT] Received at {fps:.1f} FPS | Shape: {frame.shape} | ts: {now_time:.2f}")
                            self.recv_count = 0
                            self.recv_start_time = now_time
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
        """Starts the background thread to continuously receive frames."""
        self.running = True
        self.last_receive_time = time.time() # Reset timer to avoid immediate reconnect
        self.thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.thread.start()

    def stop(self):
        """Stops the receiver thread safely."""
        print("[CAMERA_CLIENT] Stopping client...")
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=2.0)
        try:
            self.socket.close()
            self.context.term()
        except:
            pass
        print("[CAMERA_CLIENT] Shutdown complete.")

    def get_frame(self):
        """
        Returns a thread-safe copy of the latest frame.
        Returns None if no frames have been received or if frames are stale.
        """
        with self.frame_lock:
            # Warn if frames are stale (e.g. camera service crashed or ZMQ hung)
            if time.time() - self.last_receive_time > 2.0:
                if not self.reconnect_requested:
                    print("[CAMERA_CLIENT] WARNING: Frame stream is stale. Reconnecting...")
                    self.reconnect_requested = True
                self.latest_frame = None
                return None
                
            if self.latest_frame is None:
                return None
                
            return self.latest_frame.copy()

    def get_frame_with_time(self):
        """
        Returns a thread-safe copy of the latest frame and its receive timestamp.
        Returns (None, 0.0) if no frames have been received or if frames are stale.
        """
        with self.frame_lock:
            # Warn if frames are stale (e.g. camera service crashed or ZMQ hung)
            if time.time() - self.last_receive_time > 2.0:
                if not self.reconnect_requested:
                    print("[CAMERA_CLIENT] WARNING: Frame stream is stale. Reconnecting...")
                    self.reconnect_requested = True
                self.latest_frame = None
                return None, 0.0
                
            if self.latest_frame is None:
                return None, 0.0
                
            return self.latest_frame.copy(), self.last_receive_time


if __name__ == "__main__":
    from config import CAMERA_ZMQ_ADDRESS
    import argparse
    
    parser = argparse.ArgumentParser(description="Standalone ZMQ Camera Client Test")
    parser.add_argument("--zmq-addr", type=str, default=CAMERA_ZMQ_ADDRESS, help="ZMQ Publish Address")
    args = parser.parse_args()
    
    client = CameraClient(args.zmq_addr)
    client.start()
    
    cv2.namedWindow("Camera Client Test", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Camera Client Test", 640, 480)
    
    try:
        while True:
            frame = client.get_frame()
            if frame is not None:
                cv2.imshow("Camera Client Test", frame)
            
            if cv2.waitKey(10) & 0xFF == ord('q'):
                print("[TEST] Quit requested.")
                break
    except KeyboardInterrupt:
        print("\n[TEST] Keyboard interrupt.")
    finally:
        client.stop()
        cv2.destroyAllWindows()
