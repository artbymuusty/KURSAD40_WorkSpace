import sys
import os

log_file = open("logs/diagnostic.log", "w")
def print(*args, **kwargs):
    kwargs["file"] = log_file
    kwargs["flush"] = True
    __builtins__.print(*args, **kwargs)

import time
import cv2
import numpy as np
import subprocess
import os

try:
    from gz.transport13 import Node
    from gz.msgs11.image_pb2 import Image as ImageMsg
except ImportError:
    try:
        from gz.transport13 import Node
        from gz.msgs10.image_pb2 import Image as ImageMsg
    except ImportError as e:
        print("[DIAGNOSTIC FAIL] Could not import Gazebo bindings.")
        sys.exit(1)

from config import DEFAULT_GZ_TOPIC

class CameraDiagnostic:
    def __init__(self, topic):
        self.topic = topic
        self.node = Node()
        self.latest_frame = None
        self.frame_count = 0
        self.running = False
        self.saved_screenshot = False
        
        print(f"[DIAGNOSTIC] Subscribing to {self.topic}...")
        result = self.node.subscribe(ImageMsg, self.topic, self._cb)
        print(f"[DIAGNOSTIC] subscribe() returned: {result}")
        if not result:
            print(f"[DIAGNOSTIC FAIL] Failed to subscribe to {self.topic}.")
            sys.exit(1)
            
    def _cb(self, msg: ImageMsg):
        print(f"[DIAGNOSTIC] Callback triggered! Message size: {len(msg.data)} bytes")
        sys.stdout.flush()
        try:
            if not msg.data or msg.width == 0 or msg.height == 0:
                return
            
            W, H = msg.width, msg.height
            if msg.pixel_format_type == 3: # RGB_888
                np_arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((H, W, 3))
                frame = cv2.cvtColor(np_arr, cv2.COLOR_RGB2BGR)
            else:
                np_arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((H, W, 3))
                frame = np_arr
                
            self.latest_frame = frame
            self.frame_count += 1
            if self.frame_count == 1 or self.frame_count % 10 == 0:
                print(f"[DIAGNOSTIC] Received frame {self.frame_count} - Resolution: {W}x{H} - Format: {msg.pixel_format_type}")
                sys.stdout.flush()
                
            if not self.saved_screenshot and self.frame_count > 5:
                cv2.imwrite("logs/diagnostic_frame.png", frame)
                self.saved_screenshot = True
                print("[DIAGNOSTIC] Saved diagnostic_frame.png")
                
        except Exception as e:
            print(f"[DIAGNOSTIC WARN] Frame decode error: {e}")

    def run(self):
        print("[DIAGNOSTIC] Running camera test mode (Direct Gazebo Connection).")
        
        # Unpause Gazebo simulation to ensure frames are generated
        print("[DIAGNOSTIC] Ensuring simulation time is advancing (unpausing Gazebo)...")
        subprocess.run(
            "gz service -s /world/default/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 2000 --req 'pause: false'", 
            shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )
        
        # Skip X11 window if in headless environment
        headless = True
        if not headless:
            cv2.namedWindow("Gazebo Camera Diagnostic", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("Gazebo Camera Diagnostic", 640, 480)
        else:
            print("[DIAGNOSTIC] Headless environment detected. Skipping cv2.imshow, but will still receive frames.")
        
        self.running = True
        try:
            while self.running:
                if self.latest_frame is not None and not headless:
                    cv2.imshow("Gazebo Camera Diagnostic", self.latest_frame)
                    
                if not headless:
                    if cv2.waitKey(30) & 0xFF == ord('q'):
                        print("[DIAGNOSTIC] Quit requested.")
                        break
                else:
                    time.sleep(0.1)
                    if self.frame_count > 20: # Auto-exit after receiving enough frames for the test
                        print("[DIAGNOSTIC] Received enough frames for headless test. Exiting automatically.")
                        break
        except KeyboardInterrupt:
            print("\n[DIAGNOSTIC] Keyboard interrupt.")
        finally:
            if not headless:
                cv2.destroyAllWindows()
            print(f"[DIAGNOSTIC] Shutdown. Total frames received: {self.frame_count}")

if __name__ == "__main__":
    diag = CameraDiagnostic(DEFAULT_GZ_TOPIC)
    diag.run()
