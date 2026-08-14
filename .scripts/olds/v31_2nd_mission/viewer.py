import sys
import time
import cv2
from camera_client import CameraClient
from config import CAMERA_ZMQ_ADDRESS

def main():
    print("[VIEWER] Starting Standalone Camera Viewer...")
    client = CameraClient(CAMERA_ZMQ_ADDRESS)
    client.start()
    
    cv2.namedWindow("KURSAD40 V30 - Ground Station Camera", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("KURSAD40 V30 - Ground Station Camera", 640, 480)
    
    print("[VIEWER] Waiting for frames...")
    
    viewer_count = 0
    viewer_start_time = time.time()
    
    try:
        while True:
            frame = client.get_frame()
            if frame is not None:
                viewer_count += 1
                now_time = time.time()
                if now_time - viewer_start_time >= 1.0:
                    fps = viewer_count / (now_time - viewer_start_time)
                    print(f"[VIEWER] Displaying at {fps:.1f} FPS | ts: {now_time:.2f}")
                    viewer_count = 0
                    viewer_start_time = now_time
                cv2.imshow("KURSAD40 V30 - Ground Station Camera", frame)
            else:
                # Put a waiting text if no frame is received yet
                import numpy as np
                img = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(img, "Waiting for stream...", (200, 240), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
                cv2.imshow("KURSAD40 V30 - Ground Station Camera", img)
                
            if cv2.waitKey(30) & 0xFF == ord('q'):
                print("[VIEWER] Quit requested.")
                break
    except KeyboardInterrupt:
        print("\n[VIEWER] Keyboard interrupt.")
    finally:
        client.stop()
        cv2.destroyAllWindows()
        print("[VIEWER] Shutdown complete.")

if __name__ == "__main__":
    main()
