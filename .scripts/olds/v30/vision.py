# vision.py
import cv2
import numpy as np
import time
import threading
from typing import Optional, List
from config import *
from mission_types import TargetData
from vision_backends import HSVContourDetectorBackend, ByteTrackBackend
from target_selector import TargetSelector

class SharedDetectionBuffer:
    """
    Thread-safe buffer strictly passing TargetData between VisionWorker and MissionManager.
    """
    def __init__(self):
        self.lock = threading.Lock()
        self.latest_detection: Optional[TargetData] = None

    def put(self, det: Optional[TargetData]):
        with self.lock:
            self.latest_detection = det

    def get(self) -> Optional[TargetData]:
        with self.lock:
            return self.latest_detection

class VisionWorker:
    """
    Dedicated worker thread for the Vision Pipeline.
    Orchestrates Detector -> Tracker -> TargetSelector.
    """
    def __init__(self, camera_client, shared_buffer: SharedDetectionBuffer):
        self.camera_client = camera_client
        self.buffer = shared_buffer
        
        # HSVContourDetectorBackend is the working detector (ported from the
        # proven .scripts/v29.py implementation) -- UltralyticsYoloBackend is
        # kept available (see vision_backends.py) for once a real
        # custom-trained model exists, but a stock COCO yolov8n.pt never
        # actually detects the competition targets. Swapping the concrete
        # DetectorBackend here is the intended extension point; VisionWorker's
        # own orchestration (Detector -> Tracker -> TargetSelector) is unchanged.
        self.detector = HSVContourDetectorBackend()
        self.detector.load_model(YOLO_WEIGHTS_PATH)
        self.tracker = ByteTrackBackend()
        self.selector = TargetSelector()
        
        self.lock = threading.Lock()
        self.active_targets: List[str] = []
        
        self.running = False
        self.thread = None
        
        # Performance Logging
        self.frames_processed = 0
        self.valid_detections = 0
        self.last_log_time = time.time()
        self.last_frame_ts = 0
        self.last_fps_info = {"inf": 0.0, "trk": 0.0, "lat": 0.0}

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()
        print("[VISION_WORKER] Started Vision Pipeline.")

    def stop(self):
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=2.0)
        print("[VISION_WORKER] Stopped Vision Pipeline.")

    @property
    def mode(self):
        return "YOLO_PIPELINE"

    def set_filter(self, active_targets: list):
        with self.lock:
            if set(self.active_targets) != set(active_targets):
                # Filter changed, break current lock
                self.selector.reset_lock()
            self.active_targets = list(active_targets)

    def _run(self):
        while self.running:
            # 1. Camera -> Frame Buffer
            frame, ts = self.camera_client.get_frame_with_time()
            if frame is None or ts <= self.last_frame_ts:
                time.sleep(0.005)
                continue
                
            self.last_frame_ts = ts
            h, w = frame.shape[:2]
            
            with self.lock:
                active = list(self.active_targets)
                
            # Preprocessing
            scale_x, scale_y = 1.0, 1.0
            proc_frame = frame
            if w > 640:
                proc_w = 640
                proc_h = int(h * (640 / w))
                proc_frame = cv2.resize(frame, (proc_w, proc_h))
                scale_x = w / proc_w
                scale_y = h / proc_h

            t_start = time.time()
                
            # 2. Abstract Detector -> Detections
            t0_inf = time.time()
            raw_detections = self.detector.detect(proc_frame, active)
            t1_inf = time.time()
            
            # Scale coordinates back up
            for det in raw_detections:
                tx, ty = det["center"]
                det["center"] = (int(tx * scale_x), int(ty * scale_y))
                x1, y1, x2, y2 = det["bbox"]
                det["bbox"] = (int(x1 * scale_x), int(y1 * scale_y), int(x2 * scale_x), int(y2 * scale_y))
                # The exact polygon (see HSVContourDetectorBackend) was found
                # on the same downscaled proc_frame as bbox/center above, so
                # it needs the identical scale-back-up or the dashboard's
                # shape outline would be drawn shrunk toward the top-left
                # corner instead of tracing the real detection.
                poly = det.get("polygon")
                if poly:
                    det["polygon"] = [(int(px * scale_x), int(py * scale_y)) for px, py in poly]
                
            # 3. Tracker Interface -> Assign tracking_id
            t0_trk = time.time()
            tracked_detections = self.tracker.update(raw_detections, (h, w))
            t1_trk = time.time()
            
            # 4. TargetSelector -> Final TargetData
            final_target = self.selector.select(tracked_detections, (w, h))
            
            if final_target and not final_target["is_stale"]:
                self.valid_detections += 1
                
            # 5. Shared Detection Buffer
            self.buffer.put(final_target)
            
            t_end = time.time()
            
            # 6. Performance Monitoring
            self.frames_processed += 1
            now = time.time()
            if now - self.last_log_time >= 5.0:
                fps = self.frames_processed / (now - self.last_log_time)
                inf_fps = 1.0 / max((t1_inf - t0_inf), 0.001)
                trk_fps = 1.0 / max((t1_trk - t0_trk), 0.001)
                pipe_latency = (t_end - t_start) * 1000
                valid_rate = (self.valid_detections / max(1, self.frames_processed)) * 100
                
                self.last_fps_info = {"inf": inf_fps, "trk": trk_fps, "lat": pipe_latency}
                
                print(f"[VISION_PERF] Pipeline FPS: {fps:.1f} | Inference: {inf_fps:.1f} FPS | Tracker: {trk_fps:.1f} FPS | Latency: {pipe_latency:.1f}ms | Valid: {valid_rate:.1f}%")
                
                self.frames_processed = 0
                self.valid_detections = 0
                self.last_log_time = now
