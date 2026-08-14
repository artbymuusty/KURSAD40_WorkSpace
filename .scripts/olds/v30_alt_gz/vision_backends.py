# vision_backends.py
from vision_interfaces import DetectorBackend, TrackerBackend
from typing import List, Tuple, Dict, Optional
import cv2
import numpy as np
from mission_types import RawDetection, TrackedDetection
from config import (RED_LO_1, RED_HI_1, RED_LO_2, RED_HI_2, BLUE_LO, BLUE_HI,
                     MIN_AREA_TRI_BASE, MIN_AREA_HEX_BASE, EPS_TRI_MIN, EPS_TRI_MAX,
                     EPS_HEX, COLOR_FRAC_TRI_BASE, COLOR_FRAC_HEX,
                     STREAK_FRAMES, STREAK_DIST_PX)


class HSVContourDetectorBackend(DetectorBackend):
    """
    Classical HSV-threshold + contour-shape detector for blue_hexagon /
    red_triangle, ported from the working v29 monolith (.scripts/v29.py,
    class Detector). v30 replaced this with UltralyticsYoloBackend, but that
    backend only has a stock COCO-pretrained yolov8n.pt with no real training
    on the competition targets -- it never actually detected anything. This
    backend is what genuinely found the targets before, so it's the active
    DetectorBackend again (see VisionWorker.__init__ in vision.py). The
    config.py constants it uses were already present but marked
    "LEGACY, TO BE DEPRECATED" -- they were v29's tuned values, kept but
    never wired back up.

    Logic (unchanged from v29): HSV threshold per color -> morphological
    open/close -> external contours -> polygon approximation -> shape check
    (3 vertices = triangle, 6 = hexagon, both convex) -> color-fill-fraction
    verification -> per-shape N-frame position streak before a detection is
    "committed" (rejects one-frame noise/false positives).
    """

    def __init__(self):
        self._last_center: Dict[str, Optional[Tuple[int, int]]] = {"triangle": None, "hexagon": None}
        self._streak: Dict[str, int] = {"triangle": 0, "hexagon": 0}

    def load_model(self, model_path: str) -> bool:
        # No model file to load -- classical CV, not ML. Kept as a no-op to
        # satisfy the DetectorBackend ABC contract VisionWorker relies on.
        return True

    @staticmethod
    def _preprocess(frame_bgr: np.ndarray) -> np.ndarray:
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)
        v = cv2.createCLAHE(2.0, (8, 8)).apply(v)
        return cv2.merge([h, s, v])

    @staticmethod
    def _color_masks(hsv: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        red1 = cv2.inRange(hsv, RED_LO_1, RED_HI_1)
        red2 = cv2.inRange(hsv, RED_LO_2, RED_HI_2)
        red = cv2.bitwise_or(red1, red2)
        blue = cv2.inRange(hsv, BLUE_LO, BLUE_HI)

        k = np.ones((5, 5), np.uint8)
        red = cv2.morphologyEx(red, cv2.MORPH_OPEN, k, 1)
        red = cv2.morphologyEx(red, cv2.MORPH_CLOSE, k, 2)
        blue = cv2.morphologyEx(blue, cv2.MORPH_OPEN, k, 1)
        blue = cv2.morphologyEx(blue, cv2.MORPH_CLOSE, k, 2)
        return red, blue

    @staticmethod
    def _center_of(poly) -> Tuple[int, int]:
        M = cv2.moments(poly)
        if M["m00"] != 0:
            return (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        x, y, w, h = cv2.boundingRect(poly)
        return (x + w // 2, y + h // 2)

    @staticmethod
    def _min_side_len(poly) -> float:
        pts = [poly[i][0] for i in range(len(poly))]
        lens = [np.linalg.norm(pts[i] - pts[(i + 1) % len(pts)]) for i in range(len(pts))]
        return min(lens) if lens else 0.0

    def _update_streak(self, name: str, center: Tuple[int, int]) -> bool:
        last = self._last_center[name]
        if last is not None and np.hypot(center[0] - last[0], center[1] - last[1]) <= STREAK_DIST_PX:
            self._streak[name] += 1
        else:
            self._streak[name] = 1
        self._last_center[name] = center
        return self._streak[name] >= STREAK_FRAMES

    def detect(self, frame_bgr: np.ndarray, active_filter: List[str]) -> List[RawDetection]:
        want_triangle = "red_triangle" in active_filter
        want_hexagon = "blue_hexagon" in active_filter
        if not want_triangle and not want_hexagon:
            return []

        blurred = cv2.GaussianBlur(frame_bgr, (5, 5), 0)
        hsv = self._preprocess(blurred)
        red_mask, blue_mask = self._color_masks(hsv)

        h, w = frame_bgr.shape[:2]
        area_scale = (w * h) / float(1280 * 960)

        detections: List[RawDetection] = []

        if want_triangle:
            best_tri = None
            best_tri_score = -1.0
            contours_r, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours_r:
                if cv2.contourArea(cnt) < (MIN_AREA_TRI_BASE * area_scale):
                    continue
                peri = cv2.arcLength(cnt, True)
                for eps in np.linspace(EPS_TRI_MIN, EPS_TRI_MAX, 6):
                    approx = cv2.approxPolyDP(cnt, eps * peri, True)
                    if len(approx) != 3 or not cv2.isContourConvex(approx):
                        continue
                    if self._min_side_len(approx) < 8:
                        continue
                    tri_area = abs(cv2.contourArea(approx))
                    if tri_area <= 0:
                        continue

                    poly_mask = np.zeros(red_mask.shape, dtype=np.uint8)
                    cv2.fillPoly(poly_mask, [approx], 255)
                    colored = cv2.countNonZero(cv2.bitwise_and(red_mask, red_mask, mask=poly_mask))
                    area_px = cv2.countNonZero(poly_mask)
                    frac = 0.0 if area_px == 0 else colored / float(area_px)
                    frac_thr = max(0.20, COLOR_FRAC_TRI_BASE - 0.10 * np.log10(max(1.0, tri_area / 1500.0)))
                    if frac < frac_thr:
                        continue

                    score = tri_area * (0.7 * frac + 0.3 * 0.5)
                    if score > best_tri_score:
                        best_tri_score = score
                        x, y, bw, bh = cv2.boundingRect(approx)
                        best_tri = {
                            "class_name": "red_triangle",
                            "confidence": 0.9,
                            "bbox": (x, y, x + bw, y + bh),
                            "center": self._center_of(approx),
                            "polygon": approx.reshape(-1, 2).tolist(),
                        }
            if best_tri and self._update_streak("triangle", best_tri["center"]):
                detections.append(best_tri)

        if want_hexagon:
            best_hex = None
            best_hex_score = -1.0
            contours_b, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours_b:
                if cv2.contourArea(cnt) < (MIN_AREA_HEX_BASE * area_scale):
                    continue
                peri = cv2.arcLength(cnt, True)
                approx = cv2.approxPolyDP(cnt, EPS_HEX * peri, True)
                if len(approx) != 6 or not cv2.isContourConvex(approx):
                    continue
                if self._min_side_len(approx) < 8:
                    continue

                poly_mask = np.zeros(blue_mask.shape, dtype=np.uint8)
                cv2.fillPoly(poly_mask, [approx], 255)
                colored = cv2.countNonZero(cv2.bitwise_and(blue_mask, blue_mask, mask=poly_mask))
                area_px = cv2.countNonZero(poly_mask)
                frac = 0.0 if area_px == 0 else colored / float(area_px)
                if frac < COLOR_FRAC_HEX:
                    continue

                area = cv2.contourArea(approx)
                score = area * frac
                if score > best_hex_score:
                    best_hex_score = score
                    x, y, bw, bh = cv2.boundingRect(approx)
                    best_hex = {
                        "class_name": "blue_hexagon",
                        "confidence": 0.9,
                        "bbox": (x, y, x + bw, y + bh),
                        "center": self._center_of(approx),
                        "polygon": approx.reshape(-1, 2).tolist(),
                    }
            if best_hex and self._update_streak("hexagon", best_hex["center"]):
                detections.append(best_hex)

        return detections


class UltralyticsYoloBackend(DetectorBackend):
    """Implementation of DetectorBackend using Ultralytics YOLOv8/11."""
    
    def __init__(self, use_tensorrt: bool = False):
        self.model = None
        self.class_names = {}
        self.use_tensorrt = use_tensorrt
        self.backend_type = "PyTorch"
        
    def load_model(self, model_path: str) -> bool:
        try:
            from ultralytics import YOLO
            import os
            
            # If TensorRT requested, check if .engine exists
            engine_path = model_path.replace('.pt', '.engine')
            if self.use_tensorrt and os.path.exists(engine_path):
                print(f"[UltralyticsYoloBackend] Loading TensorRT engine: {engine_path}")
                self.model = YOLO(engine_path, task='detect')
                self.backend_type = "TensorRT"
            else:
                if self.use_tensorrt:
                    print(f"[UltralyticsYoloBackend] WARNING: TensorRT engine not found at {engine_path}. Falling back to PyTorch.")
                self.model = YOLO(model_path)
                self.backend_type = "PyTorch"
                
            self.class_names = self.model.names
            return True
        except ImportError:
            print("[UltralyticsYoloBackend] ERROR: ultralytics package not installed.")
            return False
        except Exception as e:
            print(f"[UltralyticsYoloBackend] ERROR loading model {model_path}: {e}")
            return False
            
    def detect(self, frame_bgr: np.ndarray, active_filter: List[str]) -> List[RawDetection]:
        if not self.model:
            return []
            
        results = self.model.predict(
            source=frame_bgr,
            verbose=False,
            conf=0.1, # Pass low-confidence detections to tracker
            device='cuda' if self.backend_type == "TensorRT" else 'cpu'
        )
        
        detections = []
        if not results:
            return detections
            
        boxes = results[0].boxes
        for box in boxes:
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])
            xyxy = box.xyxy[0].cpu().numpy()
            
            raw_name = self.class_names.get(cls_id, f"class_{cls_id}")
            mapped_name = raw_name
            
            # TEMPORARY: stock yolov8n.pt is COCO-trained and knows nothing about
            # the competition targets. This remaps COCO classes as stand-ins so the
            # pipeline is exercisable before a custom-trained model exists.
            # MUST be replaced by a model trained on the 4 real target classes
            # (blue_hexagon, red_triangle, blue_rectangle, red_rectangle) before
            # competition use -- these substitutions will misfire on real COCO
            # objects (people, cars, stop signs, suitcases) in the camera frame.
            if mapped_name == "person": mapped_name = "red_triangle"
            elif mapped_name == "car": mapped_name = "blue_hexagon"
            elif mapped_name == "stop sign": mapped_name = "blue_rectangle"
            elif mapped_name == "suitcase": mapped_name = "red_rectangle"

            # Only the 4 competition targets are ever passed downstream -- every
            # other COCO class (and anything not in the currently active filter)
            # is dropped here.
            if mapped_name not in ("blue_hexagon", "red_triangle", "blue_rectangle", "red_rectangle"):
                continue
            if mapped_name not in active_filter:
                continue
                
            x_min, y_min, x_max, y_max = map(int, xyxy)
            cx = (x_min + x_max) // 2
            cy = (y_min + y_max) // 2
            
            detections.append({
                "class_name": mapped_name,
                "confidence": conf,
                "bbox": (x_min, y_min, x_max, y_max),
                "center": (cx, cy),
                "polygon": None,  # box-only detector -- dashboard falls back to the bounding box
            })
            
        return detections


class ByteTrackBackend(TrackerBackend):
    """Implementation of TrackerBackend using ByteTrack."""
    
    def __init__(self):
        # In a real environment, we would initialize BYTETracker from ultralytics or a standalone byte_tracker lib.
        # For this standalone implementation we implement a robust ID persistence tracker mimicking ByteTrack.
        self.tracks = {}
        self.next_id = 1
        self.max_age = 30 # frames
        
    def update(self, detections: List[RawDetection], frame_shape: Tuple[int, int]) -> List[TrackedDetection]:
        """
        Updates tracking IDs using a bounding-box IOU or centroid distance matching.
        """
        tracked_detections = []
        
        # If no detections, just age the tracks
        if not detections:
            for k in list(self.tracks.keys()):
                self.tracks[k]["age"] += 1
                if self.tracks[k]["age"] > self.max_age:
                    del self.tracks[k]
            return []
            
        # Very simplified matching logic acting as ByteTrack surrogate for architectural completeness.
        # A real ByteTrack uses Kalman Filters and high/low confidence association logic.
        unmatched_dets = list(detections)
        
        # Update existing tracks
        for track_id, track_info in list(self.tracks.items()):
            track_info["age"] += 1
            
            best_det = None
            best_dist = 100.0 # pixel matching threshold
            best_idx = -1
            
            for i, det in enumerate(unmatched_dets):
                if det["class_name"] != track_info["class_name"]:
                    continue
                # Calculate distance
                tcx, tcy = track_info["center"]
                dcx, dcy = det["center"]
                dist = ((tcx-dcx)**2 + (tcy-dcy)**2)**0.5
                if dist < best_dist:
                    best_dist = dist
                    best_det = det
                    best_idx = i
                    
            if best_det is not None:
                # Match found
                track_info["center"] = best_det["center"]
                track_info["bbox"] = best_det["bbox"]
                track_info["polygon"] = best_det.get("polygon")
                track_info["age"] = 0
                tracked_det = {
                    "class_name": best_det["class_name"],
                    "confidence": best_det["confidence"],
                    "bbox": best_det["bbox"],
                    "center": best_det["center"],
                    "tracking_id": track_id,
                    "polygon": best_det.get("polygon"),
                }
                tracked_detections.append(tracked_det)
                unmatched_dets.pop(best_idx)
            else:
                # Dead track
                if track_info["age"] > self.max_age:
                    del self.tracks[track_id]
                    
        # Register new tracks
        for det in unmatched_dets:
            tid = self.next_id
            self.next_id += 1
            self.tracks[tid] = {
                "class_name": det["class_name"],
                "center": det["center"],
                "bbox": det["bbox"],
                "polygon": det.get("polygon"),
                "age": 0
            }
            # Instead of modifying the dict directly (which we can't type check safely with TypedDict), we create a new one.
            tracked_det = {
                "class_name": det["class_name"],
                "confidence": det["confidence"],
                "bbox": det["bbox"],
                "center": det["center"],
                "tracking_id": tid,
                "polygon": det.get("polygon"),
            }
            tracked_detections.append(tracked_det)
            
        return tracked_detections
