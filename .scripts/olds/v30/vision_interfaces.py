# vision_interfaces.py
from abc import ABC, abstractmethod
from typing import List, Tuple, Dict
import numpy as np
from mission_types import RawDetection, TrackedDetection

class DetectorBackend(ABC):
    """Abstract Base Class for object detection backends."""
    
    @abstractmethod
    def load_model(self, model_path: str) -> bool:
        """Loads the model weights into memory."""
        pass
        
    @abstractmethod
    def detect(self, frame_bgr: np.ndarray, active_filter: List[str]) -> List[RawDetection]:
        """
        Performs inference on the frame and returns raw detections.
        """
        pass

class TrackerBackend(ABC):
    """Abstract Base Class for object tracking backends."""
    
    @abstractmethod
    def update(self, detections: List[RawDetection], frame_shape: Tuple[int, int]) -> List[TrackedDetection]:
        """
        Processes new detections and associates them with existing tracks.
        """
        pass
