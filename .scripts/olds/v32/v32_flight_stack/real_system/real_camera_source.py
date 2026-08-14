import cv2
import numpy as np
import logging
from core.interfaces.i_camera_source import ICameraSource

logger = logging.getLogger(__name__)

class RealCameraSource(ICameraSource):
    def __init__(self, device_index_or_pipeline):
        self.device_index_or_pipeline = device_index_or_pipeline
        self.cap = None

    async def start(self) -> None:
        logger.info(f"Kamera başlatılıyor: {self.device_index_or_pipeline}")
        self.cap = cv2.VideoCapture(self.device_index_or_pipeline)
        if not self.cap.isOpened():
            raise RuntimeError(f"Kamera acilamadi: {self.device_index_or_pipeline}")
            
    async def stop(self) -> None:
        if self.cap:
            self.cap.release()
            self.cap = None
            
    async def get_frame(self) -> np.ndarray:
        if not self.cap:
            raise RuntimeError("Kamera baslatilmamis.")
            
        ret, frame = self.cap.read()
        if not ret:
            logger.error("Kameradan kare okunamadi!")
            raise RuntimeError("Kameradan kare okunamadi! Gorev tehlikede.")
            
        return frame
        
    def get_resolution(self) -> tuple[int, int]:
        if not self.cap:
            return (0, 0)
        w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        return (w, h)
