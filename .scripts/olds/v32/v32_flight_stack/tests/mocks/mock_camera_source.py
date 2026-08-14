import numpy as np
from typing import Tuple, List
from core.interfaces.i_camera_source import ICameraSource

class MockCameraSource(ICameraSource):
    def __init__(self):
        self.calls: List[Tuple[str, dict]] = []
        self._resolution = (640, 480)
        self._frame = np.zeros((480, 640, 3), dtype=np.uint8)

    async def start(self) -> None:
        self.calls.append(('start', {}))
        
    async def stop(self) -> None:
        self.calls.append(('stop', {}))
        
    async def get_frame(self) -> np.ndarray:
        self.calls.append(('get_frame', {}))
        return self._frame
        
    def get_resolution(self) -> Tuple[int, int]:
        self.calls.append(('get_resolution', {}))
        return self._resolution
