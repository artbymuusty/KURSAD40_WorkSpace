import asyncio
import logging
import numpy as np
from core.interfaces.i_camera_source import ICameraSource
from gz_system import camera_service_manager
from gz_system.camera_client import CameraClient

logger = logging.getLogger(__name__)

DEFAULT_ZMQ_ADDRESS = "tcp://127.0.0.1:5555"


class GzCameraSource(ICameraSource):
    """
    BUG FIX: this class previously never subscribed to Gazebo at all --
    start() just set a one-time np.zeros() black frame and never updated
    it, which is why the Mission Operations Center's camera panel showed
    solid black instead of the real simulated feed (V30/V31 had already
    solved this; it just wasn't carried into the v32_flight_stack rewrite).

    Now backed by the same proven two-process architecture V30/V31 used:
    camera_service.py subscribes to Gazebo's real camera topic in an
    isolated OS process (gz-transport's protobuf bindings can collide with
    other protobuf usage in this process) and republishes frames over
    ZeroMQ; this class is the consuming client side.
    """

    def __init__(self, ros2_topic: str, zmq_address: str = DEFAULT_ZMQ_ADDRESS):
        self.ros2_topic = ros2_topic
        self.zmq_address = zmq_address
        self._client: CameraClient = None
        self._resolution = (640, 480)  # updated once a real frame arrives

    async def start(self) -> None:
        logger.info(f"Gazebo kamera servisi kontrol ediliyor (topic={self.ros2_topic})...")
        loop = asyncio.get_running_loop()
        # camera_service_manager.start() spawns/verifies a separate OS
        # process and briefly blocks (~1s) confirming it started -- offload
        # so it doesn't stall the event loop during mission startup.
        await loop.run_in_executor(None, camera_service_manager.start, self.ros2_topic, self.zmq_address)

        self._client = CameraClient(self.zmq_address)
        self._client.start()
        logger.info("Gazebo kamera istemcisi baslatildi.")

    async def stop(self) -> None:
        if self._client is not None:
            self._client.stop()
            self._client = None
        loop = asyncio.get_running_loop()
        await loop.run_in_executor(None, camera_service_manager.stop)

    async def get_frame(self) -> np.ndarray:
        if self._client is None:
            raise RuntimeError("Gazebo kameradan kare okunamadi. Baglanti yok.")
        frame = self._client.get_frame()
        if frame is None:
            raise RuntimeError("Gazebo kameradan kare okunamadi. Henuz kare alinmadi veya baglanti koptu.")
        h, w = frame.shape[:2]
        self._resolution = (w, h)
        return frame

    def get_resolution(self) -> tuple[int, int]:
        return self._resolution
