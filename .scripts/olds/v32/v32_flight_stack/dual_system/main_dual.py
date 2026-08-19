"""
Bu giriş noktası, gerçek uçuş sırasında komutları EŞZAMANLI olarak Gazebo'da da 'gölge' şeklinde çalıştırıp
karşılaştırma (hardware-in-the-loop doğrulama) amacıyla kullanılır. Üretim/yarışma uçuşunda main_real.py,
geliştirme/test aşamasında main_gz.py, doğrulama/hata ayıklama aşamasında main_dual.py kullanılır.
"""

import asyncio
import yaml
import logging
import os
import uuid

from real_system.real_flight_backend import RealFlightBackend
from real_system.real_camera_source import RealCameraSource
from real_system.real_payload_actuator import RealPayloadActuator

from gz_system.gz_flight_backend import GzFlightBackend
from gz_system.gz_camera_source import GzCameraSource
from gz_system.gz_payload_actuator import GzPayloadActuator

from dual_system.dual_backend_adapter import DualFlightBackend, DualCameraSource, DualPayloadActuator

from core.detection.detection_feed import DetectionFeed
from core.detection.hsv_contour_detector import HSVContourDetector
from core.detection.target_validator import TargetValidator
from core.detection.target_selector import TargetSelector
from core.mission.debounce import DebounceTracker
from core.position_log.position_store import PositionStore
from core.mission.interlock import PayloadInterlock
from core.navigation.centering_controller import CenteringController
from core.mission.payload_release import PayloadReleaseService
from core.mission.gorev2_fsm import PayloadMissionSequencer
from core.navigation.checkpoint import MissionCheckpoint
from core.mission.gorev2_orchestrator import Gorev2Orchestrator
from core.mission.gorev3_pickup import Gorev3PickupPhase
from core.mission.gorev3_transport import Gorev3TransportPhase
from core.mission.gorev3_redrop import Gorev3RedropPhase
from core.mission.gorev3_finish import Gorev3FinishPhase
from core.mission.gorev3_orchestrator import Gorev3Orchestrator
from core.mission.master_fsm import MasterMissionController
from core.mission.rectangle_alignment_strategy import RectangleAlignmentStrategy
from core.telemetry.ops_center import build_ops_center
from core.telemetry.mission_logger import configure_all_loggers

logger = logging.getLogger(__name__)


async def _run(real_config: dict, gz_config: dict, mission_id: str) -> None:
    # ADR-004 §13: constructed and started BEFORE anything mission-related --
    # the dashboard opens the instant RUN MISSION executes, unconditionally,
    # no operator action.
    ops_center = build_ops_center(mission_id=mission_id, log_dir=real_config.get("log_dir", "logs"))
    ops_center.start()
    publisher = ops_center.bus
    context = ops_center.context
    camera = None

    try:
        real_flight = RealFlightBackend(real_config["flight_backend"]["connection_string"], publisher=publisher)
        sim_flight = GzFlightBackend(gz_config["flight_backend"]["connection_string"], publisher=publisher)
        flight = DualFlightBackend(real_flight, sim_flight)

        real_cam = RealCameraSource(real_config["camera"]["device_index_or_pipeline"])
        sim_cam = GzCameraSource(gz_config["camera"]["ros2_topic"], gz_config["camera"]["zmq_address"])
        camera = DualCameraSource(real_cam, sim_cam)
        # BUG FIX: camera.start() was never called anywhere in this codebase
        # (pre-existing, not introduced by ADR-004). DualCameraSource.start()
        # fans out to both real_cam.start() and sim_cam.start()
        # (dual_backend_adapter.py), so one call here starts both -- without
        # it, every get_frame() call on either underlying source raised
        # immediately, now surfacing from mission start instead of only once
        # _search_and_engage_loop() began.
        await camera.start()

        real_actuator = RealPayloadActuator()
        sim_actuator = GzPayloadActuator(gz_config["actuator"]["gazebo_service_name"])
        actuator = DualPayloadActuator(real_actuator, sim_actuator)

        # BUG FIX (operator-reported): see gz_system/main_gz.py for the full
        # explanation -- YoloDetector("yolov8n.pt") never actually detected
        # anything, so Mission mode ran with vision permanently blind. Swap
        # back to YoloDetector(<real trained model path>) once a real YOLO26
        # model exists.
        detector = HSVContourDetector()
        validator = TargetValidator()
        selector = TargetSelector()
        debounce = DebounceTracker(publisher=publisher)
        # BUG FIX (operator revision, 2026-08-13, "Mission Lifecycle" --
        # INVALID STATE 7): see gz_system/main_gz.py for the full
        # explanation -- mission-ID-scoped path prevents a previous
        # mission's target records from leaking into a new one.
        position_store_path = os.path.join(real_config.get("log_dir", "logs"), f"mission_positions_{mission_id}.json")
        position_store = PositionStore(storage_path=position_store_path, publisher=publisher)
        interlock = PayloadInterlock(publisher=publisher)
        checkpoint = MissionCheckpoint(publisher=publisher)

        # ADR-008 B1: one shared detection feed -- single producer
        # (Gorev2Orchestrator._detection_loop), many consumers. See
        # core/detection/detection_feed.py.
        detection_feed = DetectionFeed()

        centering = CenteringController(flight, detection_feed, camera, publisher=publisher)
        # Overwrite gains if provided (using real config as priority)
        centering.kp_horizontal = real_config["control_gains"]["kp_horizontal"]
        centering.kp_vertical = real_config["control_gains"]["kp_vertical"]
        centering.kp_altitude = real_config["control_gains"]["kp_altitude"]
        centering.tolerance_x = real_config["control_gains"]["centering_tolerance_x"]
        centering.tolerance_y = real_config["control_gains"]["centering_tolerance_y"]

        release_service = PayloadReleaseService(actuator, detection_feed, camera, centering, flight,
                                                publisher=publisher)
        sequencer = PayloadMissionSequencer(flight, centering, interlock, position_store, release_service,
                                             publisher=publisher)

        gorev2 = Gorev2Orchestrator(
            flight=flight, camera=camera, detector=detector, actuator=actuator,
            interlock=interlock, position_store=position_store, debounce=debounce,
            validator=validator, selector=selector, centering=centering, sequencer=sequencer,
            checkpoint=checkpoint, release_service=release_service,
            context=context, publisher=publisher, frame_channel=ops_center.frame_channel,
            detection_feed=detection_feed,
        )

        pickup_phase = Gorev3PickupPhase(flight, camera, detector, actuator, position_store,
                                          RectangleAlignmentStrategy(), centering)
        transport_phase = Gorev3TransportPhase(flight, position_store, centering)
        redrop_phase = Gorev3RedropPhase(flight, actuator, position_store, centering)
        finish_phase = Gorev3FinishPhase(flight, checkpoint, centering)
        gorev3 = Gorev3Orchestrator(interlock, pickup_phase, transport_phase, redrop_phase, finish_phase,
                                     context=context, publisher=publisher)

        master = MasterMissionController(gorev2, gorev3, context=context, publisher=publisher)
        # ADR-008 B2 (A2 row 6): makes the mandatory 10-minute budget act.
        ops_center.mission_timeout_hook = master.request_abort
        await master.run()
    finally:
        # ADR-004 §13 / §1: always torn down, even on failure -- the
        # dashboard must never linger past the mission it was observing.
        # camera.stop() mirrors camera.start() above -- symmetric lifecycle,
        # guarded for the case construction itself failed before `camera`
        # was ever assigned. Fans out to both real_cam.stop() and
        # sim_cam.stop(), releasing the real cv2.VideoCapture device too.
        if camera is not None:
            try:
                await camera.stop()
            except Exception as e:  # noqa: BLE001 -- shutdown must not raise over a cleanup failure
                logger.warning(f"Kamera durdurulurken hata (yoksayiliyor): {e}")
        await ops_center.stop()


def main():
    # BUG FIX (runtime investigation, 2026-08-13): see
    # configure_all_loggers()'s own docstring -- without this, virtually
    # every diagnostic log in this project was silently going nowhere.
    configure_all_loggers()
    base_dir = os.path.dirname(os.path.dirname(__file__))
    real_config_path = os.path.join(base_dir, "real_system", "config", "real_system.yaml")
    gz_config_path = os.path.join(base_dir, "gz_system", "config", "gz_system.yaml")

    with open(real_config_path, "r") as f:
        real_config = yaml.safe_load(f)
    with open(gz_config_path, "r") as f:
        gz_config = yaml.safe_load(f)

    mission_id = uuid.uuid4().hex[:12]
    asyncio.run(_run(real_config, gz_config, mission_id))

if __name__ == "__main__":
    main()
