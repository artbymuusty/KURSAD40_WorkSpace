import asyncio
import yaml
import logging
import os
import uuid

from real_system.real_flight_backend import RealFlightBackend
from real_system.real_camera_source import RealCameraSource
from real_system.real_payload_actuator import RealPayloadActuator

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


async def _run(config: dict, mission_id: str) -> None:
    # ADR-004 §13: constructed and started BEFORE anything mission-related --
    # the dashboard opens the instant RUN MISSION executes, unconditionally,
    # no operator action.
    ops_center = build_ops_center(mission_id=mission_id, log_dir=config.get("log_dir", "logs"))
    ops_center.start()
    publisher = ops_center.bus
    context = ops_center.context
    camera = None

    try:
        flight = RealFlightBackend(config["flight_backend"]["connection_string"], publisher=publisher)
        camera = RealCameraSource(config["camera"]["device_index_or_pipeline"])
        # BUG FIX: camera.start() was never called anywhere in this codebase
        # (pre-existing, not introduced by ADR-004) -- RealCameraSource only
        # ever opens cv2.VideoCapture inside start(), so every get_frame()
        # call raised "Kamera baslatilmamis." forever. Previously this only
        # surfaced once _search_and_engage_loop() began; now that vision
        # runs from mission start (_frame_grab_loop/_detection_loop), it
        # would surface immediately and continuously instead.
        await camera.start()
        actuator = RealPayloadActuator()

        # BUG FIX (operator-reported): see gz_system/main_gz.py for the full
        # explanation -- YoloDetector("yolov8n.pt") never actually detected
        # anything (unresolvable path + wrong classes even if it did), so
        # Mission mode ran with vision permanently blind. HSVContourDetector
        # is the one detector proven to work; swap back to
        # YoloDetector(<real trained model path>) once a real YOLO26 model
        # exists. Its HSV thresholds (parameters.py) were tuned for the
        # earlier v29 camera/lighting setup -- retune against the real
        # camera before competition use if colors look off on the dashboard.
        detector = HSVContourDetector()
        validator = TargetValidator()
        selector = TargetSelector()
        debounce = DebounceTracker(publisher=publisher)
        # BUG FIX (operator revision, 2026-08-13, "Mission Lifecycle" --
        # INVALID STATE 7): see gz_system/main_gz.py for the full
        # explanation -- mission-ID-scoped path prevents a previous
        # mission's target records from leaking into a new one.
        position_store_path = os.path.join(config.get("log_dir", "logs"), f"mission_positions_{mission_id}.json")
        position_store = PositionStore(storage_path=position_store_path, publisher=publisher)
        interlock = PayloadInterlock(publisher=publisher)
        checkpoint = MissionCheckpoint(publisher=publisher)

        # ADR-008 B1: one shared detection feed -- single producer
        # (Gorev2Orchestrator._detection_loop), many consumers. See
        # core/detection/detection_feed.py.
        detection_feed = DetectionFeed()

        centering = CenteringController(flight, detection_feed, camera, publisher=publisher)
        centering.kp_horizontal = config["control_gains"]["kp_horizontal"]
        centering.kp_vertical = config["control_gains"]["kp_vertical"]
        centering.kp_altitude = config["control_gains"]["kp_altitude"]
        centering.tolerance_x = config["control_gains"]["centering_tolerance_x"]
        centering.tolerance_y = config["control_gains"]["centering_tolerance_y"]

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
        # was ever assigned. For RealCameraSource this also releases the
        # real cv2.VideoCapture device -- skipping it would leak the camera
        # handle across runs.
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
    config_path = os.path.join(os.path.dirname(__file__), "config", "real_system.yaml")
    with open(config_path, "r") as f:
        config = yaml.safe_load(f)

    mission_id = uuid.uuid4().hex[:12]
    asyncio.run(_run(config, mission_id))

if __name__ == "__main__":
    main()
