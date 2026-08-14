import asyncio
import yaml
import logging
import os
import uuid

from gz_system.gz_flight_backend import GzFlightBackend
from gz_system.gz_camera_source import GzCameraSource
from gz_system.gz_payload_actuator import GzPayloadActuator

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
        flight = GzFlightBackend(config["flight_backend"]["connection_string"], publisher=publisher)
        camera = GzCameraSource(config["camera"]["ros2_topic"], config["camera"]["zmq_address"])
        # BUG FIX: camera.start() was never called anywhere in this
        # codebase (pre-existing, not introduced by ADR-004) -- GzCameraSource
        # only ever populates _last_frame inside start(), so every
        # get_frame() call raised "Baglanti yok." forever. Previously this
        # only surfaced once _search_and_engage_loop() began (after
        # connect/arm/takeoff/climb/upload all succeeded); now that vision
        # runs from mission start (_frame_grab_loop/_detection_loop), it
        # surfaced immediately and continuously instead.
        await camera.start()
        actuator = GzPayloadActuator(config["actuator"]["gazebo_service_name"])

        # BUG FIX (operator-reported): YoloDetector("yolov8n.pt") never
        # actually detected anything -- the path doesn't resolve from this
        # entrypoint's cwd, and even loaded, yolov8n.pt is a stock
        # COCO-pretrained model sharing no class names with
        # MAVI_ALTIGEN/KIRMIZI_UCGEN (see yolo_detector.py's own footgun
        # warning). detect() therefore always returned [], so
        # TargetValidator never reached track-ready and the Mission->Offboard
        # handover never had anything to trigger on -- Mission mode just ran
        # to completion untouched. HSVContourDetector is the one detector in
        # this codebase actually proven to find MAVI_ALTIGEN/KIRMIZI_UCGEN
        # (ported from the working v29/flat-v32 pipeline); swap back to
        # YoloDetector(<real trained model path>) once a real YOLO26 model
        # exists.
        detector = HSVContourDetector()
        validator = TargetValidator()
        selector = TargetSelector()
        debounce = DebounceTracker(publisher=publisher)
        # BUG FIX (operator revision, 2026-08-13, "Mission Lifecycle" --
        # INVALID STATE 7): PositionStore(publisher=publisher) used to
        # default to a FIXED "mission_positions.json" path and LOADS
        # existing data from it on construction -- a previous mission's
        # target records would silently satisfy both_required_targets_found()
        # for a brand-new mission before it ever searched anything.
        # Mission-ID-scoped path, same convention as EventStore's own
        # per-mission log file, guarantees a clean slate every run.
        position_store_path = os.path.join(config.get("log_dir", "logs"), f"mission_positions_{mission_id}.json")
        position_store = PositionStore(storage_path=position_store_path, publisher=publisher)
        interlock = PayloadInterlock(publisher=publisher)
        checkpoint = MissionCheckpoint(publisher=publisher)

        centering = CenteringController(flight, detector, camera, publisher=publisher)
        centering.kp_horizontal = config["control_gains"]["kp_horizontal"]
        centering.kp_vertical = config["control_gains"]["kp_vertical"]
        centering.kp_altitude = config["control_gains"]["kp_altitude"]
        centering.tolerance_x = config["control_gains"]["centering_tolerance_x"]
        centering.tolerance_y = config["control_gains"]["centering_tolerance_y"]

        release_service = PayloadReleaseService(actuator, detector, camera, centering, flight, publisher=publisher)
        sequencer = PayloadMissionSequencer(flight, centering, interlock, position_store, release_service,
                                             publisher=publisher)

        gorev2 = Gorev2Orchestrator(
            flight=flight, camera=camera, detector=detector, actuator=actuator,
            interlock=interlock, position_store=position_store, debounce=debounce,
            validator=validator, selector=selector, centering=centering, sequencer=sequencer,
            checkpoint=checkpoint, release_service=release_service,
            context=context, publisher=publisher, frame_channel=ops_center.frame_channel,
        )

        pickup_phase = Gorev3PickupPhase(flight, camera, detector, actuator, position_store,
                                          RectangleAlignmentStrategy(), centering)
        transport_phase = Gorev3TransportPhase(flight, position_store, centering)
        redrop_phase = Gorev3RedropPhase(flight, actuator, position_store, centering)
        finish_phase = Gorev3FinishPhase(flight, checkpoint, centering)
        gorev3 = Gorev3Orchestrator(interlock, pickup_phase, transport_phase, redrop_phase, finish_phase,
                                     context=context, publisher=publisher)

        master = MasterMissionController(gorev2, gorev3, context=context, publisher=publisher)
        await master.run()
    finally:
        # ADR-004 §13 / §1: always torn down, even on failure -- the
        # dashboard must never linger past the mission it was observing.
        # camera.stop() mirrors camera.start() above -- symmetric lifecycle,
        # guarded for the case construction itself failed before `camera`
        # was ever assigned.
        if camera is not None:
            try:
                await camera.stop()
            except Exception as e:  # noqa: BLE001 -- shutdown must not raise over a cleanup failure
                logger.warning(f"Kamera durdurulurken hata (yoksayiliyor): {e}")
        await ops_center.stop()


def main():
    # BUG FIX (runtime investigation, 2026-08-13): must run before any
    # core.*/gz_system.* module logs anything -- see configure_all_loggers()'s
    # own docstring for the full explanation of what was silently invisible
    # without this.
    configure_all_loggers()
    config_path = os.path.join(os.path.dirname(__file__), "config", "gz_system.yaml")
    with open(config_path, "r") as f:
        config = yaml.safe_load(f)

    mission_id = uuid.uuid4().hex[:12]
    asyncio.run(_run(config, mission_id))

if __name__ == "__main__":
    main()
