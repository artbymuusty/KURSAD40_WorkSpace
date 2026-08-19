import asyncio
import yaml
import logging
import os
import signal
import sys
import threading
import time
import uuid

import cv2

from core.telemetry.paint_bridge import MAIN_THREAD_PAINT
from gz_system.gz_env import apply_gz_env, describe_gz_env
from gz_system.gz_flight_backend import GzFlightBackend
from gz_system.gz_camera_source import GzCameraSource
from gz_system.gz_payload_actuator import GzPayloadActuator
from gz_system.gz_pose_monitor import GzPoseMonitor

from core.detection.detection_feed import DetectionFeed
from core.detection.vision_runtime import FeedDetector, VisionRuntime
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
from core.config.parameters import ABORT_RETURN_DEADLINE_S

# NOT __name__: this module is executed as a script (run_mission_v32_gz.sh runs
# "python -u gz_system/main_gz.py"), so __name__ is "__main__", which is outside
# every namespace configure_all_loggers() sets up ("core", "gz_system",
# "mavsdk_common", ...). Every log line this entrypoint emitted was therefore
# silently dropped. Naming the logger explicitly puts it back under "gz_system".
logger = logging.getLogger("gz_system.main_gz")


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
        # F2: started here, before takeoff, so gz-transport discovery (~2 s)
        # is paid during setup and not while a payload is hanging under the
        # vehicle waiting to be confirmed released.
        pose_monitor = GzPoseMonitor()
        await pose_monitor.start()
        actuator = GzPayloadActuator(config["actuator"]["gazebo_service_name"],
                                     pose_monitor=pose_monitor)

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

        # ADR-008 B1 / ADR-010 P3: one feed, constructed at the composition
        # root and shared by its single producer (VisionRuntime) and every
        # consumer. `detector` is handed ONLY to VisionRuntime -- anything
        # else that needs a detection takes the feed, or a FeedDetector when
        # it must satisfy the IDetector interface.
        detection_feed = DetectionFeed()
        vision = VisionRuntime(camera, detector, detection_feed,
                               frame_channel=ops_center.frame_channel, publisher=publisher)
        feed_detector = FeedDetector(detection_feed)

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

        # ADR-010 P3: Görev 3 consumes the SAME feed as Görev 2. It used to
        # be handed the real detector and call detect() itself -- a second
        # caller into HSVContourDetector's streak state, and the reason its
        # own centering saw 0 committed detections in V1'''.
        pickup_phase = Gorev3PickupPhase(flight, camera, feed_detector, actuator, position_store,
                                          RectangleAlignmentStrategy(), centering)
        transport_phase = Gorev3TransportPhase(flight, position_store, centering)
        redrop_phase = Gorev3RedropPhase(flight, actuator, position_store, centering)
        finish_phase = Gorev3FinishPhase(flight, checkpoint, centering)
        gorev3 = Gorev3Orchestrator(interlock, pickup_phase, transport_phase, redrop_phase, finish_phase,
                                     context=context, publisher=publisher)

        master = MasterMissionController(gorev2, gorev3, context=context, publisher=publisher)
        # ADR-008 B2 (A2 row 6): arms the mandatory 10-minute budget to
        # actually DO something. Wired here rather than in build_ops_center()
        # because the ops center is deliberately started before the mission
        # runtime exists (ADR-004 §13).
        ops_center.mission_timeout_hook = master.request_abort
        # ADR-010 P3: vision spans the WHOLE mission -- started before the
        # master FSM and stopped only after it returns, so frames and
        # overlay keep rendering from takeoff to disarm no matter which
        # phase is running. Previously this lived inside Gorev2Orchestrator
        # and died at GOREV2_COMPLETE.
        vision.start()
        try:
            await master.run()
        finally:
            await vision.stop()
            await pose_monitor.stop()
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


def _install_signal_handlers(request_stop) -> None:
    """ADR-010 R4: install our OWN SIGINT/SIGTERM handlers instead of
    relying on Python's default.

    Root cause of V3's failure (2026-08-17, proven directly): a process
    started with `&` from a NON-INTERACTIVE shell inherits
    SIGINT = SIG_IGN -- standard POSIX background-job behaviour. Python
    only installs default_int_handler when SIGINT is not already ignored
    at startup, so with SIG_IGN inherited, KeyboardInterrupt can NEVER be
    raised and `kill -INT` is silently discarded. The paint loop's
    `except KeyboardInterrupt` was therefore unreachable, the cancel path
    never ran, and the vehicle would have been left airborne on any
    background-launched mission -- exactly the failure ADR-008 B2 exists
    to prevent.

    signal.signal() overrides the inherited disposition, so this works
    however the process was launched. SIGTERM is included so a scripted
    shutdown gets the same controlled return-and-land as Ctrl-C."""
    def _handler(signum, _frame):
        try:
            name = signal.Signals(signum).name
        except ValueError:  # pragma: no cover
            name = str(signum)
        logger.warning("%s alindi -- gorev durduruluyor (donus + inis calisacak).", name)
        request_stop()

    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            signal.signal(sig, _handler)
        except (ValueError, OSError) as e:  # pragma: no cover -- non-main thread / unsupported
            logger.error("%s icin sinyal isleyici kurulamadi: %s", sig, e)


def _run_with_main_thread_gui(config: dict, mission_id: str) -> None:
    """macOS entrypoint (ADR-006): mission on a worker thread, GUI on main.

    Cocoa requires cv2 GUI calls on the process main thread. ADR-005 §3 keeps
    the dashboard's state/composition/lifecycle on its own dedicated thread and
    its §8 table forbids a direct cv2 call on the mission thread. Running the
    mission coroutine here on a worker thread satisfies all three at once: the
    mission thread still never touches cv2, the dashboard thread still composes,
    and the paint happens on a main thread that is no longer the mission's.

    `_run()` itself is used unmodified.
    """
    holder = {}
    mission_error = {}

    async def _wrapped():
        holder["loop"] = asyncio.get_running_loop()
        holder["task"] = asyncio.current_task()
        await _run(config, mission_id)

    def _mission_thread():
        try:
            asyncio.run(_wrapped())
        except asyncio.CancelledError:
            logger.info("Mission cancelled (shutdown requested from the UI).")
        except BaseException as e:  # noqa: BLE001 -- surfaced below, on the main thread
            mission_error["exc"] = e
        finally:
            holder["done"] = True

    # daemon=True (ADR-007 point 10): if the mission coroutine's own teardown
    # stalls -- e.g. MAVSDK's gRPC channel is already gone, which happened
    # after an aborted run -- a non-daemon thread blocks interpreter exit
    # forever in threading._shutdown, leaving main_gz.py alive holding
    # udp:14540/tcp:50051 and breaking the NEXT run with "Address already in
    # use". Daemonising bounds that: the process can always exit.
    thread = threading.Thread(target=_mission_thread, name="MissionRuntime", daemon=True)
    thread.start()

    stop_state = {"requested": False}

    def _request_mission_stop():
        stop_state["requested"] = True
        """ADR-008 B2 (A2 row 7): cancelling the mission task is now the
        START of a controlled shutdown, not the end of one.
        MasterMissionController.run() catches the CancelledError and flies
        the vehicle back to the start/finish checkpoint before landing,
        bounded by ABORT_RETURN_DEADLINE_S. Previously this cancel left the
        vehicle airborne: CancelledError is a BaseException, so master_fsm's
        `except Exception` handlers never saw it and _safe_land() was
        skipped entirely."""
        loop, task = holder.get("loop"), holder.get("task")
        if loop is not None and task is not None and not task.done():
            loop.call_soon_threadsafe(task.cancel)
            logger.warning("Mission durduruluyor -- arac baslangic/bitis noktasina donup inecek "
                           "(en fazla %.0fs).", ABORT_RETURN_DEADLINE_S)

    _install_signal_handlers(_request_mission_stop)

    window_open = False
    window_name = None
    painted = 0
    t_fps = time.time()
    fps = 0.0
    stopping = False
    try:
        # Paint until the mission finishes; ~30Hz, independent of the
        # dashboard's own (slower) composition cadence.
        while not holder.get("done"):
            item = MAIN_THREAD_PAINT.take()
            if item is not None:
                window_name, image = item
                if not window_open:
                    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
                    cv2.resizeWindow(window_name, image.shape[1], image.shape[0])
                    window_open = True
                cv2.imshow(window_name, image)
                painted += 1

            if window_open:
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27) and not stopping:
                    logger.info("Dashboard quit requested (key) -- stopping mission.")
                    _request_mission_stop()
                    # ADR-008 B2: deliberately NOT `break`. The vehicle is
                    # still airborne and is now flying its return-to-
                    # start/finish leg; breaking here would blank the
                    # dashboard for the whole descent, exactly when an
                    # operator most needs to watch it. The loop exits on
                    # holder["done"], i.e. once the mission thread has
                    # actually finished landing.
                    stopping = True
                if cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) < 1:
                    logger.info("Dashboard window closed -- stopping mission.")
                    _request_mission_stop()
                    # No window left to paint into, so this one does break --
                    # the bounded join below still waits for the return+land
                    # to complete before the process exits.
                    break

            now = time.time()
            if now - t_fps >= 5.0:
                fps = painted / (now - t_fps)
                logger.info("Dashboard paint loop: %.1f FPS (main thread)", fps)
                painted, t_fps = 0, now

            if stop_state["requested"]:
                stopping = True

            time.sleep(1.0 / 30.0)
    except KeyboardInterrupt:
        logger.info("Ctrl-C -- stopping mission.")
        _request_mission_stop()
    finally:
        # Bounded post-completion deadline (ADR-007 point 10): give the
        # mission thread a chance to finish its own teardown, but never wait
        # on it indefinitely -- combined with daemon=True this guarantees the
        # process exits and releases 14540/50051 for the next run.
        #
        # ADR-008 B2: raised from 15s. That budget predated the abort path
        # doing anything at all; now a cancel triggers a real
        # return-to-start/finish flight bounded by ABORT_RETURN_DEADLINE_S,
        # and a join shorter than that would abandon the mission thread
        # mid-return -- i.e. reintroduce the exact "vehicle left airborne"
        # failure this change exists to remove. Stays finite: the margin is
        # for teardown (camera/ops-center stop) only.
        shutdown_deadline_s = ABORT_RETURN_DEADLINE_S + 15.0
        thread.join(timeout=shutdown_deadline_s)
        if thread.is_alive():
            logger.error("Mission runtime did not finish within %.0fs of shutdown; "
                         "exiting anyway (daemon thread will be terminated).", shutdown_deadline_s)
        if window_open:
            try:
                cv2.destroyAllWindows()
                cv2.waitKey(1)  # let Cocoa actually tear the window down
            except Exception:  # noqa: BLE001
                pass

    if "exc" in mission_error:
        raise mission_error["exc"]


def main():
    # BUG FIX (runtime investigation, 2026-08-13): must run before any
    # core.*/gz_system.* module logs anything -- see configure_all_loggers()'s
    # own docstring for the full explanation of what was silently invisible
    # without this.
    configure_all_loggers()

    # gz-transport partition/IP must match the simulator's, or every Gazebo
    # subscription silently yields zero frames (macOS: the default
    # "<hostname>:<username>" partition drifts whenever the DHCP hostname
    # changes). Applied here so the mission entrypoint does not depend on a
    # shell having sourced gz_env.sh, and logged so drift is immediately
    # visible instead of showing up as a dead camera panel.
    apply_gz_env()
    logger.info("gz-transport env: %s", describe_gz_env())

    config_path = os.path.join(os.path.dirname(__file__), "config", "gz_system.yaml")
    with open(config_path, "r") as f:
        config = yaml.safe_load(f)

    mission_id = uuid.uuid4().hex[:12]

    if sys.platform == "darwin":
        _run_with_main_thread_gui(config, mission_id)
    else:
        # Linux/Windows behaviour is deliberately untouched: the mission owns
        # the main thread and MissionOpsDashboard paints on its own thread.
        asyncio.run(_run(config, mission_id))

if __name__ == "__main__":
    main()
