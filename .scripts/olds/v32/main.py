# main.py
import argparse
import time
import cv2
import os
import sys

from config import DEFAULT_GZ_TOPIC, DEFAULT_MAVSDK_URL, DEBUG_WINDOW_WIDTH, DEBUG_WINDOW_HEIGHT, CAMERA_ZMQ_ADDRESS
from camera_client import CameraClient
from vision import VisionWorker, SharedDetectionBuffer
from flight import MavController
from payload import PayloadManager
from memory import MissionMemory
from search import SearchPlanner
from servo import VisualServoController
from mission import MissionManager
from debug_view import UIWorker
from mission_types import UISnapshot, Event, event_bus

def parse_args():
    ap = argparse.ArgumentParser(description="v30 Modular Autonomy System")
    ap.add_argument("--mavsdk-url", type=str, default=DEFAULT_MAVSDK_URL,
                    help="MAVSDK connection URL")
    ap.add_argument("--no-display", action="store_true",
                    help="Disable OpenCV debug display")
    ap.add_argument("--sim-mode", action="store_true", default=True,
                    help="Run in simulation mode (default)")
    ap.add_argument("--real-mode", action="store_true",
                    help="Run in real-world hardware mode")
    ap.add_argument("--camera-only", action="store_true",
                    help="Connect only to Gazebo camera and display frames")
    return ap.parse_args()

def main():
    args = parse_args()

    sim_mode = not args.real_mode
    if not sim_mode:
        print("[INIT] Running in REAL_MODE. Hardware interfaces expected.")
    else:
        print("[INIT] Running in SIM_MODE.")

    print(f"[DEBUG] main.py process starts. PID: {os.getpid()}")

    # Initialize Camera Client
    reader = CameraClient(CAMERA_ZMQ_ADDRESS)
    reader.start()
    print(f"[INIT] Camera client listening on {CAMERA_ZMQ_ADDRESS}")

    if args.camera_only:
        print("[INIT] Running in CAMERA_ONLY mode.")
        prev_time = time.time()
        frame_count = 0
        cv2.namedWindow("V30 Camera Test", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("V30 Camera Test", DEBUG_WINDOW_WIDTH, DEBUG_WINDOW_HEIGHT)

        try:
            while True:
                frame = reader.get_frame()
                if frame is None:
                    time.sleep(0.01)
                    continue

                frame_count += 1
                now = time.time()
                if now - prev_time >= 1.0:
                    fps = frame_count / (now - prev_time)
                    print(f"[CAMERA] frames={frame_count} fps={fps:.1f}")
                    prev_time = now
                    frame_count = 0

                h, w = frame.shape[:2]
                cx, cy = w // 2, h // 2
                cv2.drawMarker(frame, (cx, cy), (0, 255, 0), cv2.MARKER_CROSS, 20, 2)

                cv2.imshow("V30 Camera Test", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        except KeyboardInterrupt:
            pass
        finally:
            reader.stop()
            cv2.destroyAllWindows()
        return

    # ==========================================
    # Autonomous Mission Core
    # ==========================================
    print("[INIT] Booting Mission Core Architecture...")

    shared_buffer = SharedDetectionBuffer()
    worker = VisionWorker(reader, shared_buffer)
    worker.start()

    mav = MavController(args.mavsdk_url)
    # mav.connected.wait() previously had NO timeout: if PX4 SITL isn't actually
    # reachable at args.mavsdk_url, this blocked forever with zero diagnostic
    # output -- from the terminal (and even in logs/mission.log) it looked
    # identical to a silent hang. Bound it and fail loud instead.
    print(f"[INIT] Waiting for MAVSDK connection at {args.mavsdk_url} ...")
    MAVSDK_CONNECT_TIMEOUT_S = 30.0
    connect_deadline = time.time() + MAVSDK_CONNECT_TIMEOUT_S
    mavsdk_ok = False
    while time.time() < connect_deadline:
        if mav.connected.wait(timeout=2.0):
            mavsdk_ok = True
            break
        print(f"[INIT] Still waiting for PX4 SITL at {args.mavsdk_url} ...")

    if not mavsdk_ok:
        print(f"[INIT] FATAL: No MAVSDK connection after {MAVSDK_CONNECT_TIMEOUT_S:.0f}s.")
        print(f"[INIT]   PX4 SITL is not reachable at {args.mavsdk_url}.")
        print(f"[INIT]   Fix: start the simulator first, in its own terminal:")
        print(f"[INIT]     ./safe_sitl_launcher.sh")
        print(f"[INIT]   then re-run ./run_mission_v30.")
        sys.exit(1)

    print("[INIT] MAVSDK Connected.")

    memory = MissionMemory()
    payload = PayloadManager(mav, sim_mode=sim_mode)
    search_planner = SearchPlanner()
    servo = VisualServoController()

    manager = MissionManager(memory, search_planner, servo)

    ui_worker = UIWorker()
    if not args.no_display:
        ui_worker.start()

    print("[INIT] All modules initialized. Starting 33Hz Flight Loop.")
    frame_id = 0
    recent_events = []
    def _on_event(ev):
        recent_events.append(ev)
        if len(recent_events) > 5:
            recent_events.pop(0)

        # Bind PayloadManager actions to state transitions via EventBus
        if ev.name == "TRANSITION_PAYLOAD_DROP":
            color = "red" if manager.locked_target_key == "blue_hexagon" else "blue"

            def _report_drop_result(success, _color=color):
                # Runs on the MAVSDK asyncio thread; publish back onto the event
                # bus so MissionManager can confirm the drop before advancing
                # instead of blindly assuming success after a fixed delay.
                event_bus.publish(Event(
                    name="PAYLOAD_DROP_RESULT",
                    source="PayloadManager",
                    payload={"success": success, "color": _color}
                ))

            payload.request_drop(color, callback=_report_drop_result)
        elif ev.name == "TRANSITION_PICKUP":
            target = memory.first_dropped_payload_color
            if target:
                def _report_pickup_result(success, _color=target):
                    # Runs on the MAVSDK asyncio thread; publish back onto the
                    # event bus so MissionManager can confirm the hook actually
                    # made contact before declaring pickup successful, instead
                    # of assuming success purely from reaching a geometric
                    # waypoint (see _state_pickup).
                    event_bus.publish(Event(
                        name="PAYLOAD_PICKUP_RESULT",
                        source="PayloadManager",
                        payload={"success": success, "color": _color}
                    ))
                payload.request_pickup_virtual(target, callback=_report_pickup_result)
        elif ev.name == "REQUEST_PAYLOAD_RELEASE":
            # KURSAD40 bonus mission: fired directly by _state_payload_release
            # once ground contact (0.30m AGL) is actually reached, NOT on
            # PAYLOAD_RELEASE state entry -- that state now also handles the
            # descent sub-phase before detach, so a TRANSITION_PAYLOAD_RELEASE
            # trigger would fire the detach request while still 3m up.
            def _report_release_result(success):
                # Runs on the MAVSDK asyncio thread; publish back onto the
                # event bus so MissionManager can confirm the hook actually
                # detached (HookAttachSystem's own ATTACH_STATE:false) before
                # declaring delivery successful, instead of assuming it from
                # reaching a geometric waypoint (see _state_payload_release).
                # REPLACES an earlier version that fired drop_picked_payload()
                # as a side effect of the RETURN_HOME transition itself --
                # fire-and-forget, with no confirmation ever reaching
                # MissionManager and no retry if the detach failed.
                event_bus.publish(Event(
                    name="PAYLOAD_RELEASE_RESULT",
                    source="PayloadManager",
                    payload={"success": success}
                ))
            payload.request_release(callback=_report_release_result)

    event_bus.subscribe(_on_event)
    event_bus.subscribe(manager.process_event)

    try:
        while True:
            # 1. Gather async telemetry/vision inputs
            frame = reader.get_frame()
            if frame is None:
                time.sleep(0.005)
                continue

            h, w = frame.shape[:2]
            target_data = shared_buffer.get()
            telemetry = {
                "alt": mav.alt(),
                # None until the position_velocity_ned stream has actually
                # delivered a real sample -- mav.get_ned() is a (0,0,0)
                # placeholder before that, and mission.py's one-shot ground
                # start capture must never mistake the placeholder for a
                # real reading (see MavController.ned_ready).
                "ned": mav.get_ned() if mav.ned_ready else None,
                "gps": mav.get_gps(),
                "armed": mav.is_armed,
                "offboard": mav.is_offboard(),
                "yaw": mav.yaw_deg,
                # KURSAD40 post-attach stability fix: real attitude/rate
                # signals so MissionManager can detect "has the vehicle
                # settled" from actual telemetry rather than a timer.
                "roll": mav.roll_deg,
                "pitch": mav.pitch_deg,
                "roll_rate": mav.roll_rate,
                "pitch_rate": mav.pitch_rate,
                "yaw_rate": mav.yaw_rate_actual,
                # Landing redesign: real velocity + PX4's own landed-state
                # detector, flattened to plain values here so mission.py
                # stays free of any direct mavsdk import/type dependency.
                "velocity_ned": mav.get_velocity_ned(),
                "landed_state": mav.landed_state.name if mav.landed_state else "UNKNOWN",
                "local_position_ok": bool(mav.health.is_local_position_ok) if mav.health else False,
            }

            # 2. Update MissionManager
            manager.update_telemetry(telemetry)
            manager.update_vision(target_data, (w, h))

            # Update Vision Worker's filter dynamically
            worker.set_filter(manager.state_data.active_filter)

            # 3. Step State Machine (Synchronous Tick)
            intent = manager.step()

            # 4. Delegate Intent to FlightController
            mav.submit(mav.execute_intent(intent))

            # 5. Dashboard / Telemetry Out
            # Everything mission-critical for this tick (telemetry, state
            # machine step, intent execution) has already happened above.
            # Everything from here down is UI-only, wrapped so that a bug in
            # snapshot construction or rendering can never take the mission
            # loop down with it -- mission logic must never depend on the UI.
            if not args.no_display:
                try:
                    payload_status = f"Dropped: {', '.join(memory.confirmed_drops)}" if memory.confirmed_drops else "None dropped"

                    # Phase 6: Construct Immutable UISnapshot
                    # Safely get metrics if available
                    fps_info = getattr(worker, 'last_fps_info', {"inf": 0, "trk": 0, "lat": 0})

                    snapshot = UISnapshot(
                        timestamp=time.time(),
                        frame_id=frame_id,
                        frame_bgr=frame,
                        target_data=target_data,
                        active_filter=manager.state_data.active_filter,
                        vision_fps=0, # Camera FPS is handled inside UIWorker or worker, we can just pass 0 for now
                        detector_fps=fps_info.get("inf", 0),
                        tracker_fps=fps_info.get("trk", 0),
                        pipeline_latency=fps_info.get("lat", 0),
1
                        mission_state=manager.state_data.current_state,
                        mission_phase=manager.state_data.active_mode,
                        mission_time=manager.state_data.state_time(),
                        completed_targets=list(memory.completed_targets),
                        pickup_target=memory.get_target_for_pickup(),
                        payload_status=payload_status,

                        flight_mode=str(mav.flight_mode).replace("FlightMode.", "") if mav.flight_mode else "UNKNOWN",
                        is_armed=mav.is_armed,
                        alt_rel=telemetry["alt"],
                        ned=telemetry["ned"],
                        velocity=intent.velocity if hasattr(intent, 'velocity') else (0,0,0),
                        yaw_deg=mav.yaw_deg,

                        flight_intent=intent.mode,
                        descent_gate=manager.descent_gate,
                        aligned_frames=servo.aligned_frames,

                        recent_events=list(recent_events),
                        dropped_snapshot_count=ui_worker.dropped_snapshots,
                        safety_banner=f"STATUS: {manager.state_data.current_state}",
                        is_logging=True,
                        ui_latency_ms=ui_worker.last_render_time_ms,

                        start_ned=((memory.start_position["north"], memory.start_position["east"], -memory.start_position["alt"])
                                   if memory.start_position else None),
                        start_heading=(memory.start_position["heading"] if memory.start_position else None),
                        drop1_ned=(memory.first_drop_position["ned"] if memory.first_drop_position else None),
                        drop1_color=memory.first_dropped_payload_color,
                        drop2_ned=(memory.second_drop_position["ned"] if memory.second_drop_position else None),
                        drop2_color=memory.second_dropped_payload_color,
                    )

                    ui_worker.update(snapshot)
                except Exception as e:
                    print(f"[MAIN] WARNING: dashboard update failed, continuing mission: {e}")

            time.sleep(0.03) # 33Hz Tick
            frame_id += 1

    except KeyboardInterrupt:
        print("[INIT] Keyboard interrupt.")
    finally:
        mav.submit(mav.stop_offboard())
        worker.stop()
        reader.stop()
        ui_worker.stop()
        print("[INIT] Shutdown complete.")

if __name__ == "__main__":
    main()
