"""Standalone ground-station camera viewer on the CANONICAL gz_system pipeline.

This is the entrypoint `run_just_cam` uses. It deliberately adds no camera
architecture of its own: it drives `GzCameraSource`, which is the same
composition the mission uses (camera_service_manager spawns/supervises
camera_service.py, CameraClient consumes its ZeroMQ stream). Previously
`run_just_cam` routed into the flat legacy `process_manager.py` stack, which
ADR-005 identifies as superseded debris ("not the live system",
"a landmine ... recommend explicit archival").

cv2 display runs on this process's MAIN thread, which is what macOS Cocoa
requires. That is possible here precisely because there is no mission
coroutine competing for the main thread -- unlike MissionOpsDashboard inside
main_gz.py (see ADR-006).

FRAME CONTRACT: GzCameraSource.get_frame() raises RuntimeError whenever no
frame is currently buffered -- at startup before the first frame arrives, and
transiently if CameraClient's stream goes stale. The mission treats this as
retryable (gorev2_orchestrator.py: catch, log, keep looping at ~30Hz) and this
viewer matches that contract exactly. An earlier version called get_frame()
immediately with no handler, so the very first pre-first-frame RuntimeError
tore the viewer down about a second after start. Reconnection remains
CameraClient's job; nothing here duplicates it.
"""
import asyncio
import os
import sys

import cv2

from gz_system import camera_service_manager
from gz_system.gz_env import apply_gz_env, describe_gz_env
from gz_system.gz_camera_source import GzCameraSource

DEFAULT_TOPIC = ("/world/default/model/x500_mono_cam_down_0/link/camera_link"
                 "/sensor/camera/image")
DEFAULT_ZMQ = "tcp://127.0.0.1:5555"
WINDOW = "KURSAD40 - Camera (canonical gz_system pipeline)"

# Gazebo/PX4 can take a few seconds to produce the first frame after
# camera_service subscribes; override with KURSAD40_VIEWER_TIMEOUT_S or argv[3].
DEFAULT_FIRST_FRAME_TIMEOUT_S = 15.0
STALL_LOG_INTERVAL_S = 2.0
FPS_REPORT_INTERVAL_S = 2.0


async def _await_first_frame(camera: GzCameraSource, timeout_s: float):
    """Block until the first frame arrives, or give up with a useful message.

    Returns the frame, or None on timeout.
    """
    loop = asyncio.get_running_loop()
    t0 = loop.time()
    last_notice = 0.0

    while loop.time() - t0 < timeout_s:
        try:
            return await camera.get_frame()
        except RuntimeError:
            pass  # expected until the first frame lands -- see FRAME CONTRACT

        now = loop.time()
        if now - last_notice >= 1.0:
            print(f"[CAMERA_VIEWER] waiting for first frame... "
                  f"({now - t0:.0f}s/{timeout_s:.0f}s)")
            last_notice = now
        await asyncio.sleep(0.05)

    return None


async def _run(topic: str, zmq_addr: str, timeout_s: float) -> int:
    camera = GzCameraSource(topic, zmq_addr)
    await camera.start()

    loop = asyncio.get_running_loop()
    try:
        frame = await _await_first_frame(camera, timeout_s)
        if frame is None:
            print(f"\n[CAMERA_VIEWER] ERROR: no camera frame after {timeout_s:.0f}s.")
            print(f"[CAMERA_VIEWER] camera_service log: {camera_service_manager.LOG_FILE}")
            print("[CAMERA_VIEWER] Check that the simulator is running and that it "
                  "shares this process's GZ_PARTITION/GZ_IP "
                  f"({describe_gz_env()}).")
            return 1

        h, w = frame.shape[:2]
        print(f"[CAMERA_VIEWER] first frame: {w}x{h}")
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)

        frames = 0
        t_fps = loop.time()
        last_stall_log = 0.0

        while True:
            try:
                frame = await camera.get_frame()
                frames += 1
            except RuntimeError as e:
                # Transient by contract (stale stream / momentary gap).
                # CameraClient reconnects on its own; just keep the window
                # alive and say so at most once every STALL_LOG_INTERVAL_S.
                now = loop.time()
                if now - last_stall_log >= STALL_LOG_INTERVAL_S:
                    print(f"[CAMERA_VIEWER] no frame right now ({e}); waiting...")
                    last_stall_log = now
            else:
                cv2.imshow(WINDOW, frame)

            # waitKey also drives the Cocoa event loop; 'q' or ESC quits.
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                print("[CAMERA_VIEWER] quit requested")
                break
            if cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                print("[CAMERA_VIEWER] window closed")
                break

            now = loop.time()
            if now - t_fps >= FPS_REPORT_INTERVAL_S:
                print(f"[CAMERA_VIEWER] {frames / (now - t_fps):.1f} FPS")
                frames = 0
                t_fps = now

            await asyncio.sleep(1.0 / 60.0)

        return 0
    finally:
        await camera.stop()
        cv2.destroyAllWindows()


def main() -> int:
    apply_gz_env()
    print(f"[CAMERA_VIEWER] gz-transport env: {describe_gz_env()}")

    topic = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_TOPIC
    zmq_addr = sys.argv[2] if len(sys.argv) > 2 else DEFAULT_ZMQ
    if len(sys.argv) > 3:
        timeout_s = float(sys.argv[3])
    else:
        timeout_s = float(os.environ.get("KURSAD40_VIEWER_TIMEOUT_S",
                                         DEFAULT_FIRST_FRAME_TIMEOUT_S))

    try:
        return asyncio.run(_run(topic, zmq_addr, timeout_s))
    except KeyboardInterrupt:
        print("\n[CAMERA_VIEWER] interrupted, shutting down")
        return 0


if __name__ == "__main__":
    sys.exit(main())
