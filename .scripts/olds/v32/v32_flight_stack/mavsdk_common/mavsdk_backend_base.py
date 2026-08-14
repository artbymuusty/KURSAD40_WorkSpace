import asyncio
import logging
from typing import Tuple
from mavsdk import System
from mavsdk.offboard import VelocityBodyYawspeed, PositionNedYaw
from mavsdk.mission import MissionItem, MissionPlan
from core.interfaces.i_flight_backend import IFlightBackend
from core.config.parameters import OFFBOARD_SETPOINT_INTERVAL_S
from core.telemetry.event_bus import NULL_PUBLISHER, EventPublisher
from core.telemetry.events import Category, Event, Severity

logger = logging.getLogger(__name__)

# ADR-004 §9.1: MAVSDK exposes drone.telemetry.speed_m_s()-equivalent cruise
# speed for MissionItem generation; a fixed default is used until the team
# fills NORMAL_MISSION_SPEED_M_S in parameters.py (still None/TODO there).
_DEFAULT_MISSION_SPEED_M_S = 5.0

class MavsdkBackendBase(IFlightBackend):
    """
    Hem real_system hem gz_system için ortak MAVSDK uçuş kontrol mantığı.
    core/ bu sınıftan habersizdir, yalnızca adaptörler kullanır.

    `publisher` is optional (defaults to a no-op) so existing callers/tests
    that construct this without one keep working unchanged (ADR-004 §14).
    """
    def __init__(self, connection_string: str, publisher: EventPublisher = NULL_PUBLISHER):
        self.connection_string = connection_string
        self.drone = System()
        self.publisher = publisher
        # BUG FIX (runtime investigation, 2026-08-13): is_mission_finished()
        # used to open a brand-new `async for ... in
        # self.drone.mission.mission_progress()` subscription on every
        # single call, then take the first pushed value. Unlike
        # position/attitude/flight_mode (which PX4 streams continuously at
        # flight-control rate, so a fresh subscription's first value
        # arrives almost immediately), mission_progress is event-driven --
        # MAVSDK/PX4 only pushes a new value when the mission actually
        # advances (e.g. a waypoint is reached). A fresh subscription
        # started between two such events can block for many seconds
        # waiting for the next one. _search_and_engage_loop's own while
        # condition calls is_mission_finished() every iteration, so this
        # silently throttled the ENTIRE search loop -- including
        # TargetValidator's consecutive-frame counting -- down to the rate
        # of PX4 mission-progress events instead of the intended ~10Hz
        # detection-processing rate. Proven via a real Gazebo run: 633
        # VISION_FRAME_PROCESSED events (~10Hz, correct) against only 3
        # TRACK_STATE_UPDATED events over the same 46s search window --
        # continuous detections were never being fed to the validator often
        # enough to reach its 5-consecutive-frame threshold before the
        # underlying PX4 mission ended. Fixed the same way every other
        # telemetry field in this class should be (see get_flight_mode
        # etc., a broader but lower-severity instance of the same pattern
        # not touched here since it's not what caused the proven failure):
        # subscribe ONCE in the background, cache the latest value, let
        # is_mission_finished() return the cache instantly.
        self._mission_finished_cache: bool = False

    def _publish(self, code, message="", severity=Severity.INFO, category=Category.TELEMETRY, data=None):
        self.publisher.publish(Event(
            code=code, subsystem="MavsdkBackendBase", category=category, severity=severity,
            message=message, data=data or {},
        ))

    async def connect(self) -> None:
        logger.info(f"Drone'a bağlanılıyor: {self.connection_string}")
        await self.drone.connect(system_address=self.connection_string)

        logger.info("Drone bağlantısı bekleniyor...")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                logger.info("Drone bağlandı!")
                self._publish("CONNECTED", f"connected to {self.connection_string}", category=Category.LIFECYCLE,
                              data={"connected": True})
                break

        # See _mission_finished_cache's own comment (__init__) for why this
        # runs once in the background instead of is_mission_finished()
        # subscribing fresh on every call.
        asyncio.ensure_future(self._mission_progress_watcher())

    async def _mission_progress_watcher(self) -> None:
        while True:
            try:
                async for progress in self.drone.mission.mission_progress():
                    self._publish("MISSION_PROGRESS", severity=Severity.DEBUG,
                                  data={"progress_current": progress.current, "progress_total": progress.total})
                    # total == 0 means "no mission was ever uploaded", not
                    # "the mission is complete" -- treating them the same
                    # is exactly the bug class ADR-005 §5 closed. A real
                    # mission is "finished" only once it has actually had
                    # items and completed.
                    self._mission_finished_cache = (
                        progress.total > 0 and progress.current == progress.total
                    )
            except Exception as e:  # noqa: BLE001 -- a stream hiccup must never take the mission down
                logger.warning(f"[MAVSDK] mission_progress stream hatasi, 1s icinde yeniden baglaniliyor: {e}")
                await asyncio.sleep(1.0)
                
    async def get_health_summary(self) -> dict:
        """ADR-004 §9.1: EKF/GPS-fix/pre-arm-check status is available from
        MAVSDK today (drone.telemetry.health()) and was previously never
        read anywhere in this codebase -- meaning a PX4 arm rejection
        surfaced only as a bare 'COMMAND_DENIED' string with zero indication
        of *which* pre-arm check actually failed. This closes that gap."""
        async for health in self.drone.telemetry.health():
            return {
                "is_gyrometer_calibration_ok": health.is_gyrometer_calibration_ok,
                "is_accelerometer_calibration_ok": health.is_accelerometer_calibration_ok,
                "is_magnetometer_calibration_ok": health.is_magnetometer_calibration_ok,
                "is_local_position_ok": health.is_local_position_ok,
                "is_global_position_ok": health.is_global_position_ok,
                "is_home_position_ok": health.is_home_position_ok,
                "is_armable": health.is_armable,
            }
        return {}

    async def arm(self) -> None:
        logger.info("Drone arm ediliyor...")
        try:
            await self.drone.action.arm()
        except Exception as e:
            health = await self.get_health_summary()
            failing_checks = [k for k, v in health.items() if k != "is_armable" and v is False]
            logger.error(f"Arm reddedildi: {e} -- health: {health}")
            self._publish("ARM_REJECTED", f"{e} -- failing pre-arm checks: {failing_checks or 'unknown (see health data)'}",
                          severity=Severity.CRITICAL, category=Category.LIFECYCLE,
                          data={"error": str(e), "health": health, "failing_checks": failing_checks})
            raise
        self._publish("ARMED", "vehicle armed", category=Category.LIFECYCLE, data={"armed": True})

    async def takeoff(self, target_altitude_m: float) -> None:
        logger.info(f"Kalkış yapılıyor (Hedef: {target_altitude_m}m)...")
        await self.drone.action.set_takeoff_altitude(target_altitude_m)
        await self.drone.action.takeoff()
        self._publish("TAKEOFF_ISSUED", f"target_altitude_m={target_altitude_m}", category=Category.LIFECYCLE,
                      data={"target_altitude_m": target_altitude_m})
        
    async def land(self) -> None:
        logger.info("İniş yapılıyor...")
        await self.drone.action.land()
        
    async def start_offboard(self) -> None:
        logger.info("Offboard başlatılıyor...")
        initial_setpoint = VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
        await self.drone.offboard.set_velocity_body(initial_setpoint)
        await self.drone.offboard.start()
        
    async def stop_offboard(self) -> None:
        logger.info("Offboard durduruluyor...")
        await self.drone.offboard.stop()
        
    async def goto_position_ned(self, north_m: float, east_m: float, down_m: float, yaw_deg: float) -> None:
        setpoint = PositionNedYaw(north_m, east_m, down_m, yaw_deg)
        await self.drone.offboard.set_position_ned(setpoint)

    async def goto_position_ned_and_hold(self, north_m: float, east_m: float, down_m: float,
                                          yaw_deg: float, duration_s: float) -> None:
        """GAP FIX (operator revision, 2026-08-13): Görev 3 fazları
        (pickup/transport/redrop/finish) previously called goto_position_ned()
        once and then asyncio.sleep() -- the exact single-shot-then-silence
        bug already fixed elsewhere in this file for hold_position(), just
        never applied here because these phases were, until now, entirely
        simulated placeholders. Streams the same setpoint at
        OFFBOARD_SETPOINT_INTERVAL_S for the whole duration so PX4 never
        sees a setpoint gap long enough to auto-exit Offboard."""
        setpoint = PositionNedYaw(north_m, east_m, down_m, yaw_deg)
        deadline = asyncio.get_event_loop().time() + duration_s
        while asyncio.get_event_loop().time() < deadline:
            await self.drone.offboard.set_position_ned(setpoint)
            await asyncio.sleep(OFFBOARD_SETPOINT_INTERVAL_S)


    async def set_velocity_body(self, forward_m_s: float, right_m_s: float, down_m_s: float, yaw_rate_deg_s: float) -> None:
        setpoint = VelocityBodyYawspeed(forward_m_s, right_m_s, down_m_s, yaw_rate_deg_s)
        await self.drone.offboard.set_velocity_body(setpoint)
        
    async def hold_position(self, duration_s: float) -> None:
        """Offboard modunda sıfır hız vererek konum koruma.

        BUG FIX (operator-reported): this used to send exactly ONE
        zero-velocity setpoint and return immediately -- the caller
        (CenteringController.hover_and_confirm) then did a bare
        asyncio.sleep(duration_s) with nothing streamed to PX4 during it.
        PX4 auto-exits Offboard after ~500ms without a new setpoint, so the
        vehicle was falling out of Offboard mid-hover on every single
        engagement, regardless of whether go_to_and_center() had just
        fixed its own streaming gap. This now owns the full duration
        itself, streaming a hold setpoint at OFFBOARD_SETPOINT_INTERVAL_S
        the whole time."""
        deadline = asyncio.get_event_loop().time() + duration_s
        while asyncio.get_event_loop().time() < deadline:
            await self.set_velocity_body(0.0, 0.0, 0.0, 0.0)
            await asyncio.sleep(OFFBOARD_SETPOINT_INTERVAL_S)
        
    async def get_position_ned(self) -> Tuple[float, float, float]:
        async for pos in self.drone.telemetry.position_velocity_ned():
            return (pos.position.north_m, pos.position.east_m, pos.position.down_m)
        return (0.0, 0.0, 0.0)

    async def get_velocity_ned(self) -> Tuple[float, float, float]:
        """BUG FIX (regression investigation, 2026-08-13): position_velocity_ned()
        already carries velocity alongside position, but get_position_ned()
        above discards it. Added so goto_global_position_and_wait() can
        gate its convergence condition on 3D speed, not position alone --
        see that method's own BUG FIX comment for the full root-cause
        proof (a position-only convergence check fired while the vehicle
        was moving at ~11 m/s through the target)."""
        async for pos in self.drone.telemetry.position_velocity_ned():
            return (pos.velocity.north_m_s, pos.velocity.east_m_s, pos.velocity.down_m_s)
        return (0.0, 0.0, 0.0)

    async def get_flight_mode(self) -> str:
        """ONBOARD (Mission) vs OFFBOARD -- the operator's single most
        useful at-a-glance signal for "is the vehicle following its
        pre-planned route right now, or is Gorev2Orchestrator actively
        flying it toward a target." Read from real MAVSDK telemetry, not
        inferred from which function we last called, so it stays correct
        even if PX4 rejects a requested mode change."""
        async for mode in self.drone.telemetry.flight_mode():
            return str(mode)
        return "UNKNOWN"

    async def get_global_position(self) -> Tuple[float, float, float]:
        async for pos in self.drone.telemetry.position():
            position = (pos.latitude_deg, pos.longitude_deg, pos.relative_altitude_m)
            flight_mode = await self.get_flight_mode()
            # ADR-004 §9.1: this is the Flight Backend heartbeat -- called
            # every orchestrator tick, so HealthMonitor can detect a frozen
            # telemetry stream the same way it detects a crashed one.
            self._publish("VEHICLE_TELEMETRY", severity=Severity.DEBUG,
                          data={"connected": True, "position": position, "flight_mode": flight_mode})
            return position
        return (0.0, 0.0, 0.0)

    async def get_yaw_deg(self) -> float:
        async for euler in self.drone.telemetry.attitude_euler():
            return euler.yaw_deg
        return 0.0

    @staticmethod
    def _to_mission_items(waypoints: list, speed_m_s: float = _DEFAULT_MISSION_SPEED_M_S) -> list:
        """waypoints: list of (lat, lon, alt_rel_m). Builds real MAVSDK
        MissionItems -- BUG FIX (ADR-005 §5): the previous implementation
        ignored this argument entirely and always uploaded an empty
        MissionPlan([]), which made PX4 report the mission "finished"
        (0==0) before ever flying anywhere."""
        items = []
        for lat, lon, alt in waypoints:
            items.append(MissionItem(
                lat, lon, alt, speed_m_s,
                True,               # is_fly_through
                float("nan"), float("nan"),  # gimbal pitch/yaw -- unused, no gimbal
                MissionItem.CameraAction.NONE,
                float("nan"),       # loiter_time_s
                float("nan"),       # camera_photo_interval_s
                float("nan"),       # acceptance_radius_m -- PX4 default
                float("nan"),       # yaw_deg -- no forced heading
                float("nan"),       # camera_photo_distance_m
                MissionItem.VehicleAction.NONE,
            ))
        return items

    async def upload_mission(self, waypoints: list) -> None:
        requested_count = len(waypoints)
        self._publish("MISSION_UPLOAD_REQUESTED", f"{requested_count} waypoints",
                      category=Category.LIFECYCLE, data={"requested_item_count": requested_count})

        mission_items = self._to_mission_items(waypoints)
        mission_plan = MissionPlan(mission_items)
        await self.drone.mission.upload_mission(mission_plan)

        # Round-trip verification (ADR-004 §7.2): read the plan back instead
        # of trusting the upload call's mere absence of an exception -- this
        # is the assertion that would have caught the empty-mission bug at
        # the source instead of it surfacing as "the drone just hovers."
        uploaded_plan = await self.drone.mission.download_mission()
        uploaded_count = len(uploaded_plan.mission_items)

        if uploaded_count != requested_count:
            self._publish("MISSION_UPLOAD_MISMATCH",
                          f"requested={requested_count} uploaded={uploaded_count}",
                          severity=Severity.CRITICAL, category=Category.LIFECYCLE,
                          data={"requested_item_count": requested_count, "uploaded_item_count": uploaded_count})
            raise RuntimeError(
                f"MISSION_UPLOAD_MISMATCH: requested {requested_count} waypoints, "
                f"PX4 reports {uploaded_count} uploaded. Refusing to start_mission() "
                f"on an unverified upload."
            )

        self._publish("MISSION_UPLOAD_CONFIRMED", f"{uploaded_count} waypoints confirmed",
                      category=Category.LIFECYCLE,
                      data={"requested_item_count": requested_count, "uploaded_item_count": uploaded_count})

    async def confirm_existing_mission(self) -> int:
        """The operator defines the search route in QGroundControl before
        flight -- this system must never generate and upload its own route
        over it (that silently overwrites whatever the operator planned,
        which is exactly what was happening before this fix). This only
        reads back whatever mission is already on the vehicle and reports
        how many items it has; it never uploads anything."""
        mission_plan = await self.drone.mission.download_mission()
        item_count = len(mission_plan.mission_items)

        if item_count == 0:
            self._publish("MISSION_ROUTE_MISSING", "no mission found on vehicle -- operator must define "
                          "waypoints in QGroundControl before RUN MISSION",
                          severity=Severity.CRITICAL, category=Category.LIFECYCLE,
                          data={"item_count": 0})
        else:
            self._publish("MISSION_ROUTE_CONFIRMED", f"{item_count} items already on vehicle (from QGroundControl)",
                          category=Category.LIFECYCLE, data={"item_count": item_count})

        return item_count

    async def start_mission(self) -> None:
        await self.drone.mission.start_mission()
        self._publish("MISSION_STARTED_ONBOARD", category=Category.LIFECYCLE)

    async def is_mission_finished(self) -> bool:
        """See _mission_finished_cache's own comment (__init__) -- this
        used to subscribe fresh to mission_progress() on every call, which
        is what actually throttled the whole search loop. Now just reads
        the value _mission_progress_watcher() keeps current in the
        background."""
        return self._mission_finished_cache
        
    async def switch_to_offboard_from_mission(self) -> None:
        logger.info("Mission'dan Offboard'a geçiş yapılıyor...")
        await self.drone.mission.pause_mission()
        # Offboard başlatma kısmı orchestrator'da ayrıca çağrılacak
