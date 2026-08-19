import asyncio
import logging
import math
import time
from core.interfaces.i_payload_actuator import IPayloadActuator
from core.config.parameters import (
    PAYLOAD_DETACH_BURST_COUNT, PAYLOAD_DETACH_BURST_INTERVAL_S,
    PAYLOAD_DETACH_CONFIRM_TIMEOUT_S, PAYLOAD_DETACH_POLL_INTERVAL_S,
    PAYLOAD_DETACH_SEPARATION_M, PAYLOAD_EXPECTED_REST_Z_M,
    PAYLOAD_ON_TARGET_Z_TOLERANCE_M,
)
from gz_system.gz_pose_monitor import GzPoseMonitor

logger = logging.getLogger(__name__)

# GAP FIX (Görev 2 Rapor Bölüm 12/13): IPayloadActuator's own docstring commits
# gz_system to a real Gazebo call with "TODO YOKTUR" -- the two Görev-2 release
# methods below previously just slept and returned True unconditionally, so a
# passing Görev-2 run never actually dropped anything in the sim. The real,
# compiled-and-SDF-wired mechanism is payload::PayloadDropSystem (confirmed
# against src/modules/simulation/gz_plugins/payload_drop/payload_drop_system.cc
# and its <plugin> block in Tools/simulation/gz/models/x500_mono_cam_down/model.sdf,
# NOT the simpler joint-removal stub of the same name under
# Tools/simulation/gz/plugins/payload_drop -- that one is not what's loaded).
# It exposes a color-addressed path: publishing a StringMsg "red"/"blue" on
# PAYLOAD_DROP_COLOR_TOPIC drops exactly that payload once, independent of
# call order, and it self-confirms per color via PAYLOAD_DROP_STATE_TOPIC
# ("<color>:true"/"<color>:false"). The `gz topic` CLI publish is the same
# known-working trigger mechanism already proven by .scripts/denemePayload.py
# and the old flat v32/payload.py's _gazebo_boolean_drop.
#
# Physical-color mapping (preserved unchanged from flat v32/payload.py,
# where it was already established as a deliberate team assignment, not
# incidental): RED payload <-> Mavi Altıgen target, BLUE payload <-> Kırmızı
# Üçgen target. Payload 1 (mavi_altigen) is always dropped before payload 2
# (kirmizi_ucgen) by PayloadInterlock, so this also lines up with the
# plugin's own legacy RED-then-BLUE stage_ ordering, though the color-select
# path used here does not depend on that ordering at all.
PAYLOAD_DETACH_TOPIC = "/payload/detach/%s"   # ADR-011: %s = red|blue
PAYLOAD_DROP_STATE_TOPIC = "/payload_drop_state"
GZ_STATE_LISTEN_TIMEOUT_S = 3.0


VEHICLE_MODEL_NAME = "x500_mono_cam_down_0"
PAYLOAD_MODEL = "payload_%s"

# Sim ground truth, read straight off Tools/simulation/gz/worlds/default.sdf.
# NOT mission input -- the mission is vision-driven and never learns where a
# target is. This exists only so a post-drop observation can be scored
# honestly ("0.41 m from the hexagon centre") instead of merely "above
# ground", which was the weak check that let a 4.9 m miss pass as a success
# on the first ADR-011 flight (F3).
TARGET_CENTERS = {"MAVI_ALTIGEN": (0.0, 15.0), "KIRMIZI_UCGEN": (0.0, 40.0)}
SHAPE_TO_COLOR = {"MAVI_ALTIGEN": "red", "KIRMIZI_UCGEN": "blue"}


class GzPayloadActuator(IPayloadActuator):
    def __init__(self, gazebo_service_name: str, world_name: str = "default",
                 pose_monitor: GzPoseMonitor = None):
        self.gazebo_service_name = gazebo_service_name
        # W4.3: only needed to address the dynamic_pose topic when reporting
        # where a released payload came to rest.
        self.world_name = world_name
        # F2: injected rather than created here so the one gz-transport
        # discovery cost is paid once, at mission start, and not at the
        # instant the servo fires.
        self.pose_monitor = pose_monitor or GzPoseMonitor(world_name)
        # F2 reporting: servo command -> observed separation, per colour.
        self.detach_latency_s = {}

    def _relative_drop(self, color: str):
        """Vehicle z minus payload z. Constant (the mount offset) while the
        joint holds; grows the moment the body is free. Measured relative to
        the vehicle rather than to the ground so a drone that is climbing
        away cannot be mistaken for a payload that is falling."""
        payload = self.pose_monitor.get(PAYLOAD_MODEL % color)
        vehicle = self.pose_monitor.get(VEHICLE_MODEL_NAME)
        if payload is None or vehicle is None:
            return None
        return vehicle[2] - payload[2]

    def measure_mount_vector(self, shape_type: str):
        """Where the payload actually hangs, measured at the instant of
        release rather than reasoned about.

        This exists because reasoning about it failed twice. The world SDF
        mounts the payloads at y = -+0.28, but that y is in Gazebo's FLU
        model frame at SPAWN yaw, while the aim correction is consumed in
        PX4's FRD body frame at FLIGHT yaw -- and the vehicle sits at ~86
        deg of Gazebo yaw once it is flying north. Two sign/frame guesses
        produced two wrong flights (0.243 m -> 0.850 m, then -0.299 m).
        Reporting the measured vector, in both frames, ends the guessing.

        Returns None when either pose is unknown."""
        color = SHAPE_TO_COLOR.get(shape_type)
        if color is None:
            return None
        payload = self.pose_monitor.get(PAYLOAD_MODEL % color)
        vehicle = self.pose_monitor.get(VEHICLE_MODEL_NAME)
        quat = self.pose_monitor.get_quat(VEHICLE_MODEL_NAME)
        if payload is None or vehicle is None or quat is None:
            return None
        dx, dy = payload[0] - vehicle[0], payload[1] - vehicle[1]
        qx, qy, qz, qw = quat
        yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        # World delta -> vehicle FLU (X forward, Y left) -> PX4 FRD right.
        forward = dx * math.cos(yaw) + dy * math.sin(yaw)
        left = -dx * math.sin(yaw) + dy * math.cos(yaw)
        return {"world_dx": round(dx, 4), "world_dy": round(dy, 4),
                "gazebo_yaw_deg": round(math.degrees(yaw), 2),
                "body_forward_m": round(forward, 4),
                "body_right_m": round(-left, 4)}

    async def _publish_detach(self, color: str) -> bool:
        topic = PAYLOAD_DETACH_TOPIC % color
        cmd = ["gz", "topic", "-t", topic, "-m", "gz.msgs.Empty", "-p", ""]
        try:
            proc = await asyncio.create_subprocess_exec(
                *cmd, stdout=asyncio.subprocess.DEVNULL,
                stderr=asyncio.subprocess.PIPE)
            _stdout, stderr = await proc.communicate()
            if proc.returncode != 0:
                logger.error("Gazebo detach komutu basarisiz (%s): %s",
                             color, stderr.decode().strip())
                return False
            return True
        except FileNotFoundError:
            logger.error("`gz` CLI bulunamadi -- Gazebo ortami PATH'te degil.")
            return False
        except Exception as e:  # noqa: BLE001
            logger.error("Gazebo detach komutu istisna verdi (%s): %s", color, e)
            return False

    async def _delayed_publish(self, color: str, delay_s: float) -> bool:
        if delay_s:
            await asyncio.sleep(delay_s)
        return await self._publish_detach(color)

    async def _burst_detach(self, color: str) -> bool:
        """Publishes the detach message several times in quick succession.

        gz-transport is a slow joiner: a publisher that advertises and sends
        in the same breath can lose the message, because the subscriber (the
        DetachableJoint plugin) has not finished connecting. Each
        `gz topic -p` is a fresh short-lived process, so it hits that window
        every single time -- the leading explanation for the several-second
        late detach measured on the first ADR-011 flight. The burst costs
        nothing; the confirmation poll below is what proves it worked."""
        sent = await asyncio.gather(*[
            self._delayed_publish(color, i * PAYLOAD_DETACH_BURST_INTERVAL_S)
            for i in range(PAYLOAD_DETACH_BURST_COUNT)])
        return any(sent)

    async def _await_separation(self, color: str, baseline, timeout_s: float,
                                started: float = None):
        """Polls the pose cache until the payload has visibly left the
        vehicle. Returns the latency in seconds, or None if it has not
        separated within the window.

        `started` is the SERVO COMMAND instant, not the moment polling
        began. Timing from the start of polling instead would have silently
        subtracted the entire publish cost and reported 0.000 s for a
        release that actually took the best part of a second."""
        started = time.monotonic() if started is None else started
        while time.monotonic() - started < timeout_s:
            current = self._relative_drop(color)
            if (current is not None
                    and abs(current - baseline) > PAYLOAD_DETACH_SEPARATION_M):
                return time.monotonic() - started
            if self._at_rest_height(color):
                return time.monotonic() - started
            await asyncio.sleep(PAYLOAD_DETACH_POLL_INTERVAL_S)
        return None

    def _at_rest_height(self, color: str) -> bool:
        """T1: a payload released from very low down has nowhere to fall.

        Separation is normally judged on the body dropping away from the
        vehicle, which needs a fall. On the Phase 13 flight payload 2 was
        released at 0.159 m -- its underside already resting on the ground
        -- so detaching moved it by nothing at all, the release was reported
        UNCONFIRMED, and the vehicle held for the full 60 s over a payload
        that was sitting exactly where it belonged. A body already at its
        rest height is released whether or not it moved to get there.

        Safe against false positives: while attached at a normal 0.45 m
        release the payload sits at ~0.474 m, an order of magnitude above
        the rest band."""
        payload = self.pose_monitor.get(PAYLOAD_MODEL % color)
        if payload is None:
            return False
        return abs(payload[2] - PAYLOAD_EXPECTED_REST_Z_M) <= PAYLOAD_ON_TARGET_Z_TOLERANCE_M

    async def _publish_color_drop(self, color: str) -> bool:
        """ADR-011 + F2: releases one payload, and CONFIRMS it separated.

        Was a StringMsg on /payload_drop_color, which made PayloadDropSystem
        SPAWN a new model -- and a runtime-spawned body gets no reliable
        collision pairs in this gz-sim build, so every drop fell through the
        world (measured: a real release logged at z=-0.72, still falling).
        The payload now exists from world load and is simply released here.

        Returns True only when separation was OBSERVED. False means the
        payload is still attached and the caller must not climb away --
        precisely the failure that put payload 2 down 4.9 m off the triangle
        while the log cheerfully said RELEASED."""
        topic = PAYLOAD_DETACH_TOPIC % color
        baseline = self._relative_drop(color)
        logger.info("Gazebo DetachableJoint tetikleniyor: %s", topic)

        # The burst runs CONCURRENTLY with the watch. Awaiting it first would
        # mean the clock only starts after ~1 s of `gz topic` process cost,
        # and the reported latency would exclude the very delay it exists to
        # measure -- the first run of this code duly reported 0.000 s.
        servo_at = time.monotonic()
        burst = asyncio.create_task(self._burst_detach(color))

        if baseline is None:
            if not await burst:
                logger.error("Gazebo detach mesaji hic yayinlanamadi (%s).", color)
                return False
            # No pose data at all: the monitor never started, or Gazebo has
            # not published this body yet. We cannot tell attached from
            # separated, so we must claim neither. Reported as unknown (not
            # failure) so a missing observer can never ground a flight.
            logger.warning("Yuk pozu okunamadi (%s) -- ayrilma DOGRULANAMADI, "
                           "basarisiz da sayilmiyor.", color)
            self.detach_latency_s[color] = None
            return True

        latency = await self._await_separation(color, baseline,
                                               PAYLOAD_DETACH_CONFIRM_TIMEOUT_S, servo_at)
        if not await burst:
            logger.error("Gazebo detach mesaji hic yayinlanamadi (%s).", color)
            return False
        if latency is None:
            logger.warning("Yuk %s ayrilmadi -- detach yeniden yayinlaniyor.", color)
            await self._burst_detach(color)
            latency = await self._await_separation(
                color, baseline, 2 * PAYLOAD_DETACH_CONFIRM_TIMEOUT_S, servo_at)

        self.detach_latency_s[color] = latency
        if latency is None:
            logger.error("[PAYLOAD_DETACH_UNCONFIRMED] %s: yuk hala araca bagli.", color)
            return False
        logger.info("Yuk ayrildi (%s), gecikme %.3f s.", color, latency)
        return True

    async def is_release_confirmed(self, shape_type: str) -> bool:
        """F2: lets the caller keep holding position and re-checking while a
        release is unconfirmed, instead of climbing away on a hope."""
        color = SHAPE_TO_COLOR.get(shape_type)
        if color is None:
            return False
        payload = self.pose_monitor.get(PAYLOAD_MODEL % color)
        vehicle = self.pose_monitor.get(VEHICLE_MODEL_NAME)
        if payload is None or vehicle is None:
            return False
        # Absolute test, not a delta: by now the body should be on the
        # ground, well below a vehicle still hovering at release altitude.
        return (vehicle[2] - payload[2]) > PAYLOAD_EXPECTED_REST_Z_M + PAYLOAD_DETACH_SEPARATION_M

    async def retry_release(self, shape_type: str) -> bool:
        color = SHAPE_TO_COLOR.get(shape_type)
        if color is None:
            return False
        return await self._burst_detach(color)

    def landing_reference(self, shape_type: str):
        """F3: (target_x, target_y, expected_rest_z) so a landing can be
        scored against the shape it was aimed at. Sim-only ground truth;
        returns None for shapes with no known centre, and core code then
        degrades to the old above-ground check."""
        centre = TARGET_CENTERS.get(shape_type)
        if centre is None:
            return None
        return (centre[0], centre[1], PAYLOAD_EXPECTED_REST_Z_M)

    def detach_latency(self, shape_type: str):
        return self.detach_latency_s.get(SHAPE_TO_COLOR.get(shape_type))

    async def release_payload_at_mavi_altigen(self) -> bool:
        """Görev 2, 1. yük bırakma: RED payload -> Mavi Altıgen hedefi."""
        # FIRST MISSION SERVO
        return await self._publish_color_drop("red")

    async def release_payload_at_kirmizi_ucgen(self) -> bool:
        """Görev 2, 2. yük bırakma: BLUE payload -> Kırmızı Üçgen hedefi."""
        # SECOND MISSION SERVO
        return await self._publish_color_drop("blue")

    # Görev 3'ün kendi algoritması operatör tarafından tanımlandı (2026-08-13
    # revizyonu, bkz. gorev3_pickup.py/gorev3_redrop.py) -- artık kapsam
    # dışı değil. Ancak bu iki metod için (pickup/drop) hâlâ SDF-instantiated
    # bir mekanizma yok: HookAttachSystem derlenmiş durumda
    # (src/modules/simulation/gz_plugins/hook_attach/HookAttachSystem.cc,
    # topics /hook/attach, /hook/detach, /hook/state) ama hiçbir .sdf/.world
    # dosyasında referans edilmiyor, yani şimdi publish etmek hiçbir şeyi
    # tetiklemez. Gerçek yük alma/bırakma mantığı (dik hizalanma, 30cm/60cm
    # hareket, doğrulama) gorev3_pickup.py/gorev3_redrop.py'de zaten
    # gerçek -- yalnızca bu iki metodun İÇİ (fiziksel servo/hook tetikleme)
    # simüle, mekanizma SDF'ye eklenene kadar.
    async def get_released_payload_pose(self, shape_type: str):
        """W4.3: where the released payload body actually ended up.

        ADR-011: the payload is a world-loaded model with a fixed name
        (payload_red / payload_blue), read straight off
        /world/<world>/dynamic_pose/info.

        Returns (x, y, z) or None. Purely observational: it is called after
        the servo has already fired, and every failure path returns None so a
        missing pose is reported as "unavailable" rather than guessed at."""
        # ADR-011: the payload is a world-loaded model with a fixed name now,
        # not a spawn named payload_drop_<color>_<n>.
        color = SHAPE_TO_COLOR.get(shape_type)
        if color is None:
            return None
        # F2: read from the shared pose cache instead of spawning a fresh
        # `gz topic -e -n 1`. That one-shot subscription cost ~2 s of
        # discovery on every call, and -- worse -- BLOCKS FOREVER once the
        # scene goes quiet, because dynamic_pose/info only publishes
        # entities that moved this step. Reading the cache returns the last
        # observed pose of a settled body immediately, which is exactly the
        # question being asked.
        return self.pose_monitor.get(PAYLOAD_MODEL % color)

    def get_released_payload_tilt_deg(self, shape_type: str):
        """F3 diagnosis: how far the payload is from lying flat.

        payload_red came to rest at z=0.156 on the first ADR-011 flight.
        0.156 is not a random number: it is the target surface (0.006) plus
        half of the prism's LONG side (0.150) -- i.e. the slab was standing
        on its edge, not lying on its face. Reporting tilt turns that from
        an inference into an observation."""
        color = SHAPE_TO_COLOR.get(shape_type)
        if color is None:
            return None
        quat = self.pose_monitor.get_quat(PAYLOAD_MODEL % color)
        if quat is None:
            return None
        qx, qy, qz, qw = quat
        # Angle between the body z axis and world z, straight from the
        # rotation matrix's (2,2) term -- no euler conversion needed.
        cos_tilt = max(-1.0, min(1.0, 1.0 - 2.0 * (qx * qx + qy * qy)))
        return round(math.degrees(math.acos(cos_tilt)), 1)

    async def activate_pickup_mechanism(self) -> bool:
        """Görev 3 Faz 1, Adım 6: Yük alma mekanizmasını aktifleştirir."""
        # THIRD MISSION SERVO
        logger.info(f"Gazebo servisi cagriliyor: {self.gazebo_service_name} - Yuk Alma")
        # TODO[GOREV3]: HookAttachSystem SDF'ye eklendiginde /hook/attach'e
        # StringMsg(child_model_name) publish edilecek (bkz. yukaridaki not).
        await asyncio.sleep(0.5)
        logger.info("Gazebo'da yuk alindi (Simulasyon).")
        return True

    async def activate_drop_mechanism(self) -> bool:
        """Görev 3 Faz 3, Adım 5: Taşınan yükü bırakır."""
        # GRAB SERVO
        logger.info(f"Gazebo servisi cagriliyor: {self.gazebo_service_name} - Yuk Birakma (Redrop)")
        # TODO[GOREV3]: HookAttachSystem SDF'ye eklendiginde /hook/detach'e
        # Boolean(true) publish edilecek (bkz. yukaridaki not).
        await asyncio.sleep(0.5)
        logger.info("Gazebo'da tasinan yuk birakildi (Simulasyon).")
        return True
