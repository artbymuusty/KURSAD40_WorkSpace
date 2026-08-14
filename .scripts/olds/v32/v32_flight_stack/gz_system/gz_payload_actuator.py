import asyncio
import logging
from core.interfaces.i_payload_actuator import IPayloadActuator

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
PAYLOAD_DROP_COLOR_TOPIC = "/payload_drop_color"
PAYLOAD_DROP_STATE_TOPIC = "/payload_drop_state"
GZ_STATE_LISTEN_TIMEOUT_S = 3.0


class GzPayloadActuator(IPayloadActuator):
    def __init__(self, gazebo_service_name: str):
        self.gazebo_service_name = gazebo_service_name

    async def _publish_color_drop(self, color: str) -> bool:
        """Publishes a color-addressed drop request to PayloadDropSystem via
        the `gz topic` CLI (same subprocess-publish mechanism verified working
        by .scripts/denemePayload.py). A zero exit code only confirms the
        message was published, not that the plugin actually spawned the
        payload -- best-effort trigger, consistent with the rest of this
        interface (payload verification afterwards is a separate, best-effort
        camera check per Bölüm 13, not gated on this return value)."""
        cmd = [
            "gz", "topic",
            "-t", PAYLOAD_DROP_COLOR_TOPIC,
            "-m", "gz.msgs.StringMsg",
            "-p", f'data: "{color}"',
        ]
        logger.info(f"Gazebo PayloadDropSystem tetikleniyor: {PAYLOAD_DROP_COLOR_TOPIC} data={color}")
        try:
            proc = await asyncio.create_subprocess_exec(
                *cmd,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE,
            )
            _stdout, stderr = await proc.communicate()
            if proc.returncode != 0:
                logger.error(f"Gazebo drop komutu basarisiz oldu ({color}): {stderr.decode().strip()}")
                return False
            logger.info(f"Gazebo drop komutu yayinlandi ({color}).")
            return True
        except FileNotFoundError:
            logger.error("`gz` CLI bulunamadi -- Gazebo Harmonic/Ionic ortami PATH'te degil.")
            return False
        except Exception as e:
            logger.error(f"Gazebo drop komutu istisna verdi ({color}): {e}")
            return False

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
