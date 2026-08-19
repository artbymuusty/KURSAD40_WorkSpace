"""
Pinhole intrinsics for the downward-facing camera, resolved from the
SIMULATION MODEL rather than restated as constants here.

Why not just write the numbers down: the camera's horizontal FOV and image
size live in the mono_cam SDF (Tools/simulation/gz/models/mono_cam/model.sdf,
`<horizontal_fov>` + `<image><width>/<height>`), and that file is the thing
that actually determines what the camera sees. A copy in this repo's config
would be a second source of truth that silently goes stale the first time
anyone retunes the sensor -- exactly the class of drift ADR-004 §9 keeps
flagging. So this parses the SDF, and only falls back to an explicitly
configured value if it cannot.

The one derived quantity everything here exists for:

    ground_distance_m = AGL * hypot(dx_px, dy_px) / focal_px

i.e. for a nadir-pointing pinhole camera, a target `r` pixels from the
image centre sits `AGL * r / f` metres from the vehicle's nadir point on
flat ground. Exact for the pinhole model (`AGL * tan(atan(r/f))` reduces to
it); it assumes the camera is looking straight down and the ground is
level, which is what the x500_mono_cam_down mount and the Görev 2 arena
both are. It is a DISPLAY/telemetry quantity: nothing in the control loop
reads it, and the centering law stays in normalized pixel error.
"""
import logging
import math
import os
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from functools import lru_cache
from typing import Optional, Tuple

logger = logging.getLogger(__name__)

# Walked upward from this file to locate the repo root that owns
# Tools/simulation/gz. Kept as a search rather than a fixed relative path
# because this package is run from three different entrypoints
# (gz_system/, real_system/, dual_system/) with different working
# directories.
_SDF_RELATIVE_PATH = os.path.join("Tools", "simulation", "gz", "models", "mono_cam", "model.sdf")


def _find_model_sdf() -> Optional[str]:
    here = os.path.abspath(os.path.dirname(__file__))
    for _ in range(8):
        candidate = os.path.join(here, _SDF_RELATIVE_PATH)
        if os.path.isfile(candidate):
            return candidate
        parent = os.path.dirname(here)
        if parent == here:
            break
        here = parent
    return None


@dataclass(frozen=True)
class CameraIntrinsics:
    horizontal_fov_rad: float
    width_px: int
    height_px: int
    source: str

    @property
    def focal_px(self) -> float:
        """f = (W/2) / tan(hfov/2). Gazebo cameras use square pixels, so the
        same focal length applies on both axes."""
        return (self.width_px / 2.0) / math.tan(self.horizontal_fov_rad / 2.0)

    def scaled_to(self, width_px: int, height_px: int) -> "CameraIntrinsics":
        """Intrinsics for a frame delivered at a different resolution than
        the SDF declares (the ZMQ camera service may downscale). FOV is a
        property of the lens and does not change with sampling; the focal
        length in PIXELS does, and `focal_px` derives it from width, so
        simply restating the size is correct."""
        if width_px == self.width_px and height_px == self.height_px:
            return self
        return CameraIntrinsics(self.horizontal_fov_rad, width_px, height_px,
                                f"{self.source} (rescaled to {width_px}x{height_px})")

    def ground_distance_m(self, dx_px: float, dy_px: float, agl_m: float) -> Optional[float]:
        """Horizontal ground distance from the vehicle's nadir point to a
        target `dx_px, dy_px` from the image centre, at `agl_m` above
        ground. None when the altitude is unusable (no fix yet, or on the
        ground) rather than a misleading 0.0."""
        if agl_m is None or agl_m <= 0.0:
            return None
        return agl_m * math.hypot(dx_px, dy_px) / self.focal_px


def load_camera_intrinsics(sdf_path: Optional[str] = None,
                           fallback_hfov_rad: Optional[float] = None,
                           fallback_size: Tuple[int, int] = (1280, 960)) -> Optional[CameraIntrinsics]:
    """Parse `<horizontal_fov>` and `<image><width>/<height>` from the first
    `<sensor type="camera">` in the model SDF.

    Returns None if neither the SDF nor a configured fallback yields a
    usable FOV -- callers then omit the distance readout entirely rather
    than display a number derived from a guessed lens.
    """
    path = sdf_path or _find_model_sdf()
    if path and os.path.isfile(path):
        try:
            root = ET.parse(path).getroot()
            for sensor in root.iter("sensor"):
                if sensor.get("type") != "camera":
                    continue
                camera = sensor.find("camera")
                if camera is None:
                    continue
                hfov_el = camera.find("horizontal_fov")
                if hfov_el is None or not hfov_el.text:
                    continue
                hfov = float(hfov_el.text)
                image = camera.find("image")
                width = int(image.findtext("width", fallback_size[0])) if image is not None else fallback_size[0]
                height = int(image.findtext("height", fallback_size[1])) if image is not None else fallback_size[1]
                intrinsics = CameraIntrinsics(hfov, width, height, os.path.relpath(path))
                logger.info("[CAMERA] intrinsics: hfov=%.4f rad (%.1f deg) %dx%d -> f=%.1f px (kaynak: %s)",
                            hfov, math.degrees(hfov), width, height, intrinsics.focal_px, intrinsics.source)
                return intrinsics
        except Exception as e:  # noqa: BLE001 -- a malformed SDF must not take the mission down
            logger.warning("[CAMERA] SDF intrinsics okunamadi (%s): %s", path, e)
    else:
        logger.warning("[CAMERA] mono_cam SDF bulunamadi (%s) -- FOV yapilandirmadan okunacak.",
                       sdf_path or _SDF_RELATIVE_PATH)

    if fallback_hfov_rad:
        intrinsics = CameraIntrinsics(fallback_hfov_rad, fallback_size[0], fallback_size[1], "config")
        logger.info("[CAMERA] intrinsics (config fallback): hfov=%.4f rad %dx%d -> f=%.1f px",
                    fallback_hfov_rad, fallback_size[0], fallback_size[1], intrinsics.focal_px)
        return intrinsics

    logger.warning("[CAMERA] kamera ic parametreleri belirlenemedi -- "
                   "yer mesafesi (d=) gosterilmeyecek.")
    return None


@lru_cache(maxsize=1)
def default_camera_intrinsics() -> Optional[CameraIntrinsics]:
    """Process-wide, parsed once. Both readers -- CenteringController (which
    puts `ground_distance_m` into CENTERING_STEP) and the dashboard overlay
    (which renders the `d=` label) -- share this so the logged number and
    the displayed number can never disagree."""
    return load_camera_intrinsics()
