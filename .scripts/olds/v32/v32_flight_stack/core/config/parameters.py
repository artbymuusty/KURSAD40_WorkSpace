"""
Bu dosyadaki TODO değerler bilerek boş bırakılmıştır; ekip fiziksel test sonrası dolduracaktır
"""

# --- Görev 2 (Görev 2 Rapor Bölüm 4, 5, 6, 9, 14) ---
YOLO_CONFIDENCE_THRESHOLD: float = 0.70          # Bölüm 5.2
CAMERA_PROCESS_FREQ_HZ_MIN: int = 10             # Bölüm 5.2
CAMERA_PROCESS_FREQ_HZ_MAX: int = 30             # Bölüm 5.2
MISSION_ALTITUDE_M: float = 15.0                 # Bölüm 4.2, 6, 9 [TASARIM KARARI]
HOVER_DURATION_S: float = 2.0                    # Bölüm 9
DEBOUNCE_DURATION_S: float = 10.0                # Bölüm 14
GOREV2_MAX_FLIGHT_DURATION_S: int = 600          # Şartname Bölüm 5.6 (10 dakika, ZORUNLU)

# --- Görev 3 (Görev 3 Rapor Bölüm 4) — TASARIM KARARI, fiziksel testle güncellenecek ---
GOREV3_TRANSIT_ALTITUDE_M: float = 3.0
GOREV3_DESCENT_ALTITUDE_M: float = 0.30
# Operatör revizyonu (2026-08-13): "1. Yüke dik, 30 cm geride pozisyonlansın"
# / "60 cm ileri giderek yükü aldığımız ortam" -- eski 0.15/0.30 değerleri
# (hiçbir zaman fiziksel testle doğrulanmamış TASARIM KARARI placeholder'lardı)
# operatörün gerçek ölçümleriyle değiştirildi.
GOREV3_RETREAT_DISTANCE_M: float = 0.30
GOREV3_ADVANCE_DISTANCE_M: float = 0.60
GOREV3_PICKUP_VERIFY_CLIMB_STEPS_M: list[float] = [1.0, 2.0, 3.0]
GOREV3_DROP_CLIMB_STEPS_M: list[float] = [1.0, 2.0]
# GOREV3_ALIGNMENT_ANGLE_DEG (fixed 90.0 placeholder) removed: superseded by
# RectangleAlignmentStrategy.compute_alignment_yaw(), which derives the real
# perpendicular heading from the detected rectangle's own orientation
# instead of assuming a fixed angle (operator revision, 2026-08-13).
GOREV3_PICKUP_ALIGN_MAX_ATTEMPTS: int = 30
GOREV3_PICKUP_VISIBILITY_CONFIRM_FRAMES: int = 3

# TODO[PARAMETRE]: Normal görev seyir hızı (Görev 2/3) hala ekip tarafından
# fiziksel testle belirlenecek -- None birakilmasi bilinçlidir (bkz.
# gorev3_transport.py'nin RuntimeError guard'ı), rastgele bir değer
# UYDURULMAYACAK.
NORMAL_MISSION_SPEED_M_S: float | None = None   # TODO: ekip tarafından doldurulacak
# Operatör revizyonu (2026-08-13): "3 m irtifada, 2 m/s seyir hızı ile ikinci
# yük bırakma konumuna ilerleyecektir" -- ekip tarafından ACIKCA verilen
# gerçek bir değer, NORMAL_MISSION_SPEED_M_S/3 formülünün yerine geçer.
GOREV3_TRANSIT_SPEED_M_S: float | None = 2.0

# Doğrulama renk eşleşmeleri (Görev 2 Rapor Bölüm 13)
VERIFICATION_MARKER: dict[str, str] = {
    "MAVI_ALTIGEN": "KIRMIZI_DIKDORTGEN",
    "KIRMIZI_UCGEN": "MAVI_DIKDORTGEN",
}

# --- Mission Operations Center (ADR-004 / ADR-005) ---
# Health heartbeat cadences: expected interval between events from a given
# subsystem before HealthMonitor marks it DEGRADED/STALE/DOWN. A subsystem
# going silent longer than interval*grace_multiplier is DOWN (ADR-004 §10).
VISION_HEARTBEAT_INTERVAL_S: float = 1.0 / CAMERA_PROCESS_FREQ_HZ_MIN  # worst-case tick at min spec Hz
FLIGHT_TELEMETRY_HEARTBEAT_INTERVAL_S: float = 1.0
HEALTH_GRACE_MULTIPLIER: float = 3.0

# Watchdog thresholds (ADR-004 §18). MISSION_TIMEOUT_S reuses the existing,
# previously-unenforced GOREV2_MAX_FLIGHT_DURATION_S -- this is what makes
# it finally fire instead of sitting dead in config.
CONNECTION_ESTABLISH_TIMEOUT_S: float = 15.0
MISSION_UPLOAD_ACK_TIMEOUT_S: float = 5.0
CENTERING_CONVERGENCE_TIMEOUT_S: float = 5.0  # matches CenteringController's own 30x0.1s budget
# BUG FIX (operator revision, 2026-08-13, "mission_gz supervisor model"):
# starting Mission mode is exclusively the operator's action in
# QGroundControl, same as the route upload itself -- this system only
# waits and observes. Generous timeout since it is a genuine human-paced
# step (arm/takeoff already happened; the operator presses Start Mission
# whenever they're ready), not a network/telemetry round-trip.
OPERATOR_MISSION_START_TIMEOUT_S: float = 300.0

# Ops Center runtime cadence
WATCHDOG_CHECK_INTERVAL_S: float = 1.0
DASHBOARD_REFRESH_HZ: float = 10.0

# QGroundControl connection visibility (operator-requested: "let's see in
# the mission board whether QGC is connected or not"). MAVSDK exposes no
# API for "which other GCS clients are connected to this vehicle" -- this
# is a genuine platform limitation, not something we can query directly.
# QGC_UDP_PORT is its conventional default listen port; QgcMonitor treats
# "something locally bound to this port" as a heuristic proxy for "QGC (or
# an equivalent GCS) is running" -- honest best-effort, not a definitive
# MAVLink-level confirmation that QGC is actively receiving telemetry.
QGC_UDP_PORT: int = 14550
QGC_CHECK_INTERVAL_S: float = 2.0

# --- Centering closed-loop control (Görev 2 Rapor Bölüm 8-9) ---
# BUG FIX (operator-reported): go_to_and_center() previously computed pixel
# error and then just checked it against a threshold -- it never called
# set_velocity_body() at all, so nothing ever actually drove the vehicle
# toward the target regardless of Offboard being active. This is the
# minimum viable, physically bounded proportional controller; kp_horizontal/
# kp_vertical (config-injected per real_system.yaml/gz_system.yaml) and
# MAX_CENTERING_SPEED_M_S are placeholders the team will retune after
# physical flight testing, same as every other TODO gain in this file --
# but centering now actually commands the vehicle, which it did not before.
MAX_CENTERING_SPEED_M_S: float = 2.0
# PX4 auto-exits Offboard if it doesn't receive a new setpoint within ~500ms
# (hard PX4 safety behavior, not configurable away) -- this must stay well
# under that, not just "fast enough to look responsive".
OFFBOARD_SETPOINT_INTERVAL_S: float = 0.1
OFFBOARD_MODE_CONFIRM_TIMEOUT_S: float = 3.0

# --- Centering precision + staged payload approach (operator revision,
# 2026-08-13) ---
# go_to_and_center() already computed error_x_norm/error_y_norm but
# converged on a raw-pixel threshold instead -- these replace that with the
# operator's actual precision requirement. Config-injected per
# real_system.yaml/gz_system.yaml, same pattern as kp_horizontal/kp_vertical.
CENTERING_TOLERANCE_X_NORM: float = 0.01
CENTERING_TOLERANCE_Y_NORM: float = 0.01
# BUG FIX (operator-reported, 2026-08-13): go_to_and_center()'ın lateral-only
# (irtifa değişmeyen) çağrıları -- yani İLK kilitlenme geçişi -- eskiden
# sabit 30 deneme (3 saniye) kullanıyordu; bu, eski 20px toleransı için
# ayarlanmıştı. ±0.01 normalize tolerans ~6-9 kat daha sıkı, ve 3 saniye
# gerçek kamera/kontrol gürültüsüyle bu hassasiyete ulaşmak için yetersiz --
# operatörün bildirdiği "kilitleniyor ama sonrasını tamamlayamıyor"
# belirtisine katkıda bulunan ikinci neden buydu (birincisi:
# _resume_mission_route eksikliği, bkz. gorev2_orchestrator.py).
CENTERING_LATERAL_TIMEOUT_S: float = 15.0

# --- Deferred payload-mission GPS navigation (operator revision, 2026-08-13:
# Mission = search-only, Payload Mission 1/2 run AFTER search completes,
# from wherever the vehicle happens to be -- see
# core/navigation/geo.py + CenteringController.goto_global_position_and_wait) ---
GPS_POSITION_CONVERGENCE_TOLERANCE_M: float = 2.0
# BUG FIX (regression investigation, 2026-08-13): goto_global_position_and_wait()
# used to declare convergence on position alone, with no check that the
# vehicle had actually slowed down -- proven via live instrumentation to
# fire while the vehicle was moving at ~11 m/s, mid-flight through the
# target's 2m radius, causing it to coast ~10-25m past before anything
# else corrected it. The prior (pre-Mission-Lifecycle-revision) codebase's
# proven working equivalent (.scripts/olds/v32/mission.py::_state_return_home,
# used only as a behavioral reference, not copied) always gated arrival on
# 3D velocity magnitude alongside position, not position alone. Its own
# value (0.05 m/s) was tuned for a strict final-landing approach; this
# tolerance only needs to be "slowed enough for the following staged-descent
# centering to have a fair chance", so it's scaled to this codebase's own
# existing tolerances (GPS_POSITION_CONVERGENCE_TOLERANCE_M=2.0,
# ALTITUDE_CONVERGENCE_TOLERANCE_M=0.3) rather than copied verbatim.
GPS_POSITION_VELOCITY_TOLERANCE_M_S: float = 0.3
GLOBAL_POSITION_NAV_TIMEOUT_S: float = 60.0
# Separate from kp_vertical (which controls the image-Y/forward-body-axis,
# not altitude) -- altitude convergence is a distinct physical axis and
# needs its own gain/tolerance.
KP_ALTITUDE: float = 0.5
ALTITUDE_CONVERGENCE_TOLERANCE_M: float = 0.3
# Payload approach sequence (operator spec): from mission altitude, step
# down through these altitudes, re-centering at each one, before the final
# forward nudge and servo release. Shared by both payload drops (Mavi
# Altıgen / Kırmızı Üçgen) and by Görev 3's redrop -- same staged-descent
# primitive, just called with different target shapes/positions.
PAYLOAD_APPROACH_ALTITUDES_M: list[float] = [10.0, 5.0, 0.30]
PAYLOAD_FINAL_FORWARD_M: float = 0.10

# --- HSVContourDetector interim fallback (core/detection/hsv_contour_detector.py) ---
# Classical HSV-threshold + contour-shape tuning, ported unchanged from the
# proven-working v29 monolith (.scripts/v29.py / .scripts/olds/v32/vision_backends.py
# HSVContourDetectorBackend, marked there "LEGACY, TO BE DEPRECATED" but never
# actually replaced by a working ML detector). This is a real, functioning
# non-ML IDetector for MAVI_ALTIGEN/KIRMIZI_UCGEN only (no verification-marker
# rectangle detection) -- usable today as an interim/testing detector while
# yolo_detector.py waits for a real trained YOLO26 model (see its own
# footgun-guard warning for why the stock yolov8n.pt is not a substitute).
HSV_RED_LO_1: tuple[int, int, int] = (0, 40, 40)
HSV_RED_HI_1: tuple[int, int, int] = (15, 255, 255)
HSV_RED_LO_2: tuple[int, int, int] = (165, 40, 40)
HSV_RED_HI_2: tuple[int, int, int] = (180, 255, 255)
HSV_BLUE_LO: tuple[int, int, int] = (90, 80, 40)
HSV_BLUE_HI: tuple[int, int, int] = (140, 255, 255)
HSV_MIN_AREA_TRI_BASE: float = 390
HSV_MIN_AREA_HEX_BASE: float = 800
HSV_EPS_TRI_MIN: float = 0.03
HSV_EPS_TRI_MAX: float = 0.09
HSV_EPS_HEX: float = 0.026
HSV_COLOR_FRAC_TRI_BASE: float = 0.35
HSV_COLOR_FRAC_HEX: float = 0.45
HSV_STREAK_FRAMES: int = 3
HSV_STREAK_DIST_PX: float = 60

# --- HSVContourDetector rectangle detection (Görev 3, operatör revizyonu
# 2026-08-13) -- KIRMIZI_DIKDORTGEN / MAVI_DIKDORTGEN. Unlike the triangle/
# hexagon constants above (ported, proven values from v29), no prior
# working rectangle detector exists anywhere in this codebase's history --
# these are new, untuned defaults (same order of magnitude as the ported
# triangle/hexagon constants) and will need real-camera calibration before
# competition use, same caveat as every HSV_* constant in this file.
HSV_MIN_AREA_RECT_BASE: float = 400
HSV_EPS_RECT_MIN: float = 0.02
HSV_EPS_RECT_MAX: float = 0.06
HSV_COLOR_FRAC_RECT: float = 0.40
