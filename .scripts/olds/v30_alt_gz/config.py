# config.py
# Constants and configurations for v30 Autonomy System

# ==========================================================
print("[DEBUG] config.py loaded.")
# ALTITUDES (m)
# NOTE: These values must be adjusted after real-world testing.
# ==========================================================
SEARCH_ALTITUDE = 20.0
APPROACH_ALTITUDE = 8.0

# ==========================================================
# BONUS TASK (TEKNOFEST Donel Kanat Gorev 2 optional pickup/transfer)
# ==========================================================
# When True (default -- matches existing behavior before this flag existed):
# after both payloads are dropped, the mission attempts the bonus
# pickup-and-transfer sequence (PICKUP_PREPARATION -> PICKUP -> PAYLOAD_TRANSFER)
# before returning home. Set False to skip the bonus task entirely and go
# straight to RETURN_HOME after the second drop -- useful for competition-day
# risk management if the bonus isn't worth attempting.

DROP_ALTITUDE = 2.0
POST_DROP_ASCEND_ALTITUDE = 7.0
THIRD_TASK_TRAVEL_ALTITUDE = 7.0
ALTITUDE_STEP_FOR_PAYLOAD_SEARCH = 0.3

# ==========================================================
# TIMEOUTS AND DURATIONS (s)
# ==========================================================
VISION_LOSS_TOLERANCE = 5.0
HOLD_TOP_DURATION = 5.0
HOLD_MID_DURATION = 5.0
HOLD_LOW_DURATION = 3.0
DROP_WAIT_DURATION = 10.0
NO_TARGET_TIMEOUT = 5.0
RECOVERY_ALT_STEP = 0.3

# ==========================================================
# VISION CORE / YOLO CONFIG
# ==========================================================
YOLO_WEIGHTS_PATH = "yolov8n.pt"
CONFIDENCE_THRESHOLD_NEW = 0.60
CONFIDENCE_THRESHOLD_TRACK = 0.45
GRACE_PERIOD_SEC = 1.5

# ==========================================================
# VISION / COLOR THRESHOLDS (LEGACY, TO BE DEPRECATED)
# ==========================================================
# Red
RED_LO_1 = (0, 40, 40)
RED_HI_1 = (15, 255, 255)
RED_LO_2 = (165, 40, 40)
RED_HI_2 = (180, 255, 255)

# Blue
BLUE_LO  = (90, 80, 40)
BLUE_HI  = (140, 255, 255)

# Areas (scaled dynamically by resolution in vision.py)
MIN_AREA_TRI_BASE = 390
MIN_AREA_HEX_BASE = 800
MIN_AREA_CIRCLE_BASE = 150 # Estimated base area for payloads

# Shape approximation
EPS_TRI_MIN = 0.03
EPS_TRI_MAX = 0.09
EPS_HEX     = 0.026

COLOR_FRAC_TRI_BASE = 0.35
COLOR_FRAC_HEX      = 0.45

# Streak filtering
STREAK_FRAMES = 3
STREAK_DIST_PX = 60

# ==========================================================
# CONTROL
# ==========================================================
KP_XY = 2.0
VMAX_XY = 4.8 # (was 1.6) Simulation test speed, tune for stability
KP_Z = 0.8
VMAX_Z = 3.9 # (was 1.3) Simulation test speed, tune for stability

# ==========================================================
# HARDWARE / MAVSDK / GAZEBO / UI
# ==========================================================
DEFAULT_MAVSDK_URL = "udpin://0.0.0.0:14540"
# NOTE: payload drop support (PayloadDropSystem plugin + dummy links) is built
# directly into Tools/simulation/gz/models/x500_mono_cam_down/model.sdf --
# there is no separate "_payload" model variant. Spawned instance is _0.
# camera_service.py's resolve_camera_topic() will auto-discover the live
# camera topic via `gz topic -l` if this exact string ever drifts from
# whichever world/model variant is actually running (e.g. a differently
# named airframe) -- this is just the preferred/expected value.
DEFAULT_GZ_TOPIC = "/world/default/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image"
CAMERA_ZMQ_ADDRESS = "tcp://127.0.0.1:5555"

# Hook rope contact sensor (see hook_rope_link in x500_mono_cam_down/model.sdf).
# Real pickup confirmation: hook_sensor_service.py subscribes to the Gazebo
# contact sensor topic and republishes a simple touch signal over this ZMQ
# socket, which PayloadManager.pickup_payload_simulated() waits on instead of
# just assuming success after a fixed delay.
# How long PayloadManager.pickup_payload_simulated() waits for a real hook
# contact signal before giving up. The low pass in mission.py's _state_pickup
# is a single vector traversal at ~0.5-1 m/s over a few meters, so this is
# generous headroom, not a tight deadline.

# HookAttachSystem (src/modules/simulation/gz_plugins/hook_attach) command/state
# topics. See model.sdf's HookAttachSystem plugin block for the matching SDF
# parameters. Attach only ever fires in response to an explicit message on
# sent by PayloadManager only after a genuine hook-contact touch is confirmed.
# HOOK_STATE_TOPIC is the only source of truth for whether the fixed joint
# was actually created/removed in Gazebo's ECM -- PayloadManager waits on
# this (relayed by hook_sensor_service.py) rather than assuming the attach
# command succeeded.
# How long to wait for HOOK_STATE_TOPIC to confirm attached=true after
# sending an attach command, before retrying.
# How many times to retry sending the attach command (each with its own

DEBUG_WINDOW_WIDTH = 640
DEBUG_WINDOW_HEIGHT = 480

DRAW_OBJECT_CENTER_DOT = True
DRAW_IMAGE_TO_OBJECT_LINE = True
DRAW_FULL_OBJECT_CROSS_LINES = False
OBJECT_CENTER_DOT_RADIUS = 5
OBJECT_TO_IMAGE_CENTER_LINE_COLOR = (0, 0, 0)
OBJECT_TO_IMAGE_CENTER_LINE_THICKNESS = 2
DISTANCE_ESTIMATION_GAIN = 1.0 # Visual estimate gain, must be calibrated

TEST_FORWARD_ALTITUDE = 5.0
TEST_FORWARD_SPEED = 0.5
TEST_FORWARD_DURATION = 20.0

SCAN_ALTITUDE = 6.0
SCAN_FORWARD_SPEED = 1.5 # (was 0.5) Simulation test speed, tune for stability
SCAN_MAX_DISTANCE = 60.0
SCAN_MAX_DURATION = 120.0

DROP_APPROACH_ALTITUDE = 4.0
DROP_ALTITUDE = 2.0
DROP_HOLD_SECONDS = 3.0
PAYLOAD_DROP_CONFIRM_WAIT = 2.0
POST_DROP_WAIT_SECONDS = 10.0
POST_DROP_ASCEND_ALTITUDE = 6.0

TARGET_CENTER_THRESHOLD_PX = 25
TARGET_LOST_TIMEOUT = 5.0

# Grace window: how long to keep flying toward last-valid detection before aborting
TARGET_STALE_DETECTION_MAX_AGE = 5.0
TRIANGLE_DETECTION_GRACE_SECONDS = 5.0
HEXAGON_DETECTION_GRACE_SECONDS = 5.0


# Simulated actuator slot for drop (MAVSDK action backend)
ACTUATOR_SLOT = 6
HEX_PULSE = 1.0  # Blue hex target -> Red payload drop pulse
TRI_PULSE = -1.0 # Red tri target -> Blue payload drop pulse

# Gazebo PayloadDropSystem plugin topic. RESTORE NOTE: an earlier pass split
# this into two color-specific topics based on a misread of the plugin's
# source (it was reading the unused stub in the Tools/simulation/gz
# submodule instead of the actually-compiled
# src/modules/simulation/gz_plugins/payload_drop version). The real plugin
# uses a single shared topic and an internal red-then-blue stage counter --
# it does not need or read a color-specific topic at all. This matches
# .scripts/denemePayload.py's known-working manual trigger.
GAZEBO_PAYLOAD_DROP_TOPIC = "/payload_drop"

# RUNTIME DEBUG FIX (KURSAD40 second-payload investigation): PayloadDropSystem
# previously had no way to report back whether a drop actually spawned
# anything -- payload.py's old "success" signal only meant the `gz topic
# pub` CLI command itself exited 0, never that the plugin received the
# message or that its EntityFactory create call actually succeeded. This
# output topic (relayed by hook_sensor_service.py, same as
# confirmation so PayloadManager can wait for a real spawn result instead of
# just a publish acknowledgement.
DEFAULT_PAYLOAD_DROP_STATE_TOPIC = "/payload_drop_state"
# How long to wait for a DROP_STATE confirmation after publishing to
# GAZEBO_PAYLOAD_DROP_TOPIC before giving up on that attempt.
PAYLOAD_DROP_STATE_CONFIRM_TIMEOUT_S = 5.0

# ==========================================================
# CAMERA & PAYLOAD OFFSETS (m)
# ==========================================================
# Note: Camera may not be exactly under drone center, and payload
# release point may not match camera optical center.
# These values must be tuned in Gazebo and real-world tests.
CAMERA_OFFSET_X_M = 0.0
CAMERA_OFFSET_Y_M = 0.0
PAYLOAD_RED_OFFSET_X_M = 0.0
PAYLOAD_RED_OFFSET_Y_M = 0.0
PAYLOAD_BLUE_OFFSET_X_M = 0.0
PAYLOAD_BLUE_OFFSET_Y_M = 0.0

# ==========================================================
# SPEEDS (m/s)
# ==========================================================
WAYPOINT_INTERRUPT_CENTER_SPEED = 2.4 # (was 0.8) Simulation test speed, tune for stability
DESCENT_SPEED = 0.5
ASCENT_SPEED = 1.0
THIRD_MISSION_SPEED = 4.5 # (was 1.5) Simulation test speed, tune for stability

# ==========================================================
# 3RD MISSION DEMO PARAMETERS
# ==========================================================
THIRD_MISSION_TRAVEL_ALTITUDE = 6.0  # cruise altitude for 3rd mission navigation (m)
PICKUP_ALTITUDE = 1.5                # how low to descend for simulated pickup (m)
FINAL_DROP_ALTITUDE = 2.0            # descent altitude for final demo drop (m)
SIMULATED_PICKUP_WAIT_SECONDS = 3.0  # hover time to simulate payload grab (s)
FINISH_LINE_SPEED = 2.0              # forward speed during finish line demo (m/s)
FINISH_LINE_FORWARD_SECONDS = 5.0   # how long to fly forward as finish demo (s)
RETURN_HOME_SPEED = 3.0              # navigation speed for return-to-home (m/s)
