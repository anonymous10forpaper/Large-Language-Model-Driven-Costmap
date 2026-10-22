


from __future__ import annotations
from zoneinfo import ZoneInfo
from collections import defaultdict, deque
import copy
import datetime as dt
import http.client
import sys
import math
import json
import queue
import torch
import time
import re
import threading
import urllib.parse
import numpy as np
from dataclasses import dataclass
from typing import List, Tuple, Optional, Dict, Any
from enum import Enum
from rclpy.duration import Duration
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from rclpy.action import ActionClient
from nav2_msgs.action import FollowWaypoints

from std_msgs.msg import Float32, String as RosString
from geometry_msgs.msg import PoseArray, Pose, PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import Image as RosImage, CameraInfo

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

import tf2_ros

from policy_bridge.waypoint_map import (
    DEFAULT_FACTORY_ZONES,
    DEFAULT_FACTORY_WAYPOINTS,
    ZoneBounds,
    waypoint_sort_key,
)
if not hasattr(np, "float"):
    np.float = float

import tf_transformations

try:
    from cv_bridge import CvBridge
except Exception:
    CvBridge = None


YOLO_AVAILABLE = False
try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except Exception:
    pass



ZoneRect = ZoneBounds

@dataclass
class WeightedRule:
    zone: str
    cost: int
    def to_dict(self) -> Dict[str, Any]:
        return {"zone": self.zone, "cost": int(self.cost)}

@dataclass
class TimedKeepout:
    zones: list[str]
    start_wall: dt.datetime
    end_wall: dt.datetime | None = None
    repeat: str | None = None
    id: int = 0
    start_ros_ns: int = 0
    end_ros_ns: int = 0

@dataclass(frozen=True)
class QueuedCommand:
    command_id: int
    text: str
    replace_active: bool = False


@dataclass(frozen=True)
class PreparedMission:
    command_id: int
    text: str
    policy: Dict[str, Any]
    source: str
    replace_active: bool = False

class CondAction(str, Enum):
    FORBID = "forbid"
    ALLOW = "allow"
    ALLOW_SHORTEST = "allow_shortest"


class PolicyUpdateRejected(RuntimeError):
    pass


class CommandInterpretationCanceled(RuntimeError):
    pass



ZONE_LABEL_RE = re.compile(r"(?<![A-Za-z0-9])([A-Z])(?![A-Za-z0-9])", re.IGNORECASE)
WEIGHT_CMD_RE = re.compile(
    r"([A-Z])\s*[^0-9A-Za-z]*?(?:around)?\s*([0-9]+(?:\.[0-9]+)?)\s*m.*?cost\s*([+-]?[0-9]{1,4})",
    flags=re.IGNORECASE,
)
WEIGHT_CMD_RE2 = re.compile(
    r"([A-Z]).{0,10}?(?:around)?\s*cost\s*([+-]?[0-9]{1,4})", re.IGNORECASE
)

DEST_KWS = [
    "go to", "arrive", "then", "next", "navigate", "move", "send", "head", "travel",
]
EXC_KWS_TMP = [
    "avoid", "ban", "forbid", "no-go", "no go", "do not enter", "don't enter",
    "do not use", "block", "closed", "unavailable",
]
EXC_KWS_PERM = ["permanent", "always"]
COND_KWS = ["if", "when", "unless", "but", "however"]
def _is_perm_cost_sentence(s: str) -> bool:
    su = s.upper()
    return any(k.upper() in su for k in EXC_KWS_PERM)

def unique_preserve(xs: List[str]) -> List[str]:
    seen, out = set(), []
    for x in xs:
        if x not in seen:
            out.append(x); seen.add(x)
    return out


def normalize_soft_cost(value: Any) -> Optional[int]:

    if isinstance(value, bool) or not isinstance(value, (int, str)):
        return None
    try:
        cost = int(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return cost if 0 <= cost <= 252 else None

def yaw_to_quat(yaw: float) -> Tuple[float, float, float, float]:
    return tf_transformations.quaternion_from_euler(0.0, 0.0, yaw)



class PolicyBridgeNode(Node):


    COLORS = {
        "reset": "\033[0m",
        "bold": "\033[1m",
        "red": "\033[31m",
        "green": "\033[32m",
        "yellow": "\033[33m",
        "blue": "\033[34m",
        "magenta": "\033[35m",
        "cyan": "\033[36m",
        "gray": "\033[90m",
    }

    def __init__(
        self,
        *,
        parameter_overrides=None,
        node_name: str = "policy_bridge",
        namespace: str = "",
        cli_args=None,
    ):
        super().__init__(
            node_name,
            namespace=namespace,
            parameter_overrides=parameter_overrides,
            cli_args=cli_args,
        )


        self.declare_parameter("nl_command_topic", "/nl_command")
        self.declare_parameter("manual_policy_topic", "/manual_policy_json")
        self.declare_parameter("validated_policy_topic", "/validated_policy")
        self.declare_parameter("event_topic", "/event")
        self.declare_parameter("policy_reset_topic", "/policy_reset")
        self.declare_parameter("forbidden_zones_topic", "/forbidden_zones_update")
        self.declare_parameter("object_positions_topic", "/object_world_positions")
        self.declare_parameter("object_avoidance_radius_topic", "/object_avoidance_radius")
        self.declare_parameter("zone_cost_overrides_topic", "/zone_cost_overrides")
        self.declare_parameter("zone_geometry_topic", "/zone_geometry_update")
        self.declare_parameter("initial_pose_topic", "/initialpose")
        self.declare_parameter("debug_image_topic", "/yolo/debug_image")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")

        self._last_pub_by_frame: Dict[str, List[Tuple[float, float]]] = {}
        self._held_object_positions: Dict[
            str, List[Tuple[float, float]]
        ] = {}


        self._trk_last: Dict[int, Tuple[float, float]] = {}
        self._trk_id = 0
        self._miss_frames = 0
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")

        self.declare_parameter("zone_labels", ["A", "B", "C", "D", "E"])
        self.declare_parameter(
            "zone_database_json",
            json.dumps(
                {
                    name: list(bounds.as_tuple())
                    for name, bounds in DEFAULT_FACTORY_ZONES.items()
                },
                ensure_ascii=False,
            )
        )


        self.declare_parameter("enable_llm", True)
        self.declare_parameter("llm_endpoint", "http://localhost:11434/api/generate")
        self.declare_parameter("llm_api_key", "")
        self.declare_parameter("llm_model", "qwen2.5:7b")
        self.declare_parameter("llm_think", False)
        self.declare_parameter("llm_timeout_s", 120.0)


        self.declare_parameter("enable_yolo", True)
        self.declare_parameter("camera_topic", "/camera/image_raw")
        self.declare_parameter("yolo_model", "yolov8m.pt")
        self.declare_parameter("object_conf_threshold", 0.3)
        self.declare_parameter("object_avg_height_m", 1.70)
        self.declare_parameter("min_distance_m", 0.50)
        self.declare_parameter("max_distance_m", 8.00)
        self.declare_parameter("yolo_inference_period_s", 0.25)

        self.declare_parameter("depth_topic", "/camera/depth/image_raw")
        self.declare_parameter("depth_info_topic", "/camera/depth/camera_info")
        self.declare_parameter("camera_optical_frame", "camera_depth_optical_frame")
        self.declare_parameter("depth_roi_bottom_ratio", 0.25)
        self.declare_parameter("depth_roi_halfw_px", 18)
        self.declare_parameter("track_alpha", 0.45)
        self.declare_parameter("track_max_jump_m", 0.7)
        self.declare_parameter("depth_valid_min_m", 0.2)
        self.declare_parameter("depth_valid_max_m", 12.0)


        self.declare_parameter("enable_console_ui", False)
        self.declare_parameter("ui_color", True)
        self.declare_parameter("objects_log_interval_s", 1.5)


        self.declare_parameter("default_weight_radius_m", 3.0)
        self.declare_parameter("object_frames", [])
        self.declare_parameter("enable_yolo_override_from_nl", True)

        self.declare_parameter("set_initial_pose", True)
        self.declare_parameter(
            "initial_pose_json",
            json.dumps({"x": 0.0, "y": 0.0, "yaw": 0.0, "frame": "map"}, ensure_ascii=False)
        )

        self.declare_parameter("permanent_exclusions", Parameter.Type.STRING_ARRAY)

        self.declare_parameter("timezone", "UTC")
        self.declare_parameter("default_forbid_minutes", 2)

        self.declare_parameter("require_nav2", True)
        self.declare_parameter("nav2_check_timeout_s", 3.0)
        self.declare_parameter("follow_waypoints_action", "/follow_waypoints")
        self.declare_parameter("navigator_namespace", "")
        self.declare_parameter("preempt_on_valid_command", False)
        self.declare_parameter("replacement_nav_settle_s", 0.75)
        self.declare_parameter("replacement_early_abort_retry_window_s", 3.0)


        self.declare_parameter("allow_overrides_permanent", True)


        self._nl_topic       = self.get_parameter("nl_command_topic").value
        self._manual_topic   = self.get_parameter("manual_policy_topic").value
        self._audit_topic    = self.get_parameter("validated_policy_topic").value
        self._event_topic    = self.get_parameter("event_topic").value
        self._reset_topic    = self.get_parameter("policy_reset_topic").value
        self._forbid_topic   = self.get_parameter("forbidden_zones_topic").value
        self._object_topic   = self.get_parameter("object_positions_topic").value
        self._object_radius_topic = self.get_parameter(
            "object_avoidance_radius_topic"
        ).value
        self._softcost_topic = self.get_parameter("zone_cost_overrides_topic").value
        self._zone_geometry_topic = self.get_parameter(
            "zone_geometry_topic"
        ).value
        self._initial_pose_topic = str(
            self.get_parameter("initial_pose_topic").value
        )
        self._debug_image_topic = str(
            self.get_parameter("debug_image_topic").value
        )
        self._camera_info_topic = str(
            self.get_parameter("camera_info_topic").value
        )
        self._navigator_namespace = str(
            self.get_parameter("navigator_namespace").value
        ).strip()

        self._global_frame = str(self.get_parameter("map_frame").value)
        self._base_frame   = self.get_parameter("base_frame").value

        self._labels: List[str] = [s.upper() for s in self.get_parameter("zone_labels").value]
        raw_db_json = self.get_parameter("zone_database_json").get_parameter_value().string_value

        self._depth_topic = str(self.get_parameter("depth_topic").value)
        self._depth_info_topic = str(self.get_parameter("depth_info_topic").value)
        self._camera_optical_frame = str(self.get_parameter("camera_optical_frame").value)
        self._roi_bottom_ratio = float(self.get_parameter("depth_roi_bottom_ratio").value)
        self._roi_halfw = int(self.get_parameter("depth_roi_halfw_px").value)
        self._trk_alpha = float(self.get_parameter("track_alpha").value)
        self._trk_max_jump = float(self.get_parameter("track_max_jump_m").value)
        self._z_min = float(self.get_parameter("depth_valid_min_m").value)
        self._z_max = float(self.get_parameter("depth_valid_max_m").value)

        self._tz = ZoneInfo(str(self.get_parameter("timezone").value))
        self._default_forbid_minutes = int(self.get_parameter("default_forbid_minutes").value)

        try:
            raw_db = json.loads(raw_db_json) if raw_db_json else {}
        except Exception:
            raw_db = {}

        self._zone_db: Dict[str, ZoneRect] = {}
        for k, v in (raw_db or {}).items():
            try:
                xs = [float(x) for x in v]
                if len(xs) == 4:
                    self._zone_db[str(k).upper()] = ZoneRect(*xs)
            except Exception:
                pass


        configured_labels = set(self._zone_db)
        self._labels = [label for label in self._labels if label in configured_labels]

        self._waypoints: Dict[str, Tuple[float, float, float]] = dict(
            DEFAULT_FACTORY_WAYPOINTS
        )

        self._enable_llm   = bool(self.get_parameter("enable_llm").value)
        self._llm_endpoint = str(self.get_parameter("llm_endpoint").value)
        self._llm_key      = str(self.get_parameter("llm_api_key").value)
        self._llm_model    = str(self.get_parameter("llm_model").value)
        self._llm_think    = bool(self.get_parameter("llm_think").value)
        self._llm_timeout_s = max(
            1.0, float(self.get_parameter("llm_timeout_s").value)
        )
        self._last_raw_policy: Optional[Dict[str, Any]] = None
        self._last_llm_metadata: Dict[str, Any] = {}
        self._last_validation_report: Dict[str, Any] = {}

        self._enable_yolo     = bool(self.get_parameter("enable_yolo").value)
        self._camera_topic    = str(self.get_parameter("camera_topic").value)
        self._yolo_model_name = str(self.get_parameter("yolo_model").value)

        _DEF_CONF = 0.3
        _DEF_H = 1.70
        _DEF_LOG = 1.5

        self._conf_thresh = float(self.get_parameter("object_conf_threshold").value)
        self._object_h = float(self.get_parameter("object_avg_height_m").value)
        self._objects_log_interval = float(self.get_parameter("objects_log_interval_s").value)

        self._min_dist = float(self.get_parameter("min_distance_m").value)
        self._max_dist = float(self.get_parameter("max_distance_m").value)
        self._yolo_inference_period = max(
            0.1,
            float(self.get_parameter("yolo_inference_period_s").value),
        )

        self._ui_enable = bool(self.get_parameter("enable_console_ui").value)
        self._ui_color  = bool(self.get_parameter("ui_color").value)
        self._isatty    = sys.stdout.isatty()

        obj_frames = list(self.get_parameter("object_frames").value or [])
        self._object_frames: List[str] = obj_frames
        if not self._object_frames:
            self._object_frames = [self._global_frame]

        self._yolo_override_from_nl = bool(self.get_parameter("enable_yolo_override_from_nl").value)

        self._set_initial_pose = bool(self.get_parameter("set_initial_pose").value)
        ip_json = self.get_parameter("initial_pose_json").get_parameter_value().string_value
        try:
            self._initial_pose = json.loads(ip_json) if ip_json else {}
        except Exception:
            self._initial_pose = {}
        self._initial_pose_applied = False

        self._configured_permanent_exclusions = [
            s.upper() for s in (self.get_parameter_or("permanent_exclusions").value or [])
        ]
        self._permanent_exclusions = self._configured_permanent_exclusions[:]

        self._allow_overrides_permanent = bool(self.get_parameter("allow_overrides_permanent").value)
        self._preempt_on_valid_command = bool(
            self.get_parameter("preempt_on_valid_command").value
        )


        self._env_state = {
            "fire_alarm": False,
            "battery_pct": 100,
        }
        self._cond_rules_store: List[Dict[str, Any]] = []
        self._last_base_exclusions: List[str] = []
        self._force_shortest = False


        self._trk_last_vel = {}
        self._trk_hist = defaultdict(lambda: deque(maxlen=5))
        self._deadband = 0.12


        forbid_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.forbidden_pub = self.create_publisher(RosString, self._forbid_topic, forbid_qos)

        self.object_pub = self.create_publisher(PoseArray, self._object_topic, 10)

        object_radius_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.object_radius_pub = self.create_publisher(
            Float32,
            self._object_radius_topic,
            object_radius_qos,
        )

        soft_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.softcost_pub = self.create_publisher(RosString, self._softcost_topic, soft_qos)
        self.zone_geometry_pub = self.create_publisher(
            RosString, self._zone_geometry_topic, soft_qos
        )
        self._zone_geometry_version = 0
        self._publish_zone_geometry()

        audit_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.validated_policy_pub = self.create_publisher(
            RosString, self._audit_topic, audit_qos
        )

        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, self._initial_pose_topic, 10
        )
        self.nl_sub = self.create_subscription(RosString, self._nl_topic, self._nl_callback, 10)
        self.manual_policy_sub = self.create_subscription(
            RosString, self._manual_topic, self._manual_policy_callback, 10
        )
        self.event_sub = self.create_subscription(RosString, self._event_topic, self._event_callback, 10)
        self.reset_sub = self.create_subscription(RosString, self._reset_topic, self._reset_callback, 10)
        self._active_softcost: Dict[str, int] = {}
        self._active_object_avoidance_radius = 0.0
        self._last_forbidden_windows: Dict[str, Tuple[dt.datetime, dt.datetime]] = {}
        self._active_policy_state = self._empty_policy()
        self._policy_version = 0
        self._last_policy_decision = "initialized"


        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self._print_lock = threading.Lock()

        self._image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )


        self.bridge = CvBridge() if (CvBridge and self._enable_yolo) else None
        self.yolo_model = None
        self.latest_image = None
        self.latest_header = None
        self.image_lock = threading.Lock()
        self.debug_img_pub = self.create_publisher(
            RosImage, self._debug_image_topic, qos_profile_sensor_data
        )

        if self._enable_yolo:
            if not YOLO_AVAILABLE:
                self._ui_print("⚠️  YOLO not available. Install 'ultralytics' or set enable_yolo:=false.", "yellow")
            else:
                try:
                    self.yolo_model = YOLO(self._yolo_model_name)
                    try:
                        self.yolo_model.fuse()
                    except Exception:
                        pass
                    if self.bridge:
                        self.image_sub = self.create_subscription(
                            RosImage, self._camera_topic, self._image_callback, self._image_qos
                        )
                        self._ui_print(f"🔎 YOLO enabled: '{self._yolo_model_name}' on {self._camera_topic}", "cyan")
                except Exception as e:
                    self._ui_print(f"❌ Failed to init YOLO: {e}", "red")


        self.declare_parameter("stale_window_s", 2.5)
        self._stale_window_s = float(self.get_parameter("stale_window_s").value)
        self._last_frame_mono = 0.0
        self._latest_seq = 0
        self._last_seq_processed = -1
        self._resub_backoff_s = 1.0
        self._last_resub_try = 0.0


        self.depth_sub = None
        self.depth_info_sub = None
        self._depth_img = None
        self._depth_header = None
        self._depth_K = None
        self._K = None
        self.create_subscription(
            CameraInfo, self._camera_info_topic, self._caminfo_cb, 10
        )

        if self._enable_yolo and CvBridge:
            self.depth_sub = self.create_subscription(RosImage, self._depth_topic, self._depth_cb, self._image_qos)
            self.depth_info_sub = self.create_subscription(CameraInfo, self._depth_info_topic, self._depth_info_cb, self._image_qos)


        self.declare_parameter("objects_ttl_s", 0.0)
        self._objects_ttl_s = float(self.get_parameter("objects_ttl_s").value)
        self._last_nonempty_pub_time: Dict[str, float] = {}


        self._sched_lock = threading.Lock()
        self._schedules: Dict[int, TimedKeepout] = {}
        self._next_sched_id = 1
        self._active_timed: Dict[int, set[str]] = {}
        self._timer_threads: Dict[int, Tuple[Any, Any]] = {}


        self._latest_dynamic_rules: List[Dict[str, Any]] = []
        self._last_wp_names: List[str] = []


        self._nav: Optional[BasicNavigator] = None
        self._command_queue: deque[QueuedCommand] = deque()
        self._command_lock = threading.Lock()
        self._latest_command_id = 0
        self._minimum_valid_command_id = 0
        self._mission_queue: deque[PreparedMission] = deque()
        self._mission_lock = threading.Lock()

        self._preempt_requested = threading.Event()
        self._cancel_requested = threading.Event()
        self._pause_requested = threading.Event()
        self._mission_paused = threading.Event()
        self._mission_active = threading.Event()
        self._replacement_nav_settle_s = max(
            0.0,
            float(self.get_parameter("replacement_nav_settle_s").value),
        )
        self._replacement_early_abort_retry_window_s = max(
            0.0,
            float(
                self.get_parameter(
                    "replacement_early_abort_retry_window_s"
                ).value
            ),
        )
        self._active_command_id: Optional[int] = None
        self._interpretation_active = threading.Event()
        self._interpretation_cancel_requested = threading.Event()
        self._active_interpretation_id: Optional[int] = None
        self._llm_request_lock = threading.Lock()
        self._active_llm_connections: set[http.client.HTTPConnection] = set()
        self._runtime_workers_lock = threading.Lock()
        self._command_thread: Optional[threading.Thread] = None
        self._mission_thread: Optional[threading.Thread] = None
        self._yolo_run = False
        self._yolo_thread: Optional[threading.Thread] = None
        self._navigation_progress_lock = threading.Lock()
        self._navigation_progress: Dict[str, Any] = {
            "current": 0,
            "total": 0,
            "destination": "",
            "remaining_m": None,
        }


        self._last_object_log_time: Dict[str, float] = {}
        if self._ui_enable:
            self._ui_print("🟢 Console UI enabled.", "green")
            self._ui_print("Help: help | status | keepouts | objects | reset | quit", "gray")
            self._console_thread = threading.Thread(target=self._console_input_loop, daemon=True)
            self._console_thread.start()


        if self._set_initial_pose:
            self._apply_initial_pose_param_once()

        self._ui_print(
            f"Policy-Bridge node up. NL: {self._nl_topic}  →  Zones: {self._forbid_topic}, Objects: {self._object_topic}, ObjectRadius: {self._object_radius_topic}, SoftCost: {self._softcost_topic}",
            "magenta",
        )

        if not self._require_nav2_or_shutdown():
            self._ui_print("🛑 Policy-Bridge disabled because Nav2 is not ready.", "red")
            self._yolo_run = False
            self._nav2_ready = False
            return

        self._nav2_ready = True
        self._start_runtime_workers()

    def _start_runtime_workers(self) -> None:

        with self._runtime_workers_lock:
            if self._command_thread is None or not self._command_thread.is_alive():
                self._command_thread = threading.Thread(
                    target=self._command_worker,
                    name="policy-bridge-command",
                    daemon=True,
                )
                self._command_thread.start()
            if self._mission_thread is None or not self._mission_thread.is_alive():
                self._mission_thread = threading.Thread(
                    target=self._mission_worker,
                    name="policy-bridge-mission",
                    daemon=True,
                )
                self._mission_thread.start()
            if self._yolo_thread is None or not self._yolo_thread.is_alive():
                self._yolo_run = True
                self._yolo_thread = threading.Thread(
                    target=self._yolo_worker_loop,
                    name="policy-bridge-yolo",
                    daemon=True,
                )
                self._yolo_thread.start()

    def _update_navigation_progress(
        self,
        *,
        current: int,
        total: int,
        destination: str,
        remaining_m: Optional[float],
    ) -> None:

        remaining = None
        if remaining_m is not None:
            try:
                value = float(remaining_m)
                if math.isfinite(value):
                    remaining = max(0.0, value)
            except (TypeError, ValueError, OverflowError):
                pass
        with self._navigation_progress_lock:
            self._navigation_progress = {
                "current": max(0, int(current)),
                "total": max(0, int(total)),
                "destination": str(destination or ""),
                "remaining_m": remaining,
            }


    def _c(self, txt: str, color: Optional[str]) -> str:
        if not (self._ui_enable and self._ui_color and self._isatty and color):
            return txt
        c = self.COLORS.get(color, "")
        r = self.COLORS.get("reset", "")
        b = self.COLORS.get("bold", "")
        if color == "bold":
            return b + txt + r
        return c + txt + r

    def _ui_print(self, msg: str, color: Optional[str] = None):
        with self._print_lock:
            sys.stdout.write(self._c(msg, color) + "\n")
            sys.stdout.flush()

    def _wall_dt_to_ros_time(self, wall_dt: dt.datetime) -> rclpy.time.Time:

        if wall_dt.tzinfo is None:
            wall_dt = wall_dt.replace(tzinfo=self._tz)

        wall_now = dt.datetime.now(self._tz)
        ros_now = self.get_clock().now()
        delta = wall_dt - wall_now
        return ros_now + Duration(seconds=delta.total_seconds())

    def _oneshot_ros_timer(self, delay_s: float, cb):

        delay_s = max(0.0, float(delay_s))
        holder = {"t": None}

        def _fire():
            try:
                cb()
            finally:
                if holder["t"] is not None:
                    holder["t"].cancel()

        holder["t"] = self.create_timer(delay_s, _fire)
        return holder["t"]

    def _require_nav2_or_shutdown(self) -> bool:
        require = bool(self.get_parameter("require_nav2").value)
        if not require:
            return True

        timeout = float(self.get_parameter("nav2_check_timeout_s").value)
        action_name = str(self.get_parameter("follow_waypoints_action").value)

        self._ui_print(f"🔎 Checking Nav2 action server: {action_name} (timeout={timeout:.1f}s)", "gray")

        try:
            fw_client = ActionClient(self, FollowWaypoints, action_name)
            ok = fw_client.wait_for_server(timeout_sec=timeout)
            fw_client.destroy()
        except Exception as e:
            self._ui_print(f"❌ Nav2 check failed: {e}", "red")
            return False

        if not ok:
            self._ui_print("❌ Nav2 is NOT running/active. Please launch Nav2 before starting this node.", "red")
            self._ui_print("   (Expected action server: /follow_waypoints)", "red")
            return False

        self._ui_print("✅ Nav2 is available.", "green")
        return True

    def _apply_aisle_aliases(self, text: str) -> str:
        if not text:
            return text

        rep = [
            (r"\bfirst\s+aisle\b", "A"),
            (r"\bsecond\s+aisle\b", "B"),
            (r"\bthird\s+aisle\b", "C"),
            (r"\bfourth\s+aisle\b", "D"),
            (r"\bfifth\s+aisle\b", "E"),
            (r"\b1st\s+aisle\b", "A"),
            (r"\b2nd\s+aisle\b", "B"),
            (r"\b3rd\s+aisle\b", "C"),
            (r"\b4th\s+aisle\b", "D"),
            (r"\b5th\s+aisle\b", "E"),
        ]
        out = text
        for pat, lab in rep:
            out = re.sub(pat, lab, out, flags=re.IGNORECASE)

        return out

    def _canonicalize_named_entities(self, text: str) -> str:

        if not text:
            return text

        def zone_replacement(match: re.Match) -> str:
            noun = match.group(1).lower()
            label = match.group(2).upper()
            return f"{noun} {label}"

        normalized = re.sub(
            r"\b(zone|aisle|section|area)\s*([a-z])\b",
            zone_replacement,
            text,
            flags=re.IGNORECASE,
        )
        normalized = re.sub(
            r"\bwp\s*0*([1-9][0-9]*)\b",
            lambda match: f"WP{int(match.group(1))}",
            normalized,
            flags=re.IGNORECASE,
        )
        return normalized

    def _split_intents_deterministic(self, text: str) -> tuple[set[str], Dict[str, int]]:
        if not text:
            return set(), {}

        chunks = re.split(r"[,.。!?]\s*|\n+", text)
        keepout_kws = [k.lower() for k in EXC_KWS_TMP]
        cost_kws = [
            "cost", "penalty", "score", "value", "rating",
            "expensive", "cheap",
        ]

        keepouts: set[str] = set()
        soft: Dict[str, int] = {}

        def _zones_in(ch: str) -> list[str]:
            return [c for c in ZONE_LABEL_RE.findall(ch.upper()) if c in self._labels]

        def _keepout_zones_in(ch: str) -> list[str]:
            out: list[str] = []
            for m in re.finditer(
                r"\b(?:avoid|ban|forbid|block|close|keep\s+closed|treat\s+as\s+closed|do\s+not\s+use)\s+(?:ZONE\s*)?([A-Z])\b",
                ch,
                flags=re.IGNORECASE,
            ):
                z = m.group(1).upper()
                if z in self._labels:
                    out.append(z)
            for m in re.finditer(
                r"\b(?:ZONE\s*)?([A-Z])\b.{0,30}\b(?:off\s+limits|no-go|no\s+go|closed|blocked|forbidden|unavailable|not\s+allowed)\b",
                ch,
                flags=re.IGNORECASE,
            ):
                z = m.group(1).upper()
                if z in self._labels:
                    out.append(z)
            return unique_preserve(out)

        number = r"([+-]?[0-9]{1,4})"
        PAIR_EN = re.compile(r"(?:\bZONE\s*)?([A-Z])\s*(?:TO|=|:)\s*" + number, re.IGNORECASE)
        PAIR_EN2 = re.compile(r"\b([A-Z])\b[^0-9A-Za-z]{0,20}(?:COST)\s*" + number, re.IGNORECASE)
        PAIR_EN_REV = re.compile(r"\bCOST\s*" + number + r"\s*(?:TO|FOR)\s*(?:ZONE\s*)?([A-Z])\b", re.IGNORECASE)
        PAIR_EN_REV2 = re.compile(number + r"\s*(?:TO|FOR)\s*(?:ZONE\s*)?([A-Z])\b", re.IGNORECASE)
        PAIR_EN_POS = re.compile(r"\b([A-Z])'?S(?:\s+\w+){0,3}?\s+(?:TO|AT)\s*" + number, re.IGNORECASE)
        PAIR_EN_AT = re.compile(r"\b(?:ZONE\s*)?([A-Z])\b[^,;.]{0,40}?\b(?:TO|AT)\s*" + number, re.IGNORECASE)
        PAIR_EN_CUE_REV = re.compile(
            r"\b(?:COST|PENALTY|SCORE|VALUE|RATING)\b\s*(?:OF\s*)?"
            + number + r"\s*(?:ON|FOR|TO)\s*(?:ZONE\s*)?([A-Z])\b",
            re.IGNORECASE,
        )
        PAIR_EN_ZONE_NUM_CUE = re.compile(
            r"\b(?:ZONE\s*)?([A-Z])\b[^,;.]{0,45}?\b"
            + number + r"\s*(?:COST|PENALTY|SCORE|VALUE|RATING)\b",
            re.IGNORECASE,
        )

        for ch in chunks:
            chs = ch.strip()
            if not chs:
                continue
            low = chs.lower()
            chU = chs.upper()

            has_keepout = any(k in low for k in keepout_kws)
            has_cost = any(k in low for k in cost_kws)

            if has_cost:
                pairs = []

                for m in PAIR_EN.finditer(chU):
                    pairs.append((m.group(1).upper(), int(m.group(2))))

                for m in PAIR_EN2.finditer(chU):
                    pairs.append((m.group(1).upper(), int(m.group(2))))

                for m in PAIR_EN_REV.finditer(chU):
                    pairs.append((m.group(2).upper(), int(m.group(1))))

                for m in PAIR_EN_REV2.finditer(chU):
                    pairs.append((m.group(2).upper(), int(m.group(1))))

                for m in PAIR_EN_POS.finditer(chU):
                    pairs.append((m.group(1).upper(), int(m.group(2))))

                for m in PAIR_EN_AT.finditer(chU):
                    pairs.append((m.group(1).upper(), int(m.group(2))))

                for m in PAIR_EN_CUE_REV.finditer(chU):
                    pairs.append((m.group(2).upper(), int(m.group(1))))

                for m in PAIR_EN_ZONE_NUM_CUE.finditer(chU):
                    pairs.append((m.group(1).upper(), int(m.group(2))))

                if pairs:
                    for z, c in pairs:
                        cost = normalize_soft_cost(c)
                        if z in self._labels and cost is not None:
                            soft[z] = cost
                    for z in _keepout_zones_in(chs):
                        keepouts.add(z)
                    continue

                chU_wo_wp = re.sub(r"\bWP\s*\d+\b", " ", chU, flags=re.IGNORECASE)
                zones = _zones_in(chU_wo_wp)
                nums = re.findall(r"(?<![0-9])([+-]?[0-9]{1,4})(?![0-9])", chU_wo_wp)

                if len(zones) == 1 and len(nums) == 1:
                    z = zones[0]
                    cost = normalize_soft_cost(nums[0])
                    if cost is not None:
                        soft[z] = cost

                continue

            if has_keepout:
                zones = _keepout_zones_in(chs) or _zones_in(chs)
                for z in zones:
                    keepouts.add(z)
                continue

        return keepouts, soft

    def _parse_duration_minutes(self, text: str) -> Optional[int]:
        s = (text or "").lower()
        m = re.search(r"\b(\d{1,3})\s*(minutes?|mins?|min)\b", s)
        if m:
            return max(1, int(m.group(1)))
        if re.search(r"\b(?:half\s+(?:an?\s+)?hour|half-hour)\b", s):
            return 30
        if re.search(r"\b(?:for\s+)?thirty\s+minutes?\b", s):
            return 30
        return None

    def _parse_start_time(self, text: str) -> Optional[dt.datetime]:
        s = (text or "").lower()
        today = dt.datetime.now(self._tz).date()
        m = re.search(r"\b(\d{1,2})(?::(\d{2}))?\s*(a\.?m\.?|p\.?m\.?|am|pm)\b.*\btoday\b", s)
        if m:
            hh = int(m.group(1))
            mm = int(m.group(2) or 0)
            ap = m.group(3)
            if "p" in ap and hh != 12:
                hh += 12
            if "a" in ap and hh == 12:
                hh = 0
            return dt.datetime(today.year, today.month, today.day, hh, mm, tzinfo=self._tz)
        m = re.search(r"\btoday\b.*\b(\d{1,2}):(\d{2})\b", s) or re.search(r"\b(\d{1,2}):(\d{2})\b.*\btoday\b", s)
        if m:
            hh = int(m.group(1))
            mm = int(m.group(2))
            return dt.datetime(today.year, today.month, today.day, hh, mm, tzinfo=self._tz)
        return None


    def _looks_like_mission_text(self, text: str) -> bool:
        s = (text or "").strip()
        if not s:
            return False
        su = s.upper()
        if ZONE_LABEL_RE.search(su):
            return True
        if any(k in su for k in (kw.upper() for kw in DEST_KWS + EXC_KWS_TMP)):
            return True
        return False

    def _handle_state_command(self, text: str) -> bool:
        s = (text or "").strip().lower()
        if not s:
            return False

        if self._looks_like_mission_text(s):
            return False

        handled = False


        fire_on = re.search(r"fire(\s*alarm)?\s*(on|start)", s)
        fire_off = re.search(r"fire(\s*alarm)?\s*(off|stop)", s)
        if fire_on:
            self._env_state["fire_alarm"] = True
            handled = True
        if fire_off:
            self._env_state["fire_alarm"] = False
            handled = True


        m = re.search(r"\bbattery\s*([0-9]{1,3})\s*%", s)
        if m:
            pct = max(0, min(100, int(m.group(1))))
            self._env_state["battery_pct"] = pct
            handled = True

        if handled:
            self._ui_print(
                f"🛎  State updated: fire={self._env_state['fire_alarm']} "
                f"battery={self._env_state['battery_pct']}%",
                "yellow"
            )
            self._recalculate_and_publish_dynamic_keepouts(
                base_exclusions=self._last_base_exclusions,
                conditional_rules=self._cond_rules_store
            )
            self._print_runtime_snapshot()
        return handled

    def _active_conditions(self) -> set[str]:
        conds = {"default"}
        if self._env_state.get("fire_alarm"):
            conds.add("fire_alarm")
        if self._env_state.get("battery_pct", 100) <= 20:
            conds.add("low_battery")
        return conds

    def _evaluate_conditional_rules(self, rules_pack: List[Dict[str, Any]]) -> tuple[set[str], set[str], bool]:
        active = self._active_conditions()
        forbid_zones = set()
        allow_zones = set()
        force_shortest = False

        for item in (rules_pack or []):
            z = str(item.get("zone", "")).upper()
            if z not in self._labels:
                continue
            rules = item.get("rules") or []
            try:
                action_rank = {"forbid": 2, "allow": 1, "allow_shortest": 0}
                rules = sorted(
                    rules,
                    key=lambda r: (
                        -int(r.get("priority", 0)),
                        -action_rank.get(str(r.get("action", "forbid")).lower(), 2),
                    ),
                )
            except Exception:
                pass

            chosen = None
            for r in rules:
                cond = str(r.get("state_condition", r.get("condition", "default"))).lower()
                if cond in active:
                    chosen = r
                    break
            if not chosen:
                for r in rules:
                    if str(r.get("state_condition", r.get("condition", "default"))).lower() == "default":
                        chosen = r
                        break

            act = str((chosen or {}).get("action", "forbid")).lower()
            if act == "forbid":
                forbid_zones.add(z)
            elif act == "allow_shortest":
                allow_zones.add(z)
                force_shortest = True
            elif act == "allow":
                allow_zones.add(z)

        return forbid_zones, allow_zones, force_shortest

    def _active_timed_zones(self) -> set[str]:
        """Read the ROS-clock interval, even if a timer callback is delayed."""
        with self._sched_lock:
            if not self._schedules:
                return set()
            now_ns = self.get_clock().now().nanoseconds
            return {
                zone
                for schedule in self._schedules.values()
                if schedule.start_ros_ns <= now_ns < schedule.end_ros_ns
                for zone in schedule.zones
            }

    def _effective_keepouts(
        self, base_exclusions: List[str], conditional_rules: List[Dict[str, Any]]
    ) -> Tuple[List[str], bool]:
        """Candidate mission fields replace old fields; retained schedules do not."""
        cond_forbid, cond_allow, force_shortest = self._evaluate_conditional_rules(conditional_rules)
        permanent = set(self._permanent_exclusions)
        merged = unique_preserve(
            self._permanent_exclusions + sorted(self._active_timed_zones())
            + sorted(cond_forbid) + base_exclusions
        )
        merged = [zone for zone in merged if zone in self._labels]
        merged = [
            zone for zone in merged
            if zone not in cond_allow
            or (zone in permanent and not self._allow_overrides_permanent)
        ]
        return merged, force_shortest

    def _validate_execution_constraints(
        self, policy: Dict[str, Any], report: Optional[Dict[str, Any]] = None
    ) -> None:
        """Reject incompatible mandatory transit, without rewriting the policy."""
        if report is None:
            report = {"mode": "execution_constraints", "conflicts": []}
        base = [
            item["zone"] for item in policy.get("exclusions", [])
            if not item.get("time_condition")
        ]
        keepouts, _ = self._effective_keepouts(base, policy.get("conditional_rules", []))
        via = set(policy.get("via_zones", []))
        overlap = sorted(via & set(keepouts))
        missing = sorted(via - set(self._zone_db))
        report["execution_context"] = {"effective_keepouts": keepouts}
        if overlap or missing:
            report["accepted"] = False
            report["conflicts"].append({
                "reason": "via_and_active_keepout" if overlap else "via_geometry_missing",
                "zones": overlap or missing,
            })
            self._last_validation_report = report
            detail = "currently forbidden" if overlap else "missing map geometry"
            raise PolicyUpdateRejected(
                f"required via zone(s) {detail}: " + ", ".join(overlap or missing)
            )

    def _recalculate_and_publish_dynamic_keepouts(
        self,
        base_exclusions: List[str] | None = None,
        conditional_rules: List[Dict[str, Any]] | None = None
    ):
        base_exclusions = base_exclusions or []
        conditional_rules = conditional_rules or []

        merged, self._force_shortest = self._effective_keepouts(base_exclusions, conditional_rules)
        self._publish_forbidden(merged)


    def _nl_callback(self, msg: RosString):
        if not getattr(self, "_nav2_ready", True):
            self._ui_print("⛔ Ignored command: Nav2 is not ready.", "red")
            return

        self._ui_print(f"📨 Received command: {msg.data}", "cyan")
        self._enqueue_nl_command(msg.data)

    @staticmethod
    def _requests_mission_replacement(text: str) -> bool:






        command = re.sub(r"[-_]", " ", text or "").strip()
        if not command:
            return False
        target = (
            r"(?:current|active|previous|last|latest|existing|ongoing|old|earlier)\s+"
            r"(?:mission|task|command|order|goal|route|navigation)"
        )
        patterns = (
            rf"\b(?:cancel|abort|terminate|discard)\s+(?:the\s+)?{target}\b",
            rf"\b(?:stop|end)\s+(?:the\s+)?{target}\b",
            rf"\breplace\s+(?:the\s+)?{target}\b",
            r"\b(?:cancel|abort)\s+(?:what(?:'s|\s+is)\s+)?"
            r"(?:currently|now)\s+(?:running|executing)\b",
        )
        return any(re.search(pattern, command, re.IGNORECASE) for pattern in patterns)

    def _enqueue_nl_command(self, text: str) -> int:
        command = (text or "").strip()
        if not command:
            return 0
        replace_active = self._requests_mission_replacement(command)
        cancel_interpretation = False
        with self._command_lock:
            self._latest_command_id += 1
            command_id = self._latest_command_id
            queued = QueuedCommand(
                command_id=command_id,
                text=command,
                replace_active=replace_active,
            )
            if replace_active:


                self._minimum_valid_command_id = command_id
                self._command_queue.clear()
                if self._interpretation_active.is_set():
                    self._interpretation_cancel_requested.set()
                    cancel_interpretation = True
                with self._mission_lock:
                    self._mission_queue.clear()
            self._command_queue.append(queued)
            queue_position = len(self._command_queue)
        if cancel_interpretation:
            self._close_active_llm_connection()
        if replace_active:
            self._ui_print(
                f"Replacement command #{command_id} queued for validation; "
                "the active mission continues until validation succeeds.",
                "yellow",
            )
        else:
            self._ui_print(
                f"Command #{command_id} added to the FIFO queue "
                f"(input position {queue_position}).",
                "gray",
            )
        return command_id

    def request_pause(self) -> bool:

        with self._mission_lock:
            if (
                not self._mission_active.is_set()
                or self._mission_paused.is_set()
                or self._pause_requested.is_set()
            ):
                return False
            self._pause_requested.set()
        self._ui_print("Pause requested by the operator.", "yellow")
        return True

    def request_resume(self) -> bool:

        with self._mission_lock:
            if not self._mission_active.is_set() or not self._mission_paused.is_set():
                return False
            self._mission_paused.clear()
        self._ui_print("Resume requested by the operator.", "green")
        return True

    def request_cancel(self) -> bool:

        cancel_interpretation = False
        canceled_pending = False
        with self._command_lock:
            with self._mission_lock:
                interpretation_active = self._interpretation_active.is_set()
                mission_active = self._mission_active.is_set()
                if not (
                    interpretation_active
                    or mission_active
                    or self._mission_queue
                    or self._command_queue
                ):
                    return False

                if mission_active:
                    self._cancel_requested.set()
                    self._mission_paused.clear()
                    self._pause_requested.clear()
                elif interpretation_active:
                    active_id = self._active_interpretation_id
                    if active_id is not None:
                        self._minimum_valid_command_id = max(
                            self._minimum_valid_command_id,
                            int(active_id) + 1,
                        )
                    self._interpretation_cancel_requested.set()
                    cancel_interpretation = True
                elif self._mission_queue:
                    self._mission_queue.popleft()
                    canceled_pending = True
                elif self._command_queue:
                    self._command_queue.popleft()
                    canceled_pending = True

        if cancel_interpretation:
            self._close_active_llm_connection()
        self._ui_print("Cancel requested by the operator.", "yellow")
        if canceled_pending:
            self._ui_print("Queued command canceled before execution.", "yellow")
        return True

    def request_stop(self) -> bool:

        return self.request_cancel()

    def _close_active_llm_connection(self) -> None:

        with self._llm_request_lock:
            connections = list(self._active_llm_connections)
        for connection in connections:
            try:
                connection.close()
            except Exception:
                pass

    def _manual_policy_callback(self, msg: RosString):
        if not getattr(self, "_nav2_ready", True):
            self._ui_print("⛔ Ignored manual policy: Nav2 is not ready.", "red")
            return
        try:
            envelope = json.loads(msg.data)
            if not isinstance(envelope, dict):
                raise PolicyUpdateRejected("manual policy message must be a JSON object")
            policy = envelope.get("policy", envelope)
            user_text = str(
                envelope.get("user_text", "Manual oracle policy")
            ).strip()
            if not user_text:
                user_text = "Manual oracle policy"
            self._last_raw_policy = copy.deepcopy(policy)
            validated = self._validate_policy_obj(policy, user_text)
        except (json.JSONDecodeError, PolicyUpdateRejected, TypeError, ValueError) as exc:
            self._last_policy_decision = f"rejected manual policy: {exc}"
            self._ui_print(
                f"⛔ Manual policy rejected; active mission continues: {exc}",
                "red",
            )
            return

        with self._command_lock:
            self._latest_command_id += 1
            command_id = self._latest_command_id
        self._queue_prepared_policy(
            command_id,
            user_text,
            validated,
            source="manual_oracle",
        )

    def _prepare_and_queue_command(
        self,
        command_id: int,
        text: str,
        *,
        replace_active: bool = False,
    ) -> bool:
        try:
            policy = self._prepare_policy(text)
        except CommandInterpretationCanceled:
            self._last_policy_decision = "command interpretation canceled"
            self._ui_print("Command interpretation canceled by the operator.", "yellow")
            return False
        except PolicyUpdateRejected as exc:
            self._last_policy_decision = f"rejected: {exc}"
            self._ui_print(
                f"⛔ Command #{command_id} rejected; active mission continues: {exc}",
                "red",
            )
            return False
        except Exception as exc:
            self._last_policy_decision = f"rejected: {exc}"
            self._ui_print(
                f"⛔ Command #{command_id} preparation failed; active mission continues: {exc}",
                "red",
            )
            return False

        source = (
            "llm"
            if getattr(self, "_enable_llm", True)
            and getattr(self, "_llm_endpoint", "configured")
            else "rule_based"
        )
        return self._queue_prepared_policy(
            command_id,
            text,
            policy,
            source=source,
            replace_active=replace_active,
        )

    def _queue_prepared_policy(
        self,
        command_id: int,
        text: str,
        policy: Dict[str, Any],
        *,
        source: str,
        replace_active: bool = False,
    ) -> bool:
        try:
            self._validate_execution_constraints(policy)
        except PolicyUpdateRejected as exc:
            self._last_policy_decision = f"rejected: {exc}"
            self._ui_print(f"Command #{command_id} rejected; active mission continues: {exc}", "red")
            return False
        prepared = PreparedMission(
            command_id=command_id,
            text=text,
            policy=copy.deepcopy(policy),
            source=source,
            replace_active=replace_active,
        )
        with self._command_lock:
            if command_id < self._minimum_valid_command_id:
                self._ui_print(
                    f"Discarded canceled command #{command_id} after validation.",
                    "gray",
                )
                return False
            with self._mission_lock:
                if replace_active:
                    self._mission_queue.clear()
                    self._mission_queue.appendleft(prepared)
                    if self._mission_active.is_set():
                        self._preempt_requested.set()
                        self._mission_paused.clear()
                        self._pause_requested.clear()
                        self._ui_print(
                            f"Validated replacement command #{command_id}; "
                            "canceling the active mission now.",
                            "yellow",
                        )
                else:
                    self._mission_queue.append(prepared)
                mission_position = len(self._mission_queue)
        self._ui_print(
            f"Policy validated for command #{command_id}; mission queue "
            f"position {mission_position}.",
            "green",
        )
        return True

    def _replacement_can_preempt(self) -> bool:
        """Recheck retained constraints before canceling the active Nav2 goal."""
        if not self._preempt_requested.is_set():
            return False
        with self._mission_lock:
            if not self._mission_queue:
                self._preempt_requested.clear()
                return False
            pending = self._mission_queue[0]
            if not pending.replace_active:
                self._preempt_requested.clear()
                return False
            try:
                self._validate_execution_constraints(pending.policy)
            except PolicyUpdateRejected as exc:
                self._mission_queue.popleft()
                self._preempt_requested.clear()
                self._last_policy_decision = f"rejected: {exc}"
                self._ui_print(
                    f"Replacement command #{pending.command_id} rejected before cancellation; "
                    f"active mission continues: {exc}", "red",
                )
                return False
        return True

    def _event_callback(self, msg: RosString):

        try:
            self._handle_state_command(msg.data)
        except Exception as e:
            self._ui_print(f"[EVENT] Exception: {e}", "red")

    def _reset_callback(self, _msg: RosString):
        self._reset_policy_for_next_trial()

    def _reset_policy_for_next_trial(self) -> bool:
        with self._command_lock:
            with self._mission_lock:
                if (
                    self._mission_active.is_set()
                    or self._interpretation_active.is_set()
                ):
                    self._ui_print(
                        "⛔ Policy reset ignored while command processing is active; wait for completion.",
                        "red",
                    )
                    return False
                self._command_queue.clear()
                self._mission_queue.clear()
                self._preempt_requested.clear()

        with self._sched_lock:
            timers = list(self._timer_threads.values())
            self._timer_threads.clear()
            self._schedules.clear()
            self._active_timed.clear()
            self._next_sched_id = 1
        for timer_pair in timers:
            for timer in timer_pair:
                if timer is not None:
                    try:
                        timer.cancel()
                    except Exception:
                        pass

        self._permanent_exclusions = self._configured_permanent_exclusions[:]
        self._last_base_exclusions = []
        self._cond_rules_store = []
        self._force_shortest = False
        self._latest_dynamic_rules = []
        self._clear_object_observations()
        self._publish_object_avoidance_radius([])
        self._last_wp_names = []
        self._last_forbidden_windows = {}
        self._env_state["fire_alarm"] = False
        self._active_policy_state = self._empty_policy()
        self._policy_version += 1
        self._last_policy_decision = f"reset version {self._policy_version}"
        self._recalculate_and_publish_dynamic_keepouts([], [])
        self._clear_softcost()
        self._ui_print("🧹 Policy state reset for the next independent trial.", "green")
        return True

    def _command_worker(self):
        while rclpy.ok():
            queued: Optional[QueuedCommand] = None
            with self._command_lock:
                if self._command_queue:
                    queued = self._command_queue.popleft()
            if queued is None:
                time.sleep(0.05)
                continue

            if isinstance(queued, QueuedCommand):
                command_id = queued.command_id
                text = queued.text
                replace_active = queued.replace_active
            else:
                command_id, text = queued[:2]
                replace_active = bool(queued[2]) if len(queued) > 2 else False
            with self._command_lock:
                if command_id < self._minimum_valid_command_id:
                    continue
            with self._command_lock:
                self._active_interpretation_id = command_id
                self._interpretation_cancel_requested.clear()
                self._interpretation_active.set()
            self._ui_print(
                f"Interpreting command #{command_id} with {self._llm_model}.",
                "cyan",
            )
            try:
                self._prepare_and_queue_command(
                    command_id,
                    text,
                    replace_active=replace_active,
                )
            finally:
                with self._command_lock:
                    if self._active_interpretation_id == command_id:
                        self._active_interpretation_id = None
                        self._interpretation_active.clear()

    def _mission_worker(self):
        while rclpy.ok():
            mission: Optional[PreparedMission] = None
            with self._mission_lock:
                if self._mission_queue:
                    mission = self._mission_queue.popleft()
                    self._preempt_requested.clear()
                    self._cancel_requested.clear()
                    self._pause_requested.clear()
                    self._mission_paused.clear()
                    self._mission_active.set()
                    self._active_command_id = mission.command_id
            if mission is None:
                time.sleep(0.05)
                continue
            version_before = self._policy_version
            try:
                self._run_single_mission(
                    mission.text,
                    prepared_policy=mission.policy,
                    policy_source=mission.source,
                    command_id=mission.command_id,
                    replace_active=mission.replace_active,
                )
            except Exception as e:
                self._ui_print(f"❌ Mission error: {e}", "red")
            finally:
                if self._policy_version != version_before:
                    self._clear_object_observations()
                    self._latest_dynamic_rules = []
                    self._publish_object_avoidance_radius([])
                with self._mission_lock:
                    self._active_command_id = None
                    self._preempt_requested.clear()
                    self._cancel_requested.clear()
                    self._pause_requested.clear()
                    self._mission_paused.clear()
                    self._mission_active.clear()


    def _active_keepout_windows(self) -> List[Tuple[str, dt.datetime, dt.datetime]]:
        now = dt.datetime.now(self._tz)
        with self._sched_lock:
            active_ids = list(self._active_timed.keys())
            schedules = {sid: self._schedules.get(sid) for sid in active_ids}
        out: List[Tuple[str, dt.datetime, dt.datetime]] = []
        for sid, tk in schedules.items():
            if not tk:
                continue
            st = tk.start_wall
            en = tk.end_wall or (tk.start_wall + dt.timedelta(minutes=self._default_forbid_minutes))
            for z in tk.zones:
                out.append((z, st, en))
        base_end = now + dt.timedelta(minutes=self._default_forbid_minutes)
        for z in (self._last_base_exclusions or []):
            out.append((z, now, base_end))
        perm_end = now + dt.timedelta(minutes=self._default_forbid_minutes)
        for z in (self._permanent_exclusions or []):
            out.append((z, now, perm_end))
        cond_forbid, _cond_allow, _ = self._evaluate_conditional_rules(self._cond_rules_store or [])
        cond_end = now + dt.timedelta(minutes=self._default_forbid_minutes)
        for z in sorted(cond_forbid):
            out.append((z, now, cond_end))
        latest: Dict[str, Tuple[dt.datetime, dt.datetime]] = {}
        for z, st, en in out:
            if z not in self._labels:
                continue
            prev = latest.get(z)
            if (prev is None) or (st > prev[0]):
                latest[z] = (st, en)
        rows = [(z, latest[z][0], latest[z][1]) for z in sorted(latest.keys())]
        return rows

    def _print_mission_snapshot(self, user_text: str, wp_names: List[str]):
        self._ui_print("==================== Mission start ====================")
        self._ui_print(f"User command: {user_text}")
        self._ui_print("🚫 Forbidden Zones:")
        if getattr(self, "_last_forbidden_windows", None) and len(self._last_forbidden_windows) > 0:
            for z in sorted(self._last_forbidden_windows.keys()):
                st, en = self._last_forbidden_windows[z]
                self._ui_print(f"  ⏰ {z}: {st.strftime('%Y-%m-%d %H:%M')} → {en.strftime('%Y-%m-%d %H:%M')}")

        now = dt.datetime.now(self._tz)
        pending_rows: List[Tuple[str, dt.datetime, dt.datetime]] = []

        with self._sched_lock:
            schedules = list(self._schedules.values())
            active_ids = set(self._active_timed.keys())

        for tk in schedules:
            if tk.id in active_ids:
                continue

            st = tk.start_wall
            en = tk.end_wall or (st + dt.timedelta(minutes=self._default_forbid_minutes))

            if st > now:
                for z in tk.zones:
                    if z in self._labels:
                        pending_rows.append((z, st, en))

        if pending_rows:
            soonest: Dict[str, Tuple[dt.datetime, dt.datetime]] = {}

            for z, st, en in pending_rows:
                prev = soonest.get(z)
                if prev is None or st < prev[0]:
                    soonest[z] = (st, en)

            self._ui_print("🚫 Scheduled Forbidden Zones:")
            for z in sorted(soonest.keys()):
                st, en = soonest[z]
                self._ui_print(f"  ⏰ {z}: {st.strftime('%Y-%m-%d %H:%M')} → {en.strftime('%Y-%m-%d %H:%M')}")

        if getattr(self, "_active_softcost", None) and len(self._active_softcost) > 0:
            items = ", ".join([f"{z}:{c}" for z, c in sorted(self._active_softcost.items())])
            self._ui_print(f"🟦 SoftCost Zones: {items}")
        else:
            self._ui_print("🟦 SoftCost Zones: ")
        objs = []
        for r in (getattr(self, "_latest_dynamic_rules", []) or []):
            try:
                cls = str(r.get("class", "")).strip()
                if cls:
                    radius = float(r.get("radius", 1.5))
                    objs.append(f"{cls.capitalize()}({radius:g}m)")
            except Exception:
                pass
        objs_pretty = ", ".join(unique_preserve(objs)) if objs else ""
        self._ui_print(f"👥 Objects: {objs_pretty}")
        seq = ", ".join(wp_names) if wp_names else "-"
        self._ui_print(f"🧭 Waypoint Order: {seq}")
        self._ui_print("")

    def _print_runtime_snapshot(self):
        self._ui_print("🚫 Forbidden Zones:")
        if getattr(self, "_last_forbidden_windows", None) and len(self._last_forbidden_windows) > 0:
            for z in sorted(self._last_forbidden_windows.keys()):
                st, en = self._last_forbidden_windows[z]
                self._ui_print(f"  ⏰ {z}: {st.strftime('%Y-%m-%d %H:%M')} → {en.strftime('%Y-%m-%d %H:%M')}")
        if getattr(self, "_active_softcost", None) and len(self._active_softcost) > 0:
            items = ", ".join([f"{z}:{c}" for z, c in sorted(self._active_softcost.items())])
            self._ui_print(f"🟦 SoftCost Zones: {items}")
        else:
            self._ui_print("🟦 SoftCost Zones: ")
        objs = []
        for r in (getattr(self, "_latest_dynamic_rules", []) or []):
            try:
                cls = str(r.get("class", "")).strip()
                if cls:
                    radius = float(r.get("radius", 1.5))
                    objs.append(f"{cls.capitalize()}({radius:g}m)")
            except Exception:
                pass
        objs_pretty = ", ".join(unique_preserve(objs)) if objs else ""
        self._ui_print(f"👥 Objects: {objs_pretty}", "yellow")
        seq = ", ".join(getattr(self, "_last_wp_names", []) or []) or "-"
        self._ui_print(f"🧭 Waypoint Order: {seq}", "cyan")
        self._ui_print("")

    def _run_single_mission(
        self,
        text: str,
        prepared_policy: Optional[Dict[str, Any]] = None,
        policy_source: str = "internal",
        command_id: Optional[int] = None,
        replace_active: bool = False,
    ):
        text = self._apply_aisle_aliases(text)
        self._ui_print("")
        self._policy_commit_replaces_previous = bool(replace_active)
        self._policy_commit_user_text = text
        try:
            if prepared_policy is None:
                validated = self._prepare_policy(text)
            else:
                validated = self._validate_policy_obj(copy.deepcopy(prepared_policy), text)
            # Resolve all required transit poses before any state change or output.
            # Later timer events may change costmaps, never erase required stops.
            via_plan = self._via_waypoints_from_zones(validated.get("via_zones", []), [])
            plan_json = self._commit_policy_snapshot(validated)
        except PolicyUpdateRejected as exc:
            self._last_policy_decision = f"rejected: {exc}"
            self._ui_print(f"⛔ Policy update rejected; previous policy preserved: {exc}", "red")
            return
        finally:
            self._policy_commit_replaces_previous = False
            self._policy_commit_user_text = ""
        self._publish_validated_policy(
            source=policy_source,
            command_id=command_id,
            user_text=text,
            policy=plan_json,
        )
        self._active_softcost = {}

        dyn_rules = plan_json.get("dynamic_object_rules", []) or []
        self._latest_dynamic_rules = dyn_rules
        self._publish_object_avoidance_radius(dyn_rules)
        if not dyn_rules:
            self._clear_object_observations()
        if dyn_rules and bool(self.get_parameter("enable_yolo_override_from_nl").value):
            self._ensure_yolo_ready()

        plan        = plan_json.get("plan", [])
        weights_raw = plan_json.get("weights", [])


        raw_excl = plan_json.get("exclusions", []) or []
        plain_exclusions: List[str] = []
        timed_rules: List[Dict[str, Any]] = []

        for e in raw_excl:
            if isinstance(e, dict) and e.get("zone"):
                z = str(e["zone"]).upper()
                tc = e.get("time_condition")
                if tc:
                    timed_rules.append({"zone": z, "time_condition": tc})
                else:
                    plain_exclusions.append(z)
            elif isinstance(e, str):
                plain_exclusions.append(e.upper())






        for r in timed_rules:
            tc = r["time_condition"]
            try:
                st = dt.datetime.fromisoformat(tc["start"])
                en = dt.datetime.fromisoformat(tc["end"])
                if st.tzinfo is None: st = st.replace(tzinfo=self._tz)
                if en.tzinfo is None: en = en.replace(tzinfo=self._tz)
            except Exception:
                continue

            rep = tc.get("repeat")
            rep = rep if rep in (None, "daily") else None

            sid = self._schedule_keepout(
                TimedKeepout(zones=[r["zone"]], start_wall=st, end_wall=en, repeat=rep)
            )



        exclusions = unique_preserve(plain_exclusions)
        self._last_base_exclusions = exclusions[:]





        cond_rules: List[Dict[str, Any]] = plan_json.get("conditional_rules", []) or []
        self._cond_rules_store = cond_rules[:]

        self._recalculate_and_publish_dynamic_keepouts(
            base_exclusions=exclusions,
            conditional_rules=cond_rules
        )

        weights_by_zone: Dict[str, int] = {}
        for w in weights_raw:
            if not isinstance(w, dict):
                continue
            z = str(w.get("zone", "")).upper()
            if z in self._labels:
                try:
                    cost = normalize_soft_cost(w.get("cost"))
                    if cost is not None:
                        weights_by_zone[z] = cost
                except Exception:
                    continue
        active_keepout_zones = set(exclusions)
        active_keepout_zones.update(r.get("zone") for r in timed_rules if isinstance(r, dict))
        weights_raw = [
            {"zone": z, "cost": c}
            for z, c in sorted(weights_by_zone.items())
            if z not in active_keepout_zones
        ]

        weights: List[WeightedRule] = []
        for w in weights_raw:
            try:
                weights.append(WeightedRule(zone=str(w["zone"]).upper(), cost=int(w["cost"])))
            except Exception:
                continue
        if weights:
            self._publish_softcost(weights)

        plan = via_plan + plan


        waypoints, names = self._waypoints_from_plan_with_names(plan)
        if not waypoints:
            self._ui_print("⚠️  No valid destination; ending mission.", "yellow")
            self._ui_print("=====================================================\n", "bold")
            return

        self._last_wp_names = names[:]
        first_name = names[0] if names else ""
        first_distance = self._dist_to_pose(waypoints[0]) if waypoints else None
        self._update_navigation_progress(
            current=1,
            total=len(waypoints),
            destination=first_name,
            remaining_m=first_distance,
        )
        self._print_mission_snapshot(user_text=text, wp_names=names)


        self._ensure_navigator()


        result = self._follow_waypoints(
            waypoints,
            names,
            retry_early_abort_once=replace_active,
        )
        if result == TaskResult.SUCCEEDED:
            self._update_navigation_progress(
                current=len(waypoints),
                total=len(waypoints),
                destination=names[-1] if names else "Mission complete",
                remaining_m=0.0,
            )
            self._ui_print("🏁 Mission done: SUCCEEDED", "green")
        elif result == TaskResult.CANCELED:
            if self._preempt_requested.is_set():
                self._ui_print(
                    "Mission state: CANCELED; validated replacement queued.",
                    "yellow",
                )
            else:
                self._ui_print("⚠️  Mission state: CANCELED", "yellow")
        else:
            self._ui_print("❌ Mission failed: FAILED", "red")


        self._recalculate_and_publish_dynamic_keepouts(
            base_exclusions=exclusions,
            conditional_rules=cond_rules
        )


        perm_cost = set([s.upper() for s in (plan_json.get("perm_cost_zones") or [])])
        sent_cost: Dict[str, int] = {w.zone.upper(): int(w.cost) for w in (weights or [])}
        retain = {z: c for z, c in sent_cost.items() if z in perm_cost}

        if retain:
            rules_retain = [WeightedRule(zone=z, cost=c) for z, c in retain.items()]
            self._publish_softcost(rules_retain)
            self._ui_print("🧷 Keeping permanent costs: " + ", ".join(f"{z}:{c}" for z, c in retain.items()), "magenta")
        else:
            self._clear_softcost()
            self._ui_print("🧽 Mission end → cleared temporary soft costs", "magenta")

        self._ui_print("=====================================================\n", "bold")


    @staticmethod
    def _empty_policy() -> Dict[str, Any]:
        return {
            "decision": "accept",
            "reason_code": "none",
            "plan": [],
            "via_zones": [],
            "exclusions": [],
            "weights": [],
            "dynamic_object_rules": [],
            "conditional_rules": [],
        }

    @staticmethod
    def _policy_has_effect(policy: Dict[str, Any]) -> bool:
        return any(
            policy.get(field)
            for field in (
                "plan",
                "via_zones",
                "exclusions",
                "weights",
                "dynamic_object_rules",
                "conditional_rules",
            )
        )

    def _validate_policy_obj(
        self,
        obj: Any,
        user_text: str,
        apply_safety_shield: bool = True,
    ) -> Dict[str, Any]:









        del user_text, apply_safety_shield
        if not isinstance(obj, dict):
            raise PolicyUpdateRejected("top-level output must be a JSON object")

        report: Dict[str, Any] = {
            "mode": "schema_and_execution_constraints",
            "discarded": [],
            "normalized": [],
            "conflicts": [],
        }

        allowed_fields = set(self._empty_policy())
        unknown_fields = sorted(set(obj) - allowed_fields)
        if unknown_fields:
            report["discarded"].append({
                "field": "top_level",
                "reason": "unknown_fields",
                "values": unknown_fields,
            })
            self._last_validation_report = report
            raise PolicyUpdateRejected(
                "unknown top-level field(s): " + ", ".join(unknown_fields)
            )

        decision = str(obj.get("decision", "accept")).strip().lower()
        reason_code = str(obj.get("reason_code", "none")).strip().lower()
        if decision not in {"accept", "reject"}:
            raise PolicyUpdateRejected("decision must be accept or reject")
        if reason_code not in {"none", "contradiction", "unsupported"}:
            raise PolicyUpdateRejected("unsupported reason_code")
        if decision != "accept":
            self._last_validation_report = report
            raise PolicyUpdateRejected(f"interpreter decision={decision}: {reason_code}")

        list_fields = (
            "plan",
            "via_zones",
            "exclusions",
            "weights",
            "dynamic_object_rules",
            "conditional_rules",
        )
        for field in list_fields:
            if field in obj and not isinstance(obj[field], list):
                raise PolicyUpdateRejected(f"field '{field}' must be a list")

        configured_zones = set(self._zone_db)

        def valid_zone(value: Any, field: str) -> str:
            if not isinstance(value, str):
                report["discarded"].append({"field": field, "reason": "zone_not_string"})
                self._last_validation_report = report
                raise PolicyUpdateRejected(f"field '{field}' contains a non-string zone")
            zone = value.strip().upper()
            if zone not in configured_zones:
                report["discarded"].append({
                    "field": field, "reason": "unknown_zone", "value": zone,
                })
                self._last_validation_report = report
                raise PolicyUpdateRejected(
                    f"field '{field}' contains unknown zone {zone}"
                )
            return zone

        plan: List[Dict[str, Any]] = []
        seen_destinations = set()
        for item in obj.get("plan", []) or []:
            if not isinstance(item, dict) or "dest" not in item:
                report["discarded"].append({"field": "plan", "reason": "invalid_item"})
                self._last_validation_report = report
                raise PolicyUpdateRejected("field 'plan' contains an invalid item")
            destination = item["dest"]
            normalized_destination: Any = None
            if isinstance(destination, str):
                match = re.fullmatch(r"WP\s*([0-9]+)", destination.strip(), re.IGNORECASE)
                waypoint = f"WP{int(match.group(1))}" if match else ""
                if waypoint in self._waypoints:
                    normalized_destination = waypoint
                else:
                    report["discarded"].append({
                        "field": "plan", "reason": "invalid_waypoint",
                        "value": destination,
                    })
            elif isinstance(destination, dict):
                try:
                    x = float(destination["x"])
                    y = float(destination["y"])
                    yaw = float(destination.get("yaw", 0.0))
                except (KeyError, TypeError, ValueError, OverflowError):
                    x = y = yaw = float("nan")
                if (
                    math.isfinite(x) and math.isfinite(y) and math.isfinite(yaw)
                ):
                    normalized_destination = {"x": x, "y": y, "yaw": yaw}
                else:
                    report["discarded"].append({
                        "field": "plan", "reason": "invalid_coordinate",
                    })
            if normalized_destination is None:
                self._last_validation_report = report
                raise PolicyUpdateRejected("field 'plan' contains an invalid destination")
            key = json.dumps(normalized_destination, sort_keys=True)
            if key not in seen_destinations:
                plan.append({"dest": normalized_destination})
                seen_destinations.add(key)

        via_zones: List[str] = []
        for item in obj.get("via_zones", []) or []:
            raw_zone = item.get("zone") if isinstance(item, dict) else item
            zone = valid_zone(raw_zone, "via_zones")
            if zone not in via_zones:
                via_zones.append(zone)

        def normalize_time_condition(value: Any) -> Optional[Dict[str, Any]]:
            if not isinstance(value, dict):
                return None
            try:
                start = dt.datetime.fromisoformat(str(value["start"]))
                end = dt.datetime.fromisoformat(str(value["end"]))
            except (KeyError, TypeError, ValueError):
                return None
            if start.tzinfo is None:
                start = start.replace(tzinfo=self._tz)
            if end.tzinfo is None:
                end = end.replace(tzinfo=self._tz)
            if end <= start:
                return None
            repeat = value.get("repeat")
            if repeat not in (None, "daily"):
                return None
            return {
                "start": start.isoformat(),
                "end": end.isoformat(),
                "repeat": repeat,
            }

        exclusions: List[Dict[str, Any]] = []
        seen_exclusions = set()
        for item in obj.get("exclusions", []) or []:
            if isinstance(item, str):
                item = {"zone": item}
            if not isinstance(item, dict):
                report["discarded"].append({"field": "exclusions", "reason": "invalid_item"})
                self._last_validation_report = report
                raise PolicyUpdateRejected("field 'exclusions' contains an invalid item")
            zone = valid_zone(item.get("zone"), "exclusions")
            rule: Dict[str, Any] = {"zone": zone}
            duration_minutes = item.get("duration_minutes")
            time_condition = item.get("time_condition")
            if duration_minutes is not None:
                try:
                    minutes = float(duration_minutes)
                except (TypeError, ValueError, OverflowError):
                    minutes = float("nan")
                if not math.isfinite(minutes) or not 0.0 < minutes <= 1440.0:
                    report["discarded"].append({
                        "field": "exclusions", "reason": "invalid_duration", "zone": zone,
                    })
                    self._last_validation_report = report
                    raise PolicyUpdateRejected(
                        f"field 'exclusions' contains an invalid duration for zone {zone}"
                    )
                start = dt.datetime.now(self._tz)
                end = start + dt.timedelta(minutes=minutes)
                rule["time_condition"] = {
                    "start": start.isoformat(), "end": end.isoformat(), "repeat": None,
                }
                report["normalized"].append({
                    "field": "exclusions.duration_minutes",
                    "reason": "converted_to_time_condition",
                    "zone": zone,
                })
            elif time_condition is not None:
                condition = normalize_time_condition(time_condition)
                if condition is None:
                    report["discarded"].append({
                        "field": "exclusions", "reason": "invalid_time_condition", "zone": zone,
                    })
                    self._last_validation_report = report
                    raise PolicyUpdateRejected(
                        f"field 'exclusions' contains an invalid time condition for zone {zone}"
                    )
                rule["time_condition"] = condition
            key = json.dumps(rule, sort_keys=True)
            if key not in seen_exclusions:
                exclusions.append(rule)
                seen_exclusions.add(key)

        weights_by_zone: Dict[str, int] = {}
        for item in obj.get("weights", []) or []:
            if not isinstance(item, dict):
                report["discarded"].append({"field": "weights", "reason": "invalid_item"})
                self._last_validation_report = report
                raise PolicyUpdateRejected("field 'weights' contains an invalid item")
            zone = valid_zone(item.get("zone"), "weights")
            cost = normalize_soft_cost(item.get("cost"))
            if cost is None:
                report["discarded"].append({
                    "field": "weights", "reason": "invalid_cost", "zone": zone,
                    "value": item.get("cost"),
                })
                self._last_validation_report = report
                raise PolicyUpdateRejected(
                    f"field 'weights' contains an invalid cost for zone {zone}"
                )
            if zone in weights_by_zone and weights_by_zone[zone] != cost:
                report["conflicts"].append({
                    "field": "weights", "reason": "conflicting_duplicate_cost",
                    "zone": zone, "values": [weights_by_zone[zone], cost],
                })
                self._last_validation_report = report
                raise PolicyUpdateRejected(
                    f"conflicting costs for zone {zone}: "
                    f"{weights_by_zone[zone]} and {cost}"
                )
            weights_by_zone[zone] = cost

        dynamic_rules: List[Dict[str, Any]] = []
        for item in obj.get("dynamic_object_rules", []) or []:
            if not isinstance(item, dict):
                report["discarded"].append({
                    "field": "dynamic_object_rules", "reason": "invalid_item",
                })
                self._last_validation_report = report
                raise PolicyUpdateRejected(
                    "field 'dynamic_object_rules' contains an invalid item"
                )
            object_class = str(item.get("class", "")).strip().lower()
            object_class = {
                "people": "person", "humans": "person", "pedestrians": "person",
            }.get(object_class, object_class)
            allowed_class = object_class in {"person", "forklift"}
            try:
                radius = float(item.get("radius"))
            except (TypeError, ValueError, OverflowError):
                radius = float("nan")
            if (
                not allowed_class
                or not math.isfinite(radius)
                or not 0.1 <= radius <= 10.0
            ):
                report["discarded"].append({
                    "field": "dynamic_object_rules", "reason": "invalid_rule",
                    "class": object_class,
                })
                self._last_validation_report = report
                raise PolicyUpdateRejected(
                    "field 'dynamic_object_rules' contains an unsupported class or radius"
                )
            rule = {"class": object_class, "radius": radius}
            if rule not in dynamic_rules:
                dynamic_rules.append(rule)

        allowed_conditions = {"default", "fire_alarm", "low_battery"}
        allowed_actions = {"forbid", "allow", "allow_shortest"}
        conditional_by_zone: Dict[str, List[Dict[str, Any]]] = {}
        conditional_keys: Dict[Tuple[str, str, int], str] = {}
        for item in obj.get("conditional_rules", []) or []:
            if not isinstance(item, dict) or not isinstance(item.get("rules"), list):
                report["discarded"].append({
                    "field": "conditional_rules", "reason": "invalid_item",
                })
                self._last_validation_report = report
                raise PolicyUpdateRejected(
                    "field 'conditional_rules' contains an invalid item"
                )
            zone = valid_zone(item.get("zone"), "conditional_rules")
            for raw_rule in item["rules"]:
                if not isinstance(raw_rule, dict):
                    report["discarded"].append({
                        "field": "conditional_rules", "reason": "invalid_rule", "zone": zone,
                    })
                    self._last_validation_report = report
                    raise PolicyUpdateRejected(
                        f"field 'conditional_rules' contains an invalid rule for zone {zone}"
                    )
                condition = str(
                    raw_rule.get("state_condition", raw_rule.get("condition", ""))
                ).strip().lower()
                action = str(raw_rule.get("action", "")).strip().lower()
                try:
                    priority = int(raw_rule.get("priority", 0))
                except (TypeError, ValueError, OverflowError):
                    priority = 0
                if condition not in allowed_conditions or action not in allowed_actions:
                    report["discarded"].append({
                        "field": "conditional_rules", "reason": "unsupported_condition_or_action",
                        "zone": zone, "condition": condition, "action": action,
                    })
                    self._last_validation_report = report
                    raise PolicyUpdateRejected(
                        f"unsupported condition or action for zone {zone}"
                    )
                if condition == "default" and action != "forbid":
                    report["discarded"].append({
                        "field": "conditional_rules",
                        "reason": "incompatible_condition_action",
                        "zone": zone,
                        "condition": condition,
                        "action": action,
                    })
                    self._last_validation_report = report
                    raise PolicyUpdateRejected(
                        f"default condition for zone {zone} only supports forbid"
                    )
                key = (zone, condition, priority)
                previous_action = conditional_keys.get(key)
                if previous_action is not None and previous_action != action:
                    report["conflicts"].append({
                        "field": "conditional_rules", "zone": zone,
                        "condition": condition, "priority": priority,
                        "actions": sorted({previous_action, action}),
                    })
                    self._last_validation_report = report
                    raise PolicyUpdateRejected(
                        f"conflicting actions for zone {zone}, condition {condition}, "
                        f"priority {priority}"
                    )
                conditional_keys[key] = action
                normalized_rule = {
                    "priority": priority,
                    "state_condition": condition,
                    "action": action,
                }
                bucket = conditional_by_zone.setdefault(zone, [])
                if normalized_rule not in bucket:
                    bucket.append(normalized_rule)





        for zone, rules in list(conditional_by_zone.items()):
            default_forbids = [
                rule for rule in rules
                if rule["state_condition"] == "default"
                and rule["action"] == "forbid"
            ]
            event_rules = [
                rule for rule in rules if rule["state_condition"] != "default"
            ]
            if not default_forbids:
                continue
            default_priority = max(rule["priority"] for rule in default_forbids)
            if event_rules and not all(
                rule["priority"] > default_priority for rule in event_rules
            ):
                report["conflicts"].append({
                    "field": "conditional_rules",
                    "reason": "event_priority_not_above_default",
                    "zone": zone,
                })
                self._last_validation_report = report
                raise PolicyUpdateRejected(
                    f"event rule priority must exceed default forbid for zone {zone}"
                )
            conditional_by_zone[zone] = event_rules
            has_static_exclusion = any(
                item["zone"] == zone and "time_condition" not in item
                for item in exclusions
            )
            if not has_static_exclusion:
                exclusions.append({"zone": zone})
            report["normalized"].append({
                "field": "conditional_rules",
                "reason": "default_forbid_moved_to_base_exclusion",
                "zone": zone,
            })
            if not event_rules:
                del conditional_by_zone[zone]

        conditional_rules = [
            {
                "zone": zone,
                "rules": sorted(
                    rules,
                    key=lambda rule: (
                        -rule["priority"], rule["state_condition"], rule["action"]
                    ),
                ),
            }
            for zone, rules in sorted(conditional_by_zone.items())
        ]

        exclusion_zones = {item["zone"] for item in exclusions}
        via_overlap = sorted(set(via_zones) & exclusion_zones)
        if via_overlap:
            report["conflicts"].append({
                "reason": "via_and_exclusion", "zones": via_overlap,
            })
            self._last_validation_report = report
            raise PolicyUpdateRejected(
                "zones cannot be both required for traversal and excluded: "
                + ", ".join(via_overlap)
            )

        conditional_zones = set(conditional_by_zone)
        conditional_route_overlap = sorted(conditional_zones & set(via_zones))
        if conditional_route_overlap:
            report["conflicts"].append({
                "reason": "conditional_and_via_roles",
                "zones": conditional_route_overlap,
            })
            self._last_validation_report = report
            raise PolicyUpdateRejected(
                "conditional zones cannot also be required for unconditional transit: "
                + ", ".join(conditional_route_overlap)
            )

        legacy_conditional_zones = {
            zone for zone, rules in conditional_by_zone.items()
            if any(rule["state_condition"] == "default" for rule in rules)
        }
        legacy_duplicates = sorted(legacy_conditional_zones & exclusion_zones)
        if legacy_duplicates:
            exclusions = [
                item for item in exclusions
                if item["zone"] not in legacy_conditional_zones
            ]
            for zone in legacy_conditional_zones:
                weights_by_zone.pop(zone, None)
            exclusion_zones = {item["zone"] for item in exclusions}
            report["discarded"].append({
                "reason": "legacy_conditional_policy_dominates_base_duplicate",
                "zones": legacy_duplicates,
            })

        overlapping_weight_zones = sorted(exclusion_zones & set(weights_by_zone))
        if overlapping_weight_zones:
            report["conflicts"].append({
                "reason": "keepout_and_soft_cost",
                "zones": overlapping_weight_zones,
            })
            self._last_validation_report = report
            raise PolicyUpdateRejected(
                "zones cannot have both keepout and soft cost: "
                + ", ".join(overlapping_weight_zones)
            )
        normalized = {
            "decision": "accept",
            "reason_code": "none",
            "plan": plan,
            "via_zones": via_zones,
            "exclusions": exclusions,
            "weights": [
                {"zone": zone, "cost": weights_by_zone[zone]}
                for zone in sorted(weights_by_zone)
            ],
            "dynamic_object_rules": dynamic_rules,
            "conditional_rules": conditional_rules,
        }
        if not self._policy_has_effect(normalized):
            self._last_validation_report = report
            raise PolicyUpdateRejected("no valid policy field remains after validation")
        self._validate_execution_constraints(normalized, report)
        report["accepted"] = True
        self._last_validation_report = report
        return normalized

    def _validate_policy_obj_semantic_legacy(
        self,
        obj: Any,
        user_text: str,
        apply_safety_shield: bool = True,
    ) -> Dict[str, Any]:
        if not isinstance(obj, dict):
            raise PolicyUpdateRejected("top-level output must be a JSON object")

        decision = str(obj.get("decision", "accept")).strip().lower()
        reason_code = str(obj.get("reason_code", "none")).strip().lower()
        if decision not in {"accept", "reject"}:
            raise PolicyUpdateRejected("decision must be accept or reject")
        if reason_code not in {"none", "contradiction", "unsupported"}:
            raise PolicyUpdateRejected("unsupported reason_code")
        if decision != "accept":
            raise PolicyUpdateRejected(f"interpreter decision={decision}: {reason_code}")

        if apply_safety_shield:
            conflict = self._detect_explicit_command_conflict(user_text)
            if conflict:
                raise PolicyUpdateRejected(f"explicit command conflict: {conflict}")

        list_fields = (
            "plan",
            "via_zones",
            "exclusions",
            "weights",
            "dynamic_object_rules",
            "conditional_rules",
        )
        for field in list_fields:
            if field in obj and not isinstance(obj[field], list):
                raise PolicyUpdateRejected(f"field '{field}' must be a list")

        configured_zones = set(self._zone_db)
        mentioned_zones = {
            zone.upper()
            for zone in re.findall(
                r"\b(?:zone|aisle|section|area)\b\s*([A-Z])\b",
                user_text or "",
                flags=re.IGNORECASE,
            )
        }
        mentioned_zones.update(re.findall(r"\b([A-Z])\b", user_text or ""))
        policy_zones = set()
        for field in (
            "via_zones", "exclusions", "weights", "conditional_rules",
        ):
            for item in obj.get(field, []) or []:
                value = item.get("zone") if isinstance(item, dict) else item
                if isinstance(value, str) and len(value.strip()) == 1:
                    policy_zones.add(value.strip().upper())
        unknown_zones = sorted((mentioned_zones | policy_zones) - configured_zones)
        if unknown_zones:
            raise PolicyUpdateRejected(
                "unknown zone label(s): " + ", ".join(unknown_zones)
                + "; configured zones: " + ", ".join(sorted(configured_zones))
            )

        normalized = self._canonicalize_plan_obj(obj, user_text)

        relative_duration = None
        duration_match = re.search(
            r"\b(\d+(?:\.\d+)?)\s*(minutes?|mins?|min)\b",
            user_text or "",
            re.IGNORECASE,
        )
        if duration_match:
            relative_duration = float(duration_match.group(1))
        else:
            hour_match = re.search(
                r"\b(\d+(?:\.\d+)?)\s*(hours?|hrs?|hr)\b",
                user_text or "",
                re.IGNORECASE,
            )
            if hour_match:
                relative_duration = float(hour_match.group(1)) * 60.0
            elif re.search(
                r"\b(?:half\s+(?:an?\s+)?hour|half-hour|thirty\s+minutes?)\b",
                user_text or "",
                re.IGNORECASE,
            ):
                relative_duration = 30.0
        if relative_duration is not None:
            timed_durations = []
            for exclusion in normalized.get("exclusions", []):
                condition = exclusion.get("time_condition") if isinstance(exclusion, dict) else None
                if not isinstance(condition, dict):
                    continue
                try:
                    start = dt.datetime.fromisoformat(condition["start"])
                    end = dt.datetime.fromisoformat(condition["end"])
                except (KeyError, TypeError, ValueError):
                    continue
                timed_durations.append((end - start).total_seconds() / 60.0)
            if not timed_durations or not any(
                math.isclose(value, relative_duration, abs_tol=1e-6)
                for value in timed_durations
            ):
                raise PolicyUpdateRejected(
                    "relative-time exclusion must preserve the explicitly requested duration"
                )
        if apply_safety_shield:
            normalized = self._enforce_explicit_hard_keepouts(normalized, user_text)

        via = set(normalized.get("via_zones", []))
        excluded = {
            item.get("zone") for item in normalized.get("exclusions", [])
            if isinstance(item, dict)
        }
        overlap = sorted(via & excluded)
        if overlap:
            if apply_safety_shield:
                raise PolicyUpdateRejected(
                    "zones cannot be both required for traversal and excluded: " + ", ".join(overlap)
                )
            normalized["via_zones"] = [zone for zone in normalized["via_zones"] if zone not in excluded]
        if not self._policy_has_effect(normalized):
            raise PolicyUpdateRejected("no grounded policy field remains after validation")
        return normalized

    def _enforce_explicit_hard_keepouts(self, policy: Dict[str, Any], user_text: str) -> Dict[str, Any]:
        text = user_text or ""
        hard = set()
        explicitly_open = set()

        def _state_after_zone(zone_ref: str, states: str, span: int = 24):
            pattern = re.compile(
                rf"\b(?:entering\s+)?{zone_ref}\b(?P<gap>.{{0,{span}}}?)\b(?:{states})\b",
                re.IGNORECASE,
            )
            for match in pattern.finditer(text):

                if not re.search(r"\b[A-Z]\b", match.group("gap")):
                    return match
            return None

        for zone in self._labels:
            zone_ref = rf"(?:(?:zone|aisle|section|area)\s*)?(?-i:{re.escape(zone)})"
            explicit_denial = _state_after_zone(
                zone_ref,
                r"not\s+(?:allowed|permitted)",
            )
            state_forbid = _state_after_zone(
                zone_ref,
                r"forbidden|blocked|closed|restricted|off[- ]limits|"
                r"being\s+serviced|under\s+maintenance|occupied\s+for\s+loading",
            )
            hard_action = re.search(
                rf"\b(?:forbid|block|close|avoid)\s+{zone_ref}\b|"
                rf"\bkeep\s+{zone_ref}\b.{{0,12}}\b(?:closed|blocked|restricted|off[- ]limits)\b",
                text,
                re.IGNORECASE,
            )
            if explicit_denial or state_forbid or hard_action:
                hard.add(zone)
            state_open = _state_after_zone(zone_ref, r"open|allowed|available", 16)
            prefix_open = re.search(
                rf"\b(?:open|allow)\s+{zone_ref}\b",
                text,
                re.IGNORECASE,
            )
            if not explicit_denial and (state_open or prefix_open):
                hard.discard(zone)
                explicitly_open.add(zone)
            if re.search(
                rf"\b(?:do\s+not|don't|not)\s+(?:avoid|forbid|block|close)\s+{zone_ref}\b|"
                rf"\b{zone_ref}\b\s+(?:is|was|should\s+be)\s+not\s+(?:forbidden|blocked|closed|restricted)\b|"
                rf"\b{zone_ref}\b.{{0,35}}\b(?:closed|blocked|restricted)\b.{{0,35}}\b(?:yesterday|previously|formerly)\b.{{0,35}}\b(?:open|available|allowed)\b.{{0,12}}\b(?:today|now|currently)\b|"
                rf"\b{zone_ref}\b.{{0,12}}\b(?:yesterday|previously|formerly)\b.{{0,25}}\b(?:closed|blocked|restricted)\b.{{0,35}}\b(?:open|available|allowed)\b.{{0,12}}\b(?:today|now|currently)\b|"
                rf"\b{zone_ref}\b.{{0,60}}\b(?:yesterday|previously|formerly|previous\s+shift|earlier)\b.{{0,80}}\b(?:open|available|allowed)\b.{{0,12}}\b(?:today|now|currently)\b|"
                rf"\b{zone_ref}\b.{{0,60}}\b(?:yesterday|previously|formerly|previous\s+shift|earlier)\b.{{0,80}}\b(?:today|now|currently)\b.{{0,12}}\b(?:open|available|allowed)\b",
                text,
                re.IGNORECASE,
            ):
                hard.discard(zone)
                explicitly_open.add(zone)

        replacement = re.search(
            r"\b(?:avoid|forbid|block|close)\s+(?:zone\s+)?([A-Z])\s+instead\b|\bkeep\s+(?:zone\s+)?([A-Z])\s+(?:closed|blocked|restricted)\s+instead\b",
            text,
            re.IGNORECASE,
        )
        if replacement and any(cue in text.lower() for cue in ("correction", "instead")):
            hard = {(replacement.group(1) or replacement.group(2)).upper()}
            if "correction" in text.lower():
                policy["exclusions"] = [
                    item for item in policy.get("exclusions", [])
                    if isinstance(item, dict) and item.get("zone") in hard
                ]

        if explicitly_open:
            policy["exclusions"] = [
                item for item in policy.get("exclusions", [])
                if not isinstance(item, dict) or item.get("zone") not in explicitly_open
            ]

        existing = {
            str(item.get("zone", "")).upper()
            for item in policy.get("exclusions", [])
            if isinstance(item, dict) and item.get("zone")
        }
        conditional_zones = {
            str(item.get("zone", "")).upper()
            for item in policy.get("conditional_rules", [])
            if isinstance(item, dict) and item.get("zone")
        }
        hard.difference_update(conditional_zones)
        for zone in sorted(hard - existing):
            policy["exclusions"].append({"zone": zone})
        if hard:
            if "via_zones" in policy:
                policy["via_zones"] = [
                    zone for zone in policy.get("via_zones", []) if zone not in hard
                ]
            policy["weights"] = [item for item in policy.get("weights", []) if item.get("zone") not in hard]
        return policy

    def _detect_explicit_command_conflict(self, user_text: str) -> Optional[str]:
        text = user_text or ""
        lower = text.lower()
        correction_cues = ("actual destination", "instead", "correction", "sorry", "reference point", "rather than")
        event_cues = (
            "if ", "when ", "during ", "unless ",
            "fire alarm", "fire warning", "fire siren", "fire emergency",
            "emergency alarm", "low battery",
        )

        waypoints = unique_preserve([
            f"WP{int(value)}"
            for value in re.findall(
                r"\bWP\s*([0-9]+)\b", text, re.IGNORECASE
            )
            if f"WP{int(value)}" in self._waypoints
        ])
        if len(waypoints) > 1 and not any(cue in lower for cue in correction_cues):
            if re.search(r"\b(?:one|single|only)\s+final\s+destination\b", lower) or re.search(
                r"\b(?:go|proceed|travel|head|move)\b.{0,20}\bWP\s*[0-9]+\b.{0,15}\band\b.{0,15}\bWP\s*[0-9]+\b",
                text,
                re.IGNORECASE,
            ) or re.search(
                r"\bWP\s*[0-9]+\b.{0,15}\band\b.{0,15}\bWP\s*[0-9]+\b.{0,30}\b(?:each|both)\b.{0,20}\b(?:single|only)\b.{0,15}\bfinal destination\b",
                text,
                re.IGNORECASE,
            ):
                return "multiple mutually exclusive final destinations"

        def _state_for_zone(zone_ref: str, states: str, span: int = 24):
            pattern = re.compile(
                rf"\b(?:entering\s+)?{zone_ref}\b(?P<gap>.{{0,{span}}}?)\b(?:{states})\b",
                re.IGNORECASE,
            )
            return next(
                (
                    match for match in pattern.finditer(text)
                    if not re.search(r"\b[A-Z]\b", match.group("gap"))
                ),
                None,
            )

        for zone in self._labels:
            zone_ref = rf"(?:(?:zone|aisle|section|area)\s*)?(?-i:{re.escape(zone)})"
            hard_action = re.search(
                rf"\b(?:forbid|block|close|avoid)\s+{zone_ref}\b|"
                rf"\bkeep\s+{zone_ref}\b.{{0,12}}\b(?:closed|blocked|off[- ]limits|restricted)\b|"
                rf"\b(?:never|do\s+not|don't)\s+(?:enter|use)\s+{zone_ref}\b",
                text,
                re.IGNORECASE,
            )
            hard_state = _state_for_zone(
                zone_ref,
                r"forbidden|blocked|closed|off[- ]limits|restricted|"
                r"not\s+(?:allowed|permitted)|being\s+serviced|"
                r"under\s+maintenance|occupied\s+for\s+loading",
            )
            hard_forbid = hard_action or hard_state
            explicit_route = re.search(
                rf"\b(?:through|via)\s+{zone_ref}\b|"
                rf"\bpass\s+through\s+{zone_ref}\b|"
                rf"\buse\s+{zone_ref}\b.{{0,15}}\b(?:transit|route|corridor|passage)\b|"
                rf"\b{zone_ref}\b.{{0,20}}\b(?:must|required)\b.{{0,15}}\b(?:use|used|route|transit)\b|"
                rf"\b(?:make|making)\s+{zone_ref}\b.{{0,15}}\b(?:part|required|route|transit)\b|"
                rf"\b{zone_ref}\s+(?:route|passage|transit corridor)\b",
                text,
                re.IGNORECASE,
            )
            explicit_denial = _state_for_zone(
                zone_ref,
                r"not\s+(?:allowed|permitted)",
            )
            prefix_open = re.search(
                rf"\b(?:open|allow)\s+{zone_ref}\b",
                text,
                re.IGNORECASE,
            )
            state_open = _state_for_zone(zone_ref, r"open|available|allowed")
            explicit_open = prefix_open or state_open
            explicit_use = explicit_route or (explicit_open and not explicit_denial)
            negated_forbid = re.search(
                rf"\b(?:do\s+not|don't|not)\s+(?:avoid|forbid|block|close)\s+{zone_ref}\b|"
                rf"\b{zone_ref}\b\s+(?:is|was|should\s+be)\s+not\s+(?:forbidden|blocked|closed|restricted)\b",
                text,
                re.IGNORECASE,
            )
            temporal_transition = (
                any(term in lower for term in (
                    "yesterday", "previously", "formerly", "previous shift", "earlier",
                ))
                and any(term in lower for term in ("today", "now", "currently"))
                and any(term in lower for term in ("open", "available", "allowed"))
            )
            if hard_forbid and explicit_use and not negated_forbid and not temporal_transition and not any(cue in lower for cue in correction_cues + event_cues):
                return f"zone {zone} is both forbidden and required for traversal"

        person_terms = ("person", "people", "pedestrian", "worker", "staff", "personnel", "operator", "crew", "human")
        avoid_terms = (
            "avoid", "keep away", "distance", "clearance", "buffer", "separation",
            "safety margin", "wide berth", "metre from", "metres from", "meter from", "meters from",
        )
        has_person = any(term in lower for term in person_terms)
        if has_person:
            minimum = re.search(
                r"\b(?:at\s+least|minimum(?:\s+of)?)\s*(\d+(?:\.\d+)?)\s*(?:m|metres?|meters?)\b",
                lower,
            )
            maximum = re.search(
                r"\b(?:within|at\s+most|no\s+more\s+than|closer\s+than)\s*(\d+(?:\.\d+)?)\s*(?:m|metres?|meters?)\b",
                lower,
            )
            if minimum and maximum and float(maximum.group(1)) < float(minimum.group(1)):
                return (
                    "minimum person separation conflicts with a smaller "
                    "maximum proximity requirement"
                )
        if has_person and any(term in lower for term in avoid_terms):
            if re.search(r"\b(?:within|inside)\s*0\.1(?:0+)?\s*(?:m|metres?|meters?)\b", lower):
                return "person avoidance conflicts with a 0.1 m proximity requirement"

        if re.search(r"\b(?:forbid|block|close)\b", lower) and re.search(
            r"\b(?:remove|lift|cancel)\b.{0,20}\b(?:restriction|keepout|block)\b.{0,10}\bnow\b",
            lower,
        ) and not any(cue in lower for cue in correction_cues):
            return "the same restriction is introduced and removed without ordering"
        return None

    def _commit_policy_snapshot(self, update: Dict[str, Any]) -> Dict[str, Any]:
        self._validate_execution_constraints(update)
        committed = copy.deepcopy(update)
        self._active_policy_state = committed
        self._policy_version = int(getattr(self, "_policy_version", 0)) + 1
        self._last_policy_decision = f"committed version {self._policy_version}"
        return copy.deepcopy(committed)

    def _prepare_policy(self, text: str) -> dict:
        text = self._apply_aisle_aliases(text)
        text = self._canonicalize_named_entities(text)

        if not (self._enable_llm and self._llm_endpoint):
            self._ui_print("ℹ️ LLM disabled → using fallback parser", "gray")
            raw_policy = self._fallback_parse(text)
            self._last_raw_policy = copy.deepcopy(raw_policy)
            return self._validate_policy_obj(raw_policy, text)

        try:
            raw_policy = self._call_llm(text)
            self._last_raw_policy = copy.deepcopy(raw_policy)
            return self._validate_policy_obj(raw_policy, text)
        except CommandInterpretationCanceled:
            raise
        except Exception as e:
            if isinstance(e, PolicyUpdateRejected):
                raise
            raise PolicyUpdateRejected(f"LLM call or JSON parsing failed: {e}") from e

    def _plan_with_llm_or_fallback(self, text: str) -> dict:
        return self._commit_policy_snapshot(self._prepare_policy(text))

    def _call_llm(self, text: str) -> dict:
        return self._call_llm_raw(text)

    def _call_llm_raw(self, text: str) -> dict:
        prompt = self._build_planner_prompt(text)

        options = {
            "temperature": 0.0,
            "top_p": 0.0,
            "seed": 42,
            "repeat_penalty": 1.0,
            "num_predict": 512,
        }

        payload = {
            "model": self._llm_model,
            "prompt": prompt,
            "stream": False,
            "format": "json",
            "think": self._llm_think,
            "options": options,
        }

        if self._interpretation_cancel_requested.is_set():
            raise CommandInterpretationCanceled()

        data = json.dumps(payload).encode("utf-8")
        result_queue: queue.Queue[Tuple[bool, Any]] = queue.Queue(maxsize=1)

        def request_worker() -> None:
            try:
                body = self._post_llm_request(data)
                result_queue.put((True, body))
            except BaseException as exc:
                result_queue.put((False, exc))

        threading.Thread(
            target=request_worker,
            name="policy-bridge-llm-http",
            daemon=True,
        ).start()

        while True:
            if self._interpretation_cancel_requested.is_set():
                self._close_active_llm_connection()
                raise CommandInterpretationCanceled()
            try:
                succeeded, value = result_queue.get(timeout=0.05)
                break
            except queue.Empty:
                continue

        if not succeeded:
            if self._interpretation_cancel_requested.is_set():
                raise CommandInterpretationCanceled() from value
            raise value
        body = str(value)

        if self._interpretation_cancel_requested.is_set():
            raise CommandInterpretationCanceled()

        return self._parse_llm_response(body)

    def _post_llm_request(self, data: bytes) -> str:

        endpoint = urllib.parse.urlsplit(self._llm_endpoint)
        if endpoint.scheme not in {"http", "https"} or not endpoint.hostname:
            raise PolicyUpdateRejected("LLM endpoint must be an HTTP(S) URL")

        connection_type = (
            http.client.HTTPSConnection
            if endpoint.scheme == "https"
            else http.client.HTTPConnection
        )
        connection = connection_type(
            endpoint.hostname,
            endpoint.port,
            timeout=float(getattr(self, "_llm_timeout_s", 120.0)),
        )
        path = endpoint.path or "/"
        if endpoint.query:
            path = f"{path}?{endpoint.query}"
        headers = {"Content-Type": "application/json"}
        if self._llm_key:
            headers["Authorization"] = f"Bearer {self._llm_key}"

        with self._llm_request_lock:
            self._active_llm_connections.add(connection)
        try:
            connection.request("POST", path, body=data, headers=headers)
            response = connection.getresponse()
            body = response.read().decode("utf-8")
            if response.status >= 400:
                raise PolicyUpdateRejected(
                    f"LLM endpoint returned HTTP {response.status}"
                )
        finally:
            with self._llm_request_lock:
                self._active_llm_connections.discard(connection)
            connection.close()
        return body

    def _parse_llm_response(self, body: str) -> dict:

        try:
            obj = json.loads(body)
        except json.JSONDecodeError as exc:
            raise PolicyUpdateRejected("LLM endpoint returned malformed JSON") from exc

        if isinstance(obj, dict) and "response" in obj and isinstance(obj["response"], str):
            txt = obj["response"]
            try:
                obj = json.loads(txt)
            except json.JSONDecodeError as exc:
                raise PolicyUpdateRejected("LLM response field is not one JSON object") from exc

        if not isinstance(obj, dict):
            raise PolicyUpdateRejected("LLM output must be one JSON object")
        self._last_llm_metadata = {
            key: value
            for key, value in json.loads(body).items()
            if key in {
                "model", "created_at", "done", "done_reason", "total_duration",
                "load_duration", "prompt_eval_count", "prompt_eval_duration",
                "eval_count", "eval_duration",
            }
        }
        return obj

    def _build_planner_prompt(self, text: str) -> str:
        labels = ", ".join(sorted(self._labels))
        via_labels = ", ".join(sorted(self._zone_db))
        waypoint_names = sorted(self._waypoints, key=waypoint_sort_key)
        waypoint_labels = ", ".join(waypoint_names)
        waypoint_schema = (
            f'{{"dest": "{waypoint_names[0]}"}} | '
            if waypoint_names
            else ""
        )
        return (
            "You are a deterministic semantic parser for a robot policy interface.\n"
            f"Valid ZONE labels: [{labels}].\n"
            f"Zones with validated transit waypoints: [{via_labels}].\n"
            f"Valid waypoint destinations: [{waypoint_labels}].\n"
            "Return ONLY a single JSON object (no markdown, no code fences).\n"
            "Schema: {\n"
            " \"decision\": \"accept|reject\",\n"
            " \"reason_code\": \"none|contradiction|unsupported\",\n"
            f" \"plan\": [ {waypoint_schema}{{\"dest\":{{\"x\":<num>,\"y\":<num>,\"yaw\":<num>}}}} , ... ],\n"
            " \"via_zones\": [<ZONE>...],\n"
            " \"exclusions\": [ {\"zone\":<ZONE>, \"duration_minutes\":<number, optional>} ... ],\n"
            " \"weights\": [ {\"zone\":<ZONE>,\"cost\":<0..252>} ... ],\n"
            " \"dynamic_object_rules\": [ {\"class\":<string>, \"radius\":<m>} ... ],\n"
            " \"conditional_rules\": [ {\"zone\":<ZONE>, \"rules\":[\n"
            "    {\"priority\":<int>, \"state_condition\":\"fire_alarm|low_battery\", \"action\":\"forbid|allow|allow_shortest\"}, ...\n"
            " ]} ... ]\n"
            "}\n"
            "Semantic rules:\n"
            "- Use decision reject with reason_code contradiction when requirements cannot all be true at the same time.\n"
            "- A contradiction requires incompatible requirements on the SAME zone, object constraint, or final destination. Closing E and travelling through D is valid.\n"
            "- Reject if the same zone is both forbidden and required for transit, if one final mission has two mutually exclusive final destinations, or if person clearance conflicts with an explicit unsafe proximity.\n"
            "MANDATORY CONTRADICTION AUDIT (run after extracting every requirement and before choosing decision):\n"
            "1. SAME-ZONE ROLE: form the exact forbidden-label set and via/open-label set. Reject only when their intersection contains the identical label. Different zones never conflict merely because one is closed and the other is used.\n"
            "2. PERSON DISTANCE: reject if avoidance or a minimum separation is combined with a closer maximum distance that violates it.\n"
            "3. FINAL GOAL: reject only if two different waypoints are both required as the same single final destination. Multiple waypoints ordered by then, next, afterward, followed by, or equivalent sequence cues are a valid route plan; preserve their order in plan and treat the last item as the final destination.\n"
            "4. SIMULTANEOUS STATE: reject if the same zone must be open and closed at the same time; a clearly ordered past-to-current transition or event exception is valid.\n"
            "5. EVENT SCOPE: a leading if/when/while/during condition applies to every coordinated requirement in its clause unless an explicit cue such as normally, usually, or by default establishes a separate base state. Never move an event-conditioned prohibition into the base policy. If the identical zone is forbidden and allowed, opened, or required for transit under the identical event, reject as contradiction.\n"
            "If any audit item is true, decision MUST be reject with reason_code contradiction and every policy array MUST be empty. This audit overrides otherwise correct field extraction.\n"
            "- For reject, return empty policy arrays. Otherwise use decision accept and reason_code none.\n"
            "- Do not invent labels/coords.\n"
            "- A mentioned waypoint is not automatically a destination. Ignore negated, reference-only, or corrected-away waypoints.\n"
            "- Multiple waypoint destinations are valid when the command gives an execution order. Sequence cues such as first, then, and next mean separate plan steps, not mutually exclusive final destinations.\n"
            "- Phrases that explicitly cancel or replace the current/previous/last mission or command are execution-control metadata handled outside the policy JSON. Extract the new navigation request that follows them; do not reject solely because the cancel phrase is present.\n"
            "- For corrections using 'actual destination', 'instead', 'correction', or 'sorry', the final corrected instruction wins.\n"
            "- Do not output an exclusion for a zone explicitly described as open, allowed, or corrected away.\n"
            f"- Only the valid zone labels and waypoint destinations listed above may be used. Valid named destinations: [{waypoint_labels}].\n"
            "- ROLE CONTRAST: through/via/cross/part of the route means via_zones; forbidden/closed/off-limits means exclusions; an explicit numeric cost means weights.\n"
            "- A zone cannot appear in both via_zones and exclusions.\n"
            "- Base policy and event overrides have separate owners: normal keepouts go in exclusions; conditional_rules contains event-triggered changes only.\n"
            "- A base keepout plus an event allow is valid only when the command explicitly separates the base state using wording such as normally, usually, or by default. Do not infer an unstated base state merely to make an event command executable.\n"
            "- Soft costs are ints 0..252; values 253..255 are reserved by Nav2 and are not soft costs. Keepout must be expressed via exclusions.\n"
            "- Keepout/no-go/blocked/closed/unavailable/being-serviced/under-maintenance/occupied-for-loading/do-not-enter/route-around/skip/without-using/off-limits zones must be represented in exclusions, not as weights with cost 255.\n"
            "- weights is only for an EXPLICIT NUMBER stated by the user, such as cost, penalty, score, value, or rating.\n"
            "- If two zones are compared, lower values mean preferred/cheaper and higher values mean discouraged/expensive.\n"
            "- Qualitative-only cost or route-preference wording is unsupported unless the user supplies an explicit numeric cost; never invent a numeric mapping.\n"
            "- If the user gives no explicit numeric cost, weights MUST be empty.\n"
            "- If alternative destinations, zones, or cost levels are offered without a unique selection or assignment, return reject/unsupported and leave every policy array empty.\n"
            "- If a zone is forbidden, do not also include it in weights.\n"
            "- For 'for the next N minutes' or an equivalent duration, put N in exclusions[].duration_minutes. Do not invent ISO timestamps.\n"
            "- Treat people, humans, pedestrians, workers, staff, personnel, operators, crew, and anyone nearby as class \"person\".\n"
            "- Any request for a buffer, separation, clearance, distance, safety margin, wide berth, avoiding close passes, not approaching, or not coming near a person means dynamic_object_rules class \"person\".\n"
            "- If no distance is stated, use radius 1.5. If an explicit distance is stated, use that numeric radius.\n"
            "- exclusions may contain only map zones explicitly named as forbidden or unavailable in the command.\n"
            "- Never place object classes such as person, worker, or forklift in exclusions; represent them only in dynamic_object_rules.\n"
            "- When a command jointly asks to avoid a zone and people, map the named zone to exclusions and people to dynamic_object_rules independently. Never add unmentioned zones.\n"
            "- Treat fire alarm, emergency alarm, fire siren, fire warning, and fire reported as state_condition \"fire_alarm\".\n"
            "- Treat low battery, battery runs low, and battery becomes low as state_condition \"low_battery\".\n"
            "- If a normally forbidden zone may be opened during a fire, put that zone once in exclusions and add only the fire_alarm action \"allow_shortest\" to conditional_rules.\n"
            "- Only output dynamic_object_rules if the user explicitly asks to avoid/detect objects (e.g., 'avoid people'). Otherwise return [].\n"
            "- Never emit a default conditional rule. The base/default state is represented by exclusions; event overrides use priority 10.\n"
            "- Before returning JSON, audit every conditional zone: its normal forbid must be in exclusions exactly once, and conditional_rules must contain only event conditions.\n"
            "- Resolve a pronoun such as 'it', 'that zone', or 'that aisle' only when exactly one compatible antecedent exists. Otherwise return reject/unsupported.\n"
            "- Do not invent zones. (Note: aisle aliases like 'first aisle' are pre-mapped to A..E before you see the text.)\n"
            "Examples are development demonstrations only; a missing optional array means an empty array.\n"
            "Example input: Please send the robot to WP6 and keep it out of Zone B for this trip.\n"
            "Example output: {\"decision\":\"accept\",\"reason_code\":\"none\",\"plan\":[{\"dest\":\"WP6\"}],\"via_zones\":[],\"exclusions\":[{\"zone\":\"B\"}],\"weights\":[],\"dynamic_object_rules\":[],\"conditional_rules\":[]}\n"
            "Example input: For this job, Zone C has cost 180, Zone E has cost 20, and the robot finishes at WP4.\n"
            "Example output: {\"decision\":\"accept\",\"reason_code\":\"none\",\"plan\":[{\"dest\":\"WP4\"}],\"via_zones\":[],\"exclusions\":[],\"weights\":[{\"zone\":\"C\",\"cost\":180},{\"zone\":\"E\",\"cost\":20}],\"dynamic_object_rules\":[],\"conditional_rules\":[]}\n"
            "Example input: Have the robot use aisle B before it reaches WP5.\n"
            "Example output: {\"decision\":\"accept\",\"reason_code\":\"none\",\"plan\":[{\"dest\":\"WP5\"}],\"via_zones\":[\"B\"],\"exclusions\":[],\"weights\":[],\"dynamic_object_rules\":[],\"conditional_rules\":[]}\n"
            "Example input: On the trip to WP6, travel along Zone C and stay out of Zone A.\n"
            "Example output: {\"decision\":\"accept\",\"reason_code\":\"none\",\"plan\":[{\"dest\":\"WP6\"}],\"via_zones\":[\"C\"],\"exclusions\":[{\"zone\":\"A\"}],\"weights\":[],\"dynamic_object_rules\":[],\"conditional_rules\":[]}\n"
            "Example input: Keep two metres away from workers on the way to WP4.\n"
            "Example output: {\"decision\":\"accept\",\"reason_code\":\"none\",\"plan\":[{\"dest\":\"WP4\"}],\"via_zones\":[],\"exclusions\":[],\"weights\":[],\"dynamic_object_rules\":[{\"class\":\"person\",\"radius\":2.0}],\"conditional_rules\":[]}\n"
            "Example input: Send the robot to WP5 while staying clear of both Zone B and nearby personnel.\n"
            "Example output: {\"decision\":\"accept\",\"reason_code\":\"none\",\"plan\":[{\"dest\":\"WP5\"}],\"via_zones\":[],\"exclusions\":[{\"zone\":\"B\"}],\"weights\":[],\"dynamic_object_rules\":[{\"class\":\"person\",\"radius\":1.5}],\"conditional_rules\":[]}\n"
            "Example input: Zone C should normally stay closed. If the fire alarm sounds, the robot may use C on the shortest route to WP6.\n"
            "Example output: {\"decision\":\"accept\",\"reason_code\":\"none\",\"plan\":[{\"dest\":\"WP6\"}],\"via_zones\":[],\"exclusions\":[{\"zone\":\"C\"}],\"weights\":[],\"dynamic_object_rules\":[],\"conditional_rules\":[{\"zone\":\"C\",\"rules\":[{\"priority\":10,\"state_condition\":\"fire_alarm\",\"action\":\"allow_shortest\"}]}]}\n"
            "Example input: For the next 12 minutes, do not use Zone D. The destination is WP1.\n"
            "Example output: {\"decision\":\"accept\",\"reason_code\":\"none\",\"plan\":[{\"dest\":\"WP1\"}],\"via_zones\":[],\"exclusions\":[{\"zone\":\"D\",\"duration_minutes\":12}],\"weights\":[],\"dynamic_object_rules\":[],\"conditional_rules\":[]}\n"
            "Example input: Stop at WP3, continue to WP6, and end the run at WP8.\n"
            "Example output: {\"decision\":\"accept\",\"reason_code\":\"none\",\"plan\":[{\"dest\":\"WP3\"},{\"dest\":\"WP6\"},{\"dest\":\"WP8\"}],\"via_zones\":[],\"exclusions\":[],\"weights\":[],\"dynamic_object_rules\":[],\"conditional_rules\":[]}\n"
            "Example input: Cancel the mission that is running, then start a new trip to WP4 with Zone E closed.\n"
            "Example output: {\"decision\":\"accept\",\"reason_code\":\"none\",\"plan\":[{\"dest\":\"WP4\"}],\"via_zones\":[],\"exclusions\":[{\"zone\":\"E\"}],\"weights\":[],\"dynamic_object_rules\":[],\"conditional_rules\":[]}\n"
            "Example input: Sorry, WP3 was the wrong destination. The robot should go to WP6 instead.\n"
            "Example output: {\"decision\":\"accept\",\"reason_code\":\"none\",\"plan\":[{\"dest\":\"WP6\"}],\"via_zones\":[],\"exclusions\":[],\"weights\":[],\"dynamic_object_rules\":[],\"conditional_rules\":[]}\n"
            "Example input: Keep Zone A closed for this trip, but make the route to WP5 go through A.\n"
            "Example output: {\"decision\":\"reject\",\"reason_code\":\"contradiction\",\"plan\":[],\"via_zones\":[],\"exclusions\":[],\"weights\":[],\"dynamic_object_rules\":[],\"conditional_rules\":[]}\n"
            "Example input: On the way to WP7, stay two metres from workers but also pass within half a metre of them.\n"
            "Example output: {\"decision\":\"reject\",\"reason_code\":\"contradiction\",\"plan\":[],\"via_zones\":[],\"exclusions\":[],\"weights\":[],\"dynamic_object_rules\":[],\"conditional_rules\":[]}\n"
            "Example input: Make WP2 the final stop, not a waypoint. WP5 must also be the final stop for the same trip.\n"
            "Example output: {\"decision\":\"reject\",\"reason_code\":\"contradiction\",\"plan\":[],\"via_zones\":[],\"exclusions\":[],\"weights\":[],\"dynamic_object_rules\":[],\"conditional_rules\":[]}\n"
            "Example input: When the battery is low, keep Zone B closed. At the same time, the route to WP4 must use B.\n"
            "Example output: {\"decision\":\"reject\",\"reason_code\":\"contradiction\",\"plan\":[],\"via_zones\":[],\"exclusions\":[],\"weights\":[],\"dynamic_object_rules\":[],\"conditional_rules\":[]}\n"
            "Now convert only the final command below. Return one JSON object only.\n"
            "FINAL_USER_COMMAND: " + text + "\n"
        )

    def _canonicalize_plan_obj(self, obj: dict, user_text: str) -> dict:
        def _finite_num(x: Any) -> Optional[float]:
            try:
                value = float(x)
            except (TypeError, ValueError):
                return None
            return value if math.isfinite(value) else None

        if not isinstance(obj, dict):
            obj = {}

        v3_requested = any(
            field in obj for field in ("decision", "reason_code", "via_zones")
        )

        plan = obj.get("plan") if isinstance(obj.get("plan"), list) else []
        via_zones = obj.get("via_zones") if isinstance(obj.get("via_zones"), list) else []
        exclusions = obj.get("exclusions") if isinstance(obj.get("exclusions"), list) else []
        weights = obj.get("weights") if isinstance(obj.get("weights"), list) else []
        cond_in = obj.get("conditional_rules") if isinstance(obj.get("conditional_rules"), list) else []

        labels_set = set(self._labels)
        text_raw = user_text or ""
        text_lower = text_raw.lower()

        grounded_zones = {
            z.upper()
            for z in re.findall(
                r"\b(?:zone|aisle)\s*([A-Z])\b",
                text_raw,
                flags=re.IGNORECASE,
            )
        }
        grounded_zones.update(re.findall(r"\b([A-Z])\b", text_raw))

        def _lab(x):
            try:
                s = str(x).strip().upper()
            except Exception:
                return None
            return s if s in labels_set else None

        def _grounded_lab(x):
            label = _lab(x)
            return label if label in grounded_zones else None

        norm_via = []
        for item in via_zones:
            zone = _grounded_lab(item.get("zone") if isinstance(item, dict) else item)
            if zone and zone in self._zone_db and zone not in norm_via:
                norm_via.append(zone)


        explicit_wp_names = {
            f"WP{int(n)}"
            for n in re.findall(
                r"\bWP\s*([0-9]+)\b",
                user_text or "",
                flags=re.IGNORECASE,
            )
            if f"WP{int(n)}" in self._waypoints
        }
        requested_coordinates: List[tuple[float, float, Optional[float]]] = []
        x_match = re.search(r"\bx\s*[=:]\s*([-+]?\d+(?:\.\d+)?)", text_raw, re.IGNORECASE)
        y_match = re.search(r"\by\s*[=:]\s*([-+]?\d+(?:\.\d+)?)", text_raw, re.IGNORECASE)
        yaw_match = re.search(r"\byaw\s*[=:]\s*([-+]?\d+(?:\.\d+)?)", text_raw, re.IGNORECASE)
        if x_match and y_match:
            requested_coordinates.append((
                float(x_match.group(1)),
                float(y_match.group(1)),
                float(yaw_match.group(1)) if yaw_match else None,
            ))
        for match in re.finditer(
            r"([-+]?\d+(?:\.\d+)?)\s*,\s*([-+]?\d+(?:\.\d+)?)",
            text_raw,
        ):
            requested_coordinates.append((float(match.group(1)), float(match.group(2)), None))

        norm_plan = []
        for it in (plan or []):
            if not isinstance(it, dict):
                continue
            d = it.get("dest")
            if isinstance(d, str):
                match = re.fullmatch(
                    r"WP\s*([0-9]+)", str(d).strip(), re.IGNORECASE
                )
                key = f"WP{int(match.group(1))}" if match else ""
                if hasattr(self, "_waypoints") and key in self._waypoints and key in explicit_wp_names:
                    norm_plan.append({"dest": key})
                else:
                    lab = _lab(d)
                    if lab:
                        pass
            elif isinstance(d, dict) and "x" in d and "y" in d:
                x = _finite_num(d.get("x"))
                y = _finite_num(d.get("y"))
                yaw = _finite_num(d.get("yaw", 0.0))
                if x is None or y is None or yaw is None:
                    continue
                grounded = any(
                    math.isclose(x, req_x, abs_tol=1e-6)
                    and math.isclose(y, req_y, abs_tol=1e-6)
                    and (req_yaw is None or math.isclose(yaw, req_yaw, abs_tol=1e-6))
                    for req_x, req_y, req_yaw in requested_coordinates
                )
                if grounded:
                    norm_plan.append({"dest": {"x": x, "y": y, "yaw": yaw}})


        raw_excl = exclusions
        norm_excl_rules: List[Dict[str, Any]] = []
        _seen = set()

        def _parse_iso(s: Any) -> Optional[dt.datetime]:
            if not isinstance(s, str):
                return None
            try:
                d = dt.datetime.fromisoformat(s)
            except Exception:
                return None
            if d.tzinfo is None:
                d = d.replace(tzinfo=self._tz)
            return d

        def _norm_time_condition(tc: Any) -> Optional[Dict[str, Any]]:
            if tc in (None, "", {}):
                return None
            if not isinstance(tc, dict):
                return None

            st = _parse_iso(tc.get("start"))
            if st is None:
                return None

            en = _parse_iso(tc.get("end")) if tc.get("end") else None
            if en is None:
                en = st + dt.timedelta(minutes=self._default_forbid_minutes)

            if en <= st:
                return None

            rep = tc.get("repeat")
            rep = rep if rep in (None, "daily") else None
            return {"start": st.isoformat(), "end": en.isoformat(), "repeat": rep}

        def _duration_from_text(value: str) -> Optional[float]:
            lower = (value or "").lower()
            match = re.search(r"\b(\d+(?:\.\d+)?)\s*(minutes?|mins?|min)\b", lower)
            if match:
                return float(match.group(1))
            match = re.search(r"\b(\d+(?:\.\d+)?)\s*(hours?|hrs?|hr)\b", lower)
            if match:
                return float(match.group(1)) * 60.0
            if re.search(r"\b(?:half\s+(?:an?\s+)?hour|half-hour|thirty\s+minutes?)\b", lower):
                return 30.0
            return None

        requested_duration = _duration_from_text(text_raw)

        def _norm_duration(value: Any) -> Optional[Dict[str, Any]]:
            duration = _finite_num(value)
            if duration is None or requested_duration is None:
                return None
            if not math.isclose(duration, requested_duration, abs_tol=1e-6):
                return None
            if not 1.0 <= duration <= 1440.0:
                return None
            start = dt.datetime.now(self._tz)
            end = start + dt.timedelta(minutes=duration)
            return {"start": start.isoformat(), "end": end.isoformat(), "repeat": None}

        if isinstance(raw_excl, list):
            for e in raw_excl:
                rule = None
                if isinstance(e, str):
                    z = _grounded_lab(e)
                    if z:
                        rule = {"zone": z}
                elif isinstance(e, dict):
                    z = _grounded_lab(e.get("zone"))
                    if z:
                        rule = {"zone": z}
                        tc_norm = _norm_duration(e.get("duration_minutes"))
                        if tc_norm is None:
                            tc_norm = _norm_time_condition(e.get("time_condition"))
                        if tc_norm:
                            rule["time_condition"] = tc_norm

                if rule:
                    key = json.dumps(rule, sort_keys=True)
                    if key not in _seen:
                        norm_excl_rules.append(rule)
                        _seen.add(key)


        text_without_wps = re.sub(
            r"\bWP\s*[0-9]+\b", " ", text_raw, flags=re.IGNORECASE
        )
        has_numeric_policy_cue = bool(re.search(
            r"\b(?:set|cost|penalty|score|value|rating)\b",
            text_without_wps,
            re.IGNORECASE,
        ))
        explicitly_numbered_zones = set()
        for zone in grounded_zones:
            zone_ref = rf"(?:(?:zone|aisle|section|area)\s*)?(?-i:{re.escape(zone)})\b"
            local_numeric = re.search(
                rf"{zone_ref}.{{0,35}}\b(?:cost|penalty|score|value|rating|at)\b\s*(?:of|to|at|is|=)?\s*[-+]?\d",
                text_without_wps,
                re.IGNORECASE,
            ) or re.search(
                rf"\b(?:cost|penalty|score|value|rating)\b.{{0,20}}{zone_ref}.{{0,12}}[-+]?\d",
                text_without_wps,
                re.IGNORECASE,
            ) or re.search(
                rf"\b(?:cost|penalty|score|value|rating)\b\s*(?:of|at|=)?\s*[-+]?\d+(?:\.\d+)?\s*(?:on|for|to)\s*{zone_ref}",
                text_without_wps,
                re.IGNORECASE,
            ) or re.search(
                rf"{zone_ref}.{{0,25}}[-+]?\d+(?:\.\d+)?\s*\b(?:cost|penalty|score|value|rating)\b",
                text_without_wps,
                re.IGNORECASE,
            ) or re.search(
                rf"[-+]?\d+(?:\.\d+)?\s+for\s+{zone_ref}",
                text_without_wps,
                re.IGNORECASE,
            )
            slot_numeric = has_numeric_policy_cue and re.search(
                rf"{zone_ref}.{{0,12}}\b(?:to|at|is|=)\s*[-+]?\d",
                text_without_wps,
                re.IGNORECASE,
            )
            if local_numeric or slot_numeric:
                explicitly_numbered_zones.add(zone)

        irrelevant_zones = {
            zone for zone in grounded_zones
            if re.search(
                rf"\b(?:ignore|disregard)\s+(?:(?:zone|aisle|section|area)\s*)?(?-i:{re.escape(zone)})\b",
                text_raw,
                re.IGNORECASE,
            )
        }

        def _has_explicit_hard_intent(zone: str) -> bool:
            zone_ref = rf"(?:(?:zone|aisle|section|area)\s*)?(?-i:{re.escape(zone)})"
            return bool(re.search(
                rf"\b(?:forbid|block|close|avoid)\s+{zone_ref}\b|"
                rf"\bkeep\s+{zone_ref}\b.{{0,12}}\b(?:closed|blocked|restricted|off[- ]limits)\b|"
                rf"\b(?:never|do\s+not|don't)\s+(?:enter|use)\s+{zone_ref}\b|"
                rf"\b{zone_ref}\b.{{0,24}}\b(?:forbidden|blocked|closed|restricted|off[- ]limits|not\s+(?:allowed|permitted)|being\s+serviced|under\s+maintenance|occupied\s+for\s+loading)\b",
                text_raw,
                re.IGNORECASE,
            ))

        soft_only_zones = {
            zone for zone in explicitly_numbered_zones
            if not _has_explicit_hard_intent(zone)
        }
        non_exclusion_zones = irrelevant_zones | soft_only_zones
        if non_exclusion_zones:
            norm_via = [zone for zone in norm_via if zone not in irrelevant_zones]
            norm_excl_rules = [
                item for item in norm_excl_rules
                if item.get("zone") not in non_exclusion_zones
            ]

        latest = {}
        for w in (weights or []):
            if not isinstance(w, dict):
                continue
            z = _grounded_lab(w.get("zone"))
            if not z or z not in explicitly_numbered_zones or z in irrelevant_zones:
                continue
            c = normalize_soft_cost(w.get("cost"))
            if c is None:
                continue
            previous_cost = latest.get(z, {}).get("cost", -1)
            latest[z] = {"zone": z, "cost": max(previous_cost, int(c))}
        _det_keepouts, explicit_soft_costs = self._split_intents_deterministic(text_raw)
        for zone, cost in explicit_soft_costs.items():
            if zone in grounded_zones and zone not in irrelevant_zones:
                previous_cost = latest.get(zone, {}).get("cost", -1)
                latest[zone] = {"zone": zone, "cost": max(previous_cost, int(cost))}
        norm_weights = [latest[k] for k in sorted(latest.keys())]

        excl_zones_for_weights = {
            r["zone"] for r in norm_excl_rules
            if isinstance(r, dict) and "zone" in r
        }
        norm_weights = [w for w in norm_weights if w["zone"] not in excl_zones_for_weights]


        raw_dyn = obj.get("dynamic_object_rules")
        norm_dyn: List[Dict[str, Any]] = []
        ALLOWED_CLASSES = {"person", "forklift"}
        person_grounded = any(
            term in text_lower
            for term in (
                "person", "people", "human", "pedestrian", "worker", "staff", "personnel",
                "operator", "crew", "anyone",
            )
        )
        forklift_grounded = "forklift" in text_lower
        object_avoidance_requested = any(
            term in text_lower
            for term in (
                "avoid", "keep away", "stay away", "clearance", "distance", "not get close",
                "away from", "don't get close", "do not get close", "getting close", "not crowd", "without crowding", "stay clear",
                "safe gap", "give space", "leaving space", "give room", "plenty of room",
                "separation", "buffer", "safety margin", "wide berth", "close pass", "approach",
                "coming near", "careful around", "don't brush past", "do not brush past", "gap",
                "metre from", "metres from", "meter from", "meters from", "breathing room",
                "closer than", "squeez", "between the robot",
            )
        ) or bool(re.search(
            r"\b(?:leave|leaving|allow|allowing)\b.{0,25}\b(?:space|room)\b|"
            r"\b(?:well|safely)\s+clear\b|\bprotective\s+radius\b",
            text_lower,
        ))

        if isinstance(raw_dyn, list):
            for r in raw_dyn:
                if not isinstance(r, dict):
                    continue
                cls = r.get("class")
                rad = r.get("radius")
                if not isinstance(cls, str):
                    continue
                cls_n = cls.strip().lower()
                cls_n = {
                    "people": "person",
                    "human": "person",
                    "humans": "person",
                    "pedestrian": "person",
                    "pedestrians": "person",
                }.get(cls_n, cls_n)
                if cls_n not in ALLOWED_CLASSES:
                    continue
                if cls_n == "person" and not (person_grounded and object_avoidance_requested):
                    continue
                if cls_n == "forklift" and not (forklift_grounded and object_avoidance_requested):
                    continue
                try:
                    rad_f = float(rad)
                except Exception:
                    continue
                explicit_person_distance = bool(re.search(
                    r"\b\d+(?:\.\d+)?[\s-]*(?:m|metres?|meters?)\b|"
                    r"\b(?:one|two|three|four|five)(?:\s+and\s+a\s+half|-and-a-half)?\s+(?:metres?|meters?)\b|"
                    r"\bhalf\s+a\s+(?:metre|meter)\b",
                    text_lower,
                ))
                if cls_n == "person" and not explicit_person_distance:
                    rad_f = 1.5
                rad_f = max(0.1, min(10.0, rad_f))
                norm_dyn.append({"class": cls_n, "radius": rad_f})


        norm_cond: Dict[str, List[Dict[str, Any]]] = {}
        ALLOWED_CONDS = {"default", "fire_alarm", "low_battery"}
        ALLOWED_ACTS = {"forbid", "allow", "allow_shortest"}
        fire_grounded = bool(
            re.search(r"\b(?:fire|emergency)\b.{0,20}\b(?:alarm|siren|warning|reported)\b", text_lower)
            or re.search(r"\bfire\s+emergency\b", text_lower)
        )
        battery_grounded = "battery" in text_lower
        conditional_language = any(k.lower() in text_lower for k in COND_KWS)
        known_event_grounded = fire_grounded or battery_grounded
        shortest_grounded = any(
            term in text_lower
            for term in (
                "shortest", "fastest", "quickest", "direct route", "direct way", "direct passage",
                "open it", "override", "lift that rule",
            )
        )

        if isinstance(cond_in, list):
            for item in cond_in:
                if not isinstance(item, dict):
                    continue
                z = _grounded_lab(item.get("zone"))
                if not z:
                    continue
                rules_out: List[Dict[str, Any]] = []
                for rr in (item.get("rules") or []):
                    if not isinstance(rr, dict):
                        continue
                    cond = str(rr.get("state_condition", rr.get("condition", "default"))).strip().lower()
                    if cond not in ALLOWED_CONDS:
                        continue
                    act = str(rr.get("action", "forbid")).strip().lower()
                    if act not in ALLOWED_ACTS:
                        continue
                    if cond == "fire_alarm" and not fire_grounded:
                        continue
                    if cond == "low_battery" and not battery_grounded:
                        continue
                    if act == "allow_shortest" and not (shortest_grounded and known_event_grounded):
                        continue
                    if cond == "default" and act in ("allow", "allow_shortest") and conditional_language and not known_event_grounded:
                        continue
                    try:
                        prio = int(rr.get("priority", 0))
                    except Exception:
                        prio = 0
                    rules_out.append({"priority": prio, "state_condition": cond, "action": act})

                if rules_out:
                    norm_cond.setdefault(z, []).extend(rules_out)

        action_safety_rank = {"forbid": 2, "allow": 1, "allow_shortest": 0}
        for zone, rules in list(norm_cond.items()):
            resolved: Dict[tuple[str, int], Dict[str, Any]] = {}
            for rule in rules:
                key = (rule["state_condition"], rule["priority"])
                previous = resolved.get(key)
                if previous is None or action_safety_rank[rule["action"]] > action_safety_rank[previous["action"]]:
                    resolved[key] = rule
            norm_cond[zone] = sorted(
                resolved.values(),
                key=lambda rule: (-rule["priority"], -action_safety_rank[rule["action"]]),
            )

        conditional_zones = set(norm_cond)
        if conditional_zones:
            norm_via = [zone for zone in norm_via if zone not in conditional_zones]
            norm_excl_rules = [
                item for item in norm_excl_rules
                if item.get("zone") not in conditional_zones
            ]
            norm_weights = [
                item for item in norm_weights
                if item.get("zone") not in conditional_zones
            ]


        weighted_zones = {w["zone"] for w in norm_weights}
        excl_zones = {r["zone"] for r in norm_excl_rules if isinstance(r, dict) and "zone" in r}
        excl_all = excl_zones

        filtered_plan = []
        for it in norm_plan:
            d = it.get("dest")
            if isinstance(d, str) and (d in excl_all or d in weighted_zones):
                continue
            filtered_plan.append(it)

        if not filtered_plan:
            filtered_plan = self._enforce_labels_only_if_no_coords(user_text, [])

        canon_plan = filtered_plan

        out = {
            "plan": canon_plan,
            "exclusions": norm_excl_rules,
            "weights": norm_weights,
            "dynamic_object_rules": norm_dyn,
            "conditional_rules": [
                {"zone": z, "rules": norm_cond[z]}
                for z in sorted(norm_cond.keys())
            ],
        }
        if v3_requested:
            out.update({
                "decision": "accept",
                "reason_code": "none",
                "via_zones": norm_via,
            })
        return out

    def _enforce_labels_only_if_no_coords(self, user_text: str, plan: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        if isinstance(plan, list) and any(isinstance(it.get("dest"), (str, dict)) for it in plan):
            return plan

        chunks = re.split(r"[,.。!?]\s*|\n+", user_text or "")
        uDEST = [k.upper() for k in DEST_KWS]
        uFORBID = [k.upper() for k in EXC_KWS_TMP]
        labels: List[str] = []
        for ch in chunks:
            chu = ch.upper()
            if any(k in chu for k in uFORBID):
                continue
            if any(k in chu for k in uDEST):
                labs = [
                    c.upper()
                    for c in re.findall(r"\b(?:zone|aisle|section|area)\s*([A-Z])\b", ch, re.IGNORECASE)
                    if c.upper() in self._labels
                ]
                labs.extend(c for c in re.findall(r"\b([A-Z])\b", ch) if c in self._labels)
                labels.extend(labs)
        labels = unique_preserve(labels)
        return [{"dest": z} for z in labels] if labels else plan


    def _fallback_parse(self, text: str) -> Dict[str, Any]:
        t = (text or "").strip()
        tu = t.upper()


        weights: List[Dict[str, Any]] = []
        for m in WEIGHT_CMD_RE.finditer(tu):
            z, c = m.group(1), int(m.group(3))
            cost = normalize_soft_cost(c)
            if cost is not None:
                weights.append({"zone": z, "cost": cost})
        for m in WEIGHT_CMD_RE2.finditer(tu):
            z, c = m.group(1), int(m.group(2))
            cost = normalize_soft_cost(c)
            if cost is not None:
                weights.append({"zone": z, "cost": cost})

        weight_zones = {w["zone"] for w in weights}

        chunks = re.split(r"[,.。!?]\s*|\n+", t)
        dest_labels: List[str] = []
        tmp_exclusions: List[str] = []
        tl = t.lower()

        dynamic_object_rules: List[Dict[str, Any]] = []
        person_terms = ("people", "person", "human", "humans", "pedestrian", "pedestrians")
        avoid_terms = ("avoid", "keeping away", "keep away", "clearance", "stay away", "detected")
        if any(p in tl for p in person_terms) and any(a in tl for a in avoid_terms):
            dynamic_object_rules.append({"class": "person", "radius": 1.5})

        def labels_in(s: str) -> List[str]:
            return [c for c in ZONE_LABEL_RE.findall(s.upper()) if c in self._labels]

        uDEST = [k.upper() for k in DEST_KWS]
        uEXC_TMP = [k.upper() for k in EXC_KWS_TMP]
        perm_cost_zones: set[str] = set()

        for ch in chunks:
            chu = ch.upper()
            labs = labels_in(ch)
            if not labs:
                continue

            if _is_perm_cost_sentence(ch):
                perm_cost_zones.update(labs)

            is_exc  = any(kw in chu for kw in uEXC_TMP)
            is_dest = any(kw in chu for kw in uDEST)
            is_cost = "COST" in chu

            if is_exc:
                tmp_exclusions.extend(labs)
            elif is_dest:
                dest_labels.extend(labs)
            elif is_cost:
                continue
            else:
                dest_labels.extend(labs)

        excl_all = set(unique_preserve(tmp_exclusions))
        dest_labels = [z for z in unique_preserve(dest_labels) if z not in excl_all]
        dest_labels = [z for z in dest_labels if z not in weight_zones]

        explicit_wps = unique_preserve(
            [
                f"WP{int(n)}"
                for n in re.findall(
                    r"\bWP\s*([0-9]+)\b", t, flags=re.IGNORECASE
                )
                if f"WP{int(n)}" in self._waypoints
            ]
        )
        plan = [{"dest": wp} for wp in explicit_wps]
        plan.extend({"dest": z} for z in dest_labels)




        return {
            "plan": plan,
            "exclusions": [{"zone": z} for z in unique_preserve(tmp_exclusions)],
            "weights": weights,
            "dynamic_object_rules": dynamic_object_rules,
            "conditional_rules": [],
            "perm_cost_zones": list(perm_cost_zones),
        }


    def _publish_zone_geometry(self) -> int:

        publisher = getattr(self, "zone_geometry_pub", None)
        if publisher is None:
            return 0
        self._zone_geometry_version = int(
            getattr(self, "_zone_geometry_version", 0)
        ) + 1
        message = RosString()
        message.data = json.dumps(
            {
                "frame_id": self._global_frame,
                "version": self._zone_geometry_version,
                "zones": {
                    name: list(bounds.as_tuple())
                    for name, bounds in sorted(self._zone_db.items())
                },
            },
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
        )
        publisher.publish(message)
        subscription_count = getattr(
            publisher, "get_subscription_count", None
        )
        if callable(subscription_count):
            try:
                return int(subscription_count())
            except Exception:
                return 0
        return 0

    def _publish_forbidden(self, zones: List[str]):
        zones = [z for z in zones if z in self._zone_db]
        geometry_subscribers = self._publish_zone_geometry()
        if zones and geometry_subscribers < 2:
            self._ui_print(
                "⚠️ Zone geometry is not connected to both costmap layers "
                f"(subscribers={geometry_subscribers}). Restart Nav2 after "
                "sourcing this workspace.",
                "yellow",
            )
        msg = RosString()
        msg.data = ",".join(zones)
        self.forbidden_pub.publish(msg)
        now = dt.datetime.now(self._tz)
        timed_windows: Dict[str, Tuple[dt.datetime, dt.datetime]] = {}
        with self._sched_lock:
            for sid in self._active_timed.keys():
                tk = self._schedules.get(sid)
                if not tk:
                    continue
                st = tk.start_wall
                en = tk.end_wall or (tk.start_wall + dt.timedelta(minutes=self._default_forbid_minutes))
                for z in tk.zones:
                    if z in self._labels:
                        timed_windows[z] = (st, en)
        default_end = now + dt.timedelta(minutes=self._default_forbid_minutes)
        out: Dict[str, Tuple[dt.datetime, dt.datetime]] = {}
        for z in zones:
            if z in timed_windows:
                out[z] = timed_windows[z]
            else:
                out[z] = (now, default_end)
        self._last_forbidden_windows = out

    def _publish_validated_policy(
        self,
        *,
        source: str,
        command_id: Optional[int],
        user_text: str,
        policy: Dict[str, Any],
    ) -> None:
        publisher = getattr(self, "validated_policy_pub", None)
        if publisher is None:
            return
        message = RosString()
        message.data = json.dumps(
            {
                "source": source,
                "stage": "committed_after_validation_and_merge",
                "command_id": command_id,
                "policy_version": int(getattr(self, "_policy_version", 0)),
                "user_text": user_text,
                "raw_policy": getattr(self, "_last_raw_policy", None),
                "validation_report": getattr(self, "_last_validation_report", {}),
                "llm_metadata": getattr(self, "_last_llm_metadata", {}),
                "policy": policy,
            },
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
        )
        publisher.publish(message)

    @staticmethod
    def _effective_object_avoidance_radius(
        dynamic_rules: List[Dict[str, Any]],
    ) -> float:

        radii: List[float] = []
        for rule in dynamic_rules or []:
            if not isinstance(rule, dict):
                continue
            try:
                radius = float(rule.get("radius"))
            except (TypeError, ValueError, OverflowError):
                continue
            if math.isfinite(radius) and radius > 0.0:
                radii.append(radius)
        return max(radii, default=0.0)

    def _publish_object_avoidance_radius(
        self,
        dynamic_rules: List[Dict[str, Any]],
    ) -> None:
        radius = self._effective_object_avoidance_radius(dynamic_rules)
        publisher = getattr(self, "object_radius_pub", None)
        if publisher is not None:
            message = Float32()
            message.data = float(radius)
            publisher.publish(message)
        self._active_object_avoidance_radius = radius

    def _publish_softcost(self, rules: List[WeightedRule]):
        zone_costs: Dict[str, int] = {}
        for r in rules:
            z = str(r.zone).upper()
            if z in self._zone_db:
                zone_costs[z] = int(r.cost)

        msg = RosString()
        geometry_subscribers = self._publish_zone_geometry()
        if zone_costs and geometry_subscribers < 2:
            self._ui_print(
                "⚠️ Zone geometry is not connected to both costmap layers "
                f"(subscribers={geometry_subscribers}). Restart Nav2 after "
                "sourcing this workspace.",
                "yellow",
            )
        msg.data = json.dumps({"zones": zone_costs}, ensure_ascii=False)
        self.softcost_pub.publish(msg)
        self._active_softcost = dict(zone_costs)

    def _clear_softcost(self):
        msg = RosString()
        msg.data = json.dumps({"zones": {}}, ensure_ascii=False)
        self.softcost_pub.publish(msg)
        self._active_softcost = {}

    def _filter_plan_by_keepouts(self, plan: List[Dict[str, Any]], keepouts: List[str]) -> List[Dict[str, Any]]:
        ko = {k.upper() for k in keepouts}
        filtered: List[Dict[str, Any]] = []
        for item in plan:
            d = item.get("dest")
            if isinstance(d, str):
                if d.upper() in ko:
                    continue
                filtered.append({"dest": d.upper()})
            elif isinstance(d, dict):
                filtered.append(item)
        return filtered


    def _schedule_keepout(self, tk: TimedKeepout) -> int:

        with self._sched_lock:
            sid = self._next_sched_id
            self._next_sched_id += 1
            tk.id = sid


        end_wall = tk.end_wall or (tk.start_wall + dt.timedelta(minutes=self._default_forbid_minutes))


        start_ros = self._wall_dt_to_ros_time(tk.start_wall)
        end_ros = self._wall_dt_to_ros_time(end_wall)
        with self._sched_lock:
            tk.start_ros_ns = int(start_ros.nanoseconds)
            tk.end_ros_ns = int(end_ros.nanoseconds)
            self._schedules[sid] = tk

        def _on_start():
            with self._sched_lock:
                if sid not in self._schedules:
                    return
                self._active_timed[sid] = set([z for z in tk.zones if z in self._labels])

            self._recalculate_and_publish_dynamic_keepouts(
                base_exclusions=self._last_base_exclusions,
                conditional_rules=self._cond_rules_store
            )

        def _on_end():
            with self._sched_lock:
                if sid not in self._schedules:
                    return
                self._active_timed.pop(sid, None)
            self._recalculate_and_publish_dynamic_keepouts(
                base_exclusions=self._last_base_exclusions,
                conditional_rules=self._cond_rules_store
            )

            if tk.repeat == "daily":
                self._schedule_keepout(TimedKeepout(
                    zones=tk.zones,
                    start_wall=tk.start_wall + dt.timedelta(days=1),
                    end_wall=(tk.end_wall + dt.timedelta(days=1)) if tk.end_wall else None,
                    repeat="daily"
                ))


        now_ros = self.get_clock().now()
        delay_start = (start_ros.nanoseconds - now_ros.nanoseconds) / 1e9
        delay_end = (end_ros.nanoseconds - now_ros.nanoseconds) / 1e9


        start_timer = None
        if delay_start <= 0.0:
            _on_start()
        else:
            start_timer = self._oneshot_ros_timer(delay_start, _on_start)


        end_timer = None
        if delay_end <= 0.0:
            _on_end()
        else:
            end_timer = self._oneshot_ros_timer(delay_end, _on_end)

        with self._sched_lock:
            if sid in self._schedules:
                self._timer_threads[sid] = (start_timer, end_timer)

        return sid


    def _ensure_navigator(self):
        if self._nav is not None:
            return
        namespace = self._navigator_namespace.strip("/")
        suffix = re.sub(r"[^A-Za-z0-9_]", "_", namespace) or "root"
        self._nav = BasicNavigator(
            node_name=f"basic_navigator_{suffix}",
            namespace=namespace,
        )
        display = f"/{namespace}" if namespace else "/"
        self._ui_print(
            f"🧭 BasicNavigator ready for namespace {display}.", "cyan"
        )

    def _apply_initial_pose_param_once(self):
        if self._initial_pose_applied or not self._set_initial_pose:
            return
        try:
            pose = self._get_robot_pose(self._global_frame)
            if pose is not None:
                self._ui_print("📍 Skip initial pose: already valid", "gray")
                self._initial_pose_applied = True
                return
        except Exception:
            pass

        try:
            x = float(self._initial_pose.get("x", 0.0))
            y = float(self._initial_pose.get("y", 0.0))
            yaw = float(self._initial_pose.get("yaw", 0.0))
            frame = str(self._initial_pose.get("frame", self._global_frame))

            pose = PoseStamped()
            pose.header.frame_id = frame
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            qx, qy, qz, qw = yaw_to_quat(yaw)
            pose.pose.orientation.x, pose.pose.orientation.y = qx, qy
            pose.pose.orientation.z, pose.pose.orientation.w = qz, qw

            ip = PoseWithCovarianceStamped()
            ip.header.frame_id = frame
            ip.header.stamp = self.get_clock().now().to_msg()
            ip.pose.pose = pose.pose
            cov = [0.0] * 36
            cov[0] = cov[7] = 0.25
            cov[35] = 0.0685
            ip.pose.covariance = cov

            self.initialpose_pub.publish(ip)
            self._ensure_navigator()
            self._nav.setInitialPose(pose)

            self._ui_print(f"📍 Initial pose applied (once): ({x:.2f},{y:.2f}, yaw {yaw:.2f}) in '{frame}'", "cyan")
            self._initial_pose_applied = True
        except Exception as e:
            self._ui_print(f"⚠️  Failed to set initial pose (ignored): {e}", "yellow")
            self._initial_pose_applied = True

    def _cancel_nav_task_and_wait(self, timeout_s: float = 5.0) -> bool:

        assert self._nav is not None
        self._nav.cancelTask()
        deadline = time.monotonic() + max(0.0, float(timeout_s))
        while time.monotonic() < deadline:
            if self._nav.isTaskComplete():
                return True
            time.sleep(0.02)
        return bool(self._nav.isTaskComplete())

    def _settle_replacement_navigation(self) -> None:

        settle_s = max(
            0.0,
            float(getattr(self, "_replacement_nav_settle_s", 0.0)),
        )
        deadline = time.monotonic() + settle_s
        while rclpy.ok() and time.monotonic() < deadline:
            time.sleep(min(0.05, max(0.0, deadline - time.monotonic())))

    def _follow_waypoints(
        self,
        waypoints: List[PoseStamped],
        names: List[str],
        *,
        retry_early_abort_once: bool = False,
    ) -> TaskResult:
        assert self._nav is not None
        self._nav.waitUntilNav2Active()
        total = len(waypoints)
        next_index = 0
        early_abort_retries = 1 if retry_early_abort_once else 0

        while next_index < total:
            active_waypoints = waypoints[next_index:]
            goal_started_at = time.monotonic()
            accepted = self._nav.followWaypoints(active_waypoints)
            if accepted is False:
                self._ui_print("❌ Nav2 rejected the waypoint goal.", "red")
                return TaskResult.FAILED

            last_local_idx = 0
            last_reported_global_idx = -1
            last_progress_update = 0.0
            restart_after_pause = False

            while not self._nav.isTaskComplete():
                if self._replacement_can_preempt():
                    self._ui_print(
                        "Canceling the active Nav2 task for the validated "
                        "replacement command.",
                        "yellow",
                    )
                    cancel_complete = self._cancel_nav_task_and_wait()
                    if not cancel_complete:
                        self._ui_print(
                            "Nav2 cancellation did not reach a terminal state "
                            "within the timeout; delaying replacement startup.",
                            "yellow",
                        )
                    self._settle_replacement_navigation()
                    return TaskResult.CANCELED
                if self._cancel_requested.is_set():
                    self._ui_print("Canceling the current Nav2 mission.", "yellow")
                    self._cancel_nav_task_and_wait()
                    return TaskResult.CANCELED
                if self._pause_requested.is_set():
                    resume_index = min(
                        total - 1,
                        next_index + max(0, last_local_idx),
                    )
                    self._cancel_nav_task_and_wait()
                    self._pause_requested.clear()
                    self._mission_paused.set()
                    resume_name = (
                        names[resume_index]
                        if resume_index < len(names)
                        else f"Waypoint {resume_index + 1}"
                    )
                    self._update_navigation_progress(
                        current=resume_index + 1,
                        total=total,
                        destination=resume_name,
                        remaining_m=self._dist_to_pose(waypoints[resume_index]),
                    )
                    self._ui_print(
                        f"Mission paused before {resume_name}; route state retained.",
                        "yellow",
                    )
                    while rclpy.ok() and self._mission_paused.is_set():
                        if (
                            self._cancel_requested.is_set()
                            or self._replacement_can_preempt()
                        ):
                            break
                        time.sleep(0.05)
                    if self._replacement_can_preempt():
                        self._ui_print(
                            "Paused mission replaced by a validated command.",
                            "yellow",
                        )
                        return TaskResult.CANCELED
                    if self._cancel_requested.is_set():
                        self._ui_print("Paused mission canceled.", "yellow")
                        return TaskResult.CANCELED
                    next_index = resume_index
                    self._ui_print(
                        f"Mission resumed from {resume_name}.",
                        "green",
                    )
                    restart_after_pause = True
                    break

                fbk = self._nav.getFeedback()
                if fbk and hasattr(fbk, "current_waypoint"):
                    local_idx = int(fbk.current_waypoint)
                    if 0 <= local_idx < len(active_waypoints):
                        last_local_idx = local_idx




                global_idx = min(
                    total - 1,
                    next_index + max(0, last_local_idx),
                )
                now_mono = time.monotonic()
                if (
                    global_idx != last_reported_global_idx
                    or (now_mono - last_progress_update) >= 0.25
                ):
                    wp = waypoints[global_idx].pose.position
                    lab = (
                        names[global_idx]
                        if global_idx < len(names)
                        else f"({wp.x:.1f},{wp.y:.1f})"
                    )
                    dist = self._dist_to_pose(waypoints[global_idx])
                    self._update_navigation_progress(
                        current=global_idx + 1,
                        total=total,
                        destination=lab,
                        remaining_m=dist,
                    )
                    last_progress_update = now_mono
                    if global_idx != last_reported_global_idx:
                        self._ui_print(
                            f"▶️  Progress: Waypoint {global_idx+1}/{total} "
                            f"→ {lab}   (remaining≈{dist:.2f} m)",
                            "blue",
                        )
                    last_reported_global_idx = global_idx
                time.sleep(0.05)





            if self._replacement_can_preempt():
                self._ui_print(
                    "Active Nav2 goal ended while a validated replacement "
                    "was pending.",
                    "yellow",
                )
                return TaskResult.CANCELED
            if self._cancel_requested.is_set():
                self._ui_print(
                    "Active Nav2 goal ended after a cancel request.",
                    "yellow",
                )
                return TaskResult.CANCELED

            if restart_after_pause:
                continue

            result = self._nav.getResult()
            if result == TaskResult.SUCCEEDED:
                self._ui_print("✅ All waypoints reached", "green")
            elif result == TaskResult.FAILED:
                elapsed_s = time.monotonic() - goal_started_at
                retry_window_s = max(
                    0.0,
                    float(
                        getattr(
                            self,
                            "_replacement_early_abort_retry_window_s",
                            0.0,
                        )
                    ),
                )
                if early_abort_retries and elapsed_s <= retry_window_s:
                    early_abort_retries -= 1
                    self._ui_print(
                        "Nav2 aborted the replacement goal during action "
                        "handoff; retrying once after cleanup.",
                        "yellow",
                    )
                    self._settle_replacement_navigation()
                    continue
            return result

        return TaskResult.SUCCEEDED

    def _dist_to_pose(self, target: PoseStamped) -> float:
        pose = self._get_robot_pose(target.header.frame_id or self._global_frame)
        if not pose:
            return 0.0
        x, y, _ = pose
        tx, ty = target.pose.position.x, target.pose.position.y
        return math.hypot(tx - x, ty - y)


    def _waypoints_from_plan_with_names(self, plan: List[Dict[str, Any]]) -> Tuple[List[PoseStamped], List[str]]:
        wps: List[PoseStamped] = []
        names: List[str] = []
        for item in plan:
            dest = item.get("dest")
            if isinstance(dest, str):
                key = dest.strip().upper()
                if hasattr(self, "_waypoints") and key in self._waypoints:
                    x, y, yaw = self._waypoints[key]
                    wps.append(self._make_pose(x, y, yaw))
                    names.append(key)
                else:
                    valid = ", ".join(
                        sorted(self._waypoints, key=waypoint_sort_key)
                    )
                    self._ui_print(
                        f"⚠️ Unknown waypoint '{key}'. Valid: {valid}",
                        "yellow",
                    )
            elif isinstance(dest, dict):
                x = float(dest.get("x", 0.0))
                y = float(dest.get("y", 0.0))
                yaw = float(dest.get("yaw", 0.0))
                wps.append(self._make_pose(x, y, yaw))
                display_name = str(item.get("display_name", "")).strip()
                names.append(display_name or f"({x:.1f},{y:.1f})")
        return wps, names

    def _via_waypoints_from_zones(
        self,
        via_zones: List[str],
        keepouts: List[str],
    ) -> List[Dict[str, Any]]:

        blocked = {str(zone).upper() for zone in keepouts}
        via_plan: List[Dict[str, Any]] = []
        for zone in via_zones:
            label = str(zone).upper()
            bounds = self._zone_db.get(label)
            if bounds is None or label in blocked:
                raise PolicyUpdateRejected(f"required via zone unavailable: {label}")
            x, y = bounds.center
            via_plan.append(
                {
                    "dest": {"x": x, "y": y, "yaw": 0.0},
                    "display_name": f"Zone {label}",
                }
            )
        return via_plan

    def _make_pose(self, x: float, y: float, yaw: float, frame: Optional[str] = None) -> PoseStamped:
        ps = PoseStamped()
        ps.header.frame_id = frame or self._global_frame
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        qx, qy, qz, qw = yaw_to_quat(yaw)
        ps.pose.orientation.x, ps.pose.orientation.y = qx, qy
        ps.pose.orientation.z, ps.pose.orientation.w = qz, qw
        return ps


    def _ensure_yolo_ready(self):
        if self.yolo_model:
            return
        if not YOLO_AVAILABLE:
            self._ui_print("⚠️ YOLO not installed", "yellow")
            return
        try:
            self.yolo_model = YOLO(self._yolo_model_name)
            if not self.bridge and CvBridge:
                self.bridge = CvBridge()
            if self.bridge and not hasattr(self, "image_sub"):
                self.image_sub = self.create_subscription(RosImage, self._camera_topic, self._image_callback, self._image_qos)
            self._ui_print("🔛 YOLO enabled", "green")
        except Exception as e:
            self._ui_print(f"YOLO dynamic enable failed: {e}", "red")

    def _image_callback(self, msg: RosImage):
        if not self.bridge or not self._latest_dynamic_rules:
            return
        try:
            import cv2, numpy as np
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            bgr = np.ascontiguousarray(bgr, dtype=np.uint8)
            with self.image_lock:
                self.latest_image = bgr
                self.latest_header = msg.header
                self._latest_seq += 1
            self._last_frame_mono = time.monotonic()
        except Exception as e:
            self._ui_print(f"cv_bridge error: {e}", "red")

    def _yolo_worker_loop(self):
        target_period = self._yolo_inference_period
        while rclpy.ok() and getattr(self, "_yolo_run", False):
            t0 = time.monotonic()



            if not self._latest_dynamic_rules:
                with self.image_lock:
                    self.latest_image = None
                    self.latest_header = None
                    self._last_seq_processed = self._latest_seq
                time.sleep(min(target_period, 0.1))
                continue

            with self.image_lock:
                frame = None
                header = self.latest_header
                seq = getattr(self, "_latest_seq", 0)
                if self.latest_image is not None and seq != self._last_seq_processed:
                    frame = self.latest_image.copy()

            since = time.monotonic() - getattr(self, "_last_frame_mono", 0.0)
            if since > self._stale_window_s:
                now = time.monotonic()
                if now - self._last_resub_try >= self._resub_backoff_s:
                    self._ui_print("⚠️ camera frame not updating. Re-subscribing...", "yellow")
                    try:
                        if hasattr(self, "image_sub"):
                            self.destroy_subscription(self.image_sub)
                        self.image_sub = self.create_subscription(RosImage, self._camera_topic, self._image_callback, self._image_qos)
                    except Exception as e:
                        self._ui_print(f"resubscribe failed: {e}", "red")
                    self._last_resub_try = now
                    self._resub_backoff_s = min(self._resub_backoff_s * 2.0, 8.0)
            else:
                self._resub_backoff_s = 1.0

            if since > max(self._stale_window_s * 1.5, 3.0):
                for frame_name in (self._object_frames or [self._global_frame]):
                    self._publish_objects(frame_name, [])

            if frame is None:
                for frame_name in (
                    self._object_frames or [self._global_frame]
                ):
                    self._publish_objects(frame_name, [])
                time.sleep(target_period)
                continue

            try:
                self._do_yolo_once(frame, header)
            finally:
                self._last_seq_processed = seq
                with self.image_lock:
                    self.latest_image = None

            time.sleep(max(0.0, target_period - (time.monotonic() - t0)))

    def _caminfo_cb(self, msg: CameraInfo):
        self._K = list(msg.k)

    def _depth_info_cb(self, msg: CameraInfo):
        self._depth_K = (float(msg.k[0]), float(msg.k[4]), float(msg.k[2]), float(msg.k[5]))

    def _depth_cb(self, msg: RosImage):
        if not self.bridge:
            return
        try:
            import numpy as np
            if msg.encoding in ("32FC1", "32FC"):
                d = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
            elif msg.encoding in ("16UC1", "mono16"):
                raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding="16UC1")
                d = raw.astype(np.float32) / 1000.0
            else:
                d = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
            self._depth_img = d
            self._depth_header = msg.header
        except Exception as e:
            self._ui_print(f"depth bridge error: {e}", "red")

    def _get_median_depth(self, u: int, v_bottom: int, v_top: int) -> Optional[float]:
        import numpy as np
        if self._depth_img is None:
            return None
        h, w = self._depth_img.shape[:2]
        u0 = max(0, u - self._roi_halfw)
        u1 = min(w, u + self._roi_halfw + 1)
        v0 = max(0, min(v_top, v_bottom))
        v1 = min(h, max(v_top, v_bottom) + 1)
        roi = self._depth_img[v0:v1, u0:u1].reshape(-1)
        good = roi[np.isfinite(roi)]
        good = good[(self._z_min <= good) & (good <= self._z_max)]
        if good.size < 20:
            return None
        return float(np.median(good))

    def _backproject_cam(self, u: float, v: float, Z: float, K=None):
        if K is None:
            K = self._depth_K or self._K
        if not K:
            return None
        if len(K) == 4:
            fx, fy, cx, cy = K
        else:
            fx, fy, cx, cy = float(K[0]), float(K[4]), float(K[2]), float(K[5])
        X = (u - cx) * Z / fx
        Y = (v - cy) * Z / fy
        return (X, Y, Z)

    def _cam_to_world(self, xyz_cam, target_frame, stamp):
        try:
            t = rclpy.time.Time.from_msg(stamp) if stamp else rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(target_frame, self._camera_optical_frame, t)
            import numpy as np
            T = tf_transformations.quaternion_matrix([
                trans.transform.rotation.x, trans.transform.rotation.y,
                trans.transform.rotation.z, trans.transform.rotation.w
            ])
            T[:3, 3] = [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z]
            p = np.array([xyz_cam[0], xyz_cam[1], xyz_cam[2], 1.0])
            pw = T.dot(p)
            return (float(pw[0]), float(pw[1]), float(pw[2]))
        except Exception:
            return None

    def _get_pose_at(self, source_frame, target_frame):
        try:
            t = rclpy.time.Time.from_msg(self.latest_header.stamp) if getattr(self, "latest_header", None) else rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(target_frame, source_frame, t)
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            z = trans.transform.translation.z
            q = trans.transform.rotation
            return (x, y, z, q.x, q.y, q.z, q.w)
        except Exception:
            return None

    def _estimate_distance_px(self, bbox, image_h):
        x1, y1, x2, y2 = bbox
        px_h = max(0.0, y2 - y1)
        fy = float(self._K[4]) if self._K else 525.0 * (image_h / 480.0)
        if px_h <= 10:
            return self._max_dist
        d = (self._object_h * fy) / px_h
        return max(self._min_dist, min(d, self._max_dist))

    def _project_with_depth(self, boxes, img_shape, target_frame, rgb_header):
        if not boxes:
            return []
        h, w = img_shape[:2]
        out = []
        for (x1, y1, x2, y2) in boxes:
            u = int((x1 + x2) * 0.5)
            v_bottom = int(min(h - 1, y2))
            v_top = int(max(0, y2 - (y2 - y1) * self._roi_bottom_ratio))
            Z = self._get_median_depth(u, v_bottom, v_top)
            if Z is None:
                Z = self._estimate_distance_px((x1, y1, x2, y2), h)
                fx = float(self._K[0]) if self._K else (w / 2.0)
                cxp = float(self._K[2]) if self._K else (w / 2.0)
                Xc = ((u - cxp) / fx) * Z
                Yc = 0.0
                Zc = Z
            else:
                v = max(0, min(h - 1, int(v_bottom) - 2))
                cam = self._backproject_cam(float(u), float(v), Z, self._depth_K or self._K)
                if cam is None:
                    continue
                Xc, Yc, Zc = cam

            pw = self._cam_to_world((Xc, Yc, Zc), target_frame, rgb_header.stamp if rgb_header else None)
            if pw:
                out.append((pw[0], pw[1]))
        return out

    def _project_objects_to_world_for_frame(self, boxes, shape, target_frame):
        if not boxes:
            return []
        h, w = shape[:2]
        pose_cam = self._get_pose_at(self._camera_optical_frame, target_frame)
        if not pose_cam:
            return []
        (cx, cy, cz, qx, qy, qz, qw) = pose_cam
        import numpy as np
        R = tf_transformations.quaternion_matrix([qx, qy, qz, qw])[:3, :3]
        fx = float(self._K[0]) if self._K else (w / 2.0)
        cxp = float(self._K[2]) if self._K else (w / 2.0)

        out = []
        for (x1, y1, x2, y2) in boxes:
            u = (x1 + x2) / 2.0
            Z = self._estimate_distance_px((x1, y1, x2, y2), h)
            X = (u - cxp) / fx * Z
            pc = np.array([X, 0.0, Z])
            pw = R.dot(pc) + np.array([cx, cy, cz])
            out.append((float(pw[0]), float(pw[1])))
        return out

    def _smooth_assign_tracks(self, pts):
        out = []
        used = set()
        assigned = {}

        for (x, y) in pts:
            best, bid = 1e9, None
            for k, (px, py) in self._trk_last.items():
                if k in used:
                    continue
                d2 = (x - px) ** 2 + (y - py) ** 2
                if d2 < best:
                    best, bid = d2, k
            if bid is None:
                self._trk_id += 1
                tid = self._trk_id
                self._trk_last[tid] = (x, y)
                assigned[tid] = (x, y)
                used.add(tid)
            else:
                px, py = self._trk_last[bid]
                if math.hypot(x - px, y - py) > self._trk_max_jump:
                    self._trk_id += 1
                    tid = self._trk_id
                    self._trk_last[tid] = (x, y)
                    assigned[tid] = (x, y)
                    used.add(tid)
                else:
                    dt_s = 0.12
                    vx = (x - px) / max(dt_s, 1e-3)
                    vy = (y - py) / max(dt_s, 1e-3)
                    speed = math.hypot(vx, vy)
                    a = max(0.25, min(0.55, 0.25 + 0.30 * (speed / 1.0)))
                    sx = a * x + (1 - a) * px
                    sy = a * y + (1 - a) * py
                    self._trk_last[bid] = (sx, sy)
                    self._trk_last_vel[bid] = (vx, vy)
                    assigned[bid] = (sx, sy)
                    used.add(bid)

        for tid, p in assigned.items():
            hx = self._trk_hist[tid]
            hx.append(p)
            if len(hx) >= 3:
                xs, ys = zip(*hx)
                mx = sorted(xs)[len(xs)//2]
                my = sorted(ys)[len(ys)//2]
                out.append((mx, my))
            else:
                out.append(p)
        return out

    def _do_yolo_once(self, frame, header=None):
        import cv2

        if frame.ndim == 2 or (frame.ndim == 3 and frame.shape[2] == 1):
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

        conf = float(self._conf_thresh)

        results = self.yolo_model.predict(
            frame, conf=conf, imgsz=640,
            iou=0.45, agnostic_nms=True, max_det=200, verbose=False,
            device=0 if torch.cuda.is_available() else 'cpu',
        )
        r0 = results[0]

        names_map = self._resolve_names_map(r0)
        target_ids = self._target_class_ids(names_map)

        if not target_ids:
            for frame_name in (self._object_frames or [self._global_frame]):
                self._publish_objects(frame_name, [])
            return

        objects_px = []
        if getattr(r0, "boxes", None):
            for box in r0.boxes:
                c_id = int(box.cls[0])
                score = float(box.conf[0])
                if (c_id in target_ids) and (score >= conf):
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    objects_px.append((float(x1), float(y1), float(x2), float(y2)))

        dbg = frame.copy()
        for (x1, y1, x2, y2) in objects_px:
            cv2.rectangle(dbg, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.putText(dbg, "object", (int(x1), int(y1) - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        if self.bridge:
            dbg_msg = self.bridge.cv2_to_imgmsg(dbg, encoding="bgr8")
            dbg_msg.header.stamp = self.get_clock().now().to_msg()
            dbg_msg.header.frame_id = "yolo_debug"
            self.debug_img_pub.publish(dbg_msg)

        target_frames = self._object_frames or [self._global_frame]
        if objects_px:
            self._miss_frames = 0
            for frame_name in target_frames:
                poses_xy = self._project_with_depth(objects_px, frame.shape, frame_name, header)
                if not poses_xy:
                    poses_xy = self._project_objects_to_world_for_frame(objects_px, frame.shape, frame_name)
                if poses_xy:
                    poses_xy = self._smooth_assign_tracks(poses_xy)
                    self._publish_objects(frame_name, poses_xy)
                else:
                    self._publish_objects(frame_name, [])
        else:
            self._miss_frames += 1
            if self._miss_frames >= 3:
                self._trk_last.clear()
            now_mono = time.monotonic()
            for frame_name in target_frames:
                last_t = self._last_nonempty_pub_time.get(frame_name, 0.0)
                if (now_mono - last_t) >= self._objects_ttl_s:
                    self._publish_objects(frame_name, [])

    def _resolve_names_map(self, r0) -> Dict[int, str]:
        nm = getattr(r0, "names", None)
        if not nm:
            nm = getattr(self.yolo_model, "names", None)
        if isinstance(nm, dict):
            return nm
        if isinstance(nm, (list, tuple)):
            return {i: n for i, n in enumerate(nm)}
        return {}

    def _target_class_ids(self, names_map: Dict[int, str]) -> set:
        dyn_rules = self._latest_dynamic_rules or []
        wanted = {
            str(r.get("class")).strip().lower()
            for r in dyn_rules
            if isinstance(r, dict) and r.get("class")
        }
        if not wanted:
            return set()
        targets = set()
        for i, n in (names_map or {}).items():
            if str(n).strip().lower() in wanted:
                targets.add(i)
        return targets

    def _object_hold_active(self) -> bool:

        mission_active = getattr(self, "_mission_active", None)
        return bool(
            self._latest_dynamic_rules
            and mission_active is not None
            and mission_active.is_set()
        )

    def _positions_with_mission_hold(
        self,
        frame: str,
        positions_xy: List[Tuple[float, float]],
        *,
        force_clear: bool = False,
    ) -> List[Tuple[float, float]]:

        if not hasattr(self, "_held_object_positions"):
            self._held_object_positions = {}
        positions = list(positions_xy or [])
        if force_clear:
            self._held_object_positions.pop(frame, None)
            return []
        if positions:
            self._held_object_positions[frame] = positions[:]
            return positions
        if self._object_hold_active():
            return list(self._held_object_positions.get(frame, []))
        self._held_object_positions.pop(frame, None)
        return []

    def _clear_object_observations(self) -> None:

        frames = set(getattr(self, "_object_frames", []) or [])
        frames.update(getattr(self, "_held_object_positions", {}))
        frames.update(getattr(self, "_last_pub_by_frame", {}))
        if not frames:
            frames.add(getattr(self, "_global_frame", "map"))
        for frame in frames:
            self._publish_objects(frame, [], force_clear=True)
        self._held_object_positions.clear()

    def _publish_objects(
        self,
        frame: str,
        positions_xy: List[Tuple[float, float]],
        *,
        force_clear: bool = False,
    ):
        positions_xy = self._positions_with_mission_hold(
            frame,
            positions_xy,
            force_clear=force_clear,
        )
        if not hasattr(self, "_last_pub_by_frame"):
            self._last_pub_by_frame = {}
        prev = self._last_pub_by_frame.get(frame)
        if prev and len(prev) == len(positions_xy):
            stable = []
            for (nx, ny), (px, py) in zip(positions_xy, prev):
                if math.hypot(nx - px, ny - py) < self._deadband:
                    stable.append((px, py))
                else:
                    stable.append((nx, ny))
            positions_xy = stable

        self._last_pub_by_frame[frame] = positions_xy[:]
        if positions_xy:
            self._last_nonempty_pub_time[frame] = time.monotonic()

        publisher = getattr(self, "object_pub", None)
        if publisher is None:
            return

        msg = PoseArray()
        msg.header.frame_id = frame
        msg.header.stamp = self.get_clock().now().to_msg()
        for (x, y) in positions_xy:
            p = Pose()
            p.position.x = float(x)
            p.position.y = float(y)
            p.orientation.w = 1.0
            msg.poses.append(p)
        publisher.publish(msg)

        now = time.time()
        last = self._last_object_log_time.get(frame, 0.0)
        if (now - last) >= self._objects_log_interval:
            n = len(positions_xy)
            if n > 0:
                pts = "; ".join([f"({x:.1f},{y:.1f})" for x, y in positions_xy[:6]])
                more = "" if n <= 6 else f" …(+{n-6})"
                self._ui_print(f"👥 Objects detected (frame '{frame}'): {n}  {pts}{more}", "yellow")
            self._last_object_log_time[frame] = now

    def _get_robot_pose(self, target_frame: str) -> Optional[Tuple[float, float, float]]:
        try:
            trans = self.tf_buffer.lookup_transform(target_frame, self._base_frame, rclpy.time.Time())
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            q = trans.transform.rotation
            _, _, yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
            return x, y, yaw
        except Exception:
            return None


    def _console_input_loop(self):
        prompt = self._c("command: ", "bold")
        while rclpy.ok():
            try:

                line = sys.stdin.readline()
                if not line:
                    break
                s = line.strip()
                if not s:
                    continue
                low = s.lower()
                if low in ("q", "quit", "exit"):
                    self._ui_print("Press Ctrl+C to stop the CLI. (Node stays running)", "yellow")
                    continue
                if low in ("h", "help", "?"):
                    self._ui_print(
                        "Usage) type natural-language commands as-is\n"
                        "Example: 'Make A permanently forbidden, ban B this time, avoid people and reach D, set cost 200 for A'\n"
                        "State commands: 'fire alarm on', 'fire alarm off', 'battery 15%'\n"
                        "Helper commands: status | keepouts | objects | reset | quit",
                        "gray"
                    )
                    continue
                if low == "status":
                    self._print_status()
                    continue
                if low == "keepouts":
                    self._print_keepouts_snapshot()
                    continue
                if low == "objects":
                    self._print_objects_snapshot()
                    continue
                if low == "reset":
                    self._reset_policy_for_next_trial()
                    continue

                self._enqueue_nl_command(s)
                self._ui_print(f"📨 Received command: {s}", "cyan")
            except KeyboardInterrupt:
                break
            except Exception as e:
                self._ui_print(f"CLI error: {e}", "red")
                break

    def _print_status(self):
        self._ui_print("===== STATUS =====", "bold")
        self._ui_print(
            f"state: fire={self._env_state['fire_alarm']} battery={self._env_state['battery_pct']}%",
            "gray"
        )
        self._print_keepouts_snapshot()
        self._print_objects_snapshot()
        self._ui_print("==================", "bold")

    def _print_keepouts_snapshot(self):
        self._ui_print(f"Permanent keepouts: {', '.join(self._permanent_exclusions) or '-'}", "yellow")

        now = dt.datetime.now(self._tz)
        with self._sched_lock:
            schedules = list(self._schedules.values())
            active_ids = set(self._active_timed.keys())

        if not schedules:
            self._ui_print("Scheduled keepouts: -", "yellow")
        else:
            self._ui_print("Scheduled keepouts:", "yellow")
            for tk in sorted(schedules, key=lambda x: x.start_wall):
                st = tk.start_wall
                en = tk.end_wall or (st + dt.timedelta(minutes=self._default_forbid_minutes))
                status = "ACTIVE" if tk.id in active_ids else ("PENDING" if st > now else "EXPIRED?")
                self._ui_print(
                    f"  #{tk.id} [{status}] zones={','.join(tk.zones)} {st.strftime('%Y-%m-%d %H:%M')} → {en.strftime('%Y-%m-%d %H:%M')}",
                    "gray"
                )

        self._ui_print(f"Publishing to: {self._forbid_topic}", "gray")

    def _print_objects_snapshot(self):
        frames = self._object_frames or [self._global_frame]
        for f in frames:
            last = self._last_object_log_time.get(f, 0.0)
            ago = time.time() - last if last > 0 else None
            self._ui_print(f"Object frame '{f}': last publish {('%.1fs ago' % ago) if ago else 'no record'}", "gray")



def main():
    rclpy.init()
    node = PolicyBridgeNode()

    if not node._require_nav2_or_shutdown():
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
        node._yolo_run = False
        if getattr(node, "_yolo_thread", None):
            node._yolo_thread.join(timeout=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
