                      
                       

from __future__ import annotations
from zoneinfo import ZoneInfo
from collections import defaultdict, deque
import datetime as dt
import sys
import math
import json
import torch
import time
import re
import threading
from dataclasses import dataclass
from typing import List, Tuple, Optional, Dict, Any
from enum import Enum
from rclpy.duration import Duration
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from rclpy.action import ActionClient
from nav2_msgs.action import FollowWaypoints

from std_msgs.msg import String as RosString
from geometry_msgs.msg import PoseArray, Pose, PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import Image as RosImage, CameraInfo

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

import tf2_ros
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


                                                                   
@dataclass
class ZoneRect:
    x_min: float
    y_min: float
    x_max: float
    y_max: float
    def center(self) -> Tuple[float, float]:
        return ((self.x_min + self.x_max) / 2.0, (self.y_min + self.y_max) / 2.0)

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

class CondAction(str, Enum):
    FORBID = "forbid"
    ALLOW = "allow"
    ALLOW_SHORTEST = "allow_shortest"


                                                                    
ZONE_LABEL_RE = re.compile(r"(?<![A-Za-z0-9])([A-G])(?![A-Za-z0-9])", re.IGNORECASE)
WEIGHT_CMD_RE = re.compile(
    r"([A-Z])\s*[^0-9A-Za-z가-힣]*?(?:주변|근처|around)?\s*([0-9]+(?:\.[0-9]+)?)\s*m.*?(?:비용|cost)\s*([0-9]{1,3})",
    flags=re.IGNORECASE,
)
WEIGHT_CMD_RE2 = re.compile(
    r"([A-Z]).{0,10}?(?:주변|근처|around)?\s*(?:비용|cost)\s*([0-9]{1,3})", re.IGNORECASE
)

DEST_KWS = ["로 가", "으로 가", "로 이동", "로 갔다가", "가다가", "방문", "도달", "도착", "go to", "arrive", "then", "next"]
EXC_KWS_TMP = ["금지", "피해", "피해서", "제외", "avoid", "ban", "forbid"]
EXC_KWS_PERM = ["영구", "항상", "permanent", "always"]
COND_KWS = ["if", "when", "unless", "but", "however", "다만", "하지만", "만약", "경우", "때", "시"]

def _is_perm_cost_sentence(s: str) -> bool:
    su = s.upper()
    return any(k.upper() in su for k in EXC_KWS_PERM)

def unique_preserve(xs: List[str]) -> List[str]:
    seen, out = set(), []
    for x in xs:
        if x not in seen:
            out.append(x); seen.add(x)
    return out

def yaw_to_quat(yaw: float) -> Tuple[float, float, float, float]:
    return tf_transformations.quaternion_from_euler(0.0, 0.0, yaw)


                                                             
class PolicyBridgeNode(Node):
    """Policy-bridge node + interactive console UI."""

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

    def __init__(self):
        super().__init__("policy_bridge")

                                                      
        self.declare_parameter("nl_command_topic", "/nl_command")
        self.declare_parameter("event_topic", "/event")
        self.declare_parameter("forbidden_zones_topic", "/fleet/forbidden_zones_update")
        self.declare_parameter("object_positions_topic", "/fleetobject_world_positions")
        self.declare_parameter("zone_cost_overrides_topic", "/fleet/zone_cost_overrides")
                                                
        self._last_pub_by_frame: Dict[str, List[Tuple[float, float]]] = {}

                                              
        self._trk_last: Dict[int, Tuple[float, float]] = {}
        self._trk_id = 0
        self._miss_frames = 0
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")

        self.declare_parameter("zone_labels", ["A", "B", "C", "D", "E", "F", "G"])
        self.declare_parameter(
            "zone_database_json",
            json.dumps({
                "A": [8.01, -1.55, 23.96, 1.90],
                "B": [8.01, -6.05, 24.01, -2.60],
                "C": [8.01, -10.70, 24.06, -7.20],
                "D": [4.11, -15.35, 8.11, -11.85],
                "E": [24.11, -15.55, 27.91, -11.75]
            }, ensure_ascii=False)
        )

             
        self.declare_parameter("enable_llm", True)
        self.declare_parameter("llm_endpoint", "http://localhost:11434/api/generate")
        self.declare_parameter("llm_api_key", "")
        self.declare_parameter("llm_model", "llama3.1:latest")

                                 
        self.declare_parameter("enable_yolo", True)
        self.declare_parameter("camera_topic", "/camera/image_raw")
        self.declare_parameter("yolo_model", "yolov8m.pt")
        self.declare_parameter("object_conf_threshold", 0.3)
        self.declare_parameter("object_avg_height_m", 1.70)
        self.declare_parameter("min_distance_m", 0.50)
        self.declare_parameter("max_distance_m", 8.00)

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
            json.dumps({"x": 16.0784, "y": -8.92587, "yaw": 0.0, "frame": "map"}, ensure_ascii=False)
        )

        self.declare_parameter("permanent_exclusions", [])

        self.declare_parameter("timezone", "Asia/Seoul")
        self.declare_parameter("default_forbid_minutes", 2)

        self.declare_parameter("require_nav2", True)
        self.declare_parameter("nav2_check_timeout_s", 3.0)
        self.declare_parameter("follow_waypoints_action", "/follow_waypoints")

                                  
        self.declare_parameter("allow_overrides_permanent", True)

                                                           
        self._nl_topic       = self.get_parameter("nl_command_topic").value
        self._event_topic    = self.get_parameter("event_topic").value
        self._forbid_topic   = self.get_parameter("forbidden_zones_topic").value
        self._object_topic   = self.get_parameter("object_positions_topic").value
        self._softcost_topic = self.get_parameter("zone_cost_overrides_topic").value

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

        self._waypoints: Dict[str, Tuple[float, float, float]] = {
            "WP1": (-0.2,   0.41,   0.0),
            "WP2": (32.2,   0.41,   0.0),
            "WP3": (-0.2,  -4.56,   0.0),
            "WP4": (32.2,  -4.56,   0.0),
            "WP5": (-0.2, -13.40,   0.0),
            "WP6": (32.2, -13.40,   0.0),
            "WP7": (-0.2, -18.10,   0.0),
            "WP8": (32.2, -18.10,   0.0),
        }

        self._enable_llm   = bool(self.get_parameter("enable_llm").value)
        self._llm_endpoint = str(self.get_parameter("llm_endpoint").value)
        self._llm_key      = str(self.get_parameter("llm_api_key").value)
        self._llm_model    = str(self.get_parameter("llm_model").value)

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

        self._permanent_exclusions = [s.upper() for s in self.get_parameter("permanent_exclusions").value]

        self._allow_overrides_permanent = bool(self.get_parameter("allow_overrides_permanent").value)

                                                                       
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

        soft_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.softcost_pub = self.create_publisher(RosString, self._softcost_topic, soft_qos)

        self.initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.nl_sub = self.create_subscription(RosString, self._nl_topic, self._nl_callback, 10)
        self.event_sub = self.create_subscription(RosString, self._event_topic, self._event_callback, 10)
        self._active_softcost: Dict[str, int] = {}
        self._last_forbidden_windows: Dict[str, Tuple[dt.datetime, dt.datetime]] = {}

                                              
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self._print_lock = threading.Lock()

        self._image_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

                                                     
        self.bridge = CvBridge() if (CvBridge and self._enable_yolo) else None
        self.yolo_model = None
        self.latest_image = None
        self.latest_header = None
        self.image_lock = threading.Lock()
        self.debug_img_pub = self.create_publisher(RosImage, "/yolo/debug_image", qos_profile_sensor_data)

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
        self.create_subscription(CameraInfo, "/camera/camera_info", self._caminfo_cb, 10)

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
        self._timer_threads: Dict[int, tuple[threading.Timer, Optional[threading.Timer]]] = {}

                 
        self._latest_dynamic_rules: List[Dict[str, Any]] = []
        self._last_wp_names: List[str] = []

                   
        self._nav: Optional[BasicNavigator] = None

                    
        self._last_object_log_time: Dict[str, float] = {}
        if self._ui_enable:
            self._ui_print("🟢 Console UI enabled.", "green")
            self._ui_print("Help: help | status | keepouts | objects | quit", "gray")
            self._console_thread = threading.Thread(target=self._console_input_loop, daemon=True)
            self._console_thread.start()

                                 
        if self._set_initial_pose:
            self._apply_initial_pose_param_once()

        self._ui_print(
            f"Policy-Bridge node up. NL: {self._nl_topic}  →  Zones: {self._forbid_topic}, Objects: {self._object_topic}, SoftCost: {self._softcost_topic}",
            "magenta",
        )

        if not self._require_nav2_or_shutdown():
            self._ui_print("🛑 Policy-Bridge disabled because Nav2 is not ready.", "red")
            self._yolo_run = False
            self._nav2_ready = False
            return

        self._nav2_ready = True

        self._mission_queue: deque[RosString] = deque()
        self._mission_lock = threading.Lock()
        self._mission_thread = threading.Thread(target=self._mission_worker, daemon=True)
        self._mission_thread.start()

        self._yolo_run = True
        self._yolo_thread = threading.Thread(target=self._yolo_worker_loop, daemon=True)
        self._yolo_thread.start()

                                                  
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
        """Wall clock datetime -> ROS time (follows /clock if use_sim_time is true)."""
        if wall_dt.tzinfo is None:
            wall_dt = wall_dt.replace(tzinfo=self._tz)

        wall_now = dt.datetime.now(self._tz)
        ros_now = self.get_clock().now()
        delta = wall_dt - wall_now
        return ros_now + Duration(seconds=delta.total_seconds())

    def _oneshot_ros_timer(self, delay_s: float, cb):
        """One-shot ROS timer (ROS-time aware)."""
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

        rep_ko = [
            (r"첫\s*번째\s*(통로|복도)", "A"),
            (r"두\s*번째\s*(통로|복도)", "B"),
            (r"세\s*번째\s*(통로|복도)", "C"),
            (r"네\s*번째\s*(통로|복도)", "D"),
            (r"다섯\s*번째\s*(통로|복도)", "E"),
        ]
        for pat, lab in rep_ko:
            out = re.sub(pat, lab, out)

        return out

    def _split_intents_deterministic(self, text: str) -> tuple[set[str], Dict[str, int]]:
        if not text:
            return set(), {}

        chunks = re.split(r"[,.。!?]\s*|\n+", text)
        keepout_kws = [k.lower() for k in EXC_KWS_TMP]
        cost_kws = ["비용", "cost"]

        keepouts: set[str] = set()
        soft: Dict[str, int] = {}

        def _zones_in(ch: str) -> list[str]:
            return [c for c in ZONE_LABEL_RE.findall(ch.upper()) if c in self._labels]

        PAIR_EN = re.compile(r"(?:\bZONE\s*)?([A-G])\s*(?:TO|=|:)\s*([0-9]{1,3})\b", re.IGNORECASE)
        PAIR_KO = re.compile(r"\b([A-G])\b[^0-9A-Za-z]{0,20}(?:비용)\s*([0-9]{1,3})\b", re.IGNORECASE)
        PAIR_EN2 = re.compile(r"\b([A-G])\b[^0-9A-Za-z]{0,20}(?:COST)\s*([0-9]{1,3})\b", re.IGNORECASE)

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

                for m in PAIR_KO.finditer(chU):
                    pairs.append((m.group(1).upper(), int(m.group(2))))

                for m in PAIR_EN2.finditer(chU):
                    pairs.append((m.group(1).upper(), int(m.group(2))))

                if pairs:
                    for z, c in pairs:
                        if z in self._labels:
                            soft[z] = max(0, min(255, int(c)))
                    continue

                chU_wo_wp = re.sub(r"\bWP\s*\d+\b", " ", chU, flags=re.IGNORECASE)
                zones = _zones_in(chU_wo_wp)
                nums = re.findall(r"\b([0-9]{1,3})\b", chU_wo_wp)

                if len(zones) == 1 and len(nums) == 1:
                    z = zones[0]
                    c = max(0, min(255, int(nums[0])))
                    soft[z] = c

                continue

            if has_keepout:
                for z in _zones_in(chs):
                    keepouts.add(z)
                continue

        keepouts = {z for z in keepouts if z not in soft.keys()}
        return keepouts, soft

    def _parse_duration_minutes(self, text: str) -> Optional[int]:
        s = (text or "").lower()
        m = re.search(r"\bfor\s+(\d{1,3})\s*(minutes?|mins?|min)\b", s)
        if m:
            return max(1, int(m.group(1)))
        m = re.search(r"(\d{1,3})\s*분\s*(동안|간)?", s)
        if m:
            return max(1, int(m.group(1)))
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
        m = re.search(r"(오늘|금일)\s*(오전|오후)?\s*(\d{1,2})\s*시\s*(\d{1,2})?\s*분?", s)
        if m:
            ap = m.group(2) or ""
            hh = int(m.group(3))
            mm = int(m.group(4) or 0)
            if "오후" in ap and hh != 12:
                hh += 12
            if "오전" in ap and hh == 12:
                hh = 0
            return dt.datetime(today.year, today.month, today.day, hh, mm, tzinfo=self._tz)
        return None

                                                                              
    def _looks_like_mission_text(self, text: str) -> bool:
        s = (text or "").strip()
        if not s:
            return False
        su = s.upper()
        if ZONE_LABEL_RE.search(su):
            return True
        cond_words = ("다만", "단 ", "만약", "경우", "상황", "때", "시", "허용", "금지")
        if any(w in s for w in cond_words):
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

              
        fire_on = re.search(r"(화재\s*(발생|경보\s*발생|경보\s*발령)|fire(\s*alarm)?\s*(on|start))", s)
        fire_off = re.search(r"(화재\s*(해제|경보\s*해제)|fire(\s*alarm)?\s*(off|stop))", s)
        if fire_on:
            self._env_state["fire_alarm"] = True
            handled = True
        if fire_off:
            self._env_state["fire_alarm"] = False
            handled = True

                  
        m = re.search(r"배터리\s*([0-9]{1,3})\s*%", s)
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
                rules = sorted(rules, key=lambda r: int(r.get("priority", 0)), reverse=True)
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

    def _recalculate_and_publish_dynamic_keepouts(
        self,
        base_exclusions: List[str] | None = None,
        conditional_rules: List[Dict[str, Any]] | None = None
    ):
        base_exclusions = base_exclusions or []
        conditional_rules = conditional_rules or []

        with self._sched_lock:
            timed_active = set().union(*self._active_timed.values()) if self._active_timed else set()

        cond_forbid, cond_allow, self._force_shortest = self._evaluate_conditional_rules(conditional_rules)

        merged = unique_preserve(
            self._permanent_exclusions + list(timed_active) + list(cond_forbid) + base_exclusions
        )
        merged = [z for z in merged if z in self._labels]

                                                              
        if cond_allow:
            if self._allow_overrides_permanent:
                merged = [z for z in merged if z not in cond_allow]
            else:
                merged = [z for z in merged if not (z in cond_allow and z not in self._permanent_exclusions)]

        self._publish_forbidden(merged)

                                               
    def _nl_callback(self, msg: RosString):
        if not getattr(self, "_nav2_ready", True):
            self._ui_print("⛔ Ignored command: Nav2 is not ready.", "red")
            return

        self._ui_print(f"📨 Received command: {msg.data}", "cyan")

        with self._mission_lock:
            self._mission_queue.append(msg)

    def _event_callback(self, msg: RosString):
        #self._ui_print(f"📡 Event received: {msg.data}", "yellow")
        try:
            self._handle_state_command(msg.data)
        except Exception as e:
            self._ui_print(f"[EVENT] Exception: {e}", "red")

    def _mission_worker(self):
        while rclpy.ok():
            msg: Optional[RosString] = None
            with self._mission_lock:
                if self._mission_queue:
                    msg = self._mission_queue.popleft()
            if msg is None:
                time.sleep(0.05)
                continue
            try:
                self._run_single_mission(msg.data)
            except Exception as e:
                self._ui_print(f"❌ Mission error: {e}", "red")

                                                                 
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
                    objs.append(cls)
            except Exception:
                pass
        objs = unique_preserve([o.lower() for o in objs])
        objs_pretty = ", ".join([o.capitalize() for o in objs]) if objs else ""
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
                    objs.append(cls)
            except Exception:
                pass
        objs = unique_preserve([o.lower() for o in objs])
        objs_pretty = ", ".join([o.capitalize() for o in objs]) if objs else ""
        self._ui_print(f"👥 Objects: {objs_pretty}", "yellow")
        seq = ", ".join(getattr(self, "_last_wp_names", []) or []) or "-"
        self._ui_print(f"🧭 Waypoint Order: {seq}", "cyan")
        self._ui_print("")

    def _run_single_mission(self, text: str):
        text = self._apply_aisle_aliases(text)
        self._ui_print("")
        self._active_softcost = {}
        plan_json = self._plan_with_llm_or_fallback(text)

        tlow = (text or "").lower()
        keepout_requested = any(kw.lower() in tlow for kw in EXC_KWS_TMP)
        permanent_requested = any(kw.lower() in tlow for kw in EXC_KWS_PERM)

        perm_from_nl = [s.upper() for s in (plan_json.get("permanent_exclusions", []) or [])]
        if perm_from_nl and keepout_requested and permanent_requested:
            self._permanent_exclusions = unique_preserve(self._permanent_exclusions + perm_from_nl)
        #elif perm_from_nl:
            #self._ui_print("ℹ️ Ignored LLM permanent_exclusions (no explicit permanent keepout intent).", "gray")

        dyn_rules = plan_json.get("dynamic_object_rules", []) or []
        self._latest_dynamic_rules = dyn_rules
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

        keepout_zones_det, softcost_det = self._split_intents_deterministic(text)

        immediate_keepouts: set[str] = set()

        if keepout_zones_det:
            dur_min = self._parse_duration_minutes(text)
            start_dt = self._parse_start_time(text)
            now_dt = dt.datetime.now(self._tz)

            if start_dt is None:
                start_dt = now_dt

            end_dt = start_dt + dt.timedelta(minutes=(dur_min if dur_min else self._default_forbid_minutes))

            start_is_future = start_dt > (now_dt + dt.timedelta(seconds=0.25))

            for z in keepout_zones_det:
                timed_rules.append({
                    "zone": z,
                    "time_condition": {
                        "start": start_dt.isoformat(),
                        "end": end_dt.isoformat(),
                        "repeat": None,
                    }
                })

            if not start_is_future:
                immediate_keepouts = set(keepout_zones_det)

            plain_exclusions = [z for z in plain_exclusions if z not in keepout_zones_det]

        keepout_requested = len(keepout_zones_det) > 0

        if not keepout_requested:
            plain_exclusions = []
            timed_rules = []

                                                       
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

                         
        def _extract_dest_wps(txt: str) -> List[str]:
            chunks = re.split(r"[,.。!?]\s*|\n+", txt or "")
            uDEST = [k.upper() for k in DEST_KWS]
            out: List[str] = []

            for ch in chunks:
                chu = ch.upper()
                if any(k in chu for k in uDEST):
                    wps = re.findall(r"\bWP\s*([1-8])\b", chu, flags=re.IGNORECASE)
                    for n in wps:
                        out.append(f"WP{int(n)}")

            return unique_preserve(out)

        dest_from_text = _extract_dest_wps(text)
        if dest_from_text:
            plan = [{"dest": z} for z in dest_from_text]

        conditional_requested = any(k in tlow for k in COND_KWS)
        cond_rules: List[Dict[str, Any]] = plan_json.get("conditional_rules", []) or []
        if not conditional_requested:
            cond_rules = []
        self._cond_rules_store = cond_rules[:]

        self._recalculate_and_publish_dynamic_keepouts(
            base_exclusions=exclusions,
            conditional_rules=cond_rules
        )

        weights_raw = [{"zone": z, "cost": c} for z, c in sorted(softcost_det.items())]

        weights: List[WeightedRule] = []
        for w in weights_raw:
            try:
                weights.append(WeightedRule(zone=str(w["zone"]).upper(), cost=int(w["cost"])))
            except Exception:
                continue
        if weights:
            self._publish_softcost(weights)

        wp_names = []
        for it in (plan or []):
            d = it.get("dest") if isinstance(it, dict) else None
            if isinstance(d, str) and d.strip().upper().startswith("WP"):
                wp_names.append(d.strip().upper())
        self._last_wp_names = wp_names[:]

        self._print_mission_snapshot(user_text=text, wp_names=wp_names)

        with self._sched_lock:
            timed_active = set().union(*self._active_timed.values()) if self._active_timed else set()
        cond_forbid, cond_allow, self._force_shortest = self._evaluate_conditional_rules(cond_rules)

        merged_keepouts = unique_preserve(
            self._permanent_exclusions + list(timed_active) + list(cond_forbid) + exclusions
        )
        if self._allow_overrides_permanent:
            merged_keepouts = [z for z in merged_keepouts if z not in cond_allow]
        else:
            merged_keepouts = [z for z in merged_keepouts if not (z in cond_allow and z not in self._permanent_exclusions)]

        #if self._force_shortest:
            #self._ui_print("🧭 state_condition applied: allow_shortest detected (keepout lifted + shortest-priority flag)", "cyan")

                        
        plan = self._filter_plan_by_keepouts(plan, merged_keepouts)

                   
        waypoints, names = self._waypoints_from_plan_with_names(plan)
        if not waypoints:
            self._ui_print("⚠️  No valid destination; ending mission.", "yellow")
            self._ui_print("=====================================================\n", "bold")
            return

                 
        self._ensure_navigator()

            
        result = self._follow_waypoints(waypoints, names)
        if result == TaskResult.SUCCEEDED:
            self._ui_print("🏁 Mission done: SUCCEEDED", "green")
        elif result == TaskResult.CANCELED:
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

                                                        
    def _plan_with_llm_or_fallback(self, text: str) -> dict:
        text = self._apply_aisle_aliases(text)
        fb = self._fallback_parse(text)
        fb = self._canonicalize_plan_obj(fb, text)

        if not (self._enable_llm and self._llm_endpoint):
            self._ui_print("ℹ️ LLM disabled → using fallback parser", "gray")
            return fb

        try:
            llm = self._call_llm(text)
        except Exception as e:
            self._ui_print(f"⚠️  LLM call failed → using fallback: {e}", "yellow")
            return fb

                                            
        latest = {w["zone"]: w for w in fb.get("weights", [])}
        for w in llm.get("weights", []):
            latest[w["zone"]] = w
        weights = [latest[k] for k in sorted(latest.keys())]

        def _uniq(seq):
            out, seen = [], set()
            for z in seq:
                if z not in seen:
                    out.append(z); seen.add(z)
            return out

        def _merge_rule_list(a, b):
            out = []
            seen = set()
            for r in (a or []) + (b or []):
                if not isinstance(r, dict):
                    continue
                k = json.dumps(r, sort_keys=True)
                if k not in seen:
                    out.append(r)
                    seen.add(k)
            return out

        exclusions = _merge_rule_list(fb.get("exclusions", []), llm.get("exclusions", []))
        perma = _uniq(fb.get("permanent_exclusions", []) + llm.get("permanent_exclusions", []))

        cand_plan = llm.get("plan") or fb.get("plan")

        dyn = []
        if isinstance(fb.get("dynamic_object_rules"), list):
            dyn += fb.get("dynamic_object_rules", [])
        if isinstance(llm.get("dynamic_object_rules"), list):
            dyn += llm.get("dynamic_object_rules", [])

                                     
        cond_rules = llm.get("conditional_rules", []) or fb.get("conditional_rules", [])

        tmp = {
            "plan": cand_plan,
            "exclusions": exclusions,
            "permanent_exclusions": perma,
            "weights": weights,
            "dynamic_object_rules": dyn,
            "conditional_rules": cond_rules,         
            "path_id": None,
        }
        return self._canonicalize_plan_obj(tmp, text)

    def _call_llm(self, text: str) -> dict:
        import urllib.request
        prompt = self._build_planner_prompt(text)

        options = {
            "temperature": 0.0,
            "top_p": 0.0,
            "seed": 42,
            "repeat_penalty": 1.0,
            "num_predict": 256,
        }

        payload = {
            "model": self._llm_model,
            "prompt": prompt,
            "stream": False,
            "format": "json",
            "options": options,
        }

        data = json.dumps(payload).encode("utf-8")
        req = urllib.request.Request(
            self._llm_endpoint,
            data=data,
            headers={
                "Content-Type": "application/json",
                "Authorization": f"Bearer {self._llm_key}" if self._llm_key else "",
            },
        )
        with urllib.request.urlopen(req, timeout=60) as resp:
            body = resp.read().decode("utf-8")

        try:
            obj = json.loads(body)
        except json.JSONDecodeError:
            m = re.search(r"\{[\s\S]*\}\s*$", body)
            obj = json.loads(m.group(0)) if m else {}

        if isinstance(obj, dict) and "response" in obj and isinstance(obj["response"], str):
            txt = obj["response"]
            m = re.search(r"\{[\s\S]*\}", txt)
            obj = json.loads(m.group(0)) if m else {}

        return self._canonicalize_plan_obj(obj, text)

    def _build_planner_prompt(self, text: str) -> str:
        labels = ", ".join(sorted(self._labels))
        return (
            "You are a deterministic robot mission planner.\n"
            f"Valid ZONE labels: [{labels}].\n"
            "Return ONLY a single JSON object (no markdown, no code fences).\n"
            "Schema: {\n"
            " \"plan\": [ {\"dest\": \"WP1\"} | {\"dest\":{\"x\":<num>,\"y\":<num>,\"yaw\":<num>}} , ... ],\n"
            " \"exclusions\": [ {\"zone\":<ZONE>, \"time_condition\": null | {\"start\":<iso>,\"end\":<iso>,\"repeat\":null|\"daily\"} } ... ],\n"
            " \"permanent_exclusions\": [<ZONE>...],\n"
            " \"weights\": [ {\"zone\":<ZONE>,\"cost\":<0..255>} ... ],\n"
            " \"dynamic_object_rules\": [ {\"class\":<string>, \"radius\":<m>} ... ],\n"
            " \"conditional_rules\": [ {\"zone\":<ZONE>, \"rules\":[\n"
            "    {\"priority\":<int>, \"state_condition\":\"default|fire_alarm|low_battery\", \"action\":\"forbid|allow|allow_shortest\"}, ...\n"
            " ]} ... ],\n"
            " \"path_id\": null\n"
            "}\n"
            "Hard rules:\n"
            "- Do not invent labels/coords.\n"
            "- Zones A..G are policy labels only; destinations must be WP1..WP8.\n"
            "- costs are ints 0..255.\n"
            "- Only output dynamic_object_rules if the user explicitly asks to avoid/detect objects (e.g., 'avoid people'). Otherwise return [].\n"
            "- Use conditional_rules when user says '평소/기본' + '하지만/다만/화재시/배터리 낮을때' etc.\n"
            "Example conditional_rules: {\"zone\":\"A\",\"rules\":["
            "{\"priority\":5,\"state_condition\":\"default\",\"action\":\"forbid\"},"
            "{\"priority\":10,\"state_condition\":\"fire_alarm\",\"action\":\"allow_shortest\"}]}\n"
            "Conditional policy rule:\n"
            "- If the user says like: \"A is forbidden, but if a fire is detected, you may take the shortest path through it\",\n"
            "  output conditional_rules for zone A exactly like:\n"
            "  [{\"zone\":\"A\",\"rules\":[\n"
            "     {\"priority\":5,\"state_condition\":\"default\",\"action\":\"forbid\"},\n"
            "     {\"priority\":10,\"state_condition\":\"fire_alarm\",\"action\":\"allow_shortest\"}\n"
            "  ]}]\n"
            "- The pronoun 'it' refers to the same zone mentioned earlier in the sentence.\n"
            "- Do not invent zones. (Note: aisle aliases like 'first aisle' are pre-mapped to A..E before you see the text.)\n"
            "User command: " + text + "\n"
        )

    def _canonicalize_plan_obj(self, obj: dict, user_text: str) -> dict:
        def _num(x, default=0.0):
            try: return float(x)
            except Exception: return float(default)

        def _int255(x, default=0):
            try: v = int(x)
            except Exception: v = int(default)
            return max(0, min(255, v))

        if not isinstance(obj, dict):
            obj = {}

        plan = obj.get("plan") if isinstance(obj.get("plan"), list) else []
        exclusions = obj.get("exclusions") if isinstance(obj.get("exclusions"), list) else []
        weights = obj.get("weights") if isinstance(obj.get("weights"), list) else []
        perma = obj.get("permanent_exclusions") if isinstance(obj.get("permanent_exclusions"), list) else []
        cond_in = obj.get("conditional_rules") if isinstance(obj.get("conditional_rules"), list) else []
        path_id = obj.get("path_id")

        labels_set = set(self._labels)

        def _lab(x):
            try:
                s = str(x).strip().upper()
            except Exception:
                return None
            return s if s in labels_set else None

                        
        norm_plan = []
        for it in (plan or []):
            if not isinstance(it, dict):
                continue
            d = it.get("dest")
            if isinstance(d, str):
                key = str(d).strip().upper()
                if hasattr(self, "_waypoints") and key in self._waypoints:
                    norm_plan.append({"dest": key})
                else:
                    lab = _lab(d)
                    if lab:
                        pass
            elif isinstance(d, dict):
                x = _num(d.get("x", 0.0))
                y = _num(d.get("y", 0.0))
                yaw = _num(d.get("yaw", 0.0))
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

        if isinstance(raw_excl, list):
            for e in raw_excl:
                rule = None
                if isinstance(e, str):
                    z = _lab(e)
                    if z:
                        rule = {"zone": z}
                elif isinstance(e, dict):
                    z = _lab(e.get("zone"))
                    if z:
                        rule = {"zone": z}
                        tc_norm = _norm_time_condition(e.get("time_condition"))
                        if tc_norm:
                            rule["time_condition"] = tc_norm

                if rule:
                    key = json.dumps(rule, sort_keys=True)
                    if key not in _seen:
                        norm_excl_rules.append(rule)
                        _seen.add(key)

                   
        norm_perm, seen = [], set()
        for e in perma:
            lab = _lab(e)
            if lab and lab not in seen:
                norm_perm.append(lab)
                seen.add(lab)

                                                
        latest = {}
        for w in (weights or []):
            if not isinstance(w, dict):
                continue
            z = _lab(w.get("zone"))
            if not z:
                continue
            c = _int255(w.get("cost", 0))
            latest[z] = {"zone": z, "cost": int(c)}
        norm_weights = [latest[k] for k in sorted(latest.keys())]

        weighted_zones = {w["zone"] for w in norm_weights}

        norm_excl_rules = [
            r for r in norm_excl_rules
            if not (isinstance(r, dict) and r.get("zone") in weighted_zones)
        ]

        norm_perm = [z for z in norm_perm if z not in weighted_zones]

                                        
        raw_dyn = obj.get("dynamic_object_rules")
        norm_dyn: List[Dict[str, Any]] = []
        ALLOWED_CLASSES = {"person", "forklift"}

        if isinstance(raw_dyn, list):
            for r in raw_dyn:
                if not isinstance(r, dict):
                    continue
                cls = r.get("class")
                rad = r.get("radius")
                if not isinstance(cls, str):
                    continue
                cls_n = cls.strip().lower()
                if cls_n not in ALLOWED_CLASSES:
                    continue
                try:
                    rad_f = float(rad)
                except Exception:
                    continue
                rad_f = max(0.1, min(10.0, rad_f))
                norm_dyn.append({"class": cls_n, "radius": rad_f})

                                     
        norm_cond: Dict[str, List[Dict[str, Any]]] = {}
        ALLOWED_CONDS = {"default", "fire_alarm", "low_battery"}
        ALLOWED_ACTS = {"forbid", "allow", "allow_shortest"}

        if isinstance(cond_in, list):
            for item in cond_in:
                if not isinstance(item, dict):
                    continue
                z = _lab(item.get("zone"))
                if not z:
                    continue
                rules_out: List[Dict[str, Any]] = []
                for rr in (item.get("rules") or []):
                    if not isinstance(rr, dict):
                        continue
                    cond = str(rr.get("state_condition", rr.get("condition", "default"))).strip().lower()
                    if cond not in ALLOWED_CONDS:
                        cond = "default"
                    act = str(rr.get("action", "forbid")).strip().lower()
                    if act not in ALLOWED_ACTS:
                        act = "forbid"
                    try:
                        prio = int(rr.get("priority", 0))
                    except Exception:
                        prio = 0
                    rules_out.append({"priority": prio, "state_condition": cond, "action": act})

                rules_out.sort(key=lambda x: x["priority"], reverse=True)
                if rules_out:
                    norm_cond[z] = rules_out

                                   
        weighted_zones = {w["zone"] for w in norm_weights}
        excl_zones = {r["zone"] for r in norm_excl_rules if isinstance(r, dict) and "zone" in r}
        excl_all = excl_zones | set(norm_perm)

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
            "permanent_exclusions": norm_perm,
            "weights": norm_weights,
            "dynamic_object_rules": norm_dyn,
            "conditional_rules": [
                {"zone": z, "rules": norm_cond[z]}
                for z in sorted(norm_cond.keys())
            ],
            "path_id": (int(path_id) if isinstance(path_id, (int, float)) else None),
        }
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
                labs = [c for c in ZONE_LABEL_RE.findall(chu) if c in self._labels]
                labels.extend(labs)
        labels = unique_preserve(labels)
        return [{"dest": z} for z in labels] if labels else plan

                                                       
    def _fallback_parse(self, text: str) -> Dict[str, Any]:
        t = (text or "").strip()
        tu = t.upper()

                                         
        weights: List[Dict[str, Any]] = []
        for m in WEIGHT_CMD_RE.finditer(tu):
            z, c = m.group(1), int(m.group(3))
            weights.append({"zone": z, "cost": max(0, min(255, c))})
        for m in WEIGHT_CMD_RE2.finditer(tu):
            z, c = m.group(1), int(m.group(2))
            weights.append({"zone": z, "cost": max(0, min(255, c))})

        weight_zones = {w["zone"] for w in weights}

        chunks = re.split(r"[,.。!?]\s*|\n+", t)
        dest_labels: List[str] = []
        tmp_exclusions: List[str] = []
        perm_exclusions: List[str] = []
        tl = t.lower()

        dynamic_object_rules: List[Dict[str, Any]] = []
        if ("사람 피해" in t) or ("사람 피해서" in t) or ("avoid people" in tl):
            dynamic_object_rules.append({"class": "person", "radius": 1.5})

        def labels_in(s: str) -> List[str]:
            return [c for c in ZONE_LABEL_RE.findall(s.upper()) if c in self._labels]

        uDEST = [k.upper() for k in DEST_KWS]
        uEXC_TMP = [k.upper() for k in EXC_KWS_TMP]
        uEXC_PERM = [k.upper() for k in EXC_KWS_PERM]

        perm_cost_zones: set[str] = set()

        for ch in chunks:
            chu = ch.upper()
            labs = labels_in(ch)
            if not labs:
                continue

            if _is_perm_cost_sentence(ch):
                perm_cost_zones.update(labs)

            is_perm = any(kw in chu for kw in uEXC_PERM)
            is_exc  = any(kw in chu for kw in uEXC_TMP)
            is_dest = any(kw in chu for kw in uDEST)
            is_cost = ("비용" in chu) or ("COST" in chu)

            if is_exc and is_perm:
                perm_exclusions.extend(labs)
            elif is_exc:
                tmp_exclusions.extend(labs)
            elif is_dest:
                dest_labels.extend(labs)
            elif is_cost:
                continue
            else:
                dest_labels.extend(labs)

        excl_all = set(unique_preserve(tmp_exclusions + perm_exclusions))
        dest_labels = [z for z in unique_preserve(dest_labels) if z not in excl_all]
        dest_labels = [z for z in dest_labels if z not in weight_zones]

        plan = [{"dest": z} for z in dest_labels]

                                                 
                                      

        return {
            "plan": plan,
            "exclusions": [{"zone": z} for z in unique_preserve(tmp_exclusions)],
            "permanent_exclusions": unique_preserve(perm_exclusions),
            "weights": weights,
            "dynamic_object_rules": dynamic_object_rules,
            "conditional_rules": [],
            "path_id": None,
            "perm_cost_zones": list(perm_cost_zones),
        }

                                                          
    def _publish_forbidden(self, zones: List[str]):
        zones = [z for z in zones if z in self._labels]
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

    def _publish_softcost(self, rules: List[WeightedRule]):
        zone_costs: Dict[str, int] = {}
        for r in rules:
            z = str(r.zone).upper()
            if z and len(z) == 1 and z.isalpha():
                zone_costs[z] = int(r.cost)

        msg = RosString()
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
            self._schedules[sid] = tk

                    
        end_wall = tk.end_wall or (tk.start_wall + dt.timedelta(minutes=self._default_forbid_minutes))

                                  
        start_ros = self._wall_dt_to_ros_time(tk.start_wall)
        end_ros = self._wall_dt_to_ros_time(end_wall)
        tk.start_ros_ns = int(start_ros.nanoseconds)
        tk.end_ros_ns = int(end_ros.nanoseconds)

        def _on_start():
            with self._sched_lock:
                self._active_timed[sid] = set([z for z in tk.zones if z in self._labels])
                                 
            self._recalculate_and_publish_dynamic_keepouts(
                base_exclusions=self._last_base_exclusions,
                conditional_rules=self._cond_rules_store
            )

        def _on_end():
            with self._sched_lock:
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

                  
        if delay_start <= 0.25:
            _on_start()
        else:
            self._oneshot_ros_timer(delay_start, _on_start)

                
        if delay_end <= 0.0:
            _on_end()
        else:
            self._oneshot_ros_timer(delay_end, _on_end)

        return sid

                                            
    def _ensure_navigator(self):
        if self._nav is not None:
            return
        self._nav = BasicNavigator()
        self._ui_print("🧭 BasicNavigator ready.", "cyan")

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

    def _follow_waypoints(self, waypoints: List[PoseStamped], names: List[str]) -> TaskResult:
        assert self._nav is not None
        self._nav.waitUntilNav2Active()
        self._nav.followWaypoints(waypoints)

        last_idx = -1
        while not self._nav.isTaskComplete():
            fbk = self._nav.getFeedback()
            if fbk and hasattr(fbk, "current_waypoint"):
                idx = int(fbk.current_waypoint)
                if idx != last_idx and 0 <= idx < len(waypoints):
                    wp = waypoints[idx].pose.position
                    lab = names[idx] if idx < len(names) else f"({wp.x:.1f},{wp.y:.1f})"
                    dist = self._dist_to_pose(waypoints[idx])
                    self._ui_print(f"▶️  Progress: WP {idx+1}/{len(waypoints)} → {lab}   (remaining≈{dist:.2f} m)", "blue")
                    last_idx = idx
            time.sleep(0.05)

        res = self._nav.getResult()
        if res == TaskResult.SUCCEEDED:
            self._ui_print("✅ All waypoints reached", "green")
        return res

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
                    self._ui_print(f"⚠️ Destination must be WP1..WP8. Ignoring dest='{key}'", "yellow")
            elif isinstance(dest, dict):
                x = float(dest.get("x", 0.0))
                y = float(dest.get("y", 0.0))
                yaw = float(dest.get("yaw", 0.0))
                wps.append(self._make_pose(x, y, yaw))
                names.append(f"({x:.1f},{y:.1f})")
        return wps, names

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
        if not self.bridge:
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
        target_period = 0.08        
        while rclpy.ok() and getattr(self, "_yolo_run", False):
            t0 = time.monotonic()

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

    def _publish_objects(self, frame: str, positions_xy: List[Tuple[float, float]]):
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

        msg = PoseArray()
        msg.header.frame_id = frame
        msg.header.stamp = self.get_clock().now().to_msg()
        for (x, y) in positions_xy:
            p = Pose()
            p.position.x = float(x)
            p.position.y = float(y)
            p.orientation.w = 1.0
            msg.poses.append(p)
        self.object_pub.publish(msg)

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
                        "Helper commands: status | keepouts | objects | quit",
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

                with self._mission_lock:
                    self._mission_queue.append(RosString(data=s))
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
        rclpy.shutdown()

if __name__ == "__main__":
    main()