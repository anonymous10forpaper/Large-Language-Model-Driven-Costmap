


from __future__ import annotations

import argparse
import copy
import datetime as dt
import html
import json
from pathlib import Path
import re
import sys
import threading
import time
from typing import Any, Callable, Dict, List, Optional, Tuple
import urllib.error
import urllib.parse
import urllib.request

from PyQt5.QtCore import QObject, QSettings, Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QCloseEvent, QFont, QKeyEvent, QPixmap
from PyQt5.QtWidgets import (
    QAbstractItemView,
    QApplication,
    QCheckBox,
    QComboBox,
    QFileDialog,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QMainWindow,
    QMenu,
    QMessageBox,
    QPlainTextEdit,
    QProgressBar,
    QPushButton,
    QScrollArea,
    QSpinBox,
    QSplitter,
    QStackedWidget,
    QStyle,
    QTabWidget,
    QTableWidget,
    QTableWidgetItem,
    QTextEdit,
    QToolButton,
    QVBoxLayout,
    QWidget,
)
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseArray
from nav2_msgs.action import FollowWaypoints
from nav2_simple_commander.robot_navigator import TaskResult
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32
from std_msgs.msg import String as RosString

from policy_bridge.gui_theme import APP_STYLESHEET
from policy_bridge.policybridge import PolicyBridgeNode
from policy_bridge.waypoint_map import (
    DEFAULT_FACTORY_ZONES,
    DEFAULT_FACTORY_WAYPOINTS,
    RosMapSpec,
    Waypoint,
    ZoneBounds,
    discover_ros_maps,
    load_map_annotations,
    load_ros_map_yaml,
    normalize_waypoints,
    normalize_zones,
    save_waypoint_config,
    waypoint_config_path,
)
from policy_bridge.waypoint_map_gui import WaypointMapPage


EXAMPLE_COMMANDS: List[Tuple[str, str]] = [
    ("Choose an example", ""),
    ("Keepout and destination", "Forbid zone A and go to WP2."),
    (
        "Differential zone costs",
        "Set the cost of zone B to 200 and zone A to 5, and go to WP3.",
    ),
    ("Person-aware navigation", "Go to WP8 and avoid people within 1.5 m."),
    (
        "Emergency priority",
        (
            "Go to WP2, and zone A is forbidden, but if the fire alarm "
            "goes off, allow shortest-path traversal."
        ),
    ),
    (
        "Timed keepout",
        "From now on, forbid zone A for 30 minutes and go to WP2.",
    ),
]

STAGE_LABELS = [
    "Command received",
    "Language interpreted",
    "Policy validated",
    "Costmap updated",
    "Navigation running",
    "Mission complete",
]

LOG_COLORS = {
    "red": "#b43d3d",
    "green": "#2f8157",
    "yellow": "#9a6816",
    "blue": "#2f69bd",
    "magenta": "#7451a6",
    "cyan": "#27758a",
    "gray": "#6f7b8b",
    "bold": "#202936",
    "": "#3a4655",
}

DEFAULT_OLLAMA_ENDPOINT = "http://localhost:11434/api/generate"
PREFERRED_GUI_MODELS = (
    "qwen3.5:9b",
    "qwen3:8b",
    "qwen2.5:14b",
    "qwen2.5:7b",
    "llama3.1:latest",
)


def _set_object_name(widget: QWidget, name: str) -> None:

    if widget.objectName() == name:
        return
    widget.setObjectName(name)
    widget.style().unpolish(widget)
    widget.style().polish(widget)
    widget.update()


def _clear_layout(layout: QHBoxLayout) -> None:

    while layout.count():
        item = layout.takeAt(0)
        widget = item.widget()
        if widget is not None:
            widget.deleteLater()


def _pretty_json(value: Any) -> str:

    if value in (None, {}, []):
        return "{}"
    try:
        return json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True)
    except (TypeError, ValueError):
        return str(value)


def _ollama_tags_url(generation_endpoint: str) -> str:

    endpoint = (generation_endpoint or "").strip()
    if not endpoint:
        raise ValueError("LLM endpoint is not configured")
    parsed = urllib.parse.urlsplit(endpoint)
    if not parsed.scheme or not parsed.netloc:
        raise ValueError("LLM endpoint must be an HTTP URL")

    path = parsed.path.rstrip("/")
    if "/api/" in path:
        prefix = path.split("/api/", 1)[0]
    elif path.endswith("/api"):
        prefix = path[:-4]
    else:
        prefix = path
    tags_path = f"{prefix.rstrip('/')}/api/tags"
    return urllib.parse.urlunsplit(
        (parsed.scheme, parsed.netloc, tags_path, "", "")
    )


def _fetch_ollama_models(
    generation_endpoint: str,
    api_key: str = "",
    timeout_s: float = 3.0,
) -> List[str]:

    headers = {"Accept": "application/json"}
    if api_key:
        headers["Authorization"] = f"Bearer {api_key}"
    request = urllib.request.Request(
        _ollama_tags_url(generation_endpoint), headers=headers
    )
    with urllib.request.urlopen(request, timeout=timeout_s) as response:
        payload = json.loads(response.read().decode("utf-8"))

    models = payload.get("models", []) if isinstance(payload, dict) else []
    names: List[str] = []
    for model in models:
        if not isinstance(model, dict):
            continue
        name = str(model.get("name") or model.get("model") or "").strip()
        if name and name not in names:
            names.append(name)
    return names


def _ordered_model_inventory(models: List[str]) -> List[str]:

    installed = {
        str(model).strip() for model in models if str(model).strip()
    }
    preferred = [
        model for model in PREFERRED_GUI_MODELS if model in installed
    ]
    remaining = sorted(
        installed.difference(preferred), key=lambda value: value.casefold()
    )
    return preferred + remaining


def _model_locations(
    endpoints: List[str],
    api_key: str = "",
    fetcher: Callable[..., List[str]] = _fetch_ollama_models,
) -> Tuple[Dict[str, str], List[str]]:

    locations: Dict[str, str] = {}
    errors: List[str] = []
    unique_endpoints = list(
        dict.fromkeys(endpoint.strip() for endpoint in endpoints if endpoint)
    )
    for endpoint in unique_endpoints:
        try:
            installed = fetcher(endpoint, api_key)
        except (
            OSError,
            ValueError,
            json.JSONDecodeError,
            urllib.error.URLError,
        ) as exc:
            errors.append(f"{endpoint}: {exc}")
            continue
        for model in _ordered_model_inventory(installed):
            locations.setdefault(model, endpoint)
    ordered = {
        model: locations[model]
        for model in _ordered_model_inventory(list(locations))
    }
    return ordered, errors


class GuiSignals(QObject):


    log_event = pyqtSignal(str, str)
    models_event = pyqtSignal(object, str)

    def __init__(self):
        super().__init__()
        self._lock = threading.Lock()
        self._ready = False
        self._backlog: List[Tuple[str, str]] = []

    def publish(self, message: str, color: str) -> None:

        with self._lock:
            if not self._ready:
                self._backlog.append((message, color))
                return
        self.log_event.emit(message, color)

    def activate(self) -> List[Tuple[str, str]]:

        with self._lock:
            self._ready = True
            backlog = list(self._backlog)
            self._backlog.clear()
        return backlog


def _normalized_robot_namespace(namespace: str) -> str:

    return str(namespace or "").strip().strip("/")


def discover_robot_namespaces(topic_names: List[str]) -> List[str]:

    discovered = set()
    for topic in topic_names:
        match = re.fullmatch(
            r"/(.+)/global_costmap/(?:costmap|costmap_raw)",
            str(topic),
        )
        if match:
            namespace = _normalized_robot_namespace(match.group(1))
            if namespace:
                discovered.add(namespace)

    def sort_key(value: str) -> Tuple[Any, ...]:
        return tuple(
            int(part) if part.isdigit() else part.lower()
            for part in re.split(r"(\d+)", value)
        )

    return sorted(discovered, key=sort_key)


class ExecutorManagedNavigator:







    def __init__(
        self,
        action_client: ActionClient,
        *,
        server_timeout_s: float = 10.0,
    ):
        self._client = action_client
        self._server_timeout_s = max(0.1, float(server_timeout_s))
        self._lock = threading.RLock()
        self._goal_handle = None
        self._result_future = None
        self._cancel_future = None
        self._feedback = None
        self._status = GoalStatus.STATUS_UNKNOWN

    @staticmethod
    def _wait_for_future(future: Any, timeout_s: float) -> bool:
        deadline = time.monotonic() + max(0.0, float(timeout_s))
        while rclpy.ok() and time.monotonic() < deadline:
            if future.done():
                return True
            time.sleep(0.01)
        return bool(future.done())

    def waitUntilNav2Active(self) -> None:

        if self._client.server_is_ready():
            return
        if not self._client.wait_for_server(
            timeout_sec=self._server_timeout_s
        ):
            raise RuntimeError("Nav2 follow_waypoints action is unavailable")

    def _on_feedback(self, message: Any) -> None:
        with self._lock:
            self._feedback = message.feedback

    def followWaypoints(self, waypoints: List[Any]) -> bool:

        self.waitUntilNav2Active()
        goal = FollowWaypoints.Goal()
        goal.poses = list(waypoints)
        with self._lock:
            self._goal_handle = None
            self._result_future = None
            self._cancel_future = None
            self._feedback = None
            self._status = GoalStatus.STATUS_UNKNOWN

        send_future = self._client.send_goal_async(
            goal, feedback_callback=self._on_feedback
        )
        if not self._wait_for_future(send_future, self._server_timeout_s):
            return False
        try:
            goal_handle = send_future.result()
        except Exception:
            return False
        if goal_handle is None or not goal_handle.accepted:
            return False

        with self._lock:
            self._goal_handle = goal_handle
            self._result_future = goal_handle.get_result_async()
        return True

    def isTaskComplete(self) -> bool:
        with self._lock:
            future = self._result_future
        if future is None:
            return True
        if not future.done():
            return False
        try:
            wrapped_result = future.result()
            status = int(wrapped_result.status)
        except Exception:
            status = GoalStatus.STATUS_ABORTED
        with self._lock:
            self._status = status
        return True

    def getFeedback(self) -> Any:
        with self._lock:
            return self._feedback

    def getResult(self) -> TaskResult:
        with self._lock:
            status = self._status
        if status == GoalStatus.STATUS_SUCCEEDED:
            return TaskResult.SUCCEEDED
        if status == GoalStatus.STATUS_CANCELED:
            return TaskResult.CANCELED
        if status == GoalStatus.STATUS_ABORTED:
            return TaskResult.FAILED
        return TaskResult.UNKNOWN

    def getRawStatus(self) -> int:
        with self._lock:
            return int(self._status)

    def cancelTask(self) -> None:
        with self._lock:
            goal_handle = self._goal_handle
        if goal_handle is None:
            return
        try:
            cancel_future = goal_handle.cancel_goal_async()
        except Exception:
            return
        with self._lock:
            self._cancel_future = cancel_future


def _multi_robot_parameter_overrides(namespace: str) -> List[Parameter]:

    robot = _normalized_robot_namespace(namespace)
    prefix = f"/{robot}"
    output_prefix = f"{prefix}/policy_bridge/fleet_output"
    return [
        Parameter(
            "nl_command_topic", value=f"{prefix}/policy_bridge/nl_command"
        ),
        Parameter(
            "manual_policy_topic",
            value=f"{prefix}/policy_bridge/manual_policy_json",
        ),
        Parameter(
            "validated_policy_topic",
            value=f"{prefix}/policy_bridge/validated_policy",
        ),
        Parameter("event_topic", value=f"{prefix}/policy_bridge/event"),
        Parameter(
            "policy_reset_topic", value=f"{prefix}/policy_bridge/reset"
        ),
        Parameter(
            "forbidden_zones_topic",
            value=f"{output_prefix}/forbidden_zones_update",
        ),
        Parameter(
            "object_positions_topic",
            value=f"{output_prefix}/object_world_positions",
        ),
        Parameter(
            "object_avoidance_radius_topic",
            value=f"{output_prefix}/object_avoidance_radius",
        ),
        Parameter(
            "zone_cost_overrides_topic",
            value=f"{output_prefix}/zone_cost_overrides",
        ),
        Parameter("zone_geometry_topic", value="/zone_geometry_update"),
        Parameter("initial_pose_topic", value=f"{prefix}/initialpose"),
        Parameter("debug_image_topic", value=f"{prefix}/yolo/debug_image"),
        Parameter("camera_info_topic", value=f"{prefix}/camera/camera_info"),
        Parameter("camera_topic", value=f"{prefix}/camera/image_raw"),
        Parameter(
            "depth_topic", value=f"{prefix}/camera/depth/image_raw"
        ),
        Parameter(
            "depth_info_topic",
            value=f"{prefix}/camera/depth/camera_info",
        ),
        Parameter(
            "follow_waypoints_action", value=f"{prefix}/follow_waypoints"
        ),
        Parameter("navigator_namespace", value=robot),
        Parameter("set_initial_pose", value=False),
        Parameter("require_nav2", value=False),
        Parameter("enable_yolo", value=False),
    ]


class GuiPolicyBridgeNode(PolicyBridgeNode):


    def __init__(
        self,
        signals: GuiSignals,
        *,
        robot_namespace: str = "",
        parameter_overrides: Optional[List[Parameter]] = None,
        fleet_epoch_factory: Optional[
            Callable[
                [str, Dict[str, Any], Dict[str, Any], bool], int
            ]
        ] = None,
        fleet_policy_sink: Optional[
            Callable[[str, str, Any, int], None]
        ] = None,
    ):
        self._gui_signals = signals
        self._gui_robot_namespace = _normalized_robot_namespace(
            robot_namespace
        )
        self._gui_fleet_epoch_factory = fleet_epoch_factory
        self._gui_fleet_policy_sink = fleet_policy_sink
        self._gui_fleet_epoch = 0
        overrides: Dict[str, Parameter] = {}
        if self._gui_robot_namespace:
            overrides.update(
                {
                    item.name: item
                    for item in _multi_robot_parameter_overrides(
                        self._gui_robot_namespace
                    )
                }
            )
        for item in parameter_overrides or []:
            overrides[item.name] = item
        cli_args = None
        if self._gui_robot_namespace:
            prefix = f"/{self._gui_robot_namespace}"
            cli_args = [
                "--ros-args",
                "-r",
                f"/tf:={prefix}/tf",
                "-r",
                f"/tf_static:={prefix}/tf_static",
            ]
        super().__init__(
            parameter_overrides=list(overrides.values()) or None,
            node_name="policy_bridge",
            namespace=self._gui_robot_namespace,
            cli_args=cli_args,
        )
        self.declare_parameter("gui_llm_endpoint", DEFAULT_OLLAMA_ENDPOINT)
        self.declare_parameter(
            "gui_map_yaml", str(Path.home() / "map" / "factory6.yaml")
        )
        self._llm_endpoint = str(
            self.get_parameter("gui_llm_endpoint").value
        )
        self.set_parameters(
            [Parameter("llm_endpoint", value=self._llm_endpoint)]
        )
        self._gui_map_yaml = str(self.get_parameter("gui_map_yaml").value)
        self._gui_model_refresh_lock = threading.Lock()
        self._gui_waypoint_lock = threading.RLock()
        self._gui_zone_lock = threading.RLock()
        self._gui_available_models: List[str] = []
        self._gui_model_endpoints: Dict[str, str] = {}
        self._gui_command_pub = self.create_publisher(
            RosString, self._nl_topic, 10
        )
        self._gui_event_pub = self.create_publisher(
            RosString, self._event_topic, 10
        )
        self._gui_reset_pub = self.create_publisher(
            RosString, self._reset_topic, 10
        )
        action_name = str(
            self.get_parameter("follow_waypoints_action").value
        )
        self._gui_follow_waypoints_action = action_name
        self._gui_nav2_client = ActionClient(
            self, FollowWaypoints, action_name
        )
        self._gui_nav2_monitor_stop = threading.Event()
        self._gui_nav2_misses = 0
        self._gui_nav2_monitor_thread = threading.Thread(
            target=self._monitor_nav2,
            name="policy-bridge-nav2-monitor",
            daemon=True,
        )
        self._gui_nav2_monitor_thread.start()

    def _commit_policy_snapshot(
        self, update: Dict[str, Any]
    ) -> Dict[str, Any]:

        requested_update = copy.deepcopy(update)
        committed = super()._commit_policy_snapshot(update)
        epoch_factory = getattr(self, "_gui_fleet_epoch_factory", None)
        if self._gui_robot_namespace and callable(epoch_factory):
            self._gui_fleet_epoch = int(
                epoch_factory(
                    self._gui_robot_namespace,
                    copy.deepcopy(committed),
                    requested_update,
                    bool(
                        getattr(
                            self,
                            "_policy_commit_replaces_previous",
                            False,
                        )
                    ),
                )
            )
        return committed

    def _clear_mission_scoped_fleet_keepouts(self) -> bool:






        if not self._gui_robot_namespace:
            return False
        had_temporary_policy = bool(
            getattr(self, "_last_base_exclusions", [])
            or getattr(self, "_cond_rules_store", [])
        )
        self._last_base_exclusions = []
        self._cond_rules_store = []
        self._force_shortest = False
        self._recalculate_and_publish_dynamic_keepouts([], [])
        return had_temporary_policy

    def _run_single_mission(self, *args: Any, **kwargs: Any) -> Any:
        version_before = self._policy_version
        try:
            return super()._run_single_mission(*args, **kwargs)
        finally:
            if (
                self._policy_version != version_before
                and self._clear_mission_scoped_fleet_keepouts()
            ):
                self._ui_print(
                    "Mission end -> cleared temporary fleet keepouts",
                    "magenta",
                )

    def _publish_fleet_value(self, field: str, value: Any) -> None:
        sink = getattr(self, "_gui_fleet_policy_sink", None)
        if self._gui_robot_namespace and callable(sink):
            sink(
                self._gui_robot_namespace,
                field,
                copy.deepcopy(value),
                int(getattr(self, "_gui_fleet_epoch", 0)),
            )

    def _publish_forbidden(self, zones: List[str]) -> None:
        super()._publish_forbidden(zones)
        active = [zone for zone in zones if zone in self._zone_db]
        self._publish_fleet_value("forbidden", active)

    def _publish_softcost(self, rules: List[Any]) -> None:
        super()._publish_softcost(rules)
        self._publish_fleet_value(
            "soft_costs", dict(getattr(self, "_active_softcost", {}))
        )

    def _clear_softcost(self) -> None:
        super()._clear_softcost()
        self._publish_fleet_value("soft_costs", {})

    def _publish_object_avoidance_radius(
        self, dynamic_rules: List[Dict[str, Any]]
    ) -> None:
        super()._publish_object_avoidance_radius(dynamic_rules)
        self._publish_fleet_value(
            "object_radius",
            float(getattr(self, "_active_object_avoidance_radius", 0.0)),
        )

    def _publish_objects(
        self,
        frame: str,
        positions_xy: List[Tuple[float, float]],
        *,
        force_clear: bool = False,
    ) -> None:

        super()._publish_objects(
            frame,
            positions_xy,
            force_clear=force_clear,
        )
        published = list(
            getattr(self, "_last_pub_by_frame", {}).get(frame, [])
        )
        self._publish_fleet_value(
            "object_positions",
            {"frame": frame, "positions": published},
        )

    def _ensure_navigator(self) -> None:

        if self._nav is not None:
            return
        if not self._gui_robot_namespace:
            super()._ensure_navigator()
            return
        self._nav = ExecutorManagedNavigator(self._gui_nav2_client)
        self._ui_print(
            "Nav2 action client ready for namespace "
            f"/{self._gui_robot_namespace}.",
            "cyan",
        )

    def _ui_print(self, msg: str, color: Optional[str] = None):

        signal = getattr(self, "_gui_signals", None)
        if signal is not None:
            signal.publish(str(msg), color or "")

    def _set_nav2_availability(self, available: bool) -> None:

        available = bool(available)
        previous = bool(getattr(self, "_nav2_ready", False))
        if available == previous:
            return

        self._nav2_ready = available
        if available:
            action_name = str(
                getattr(
                    self,
                    "_gui_follow_waypoints_action",
                    "/follow_waypoints",
                )
            )
            self._ui_print(
                f"Nav2 connected: {action_name} is available.", "green"
            )
            if self._set_initial_pose:
                self._initial_pose_applied = False
                self._apply_initial_pose_param_once()
            self._start_runtime_workers()
        else:
            self._ui_print(
                "Nav2 disconnected: waiting for /follow_waypoints.",
                "yellow",
            )

    def _monitor_nav2(self) -> None:

        while rclpy.ok() and not self._gui_nav2_monitor_stop.is_set():
            try:
                available = self._gui_nav2_client.server_is_ready()
                if not available:
                    available = self._gui_nav2_client.wait_for_server(
                        timeout_sec=0.25
                    )
            except Exception:
                available = False

            if available:
                self._gui_nav2_misses = 0
                self._set_nav2_availability(True)
            else:
                self._gui_nav2_misses += 1
                if self._gui_nav2_misses >= 3:
                    self._set_nav2_availability(False)
            self._gui_nav2_monitor_stop.wait(0.75)

    def stop_gui_monitor(self) -> None:

        self._gui_nav2_monitor_stop.set()
        monitor = getattr(self, "_gui_nav2_monitor_thread", None)
        if monitor is not None and monitor.is_alive():
            monitor.join(timeout=1.0)

    def submit_from_gui(self, command: str) -> bool:

        command = (command or "").strip()
        if not command or not getattr(self, "_nav2_ready", False):
            return False
        message = RosString()
        message.data = command
        self._gui_command_pub.publish(message)
        return True

    def reset_from_gui(self) -> bool:

        if (
            getattr(self, "_mission_active", threading.Event()).is_set()
            or getattr(
                self, "_interpretation_active", threading.Event()
            ).is_set()
        ):
            return False
        message = RosString()
        message.data = "reset"
        self._gui_reset_pub.publish(message)
        return True

    def cancel_from_gui(self) -> bool:

        return self.request_cancel()

    def toggle_pause_from_gui(self) -> bool:

        if getattr(self, "_mission_paused", threading.Event()).is_set():
            return self.request_resume()
        return self.request_pause()

    def set_fire_alarm_from_gui(self, enabled: bool) -> None:

        message = RosString()
        message.data = "fire alarm on" if enabled else "fire alarm off"
        self._gui_event_pub.publish(message)

    def set_battery_from_gui(self, percentage: int) -> None:

        message = RosString()
        message.data = f"battery {max(0, min(100, int(percentage)))}%"
        self._gui_event_pub.publish(message)

    def request_model_list_from_gui(self) -> bool:

        if not self._gui_model_refresh_lock.acquire(blocking=False):
            return False

        primary_endpoint = str(getattr(self, "_llm_endpoint", ""))
        api_key = str(getattr(self, "_llm_key", ""))

        def worker() -> None:
            models: List[str] = []
            error = ""
            try:
                endpoints = [primary_endpoint, DEFAULT_OLLAMA_ENDPOINT]
                locations, errors = _model_locations(
                    endpoints, api_key
                )
                models = list(locations)
                self._gui_model_endpoints = dict(locations)
                self._gui_available_models = list(models)
                if not models and errors:
                    error = "; ".join(errors)
            except Exception as exc:
                error = str(exc)
            finally:
                self._gui_model_refresh_lock.release()
            self._gui_signals.models_event.emit(models, error)

        threading.Thread(
            target=worker,
            name="policy-bridge-model-list",
            daemon=True,
        ).start()
        return True

    def select_model_from_gui(self, model: str) -> Tuple[bool, str]:

        model = (model or "").strip()
        if not model:
            return False, "Model name is empty"
        available = list(getattr(self, "_gui_available_models", []))
        if available and model not in available:
            return False, f"Model is not installed in Ollama: {model}"
        endpoint = getattr(self, "_gui_model_endpoints", {}).get(
            model, getattr(self, "_llm_endpoint", DEFAULT_OLLAMA_ENDPOINT)
        )
        if (
            model == getattr(self, "_llm_model", "")
            and endpoint == getattr(self, "_llm_endpoint", "")
        ):
            return True, ""

        try:
            results = self.set_parameters(
                [
                    Parameter("llm_model", value=model),
                    Parameter("llm_endpoint", value=endpoint),
                ]
            )
        except Exception as exc:
            return False, str(exc)
        failed = next(
            (
                result
                for result in results
                if not getattr(result, "successful", False)
            ),
            None,
        )
        if failed is not None:
            return False, failed.reason or "ROS parameter update failed"

        self._llm_model = model
        self._llm_endpoint = endpoint
        self._ui_print(
            (
                f"Active LLM model: {model} via "
                f"{urllib.parse.urlsplit(endpoint).netloc}. "
                "New commands will use it."
            ),
            "green",
        )
        return True, ""

    def waypoints_from_gui(self) -> Dict[str, Waypoint]:

        with self._gui_waypoint_lock:
            return copy.deepcopy(self._waypoints)

    def replace_waypoints_from_gui(
        self, waypoints: Dict[str, Waypoint]
    ) -> Tuple[bool, str]:

        try:
            normalized = normalize_waypoints(waypoints)
        except ValueError as exc:
            return False, str(exc)
        with self._gui_waypoint_lock:
            self._waypoints = dict(normalized)
        return True, ""

    def zones_from_gui(self) -> Dict[str, ZoneBounds]:

        with self._gui_zone_lock:
            return {
                name: ZoneBounds(
                    bounds.x_min,
                    bounds.y_min,
                    bounds.x_max,
                    bounds.y_max,
                )
                for name, bounds in self._zone_db.items()
            }

    def ros_context_is_active(self) -> bool:

        try:
            context = self.context
            return bool(context.ok())
        except Exception:
            return False

    def replace_zones_from_gui(
        self, zones: Dict[str, ZoneBounds]
    ) -> Tuple[bool, str]:

        if not self.ros_context_is_active():
            return (
                False,
                "ROS 2 communication has stopped. Close and restart "
                "Policy Bridge before editing policy zones.",
            )
        if (
            getattr(self, "_mission_active", threading.Event()).is_set()
            or getattr(
                self, "_interpretation_active", threading.Event()
            ).is_set()
        ):
            return (
                False,
                "Wait for the active command to finish before editing zones",
            )
        try:
            normalized = normalize_zones(zones)
        except ValueError as exc:
            return False, str(exc)

        active = set(getattr(self, "_active_softcost", {}))
        active.update(getattr(self, "_last_forbidden_windows", {}))
        missing = sorted(active.difference(normalized))
        if missing:
            return (
                False,
                "Reset the active policy before removing Zone "
                + ", ".join(missing),
            )

        labels = list(normalized)
        database_json = json.dumps(
            {
                name: list(bounds.as_tuple())
                for name, bounds in normalized.items()
            },
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
        )
        try:
            results = self.set_parameters(
                [
                    Parameter(
                        "zone_labels",
                        Parameter.Type.STRING_ARRAY,
                        labels,
                    ),
                    Parameter("zone_database_json", value=database_json),
                ]
            )
        except Exception as exc:
            if not self.ros_context_is_active():
                return (
                    False,
                    "ROS 2 communication stopped while updating the zones. "
                    "Close and restart Policy Bridge, then try again.",
                )
            return False, str(exc)
        failed = next(
            (
                result
                for result in results
                if not getattr(result, "successful", False)
            ),
            None,
        )
        if failed is not None:
            return False, failed.reason or "ROS parameter update failed"

        with self._gui_zone_lock:
            self._zone_db = {
                name: ZoneBounds(*bounds.as_tuple())
                for name, bounds in normalized.items()
            }
            self._labels = labels
        subscriber_count = self._publish_zone_geometry()
        if subscriber_count < 2:
            self._ui_print(
                (
                    f"Zone geometry updated: "
                    f"{', '.join(self._labels) or 'none'}, but only "
                    f"{subscriber_count}/2 costmap layer subscribers were "
                    "found. Restart Nav2 after sourcing this workspace."
                ),
                "yellow",
            )
        else:
            self._ui_print(
                (
                    f"Zone geometry updated: "
                    f"{', '.join(self._labels) or 'none'} "
                    f"({subscriber_count}/2 costmap layers connected)."
                ),
                "green",
            )
        return True, ""

    def set_map_path_from_gui(self, path: str) -> Tuple[bool, str]:

        resolved = str(Path(path).expanduser().resolve())
        try:
            results = self.set_parameters(
                [Parameter("gui_map_yaml", value=resolved)]
            )
        except Exception as exc:
            return False, str(exc)
        if results and not results[0].successful:
            return False, results[0].reason or "ROS parameter update failed"
        self._gui_map_yaml = resolved
        return True, ""

    def gui_snapshot(self) -> Dict[str, Any]:

        try:
            with self._command_lock:
                command_queued = bool(self._command_queue)
                pending_command_ids = {
                    int(
                        item.command_id
                        if hasattr(item, "command_id")
                        else item[0]
                    )
                    for item in self._command_queue
                }
                interpretation_active = getattr(
                    self, "_interpretation_active", threading.Event()
                ).is_set()
                interpretation_command_id = getattr(
                    self, "_active_interpretation_id", None
                )
                latest_command_id = int(
                    getattr(self, "_latest_command_id", 0)
                )
            with self._mission_lock:
                mission_pending = bool(self._mission_queue)
                pending_mission_ids = {
                    int(
                        item.command_id
                        if hasattr(item, "command_id")
                        else item[0]
                    )
                    for item in self._mission_queue
                }
                mission_active = self._mission_active.is_set()
                mission_paused = getattr(
                    self, "_mission_paused", threading.Event()
                ).is_set()
                command_id = self._active_command_id
        except AttributeError:
            command_queued = False
            pending_command_ids = set()
            interpretation_active = False
            interpretation_command_id = None
            latest_command_id = 0
            mission_pending = False
            pending_mission_ids = set()
            mission_active = False
            mission_paused = False
            command_id = None

        pending_ids = pending_command_ids | pending_mission_ids
        if (
            mission_active
            and interpretation_active
            and interpretation_command_id is not None
        ):
            pending_ids.add(int(interpretation_command_id))
        queued_missions = len(pending_ids)

        try:
            forbidden = sorted(self._last_forbidden_windows.keys())
        except AttributeError:
            forbidden = []

        configured_zones = sorted(
            {
                str(zone).strip().upper()
                for zone in getattr(self, "_zone_db", {})
                if str(zone).strip()
            }
        )
        if not configured_zones:
            configured_zones = sorted(
                {
                    str(zone).strip().upper()
                    for zone in getattr(self, "_labels", [])
                    if str(zone).strip()
                }
            )
        zone_geometry_publisher = getattr(self, "zone_geometry_pub", None)
        zone_geometry_subscribers = 0
        if zone_geometry_publisher is not None:
            try:
                zone_geometry_subscribers = int(
                    zone_geometry_publisher.get_subscription_count()
                )
            except (AttributeError, RuntimeError):
                zone_geometry_subscribers = 0

        try:
            with self._navigation_progress_lock:
                navigation_progress = copy.deepcopy(
                    self._navigation_progress
                )
        except AttributeError:
            navigation_progress = {
                "current": 0,
                "total": 0,
                "destination": "",
                "remaining_m": None,
            }

        return {
            "robot_namespace": getattr(self, "_gui_robot_namespace", ""),
            "nav2_ready": bool(getattr(self, "_nav2_ready", False)),
            "mission_active": mission_active,
            "mission_paused": mission_paused,
            "mission_pending": mission_pending,
            "queued_missions": queued_missions,
            "interpretation_active": interpretation_active,
            "command_queued": command_queued,
            "command_pending": (
                command_queued or interpretation_active or mission_pending
            ),
            "active_command_id": command_id,
            "interpretation_command_id": interpretation_command_id,
            "latest_command_id": latest_command_id,
            "policy_version": int(getattr(self, "_policy_version", 0)),
            "decision": str(
                getattr(self, "_last_policy_decision", "initialized")
            ),
            "llm_enabled": bool(getattr(self, "_enable_llm", False)),
            "llm_model": str(getattr(self, "_llm_model", "not configured")),
            "configured_zones": configured_zones,
            "zone_geometry_subscribers": zone_geometry_subscribers,
            "forbidden": forbidden,
            "soft_costs": copy.deepcopy(
                getattr(self, "_active_softcost", {})
            ),
            "object_rules": copy.deepcopy(
                getattr(self, "_latest_dynamic_rules", [])
            ),
            "object_radius": float(
                getattr(self, "_active_object_avoidance_radius", 0.0)
            ),
            "route": list(getattr(self, "_last_wp_names", []) or []),
            "navigation_progress": navigation_progress,
            "policy": copy.deepcopy(
                getattr(self, "_active_policy_state", {})
            ),
            "raw_policy": copy.deepcopy(
                getattr(self, "_last_raw_policy", None)
            ),
            "validation": copy.deepcopy(
                getattr(self, "_last_validation_report", {})
            ),
            "llm_metadata": copy.deepcopy(
                getattr(self, "_last_llm_metadata", {})
            ),
            "fire_alarm": bool(
                getattr(self, "_env_state", {}).get("fire_alarm", False)
            ),
            "battery_pct": int(
                getattr(self, "_env_state", {}).get("battery_pct", 100)
            ),
            "objects_detected": sum(
                len(value)
                for value in getattr(self, "_last_pub_by_frame", {}).values()
            ),
        }


class MultiRobotBridgeManager(QObject):


    robots_changed = pyqtSignal(object)
    robot_log_event = pyqtSignal(str, str, str)

    def __init__(
        self,
        graph_node: GuiPolicyBridgeNode,
        executor: MultiThreadedExecutor,
    ):
        super().__init__()
        self._graph_node = graph_node
        self._executor = executor
        self._nodes: Dict[str, GuiPolicyBridgeNode] = {}
        self._signals: Dict[str, GuiSignals] = {}
        self._active_namespaces: List[str] = []
        self._lock = threading.RLock()
        self._fleet_epoch = 0
        self._fleet_field_owners: Dict[str, Tuple[str, int]] = {}
        self._fleet_policy_snapshot: Dict[str, Any] = {
            "configured_zones": list(
                getattr(self._graph_node, "_labels", [])
            ),
            "zone_geometry_subscribers": 0,
            "forbidden": [],
            "soft_costs": {},
            "object_rules": [],
            "object_radius": 0.0,
        }
        policy_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._fleet_forbidden_pub = graph_node.create_publisher(
            RosString, "/fleet/forbidden_zones_update", policy_qos
        )
        self._fleet_softcost_pub = graph_node.create_publisher(
            RosString, "/fleet/zone_cost_overrides", policy_qos
        )
        self._fleet_object_radius_pub = graph_node.create_publisher(
            Float32, "/fleet/object_avoidance_radius", policy_qos
        )
        self._fleet_object_pub = graph_node.create_publisher(
            PoseArray, "/fleet/object_world_positions", 10
        )
        self._publish_initial_fleet_clear()

    @staticmethod
    def _policy_forbidden(policy: Dict[str, Any]) -> List[str]:
        zones: List[str] = []
        for item in policy.get("exclusions", []) or []:
            if isinstance(item, dict) and item.get("time_condition"):
                continue
            zone = item.get("zone") if isinstance(item, dict) else item
            if isinstance(zone, str):
                zones.append(zone.upper())
        return list(dict.fromkeys(zones))

    @staticmethod
    def _policy_soft_costs(policy: Dict[str, Any]) -> Dict[str, int]:
        result: Dict[str, int] = {}
        for item in policy.get("weights", []) or []:
            if not isinstance(item, dict) or not item.get("zone"):
                continue
            try:
                result[str(item["zone"]).upper()] = int(item["cost"])
            except (KeyError, TypeError, ValueError):
                continue
        return result

    @staticmethod
    def _policy_object_radius(policy: Dict[str, Any]) -> float:
        radii: List[float] = []
        for item in policy.get("dynamic_object_rules", []) or []:
            if not isinstance(item, dict):
                continue
            try:
                radii.append(float(item["radius"]))
            except (KeyError, TypeError, ValueError):
                continue
        return max(radii, default=0.0)

    def _publish_initial_fleet_clear(self) -> None:
        forbidden = RosString()
        forbidden.data = ""
        self._fleet_forbidden_pub.publish(forbidden)
        softcost = RosString()
        softcost.data = json.dumps({"zones": {}})
        self._fleet_softcost_pub.publish(softcost)
        radius = Float32()
        radius.data = 0.0
        self._fleet_object_radius_pub.publish(radius)
        self._publish_fleet_object_positions("map", [])

    def _begin_fleet_policy(
        self,
        namespace: str,
        policy: Dict[str, Any],
        requested_update: Optional[Dict[str, Any]] = None,
        replace_active: bool = False,
    ) -> int:

        requested = requested_update or policy
        touches_forbidden = bool(
            requested.get("exclusions")
            or requested.get("conditional_rules")
        )
        touches_soft_costs = bool(requested.get("weights"))
        touches_objects = bool(requested.get("dynamic_object_rules"))
        authoritative_fields = {
            field
            for field, touched in (
                ("forbidden", touches_forbidden),
                ("soft_costs", touches_soft_costs),
                ("object_radius", touches_objects),
                ("object_positions", touches_objects),
            )
            if touched or replace_active
        }

        with self._lock:
            current_forbidden = list(
                self._fleet_policy_snapshot.get("forbidden", [])
            )
            current_soft_costs = dict(
                self._fleet_policy_snapshot.get("soft_costs", {})
            )
            current_object_rules = copy.deepcopy(
                self._fleet_policy_snapshot.get("object_rules", [])
            )
            current_object_radius = float(
                self._fleet_policy_snapshot.get("object_radius", 0.0)
            )

            if touches_forbidden:
                requested_forbidden = self._policy_forbidden(requested)
                current_forbidden = list(dict.fromkeys(requested_forbidden))
            elif replace_active:
                current_forbidden = []

            if touches_soft_costs:
                current_soft_costs = self._policy_soft_costs(requested)
            elif replace_active:
                current_soft_costs = {}

            if touches_objects:
                current_object_rules = copy.deepcopy(
                    requested.get("dynamic_object_rules", []) or []
                )
                current_object_radius = self._policy_object_radius(requested)
            elif replace_active:
                current_object_rules = []
                current_object_radius = 0.0

            self._fleet_epoch += 1
            epoch = self._fleet_epoch
            for field in authoritative_fields:
                self._fleet_field_owners[field] = (namespace, epoch)
            self._fleet_policy_snapshot.update({
                "forbidden": current_forbidden,
                "soft_costs": current_soft_costs,
                "object_rules": current_object_rules,
                "object_radius": current_object_radius,
                "policy_source": namespace,
                "policy_epoch": epoch,
            })
        self._publish_fleet_policy_values(
            forbidden=current_forbidden,
            soft_costs=current_soft_costs,
            object_radius=current_object_radius,
        )
        if touches_objects or replace_active:
            self._publish_fleet_object_positions("map", [])
        return epoch

    def _publish_fleet_object_positions(
        self,
        frame: str,
        positions: List[Tuple[float, float]],
    ) -> None:
        publisher = getattr(self, "_fleet_object_pub", None)
        if publisher is None:
            return
        message = PoseArray()
        message.header.frame_id = frame or "map"
        message.header.stamp = self._graph_node.get_clock().now().to_msg()
        for x, y in positions:
            pose = Pose()
            pose.position.x = float(x)
            pose.position.y = float(y)
            pose.orientation.w = 1.0
            message.poses.append(pose)
        publisher.publish(message)

    def _publish_fleet_policy_values(
        self,
        *,
        forbidden: Optional[List[str]] = None,
        soft_costs: Optional[Dict[str, int]] = None,
        object_radius: Optional[float] = None,
    ) -> None:
        if forbidden is not None:
            message = RosString()
            message.data = ",".join(forbidden)
            self._fleet_forbidden_pub.publish(message)
        if soft_costs is not None:
            message = RosString()
            message.data = json.dumps({"zones": soft_costs})
            self._fleet_softcost_pub.publish(message)
        if object_radius is not None:
            message = Float32()
            message.data = float(object_radius)
            self._fleet_object_radius_pub.publish(message)

    def _accept_fleet_policy_value(
        self,
        namespace: str,
        field: str,
        value: Any,
        epoch: int,
    ) -> None:

        with self._lock:
            owner = getattr(self, "_fleet_field_owners", {}).get(field)
            if owner != (namespace, int(epoch)):
                return
            self._fleet_policy_snapshot[field] = copy.deepcopy(value)
            if field == "object_radius" and float(value) <= 0.0:
                self._fleet_policy_snapshot["object_rules"] = []
            self._fleet_policy_snapshot["policy_source"] = namespace
        if field == "forbidden":
            self._publish_fleet_policy_values(forbidden=list(value or []))
        elif field == "soft_costs":
            self._publish_fleet_policy_values(
                soft_costs=dict(value or {})
            )
        elif field == "object_radius":
            self._publish_fleet_policy_values(object_radius=float(value))
        elif field == "object_positions":
            payload = value if isinstance(value, dict) else {}
            positions = []
            for item in payload.get("positions", []) or []:
                try:
                    positions.append((float(item[0]), float(item[1])))
                except (IndexError, TypeError, ValueError):
                    continue
            self._publish_fleet_object_positions(
                str(payload.get("frame", "map")),
                positions,
            )

    @property
    def active_namespaces(self) -> List[str]:
        with self._lock:
            return list(self._active_namespaces)

    def _node_overrides(self) -> List[Parameter]:
        overrides = [
            Parameter(
                "llm_model",
                value=str(getattr(self._graph_node, "_llm_model", "")),
            ),
            Parameter(
                "llm_endpoint",
                value=str(getattr(self._graph_node, "_llm_endpoint", "")),
            ),
            Parameter(
                "zone_labels",
                Parameter.Type.STRING_ARRAY,
                list(getattr(self._graph_node, "_labels", [])),
            ),
            Parameter(
                "zone_database_json",
                value=json.dumps(
                    {
                        name: list(bounds.as_tuple())
                        for name, bounds in getattr(
                            self._graph_node, "_zone_db", {}
                        ).items()
                    },
                    ensure_ascii=False,
                ),
            ),
        ]
        return overrides

    def _create_robot_node(self, namespace: str) -> GuiPolicyBridgeNode:
        signals = GuiSignals()
        signals.log_event.connect(
            lambda message, color, robot=namespace: self._forward_robot_log(
                robot, message, color
            )
        )
        node = GuiPolicyBridgeNode(
            signals,
            robot_namespace=namespace,
            parameter_overrides=self._node_overrides(),
            fleet_epoch_factory=self._begin_fleet_policy,
            fleet_policy_sink=self._accept_fleet_policy_value,
        )
        node.replace_waypoints_from_gui(
            self._graph_node.waypoints_from_gui()
        )
        self._executor.add_node(node)
        for message, color in signals.activate():
            self.robot_log_event.emit(namespace, message, color)
        self._signals[namespace] = signals
        self._nodes[namespace] = node
        return node

    def _forward_robot_log(
        self, namespace: str, message: str, color: str
    ) -> None:
        self.robot_log_event.emit(namespace, message, color)

    def refresh_discovery(self) -> List[str]:

        try:
            topic_names = [
                name
                for name, _types in (
                    self._graph_node.get_topic_names_and_types()
                )
            ]
        except Exception:
            topic_names = []
        discovered = discover_robot_namespaces(topic_names)
        with self._lock:
            for namespace in discovered:
                if namespace not in self._nodes:
                    try:
                        self._create_robot_node(namespace)
                    except Exception as exc:
                        self.robot_log_event.emit(
                            namespace,
                            f"Robot bridge startup failed: {exc}",
                            "red",
                        )
            changed = discovered != self._active_namespaces
            self._active_namespaces = list(discovered)
        if changed:
            self.robots_changed.emit(list(discovered))
        return discovered

    def node(self, namespace: str) -> Optional[GuiPolicyBridgeNode]:
        with self._lock:
            return self._nodes.get(_normalized_robot_namespace(namespace))

    def snapshots(self) -> Dict[str, Dict[str, Any]]:
        result: Dict[str, Dict[str, Any]] = {}
        for namespace in self.active_namespaces:
            node = self.node(namespace)
            if node is None:
                continue
            try:
                result[namespace] = node.gui_snapshot()
            except Exception as exc:
                result[namespace] = {
                    "robot_namespace": namespace,
                    "nav2_ready": False,
                    "decision": f"snapshot error: {exc}",
                }
        geometry_subscribers = max(
            (
                int(item.get("zone_geometry_subscribers", 0) or 0)
                for item in result.values()
            ),
            default=0,
        )
        with self._lock:
            self._fleet_policy_snapshot[
                "zone_geometry_subscribers"
            ] = geometry_subscribers
            fleet_policy = copy.deepcopy(self._fleet_policy_snapshot)
        if fleet_policy:
            for snapshot in result.values():
                snapshot.update(copy.deepcopy(fleet_policy))
        return result

    def submit(self, namespace: str, command: str) -> bool:
        node = self.node(namespace)
        return bool(node and node.submit_from_gui(command))

    def toggle_pause(self, namespace: str) -> bool:
        node = self.node(namespace)
        return bool(node and node.toggle_pause_from_gui())

    def cancel(self, namespace: str) -> bool:
        node = self.node(namespace)
        return bool(node and node.cancel_from_gui())

    def select_model(self, model: str) -> List[str]:
        errors: List[str] = []
        endpoint = getattr(self._graph_node, "_gui_model_endpoints", {}).get(
            model, getattr(self._graph_node, "_llm_endpoint", "")
        )
        with self._lock:
            nodes = list(self._nodes.items())
        for namespace, node in nodes:
            node._gui_available_models = list(
                getattr(self._graph_node, "_gui_available_models", [])
            )
            node._gui_model_endpoints = {model: endpoint}
            accepted, reason = node.select_model_from_gui(model)
            if not accepted:
                errors.append(f"{namespace}: {reason}")
        return errors

    def replace_waypoints(self, waypoints: Dict[str, Waypoint]) -> List[str]:
        errors: List[str] = []
        with self._lock:
            nodes = list(self._nodes.items())
        for namespace, node in nodes:
            accepted, reason = node.replace_waypoints_from_gui(waypoints)
            if not accepted:
                errors.append(f"{namespace}: {reason}")
        return errors

    def replace_zones(self, zones: Dict[str, ZoneBounds]) -> List[str]:
        errors: List[str] = []
        with self._lock:
            nodes = list(self._nodes.items())
        for namespace, node in nodes:
            accepted, reason = node.replace_zones_from_gui(zones)
            if not accepted:
                errors.append(f"{namespace}: {reason}")
        return errors

    def shutdown(self) -> None:
        with self._lock:
            nodes = list(self._nodes.values())
        for node in nodes:
            node.stop_gui_monitor()
            node._yolo_run = False
            try:
                node.request_cancel()
            except Exception:
                pass

    def destroy_nodes(self) -> None:
        with self._lock:
            nodes = list(self._nodes.values())
            self._nodes.clear()
            self._signals.clear()
            self._active_namespaces = []
        for node in nodes:
            try:
                self._executor.remove_node(node)
            except Exception:
                pass
            navigator = getattr(node, "_nav", None)
            if navigator is not None:
                try:
                    navigator.destroy_node()
                except Exception:
                    pass
            try:
                node.destroy_node()
            except Exception:
                pass


class CommandEdit(QPlainTextEdit):


    submit_requested = pyqtSignal()

    def keyPressEvent(self, event: QKeyEvent) -> None:
        if (
            event.key() in (Qt.Key_Return, Qt.Key_Enter)
            and event.modifiers() & Qt.ControlModifier
        ):
            self.submit_requested.emit()
            return
        super().keyPressEvent(event)


class Surface(QFrame):


    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.setObjectName("surface")


class MetricTile(QFrame):


    def __init__(self, label: str, value: str):
        super().__init__()
        self.setObjectName("metricTile")
        self.setMinimumHeight(58)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(12, 8, 12, 8)
        layout.setSpacing(2)
        label_widget = QLabel(label)
        label_widget.setObjectName("metricLabel")
        self.value = QLabel(value)
        self.value.setObjectName("metricValue")
        self.value.setTextInteractionFlags(Qt.TextSelectableByMouse)
        layout.addWidget(label_widget)
        layout.addWidget(self.value)


class ModelMetricTile(QFrame):


    clicked = pyqtSignal()

    def __init__(self, value: str, *, compact: bool = False):
        super().__init__()
        self.setObjectName(
            "compactModelSelector" if compact else "modelMetricTile"
        )
        if compact:
            self.setMinimumSize(250, 34)
            self.setMaximumHeight(34)
        else:
            self.setMinimumHeight(58)
        self.setCursor(Qt.PointingHandCursor)
        self.setFocusPolicy(Qt.StrongFocus)
        self.setAccessibleName("Select LLM model")
        self.setToolTip("Select an installed Ollama model")

        layout = QHBoxLayout(self)
        layout.setContentsMargins(10, 3 if compact else 8, 6, 3 if compact else 8)
        layout.setSpacing(8)
        label = QLabel("LLM MODEL")
        label.setObjectName("metricLabel")
        self.value = QLabel(value)
        self.value.setObjectName("metricValue")
        if compact:
            layout.addWidget(label)
            layout.addWidget(self.value, 1)
        else:
            text_layout = QVBoxLayout()
            text_layout.setSpacing(2)
            text_layout.addWidget(label)
            text_layout.addWidget(self.value)
            layout.addLayout(text_layout, 1)

        self.arrow = QToolButton()
        self.arrow.setObjectName("modelArrow")
        self.arrow.setArrowType(Qt.DownArrow)
        self.arrow.setToolTip("Choose model")
        self.arrow.clicked.connect(self.clicked.emit)
        layout.addWidget(self.arrow, 0, Qt.AlignVCenter)

    def mouseReleaseEvent(self, event) -> None:
        if event.button() == Qt.LeftButton:
            self.clicked.emit()
            event.accept()
            return
        super().mouseReleaseEvent(event)

    def keyPressEvent(self, event: QKeyEvent) -> None:
        if event.key() in (Qt.Key_Return, Qt.Key_Enter, Qt.Key_Space):
            self.clicked.emit()
            event.accept()
            return
        super().keyPressEvent(event)


class PolicySummary(Surface):


    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(14, 12, 14, 12)
        layout.setSpacing(9)

        title_row = QHBoxLayout()
        title = QLabel("Active policy")
        title.setObjectName("sectionTitle")
        self.version = QLabel("Version 0")
        self.version.setObjectName("sectionMeta")
        self.zone_layer_status = QLabel("Zone layers --")
        self.zone_layer_status.setObjectName("sectionMeta")
        self.zone_layer_status.setToolTip(
            "Live subscribers to the authoritative zone-geometry topic"
        )
        title_row.addWidget(title)
        title_row.addStretch(1)
        title_row.addWidget(self.zone_layer_status)
        title_row.addWidget(self.version)
        layout.addLayout(title_row)

        forbidden_label = QLabel("FORBIDDEN ZONES")
        forbidden_label.setObjectName("fieldLabel")
        layout.addWidget(forbidden_label)

        self.zones_row = QHBoxLayout()
        self.zones_row.setSpacing(7)
        self.zone_labels: Dict[str, QLabel] = {}
        layout.addLayout(self.zones_row)

        cost_label = QLabel("SOFT COSTS")
        cost_label.setObjectName("fieldLabel")
        layout.addWidget(cost_label)
        self.cost_grid = QGridLayout()
        self.cost_grid.setHorizontalSpacing(8)
        self.cost_grid.setVerticalSpacing(5)
        self.cost_names: Dict[str, QLabel] = {}
        self.cost_bars: Dict[str, QProgressBar] = {}
        self.cost_values: Dict[str, QLabel] = {}
        self.cost_grid.setColumnStretch(1, 1)
        layout.addLayout(self.cost_grid)

        self._sync_zone_widgets(list("ABCDE"))

        object_label = QLabel("OBJECT RULES")
        object_label.setObjectName("fieldLabel")
        layout.addWidget(object_label)
        self.object_row = QHBoxLayout()
        self.object_row.setSpacing(6)
        layout.addLayout(self.object_row)

        route_label = QLabel("ROUTE ORDER")
        route_label.setObjectName("fieldLabel")
        layout.addWidget(route_label)
        self.route_row = QHBoxLayout()
        self.route_row.setSpacing(5)
        layout.addLayout(self.route_row)
        layout.addStretch(1)

        self.update_policy({})

    def _sync_zone_widgets(self, zones: List[str]) -> None:

        normalized = sorted(
            {
                str(zone).strip().upper()
                for zone in zones
                if str(zone).strip()
            }
        )
        if not normalized:
            normalized = list("ABCDE")
        if normalized == list(self.zone_labels):
            return

        _clear_layout(self.zones_row)
        _clear_layout(self.cost_grid)
        self.zone_labels.clear()
        self.cost_names.clear()
        self.cost_bars.clear()
        self.cost_values.clear()

        for zone in normalized:
            item = QLabel(zone)
            item.setObjectName("zoneInactive")
            item.setAlignment(Qt.AlignCenter)
            item.setFixedSize(38, 30)
            item.setToolTip(f"Zone {zone}")
            self.zones_row.addWidget(item)
            self.zone_labels[zone] = item
        self.zones_row.addStretch(1)

        for row, zone in enumerate(normalized):
            name = QLabel(zone)
            name.setFixedWidth(14)
            name.setObjectName("sectionMeta")
            bar = QProgressBar()
            bar.setRange(0, 252)
            bar.setValue(0)
            bar.setTextVisible(False)
            value = QLabel("-")
            value.setFixedWidth(28)
            value.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            value.setObjectName("sectionMeta")
            self.cost_grid.addWidget(name, row, 0)
            self.cost_grid.addWidget(bar, row, 1)
            self.cost_grid.addWidget(value, row, 2)
            self.cost_names[zone] = name
            self.cost_bars[zone] = bar
            self.cost_values[zone] = value
        self.cost_grid.setColumnStretch(1, 1)

    def update_policy(self, snapshot: Dict[str, Any]) -> None:

        self.version.setText(
            f"Version {int(snapshot.get('policy_version', 0))}"
        )
        subscriber_count = int(
            snapshot.get("zone_geometry_subscribers", 0) or 0
        )
        self.zone_layer_status.setText(
            f"Zone layers {min(subscriber_count, 2)}/2"
        )
        _set_object_name(
            self.zone_layer_status,
            "statusOnline" if subscriber_count >= 2 else "statusOffline",
        )
        self.zone_layer_status.setToolTip(
            "Both zone-aware custom layers are connected"
            if subscriber_count >= 2
            else (
                "Zone geometry is not connected to both custom layers; "
                "restart Nav2 after sourcing this workspace"
            )
        )

        forbidden = {
            str(zone).strip().upper()
            for zone in (snapshot.get("forbidden", []) or [])
            if str(zone).strip()
        }
        costs = {
            str(zone).strip().upper(): value
            for zone, value in (snapshot.get("soft_costs", {}) or {}).items()
            if str(zone).strip()
        }
        configured_zones = snapshot.get("configured_zones", []) or []
        self._sync_zone_widgets(
            list(configured_zones) + list(forbidden) + list(costs)
        )
        for zone, label in self.zone_labels.items():
            name = "zoneForbidden" if zone in forbidden else "zoneInactive"
            _set_object_name(label, name)

        for zone, bar in self.cost_bars.items():
            value = int(costs.get(zone, 0))
            bar.setValue(value)
            cost_text = str(value) if zone in costs else "-"
            self.cost_values[zone].setText(cost_text)
            if value >= 180:
                color = "#c54b4b"
            elif value >= 80:
                color = "#d79b2f"
            else:
                color = "#4e9b70"
            bar.setStyleSheet(
                "QProgressBar::chunk {"
                f"background: {color}; border-radius: 4px;"
                "}"
            )

        _clear_layout(self.object_row)
        rules = snapshot.get("object_rules", []) or []
        if rules:
            for rule in rules:
                if not isinstance(rule, dict):
                    continue
                object_class = str(rule.get("class", "object")).capitalize()
                try:
                    radius = float(rule.get("radius", 0.0))
                    text = f"{object_class}  {radius:g} m"
                except (TypeError, ValueError):
                    text = object_class
                label = QLabel(text)
                label.setObjectName("objectRule")
                self.object_row.addWidget(label)
        else:
            empty = QLabel("No active object rule")
            empty.setObjectName("emptyValue")
            self.object_row.addWidget(empty)
        self.object_row.addStretch(1)

        _clear_layout(self.route_row)
        route = snapshot.get("route", []) or []
        if route:
            for index, destination in enumerate(route):
                if index:
                    arrow = QLabel("->")
                    arrow.setObjectName("sectionMeta")
                    self.route_row.addWidget(arrow)
                tag = QLabel(str(destination))
                tag.setObjectName("routeTag")
                self.route_row.addWidget(tag)
        else:
            empty = QLabel("No active route")
            empty.setObjectName("emptyValue")
            self.route_row.addWidget(empty)
        self.route_row.addStretch(1)


class MissionProgress(Surface):


    fire_changed = pyqtSignal(bool)
    battery_changed = pyqtSignal(int)
    pause_resume_requested = pyqtSignal()
    cancel_requested = pyqtSignal()

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self._updating_environment = False
        layout = QVBoxLayout(self)
        layout.setContentsMargins(14, 12, 14, 12)
        layout.setSpacing(8)

        top = QHBoxLayout()
        title = QLabel("Mission progress")
        title.setObjectName("sectionTitle")
        self.stop_button = QPushButton("Stop")
        self.stop_button.setObjectName("secondaryButton")
        self.stop_button.setIcon(
            self.style().standardIcon(QStyle.SP_MediaStop)
        )
        self.stop_button.setEnabled(False)
        self.stop_button.setToolTip(
            "Temporarily stop the active mission and retain its route"
        )
        self.cancel_button = QPushButton("Cancel")
        self.cancel_button.setObjectName("dangerButton")
        self.cancel_button.setIcon(
            self.style().standardIcon(QStyle.SP_DialogCancelButton)
        )
        self.cancel_button.setEnabled(False)
        self.cancel_button.setToolTip("Cancel the current mission completely")
        self.state = QLabel("Idle")
        self.state.setObjectName("statusOnline")
        top.addWidget(title)
        top.addStretch(1)
        top.addWidget(self.stop_button)
        top.addWidget(self.cancel_button)
        top.addWidget(self.state)
        layout.addLayout(top)

        self.destination = QLabel("Waiting for a command")
        self.destination.setObjectName("metricValue")
        self.remaining = QLabel("Remaining distance: -")
        self.remaining.setObjectName("sectionMeta")
        self.queue_status = QLabel("Queue: empty")
        self.queue_status.setObjectName("sectionMeta")
        layout.addWidget(self.destination)
        layout.addWidget(self.remaining)
        layout.addWidget(self.queue_status)

        self.progress = QProgressBar()
        self.progress.setObjectName("missionProgress")
        self.progress.setRange(0, 1)
        self.progress.setValue(0)
        self.progress.setTextVisible(False)
        layout.addWidget(self.progress)

        self.stage_labels: List[QLabel] = []
        for stage in STAGE_LABELS:
            label = QLabel(f"○  {stage}")
            label.setObjectName("stagePending")
            self.stage_labels.append(label)
            layout.addWidget(label)

        layout.addStretch(1)

        env_rule = QFrame()
        env_rule.setFrameShape(QFrame.HLine)
        env_rule.setStyleSheet("color: #e0e5eb;")
        layout.addWidget(env_rule)

        env_title = QLabel("Environment state")
        env_title.setObjectName("fieldLabel")
        layout.addWidget(env_title)

        controls = QHBoxLayout()
        self.fire_alarm = QCheckBox("Fire alarm")
        self.fire_alarm.setToolTip("Toggle the fire-alarm event")
        self.battery = QSpinBox()
        self.battery.setRange(0, 100)
        self.battery.setSuffix(" %")
        self.battery.setValue(100)
        self.battery.setToolTip("Publish the current battery level")
        self.apply_battery = QToolButton()
        self.apply_battery.setIcon(
            self.style().standardIcon(QStyle.SP_DialogApplyButton)
        )
        self.apply_battery.setToolTip("Apply battery level")
        controls.addWidget(self.fire_alarm)
        controls.addSpacing(8)
        controls.addWidget(QLabel("Battery"))
        controls.addWidget(self.battery)
        controls.addWidget(self.apply_battery)
        controls.addStretch(1)
        layout.addLayout(controls)

        self.fire_alarm.toggled.connect(self._on_fire_toggled)
        self.apply_battery.clicked.connect(
            lambda: self.battery_changed.emit(self.battery.value())
        )
        self.stop_button.clicked.connect(self.pause_resume_requested.emit)
        self.cancel_button.clicked.connect(self.cancel_requested.emit)

    def _on_fire_toggled(self, enabled: bool) -> None:
        if not self._updating_environment:
            self.fire_changed.emit(enabled)

    def set_stage(self, active: int, failed: bool = False) -> None:

        for index, label in enumerate(self.stage_labels):
            if failed and index == active:
                label.setText(f"x  {STAGE_LABELS[index]}")
                _set_object_name(label, "stageFailed")
            elif index < active or active >= len(self.stage_labels):
                label.setText(f"+  {STAGE_LABELS[index]}")
                _set_object_name(label, "stageDone")
            elif index == active:
                label.setText(f">  {STAGE_LABELS[index]}")
                _set_object_name(label, "stageActive")
            else:
                label.setText(f"○  {STAGE_LABELS[index]}")
                _set_object_name(label, "stagePending")

    def set_mission_state(self, state: str, kind: str) -> None:
        self.state.setText(state)
        object_name = {
            "online": "statusOnline",
            "busy": "statusBusy",
            "error": "statusOffline",
        }.get(kind, "statusOnline")
        _set_object_name(self.state, object_name)

    def update_environment(self, snapshot: Dict[str, Any]) -> None:
        self._updating_environment = True
        self.fire_alarm.setChecked(bool(snapshot.get("fire_alarm", False)))
        self.battery.setValue(int(snapshot.get("battery_pct", 100)))
        self._updating_environment = False
        cancellable = any(
            bool(snapshot.get(key, False))
            for key in (
                "mission_active",
                "interpretation_active",
                "command_pending",
            )
        )
        mission_active = bool(snapshot.get("mission_active", False))
        mission_paused = bool(snapshot.get("mission_paused", False))
        self.stop_button.setEnabled(mission_active)
        if mission_paused:
            self.stop_button.setText("Resume")
            self.stop_button.setIcon(
                self.style().standardIcon(QStyle.SP_MediaPlay)
            )
            self.stop_button.setToolTip("Resume the paused mission")
        else:
            self.stop_button.setText("Stop")
            self.stop_button.setIcon(
                self.style().standardIcon(QStyle.SP_MediaStop)
            )
            self.stop_button.setToolTip(
                "Temporarily stop the active mission and retain its route"
            )
        self.cancel_button.setEnabled(cancellable)
        queued = max(0, int(snapshot.get("queued_missions", 0) or 0))
        self.queue_status.setText(
            "Queue: empty" if queued == 0 else f"Queued missions: {queued}"
        )


class CommandComposer(Surface):


    command_submitted = pyqtSignal(str)
    reset_requested = pyqtSignal()

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(14, 12, 14, 12)
        layout.setSpacing(8)

        top = QHBoxLayout()
        title = QLabel("Natural-language command")
        title.setObjectName("sectionTitle")
        self.counter = QLabel("0 characters")
        self.counter.setObjectName("sectionMeta")
        top.addWidget(title)
        top.addStretch(1)
        top.addWidget(self.counter)
        layout.addLayout(top)

        self.editor = CommandEdit()
        self.editor.setObjectName("commandInput")
        self.editor.setMinimumHeight(76)
        self.editor.setMaximumHeight(100)
        self.editor.setPlaceholderText("Enter an English navigation command")
        self.editor.setAccessibleName("Natural-language navigation command")
        layout.addWidget(self.editor)

        actions = QHBoxLayout()
        self.actions = actions
        self.examples = QComboBox()
        for label, command in EXAMPLE_COMMANDS:
            self.examples.addItem(label, command)
        self.examples.setMinimumWidth(230)
        self.examples.setToolTip("Load a representative command")

        self.reset_button = QToolButton()
        self.reset_button.setIcon(
            self.style().standardIcon(QStyle.SP_BrowserReload)
        )
        self.reset_button.setToolTip("Reset the current policy")
        self.reset_button.setAccessibleName("Reset policy")

        self.send_button = QPushButton("Execute")
        self.send_button.setObjectName("primaryButton")
        self.send_button.setIcon(
            self.style().standardIcon(QStyle.SP_MediaPlay)
        )
        self.send_button.setMinimumWidth(112)
        self.send_button.setToolTip("Interpret and execute this command")
        actions.addWidget(self.examples)
        actions.addWidget(self.reset_button)
        actions.addStretch(1)
        actions.addWidget(self.send_button)
        layout.addLayout(actions)

        self.editor.textChanged.connect(self._on_text_changed)
        self.editor.submit_requested.connect(self.submit)
        self.send_button.clicked.connect(self.submit)
        self.reset_button.clicked.connect(self.reset_requested.emit)
        self.examples.currentIndexChanged.connect(self._load_example)

    def add_model_selector(self, selector: QWidget) -> None:

        self.actions.insertWidget(2, selector)

    def _on_text_changed(self) -> None:
        length = len(self.editor.toPlainText())
        self.counter.setText(f"{length} characters")

    def _load_example(self, index: int) -> None:
        command = str(self.examples.itemData(index) or "")
        if command:
            self.editor.setPlainText(command)
            self.editor.setFocus()

    def submit(self) -> None:
        command = self.editor.toPlainText().strip()
        if command:
            self.command_submitted.emit(command)

    def set_available(self, available: bool) -> None:
        self.editor.setEnabled(available)
        self.send_button.setEnabled(available)


class RobotMissionCard(Surface):


    command_submitted = pyqtSignal(str, str)
    pause_resume_requested = pyqtSignal(str)
    cancel_requested = pyqtSignal(str)

    def __init__(self, namespace: str):
        super().__init__()
        self.namespace = _normalized_robot_namespace(namespace)
        self._logs: List[Tuple[str, str, str]] = []
        self.setMinimumHeight(250)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(14, 11, 14, 12)
        layout.setSpacing(8)

        header = QHBoxLayout()
        self.robot_name = QLabel(self.namespace.upper())
        self.robot_name.setObjectName("robotName")
        self.endpoint = QLabel(f"/{self.namespace}/follow_waypoints")
        self.endpoint.setObjectName("sectionMeta")
        self.state = QLabel("Checking")
        self.state.setObjectName("statusBusy")
        header.addWidget(self.robot_name)
        header.addWidget(self.endpoint)
        header.addStretch(1)
        header.addWidget(self.state)
        layout.addLayout(header)

        body = QHBoxLayout()
        body.setSpacing(14)

        command_column = QVBoxLayout()
        command_column.setSpacing(7)
        self.editor = CommandEdit()
        self.editor.setObjectName("compactCommandInput")
        self.editor.setPlaceholderText(
            f"Enter an English command for {self.namespace.upper()}"
        )
        self.editor.setMinimumHeight(72)
        self.editor.setMaximumHeight(82)
        command_column.addWidget(self.editor)

        controls = QHBoxLayout()
        self.execute_button = QPushButton("Execute")
        self.execute_button.setObjectName("primaryButton")
        self.execute_button.setIcon(
            self.style().standardIcon(QStyle.SP_MediaPlay)
        )
        self.pause_button = QPushButton("Stop")
        self.pause_button.setObjectName("secondaryButton")
        self.pause_button.setIcon(
            self.style().standardIcon(QStyle.SP_MediaStop)
        )
        self.cancel_button = QPushButton("Cancel")
        self.cancel_button.setObjectName("dangerButton")
        self.cancel_button.setIcon(
            self.style().standardIcon(QStyle.SP_DialogCancelButton)
        )
        controls.addWidget(self.execute_button)
        controls.addStretch(1)
        controls.addWidget(self.pause_button)
        controls.addWidget(self.cancel_button)
        command_column.addLayout(controls)
        body.addLayout(command_column, 3)

        status_column = QVBoxLayout()
        status_column.setSpacing(5)
        self.route = QLabel("Route  -")
        self.route.setObjectName("robotPolicyLine")
        self.forbidden = QLabel("Fleet keepout  -")
        self.forbidden.setObjectName("robotPolicyLine")
        self.costs = QLabel("Fleet costs  -")
        self.costs.setObjectName("robotPolicyLine")
        self.objects = QLabel("Object rule  -")
        self.objects.setObjectName("robotPolicyLine")
        for widget in (self.route, self.forbidden, self.costs, self.objects):
            widget.setTextInteractionFlags(Qt.TextSelectableByMouse)
            status_column.addWidget(widget)
        self.progress_text = QLabel("Waiting for a command")
        self.progress_text.setObjectName("sectionMeta")
        self.progress = QProgressBar()
        self.progress.setObjectName("missionProgress")
        self.progress.setRange(0, 1)
        self.progress.setValue(0)
        self.progress.setTextVisible(False)
        status_column.addWidget(self.progress_text)
        status_column.addWidget(self.progress)
        body.addLayout(status_column, 2)
        layout.addLayout(body)

        self.activity = QTextEdit()
        self.activity.setObjectName("compactActivityLog")
        self.activity.setReadOnly(True)
        self.activity.setMinimumHeight(54)
        self.activity.setMaximumHeight(62)
        layout.addWidget(self.activity)

        self.editor.submit_requested.connect(self.submit)
        self.execute_button.clicked.connect(self.submit)
        self.pause_button.clicked.connect(
            lambda: self.pause_resume_requested.emit(self.namespace)
        )
        self.cancel_button.clicked.connect(
            lambda: self.cancel_requested.emit(self.namespace)
        )
        self.update_snapshot({"nav2_ready": False})

    def submit(self) -> None:
        command = self.editor.toPlainText().strip()
        if command:
            self.command_submitted.emit(self.namespace, command)

    def append_log(self, message: str, color: str = "") -> None:
        self._logs.append(
            (dt.datetime.now().strftime("%H:%M:%S"), color, message)
        )
        self._logs = self._logs[-20:]
        rows = []
        for timestamp, entry_color, text_value in self._logs[-3:]:
            color_value = LOG_COLORS.get(entry_color, LOG_COLORS[""])
            rows.append(
                f'<span style="color:#8a95a4">{timestamp}</span> '
                f'<span style="color:{color_value}">'
                f'{html.escape(text_value)}</span>'
            )
        self.activity.setHtml("<br>".join(rows))
        self.activity.verticalScrollBar().setValue(
            self.activity.verticalScrollBar().maximum()
        )

    def update_snapshot(self, snapshot: Dict[str, Any]) -> None:
        ready = bool(snapshot.get("nav2_ready", False))
        active = bool(snapshot.get("mission_active", False))
        paused = bool(snapshot.get("mission_paused", False))
        interpreting = bool(snapshot.get("interpretation_active", False))
        pending = bool(snapshot.get("command_pending", False))

        if not ready:
            state_text, state_kind = "Offline", "statusOffline"
        elif paused:
            state_text, state_kind = "Paused", "statusBusy"
        elif active:
            state_text, state_kind = "Navigating", "statusBusy"
        elif interpreting or pending:
            state_text, state_kind = "Interpreting", "statusBusy"
        else:
            state_text, state_kind = "Ready", "statusOnline"
        self.state.setText(state_text)
        _set_object_name(self.state, state_kind)

        self.editor.setEnabled(ready)
        self.execute_button.setEnabled(ready)
        self.pause_button.setEnabled(active)
        self.cancel_button.setEnabled(active or interpreting or pending)
        self.pause_button.setText("Resume" if paused else "Stop")
        self.pause_button.setIcon(
            self.style().standardIcon(
                QStyle.SP_MediaPlay if paused else QStyle.SP_MediaStop
            )
        )

        route = list(snapshot.get("route", []) or [])
        self.route.setText("Route  " + (" -> ".join(route) or "-"))
        forbidden = list(snapshot.get("forbidden", []) or [])
        self.forbidden.setText(
            "Fleet keepout  " + (", ".join(forbidden) or "-")
        )
        costs = snapshot.get("soft_costs", {}) or {}
        cost_text = ", ".join(
            f"{zone}:{cost}" for zone, cost in sorted(costs.items())
        )
        self.costs.setText("Fleet costs  " + (cost_text or "-"))
        object_rules = snapshot.get("object_rules", []) or []
        object_texts = []
        for rule in object_rules:
            if not isinstance(rule, dict):
                continue
            label = str(rule.get("class", "object")).capitalize()
            try:
                label += f"({float(rule.get('radius')):g}m)"
            except (TypeError, ValueError):
                pass
            object_texts.append(label)
        self.objects.setText(
            "Object rule  " + (", ".join(object_texts) or "-")
        )

        progress = snapshot.get("navigation_progress", {}) or {}
        current = max(0, int(progress.get("current", 0) or 0))
        total = max(0, int(progress.get("total", 0) or 0))
        destination = str(progress.get("destination", "") or "")
        remaining = progress.get("remaining_m")
        if active or paused:
            suffix = ""
            if remaining is not None:
                try:
                    suffix = f" | {float(remaining):.2f} m remaining"
                except (TypeError, ValueError):
                    pass
            self.progress_text.setText(
                f"Waypoint {max(1, current)}/{max(1, total)}  "
                f"{destination or '-'}{suffix}"
            )
            self.progress.setRange(0, max(1, total))
            self.progress.setValue(max(0, current - 1))
        elif interpreting or pending:
            self.progress_text.setText("Interpreting and validating policy")
            self.progress.setRange(0, 0)
        else:
            self.progress_text.setText("Waiting for a command")
            self.progress.setRange(0, 1)
            self.progress.setValue(0)


class MultiRobotMissionPage(QWidget):


    command_submitted = pyqtSignal(str, str)
    pause_resume_requested = pyqtSignal(str)
    cancel_requested = pyqtSignal(str)
    refresh_requested = pyqtSignal()

    def __init__(self):
        super().__init__()
        self.cards: Dict[str, RobotMissionCard] = {}
        layout = QVBoxLayout(self)
        layout.setContentsMargins(14, 12, 14, 14)
        layout.setSpacing(9)

        toolbar = Surface()
        toolbar_layout = QHBoxLayout(toolbar)
        toolbar_layout.setContentsMargins(13, 9, 10, 9)
        title_column = QVBoxLayout()
        title_column.setSpacing(1)
        title = QLabel("Fleet missions")
        title.setObjectName("sectionTitle")
        note = QLabel(
            "Routes are robot-specific; keepout and cost policies are shared "
            "across the fleet."
        )
        note.setObjectName("sectionMeta")
        title_column.addWidget(title)
        title_column.addWidget(note)
        self.count = QLabel("0 robots")
        self.count.setObjectName("statusOffline")
        self.refresh_button = QToolButton()
        self.refresh_button.setIcon(
            self.style().standardIcon(QStyle.SP_BrowserReload)
        )
        self.refresh_button.setToolTip("Refresh robot namespaces")
        self.model_selector = ModelMetricTile("Not configured")
        self.model_selector.setFixedWidth(210)
        self.model_selector.setToolTip(
            "Select the LLM model used by every fleet mission"
        )
        toolbar_layout.addLayout(title_column)
        toolbar_layout.addStretch(1)
        toolbar_layout.addWidget(self.model_selector)
        toolbar_layout.addWidget(self.count)
        toolbar_layout.addWidget(self.refresh_button)
        layout.addWidget(toolbar)

        self.scroll = QScrollArea()
        self.scroll.setWidgetResizable(True)
        self.scroll.setFrameShape(QFrame.NoFrame)
        self.scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.card_host = QWidget()
        self.card_layout = QVBoxLayout(self.card_host)
        self.card_layout.setContentsMargins(0, 0, 0, 0)
        self.card_layout.setSpacing(9)
        self.empty = Surface()
        empty_layout = QVBoxLayout(self.empty)
        empty_layout.setContentsMargins(24, 38, 24, 38)
        empty_title = QLabel("No namespaced Nav2 robots detected")
        empty_title.setObjectName("sectionTitle")
        empty_title.setAlignment(Qt.AlignCenter)
        empty_text = QLabel(
            "Start the multi-robot launch, then refresh this view."
        )
        empty_text.setObjectName("sectionMeta")
        empty_text.setAlignment(Qt.AlignCenter)
        empty_layout.addWidget(empty_title)
        empty_layout.addWidget(empty_text)
        self.card_layout.addWidget(self.empty)
        self.card_layout.addStretch(1)
        self.scroll.setWidget(self.card_host)
        layout.addWidget(self.scroll, 1)

        self.refresh_button.clicked.connect(self.refresh_requested.emit)

    def set_namespaces(self, namespaces: List[str]) -> None:
        normalized = [
            _normalized_robot_namespace(value)
            for value in namespaces
            if _normalized_robot_namespace(value)
        ]
        for namespace in list(self.cards):
            if namespace not in normalized:
                card = self.cards.pop(namespace)
                self.card_layout.removeWidget(card)
                card.deleteLater()
        for namespace in normalized:
            if namespace in self.cards:
                continue
            card = RobotMissionCard(namespace)
            card.command_submitted.connect(self.command_submitted.emit)
            card.pause_resume_requested.connect(
                self.pause_resume_requested.emit
            )
            card.cancel_requested.connect(self.cancel_requested.emit)
            self.card_layout.insertWidget(
                max(0, self.card_layout.count() - 1), card
            )
            self.cards[namespace] = card
        self.empty.setVisible(not normalized)
        suffix = "" if len(normalized) == 1 else "s"
        self.count.setText(f"{len(normalized)} robot{suffix}")
        _set_object_name(
            self.count, "statusOnline" if normalized else "statusOffline"
        )

    def update_snapshots(
        self, snapshots: Dict[str, Dict[str, Any]]
    ) -> None:
        for namespace, card in self.cards.items():
            card.update_snapshot(
                snapshots.get(namespace, {"nav2_ready": False})
            )

    def append_robot_log(
        self, namespace: str, message: str, color: str = ""
    ) -> None:
        card = self.cards.get(_normalized_robot_namespace(namespace))
        if card is not None:
            card.append_log(message, color)


class PolicyInspectorPage(QWidget):


    def __init__(self):
        super().__init__()
        layout = QHBoxLayout(self)
        layout.setContentsMargins(14, 14, 14, 14)
        layout.setSpacing(10)

        self.summary = PolicySummary()
        self.summary.setMinimumWidth(360)
        self.summary.setMaximumWidth(470)
        layout.addWidget(self.summary)

        tabs = QTabWidget()
        self.policy_json = self._json_view()
        self.raw_json = self._json_view()
        self.validation_json = self._json_view()
        self.metadata_json = self._json_view()
        tabs.addTab(self.policy_json, "Validated")
        tabs.addTab(self.raw_json, "Raw output")
        tabs.addTab(self.validation_json, "Validator")
        tabs.addTab(self.metadata_json, "LLM metadata")
        layout.addWidget(tabs, 1)

    @staticmethod
    def _json_view() -> QPlainTextEdit:
        view = QPlainTextEdit()
        view.setObjectName("jsonView")
        view.setReadOnly(True)
        view.setLineWrapMode(QPlainTextEdit.NoWrap)
        return view

    def update_snapshot(self, snapshot: Dict[str, Any]) -> None:
        self.summary.update_policy(snapshot)
        self.policy_json.setPlainText(_pretty_json(snapshot.get("policy")))
        self.raw_json.setPlainText(_pretty_json(snapshot.get("raw_policy")))
        self.validation_json.setPlainText(
            _pretty_json(snapshot.get("validation"))
        )
        self.metadata_json.setPlainText(
            _pretty_json(snapshot.get("llm_metadata"))
        )


class ActivityPage(QWidget):


    save_requested = pyqtSignal()
    clear_requested = pyqtSignal()

    def __init__(self):
        super().__init__()
        layout = QVBoxLayout(self)
        layout.setContentsMargins(14, 14, 14, 14)
        layout.setSpacing(10)

        history_surface = Surface()
        history_layout = QVBoxLayout(history_surface)
        history_layout.setContentsMargins(12, 10, 12, 12)
        history_title = QLabel("Command history")
        history_title.setObjectName("sectionTitle")
        history_layout.addWidget(history_title)
        self.history = QTableWidget(0, 4)
        self.history.setHorizontalHeaderLabels(
            ["Time", "Command", "Status", "Policy version"]
        )
        self.history.setAlternatingRowColors(True)
        self.history.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.history.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.history.verticalHeader().setVisible(False)
        self.history.horizontalHeader().setSectionResizeMode(
            0, QHeaderView.ResizeToContents
        )
        self.history.horizontalHeader().setSectionResizeMode(
            1, QHeaderView.Stretch
        )
        self.history.horizontalHeader().setSectionResizeMode(
            2, QHeaderView.ResizeToContents
        )
        self.history.horizontalHeader().setSectionResizeMode(
            3, QHeaderView.ResizeToContents
        )
        history_layout.addWidget(self.history)
        layout.addWidget(history_surface, 1)

        log_surface = Surface()
        log_layout = QVBoxLayout(log_surface)
        log_layout.setContentsMargins(12, 10, 12, 12)
        top = QHBoxLayout()
        title = QLabel("Activity log")
        title.setObjectName("sectionTitle")
        self.filter = QComboBox()
        self.filter.addItems(["All", "Warnings", "Errors"])
        self.filter.setFixedWidth(112)
        self.save_button = QToolButton()
        self.save_button.setIcon(
            self.style().standardIcon(QStyle.SP_DialogSaveButton)
        )
        self.save_button.setToolTip("Save activity log")
        self.clear_button = QToolButton()
        self.clear_button.setIcon(
            self.style().standardIcon(QStyle.SP_DialogResetButton)
        )
        self.clear_button.setToolTip("Clear activity log")
        top.addWidget(title)
        top.addStretch(1)
        top.addWidget(self.filter)
        top.addWidget(self.save_button)
        top.addWidget(self.clear_button)
        log_layout.addLayout(top)
        self.log = QTextEdit()
        self.log.setObjectName("activityLog")
        self.log.setReadOnly(True)
        self.log.setMinimumHeight(180)
        log_layout.addWidget(self.log)
        layout.addWidget(log_surface, 1)

        self.save_button.clicked.connect(self.save_requested.emit)
        self.clear_button.clicked.connect(self.clear_requested.emit)


class PolicyBridgeWindow(QMainWindow):


    def __init__(
        self,
        node: Optional[GuiPolicyBridgeNode],
        signals: Optional[GuiSignals] = None,
        multi_manager: Optional[MultiRobotBridgeManager] = None,
        on_close: Optional[Callable[[], None]] = None,
        demo_mode: bool = False,
    ):
        super().__init__()
        self.node = node
        self.multi_manager = multi_manager
        self.on_close = on_close
        self.demo_mode = demo_mode
        self._closing = False
        self._logs: List[Tuple[str, str, str]] = []
        self._history: List[Dict[str, str]] = []
        self._last_snapshot: Dict[str, Any] = {}
        self._last_policy_version = 0
        self._last_policy_render_key = ""
        self._last_submitted_command = ""
        self._current_stage = -1
        self._log_render_pending = False
        self._activity_log_dirty = False
        self._available_models: List[str] = []
        self._demo_multi_snapshots: Dict[str, Dict[str, Any]] = {}
        self._model_list_error = ""
        self._model_refreshing = False
        self._model_menu = QMenu(self)
        self._map_spec: Optional[RosMapSpec] = None
        self._waypoints: Dict[str, Waypoint] = {}
        self._saved_waypoints: Dict[str, Waypoint] = {}
        self._zones: Dict[str, ZoneBounds] = {}
        self._saved_zones: Dict[str, ZoneBounds] = {}
        self._waypoint_config_path: Optional[Path] = None
        self._settings = QSettings("Policy Bridge", "Policy Bridge")
        self._map_search_dirs = {Path.home() / "map"}

        self.setWindowTitle("Policy Bridge")
        self.setMinimumSize(1120, 720)
        self.resize(1440, 900)
        self.setStyleSheet(APP_STYLESHEET)
        self._build_ui()

        if signals is not None:
            signals.log_event.connect(self.on_log_event)
            signals.models_event.connect(self._on_models_loaded)
            for message, color in signals.activate():
                self.on_log_event(message, color)
        if self.multi_manager is not None:
            self.multi_manager.robots_changed.connect(
                self.multi_page.set_namespaces
            )
            self.multi_manager.robot_log_event.connect(
                self.on_robot_log_event
            )

        self.refresh_timer = QTimer(self)
        self.refresh_timer.setInterval(250)
        self.refresh_timer.timeout.connect(self.refresh_snapshot)
        self.refresh_timer.start()

        self.multi_discovery_timer = QTimer(self)
        self.multi_discovery_timer.setInterval(2000)
        self.multi_discovery_timer.timeout.connect(self.refresh_multi_robots)
        if self.multi_manager is not None:
            self.multi_discovery_timer.start()
            QTimer.singleShot(100, self.refresh_multi_robots)

        self.context_timer = QTimer(self)
        self.context_timer.setInterval(500)
        self.context_timer.timeout.connect(self._monitor_ros_context)
        if not self.demo_mode and self.node is not None:
            self.context_timer.start()

        if self.demo_mode:
            self._available_models = list(PREFERRED_GUI_MODELS)
            self._apply_demo_state()
            self.multi_page.set_namespaces(["tb1", "tb2"])
            demo_multi = {
                "nav2_ready": True,
                "mission_active": False,
                "configured_zones": list("ABCDE"),
                "forbidden": ["B"],
                "soft_costs": {},
                "object_rules": [],
                "route": [],
                "navigation_progress": {},
            }
            self._demo_multi_snapshots = {
                "tb1": dict(demo_multi),
                "tb2": dict(demo_multi),
            }
            self.multi_page.update_snapshots(self._demo_multi_snapshots)
        elif self.node is not None:
            QTimer.singleShot(150, self.refresh_available_models)
        QTimer.singleShot(200, self.load_default_map)

    def _monitor_ros_context(self) -> None:

        if self.node is None or self.node.ros_context_is_active():
            return
        self.context_timer.stop()
        self.composer.set_available(False)
        _set_object_name(self.connection, "statusOffline")
        self.connection.setText("ROS 2 stopped")
        self.map_page.set_status(
            "ROS 2 communication stopped; restart Policy Bridge", "error"
        )
        self.on_log_event(
            "ROS 2 communication stopped. Restart Policy Bridge before "
            "editing zones or starting another mission.",
            "red",
        )

    def _build_ui(self) -> None:
        root = QWidget()
        root.setObjectName("appRoot")
        self.setCentralWidget(root)
        root_layout = QHBoxLayout(root)
        root_layout.setContentsMargins(0, 0, 0, 0)
        root_layout.setSpacing(0)

        self.sidebar = self._build_sidebar()
        root_layout.addWidget(self.sidebar)

        content = QWidget()
        content_layout = QVBoxLayout(content)
        content_layout.setContentsMargins(0, 0, 0, 0)
        content_layout.setSpacing(0)
        self.header = self._build_header()
        content_layout.addWidget(self.header)

        self.pages = QStackedWidget()
        self.pages.setObjectName("pageStack")
        self.mission_page = self._build_mission_page()
        self.multi_page = MultiRobotMissionPage()
        self.map_page = WaypointMapPage()
        self.policy_page = PolicyInspectorPage()
        self.activity_page = ActivityPage()
        self.pages.addWidget(self.mission_page)
        self.pages.addWidget(self.multi_page)
        self.pages.addWidget(self.map_page)
        self.pages.addWidget(self.policy_page)
        self.pages.addWidget(self.activity_page)
        content_layout.addWidget(self.pages, 1)
        root_layout.addWidget(content, 1)

        self.activity_page.filter.currentTextChanged.connect(
            self._render_activity_log
        )
        self.activity_page.save_requested.connect(self.save_log)
        self.activity_page.clear_requested.connect(self.clear_log)
        self.multi_page.command_submitted.connect(
            self.submit_multi_robot_command
        )
        self.multi_page.pause_resume_requested.connect(
            self.toggle_multi_robot_pause
        )
        self.multi_page.cancel_requested.connect(
            self.cancel_multi_robot_mission
        )
        self.multi_page.refresh_requested.connect(self.refresh_multi_robots)
        self.multi_page.model_selector.clicked.connect(
            lambda: self.show_model_menu(self.multi_page.model_selector)
        )
        self.map_page.map_open_requested.connect(self.load_map)
        self.map_page.map_selected.connect(self.load_map)
        self.map_page.map_reload_requested.connect(self.reload_map)
        self.map_page.waypoint_update_requested.connect(
            self.update_waypoint
        )
        self.map_page.waypoint_add_requested.connect(self.add_waypoint)
        self.map_page.waypoint_remove_requested.connect(
            self.remove_waypoint
        )
        self.map_page.zone_update_requested.connect(self.update_zone)
        self.map_page.zone_add_requested.connect(self.add_zone)
        self.map_page.zone_remove_requested.connect(self.remove_zone)
        self.map_page.save_requested.connect(self.save_waypoints)
        self.map_page.revert_requested.connect(self.revert_waypoints)

    def _build_sidebar(self) -> QFrame:
        sidebar = QFrame()
        sidebar.setObjectName("sidebar")
        sidebar.setFixedWidth(228)
        layout = QVBoxLayout(sidebar)
        layout.setContentsMargins(12, 18, 12, 14)
        layout.setSpacing(7)

        brand = QHBoxLayout()
        mark = QLabel("PB")
        mark.setObjectName("brandMark")
        mark.setAlignment(Qt.AlignCenter)
        mark.setFixedSize(36, 36)
        names = QVBoxLayout()
        names.setSpacing(0)
        name = QLabel("Policy Bridge")
        name.setObjectName("brandName")
        subtitle = QLabel("ROS 2 Navigation")
        subtitle.setObjectName("brandSubtitle")
        names.addWidget(name)
        names.addWidget(subtitle)
        brand.addWidget(mark)
        brand.addSpacing(7)
        brand.addLayout(names)
        layout.addLayout(brand)
        layout.addSpacing(18)

        self.nav_buttons: List[QPushButton] = []
        for index, label in enumerate(
            (
                "Mission (single robot)",
                "Mission (multi robot)",
                "Map / Setup",
                "Policy Inspector",
                "Activity",
            )
        ):
            button = QPushButton(label)
            button.setObjectName("navButton")
            button.setCheckable(True)
            button.setAutoExclusive(True)
            button.clicked.connect(
                lambda checked, page=index: self.switch_page(page)
            )
            layout.addWidget(button)
            self.nav_buttons.append(button)
        self.nav_buttons[0].setChecked(True)
        layout.addStretch(1)

        divider = QFrame()
        divider.setFrameShape(QFrame.HLine)
        divider.setStyleSheet("color: #2b3748;")
        layout.addWidget(divider)
        self.sidebar_model = QLabel("Model\nnot configured")
        self.sidebar_model.setObjectName("sidebarMeta")
        self.sidebar_model.setWordWrap(True)
        layout.addWidget(self.sidebar_model)
        return sidebar

    def _build_header(self) -> QFrame:
        header = QFrame()
        header.setObjectName("headerBar")
        header.setFixedHeight(70)
        layout = QHBoxLayout(header)
        layout.setContentsMargins(16, 8, 16, 8)

        titles = QVBoxLayout()
        titles.setSpacing(1)
        self.page_title = QLabel("Mission Control")
        self.page_title.setObjectName("pageTitle")
        self.page_subtitle = QLabel(
            "Natural-language policy execution for Nav2"
        )
        self.page_subtitle.setObjectName("pageSubtitle")
        titles.addWidget(self.page_title)
        titles.addWidget(self.page_subtitle)
        layout.addLayout(titles)
        layout.addStretch(1)

        self.connection = QLabel("Nav2 checking")
        self.connection.setObjectName("statusBusy")
        layout.addWidget(self.connection)
        return header

    def _build_mission_page(self) -> QWidget:
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        page = QWidget()
        page.setMinimumHeight(790)
        layout = QVBoxLayout(page)
        layout.setContentsMargins(14, 12, 14, 14)
        layout.setSpacing(9)

        self.composer = CommandComposer()
        self.composer.command_submitted.connect(self.submit_command)
        self.composer.reset_requested.connect(self.reset_policy)
        self.model_metric = ModelMetricTile("Not configured", compact=True)
        self.model_metric.clicked.connect(self.show_model_menu)
        self.composer.add_model_selector(self.model_metric)
        layout.addWidget(self.composer)

        splitter = QSplitter(Qt.Horizontal)
        splitter.setChildrenCollapsible(False)
        self.policy_summary = PolicySummary()
        self.mission_progress = MissionProgress()
        self.mission_progress.fire_changed.connect(self.set_fire_alarm)
        self.mission_progress.battery_changed.connect(self.set_battery)
        self.mission_progress.pause_resume_requested.connect(
            self.toggle_pause_mission
        )
        self.mission_progress.cancel_requested.connect(self.cancel_mission)
        splitter.addWidget(self.policy_summary)
        splitter.addWidget(self.mission_progress)
        splitter.setMinimumHeight(330)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 2)
        splitter.setSizes([720, 470])
        layout.addWidget(splitter, 1)

        log_surface = Surface()
        log_layout = QVBoxLayout(log_surface)
        log_layout.setContentsMargins(12, 9, 12, 10)
        log_title = QHBoxLayout()
        title = QLabel("Live activity")
        title.setObjectName("sectionTitle")
        self.log_count = QLabel("0 events")
        self.log_count.setObjectName("sectionMeta")
        log_title.addWidget(title)
        log_title.addStretch(1)
        log_title.addWidget(self.log_count)
        log_layout.addLayout(log_title)
        self.live_log = QTextEdit()
        self.live_log.setObjectName("activityLog")
        self.live_log.setReadOnly(True)
        self.live_log.setMinimumHeight(92)
        self.live_log.setMaximumHeight(132)
        log_layout.addWidget(self.live_log)
        layout.addWidget(log_surface)
        scroll.setWidget(page)
        return scroll

    def show_model_menu(
        self, anchor_widget: Optional[QWidget] = None
    ) -> None:

        if not self._available_models and not self._model_refreshing:
            self.refresh_available_models()
        self._populate_model_menu()
        anchor_widget = anchor_widget or self.model_metric
        self._model_menu.setMinimumWidth(anchor_widget.width())
        anchor = anchor_widget.mapToGlobal(
            anchor_widget.rect().bottomLeft()
        )
        self._model_menu.popup(anchor)

    def _populate_model_menu(self) -> None:
        self._model_menu.clear()
        heading = self._model_menu.addAction("Available Ollama models")
        heading.setEnabled(False)
        self._model_menu.addSeparator()

        current = self.model_metric.value.text()
        available = set(self._available_models)
        for model in self._available_models:
            action = self._model_menu.addAction(model)
            action.setCheckable(True)
            action.setChecked(model == current)
            if self.node is not None:
                endpoint = getattr(
                    self.node, "_gui_model_endpoints", {}
                ).get(model, "")
                if endpoint:
                    action.setToolTip(endpoint)
            action.triggered.connect(
                lambda checked=False, selected=model: (
                    self.select_llm_model(selected)
                )
            )

        if not available:
            if self._model_refreshing:
                status = "Searching local Ollama servers..."
            elif self._model_list_error:
                status = "Ollama model list unavailable"
            else:
                status = "No installed models found"
            unavailable = self._model_menu.addAction(status)
            unavailable.setEnabled(False)

        if (
            current
            and current not in available
            and current != "Not configured"
        ):
            self._model_menu.addSeparator()
            configured = self._model_menu.addAction(
                f"Configured: {current} (not reported by Ollama)"
            )
            configured.setEnabled(False)

        self._model_menu.addSeparator()
        refresh = self._model_menu.addAction("Refresh model list")
        refresh.setEnabled(not self._model_refreshing)
        refresh.triggered.connect(self.refresh_available_models)
        if self._model_list_error:
            self._model_menu.setToolTip(self._model_list_error)
        else:
            self._model_menu.setToolTip("")

    def refresh_available_models(self) -> None:

        if self.demo_mode:
            self._model_list_error = ""
            self._model_refreshing = False
            self._populate_model_menu()
            return
        if self.node is None or self._model_refreshing:
            return
        self._model_refreshing = bool(
            self.node.request_model_list_from_gui()
        )
        self._populate_model_menu()

    def _on_models_loaded(self, models: object, error: str) -> None:
        loaded = models if isinstance(models, list) else []
        self._available_models = _ordered_model_inventory(
            list(dict.fromkeys(map(str, loaded)))
        )
        self._model_list_error = error
        self._model_refreshing = False
        if self.node is not None:
            current = self.model_metric.value.text()
            if current in self._available_models:
                accepted, reason = self.node.select_model_from_gui(current)
                if not accepted:
                    self.on_log_event(
                        f"Model endpoint selection failed: {reason}",
                        "yellow",
                    )
        self._populate_model_menu()

    def select_llm_model(self, model: str) -> None:

        current = self.model_metric.value.text()
        if model == current:
            return
        if self.demo_mode:
            self._last_snapshot["llm_model"] = model
            self.on_log_event(
                f"Active LLM model: {model}. New commands will use it.",
                "green",
            )
            self.refresh_snapshot()
            return
        if self.node is None:
            return
        accepted, reason = self.node.select_model_from_gui(model)
        if not accepted:
            QMessageBox.warning(
                self,
                "Model selection failed",
                reason or "The selected model could not be activated.",
            )
            return
        if self.multi_manager is not None:
            errors = self.multi_manager.select_model(model)
            if errors:
                self.on_log_event(
                    "Multi-robot model update warning: " + "; ".join(errors),
                    "yellow",
                )
        self.model_metric.value.setText(model)
        self.multi_page.model_selector.value.setText(model)
        self.sidebar_model.setText(f"Active model\n{model}")
        self._populate_model_menu()

    def load_default_map(self) -> None:

        factory_path = Path.home() / "map" / "factory6.yaml"
        if self.node is not None:
            configured = Path(
                getattr(
                    self.node,
                    "_gui_map_yaml",
                    factory_path,
                )
            ).expanduser()
            saved = str(self._settings.value("last_map_yaml", ""))
            saved_path = Path(saved).expanduser() if saved else None
            if (
                configured.resolve() == factory_path.resolve()
                and saved_path is not None
                and saved_path.is_file()
            ):
                path = saved_path
            else:
                path = configured
        else:
            path = factory_path
        self._map_search_dirs.add(path.parent)
        self.refresh_map_choices(str(path.resolve()))
        self.load_map(str(path), force=True)

    def refresh_map_choices(self, current_path: str = "") -> None:

        paths = discover_ros_maps(self._map_search_dirs)
        if current_path:
            resolved = Path(current_path).expanduser().resolve()
            if resolved.is_file() and resolved not in paths:
                paths.append(resolved)
        paths.sort(key=lambda item: (item.stem.lower(), str(item)))
        self.map_page.set_map_choices(
            [str(path) for path in paths], current_path
        )

    def _runtime_waypoints(self) -> Dict[str, Waypoint]:
        if self.node is not None:
            return self.node.waypoints_from_gui()
        if self._waypoints:
            return copy.deepcopy(self._waypoints)
        return copy.deepcopy(DEFAULT_FACTORY_WAYPOINTS)

    def _replace_runtime_waypoints(
        self, waypoints: Dict[str, Waypoint]
    ) -> Tuple[bool, str]:
        if self.node is not None:
            accepted, reason = self.node.replace_waypoints_from_gui(
                waypoints
            )
            if accepted and self.multi_manager is not None:
                errors = self.multi_manager.replace_waypoints(waypoints)
                if errors:
                    return False, "; ".join(errors)
            return accepted, reason
        try:
            normalized = normalize_waypoints(waypoints)
        except ValueError as exc:
            return False, str(exc)
        self._waypoints = normalized
        return True, ""

    def _runtime_zones(self) -> Dict[str, ZoneBounds]:
        if self.node is not None:
            return self.node.zones_from_gui()
        if self._zones:
            return copy.deepcopy(self._zones)
        return copy.deepcopy(DEFAULT_FACTORY_ZONES)

    def _replace_runtime_zones(
        self, zones: Dict[str, ZoneBounds]
    ) -> Tuple[bool, str]:
        if self.node is not None:
            accepted, reason = self.node.replace_zones_from_gui(zones)
            if accepted and self.multi_manager is not None:
                errors = self.multi_manager.replace_zones(zones)
                if errors:
                    return False, "; ".join(errors)
            return accepted, reason
        try:
            normalized = normalize_zones(zones)
        except ValueError as exc:
            return False, str(exc)
        self._zones = normalized
        return True, ""

    def _confirm_map_change(self) -> bool:
        if not self.map_page.dirty:
            return True
        answer = QMessageBox.question(
            self,
            "Unsaved map setup",
            "Save waypoint and zone changes before continuing?",
            QMessageBox.Save | QMessageBox.Discard | QMessageBox.Cancel,
            QMessageBox.Save,
        )
        if answer == QMessageBox.Cancel:
            return False
        if answer == QMessageBox.Save:
            return self.save_waypoints()
        self._replace_runtime_waypoints(self._saved_waypoints)
        self._replace_runtime_zones(self._saved_zones)
        return True

    def load_map(self, path: str, force: bool = False) -> bool:

        if not force and not self._confirm_map_change():
            current = str(self._map_spec.yaml_path) if self._map_spec else ""
            self.refresh_map_choices(current)
            return False
        try:
            spec = load_ros_map_yaml(path)
            map_image = QPixmap(str(spec.image_path))
            if map_image.isNull():
                raise ValueError(
                    f"Map image could not be displayed: {spec.image_path}"
                )

            def zones_fit_map(values: Dict[str, ZoneBounds]) -> bool:
                return all(
                    spec.contains_world(
                        x,
                        y,
                        map_image.width(),
                        map_image.height(),
                    )
                    for bounds in values.values()
                    for x, y in (
                        (bounds.x_min, bounds.y_min),
                        (bounds.x_max, bounds.y_min),
                        (bounds.x_max, bounds.y_max),
                        (bounds.x_min, bounds.y_max),
                    )
                )

            runtime_zones = self._runtime_zones()
            compatible_runtime_zones = (
                runtime_zones if zones_fit_map(runtime_zones) else {}
            )
            self._map_search_dirs.add(spec.yaml_path.parent)
            self.refresh_map_choices(str(spec.yaml_path))
            config_path = waypoint_config_path(spec.yaml_path)
            if config_path.is_file():
                annotations = load_map_annotations(config_path)
                waypoints = annotations.waypoints
                zones = (
                    annotations.zones
                    if annotations.zones_defined
                    else compatible_runtime_zones
                )
                source = (
                    f"Loaded {len(waypoints)} waypoints and "
                    f"{len(zones)} zones"
                )
            else:
                waypoints = self._runtime_waypoints()
                zones = compatible_runtime_zones
                source = (
                    f"Using {len(waypoints)} runtime waypoints and "
                    f"{len(zones)} zones"
                )
            accepted, reason = self._replace_runtime_waypoints(waypoints)
            if not accepted:
                raise ValueError(reason)
            for name, bounds in zones.items():
                if not zones_fit_map({name: bounds}):
                    raise ValueError(
                        f"Saved Zone {name} extends outside the selected map"
                    )
            accepted, reason = self._replace_runtime_zones(zones)
            if not accepted:
                raise ValueError(reason)
            self.map_page.set_map(spec)
        except ValueError as exc:
            self.map_page.set_status(str(exc), "error")
            current = str(self._map_spec.yaml_path) if self._map_spec else ""
            self.refresh_map_choices(current)
            if not force:
                QMessageBox.warning(self, "Map could not be loaded", str(exc))
            return False

        self._map_spec = spec
        self._waypoints = copy.deepcopy(waypoints)
        self._saved_waypoints = copy.deepcopy(waypoints)
        self._zones = copy.deepcopy(zones)
        self._saved_zones = copy.deepcopy(zones)
        self._waypoint_config_path = config_path
        if self.node is not None:
            accepted, reason = self.node.set_map_path_from_gui(
                str(spec.yaml_path)
            )
            if not accepted:
                self.on_log_event(
                    f"Map parameter update failed: {reason}", "yellow"
                )
        if not self.demo_mode:
            self._settings.setValue("last_map_yaml", str(spec.yaml_path))
        self.map_page.set_waypoints(self._waypoints, dirty=False)
        self.map_page.set_zones(self._zones, dirty=False)
        outside = self._outside_map_waypoints()
        if outside:
            self.map_page.set_status(
                f"{source}; outside map: {', '.join(outside)}",
                "busy",
            )
        else:
            self.map_page.set_status(source, "online")
        self.on_log_event(
            (
                f"Map loaded: {spec.yaml_path.name} "
                f"({len(waypoints)} waypoints, {len(zones)} zones)"
            ),
            "green",
        )
        return True

    def reload_map(self) -> None:
        if self._map_spec is None:
            self.load_default_map()
            return
        if not self._confirm_map_change():
            return
        self.load_map(str(self._map_spec.yaml_path), force=True)

    def _outside_map_waypoints(self) -> List[str]:
        if self._map_spec is None:
            return []
        return [
            name
            for name, (x, y, _yaw) in self._waypoints.items()
            if not self._map_spec.contains_world(
                x,
                y,
                self.map_page.map_view.image_width,
                self.map_page.map_view.image_height,
            )
        ]

    def update_waypoint(
        self, name: str, x: float, y: float, yaw: float
    ) -> None:
        if self._map_spec is None or name not in self._waypoints:
            return
        if not self._map_spec.contains_world(
            x,
            y,
            self.map_page.map_view.image_width,
            self.map_page.map_view.image_height,
        ):
            QMessageBox.warning(
                self,
                "Coordinate outside map",
                "The selected map coordinate is outside the loaded map.",
            )
            return
        updated = copy.deepcopy(self._waypoints)
        updated[name] = (float(x), float(y), float(yaw))
        accepted, reason = self._replace_runtime_waypoints(updated)
        if not accepted:
            QMessageBox.warning(self, "Waypoint update failed", reason)
            return
        self._waypoints = updated
        self.map_page.set_waypoints(updated, selected=name, dirty=True)
        self.map_page.set_status(
            f"{name} set to ({x:.2f}, {y:.2f})", "busy"
        )
        self.on_log_event(
            f"Waypoint updated: {name}=({x:.2f}, {y:.2f}, {yaw:.3f})",
            "blue",
        )

    def add_waypoint(self, name: str) -> None:
        if self._map_spec is None or name in self._waypoints:
            return
        if self._map_spec.contains_world(
            0.0,
            0.0,
            self.map_page.map_view.image_width,
            self.map_page.map_view.image_height,
        ):
            initial = (0.0, 0.0, 0.0)
        else:
            initial_x, initial_y = self._map_spec.pixel_to_world(
                self.map_page.map_view.image_width / 2.0,
                self.map_page.map_view.image_height / 2.0,
                self.map_page.map_view.image_height,
            )
            initial = (initial_x, initial_y, 0.0)
        updated = copy.deepcopy(self._waypoints)
        updated[name] = initial
        accepted, reason = self._replace_runtime_waypoints(updated)
        if not accepted:
            QMessageBox.warning(self, "Waypoint add failed", reason)
            return
        self._waypoints = normalize_waypoints(updated)
        self.map_page.set_waypoints(
            self._waypoints, selected=name, dirty=True
        )
        self.map_page.begin_placement(name)
        self.map_page.set_status(f"{name} added", "busy")

    def remove_waypoint(self, name: str) -> None:
        if name not in self._waypoints:
            return
        updated = copy.deepcopy(self._waypoints)
        del updated[name]
        accepted, reason = self._replace_runtime_waypoints(updated)
        if not accepted:
            QMessageBox.warning(self, "Waypoint removal failed", reason)
            return
        self._waypoints = updated
        self.map_page.set_waypoints(updated, dirty=True)
        self.map_page.set_status(f"{name} removed", "busy")
        self.on_log_event(f"Waypoint removed: {name}", "yellow")

    def _zone_inside_map(self, bounds: ZoneBounds) -> bool:
        if self._map_spec is None:
            return False
        return all(
            self._map_spec.contains_world(
                x,
                y,
                self.map_page.map_view.image_width,
                self.map_page.map_view.image_height,
            )
            for x, y in (
                (bounds.x_min, bounds.y_min),
                (bounds.x_max, bounds.y_min),
                (bounds.x_max, bounds.y_max),
                (bounds.x_min, bounds.y_max),
            )
        )

    def update_zone(
        self,
        name: str,
        x_min: float,
        y_min: float,
        x_max: float,
        y_max: float,
    ) -> None:
        if self._map_spec is None or name not in self._zones:
            return
        try:
            bounds = normalize_zones(
                {name: (x_min, y_min, x_max, y_max)}
            )[name]
        except ValueError as exc:
            QMessageBox.warning(self, "Invalid zone bounds", str(exc))
            return
        if not self._zone_inside_map(bounds):
            QMessageBox.warning(
                self,
                "Zone outside map",
                "All four zone corners must be inside the loaded map.",
            )
            return
        updated = copy.deepcopy(self._zones)
        updated[name] = bounds
        accepted, reason = self._replace_runtime_zones(updated)
        if not accepted:
            QMessageBox.warning(self, "Zone update failed", reason)
            return
        self._zones = normalize_zones(updated)
        self.map_page.set_zones(self._zones, selected=name, dirty=True)
        self.map_page.set_status(
            f"Zone {name} bounds updated", "busy"
        )
        self.on_log_event(
            (
                f"Zone updated: {name}="
                f"[{bounds.x_min:.2f}, {bounds.y_min:.2f}] to "
                f"[{bounds.x_max:.2f}, {bounds.y_max:.2f}]"
            ),
            "blue",
        )

    def add_zone(self, name: str) -> None:
        if self._map_spec is None or name in self._zones:
            return
        width = self.map_page.map_view.image_width
        height = self.map_page.map_view.image_height
        center_x, center_y = self._map_spec.pixel_to_world(
            width / 2.0, height / 2.0, height
        )
        span = max(self._map_spec.resolution * 8.0, 0.5)
        bounds = ZoneBounds(
            center_x - span,
            center_y - span,
            center_x + span,
            center_y + span,
        )
        updated = copy.deepcopy(self._zones)
        updated[name] = bounds
        accepted, reason = self._replace_runtime_zones(updated)
        if not accepted:
            QMessageBox.warning(self, "Zone add failed", reason)
            return
        self._zones = normalize_zones(updated)
        self.map_page.set_zones(self._zones, selected=name, dirty=True)
        self.map_page.begin_zone_placement(name)
        self.map_page.set_status(
            f"Zone {name} added; drag its bounds on the map", "busy"
        )

    def remove_zone(self, name: str) -> None:
        if name not in self._zones:
            return
        updated = copy.deepcopy(self._zones)
        del updated[name]
        accepted, reason = self._replace_runtime_zones(updated)
        if not accepted:
            QMessageBox.warning(self, "Zone removal failed", reason)
            return
        self._zones = updated
        self.map_page.set_zones(updated, dirty=True)
        self.map_page.set_status(f"Zone {name} removed", "busy")
        self.on_log_event(f"Zone removed: {name}", "yellow")

    def save_waypoints(self) -> bool:
        if self._map_spec is None or self._waypoint_config_path is None:
            return False
        try:
            saved_path = save_waypoint_config(
                self._waypoint_config_path,
                self._map_spec.yaml_path,
                self._waypoints,
                self._zones,
            )
        except (OSError, ValueError) as exc:
            QMessageBox.warning(self, "Waypoint save failed", str(exc))
            return False
        self._saved_waypoints = copy.deepcopy(self._waypoints)
        self._saved_zones = copy.deepcopy(self._zones)
        self.map_page.set_dirty(False)
        self.map_page.set_status(
            (
                f"Saved {len(self._waypoints)} waypoints and "
                f"{len(self._zones)} zones"
            ),
            "online",
        )
        self.on_log_event(f"Waypoints saved: {saved_path}", "green")
        return True

    def revert_waypoints(self) -> None:
        accepted, reason = self._replace_runtime_waypoints(
            self._saved_waypoints
        )
        if not accepted:
            QMessageBox.warning(self, "Waypoint revert failed", reason)
            return
        accepted, reason = self._replace_runtime_zones(self._saved_zones)
        if not accepted:
            QMessageBox.warning(self, "Zone revert failed", reason)
            return
        self._waypoints = copy.deepcopy(self._saved_waypoints)
        self._zones = copy.deepcopy(self._saved_zones)
        self.map_page.set_waypoints(self._waypoints, dirty=False)
        self.map_page.set_zones(self._zones, dirty=False)
        self.map_page.set_status("Saved map setup restored", "online")

    def switch_page(self, index: int) -> None:
        self.pages.setCurrentIndex(index)
        titles = (
            (
                "Single-Robot Mission",
                "Natural-language policy execution for one Nav2 robot",
            ),
            (
                "Multi-Robot Missions",
                "Independent routes with a shared fleet cost policy",
            ),
            (
                "Map Setup",
                "Map-frame waypoint and policy-zone configuration",
            ),
            ("Policy Inspector", "Raw, validated, and active policy state"),
            ("Activity", "Command history and runtime events"),
        )
        title, subtitle = titles[index]
        self.page_title.setText(title)
        self.page_subtitle.setText(subtitle)
        if index == 1:
            self.refresh_multi_robots()
        if index == 4 and self._activity_log_dirty:
            self._render_activity_log()
            self._activity_log_dirty = False

    def submit_command(self, command: str) -> None:
        if not command.strip():
            return
        if self.node is None and not self.demo_mode:
            return
        if self.demo_mode:
            accepted = True
        else:
            accepted = bool(self.node and self.node.submit_from_gui(command))
        if not accepted:
            QMessageBox.warning(
                self,
                "Nav2 unavailable",
                "Start Nav2 before submitting a navigation command.",
            )
            return
        self._last_submitted_command = command
        mission_active = bool(self._last_snapshot.get("mission_active", False))
        self._add_history(
            command, "Queued" if mission_active else "Interpreting"
        )
        if mission_active:
            queued = max(
                1,
                int(self._last_snapshot.get("queued_missions", 0) or 0) + 1,
            )
            self.mission_progress.queue_status.setText(
                f"Queued missions: {queued}"
            )
        else:
            self._current_stage = 1
            self.mission_progress.set_stage(1)
            self.mission_progress.set_mission_state("Interpreting", "busy")
            self.mission_progress.destination.setText("Building a safe policy")
            self.mission_progress.remaining.setText("Remaining distance: -")
            self.mission_progress.progress.setRange(0, 0)
        if self.demo_mode:
            self.on_log_event(f"Received command: {command}", "cyan")

    def refresh_multi_robots(self) -> None:

        if self.demo_mode:
            return
        if self.multi_manager is None:
            self.multi_page.set_namespaces([])
            return
        namespaces = self.multi_manager.refresh_discovery()
        self.multi_page.set_namespaces(namespaces)
        self.multi_page.update_snapshots(self.multi_manager.snapshots())

    def submit_multi_robot_command(
        self, namespace: str, command: str
    ) -> None:

        card = self.multi_page.cards.get(namespace)
        if self.demo_mode:
            if card is not None:
                card.append_log(f"Received command: {command}", "cyan")
                card.append_log(
                    "Policy shared with fleet; route sent to this robot.",
                    "green",
                )
            return
        accepted = bool(
            self.multi_manager
            and self.multi_manager.submit(namespace, command)
        )
        if not accepted:
            QMessageBox.warning(
                self,
                "Robot Nav2 unavailable",
                f"/{namespace}/follow_waypoints is not available.",
            )
            return
        if card is not None:
            card.append_log(f"Received command: {command}", "cyan")

    def toggle_multi_robot_pause(self, namespace: str) -> None:
        if self.demo_mode:
            card = self.multi_page.cards.get(namespace)
            if card is not None:
                card.append_log("Pause or resume requested.", "yellow")
            return
        if self.multi_manager is not None:
            self.multi_manager.toggle_pause(namespace)

    def cancel_multi_robot_mission(self, namespace: str) -> None:
        if self.demo_mode:
            card = self.multi_page.cards.get(namespace)
            if card is not None:
                card.append_log("Mission canceled.", "yellow")
            return
        if self.multi_manager is not None:
            self.multi_manager.cancel(namespace)

    def on_robot_log_event(
        self, namespace: str, message: str, color: str = ""
    ) -> None:
        self.multi_page.append_robot_log(namespace, message, color)

    def reset_policy(self) -> None:
        if self.demo_mode:
            self.on_log_event("Policy state reset for a new trial.", "green")
            self._apply_empty_demo_state()
            return
        if self.node is None or not self.node.reset_from_gui():
            QMessageBox.information(
                self,
                "Mission active",
                (
                    "Stop or complete the active mission before resetting "
                    "policy state."
                ),
            )

    def toggle_pause_mission(self) -> None:
        if self.demo_mode:
            paused = bool(self._last_snapshot.get("mission_paused", False))
            self._last_snapshot["mission_paused"] = not paused
            self.on_log_event(
                "Mission resumed from WP2."
                if paused
                else "Mission paused before WP2; route state retained.",
                "green" if paused else "yellow",
            )
            return
        if self.node is not None and not self.node.toggle_pause_from_gui():
            self.mission_progress.stop_button.setEnabled(False)

    def stop_mission(self) -> None:

        self.toggle_pause_mission()

    def cancel_mission(self) -> None:
        if self.demo_mode:
            self.on_log_event("Cancel requested by the operator.", "yellow")
            self.on_log_event("Mission state: CANCELED", "yellow")
            return
        if self.node is not None and not self.node.cancel_from_gui():
            self.mission_progress.cancel_button.setEnabled(False)

    def set_fire_alarm(self, enabled: bool) -> None:
        if self.demo_mode:
            self._last_snapshot["fire_alarm"] = enabled
            self.on_log_event(
                f"State updated: fire={str(enabled).lower()}", "yellow"
            )
            return
        if self.node is not None:
            self.node.set_fire_alarm_from_gui(enabled)

    def set_battery(self, value: int) -> None:
        if self.demo_mode:
            self._last_snapshot["battery_pct"] = value
            self.on_log_event(f"State updated: battery={value}%", "yellow")
            return
        if self.node is not None:
            self.node.set_battery_from_gui(value)

    def refresh_snapshot(self) -> None:
        if self.demo_mode:
            snapshot = self._last_snapshot
        elif self.node is not None:
            try:
                snapshot = self.node.gui_snapshot()
            except Exception as exc:
                self.on_log_event(f"State refresh failed: {exc}", "red")
                return
        else:
            snapshot = {}
        if not snapshot:
            return

        multi_snapshots: Dict[str, Dict[str, Any]] = {}
        if self.multi_manager is not None:
            multi_snapshots = self.multi_manager.snapshots()
            self.multi_page.update_snapshots(multi_snapshots)
        elif self.demo_mode:
            multi_snapshots = self._demo_multi_snapshots

        self._last_snapshot = snapshot
        multi_mode = self.pages.currentIndex() == 1
        if multi_mode:
            ready_count = sum(
                bool(item.get("nav2_ready", False))
                for item in multi_snapshots.values()
            )
            robot_count = len(multi_snapshots)
            nav2_ready = robot_count > 0 and ready_count == robot_count
            connection_text = (
                f"{ready_count}/{robot_count} robots connected"
                if robot_count
                else "No robots detected"
            )
        else:
            nav2_ready = bool(snapshot.get("nav2_ready", False))
            connection_text = (
                "Nav2 connected" if nav2_ready else "Nav2 offline"
            )
        _set_object_name(
            self.connection, "statusOnline" if nav2_ready else "statusOffline"
        )
        self.connection.setText(connection_text)
        self.composer.set_available(nav2_ready or self.demo_mode)

        model = str(snapshot.get("llm_model", "not configured"))
        self.model_metric.value.setText(model)
        self.multi_page.model_selector.value.setText(model)
        self.sidebar_model.setText(f"Active model\n{model}")
        version = int(snapshot.get("policy_version", 0))

        policy_render_payload = {
            key: snapshot.get(key)
            for key in (
                "policy_version",
                "configured_zones",
                "zone_geometry_subscribers",
                "forbidden",
                "soft_costs",
                "object_rules",
                "route",
                "policy",
                "raw_policy",
                "validation",
                "llm_metadata",
            )
        }
        policy_render_key = json.dumps(
            policy_render_payload,
            ensure_ascii=False,
            sort_keys=True,
            default=str,
        )
        if policy_render_key != self._last_policy_render_key:
            self._last_policy_render_key = policy_render_key
            self.policy_summary.update_policy(snapshot)
            self.policy_page.update_snapshot(snapshot)
        self.mission_progress.update_environment(snapshot)

        mission_active = bool(snapshot.get("mission_active", False))
        interpretation_active = bool(
            snapshot.get("interpretation_active", False)
        )
        command_queued = bool(snapshot.get("command_queued", False))
        mission_pending = bool(snapshot.get("mission_pending", False))
        interpreting_new_command = interpretation_active or command_queued
        mission_paused = bool(snapshot.get("mission_paused", False))



        if mission_active:
            self._current_stage = 4
            self.mission_progress.set_stage(4)
            if mission_paused:
                self.mission_progress.set_mission_state("Paused", "busy")
            elif self.mission_progress.state.text() not in {
                "Canceling", "Pausing"
            }:
                self.mission_progress.set_mission_state(
                    "Navigating", "busy"
                )
            progress = snapshot.get("navigation_progress", {}) or {}
            current = max(0, int(progress.get("current", 0) or 0))
            total = max(1, int(progress.get("total", 0) or 0))
            destination = str(progress.get("destination", "") or "")
            remaining = progress.get("remaining_m")
            if destination:
                self.mission_progress.destination.setText(destination)
            if remaining is not None:
                self.mission_progress.remaining.setText(
                    f"Remaining distance: {float(remaining):.2f} m"
                )
            self.mission_progress.progress.setRange(0, total)
            self.mission_progress.progress.setValue(
                max(0, min(total, current - 1))
            )
        elif interpreting_new_command:
            self._current_stage = 1
            self.mission_progress.set_stage(1)
            self.mission_progress.set_mission_state(
                "Interpreting", "busy"
            )
            self.mission_progress.destination.setText(
                "Building a safe policy"
            )
            self.mission_progress.remaining.setText(
                "Remaining distance: -"
            )
            self.mission_progress.progress.setRange(0, 0)
        elif mission_pending:
            self._current_stage = 3
            self.mission_progress.set_stage(3)
            self.mission_progress.set_mission_state(
                "Applying policy", "busy"
            )
            self.mission_progress.destination.setText(
                "Starting updated navigation"
            )
            self.mission_progress.remaining.setText(
                "Remaining distance: -"
            )
            self.mission_progress.progress.setRange(0, 0)

        if version > self._last_policy_version:
            self._last_policy_version = version
            history_status = (
                "Navigating"
                if mission_active and not mission_pending
                else "Validated"
            )
            self._set_latest_history(history_status, str(version))
            if not mission_active:
                self._current_stage = max(self._current_stage, 3)
                self.mission_progress.set_stage(self._current_stage)

        self.mission_progress.stop_button.setEnabled(mission_active)
        self.mission_progress.cancel_button.setEnabled(
            mission_active
            or interpretation_active
            or command_queued
            or mission_pending
        )

    def on_log_event(self, message: str, color: str = "") -> None:
        timestamp = dt.datetime.now().strftime("%H:%M:%S")
        clean_lines = [
            line for line in str(message).splitlines() if line.strip()
        ]
        for line in clean_lines:
            self._logs.append((timestamp, color or "", line.strip()))
        if len(self._logs) > 800:
            self._logs = self._logs[-800:]
        self.log_count.setText(f"{len(self._logs)} events")
        self._apply_log_state(message)
        self._activity_log_dirty = True
        if not self._log_render_pending:
            self._log_render_pending = True
            QTimer.singleShot(50, self._flush_log_render)

    def _flush_log_render(self) -> None:

        self._log_render_pending = False
        self._render_live_log()
        if self.pages.currentIndex() == 4:
            self._render_activity_log()
            self._activity_log_dirty = False

    def _apply_log_state(self, message: str) -> None:
        lower = message.lower()
        progress_match = re.search(
            r"waypoint\s+(\d+)/(\d+).*?(?:->|→)\s*(.*?)\s+"
            r"\(remaining(?:≈|~)?([0-9.]+)\s*m\)",
            message,
            flags=re.IGNORECASE,
        )

        if "received command:" in lower:
            self._current_stage = 1
            self.mission_progress.set_stage(1)
            self.mission_progress.set_mission_state("Interpreting", "busy")
        if "policy validated for command" in lower:
            self._current_stage = 3
            self.mission_progress.set_stage(3)
            self.mission_progress.set_mission_state("Validated", "online")
        if "mission start" in lower:
            self._current_stage = 4
            self.mission_progress.set_stage(4)
            self.mission_progress.set_mission_state("Navigating", "busy")
            self.mission_progress.progress.setRange(0, 1)
            self.mission_progress.progress.setValue(0)
            self._set_latest_history("Navigating")
        if progress_match:
            current = int(progress_match.group(1))
            total = max(1, int(progress_match.group(2)))
            destination = progress_match.group(3).strip()
            remaining = float(progress_match.group(4))
            self.mission_progress.destination.setText(destination)
            self.mission_progress.remaining.setText(
                f"Remaining distance: {remaining:.2f} m"
            )
            self.mission_progress.progress.setRange(0, total)
            self.mission_progress.progress.setValue(current - 1)
        if "all waypoints reached" in lower:
            self._current_stage = 5
            self.mission_progress.set_stage(5)
            maximum = max(1, self.mission_progress.progress.maximum())
            self.mission_progress.progress.setRange(0, maximum)
            self.mission_progress.progress.setValue(maximum)
        if "mission done: succeeded" in lower:
            self._current_stage = len(STAGE_LABELS)
            self.mission_progress.set_stage(self._current_stage)
            self.mission_progress.set_mission_state("Succeeded", "online")
            self.mission_progress.destination.setText("Mission complete")
            self.mission_progress.remaining.setText(
                "Remaining distance: 0.00 m"
            )
            self._set_latest_history("Succeeded")
        if "rejected" in lower:
            self._current_stage = 2
            self.mission_progress.set_stage(2, failed=True)
            self.mission_progress.set_mission_state("Rejected", "error")
            self._set_latest_history("Rejected")
            self.mission_progress.progress.setRange(0, 1)
            self.mission_progress.progress.setValue(0)
        if "mission failed" in lower or "nav2 rejected" in lower:
            self._current_stage = 4
            self.mission_progress.set_stage(4, failed=True)
            self.mission_progress.set_mission_state("Failed", "error")
            self._set_latest_history("Failed")
        if "pause requested" in lower:
            self.mission_progress.set_mission_state("Pausing", "busy")
        if "mission paused before" in lower:
            self.mission_progress.set_mission_state("Paused", "busy")
            self.mission_progress.stop_button.setText("Resume")
            self.mission_progress.stop_button.setIcon(
                self.style().standardIcon(QStyle.SP_MediaPlay)
            )
        if "resume requested" in lower:
            self.mission_progress.set_mission_state("Resuming", "busy")
        if "mission resumed from" in lower:
            self.mission_progress.set_mission_state("Navigating", "busy")
            self.mission_progress.stop_button.setText("Stop")
            self.mission_progress.stop_button.setIcon(
                self.style().standardIcon(QStyle.SP_MediaStop)
            )
        if "cancel requested" in lower or "stop requested" in lower:
            self.mission_progress.set_mission_state("Canceling", "busy")
        if (
            "mission state: canceled" in lower
            or "command interpretation canceled" in lower
            or "command canceled before execution" in lower
            or "queued command canceled before execution" in lower
        ):
            self.mission_progress.set_mission_state("Canceled", "online")
            self.mission_progress.destination.setText("Command canceled")
            self.mission_progress.remaining.setText("Remaining distance: -")
            self.mission_progress.progress.setRange(0, 1)
            self.mission_progress.progress.setValue(0)
            self.mission_progress.stop_button.setEnabled(False)
            self.mission_progress.cancel_button.setEnabled(False)
            self._set_latest_history("Canceled")
        if "policy state reset" in lower:
            self.mission_progress.set_stage(-1)
            self.mission_progress.set_mission_state("Idle", "online")
            self.mission_progress.destination.setText("Waiting for a command")
            self.mission_progress.remaining.setText("Remaining distance: -")
            self.mission_progress.progress.setRange(0, 1)
            self.mission_progress.progress.setValue(0)

    def _html_log_line(self, entry: Tuple[str, str, str]) -> str:
        timestamp, color, message = entry
        color_value = LOG_COLORS.get(color, LOG_COLORS[""])
        return (
            f'<span style="color:#8a95a4">{html.escape(timestamp)}</span> '
            f'<span style="color:{color_value}">{html.escape(message)}</span>'
        )

    def _render_live_log(self) -> None:
        entries = self._logs[-8:]
        self.live_log.setHtml("<br>".join(map(self._html_log_line, entries)))
        self.live_log.verticalScrollBar().setValue(
            self.live_log.verticalScrollBar().maximum()
        )

    def _render_activity_log(self) -> None:
        mode = self.activity_page.filter.currentText()
        entries = self._logs
        if mode == "Warnings":
            entries = [item for item in entries if item[1] == "yellow"]
        elif mode == "Errors":
            entries = [item for item in entries if item[1] == "red"]
        self.activity_page.log.setHtml(
            "<br>".join(map(self._html_log_line, entries))
        )
        self.activity_page.log.verticalScrollBar().setValue(
            self.activity_page.log.verticalScrollBar().maximum()
        )

    def _add_history(self, command: str, status: str) -> None:
        self._history.insert(
            0,
            {
                "time": dt.datetime.now().strftime("%H:%M:%S"),
                "command": command,
                "status": status,
                "version": "-",
            },
        )
        self._render_history()

    def _set_latest_history(
        self, status: str, version: Optional[str] = None
    ) -> None:
        if not self._history:
            return
        self._history[0]["status"] = status
        if version is not None:
            self._history[0]["version"] = version
        self._render_history()

    def _render_history(self) -> None:
        table = self.activity_page.history
        table.setRowCount(len(self._history))
        for row, item in enumerate(self._history):
            values = (
                item["time"],
                item["command"],
                item["status"],
                item["version"],
            )
            for column, value in enumerate(values):
                cell = QTableWidgetItem(value)
                if column in (0, 2, 3):
                    cell.setTextAlignment(Qt.AlignCenter)
                table.setItem(row, column, cell)

    def clear_log(self) -> None:
        self._logs = []
        self._render_live_log()
        self._render_activity_log()
        self.log_count.setText("0 events")

    def save_log(self) -> None:
        path, _selected = QFileDialog.getSaveFileName(
            self,
            "Save activity log",
            str(Path.home() / "policy_bridge_activity.log"),
            "Log files (*.log);;Text files (*.txt)",
        )
        if not path:
            return
        content = "\n".join(
            f"[{stamp}] [{color or 'info'}] {message}"
            for stamp, color, message in self._logs
        )
        Path(path).write_text(content + "\n", encoding="utf-8")

    def _apply_demo_state(self) -> None:
        policy = {
            "decision": "accept",
            "plan": [{"dest": "WP8"}],
            "via_zones": ["C"],
            "exclusions": [{"zone": "A"}],
            "weights": [
                {"zone": "B", "cost": 200},
                {"zone": "D", "cost": 40},
            ],
            "dynamic_object_rules": [
                {"class": "person", "radius": 1.5}
            ],
            "conditional_rules": [],
        }
        self._last_snapshot = {
            "nav2_ready": True,
            "mission_active": True,
            "command_pending": False,
            "active_command_id": 12,
            "policy_version": 12,
            "decision": "committed version 12",
            "llm_enabled": True,
            "llm_model": "qwen3:8b",
            "configured_zones": list("ABCDEF"),
            "zone_geometry_subscribers": 2,
            "forbidden": ["A"],
            "soft_costs": {"B": 200, "D": 40},
            "object_rules": [{"class": "person", "radius": 1.5}],
            "object_radius": 1.5,
            "route": ["Zone C", "WP8"],
            "policy": policy,
            "raw_policy": policy,
            "validation": {
                "accepted": True,
                "discarded": [],
                "normalized": [],
            },
            "llm_metadata": {"model": "qwen3:8b", "latency_s": 0.63},
            "fire_alarm": False,
            "battery_pct": 84,
            "objects_detected": 2,
        }
        command = (
            "Go through zone C to WP8, forbid A, set B to 200, "
            "and avoid people within 1.5 m."
        )
        self.composer.editor.setPlainText(command)
        self._add_history(command, "Navigating")
        demo_logs = [
            ("Policy Bridge node ready", "green"),
            (f"Received command: {command}", "cyan"),
            ("Validated policy committed as version 12", "green"),
            ("Mission start", "blue"),
            (
                "Progress: Waypoint 1/2 -> Zone C (remaining~6.84 m)",
                "blue",
            ),
            ("Objects detected in map frame: 2", "yellow"),
        ]
        for message, color in demo_logs:
            self.on_log_event(message, color)
        self.refresh_snapshot()

    def _apply_empty_demo_state(self) -> None:
        self._last_snapshot.update(
            {
                "mission_active": False,
                "policy_version": (
                    self._last_snapshot.get("policy_version", 0) + 1
                ),
                "decision": "reset",
                "forbidden": [],
                "soft_costs": {},
                "object_rules": [],
                "route": [],
                "policy": {},
                "raw_policy": None,
                "validation": {},
                "objects_detected": 0,
            }
        )
        self.refresh_snapshot()

    def closeEvent(self, event: QCloseEvent) -> None:
        if self._closing:
            event.accept()
            return
        if self.map_page.dirty and not self._confirm_map_change():
            event.ignore()
            return
        self._closing = True
        self.refresh_timer.stop()
        self.multi_discovery_timer.stop()
        self.context_timer.stop()
        if self.on_close is not None:
            self.on_close()
        event.accept()


def _parse_gui_arguments(
    argv: List[str],
) -> Tuple[argparse.Namespace, List[str]]:
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--demo-ui", action="store_true")
    parser.add_argument(
        "--demo-page",
        choices=("mission", "multi", "map", "policy", "activity"),
        default="mission",
    )
    parser.add_argument("--demo-width", type=int, default=1440)
    parser.add_argument("--demo-height", type=int, default=900)
    parser.add_argument("--screenshot", default="")
    options, ros_args = parser.parse_known_args(argv[1:])
    return options, [argv[0], *ros_args]


def _save_screenshot(
    app: QApplication,
    window: PolicyBridgeWindow,
    path: str,
) -> None:
    output = Path(path).expanduser()
    output.parent.mkdir(parents=True, exist_ok=True)
    window.grab().save(str(output))
    app.quit()


def main() -> None:

    options, ros_args = _parse_gui_arguments(sys.argv)
    if hasattr(Qt, "AA_EnableHighDpiScaling"):
        QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    app = QApplication([sys.argv[0]])
    app.setApplicationName("Policy Bridge")
    app.setOrganizationName("Policy Bridge")
    app.setFont(QFont("Noto Sans", 10))

    if options.demo_ui:
        window = PolicyBridgeWindow(None, demo_mode=True)
        window.resize(options.demo_width, options.demo_height)
        page_index = {
            "mission": 0,
            "multi": 1,
            "map": 2,
            "policy": 3,
            "activity": 4,
        }
        window.nav_buttons[page_index[options.demo_page]].setChecked(True)
        window.switch_page(page_index[options.demo_page])
        window.show()
        if options.screenshot:
            QTimer.singleShot(
                700,
                lambda: _save_screenshot(app, window, options.screenshot),
            )
        sys.exit(app.exec_())

    rclpy.init(args=ros_args)
    signals = GuiSignals()
    node = GuiPolicyBridgeNode(signals)
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    multi_manager = MultiRobotBridgeManager(node, executor)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    shutdown_lock = threading.Lock()
    shutdown_complete = False

    def shutdown() -> None:
        nonlocal shutdown_complete
        with shutdown_lock:
            if shutdown_complete:
                return
            shutdown_complete = True
        multi_manager.shutdown()
        node.stop_gui_monitor()
        node._yolo_run = False
        if rclpy.ok():
            rclpy.shutdown()
        executor.shutdown(timeout_sec=1.0)
        spin_thread.join(timeout=1.0)
        multi_manager.destroy_nodes()
        node.destroy_node()

    window = PolicyBridgeWindow(
        node,
        signals,
        multi_manager=multi_manager,
        on_close=shutdown,
    )
    window.show()
    exit_code = app.exec_()
    shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
