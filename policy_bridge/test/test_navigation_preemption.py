import json
import threading
import time
from collections import deque
from types import SimpleNamespace

from nav2_simple_commander.robot_navigator import TaskResult
from std_msgs.msg import String

from policy_bridge.policybridge import (
    CommandInterpretationCanceled,
    PolicyBridgeNode,
    PolicyUpdateRejected,
    PreparedMission,
    QueuedCommand,
)
import policy_bridge.policybridge as policybridge_module


def bare_node():
    node = object.__new__(PolicyBridgeNode)
    node._ui_print = lambda *_args, **_kwargs: None
    node._command_lock = threading.Lock()
    node._command_queue = deque()
    node._mission_lock = threading.Lock()
    node._mission_queue = deque()
    node._mission_active = threading.Event()
    node._preempt_requested = threading.Event()
    node._cancel_requested = threading.Event()
    node._pause_requested = threading.Event()
    node._mission_paused = threading.Event()
    node._interpretation_active = threading.Event()
    node._interpretation_cancel_requested = threading.Event()
    node._active_interpretation_id = None
    node._llm_request_lock = threading.Lock()
    node._active_llm_connections = set()
    node._preempt_on_valid_command = True
    node._replacement_nav_settle_s = 0.0
    node._replacement_early_abort_retry_window_s = 3.0
    node._latest_command_id = 0
    node._minimum_valid_command_id = 0
    node._last_policy_decision = "initialized"
    node._sched_lock = threading.Lock()
    node._schedules = {}
    node._permanent_exclusions = []
    node._allow_overrides_permanent = True
    node._env_state = {"fire_alarm": False, "battery_pct": 100}
    node._labels = list("ABCDE")
    node._zone_db = {label: object() for label in node._labels}
    return node


def sample_policy(destination="WP7"):
    return {
        "decision": "accept",
        "reason_code": "none",
        "plan": [{"dest": destination}],
        "via_zones": ["A"],
        "exclusions": [{"zone": "E"}],
        "weights": [],
        "dynamic_object_rules": [],
        "conditional_rules": [],
    }


def test_policy_preparation_does_not_commit_before_execution():
    node = bare_node()
    node._enable_llm = False
    node._llm_endpoint = ""
    node._apply_aisle_aliases = lambda text: text
    node._fallback_parse = lambda _text: sample_policy()
    node._validate_policy_obj = lambda policy, _text: policy
    node._active_policy_state = node._empty_policy()
    node._policy_version = 0

    prepared = node._prepare_policy("Go through A to WP7 and forbid E.")

    assert prepared["plan"] == [{"dest": "WP7"}]
    assert node._policy_version == 0
    assert node._active_policy_state["plan"] == []

    committed = node._commit_policy_snapshot(prepared)
    assert committed["plan"] == [{"dest": "WP7"}]
    assert node._policy_version == 1


def test_valid_followup_command_waits_without_preempting_active_goal():
    node = bare_node()
    node._mission_active.set()
    node._prepare_policy = lambda _text: sample_policy()

    accepted = node._prepare_and_queue_command(1, "Go to WP7 next.")

    assert accepted is True
    assert not node._preempt_requested.is_set()
    assert len(node._mission_queue) == 1
    assert node._mission_queue[0].command_id == 1
    assert node._mission_queue[0].source == "llm"


def test_explicit_cancel_and_replace_preempts_only_after_validation():
    node = bare_node()
    node._mission_active.set()
    node._prepare_policy = lambda _text: sample_policy()

    accepted = node._prepare_and_queue_command(
        1,
        "Cancel the current mission and go to WP7.",
        replace_active=True,
    )

    assert accepted is True
    assert node._preempt_requested.is_set()
    assert len(node._mission_queue) == 1
    assert node._mission_queue[0].replace_active is True


def test_normal_commands_are_retained_in_fifo_input_order():
    node = bare_node()

    first_id = node._enqueue_nl_command("Go to WP1.")
    second_id = node._enqueue_nl_command("Then go to WP2.")

    assert [item.command_id for item in node._command_queue] == [
        first_id,
        second_id,
    ]
    assert [item.text for item in node._command_queue] == [
        "Go to WP1.",
        "Then go to WP2.",
    ]
    assert not node._interpretation_cancel_requested.is_set()


def test_explicit_replacement_clears_earlier_pending_work():
    node = bare_node()
    node._command_queue.append(QueuedCommand(1, "Go to WP1."))
    node._mission_queue.append(
        PreparedMission(2, "Go to WP2.", sample_policy("WP2"), "llm")
    )
    node._latest_command_id = 2

    command_id = node._enqueue_nl_command(
        "Cancel the previous mission and go to WP7."
    )

    assert len(node._command_queue) == 1
    assert node._command_queue[0].command_id == command_id
    assert node._command_queue[0].replace_active is True
    assert list(node._mission_queue) == []
    assert node._minimum_valid_command_id == command_id


def test_only_explicit_mission_control_phrases_request_replacement():
    assert PolicyBridgeNode._requests_mission_replacement(
        "Cancel the current mission and go to WP7."
    )
    assert PolicyBridgeNode._requests_mission_replacement(
        "Abort the previous task; head to WP4."
    )
    assert PolicyBridgeNode._requests_mission_replacement(
        "Cancel the last command and forbid zone C for 5 minutes and go to "
        "WP2, and then go to WP5, then go to WP6."
    )
    assert PolicyBridgeNode._requests_mission_replacement(
        "Discard the latest order and go to WP3."
    )
    assert not PolicyBridgeNode._requests_mission_replacement(
        "Go to WP1, then WP2."
    )
    assert not PolicyBridgeNode._requests_mission_replacement(
        "Cancel the zone B restriction and go to WP7."
    )


def test_manual_oracle_uses_validated_prepared_mission_path():
    node = bare_node()
    node._nav2_ready = True
    node._validate_policy_obj = lambda policy, _text: policy
    message = String()
    message.data = json.dumps({
        "user_text": "Forbid zone A and go to WP2.",
        "policy": sample_policy("WP2"),
    })

    node._manual_policy_callback(message)

    assert len(node._mission_queue) == 1
    assert node._mission_queue[0].source == "manual_oracle"
    assert node._mission_queue[0].policy["plan"] == [{"dest": "WP2"}]


def test_rejected_command_does_not_interrupt_active_goal():
    node = bare_node()
    node._mission_active.set()

    def reject(_text):
        raise PolicyUpdateRejected("contradiction")

    node._prepare_policy = reject

    accepted = node._prepare_and_queue_command(1, "Contradictory command")

    assert accepted is False
    assert not node._preempt_requested.is_set()
    assert list(node._mission_queue) == []


def test_superseded_llm_result_cannot_replace_newer_command():
    node = bare_node()
    node._latest_command_id = 2
    node._minimum_valid_command_id = 2
    node._mission_active.set()
    node._prepare_policy = lambda _text: sample_policy()

    accepted = node._prepare_and_queue_command(1, "Old command")

    assert accepted is False
    assert not node._preempt_requested.is_set()
    assert list(node._mission_queue) == []


class FakeNavigator:
    def __init__(self):
        self.cancel_count = 0
        self.complete = False

    def waitUntilNav2Active(self):
        return None

    def followWaypoints(self, _waypoints):
        self.complete = False
        return True

    def isTaskComplete(self):
        return self.complete

    def cancelTask(self):
        self.cancel_count += 1
        self.complete = True

    def getFeedback(self):
        return None


def test_remaining_distance_updates_without_waypoint_feedback():
    class NavigatorWithoutFeedback:
        def __init__(self):
            self.polls = 0

        def waitUntilNav2Active(self):
            return None

        def followWaypoints(self, _waypoints):
            return True

        def isTaskComplete(self):
            self.polls += 1
            return self.polls > 7

        def getFeedback(self):
            return None

        def getResult(self):
            return TaskResult.SUCCEEDED

    waypoint = SimpleNamespace(
        pose=SimpleNamespace(
            position=SimpleNamespace(x=10.0, y=0.0)
        )
    )
    node = bare_node()
    node._nav = NavigatorWithoutFeedback()
    distances = iter([8.0, 6.0, 4.0])
    node._dist_to_pose = lambda _target: next(distances, 2.0)
    updates = []
    node._update_navigation_progress = (
        lambda **values: updates.append(values)
    )

    result = node._follow_waypoints([waypoint], ["WP2"])

    assert result == TaskResult.SUCCEEDED
    assert len(updates) >= 2
    assert updates[0]["remaining_m"] > updates[-1]["remaining_m"]


def test_follow_waypoints_cancels_for_valid_replacement():
    node = bare_node()
    node._nav = FakeNavigator()
    node._preempt_requested.set()
    node._mission_queue.append(PreparedMission(1, "replace", sample_policy(), "llm", True))

    result = node._follow_waypoints([object()], ["WP2"])

    assert result == TaskResult.CANCELED
    assert node._nav.cancel_count == 1


def test_completed_abort_is_classified_as_replaced_when_preempt_races():
    class AlreadyAbortedNavigator:
        def waitUntilNav2Active(self):
            return None

        def followWaypoints(self, _waypoints):
            return True

        def isTaskComplete(self):
            return True

        def getResult(self):
            return TaskResult.FAILED

    node = bare_node()
    node._nav = AlreadyAbortedNavigator()
    node._preempt_requested.set()
    node._mission_queue.append(PreparedMission(1, "replace", sample_policy(), "llm", True))

    result = node._follow_waypoints([object()], ["WP8"])

    assert result == TaskResult.CANCELED


def test_completed_abort_remains_failed_without_control_request():
    class AlreadyAbortedNavigator:
        def waitUntilNav2Active(self):
            return None

        def followWaypoints(self, _waypoints):
            return True

        def isTaskComplete(self):
            return True

        def getResult(self):
            return TaskResult.FAILED

    node = bare_node()
    node._nav = AlreadyAbortedNavigator()

    result = node._follow_waypoints([object()], ["WP8"])

    assert result == TaskResult.FAILED


def test_replacement_retries_one_immediate_nav2_abort():
    class AbortThenSucceedNavigator:
        def __init__(self):
            self.goal_count = 0

        def waitUntilNav2Active(self):
            return None

        def followWaypoints(self, _waypoints):
            self.goal_count += 1
            return True

        def isTaskComplete(self):
            return True

        def getResult(self):
            if self.goal_count == 1:
                return TaskResult.FAILED
            return TaskResult.SUCCEEDED

    node = bare_node()
    node._nav = AbortThenSucceedNavigator()

    result = node._follow_waypoints(
        [object()],
        ["WP3"],
        retry_early_abort_once=True,
    )

    assert result == TaskResult.SUCCEEDED
    assert node._nav.goal_count == 2


class FakeConnection:
    def __init__(self):
        self.closed = False

    def close(self):
        self.closed = True


def test_cancel_cancels_active_interpretation_and_retains_later_queue():
    node = bare_node()
    connection = FakeConnection()
    node._interpretation_active.set()
    node._active_interpretation_id = 1
    node._active_llm_connections.add(connection)
    node._command_queue.append(QueuedCommand(2, "newer command"))

    canceled = node.request_cancel()

    assert canceled is True
    assert node._interpretation_cancel_requested.is_set()
    assert node._minimum_valid_command_id == 2
    assert [item.command_id for item in node._command_queue] == [2]
    assert list(node._mission_queue) == []
    assert connection.closed is True


def test_cancel_returns_false_when_there_is_no_cancellable_work():
    node = bare_node()

    assert node.request_cancel() is False


def test_pause_and_resume_reissues_only_remaining_waypoints(monkeypatch):
    class PausableNavigator:
        def __init__(self):
            self.follow_calls = []
            self.cancel_count = 0
            self.task_complete = threading.Event()

        def waitUntilNav2Active(self):
            return None

        def followWaypoints(self, waypoints):
            self.follow_calls.append(list(waypoints))
            self.task_complete.clear()
            return True

        def isTaskComplete(self):
            return self.task_complete.is_set()

        def cancelTask(self):
            self.cancel_count += 1
            self.task_complete.set()

        def getFeedback(self):
            return SimpleNamespace(current_waypoint=1)

        def getResult(self):
            return TaskResult.SUCCEEDED

    class Pose:
        def __init__(self):
            self.pose = SimpleNamespace(
                position=SimpleNamespace(x=0.0, y=0.0)
            )

    node = bare_node()
    node._mission_active.set()
    node._nav = PausableNavigator()
    node._dist_to_pose = lambda _pose: 1.0
    node._update_navigation_progress = lambda **_kwargs: None
    monkeypatch.setattr(policybridge_module.rclpy, "ok", lambda: True)
    result = []
    worker = threading.Thread(
        target=lambda: result.append(
            node._follow_waypoints(
                [Pose(), Pose(), Pose()],
                ["WP1", "WP2", "WP3"],
            )
        )
    )
    worker.start()

    deadline = time.monotonic() + 1.0
    while len(node._nav.follow_calls) < 1 and time.monotonic() < deadline:
        time.sleep(0.01)
    assert node.request_pause() is True
    deadline = time.monotonic() + 1.0
    while not node._mission_paused.is_set() and time.monotonic() < deadline:
        time.sleep(0.01)
    assert node._mission_paused.is_set()
    assert node.request_resume() is True
    deadline = time.monotonic() + 1.0
    while len(node._nav.follow_calls) < 2 and time.monotonic() < deadline:
        time.sleep(0.01)
    assert [len(call) for call in node._nav.follow_calls] == [3, 2]
    node._nav.task_complete.set()
    worker.join(timeout=1.0)

    assert worker.is_alive() is False
    assert result == [TaskResult.SUCCEEDED]
    assert node._nav.cancel_count == 1


def test_llm_wait_returns_promptly_after_interpretation_cancel():
    node = bare_node()
    request_started = threading.Event()
    release_request = threading.Event()
    outcome = []
    node._build_planner_prompt = lambda text: text
    node._llm_model = "test-model"
    node._llm_think = False

    def slow_request(_data):
        request_started.set()
        release_request.wait(timeout=2.0)
        return '{"response": "{}"}'

    node._post_llm_request = slow_request

    def call_llm():
        try:
            node._call_llm_raw("Go to WP2")
        except Exception as exc:
            outcome.append(exc)

    worker = threading.Thread(target=call_llm)
    worker.start()
    assert request_started.wait(timeout=0.5)
    node._interpretation_cancel_requested.set()
    worker.join(timeout=0.5)
    release_request.set()

    assert worker.is_alive() is False
    assert len(outcome) == 1
    assert isinstance(outcome[0], CommandInterpretationCanceled)


def test_structured_navigation_progress_is_sanitized_and_replaced():
    node = bare_node()
    node._navigation_progress_lock = threading.Lock()
    node._navigation_progress = {}

    node._update_navigation_progress(
        current=2,
        total=3,
        destination="Zone C",
        remaining_m=4.25,
    )

    assert node._navigation_progress == {
        "current": 2,
        "total": 3,
        "destination": "Zone C",
        "remaining_m": 4.25,
    }


def test_waypoint_names_include_zone_label_for_named_coordinate():
    node = bare_node()
    node._waypoints = {"WP1": (1.0, 2.0, 0.0)}
    node._make_pose = lambda x, y, yaw: (x, y, yaw)
    node._ui_print = lambda *_args: None
    plan = [
        {
            "dest": {"x": 16.0, "y": 0.2, "yaw": 0.0},
            "display_name": "Zone A",
        },
        {"dest": "WP1"},
    ]

    waypoints, names = node._waypoints_from_plan_with_names(plan)

    assert len(waypoints) == 2
    assert names == ["Zone A", "WP1"]


def test_unnamed_coordinate_keeps_coordinate_fallback_label():
    node = bare_node()
    node._make_pose = lambda x, y, yaw: (x, y, yaw)
    node._ui_print = lambda *_args: None

    _waypoints, names = node._waypoints_from_plan_with_names([
        {"dest": {"x": 16.0, "y": 0.2, "yaw": 0.0}},
    ])

    assert names == ["(16.0,0.2)"]


class FakeTimer:
    def __init__(self):
        self.cancelled = False

    def cancel(self):
        self.cancelled = True


def configure_reset_state(node):
    timer = FakeTimer()
    node._command_queue = deque([(2, "pending")])
    node._mission_queue = deque()
    node._sched_lock = threading.Lock()
    node._timer_threads = {4: (timer, None)}
    node._schedules = {4: object()}
    node._active_timed = {4: {"A"}}
    node._next_sched_id = 5
    node._configured_permanent_exclusions = ["G"]
    node._permanent_exclusions = ["E"]
    node._last_base_exclusions = ["A"]
    node._cond_rules_store = [object()]
    node._force_shortest = True
    node._latest_dynamic_rules = [object()]
    node._last_wp_names = ["WP2"]
    node._last_forbidden_windows = {"A": object()}
    node._env_state = {"fire_alarm": True, "battery_pct": 100}
    node._active_policy_state = sample_policy("WP2")
    node._policy_version = 3
    node._recalculate_and_publish_dynamic_keepouts = lambda *_args: None
    node._clear_softcost = lambda: None
    return timer


def test_trial_reset_clears_persistent_test_state_when_idle():
    node = bare_node()
    timer = configure_reset_state(node)

    reset = node._reset_policy_for_next_trial()

    assert reset is True
    assert timer.cancelled is True
    assert node._schedules == {}
    assert node._active_timed == {}
    assert node._permanent_exclusions == ["G"]
    assert node._last_base_exclusions == []
    assert node._cond_rules_store == []
    assert node._active_policy_state["plan"] == []
    assert node._env_state["fire_alarm"] is False


def test_trial_reset_is_rejected_while_navigation_is_active():
    node = bare_node()
    timer = configure_reset_state(node)
    node._mission_active.set()

    reset = node._reset_policy_for_next_trial()

    assert reset is False
    assert timer.cancelled is False
    assert node._schedules
