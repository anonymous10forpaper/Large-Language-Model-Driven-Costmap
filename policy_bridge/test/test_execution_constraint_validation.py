"""State-aware validator regressions with no live inference or navigation."""

import copy
import datetime as dt
from types import SimpleNamespace
from unittest.mock import patch

import pytest
import rclpy
from nav2_simple_commander.robot_navigator import TaskResult

from policy_bridge.policybridge import PolicyBridgeNode, PolicyUpdateRejected, TimedKeepout
from policy_bridge.policybridge_gui import GuiPolicyBridgeNode
from r2_3_state_regression_runner import isolated_probe, old_policy, policy


@pytest.fixture(scope="module", autouse=True)
def ros_context():
    rclpy.init(args=[])
    yield
    rclpy.shutdown()


@pytest.fixture
def probe():
    with isolated_probe("execution_constraints") as instance:
        yield instance


def transit():
    return policy(plan=[{"dest": "WP3"}], via_zones=["A"])


def schedule(node, start, end, *, callback_active):
    now = dt.datetime.now(node._tz)
    node._schedules[1] = TimedKeepout(
        zones=["A"], start_wall=now, end_wall=now + dt.timedelta(minutes=5),
        id=1, start_ros_ns=start, end_ros_ns=end,
    )
    node._active_timed = {1: {"A"}} if callback_active else {}


@pytest.mark.parametrize("now_ns,blocked", [(99, False), (100, True), (199, True), (200, False)])
def test_ros_clock_interval_not_timer_callback_membership(probe, now_ns, blocked):
    node = probe.node
    schedule(node, 100, 200, callback_active=not blocked)
    fake_clock = SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=now_ns))
    before = probe.state()
    with patch.object(node, "get_clock", return_value=fake_clock):
        if blocked:
            with pytest.raises(PolicyUpdateRejected, match="currently forbidden: A"):
                node._validate_policy_obj(transit(), "Not interpreted by the validator.")
        else:
            assert node._validate_policy_obj(transit(), "") == transit()
        assert node._effective_keepouts([], [])[0] == (["A"] if blocked else [])
    assert probe.state() == before


def test_real_permanent_parameter_override():
    with isolated_probe("permanent_parameter", permanent=True) as probe:
        assert probe.node.get_parameter("permanent_exclusions").value == ["A"]
        with pytest.raises(PolicyUpdateRejected, match="currently forbidden: A"):
            probe.node._validate_policy_obj(transit(), "")


def test_invalid_replacement_preserves_existing_queue_and_active_goal(probe):
    node = probe.node
    assert node._queue_prepared_policy(1, "old pending", policy(plan=[{"dest": "WP2"}]), source="manual")
    queue_before = list(node._mission_queue)
    node._mission_active.set()
    node._permanent_exclusions = ["A"]
    before = probe.state()
    offset = len(probe.recorder.events)
    with patch.object(node, "_call_llm", return_value=transit()):
        accepted = node._prepare_and_queue_command(2, "replace", replace_active=True)
    assert not accepted
    assert list(node._mission_queue) == queue_before
    assert not node._preempt_requested.is_set()
    assert probe.state() == before
    assert all(e["kind"] == "log" for e in probe.recorder.events[offset:])


def test_queue_checks_constraints_again_before_signaling_preemption(probe):
    node = probe.node
    prepared = node._validate_policy_obj(transit(), "")
    node._permanent_exclusions = ["A"]
    node._mission_active.set()
    assert not node._queue_prepared_policy(1, "replace", prepared, source="manual", replace_active=True)
    assert not node._mission_queue
    assert not node._preempt_requested.is_set()


def test_new_constraint_prevents_canceling_the_old_goal(probe):
    node = probe.node
    node._mission_active.set()
    assert node._queue_prepared_policy(1, "replace", transit(), source="manual", replace_active=True)
    node._permanent_exclusions = ["A"]
    before = probe.state()
    with patch.object(node, "_cancel_nav_task_and_wait") as cancel:
        assert not node._replacement_can_preempt()
        cancel.assert_not_called()
    assert not node._preempt_requested.is_set()
    assert not node._mission_queue
    assert probe.state() == before


def test_navigation_loop_keeps_running_after_replacement_becomes_invalid(probe):
    node = probe.node
    node._mission_active.set()
    assert node._queue_prepared_policy(1, "replace", transit(), source="manual", replace_active=True)
    node._permanent_exclusions = ["A"]
    done = iter([False, True])
    node._nav = SimpleNamespace(
        waitUntilNav2Active=lambda: None,
        followWaypoints=lambda _poses: True,
        isTaskComplete=lambda: next(done),
        getFeedback=lambda: None,
        getResult=lambda: TaskResult.SUCCEEDED,
    )
    with patch.object(node, "_cancel_nav_task_and_wait") as cancel:
        with patch.object(node, "_dist_to_pose", return_value=1.0):
            result = PolicyBridgeNode._follow_waypoints(node, [node._make_pose(1, 1, 0)], ["WP2"])
    assert result == TaskResult.SUCCEEDED
    cancel.assert_not_called()
    assert not node._mission_queue


@pytest.mark.parametrize("invalid_schema", [False, True])
def test_queued_mission_revalidated_before_commit_and_projection(probe, invalid_schema):
    node = probe.node
    probe.submit(old_policy(), "seed")
    prepared = node._validate_policy_obj(transit(), "")
    if invalid_schema:
        prepared["weights"] = [{"zone": "B", "cost": 253}]
    else:
        node._permanent_exclusions = ["A"]
    before = probe.state()
    offset = len(probe.recorder.events)
    node._run_single_mission("queued", prepared_policy=prepared)
    assert probe.state() == before
    assert all(e["kind"] == "log" for e in probe.recorder.events[offset:])


def test_rejected_queued_mission_does_not_trigger_worker_cleanup(probe):
    node = probe.node
    probe.submit(old_policy(), "seed")
    assert node._queue_prepared_policy(1, "queued", transit(), source="manual")
    node._permanent_exclusions = ["A"]
    before = probe.state()
    offset = len(probe.recorder.events)
    with patch("policy_bridge.policybridge.rclpy.ok", side_effect=[True, False]):
        node._mission_worker()
    assert probe.state() == before
    assert all(e["kind"] == "log" for e in probe.recorder.events[offset:])


def test_gui_rejection_does_not_clear_fleet_policy():
    node = object.__new__(GuiPolicyBridgeNode)
    node._policy_version = 7
    with patch.object(PolicyBridgeNode, "_run_single_mission", return_value=None):
        with patch.object(node, "_clear_mission_scoped_fleet_keepouts") as cleanup:
            node._run_single_mission("rejected")
    cleanup.assert_not_called()


def test_gui_commit_guard_runs_before_fleet_epoch_publication(probe):
    node = object.__new__(GuiPolicyBridgeNode)
    node._gui_robot_namespace = "robot1"
    node._policy_version = 7
    node._active_policy_state = copy.deepcopy(old_policy())
    probe.node._permanent_exclusions = ["A"]
    with patch.object(node, "_validate_execution_constraints", probe.node._validate_execution_constraints):
        with patch.object(node, "_gui_fleet_epoch_factory", create=True) as publish:
            with pytest.raises(PolicyUpdateRejected):
                node._commit_policy_snapshot(transit())
    publish.assert_not_called()
    assert node._policy_version == 7
    assert node._active_policy_state == old_policy()


@pytest.mark.parametrize("labels,keepouts", [(["C", "A"], ["A"]), (["C", "UNKNOWN"], [])])
def test_via_conversion_never_returns_a_partial_route(probe, labels, keepouts):
    with pytest.raises(PolicyUpdateRejected):
        probe.node._via_waypoints_from_zones(labels, keepouts)


def test_new_policy_does_not_inherit_old_conditional_rules(probe):
    node = probe.node
    node._last_base_exclusions = ["A"]
    node._cond_rules_store = [{"zone": "A", "rules": [
        {"priority": 10, "state_condition": "fire_alarm", "action": "forbid"},
    ]}]
    node._env_state["fire_alarm"] = True
    assert node._validate_policy_obj(transit(), "") == transit()


@pytest.mark.parametrize("allow_permanent", [False, True])
def test_event_projection_and_execution_check_use_same_priority(probe, allow_permanent):
    node = probe.node
    node._permanent_exclusions = ["A"]
    node._allow_overrides_permanent = allow_permanent
    node._env_state["fire_alarm"] = True
    rules = [{"zone": "A", "rules": [
        {"priority": 10, "state_condition": "fire_alarm", "action": "allow_shortest"},
    ]}]
    candidate = policy(plan=[{"dest": "WP3"}], conditional_rules=rules)
    normalized = node._validate_policy_obj(candidate, "")
    expected = [] if allow_permanent else ["A"]
    assert node._last_validation_report["execution_context"]["effective_keepouts"] == expected
    with patch.object(node, "_publish_forbidden") as publish:
        node._recalculate_and_publish_dynamic_keepouts([], normalized["conditional_rules"])
    publish.assert_called_once_with(expected)


def test_activation_during_dispatch_never_silently_removes_a_required_stop(probe):
    node = probe.node
    original = node._publish_validated_policy

    def activate_after_commit(**values):
        original(**values)
        node._permanent_exclusions = ["A"]

    with patch.object(node, "_publish_validated_policy", side_effect=activate_after_commit):
        node._run_single_mission("go", prepared_policy=transit())
    goals = [e for e in probe.recorder.events if e["kind"] == "nav_goal_attempt"]
    assert goals[-1]["names"] == ["Zone A", "WP3"]
    assert "A" in goals[-1]["state"]["forbidden_windows"]
