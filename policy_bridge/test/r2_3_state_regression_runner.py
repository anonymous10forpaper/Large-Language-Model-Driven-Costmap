"""Read-only R2-3 node-level audit; no LLM requests, DDS publishes, or navigation."""

import argparse
from contextlib import ExitStack, contextmanager
import copy
import datetime as dt
import hashlib
import inspect
import json
import os
from pathlib import Path
import sys
import time
from unittest.mock import patch
from zoneinfo import ZoneInfo

import rclpy
from rclpy.parameter import Parameter
from rosidl_runtime_py.convert import message_to_ordereddict
from nav2_simple_commander.robot_navigator import TaskResult

import policy_bridge.policybridge as runtime
from policy_bridge.policybridge import PolicyBridgeNode


def policy(**updates):
    result = PolicyBridgeNode._empty_policy()
    result.update(updates)
    return result


def old_policy():
    return policy(
        plan=[{"dest": "WP2"}],
        via_zones=["C"],
        exclusions=[{"zone": "A"}, {"zone": "D"}],
        weights=[{"zone": "B", "cost": 20}],
        dynamic_object_rules=[{"class": "person", "radius": 1.5}],
        conditional_rules=[{
            "zone": "D",
            "rules": [{"priority": 10, "state_condition": "fire_alarm", "action": "allow_shortest"}],
        }],
    )


class Recorder:
    def __init__(self):
        self.events = []

    def add(self, kind, **values):
        self.events.append({"sequence": len(self.events), "kind": kind, **copy.deepcopy(values)})

    def publisher(self, _message_type, topic, *_args, **_kwargs):
        recorder = self

        class Publisher:
            def publish(self, message):
                recorder.add("publish", topic=topic, message=message_to_ordereddict(message))

            def get_subscription_count(self):
                return 2

        return Publisher()


class Probe:
    def __init__(self, node, recorder):
        self.node = node
        self.recorder = recorder
        self.steps = []
        self.command_id = 0

    def state(self):
        node = self.node
        return copy.deepcopy({
            "policy": node._active_policy_state,
            "version": node._policy_version,
            "base_exclusions": node._last_base_exclusions,
            "permanent_exclusions": node._permanent_exclusions,
            "active_timed": {str(k): sorted(v) for k, v in node._active_timed.items()},
            "schedule_count": len(node._schedules),
            "soft_costs": node._active_softcost,
            "conditional_rules": node._cond_rules_store,
            "object_radius": node._active_object_avoidance_radius,
            "forbidden_windows": {
                k: [start.isoformat(), end.isoformat()]
                for k, (start, end) in node._last_forbidden_windows.items()
            },
        })

    def capture_goal(self, poses, names, **kwargs):
        self.recorder.add(
            "nav_goal_attempt", names=names,
            poses=[message_to_ordereddict(pose) for pose in poses],
            state=self.state(), arguments=kwargs,
            note="Recorded only. SUCCEEDED is a test-double return value, not a navigation result.",
        )
        return TaskResult.SUCCEEDED

    def submit(self, candidate, label):
        self.command_id += 1
        before = self.state()
        offset = len(self.recorder.events)
        # Only the external interpreter is substituted. Validation, queuing,
        # commit, scheduling, projection and waypoint construction remain real.
        with patch.object(self.node, "_call_llm", return_value=copy.deepcopy(candidate)) as injected:
            accepted = self.node._prepare_and_queue_command(self.command_id, label)
        after_validation = self.state()
        queued_policy = None
        if accepted:
            with self.node._mission_lock:
                queued = self.node._mission_queue.popleft()
            queued_policy = copy.deepcopy(queued.policy)
            self.node._run_single_mission(
                queued.text, prepared_policy=queued.policy,
                policy_source="test_fixture_not_model_output",
                command_id=queued.command_id, replace_active=queued.replace_active,
            )
        events = copy.deepcopy(self.recorder.events[offset:])
        step = {
            "label": label, "input": candidate, "accepted_at_validation": accepted,
            "injected_interpreter_calls": injected.call_count,
            "before": before, "after_validation": after_validation,
            "normalized_policy": queued_policy, "after": self.state(),
            "validation_report": copy.deepcopy(self.node._last_validation_report),
            "events": events,
            "goals": [event for event in events if event["kind"] == "nav_goal_attempt"],
            "publish_count": sum(event["kind"] == "publish" for event in events),
        }
        self.steps.append(step)
        return step


@contextmanager
def isolated_probe(case_id, permanent=False):
    recorder = Recorder()
    parameters = [Parameter(name, value=value) for name, value in {
        "enable_llm": True, "llm_endpoint": "mock://r2-3-no-network",
        "enable_yolo": False, "enable_yolo_override_from_nl": False,
        "require_nav2": False, "set_initial_pose": False,
        "enable_console_ui": False, "use_sim_time": False,
    }.items()]
    if permanent:
        parameters.append(Parameter("permanent_exclusions", value=["A"]))
    with ExitStack() as stack:
        stack.enter_context(patch.object(PolicyBridgeNode, "create_publisher", recorder.publisher))
        stack.enter_context(patch.object(PolicyBridgeNode, "_start_runtime_workers"))
        stack.enter_context(patch.object(PolicyBridgeNode, "_ui_print", lambda _self, text, *_: recorder.add("log", text=text)))
        stack.enter_context(patch.object(runtime, "BasicNavigator", side_effect=AssertionError("Real Nav2 is prohibited")))
        stack.enter_context(patch.object(PolicyBridgeNode, "_post_llm_request", side_effect=AssertionError("Real LLM is prohibited")))
        node = PolicyBridgeNode(parameter_overrides=parameters, node_name="r2_3_" + case_id.lower().replace("-", "_"))
        probe = Probe(node, recorder)
        if permanent:
            assert node.get_parameter("permanent_exclusions").value == ["A"]
            recorder.add("parameter_setup", permanent_exclusions=["A"], note="Actual ROS STRING_ARRAY parameter override.")
        stack.enter_context(patch.object(node, "_ensure_navigator"))
        stack.enter_context(patch.object(node, "_follow_waypoints", probe.capture_goal))
        stack.enter_context(patch.object(node, "_dist_to_pose", return_value=None))
        try:
            yield probe
        finally:
            for pair in node._timer_threads.values():
                for timer in pair:
                    if timer is not None:
                        timer.cancel()
            node.destroy_node()


def last_goal(step):
    return step["goals"][-1] if step["goals"] else {}


def no_effect_rejection(step):
    return {
        "new_update_rejected": not step["accepted_at_validation"],
        "previous_execution_state_preserved": step["before"] == step["after"],
        "no_new_projection_publication": step["publish_count"] == 0,
        "no_new_navigation_goal": not step["goals"],
    }


def run_case(case_id):
    with isolated_probe(case_id, permanent=case_id == "T3c") as probe:
        if case_id == "T1":
            step = probe.submit(policy(
                plan=[{"dest": "WP3"}], via_zones=["C"],
                exclusions=[{"zone": "A"}], weights=[{"zone": "B", "cost": 200}],
            ), "Use C to reach WP3, keep A closed and set B to 200.")
            goal = last_goal(step)
            checks = {
                "accepted": step["accepted_at_validation"],
                "validation_does_not_commit": step["before"] == step["after_validation"],
                "exact_route": goal.get("names") == ["Zone C", "WP3"],
                "exact_keepout": set(goal.get("state", {}).get("forbidden_windows", {})) == {"A"},
                "exact_soft_cost": goal.get("state", {}).get("soft_costs") == {"B": 200},
                "one_goal_attempt": len(step["goals"]) == 1,
            }
        elif case_id == "T2":
            probe.submit(old_policy(), "Seed the previous ordinary mission.")
            step = probe.submit(policy(
                plan=[{"dest": "WP3"}], via_zones=["A"],
                weights=[{"zone": "B", "cost": 200}],
            ), "Go to WP3 through A and set B to 200.")
            goal = last_goal(step)
            state = goal.get("state", {})
            checks = {
                "accepted": step["accepted_at_validation"],
                "snapshot_replaced_not_unioned": state.get("policy") == step["normalized_policy"],
                "previous_keepouts_removed": state.get("forbidden_windows") == {},
                "old_via_replaced": goal.get("names") == ["Zone A", "WP3"],
                "cost_replaced": state.get("soft_costs") == {"B": 200},
                "old_object_and_event_rules_removed": state.get("object_radius") == 0.0 and state.get("conditional_rules") == [],
            }
        elif case_id == "T3a":
            probe.submit(old_policy(), "Seed a valid policy.")
            step = probe.submit(policy(
                plan=[{"dest": "WP3"}], via_zones=["A"], exclusions=[{"zone": "A"}],
            ), "Forbid A and go to WP3 through A.")
            checks = no_effect_rejection(step)
            checks["explicit_role_conflict"] = any(
                item.get("reason") == "via_and_exclusion"
                for item in step["validation_report"].get("conflicts", [])
            )
        elif case_id in ("T3b", "T3c", "T3d"):
            if case_id == "T3b":
                probe.submit(policy(
                    plan=[{"dest": "WP2"}], exclusions=[{"zone": "A", "duration_minutes": 5}],
                ), "Close A for five minutes and go to WP2.")
                assert any("A" in zones for zones in probe.node._active_timed.values())
            elif case_id == "T3c":
                probe.submit(policy(plan=[{"dest": "WP2"}]), "Go to WP2 with configured permanent A keepout.")
                assert probe.node._permanent_exclusions == ["A"]
            else:
                now = dt.datetime.now(probe.node._tz)
                probe.submit(policy(
                    plan=[{"dest": "WP2"}], exclusions=[{
                        "zone": "A", "time_condition": {
                            "start": (now - dt.timedelta(seconds=1)).isoformat(),
                            "end": (now + dt.timedelta(seconds=1)).isoformat(),
                        },
                    }],
                ), "Use an accelerated one-second timed closure of A before the next mission.")
                assert any("A" in zones for zones in probe.node._active_timed.values())
                deadline = time.monotonic() + 5
                while probe.node._active_timed and time.monotonic() < deadline:
                    rclpy.spin_once(probe.node, timeout_sec=0.05)
                assert not probe.node._active_timed, "The real ROS timer did not expire"
                probe.recorder.add("expiry_observed", state=probe.state())
            step = probe.submit(policy(plan=[{"dest": "WP3"}], via_zones=["A"]), "Go through A to WP3.")
            if case_id == "T3d":
                checks = {
                    "accepted_after_expiry": step["accepted_at_validation"],
                    "timed_keepout_expired": not step["before"]["active_timed"],
                    "required_via_preserved": last_goal(step).get("names") == ["Zone A", "WP3"],
                    "expired_keepout_not_projected": "A" not in last_goal(step).get("state", {}).get("forbidden_windows", {}),
                }
            else:
                checks = no_effect_rejection(step)
                checks["required_via_not_silently_dropped"] = not step["goals"] or "Zone A" in last_goal(step).get("names", [])
        elif case_id == "T4":
            probe.submit(old_policy(), "Seed a valid policy.")
            step = probe.submit(policy(
                plan=[{"dest": "WP3"}], weights=[{"zone": "B", "cost": 253}],
            ), "Set B to invalid soft cost 253 and go to WP3.")
            checks = no_effect_rejection(step)
            checks["explicit_invalid_cost"] = any(
                item.get("reason") == "invalid_cost"
                for item in step["validation_report"].get("discarded", [])
            )
        elif case_id == "T5":
            candidate = policy(plan=[{"dest": "WP3"}], via_zones=["C"], weights=[{"zone": "B", "cost": 200}])
            first = probe.submit(candidate, "Apply a valid static policy.")
            second = probe.submit(first["normalized_policy"], "Reapply the identical normalized policy.")
            checks = {
                "both_accepted": first["accepted_at_validation"] and second["accepted_at_validation"],
                "policy_content_idempotent": first["after"]["policy"] == second["after"]["policy"],
                "route_content_idempotent": last_goal(first).get("names") == last_goal(second).get("names"),
                "cost_content_idempotent": last_goal(first).get("state", {}).get("soft_costs") == last_goal(second).get("state", {}).get("soft_costs"),
                "version_increments_as_designed": second["after"]["version"] == first["after"]["version"] + 1,
            }
        else:
            raise ValueError(case_id)
        return {
            "id": case_id, "passed": all(checks.values()), "checks": checks,
            "steps": probe.steps, "all_events": copy.deepcopy(probe.recorder.events),
        }


CASES = ["T1", "T2", "T3a", "T3b", "T3c", "T3d", "T4", "T5"]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    if os.environ.get("ROS_DOMAIN_ID") != "218" or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        parser.error("Use ROS_DOMAIN_ID=218 ROS_LOCALHOST_ONLY=1 for this isolated audit")
    if args.output_dir.exists():
        parser.error("Use a new output directory; existing evidence is never overwritten")
    args.output_dir.mkdir(parents=True)
    source = Path(inspect.getfile(PolicyBridgeNode)).resolve()
    source_bytes = source.read_bytes()
    manifest = {
        "source_file": str(source), "source_sha256": hashlib.sha256(source_bytes).hexdigest(),
        "started_at": dt.datetime.now(ZoneInfo("UTC")).isoformat(),
        "ros_domain_id": 218, "clock": "ROS system clock, use_sim_time=false",
        "real_llm_calls": 0, "real_navigation_goals": 0, "real_application_topic_publications": 0,
        "method": "Actual ROS2 node with interpreter/transport/action doubles; real validation, queue, commit, projection, waypoint generation and ROS timers.",
        "limitations": [
            "Not a language-understanding benchmark or Gazebo/physical navigation experiment.",
            "Synthetic action success only permits observing the mission-end code; it is not evidence of arrival.",
            "T3d uses an accelerated real ROS timer, not five minutes of navigation.",
            "T5 checks static normalized content, not timer/version/goal side-effect idempotence.",
            "Each case uses a fresh node; stateful setup commands are part of its recorded trace.",
            "T3c uses an actual ROS STRING_ARRAY override, unlike the baseline runtime-state fixture; behavioral assertions are unchanged.",
            "Required behavior for T3b/c is explicit new-mission rejection with state preservation; filtering out mandatory via zones does not pass.",
        ],
    }
    (args.output_dir / "policybridge_tested.py").write_bytes(source_bytes)
    (args.output_dir / "runner_used.py").write_bytes(Path(__file__).read_bytes())
    rclpy.init(args=[])
    results = []
    try:
        for case_id in CASES:
            try:
                result = run_case(case_id)
            except Exception as error:
                import traceback
                result = {"id": case_id, "passed": False, "harness_error": repr(error), "traceback": traceback.format_exc()}
            results.append(result)
            (args.output_dir / f"{case_id}.json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")
            print(case_id, "PASS" if result["passed"] else "FAIL", result.get("checks", result.get("harness_error")), flush=True)
    finally:
        rclpy.shutdown()
    manifest["source_unchanged_after_run"] = source.read_bytes() == source_bytes
    manifest["finished_at"] = dt.datetime.now(ZoneInfo("UTC")).isoformat()
    report = {"manifest": manifest, "passed": sum(r["passed"] for r in results), "total": len(results), "cases": results}
    (args.output_dir / "results.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    print(f"Evidence: {args.output_dir}; {report['passed']}/{report['total']} requirements passed")
    return 0 if report["passed"] == report["total"] else 1


if __name__ == "__main__":
    sys.exit(main())
