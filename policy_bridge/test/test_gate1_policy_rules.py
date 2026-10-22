import copy

import pytest
import rclpy

from policy_bridge.policybridge import (
    PolicyBridgeNode,
    PolicyUpdateRejected,
    normalize_soft_cost,
)


@pytest.fixture(scope="module")
def node():
    rclpy.init(args=[
        "gate1_policy_rules",
        "--ros-args",
        "-p", "enable_llm:=false",
        "-p", "enable_yolo:=false",
        "-p", "require_nav2:=false",
        "-p", "set_initial_pose:=false",
        "-p", "enable_console_ui:=false",
    ])
    instance = PolicyBridgeNode()
    instance._ui_print = lambda *args, **kwargs: None
    instance._yolo_run = False
    if hasattr(instance, "_yolo_thread"):
        instance._yolo_thread.join(timeout=1.0)
    yield instance
    instance.destroy_node()
    rclpy.shutdown()


def test_soft_cost_range_excludes_lethal_and_invalid_values():
    assert normalize_soft_cost(0) == 0
    assert normalize_soft_cost("252") == 252
    assert normalize_soft_cost(253) is None
    assert normalize_soft_cost(-10) is None
    assert normalize_soft_cost(254) is None
    assert normalize_soft_cost(255) is None
    assert normalize_soft_cost(999) is None
    assert normalize_soft_cost(True) is None


def test_invalid_fields_reject_the_entire_update(node):
    command = "Go to WP2, keep A closed, but if the moon is full, allow the shortest path."
    raw = {
        "plan": [{"dest": "WP2"}],
        "exclusions": [{"zone": "A"}],
        "weights": [{"zone": "B", "cost": 999}],
        "dynamic_object_rules": [{"class": "person", "radius": 1.5}],
        "conditional_rules": [{
            "zone": "A",
            "rules": [
                {"priority": 10, "state_condition": "moon_full", "action": "allow_shortest"},
                {"priority": 9, "state_condition": "default", "action": "allow_shortest"},
                {"priority": 5, "state_condition": "default", "action": "forbid"},
            ],
        }],
    }

    with pytest.raises(PolicyUpdateRejected, match="invalid cost"):
        node._validate_policy_obj(raw, command)


def test_known_fire_alarm_rule_is_preserved(node):
    command = (
        "Go to WP2, and zone A is forbidden, but if the fire alarm goes off, "
        "allow shortest-path traversal."
    )
    raw = {
        "plan": [{"dest": "WP2"}],
        "conditional_rules": [{
            "zone": "A",
            "rules": [
                {"priority": 10, "state_condition": "fire_alarm", "action": "allow_shortest"},
                {"priority": 5, "state_condition": "default", "action": "forbid"},
            ],
        }],
    }

    policy = node._validate_policy_obj(raw, command)

    assert len(policy["conditional_rules"]) == 1
    assert policy["conditional_rules"][0]["rules"][0]["state_condition"] == "fire_alarm"
    assert policy["conditional_rules"][0]["rules"][0]["action"] == "allow_shortest"


def test_fire_alarm_off_on_off_restores_default_forbid(node):
    rules = [{
        "zone": "A",
        "rules": [
            {"priority": 10, "state_condition": "fire_alarm", "action": "allow_shortest"},
            {"priority": 5, "state_condition": "default", "action": "forbid"},
        ],
    }]

    node._env_state["fire_alarm"] = False
    forbid, allow, shortest = node._evaluate_conditional_rules(rules)
    assert forbid == {"A"}
    assert allow == set()
    assert shortest is False

    node._env_state["fire_alarm"] = True
    forbid, allow, shortest = node._evaluate_conditional_rules(rules)
    assert forbid == set()
    assert allow == {"A"}
    assert shortest is True

    node._env_state["fire_alarm"] = False
    forbid, allow, shortest = node._evaluate_conditional_rules(rules)
    assert forbid == {"A"}
    assert allow == set()
    assert shortest is False


def test_keepout_and_soft_cost_conflict_rejects_entire_update(node):
    raw = {
        "plan": [{"dest": "WP2"}],
        "exclusions": [{"zone": "A"}],
        "weights": [{"zone": "A", "cost": 5}, {"zone": "B", "cost": 200}],
    }
    with pytest.raises(PolicyUpdateRejected, match="both keepout and soft cost"):
        node._validate_policy_obj(
            raw,
            "Forbid zone A, set A to 5 and B to 200, then go to WP2.",
        )


def test_structurally_valid_destinations_are_not_semantically_repaired(node):
    raw = {
        "plan": [
            {"dest": "WP1"},
            {"dest": {"x": 99.0, "y": 99.0, "yaw": 0.0}},
        ],
        "exclusions": [{"zone": "A"}],
    }
    policy = node._validate_policy_obj(raw, "Avoid A and go somewhere safe.")

    assert policy["plan"] == [
        {"dest": "WP1"},
        {"dest": {"x": 99.0, "y": 99.0, "yaw": 0.0}},
    ]
    assert policy["exclusions"] == [{"zone": "A"}]


def test_wrong_schema_type_rejects_entire_update(node):
    with pytest.raises(PolicyUpdateRejected):
        node._validate_policy_obj({"plan": "WP2"}, "Go to WP2.")


def test_empty_policy_rejects_but_valid_object_only_policy_is_allowed(node):
    with pytest.raises(PolicyUpdateRejected):
        node._validate_policy_obj({}, "Please do the best route.")
    validated = node._validate_policy_obj(
        {"dynamic_object_rules": [{"class": "person", "radius": 1.5}]},
        "Please keep the standard clearance from workers.",
    )
    assert validated["dynamic_object_rules"] == [
        {"class": "person", "radius": 1.5}
    ]


def test_rejected_llm_update_preserves_previous_snapshot(node):
    node._enable_llm = True
    node._llm_endpoint = "mock://llm"
    node._active_policy_state = node._empty_policy()
    node._policy_version = 0

    node._call_llm = lambda _text: {
        "plan": [{"dest": "WP2"}],
        "exclusions": [{"zone": "A"}],
    }
    committed = node._plan_with_llm_or_fallback("Forbid zone A and go to WP2.")
    previous = copy.deepcopy(node._active_policy_state)
    previous_version = node._policy_version
    assert committed == previous

    def rejected(_text):
        raise PolicyUpdateRejected("malformed JSON")

    node._call_llm = rejected
    with pytest.raises(PolicyUpdateRejected):
        node._plan_with_llm_or_fallback("Go somewhere safe.")

    assert node._active_policy_state == previous
    assert node._policy_version == previous_version


def test_valid_snapshot_replaces_mission_fields_but_keeps_permanent_rules(node, monkeypatch):
    monkeypatch.setattr(node, "_permanent_exclusions", ["D"])
    node._active_policy_state = {
        **node._empty_policy(),
        "plan": [{"dest": "WP2"}],
        "exclusions": [{"zone": "A"}],
    }
    node._policy_version = 1

    committed = node._commit_policy_snapshot({
        **node._empty_policy(),
        "plan": [{"dest": "WP3"}],
        "weights": [{"zone": "B", "cost": 200}],
    })

    assert committed["plan"] == [{"dest": "WP3"}]
    assert committed["exclusions"] == []
    assert committed["weights"] == [{"zone": "B", "cost": 200}]
    assert "permanent_exclusions" not in committed
    assert node._permanent_exclusions == ["D"]


def test_explicit_zone_and_waypoint_identifiers_are_case_normalized(node):
    normalized = node._canonicalize_named_entities(
        "forbid zone f and go to wp01"
    )

    assert normalized == "forbid zone F and go to WP1"


def test_deterministic_parser_does_not_strip_negative_sign_or_clip_high_cost(node):
    _, negative = node._split_intents_deterministic(
        "Set zone B cost to -10 and go to WP3."
    )
    _, high = node._split_intents_deterministic(
        "Set zone B cost to 999 and go to WP3."
    )
    _, valid = node._split_intents_deterministic(
        "Set zone B cost to 200 and go to WP3."
    )

    assert negative == {}
    assert high == {}
    assert valid == {"B": 200}
