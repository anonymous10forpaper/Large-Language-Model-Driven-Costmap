import copy
from unittest.mock import Mock

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
        "soft_cost_contract", "--ros-args",
        "-p", "enable_llm:=false", "-p", "enable_yolo:=false",
        "-p", "require_nav2:=false", "-p", "set_initial_pose:=false",
        "-p", "enable_console_ui:=false",
    ])
    instance = PolicyBridgeNode()
    instance._ui_print = lambda *_args, **_kwargs: None
    yield instance
    instance.destroy_node()
    rclpy.shutdown()


def test_every_allowed_integer_is_preserved():
    for value in range(253):
        assert normalize_soft_cost(value) == value
        assert normalize_soft_cost(str(value)) == value


@pytest.mark.parametrize("value", [
    -1, 253, 254, 255, 256, 999, 1000, 2**64,
    "-1", "253", "254", "255", "999", "252.9", "invalid",
    True, False, None, [], {}, 252.0, 252.9, -0.1,
    float("nan"), float("inf"),
])
def test_invalid_cost_is_not_clipped_or_truncated(value):
    assert normalize_soft_cost(value) is None


@pytest.mark.parametrize("cost", [0, 1, 200, 251, 252, "252"])
def test_validator_accepts_soft_cost_boundary(node, cost):
    policy = node._validate_policy_obj(
        {"plan": [{"dest": "WP2"}], "weights": [{"zone": "B", "cost": cost}]},
        "Set Zone B's cost and go to WP2.",
    )
    assert policy["weights"] == [{"zone": "B", "cost": int(cost)}]


@pytest.mark.parametrize("cost", [-1, 253, 254, 255, 1000, True, 252.9])
def test_rejected_update_preserves_policy_and_never_publishes(node, monkeypatch, cost):
    previous = {
        **node._empty_policy(),
        "plan": [{"dest": "WP1"}],
        "weights": [{"zone": "A", "cost": 200}],
    }
    monkeypatch.setattr(node, "_active_policy_state", copy.deepcopy(previous))
    monkeypatch.setattr(node, "_policy_version", 7)
    monkeypatch.setattr(node, "_enable_llm", True)
    monkeypatch.setattr(node, "_llm_endpoint", "mock://soft-cost-test")
    raw = {
        "plan": [{"dest": "WP2"}],
        "weights": [{"zone": "A", "cost": 5}, {"zone": "B", "cost": cost}],
    }
    monkeypatch.setattr(node, "_call_llm", lambda _text: copy.deepcopy(raw))
    publish = Mock()
    monkeypatch.setattr(node, "_publish_softcost", publish)

    with pytest.raises(PolicyUpdateRejected, match="invalid cost"):
        node._plan_with_llm_or_fallback("Set the zone costs and go to WP2.")

    assert node._active_policy_state == previous
    assert node._policy_version == 7
    publish.assert_not_called()


def test_prompt_advertises_the_same_soft_cost_range(node):
    prompt = node._build_planner_prompt("Go to WP2.")
    assert "0..252" in prompt
    assert "0..253" not in prompt

