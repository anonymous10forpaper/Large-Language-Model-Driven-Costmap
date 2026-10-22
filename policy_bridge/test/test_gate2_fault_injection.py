import pytest
import rclpy

from policy_bridge.policybridge import PolicyBridgeNode, PolicyUpdateRejected
from gate2_fault_injection_runner import evaluate


@pytest.fixture(scope="module")
def node():
    rclpy.init(args=[
        "gate2_policy_rules", "--ros-args",
        "-p", "enable_llm:=false", "-p", "enable_yolo:=false",
        "-p", "require_nav2:=false", "-p", "set_initial_pose:=false",
        "-p", "enable_console_ui:=false",
    ])
    instance = PolicyBridgeNode()
    instance._ui_print = lambda *_args, **_kwargs: None
    instance._yolo_run = False
    if hasattr(instance, "_yolo_thread"):
        instance._yolo_thread.join(timeout=1.0)
    yield instance
    instance.destroy_node()
    rclpy.shutdown()


def test_gate2_fault_injection(node):
    report = evaluate(node)
    summary = report["summary"]

    assert summary["total_cases"] == 43
    assert summary["failed_cases"] == 0
    assert summary["injected_fault_cases"] == 37
    assert summary["fault_containment_rate"] == 1.0
    assert summary["validator_rejections"] == summary["validator_fault_cases"]
    assert summary["rejected_fault_previous_policy_preservation_rate"] == 1.0
    assert summary["rejected_fault_stage3_entry_count"] == 0
    assert summary["semantic_boundary_controls_accepted_unchanged"] == 2
    assert all(case["passed"] for case in report["cases"])


def test_current_policy_contract_has_only_eight_fields():
    assert list(PolicyBridgeNode._empty_policy()) == [
        "decision",
        "reason_code",
        "plan",
        "via_zones",
        "exclusions",
        "weights",
        "dynamic_object_rules",
        "conditional_rules",
    ]


def test_lowercase_article_is_not_treated_as_zone_a(node):
    with pytest.raises(PolicyUpdateRejected):
        node._validate_policy_obj({}, "Move to x=not-a-number, y=?")


@pytest.mark.parametrize(
    ("command", "minutes"),
    [
        ("Close A for 30 minutes.", 30),
        ("Keep aisle A unavailable for half an hour.", 30),
        ("Temporarily bypass A for thirty minutes.", 30),
    ],
)
def test_development_duration_phrases(node, command, minutes):
    assert node._parse_duration_minutes(command) == minutes
