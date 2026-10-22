#!/usr/bin/env python3
import argparse
import copy
import datetime as dt
import json
import math
import os
from collections import Counter
from unittest.mock import patch

import rclpy

from policy_bridge.policybridge import PolicyBridgeNode, PolicyUpdateRejected


def policy(**fields):
    value = PolicyBridgeNode._empty_policy()
    value.update(fields)
    return value


SEED_POLICY = policy(
    plan=[{"dest": "WP1"}],
    exclusions=[{"zone": "E"}],
)


def raw_cases():
    command = "Keep zone D closed and send the robot to WP5."
    return [
        {
            "id": "A01", "category": "endpoint_and_json",
            "fault": "malformed endpoint JSON", "command": command,
            "body": "{", "expected_disposition": "reject",
            "realism": "HTTP response is truncated before JSON parsing.",
        },
        {
            "id": "A02", "category": "endpoint_and_json",
            "fault": "truncated endpoint envelope", "command": command,
            "body": '{"response":', "expected_disposition": "reject",
            "realism": "The local model endpoint closes during serialization.",
        },
        {
            "id": "A03", "category": "endpoint_and_json",
            "fault": "malformed JSON in response field", "command": command,
            "body": json.dumps({"response": '{"plan": ['}),
            "expected_disposition": "reject",
            "realism": "The endpoint envelope is valid but the model payload is truncated.",
        },
        {
            "id": "A04", "category": "endpoint_and_json",
            "fault": "markdown-fenced model payload", "command": command,
            "body": json.dumps({"response": '```json\n{"plan": []}\n```'}),
            "expected_disposition": "reject",
            "realism": "Chat-tuned models sometimes wrap JSON in Markdown fences.",
        },
        {
            "id": "A05", "category": "endpoint_and_json",
            "fault": "multiple JSON objects in response", "command": command,
            "body": json.dumps({"response": "{} {}"}),
            "expected_disposition": "reject",
            "realism": "A model emits a correction as a second object.",
        },
        {
            "id": "A06", "category": "endpoint_and_json",
            "fault": "empty model response", "command": command,
            "body": json.dumps({"response": ""}),
            "expected_disposition": "reject",
            "realism": "The endpoint returns metadata without generated content.",
        },
        {
            "id": "A07", "category": "endpoint_and_json",
            "fault": "endpoint timeout", "command": command,
            "exception": TimeoutError("injected timeout"),
            "expected_disposition": "reject",
            "realism": "The local inference service misses the request timeout.",
        },
        {
            "id": "A08", "category": "endpoint_and_json",
            "fault": "endpoint returns a JSON array", "command": command,
            "body": "[]", "expected_disposition": "reject",
            "realism": "Syntactically valid JSON has the wrong top-level shape.",
        },
    ]


def object_cases():
    wp2 = [{"dest": "WP2"}]
    wp3 = [{"dest": "WP3"}]
    wp5 = [{"dest": "WP5"}]
    wp8 = [{"dest": "WP8"}]
    fire_event = [{
        "zone": "A",
        "rules": [{
            "priority": 10,
            "state_condition": "fire_alarm",
            "action": "allow_shortest",
        }],
    }]
    cases = [
        {
            "id": "B01", "category": "schema_and_type",
            "fault": "parsed output is a top-level array",
            "command": "Visit WP2 first, then finish at WP7.",
            "input": [], "expected_disposition": "reject",
            "realism": "A valid JSON value violates the policy object schema.",
        },
        {
            "id": "B02", "category": "schema_and_type",
            "fault": "plan is a string instead of a list",
            "command": "Visit WP2 first, then finish at WP7.",
            "input": {"plan": "WP2, WP7"}, "expected_disposition": "reject",
            "realism": "The model compresses a repeated field into plain text.",
        },
        {
            "id": "B03", "category": "schema_and_type",
            "fault": "exclusions is an object instead of a list",
            "command": "Keep zone D closed and send the robot to WP5.",
            "input": {"plan": wp5, "exclusions": {"zone": "D"}},
            "expected_disposition": "reject",
            "realism": "A singleton list is serialized as an object.",
        },
        {
            "id": "B04", "category": "schema_and_type",
            "fault": "weights is an object instead of a list",
            "command": "Set A to cost 5 and B to cost 200, then go to WP3.",
            "input": {"plan": wp3, "weights": {"zone": "B", "cost": 200}},
            "expected_disposition": "reject",
            "realism": "A multi-zone field is emitted with the wrong container type.",
        },
        {
            "id": "B05", "category": "schema_and_type",
            "fault": "dynamic_object_rules is an object",
            "command": "Go to WP8 and keep 1.5 metres from people.",
            "input": {
                "plan": wp8,
                "dynamic_object_rules": {"class": "person", "radius": 1.5},
            },
            "expected_disposition": "reject",
            "realism": "A singleton object rule is not wrapped in an array.",
        },
        {
            "id": "B06", "category": "schema_and_type",
            "fault": "conditional_rules is an object",
            "command": (
                "Go to WP2; normally keep A closed, but open it when the "
                "authorized fire-response signal is active."
            ),
            "input": {"plan": wp2, "conditional_rules": fire_event[0]},
            "expected_disposition": "reject",
            "realism": "The event policy uses an object where the schema requires a list.",
        },
        {
            "id": "B07", "category": "schema_and_type",
            "fault": "retired legacy policy fields",
            "command": "Keep zone D closed and send the robot to WP5.",
            "input": {
                "plan": wp5, "exclusions": [{"zone": "D"}],
                "permanent_exclusions": ["D"],
                "qualitative_costs": [{"zone": "A", "level": "medium"}],
                "zone_preferences": [{"zone": "B", "level": "prefer"}],
                "path_id": 1,
            },
            "expected_disposition": "reject",
            "realism": (
                "A model or stale client emits fields removed from the current "
                "eight-field policy contract."
            ),
        },
        {
            "id": "B08", "category": "schema_and_type",
            "fault": "unsupported decision enum",
            "command": "Keep zone D closed and send the robot to WP5.",
            "input": {"decision": "proceed", "plan": wp5},
            "expected_disposition": "reject",
            "realism": "The model paraphrases an enum value instead of using the schema token.",
        },
        {
            "id": "C01", "category": "domain_and_range",
            "fault": "unknown zone F in model output",
            "command": "Keep zone D closed and send the robot to WP5.",
            "input": {"plan": wp5, "exclusions": [{"zone": "F"}]},
            "expected_disposition": "reject",
            "realism": "The model substitutes an adjacent but unconfigured zone label.",
        },
        {
            "id": "C02", "category": "domain_and_range",
            "fault": "unknown waypoint WP99 in model output",
            "command": "Keep zone D closed and send the robot to WP5.",
            "input": {"plan": [{"dest": "WP99"}], "exclusions": [{"zone": "D"}]},
            "expected_disposition": "reject",
            "realism": "A waypoint identifier is syntactically plausible but absent from the map.",
        },
        {
            "id": "C03", "category": "domain_and_range",
            "fault": "zone label has a numeric type",
            "command": "Keep zone D closed and send the robot to WP5.",
            "input": {"plan": wp5, "exclusions": [{"zone": 4}]},
            "expected_disposition": "reject",
            "realism": "The model emits an index instead of the configured zone label.",
        },
        {
            "id": "C04", "category": "domain_and_range",
            "fault": "negative traversal cost",
            "command": "Set zone B to cost 200 and go to WP3.",
            "input": {"plan": wp3, "weights": [{"zone": "B", "cost": -10}]},
            "expected_disposition": "reject",
            "realism": "A sign error creates a value outside Nav2's supported soft-cost range.",
        },
        {
            "id": "C05", "category": "domain_and_range",
            "fault": "reserved lethal value used as soft cost",
            "command": "Set zone B to cost 200 and go to WP3.",
            "input": {"plan": wp3, "weights": [{"zone": "B", "cost": 254}]},
            "expected_disposition": "reject",
            "realism": "The model crosses from the allowed 0-252 range into a reserved cost.",
        },
        {
            "id": "C06", "category": "domain_and_range",
            "fault": "boolean traversal cost",
            "command": "Set zone B to cost 200 and go to WP3.",
            "input": {"plan": wp3, "weights": [{"zone": "B", "cost": True}]},
            "expected_disposition": "reject",
            "realism": "JSON boolean must not be accepted as integer 1.",
        },
        {
            "id": "C07", "category": "domain_and_range",
            "fault": "negative person-clearance radius",
            "command": "Go to WP8 and keep 1.5 metres from people.",
            "input": {
                "plan": wp8,
                "dynamic_object_rules": [{"class": "person", "radius": -1.0}],
            },
            "expected_disposition": "reject",
            "realism": "A malformed numeric sign would invert a safety distance.",
        },
        {
            "id": "C08", "category": "domain_and_range",
            "fault": "person-clearance radius exceeds bound",
            "command": "Go to WP8 and keep 1.5 metres from people.",
            "input": {
                "plan": wp8,
                "dynamic_object_rules": [{"class": "person", "radius": 99.0}],
            },
            "expected_disposition": "reject",
            "realism": "A decimal or unit error produces an impractical avoidance radius.",
        },
        {
            "id": "C09", "category": "domain_and_range",
            "fault": "unsupported industrial object class",
            "command": "Go to WP8 and keep 1.5 metres from people.",
            "input": {
                "plan": wp8,
                "dynamic_object_rules": [{"class": "pallet", "radius": 1.5}],
            },
            "expected_disposition": "reject",
            "realism": "Pallet is plausible in a warehouse but not supported by this detector contract.",
        },
        {
            "id": "C10", "category": "domain_and_range",
            "fault": "incomplete coordinate destination",
            "command": "Send the robot to the registered point at x=2.0, y=3.0.",
            "input": {"plan": [{"dest": {"x": 2.0}}]},
            "expected_disposition": "reject",
            "realism": "A structured destination loses one required coordinate.",
        },
        {
            "id": "C11", "category": "domain_and_range",
            "fault": "non-finite coordinate destination",
            "command": "Send the robot to the registered point at x=2.0, y=3.0.",
            "input": {"plan": [{"dest": {"x": "nan", "y": 3.0}}]},
            "expected_disposition": "reject",
            "realism": "A failed numeric conversion reaches the structured output.",
        },
        {
            "id": "D01", "category": "time_and_conflict",
            "fault": "negative timed-closure duration",
            "command": "Close zone A for the next 30 minutes and go to WP2.",
            "input": {
                "plan": wp2,
                "exclusions": [{"zone": "A", "duration_minutes": -30}],
            },
            "expected_disposition": "reject",
            "realism": "A sign error would invert a temporary closure duration.",
        },
        {
            "id": "D02", "category": "time_and_conflict",
            "fault": "timed-closure duration exceeds supported horizon",
            "command": "Close zone A for the next 30 minutes and go to WP2.",
            "input": {
                "plan": wp2,
                "exclusions": [{"zone": "A", "duration_minutes": 2880}],
            },
            "expected_disposition": "reject",
            "realism": "A unit error changes minutes into a multi-day closure.",
        },
        {
            "id": "D03", "category": "time_and_conflict",
            "fault": "invalid ISO time strings",
            "command": "Keep A closed from 09:00 to 10:00 on 20 July 2026.",
            "input": {
                "exclusions": [{
                    "zone": "A",
                    "time_condition": {"start": "nine o'clock", "end": "ten o'clock"},
                }],
            },
            "expected_disposition": "reject",
            "realism": "The model copies natural-language time into an ISO-only field.",
        },
        {
            "id": "D04", "category": "time_and_conflict",
            "fault": "time interval ends before it starts",
            "command": "Keep A closed from 09:00 to 10:00 on 20 July 2026.",
            "input": {
                "exclusions": [{
                    "zone": "A",
                    "time_condition": {
                        "start": "2026-07-20T10:00:00+09:00",
                        "end": "2026-07-20T09:00:00+09:00",
                    },
                }],
            },
            "expected_disposition": "reject",
            "realism": "Start and end fields are swapped during serialization.",
        },
        {
            "id": "D05", "category": "time_and_conflict",
            "fault": "unsupported monthly recurrence",
            "command": "Keep A closed daily from 09:00 to 10:00.",
            "input": {
                "exclusions": [{
                    "zone": "A",
                    "time_condition": {
                        "start": "2026-07-20T09:00:00+09:00",
                        "end": "2026-07-20T10:00:00+09:00",
                        "repeat": "monthly",
                    },
                }],
            },
            "expected_disposition": "reject",
            "realism": "A plausible recurrence token is outside the implemented enum.",
        },
        {
            "id": "D06", "category": "time_and_conflict",
            "fault": "unsupported event condition",
            "command": (
                "Normally keep A closed; open it only when the authorized "
                "fire-response signal is active."
            ),
            "input": {
                "exclusions": [{"zone": "A"}],
                "conditional_rules": [{
                    "zone": "A",
                    "rules": [{
                        "priority": 10,
                        "state_condition": "maintenance_mode",
                        "action": "allow_shortest",
                    }],
                }],
            },
            "expected_disposition": "reject",
            "realism": "Maintenance mode is plausible but not connected to the current event interface.",
        },
        {
            "id": "D07", "category": "time_and_conflict",
            "fault": "unsupported conditional action",
            "command": (
                "Normally keep A closed; open it only when the authorized "
                "fire-response signal is active."
            ),
            "input": {
                "exclusions": [{"zone": "A"}],
                "conditional_rules": [{
                    "zone": "A",
                    "rules": [{
                        "priority": 10,
                        "state_condition": "fire_alarm",
                        "action": "reduce_speed",
                    }],
                }],
            },
            "expected_disposition": "reject",
            "realism": "Speed reduction is meaningful but absent from the policy action enum.",
        },
        {
            "id": "D08", "category": "time_and_conflict",
            "fault": "same zone required and excluded",
            "command": "Use B on the way to WP4, while keeping A off limits.",
            "input": {
                "plan": [{"dest": "WP4"}],
                "via_zones": ["B"],
                "exclusions": [{"zone": "A"}, {"zone": "B"}],
            },
            "expected_disposition": "reject",
            "realism": "The model assigns one mentioned aisle to both semantic roles.",
        },
        {
            "id": "D09", "category": "time_and_conflict",
            "fault": "unconditional transit conflicts with event-controlled zone",
            "command": (
                "Go through B to WP4. If the fire alarm is active, open A for "
                "the shortest safe route."
            ),
            "input": {
                "plan": [{"dest": "WP4"}],
                "via_zones": ["A", "B"],
                "conditional_rules": fire_event,
            },
            "expected_disposition": "reject",
            "realism": "A conditional zone is also emitted as an unconditional route requirement.",
        },
        {
            "id": "D10", "category": "time_and_conflict",
            "fault": "same event and priority have opposite actions",
            "command": (
                "Normally keep A closed; open it only when the authorized "
                "fire-response signal is active."
            ),
            "input": {
                "exclusions": [{"zone": "A"}],
                "conditional_rules": [{
                    "zone": "A",
                    "rules": [
                        {
                            "priority": 10,
                            "state_condition": "fire_alarm",
                            "action": "allow_shortest",
                        },
                        {
                            "priority": 10,
                            "state_condition": "fire_alarm",
                            "action": "forbid",
                        },
                    ],
                }],
            },
            "expected_disposition": "reject",
            "realism": "Two generated clauses produce an unresolved action conflict.",
        },
        {
            "id": "E01", "category": "meaning_preserving_normalization",
            "fault": "relative duration requires canonical time window",
            "command": "Close zone A for the next 30 minutes and go to WP2.",
            "input": {
                "plan": wp2,
                "exclusions": [{"zone": "A", "duration_minutes": 30}],
            },
            "expected_disposition": "normalize",
            "realism": "This is the supported operator-facing time form from the paper.",
        },
        {
            "id": "E02", "category": "meaning_preserving_normalization",
            "fault": "exact duplicate entries",
            "command": "Set A to cost 5 and go to WP3 while keeping D closed.",
            "input": {
                "plan": wp3 + wp3,
                "exclusions": [{"zone": "D"}, {"zone": "D"}],
                "weights": [{"zone": "A", "cost": 5}, {"zone": "A", "cost": 5}],
            },
            "expected_disposition": "normalize",
            "realism": "Autoregressive generation can repeat an otherwise valid item.",
        },
        {
            "id": "E03", "category": "meaning_preserving_normalization",
            "fault": "legacy fire policy representation",
            "command": (
                "Go to WP2; normally keep A closed, but open it when the "
                "authorized fire-response signal is active."
            ),
            "input": {
                "plan": wp2,
                "conditional_rules": [{
                    "zone": "A",
                    "rules": [
                        {
                            "priority": 5,
                            "state_condition": "default",
                            "action": "forbid",
                        },
                        {
                            "priority": 10,
                            "state_condition": "fire_alarm",
                            "action": "allow_shortest",
                        },
                    ],
                }],
            },
            "expected_disposition": "normalize",
            "realism": "Archived and current prompts encode the same state machine differently.",
        },
        {
            "id": "E04", "category": "merge_property",
            "fault": "valid snapshot replaces stale temporary fields",
            "command": "Set zone B to cost 200 and go to WP3.",
            "input": {
                "plan": wp3,
                "weights": [{"zone": "B", "cost": 200}],
            },
            "expected_disposition": "merge_property",
            "realism": (
                "A new mission must replace stale temporary keepouts and costs, "
                "and repeated commits must remain content-idempotent."
            ),
        },
        {
            "id": "F01", "category": "semantic_boundary_control",
            "fault": "structurally valid but semantically wrong waypoint",
            "command": "Visit WP2 first, then finish at WP7.",
            "input": {"plan": [{"dest": "WP6"}]},
            "expected_disposition": "semantic_boundary",
            "realism": "This control demonstrates that Stage 2 cannot repair a valid Stage 1 choice.",
        },
        {
            "id": "F02", "category": "semantic_boundary_control",
            "fault": "requested keepout is omitted from a valid policy",
            "command": "Keep zone D closed and send the robot to WP5.",
            "input": {"plan": wp5},
            "expected_disposition": "semantic_boundary",
            "realism": "This control bounds the validator: missing semantics remain a Stage 1 error.",
        },
    ]
    return cases


class FakeResponse:
    def __init__(self, body):
        self.body = body.encode("utf-8")

    def __enter__(self):
        return self

    def __exit__(self, *_args):
        return False

    def read(self):
        return self.body


def reset_state(node):
    node._active_policy_state = copy.deepcopy(SEED_POLICY)
    node._policy_version = 7
    node._last_policy_decision = "seeded for Gate 2 v3"


def raw_patch_kwargs(case):
    if "exception" in case:
        return {"side_effect": case["exception"]}
    return {"return_value": case["body"]}


def stage3_probe_for_rejected_case(node, case):
    reset_state(node)
    previous = copy.deepcopy(node._active_policy_state)
    previous_version = node._policy_version
    with patch.object(node, "_publish_validated_policy") as stage3_entry:
        if case["id"].startswith("A"):
            with patch.object(node, "_post_llm_request", **raw_patch_kwargs(case)):
                node._run_single_mission(case["command"])
        else:
            with patch.object(node, "_call_llm", return_value=copy.deepcopy(case["input"])):
                node._run_single_mission(case["command"])
    return {
        "stage3_entry_count": stage3_entry.call_count,
        "state_preserved": node._active_policy_state == previous,
        "version_preserved": node._policy_version == previous_version,
    }


def check_duration_normalization(normalized):
    exclusions = normalized.get("exclusions", [])
    if len(exclusions) != 1 or exclusions[0].get("zone") != "A":
        return False
    condition = exclusions[0].get("time_condition", {})
    try:
        start = dt.datetime.fromisoformat(condition["start"])
        end = dt.datetime.fromisoformat(condition["end"])
    except (KeyError, TypeError, ValueError):
        return False
    return math.isclose((end - start).total_seconds(), 1800.0, abs_tol=1.0)


def expected_normalized(case):
    if case["id"] == "E02":
        return policy(
            plan=[{"dest": "WP3"}],
            exclusions=[{"zone": "D"}],
            weights=[{"zone": "A", "cost": 5}],
        )
    if case["id"] == "E03":
        return policy(
            plan=[{"dest": "WP2"}],
            exclusions=[{"zone": "A"}],
            conditional_rules=[{
                "zone": "A",
                "rules": [{
                    "priority": 10,
                    "state_condition": "fire_alarm",
                    "action": "allow_shortest",
                }],
            }],
        )
    if case["id"] == "E04":
        return policy(
            plan=[{"dest": "WP3"}],
            weights=[{"zone": "B", "cost": 200}],
        )
    if case["id"] == "F01":
        return policy(plan=[{"dest": "WP6"}])
    if case["id"] == "F02":
        return policy(plan=[{"dest": "WP5"}])
    return None


def evaluate_raw_case(node, case):
    reset_state(node)
    previous = copy.deepcopy(node._active_policy_state)
    previous_version = node._policy_version
    reason = ""
    with patch.object(node, "_post_llm_request", **raw_patch_kwargs(case)) as model_call:
        try:
            node._plan_with_llm_or_fallback(case["command"])
            decision = "accepted"
        except PolicyUpdateRejected as exc:
            decision = "rejected"
            reason = str(exc)
    probe = stage3_probe_for_rejected_case(node, case)
    passed = (
        decision == "rejected"
        and model_call.call_count == 1
        and node._active_policy_state == previous
        and node._policy_version == previous_version
        and probe["stage3_entry_count"] == 0
        and probe["state_preserved"]
        and probe["version_preserved"]
    )
    return {
        **{key: case[key] for key in (
            "id", "category", "fault", "command", "expected_disposition", "realism"
        )},
        "injected_input": case.get("body", repr(case.get("exception"))),
        "actual_decision": decision,
        "decision_reason": reason,
        "model_call_count": model_call.call_count,
        "normalized_update": None,
        "validation_report": copy.deepcopy(getattr(node, "_last_validation_report", {})),
        "previous_policy_preserved": probe["state_preserved"],
        "previous_version_preserved": probe["version_preserved"],
        "stage3_entry_count": probe["stage3_entry_count"],
        "passed": passed,
    }


def evaluate_object_case(node, case):
    reset_state(node)
    if case["id"] == "E04":
        node._active_policy_state = policy(
            plan=[{"dest": "WP2"}],
            exclusions=[{"zone": "A"}],
            weights=[{"zone": "B", "cost": 5}],
        )
    previous = copy.deepcopy(node._active_policy_state)
    previous_version = node._policy_version
    normalized = None
    committed = None
    reason = ""
    validation_idempotent = None
    merge_content_idempotent = None
    try:
        normalized = node._validate_policy_obj(
            copy.deepcopy(case["input"]), case["command"]
        )
        validation_idempotent = (
            node._validate_policy_obj(copy.deepcopy(normalized), case["command"])
            == normalized
        )
        committed = node._commit_policy_snapshot(normalized)
        decision = "accepted"
    except PolicyUpdateRejected as exc:
        decision = "rejected"
        reason = str(exc)

    disposition = case["expected_disposition"]
    stage3_entry_count = None
    if disposition == "reject":
        probe = stage3_probe_for_rejected_case(node, case)
        stage3_entry_count = probe["stage3_entry_count"]
        passed = (
            decision == "rejected"
            and node._active_policy_state == previous
            and node._policy_version == previous_version
            and probe["state_preserved"]
            and probe["version_preserved"]
            and stage3_entry_count == 0
        )
    else:
        expected = expected_normalized(case)
        normalized_ok = normalized == expected if expected is not None else True
        if case["id"] == "E01":
            normalized_ok = check_duration_normalization(normalized or {})
        if case["id"] == "E04" and decision == "accepted":
            second = node._commit_policy_snapshot(copy.deepcopy(normalized))
            merge_content_idempotent = second == committed
            normalized_ok = normalized == expected
            committed_ok = (
                committed is not None
                and committed.get("plan") == [{"dest": "WP3"}]
                and committed.get("exclusions") == []
                and committed.get("weights") == [{"zone": "B", "cost": 200}]
                and merge_content_idempotent
            )
        else:
            committed_ok = decision == "accepted"
        passed = (
            decision == "accepted"
            and normalized_ok
            and validation_idempotent is True
            and committed_ok
        )

    return {
        **{key: case[key] for key in (
            "id", "category", "fault", "command", "expected_disposition", "realism"
        )},
        "injected_input": case["input"],
        "actual_decision": decision,
        "decision_reason": reason,
        "normalized_update": normalized,
        "committed_state": committed,
        "validation_report": copy.deepcopy(getattr(node, "_last_validation_report", {})),
        "previous_policy_preserved": node._active_policy_state == previous,
        "previous_version_preserved": node._policy_version == previous_version,
        "validation_idempotent": validation_idempotent,
        "merge_content_idempotent": merge_content_idempotent,
        "stage3_entry_count": stage3_entry_count,
        "passed": passed,
    }


def case_specification():
    cases = []
    for case in raw_cases() + object_cases():
        row = {
            key: copy.deepcopy(case[key])
            for key in (
                "id", "category", "fault", "command", "expected_disposition", "realism"
            )
        }
        row["injected_input"] = case.get("body", repr(case.get("exception"))) \
            if case["id"].startswith("A") else copy.deepcopy(case["input"])
        cases.append(row)
    return {
        "gate": 2,
        "version": "gate2-eight-field-schema-v4-2026-07-24",
        "status": "frozen_before_result_execution",
        "total_cases": len(cases),
        "policy_fields": list(PolicyBridgeNode._empty_policy()),
        "retired_fields": [
            "path_id",
            "zone_preferences",
            "qualitative_costs",
            "permanent_exclusions",
        ],
        "design_note": (
            "Commands are plausible warehouse tasks adapted from, but not copied as a "
            "second paraphrase benchmark of, Gate 3 executable50. Faults are injected "
            "directly at the endpoint or policy-object boundary."
        ),
        "contract": {
            "reject": "No commit, previous policy/version preserved, no Stage 3 entry.",
            "normalize": "Only representation-preserving canonicalization is allowed.",
            "merge_property": (
                "Mission fields use snapshot replacement and repeated commits are "
                "content-idempotent."
            ),
            "semantic_boundary": (
                "Structurally valid but semantically wrong or incomplete policy remains "
                "a Stage 1 error and is not repaired by Stage 2."
            ),
        },
        "cases": cases,
    }


def evaluate(node):
    # Exercise the real endpoint/JSON path even when the test node starts offline.
    with patch.object(node, "_enable_llm", True), patch.object(node, "_llm_endpoint", "mock://gate2"):
        return _evaluate(node)


def _evaluate(node):
    results = [evaluate_raw_case(node, case) for case in raw_cases()]
    results.extend(evaluate_object_case(node, case) for case in object_cases())
    failed = [case for case in results if not case["passed"]]
    injected_faults = [
        case for case in results
        if case["expected_disposition"] == "reject"
    ]
    validator_faults = [
        case for case in injected_faults if not case["id"].startswith("A")
    ]
    boundary_controls = [
        case for case in results
        if case["expected_disposition"] == "semantic_boundary"
    ]
    return {
        "metadata": {
            "gate": 2,
            "version": "gate2-eight-field-schema-v4-2026-07-24",
            "evaluation": "direct fault injection before Stage 3 costmap integration",
            "generated_at": dt.datetime.now(dt.timezone.utc).isoformat(),
            "policy_fields": list(PolicyBridgeNode._empty_policy()),
            "retired_fields": [
                "path_id",
                "zone_preferences",
                "qualitative_costs",
                "permanent_exclusions",
            ],
            "note": (
                "Malformed endpoint outputs and invalid policy objects are injected "
                "without Gazebo or Nav2. The Stage 3 trace publisher is spied on for "
                "every rejected case."
            ),
        },
        "summary": {
            "total_cases": len(results),
            "passed_cases": len(results) - len(failed),
            "failed_cases": len(failed),
            "contract_pass_rate": (len(results) - len(failed)) / len(results),
            "injected_fault_cases": len(injected_faults),
            "injected_faults_contained": sum(case["passed"] for case in injected_faults),
            "fault_containment_rate": (
                sum(case["passed"] for case in injected_faults) / len(injected_faults)
            ),
            "endpoint_or_json_rejections": sum(
                case["actual_decision"] == "rejected"
                for case in injected_faults if case["id"].startswith("A")
            ),
            "validator_rejections": sum(
                case["actual_decision"] == "rejected" for case in validator_faults
            ),
            "validator_fault_cases": len(validator_faults),
            "rejected_fault_previous_policy_preservation_rate": (
                sum(
                    case["previous_policy_preserved"]
                    and case["previous_version_preserved"]
                    for case in injected_faults
                ) / len(injected_faults)
            ),
            "rejected_fault_stage3_entry_count": sum(
                case["stage3_entry_count"] or 0 for case in injected_faults
            ),
            "normalization_cases": sum(
                case["expected_disposition"] == "normalize" for case in results
            ),
            "merge_property_cases": sum(
                case["expected_disposition"] == "merge_property" for case in results
            ),
            "semantic_boundary_controls": len(boundary_controls),
            "semantic_boundary_controls_accepted_unchanged": sum(
                case["actual_decision"] == "accepted" and case["passed"]
                for case in boundary_controls
            ),
            "category_counts": dict(sorted(Counter(
                case["category"] for case in results
            ).items())),
        },
        "cases": results,
    }


def make_node():
    rclpy.init(args=[
        "gate2_fault_injection_v4", "--ros-args",
        "-p", "enable_llm:=false", "-p", "enable_yolo:=false",
        "-p", "require_nav2:=false", "-p", "set_initial_pose:=false",
        "-p", "enable_console_ui:=false",
    ])
    node = PolicyBridgeNode()
    node._ui_print = lambda *_args, **_kwargs: None
    node._enable_llm = True
    node._llm_endpoint = "mock://gate2-v4"
    return node


def write_json(path, value):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", encoding="utf-8") as stream:
        json.dump(value, stream, ensure_ascii=True, indent=2)
        stream.write("\n")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output")
    parser.add_argument("--spec-output")
    parser.add_argument("--write-spec-only", action="store_true")
    args = parser.parse_args()
    if not args.output and not args.spec_output:
        parser.error("at least one of --output or --spec-output is required")
    if args.spec_output:
        write_json(args.spec_output, case_specification())
    if args.write_spec_only:
        return
    if not args.output:
        parser.error("--output is required unless --write-spec-only is used")
    node = make_node()
    try:
        report = evaluate(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    write_json(args.output, report)
    print(json.dumps(report["summary"], indent=2))
    raise SystemExit(0 if report["summary"]["failed_cases"] == 0 else 1)


if __name__ == "__main__":
    main()
