"""Frozen regex/slot-filling baseline for the Gate 3 v4 evaluation."""

# Paper baseline: parse_rule_policy() in this v4 module was used for the
# 150-command, 100-contradiction, and non-expert evaluations.
# Requires the v3 helpers below; not the GUI fallback or Policy Validator.

import re

from gate3_v3_rule_parser import (
    _destination,
    _has_direct_conflict,
    _keepouts,
    _object_rules,
    _via_zones,
    _weights as _legacy_weights,
)


def _duration_minutes(command):
    lower = command.lower()
    match = re.search(r"\b(\d+(?:\.\d+)?)\s*(minutes?|mins?|min)\b", lower)
    if match:
        return float(match.group(1))
    match = re.search(r"\b(\d+(?:\.\d+)?)\s*(hours?|hrs?|hr)\b", lower)
    if match:
        return float(match.group(1)) * 60.0
    if re.search(r"\b(?:half\s+(?:an?\s+)?hour|half-hour|thirty\s+minutes?)\b", lower):
        return 30.0
    return None


def _conditional_rules(command):
    lower = command.lower()
    fire_event = bool(
        re.search(r"\b(?:fire|emergency)\b.{0,20}\b(?:alarm|siren|warning|reported)\b", lower)
        or "fire emergency" in lower
    )
    default_forbid = bool(
        re.search(r"\b(?:normally|default|usual(?:ly)?)\b.{0,45}\b(?:forbid|forbidden|close|closed|avoid|route around)\b", lower)
        or re.search(r"\b(?:forbid|forbidden|close|closed)\b.{0,35}\b(?:normally|by default)\b", lower)
        or re.search(r"\b(?:zone|aisle|section|area)\s*[a-g]\b.{0,20}\b(?:forbidden|closed|off[- ]limits)\b", lower)
    )
    emergency_allow = bool(
        re.search(r"\b(?:shortest|fastest|quickest|direct)\b.{0,20}\b(?:route|way|path|passage)?\b", lower)
        or re.search(r"\b(?:allow|open|permit|override)\b.{0,35}\b(?:through|route|way|passage)\b", lower)
    )
    if not (fire_event and default_forbid and emergency_allow):
        return []

    match = re.search(
        r"\b(?:zone|aisle|section|area)\s*([A-G])\b|\b([A-G])\b",
        command,
        re.IGNORECASE,
    )
    if not match:
        return []
    zone = (match.group(1) or match.group(2)).upper()
    return [{
        "zone": zone,
        "rules": [
            {"priority": 10, "state_condition": "fire_alarm", "action": "allow_shortest"},
        ],
    }]


def _weights(command):
    slots = {
        item["zone"]: item["cost"]
        for item in _legacy_weights(command)
    }
    if not re.search(r"\b(?:cost|penalty|score|value|rating)\b", command, re.IGNORECASE):
        return [{"zone": zone, "cost": slots[zone]} for zone in sorted(slots)]
    patterns = (
        r"\b(?:zone|aisle)\s*([A-G])\b.{0,18}?\b(?:cost|penalty|score|value|rating)\b\s*(?:of|to|at|is|=)?\s*(-?\d+(?:\.\d+)?)",
        r"\b(?:cost|penalty|score|value|rating)\b.{0,12}?\b(?:zone|aisle)\s*([A-G])\b\s*(?:of|to|at|is|=)?\s*(-?\d+(?:\.\d+)?)",
        r"\b(?:zone|aisle)\s*([A-G])\b\s*(?:to|at|is|=)\s*(-?\d+(?:\.\d+)?)",
        r"\b([A-G])\b\s+(?:should\s+carry\s+)?(?:cost|penalty|score|value|rating)\s*(?:of|to|at|is|=)?\s*(-?\d+(?:\.\d+)?)",
    )
    for pattern in patterns:
        for match in re.finditer(pattern, command, re.IGNORECASE):
            value = float(match.group(2))
            slots[match.group(1).upper()] = int(value) if value.is_integer() else value
    return [{"zone": zone, "cost": slots[zone]} for zone in sorted(slots)]


def parse_rule_policy(command):
    destination = _destination(command)
    exclusions = _keepouts(command)
    via = _via_zones(command)
    conditional = _conditional_rules(command)
    conditional_zones = {item["zone"] for item in conditional}
    exclusions = list(dict.fromkeys(exclusions + sorted(conditional_zones)))

    if _has_direct_conflict(command, exclusions, via):
        return {
            "decision": "reject",
            "reason_code": "contradiction",
            "plan": [],
            "via_zones": [],
            "exclusions": [],
            "weights": [],
            "dynamic_object_rules": [],
            "conditional_rules": [],
        }

    duration = _duration_minutes(command)
    exclusion_rules = []
    for zone in exclusions:
        rule = {"zone": zone}
        if duration is not None:
            rule["duration_minutes"] = duration
        exclusion_rules.append(rule)

    return {
        "decision": "accept",
        "reason_code": "none",
        "plan": [{"dest": destination}] if destination else [],
        "via_zones": via,
        "exclusions": exclusion_rules,
        "weights": _weights(command),
        "dynamic_object_rules": _object_rules(command),
        "conditional_rules": conditional,
    }
