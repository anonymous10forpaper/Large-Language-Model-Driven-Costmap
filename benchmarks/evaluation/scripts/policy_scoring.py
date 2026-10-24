import datetime as dt
import json
import math


ARRAY_FIELDS = (
    "plan", "via_zones", "exclusions", "weights",
    "dynamic_object_rules", "conditional_rules",
)
POLICY_FIELDS = {"decision", "reason_code", *ARRAY_FIELDS}
SIGNATURE_FIELDS = (
    "destinations", "via_zones", "keepouts", "timed_keepouts",
    "weights", "objects", "conditional",
)


def stable_unique(values):
    return [json.loads(value) for value in sorted({
        json.dumps(item, sort_keys=True, allow_nan=False) for item in values
    })]


def numeric(value):
    if isinstance(value, bool):
        raise ValueError("boolean_numeric_value")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError("nonfinite_numeric_value")
    return result


def normalize_gold(gold):
    result = {
        "destinations": [str(x).upper() for x in gold.get("destinations", [])],
        "via_zones": sorted(set(str(x).upper() for x in gold.get("via_zones", []))),
        "keepouts": sorted(set(str(x).upper() for x in gold.get("keepouts", []))),
        "timed_keepouts": {
            str(k).upper(): round(numeric(v), 3)
            for k, v in gold.get("timed_keepouts", {}).items()
        },
        "weights": {str(k).upper(): numeric(v) for k, v in gold.get("weights", {}).items()},
        "objects": {str(k).lower(): numeric(v) for k, v in gold.get("objects", {}).items()},
        "conditional": [],
    }
    for item in gold.get("conditional", []):
        parts = str(item).split(":", 3)
        if len(parts) == 4 and parts[1:3] == ["default", "forbid"]:
            zone = parts[0].upper()
            result["keepouts"].append(zone)
            result["weights"].pop(zone, None)
        else:
            result["conditional"].append(str(item))
    result["keepouts"] = sorted(set(result["keepouts"]))
    result["conditional"] = sorted(set(result["conditional"]))
    return result


def policy_signature(policy):
    result = {"destinations": [], "via_zones": [], "keepouts": [],
              "timed_keepouts": {}, "weights": {}, "objects": {}, "conditional": []}
    issues = []

    def issue(path, reason, value):
        issues.append({"path": path, "reason": reason, "value": value})

    def fields(value, allowed, path):
        if not isinstance(value, dict):
            issue(path, "object_required", value)
            return False
        for key in sorted(set(value) - set(allowed)):
            issue(path + "." + key, "unexpected_field", value[key])
        return True

    def zone(value, path):
        if not isinstance(value, str) or not value:
            issue(path, "zone_string_required", value)
            return None
        return value.upper()

    def assign(target, key, value, path):
        if key in target and target[key] != value:
            issue(path, "conflicting_duplicate", [target[key], value])
        target[key] = value

    if not fields(policy, POLICY_FIELDS, "policy"):
        return result, issues
    if str(policy.get("decision", "accept")).strip().lower() != "accept":
        issue("decision", "accept_required", policy.get("decision"))
    if str(policy.get("reason_code", "none")).strip().lower() != "none":
        issue("reason_code", "none_required", policy.get("reason_code"))
    arrays = {}
    for name in ARRAY_FIELDS:
        value = policy.get(name, [])
        if not isinstance(value, list):
            issue(name, "array_required", value)
            value = []
        arrays[name] = value
    for i, item in enumerate(arrays["plan"]):
        path = f"plan[{i}]"
        if not fields(item, {"dest"}, path):
            continue
        if "dest" not in item:
            issue(path, "destination_required", item)
            continue
        value = item["dest"]
        result["destinations"].append(value.upper() if isinstance(value, str) else value)
    for i, item in enumerate(arrays["via_zones"]):
        path = f"via_zones[{i}]"
        if isinstance(item, dict):
            fields(item, {"zone"}, path)
            item = item.get("zone")
        value = zone(item, path)
        if value is not None:
            result["via_zones"].append(value)
    for i, item in enumerate(arrays["exclusions"]):
        path = f"exclusions[{i}]"
        if isinstance(item, str):
            item = {"zone": item}
        if not fields(item, {"zone", "duration_minutes", "time_condition"}, path):
            continue
        value = zone(item.get("zone"), path + ".zone")
        if value is None:
            continue
        try:
            if "duration_minutes" in item and item["duration_minutes"] is not None:
                duration = round(numeric(item["duration_minutes"]), 3)
                if "time_condition" in item and item["time_condition"] is not None:
                    issue(path, "multiple_time_representations", item)
                assign(result["timed_keepouts"], value, duration, path)
            elif item.get("time_condition") is not None:
                condition = item["time_condition"]
                if not fields(condition, {"start", "end", "repeat"}, path + ".time_condition"):
                    continue
                start = dt.datetime.fromisoformat(str(condition["start"]))
                end = dt.datetime.fromisoformat(str(condition["end"]))
                duration = round((end - start).total_seconds() / 60, 3)
                if duration <= 0 or condition.get("repeat") is not None:
                    issue(path, "time_condition_mismatch", condition)
                assign(result["timed_keepouts"], value, duration, path)
            else:
                result["keepouts"].append(value)
        except (TypeError, ValueError, KeyError, OverflowError):
            issue(path, "invalid_time", item)
    for name, key_name, value_name, signature_name in (
        ("weights", "zone", "cost", "weights"),
        ("dynamic_object_rules", "class", "radius", "objects"),
    ):
        for i, item in enumerate(arrays[name]):
            path = f"{name}[{i}]"
            if not fields(item, {key_name, value_name}, path):
                continue
            key = item.get(key_name)
            if not isinstance(key, str) or not key:
                issue(path, "identifier_string_required", key)
                continue
            key = key.upper() if key_name == "zone" else key.lower()
            try:
                value = numeric(item[value_name])
                assign(result[signature_name], key, value, path)
            except (TypeError, ValueError, KeyError, OverflowError):
                issue(path, "invalid_numeric_value", item.get(value_name))
    for i, item in enumerate(arrays["conditional_rules"]):
        path = f"conditional_rules[{i}]"
        if not fields(item, {"zone", "rules"}, path):
            continue
        value = zone(item.get("zone"), path + ".zone")
        rules = item.get("rules")
        if not isinstance(rules, list):
            issue(path, "rules_array_required", rules)
            continue
        for j, rule in enumerate(rules):
            rule_path = f"{path}.rules[{j}]"
            if not fields(rule, {"priority", "state_condition", "condition", "action"}, rule_path):
                continue
            try:
                priority = numeric(rule.get("priority", 0))
                if not priority.is_integer():
                    issue(rule_path, "integer_priority_required", priority)
                condition = str(rule.get("state_condition", rule.get("condition", ""))).lower()
                action = str(rule.get("action", "")).lower()
                if "state_condition" in rule and "condition" in rule and rule["state_condition"] != rule["condition"]:
                    issue(rule_path, "condition_alias_conflict", rule)
                result["conditional"].append(f"{value}:{condition}:{action}:{int(priority)}")
            except (ValueError, TypeError, OverflowError):
                issue(rule_path, "invalid_priority", rule.get("priority"))
    for name in ("via_zones", "keepouts", "conditional"):
        result[name] = sorted(set(result[name]))
    return result, issues


def compare_policy(policy, gold):
    actual, issues = policy_signature(policy)
    expected = normalize_gold(gold)
    mismatches = {field: {"expected": expected[field], "actual": actual[field]}
                  for field in SIGNATURE_FIELDS if expected[field] != actual[field]}
    return {"exact": not issues and not mismatches, "mismatches": mismatches, "issues": issues}


def contradiction_metrics(policy, validation):
    recognized = isinstance(policy, dict) and str(policy.get("decision", "")).strip().lower() == "reject" and str(policy.get("reason_code", "")).strip().lower() == "contradiction"
    strict = recognized and set(policy) == POLICY_FIELDS and all(policy.get(field) == [] for field in ARRAY_FIELDS)
    blocked = validation["raw_error"] is not None or validation["validation_error"] is not None
    contained = blocked and validation["validated_policy"] is None and validation["policy_state_unchanged"] and validation["policy_version_unchanged"]
    return {"contradiction_recognition": bool(recognized), "strict_reject_contract": bool(strict),
            "end_to_end_containment": bool(contained), "stage3_eligible": validation["validated_policy"] is not None}


def wilson_interval(successes, n):
    z = 1.959963984540054
    p = successes / n
    denominator = 1 + z * z / n
    center = (p + z * z / (2 * n)) / denominator
    radius = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / denominator
    return [round(100 * (center - radius), 4), round(100 * (center + radius), 4)]
