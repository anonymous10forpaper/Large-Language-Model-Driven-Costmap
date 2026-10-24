import argparse
import ast
import copy
import datetime as dt
import hashlib
import json
import math
import re
from collections import Counter
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Dict, List, Optional, Tuple
from zoneinfo import ZoneInfo

from policy_scoring import compare_policy, contradiction_metrics, wilson_interval


OUTCOMES = (
    "correct_acceptance", "incorrect_acceptance", "interpreter_rejection",
    "parsing_rejection", "validator_rejection",
)


def processing_outcome(record, post_exact):
    raw = record["raw_output"]
    if record["raw_error"] is not None:
        return "parsing_rejection"
    if isinstance(raw, dict) and str(raw.get("decision", "")).strip().lower() == "reject":
        return "interpreter_rejection"
    if record["validation_error"] is not None or record["validated_policy"] is None:
        return "validator_rejection"
    return "correct_acceptance" if post_exact else "incorrect_acceptance"


def score_record(record, gold, experiment):
    if experiment == "contradictions_100":
        metrics = contradiction_metrics(record["raw_output"], record)
        metrics["processing_outcome"] = processing_outcome(record, False)
        return metrics
    raw = compare_policy(record["raw_output"], gold)
    post = compare_policy(record["validated_policy"], gold)
    raw_exact = record["raw_error"] is None and raw["exact"]
    post_exact = record["raw_error"] is None and record["validation_error"] is None and post["exact"]
    return {
        "raw_exact": raw_exact,
        "post_validator_exact": post_exact,
        "processing_outcome": processing_outcome(record, post_exact),
        "raw_mismatches": raw["mismatches"],
        "raw_issues": raw["issues"],
        "post_validator_mismatches": post["mismatches"],
        "post_validator_issues": post["issues"],
    }


def summarize(scores, experiment):
    n = len(scores)
    values = list(scores.values())
    if experiment == "contradictions_100":
        names = ("contradiction_recognition", "strict_reject_contract",
                 "end_to_end_containment", "stage3_eligible")
        result = {name: sum(row[name] for row in values) for name in names}
        result["recognized_with_residual_policy"] = result["contradiction_recognition"] - result["strict_reject_contract"]
        result["validator_containment_without_recognition"] = sum(
            row["end_to_end_containment"] and not row["contradiction_recognition"] for row in values
        )
        result["recognition_wilson95_percent"] = wilson_interval(result["contradiction_recognition"], n)
        result["containment_wilson95_percent"] = wilson_interval(result["end_to_end_containment"], n)
    else:
        result = {name: sum(row[name] for row in values)
                  for name in ("raw_exact", "post_validator_exact")}
        counts = Counter(row["processing_outcome"] for row in values)
        result["processing_outcomes"] = {name: counts[name] for name in OUTCOMES}
        result["raw_accuracy_percent"] = round(100 * result["raw_exact"] / n, 4)
        result["post_validator_accuracy_percent"] = round(100 * result["post_validator_exact"] / n, 4)
        result["post_validator_wilson95_percent"] = wilson_interval(result["post_validator_exact"], n)
    return {"commands": n, **result}


def load_validator(source, context):
    module = ast.parse(source.read_text(encoding="utf-8"))
    definitions = []
    method_names = {"_empty_policy", "_policy_has_effect", "_validate_policy_obj"}
    for item in module.body:
        if isinstance(item, ast.FunctionDef) and item.name == "normalize_soft_cost":
            definitions.append(item)
        elif isinstance(item, ast.ClassDef) and item.name == "PolicyUpdateRejected":
            definitions.append(item)
        elif isinstance(item, ast.ClassDef) and item.name == "PolicyBridgeNode":
            definitions.append(ast.ClassDef(
                name="OfflineValidator", bases=[], keywords=[],
                body=[node for node in item.body if isinstance(node, ast.FunctionDef) and node.name in method_names],
                decorator_list=[],
            ))
    tree = ast.fix_missing_locations(ast.Module(body=definitions, type_ignores=[]))
    namespace = {"dt": dt, "math": math, "json": json, "re": re,
                 "Any": Any, "Dict": Dict, "List": List, "Optional": Optional, "Tuple": Tuple}
    exec(compile(tree, str(source), "exec"), namespace)
    normalize = namespace["normalize_soft_cost"]
    minimum, maximum = context["soft_cost_min"], context["soft_cost_max"]
    if (normalize(minimum) != minimum or normalize(maximum) != maximum
            or normalize(minimum - 1) is not None or normalize(maximum + 1) is not None):
        raise ValueError("validator_soft_cost_context_mismatch")
    node = namespace["OfflineValidator"]()
    node._zone_db = {name: None for name in context["zones"]}
    node._waypoints = {name: None for name in context["waypoints"]}
    node._tz = ZoneInfo(context["timezone"])
    node._last_validation_report = {}
    return node


def temporal_representation(policy):
    value = copy.deepcopy(policy)
    if isinstance(value, dict):
        for item in value.get("exclusions", []):
            if not isinstance(item, dict) or not isinstance(item.get("time_condition"), dict):
                continue
            condition = item["time_condition"]
            start = dt.datetime.fromisoformat(condition["start"])
            end = dt.datetime.fromisoformat(condition["end"])
            item["time_condition"] = {
                "duration_seconds": round((end - start).total_seconds(), 6),
                "repeat": condition.get("repeat"),
            }
    return value


def replay_validation(node, record, command):
    if record["raw_error"] is not None:
        return record["validated_policy"] is None
    result = None
    error = None
    try:
        result = node._validate_policy_obj(copy.deepcopy(record["raw_output"]), command)
    except (RuntimeError, TypeError, ValueError) as exc:
        error = f"{type(exc).__name__}: {exc}"
    return error == record["validation_error"] and temporal_representation(result) == temporal_representation(record["validated_policy"])


def verify_hashes(root):
    for line in (root / "SHA256SUMS").read_text(encoding="utf-8").splitlines():
        digest, relative = line.split("  ", 1)
        target = (root / relative).resolve()
        if not target.is_relative_to(root.resolve()):
            raise ValueError(relative)
        if hashlib.sha256(target.read_bytes()).hexdigest() != digest:
            raise ValueError(f"hash_mismatch:{relative}")


def compute(root, verify=False, replay=False):
    if verify:
        verify_hashes(root)
    run_manifest = json.loads((root / "manifests/run_manifest.json").read_text(encoding="utf-8"))
    validator = load_validator(root / "source_snapshots/policybridge.py", run_manifest["validation_context"]) if replay else None
    all_results = {}
    replay_count = 0
    for experiment in ("benchmark_150", "contradictions_100"):
        references = json.loads((root / f"datasets/{experiment}.json").read_text(encoding="utf-8"))
        by_id = {row["id"]: row for row in references}
        runs = []
        for file in sorted((root / "records" / experiment).glob("run_*/*.json")):
            payload = json.loads(file.read_text(encoding="utf-8"))
            ids = [row["id"] for row in payload["results"]]
            if len(set(ids)) != len(ids) or set(ids) != set(by_id):
                raise ValueError(f"record_id_mismatch:{file}")
            scores = {}
            for record in payload["results"]:
                reference = by_id[record["id"]]
                scores[record["id"]] = score_record(record, reference.get("gold", {}), experiment)
                if replay:
                    if not replay_validation(validator, record, reference["command"]):
                        raise ValueError(f"validation_mismatch:{experiment}/{file.parent.name}/{file.stem}/{record['id']}")
                    replay_count += 1
            result = {"method": payload["method"], "run": file.parent.name,
                      "summary": summarize(scores, experiment), "results": scores}
            if verify:
                expected_file = root / "results" / experiment / file.parent.name / file.name
                expected = json.loads(expected_file.read_text(encoding="utf-8"))
                if result != expected:
                    raise ValueError(f"score_mismatch:{expected_file}")
            runs.append({"method": result["method"], "run": result["run"], **result["summary"]})
        all_results[experiment] = {"unique_commands": len(by_id), "runs": runs}
        if verify:
            expected = json.loads((root / f"results/{experiment}_summary.json").read_text(encoding="utf-8"))
            if expected != all_results[experiment]:
                raise ValueError(f"summary_mismatch:{experiment}")
    return {"evaluation": all_results, "validation_records_replayed": replay_count}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--verify", action="store_true")
    parser.add_argument("--replay-validation", action="store_true")
    args = parser.parse_args()
    print(json.dumps(compute(args.root, args.verify, args.replay_validation), ensure_ascii=True, indent=2))


if __name__ == "__main__":
    main()
