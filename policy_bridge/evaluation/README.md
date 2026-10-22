# Rule-Based Baseline Used in the Paper

This folder contains the deterministic keyword-and-slot parser used as the
rule-based comparator in the paper's natural-language-to-policy experiments:

- 150 English commands across phrasing levels L0-L4.
- 100 contradictory commands across five conflict types.
- 80 English commands from 10 participants across eight policy tasks (T1-T8)
  in the non-expert evaluation.

The paper entry point is **`gate3_v4_rule_parser.parse_rule_policy(command)`**.
The parser uses predefined vocabulary and regular expressions, without LLM
calls or prompt demonstrations. Its parsing logic is unchanged from the
evaluated baseline; comments identify its role in the paper.

## Files

| File | Role |
| --- | --- |
| `gate3_v4_rule_parser.py` | Actual baseline entry point used for the paper comparisons. |
| `gate3_v3_rule_parser.py` | Required helper functions imported by v4, including destination, zone, cost, person-clearance, and direct-conflict extraction. |

Keep both Python files in the same directory. The v3 file is an active
dependency, not an optional historical backup. Its own `parse_rule_policy()`
function and legacy preference code are not the final paper entry point.

## Run a Command

Python 3 and its standard library are sufficient. ROS, Nav2, Ollama, a GPU,
and a package build are not required for this parser-only example.

From the repository root:

```bash
cd policy_bridge/evaluation
python3 - <<'PY'
import json
from gate3_v4_rule_parser import parse_rule_policy

command = "Forbid zone A and go to WP2."
print(json.dumps(parse_rule_policy(command), indent=2))
PY
```

Expected output:

```json
{
  "decision": "accept",
  "reason_code": "none",
  "plan": [{"dest": "WP2"}],
  "via_zones": [],
  "exclusions": [{"zone": "A"}],
  "weights": [],
  "dynamic_object_rules": [],
  "conditional_rules": []
}
```

For a direct contradiction:

```python
parse_rule_policy("Forbid zone A and go to WP2 through zone A.")
```

This returns `decision: "reject"` and `reason_code: "contradiction"`, with
all six executable policy arrays empty. The example is one recognized
pattern, not a claim that the rule-based parser detects every contradiction.

## Evaluation Scope

The returned object is the parser's **raw candidate policy**, not a
ground-truth label or a validated navigation command. This example does not
publish ROS topics, change a costmap, or start navigation.

The paper separately evaluates raw conversion, post-validation exact match,
and contradiction containment. Post-validation and end-to-end containment
also require the Policy Validator and the evaluation harness; running this
parser alone does not reproduce those downstream metrics. Full benchmark
runners and result-generation scripts are not included in this folder.

This baseline is separate from both the runtime GUI's `_fallback_parse()`
and the Policy Validator. It is not used to replace LLM interpretation in
the normal GUI workflow, and it does not dynamically learn new labels from
the GUI's map registry.

## Related Materials

- [Benchmark commands, ground-truth policies, and prompt demonstrations](https://anonymous10forpaper.github.io/Large-Language-Model-Driven-Costmap_Page/benchmark.html)
- [Non-expert study instructions and participant commands](https://anonymous10forpaper.github.io/Large-Language-Model-Driven-Costmap_Page/user-study.html)
- [Runtime installation and navigation setup](../../README.md)
