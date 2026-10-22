# Shared helper dependency of the paper's gate3_v4_rule_parser baseline.
# Final paper evaluations call v4.parse_rule_policy(), not this v3 entry point.

import re


KEEP_OUT = (
    "forbid", "forbidden", "block", "blocked", "close", "closed", "off limits",
    "off-limits", "restricted", "unavailable", "avoid", "no-entry", "no go",
)
OPEN = ("open", "allowed", "available", "may use", "can use")
PERSON = (
    "person", "people", "pedestrian", "worker", "staff", "personnel", "operator",
    "crew", "human", "anyone",
)
PERSON_CLEARANCE = (
    "avoid", "space", "room", "clear", "distance", "clearance", "buffer",
    "separation", "safety margin", "wide berth", "close pass", "protective radius",
    "approach", "brush past", "away from", "gap", "metre", "meter",
)


def _zones(text):
    found = []
    occupied = []
    for match in re.finditer(r"\b(?:zone|aisle|section|area)\s*([A-E])\b", text, re.IGNORECASE):
        found.append((match.group(1).upper(), match.start(1)))
        occupied.append((match.start(1), match.end(1)))
    for match in re.finditer(r"\b([A-E])\b", text):
        if not any(start <= match.start(1) < end for start, end in occupied):
            found.append((match.group(1), match.start(1)))
    return [zone for zone, _position in sorted(found, key=lambda item: item[1])]


def _destination(command):
    patterns = (
        r"actual destination is\s*(WP\s*[1-8])",
        r"real destination(?:,| is)?\s*(WP\s*[1-8])",
        r"final (?:destination|drop point)(?: is)?\s*(WP\s*[1-8])",
        r"(WP\s*[1-8])\s+is the (?:final|actual|real) (?:destination|drop point|goal)",
    )
    for pattern in patterns:
        match = re.search(pattern, command, re.IGNORECASE)
        if match:
            return "WP" + re.search(r"[1-8]", match.group(1)).group(0)
    values = [f"WP{int(value)}" for value in re.findall(r"\bWP\s*([1-8])\b", command, re.IGNORECASE)]
    if not values:
        return None
    if len(values) > 1 and any(
        cue in command.lower()
        for cue in (
            "instead", "correction", "reference", "not the destination", "not our goal",
            "not the target", "cancelled request", "rather than",
        )
    ):
        return values[-1]
    return values[0]


def _is_negated(clause):
    return bool(re.search(r"\b(?:do\s+not|don't|not)\s+(?:avoid|forbid|block|close)\b", clause, re.IGNORECASE))


def _keepouts(command):
    keep = set()
    opened = set()
    clauses = re.split(r"[.,;—]|\bbut\b|\bwhereas\b|\band\b", command, flags=re.IGNORECASE)
    for clause in clauses:
        lower = clause.lower()
        zones = _zones(clause)
        has_open = any(re.search(rf"\b{re.escape(term)}\b", lower) for term in OPEN)
        if _is_negated(clause) or has_open:
            opened.update(zones)
            continue
        if any(term in lower for term in KEEP_OUT):
            keep.update(zones)
    replacement = re.search(
        r"(?:avoid|forbid|block|close|keep)\s+(?:zone\s+)?([A-E]).{0,15}\binstead\b",
        command,
        re.IGNORECASE,
    )
    if replacement and any(cue in command.lower() for cue in ("correction", "instead")):
        keep = {replacement.group(1).upper()}
    return sorted(keep - opened)


def _via_zones(command):
    via = []
    patterns = (
        r"\b(?:through|via)\s+(?:zone|aisle|section|area)?\s*([A-E])\b",
        r"\bpass\s+through\s+(?:zone|aisle|section|area)?\s*([A-E])\b",
        r"\buse\s+(?:zone|aisle|section|area)?\s*([A-E])\s+as\s+(?:the\s+)?(?:transit|route|corridor|passage)",
    )
    for pattern in patterns:
        for match in re.finditer(pattern, command, re.IGNORECASE):
            prefix = command[max(0, match.start() - 12):match.start()].lower()
            if not re.search(r"\b(?:not|never|don't)\b", prefix):
                via.append(match.group(1).upper())
    return list(dict.fromkeys(via))


def _weights(command):
    slots = {}
    source = command
    lower = command.lower()
    for cue in ("current", "replace it with"):
        if cue in lower:
            source = command[lower.rfind(cue):]
            break
    patterns = (
        r"\b(?:zone\s+)?(?-i:([A-E]))(?:'s)?\s+(?:a\s+)?(?:route\s+|traversal\s+)?(?:cost|penalty|score|value|rating)\s*(?:of|to|at|is|=)?\s*(-?\d+(?:\.\d+)?)",
        r"\b(?:cost|penalty|score|value|rating)\b.{0,12}?\b(?:zone\s+)?(?-i:([A-E]))\b\s*(?:to|at|=)?\s*(-?\d+(?:\.\d+)?)",
        r"\b(?-i:([A-E]))\s*=\s*(-?\d+(?:\.\d+)?)",
    )
    for pattern in patterns:
        for match in re.finditer(pattern, source, re.IGNORECASE):
            value = float(match.group(2))
            slots[match.group(1).upper()] = int(value) if value.is_integer() else value
    return [{"zone": zone, "cost": slots[zone]} for zone in sorted(slots)]


def _preferences(command):
    preferences = {}
    for zone in _zones(command):
        zone_ref = rf"(?:(?:zone|aisle|section|area)\s*)?(?-i:{zone})\b"
        window_patterns = (
            ("strong_discourage", rf"(?:strongly|really)\b.{{0,35}}(?:not\s+(?:use|take)|avoid)\b.{{0,15}}{zone_ref}|{zone_ref}.{{0,35}}(?:last resort|strongly discouraged)"),
            ("slight_discourage", rf"(?:slightly|somewhat|mildly)\b.{{0,35}}(?:avoid|discourage|undesirable)\b.{{0,15}}{zone_ref}|{zone_ref}.{{0,35}}(?:slightly|somewhat|mildly)\b.{{0,20}}(?:avoid|discouraged|undesirable)"),
            ("discourage", rf"(?:prefer|rather)\b.{{0,15}}not\b.{{0,20}}(?:use|take)\b.{{0,15}}{zone_ref}|{zone_ref}.{{0,30}}(?:use less often|not keen|discouraged)"),
            ("prefer", rf"(?:prefer|favor|favour)\b.{{0,20}}{zone_ref}|{zone_ref}.{{0,25}}(?:preferred|favored|favoured)"),
        )
        for level, pattern in window_patterns:
            if re.search(pattern, command, re.IGNORECASE):
                preferences[zone] = level
                break
    return [{"zone": zone, "level": preferences[zone]} for zone in sorted(preferences)]


def _object_rules(command):
    lower = command.lower()
    if not (any(term in lower for term in PERSON) and any(term in lower for term in PERSON_CLEARANCE)):
        return []
    match = re.search(r"\b(\d+(?:\.\d+)?)\s*(?:m|metres?|meters?)\b", lower)
    radius = float(match.group(1)) if match else 1.5
    if not match:
        for word, value in (("two", 2.0), ("three", 3.0)):
            if re.search(rf"\b{word}(?:-metre| metre| meter)", lower):
                radius = value
                break
    return [{"class": "person", "radius": radius}]


def _has_direct_conflict(command, keepouts, via):
    if set(keepouts) & set(via):
        return True
    lower = command.lower()
    waypoints = re.findall(r"\bWP\s*([1-8])\b", command, re.IGNORECASE)
    if len(set(waypoints)) > 1 and "only final destination" in lower:
        return True
    if any(term in lower for term in PERSON) and "0.1" in lower:
        if any(term in lower for term in PERSON_CLEARANCE):
            return True
    for zone in set(_zones(command)):
        zone_ref = rf"(?:(?:zone|aisle|section|area)\s*)?(?-i:{zone})\b"
        if re.search(rf"\b(?:close|block|forbid)\b.{{0,20}}{zone_ref}", command, re.IGNORECASE) and re.search(
            rf"\bopen\b.{{0,20}}{zone_ref}|{zone_ref}.{{0,20}}\bopen\b", command, re.IGNORECASE
        ):
            negated = re.search(
                rf"\b(?:do\s+not|don't|not)\s+(?:close|block|forbid)\s+{zone_ref}",
                command,
                re.IGNORECASE,
            )
            temporal = "yesterday" in lower and any(term in lower for term in ("today", "now", "currently"))
            if not negated and not temporal and not any(cue in lower for cue in ("if ", "when ", "during ", "instead", "correction")):
                return True
    return False


def parse_rule_policy(command):
    destination = _destination(command)
    exclusions = _keepouts(command)
    via = _via_zones(command)
    contradiction = _has_direct_conflict(command, exclusions, via)
    if contradiction:
        return {
            "decision": "reject", "reason_code": "contradiction", "plan": [],
            "via_zones": [], "exclusions": [], "permanent_exclusions": [], "weights": [],
            "zone_preferences": [], "dynamic_object_rules": [], "conditional_rules": [],
            "path_id": None,
        }
    return {
        "decision": "accept",
        "reason_code": "none",
        "plan": [{"dest": destination}] if destination else [],
        "via_zones": via,
        "exclusions": [{"zone": zone} for zone in exclusions],
        "permanent_exclusions": [],
        "weights": _weights(command),
        "zone_preferences": _preferences(command),
        "dynamic_object_rules": _object_rules(command),
        "conditional_rules": [],
        "path_id": None,
    }
