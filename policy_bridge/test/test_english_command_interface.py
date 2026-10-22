import datetime as dt
from types import SimpleNamespace
from unittest.mock import Mock
from zoneinfo import ZoneInfo

import pytest

from policy_bridge.policybridge import PolicyBridgeNode, WEIGHT_CMD_RE, WEIGHT_CMD_RE2
from policy_bridge.policybridge_gui import GuiPolicyBridgeNode


@pytest.fixture
def node():
    instance = object.__new__(PolicyBridgeNode)
    instance._labels = list("ABCDE")
    instance._waypoints = {"WP2": object()}
    instance._tz = ZoneInfo("UTC")
    instance._env_state = {"fire_alarm": False, "battery_pct": 100}
    instance._last_base_exclusions = []
    instance._cond_rules_store = []
    instance._ui_print = Mock()
    instance._print_runtime_snapshot = Mock()
    instance._recalculate_and_publish_dynamic_keepouts = Mock()
    return instance


@pytest.mark.parametrize("percentage,expected", [(-5, 0), (0, 0), (15, 15), (20.9, 20), (100, 100), (105, 100)])
def test_gui_battery_message_updates_state(node, percentage, expected):
    messages = []
    gui = SimpleNamespace(_gui_event_pub=SimpleNamespace(publish=messages.append))

    GuiPolicyBridgeNode.set_battery_from_gui(gui, percentage)

    assert messages[0].data == f"battery {expected}%"
    assert node._handle_state_command(messages[0].data)
    assert node._env_state["battery_pct"] == expected
    assert ("low_battery" in node._active_conditions()) == (expected <= 20)
    node._recalculate_and_publish_dynamic_keepouts.assert_called_once()


@pytest.mark.parametrize("enabled", [True, False])
def test_gui_fire_message_updates_state(node, enabled):
    messages = []
    gui = SimpleNamespace(_gui_event_pub=SimpleNamespace(publish=messages.append))
    node._env_state["fire_alarm"] = not enabled

    GuiPolicyBridgeNode.set_fire_alarm_from_gui(gui, enabled)

    assert node._handle_state_command(messages[0].data)
    assert node._env_state["fire_alarm"] is enabled


@pytest.mark.parametrize("command,expected", [
    ("Close A for 15 minutes.", 15),
    ("Close A for half an hour.", 30),
    ("Close A for thirty minutes.", 30),
])
def test_english_duration(node, command, expected):
    assert node._parse_duration_minutes(command) == expected


@pytest.mark.parametrize("command,hour,minute", [
    ("9:30 am today", 9, 30),
    ("2 pm today", 14, 0),
    ("today at 14:30", 14, 30),
])
def test_english_start_time(node, command, hour, minute):
    start = node._parse_start_time(command)
    assert (start.hour, start.minute) == (hour, minute)
    assert start.utcoffset() == dt.timedelta(0)


def test_english_aisle_aliases(node):
    assert node._apply_aisle_aliases("first aisle, second aisle, 5th aisle") == "A, B, E"


@pytest.mark.parametrize("command", ["Set costs A to 5 and B to 200.", "A cost 5, B cost 200."])
def test_english_soft_costs(node, command):
    keepouts, costs = node._split_intents_deterministic(command)
    assert keepouts == set()
    assert costs == {"A": 5, "B": 200}


def test_english_cost_patterns():
    assert WEIGHT_CMD_RE.search("A around 3 m cost 20").groups() == ("A", "3", "20")
    assert WEIGHT_CMD_RE2.search("B cost 200").groups() == ("B", "200")


def test_english_person_clearance_fallback(node):
    policy = node._fallback_parse("Avoid people, then go to WP2.")
    assert policy["plan"] == [{"dest": "WP2"}]
    assert policy["dynamic_object_rules"] == [{"class": "person", "radius": 1.5}]


def test_english_mission_replacement():
    assert PolicyBridgeNode._requests_mission_replacement("Cancel the current mission and go to WP2.")
    assert not PolicyBridgeNode._requests_mission_replacement("Go to WP2.")
