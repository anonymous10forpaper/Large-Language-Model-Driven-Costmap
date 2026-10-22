# Large-Language-Model-Driven Costmap

Natural-language navigation for ROS 2 and Nav2. The framework translates English instructions into structured navigation policies and applies them through custom costmap layers, allowing operators to specify destinations, restricted areas, zone costs, and person-clearance requirements.

[Project Page](https://anonymous10forpaper.github.io/Large-Language-Model-Driven-Costmap_Page/) · [Benchmark](https://anonymous10forpaper.github.io/Large-Language-Model-Driven-Costmap_Page/benchmark.html) · [User Study](https://anonymous10forpaper.github.io/Large-Language-Model-Driven-Costmap_Page/user-study.html)

## Overview

- **Natural-language policies:** Convert operator instructions into navigation policies using a local LLM, with validation before execution.
- **Costmap integration:** Apply forbidden zones, zone-specific soft costs (0–252), and person-clearance rules to Nav2.
- **Interactive operation:** Define waypoints and zones on a map, submit missions, and manage navigation through a graphical interface.
- **Multi-robot support:** Assign missions to individual robots and share navigation policies across namespaced Nav2 systems.

## Packages

| Package | Purpose |
| --- | --- |
| [`policy_bridge`](policy_bridge/) | Natural-language interpretation, policy validation, mission management, and the operator GUI. |
| [`my_costmap_layers`](my_costmap_layers/) | Nav2 plugins for forbidden zones, soft costs, and object-aware clearance. |

The paper's [rule-based comparison method](policy_bridge/evaluation/) is also provided, with a standalone usage example.

## Getting Started

The reference platform uses **Ubuntu 22.04, ROS 2 Humble, Nav2, and TurtleBot3**, with **Ollama** for local LLM inference.

### 1. Build the packages

In a ROS 2 workspace:

```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws/src
git clone https://github.com/anonymous10forpaper/Large-Language-Model-Driven-Costmap.git
cd ~/turtlebot3_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select my_costmap_layers policy_bridge
source install/setup.bash
```

### 2. Prepare a local model

Install Ollama and start its service, then download a model, for example:

```bash
ollama pull qwen3:8b
```

If the service is not running, start `ollama serve` in a separate terminal. The GUI lists the models available on the connected server.

### 3. Configure Nav2

Add the policy layers to the global costmap, before the inflation layer, and restart Nav2.

<details>
<summary>Global costmap configuration example</summary>

Merge these entries into the robot's existing Nav2 configuration.

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      plugins:
        - static_layer
        - obstacle_layer
        - keepout_command_layer
        - zone_softcost_layer
        - object_avoidance_layer
        - inflation_layer

      keepout_command_layer:
        plugin: "my_costmap_layers::KeepoutCommandLayer"
        enabled: true
        forbidden_zones_topic: "/forbidden_zones_update"
        zone_geometry_topic: "/zone_geometry_update"

      zone_softcost_layer:
        plugin: "my_costmap_layers::ZoneSoftCostLayer"
        enabled: true
        zone_cost_overrides_topic: "/zone_cost_overrides"
        zone_geometry_topic: "/zone_geometry_update"

      object_avoidance_layer:
        plugin: "my_costmap_layers::ObjectAvoidanceLayer"
        enabled: true
        object_positions_topic: "/object_world_positions"
        object_avoidance_radius_topic: "/object_avoidance_radius"
        avoidance_radius: 1.5
```

</details>

### 4. Launch the interface

With the robot's Nav2 stack and Ollama running:

```bash
source ~/turtlebot3_ws/install/setup.bash
ros2 run policy_bridge policy_bridge_gui
```

1. Open a ROS map YAML through **Map / Setup**.
2. Define and save the waypoints and policy zones.
3. Select a model and enter a navigation instruction using the saved labels.

For example, with zone `A` and waypoint `WP2` defined:

> Forbid zone A and go to WP2.

Scheduled commands use UTC by default; set the `timezone` ROS parameter to use a different time zone.

## Optional Setup

<details>
<summary>Camera-based person detection</summary>

Install the additional dependencies when using camera-based person detection:

```bash
python3 -m pip install torch ultralytics
```

</details>

<details>
<summary>Custom Ollama endpoint</summary>

The default endpoint is `http://localhost:11434/api/generate`. To connect to another server:

```bash
ros2 run policy_bridge policy_bridge_gui --ros-args \
  -p gui_llm_endpoint:="http://<server-address>:11434/api/generate"
```

</details>

<details>
<summary>Multi-robot configuration</summary>

The GUI discovers namespaced Nav2 actions, such as `/tb1/follow_waypoints`, and provides a mission panel for each robot. For shared policies, use the following topics in each robot's global costmap:

| Layer parameter | Topic |
| --- | --- |
| `forbidden_zones_topic` | `/fleet/forbidden_zones_update` |
| `zone_cost_overrides_topic` | `/fleet/zone_cost_overrides` |
| `object_positions_topic` | `/fleet/object_world_positions` |
| `object_avoidance_radius_topic` | `/fleet/object_avoidance_radius` |
| `zone_geometry_topic` | `/zone_geometry_update` |

Navigation goals remain robot-specific.

</details>

## License

This runtime code accompanies an anonymous manuscript submission and is provided for academic review and reproducibility. All rights are reserved by the anonymous authors.
