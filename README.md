# LLM-Driven Costmap Framework for Industrial AMRs

## Overview

This module is designed for use with ROS 2 Humble and the Navigation2 stack.

It has been tested in multi-AMR environments using ROS 2 Humble on Ubuntu 22.04 with standard Nav2 navigation behavior.

This framework converts natural-language commands into navigation policies and projects them onto Nav2 costmaps via custom layers. High-level policies can be shared across a fleet, while Nav2 planners handle path computation and execution.

## 📦 Packages

This repository provides two ROS 2 packages for natural-language-driven navigation policies:

- **policy_bridge**: A runtime policy conversion module that processes natural language commands and publishes costmap updates.
- **my_costmap_layers**: Nav2 plugin layers that integrate policy-derived costs into the costmap.

## Package 1: policy_bridge

### Overview

`policy_bridge` implements a policy bridge node that converts natural-language commands into structured navigation policies.

### Features

- Natural language command processing via LLM
- Policy validation and merging
- Spatiotemporal constraint handling
- Semantic object detection and localization
- Multi-waypoint navigation
- Multi-robot fleet support with shared policy topics

### Nodes

- `policybridge`: Single-robot policy bridge node
- `policybridge_multi`: Multi-robot fleet policy bridge node
- `nl_command_sender`: Command sender for natural language mission commands
- `event_sender`: Event sender for state events (fire alarm, battery, etc.)

### Build Instructions

```bash
cd ~/your_ros2_ws/src
git clone <this_repository>
cd ~/your_ros2_ws
colcon build --packages-select policy_bridge
source install/setup.bash
```

### Run Instructions

**Single Robot:**

```bash
ros2 run policy_bridge policy_bridge
```

**Multi-Robot Fleet:**

```bash
ros2 run policy_bridge policy_bridge_multi
```

**Command Sender:**

```bash
ros2 run policy_bridge nl_command_sender
```

**Event Sender:**

```bash
ros2 run policy_bridge event_sender
```

## Package 2: my_costmap_layers

### Overview

`my_costmap_layers` implements three custom Nav2 costmap layers that integrate policy-derived costs with sensor-based costs. The layers subscribe to policy topics and update the costmap accordingly.

### Features

- Implements `nav2_costmap_2d::Layer` interface
- Subscribes to policy topics (`/forbidden_zones_update`, `/zone_cost_overrides`, `/object_world_positions`)
- Applies region-specific cost updates efficiently
- Fully compatible with pluginlib

### Layers

1. **KeepoutCommandLayer**: Assigns lethal cost (254) to forbidden zones
2. **ZoneSoftCostLayer**: Applies zone-specific differential costs (soft preferences)
3. **ObjectAvoidanceLayer**: Generates disc-shaped costs around detected objects

### Build Instructions

```bash
cd ~/your_ros2_ws/src
git clone <this_repository>
cd ~/your_ros2_ws
colcon build --packages-select my_costmap_layers
source install/setup.bash
```

### Plugin Configuration

Add the following to your `nav2_params.yaml`:

```yaml
global_costmap:
  ros__parameters:
    plugins: ["static_layer", "obstacle_layer", "voxel_layer", "keepout_command_layer", "zone_softcost_layer", "object_avoidance_layer", "inflation_layer"]
    keepout_command_layer:
      plugin: "my_costmap_layers::KeepoutCommandLayer"
      enabled: True
      forbidden_zones_topic: "/forbidden_zones_update"        # multi-robot: use "/fleet/forbidden_zones_update"
    object_avoidance_layer:
      plugin: "my_costmap_layers::ObjectAvoidanceLayer"
      enabled: True
      object_positions_topic: "/object_world_positions"       # multi-robot: use "/fleet/object_world_positions"
      avoidance_radius: 2.0
      hold_after_clear_s: 0.6
    zone_softcost_layer:
      plugin: "my_costmap_layers::ZoneSoftCostLayer"
      enabled: True
      zone_cost_overrides_topic: "/zone_cost_overrides"       # multi-robot: use "/fleet/zone_cost_overrides"
```

## 📡 ROS Topics

### Published Topics

- `/forbidden_zones_update` (std_msgs/String): JSON-encoded forbidden zone list
- `/zone_cost_overrides` (std_msgs/String): JSON-encoded zone cost overrides
- `/object_world_positions` (geometry_msgs/PoseArray): Detected object poses in world frame
- `/fleet/forbidden_zones_update` (multi-robot): Fleet-wide forbidden zones
- `/fleet/zone_cost_overrides` (multi-robot): Fleet-wide cost overrides
- `/fleet/object_world_positions_json` (multi-robot): Fleet-wide object positions (JSON)

### Subscribed Topics

- `/nl_command` (std_msgs/String): Natural language commands
- `/camera/image_raw` (sensor_msgs/Image): RGB camera stream (for YOLO)
- `/camera/depth/image_raw` (sensor_msgs/Image): Depth image (for 3D object localization)
- `/camera/camera_info` (sensor_msgs/CameraInfo): Camera intrinsics

## Summary

This system enables adaptive costmap updates in response to natural-language policy commands, supporting keepout zones, soft costs, temporal conditions, dynamic object avoidance, and multi-robot operation.

## Folder Structure

```
.
├── policy_bridge/
│   ├── policy_bridge/
│   │   ├── __init__.py
│   │   ├── policybridge.py
│   │   ├── policybridge_multi.py
│   │   ├── nl_command_sender.py
│   │   └── event_sender.py
│   ├── resource/
│   │   └── policy_bridge
│   ├── setup.py
│   ├── setup.cfg
│   └── package.xml
├── my_costmap_layers/
│   ├── src/
│   │   ├── KeepoutCommandLayer.cpp
│   │   ├── ObjectAvoidanceLayer.cpp
│   │   └── ZoneSoftCostLayer.cpp
│   ├── include/
│   │   └── my_costmap_layers/
│   │       ├── keepout_command_layer.hpp
│   │       ├── object_avoidance_layer.hpp
│   │       └── zone_soft_cost_layer.hpp
│   ├── plugins/
│   │   └── costmap_plugins.xml
│   ├── CMakeLists.txt
│   └── package.xml
```

## 🔗 Dependencies

### LLM Backend

- HTTP-accessible LLM endpoint compatible with the Ollama-style generate API
- Example (Ollama on Ubuntu):
  - Install Ollama: see `https://ollama.com`
  - Pull model: `ollama pull llama3.1`
  - Ensure the service is listening on `http://localhost:11434` (default)
- Configure `llm_endpoint` and `llm_model` parameters in your launch file

### Python Packages (policy_bridge)

- `torch`
- `ultralytics` (YOLO)
- `opencv-python` (for debugging and image handling)

Install example:

```bash
pip install torch ultralytics opencv-python
```

C++/ROS dependencies for `my_costmap_layers` follow the standard Nav2 costmap plugin requirements (see `package.xml` and `CMakeLists.txt`).

## 🔧 Troubleshooting

### LLM Not Responding

- Check `llm_endpoint` parameter and network connectivity
- Verify LLM service is running (e.g., Ollama)

### Zones Not Appearing on Costmap

- Verify zone database JSON contains correct coordinates
- Check that layer plugins are registered in costmap configuration
- Ensure topics are being published (use `ros2 topic echo`)

### Objects Not Detected

- Verify YOLO model file exists and is accessible
- Check that camera topics are correctly mapped to your actual RGB/Depth sensors
- Ensure `enable_yolo` parameter is `true`
- Note: the default topic names in this repository assume the Gazebo TurtleBot3 Waffle model with a RealSense D435; you may need to change them for your own camera setup.

### Multi-Robot Policies Not Syncing

- Verify all robots are on the same ROS 2 network


## 📄 License

This code is made available for academic purposes accompanying a manuscript submission to Robotics and Autonomous Systems.

Note: Unauthorized reproduction, distribution, or modification of this code is strictly prohibited.

© Anonymous Authors. All rights reserved.

## 📧 Contact

If you have questions regarding the paper or this framework, please refer to the official Robotics and Autonomous Systems submission.
