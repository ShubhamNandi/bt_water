# BT Water Package

ROS 2 package for water dripping and draining cycle control using Behavior Trees.

## Overview

This package implements a Behavior Tree that executes a single water cycle on launch:
1. Start water dripping (FESTO IO)
2. Wait for water level to reach threshold (sensor)
3. Stop water dripping (FESTO IO)
4. Start water draining (FESTO IO)
5. Wait for water level to drop below threshold (sensor)
6. Stop water draining (FESTO IO)

The BT runs once per launch and completes. Restart the node to run another cycle.

## Quick Start

### Build
```bash
( go inside the workspace and build the package)
colcon build --packages-select bt_water
source install/setup.bash
```

### Run
```bash
ros2 launch bt_water bt_water.launch.py
```

## Configuration

Edit `config/config.yaml` to adjust:
- `tick_rate`: Behavior tree tick frequency (Hz)
- `tree_file`: Path to BT XML file

## Behavior Tree

The behavior tree is defined in `trees/water_cycle.xml`. It uses a simple `Sequence` node that executes all 6 action nodes in order.

## Action Nodes (Placeholders)

All action nodes are currently placeholders using `AlwaysSuccess`. Replace them with real implementations:

- `StartDrip`: Activate FESTO IO drip valve
- `WaitDripComplete`: Monitor water level sensor
- `StopDrip`: Deactivate FESTO IO drip valve
- `StartDrain`: Activate FESTO IO drain valve
- `WaitDrainComplete`: Monitor water level sensor
- `StopDrain`: Deactivate FESTO IO drain valve

## Integration

See `IMPLEMENTATION.md` for detailed integration examples for FESTO IO and water level sensors.

## Dependencies

- `rclcpp`: ROS 2 C++ client library
- `behaviortree_cpp`: BehaviorTree.CPP library
- `ament_index_cpp`: Package resource lookup

