# Camera Offset Publisher

A simulated camera-based dock detection node for testing the docking server's external detection feature.

## Overview

This package publishes `geometry_msgs/msg/PoseStamped` messages on the `/detected_dock_pose` topic to simulate camera detection of dock positions. It calculates the dock position relative to the robot by:
1. Subscribing to `/final_pose_nav` to get the current robot position
2. Computing the offset from robot to a fixed goal position
3. Publishing this offset in the `base_footprint` frame as the detected dock pose

## Building

```bash
cd /home/kesler/eu26/Eurobot-2026-Navigation2
colcon build --packages-select camera_offset_publisher
source install/setup.bash
```

## Usage

### Basic Launch

```bash
ros2 launch camera_offset_publisher camera_offset_publisher.launch.py
```

### Custom Goal Position

```bash
ros2 launch camera_offset_publisher camera_offset_publisher.launch.py \
  goal_x:=2.5 \
  goal_y:=1.0
```

### Custom Publishing Rate

```bash
ros2 launch camera_offset_publisher camera_offset_publisher.launch.py \
  publish_rate:=50.0
```

## Run Directly

```bash
ros2 run camera_offset_publisher camera_offset_publisher_node \
  --ros-args \
  -p goal_x:=1.5 \
  -p goal_y:=0.8 \
  -p publish_rate:=100.0
```

## Monitoring

Check published messages:
```bash
ros2 topic echo /detected_dock_pose
```

Check publishing rate:
```bash
ros2 topic hz /detected_dock_pose
```

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `goal_x` | double | 1.5 | Target dock X position in map frame (meters) |
| `goal_y` | double | 1.0 | Target dock Y position in map frame (meters) |
| `publish_rate` | double | 100.0 | Publishing frequency in Hz |

## Integration with Docking Server

The docking server subscribes to `/detected_dock_pose` and uses the published values for external dock detection. This node:
1. Reads the robot's current position from `/final_pose_nav` (Odometry)
2. Calculates the relative position from robot to goal: `(goal_x - robot_x, goal_y - robot_y)`
3. Publishes this as a PoseStamped in `base_footprint` frame

The published PoseStamped message contains:
- `header.frame_id`: `base_footprint` (dock position relative to robot)
- `pose.position.x/y`: Offset from robot to dock in meters
- `pose.orientation`: Fixed to identity quaternion (w=1.0)

### Quick Start for Camera-Based Docking

**0. Start the docking server (if not already running):**
```bash
# Terminal 1: Start docking server
ros2 launch camera_offset_publisher start_docking_server.launch.py

# Terminal 2: Configure and activate the lifecycle node
ros2 lifecycle set /docking_server configure
ros2 lifecycle set /docking_server activate
```

**1. Start the test launch (camera publisher + set controller type):**
```bash
ros2 launch camera_offset_publisher test_camera_docking.launch.py \
  goal_x:=1.5 \
  goal_y:=0.8
```

**2. Send a docking goal:**

Using dock ID from database:
```bash
ros2 run camera_offset_publisher send_dock_goal.py --dock_id my_dock
```

Or using a pose directly:
```bash
ros2 run camera_offset_publisher send_dock_goal.py \
  --pose 2.0 1.0 0.0 \
  --dock_type simple_charging_dock
```

### Manual Setup

If you prefer manual setup:

**1. Start camera offset publisher:**
```bash
ros2 launch camera_offset_publisher camera_offset_publisher.launch.py
```

**2. Enable camera mode:**
```bash
ros2 topic pub /dock_controller_type std_msgs/msg/String "data: 'Cam'" --once
```

**3. Send docking action via terminal:**
```bash
# Using dock ID
ros2 action send_goal /dock_robot opennav_docking_msgs/action/DockRobot \
  "{use_dock_id: true, dock_id: 'my_dock', navigate_to_staging_pose: true, max_staging_time: 30.0}"

# Using pose
ros2 action send_goal /dock_robot opennav_docking_msgs/action/DockRobot \
  "{use_dock_id: false, dock_type: 'simple_charging_dock', \
   dock_pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}, \
   orientation: {w: 1.0}}}, navigate_to_staging_pose: true, max_staging_time: 30.0}"
```

### Helper Script Options

The `send_dock_goal.py` script provides easier goal submission:

```bash
# Show all options
ros2 run camera_offset_publisher send_dock_goal.py --help

# Dock with specific type
ros2 run camera_offset_publisher send_dock_goal.py \
  --pose 1.5 0.8 0.0 \
  --dock_type my_custom_dock

# Skip navigation to staging pose
ros2 run camera_offset_publisher send_dock_goal.py \
  --dock_id my_dock \
  --no-navigate

# Set custom staging timeout
ros2 run camera_offset_publisher send_dock_goal.py \
  --dock_id my_dock \
  --max-staging-time 60.0
```

### Monitoring

**Check detected dock pose being published:**
```bash
ros2 topic echo /detected_dock_pose
```

**Check controller mode:**
```bash
ros2 topic echo /dock_controller_type
```

**Monitor docking action feedback:**
```bash
ros2 action info /dock_robot
```

**View docking server logs:**
Look for yellow `[1;33m` colored messages indicating camera mode activation and offset values.

### Example Camera Docking Command

```bash
# Start the camera offset publisher with goal at (1.0, 0.8)
ros2 launch camera_offset_publisher test_camera_docking.launch.py goal_x:=1.0 goal_y:=0.8

# Send docking goal with camera detection
ros2 action send_goal /dock_robot opennav_docking_msgs/action/DockRobot "{
  dock_type: 'dock',
  use_dock_id: false,
  navigate_to_staging_pose: true,
  max_staging_time: 1000.0, 
  dock_pose: {
    header: {
      frame_id: 'map'
    },
    pose: {
      position: {x: 1.0, y: 0.8, z: 0.0}, 
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    }
  }
}"
```