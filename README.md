# Navigation Packages for DIT Robotics Eurobot 2026

This repository contains a customized navigation system based on the [ROS 2 Navigation Stack (Nav2)](https://github.com/ros-navigation/navigation2.git), adapted for DIT Robotics' Eurobot 2026 competition requirements.

## Features

### Basic Navigation
Enables smooth and efficient autonomous movement.
- Uses `/nav_to_pose` and `/nav_thru_poses` action servers for basic navigation.

### Docking Integration
Seamlessly integrates docking and navigation for autonomous charging or station return.
- Utilizes the `/dock_robot` action server to control the docking process.

### Multi-Functional Interfaces
Offers a variety of commands for enhanced control and flexibility:

- `/stopRobot`: Lock/unlock the robot.
- `/keepout_zone`: Dynamically set keepout zones to avoid certain areas.
- `/dock_robot`: Supports flexible keyword-based commands via the `dock_type` parameter.
- `rival_param.yaml`: Supports dynamic rival data setup adjustments.

#### Stop Robot

Control the robot's emergency stop or resume behavior via the `/stopRobot` topic.

- `true`: Immediately stops and locks the robot.  
- `false`: Unlocks and resumes normal operation.  
- **Message Type**: `std_msgs/msg/Bool`

#### Keepout Zone Index

The keepout zones correspond to specific regions on the Eurobot 2026 field, used to restrict robot access dynamically via `/keepout_zone`.

![Keepout Zones Index 1](/src/custom_layer/keepout_layer/pantry_keepout_zone_index.png)
![Keepout Zones Index 2](/src/custom_layer/keepout_layer/hazelnut_keepout_zone_index.png)

- Pantry zones are labeled **A** through **J** on the field map.  
- Hazelnut zones are labeled **K** through **R** on the field map.  
- These zones can be toggled at runtime using the `/keepout_zone` topic.  
- Suitable for strategic behaviors like avoiding opponent areas or obstacle fields.  
- **Message Type**: `std_msgs/msg/String`
- parameter settings: 
```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      keepout_layer:
        inflation_length: 0.15   # distance over which cost is inflated
        cost_scaling_factor: 5.0   # higher value -> steeper cost increase
        keepout_expand_mode: 1   # 0: Circle, 1: Square
```
see more about the params [/navigation2_run/params/nav2_params_default.yaml](https://github.com/DIT-ROBOTICS/Eurobot-2026-Navigation2/blob/develop/src/navigation2_run/params/nav2_params_default.yaml#L194)

#### Custom Controller: `follow_path_controller`

Main features:

- **Look-ahead path following**  
  Uses a configurable `look_ahead_distance` to pick a look-ahead target on the
  global path and generate smooth velocity commands.

- **Costmap-aware slowdown and replanning**  
  Checks cost values along the segment from current pose to a point ahead on the path.  
  If any cell exceeds `costmap_tolerance`:
  - Scales linear velocity by `speed_decade` (slowdown).  
  - Sets angular velocity to zero temporarily.  
  - Sets `update_plan_ = true` to request replanning.  
  - If the robot is almost stopped, throws `PlannerException("Obstacle detected")`
    so the BT can enter recovery.

- **Goal yaw alignment**  
  Tracks the final goal yaw (`final_goal_angle_`).  
  Uses `angular_kp` to scale the yaw error and clamps it with `max_angular_vel_`.  
  Uses `yaw_goal_tolerance_` to decide when the orientation is aligned.

- **Rival-aware slowdown (optional)**  
  Computes the distance between the robot and the rival.  
  When the distance is smaller than `rival_slowdown_radius_`, scales linear speed
  by `rival_decay_`.  
  Publishes the current distance on the `rival_distance` topic.

- **DelaySpin behavior**  
  When `controller_function` is set to `"DelaySpin"`, in-place spinning is
  disabled near the start of the path.  
  Spinning is only allowed after the robot moves beyond `spin_delay_threshold_`,
  then the mode automatically switches back to `"None"`.

- **Dynamic speed limiting**  
  Supports `setSpeedLimit()` to apply dynamic speed limits, either as a
  percentage of a base speed or as an absolute value.

#### Supported Keywords for `/dock_robot` API parameter `/dock_type`
(Keyword order does not matter and is designed for compatibility.)

- **Template Base**:  
  - `dock`: Triggers the docking process.

- **Functional Tags**:  
  - **Controller Type**: `fast`, `slow`, `linearBoost`, `angularBoost`  
  - **Goal Checker Type**: `precise`, `loose`  
  - **Offset Direction**: `x`, `y`, `z`  
  - **Docking Style**: `ordinary`, `gentle`, `rush`  
  - **Special Control**: `delaySpin`

#### Format for `rival_param.yaml`

```yaml
rival_parameters:
  rival_inscribed_radius: *data(double)*
```

---

## Environment Setup

### Quick Start

This repository includes a convenient `nav2.sh` script for managing Docker containers and development workflows.

#### Installation

1. Clone the repository:
```bash
git clone https://github.com/DIT-ROBOTICS/Eurobot-2026-Navigation2.git
cd Eurobot-2026-Navigation2
```

2. Install the `nav2` command globally:
```bash
./nav2.sh install
```

3. Add to your PATH (if not already added):
```bash
echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc
source ~/.bashrc
```

#### Available Commands

Once installed, you can use the following commands from anywhere:

**Build & Development**
- `nav2 rebuild` - Rebuild Docker images
- `nav2 build` - Build the entire ROS2 workspace
- `nav2 dev` - Enter interactive development shell

**Running Services**
- `nav2 run` - Launch Navigation2 stack (detached)
- `nav2 vnc` - Start VNC server with GUI support (port 5901)

**Container Management**
- `nav2 stop [service]` - Stop containers (all or specific: build, dev, run, vnc)
- `nav2 ps` - Show running containers status
- `nav2 clean` - Stop all and remove containers

**Monitoring & Debugging**
- `nav2 logs [service]` - View container logs (follow mode)
- `nav2 exec <service> <cmd>` - Execute command in running container
  - Example: `nav2 exec dev "ros2 topic list"`

**Help**
- `nav2 tools` - Show detailed command reference

#### Docker Services

- **build** - Compiles the ROS2 workspace using `colcon build`
- **dev** - Development environment for testing and debugging
- **run** - Runs the main navigation stack (`real_launch.py`)
- **vnc** - Provides GUI access via VNC (useful for rviz2)

#### Examples

```bash
# Build the workspace
nav2 build

# Enter development container
nav2 dev

# Inside the container, you can:
# - source install/local_setup.bash
# - ros2 launch navigation2_run real_launch.py
# - ros2 topic list
# - rviz2

# Run navigation (detached)
nav2 run

# Check logs
nav2 logs run

# Stop a specific service
nav2 stop run
```

---

## How to Use
For detailed environment setup instructions and Docker configurations, please refer to the [docker/](docker/) directory.

---

## Repository Structure
```
Eurobot-2026
└── Eurobot-2026-ws/
   └── src/
      ├── Eurobot-2026-Navigation2/         # Core navigation system code
         ├── custom_bts/                     # Custom behavior trees
         ├── custom_controller/              # Custom controller plugins
         ├── custom_layer/                   # Custom costmap layers
         ├── navigation2_run/                # Navigation system packages
         ├── Navigation2/                    # Modified version of Nav2
         └── opennav_docking/                # Docking server implementation
      └── Eurobot-2026-Navigation2-envs/    # Docker environments
         ├── Navigation2-humble-local/       # Local PC environment
         └── Navigation2-humble-deploy/      # Remote machine environment

```

---

## Contribution
Contributions are welcome! Please follow these steps:
1. Fork the repository.
2. Create a feature branch.
3. Commit your changes.
4. Submit a pull request.

---

## License
This project is licensed under the MIT License. See the `LICENSE` file for details.

Portions of this software are based on the ROS 2 Navigation Stack (Nav2), which is licensed under the Apache License 2.0. See the Navigation2 submodule for more details.

---

## Contact
For any issues or inquiries, please open an issue on GitHub or contact the DIT Robotics team.
