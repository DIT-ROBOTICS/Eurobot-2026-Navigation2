readme要怎麼換行

# Eurobot-2025-Navigation2-envs
The Docker Environment of ROS2 Humble for Eurobot-2025-Navigation2

## One-Line Command To Run

On machine, run mode
```
docker compose -f /home/navigation/Eurobot-2025-machine-ws/src/Eurobot-2025-Navigation2-envs/Navigation2-humble-deploy/docker-compose.yaml run --rm navigation-run
```

On machine, develop mode
```
docker compose -f /home/navigation/Eurobot-2025-machine-ws/src/Eurobot-2025-Navigation2-envs/Navigation2-humble-deploy/docker-compose.yaml run --rm navigation-develop
```

On machine, build mode
```
docker compose -f /home/navigation/Eurobot-2025-machine-ws/src/Eurobot-2025-Navigation2-envs/Navigation2-humble-deploy/docker-compose.yaml run --rm navigation-build
```

On Local, rviz mode for machine-11
```
docker compose -f /home/{user}/Eurobot-2026-Navigation2/docker/local/docker-bringup.yaml run --rm navigation-rviz-local-11
```
On Local, rviz mode for machine-14
```
docker compose -f /home/{user}/Eurobot-2026-Navigation2/docker/local/docker-bringup.yaml run --rm navigation-rviz-local-14
```
On local, develop mode
```
docker compose -f /home/{user}/Eurobot-2026-Navigation2/docker/local/docker-bringup.yaml run --rm navigation-develop-local
```
On local, build mode
```
docker compose -f /home/{user}/Eurobot-2026-Navigation2/docker/local/docker-bringup.yaml run --rm navigation-build-local
```
On local, run mode
```
docker compose -f /home/{user}/Eurobot-2026-Navigation2/docker/local/docker-bringup.yaml run --rm navigation-run-local
```

On local, using vnc
```
docker volume create ros_x11

# start vnc
docker compose -f /home/{user}/Eurobot-2026-Navigation2/docker/vnc/docker-compose.yaml up -d

# start navigation
docker compose -f /home/{user}/Eurobot-2026-Navigation2/docker/local/docker-compose.vnc.yaml up -d
```
## ------------ Basic commands ------------

## Pull docker image for container
```
docker pull justinshih0918/eurobot2026-nav2-envs
```

## Build the image from Dockerfile (Under Navigation2-humble-local)
```
docker compose build
```

## Start Container
```
docker compose -f /home/{user}/Eurobot-2026-Navigation2/docker/local/docker-compose.yaml up -d
```

## Attach Container
```
docker exec -it navigation2 bash
```

## Launching Navigation2
### Simulation On Local Machine - open rviz & navigation with odometry simulation
```
ros2 launch navigation2_run sim_launch.py
```

#### you can use shorten command as well
```
build # colcon build with symlink and parallel-workers 4
```

```
sim # sim_launch.py activation
```
### Run On Real Machine - open navigation with listening to the topic /final_pose from localization
```
# on remote machine
ros2 launch navigation2_run real_launch.py 

# on local machine
ros2 launch navigation2_run rviz_launch.py
```
## ------------ VNC Mode (for macOS / systems without native X11) ------------

The VNC mode allows running RViz and all GUI-based ROS2 tools even on systems
that do not have an X11 server (e.g., macOS).  
The computation runs inside the Navigation2 container, while rendering is done
by the VNC/XFCE container.

---
### Create shared volume
```
docker volume create ros_x11
```

### Activate VNC + XFCE (`ros2-vnc` container)
```
cd /home/{user}/Eurobot-2026-Navigation2/docker/vnc
docker compose up -d
```
You an now connect via any VNC client:

Address: localhost:5901

Password: ros

You will see an XFCE desktop with ROS environment already sourced.

### Start Navigation2 (GUI output displayed via VNC)

Rebuild container
```
cd /home/{user}/Eurobot-2026-Navigation2/docker/local
docker compose -f docker-compose.vnc.yaml up -d
```

Attach to the Navigation2 container:
```
docker exec -it navigation2 bash
```

Launch simulation + RViz (RViz will appear on the VNC desktop):
```
sim
# or:
# ros2 launch navigation2_run sim_launch.py
```
