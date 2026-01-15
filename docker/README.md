# Eurobot-2026-Navigation2-envs
The Docker Environment of ROS2 Humble for Eurobot-2026-Navigation2

## One-Line Command To Run

On machine, run mode
```
# under Eurobot-2026-Navigation2/docker/deploy
docker compose -f /home/user/Eurobot-2026-Navigation2/docker/deploy/docker-compose.yaml run --rm navigation-run
```

On machine, develop mode
```
docker compose -f /home/user/Eurobot-2026-Navigation2/docker/deploy/docker-compose.yaml run --rm navigation-develop
```

On machine, build mode
```
docker compose -f /home/user/Eurobot-2026-Navigation2/docker/deploy/docker-compose.yaml run --rm navigation-build
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
docker pull justinshih0918/eurobot2026-nav2-envs:machine-amd64
```

## Build the image from Dockerfile (Under docker/deploy)
```
# Navigate to docker/deploy directory
cd docker/deploy

# Build the image
docker build -t justinshih0918/eurobot2026-nav2-envs:machine-amd64 .
```

## Start Container
```
docker compose -f /home/user/Eurobot-2026-Navigation2/docker/deploy/docker-compose.yaml up -d
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
## ------------ VNC Mode (for Remote Access / macOS / systems without native X11) ------------

The VNC mode provides a complete desktop environment with RViz and all GUI-based ROS2 tools.
Everything (VNC server + Navigation2 + XFCE desktop) runs in a single combined container,
eliminating X11 authentication issues.

**Use cases:**
- Remote development and visualization
- macOS or systems without native X11
- Running RViz on headless servers

---

### Start the Combined VNC + Navigation Container

On deploy (machine):
```bash
cd /home/user/Eurobot-2026-Navigation2/docker/deploy
docker compose up -d navigation-dev-vnc
```

On local:
```bash
cd /home/{user}/Eurobot-2026-Navigation2/docker/local
docker compose -f docker-compose.vnc.yaml up -d
```

### Connect to VNC Desktop

Use any VNC client to connect:

**VNC Connection:**
- **Address**: `<machine-ip>:5901` (black machine is 192.168.50.14)
- **Password**: `ros`

**Screen Lock (if appears):**
- **Username**: `user`
- **Password**: `user`

You will see an XFCE desktop with full ROS2 environment.

### Using Navigation2 with VNC

**Attach to Container**
```bash
docker exec -it navigation2-vnc bash
rviz2  # Will display in VNC
```

### Rebuild VNC Container (if needed)
```bash
cd /home/user/Eurobot-2026-Navigation2/docker/deploy
docker compose down navigation-dev-vnc
docker compose up -d --build navigation-dev-vnc
```
