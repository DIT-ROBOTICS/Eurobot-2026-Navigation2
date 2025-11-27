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
