# Multi-User Development Guide

## Overview

This setup enables multiple DIT-ROBOTICS team members to work simultaneously on the same robot without permission conflicts or container naming collisions.

## Key Features

✅ **Dynamic User ID Matching** - Container user automatically matches your host user  
✅ **No Permission Issues** - Files created in containers have correct ownership  
✅ **Unique Container Names** - Each team member gets their own container instance  
✅ **Optional VNC Support** - Visual desktop environment when needed  
✅ **Zero Configuration** - Works automatically for all team members  

---

## Quick Start

### 1. Set Your User ID (One-time Setup)

```bash
# Add to your ~/.bashrc
echo 'export USER_UID=$(id -u)' >> ~/.bashrc
source ~/.bashrc
```

### 2. Choose Your Service

```bash
cd docker/deploy

# For development (interactive shell)
docker compose up navigation-develop

# For building the workspace
docker compose up navigation-build

# For running navigation stack
docker compose up navigation-run

# For development with VNC desktop
docker compose up navigation-dev-vnc
```

### 3. Attach to Running Container

```bash
# Each team member gets a unique container
# This automatically runs as the 'user' account
docker exec -it navigation2-dev-$USER bash
```

---

## How It Works

### Dynamic Permission Matching

The `entrypoint.sh` script automatically:
1. Detects your host user ID from the `USERID` environment variable
2. Adjusts the container's internal user to match
3. Fixes file ownership to prevent permission errors

**Example:**
- User `alice` (UID 1001) creates files → owned by 1001
- User `bob` (UID 1002) creates files → owned by 1002
- Both can work on the same mounted workspace without conflicts!

### Unique Container Names

Container names include `$USER` to avoid collisions:
```
navigation2-dev-alice    ← Alice's container
navigation2-dev-bob      ← Bob's container
navigation2-vnc-charlie  ← Charlie's VNC container
```

---

## Service Descriptions

### `navigation-develop`
Interactive development environment with bash shell.

**Use when:** Writing code, debugging, testing

```bash
docker compose up navigation-develop
docker exec -it navigation2-dev-$USER bash
```

### `navigation-build`
Builds the entire ROS 2 workspace once and exits.

**Use when:** Building packages after changes

```bash
docker compose up navigation-build
```

### `navigation-run`
Launches the navigation stack and logs output.

**Use when:** Running the robot in production

```bash
docker compose up navigation-run
```

### `navigation-dev-vnc`
Full XFCE desktop with VNC access for graphical tools.

**Use when:** Running RViz, rqt, or other GUI applications

```bash
# Default VNC port 5901
docker compose up navigation-dev-vnc

# Custom port for multiple users
VNC_PORT=5902 docker compose up navigation-dev-vnc
```

**Connect via VNC:**
```
vncviewer <robot-ip>:5901
Password: ros
```

---

## Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `USER_UID` | `1000` | Your host user ID (auto-detected) |
| `ROS_DOMAIN_ID` | - | ROS 2 domain for network isolation |
| `VNC_PORT` | `5901` | VNC server port (change if conflicts) |
| `VNC_PASSWORD` | `ros` | VNC access password |

---

## Multi-User Scenarios

### Scenario 1: Same Robot, Different Containers
```bash
# User alice starts development
alice@robot:~$ docker compose up navigation-develop
alice@robot:~$ docker exec -it navigation2-dev-alice bash

# User bob starts VNC session (different terminal/tmux)
bob@robot:~$ VNC_PORT=5902 docker compose up navigation-dev-vnc
# Bob connects via VNC to port 5902
```

### Scenario 2: Building vs Running
```bash
# Alice builds the workspace
alice@robot:~$ docker compose up navigation-build

# Bob runs the navigation stack
bob@robot:~$ docker compose up navigation-run
```

### Scenario 3: Avoiding Port Conflicts
```bash
# Multiple VNC users need different ports
USER_UID=1001 VNC_PORT=5901 docker compose up navigation-dev-vnc  # alice
USER_UID=1002 VNC_PORT=5902 docker compose up navigation-dev-vnc  # bob
USER_UID=1003 VNC_PORT=5903 docker compose up navigation-dev-vnc  # charlie
```

---

## Troubleshooting

### Permission Denied Errors

**Problem:** Can't write to workspace files  
**Solution:** Check `USER_UID` is set correctly
```bash
echo $USER_UID
# Should match: id -u
```

### Container Name Conflicts

**Problem:** "container name already in use"  
**Solution:** Ensure `$USER` is set or use unique names
```bash
# Check current user
echo $USER

# Or manually override
USER=myname docker compose up navigation-develop
```

### VNC Port Already in Use

**Problem:** "bind: address already in use"  
**Solution:** Choose a different port
```bash
VNC_PORT=5902 docker compose up navigation-dev-vnc
```

### Files Owned by Wrong User

**Problem:** Files created before multi-user setup have wrong ownership  
**Solution:** Fix ownership on host
```bash
sudo chown -R $USER:$USER /path/to/workspace
```

---

## Architecture

### Scripts

| Script | Purpose |
|--------|---------|
| `scripts/entrypoint.sh` | Dynamic user ID matching |
| `scripts/install_vnc.sh` | VNC installation (optional) |
| `scripts/start-vnc.sh` | VNC server startup |

### Dockerfiles

| File | Purpose |
|------|---------|
| `Dockerfile` | Base navigation environment |
| `vnc/Dockerfile.navigation-vnc` | Navigation + VNC desktop |

### Build Arguments

Both Dockerfiles support:
- `USERID` - Container user ID
- `USERGID` - Container group ID
- `USERNAME` - Container username (default: `user`)

---

## Best Practices

1. **Always set `USER_UID`** in your shell profile
2. **Use tmux/screen** for persistent sessions on shared robots
3. **Coordinate VNC ports** among team members (5901, 5902, 5903, ...)
4. **Stop containers** when done to free resources
   ```bash
   docker compose down
   ```
5. **Clean up old containers** periodically
   ```bash
   docker container prune
   ```

---

## Examples

### Full Development Workflow
```bash
# Terminal 1: Development container
docker compose up navigation-develop
docker exec -it navigation2-dev-$USER bash
# Inside container: edit code, build, test

# Terminal 2: VNC for visualization
VNC_PORT=5901 docker compose up navigation-dev-vnc
# Connect via VNC, launch RViz

# Terminal 3: Run navigation
docker compose up navigation-run
# Watch logs and navigation behavior
```

### Quick Build and Test
```bash
# Build
docker compose up navigation-build

# Test run
docker compose up navigation-develop
docker exec -it navigation2-dev-$USER bash
user@container:~$ cd Eurobot-2026-Navigation2-ws
user@container:~/ws$ source install/local_setup.bash
user@container:~/ws$ ros2 launch navigation2_run real_launch.py
```

---

## FAQ

**Q: Do I need to rebuild the image for each user?**  
A: No, one image works for all users. The entrypoint handles user ID dynamically.

**Q: Can multiple users access the same container?**  
A: Yes, but they'll share the same environment. Better to use separate containers.

**Q: What if I don't set `USER_UID`?**  
A: It defaults to 1000, which might cause permission issues if your host UID differs.

**Q: How do I enable VNC in the base Dockerfile?**  
A: VNC is separate. Use `navigation-dev-vnc` service or set `ENABLE_VNC=true` build arg (experimental).

**Q: Can I use this on different robots?**  
A: Yes, each robot can have multiple users with this same setup.

---

## Support

For issues or questions:
1. Check container logs: `docker logs navigation2-dev-$USER`
2. Verify environment: `env | grep -E 'USER|UID|ROS'`
3. Contact: DIT-ROBOTICS team leads

---

**Last Updated:** January 15, 2026  
**Maintained By:** DIT-ROBOTICS Navigation Team
