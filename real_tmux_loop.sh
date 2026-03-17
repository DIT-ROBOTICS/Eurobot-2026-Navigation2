#!/usr/bin/env bash

set -euo pipefail

SCRIPT_SOURCE="${BASH_SOURCE[0]}"
while [ -h "$SCRIPT_SOURCE" ]; do
  SCRIPT_DIR="$(cd -P "$(dirname "$SCRIPT_SOURCE")" && pwd)"
  SCRIPT_SOURCE="$(readlink "$SCRIPT_SOURCE")"
  [[ $SCRIPT_SOURCE != /* ]] && SCRIPT_SOURCE="$SCRIPT_DIR/$SCRIPT_SOURCE"
done
SCRIPT_DIR="$(cd -P "$(dirname "$SCRIPT_SOURCE")" && pwd)"
PROJECT_DIR="$SCRIPT_DIR"
DOCKER_DEPLOY_DIR="${DOCKER_DEPLOY_DIR:-$PROJECT_DIR/docker/deploy}"

CONTAINER_NAME="${CONTAINER_NAME:-navigation2-vnc}"
SESSION_NAME="${SESSION_NAME:-real-dock-loop}"
WORKSPACE_DIR="${WORKSPACE_DIR:-/home/user/Eurobot-2026-Navigation2-ws}"
STARTUP_DELAY="${STARTUP_DELAY:-5}"
SHUTDOWN_DELAY="${SHUTDOWN_DELAY:-7}"
CHECK_INTERVAL="${CHECK_INTERVAL:-1}"
TMUX_HISTORY_LINES="${TMUX_HISTORY_LINES:-200}"
STUCK_REGEX="${STUCK_REGEX:-\\[follow_path\\] \\[ActionServer\\] Aborting handle\\.|Failed to get result for compute_path_to_pose in node halt!}"
RUN_MODE="${RUN_MODE:-auto}"
MAX_CYCLES="${MAX_CYCLES:-0}"

is_inside_container() {
  [[ -f /.dockerenv ]]
}

resolve_run_mode() {
  case "$RUN_MODE" in
    auto)
      if is_inside_container; then
        echo "container"
      else
        echo "host"
      fi
      ;;
    host|container)
      echo "$RUN_MODE"
      ;;
    *)
      echo "Invalid RUN_MODE: $RUN_MODE (expected: auto, host, container)" >&2
      exit 1
      ;;
  esac
}
usage() {
  cat <<'EOF'
Usage:
  ./real_tmux_loop.sh

Environment overrides:
  DOCKER_DEPLOY_DIR  Host path to docker/deploy
  CONTAINER_NAME  Docker container to exec into (default: navigation2-vnc)
  RUN_MODE        auto|host|container (default: auto)
  SESSION_NAME    tmux session name (default: real-dock-loop)
  WORKSPACE_DIR   Workspace path inside container
  STARTUP_DELAY   Seconds to wait after real_launch starts before docking
  SHUTDOWN_DELAY  Seconds to wait after stopping real_launch before restart
  CHECK_INTERVAL  Seconds between failure checks while docking runs
  TMUX_HISTORY_LINES  Number of left-pane lines to scan for failure patterns
  STUCK_REGEX     Regex used to decide the workflow hit an error
  MAX_CYCLES      Stop after N full cycles (2 goals per cycle); 0 means infinite

Stop the workflow with Ctrl-C inside tmux, or from another terminal:
  tmux kill-session -t <session-name>
EOF
}

require_command() {
  local cmd="$1"
  if ! command -v "$cmd" >/dev/null 2>&1; then
    echo "Missing required command: $cmd" >&2
    exit 1
  fi
}

run_workspace_cmd() {
  local run_mode="$1"
  local container_name="$2"
  local workspace_dir="$3"
  local cmd="$4"

  if [[ "$run_mode" == "container" ]]; then
    bash -ic "cd '$workspace_dir' && source install/setup.bash && $cmd"
  else
    docker exec -i "$container_name" bash -ic "cd '$workspace_dir' && source install/setup.bash && $cmd"
  fi
}

goal_one() {
  cat <<'EOF'
{
  dock_type: 'mission_dock_rush_y',
  use_dock_id: false,
  navigate_to_staging_pose: true,
  max_staging_time: 1000.0,
  dock_pose: {
    header: {
      frame_id: 'map'
    },
    pose: {
      position: {x: 2.4, y: 0.6, z: 0.1},
      orientation: {x: 0.0, y: 0.0, z: 0.707, w: -0.707}
    }
  }
}
EOF
}

goal_two() {
  cat <<'EOF'
{
  dock_type: 'mission_dock_rush_y',
  use_dock_id: false,
  navigate_to_staging_pose: true,
  max_staging_time: 1000.0,
  dock_pose: {
    header: {
      frame_id: 'map'
    },
    pose: {
      position: {x: 2.1, y: 0.8, z: 0.2},
      orientation: {x: 0.0, y: 0.0, z: 1.0, w: 0.0}
    }
  }
}
EOF
}

worker() {
  local session_name="$1"
  local run_mode="$2"
  local container_name="$3"
  local workspace_dir="$4"
  local startup_delay="$5"
  local shutdown_delay="$6"
  local max_cycles="$7"

  local goal
  local cycle=1
  local goal_index=0
  local stopped_due_to_error=0

  if [[ ! "$max_cycles" =~ ^[0-9]+$ ]]; then
    echo "Invalid MAX_CYCLES: $max_cycles (expected integer >= 0)" >&2
    exit 1
  fi

  hold_for_inspection() {
    echo "Loop stopped. Inspect the left pane for the error logs."
    echo "Kill the session with: tmux kill-session -t ${session_name}"
    while true; do
      sleep 3600
    done
  }

  launch_pane_has_error() {
    tmux capture-pane -pt "${session_name}:0.0" -S "-${TMUX_HISTORY_LINES}" \
      | grep -Eiq "$STUCK_REGEX"
  }

  echo "Docking loop is running in tmux session '$session_name'."
  echo "Waiting ${startup_delay}s for real_launch startup before the first docking request."
  sleep "$startup_delay"

  while true; do
    if launch_pane_has_error; then
      echo "Detected error pattern before docking starts. Stopping the loop."
      stopped_due_to_error=1
      break
    fi

    if (( goal_index % 2 == 0 )); then
      if (( max_cycles > 0 && cycle > max_cycles )); then
        echo "Reached MAX_CYCLES=${max_cycles}. Stopping loop cleanly."
        break
      fi
      goal="$(goal_one)"
      printf '\n[%s] Docking cycle %d to point 1\n' "$(date '+%F %T')" "$cycle"
    else
      goal="$(goal_two)"
      printf '\n[%s] Docking cycle %d to point 2\n' "$(date '+%F %T')" "$cycle"
      cycle=$((cycle + 1))
    fi

    run_workspace_cmd "$run_mode" "$container_name" "$workspace_dir" "ros2 action send_goal /dock_robot opennav_docking_msgs/action/DockRobot \"$goal\"" &
    local dock_pid=$!

    while kill -0 "$dock_pid" >/dev/null 2>&1; do
      if launch_pane_has_error; then
        echo "Detected error pattern in real_launch output. Stopping the loop."
        kill -INT "$dock_pid" >/dev/null 2>&1 || true
        wait "$dock_pid" || true
        stopped_due_to_error=1
        break
      fi
      sleep "$CHECK_INTERVAL"
    done

    if [[ "$stopped_due_to_error" == "1" ]]; then
      break
    fi

    if ! wait "$dock_pid"; then
      echo "Docking command failed. Stopping the loop."
      stopped_due_to_error=1
      break
    fi

    if launch_pane_has_error; then
      echo "Detected error pattern after docking. Stopping the loop."
      stopped_due_to_error=1
      break
    fi

    goal_index=$((goal_index + 1))

    if (( goal_index % 2 == 0 )); then
      echo "Two docking goals finished. Restarting real_launch.py."
      tmux send-keys -t "${session_name}:0.0" C-c
      echo "Waiting ${shutdown_delay}s for real_launch shutdown before restart."
      sleep "$shutdown_delay"
      tmux send-keys -t "${session_name}:0.0" "$(printf "%q" "$SCRIPT_DIR/real_tmux_loop.sh") --workspace-cmd '$run_mode' '$container_name' '$workspace_dir' 'ros2 launch navigation2_run real_launch.py'" C-m
      echo "Waiting ${startup_delay}s for real_launch startup before the next docking request."
      sleep "$startup_delay"
    fi
  done

  if [[ "$stopped_due_to_error" == "1" ]]; then
    hold_for_inspection
  fi
}

main() {
  local resolved_run_mode

  if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    usage
    exit 0
  fi

  require_command tmux

  resolved_run_mode="$(resolve_run_mode)"

  if [[ "$resolved_run_mode" == "host" ]]; then
    require_command docker
  fi

  if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
    echo "tmux session '$SESSION_NAME' already exists." >&2
    echo "Use a different SESSION_NAME or kill the old session first." >&2
    exit 1
  fi

  if [[ "$resolved_run_mode" == "host" ]]; then
    if [[ ! -d "$DOCKER_DEPLOY_DIR" ]]; then
      echo "docker/deploy directory not found: $DOCKER_DEPLOY_DIR" >&2
      exit 1
    fi

    if ! docker ps --format '{{.Names}}' | grep -Fxq "$CONTAINER_NAME"; then
      (
        cd "$DOCKER_DEPLOY_DIR"
        docker compose up -d navigation-vnc
      )
    fi

    if ! docker ps --format '{{.Names}}' | grep -Fxq "$CONTAINER_NAME"; then
      echo "Container '$CONTAINER_NAME' is not running after docker compose up." >&2
      echo "Set CONTAINER_NAME if your container uses a different name." >&2
      exit 1
    fi
  fi

  tmux new-session -d -s "$SESSION_NAME" "bash"
  sleep 1
  tmux send-keys -t "${SESSION_NAME}:0.0" "$(printf "%q" "$SCRIPT_DIR/real_tmux_loop.sh") --workspace-cmd '$resolved_run_mode' '$CONTAINER_NAME' '$WORKSPACE_DIR' 'ros2 launch navigation2_run real_launch.py'" C-m

  tmux split-window -h -t "${SESSION_NAME}:0" "bash '$SCRIPT_DIR/real_tmux_loop.sh' --worker '$SESSION_NAME' '$resolved_run_mode' '$CONTAINER_NAME' '$WORKSPACE_DIR' '$STARTUP_DELAY' '$SHUTDOWN_DELAY' '$MAX_CYCLES'"
  tmux select-layout -t "${SESSION_NAME}:0" even-horizontal

  if [[ -n "${TMUX:-}" ]]; then
    tmux switch-client -t "$SESSION_NAME"
  else
    tmux attach-session -t "$SESSION_NAME"
  fi
}

workspace_cmd_main() {
  local run_mode="$1"
  local container_name="$2"
  local workspace_dir="$3"
  local command_to_run="$4"

  if [[ "$run_mode" == "host" ]]; then
    require_command docker
  fi

  run_workspace_cmd "$run_mode" "$container_name" "$workspace_dir" "$command_to_run"
}

if [[ "${1:-}" == "--worker" ]]; then
  shift
  worker "$@"
elif [[ "${1:-}" == "--workspace-cmd" ]]; then
  shift
  workspace_cmd_main "$@"
else
  main "$@"
fi
