#!/usr/bin/env bash

set -euo pipefail

SCRIPT_SOURCE="${BASH_SOURCE[0]}"
while [ -h "$SCRIPT_SOURCE" ]; do
  SCRIPT_DIR="$(cd -P "$(dirname "$SCRIPT_SOURCE")" && pwd)"
  SCRIPT_SOURCE="$(readlink "$SCRIPT_SOURCE")"
  [[ $SCRIPT_SOURCE != /* ]] && SCRIPT_SOURCE="$SCRIPT_DIR/$SCRIPT_SOURCE"
done
SCRIPT_DIR="$(cd -P "$(dirname "$SCRIPT_SOURCE")" && pwd)"
SCRIPT_PATH="$SCRIPT_DIR/$(basename "$SCRIPT_SOURCE")"

CONTAINER_NAME="${CONTAINER_NAME:-navigation2}"
SESSION_NAME="${SESSION_NAME:-dock-loop}"
WORKSPACE_DIR="${WORKSPACE_DIR:-/home/user/Eurobot-2026-Navigation2-ws}"
STARTUP_DELAY="${STARTUP_DELAY:-5}"
RESTART_DELAY="${RESTART_DELAY:-3}"
CHECK_INTERVAL="${CHECK_INTERVAL:-1}"
TMUX_HISTORY_LINES="${TMUX_HISTORY_LINES:-200}"
STUCK_REGEX="${STUCK_REGEX:-failed to create plan|Planning algorithm .* failed to generate a valid path|\\[compute_path_to_pose\\] \\[ActionServer\\] Aborting handle}"

usage() {
  cat <<'EOF'
Usage:
  ./docking_tmux_loop.sh

Environment overrides:
  CONTAINER_NAME  Docker container to exec into (default: navigation2)
  SESSION_NAME    tmux session name (default: dock-loop)
  WORKSPACE_DIR   Workspace path inside container
  STARTUP_DELAY   Seconds to wait after starting/restarting sim before docking
  RESTART_DELAY   Seconds to wait between stopping sim and starting it again
  CHECK_INTERVAL  Seconds between stuck-state checks while docking runs
  TMUX_HISTORY_LINES  Number of sim-pane lines to scan for failure patterns
  STUCK_REGEX     Regex used to decide navigation is stuck

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

dock_goal() {
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
      position: {x: 0.3, y: 1.5, z: 0.2},
      orientation: {x: 0.0, y: 0.0, z: 0.707, w: -0.707}
    }
  }
}
EOF
}

worker() {
  local session_name="$1"
  local container_name="$2"
  local workspace_dir="$3"
  local startup_delay="$4"
  local restart_delay="$5"

  local goal
  goal="$(dock_goal)"
  local cycle=1
  local stopped_due_to_breakdown=0

  hold_for_inspection() {
    echo "Loop stopped. Inspect the sim pane for the breakdown logs."
    echo "Kill the session with: tmux kill-session -t ${session_name}"
    while true; do
      sleep 3600
    done
  }

  sim_pane_is_stuck() {
    tmux capture-pane -pt "${session_name}:0.0" -S "-${TMUX_HISTORY_LINES}" \
      | grep -Eiq "$STUCK_REGEX"
  }

  echo "Docking loop is running in tmux session '$session_name'."
  echo "Waiting ${startup_delay}s for sim startup before the first docking request."
  sleep "$startup_delay"

  while true; do
    printf '\n[%s] Docking cycle %d\n' "$(date '+%F %T')" "$cycle"

    if sim_pane_is_stuck; then
      echo "Navigation looks stuck before docking starts. Stopping the loop."
      break
    fi

    docker exec -i "$container_name" bash -ic "cd '$workspace_dir' && source install/setup.bash && ros2 action send_goal /dock_robot opennav_docking_msgs/action/DockRobot \"$goal\"" &
    local dock_pid=$!

    while kill -0 "$dock_pid" >/dev/null 2>&1; do
      if sim_pane_is_stuck; then
        echo "Detected navigation stuck pattern in sim output. Stopping the loop."
        kill -INT "$dock_pid" >/dev/null 2>&1 || true
        wait "$dock_pid" || true
        stopped_due_to_breakdown=1
        break
      fi
      sleep "$CHECK_INTERVAL"
    done

    if [[ "$stopped_due_to_breakdown" == "1" ]]; then
      break
    fi

    if ! wait "$dock_pid"; then
      echo "Docking command failed. Leaving the loop running stopped in this pane."
      break
    fi

    if sim_pane_is_stuck; then
      echo "Detected navigation stuck pattern after docking. Stopping the loop."
      stopped_due_to_breakdown=1
      break
    fi

    echo "Docking finished. Restarting sim."
    tmux send-keys -t "${session_name}:0.0" C-c
    sleep "$restart_delay"
    tmux send-keys -t "${session_name}:0.0" "sim" C-m

    cycle=$((cycle + 1))
    echo "Waiting ${startup_delay}s for sim to come back up."
    sleep "$startup_delay"
  done

  if [[ "$stopped_due_to_breakdown" == "1" ]]; then
    hold_for_inspection
  fi

  return
}

main() {
  if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    usage
    exit 0
  fi

  require_command docker
  require_command tmux

  if ! docker ps --format '{{.Names}}' | grep -Fxq "$CONTAINER_NAME"; then
    echo "Container '$CONTAINER_NAME' is not running." >&2
    echo "Set CONTAINER_NAME if your container uses a different name." >&2
    exit 1
  fi

  if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
    echo "tmux session '$SESSION_NAME' already exists." >&2
    echo "Use a different SESSION_NAME or kill the old session first." >&2
    exit 1
  fi

  tmux new-session -d -s "$SESSION_NAME" "docker exec -it $CONTAINER_NAME bash"
  sleep 1

  tmux send-keys -t "${SESSION_NAME}:0.0" "cd '$WORKSPACE_DIR' && source install/setup.bash && sim" C-m

  tmux split-window -h -t "${SESSION_NAME}:0" "bash '$SCRIPT_PATH' --worker '$SESSION_NAME' '$CONTAINER_NAME' '$WORKSPACE_DIR' '$STARTUP_DELAY' '$RESTART_DELAY'"
  tmux select-layout -t "${SESSION_NAME}:0" even-horizontal

  if [[ -n "${TMUX:-}" ]]; then
    tmux switch-client -t "$SESSION_NAME"
  else
    tmux attach-session -t "$SESSION_NAME"
  fi
}

if [[ "${1:-}" == "--worker" ]]; then
  shift
  worker "$@"
else
  main "$@"
fi
