#!/bin/bash

# Get the directory of the script (resolves symlinks)
SOURCE="${BASH_SOURCE[0]}"
while [ -h "$SOURCE" ]; do
  DIR="$( cd -P "$( dirname "$SOURCE" )" &> /dev/null && pwd )"
  SOURCE="$(readlink "$SOURCE")"
  [[ $SOURCE != /* ]] && SOURCE="$DIR/$SOURCE"
done
DIR="$( cd -P "$( dirname "$SOURCE" )" &> /dev/null && pwd )"

# Change to project root
cd "$DIR"

# Open Xhost
xhost +local: > /dev/null 2>&1

cd ./docker/deploy

# Environment variables
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:=100}
export USER_UID=${USER_UID:=$(id -u)}

# Ensure cleanup on script exit
cleanup() {
    xhost -local: > /dev/null 2>&1
}
trap cleanup EXIT

# Argument handling
ACTION=${1:-}
SERVICE=${2:-}

print_usage() {
    echo -e "\033[1;32m----- [ Navigation2 Usage ] -----------------\033[0m"
    echo -e "\033[1;32m|\033[0m   rebuild         - Rebuild Docker images"
    echo -e "\033[1;32m|\033[0m   build           - Build the ROS workspace"
    echo -e "\033[1;32m|\033[0m   run             - Run navigation2"
    echo -e "\033[1;32m|\033[0m   dev             - Enter development container"
    echo -e "\033[1;32m|\033[0m   vnc             - Start VNC container"
    echo -e "\033[1;32m|\033[0m   stop [service]  - Stop container(s)"
    echo -e "\033[1;32m|\033[0m   logs [service]  - View container logs"
    echo -e "\033[1;32m|\033[0m   exec <service> <cmd> - Run command in service"
    echo -e "\033[1;32m|\033[0m   ps              - List running containers"
    echo -e "\033[1;32m|\033[0m   tools           - Show all available commands"
    echo -e "\033[1;32m|\033[0m   clean           - Stop all and remove containers"
    echo -e "\033[1;32m|\033[0m   install         - Install 'nav2' command globally"
    echo -e "\033[1;32m---------------------------------------------\033[0m"
    echo -e "Services: build, dev, run, vnc"
}

print_tools() {
    echo -e "\033[1;36m━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\033[0m"
    echo -e "\033[1;36m  Available Commands for Navigation2\033[0m"
    echo -e "\033[1;36m━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\033[0m"
    echo ""
    echo -e "\033[1;33m[ Build & Development ]\033[0m"
    echo -e "  \033[1;32mrebuild\033[0m            Rebuild Docker images from Dockerfile"
    echo -e "                     Use after modifying Dockerfile or dependencies"
    echo -e "  \033[1;32mbuild\033[0m              Build the entire ROS2 workspace"
    echo -e "                     Uses colcon build in container"
    echo ""
    echo -e "\033[1;33m[ Running Services ]\033[0m"
    echo -e "  \033[1;32mrun\033[0m                Launch Navigation2 stack (detached)"
    echo -e "                     Runs real_launch.py from navigation2_run"
    echo -e "  \033[1;32mdev\033[0m                Enter interactive development shell"
    echo -e "                     For testing, debugging, and development"
    echo -e "  \033[1;32mvnc\033[0m                Start VNC server with GUI support"
    echo -e "                     Access via VNC viewer on port 5901"
    echo ""
    echo -e "\033[1;33m[ Container Management ]\033[0m"
    echo -e "  \033[1;32mstop\033[0m [service]    Stop containers (all or specific)"
    echo -e "                     Examples: 'stop', 'stop run', 'stop dev'"
    echo -e "  \033[1;32mps\033[0m                 Show running containers status"
    echo -e "  \033[1;32mclean\033[0m              Stop and remove all containers"
    echo ""
    echo -e "\033[1;33m[ Monitoring & Debugging ]\033[0m"
    echo -e "  \033[1;32mlogs\033[0m [service]    View container logs (follow mode)"
    echo -e "                     Examples: 'logs run', 'logs build'"
    echo -e "  \033[1;32mexec\033[0m <srv> <cmd>  Execute command in running container"
    echo -e "                     Example: 'exec dev \"ros2 topic list\"'"
    echo ""
    echo -e "\033[1;33m[ Setup ]\033[0m"
    echo -e "  \033[1;32minstall\033[0m            Install global 'nav2' command"
    echo -e "                     After install, use 'nav2 build' instead of './nav2.sh build'"
    echo -e "  \033[1;32mtools\033[0m              Show this help message"
    echo ""
    echo -e "\033[1;36m━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\033[0m"
    echo -e "\033[1;90m  Tip: Run 'install' to use commands without './nav2.sh'\033[0m"
    echo -e "\033[1;36m━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\033[0m"
}

case $ACTION in
    rebuild)
        echo "Rebuilding all Docker images..."
        echo "Building base image (without VNC)..."
        docker compose build navigation-build navigation-develop navigation-run
        echo ""
        echo "Building VNC image..."
        docker compose build navigation-vnc
        echo ""
        echo "✓ All Docker images rebuilt successfully."
        ;;
    
    build)
        echo "Building the ROS workspace..."
        docker compose up navigation-build
        ;;
    
    run)
        echo "Running navigation2..."
        docker compose up -d navigation-run
        echo "Navigation2 is running. Use 'nav2 logs run' to view logs."
        echo "Use 'nav2 stop run' to stop it."
        ;;
    
    dev|develop)
        echo "Starting development container..."
        if ! docker ps --filter "name=navigation2-dev" --filter "status=running" | grep -q "navigation2-dev"; then
            docker compose up -d navigation-develop
        fi
        docker compose exec navigation-develop bash || echo "Exited container."
        ;;
    
    vnc)
        echo "Starting VNC container..."
        docker compose up -d navigation-vnc
        echo "VNC server started on port ${VNC_PORT:-5901}"
        echo "Connect using VNC viewer: localhost:${VNC_PORT:-5901}"
        echo "Password: ${VNC_PASSWORD:-ros}"
        docker compose exec navigation-vnc bash || echo "Exited container."
        ;;
    
    stop)
        if [[ -z "$SERVICE" ]]; then
            echo "Stopping all navigation containers..."
            docker compose down
        else
            case $SERVICE in
                build)
                    docker compose stop navigation-build
                    docker compose rm -f navigation-build
                    ;;
                dev|develop)
                    docker compose stop navigation-develop
                    docker compose rm -f navigation-develop
                    ;;
                run)
                    docker compose stop navigation-run
                    docker compose rm -f navigation-run
                    ;;
                vnc)
                    docker compose stop navigation-vnc
                    docker compose rm -f navigation-vnc
                    ;;
                *)
                    echo "Unknown service: $SERVICE"
                    echo "Available: build, dev, run, vnc"
                    exit 1
                    ;;
            esac
            echo "Stopped $SERVICE"
        fi
        ;;
    
    logs)
        if [[ -z "$SERVICE" ]]; then
            docker compose logs -f
        else
            case $SERVICE in
                build)
                    docker compose logs -f navigation-build
                    ;;
                dev|develop)
                    docker compose logs -f navigation-develop
                    ;;
                run)
                    docker compose logs -f navigation-run
                    ;;
                vnc)
                    docker compose logs -f navigation-vnc
                    ;;
                *)
                    echo "Unknown service: $SERVICE"
                    exit 1
                    ;;
            esac
        fi
        ;;
    
    exec)
        if [[ -z "$SERVICE" ]]; then
            echo "Error: No service specified."
            echo "Usage: ./nav2.sh exec <service> <command>"
            exit 1
        fi
        COMMAND=${@:3}
        if [[ -z "$COMMAND" ]]; then
            echo "Error: No command specified."
            exit 1
        fi
        
        case $SERVICE in
            build)
                docker compose exec navigation-build bash -c "$COMMAND"
                ;;
            dev|develop)
                docker compose exec navigation-develop bash -c "$COMMAND"
                ;;
            run)
                docker compose exec navigation-run bash -c "$COMMAND"
                ;;
            vnc)
                docker compose exec navigation-vnc bash -c "$COMMAND"
                ;;
            *)
                echo "Unknown service: $SERVICE"
                exit 1
                ;;
        esac
        ;;
    
    ps|status)
        echo "Running navigation containers:"
        docker compose ps
        ;;
    
    clean)
        echo "Stopping and removing all containers..."
        docker compose down -v
        echo "Cleanup complete."
        ;;
    
    tools)
        print_tools
        ;;
    
    install)
        echo "Installing 'nav2' command globally..."
        SCRIPT_PATH="$DIR/nav2.sh"
        
        # Create ~/.local/bin if it doesn't exist
        mkdir -p "$HOME/.local/bin"
        
        # Create symlink
        ln -sf "$SCRIPT_PATH" "$HOME/.local/bin/nav2"
        echo "✓ Installed to ~/.local/bin/nav2"
        
        # Check if ~/.local/bin is in PATH
        if [[ ":$PATH:" != *":$HOME/.local/bin:"* ]]; then
            echo ""
            echo -e "\033[1;33m⚠ Note: ~/.local/bin is not in your PATH\033[0m"
            echo "Add this line to your ~/.bashrc:"
            echo -e "\033[1;36m  export PATH=\"\$HOME/.local/bin:\$PATH\"\033[0m"
            echo "Then run: source ~/.bashrc"
        else
            echo "✓ You can now use 'nav2 build', 'nav2 run', etc. from anywhere!"
        fi
        ;;
    
    *)
        print_usage
        exit 1
        ;;
esac
