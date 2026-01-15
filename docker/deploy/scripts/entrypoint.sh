#!/bin/bash
set -e

##############################################################################
# Entrypoint for Dynamic User ID Matching
# Runs only during container startup (not docker exec)
##############################################################################

DESIRED_UID=${USERID:-1000}
DESIRED_GID=${USERGID:-$DESIRED_UID}
CONTAINER_USER=${USERNAME:-user}

# Only adjust IDs if running as root (container startup)
if [ "$(id -u)" = "0" ]; then
    echo "=== Eurobot 2026 Navigation Container ==="
    echo "Adjusting user IDs: UID=$DESIRED_UID GID=$DESIRED_GID"
    
    CURRENT_UID=$(id -u $CONTAINER_USER 2>/dev/null || echo "1000")
    CURRENT_GID=$(id -g $CONTAINER_USER 2>/dev/null || echo "1000")
    
    if [ "$CURRENT_UID" != "$DESIRED_UID" ] || [ "$CURRENT_GID" != "$DESIRED_GID" ]; then
        groupmod -o -g "$DESIRED_GID" "$CONTAINER_USER" 2>/dev/null || true
        usermod -o -u "$DESIRED_UID" "$CONTAINER_USER" 2>/dev/null || true
        chown -R ${DESIRED_UID}:${DESIRED_GID} /home/${CONTAINER_USER} 2>/dev/null || true
        echo "✓ User IDs updated"
    else
        echo "✓ User IDs already match"
    fi
    
    echo "Switching to user: $CONTAINER_USER"
    exec gosu $CONTAINER_USER "$0" "$@"
fi

# Running as non-root user (after gosu switch)
# Execute the provided command or start bash
if [ $# -eq 0 ]; then
    exec /bin/bash
else
    exec "$@"
fi
