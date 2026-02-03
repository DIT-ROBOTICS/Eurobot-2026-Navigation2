#!/bin/bash
##############################################################################
# VNC Server Startup Script
# Starts VNC server with XFCE desktop environment
##############################################################################

echo "Starting VNC server..."

# Fix hostname resolution issue in Docker containers FIRST
# This must happen before any command that needs hostname resolution (including sudo)
HOSTNAME=$(hostname)
if ! getent hosts "$HOSTNAME" > /dev/null 2>&1; then
    echo "Fixing hostname resolution for: $HOSTNAME"
    # Write directly - container runs as privileged so we can use su or write directly
    # Use python as a portable way to append without sudo
    python3 -c "
import os
with open('/etc/hosts', 'a') as f:
    f.write('127.0.0.1 $HOSTNAME\n')
" 2>/dev/null || echo "127.0.0.1 $HOSTNAME" | sudo tee -a /etc/hosts > /dev/null
fi

# Create .vnc directory if it doesn't exist
mkdir -p ~/.vnc
sudo chown -R $USER:$USER ~/.vnc 2>/dev/null || true
chmod 700 ~/.vnc

# Remove old .Xauthority to avoid conflicts
rm -f ~/.Xauthority ~/.Xauthority-*

# Set VNC password (default: ros)
VNC_PASSWORD=${VNC_PASSWORD:-ros}
echo "$VNC_PASSWORD" | vncpasswd -f > ~/.vnc/passwd
chmod 600 ~/.vnc/passwd

# Create xstartup script
cat > ~/.vnc/xstartup << 'EOF'
#!/bin/bash
unset SESSION_MANAGER
unset DBUS_SESSION_BUS_ADDRESS
export XDG_SESSION_TYPE=x11
export GDK_BACKEND=x11
exec dbus-launch startxfce4
EOF

chmod +x ~/.vnc/xstartup

# VNC display number (default: 5 to avoid conflict with host X server when using network_mode: host)
VNC_DISPLAY=${VNC_DISPLAY:-5}
VNC_PORT=$((5900 + VNC_DISPLAY))

# Kill any existing VNC server and clean up stale files
vncserver -kill :$VNC_DISPLAY 2>/dev/null || true
rm -f /tmp/.X${VNC_DISPLAY}-lock /tmp/.X11-unix/X${VNC_DISPLAY} 2>/dev/null || true

# Start VNC server
vncserver :$VNC_DISPLAY -geometry 1920x1080 -depth 24 -localhost no

echo "VNC server started on :$VNC_DISPLAY (port $VNC_PORT)"
echo "Password: $VNC_PASSWORD"
echo "Connect with: <hostname>:$VNC_PORT"

# Keep container running
if [ $# -eq 0 ]; then
    echo "Tailing VNC logs..."
    tail -f ~/.vnc/*.log
else
    # Execute provided command
    exec "$@"
fi
