#!/usr/bin/env bash

# Combined VNC + Navigation startup script

# Ensure DISPLAY has a value
: "${DISPLAY:=:1}"

# Set /tmp/.X11-unix permissions
sudo mkdir -p /tmp/.X11-unix
sudo chmod 1777 /tmp/.X11-unix

# 1. Set VNC password
mkdir -p "$HOME/.vnc"
: "${VNC_PASSWORD:=ros}"
echo "$VNC_PASSWORD" | vncpasswd -f > "$HOME/.vnc/passwd"
chmod 600 "$HOME/.vnc/passwd"

# 2. Create xstartup: start XFCE
cat > "$HOME/.vnc/xstartup" << 'EOF'
#!/bin/sh
xrdb "$HOME/.Xresources" 2>/dev/null || true

export DESKTOP_SESSION=xfce
export XDG_CURRENT_DESKTOP=XFCE

# Disable screen blanking
xset s off 2>/dev/null || true
xset -dpms 2>/dev/null || true
xset s noblank 2>/dev/null || true

startxfce4
EOF

chmod +x "$HOME/.vnc/xstartup"

# 2.1 Disable XFCE screensaver and power management
mkdir -p "$HOME/.config/xfce4/xfconf/xfce-perchannel-xml"

# Disable power management
cat > "$HOME/.config/xfce4/xfconf/xfce-perchannel-xml/xfce4-power-manager.xml" << 'EOF'
<?xml version="1.0" encoding="UTF-8"?>
<channel name="xfce4-power-manager" version="1.0">
  <property name="xfce4-power-manager" type="empty">
    <property name="dpms-enabled" type="bool" value="false"/>
    <property name="blank-on-ac" type="int" value="0"/>
    <property name="blank-on-battery" type="int" value="0"/>
  </property>
</channel>
EOF

# Disable screensaver and screen lock
cat > "$HOME/.config/xfce4/xfconf/xfce-perchannel-xml/xfce4-screensaver.xml" << 'EOF'
<?xml version="1.0" encoding="UTF-8"?>
<channel name="xfce4-screensaver" version="1.0">
  <property name="saver" type="empty">
    <property name="enabled" type="bool" value="false"/>
    <property name="mode" type="int" value="0"/>
  </property>
  <property name="lock" type="empty">
    <property name="enabled" type="bool" value="false"/>
  </property>
</channel>
EOF

# 3. Clean up old X server files
DNUM="${DISPLAY#:}"
sudo rm -f "/tmp/.X11-unix/X${DNUM}" "/tmp/.X${DNUM}-lock"

# 4. Start VNC server
vncserver "$DISPLAY" -geometry 1600x900 -localhost no

# 5. Wait for VNC to start
sleep 5

# 6. Keep container running and allow interactive shell
echo "VNC Server started on display $DISPLAY"
echo "Connect via VNC to port 5901"
echo "You can now run navigation commands in this container"

# Keep container alive
sleep infinity
