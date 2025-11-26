#!/usr/bin/env bash

# 確保 DISPLAY 有值
: "${DISPLAY:=:1}"

# /tmp/.X11-unix 掛在 volume 上，權限要 1777 給 X 用
sudo mkdir -p /tmp/.X11-unix
sudo chmod 1777 /tmp/.X11-unix

# 1. 設定 VNC 密碼
mkdir -p "$HOME/.vnc"
: "${VNC_PASSWORD:=ros}"
echo "$VNC_PASSWORD" | vncpasswd -f > "$HOME/.vnc/passwd"
chmod 600 "$HOME/.vnc/passwd"

# 2. xstartup：啟動 XFCE
cat > "$HOME/.vnc/xstartup" << 'EOF'
#!/bin/sh
xrdb "$HOME/.Xresources" 2>/dev/null || true

export DESKTOP_SESSION=xfce
export XDG_CURRENT_DESKTOP=XFCE

startxfce4
EOF

chmod +x "$HOME/.vnc/xstartup"

# 3. 有 dbus 就開
if command -v dbus-launch >/dev/null 2>&1; then
  eval "$(dbus-launch --sh-syntax)"
fi

# 4. 設定 X 授權檔
export XAUTHORITY="$HOME/.Xauthority"

# 5. 啟動 VNC / X server :1
vncserver "$DISPLAY" -geometry 1600x900 -localhost no

# 6. 放寬 access control，允許 local client（包含 navigation2）透過 UNIX socket 連過來
xhost +local:

# 7. 保持 container 活著
sleep infinity