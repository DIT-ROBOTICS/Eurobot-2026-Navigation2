FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

RUN apt-get update && \
    apt-get install -y \
      wget \
      curl \
      tigervnc-standalone-server \
      xterm \
      dbus-x11 \
      xfce4 \
      xfce4-terminal \
      ros-humble-rmw-cyclonedds-cpp \
      x11-xserver-utils \ 
      sudo \
      ros-humble-navigation2 \
      ros-humble-nav2-bringup \
      ros-humble-bondcpp \
      ros-humble-behaviortree-cpp-v3 \
    && rm -rf /var/lib/apt/lists/*

# build user
RUN useradd -m -s /bin/bash user && \
    echo "user ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/user

WORKDIR /home/user

COPY start-vnc.sh /home/user/start-vnc.sh
RUN chown user:user /home/user/start-vnc.sh && \
    chmod +x /home/user/start-vnc.sh

USER user

# setup bashrc
RUN cat << 'EOF' >> /home/user/.bashrc

export TERM=xterm-256color

if [ -f /opt/ros/humble/setup.bash ]; then
  . /opt/ros/humble/setup.bash
fi

if [ -f "$HOME/Eurobot-2026-Navigation2-ws/install/setup.bash" ]; then
  . "$HOME/Eurobot-2026-Navigation2-ws/install/setup.bash"
fi

PS1='\[\e[01;32m\]\u@\h\[\e[00m\]:\[\e[01;34m\]\w\[\e[00m\]\$ '

alias sim='ros2 launch navigation2_run sim_launch.py'
alias object='ros2 run object_layer ObjectSim'
alias rival='ros2 run object_layer ObjectSim'

EOF

# end of bashrc

ENV VNC_PASSWORD=ros
ENV DISPLAY=:1

EXPOSE 5901

ENTRYPOINT ["/home/user/start-vnc.sh"]
CMD [""]
