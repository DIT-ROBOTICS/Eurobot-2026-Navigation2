FROM --platform=$BUILDPLATFORM ros:humble-ros-base-jammy

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
    && rm -rf /var/lib/apt/lists/*

# build user
RUN useradd -m -s /bin/bash user && \
    echo "user ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/user

WORKDIR /home/user

COPY start-vnc.sh /home/user/start-vnc.sh
RUN chown user:user /home/user/start-vnc.sh && \
    chmod +x /home/user/start-vnc.sh

USER user

ENV VNC_PASSWORD=ros
ENV DISPLAY=:1

EXPOSE 5901

ENTRYPOINT ["/home/user/start-vnc.sh"]
CMD [""]
