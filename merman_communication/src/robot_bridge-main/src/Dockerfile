FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONUNBUFFERED=1 \
    PIP_NO_CACHE_DIR=1 \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8

RUN apt-get update && apt-get install -y \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Python deps used by MQTT command client.
RUN pip3 install --no-cache-dir \
    paho-mqtt \
    pyyaml

WORKDIR /app
COPY . /app

# Keep ROS2 environment available in shell form entrypoint.
CMD ["bash", "-lc", "source /opt/ros/humble/setup.bash && python3 -m robot_bridge"]
