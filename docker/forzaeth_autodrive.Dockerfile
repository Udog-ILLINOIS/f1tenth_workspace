# ForzaETH race stack (race_stack:humble_arm, built by the upstream Makefile/compose) plus
# what the AutoDRIVE ROS 2 bridge needs. Socket.IO pins match AutoDRIVE 0.3.0's client
# (same as autodrive/requirements_arm64.txt); gevent comes from apt because pip can't build it
# against the stack's pinned setuptools 58; attrdict3 replaces attrdict, which breaks on 3.10.
FROM race_stack:humble_arm
USER root
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-foxglove-bridge ros-humble-cv-bridge \
        python3-gevent python3-gevent-websocket \
    && rm -rf /var/lib/apt/lists/*
RUN pip3 install --no-cache-dir attrdict3 python-socketio==4.2.0 python-engineio==3.13.0
ARG USER
USER ${USER}
