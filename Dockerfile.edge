FROM osrf/ros:humble-desktop-full

WORKDIR '/edge'

SHELL ["/bin/bash", "-c"]

RUN apt-get update --fix-missing && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-vcstool \
    ros-humble-ackermann-msgs

COPY ./muto.repos ./muto.repos
COPY ./launch ./launch
COPY ./config ./config
RUN mkdir -p ./src && vcs import src < muto.repos

RUN source /opt/ros/humble/setup.bash && \
    rosdep install -y -r --from-paths ./src --ignore-src --rosdistro humble && \
    colcon build
    
COPY ./entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
