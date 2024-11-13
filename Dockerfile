FROM ros:humble-ros-core

RUN apt update && apt install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    ros-humble-ackermann-msgs \
    ros-humble-tf2-msgs \
    git

RUN rosdep init && rosdep update

COPY . /root/muto/

WORKDIR /root/muto

RUN mkdir launch/ config/
COPY ./muto.launch.py /root/muto/launch/
COPY ./muto.yaml /root/muto/config/

RUN ./setup_ws.bash
RUN rosdep install --from-paths src --ignore-src -r -y
RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

RUN echo "source /root/muto/install/setup.bash" >> ~/.bashrc

ENTRYPOINT ["/bin/bash", "-c", "ls && pwd && source ./install/setup.bash && ros2 launch ./launch/muto.launch.py"]
