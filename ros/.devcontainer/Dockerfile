# Host-side ROS 2 environment for ros/ (PLAN.md §6). Firmware, agent and host stay on
# the same ROS distro; move all three to Lyrical together (§9).
FROM ros:jazzy

ENV LC_ALL=C.UTF-8
ENV LANG=C.UTF-8

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions \
        ros-jazzy-ros2-control \
        ros-jazzy-ros2-controllers \
        ros-jazzy-joint-state-topic-hardware-interface \
        ros-jazzy-twist-mux \
        ros-jazzy-xacro \
        ros-jazzy-robot-state-publisher \
        ros-jazzy-teleop-twist-keyboard \
        ros-jazzy-diagnostics \
        ros-jazzy-launch-testing-ament-cmake \
        ros-jazzy-launch-testing-ros \
        ros-jazzy-ament-lint-auto \
        ros-jazzy-ament-lint-common \
    && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc \
    && echo "[ -f /workspaces/ros/install/setup.bash ] && source /workspaces/ros/install/setup.bash" >> ~/.bashrc
