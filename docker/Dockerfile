ARG WORKSPACE=/opt/workspace

FROM osrf/ros:humble-desktop

# install build tools
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    python3-colcon-common-extensions \
    git-core \
    && rm -rf /var/lib/apt/lists/*

# get ubr code
ARG WORKSPACE
WORKDIR $WORKSPACE/src
RUN git clone https://github.com/mikeferguson/ubr_reloaded.git \
    && touch ubr_reloaded/ubr1_bringup/COLCON_IGNORE \
    && touch ubr_reloaded/ubr1_calibration/COLCON_IGNORE \
    && touch ubr_reloaded/ubr1_gazebo/COLCON_IGNORE \
    && touch ubr_reloaded/ubr1_moveit/COLCON_IGNORE \
    && touch ubr_reloaded/ubr1_navigation/COLCON_IGNORE \
    && touch ubr_reloaded/ubr_msgs/COLCON_IGNORE \
    && touch ubr_reloaded/ubr_teleop/COLCON_IGNORE

# install dependencies
ARG WORKSPACE
WORKDIR $WORKSPACE
RUN . /opt/ros/$ROS_DISTRO/setup.sh \
    && apt-get update && rosdep install -q -y \
      --from-paths src \
      --ignore-src \
    && rm -rf /var/lib/apt/lists/*

# build ubr code
ARG WORKSPACE
WORKDIR $WORKSPACE
RUN . /opt/ros/$ROS_DISTRO/setup.sh \
    && colcon build

# setup entrypoint
COPY ./ros_entrypoint.sh /

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
