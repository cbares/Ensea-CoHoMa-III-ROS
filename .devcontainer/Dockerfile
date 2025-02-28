FROM ros:jazzy

ARG USERNAME=theseus
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Rename default user
ARG OLD_USERNAME=ubuntu
RUN usermod --login $USERNAME --move-home --home /home/$USERNAME $OLD_USERNAME\
    && groupmod --new-name $USERNAME $OLD_USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

RUN apt-get update && apt-get upgrade -y
RUN apt-get update && apt-get install -y python3-pip
ENV SHELL=/bin/bash

# ********************************************************
# * Anything else you want to do like clean up goes here *
# ********************************************************

RUN apt-get update && apt-get install -y ros-${ROS_DISTRO}-joy-teleop ros-${ROS_DISTRO}-joy \
    ros-${ROS_DISTRO}-teleop-twist-joy ros-${ROS_DISTRO}-teleop-twist-keyboard \ 
    ros-${ROS_DISTRO}-joy-linux ros-${ROS_DISTRO}-joy-tester

RUN apt-get update && apt-get install -y ros-${ROS_DISTRO}-urdf-tutorial ros-${ROS_DISTRO}-control-toolbox
RUN apt-get update && apt-get install -y ros-${ROS_DISTRO}-rqt*

RUN apt-get update && apt-get install -y ros-${ROS_DISTRO}-ros2-control ros-${ROS_DISTRO}-ros2-controllers
RUN apt-get update && apt-get install -y libcxx-serial-dev libcxx-serial1

RUN apt-get update && apt-get install -y ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup ros-${ROS_DISTRO}-nav2-rviz-plugins ros-${ROS_DISTRO}-twist-mux

# rosdep request to install dependencies:
RUN apt-get update && apt-get install -y ros-${ROS_DISTRO}-ros2-controllers-test-nodes liburdfdom-tools


# Change the size of rviz window (hpi screen)
RUN sed -i -e "s/1200/4000/" /opt/ros/${ROS_DISTRO}/share/urdf_tutorial/rviz/urdf.rviz
RUN sed -i -e "s/800/2000/" /opt/ros/${ROS_DISTRO}/share/urdf_tutorial/rviz/urdf.rviz

ENV RCUTILS_COLORIZED_OUTPUT=1
USER ${USERNAME}
RUN mkdir -p /home/${USERNAME}/ws/src

WORKDIR /home/${USERNAME}/ws

RUN rosdep update && rosdep install --from-paths /home/${USERNAME}/ws/src --ignore-src -y && sudo chown -R $(whoami) /home/${USERNAME}/ws/
SHELL ["/bin/bash", "--login", "-c"] 
RUN grep -qF 'TAG1' $HOME/.profile || echo 'source /opt/ros/${ROS_DISTRO}/setup.bash # TAG1' >> $HOME/.profile
RUN --mount=type=bind,source=modelidar,target=src/modelidar \
    /bin/bash -c "source  /opt/ros/${ROS_DISTRO}/setup.bash; colcon build" 
RUN grep -qF 'TAG2' $HOME/.profile || echo 'source ~/ws/install/setup.bash # TAG2' >> $HOME/.profile

COPY --chmod=+x <<-"EOF" /entrypoint.sh
#!/bin/bash
source /opt/ros/${ROS_DISTRO}/setup.bash
source ~/ws/install/setup.bash
exec "$@"
EOF

ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]
CMD [ "ros2", "launch", "modelidar", "modelidar.launch.py"]


# ENTRYPOINT ["tail", "-f", "/dev/null"]
