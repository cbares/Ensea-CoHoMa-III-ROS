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

# dependencies for LSlidar
RUN apt-get update && apt-get install -y libpcl-dev ros-${ROS_DISTRO}-pcl-ros ros-${ROS_DISTRO}-diagnostic-updater 
RUN apt-get update && apt-get install -y libflann-dev libpcap-dev

RUN apt-get update && apt-get install -y ros-${ROS_DISTRO}-rmw-cyclonedds-cpp

RUN apt-get update && apt-get install -y ros-${ROS_DISTRO}-slam-toolbox ros-${ROS_DISTRO}-nav2-route

ENV RCUTILS_COLORIZED_OUTPUT=1
USER ${USERNAME}
RUN mkdir -p /home/${USERNAME}/ws/src

WORKDIR /home/${USERNAME}/ws

# Add user to video group to allow access to webcams and dialout for serial devices
RUN sudo usermod --append --groups video,dialout $USERNAME

RUN rosdep update && rosdep install --from-paths /home/${USERNAME}/ws/src --ignore-src -y && sudo chown -R $(whoami) /home/${USERNAME}/ws/
# SHELL ["/bin/bash", "--login", "-c"] 
RUN grep -qF 'TAG1' $HOME/.bashrc || echo 'source /opt/ros/${ROS_DISTRO}/setup.bash # TAG1' >> $HOME/.bashrc
RUN grep -qF 'TAG2' $HOME/.bashrc || echo 'source ~/ws/install/setup.bash # TAG2' >> $HOME/.bashrc

RUN --mount=type=bind,source=modelidar,target=src/modelidar \
    --mount=type=bind,source=Lslidar_ROS2_driver,target=src/Lslidar_ROS2_driver \
    /bin/bash -c "source  /opt/ros/${ROS_DISTRO}/setup.bash; colcon build --symlink-install" 

COPY --chmod=+x <<-"EOF" /entrypoint.sh
#!/bin/bash -l
source /opt/ros/${ROS_DISTRO}/setup.bash
source ~/ws/install/setup.bash
export DISPLAY=:0
exec "$@"
EOF
# COPY -chmod work only with in octal value in some Dockerfile
RUN sudo chmod +x /entrypoint.sh 

ENTRYPOINT ["/entrypoint.sh"]
CMD [ "ros2", "launch", "modelidar", "view_robot.launch.py"]
# CMD [ "ros2", "launch", "modelidar", "modelidar.launch.py"]
# CMD ["tail", "-f", "/dev/null"]
