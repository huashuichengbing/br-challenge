FROM osrf/ros:melodic-desktop-full

RUN apt update

RUN apt install -y \
    ros-$ROS_DISTRO-rqt-py-trees \
    ros-$ROS_DISTRO-rqt-reconfigure

RUN apt install -y \
    ros-$ROS_DISTRO-joy ros-$ROS_DISTRO-teleop-twist-joy \
    ros-$ROS_DISTRO-teleop-twist-keyboard ros-$ROS_DISTRO-laser-proc \
    ros-$ROS_DISTRO-rgbd-launch ros-$ROS_DISTRO-depthimage-to-laserscan \
    ros-$ROS_DISTRO-rosserial-arduino ros-$ROS_DISTRO-rosserial-python \
    ros-$ROS_DISTRO-rosserial-server ros-$ROS_DISTRO-rosserial-client \
    ros-$ROS_DISTRO-rosserial-msgs ros-$ROS_DISTRO-amcl ros-$ROS_DISTRO-map-server \
    ros-$ROS_DISTRO-move-base ros-$ROS_DISTRO-urdf ros-$ROS_DISTRO-xacro \
    ros-$ROS_DISTRO-compressed-image-transport ros-$ROS_DISTRO-rqt* \
    ros-$ROS_DISTRO-gmapping ros-$ROS_DISTRO-navigation ros-$ROS_DISTRO-interactive-markers

RUN apt install -y \
    ros-$ROS_DISTRO-turtlebot3-msgs ros-$ROS_DISTRO-turtlebot3

RUN echo "source /opt/ros/melodic/setup.bash" >> /root/.bashrc
RUN echo 'PS1="[\[\e[1;36m\]\w\[\e[0m\]]\[\e[1;31m\]>\[\e[0m\] "' >> /root/.bashrc
RUN echo "export TURTLEBOT3_MODEL=burger" >> /root/.bashrc
WORKDIR /root/catkin_ws

CMD ["/bin/bash"]
