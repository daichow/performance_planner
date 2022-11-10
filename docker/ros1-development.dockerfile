FROM osrf/ros:noetic-desktop-full

# use bash instead of sh
SHELL ["/bin/bash", "-c"]
# accept the default answer for all questions
ENV DEBIAN_FRONTEND=noninteractive
# added by Alex, not sure how we use this
ENV CONAN_REVISIONS_ENABLED=1

# update system
RUN apt-get update && apt-get upgrade --yes

# get python deps
RUN apt-get update && apt-get -y install python3 python3-pip
RUN pip install \
    conan \
    pyzbar \
    imutils

# get ros deps and git
RUN apt-get update && apt-get -y install \
    ros-noetic-moveit \
    ros-noetic-moveit-kinematics \
    ros-noetic-xacro \
    ros-noetic-ros-controllers \
    ros-noetic-ros-control \
    ros-noetic-octomap \
    ros-noetic-octomap-mapping \
    libzbar0 \
    ros-noetic-rgbd-launch \
    ros-noetic-rosbridge-server \
    ros-noetic-web-video-server \
    ros-noetic-catkin \
    python3-catkin-tools \
    ros-noetic-trac-ik-kinematics-plugin \
    git \
    wget

RUN apt-get update && apt-get -y install \
    gstreamer1.0-tools \
    gstreamer1.0-libav \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer-plugins-good1.0-dev \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-base

RUN mkdir -p /catkin_ws/src
WORKDIR /catkin_ws

# configure conman
RUN conan config set general.revisions_enabled=1 \
    && conan profile new default --detect > /dev/null \
    && conan profile update settings.compiler.libcxx=libstdc++11 default

# download ros kortex 
RUN wget -c https://github.com/Kinovarobotics/ros_kortex/archive/refs/heads/noetic-devel.tar.gz -O - | tar -xz -C /catkin_ws/src
# RUN mv /catkin_ws/src/ros_kortex-noetic-devel /catkin_ws/src/ros_kortex

# download ros kortex vision
# RUN wget -c https://github.com/Kinovarobotics/ros_kortex_vision/archive/refs/tags/1.1.4.tar.gz -O - | tar -xz -C /catkin_ws/src
# RUN mv /catkin_ws/src/ros_kortex_vision-1.1.4 /catkin_ws/src/ros_kortex_vision

#copy ROS packages into container
# COPY  ros_kortex-noetic-devel /catkin_ws/src/ros_kortex
# COPY  tbot_bringup /catkin_ws/src/tbot_bringup
# COPY  tbot_gripper /catkin_ws/src/tbot_gripper
# COPY  tbot_path_planning /catkin_ws/src/tbot_path_planning
# COPY  tbot_scripts /catkin_ws/src/tbot_scripts
# COPY  tbot_vision /catkin_ws/src/tbot_vision

# install ros package dependencies
RUN rosdep install --from-paths src --ignore-src -y

# build
# compile packages
RUN catkin config --extend /opt/ros/$ROS_DISTRO \
    && catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && catkin build

# RUN python3 -m pip install  /catkin_ws/src/tbot_gripper/kortex_api/kortex_api-2.3.0.post34-py3-none-any.whl

# add sourcing catkin_ws to entrypoint
RUN sed --in-place --expression \
    '$isource "/catkin_ws/devel/setup.bash"' \
    /ros_entrypoint.sh

# source packages on entry to bash to ease developement
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
RUN echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc
