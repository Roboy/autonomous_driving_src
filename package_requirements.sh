#!/bin/sh
apt-get update  # To get the latest package lists
apt-get install ros-kinetic-map-server ros-kinetic-sick-scan python-wstool python-rosdep ninja-build ros-kinetic-abseil-cpp ros-kinetic-geometry2 libarmadillo-dev ros-kinetic-moveit-msgs librealsense2-dkms librealsense2-utils librealsense2-dev ros-kinetic-rviz librealsense2-dbg ros-kinetic-cv-bridge ros-kinetic-rgbd-launch libgazebo7-dev ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control ros-kinetic-ros-controllers -y
#etc.
