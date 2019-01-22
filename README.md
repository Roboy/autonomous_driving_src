# Automated Driving `/src/`
`/src/` directory of `catkin_ws` for Roboys' automated driving project. It consists of a conglomeration of code enabling us to achieve our next goal of Roboy riding his rickshaw from the subway station to UTUM. This ranges from from sensor setup for environmental perception to mapping, navigation and finally control output.

To build, follow the instructions below or create a Docker as described [here](https://github.com/Roboy/autonomous_driving/tree/devel/dockers).

# Getting ready to go

## Cloning
Clone this repository to your catkin workspace's source directory by running the following command (notice the dot in the end!).
```
git clone https://github.com/Roboy/autonomous_driving_src.git .
```

## Things to do before building

### Install...

To simply install all packages listed below run
```
sudo ./package_requirements.sh
```
Otherwise, go through this list manually:

#### ... Map Server:
```
sudo apt-get install ros-kinetic-map-server
```
#### ... LIDAR:
```
sudo apt-get install ros-kinetic-sick-scan
```
#### ... Cartographer
```
sudo apt-get install -y python-wstool python-rosdep ninja-build
sudo apt-get install ros-kinetic-abseil-cpp
```
#### ... TF2
```
sudo apt-get install ros-kinetic-geometry2
```
#### ... for obstacle_detector:
```
sudo apt-get install libarmadillo-dev
sudo apt-get install ros-kinetic-rviz # only needed if you install ROS-Base or build from docker image
```
#### ... for communication messages:
```
sudo apt-get install ros-kinetic-moveit-msgs
```
#### ... for Intel Realsense Camera (steps copied from [official documentation](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages)):
```
sudo apt-key adv --keyserver keys.gnupg.net --recv-key C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C8B3A55A6F3EFCDE
sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main" -u
sudo rm -f /etc/apt/sources.list.d/realsense-public.list
sudo apt-get update
sudo apt-get install librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg
sudo apt-get install ros-kinetic-rgbd-launch
```

#### ... for gazebo
```
sudo apt-get install libgazebo7-dev ros-kinetic-gazebo-ros-pkgs 
ros-kinetic-gazebo-ros-control ros-kinetic-ros-controllers
```
### Git Submodules
```
git submodule init
git submodule update
```
### Compiling Cartographer_ROS
**Before compiling, leave your src directory and go to your catkin_ws**

Compiling Cartographer_ROS works similar to the [Cartographer_ROS documentation](https://google-cartographer-ros.readthedocs.io/en/latest/compilation.html) but some paths differ due to Roboy using its own fork of Cartographer_ROS.
```
wstool init src
wstool merge -t src https://raw.githubusercontent.com/Roboy/cartographer_ros/roboy/cartographer_ros.rosinstall
wstool update -t src
```

```
src/cartographer/scripts/install_proto3.sh
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y
```
To be able to run pure localization, the following step is essential:
```
cd src/cartographer/
git checkout master
```

## Building
After you completed all of the above steps, run
```
catkin build
```

# HOW-TO

## Google Cartographer_ROS
[Cartographer](https://github.com/googlecartographer/cartographer) is a system that provides real-time simultaneous localization and mapping [SLAM](https://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping) in 2D and 3D across multiple platforms and sensor configurations. This project provides Cartographer's ROS integration.

## Geometry2
[Geometry2](http://wiki.ros.org/geometry2) is a metapackage to bring in the default packages second generation Transform Library in ROS. Make sure you get the version for kinetic when building (Switch branches!).

## Obstacle_Detector
[Obstacle Detector](https://github.com/tysik/obstacle_detector) is a ROS package for 2D obstacle detection based on laser range data.

## Intel Realsense Camera
[Intel(R) RealSense(TM) ROS Wrapper](https://github.com/intel-ros/realsense) for D400 series and SR300 Camera http://wiki.ros.org/RealSense

Follow *Usage Instructions* in provided link for first steps.

## Sick_Scan
[Sick Scan](http://wiki.ros.org/sick_scan) is the ROS-package provided by the manufacturer of the LiDAR. Before launching the according file, it is required to set the LIDAR IP adress accordingly (i.e. 192.168.0.42). Alternatively you can provide the parameter as an argument with roslaunch.
```
roslaunch roboy_ad sick_lms_155.launch -use_binary_protocol
```


# FAQ

Q: My `catkin build` was successfull at first but when I execute it again there is an error for `cartographer_ros`, `cartogarpher` or it even wants some `Abseil-function`.

A: You need to compile cartographer_ros again. First, do `catkin clean` and delete the following directories: `catkin_ws/src/cartographer` `catkin_ws/src/ceres-solver` and `catkin_ws/protobuf`. Then follow the steps to compile cartographer_ros.


Q: `roslaunch` command not found

A: run `source /devel/setup.bash` in your catkin directory. 
