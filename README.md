# Automated Driving `/src/`
`/src/` directory of `catkin_ws` for Roboys' automated driving project. Note that this purely is a conglomeration of code that by no means enables true autonomy. 

# Getting ready to go

## Cloning
Clone this repository to your catkin workspace's source directory by running the following command (notice the dot in the end!).
```
git clone https://github.com/Roboy/autonomous_driving_src.git .
```

## Things to do before building

### Install...
... Map Server:
```
sudo apt-get install ros-kinetic-map-server
```
... LIDAR:
```
sudo apt-get install ros-kinetic-sick-scan
```
... TF2
```
sudo apt-get install ros-kinetic-geometry2
```
... for obstacle_detector:
```
sudo apt-get install libarmadillo-dev
```
... for communication messages:
```
sudo apt-get install ros-kinetic-moveit-msgs
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
