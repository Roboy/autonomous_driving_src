# IMU 

This is to convert the [custom ROS message](http://docs.ros.org/api/sbg_driver/html/msg/SbgImuData.html) of a [SBG Systems Ellipse2-A](https://www.sbg-systems.com/products/ellipse-2-series/) IMU unit to a standard [ROS Kinetic IMU sensor message](http://docs.ros.org/kinetic/api/sensor_msgs/html/msg/Imu.html) for the use in [Google Cartographer](https://github.com/Roboy/cartographer_ros/tree/roboy).

## Setup
### Install
```
sudo apt-get install ros-kinetic-sbg-driver
```
Add your session `username`to the dialout group and **reboot your machine**.
```
sudo adduser username dialout
```
Clone this repository into your catkin/src folder and build using
```
catkin build imu
source devel/setup.bash
```

## Run
Launch IMU script:
```
roslaunch imu imu_converter.launch
```


