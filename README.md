# Automated Driving `/src/`
`/src/` directory of `catkin_ws` for Roboys' automated driving project. It consists of a conglomeration of code enabling us to achieve our next goal of Roboy riding his rickshaw from the subway station to UTUM. This ranges from from sensor setup for environmental perception to mapping, navigation and finally control output. For more information, [visit Roboy.org](https://roboy.org).

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

### Git Submodules
```
git submodule init
git submodule update
```

## Building
After you completed all of the above steps, run
```
catkin build roboy_ad
```

# HOW-TO

## Obstacle_Detector
[Obstacle Detector](https://github.com/tysik/obstacle_detector) is a ROS package for 2D obstacle detection based on laser range data.

## Intel Realsense Camera
[Intel(R) RealSense(TM) ROS Wrapper](https://github.com/intel-ros/realsense) for D400 series and SR300 Camera http://wiki.ros.org/RealSense

Follow *Usage Instructions* in provided link for first steps.

## Calibration
The submodule [radlocc_calibration](https://github.com/bernardomig/radlocc_calibration) contains a tool that can be used to record camera and lidar data for extrinsic-calibration between camera an lidar. For more info about how to do the calibration visit the [wiki article](https://github.com/Roboy/autonomous_driving/wiki/Calibration:-Extrinsic-calibration-between-camera-and-lidar) of the [Roboy autonomous_driving repository](https://github.com/Roboy/autonomous_driving).

