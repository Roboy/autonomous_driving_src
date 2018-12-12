# Automated Driving `/src/`
`/src/` directory of `catkin_ws` for Roboys' automated driving project. Note that this purely is a conglomeration of code that by no means enables true autonomy. 

# Building

## Cloning
Clone this repository to your catkin workspace's source directory by running the following command (notice the dot in the end!).
```
git clone https://github.com/Roboy/autonomous_driving_src.git .
```

## Things to do before building

### Git Submodules
```
git submodule init
git submodule update
```

### Fixing detached heads & switching branches
- cartographer_ros
```
git checkout roboy
```

- geometry2
```
git pull origin indigo-devel
git checkout indigo-devel
```

- navigation
```
sudo apt-get install ros-kinetic-navigation
```
```
git checkout kinetic-devel
```
- obstacle_detector
```
sudo apt-get install libarmadillo-dev
```

- pointcloud_to_laserscan 
```
git pull origin lunar-devel
git checkout indigo-devel
```
- sick_scan
```
git checkout master
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
### Building
After you completed all of the above steps, run
```
catkin build
```
(This may very well take more than 30 minutes)


# About the Submodules

### Google Cartographer_ROS

#### pointcloud to laserscan recording
`roboy_indoor_offline.launch`, [line 29](https://github.com/Roboy/cartographer_ros/blob/55defd7b8d6be13b5f1b2d2205e842b1b016661c/cartographer_ros/launch/roboy_indoor_offline.launch#L29-L30)
```
<remap from="scan" to="fake/scan" />
```
add the following lines to the `.launch` file:
```
<node pkg="tf" type="static_transform_publisher" name="world_to_map_broadcaster" args="0 0 0 0 0 0 world map 50" />
<node pkg="tf" type="static_transform_publisher" name="base_to_laser_broadcaster" args="0 0 0 0 0 0 base_link laser 50" />
```

`roboy.lua`, [lines 30 and 31](https://github.com/Roboy/cartographer_ros/blob/c4a82825c947e6853b1fc0132a6c53e486d7a63a/cartographer_ros/configuration_files/roboy.lua#L30-L31):
```
num_laser_scans = 1,
num_multi_echo_laser_scans = 0,
```
Finally, to run execute
```
roslaunch cartographer_ros roboy_indoor_offline.launch bag_filename:=${HOME}/Documents/Roboy/catkin_ws/2018-11-15-17-36-28.bag
```

##### Deutsches Museum 2D

`roboy_indoor_offline.launch`, [line 29](https://github.com/Roboy/cartographer_ros/blob/55defd7b8d6be13b5f1b2d2205e842b1b016661c/cartographer_ros/launch/roboy_indoor_offline.launch#L29-L30)
```
<remap from="echoes" to="horizontal_laser_2d" />
```

`roboy.lua`, [lines 30 and 31](https://github.com/Roboy/cartographer_ros/blob/c4a82825c947e6853b1fc0132a6c53e486d7a63a/cartographer_ros/configuration_files/roboy.lua#L30-L31):
```
num_laser_scans = 0,
num_multi_echo_laser_scans = 1,
```
Finally, to run execute
```
roslaunch cartographer_ros roboy_indoor_offline.launch bag_filename:=${HOME}/Downloads/cartographer_paper_deutsches_museum.bag
```
### Geometry2
[Geometry2](http://wiki.ros.org/geometry2) is a metapackage to bring in the default packages second generation Transform Library in ROS. Make sure you get the version for kinetic when building (Switch branches!).

### Navigation
includes `map_server`
```
rosrun map_server map_server mymap.yaml

```

### Obstacle_Detector

### sick_scan
[Sick Scan](http://wiki.ros.org/sick_scan) is the ROS-package provided by the manufacturer of the LiDAR. Before launching the according file, it is required to set the LIDAR IP adress accordingly (i.e. 192.168.1.42). 
```
roslaunch sick_scan sick_lms_1xx.launch -use_binary_protocol
```

### pointcloud to laserscan
[Pointcloud_to_Laserscan](http://wiki.ros.org/pointcloud_to_laserscan) converts a 3D Point Cloud into a 2D laser scan. Make sure you get the version for kinetic before building (Switch branches!).

Make sure to [remove lines 7 to 10](https://github.com/ros-perception/pointcloud_to_laserscan/blob/1f4e90539e4d2c3d05b8dfe022d03008f322d37b/launch/sample_node.launch#L7-L10)
```
<!-- start sensor-->
<include file="$(find openni2_launch)/launch/openni2.launch">
<arg name="camera" default="$(arg camera)"/>
</include>
```
Furthermore, change [lines 15 and 16](https://github.com/ros-perception/pointcloud_to_laserscan/blob/ead080498d177c48fa4906a0b6264f60ae69e6ba/launch/sample_node.launch#L15-L16) to
```
<remap from="cloud_in" to="cloud"/>
<remap from="scan" to="/fake/scan"/>
```
Make sure you set the Params in the launch-file located at `src/pointcloud_to_laserscan/launch/sample_node.launch` accordingly. Don't forget to LIDAR-dependently set the parameters in lines 20 to 22 in radiants. 
```
angle_min: -1.047
angle_max: 1.047
angle_increment: 0.002268928 
```
Which leaves the file to be
```
<?xml version="1.0"?>

<launch>

<!-- run pointcloud_to_laserscan node -->
<node pkg="pointcloud_to_laserscan" type="pointcloud_to_laserscan_node" name="pointcloud_to_laserscan">

<remap from="cloud_in" to="cloud"/>
<remap from="scan" to="fake/scan"/>
<rosparam>
# target_frame: camera_link # Leave disabled to output scan in pointcloud frame
transform_tolerance: 0.1
min_height: 0.0
max_height: 4.0

angle_min: -1.0472 # -M_PI/2
angle_max: 1.0472 # M_PI/2
angle_increment: 0.002268928 # M_PI/360.0
scan_time: 0.1
range_min: 0.45
range_max: 70.0
use_inf: true

# Concurrency level, affects number of pointclouds queued for processing and number of threads used
# 0 : Detect number of cores
# 1 : Single threaded
# 2->inf : Parallelism level
concurrency_level: 1
</rosparam>

</node>

</launch>
```
Afterwards, launch by executing
```
roslaunch pointcloud_to_laserscan sample_node.launch
```


## FAQ

Q: My `catkin build` was successfull at first but when I execute it again there is an error for `cartographer_ros`, `cartogarpher` or it even wants some `Abseil-function`.

A: You need to compile cartographer_ros again. First, do `catkin clean` and delete the following directories: `catkin_ws/src/cartographer` `catkin_ws/src/ceres-solver` and `catkin_ws/protobuf`. Then follow the steps to compile cartographer_ros.


Q: `roslaunch` command not found

A: run `source /devel/setup.bash` in your catkin directory. 
