# autonomous_driving_src
/src/ folder of catkin_ws for Roboys autonomous driving project

## Cloning
Clone this repository to your catkin workspace's source folder by running the following command (notice the dot in the end!)
```
git clone https://github.com/Roboy/autonomous_driving_src.git .
```

## Things to do before building

### Git Submodules
```
git submodule init
git submodule update
```

### Fix detached Heads & Switching Branches
- cartographer_ros
```
git checkout roboy
```
- geometry2
```
git pull origin indigo-devel
git checkout indigo-devel
```
- pointcloud_to_laserscan 
```
git pull origin lunar-devel
git checkout indigo-devel
```

### Compiling Cartographer
**leave your src folder and go to your catkin_ws first**
Note the changed paths to Roboys cartographer fork:
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
```
catkin build
```


## Included Modules

#### sick_scan
[Sick Scan](http://wiki.ros.org/sick_scan) is the ROS-package provided by the manufacturer of the LiDAR. 
```roslaunch sick_scan sick_mrs_6xxx.launch```
Required to set the according LIDAR IP adress (192.168.1.24)
Double Check through browser 

#### Geometry2
[Geometry2](http://wiki.ros.org/geometry2) is a metapackage to bring in the default packages second generation Transform Library in ROS. Make sure you get the version for kinetic when building (Switch branches!).
In case you run into issues building geometry2 
- test_tf2 apparently doesnt work when building with catkin_make_isolated
- tf2_ ... requires a sudo apt-get install
- it is recommended to build with catkin build

#### pointcloud to laserscan
[Pointcloud_to_Laserscan](http://wiki.ros.org/pointcloud_to_laserscan) converts a 3D Point Cloud into a 2D laser scan. Make sure you get the version for kinetic before building (Switch branches!).

Make sure you set the Params in the launch-file located at
```
src/pointcloud_to_laserscan/launch/sample_node.launch
```
accordingly. Especially, note to LIDAR-dependent set the parameters in lines 20 to 22 in radiants. 
```
angle_min: -1.047
angle_max: 1.047
angle_increment: 0.002268928 
```
Furthermore, make sure to [remove lines 7 to 10](https://github.com/ros-perception/pointcloud_to_laserscan/blob/1f4e90539e4d2c3d05b8dfe022d03008f322d37b/launch/sample_node.launch#L7-L10)
```
<!-- start sensor-->
<include file="$(find openni2_launch)/launch/openni2.launch">
<arg name="camera" default="$(arg camera)"/>
</include>
```
Afterwards, launch by executing
```
roslaunch pointcloud_to_laserscan sample_node.launch
```
#### Google Cartographer ROS

According files for Roboy are defined. To test with demo bag run:
```roslaunch cartographer_ros roboy_indoor.launch bag_filename:=${HOME}/Downloads/cartographer_paper_deutsches_museum.bag```


##### Launch Files
- located at `src/cartographer_ros/cartographer_ros/launch`
- add call for sickscan so we don't have to do it manually`. See here:
```https://github.com/SICKAG/sick_scan/issues/5```
- adapt names to point to according urdf file

##### Configuration Files
- located at `src/cartographer_ros/cartographer_ros/configuration`
- `.lua` files
- compare dokumentation

##### URDF Files
- located at `src/cartographer_ros/cartographer_ros/urdf`
`urdf`-files essentially define the physical configuration of the robot such as relative positions of different sensors. More can be found ´here ROS wiki urdf´

