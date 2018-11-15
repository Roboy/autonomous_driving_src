# Automated Driving `/src/`
`/src/` folder of catkin_ws for Roboys' automated driving project. Note that this purely is a conglomeration of code that by no means enables true autonomy. 

## Cloning
Clone this repository to your catkin workspace's source folder by running the following command (notice the dot in the end!).
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
- sick_scan
```
git checkout master
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

### Compiling Cartographer_ROS
**Before compiling, leave your src folder and go to your catkin_ws**

Works similar to the [Cartographer_ROS documentation](https://google-cartographer-ros.readthedocs.io/en/latest/compilation.html) but with some changed paths due to Roboy using its own fork of Cartographer_ROS.
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
After all of this is done, run
```
catkin build
```


## Included Modules

### Google Cartographer_ROS
Create Roboys own bag [like here](https://google-cartographer-ros.readthedocs.io/en/latest/your_bag.html).
According files for Roboy are defined. To test with Roboys bag run:
```
roslaunch cartographer_ros roboy_indoor.launch bag_filename:=${HOME}/Downloads/cartographer_paper_deutsches_museum.bag
```
#### Launch Files
`.launch` files of cartographer_ros are located at [`src/cartographer_ros/cartographer_ros/launch`](https://github.com/Roboy/cartographer_ros/tree/c4a82825c947e6853b1fc0132a6c53e486d7a63a/cartographer_ros/launch). Make sure you call the according `roboy` files in your launch file. Also, for the SICK LIDAR note [this github issue](https://github.com/SICKAG/sick_scan/issues/5).

#### Configuration Files
- located at [`src/cartographer_ros/cartographer_ros/configuration`](https://github.com/Roboy/cartographer_ros/tree/c4a82825c947e6853b1fc0132a6c53e486d7a63a/cartographer_ros/configuration_files)
- `.lua` files
- compare dokumentation

#### URDF Files
`urdf`-files essentially define the physical configuration of the robot such as relative positions of different sensors. More can be found in the [ROS wiki about urdf](http://wiki.ros.org/urdf).
In cartographer_ros, these are located at [`src/cartographer_ros/cartographer_ros/urdf`](https://github.com/Roboy/cartographer_ros/tree/c4a82825c947e6853b1fc0132a6c53e486d7a63a/cartographer_ros/urdf)



### sick_scan
[Sick Scan](http://wiki.ros.org/sick_scan) is the ROS-package provided by the manufacturer of the LiDAR. 
```
roslaunch sick_scan sick_mrs_6xxx.launch
```
It is required to set the LIDAR IP adress accordingly (192.168.1.24) which can be easily verified through entering it in a browser. 

### Geometry2
[Geometry2](http://wiki.ros.org/geometry2) is a metapackage to bring in the default packages second generation Transform Library in ROS. Make sure you get the version for kinetic when building (Switch branches!).

### pointcloud to laserscan
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


