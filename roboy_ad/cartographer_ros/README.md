# Purpose

[Cartographer](https://github.com/googlecartographer/cartographer) is a system that provides real-time simultaneous localization
and mapping ([SLAM](https://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping) in 2D and 3D across multiple platforms and sensor
configurations. This project provides Cartographer's ROS integration.

.. _SLAM: 

# Getting started
Learn to use Cartographer with ROS at the [Read the Docs site](https://google-cartographer-ros.readthedocs.io). Install cartographer by executing
```
sudo apt-get install ros-kinetic-cartographer*
```
PDF containing [Google Cartographer_ROS documentation](https://media.readthedocs.org/pdf/google-cartographer-ros/latest/google-cartographer-ros.pdf)

To runt he examples here, you need to get Roboys Lidar Recordings from [here](https://drive.google.com/drive/folders/1ZM3ox1b3obriWD1hJtNl5FpDvfjspb3m)

# Some useful syntax

## Record a  `.bag`-file

Create Roboys own bag [like here](https://google-cartographer-ros.readthedocs.io/en/latest/your_bag.html)

## Check your `.bag`-file

[ROS Bag files](http://wiki.ros.org/Bags)
```
rosrun cartographer_ros cartographer_rosbag_validate -bag_filename your_bag.bag
```

## Cut `.bag`-files

Cut certain timeframe from `.bag`-file: 

```
rosbag filter Input.bag Output.bag "t.secs>= 1461760303 and t.secs <= 1461760503"
````

## Run Cartographer online
According files for Roboy are defined. To test with Roboy's bag run
```
roslaunch cartographer_ros roboy_indoor_online.launch 
rosbag play ${HOME}/data/utum/utum_groundfloor_cw.bag
```

## Run Cartographer offline on a  `.bag`-file

According files for Roboy are defined. To test with Roboys bag run::

```
roslaunch cartographer_ros roboy_indoor_offline.launch bag_filenames:=${HOME}/data/utum/utum_groundfloor_cw.bag
```

## Save a Map 

Instances given for saving a map after SLAM has finished to [generate a .pbstream-file](https://github.com/googlecartographer/cartographer_ros/blob/master/docs/source/assets_writer.rst) and then converting it to a ROS `.yaml` map file  

```
Finish the first trajectory. No further data will be accepted on it.
rosservice call /finish_trajectory 0

# Ask Cartographer to serialize its current state.
# (press tab to quickly expand the parameter syntax)
rosservice call /write_state "{filename: '${HOME}/Downloads/b3-2016-04-05-14-14-00.bag.pbstream', include_unfinished_submaps: 'true'}"

Visualize `.pbstream`-file::

roslaunch cartographer_ros visualize_pbstream.launch pbstream_filename:=${HOME}/Downloads/DeuMu.bag.pbstream

Convert  `.pbstream`-file to `.yaml` map file::

rosrun cartographer_ros cartographer_pbstream_to_ros_map -pbstream_filename ${HOME}/Downloads/DeuMu.bag.pbstream
```

## Pure Localization

Launch cartographer_ros and provide it with the `.pbstream`-file saved from a previous offline-run with SLAM:
```
roslaunch cartographer_ros roboy_localization.launch load_state_filename:=${HOME}/Downloads/DeuMu.bag.pbstream
```
Play a `.bag`-file faking the live location of the robot:
```
rosbag play ${HOME}/Downloads/b2-2016-04-05-14-44-52.bag
```

# Structure

## Launch Files

`.launch`-files of cartographer_ros are located at `src/cartographer_ros/cartographer_ros/launch`_. Make sure you call the according `roboy` files in your launch file. Also, for the SICK LIDAR note `this github issue`_.

.. _src/cartographer_ros/cartographer_ros/launch: https://github.com/Roboy/cartographer_ros/tree/c4a82825c947e6853b1fc0132a6c53e486d7a63a/cartographer_ros/launch
.. _this github issue: https://github.com/SICKAG/sick_scan/issues/5

## Configuration Files

Configuration is stored in  `.lua`-files located at `src/cartographer_ros/cartographer_ros/configuration`_. `How to use them in cartographer.` 

.. _src/cartographer_ros/cartographer_ros/configuration: https://github.com/Roboy/cartographer_ros/tree/c4a82825c947e6853b1fc0132a6c53e486d7a63a/cartographer_ros/configuration_files
.. _How to use them in cartographer.: https://google-cartographer-ros.readthedocs.io/en/latest/configuration.html

## URDF Files

`urdf`-files essentially define the physical configuration of the robot such as relative positions of different sensors. More can be found in the `ROS wiki about urdf`_ .
In cartographer_ros, these are located at `src/cartographer_ros/cartographer_ros/urdf`_.

.. _ROS wiki about urdf: http://wiki.ros.org/urdf
.. _src/cartographer_ros/cartographer_ros/urdf: https://github.com/Roboy/cartographer_ros/tree/c4a82825c947e6853b1fc0132a6c53e486d7a63a/cartographer_ros/urdf

# Roboy

There are online, offline and localization scripts for Roboy so far.
