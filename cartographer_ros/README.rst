.. Copyright 2016 The Cartographer Authors

.. Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

..      http://www.apache.org/licenses/LICENSE-2.0

.. Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.

============================
Cartographer ROS Integration
============================

|build| |docs| |license|

Purpose
=======

`Cartographer`_ is a system that provides real-time simultaneous localization
and mapping (`SLAM`_) in 2D and 3D across multiple platforms and sensor
configurations. This project provides Cartographer's ROS integration.

.. _Cartographer: https://github.com/googlecartographer/cartographer
.. _SLAM: https://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping

Getting started
===============

* Learn to use Cartographer with ROS at `our Read the Docs site`_.
* You can ask a question by `creating an issue`_.
* To run the examples shown, you need to get `Roboy's LIDAR recordings from GDrive`_.
* PDF containing `Google Cartographer_ROS documentation`_ .

.. _our Read the Docs site: https://google-cartographer-ros.readthedocs.io
.. _creating an issue: https://github.com/googlecartographer/cartographer_ros/issues/new?labels=question
.. _Roboy's LIDAR recordings from GDrive: https://drive.google.com/drive/folders/1ZM3ox1b3obriWD1hJtNl5FpDvfjspb3m
.. _Google Cartographer_ROS documentation: https://media.readthedocs.org/pdf/google-cartographer-ros/latest/google-cartographer-ros.pdf

Building Cartographer
=====================
::

	sudo apt-get install -y python-wstool python-rosdep ninja-build
	sudo apt-get install ros-kinetic-abseil-cpp

go to /catkin_ws/

::

	wstool init src
	wstool merge -t src https://raw.githubusercontent.com/Roboy/cartographer_ros/roboy/cartographer_ros.rosinstall
	wstool update -t src

::

	src/cartographer/scripts/install_proto3.sh
	sudo rosdep init
	rosdep update
	rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y

::

	catkin build cartographer_rviz


To be able to run Roboys' code::

	cd src/cartographer_ros/
	git checkout roboy


Data
====

Recording
---------
Create Roboys own bag `like here`_.

.. _like here: https://google-cartographer-ros.readthedocs.io/en/latest/your_bag.html

Checking
--------
`ROS Bag files`_ 

.. _ROS Bag files: http://wiki.ros.org/Bags

::

	rosrun cartographer_ros cartographer_rosbag_validate -bag_filename your_bag.bag


Cutting
-------
Cut certain timeframe from `.bag`-file: 

::

	rosbag filter Input.bag Output.bag "t.secs>= 1461760303 and t.secs <= 1461760503"

SLAMing
=======

live ('online')
--------------------------------
According files for Roboy are defined. To test with Roboy's bag run::

	roslaunch cartographer_ros roboy_indoor_online.launch 
	rosbag play ${HOME}/data/utum/utum_groundfloor_cw.bag

Run Cartographer offline
------------------------
According files for Roboy are defined. To test with Roboys bag run::

	roslaunch cartographer_ros roboy_indoor_offline.launch bag_filenames:=${HOME}/data/utum/utum_groundfloor_cw.bag
Map
===

Saving
------
Only needed for live cartographer. Instances given for saving a map after SLAM has finished to `generate a .pbstream-file`_ and then converting it to a ROS `.yaml` map file  

.. _generate a .pbstream-file: https://github.com/googlecartographer/cartographer_ros/blob/master/docs/source/assets_writer.rst

::

	# Finish the first trajectory. No further data will be accepted on it.
	rosservice call /finish_trajectory 0

	# Ask Cartographer to serialize its current state.
	# (press tab to quickly expand the parameter syntax)
	rosservice call /write_state "{filename: '${HOME}/Downloads/b3-2016-04-05-14-14-00.bag.pbstream', include_unfinished_submaps: 'true'}"

Visualize `.pbstream`-file::

	roslaunch cartographer_ros visualize_pbstream.launch pbstream_filename:=${HOME}/Downloads/DeuMu.bag.pbstream

Convert & Publish
-----------------
Convert  `.pbstream`-file to `.yaml` map file::

	rosrun cartographer_ros cartographer_pbstream_to_ros_map -pbstream_filename ${HOME}/Downloads/DeuMu.bag.pbstream

Cartographer publishes the `/map` topic in pure localization mode with the map it loads from the provided `.pbstream` file (compare next section). To publish i.e. an adapted map on `/nav_map` topic, do i.e.::

	rosrun map_server map_server nav_map.yaml /map:=/nav_map /map_metadata:=/nav_map_metadata

Editing
-------
We use GIMP::

	sudo apt-get install gimp 
	
Drag & Drop the `.pgm`-file into gimp. Use i.e. rectangle selection tool and bucket fill tool to mark large areas as not-navigatable. Do rectangle selection and use pencil tool for fine selection. the Export file as `.pgm` file.

Pure Localization
=================
Pure localizations publishes /tf frames between the map origin and the robot's base_link. For these /tf messages, ROS times are essential. This is why there are different procedures for Testing (a.k. from bag file) and live localization. 

Testing & Developing
--------------------
Launch cartographer_ros and provide it with the `.pbstream`-file saved from a previous offline-run with SLAM::

	roslaunch cartographer_ros roboy_mw_test_localization.launch load_state_filename:=${HOME}/data/maps/MW_1.pbstream

Play a `.bag`-file faking the live location of the robot::

	rosbag play ${HOME}/data/2019_03_05/MW_drive.bag --clock

Going Live
----------
Start your Lidar and IMU nodes and run::

	roslaunch cartographer_ros roboy_mw_localization.launch load_state_filename:=${HOME}/data/maps/MW_1.pbstream

Structure
=========
Launch Files
------------
`.launch`-files of cartographer_ros are located at `src/cartographer_ros/cartographer_ros/launch`_. Make sure you call the according `roboy` files in your launch file. Also, for the SICK LIDAR note `this github issue`_.

.. _src/cartographer_ros/cartographer_ros/launch: https://github.com/Roboy/cartographer_ros/tree/c4a82825c947e6853b1fc0132a6c53e486d7a63a/cartographer_ros/launch
.. _this github issue: https://github.com/SICKAG/sick_scan/issues/5

Configuration Files
-------------------
Configuration is stored in  `.lua`-files located at `src/cartographer_ros/cartographer_ros/configuration`_. `How to use them in cartographer.` 

.. _src/cartographer_ros/cartographer_ros/configuration: https://github.com/Roboy/cartographer_ros/tree/c4a82825c947e6853b1fc0132a6c53e486d7a63a/cartographer_ros/configuration_files
.. _How to use them in cartographer.: https://google-cartographer-ros.readthedocs.io/en/latest/configuration.html

URDF Files
----------
`urdf`-files essentially define the physical configuration of the robot such as relative positions of different sensors. More can be found in the `ROS wiki about urdf`_ .
In cartographer_ros, these are located at `src/cartographer_ros/cartographer_ros/urdf`_.

.. _ROS wiki about urdf: http://wiki.ros.org/urdf
.. _src/cartographer_ros/cartographer_ros/urdf: https://github.com/Roboy/cartographer_ros/tree/c4a82825c947e6853b1fc0132a6c53e486d7a63a/cartographer_ros/urdf

Roboy
=====

There are online, offline and localization scripts for Roboy so far.



