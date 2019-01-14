A customized [ROS Navigation Stack](http://wiki.ros.org/navigation) for Rickshaw control.

## Background

**/map** - world frame
**tricycle_frame** - frame of the tricycle

## Running

To start navigation stack run `roslaunch navigation roboy_nav.launch`
To run a custom simulation of Rickshaw movement, run `rosrun navigation 
custom_sim.py`

To run a simulation in gazebo, run `roslaunch tricycle_gazebo tricycle.launch`