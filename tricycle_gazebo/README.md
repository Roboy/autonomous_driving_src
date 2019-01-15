## Usage

To start gazebo with pre-loaded tricycle model run `roslaunch tricycle_gazebo tricycle.launch
`
After this you can control the model over the [cmd_vel](http://wiki.ros.org/Robots/TIAGo/Tutorials/motions/cmd_vel) topic or by running the navigation stack. 

To run a simple experiment, run start the navigation stack using `roslaunch 
navigation roboy_nav.launch` and `rosrun navigation custom_sim.py`. Then, in 
RVIZ set a target for the tricycle and verify that it started moving. Then, 
in gazebo window you should be able to see the model moving as well.