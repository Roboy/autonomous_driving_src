This ROS package has several main functions:

1. It defines two models: a simple 4-link tricycle model and a more complicated rickshaw model
2. Both models are ready to be simulated in Gazebo
3. Both model expose APIs and can be controlled by sending commands to `/cmd_vel` topic
4. Visualization is done in RVIZ

## Usage 

Run `roslaunch roboy_models rickshaw.launch` and `roslaunch navigation 
roboy_nav.launch`. Now you can control the model in RVIZ
