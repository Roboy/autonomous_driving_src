This ROS package has several main functions:

1. It defines two models: a simple 4-link tricycle model and a more complicated rickshaw model
2. Both models are ready to be simulated in Gazebo
3. Both model expose APIs and can be controlled by sending commands to `/cmd_vel` topic
4. Visualization is done in RVIZ

## Usage 

1. `roslaunch roboy_models rickshaw.launch` 
2. `roslaunch navigation roboy_nav.launch`

Note: commands should be executed in the order specified. 

After that you can control the model in RVIZ by selecting __2D Nav Goal__  in the toolbar and placing it to the desired location on the map. 
