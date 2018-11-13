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

### Fix detached Heads
running `git status` in a submodule will tell you the head is detached. fix by `git pull origin <branch>`. `branch` refers to either `master` or `indigo-devel`

### Switching branches
- cartographer_ros
```
git checkout roboy
```
- geometry2
```
git pull origin indigo_devel
git checkout indigo_devel
```
- pointcloud_to_laserscan 
```
git pull origin lunar-devel
git checkout indigo_devel
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
