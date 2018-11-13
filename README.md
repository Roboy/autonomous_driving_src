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

### Fix detached heads
running `git status` in a submodule will tell you the head is detached. fix by `git pull origin <branch>`. `branch` refers to either `master` or `indigo-devel`

### Switching branches
- cartographer_ros to `roboy`
- geometry2 to `indigo_devel`
- pointcloud_to_laserscan to `indigo_devel`

### Compiling cartographer
**leave your src folder and go to your catkin_ws first**
```
wstool init src
wstool merge -t src https://raw.githubusercontent.com/Roboy/cartographer_ros/master/cartographer_ros.rosinstall
wstool merge -t src https://raw.githubusercontent.com/googlecartographer/cartographer_ros/master/cartographer_ros.rosinstall

wstool update -t src

```
