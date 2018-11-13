# autonomous_driving_src
/src/ folder of catkin_ws for Roboys autonomous driving project

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

