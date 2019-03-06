# Automated Driving `/src/`
`/src/` directory of `catkin_ws` for Roboys' automated driving project. It consists of a conglomeration of code enabling us to achieve our next goal of Roboy riding his rickshaw from the subway station to UTUM. This ranges from from sensor setup for environmental perception to mapping, navigation and finally control output. For more information, [visit Roboy.org](https://roboy.org).

To build, follow the instructions below or create a Docker as described [here](https://github.com/Roboy/autonomous_driving/tree/devel/dockers).

## Clone
Clone this repository to your catkin workspace's source directory by running the following command (notice the dot in the end!).
```
git clone https://github.com/Roboy/autonomous_driving_src.git .
```

## ToDo

### Install Packages

To simply install all packages listed below run
```
sudo ./package_requirements.sh
```

### Prepare Git Submodules
```
git submodule init
git submodule update
```

## Build
After you completed all of the above steps, run
```
catkin build roboy_ad
```
