# IMU 

## Hardware
SBG Systems Ellipse2-A

## Setup
### Install
```
sudo apt-get install ros-kinetic-sbg-driver
```

Add your `username`to the dialout group and reboot your machine.
```
sudo adduser username dialout
```

Launch IMU script:
```
roslaunch imu imu_converter.launch
```


