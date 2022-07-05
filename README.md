This repository is forked from https://github.com/ZJU-FAST-Lab/ego-planner.

# build
```
sudo apt-get install libarmadillo-dev
cd ego-planner
catkin_make
source devel/setup.bash
```

# Test with rosbag
- rosbag file path : ```jmarple-nas://Projects/2022_LIO/rosbag/livox_avia/2022_0616_LidarFlight_water_garden/filtered_05_yongin_bando.bag```
- edited faster-lio : https://github.com/j-marple-dev/LIO-Algorithm/tree/test/ego-planner

### in Docker of LIO-Algorithm
*** Check your Internet(ROS) settings ***
```
### shell 1
roslaunch faster_lio mapping_avia.launch
### shell 2
rosbag play filtered_05_yongin_bando.bag
```
### in j-marple-dev/ego-planner
```
### shell 1
roslaunch ego_planner run_in_bag.launch
### shell 2
roslaunch rc_demo rc_demo.launch
```

# Test with PX4 SITL
- PX4 firmware is required
- gazebo_models (for bayland)
- these are include in https://github.com/j-marple-dev/PX4-container-custom/tree/ego-planner

### in Docker of PX4-container-custom
```
roslaunch simulation_environment ego_depth.launch
```

### in ego-planner installed env
```
### shell 1
roslaunch ego_planner run_in_px4sim.launch
### shell 2
roslaunch rc_demo rc_demo.launch
```

## PX4 setting
- EKF2_AID_MASK = 24
- EKF2_HGT_MODE = Vision
- MIS_TAKEOFF_ALT = 1.0

# rc_demo
- enable only mouse-drag
- left : throttle & yawrate
- right : pitch & roll
