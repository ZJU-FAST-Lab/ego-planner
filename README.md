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

# Test with px4 SITL
- PX4 firmware is required : https://github.com/PX4/PX4-Autopilot
- gazebo_models (for bayland) : https://github.com/osrf/gazebo_models

### build sim_ws
```
cd sim_ws
catkin_make
source devel/setup.bash
```
- If the error occurs: ```Resource not found: px4```, override ROS_PACKAGE_PATH.
- *** Check your path of ```Firmware``` ***
```
Firmware=~/Firmware
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:${Firmware}:${Firmware}/Tools/sitl_gazebo
```
- Check GAZEBO_MODEL_PATH
```
export GAZEBO_MODEL_PATH=~/gazebo_models:~/ego-planner/sim_ws/src/simulation_environment/sim/models
```

### Simulation!
```
### shell 1
roslaunch simulation_environment depth_camera.launch
### shell 2
roslaunch ego_planner run_in_px4sim.launch
### shell 3
roslaunch rc_demo rc_demo.launch
```

# rc_demo
left : throttle & yawrate
right : pitch & roll
