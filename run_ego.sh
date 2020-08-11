#!/bin/bash

echo -e "\033[42;37m\nInstall Armadillo\033[0m";
sudo apt-get install libarmadillo-dev;

echo -e "\033[42;37m\nExecuting catkin_make\033[0m";
catkin_make -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=Yes;

echo -e "\033[42;37m\nEGO-Planner starts\033[0m";
source ./devel/setup.bash & sleep 1.0;
roslaunch ego_planner rviz.launch & sleep 5;
roslaunch ego_planner run_in_sim.launch;
wait;

rosnode kill -a;


