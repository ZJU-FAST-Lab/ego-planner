#!/bin/bash

ret=$(ldconfig -p | grep libarmadillo)
if test ${#ret} -gt 1
then
    echo -e "\033[42;37m\nArmadillo is already installed\033[0m";
else
    echo -e "\033[42;37m\nEnter password to install Armadillo\033[0m";
    sudo apt-get install libarmadillo-dev;
fi

echo -e "\033[42;37m\nExecuting catkin_make\033[0m";
catkin_make -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=Yes;

echo -e "\033[42;37m\nEGO-Planner starts\033[0m";
source devel/setup.bash & sleep 0.1;
roslaunch ego_planner rviz.launch & sleep 5;
roslaunch ego_planner run_in_sim.launch;
wait;

rosnode kill -a;


