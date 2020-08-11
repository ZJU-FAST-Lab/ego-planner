echo -e "\033[42;37m\ninstalling required softwares\033[0m"
sudo apt-get install libarmadillo-dev

echo -e "\033[42;37m\nexecuting catkin_make\033[0m"
catkin_make

echo -e "\033[42;37m\nEGO-Planner starts\033[0m"
source devel/setup.bash & sleep 0.1;
roslaunch ego_planner rviz.launch & sleep 5;
roslaunch ego_planner run_in_sim.launch;
wait;


