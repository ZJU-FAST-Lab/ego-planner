#!/bin/bash
#
# Start lidar companion processes
#
# - Author: Jongkuk Lim
# - Contact: limjk@jmarple.ai

ORG=jmarpledev

PRJ_DIR=$(dirname $PWD)
PRJ_NAME=${PRJ_DIR##*/}
PRJ_NAME="$(tr [A-Z] [a-z] <<< "$PRJ_NAME")"
ARCH=$(uname -m)

DOCKER_TAG=$ORG/$PRJ_NAME:$ARCH

# Wait for roscore to be launched by lidar_companion.service
sleep 30;

echo "Running docker ..."
docker run -t --network host --privileged -v $PRJ_DIR/src:/home/user/catkin_ws/src $DOCKER_TAG /bin/bash -lic "source /home/user/.bashrc && roslaunch ego_planner run_in_flight.launch"

