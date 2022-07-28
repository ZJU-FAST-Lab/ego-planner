#!/bin/bash
#
# Docker build image, run container, execute last container.
#
# - Author: Jongkuk Lim
# - Contact: limjk@jmarple.ai

xhost +

ORG=jmarpledev

PRJ_NAME=${PWD##*/}
PRJ_NAME="$(tr [A-Z] [a-z] <<< "$PRJ_NAME")"
# PRJ_NAME=lio-algorithm
ARCH=$(uname -m)

DOCKER_TAG=$ORG/$PRJ_NAME:$ARCH

CMD_ARGS=( ${@}  )
CMD_ARGS=${CMD_ARGS[*]:1}

if [ "$ARCH" = "aarch64" ]; then
    DOCKER_FILE=./docker/Dockerfile.aarch64
else
    DOCKER_FILE=./docker/Dockerfile
fi

RUN_SHELL=/bin/bash

if [[ $2 == :* ]]; then
    DOCKER_TAG=$DOCKER_TAG$2
    CMD_ARGS=${CMD_ARGS[*]:2}
elif [ "$2" = "bash" ]; then
    RUN_SHELL=/bin/bash
    CMD_ARGS=${CMD_ARGS[*]:2}
elif [ "$2" = "nosh" ]; then
    # TODO(jeikeilim): Currently this does not work.
    # RUN_SHELL="/bin/bash -lic \"${CMD_ARGS[*]:2}\""
    # Currently no space only supported
    RUN_SHELL="/bin/bash -lic ${CMD_ARGS[*]:2}"
    CMD_ARGS=''
fi

if [ "$1" = "build" ]; then
    # Build docker
    echo "Build docker [dockerfile: $DOCKER_FILE] [docker tag: $DOCKER_TAG]"
    docker build . -t $DOCKER_TAG -f $DOCKER_FILE --build-arg CATKIN_REBUILD=$(date +"%s") $CMD_ARGS --build-arg UID=`id -u` --build-arg GID=`id -g`
elif [ "$1" = "run" ]; then
    if [[ $CMD_ARGS == "/bin/zsh" ]] || [[ $CMD_ARGS == "/bin/bash" ]]; then
        DOCKER_OPT="-tid"
    else
        DOCKER_OPT="-ti"
    fi

    # Add gpu option if exist
	if [[ -f /proc/driver/nvidia/version ]]; then
        CMD_ARGS="$CMD_ARGS --gpus all"
    fi

    # TODO(ulken94): The LIO-Livox should not be mounted for aarch64.
    if [ "$ARCH" = "aarch64" ]; then
        docker run $DOCKER_OPT --privileged \
            -e DISPLAY=${DISPLAY} \
            -e TERM=xterm-256color \
            -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
            -v /dev:/dev \
            -v $PWD/src:/home/user/catkin_ws/src \
            --network host \
            $CMD_ARGS \
            $DOCKER_TAG \
            $RUN_SHELL
    else
        docker run $DOCKER_OPT --privileged \
            -e DISPLAY=${DISPLAY} \
            -e TERM=xterm-256color \
            -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
            -v /dev:/dev \
            -v $PWD/catkin_ws/src:/home/user/catkin_ws/src \
            -v $PWD/res:/home/user/res \
            --network host \
            $CMD_ARGS \
            $DOCKER_TAG \
            $RUN_SHELL
    fi

    last_cont_id=$(docker ps -qn 1)
    echo $(docker ps -qn 1) > $PWD/.last_exec_cont_id.txt

    if [[ $CMD_ARGS == "/bin/zsh" ]] || [[ $CMD_ARGS == "/bin/bash" ]]; then
        docker exec -ti $last_cont_id $RUN_SHELL
    fi
elif [ "$1" = "exec" ]; then
    last_cont_id=$(tail -1 $PWD/.last_exec_cont_id.txt)
    docker start ${last_cont_id}
    docker exec -ti ${last_cont_id} $RUN_SHELL
else
    echo ""
    echo "============= $0 [Usages] ============"
    echo "1) $0 build - build docker image"
    echo "      build --no-cache : Build docker image without cache"
    echo "2) $0 run - launch a new docker container"
    echo "      run :TAG_NAME - Run new docker container of TAG_NAME"
    echo "      run bash - Run new docker container with bash"
    echo "      run nosh COMMAND - Run instant container with COMMAND"
    echo "3) $0 exec - execute last container launched"
fi
