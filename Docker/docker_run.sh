#!/usr/bin/env bash

ARGS=("$@")

# Make sure processes in the container can connect to the x server
# Necessary so gazebo can create a context for OpenGL rendering (even headless)
XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]; then
    xauth_list=$(xauth nlist $DISPLAY)
    xauth_list=$(sed -e 's/^..../ffff/' <<<"$xauth_list")
    if [ ! -z "$xauth_list" ]; then
        echo "$xauth_list" | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

DOCKER_OPTS=

# Prevent executing "docker run" when xauth failed.
if [ ! -f $XAUTH ]; then
    echo "[$XAUTH] was not properly created. Exiting..."
    exit 1
fi

if [ ! -z "$1" ]; then
    ROS_MASTER_URI=http://$1:11311
    echo "ROS_MASTER $1"
fi

if [ ! -z "$2" ]; then
    ROS_IP=$2
    echo "ROS_IP $2"
fi

BASH_OPTION=bash

docker run \
    -it \
    --rm \
    -e DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e XAUTHORITY=$XAUTH \
    -e ROS_MASTER_URI=$ROS_MASTER_URI \
    -e ROS_IP=$ROS_IP \
    -v "$XAUTH:$XAUTH" \
    -v "/home/$USER/dt-kv260:/home/argnctu/dt-kv260" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix" \
    -v "/etc/localtime:/etc/localtime:ro" \
    -v "/dev:/dev" \
    -v "/var/run/docker.sock:/var/run/docker.sock" \
    --workdir "/home/argnctu/dt-kv260" \
    --user root \
    --name argnctu \
    --network host \
    --privileged \
    --security-opt seccomp=unconfined \
    $DOCKER_OPTS \
    argnctu/dt-kv260:latest \
    $BASH_OPTION
