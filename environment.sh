#! /bin/bash

ROS_MASTER_URI=http://127.0.0.1:11311
ROS_IP=127.0.0.1

if [ ! -z "$1" ]; then
    ROS_MASTER_URI=http://$1:11311
fi

if [ ! -z "$2" ]; then
    ROS_IP=$2
fi

source /opt/ros/noetic/setup.bash
source $HOME/dt-kv260/catkin_ws/devel/setup.bash

echo "ROS_MASTER $ROS_MASTER_URI"
echo "ROS_IP $ROS_IP"
echo "ros1 environment"
