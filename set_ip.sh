#! /bin/bash

if [ "$1" ]; then
    echo "ROS MASRER $1"
    export ROS_MASTER_URI=http://$1:11311
else
    export ROS_MASTER_URI=http://127.0.0.1:11311
    echo $ROS_MASTER_URI
fi

if [ "$2" ]; then
    echo "ROS IP $2"
    export ROS_IP=$2
else
    export ROS_IP=127.0.0.1
    echo $ROS_IP
fi
