#!/bin/bash
#
# requires ROS_DATA to be set to ${PWD}!
if [ -z ${ROS_DATA+x} ]; then 
    echo "ERROR: \$ROS_DATA is not set --  please add:"
    echo "       alias roslaunch='ROS_DATA=${PWD}' roslaunch"
    echo "       to .bash_aliases!"
    exit 1
fi

if [ ! -z "$(catkin locate 2>/dev/null)"  ]; then
    echo "ERROR: You attempted to initialize a data directory within a catkin workspace!"
    echo "       You probably did this by accident. "
    exit 2
fi

BASEDIR=$(dirname "$0")
cp -n ${BASEDIR}/../ros_data_skeleton/* "${ROS_DATA}"
cp -n ${BASEDIR}/../readme.md "${ROS_DATA}"

