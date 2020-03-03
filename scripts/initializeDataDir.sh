#!/bin/bash
#
# requires ROS_DATA to be set to ${PWD}!
if [ -z ${ROS_DATA+x} ]; then 
    echo "\$ROS_DATA is not set --  please add:"
    echo "     alias roslaunch='ROS_DATA=${PWD}' roslaunch"
    echo "to .bash_aliases!"
    exit 1
else
    cp -n ../ros_data_skeleton/* "${ROS_DATA}"
    cp -n ../readme.md "${ROS_DATA}"
fi
