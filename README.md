
# PhastaPromp Stack

This repository contains the packages necessary to learn and execute Phasestatemachine + ProMP beahviors

The main entry point is the phastapromep package, which provides a set of launch files for recording demonstrations, learning promps and state graphs, and executing the learnt behaviors in simulation or on  the robot

## Setup

Create catkin ws, checkout repo into src, run catkin_make, source setup.bash.

Install ros dependencies

    rosdep --ignore-src install phastapromep

apt-get install python-kivy


Install additional python dependencies 
    
    For Ubuntu 18.04, you need to install a newer scipy which provides scipy.spatial.transform, which pulls in numpy>1.15 which requires pytables>1.5.0 which is not in the version requirements of numpy:

    pip install scipy>=1.2.0 tables>=1.5.0
    

## Tell git to update subgits automatically
You probably want to switch on automatic subgit-pulling for the phastapromep git:

    git config --local submodule.recurse true

Or even set this as default behavior for all your git repos, to avoid having subgits run out of sync:

    git config --global submodule.recurse true


## Dependencies for running behaviors

### Panda controller messages
    git clone --recursive git@gitlab.tubit.tu-berlin.de:mti-engage/panda_msgs_mti.git

### ROS descriptions of the panda robot from franka_ros (for viszualization)
    git clone --recursive https://github.com/frankaemika/franka_ros.git
Catkin WS won't build without libfranka (see below)

## Dependencies for running behaviors on the real robot:

### panda_mti stack 
    git clone --recursive git@gitlab.tubit.tu-berlin.de:mti-engage/panda_mti.git
    
    
### libfranka
    git clone https://github.com/raphaeldeimel/libfranka.git
    (system-wide installation)

### Python wrapper for libfranka:
    git clone --recursive git@gitlab.tubit.tu-berlin.de:mti-engage/python-libfranka.git
    (system-wide installation)
