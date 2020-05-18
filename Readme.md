
This repository contains the packages necessary to learn and execute Phasestatemachine + ProMP beahviors

The main entry point is the phastapromep package, which provides a set of launch files for recording demonstrations, learning promps and state graphs, and executing the learnt behaviors in simulation or on  the robot

This repository relies on:

https://github.com/raphaeldeimel/panda_mti
https://github.com/raphaeldeimel/python-phasestatemachine
https://github.com/raphaeldeimel/python-promep


# Setup

This version is tested on Ubuntu 20.04 + ROS melodic

For other (especially older) systems, especially make sure to use a recent numpy (>=1.16) and scipy (>=1.2) version. (e.g. via pip)

## Quick guide:

Create catkin ws, make and go to the src/ and do:
'''
git checkout https://github.com/raphaeldeimel/panda_mti
git checkout https://github.com/raphaeldeimel/python-phasestatemachine
git checkout https://github.com/raphaeldeimel/python-promep
git checkout https://github.com/raphaeldeimel/phastapromep

apt-get install python-kivy

catkin build    #install any missing dependencies
source ../devel/setup.bash
'''


# teachingApp Tutorial


In this package you find an example of using the TeachinApp to create a graph of motions from demonstrations on the robot. In general, you follow the folowing work flow:

Create a directory and go there:
`mkdir myfirstexperiment && cd myfirstexperiment` 
Then start there the teaching app from the phastapromp package in this directory:
`roslaunch phastapromep teachingApp.launch`
(This will create a number of stub files if they are not present yet)

Now go, enable the robot arm and demonstrate segments of desired motions/behaviors by physically moving the robot arm

Within teaching app:
1. Record demonstrations / observations with the record button (or using the Enter key)
2. Label each recorded segment to indicate which are instances of the same motion (or which observations to discard)
    - You can replay segments on the robot (Panda's activation button up), or just watch it in Rviz (Panda's activation button pressed)
3. (Optional) Edit the session.yaml file configuring metaparameters of the learning session
4. Generate/Learn the behavior as a set of ProMePs, state controllers, and a connectivity graph for the phase-state machine
    You can either use the "learn behavior" button in the teaching app, or run `rosrun phastapromp learnGraph.py session.yaml`
5. Run the behavior using phastanode (executes the graph), mixernode (governs the motions) and pdcontrolleremulatornode (emulates a pd-controlled panda arm for visualization)
    `roslaunch phastapromep runBehavior.launch`
    To additionally launch monitoring nodes, try:
    `roslaunch phastapromep startGUIsAndPerceptionMockupAndBehavior.launch`
    To run the learned behavior on the real robot, use 
    `roslaunch phastapromep runBehaviorOnRealRobot.launch`
    instead.



## session.yaml

The teachingApp creates a template session.yaml if not there already. This file specifies where to find recorded demonstrations, where to save output, metaparameters for learning ProMePs and other options.

Things you usually always do:
    - edit the transition label list


## Label the segments and learn a behavior

With this application, you can review the recorded segments and assign labels to group segments which are demonstrations of the same action. You can also discard segments, e.g. when something went wrong during recording, or when the segment was only used to pre-position the robot.

When you assigned labels to all segments, you can click the learn button, which will start the learning script. From the sequence of labels, the script infers the state graph topology. Then, for each transition it learns a ProMP from the given demonstrations, and a state controller for each state using all incoming and outgoing transitions.
The resulting data are saved into a directory (usually "behavior/"). Additionally, plots of each learned ProMP are saved to "illustrations/" for visual inspection.

### State labels

In case you want to let learnBehavior.py assign specific names to states, you have two options:
1. Label at least one transitions to or from the state either "to_state" or "from_state" respectively.
2. Add state name hints to the session.yaml with a dictionary named succeeding_controller_name_hints

## Run the behavior

Once you have generated the "behavior/" directory, you can load and execute the behavior using nodes provided in the phastapromep package:

    - phastanode running a phase-state machine
    - mixernode blending ProMePs and state goals based on activations and phases

In addition, this package provides several monitor nodes that visualize the output of those two nodes. (require pyqtgraph to be installed)


To start a phase-state-machine node, a prompmixer node, and a simulated robot, run:
    roslaunch phastapromep startGUIsAndPerceptionMockupAndBehavior.launch


After you checked(!) that the behavior works in simulation, you can execute the behavior on a real robot with:
    roslaunch phastapromep startGUIsAndPerceptionMockupAndBehaviorOnRealRobot.launch


## Common Pitfalls

### pytables vs. numpy compatibility break

numpy > 1.16 requires you to have pytables > 3.5.0. On some systems (specifically, ubuntu 18.04) you may need to upgrade pytables manually.


