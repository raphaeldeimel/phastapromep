# ProMP teacher with LabelApp2 / phastapromep Tutorial


In this pacakge you find an example of using the LabelApp to create a graph of motions from demonstrations on the robot. In general, you follow the folowing work flow:

Create a directory and go there:
`mkdir myfirstexperiment && cd myfirstexperiment` 
Then start there the teaching app from the phastapromp package in this directory:
`roslaunch phastapromep teachingApp.launch`
(This will create a number of stub files if they are not present yet)
Now go and record sequences of motions into a hdf5 database (default name: observations.h5)

Within tzeaching app:
1. Record demonstrations / observations
2. Label segments to indicate which are instances of the same motion
3. (Optional) Edit the session.yaml file configuring metaparameters of the learning session
4. Generate/Learn the behavior as a set of ProMePs, state controllers, and a connectivity graph for the phase-state machine
    You can either use the "learn behavior" button in the teaching app, or run `rosrun phastapromp learnGraph.py session.yaml`
5. Run the behavior using phastanode (executes the graph), mixernode (governs the motions) and pdcontrolleremulatornode (emulates a pd-controlled panda arm for visualization)
    `roslaunch phastapromep runBehavior.launch`
    To additionally launch monitoring nodes, try:
    `roslaunch phastapromep startGUIsAndPerceptionMockupAndBehavior.launch`

## Common Pitfalls

### pytables vs. numpy compatibility break

numpy > 1.16 requires you to have pytables > 3.5.0. On some systems (specifically, ubuntu 18.04) you may need to upgrade pytables manually: 
    '''pip install 


## Record sequences of motion

Demonstrate as many segments as you like. If you want segments to be executable in sequence, demonstrate the sequence at least once.

If you want to reposition the robot, i.e. to start a new sequence, simply move the robot. You can discard that segment later on in the LabelApp.

## Setup session.yaml

The recordDemonstrations.launch also creates a template session.yaml if not there already. This file specifies where to find recorded demonstrations, where to save output, and which labels the LabelApp should offer. Furhter, it specifies the metaparameters for learning ProMPs, such as the number of supports to use, and whether to also model torques (or fake them)

Things you usually always do:
    - set the name of the hdf5 file(s)
    - edit the transition label list


## Label the segments and learn a behavior

With this application, you can review the recorded segments and assign labels to group segments which are demonstrations of the same action. You can also discard segments, e.g. when something went wrong during recording, or when the segment was only used to pre-position the robot.

When you assigned labels to all segments, you can click the learn button, which will start the learning script. From the sequence of labels, the script infers the state graph topology. Then, for each transition it learns a ProMP from the given demonstrations, and a state controller for each state using all incoming and outgoing transitions.
The resulting data are saved into a directory (usually "behavior/"). Additionally, plots of each learned ProMP are saved to "illustrations/" for visual inspection.

## Run the behavior

Once you generated the "behavior/" directory, you can load and execute it using the nodes in the phastapromep package.

To start a phase-state-machine node, a prompmixer node, and a simulated robot, run:
    roslaunch phastapromep startGUIsAndPerceptionMockupAndBehavior.launch


After you checked(!) that the behavior works in simulation, you can execute the behavior on a real robot with:
    roslaunch phastapromep startGUIsAndPerceptionMockupAndBehaviorOnRealRobot.launch

-------------------------------------------------------------------------------------------
---------------------------    Documentation    ----------------------------------------------
-------------------------------------------------------------------------------------------

#Creating Documentation with doxygen

Clone doxygen from the official source list:

    git clone https://github.com/doxygen/doxygen.git
    cd doxygen

After that you can use:

    mkdir build
    cd build
    cmake -G "Unix Makefiles" ..
    make 
    make install

Fore system wide installation, root priveleges might be needed (sudo)

Then run:

    doxygen LabelApp2DoxyFile



It will create a new Folder Documentation.
Find the Wiki html pages by going to 

    Documentation/html/index.html

For more information see:

http://www.doxygen.nl/download.html




