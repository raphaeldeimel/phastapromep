#!/usr/bin/env python
# -*- coding: utf-8 -*-
#LabelApp2
#Copyright Raphael Deimel
#
#Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
#
#1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
#
#2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
#
#THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""
@author: Jan Martin
@author: Raphael Deimel
@copyright: 2019
@licence: 2-clause BSD licence
"""
import traceback

import collections
import pandas as pd
import warnings
import os
from enum import Enum

import numpy as np

#========ROS=========#
import rosparam
import rospy
from sensor_msgs.msg import JointState
from threading import Thread
from panda_msgs_mti.msg import RobotModeMsg
from panda_msgs_mti.msg import PDControllerGoal8
from panda_msgs_mti.msg import RobotState


try:
    from ruamel import yaml   #replaces deprecated pyYAML
except ImportError as e:
    print("\nCould not find ruamel. Please download/install the python-ruamel package!\n”")
    raise e

def yaml_constructor(loader, node):
    """ fixing bug with unicode patterns in yamlconfig File"""   
    return node.value
yaml.SafeLoader.add_constructor("tag:yaml.org,2002:python/unicode", yaml_constructor)


class ROSTime(object):
    """ Custom timestamp definition for ROS conformity """
    nsecs = 0
    secs = 0


class PandaRobotMode(Enum):
    kOther = 0
    kIdle = 1
    kMove = 2
    kGuiding = 3
    kReflex = 4
    kUserStopped = 5
    kAutomaticErrorRecovery = 6



class PandaPublisherModeEnum(Enum):
    replay_requested = 0
    paused_trajectory = 1
    replaying_trajectory = 2
    idle = 3
    prepare_for_publishing = 4 
    advance_initial_position = 5
    publishSample = 6
    wait_for_behavior = 7
    endPublishing = 8
    recording = 9



def makeMetaDataIndex():
    tuples = [('observed', 'time', 'secs'),('observed', 'time', 'nsecs'),('observed','time', 't')]
    for level1 ,level2 in (('observed', 'position'), 
                ('observed','velocity'), 
                ('observed', 'acceleration'), 
                ('observed', 'torque'), 
                ('exteroceptive', 'torque'), 
                ('commanded', 'position'), 
                ('commanded', 'velocity')): #names used by ProMeP
        for i in range(8):
            tuples.append( (level1, level2, str(i)) )
    for i in range(16):
        tuples.append( ('observed_EE','htransform', str(i)) )
    for i in range(6):
        tuples.append( ('observed_EE','wrench', str(i)) )
    for i in range(42):
        tuples.append( ('observed_EE','jacobian', str(i)) )
    for i in range(42):
        tuples.append( ('observed_EE','dotjacobian', str(i)) )
    return pd.MultiIndex.from_tuples(tuples, names=('origin', 'id', 'dof'))
metaDataIndex = makeMetaDataIndex() #create the metadata table index once globally, and reuse it




class LabelController(object):

    def __init__(self, sessionConfigYamlpath):
        """ Prepares and loads all default elements for LabelApp """    
        self.sessionConfigYamlpath = sessionConfigYamlpath
        try:
             with open(self.sessionConfigYamlpath) as f:
                yaml.SafeLoader.add_constructor("tag:yaml.org,2002:python/unicode", yaml_constructor)
                self.sessionConfig = yaml.safe_load(f)
        except IOError:
                rospy.logerror("LabelApp: Could not open session configuration from: {0}".format(sessionConfigYamlpath))
                raise SystemExit(1)


        self.db_filename = self.sessionConfig['hdf5_filename']
        self.store = pd.HDFStore(self.db_filename)

        self.debug = False             
        self.activeLabel = "unlabeled" # Labelname
        try:
            availableLabelsList = list(self.store.get("available_labels")[0])
        except KeyError:
            availableLabelsList = []
        self.availableLabels = collections.OrderedDict.fromkeys(availableLabelsList)
        self.specialLabels = ['discarded', 'unlabeled']
        
        try:
            self.metadata = self.store.get('metadata')
        except KeyError:
            self.metadata = pd.DataFrame([[0,'',0.0,0,0,0,0.0]], columns = ['i','label','playback_stiffness','inpoint','outpoint','samples','duration'])
            self.metadata.drop(0, inplace=True)
            
        self.minStiffness = 0.0  
        self.maxStiffness = 10.0
        self.stiffnessScaleStep = 0.8
        self.defaultStiffness = 0.2
        
        self.TrajectoryTimeDirection = 1

        self.kp_max= pd.Series([80,70,70,70,60,40,40,30])
        self.kd_max= pd.Series([15,15,15,15,5,5,5,5])
        
        self.pdcontrollermode = PandaRobotMode(PandaRobotMode.kOther)

        self.isStopped = True
        self.speedfactor  = 5.0
        self.speedfactor_ = self.speedfactor # do not change
        self.isspeedchanged = False
        self.isreset = False

        self.lastPublishedSampleIndex = None
        self.currentlyPublishingSampleIndex = None

        self.statusText = "Status"
        self.timespan = 0
        self._nextNominalPeriod = 1.0/180

        self.request_gui_update = 10

        self.isObserving = False
        self.destination_reached = False
        self.cut_play_active = True 


        self.PublisherMode = PandaPublisherModeEnum.idle
        self.lastPublisherMode = self.PublisherMode

        self.pdgoal_during_passive_recording = PDControllerGoal8()
        self.pdgoal_during_passive_recording.position = [0.0]*8
        self.pdgoal_during_passive_recording.velocity = [0.0]*8
        self.pdgoal_during_passive_recording.torque = [0.0]*8
        self.pdgoal_during_passive_recording.kp = [0.0]*8 #zero gains on position
        self.pdgoal_during_passive_recording.kv = [1.,1.,1.,1.,1.,0,0,0] #add some (slight) damping        

        self.last_robot_state_msg = []
        self.grippermode = "open"

        self.automovetostart = False # automatically moving to start when active
        self.autoreplay = False


        self.start_up_in_progress = 0 #set to zero for having soft controller gains
        self.max_startposition_tolerance = 0.06 #rad  maximal error from initial position when starting Trajectory
        self.advance_init_timefactor = 1.001 # timefactor for gain, when appoaching initial position

        # array that we use to deliver goals to pdcontroller_goal
        # for redoing Trajectories with higher certainty ...
        # and running the simulation
        self.currentTrajectory = None
        self.currentlyActiveTrajectoryNumber = None
                
        #Instantiate publishers, and set which none to publish controller goals to:
        self.rosPublisherToPdcontroller = rospy.Publisher("robot/pdcontroller_goal", PDControllerGoal8, queue_size=3)
        #self.rosPublisherToRviz = rospy.Publisher("robot/pdcontroller_goal_simulation", PDControllerGoal8, queue_size=3)

        # listen to for new Trajectories
        self.observedRobotStatesList = []# Buffer to Store one Trajectory at a time(the current one)
        rospy.Subscriber("robot/currentstate", RobotState, self.PandaRobotStateCallback)
        self.dof = len(self.sessionConfig['joint_indices_list'])

        self.sessionConfig['data_directory'] = os.path.abspath(os.path.expanduser(self.sessionConfig['data_directory']))


    def getNextObservationId(self):
        i = self.metadata['i'].max()
        if pd.isnull(i):
            return 0
        return i+1
        

    def getObservationsCount(self):
        """
        returns the number of recorded observations
        """
        return len(self.metadata.index)

    def deleteFromAvailableLabels(self, label):
        """ callback for GUI to add a label
        """
        if not label in self.availableLabels:
            return
        del self.availableLabels[label] #we use an OrderedDict to emulate an ordered set for python<3.7
        self.store.put("available_labels",pd.DataFrame(self.availableLabels.keys()))

    def addToAvailableLabels(self, label):
        """ callback for GUI to add a label
        """
        if label in self.availableLabels:
            return
        self.availableLabels[label] = None #we use an OrderedDict to emulate an ordered set for python<3.7
        self.store.put("available_labels",pd.DataFrame(self.availableLabels.keys()))

        
    def setLabelOfCurrentlyActiveTrajectory(self, label):
        """
        Callback for GUI to set the label of the currently active trajectory
        """
        if self.currentlyActiveTrajectoryNumber is None:
            return
        self.setLabelOfTrajectory(label, self.currentlyActiveTrajectoryNumber)
        
    def setLabelOfTrajectory(self, label, TrajectoryNumber):
        if not label in self.availableLabels and not label in self.specialLabels:
            raise ValueError("Tried to set a label that is not listed in availableLabels")    
        self.metadata.loc[TrajectoryNumber, 'label'] = label
        self.commitMetaData()



    def enableRecordingPanda(self,keystroke = 0, mode = 0):
        """ Enable recording samples from Panda. Samples are stored in self.observedRobotStatesList"""
        self.isObserving = True

    def disableRecordingPanda(self,keystroke=0,mode=0):
        """ Disable recording Samples from Panda 
            Moves all Samples recorded from self.observedRobotStatesList to self.currentTrajectory so that they can be replayed and
            stores a backup into the database backend
        """           
        if(len(self.observedRobotStatesList) < 3):
            rospy.logerr("Warning. No or not enough samples collected! Ignoring request to save")
            return

        self.PublisherMode = PandaPublisherModeEnum.prepare_for_publishing
        self.isObserving = False

        df = pd.DataFrame(self.observedRobotStatesList, columns = metaDataIndex) #todo: use pandas's downsampling methods
        
        #compute the relative time float-value post-hoc:
        times = df['observed', 'time']
        ref_timestamp = times.iloc[0]
        rel_timestamps = times - ref_timestamp
        df[('observed', 'time', 't')]  = rel_timestamps['secs'] + 1e-9 * rel_timestamps['nsecs']  #add a (somewhat redundant) relative time column to make life easier

        newentry_number = self.getNextObservationId()
        d_line = {
            'i': newentry_number,
            'label': 'unlabeled',
            'playback_stiffness': self.defaultStiffness,
            'inpoint': 0,
            'outpoint': len(df.index)-1,
            'samples': len(df.index),
            'duration': df[('observed', 'time', 't')].iloc[-1]
        }
        newentry = pd.DataFrame(d_line, index=[newentry_number] )
        #save data also to database backend:
        self.store.put("observations/observation{0}".format(newentry_number), df)
        self.metadata = pd.concat([self.metadata,newentry])
        self.commitMetaData()
                
        self.currentlyActiveTrajectoryNumber = newentry_number
        self.currentTrajectory = df
        self.currentlyPublishingSampleIndex =  self.metadata.loc[self.currentlyActiveTrajectoryNumber,'samples'] -1   #set position for playback to the last row, as this is usually close to where the robot currently is
        self.observedRobotStatesList = [] # empty Buffer for new data to append
        self.kivyinterface.registerTrajectory(self.currentlyActiveTrajectoryNumber,self.activeLabel, certainty=self.metadata['playback_stiffness'].iloc[self.currentlyActiveTrajectoryNumber] )
        self.PublisherMode = PandaPublisherModeEnum.idle
        self.request_gui_update = 5


    def setRootWidget(self, rootwidget):
        """ Handle for interfacing wit the KivySurface """
        self.kivyinterface = rootwidget
        self.loadActiveTrajectory(self.currentlyActiveTrajectoryNumber) #load the Trajectory after Interface


    def startPlaying(self):
        """ start replaying samples from currently active Trajectory """ 
        if self.currentTrajectory is None or self.currentlyActiveTrajectoryNumber is None:
            return
        self.start_up_in_progress = 0 #Controller will use low gains 

        if self.PublisherMode == PandaPublisherModeEnum.prepare_for_publishing:
            return False

        if self.currentlyActiveTrajectoryNumber < 0  or self.currentlyActiveTrajectoryNumber >= self.getObservationsCount():
            raise RuntimeError("current active trajectory outside of list of available trajectories!")
            
        self.PublisherMode = PandaPublisherModeEnum.replay_requested

        return True

    def pauseTrajectory(self):
        """ Function pauses all ongoing action by PlaybackController by simply setting PublisherMode to idle."""
        if self.currentTrajectory is None:
            return        
        self.PublisherMode = PandaPublisherModeEnum.paused_trajectory

    def toggleReplaying(self, direction=1):
        """ Function switches between Replaying the Trajectory and Idle Mode """
        if self.currentTrajectory is None:
            return
        if self.PublisherMode == PandaPublisherModeEnum.prepare_for_publishing:
            return
                    
        self.start_up_in_progress = 0 #Controller will use low gains 

        self.TrajectoryTimeDirection = direction
        if self.PublisherMode == PandaPublisherModeEnum.replaying_trajectory:
            self.pauseTrajectory()
        else:
            self.startPlaying()

    def toggleRecordReplay(self):
        self.toggleReplaying(direction=1)
        self.toggleRecordingPanda()

    def jumpBack(self):
        """ Jump back to beginning of Trajectory """
        if self.currentTrajectory is None:
            return
            
        self.start_up_in_progress = 0 #Controller will use low gains 
        self.TrajectoryTimeDirection = 1 
        if self.cut_play_active == True:
            self.currentlyPublishingSampleIndex = self.currentTrajectoryInPoint
        else:
            self.currentlyPublishingSampleIndex = 0
        self.PublisherMode = PandaPublisherModeEnum.publishSample
        print("jumpback!")
        return True

    def updateRobotStateOnce(self):
        print("updateRobotStateOnce")
        if self.PublisherMode == PandaPublisherModeEnum.idle:   
            #self.PublisherMode = PandaPublisherModeEnum.publishSample
            pass

    def loadActiveTrajectory(self,requestedTrajectory):
        """ Function to load (new) active Trajectory and handle all configuration
            requestedTrajectory: Trajectory that will be loaded and active
        """
        if requestedTrajectory is None:
            return
        self.start_up_in_progress = 0 #Controller will use low gains 
        try:
            row = self.metadata.iloc[requestedTrajectory]
        except IndexError:
            rospy.loginfo('Requested observation does not exist in metadata!', requestedTrajectory)
            return

        self.PublisherMode = PandaPublisherModeEnum.idle # stop publishing
        label = self.metadata.loc[requestedTrajectory, 'label']
        self.activeLabel = label

        #read in the now active trajectory:
        self.currentTrajectory = self.store.get("observations/observation{}".format(requestedTrajectory)) 
        self.currentlyActiveTrajectoryNumber = requestedTrajectory
        self.currentTrajectoryInPoint = row['inpoint']
        self.currentlyPublishingSampleIndex = self.currentTrajectoryInPoint
        self.currentTrajectoryOutPoint = row['outpoint']
        self.request_gui_update = 5



    def setStiffnessOfCurrentActiveTrajectory(self, stiffness):
        if self.currentlyActiveTrajectoryNumber is None:
            return 0.0
        stiffness = min(max(stiffness, self.minStiffness), self.maxStiffness)
        self.metadata.loc[self.currentlyActiveTrajectoryNumber, 'playback_stiffness'] = stiffness
        self.commitMetaData()
        return stiffness

    def increaseStiffnessOfCurrentActiveTrajectory(self):
        if self.currentlyActiveTrajectoryNumber is None:
            return
        currentStiffness  =self.metadata['playback_stiffness'].iloc[self.currentlyActiveTrajectoryNumber]
        proposedStiffness = currentStiffness / self.stiffnessScaleStep
        return  self.setStiffnessOfCurrentActiveTrajectory(proposedStiffness)

    def decreaseStiffnessOfCurrentActiveTrajectory(self):
        if self.currentlyActiveTrajectoryNumber is None:
            return
        currentStiffness  =self.metadata['playback_stiffness'].iloc[self.currentlyActiveTrajectoryNumber]
        proposedStiffness = currentStiffness * self.stiffnessScaleStep
        return self.setStiffnessOfCurrentActiveTrajectory(proposedStiffness)


    def getRelativeSliderTimestamps(self,requestedTrajectory): 
        """ returns relative start and end of CutlistSliders
            @return: success, rel_start, rel_stop
            success --> True/False
            rel_start --> between 0 and 1
            rel_stop  --> between 0 and 1
        """
        try:
            row = self.metadata.iloc[int(requestedTrajectory)]
        except IndexError:
            return 0.0, 1.0
        stepsize = 1.0 / (row['samples']-1)
        in_relative = row['inpoint'] * stepsize
        out_relative = row['outpoint'] * stepsize
        return in_relative, out_relative

    def setKivySliderValuesandButtonforTrajectory(self):
        """ Update GUI elements"""
        TrajectoryNumber = self.currentlyActiveTrajectoryNumber
        if TrajectoryNumber is None:
            self.kivyinterface.setTrajectoryCursorParameters(None, None, None) 
            self.kivyinterface.presetPlaybackImpedanceDisplay(None)
            return
        rel_start, rel_stop = self.getRelativeSliderTimestamps(TrajectoryNumber)    
        samplevalue = float(self.currentlyPublishingSampleIndex )/ self.metadata.loc[TrajectoryNumber, 'samples']
        self.kivyinterface.setTrajectoryCursorParameters(rel_start, samplevalue, rel_stop) 
        self.kivyinterface.updateTrajectoryButtonColorAndText(TrajectoryNumber,self.activeLabel)
        self.kivyinterface.updateCutlistinScrollview(TrajectoryNumber,rel_start, rel_stop)
        self.kivyinterface.presetPlaybackImpedanceDisplay(self.metadata['playback_stiffness'].iloc[self.currentlyActiveTrajectoryNumber])

        print("updated display to: {}, {},{}".format(rel_start, samplevalue, rel_stop))

    def setInPointofCurrentTrajectory(self, rel_start):
        if rel_start == None or self.currentTrajectory is None or self.currentlyActiveTrajectoryNumber is None:
            return
        i = int(rel_start * self.metadata['samples'].iloc[self.currentlyActiveTrajectoryNumber] + 0.5) 
        i = max(0,min(i,self.metadata['samples'].iloc[self.currentlyActiveTrajectoryNumber]-1))
        self.metadata.loc[self.currentlyActiveTrajectoryNumber, 'inpoint'] = i
        self.currentTrajectoryInPoint = i
        
    def setOutPointofCurrentTrajectory(self, rel_end):
        if rel_end == None or self.currentTrajectory is None or self.currentlyActiveTrajectoryNumber is None:
            return
        i = int(rel_end * self.metadata['samples'].iloc[self.currentlyActiveTrajectoryNumber] + 0.5)
        i = max(0,min(i,self.metadata['samples'].iloc[self.currentlyActiveTrajectoryNumber]-1))
        self.metadata.loc[self.currentlyActiveTrajectoryNumber, 'outpoint'] = i
        self.currentTrajectoryOutPoint = i
        
    def commitMetaData(self):
        self.store.put("metadata", self.metadata, format='table')

    def getUpdateTimePeriod(self):
        """ Return Looptime for periodic execution of periodic()  """
        return self._nextNominalPeriod






    def toggleRecordingPanda(self,keystroke = 0, mode = 0):
        if self.isObserving:
            self.disableRecordingPanda()
        else:
            self.enableRecordingPanda()
            


    def PandaRobotStateCallback(self,msg):
        """ Callback for Robot states to record"""
        self.pdcontrollermode = PandaRobotMode(msg.mode)
        #assemble all data into a db row. The 't' Column  is filled with nan as it is computed post-recording:
        #Warning!: Check that the row order below matches with the metaDataIndex list!
        if not self.isObserving:
            return
        row = [msg.stamp.secs, msg.stamp.nsecs, np.nan] + list(msg.q) + list(msg.dq) + list(msg.ddq) + list(msg.tau) + list(msg.tau_ext) + list(msg.qd) + list(msg.dqd) + list(msg.ee_htransform_base) + list(msg.ee_wrench_ee) + list(msg.ee_jacobian_ee) + list(msg.ee_dotjacobian_ee)
        self.observedRobotStatesList.append(row)


    def publish_pd_controllergoal(self,PDgoal8=None,Samplecounter = None, time_speed=1, soft_start=True ):
        """ Function for Publishing a PDControllerGoal8.msg to Panda """
        if PDgoal8 != None:
            PDControllerGoal8Msg = PDgoal8
        else:
            if self.currentlyActiveTrajectoryNumber is None:
                return
            if Samplecounter == None: 
                return        
            if Samplecounter < 0 :
                return
            if Samplecounter >= self.metadata['samples'].iloc[self.currentlyActiveTrajectoryNumber]:
                return
            PDControllerGoal8Msg = PDControllerGoal8()
            row = self.currentTrajectory.iloc[Samplecounter]
            playback_stiffness = self.metadata['playback_stiffness'].iloc[self.currentlyActiveTrajectoryNumber]
            PDControllerGoal8Msg = self._convertRowToPDgoalMsg(row,playback_stiffness, time_speed = time_speed) # self.currentlyPublishingSampleIndex iterates the number of Samples in Trajectory
            PDControllerGoal8Msg.stamp = rospy.get_rostime() + rospy.Duration(0.1) #overwrite the trajectory's timestamp to the current real time

        if soft_start:
            if self.start_up_in_progress < 10:
                self.start_up_in_progress = self.start_up_in_progress + 1 # have first samples soft
                PDControllerGoal8Msg.kp = 0.1*self.kp_max
                PDControllerGoal8Msg.kv = 0.01*self.kd_max
        else:
               self.start_up_in_progress=0 
        self.rosPublisherToPdcontroller.publish(PDControllerGoal8Msg)



    def _convertRowToPDgoalMsg(self,row, playback_stiffness, time_speed=1.0):
        """ Converts numpy-array read from HDF5 to PDControllerGoal8 for replaying on panda """
        msg=PDControllerGoal8()
        time = row[('observed', 'time')]
        positions = row[('observed', 'position')]
        velocities = row[('observed', 'velocity')]
        torques = row[('observed', 'torque')]
        msg.stamp.secs = time['secs']
        msg.stamp.nsecs = time['nsecs']
        for i in range(self.dof):
            msg.position[i] =  positions[str(i)]
            msg.velocity[i] =  time_speed * velocities[str(i)]
            msg.torque[i] =    time_speed * torques[str(i)]
            msg.kp[i] = playback_stiffness*self.kp_max[i] # soft
            msg.kv[i] = self.kd_max[i] #and slow
        return msg


    def check_command_initial_position(self):
        """SafetyFeature that is called to check wether the start of Trajectory is reached. If not it will stay in loop and commands soft approach """
        if not isinstance(self.last_robot_state_msg,RobotState):
            rospy.logwarn("Robot is not active!")
            return True
        if self.currentlyActiveTrajectoryNumber is None:
            return
        state = self.last_robot_state_msg # shall not change
             
        actual_time = rospy.get_rostime()
        time_difference = ((actual_time.secs*1000 + actual_time.nsecs/1000000) -(state.stamp.secs*1000 + state.stamp.nsecs/1000000)) #[ms]
        if time_difference > 100 :
            rospy.logwarn("Watchdog: Last Robot State is too old. time_difference {0}[ms]".format(time_difference))
            return False
        if self.metadata['samples'].iloc[self.currentlyActiveTrajectoryNumber] == self.currentlyPublishingSampleIndex:
            self.currentlyPublishingSampleIndex = self.currentlyPublishingSampleIndex -1
            return False
        if self.metadata['samples'].iloc[self.currentlyActiveTrajectoryNumber] < self.currentlyPublishingSampleIndex:
            rospy.logwarn("SampleCounter is too high. Stop here.")
            raise SystemError

        playback_stiffness = self.metadata['playback_stiffness'].iloc[self.currentlyActiveTrajectoryNumber]
        next_goal = self._convertRowToPDgoalMsg(self.currentTrajectory.iloc[self.currentlyPublishingSampleIndex],playback_stiffness)
        if type(next_goal) != PDControllerGoal8:
            return False
        q_soll = np.array(next_goal.position)    
        q = np.array(state.q) 
        difference = np.empty(q_soll.shape,dtype=float)
        for i in range(len(q_soll)):
           difference[i] = q_soll[i]-q[i]

        if len(difference[abs(difference)>self.max_startposition_tolerance]): #some 
            #q_soll[difference >self.max_startposition_tolerance]  =  q[difference > self.max_startposition_tolerance]+0.8*self.max_startposition_tolerance 
            #q_soll[-difference >self.max_startposition_tolerance] =  q[-difference >self.max_startposition_tolerance]-0.8*self.max_startposition_tolerance
            
            for i in range(len(q_soll)-1):
                if difference[i] > self.max_startposition_tolerance:
                    q_soll[i] = q[i] + 0.8*self.max_startposition_tolerance     
                if -difference[i] > self.max_startposition_tolerance:
                    q_soll[i] = q[i] - 0.8*self.max_startposition_tolerance

            
            
            next_goal.position = q_soll
            next_goal.position[self.dof - 1] = 0.1 # Force Gripper open

            next_goal.kp = [60,60,60,50,50,50,30,30]
            next_goal.kv = [5,5,5,5,5,5,5,5]
            self.publish_pd_controllergoal(PDgoal8 = next_goal)
            rospy.loginfo("(Slowly) Moving to Initialposition of next Trajectory")
            return False
        return True    



    def get_Controller_State(self): 
        """ Returns speedfactor,currentlyActiveTrajectoryNumber,PublisherMode,currentlyPublishingSampleIndex,Number of Recorded Samples,Number of Samples published / publishable, guiding -->recording is true/false """
        if self.observedRobotStatesList is None:
            len_observed = 0
        else:
            len_observed = len(self.observedRobotStatesList)
        if self.currentlyActiveTrajectoryNumber is None:
            len_currentTrajectory = 0
        else:
            len_currentTrajectory = self.metadata['samples'].iloc[self.currentlyActiveTrajectoryNumber]
        return self.speedfactor,self.currentlyActiveTrajectoryNumber,self.PublisherMode,self.currentlyPublishingSampleIndex,len_observed,len_currentTrajectory, self.isObserving
        
    def getCurrentlyActiveTrajectoryNumber(self):
        """ Will return the Number of the currently active Trajectory """
        return self.currentlyActiveTrajectoryNumber

    def setGripper(self,mode="toggle",buttoninstance=0):
        """ 
            Function for sending command to the robots Gripper

            modes:   
            -------------------------------------------------
            toggle --> toggle the actual state of the gripper 
            open   --> open Gripper
            close  --> close Gripper  
        """
        if not isinstance(self.last_robot_state_msg,RobotState):
            rospy.logwarn("Cannot command Gripper. Robot seems inactive.")    
            return True#skip

        last_robot_state_msg = self.last_robot_state_msg
        actual_time = rospy.get_rostime()
        difference = (1000*(actual_time.secs-last_robot_state_msg.stamp.secs) + (actual_time.nsecs-last_robot_state_msg.stamp.nsecs)/1000000) #[ms] as integer
        if difference > 100 :
            rospy.logwarn("Watchdog: Last Robot State is too old. Difference {0}[ms]".format(difference))
            return False
            
            
        msg = PDControllerGoal8()
        msg.stamp =    last_robot_state_msg.stamp
        msg.position = last_robot_state_msg.position
        msg.velocity = last_robot_state_msg.velocity
        msg.torque = last_robot_state_msg.torque
        msg.kp = (0,0,0,0,0,0,0,10)
        msg.kv = (0,0,0,0,0,0,0,10)
        #print("Done copying.")

        
        gripper_dof = 7
        msg.stamp = rospy.get_rostime() + rospy.Duration(0.2)#TODO take 100ms second
        if(mode == "open"):
            msg.position[gripper_dof] = 0.08 # max_open
        elif(mode == "close"):
            msg.position[gripper_dof] = 0 # max_open
        elif(mode == "toggle"):
            if self.grippermode == "close":
                self.grippermode = "open"
                msg.position[gripper_dof] = 0.07 # max_open    
            else:
                self.grippermode = "close"
                msg.position[gripper_dof] = 0 # max_open    
        else:
            rospy.logerror("Mode not valid.")
            return False    
        self.rosPublisherToPdcontroller.publish(msg)
 
        return msg
        
        
    def disable_cutplay(self):
        """ diables stopping Play at cuts when replaying the Trajectory """
        self.cut_play_active = False 

    def enable_cutplay(self):
        """ enables stopping Play at cuts when replaying the Trajectory """
        self.cut_play_active = True 


    ###################################################
    # Functions for Handling Playbackcontroller State #
    ###################################################   
    def central_state_handler(self):
        """ Central for handling the mode in which the PlaybackController currently operates"""
        # do not restart on your own
#        print(self.PublisherMode, self.isObserving)
        if self.PublisherMode != self.lastPublisherMode:            
            print(self.PublisherMode, self.isObserving)
        self.lastPublisherMode = self.PublisherMode

        if self.PublisherMode == PandaPublisherModeEnum.recording and not self.isObserving:
                self.PublisherMode = PandaPublisherModeEnum.idle
        elif self.PublisherMode == PandaPublisherModeEnum.idle and self.isObserving:
                self.PublisherMode = PandaPublisherModeEnum.recording

        if (self.PublisherMode == PandaPublisherModeEnum.wait_for_behavior or 
            self.PublisherMode == PandaPublisherModeEnum.idle or 
            self.PublisherMode == PandaPublisherModeEnum.prepare_for_publishing):
              return

        if self.PublisherMode == PandaPublisherModeEnum.recording:
             self.start_up_in_progress = False
             self.publish_pd_controllergoal(self.pdgoal_during_passive_recording, soft_start=False)
             return

        if self.PublisherMode == PandaPublisherModeEnum.publishSample:   
                self.publish_pd_controllergoal(Samplecounter = self.currentlyPublishingSampleIndex, soft_start=False) # advance manually through Trajectory -> Publish this sample
                self.PublisherMode = PandaPublisherModeEnum.idle
                return False #stop here

        if self.currentlyActiveTrajectoryNumber is None: #everything below requires an active trajectory to work
            return
            
        now = rospy.Time.now()
        if self.PublisherMode == PandaPublisherModeEnum.replay_requested:
            if self.TrajectoryTimeDirection >= 0 and self.currentlyPublishingSampleIndex >= self.currentTrajectoryOutPoint:
                self.PublisherMode = PandaPublisherModeEnum.idle
            elif self.TrajectoryTimeDirection < 0 and self.currentlyPublishingSampleIndex <= self.currentTrajectoryInPoint:
                self.PublisherMode = PandaPublisherModeEnum.idle
            else:
                self.PublisherMode = PandaPublisherModeEnum.replaying_trajectory # just replay Trajectory
                self.realTimeOfLastPublication = now
            

        if self.currentlyPublishingSampleIndex >= self.metadata.loc[self.currentlyActiveTrajectoryNumber,'samples']: 
            self.PublisherMode = PandaPublisherModeEnum.idle # go back to idle


        elif self.PublisherMode == PandaPublisherModeEnum.replaying_trajectory:
               if self.lastPublishedSampleIndex is None:
                    self.currentlyPublishingSampleIndex = 0
               elif self.lastPublishedSampleIndex >= self.metadata['samples'].iloc[self.currentlyActiveTrajectoryNumber]:
                    self.PublisherMode = PandaPublisherModeEnum.idle
                    self.currentlyPublishingSampleIndex = None
               else:
                    #advance in time to the correct sample:
                    t = self.currentTrajectory['observed', 'time', 't']
                    relativeTimeofLastPublication = t.iloc[self.lastPublishedSampleIndex]
                    durationSinceLastPublication = (now.secs - self.realTimeOfLastPublication.secs)+1e-9*(now.nsecs - self.realTimeOfLastPublication.nsecs)
                    if self.TrajectoryTimeDirection > 0:  #trajectory time moves forward
                        if self.cut_play_active:
                            z = self.currentTrajectoryOutPoint
                        else:
                            z = self.metadata['samples'].iloc[self.currentlyActiveTrajectoryNumber]-1
                        for i in range(self.currentlyPublishingSampleIndex, z+1):
                            if t.iloc[i] < relativeTimeofLastPublication + durationSinceLastPublication:
                                self.currentlyPublishingSampleIndex = i
                            else:
                                break
                    else:   #trajectory time moves backwards
                        if self.cut_play_active:
                            z = self.currentTrajectoryInPoint
                        else:
                            z = 0
                        for i in range(self.currentlyPublishingSampleIndex, z-1, -1):
                            if t.iloc[i] > relativeTimeofLastPublication - durationSinceLastPublication:
                                self.currentlyPublishingSampleIndex = i
                            else:
                                break
                    if self.currentlyPublishingSampleIndex == z: #reached the end of the trajectory
                         self.PublisherMode = PandaPublisherModeEnum.endPublishing
                         
               #if the sample changed, publish new sample and remember it:
               if self.lastPublishedSampleIndex != self.currentlyPublishingSampleIndex:
                    self.realTimeOfLastPublication = now
                    self.publish_pd_controllergoal(Samplecounter = self.currentlyPublishingSampleIndex, time_speed = self.TrajectoryTimeDirection)
                    self.lastPublishedSampleIndex = self.currentlyPublishingSampleIndex
                    value = self.currentlyPublishingSampleIndex / float(self.metadata.loc[self.currentlyActiveTrajectoryNumber,'samples'])
                    self.kivyinterface.trajectoryEditorBar.set_value_to_cursor('currentsample',value)


        if self.PublisherMode == PandaPublisherModeEnum.paused_trajectory:
            self.realTimeOfLastPublication = now #don't advance trajectories during pausing
        
        if self.PublisherMode == PandaPublisherModeEnum.endPublishing: #we finished playing, now what?

            stiffness = self.metadata['playback_stiffness'].iloc[self.currentlyActiveTrajectoryNumber]

            if self.PublisherMode == PandaPublisherModeEnum.replay_requested or self.PublisherMode == PandaPublisherModeEnum.advance_initial_position:
                if not self.check_command_initial_position():
                    self.PublisherMode = PandaPublisherModeEnum.advance_initial_position
                    return False

                self.PublisherMode = PandaPublisherModeEnum.replaying_trajectory
                self.speedfactor_ = self.speedfactor # make sure speedfactor_ is not to be changed during replaying_trajectory  
        
            else:
                self.PublisherMode = PandaPublisherModeEnum.idle
                self.statusText = "Idle."


        return True

 
    def periodic(self):
        """ function for retriggering everything that is updated regularly
        --> priodic Call for central_state_handler()
        """
        if rospy.is_shutdown():
            raise SystemExit("App has been terminated!")
        self.central_state_handler()

        text = "{:20s} | Me:{}".format(self.pdcontrollermode.name, self.PublisherMode.name) 
        self.kivyinterface.statusBar.text = text
        self.kivyinterface.modePane.setInfo(self.pdcontrollermode, self.PublisherMode.name)
        if self.currentlyActiveTrajectoryNumber is not None:
            self.kivyinterface.enableButtonsManipulatingTrajectories()

        if self.request_gui_update == 0:
            self.setKivySliderValuesandButtonforTrajectory()
        if self.request_gui_update >= 0:
            self.request_gui_update = self.request_gui_update - 1


    ###################################################
    #             End of Central Handler              #
    ###################################################

