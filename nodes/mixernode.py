#!/usr/bin/env python2
# -*- coding: utf-8 -*-
#Copyright 2017 Raphael Deimel
#
#Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
#
#1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
#
#2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
#
#THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""
@author: Raphael Deimel
@copyright: 2020
@licence: 2-clause BSD licence
"""


import numpy as _np
_np.set_printoptions(precision=2, suppress=True, formatter={'float': '{: 0.1f}'.format})
import os
from operator import itemgetter

import promep
import phasestatemachine
import phasestatemachine.msg
import phastapromp
import pandadynamicsmodel


import rospy
from sensor_msgs.msg import JointState
from panda_msgs_mti.msg import PDControllerGoal8, MechanicalStateDistribution8TorquePosVel, RobotState8, RobotEEState
from visualization_msgs.msg import Marker, InteractiveMarkerFeedback
import tf as _tf 

from promp._tensorfucntions import asList, makeCovarianceTensorUncorrelated #need to get rid of it


class MixerNode(object):

    def __init__(self, DefinitionsDirectory, doProfiling=False):
        """
        """
        
        self._profiler = None
        if doProfiling:
            import cProfile
            self._profiler = cProfile.Profile()
            self._profiler.bias=3e-6

        self.watchdogPeriod = rospy.Duration.from_sec( rospy.get_param('watchdog timeout', 0.5))
        self.jointaccmax = rospy.get_param('max joint acceleration', 1.0) #rad²/s²
        self.jointvelmax = rospy.get_param('max joint acceleration', 2.0) #rad/s
        self.updateFrequency = rospy.get_param('frequency', 50.0)
        self.max_active_inputs = rospy.get_param('max_active_inputs',5) 

        #some global paramters related to MSDs and phases:
        self.msd_size_realms = rospy.get_param('msd_size_realms',2)  #number of realms to use
        self.msd_size_derivatives = rospy.get_param('msd_size_derivatives',2) #number of derivatives to consider for MSDs and phase
        self.msd_size_dofs = rospy.get_param('msd_size_derivatives',8) 
        self.phase_size_derivatives = rospy.get_param('phase_size_derivatives',2) #number of derivatives to expect for the generalized phase

        self.integration_timestep = 1.0 / self.updateFrequency

        self.integration_substeps

        self.DefinitionsDirectory = DefinitionsDirectory

        #make a tensor namespace that hold the right index definitions:
        self.tns = mechanicalstate.makeTensorNameSpaceForMechanicalStateDistributions(r=self.msd_size_realms, g=self.msd_size_derivatives, d=self.msd_size_dofs)

        self.mixer = promep.Mixer(self.tns, max_active_inputs=self.max_active_inputs)
        self.last_msd = self.mixer.msd_mixed

        #load the promp definitions and instantiate a phase-state machine
        with open(os.path.join(self.DefinitionsDirectory, 'phasta.yaml')) as f:
            graphconfig = yaml.safe_load(f)
        #create msd_generators: 
        self.size_states = graphconfig['number_of_states']
        msd_generators = _np.array([[None]*self.size_states]*self.size_states)
        t = graphconfig['transition_names']
        for transitionLabel in t: #for all transitions:
            filename=os.path.join( DefinitionsDirectory, + "{}.promep.h5".format(transitionLabel) )
            p = promep.ProMeP.makeFromFile(filename)
            msd_generators[[t[transitionLabel][1], t[transitionLabel][0]]] = p

        for i, params in enumerate(graphconfig['state_controllers']):  #for all states: 
            c  =  staticprimitives.LTIGoal.makeFromDict(params)
            msd_generators[i,i] = c

        
        #initial node-global values:
        self.msd_generators = msd_generators
        self.phases = _np.zeros((self.phase_size_derivatives,self.size_states,self.size_states ))
        self.activations = _np.zeros(self.size_states, self.size_states)
        self.activationsTime = None
        self.validUntil=None
        self.meansDesired = None
        self.jacobianEE = None
        self.jacobianBase = None
        self.hTransform = None
        self.hTransformGoal = None
        

        try: 
            dynamicsModel = pandadynamicsmodel.PandaURDFModel()
        except:
            dynamicsModel = None

        self.timeintegrator = mechanicalstate.TimeIntegrator(self.tns, dynamicsModel = dynamicsModel)
        
        self.kinematicsModel = pandadynamicsmodel.PandaURDFModel()
        
        self.publishMarkers()

        # Subscribe to Jacobian from Controller
        self.JacobianHTListener = rospy.Subscriber("/panda/currentEEstate", RobotEEState, self.JacobianCallback, queue_size=3) 

        # Subscribe to Task Space Goal Marker
        self.JacobianHTListener = rospy.Subscriber("/task_space_goal_marker/feedback", InteractiveMarkerFeedback, self.desiredTaskSpacePoseCallback, queue_size=3) 

        rospy.loginfo("MSD mixer node initialized")
 
    def setStartPosition(self, robotstatemessage):
        indices_position = self.last_msd.readable_names_to_realm_derivative_indices['position']
        indices_velocity = self.last_msd.readable_names_to_realm_derivative_indices['position']
        self.last_msd.means.data[...] = 0.0
        self.last_msd.means.data[indices_position][:] = robotstatemessage.q
        self.last_msd.means.data[indices_velocity][:] = robotstatemessage.dq

        var = _np.dot(_np.array([0,1e-3,1e-3])[:,_np.newaxis], _np.ones((1,self.dofs)))        
        self.last_msd.covariances_flatview[:,:] = 0.0
        self.last_msd.variancesView[:] = var
        

    def publishMarkers(self):
        #publish a marker describing the current mixer state:
        self.markerpublisher = rospy.Publisher("/phasta/state_marker", Marker, queue_size=3)
        m = Marker()
        m.type = Marker.TEXT_VIEW_FACING 
        m.header.frame_id = "panda_link0" 
        m.color.r = 1 
        m.color.g = 1 
        m.color.b = 1 
        m.color.a = 1 
        m.scale.x = 0.1 
        m.scale.y = 0.1 
        m.scale.z = 0.1 
        m.pose.position.x = 0.5 
        m.pose.position.y = 0.5 
        m.pose.position.z = -0.2 
        self.mixedComponentsMarker = m
        self.markerpublisher.publish(self.mixedComponentsMarker)

        self.desiredControllerGoalPublisher = rospy.Publisher("mixer/pdcontroller_goal", PDControllerGoal8, queue_size=2, tcp_nodelay=True)
        self.MStatePublisher = rospy.Publisher("mixer/mstate_distribution", MechanicalStateDistribution8TorquePosVel, queue_size=2, tcp_nodelay=True)
        self.PhasesActivationsListener = rospy.Subscriber("/phasta/phases_activations", phasestatemachine.msg.PhasesActivations, self.PhasesActivationsCallback, queue_size=3) 


    def _updateStateMarker(self):
        #determine a nice text description of what is currently active and how much:
        namesActiveStrings = []
        for indices in self.mixer.active_generators_indices:
            name = self.msd_generators[indices].name
            a = self.activations[indices]
            if self.msd_generators[indices].isPhaseAssociable:
                phi = self.phases[0,...][indices]
                namesActiveStrings.append(" {1:.2f} {0} - {2:2.0f}%".format(name, a, 100*phi))
            else:
                namesActiveStrings.append(" {1:.2f} {0}".format(name, a))   
        namesActiveStrings = namesActiveStrings + [" "]*(3-len(namesActiveStrings))
        self.mixedComponentsMarker.text = "\n".join( namesActiveStrings )
        self.mixedComponentsMarker.header.stamp = now
        self.markerpublisher.publish(self.mixedComponentsMarker)


    #read new activation and phase, update the state distribution, compute the resulting desired controller and publish the values
    def PhasesActivationsCallback(self, data):
        if data.states != self.num_states:
            rospy.logerr("PhasesActivations message has wrong state size {0}x{0} instead of {1}x{1}".format(data.states, self.num_states))        

        self.activationsTime = data.stamp
        self.phases[0,:,:] = _np.asarray(data.phases).reshape((data.states, data.states))
        if self.phase_size_derivatives >=1:
            self.phases[1,:,:] = _np.asarray(data.phaseVelocities).reshape((data.states, data.states)) 
        else: 
            self.phases[1,:,:] = 0.0
        if self.phase_size_derivatives >=2:
            self.phases[2,:,:] = _np.asarray(data.phaseAccelerations).reshape((data.states, data.states))
        else: 
            self.phases[2,:,:] = 0.0
        
        self.activations[:,:] = _np.asarray(data.activations).reshape((data.states, data.states))
        if self._profiler is not None: self._profiler.enable()
        self.update()
        if self._profiler is not None: self._profiler.disable()



    def update(self):
        now =  rospy.Time.now()

        #compute the expected current state distribution by integration from the last timestep using the (expected) system dynamics:
        self.timeintegrator.integrate(self.last_msd, self.integration_timestep, self.integration_substeps)
        
        #TODO: query self.kinematicsModel for an Yref,Xref,T map to EE
        task_spaces = {}
    
        #core call: get the mixed m-state distribution:
        self.mixer.mix(self.msd_generators, self.activations, self.phases, self.timeintegrator.msd_current, task_spaces)
        
        self._updateStateMarker() #update the RViz marker that shows which components are active

        #pulish mixer result:
        mixed_msd_msg = MechanicalStateDistribution(stamp = now, 
                r=self.index_sizes['r'], 
                g=self.index_sizes['g'], 
                d=self.index_sizes['d'], 
                position_rg=mixed_msd.name2rg['posiition'],
                torque_rg=mixed_msd.name2rg['torque'],
                means = mixed_msd.getMeansData().reshape(-1).toList(),
                covariances = mixed_msd.getCovariancesData().reshape(-1).toList() 
        )
        self.MStatePublisher.publish(mixed_msd_msg)

        self.last_msd = mixed_msd

        
    def run(self):
        rate = rospy.Rate(self.updateFrequency) # 10hz
        while not rospy.is_shutdown():
            rate.sleep()
            if self.activationsTime is not None:
                #self.update()
                pass
        if self._profiler is not None:
            import StringIO, pstats
            s = StringIO.StringIO()
            pstats.Stats(self._profiler, stream=s).sort_stats('cumulative').print_stats()
            print(s.getvalue())


try:
    os.chdir(os.environ["ROS_DATA"])
except KeyError:
    ROS_ERROR("Please set ROS_DATA ( e.g. ROS_DATA=${PWD} )")
    raise SystemExit(1)
    
rospy.init_node('msd_mixer')
mixernode = MixerNode(rospy.get_param('DefinitionsDirectory', 'behavior/'), doProfiling=False)

rospy.loginfo("msd_mixer: Waiting for a current robot posture on /panda/currentstate..")
currentrobotPosture = rospy.wait_for_message('/panda/currentstate', RobotState8)
mixernode.setStartPosition(currentrobotPosture)
import time
time.sleep(0.1)
rospy.loginfo("msd_mixer: Starting the mixer")
mixernode.run()


