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
@copyright: 2018
@licence: 2-clause BSD licence
"""


import numpy
import numpy as _np
import matplotlib.pylab as _pl
import os as _os

import phastapromep
import phastapromep.msg

import rospy, tf
from sensor_msgs.msg import JointState

import std_msgs.msg

try:
    from ruamel import yaml   #replaces deprecated pyYAML
except ImportError as e:
    if "ruamel" in e.message:
        print("\nCould not find ruamel. Please download/install the python-ruamel package!\n”")
    raise e



def createPhaseStateMachine(behavior_dir):
    """
    from the files in DefinitionsDirectoy, construct a phase-state machine and ProMPMixer

    The directory can be created using saveGraphDefinitionsToDirectory()

    Files needed:
            phasta.yaml:  configuration of the phase-state machine and the associations to promps
            <name>.promp.mat: serialized parameters of the ProMPs to use

    returns: (phasta, mixer)
                phasta: PhaseStateMachine object
                mixer: ProMPMAtrixMixer object, ready to consume phase and activation matrices from the phasta object
    """
    #load the graph configuration and instantiate a phase-state machine

    #load the promp definitions and instantiate a phase-state machine
    with open(os.path.join(behavior_dir, 'phasta.yaml')) as f:
        graphconfig = yaml.safe_load(f)
        
    kwargs = {'numStates': graphconfig['number_of_states'],
              'successors': graphconfig['successors'], 
              'reset' : True
    }

    initialTransitionVelocityExponent = 0
    initialBiasInput = 0.001
    initialPhasesInput = 0.0
    initialVelocityEnslavementGain = 0.0
    initialGreediness = 0.5
    if "phasta_parameters" in graphconfig:
        for keyword in graphconfig["phasta_parameters"]:
            if keyword == "initial_bias":
                initialBiasInput = graphconfig["phasta_parameters"][keyword]
            elif keyword == "initial_transition_velocity_exponents":
                initialTransitionVelocityExponent = graphconfig["phasta_parameters"][keyword]
            elif keyword == "initial phases":
                initialPhasesInput = graphconfig["phasta_parameters"][keyword]
            elif keyword == "initial phase enslavement gain":
                initialVelocityEnslavementGain = graphconfig["phasta_parameters"][keyword]
            elif keyword == "initial greediness":
                initialGreediness = graphconfig["phasta_parameters"][keyword]
            else:
                kwargs[keyword] = graphconfig["phasta_parameters"][keyword]
    phasta = phasestatemachine.Kernel(**kwargs)
    phasta.updateBiases(initialBiasInput)
    phasta.updateTransitionPhaseVelocityExponentInput(initialTransitionVelocityExponent)
    phasta.updatePhasesInput(initialPhasesInput)
    phasta.updateVelocityEnslavementGain(initialVelocityEnslavementGain)
    phasta.updateGreediness(initialGreediness)

    return phasta



rospy.init_node('phasta', log_level=rospy.INFO)




publish_hz = 50.0
n_oversampling = 4
phasta_dt = 1.0 / (n_oversampling * publish_hz)

_os.chdir(rospy.get_param('data_dir'))
behavior_dir = rospy.get_param('behavior_dir', 'behavior/')

with open(_os.path.join(behavior_dir, 'phasta.yaml')) as f:
    graphconfig = yaml.safe_load(f)

#gather all arguments for instantiating phasestatemachine object from the config file:
kwargs = {'numStates': graphconfig['number_of_states'],
            'successors': graphconfig['successors'], 
            'reset' : True
}
initialTransitionVelocityExponent = 0
initialBiasInput = 0.001
initialPhasesInput = 0.0
initialVelocityEnslavementGain = 0.0
initialGreediness = 0.5
if "phasta_parameters" in graphconfig:
    for keyword in graphconfig["phasta_parameters"]:
        if keyword == "initial_bias":
            initialBiasInput = graphconfig["phasta_parameters"][keyword]
        elif keyword == "initial_transition_velocity_exponents":
            initialTransitionVelocityExponent = graphconfig["phasta_parameters"][keyword]
        elif keyword == "initial phases":
            initialPhasesInput = graphconfig["phasta_parameters"][keyword]
        elif keyword == "initial phase enslavement gain":
            initialVelocityEnslavementGain = graphconfig["phasta_parameters"][keyword]
        elif keyword == "initial greediness":
            initialGreediness = graphconfig["phasta_parameters"][keyword]
        else:
            kwargs[keyword] = graphconfig["phasta_parameters"][keyword]

phasta = phasestatemachine.Kernel(**kwargs)
phasta.updateBiases(initialBiasInput)
phasta.updateTransitionPhaseVelocityExponentInput(initialTransitionVelocityExponent)
phasta.updatePhasesInput(initialPhasesInput)
phasta.updateVelocityEnslavementGain(initialVelocityEnslavementGain)
phasta.updateGreediness(initialGreediness)













phasta.updateDt(phasta_dt)


lastBiasesMsg = None

def BiasMatrixCallback(data):
    global lastBiasesMsg
    lastBiasesMsg = data

biasListener = rospy.Subscriber("/phasta/biases", phastapromep.msg.Biases, BiasMatrixCallback, queue_size=1) 

lastGreedinessesMsg = None
def GreedinessesCallback(data):
    global lastGreedinessesMsg
    lastGreedinessesMsg = data

greedinessListener = rospy.Subscriber("/phasta/greedinesses", phastapromep.msg.Greedinesses, GreedinessesCallback, queue_size=1) 


def SpeedMatrixCallback(data):
    global phasta
    if data.states != numStates:
        rospy.logerr("Speed Exponentials message has wrong size {0}x{0} instead of {1}x{1}".format(data.states, numStates))
    Kd = _np.asarray(data.kd).reshape((numStates, numStates))
    phasta.updateTransitionPhaseVelocityExponentInput(Kd)

#speedListener = rospy.Subscriber("/phasta/speedexponentials", phastapromep.msg.SpeedExponentials, SpeedMatrixCallback, queue_size=1) 


phasesActivationsMsg = phastapromep.msg.PhasesActivations()
phasesActivationsMsg.states = phasta.numStates
PhasesActivationsPublisher= rospy.Publisher("/phasta/phases_activations", phastapromep.msg.PhasesActivations, queue_size=3)


statesMsg = phastapromep.msg.StateVector()
statesMsg.states = phasta.numStates
StatePublisher = rospy.Publisher("/phasta/x", phastapromep.msg.StateVector, queue_size=3)

statesMsg = phastapromep.msg.StateVector()
statesMsg.states = phasta.numStates
StatePublisher = rospy.Publisher("/phasta/x", phastapromep.msg.StateVector, queue_size=3)

state1DMsg = std_msgs.msg.Float32()
State1DPublisher = rospy.Publisher("/phasta/state1D", std_msgs.msg.Float32, queue_size=3)

stateConnectivityMsg = phastapromep.msg.StateMatrix()
stateConnectivityMsg.states = phasta.numStates
stateConnectivityMsg.matrix = list(phasta.stateConnectivityAbs.flat)
stateConnectivityMsgPublisher= rospy.Publisher("/phasta/stateconnectivity", phastapromep.msg.StateMatrix, queue_size=3)

rate = rospy.Rate(publish_hz)
startTime = rospy.Time.now()
saystate_last = ""

slow_update_n = 50
slow_update_count = 0

rospy.loginfo("Starting Phase-State machine node")
while not rospy.is_shutdown():
    now  = rospy.Time.now()
    deltaTime = (now - startTime)
    
    #update inputs:
    if lastBiasesMsg is not None:
        data = lastBiasesMsg
        lastBiasesMsg = None
        if data.states != phasta.numStates:
            rospy.logerr("Biases message has wrong size {0}x{0} instead of {1}x{1}".format(data.states, phasta.numStates))
        else:
            #treat the data as matrix/vector/scalar depending on the number of sent values:
            if (len(data.mean) == data.states): #we got a vector, not a matrix:
                B = _np.asarray(data.mean).reshape((phasta.numStates, 1)) * _np.ones((1,phasta.numStates))
                phasta.updateBiases(B)  #todo: make kernel accept variance too
            elif len(data.mean) == data.states*phasta.numStates:
                B = _np.asarray(data.mean).reshape((phasta.numStates, phasta.numStates))
                phasta.updateBiases(B)  #todo: make kernel accept variance too
            elif len(data.mean) == 1:
                B = _np.full((phasta.numStates, phasta.numStates),data.mean[0])
                phasta.updateBiases(B)  #todo: make kernel accept variance too
            else:
                rospy.logerr("Biases message has wrong length of .means: {0} instead of {1} or {2} or 1".format(len(data.mean), phasta.numStates*phasta.numStates,phasta.numStates))
                
    
    if lastGreedinessesMsg is not None:
        data = lastGreedinessesMsg
        lastGreedinessesMsg = None
        n = len(data.greedinesses)
        G = _np.asarray(data.greedinesses)
        #treat the data as matrix/vector/scalar depending on the number of sent values:
        if n == phasta.numStates**2: #we got a vector, not a matrix:
            G = G.reshape((phasta.numStates, phasta.numStates)) 
        elif n == phasta.numStates:
            pass
        elif n == 1:
            G = G[0]
        else:
            rospy.logerr("Greedinesses has wrong size {0}, expecting 1 or {1} or {2}".format(n, phasta.numStates,phasta.numStates**2))
        phasta.updateGreediness(G)
    
    #step the phasestatemachine
    phasta.step(nr_steps=n_oversampling)

    #log changes of state:    
    saystate = phasta.sayState()
    if saystate_last != saystate: 
        rospy.loginfo(saystate)
    saystate_last = saystate
    state1DMsg.data =  phasta.get1DState()
    State1DPublisher.publish(state1DMsg)
    
    #publish metadata:
    slow_update_count-=1
    if slow_update_count < 0:
        slow_update_count = slow_update_n
        stateConnectivityMsgPublisher.publish(stateConnectivityMsg)
    
    #publish stateVector:
    statesMsg.x = phasta.statevector
    statesMsg.stamp = now
    StatePublisher.publish(statesMsg)

    #publish phases and activations
    phasesActivationsMsg.phases = list(phasta.phasesProgress.flat)
    phasesActivationsMsg.phaseVelocities = list(phasta.phasesProgressVelocities.flat)
    phasesActivationsMsg.activations = list(phasta.phasesActivation.flat)
    phasesActivationsMsg.stamp = now
    PhasesActivationsPublisher.publish(phasesActivationsMsg)
    
    rate.sleep()


