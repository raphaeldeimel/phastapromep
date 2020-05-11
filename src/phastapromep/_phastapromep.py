#!/usr/bin/env python
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

This script takes a recorded session of demonstrated motions, segments them according to a labeled cutlist, learns promeps for each labeled transition and constructs a state graph for sequencing the promeps.

Cutlists can be created with LabelApp.py

"""

import warnings
import numpy as _np
from scipy.special import betainc as _betainc
from  matplotlib import cm
import matplotlib.pylab as _pl
import matplotlib.pyplot as plt

import pandas as _pd

import os
import sys
import copy

try:
    import promep as _promep
except ImportError as e:
    if "promep" in e.message:
        print("\nCould not find promep. Please download/install the python-promep package!\n”")
    raise e

try:
    from ruamel import yaml   #replaces deprecated pyYAML
except ImportError as e:
    if "ruamel" in e.message:
        print("\nCould not find ruamel. Please download/install the python-ruamel package!\n”")
    raise e

def yaml_constructor(loader, node):
    """ fixing bug with unicode patterns in yamlconfig File"""   
    return node.value
yaml.SafeLoader.add_constructor("tag:yaml.org,2002:python/unicode", yaml_constructor)



def ExtractStateGraph(observationsMetadata):
    """
    Extract the state graph that is implicit in the sequential trajectoriesMetadata:

    Heuristic:
        Assume that observations are paths through the state graph
        Paths start and end at "discarded" observations, or at the start or end of the observationsMetadata
    """
    #Assign state numbers and collect likely transitons:
    stateCount = 0
    StateNumberMap = {}
    Transitions = {}  # label -> (prev/next) tuple
    TransitionSampleCount = {}  # label -> sample count
    TransitionLabels = {}

    observationsMetadata_padded = []
    observationsMetadata_padded.append({'Label':'discarded'})
    for i in range(len(observationsMetadata)): 
        observationsMetadata_padded.append(dict(Count     = observationsMetadata['samples'][i], # TODO: Count +1?
                             Label     = observationsMetadata['label'][i],
                             Starttime = observationsMetadata['inpoint'][i],
                             Stoptime  = observationsMetadata['outpoint'][i],
                             Duration  = observationsMetadata['duration'][i],
                             )
                        )
    observationsMetadata_padded.append({'Label':'discarded'})

    for observation_prev, observation, observation_next in zip(observationsMetadata_padded[:-2], observationsMetadata_padded[1:-1],  observationsMetadata_padded[2:]):
        #if preceeding/succeedings state of a transition has not been assigned a number yet, do so:
        label_prev = observation_prev['Label']
        label = observation['Label']
        label_next = observation_next['Label']
        if label == 'discarded': #skip the observation
            continue

        predecessor, successor = (-1, -1)
        n = 0
        if label in Transitions: #we have already seen that transition, get the associated states
            predecessor, successor = Transitions[label]
            n = TransitionSampleCount[label]

        if predecessor == -1: #preceeding state not known yet
            if label_prev in Transitions:  #previous transition is known, assume they share states
                preceedingtransititon_successor = Transitions[label_prev][1]
                if preceedingtransititon_successor == -1: #both transitions haven't had a state assigned yet! Do it now.
                    Transitions[label_prev][1]  = stateCount #else create new successor state
                    stateCount += 1
                predecessor = Transitions[label_prev][1] #assign the existing shared stateLabelAppDatahandler.py

        if successor == -1:  #succeeding state not known yet
            if label_next in Transitions:  #next transition is known, assume they share states
                succeedingtransititon_predecessor =Transitions[label_next][0]
                if succeedingtransititon_predecessor == -1: #both transitions haven't had a state assigned yet! Do it now.
                    Transitions[label_next][0] = stateCount
                    stateCount += 1
                #assign the existing shared state
                successor = Transitions[label_next][0]


        #sanity-check :
        if label_prev in Transitions:
            preceeding_successor = Transitions[label_prev][1]
            if preceeding_successor != predecessor and preceeding_successor != -1:
                print("Warning: preceeding Successor of {0} and Predecessor of {1} do not match!".format(label_prev, label))
                print( observation_prev["Count"],observation_prev["Label"], observation["Count"],observation["Label"], preceeding_successor, predecessor )
        if label_next in Transitions:
            succeeding_predecessor = Transitions[label_next][0]
            if succeeding_predecessor != successor and succeeding_predecessor != -1:
                print("Warning: Successor of {0} and succeeding predecessor of {1} do not match!".format(label, label_next))
                print(observation["Count"],observation["Label"], observation_next["Count"],observation_next["Label"], succeeding_predecessor, successor )

        #save transition and sample count:
        Transitions[label] = [predecessor, successor]
        TransitionSampleCount[label] = n + 1

    #go through all transitions and create states for any unassigned predecessor/successor states (i.e. terminal states and start states)
    for label in Transitions:
        if Transitions[label][0] == -1:
            Transitions[label][0] = stateCount
            stateCount += 1
        if Transitions[label][1] == -1:
            Transitions[label][1] = stateCount
            stateCount += 1


    #transform transitions list into a list of successor states:
    successors = [ [] for i in range(stateCount) ]
    for label in Transitions:
        i,j = Transitions[label]
        successors[i].append(j)

    config = {}
    config['number_of_states'] = stateCount
    config['successors'] = successors
    config['transition_names'] = Transitions
    config['demonstrations_count'] = TransitionSampleCount
    return config



def learnGraphFromDemonstration(demonstrationSessionConfig, observationsHDFStore = None):
    """
    
    Main function to create a state graph and learn the primitives associated with the edges
    as well as the static goals associated with states
    
    """
    jointsList = []
    os.chdir(os.environ["ROS_DATA"])
    #get the state graph:
    if observationsHDFStore is None: #if no database is given, try to look up its name from the config file: 
        observationsHDFStore = _pd.HDFStore(config['hdf5 filename'])
    
    #get metadata from database:
    observationsMetadata = observationsHDFStore.get('metadata')
    avilableLabels = observationsHDFStore.get('available_labels')
    
    graphconfig = ExtractStateGraph(observationsMetadata)
    print("GraphConfig: {}".format(graphconfig))
    if 'joint indices list' in demonstrationSessionConfig:
        jointsList = demonstrationSessionConfig['joint indices list']
    else:
        jointsList  = None

    if jointsList is None:
        graphconfig['dofs'] = None
    else:
        graphconfig['dofs'] = len(jointsList)
        
    observationsPerLabel =  observationsMetadata['label'].value_counts()
    for label in observationsPerLabel.index:
        if observationsPerLabel[label] < demonstrationSessionConfig['minimum sample count']:
            print("Warning: Transition {0} has only {1} observations!. ProMeP quality may suffer.".format(label,observationsPerLabel[label]))
    phaseVelocityProfileAssumption=demonstrationSessionConfig['promep learner velocity profile assumption']
    
    primitives=[]
    for label in graphconfig['transition_names']:
        print("Learning {0} using {1} observations ...".format(label,observationsPerLabel[label]))

        if label in ('None','unlabeled','discarded'):
            continue #skip

        #load/combine parameters for the promep to learn: 
        learningParams = dict(demonstrationSessionConfig[u'promep learning prior']['defaults'])
        if label in demonstrationSessionConfig[u'promep learning prior']:
            learningParams.update(learningParams[label])
        
        observations_indices = observationsMetadata[ observationsMetadata['label'] == label]['i']

        if learningParams['trim_strategy'] != 'inpoint_outpoint':
            raise NotImplementedError("Only trim_stragey: inpoint_outpoint is supported!")
        observations=[]
        for i in observations_indices:
            observation_untrimmed = observationsHDFStore.get('observations/observation{0}'.format(i))
            metadata = observationsMetadata.iloc[i]

            #trim data:
            observation = (observation_untrimmed.iloc[metadata['inpoint']:metadata['outpoint']]).copy(deep=True)

            #annotate observations with phase values (maybe learn in the future, or use phasta simulation to generate more accurate profiles)
            t = observation['observed', 'time','t']
            duration = t.iloc[-1]-t.iloc[0]
            time_normalized = (t - t[0]) / duration
            if phaseVelocityProfileAssumption == 'constant':
                phi = time_normalized
            elif phaseVelocityProfileAssumption == 'beta22':
                phi = _betainc(2,2,time_normalized.as_matrix()) # expensive to compute
            elif phaseVelocityProfileAssumption == 'kumaraswamy1.400,1.455':
                phi = _promep._kumaraswamy.cdf(1.400,1.455,time_normalized.as_matrix()) #cheap approximation of beta(2,2)
            elif phaseVelocityProfileAssumption == 'kumaraswamy1.645,1.800':
                phi = _promep._kumaraswamy.cdf(1.645,1.800,time_normalized.as_matrix()) #closer to phase-state-machine's usual phase profile
            elif phaseVelocityProfileAssumption == 'sine': #sinusoidal phase velocity
                phi = 1.0 - 0.5*_np.cos(time_normalized * _np.pi )
            else:
                raise ValueError("Unknown phase velocity profile specified")
            
            #synthesize phase observations and add them to the observation:
            dt = t.diff()
            dphi = phi.diff()
            ddphi = dphi.diff()
            observation[('observed', 'phi', 0)] = phi
            observation[('observed', 'phi', 1)] = ( dphi/ dt ).interpolate(method='index', limit_direction='both')
            observation[('observed', 'phi', 2)] = (ddphi / dt**2 ).interpolate(method='index', limit_direction='both') #todo: check

            #append to the observations:
            observations.append(observation)


        index_sizes = {
                'r':  2,  
                'rtilde':  2,  
                'g':  2,
                'gtilde': 2,
                'gphi':  2,
                'stilde': learningParams['supports_count'],
                'd':  graphconfig['dofs'],
                'dtilde':  graphconfig['dofs'],
        }
        p = _promep.ProMeP(name=label, index_sizes=index_sizes)
        p.learnFromObservations(observations)
        primitives.append(p)

    if "promep learner use fake torquesList" in demonstrationSessionConfig: #copy values to fake over to the behavior configuration
        if demonstrationSessionConfig["promep learner use fake torquesList"]: #copy values to fake over to the behavior configuration
            graphconfig["fake effort"] = demonstrationSessionConfig["fake torque covariances"]

    #set the phasta parameters:
    if "phasta_parameters" in demonstrationSessionConfig:
        graphconfig["phasta_parameters"] = demonstrationSessionConfig["phasta_parameters"]
    else:
        graphconfig["phasta_parameters"] = {
                  'alpha': 20.0, 
                  'epsilon': 1e-5, 
                  'nu': 2.0, 
                  'dt': 1e-2,
                  'initial_bias': 1e-9,
                  'initial_transition_velocity_exponents': -5,
        }
        
    graphconfig['state_controllers'] = createPDControllerParamsFromDemonstration(graphconfig, primitives, demonstrationSessionConfig)
    
    graphconfig['mixer_weighing_method'] = demonstrationSessionConfig['mixer_weighing_method']
    
    return (graphconfig, primitives)


def createPDControllerParamsFromDemonstration(graphconfig, primitives, sessionconfig):
    """
    create a set of state controllers for the given graph configuration and the give controller parameters

    The controllers are all PD-controllers, desired position is set from
    averaging the final position of all primitives leading to the controller's state

    keywords used in graphconfig:
        'number_of_states' : count of controllers/states
        'dofs' : count of dofs to control

    keywords used in sessionconfig:
        'controller defaults': dictionary of parameter dictionaries for each state, and default parameters

    """
    #compute state controller parameters:
    defaultControllerParameters = sessionconfig['controller defaults']

    stateControllerParams=[]
    tns  = _mechanicalstate.makeTensorNameSpaceForMechanicalStateDistributions()
    mixer = _mechanicalstate.Mixer(tns) #used to blend ends/starts of Primtives into a state controller
    msd_generators_list = primitives
    for i in range(graphconfig['number_of_states']):

        nsamples = 0
        positionsSummed = _np.zeros((1,graphconfig['dofs']))
        means = []
        inverseCovariances = []
        activations_list = _np.zeros((len(primitives)))
        phases_generalized = _np.zeros((mixer.tns['gphi'].size, len(primitives)))
        
        #determine controller name
        likelyControllerName =  None
        for p_id,p in enumerate(primitives):
            ij = graphconfig['transition_names'][p.name]
            if ij[1] == i:  #is incoming transition?
                 if likelyControllerName is None:
                     if p.name.startswith('to'):
                         likelyControllerName = "state {0}".format(p.name[2:])
                     if p.name.startswith('to_'):
                         likelyControllerName = "state {0}".format(p.name[3:])
                     if 'succeeding controller name hints' in sessionconfig:
                         if p.name in sessionconfig['succeeding controller name hints']:
                            likelyControllerName = sessionconfig['succeeding controller name hints'][p.name]
            if ij[0] == i:  #is outgoing transition?
                 if likelyControllerName is None:
                     if p.name.startswith('from'):
                         likelyControllerName = "state {0}".format(p.name[2:])
                     if p.name.startswith('from_'):
                         likelyControllerName = "state {0}".format(p.name[3:])
        if likelyControllerName is None:
            likelyControllerName = "state {0}".format(i) #if no hints were found, use a generic name
        
        #get controller parameters from sessionconfig:
        params = { k : copy.deepcopy(defaultControllerParameters[k]) for k in [u'kp', u'kv', u'expectedTorqueNoise', u'learnFrom'] }
        if likelyControllerName in sessionconfig['controller defaults']:
            params.update(sessionconfig['controller defaults'][name])
        params['name'] = likelyControllerName
        params['type'] = 'PDController'
        
        #select which primitive to use for estimating desired position
        for p_id,p in enumerate(primtives):
            ij = graphconfig['transition_names'][p.name]
            phases_generalized[:, p_id] = 0.0 #set all phase derivatives to zero first
            if ij[1] == i and params[u'learnFrom'] != u'outgoing':  #is incoming transition?
                 activations_list[p_id]  = 1.0
                 phases_generalized[0, p_id] = 1.0
            if ij[0] == i and params[u'learnFrom'] != u'incoming':  #is outgoing transition?
                 activations_list[p_id]  = 1.0
                 phases_generalized[0, p_id] = 0.0
        phaseVelocityVectorPrimtives = _np.ones(len(primtives)) #position controllers
        
        with warnings.catch_warnings():
            warnings.simplefilter("ignore") #we mix with full activations, therefore the sum is usually greater than 1. suppress the mixer's warning here
            mixer.mix(msd_generators_list, activations_list, phases_generalized, initial_msd)
            mixer.mix(msd_generators_list, activations_list, phases_generalized, mixer.mixed_msd)
            mixer.msd_mixed            
        params['desiredPosition']  = mixer.msd_mixed.getMeansData('position').tolist()
        stateControllerParams.append(params)
    return stateControllerParams



def saveGraphDefinitionsToDirectory(graphconfig, promps, DefinitionsDirectory):
    """
    Saves all data needed to recreate the graph completely into the given directory

    The function creates the following files:
        - phasta.yaml: Main definition of the graph, and the association between
                       transitions and movement primitives, and states and state controllers
        - <name>.promp.mat: Serialized Parameters of the ProMP trajectory generators
    """
    if not os.path.exists(DefinitionsDirectory):
        os.makedirs(DefinitionsDirectory)
    #create the output directories to save the illustrations to:
    #save the learned state graph, the promps, and illustrate state graph and promps:
    #print("Saving learnt graph and movement primitives to {0}".format(DefinitionsDirectory))
    with open(os.path.join(DefinitionsDirectory, 'phasta.yaml'), 'w') as f:
        yaml.dump(graphconfig, f)
    for p in promps:
        p.saveToFile(path=DefinitionsDirectory)

def _plotStateGraph(graphconfig, filename):
    """
    plots the state graph using graphviz
    """
    try:
        import pygraphviz
    except ImportError as e:
        if "pygraphviz" in e.message:
            print("\nPlease download/install the python-pygraphviz package!\n")
        raise e
    G = pygraphviz.AGraph(directed=True, sep=1.0)
    for label in graphconfig['transition_names']:
        i,j = graphconfig['transition_names'][label]
        G.add_edge(i, j, label, label=label)
    G.layout(prog='dot')
    G.draw(filename)


def illustrateGraph(graphconfig, promps, IllustrationsDirectory):
    """
    Draws the state graph and illustrates the learned promps.

    Files get placed in IllustrationsDirectory
    """
    if not os.path.exists(IllustrationsDirectory):
        os.makedirs(IllustrationsDirectory)
    _plotStateGraph(graphconfig, os.path.join(IllustrationsDirectory, 'stategraph.pdf') )

#    for controllerconf in graphconfig["state_controllers"]:
#        _pl.figure()
#        promp.plotYSpaceCovarianceTensor(cov=_np.array(controllerconf["desiredCovariances"]), interpolationKernel=promps[0].interpolationKernel)
#        _pl.savefig(os.path.join(IllustrationsDirectory, controllerconf["name"] + "_goalCovariances.pdf"))
#        _pl.close()

    for p in promps:
        _pl.figure()
        p.plotCovarianceTensor(normalized=True)
        _pl.savefig(os.path.join(IllustrationsDirectory, p.name + "_correlations.pdf"))
        _pl.close()
        _pl.figure()
        p.plotCovarianceTensor(normalized=False)
        _pl.savefig(os.path.join(IllustrationsDirectory, p.name + "_covariances.pdf"))
        _pl.close()
        _pl.figure()
        p.plot(withSampledTrajectories=15, withSupportsMarked=False, posLimits=[-3.2, 3.2], velLimits=[-2.2, 2.2], )
        _pl.savefig(os.path.join(IllustrationsDirectory, p.name + ".pdf"))
        _pl.close('all')



