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


import numpy as _np
_np.set_printoptions(precision=4, suppress=True, formatter={'float': '{: 0.3f}'.format})
import os


import rospy
from sensor_msgs.msg import JointState
from panda_msgs_mti.msg import PDControllerGoal8, RobotState8, RobotEEState
import phastapromp
import promp
import pandadynamicsmodel

rospy.init_node('emulated_pdcontroller')


#import the mass model of panda necessary, for simulation
try:
    dynamicsModel = pandadynamicsmodel.PandaDynamicsModel()
except RuntimeError:
    print("pdcontrolleremulator: Warning: Using fake unit-mass dynamics!")
    dynamicsModel = promp.FakeDynamicsModel(8)

urdfModel = pandadynamicsmodel.PandaURDFModel()

def getJointParameters(jointNameList):
    """
    read joint parameters (i.e. limits) from the robot's urdf description
    """
    import xml
    import rosparam

    jointNameSet = set(jointNameList)
    urdfTree = xml.etree.ElementTree.fromstring(rosparam.get_param('/robot_description')) 
    jointParameters = {}
    for joint in urdfTree.findall('joint'):
        name = joint.attrib['name']
        if name not in jointNameSet:
            continue
        limits = joint.find('limit').attrib
        jointParameters[name] = {
            'name': name,
            'type': joint.attrib['type'],
            'lower limit': float(limits['lower']),
            'upper limit': float(limits['upper']),
            'velocity limit': float(limits['velocity']),
            'effort limit': float(limits['effort']),
        }
    return jointParameters



watchdogPeriod = rospy.Duration.from_sec( rospy.get_param('watchdog timeout', 0.5))
jointaccmax = rospy.get_param('max joint acceleration', 4.0) #rad²/s²
performance_margin = rospy.get_param('performance margin', 1.0)  #factor of how much of the available performance (effort, velocity) is allowed to be used


updateFrequency = rospy.get_param('frequency', 50.0)
substeps=int(rospy.get_param('substeps time integration', 10)) #do Euler integration on a smaller dt:


warnIfLimitsReached= rospy.get_param('warn limits', False)

dt = 1.0 / updateFrequency
rate = rospy.Rate(updateFrequency) 
substepdt = dt / substeps



#indices for the mechanical states dimension:
iTau = 0
iPos = 1
iVel = 2

mechanicalStateCount = 3
dofs = 8
jointNameList  = ["panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6","panda_joint7", "panda_finger_joint1", "panda_finger_joint2"]

jointParameters = getJointParameters(jointNameList)
jointposmax = _np.zeros((dofs))
jointposmin = _np.zeros((dofs))
jointvelmax = _np.zeros((dofs))
jointtaumax = _np.zeros((dofs))
jointtaumin = _np.zeros((dofs))
for i in range(dofs):
    p=jointParameters[jointNameList[i]]
    jointposmax[i] = p['upper limit']
    jointposmin[i] = p['lower limit']
    jointvelmax[i]  = p['velocity limit']* performance_margin
    jointtaumax[i]  = p['effort limit'] * performance_margin
    jointtaumin[i] = - jointtaumax[i] * performance_margin


#fix finger joint limits
jointposmax[-1] = 0.2

currentJointState = _np.zeros((mechanicalStateCount, dofs))
#set start posture to the controller goal of the first state:
currentJointState[iPos,:] = rospy.get_param('phastapromp/state_controllers')[rospy.get_param('phastapromp/initial state', 0)]['desiredPosition']

goal = _np.array(currentJointState)
kp = _np.zeros((dofs))
kv = _np.zeros((dofs))
kd = _np.full((dofs), 15.0)
ktau = _np.ones((dofs))
watchDogTriggered=True
startTime = rospy.Time.now() - rospy.Duration(dt)
goalTime = startTime

currentJointStateMsg = _np.zeros((mechanicalStateCount,dofs+1))

#listener for new controller goals/gains:
def ControllerGoalCallback(data):
    global goal, kp,kv, goalTime, jointposmin, jointposmax, jointvelmax, jointaccmax, jointtaumax
    newgoalTime = data.stamp
    newgoal = _np.empty((mechanicalStateCount, dofs))
    newgoal[iTau,:] = data.torque
    newgoal[iPos,:] = data.position
    newgoal[iVel,:] = data.velocity
    newkp = _np.empty((dofs))
    newkp[:] = data.kp
    newkv = _np.empty((dofs))
    newkv[:] = data.kv
    
    #for safety: limit positions:
    posLimited = _np.clip(newgoal[iPos,:], jointposmin, jointposmax)
    limitAmount = newgoal[iPos,:] - posLimited
    if _np.any( _np.abs(limitAmount >1e-5)) and warnIfLimitsReached:
        rospy.loginfo("limiting positions by:{0}".format(limitAmount))
    newgoal[iPos,:] = posLimited

    #for safety: try to limit torques:
    tauLimited = _np.clip(newgoal[iTau,:], jointtaumin, jointtaumax)
    newgoal[iTau,:] = tauLimited

    delta = newgoal - goal
    dt  = (newgoalTime - goalTime).to_sec()
    
    #for safety: try to limit acceleration
    deltaAcc = delta[iPos,:] - goal[iVel,:]
    deltaAcc[ currentJointState[iVel,:] * deltaAcc <= 0.0 ] = 0.0  #don't limit decelerating actions
    jointaccmax = 10.0 # _np.dot(jointSpaceInvMassMatrix, jointtaumax)
    accelerationBudget = _np.min( jointaccmax*dt /  (_np.abs(deltaAcc) +1e-7) )
    if _np.any(accelerationBudget < 1.0):
        delta = delta * accelerationBudget
        if warnIfLimitsReached:
            rospy.loginfo("limiting acceleration")

    #for safety: try to limit velocity
    velocityBudget = _np.min(jointvelmax*dt /  (_np.abs(delta[iVel,:])+1e-7))
    if _np.any(velocityBudget < 1.0):
        delta = delta * velocityBudget
        if warnIfLimitsReached:
            rospy.loginfo("limiting velocity")


    #publish new goal data:
    goal = goal + delta
    goalTime = newgoalTime
    kp[:] = newkp
    kv[:] = newkv
    kd[:] = 0


#publisher for the emulated robot pose:
jointMsg = JointState()
jointMsg.name=jointNameList
desiredJointPublisher = rospy.Publisher("joint_states", JointState,  queue_size=3)
desiredControllerGoalListener = rospy.Subscriber("panda/pdcontroller_goal", PDControllerGoal8, ControllerGoalCallback, queue_size=3)

currentStateMsg = RobotState8()
currentStateMsg.q = [0]*8
currentStateMsg.dq = [0]*8
currentStateMsg.ddq = [0]*8
currentStateMsg.tau = [0]*8
currentStateMsg.tau_ext = [0]*8
currentStateMsg.qd = [0]*8
currentStateMsg.dqd = [0]*8
currentStateMsg.mode = 0
currentStatePublisher = rospy.Publisher("/panda/currentstate", RobotState8,  queue_size=3)

# Create Publisher for current jacobian
currentEEStateMsg = RobotEEState()
currentEEStateMsg.jacobian_ee = [0]*42
currentEEStatePublisher = rospy.Publisher("/panda/currentEEstate", RobotEEState,  queue_size=3)

rospy.loginfo('starting up.')
while not rospy.is_shutdown():
    rate.sleep()
    now  = rospy.Time.now()
    deltaTime = (now - startTime)

    #if no recent goal was posted, do some damage control
    if goalTime + watchdogPeriod < now:
        if watchDogTriggered == False:
            rospy.logwarn("nobody is sending me updates!! Stopping for safety.")
        watchDogTriggered = True
        goal[iTau,:] = 0.0
        goal[iVel,:] = 0.0
        kp[:] = 0
        kv[:] = 0
        kd[:] = 15.
    else:
        if watchDogTriggered == True:
            watchDogTriggered = False
            rospy.logwarn("Resuming control.")

##for debugging: print commanded values:    
#    rospy.logwarn("==")
#    rospy.logwarn("kp: {0}".format(kp))
#    rospy.logwarn("kd: {0}".format(kd))
#    rospy.logwarn("Tau: {0}".format(goal[iTau,:]))
#    rospy.logwarn("Pos: {0}".format(goal[iPos,:]))
#    rospy.logwarn("Vel: {0}".format(goal[iVel,:]))

    #emulate thresholded gripper:
    if goal[iPos,dofs-1] > 0.03:
        goal[iPos,dofs-1] = 0.06
    else:
        goal[iPos,dofs-1] = 0.0
    goal[iVel,dofs-1] = 0.0
    goal[iTau,dofs-1] = 0.0

    
    #emulate the pd control:
    #PD control law:
    for i in range(substeps):
        delta = goal - currentJointState
        torques = ktau * goal[iTau,:] + kp * delta[iPos,:] + kv * delta[iVel,:] - kd * currentJointState[iVel,:]
        
        #emulate effect:
        #integrate over the given time interval:
        jointSpaceInertiaMatrix =  dynamicsModel.getInertiaMatrix(currentJointState[iPos,:])
        jointSpaceInvInertiaMatrix = _np.linalg.inv(jointSpaceInertiaMatrix)
        
#        print(_np.linalg.eigvals(jointSpaceInertiaMatrix))
#        print(_np.diag(jointSpaceInvInertiaMatrix[:7,:7]))
        
        friction_torque = currentJointState[iVel,:] * dynamicsModel.getViscuousFrictionCoefficients(currentJointState)
        ddq = _np.dot(jointSpaceInvInertiaMatrix, torques - friction_torque  )
        currentJointState[iVel,:]  =  currentJointState[iVel,:] + ddq * substepdt
        currentJointState[iPos,:]  =  currentJointState[iPos,:] + currentJointState[iVel,:] * substepdt + 0.5 * ddq * substepdt*substepdt
        currentJointState[iTau,:] = torques
    currentJointState[iTau,:] = _np.clip(currentJointState[iTau,:], jointtaumin, jointtaumax)
    currentJointState[iVel,:] = _np.clip(currentJointState[iVel,:], -jointvelmax, jointvelmax)
    currentJointState[iPos,:] = _np.clip(currentJointState[iPos,:], jointposmin, jointposmax)

    currentStateMsg.q[:] = currentJointState[iPos,:] 
    currentStateMsg.dq[:] =  currentJointState[iVel,:]
    currentStateMsg.tau[:] = currentJointState[iTau,:]
    currentStateMsg.qd[:] = goal[iPos,:]  
    currentStateMsg.dqd[:] = goal[iVel,:]
    currentStateMsg.mode = 0
    currentStatePublisher.publish(currentStateMsg)

    # Jacobian
    urdfModel.setJointPosition(currentJointState[iPos,:])
    jacoKDLBase = urdfModel.getJacobian()

    # hTransform 
    hTransform = urdfModel.getEELocation()

    # Calculate EE Jacobian from Base 
    hTransformI = _np.linalg.inv(hTransform)
    hTranslateSkewSym = _np.array([ [0,-hTransformI[2,3],hTransformI[1,3]],
                                    [hTransformI[2,3], 0, -hTransformI[0,3]],
                                    [-hTransformI[1,3], hTransformI[0,3], 0]])
    hTransformIAdjoint =  _np.vstack((_np.hstack((hTransformI[0:3,0:3], _np.zeros((3,3)))), _np.hstack((_np.matmul(hTranslateSkewSym,hTransformI[0:3,0:3]),hTransformI[0:3,0:3]))))
    jacoKDLEE = _np.matmul(hTransformIAdjoint,jacoKDLBase)

    # convert numpy to column major format list
    pos_count = 0
    for column in range(jacoKDLBase.shape[1]):
        for row in range(jacoKDLBase.shape[0]):
            currentEEStateMsg.jacobian_ee[pos_count] = jacoKDLEE[row][column]
            currentEEStateMsg.jacobian_base[pos_count] = jacoKDLBase[row][column]
            pos_count += 1

    # check if jacobian has right size 6x7
    if not len(currentEEStateMsg.jacobian_ee) == 42:
        rospy.logerr("Emulated jacobian has size %d, but should have 42.", len(currentEEStateMsg.jacobian_ee))

    # convert numpy to column major format list
    pos_count = 0
    for column in range(hTransform.shape[1]):
        for row in range(hTransform.shape[0]):
            currentEEStateMsg.htransform[pos_count] = hTransform[row][column]
            pos_count += 1

    # check if hTransform has right size 4x4
    if not len(currentEEStateMsg.htransform) == 16:
        rospy.logerr("Emulated hTransform has size %d, but should have 16.", len(currentEEStateMsg.htransform))

    # publish jaco + hTransform
    currentEEStateMsg.stamp = now
    currentEEStatePublisher.publish(currentEEStateMsg)
    
    
    # for the JointStateMsg, split pos, vel, effort for the gripper dof into two states:
    currentJointStateMsg[:,:dofs-1] =     currentJointState[:,:dofs-1]
    currentJointStateMsg[:, dofs-1] = 0.5*currentJointState[:, dofs-1]
    currentJointStateMsg[:, dofs  ] = 0.5*currentJointState[:, dofs-1]
    jointMsg.position = currentJointStateMsg[iPos,:]
    jointMsg.velocity = currentJointStateMsg[iVel,:]
    jointMsg.effort  =  currentJointStateMsg[iTau,:]
    jointMsg.header.stamp = now 
    desiredJointPublisher.publish(jointMsg)
    
    

