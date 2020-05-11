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

from PyQt5 import QtCore, QtGui

import numpy as _np
_np.set_printoptions(precision=2, suppress=True, formatter={'float': '{: 0.3f}'.format})
import os

import promp
import promp._tensorfunctions as _t
import phasestatemachine
import phasestatemachine.msg
import phastapromep
import yaml


import rospy
from panda_msgs_mti.msg import MechanicalStateDistribution8TorquePosVel, RobotState
from sensor_msgs.msg import JointState
import pyqtgraph as _pg
import pyqtgraph.exporters



class MStateMonitorNode(object):

    def __init__(self, graphicslayoutInstance, pensDict, rate=50, showDuration=10.0, withFills=True, withJointStates=True):
        """
        """
        self.dofs = 8
        self.mstates = 3
        self.iTau=0
        self.iPos=1
        self.iVel=2
        self.mstateNames = ["Torque", "Position", "Velocity"]
        self.nStride = int(rate * showDuration * 2.1)
        self.minRecordingTimestep = 0.99 / rate
        self.isWithJointStates = withJointStates
        self.means  = _np.zeros((self.dofs, 3, self.nStride))
        self.sigmas = _np.zeros((self.dofs, 3, self.nStride))
        self.gains  = _np.zeros((self.dofs, 2, self.nStride))
        self.velfrompos  = _np.zeros((self.dofs, self.nStride))
        self.jointstates = _np.zeros((self.dofs, 3, self.nStride))    
        self.times  = _np.zeros((self.nStride))
        self.capacity = self.nStride
        self.startTime = None
        self.lastRobotStateMsg = None
        self.lastMsgIndex  = 0
        self.showDuration  = showDuration
        self.showDurationCount = int(self.showDuration * rate)

        self.layout = graphicslayoutInstance
        self.subplots = []
        self.means_curves = []
        self.lowervariancetube_curves = []
        self.uppervariancetube_curves = []
        self.fills = []
        self.jointstate_curves = []
        self.velocity_from_position_curves = []
        self.gains_kp_curves = []
        self.gains_kv_curves = []

        self.pens = pensDict
        

        yranges = {
            "Torque": (-20, 20),
            "Position": (-3.2, 3.2),
            "Velocity": (-1, 1),
            "PD Gain" : (-5, 50)
        }
        for mtype in self.mstateNames:
            for i in range(self.dofs):
                p = self.layout.addPlot(title=mtype)
                #p.setDownsampling(mode='mean')
                #p.setClipToView(True)
                p.setRange(xRange=[-self.showDuration, 0.0], yRange=yranges[mtype])
                p.setLimits(xMax=0, yMin=yranges[mtype][0], yMax=yranges[mtype][1])
                #p.addLine(x=None, y=0.0, z=None)
                self.subplots.append(p)
                if withFills:
                    c_lower = _pg.PlotCurveItem()
                    c_upper = _pg.PlotCurveItem()
                    fill = _pg.FillBetweenItem(c_lower, c_upper,self.pens["confidence fillbrush"] )
                    p.addItem(fill)
                    self.fills.append(fill)
                else:
                    c_lower = p.plot(pen=self.pens["confidencebound"])
                    c_upper = p.plot(pen=self.pens["confidencebound"])
                    
                self.lowervariancetube_curves.append(c_lower)
                self.uppervariancetube_curves.append(c_upper)
                if mtype == 'Velocity':
                    self.velocity_from_position_curves.append(p.plot(pen=self.pens["vel from pos"]))
                self.means_curves.append(p.plot(pen=self.pens["mean desired"]))
                if self.isWithJointStates:
                    self.jointstate_curves.append(p.plot(pen=self.pens["mean actual"]))
            self.layout.nextRow()
        #gains:
        mtype = "PD Gain"
        for i in range(self.dofs):
            p = self.layout.addPlot(title=mtype)
            p.setRange(xRange=[-self.showDuration, 0.0], yRange=yranges[mtype])
            p.setLimits(xMax=0, yMin=yranges[mtype][0], yMax=yranges[mtype][1])
            self.subplots.append(p)
            self.gains_kp_curves.append(p.plot(pen=self.pens["kp"]))
            self.gains_kv_curves.append(p.plot(pen=self.pens["kv"]))
            p.addLine(x=None, y=0.0, z=None, pen=self.pens["dashed"])

        self.update()


    def MStateMsgCallback(self, data):
        if self.startTime is None:
            self.startTime = data.stamp 
        t = (data.stamp - self.startTime).to_sec()
        if (t-self.times[self.lastMsgIndex]) < self.minRecordingTimestep: #ignore updates coming faster than 10Hz
            return
        if self.lastMsgIndex + 1 >= self.capacity:
            n  = self.showDurationCount
            n1 = self.lastMsgIndex-self.showDurationCount
            n2 = self.lastMsgIndex
            self.means[ :,:,:n] =  self.means[:,:,n1:n2]
            self.sigmas[:,:,:n] = self.sigmas[:,:,n1:n2]
            self.gains[ :,:,:n] =  self.gains[:,:,n1:n2]
            self.velfrompos[ :,:n] = self.velfrompos[ :,n1:n2]
            if self.isWithJointStates:
                self.jointstates[ :,:,:n] =  self.jointstates[:,:,n1:n2]
            self.times[:n] =  self.times[n1:n2]
            idx = n
        else:
            idx = self.lastMsgIndex + 1

        self.times[idx] = t
        if self.times[idx] - self.times[self.lastMsgIndex] > 1.0:
            self.means[:,:,self.lastMsgIndex] = _np.nan #cut the data
            self.gains[:,:,self.lastMsgIndex] = _np.nan
            self.sigmas[:,:,self.lastMsgIndex] = _np.nan
            if self.isWithJointStates:
                self.jointstates[:,:,self.lastMsgIndex] = _np.nan
            
        meansData = _np.asarray(data.meansMatrix)
        meansData.shape = (self.mstates,self.dofs)
        self.means[:,:,idx] = meansData.T
        if idx > 0:
            self.velfrompos[:,idx] = (self.means[:,self.iPos,idx] - self.means[:,self.iPos,idx-1]) / (self.times[idx]-self.times[idx-1])
        covTensor = _np.asarray(data.covarianceTensor)
        covTensor.shape = (self.mstates,self.dofs,self.mstates,self.dofs)

        for i in range(self.mstates):
            for dof in range(self.dofs):
                self.sigmas[dof,i,idx] = _np.sqrt(covTensor[i,dof,i,dof])


        #compute pd gains:
        #sigma_qt = covTensor[self.iTau,  :, self.iPos:,:]
        #sigma_qq = covTensor[self.iPos:, :, self.iPos:,:]
        #sigma_qq_inv = _t.pinv(sigma_qq, regularization=0)
        #gains = -_t.dot(sigma_qt, sigma_qq_inv, shapes = ((1,2),(2,2)) ) #gains is (dof, mstate x dof)
        gains = promp.extractPDGains(covTensor)
        for dof in range(self.dofs):
            self.gains[dof,:,idx]= gains[dof, :, dof]
        

        if self.isWithJointStates and self.lastRobotStateMsg is not None:
            if self.lastRobotStateMsg.dofs != self.dofs:
                raise NotImplementedError()
            self.jointstates[:self.dofs,self.iTau,idx] = self.lastRobotStateMsg.tau[:self.dofs]
            self.jointstates[:self.dofs,self.iPos,idx] = self.lastRobotStateMsg.q[:self.dofs]
            self.jointstates[:self.dofs,self.iVel,idx] = self.lastRobotStateMsg.dq[:self.dofs]


        self.lastMsgIndex = idx

    def RobotStateMsgCallback(self, data):
        self.lastRobotStateMsg = data

    def update(self):
        n = self.lastMsgIndex
        n_cutoff = max(0, n - self.showDurationCount)
        t = self.times[n_cutoff:n] - self.times[n]
        for i in range(self.dofs):
            self.gains_kp_curves[i].setData(t, self.gains[i,0, n_cutoff:n], connect='finite')
            self.gains_kv_curves[i].setData(t, self.gains[i,1,n_cutoff:n], connect='finite')
            c = self.velocity_from_position_curves[i]
            c.setData(t, self.velfrompos[i,n_cutoff:n],  connect='finite')
            for j in range(self.mstates):
                idx = i+j*self.dofs
                curve_mean  = self.means_curves[idx]
                curve_lower  = self.lowervariancetube_curves[idx]
                curve_upper  = self.uppervariancetube_curves[idx]
                if self.isWithJointStates:
                    self.jointstate_curves[idx].setData(t, self.jointstates[i,j,n_cutoff:n], connect='finite' )
                
                curve_mean.setData(t, self.means[i,j,n_cutoff:n],  connect='finite')
                curve_upper.setData(t, self.means[i,j,n_cutoff:n] + 1.95 * self.sigmas[i,j,n_cutoff:n],  connect='finite')
                curve_lower.setData(t, self.means[i,j,n_cutoff:n] - 1.95 * self.sigmas[i,j,n_cutoff:n],  connect='finite')




def themePyQTGraph():
    """
    central definition of themes:
    """
    if rospy.get_param("/monitors/dark_theme", True):
        _pg.setConfigOption('background', 'k')
        _pg.setConfigOption('foreground', (150,150,150))
        pens_dict  = {
                "confidencebound": None,
                "confidence fillbrush": _pg.mkBrush(100,100,100),
                "mean desired": _pg.mkPen(255,255,255),
                "mean actual": _pg.mkPen(0,200,0),
                "vel from pos": _pg.mkPen(150,0,0),
                "kp": _pg.mkPen(255,100,100),
                "kv": _pg.mkPen(150,150,255),
                "dashed" : _pg.mkPen(100,100,100, width=1, style=QtCore.Qt.DashLine),
                "trace light weight": _pg.mkPen(100,100,100,width=2.5),
                "trace medium weight": _pg.mkPen(155,155,155,width=2.5),
                "trace full weight": _pg.mkPen(230,230,230,width=2.5),
                "state fillbrush": _pg.mkBrush(255,255,255),
                
        }
    else:
        _pg.setConfigOption('background', 'w')
        _pg.setConfigOption('foreground', (100,100,100))
        pens_dict  = {
                "confidencebound": None,
                "confidence fillbrush": _pg.mkBrush(200,200,200),
                "mean desired": _pg.mkPen(0,0,0),
                "mean actual": _pg.mkPen(50,150,50),
                "vel from pos": _pg.mkPen(150,50,50),
                "kp": _pg.mkPen(200,0,0),
                "kv": _pg.mkPen(0,0,200),
                "dashed" : _pg.mkPen(200,200,200, width=1, style=QtCore.Qt.DashLine),
                "trace light weight": _pg.mkPen(200,200,200,width=2.5),
                "trace medium weight": _pg.mkPen(180,180,180,width=2.5),
                "trace full weight": _pg.mkPen(150,150,150,width=2.5),
                "state fillbrush": _pg.mkBrush(0,0,0),
        }    
    _pg.setConfigOptions(antialias=True)
    return pens_dict

def main(samplerate, duration):
    persistentconfigfile = os.path.join(os.environ["ROS_DATA"], ".mstatemonitor.config.yaml")
    try:
        with open(persistentconfigfile, 'r') as f:
            persistentconfig = yaml.load(f)
            if persistentconfig is None:
                persistentconfig = {}
    except:
        persistentconfig = {}
        pass
    rospy.init_node('mstateMonitor')
    pens_dict = themePyQTGraph()


    aboutToQuit=False
    app = QtGui.QApplication([])
    app.setOrganizationName("MTI-engAge")
    app.setApplicationName("mechanical state monitor")
    app.setWindowIcon(QtGui.QIcon(os.path.join(os.path.dirname(__file__),'../share/icon_mstatemonitor.svg')))
    settings = QtCore.QSettings( "MTI-engAge", "mechanical state monitor")
        
    view = _pg.GraphicsView()
    def guiExitHandler():
        global aboutToQuit
        aboutToQuit = True
        print("GUI closed")
    app.aboutToQuit.connect(guiExitHandler)
    
    l = _pg.GraphicsLayout(border=(100,100,100))
    l.layout.setSpacing(0)
    #l.setContentsMargins(30, 30, 30, 30)

    view.setCentralItem(l)
    view.show()
    view.setWindowTitle('Mechanical State Distribution')
    view.resize(settings.value("size", QtCore.QSize(800, 300))) 
    view.move(settings.value("pos", QtCore.QPoint(200, 200))) 
    
    monitor = MStateMonitorNode(l, pens_dict, samplerate, duration, withJointStates=True)
    skipupdates_n  = int(samplerate / 10)
    MStateListener       = rospy.Subscriber("mixer/mstate_distribution", MechanicalStateDistribution8TorquePosVel, monitor.MStateMsgCallback    , queue_size=3)
    if monitor.isWithJointStates:
        CurrentStateListener = rospy.Subscriber("/panda/currentstate"            ,                               RobotState, monitor.RobotStateMsgCallback, queue_size=3)

    
    rate = rospy.Rate(samplerate)
    i = 0
    while not rospy.is_shutdown() and not aboutToQuit:
        rate.sleep()
        i = i-1
        if i < 0:
            monitor.update()
            i = skipupdates_n
        QtGui.QApplication.processEvents()

    settings.setValue("pos", view.pos())
    settings.setValue("size", view.size())

if __name__ == '__main__':
    main(20, 5.0)
        



