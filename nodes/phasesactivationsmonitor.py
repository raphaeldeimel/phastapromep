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

import phastapromep
import phastapromep.msg

import rospy
import pyqtgraph as _pg
import pyqtgraph.exporters


from mstatemonitornode import themePyQTGraph

class PhastaStateEvolutionMonitorNode(object):

    def __init__(self, graphicsView, pens_dict,  showDuration=60.0, timeResolution=0.05):
        """
        """
        self.minRecordingTimestep = timeResolution
        self.nStride =  2*int((showDuration+1.0)/self.minRecordingTimestep)
        self.activityThreshold = 0.01
        self.activation_scaler = 1.0        
        self.startTime = None
        self.showDuration  = showDuration
        self.graphicsView = graphicsView
        self.layout = None
        self.dofs = None
        self.dofsRequested = None
        self.lastUpdatedAtIndex = None
        self.pens_dict = pens_dict

    def layoutGrid(self, dofs, stateConnectivity=None):
        #once dofs are known:
        self.dofs = dofs
        if stateConnectivity is None:
            self.stateConnectivity = 1-_np.eye(self.dofs)
        else:
            self.stateConnectivity = stateConnectivity
        self.activationmatrix = _np.zeros((self.dofs,self.dofs, self.nStride))
        self.phasematrix = _np.zeros((self.dofs,self.dofs, self.nStride))
        
        self.times  = _np.zeros((self.nStride))
        self.firstMsgIndex = 0
        self.lastMsgIndex  = 0
        self.lastCutoffIndex = 1

        layout = _pg.GraphicsLayout()
        layout.layout.setSpacing(0.)                                                             
        layout.setContentsMargins(0., 0., 0., 0.)            
        self.subplots = []
        
        xRange=[-self.showDuration, 0.0]
        yRange=[-0.1, self.dofs+0.1]

        p = layout.addPlot(title="activations over time")
        viewbox = p.getViewBox()
        #p.setDownsampling(mode='mean')
        #p.setClipToView(True)
        p.setRange(xRange=xRange)
        p.setLimits(xMin=xRange[0], xMax=xRange[1])
        p.showGrid(x=False, y=True)
        #p.setLabel('bottom', "time")
        p.setLabel('left', "activation")
        offset_delta =0.0
        offset = 0.0
        self.activation_curves = []
        self.pens_curves = []
        for i in range(self.dofs):
            offset =  offset + offset_delta
            curve = p.plot(pen=self._makePen(i,i))
            self.activation_curves.append( (i,i,curve, offset) )
            p.addLine(y=offset, pen=self.pens_dict["dashed"])        
        
        for i in range(self.dofs):
            for j in range(self.dofs):
                if self.stateConnectivity[j,i] != 0:
                    offset = offset + offset_delta
                    self.activation_curves.append( ( i,j, p.plot(pen=self._makePen(i,j)),offset) )
                    p.addLine(y=offset, pen=self.pens_dict["dashed"])
        self.activations_plot = p

        layout.nextRow()
        yRange=[-0.1, 1.1]
        p = layout.addPlot(title="phases over time")
        p.setLabel('bottom', "time")
        p.setLabel('left', "phase")
        p.setRange(xRange=xRange, yRange=yRange)
        p.setLimits(xMin=xRange[0], xMax=xRange[1], yMin=yRange[0], yMax=yRange[1])
        self.phases_curves = []
        for i in range(self.dofs):
            for j in range(self.dofs):
                if self.stateConnectivity[j,i] != 0:
                    self.phases_curves.append( ( i,j, p.plot(pen=self._makePen(i,j)) ) )
        self.phases_plot = p


        self.graphicsView.setCentralItem(layout)
        if self.layout is not None:
            del self.layout #make clear this is to be dumped
        self.layout = layout

    def _makePen(self, i,j):
        lower  = 50
        upper = 235
        a = 0.5*i/self.dofs
        b = 0.5*j/self.dofs
        r = lower + (upper-lower)*(2*a)
        g = lower + (upper-lower)*(2*b)
        if i==j:
            b = lower + (upper-lower)*i/self.dofs
        else:
            b = lower + (upper-lower)*(1-a-b)
        return _pg.mkPen(r,g,b,width=3.5)


    def connectivityCallback(self, data):
        if data.states != self.dofs and self.dofsRequested is None:  #dofs changed, request update of the view
            self.stateConnectivity = _np.array(data.matrix).reshape((data.states,data.states))
            self.dofsRequested = data.states



    def PhasesActivationsMsgCallback(self, data):
    
        if self.dofs is None: #not initialized yet
            return
        
        if self.startTime is None:
            self.startTime = data.stamp 
        t = (data.stamp - self.startTime).to_sec()
        cutOffTime = t - self.showDuration
        if (t-self.times[self.lastMsgIndex]) < self.minRecordingTimestep: #ignore updates coming faster than 10Hz
            return

        if data.states != self.dofs:
            print("Internal Dofs and received state count differ.")
            return
            
        while self.times[self.firstMsgIndex] < cutOffTime and self.firstMsgIndex < self.lastMsgIndex and (self.lastMsgIndex - self.firstMsgIndex) < self.nStride//2:
            self.firstMsgIndex += 1

        n = self.lastMsgIndex-self.firstMsgIndex + 1
        if self.lastMsgIndex >= self.nStride -1:
            self.phasematrix[:,:,:n] = self.phasematrix[:,:, self.firstMsgIndex:self.lastMsgIndex+1]
            self.activationmatrix[:,:,:n] = self.activationmatrix[:,:, self.firstMsgIndex:self.lastMsgIndex+1]
            self.times[:n] = self.times[self.firstMsgIndex:self.lastMsgIndex+1] - t
            t = 0.0
            self.firstMsgIndex = 0
            self.lastMsgIndex = n-1
            self.startTime = data.stamp
        
        phases = _np.asarray(data.phases).reshape((self.dofs, self.dofs))
        activations =  _np.asarray(data.activations).reshape((self.dofs, self.dofs))
        
        #make inactive controllers invisible to avoid visual clutter:
        invisible = activations < self.activityThreshold
        phases[ invisible ] = _np.nan
        activations[ invisible ] = _np.nan
        
        idx = self.lastMsgIndex + 1
        self.times[idx] = t
        self.phasematrix[:,:,idx] = phases
        self.activationmatrix[:,:,idx] = activations*self.activation_scaler
        self.lastMsgIndex = idx


    def update(self):
        if self.dofsRequested is not None:
            if self.dofsRequested > 0:
                self.layoutGrid(self.dofsRequested, self.stateConnectivity)
            self.dofsRequested = None
            
        if self.layout is None:  #nothing to do yet for drawing
            return
        if self.startTime is None:
            return
        
        n_start = self.firstMsgIndex 
        n_end = self.lastMsgIndex+1
        if n_end - n_start < 10: 
            return

        t =  self.times[n_start:n_end] - self.times[n_end-1] 

        for i,j,curve, offset in self.activation_curves:
            L = self.activationmatrix[j,i,n_start:n_end]
            curve.setData(t, L, connect='finite')
        self.activations_plot.setRange(yRange=[-0.1, offset+1.1])
        
        for i,j,curve in self.phases_curves:
            Phi = self.phasematrix[j,i,n_start:n_end]
            #if _np.any(_np.isfinite(Phi)):
            curve.setData(t, Phi, connect='finite')


def main(updaterate=5):

    rospy.init_node('PhasesActivationsMonitor')


    _pg.setConfigOptions(antialias=True)
    pens_dict = themePyQTGraph()
    
    aboutToQuit=False
    app = QtGui.QApplication([])
    app.setOrganizationName("MTI-engAge")
    app.setApplicationName("Phases and Activations monitor")
    settings = QtCore.QSettings( "MTI-engAge", "Phases and Activations monitor")
    app.setWindowIcon(QtGui.QIcon(os.path.join(os.path.dirname(__file__),'../share/icon_phasesactivationsmonitor.svg')))
    view = _pg.GraphicsView()
    def guiExitHandler():
        global aboutToQuit
        aboutToQuit = True
        print("GUI closed")
    app.aboutToQuit.connect(guiExitHandler)
    
    view.setWindowTitle('Activations and Phases of Phase-State-Machine')
    view.resize(settings.value("size", QtCore.QSize(800, 300)))
    view.move(settings.value("pos", QtCore.QPoint(200, 200))) 
    
    monitor = PhastaStateEvolutionMonitorNode(view, pens_dict)

    MStateListener       = rospy.Subscriber("/phasta/phases_activations", phastapromep.msg.PhasesActivations, monitor.PhasesActivationsMsgCallback    , queue_size=20)
    StateConnectivityListener       = rospy.Subscriber("/phasta/stateconnectivity", phastapromep.msg.StateMatrix, monitor.connectivityCallback    , queue_size=1)
    
    rate = rospy.Rate(updaterate)
    view.show() #show window
    while not rospy.is_shutdown() and not aboutToQuit:
        rate.sleep()
        monitor.update()
        QtGui.QApplication.processEvents()

    settings.setValue("pos", view.pos())
    settings.setValue("size", view.size())

    
    
if __name__ == '__main__':
    main()
        



