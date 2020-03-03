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

import phasestatemachine
import phasestatemachine.msg

import rospy
import pyqtgraph as _pg
import pyqtgraph.exporters
import yaml


from mstatemonitornode import themePyQTGraph


class PhastaStateMonitorNode(object):

    def __init__(self, graphicsView, pens_dict, showDuration=32.0):
        """
        """
        self.minRecordingTimestep = 0.05
        self.nStride = int(showDuration / self.minRecordingTimestep * 3)
        self.tracelengthCount = int(0.5 / self.minRecordingTimestep)
        self.startTime = None
        self.showDuration  = showDuration
        self.graphicsView = graphicsView
        self.pens_dict = pens_dict
        self.layout = None
        self.dofs = None
        self.dofsRequested = None

    def layoutGrid(self, dofs, stateConnectivity=None):
        #once dofs are known:
        self.dofs = dofs
        if stateConnectivity is None:
            self.stateConnectivity = 1-_np.eye(self.dofs)
        else:
            self.stateConnectivity = stateConnectivity
        self.states = _np.zeros((self.dofs, self.nStride))    
        self.times  = _np.zeros((self.nStride))
        self.capacity = self.nStride
        self.lastMsgIndex  = 0
        self.lastCutoffIndex = 1

        layout = _pg.GraphicsLayout()
        layout.layout.setSpacing(0.)                                                             
        layout.setContentsMargins(0., 0., 0., 0.)            
        self.subplots = []
        limits = [-0.05, 1.05]

        plotindices_list = [ [] for i in range(self.dofs) ]
        for j in range(self.dofs):
            for i in range(self.dofs):
                if self.stateConnectivity[j,i] > 0.5: 
                    plotindices_list[i].append(j)
        for i in range(self.dofs):
                    plotindices_list[i].reverse() #lifo to fifo

        rowscount = max([ len(l) for l in plotindices_list])
        self.plotIndicesSet = []
        predecessor_successor_curves  = []
        currentstate_curves  = []
        for rowidx in range(rowscount):
            curves_ofsuccessor = []
            curves_currentstate_row = []
            for colidx in range(self.dofs):
                i = colidx
                try:
                    j = plotindices_list[i].pop()
                except:
                    j = None
                    
                if j is not None:
                    p = layout.addPlot(title="{0}->{1}".format(i,j))
                    #p.setDownsampling(mode='mean')
                    #p.setClipToView(True)
                    p.setRange(xRange=limits, yRange=limits)
                    p.setLimits(xMin=limits[0], xMax=limits[1], yMin=limits[0], yMax=limits[1])
                    p.setLabel('bottom', "coordinate {0}".format(j))
                    p.setLabel('left', "coordinate {0}".format(i))
                    curve_pen = _pg.mkPen(self.pens_dict["trace full weight"])
                    c = [ p.plot(pen=self.pens_dict["trace light weight"]),
                          p.plot(pen=self.pens_dict["trace medium weight"]),
                          p.plot(pen=curve_pen),
                    ]
                    point_brush = _pg.mkBrush( self.pens_dict["state fillbrush"] )
                    points = p.plot(pen=None, symbol='o', symbolPen=None, symbolBrush=point_brush)
                    self.plotIndicesSet.append( (j,i, rowidx,colidx, p, c, points, point_brush, curve_pen) )
                else:
                    layout.nextColumn()
                    p = None
                    c= None
                    points=None
                self.subplots.append(p)
                curves_ofsuccessor.append(c)
                curves_currentstate_row.append(points)
            layout.nextRow()
            predecessor_successor_curves.append(curves_ofsuccessor)
            currentstate_curves.append(curves_currentstate_row)
        
        self.predecessor_successor_curves = predecessor_successor_curves
        self.currentstate_curves = currentstate_curves
        #update the layout in graphicsview:
        self.graphicsView.setCentralItem(layout)
        if self.layout is not None:
            del self.layout #make clear this is to be dumped
        self.layout = layout

    def connectivityCallback(self, data):
        if data.states != self.dofs and self.dofsRequested is None:  #dofs changed, request update of the view
            self.stateConnectivity = _np.array(data.matrix).reshape((data.states,data.states))
            self.dofsRequested = data.states

    def StateVectorMsgCallback(self, data):
    
        if self.dofs is None: #not initialized yet
            return
        
        if self.startTime is None:
            self.startTime = data.stamp 
        t = (data.stamp - self.startTime).to_sec()
        if (t-self.times[self.lastMsgIndex]) < self.minRecordingTimestep: #ignore updates coming faster than 10Hz
            return
            
            
        if self.lastMsgIndex + 1 >= self.capacity:
            n  = self.lastMsgIndex - self.lastCutoffIndex
            n1 = self.lastCutoffIndex
            n2 = self.lastMsgIndex
            self.states[:,:n] =  self.states[:,n1:n2]
            self.times[   :n] =  self.times[   n1:n2]
            self.lastMsgIndex = n-1
            self.lastCutoffIndex = 0

        idx = self.lastMsgIndex + 1
        self.times[idx] = t
        self.states[:,idx] = _np.abs(_np.asarray(data.x))

        self.lastMsgIndex = idx
        cutOffTime = self.times[idx] - self.showDuration
        while self.times[self.lastCutoffIndex] < cutOffTime and self.lastCutoffIndex < self.lastMsgIndex:
            self.lastCutoffIndex += 1


    def update(self):
        if self.dofsRequested is not None:
            self.layoutGrid(self.dofsRequested, self.stateConnectivity)
            self.dofsRequested = None
            
        if self.layout is None:  #nothing to do yet for drawing
            return
        
        n = self.lastMsgIndex
        n_cutoff  = self.lastCutoffIndex
        if n-n_cutoff <= 2*self.tracelengthCount +1: 
            return
        
        n_cutoff1 = n - 1*self.tracelengthCount
        n_cutoff2 = n - 2*self.tracelengthCount
        n_cutoff3 = n_cutoff+1

        
        t = self.times[n_cutoff:n] - self.times[n]
        for j, i, rowidx,colidx, p, curves, point, pointbrush, curvepen in self.plotIndicesSet:
            a =  _np.clip(  1 - (1 - self.states[i,n])**4 , 0.0, 1.0) #predecessor activation (approximation)
            
            QCol  = pointbrush.color()
            QCol.setAlphaF(a)
            pointbrush.setColor(QCol)

            #QCol  = curvepen.color()
            #QCol.setAlphaF(a)
            #curvepen.setColor(QCol)

            curves[0].setData(self.states[j,n_cutoff3:n_cutoff2], self.states[i,n_cutoff3:n_cutoff2], connect='finite')
            curves[1].setData(self.states[j,n_cutoff2-1:n], self.states[i,n_cutoff2-1:n]  )
            curves[2].setData(self.states[j,n_cutoff1-1:n], self.states[i,n_cutoff1-1:n]  )

            point.setData(self.states[j,n-1:n], self.states[i,n-1:n])
            


def main(updaterate=20):

    persistentconfigfile = os.path.join(os.environ["ROS_DATA"], ".phastamonitor.config.yaml")
    try:
        with open(persistentconfigfile, 'r') as f:
            persistentconfig = yaml.load(f)
            if persistentconfig is None:
                persistentconfig = {}
    except:
        persistentconfig = {}
        pass

    rospy.init_node('phastaStateMonitor')
    pens_dict = themePyQTGraph()

    _pg.setConfigOptions(antialias=True)
    aboutToQuit=False
    app = QtGui.QApplication([])
    app.setOrganizationName("MTI-engAge")
    app.setApplicationName("phase-state machine monitor")
    app.setWindowIcon(QtGui.QIcon(os.path.join(os.path.dirname(__file__),'../share/icon_phastamonitor.svg')))
    settings = QtCore.QSettings( "MTI-engAge", "phase-state machine monitor")
    view = _pg.GraphicsView()
    def guiExitHandler():
        global aboutToQuit
        aboutToQuit = True
        print("GUI closed")
    app.aboutToQuit.connect(guiExitHandler)
    
    view.setWindowTitle('States of Phase-State-Machine')
    view.resize(settings.value("size", QtCore.QSize(800, 300))) 
    view.move(settings.value("pos", QtCore.QPoint(200, 200))) 
    
    
    monitor = PhastaStateMonitorNode(view, pens_dict)


    MStateListener       = rospy.Subscriber("/phasta/x", phasestatemachine.msg.StateVector, monitor.StateVectorMsgCallback    , queue_size=20)
    StateConnectivityListener       = rospy.Subscriber("/phasta/stateconnectivity", phasestatemachine.msg.StateMatrix, monitor.connectivityCallback    , queue_size=1)
    
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
        



