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

# #Comment in for Debugging
# import ptvsd
# ptvsd.enable_attach(address = ('localhost', 5678))
# ptvsd.wait_for_attach()


try:
    import kivy
    import kivy.clock
except ImportError:
    print("Please install python-kivy !")
    raise SystemExit("Please install python-kivy !")
import labelapp 
import rospy
import os
import sys


class TeachingApp(kivy.app.App):

 def __init__(self):
    """ load the configuration, existing labeling data and start ros-interface: """
    kivy.app.App.__init__(self)
    rospy.init_node('teaching_app')
    os.chdir(os.environ["ROS_DATA"])
    if(len(sys.argv) == 2):
        self.sessionConfigYamlpath = sys.argv[1]
    else:
        self.sessionConfigYamlpath = 'session.yaml'

 def build(self):
     """ initiates interfacing between core elements of labelapp. needed by kivy. """
     self.playbackController = labelapp.LabelController(self.sessionConfigYamlpath)
     self.kivyinterface = labelapp.KivyRootWidget(self.sessionConfigYamlpath, self.playbackController)
     self.playbackController.setRootWidget(self.kivyinterface)
     #kick off event loop:
     kivy.clock.Clock.schedule_once(self.kivyinterface.periodic, self.playbackController.getUpdateTimePeriod())
     return self.kivyinterface


if __name__ == '__main__':
    """Starts the LabelApp"""
    print("starting GUI")
    app = TeachingApp()
    app.run()
