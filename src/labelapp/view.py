#!/usr/bin/env python
# -*- coding: utf-8 -*-
#LabelApp2 KivyInterface
#Copyright Raphael Deimel, Jan Martin
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

try:
    from kivy.app import App
    from kivy.clock import Clock
    from kivy.uix.widget import Widget
    from kivy.uix.relativelayout import RelativeLayout
    from kivy.uix.boxlayout import BoxLayout
    from kivy.uix.popup import Popup
    from kivy.uix.label import Label
    from kivy.uix.button import Button
    from kivy.uix.togglebutton import ToggleButton
    from kivy.uix.textinput import TextInput
    from kivy.uix.scrollview import ScrollView
    from kivy.uix.scatter import Scatter
    from kivy.uix.gridlayout import GridLayout
    from kivy.properties import ListProperty
    from kivy.properties import StringProperty
    from kivy.core.image import Image as CoreImage
    from kivy.uix.image import Image
    from kivy.graphics import Color, Rectangle, Ellipse, Line
    from kivy.core.window import Window
    from kivy.uix.slider import Slider
    from kivy.uix.progressbar import ProgressBar
    from kivy.lang import Builder
    from kivy.uix.floatlayout import FloatLayout
    from kivy.uix.switch import Switch
    from kivy.uix.actionbar import ActionBar
    from kivy.uix.actionbar import ActionView
    from kivy.uix.actionbar import ActionButton
    from kivy.uix.actionbar import ActionDropDown
    from kivy.uix.actionbar import ActionPrevious      
    from kivy.uix.actionbar import ActionOverflow
    from kivy.uix.actionbar import ActionGroup
    from kivy.base import runTouchApp
    from kivy.uix.label import Label
    from kivy.config import Config


except ImportError:
    print("Please install python-kivy !")
    raise SystemExit("Please install python-kivy !")




import pandas as pd
import warnings
import subprocess
import signal
import time
import os
import fcntl
import sys
import h5py
import rospy
import numpy as np
from functools import partial
from enum import Enum
from StringIO import StringIO        
from shutil import copyfile


from labelapp.controller import PandaPublisherModeEnum, PandaRobotMode

try:
    from simple_wifileds import msg as led_color
except ImportError as e:
    print("if you plan to use external WifiLeds, please install the simple_wifileds package.")
    print("And also set wifi_leds arg True in launch File.")


try:
    from ruamel import yaml   #replaces deprecated pyYAML
except ImportError as e:
    if "promp" in e.message:
        print("\nCould not find ruamel. Please download/install the python-ruamel package!\n”")
    raise e

def yaml_constructor(loader, node):
    """ fixing bug with unicode patterns in yamlconfig File"""   
    return node.value
yaml.SafeLoader.add_constructor("tag:yaml.org,2002:python/unicode", yaml_constructor)


try: # get global path for kivy images. todo -> fixing this
    import rospkg
    rospack = rospkg.RosPack()
    global_path_to_kivy_images = "{}/share/labelapp_images/".format(rospack.get_path('phastapromep'))
except:
    print("Warning: No LabelApp-Images inside labelapp_images Folder in phastapromep.")

class KivyRootWidget(RelativeLayout):

    def __init__(self, sessionConfigYamlpath, playbackController, **kwargs):
        """ Function executes the complete setup for Kivy-Graphics of LabelApp """
        self.status_canvas_created = False # lock callback until canvas has been created
        super(KivyRootWidget, self).__init__(**kwargs)        

        #check if PandaLedBarometer is imported
        if PandaLedBarometer in sys.modules:
            self.LedBarometer = PandaLedBarometer()
        else: 
            self.LedBarometer = None    
        # -------------------------------------------- #
        # -------- Load Session Config Data ---------- #
        # -------------------------------------------- #
        self.sessionConfigYamlpath = sessionConfigYamlpath
        try:
            with open(self.sessionConfigYamlpath) as f:
                self.sessionConfig = yaml.safe_load(f)
        except IOError:
                print("LabelApp: Could not open session configuration from: {0}".format(sessionConfigYamlpath))
                raise SystemExit(1)
        self.sessionConfig['data directory'] = os.path.abspath(os.path.expanduser(self.sessionConfig['data directory']))
        Config.set('input', 'mouse', 'mouse,multitouch_on_demand') #remove red Dot's when doing right click

        self.setBackground("grey")


        self.playbackController = playbackController # store access to the Ros-Publisher for later
        self.labelstrings = self.playbackController.availableLabels
        self.highlightedTrajectoryButtonHandle = None
        self.debouncer = 0 #dummy for Slider Callback
        self.undoStack = []
        self.SimulatingPhastaPromp = False
        self.UpdateCursorEvent = None
        self.cutlist_enabled = False
        self.learnGraphPopup = None
        self.learnGraphPoller = None
        self.learnGraphPopupWidget = None
        self.learnGraphSubprocess = None

        self.init_kivy_elements()
        self.InterfaceUpdateIntervall = 0.1 #10Hz
        self.initkeyboard()
        Clock.schedule_once(self.resize_scrollview, 1)  # wait for a long time to have everything rendered before TODO
        Clock.schedule_once(self.resize_scrollview, 4)  #  
        Clock.schedule_once(self.resize_scrollview, 10) #  
        Clock.schedule_once(self.resize_scrollview, 30) #  

        self.DisplayUpdateTimer = Clock.schedule_interval(self.updateDisplayedValues, self.InterfaceUpdateIntervall)#10Hz


    def init_kivy_elements(self):
        """ Initializes and orders all Kivy-Interface-Elements that are visible to the user."""   

        # -------------------------------------------- #
        # -------- Setup the KivyGui-Layout ---------- #
        # -------------------------------------------- #
       

        Interface2= RelativeLayout(size_hint=(0.8,1),pos_hint={'x': 0.1, 'y': 0.04})
        self.add_widget(Interface2)

        self.Tooltip_Config_List = TooltipConfigListCreatorTool().create_TooltipConfigList()

        layout_helper = RelativeLayout(pos_hint={'x': 0.35, 'y': 0.5},size_hint=(0.4,0.2))
        ProgressplayerLayout = BoxLayout(orientation='vertical',spacing=10)
        self.TrajectoryProgressBar = CustomSliderProgressbar(size_hint=(0.8,1),pos_hint={'x': 0.1, 'y': 0.0})
        ProgressplayerLayout.add_widget(self.TrajectoryProgressBar)
        ProgressplayerLayout.add_widget(Label(text='',size_hint_y=0.2))     #cheap vertical space
        actionbar_ = LabelAppRvizPlayer(parent=self,TooltipConfig_List = self.Tooltip_Config_List,background_color=KivyColors.blue_dark,pos_hint_x=-0.1)
        if actionbar_ != None: #prevents from not successfully loaded Buttons
            ProgressplayerLayout.add_widget(actionbar_)            
        layout_helper.add_widget(ProgressplayerLayout)  
        self.add_widget(layout_helper)


        Labelmenu = RelativeLayout(orientation='horizontal',size_hint=(0.5,0.6),pos_hint={'x': 0.2, 'y': 0.04})
        Interface2.add_widget(Labelmenu)
        Vieweroptions,Vieweroptions2,self.simulationbtn = ViewerOptionsCreator().create_widgets(parent=self,TooltipConfig_List=self.Tooltip_Config_List)
        Interface2.add_widget(Vieweroptions)
        Interface2.add_widget(Vieweroptions2)

        statuslayout = BoxLayout(orientation='horizontal',size_hint=(0.75,0.3),pos_hint={'x': 0.05, 'y': 0.75})
        Interface2.add_widget(statuslayout)


        # -------------------------------------------- #
        # -------- Slider to cut Trajectories ------------  #
        # -------------------------------------------- #


        self.innerLabels = BoxLayout(orientation='vertical',size_hint=(0.5,0.6),pos_hint={'x': 0.4, 'y': 0})
        Labelmenu.add_widget(self.innerLabels)

        btnsize=(1,0.5)
        btnpos={'x': 0, 'y': 0.5}

        self.innerLabels.add_widget(Label(text='',size_hint_y=0.1)) #cheap vertical space

        self.StartCursorpos = None 
        self.EndCursorpos = None # Last CursorSample


        # -------------------------------------------- #
        # -------- create labelname buttons ----------  #
        # -------------------------------------------- #


        btnsize=(0.3,0.3)

        btnsize=(1,1)

        tooltiptextinput = TooltipTextInput(
                                            text="Enter New Label",
                                            pos_hint={'x': 0, 'y': 0}, size_hint = (1,1),
                                            tooltip_config = self.Tooltip_Config_List['EnterNewLabel']
        )
        tooltiptextinput.bind(on_text_validate=self.on_enter)
        self.innerLabels.add_widget(tooltiptextinput)


        btnsize=(1,1)         
        discardedButton = TooltipButton(
                    text = "discarded",
                    pos_hint=btnpos, size_hint = btnsize,
                    tooltip_config = self.Tooltip_Config_List['discarded']
        )


        discardedButton.bind(on_press=partial(self.setLabel, 'discarded'))
        self.innerLabels.add_widget(discardedButton)


        self.LabelButtons = {}
         
        for i, label in enumerate(self.playbackController.availableLabels):
            btn = TooltipLabelButton(
                    parent = self,
                    text = label,
                    size_hint = (1,1),
                    pos_hint={'x': 0, 'y': 0},
                    tooltip_config = TooltipConfig(text="Label Trajectory as: {0}".format(label)),
                    playbackController=self.playbackController
            )

            callbackfunc = partial(self.setLabel, label)
            btn.bind(on_press=callbackfunc)
            self.innerLabels.add_widget(btn)
            self.LabelButtons[label] = btn

        btn = TooltipButton(
                    text = "Undo",
                    pos_hint={'x': 0.9, 'y': 0.0}, size_hint = (0.3,0.1),
                    tooltip_config = self.Tooltip_Config_List['Undo']
        )
        
        btn.bind(on_press=self.undo_label)
        Labelmenu.add_widget(btn)



        self.currentTrajectoryNumberWidget = Label(text='1', pos_hint={'x': 0.14, 'y': -0.2})
        currentTrajectoryNumberWidget_container = TooltipContainer(
                                tooltip_config = self.Tooltip_Config_List['ActiveTrajectory'] ,
                                size_hint=(None,None),pos_hint={'x': 0, 'y': 0})
        currentTrajectoryNumberWidget_container.add_widget(self.currentTrajectoryNumberWidget)
        self.currentTrajectoryNumberWidget.font_size=50
        statuslayout.add_widget(currentTrajectoryNumberWidget_container)
    
        self.container =  TooltipBox(
                                        tooltip_config = self.Tooltip_Config_List['StatusField'] ,
                                        size_hint=(1,0.7),pos_hint={'x': -0.2, 'y': -0.2}
        )
        statuslayout.add_widget(self.container)

        self.status_canvas_created = True # canvas has been created and can be accessed in callback

        self.statusfield = Label(text = 'statusfield',font_size=18)
        self.container.layout_helper.add_widget(self.statusfield)

        self.samplecounter = Label(text = ' (record a trajectory)',font_size=18)
        self.container.layout_helper.add_widget(self.samplecounter)

        self.goalstatus = Label(text = '',font_size=18)
        self.container.layout_helper.add_widget(self.goalstatus)

        # ========================= MTI-Logo ================================================#

        try:
            logolayout = RelativeLayout(size_hint=(0.07,0.4),pos_hint={'x': 0.83, 'y': 0.65})
            logolayout.add_widget(Image(source=global_path_to_kivy_images+ 'mti_engage_vertical.png',size=logolayout.size_hint,pos=logolayout.pos))
            self.add_widget(logolayout)
        except:    
            print('Logo not found.')    
            
        # ========================= scrollview of LabelList ================================================#

        self.cutlistbtn = TooltipButton( # might be needed for resize_scrollview()
            size_hint_y = 0.08,
            background_color = KivyColors.grey_dark,
            color=KivyColors.green_dark,
            text =  'Hide / Show Cropping',
            tooltip_config = TooltipConfig(text="Press this button to hide / update all cuts from Trajectory")
        )
        self.cutlistbtn.bind(on_press=self.toggleCutlistInScrollview)


        self.TrajectoryButtonsLayout = GridLayout(cols=1, padding=0.01, spacing=0, size_hint=(1, None))
        self.TrajectoryButtonsLayout.bind(minimum_height=self.TrajectoryButtonsLayout.setter('height'))
             
        #store all trajectory buttons in one array for global access
        self.trajectoryButtons = []
        #initial populate:
        for i in range(self.playbackController.getObservationsCount()):
            row = self.playbackController.metadata.iloc[i]
            self.registerTrajectory(i,row['label'],row['playback_stiffness'])

        #make scrollview containing gridlayout
        scrollview = ScrollView(do_scroll_x=False, bar_margin=10, bar_pos_y='left', size_hint=(1, 0.9))
        scrollview.add_widget(self.TrajectoryButtonsLayout)

        #make Boxlayout containing scroll view
        vbox = BoxLayout(orientation='vertical')
        vbox.add_widget(self.cutlistbtn)
        vbox.add_widget(Label(text="Recorded Observations:",size_hint_y=0.08))
        vbox.add_widget(scrollview)

        #make RelativeLayout containing BoxLayout
        Interface1 = RelativeLayout(orientation='vertical',size_hint=(0.25,0.9),pos_hint={'x': 0.02, 'y': 0.05})
        Interface1.add_widget(vbox)
        self.add_widget(Interface1)
        Window.bind(on_resize=self.labelapp_resize)


    def periodic(self, dt):
        """ function for retriggering everything that is updated regularly
        --> priodic Call for central_state_handler()
        """
        self.playbackController.periodic()
        Clock.schedule_once(self.periodic, self.playbackController.getUpdateTimePeriod())


    def addTrajectoryButton(self, label):
        """
        create and append a new trajectory button to the scroll view
        Is called by the controller if observations are loaded or new ones stored 
        """
        

        # ========================= End scrollview of LabelList ================================================#

    def labelapp_resize(self,window=None, width=0, height=0):
        self.resize_scrollview()
        self.resize_SliderProgressbar()

    def resize_SliderProgressbar(self,dt=0):
        self.TrajectoryProgressBar.render_widget()    

    def resize_scrollview(self,dt=0):
        """ Function that scales Scollview in Kivy Interface according to the window-size (bug fix)."""
        if self.cutlist_enabled:
            self.toggleCutlistInScrollview()
            self.toggleCutlistInScrollview()
        else:    
            for i,btn in enumerate(self.trajectoryButtons):
                self.updateTrajectoryButtonColorAndText(i)
       
 
    def initkeyboard(self):
        """ Function holds commandmapping, default config of they kivy app and inits the Bluetooth Keyboard""" 
        self.keyboard_lock = False
        self.keyboard_Sign = None
        self._keyboard = Window.request_keyboard(self._keyboard_closed, self)
        self._keyboard.bind(on_key_down=self._on_keyboard_down)
        self._keyboard.bind(on_key_up=self._on_keyboard_up) # not available with doodle presenter
        


        # Command-Mapping for (Bluetooth) Keyboard
        self.active_key = ""
        default_keyboard_mapping = { 
                'numpad0':          "publish",
                'insert':           "publish", #same key but with numlock

                'numpad2':          "next Trajectory",
                'down':             "next Trajectory",

                'numpad8':          "previous Trajectory", 
                'up':               "previous Trajectory", 
                
                'numpad6':          "increase Certainty",
                'right':            "increase Certainty",

                'numpad4':          "reduce Certainty",
                'left':             "reduce Certainty",


                'backspace':        "Play",    
                'numpadsubstract':  "PlayBackwards",
                'numpadadd':        "jumpBack",
                'numpadenter':      "record",
                'numpad5':          "Play and record",


                'numpaddot':        "switch gripper",
                'delete':           "switch gripper",


            }
        if "keyboard mapping" in self.sessionConfig:
            self.keyboard_mapping =  self.sessionConfig["keyboard mapping"]
            if cmp(self.keyboard_mapping,default_keyboard_mapping) != 0:
                print("Warning: keyboard_mapping in Session.yaml overwrites default keyboard_mapping.")
            else:
                print("Keyboard-Mapping in Session.yaml is default.")    
        else:
            self.keyboard_mapping = default_keyboard_mapping 
            print("No Keyboardmapping in session-File. Using default.")

        self.command_mapping = {
        
            "next Trajectory":     self.nxtcallback,
            "previous Trajectory": self.prevcallback,
    
            "increase Stiffness":  self.playbackController.increaseStiffnessOfCurrentActiveTrajectory,
            "reduce Stiffness":    self.playbackController.decreaseStiffnessOfCurrentActiveTrajectory,

            "Play" :               self.playbackController.toggleReplaying , #request Replay of Trajectory
            "Play and record" :    self.playbackController.toggleRecordReplay, #request Replay of Trajectory

            "PlayBackwards":       self.togglePlayBackwards,
            "jumpBack":            self.jumpBack,

            "record":              self.playbackController.toggleRecordingPanda,

            "switch gripper":      self.setGripper,

            "run behavior":        self.runBehaviorSimulation,

        }
        #arguments for 1st command
        self.cmd1_args1 = {
            "increase Certainty":  self.playbackController.getCurrentlyActiveTrajectoryNumber,
            "reduce Certainty":    self.playbackController.getCurrentlyActiveTrajectoryNumber,
        }

        # mapping if 2nd command needed
        self.command_mapping2 = {
            "increase Certainty":  self.updateTrajectoryButtonColorAndText,
            "reduce Certainty":    self.updateTrajectoryButtonColorAndText,
        }
        #arguments for second command
        self.cmd2_args1 = {
            "increase Certainty":  self.playbackController.getCurrentlyActiveTrajectoryNumber,
            "reduce Certainty":    self.playbackController.getCurrentlyActiveTrajectoryNumber,
        }

 
    def on_enter(self,instance):
        """ Callback: Textinput in KivyInterface for naming labels at runtime """
        label = str(instance.text)
        if label in self.playbackController.availableLabels: #don't add another label buttin if label is already there
            return
        btn = TooltipLabelButton(
                parent = self,
                text = label,
                size_hint = (1,1),
                pos_hint={'x': 0, 'y': 0},
                tooltip_config = TooltipConfig(text="Label Trajectory as: {0}".format(label)),
                playbackController=self.playbackController
        )
        btn.bind(on_press=partial(self.setLabel,label))
        self.innerLabels.add_widget(btn)
        self.playbackController.addToAvailableLabels(label)
   
    def _keyboard_closed(self):
        """ Stop Keyboard Watchdog when Kivy is closed """
        self._keyboard.unbind(on_key_down=self._on_keyboard_down)
        self._keyboard = None

    def _on_keyboard_up(self, keyboard, keycode):
        """ LabelApp-Action when keyboard Button is released"""  
        if(keycode[1] not in self.keyboard_mapping):
            return  # Error Catching

    def _on_keyboard_down(self, keyboard, keycode, text, modifiers):
        """ LabelApp-Action when keyboard Button is pressed """
        self.activate_keyboardSign(keycode[1])
        Clock.schedule_once(self.deactivate_keyboardSign, 0.5) #2Hz #remove sign from screen after 500ms

        if self.keyboard_lock == False:

            key = str(keycode[1])
            if(key == "numlock" or key not in self.keyboard_mapping):
                return  

            self.active_key = self.keyboard_mapping[key]

            if self.active_key != "increase Certainty"  and self.active_key != "reduce Certainty" and self.active_key != "next Trajectory" and self.active_key != "previous Trajectory":
                self.keyboard_lock = True #lock
                Clock.schedule_once(self.unlock_keyboard, 0.5)#2Hz #remove lock after 500ms       

            
            if self.active_key in self.command_mapping:
                if self.active_key in self.cmd1_args1:
                    self.command_mapping[self.active_key](self.cmd1_args1[self.active_key]())
                else:    
                    self.command_mapping[self.active_key]()
                
            if self.active_key in self.command_mapping2:
                if self.active_key in self.cmd2_args1:
                    self.command_mapping2[self.active_key](self.cmd2_args1[self.active_key]())
                else:
                    self.command_mapping2[self.active_key]()
        else:
            print("Debounced a key")

    def unlock_keyboard(self,dt):
        """ Debouncing keyboar callbacks """
        self.keyboard_lock = False
        
    # -------------------------------------------- #
    # -------- LabelApp Callback Funcs ----------  #
    # -------------------------------------------- #


    def quitApplicationCallback(self, instance):
        """ Kivy Quit-Button Callback """
        raise SystemExit("Label App was terminated via user input.")

    def setTrajectory(self,instance):
        """Switch between Trajectories of current Label"""
        lastTrajectoryNumber = self.playbackController.currentlyActiveTrajectoryNumber
        nextTrajectoryNumber = int(instance.id)
        self.playbackController.loadActiveTrajectory(nextTrajectoryNumber)
        self.updateColorOfButton(instance)

    def setLabel(self,label, buttonInstance):
        """ assign a  Label to a current trajectory """
        self.undoStack.append((self.playbackController.currentlyActiveTrajectoryNumber,self.playbackController.activeLabel))
        self.playbackController.setLabelOfCurrentlyActiveTrajectory(label) #update LabelTag for specific trajectory
        self.playbackController.activeLabel = label
        self.updateTrajectoryButtonColorAndText(self.playbackController.currentlyActiveTrajectoryNumber,label)


    def undo_label(self, instance = 0):
        """
        Undo the last trajectory assignment to a label
        """
        if(len(self.undoStack)==0):
            print("nothing to Undo.")
            return False
        TrajNum,label = self.undoStack.pop()
        self.playbackController.setLabelOfTrajectory(label,TrajNum) #update LabelTag for specific trajectory
        self.playbackController.activeLabel = label
        self.updateTrajectoryButtonColorAndText(TrajNum,label)
        print("Undo Trajectory {0} to Label: {1}".format(TrajNum,label))


    def toggleReplaying(self,instance = 0):
        """
        Replay the Trajectory. This is a Umleitung.
        """
        self.playbackController.toggleReplaying(direction=1)
        if self.playbackController.PublisherMode == PandaPublisherModeEnum.replaying_trajectory:
            self.togglereplaybtn.change_image(image_source = global_path_to_kivy_images +"pausebtn.png")
            self.togglereplaybtn.set_tooltip_config(self.Tooltip_Config_List['Pause'])
        else:
            self.togglereplaybtn.change_image(image_source = global_path_to_kivy_images + "playbtn.png")
            self.togglereplaybtn.set_tooltip_config(self.Tooltip_Config_List['Play'])


    def pauseTrajectory(self,instance = 0):
        """ Commands PlaybackController to Pause the Trajectory """
        self.playbackController.pauseTrajectory() 

    def toggleReplayRecord(self,instance=0):
        """
        Replay the Trajectory. And Start recording.
        """
        self.playbackController.toggleRecordReplay()
        return

    
    def toggleRecording(self,instance=0,state=0):
        """ Callback from Actionbutton to toggle recording Panda """
        self.playbackController.toggleRecordingPanda()



    def togglePlayBackwards(self,instance=0,state=0):
        """
        Move Panda back step by step
        """
        self.playbackController.toggleReplaying(direction=-1)

    def jumpBack(self,instance=0,state=0):
        """
        Jump back to beginning of Trajectory
        """
        self.playbackController.jumpBack()


    def jumpBackAndRepeat(self,instance=0,state=0):
        """
        Jump back to beginning of Trajectory adn replay
        """
        self.playbackController.jumpBack()
        self.playbackController.startPlaying()

    def nxtcallback(self,instance = 0):
        """ Switch to next Trajectory """
        if(self.playbackController.currentlyActiveTrajectoryNumber < self.playbackController.getObservationsCount()-1):
            self.playbackController.loadActiveTrajectory(self.playbackController.currentlyActiveTrajectoryNumber+1)

    def prevcallback(self,instance = 0):
        """ Switch to prev Trajectory """
        if(self.playbackController.currentlyActiveTrajectoryNumber > 0):
            self.playbackController.loadActiveTrajectory(self.playbackController.currentlyActiveTrajectoryNumber-1)



    def setGripper(self,instance=0,state=0):
        """ Tries to command last Position to Panda (hold position), but with  
            Gripperposition toggled.
        """ 
        self.playbackController.setGripper(mode="toggle")

        
        
    def activate_keyboardSign(self,keySign):
        """ display actual keyboard  Sign in the Corner of the GUI """
        if not isinstance(self.keyboard_Sign,BoxLayout):
            self.keyboard_Sign = BoxLayout(orientation='horizontal',size_hint=(0.2,0.2),pos_hint={'x': 0.87, 'y': 0.88})
            info = Label(text = keySign ,font_size=14, color=KivyColors.yellow)
            self.keyboard_Sign.add_widget(info)
            self.add_widget(self.keyboard_Sign)            
    
    def deactivate_keyboardSign(self,instance=0):
        """ remove actual keyboard from Corner of the GUI """
        if isinstance(self.keyboard_Sign,BoxLayout):
            self.remove_widget(self.keyboard_Sign)
            self.keyboard_Sign = None
       


    # we might take less here?
    def generateBehavior(self, instance=0):
        """ Start generating promps and illustration data from labeled Trajectories """
        self.learnGraphPopupWidget = BoxLayout(orientation='vertical')
        helper = RelativeLayout(orientation='vertical',size_hint=(1,0.5),pos_hint={'x': 0.02, 'y': 0.05})
        
        self.stopprogressCounter = False
        self.progresscircle = ProgressCounter(pos_hint={'x': 0.85, 'y': 0.4},max=100)
        helper.add_widget(self.progresscircle)
        helper.add_widget(Label(text="Generating behavior graph (state graph, ProMPs and state controllers)\nfrom the labeled segments..\n\n(This may take some time)", size_hint_y=0.5))
        self.learnGraphPopupWidget.add_widget(helper)

    
       
        self.learnGraphPopupWidgetProgressText = TextInput(text="")
        self.learnGraphPopupWidget.add_widget(self.learnGraphPopupWidgetProgressText)
            


        self.learnGraphPopup = Popup(title='Generating behavior graph', content=self.learnGraphPopupWidget, auto_dismiss=False)
        self.learnGraphPopup.open()

        # Prevents Access on Datahandler and stop all actions - undo this after Thread finished (Dissmissbtn)
        self.playbackController.PublisherMode = PandaPublisherModeEnum.wait_for_behavior 
 
        self.learnGraphSubprocess = subprocess.Popen(["rosrun", "phastapromep", "learnGraph2.py","session.yaml"], stdout=subprocess.PIPE, universal_newlines=True)
        fl = fcntl.fcntl(self.learnGraphSubprocess.stdout, fcntl.F_GETFL)
        fcntl.fcntl(self.learnGraphSubprocess.stdout, fcntl.F_SETFL, fl | os.O_NONBLOCK)

        self.DissmissButton = Button(text="Cancel",size_hint_y=0.15, on_press=self.learnGraphPopupDismissCallback)
        self.learnGraphPopupWidget.add_widget(self.DissmissButton)
        #redirect all output to kivy Window
        fl = fcntl.fcntl(self.learnGraphSubprocess.stdout, fcntl.F_GETFL)
        fcntl.fcntl(self.learnGraphSubprocess.stdout, fcntl.F_SETFL, fl | os.O_NONBLOCK)
        self.DisplayUpdateTimer.cancel()
        self.learnGraphPoller = Clock.schedule_interval(self.polllearnGraphFinished, 0.1)


    def polllearnGraphFinished(self, instance=0):
        """ Update Popup for behavior generation """
        try:
            if self.stopprogressCounter != True:
                self.progresscircle.set_value(self.progresscircle.counter + 1)
            for line in self.learnGraphSubprocess.stdout:
                self.learnGraphPopupWidgetProgressText.insert_text(line)
        except IOError:
            rospy.logerr("polllearnGraphFinished stopped")
            pass
        if self.learnGraphSubprocess.poll() is not None:
            self.stopprogressCounter = True            
            self.DissmissButton.text = "Ok"
            self.DissmissButton.background_color = KivyColors.green
                



    def learnGraphPopupDismissCallback(self, instance=0):
        """ Dismiss learn Behavior Popup """
        if self.learnGraphPopup != None:
            self.learnGraphPopup.dismiss()
        if self.learnGraphPoller != None:
            self.learnGraphPoller.cancel()
        if self.learnGraphPopupWidget != None:
            self.remove_widget(self.learnGraphPopupWidget)
        if self.learnGraphSubprocess != None:
            try: 
                self.learnGraphSubprocess.kill()
            except:
                print("Learn Behavior Subprocess already ended.")
        self.playbackController.PublisherMode = PandaPublisherModeEnum.idle
        self.DisplayUpdateTimer = Clock.schedule_interval(self.updateDisplayedValues, self.InterfaceUpdateIntervall)#10Hz
        print("Learn Behavior ended.")

    def runBehaviorSimulation(self,instance=0):
        """ Toggles the parallel Simulation of the phastaprom phase state machnine"""
        if self.SimulatingPhastaPromp == False :
            self.SimulatingPhastaPromp = True
           
            self.startmstatemonitornode = subprocess.Popen(["rosrun", "phastapromep", "mstatemonitornode.py"], stdout=subprocess.PIPE,preexec_fn=os.setsid, universal_newlines=True)       
            self.startphastamonitor = subprocess.Popen(["rosrun", "phastapromep", "phastamonitor.py"], stdout=subprocess.PIPE,preexec_fn=os.setsid, universal_newlines=True)       
            self.startactivationsmonitor = subprocess.Popen(["rosrun", "phastapromep", "phasesactivationsmonitor.py"], stdout=subprocess.PIPE,preexec_fn=os.setsid, universal_newlines=True)       
            self.SimulationSubprocess = subprocess.Popen(["roslaunch", "phastapromep", "testBehavior.launch"], stdout=subprocess.PIPE,preexec_fn=os.setsid, universal_newlines=True)
            self.simulationbtn.background_color = KivyColors.red

        else:
            self.SimulatingPhastaPromp = False
            try: 
                os.kill(self.startmstatemonitornode.pid, signal.SIGTERM)
                os.kill(self.startphastamonitor.pid, signal.SIGTERM)  
                os.kill(self.startactivationsmonitor.pid, signal.SIGTERM)  
                os.kill(self.SimulationSubprocess.pid, signal.SIGTERM)  
  
            except:
                print("Run Behavior Subprocess already ended.")
    
            self.simulationbtn.background_color = KivyColors.white  


    def updateDisplayedValues(self,dt=0):
        """ method to let other objects request an update of displayed values """
        if self.playbackController.PublisherMode == PandaPublisherModeEnum.wait_for_behavior:
            return False #nothing to do anyway
        currentlyActiveTrajectoryNumber = self.playbackController.getCurrentlyActiveTrajectoryNumber()
        if currentlyActiveTrajectoryNumber is None:        
            row = None
        else:
            row = self.playbackController.metadata.iloc[currentlyActiveTrajectoryNumber]
        mode = self.playbackController.PublisherMode
        currentSample = self.playbackController.currentlyPublishingSampleIndex
        
        colorSwitch = PandaPublisherColorSwitch()
     
        color = colorSwitch.getKivyColor(str(mode))
        if self.status_canvas_created == True:
            self.container.layout_helper.canvas.before.clear()
            with self.container.layout_helper.canvas.before:

                Color(color[0],color[1],color[2],color[3]) # green; colors range from 0-1 instead of 0-255
                Rectangle(size=self.container.layout_helper.size,
                                    pos=self.container.layout_helper.pos)
   
        if currentlyActiveTrajectoryNumber is None:
            self.currentTrajectoryNumberWidget.text = "-"
        else:
            self.currentTrajectoryNumberWidget.text = str(currentlyActiveTrajectoryNumber)
        self.statusfield.text = str(mode.name)
        if row is None:
            self.samplecounter.text  = ""
            self.goalstatus.text = ""
        else:
            self.samplecounter.text  = "{0}| {1} | {2}".format(row['inpoint'],currentSample,row['outpoint'])
            self.goalstatus.text = "Playback Stiffness: {0}".format(row['playback_stiffness'])
        if row is None or currentSample is None:
            self.TrajectoryProgressBar.update_value(value=0.0)
        else:
            self.TrajectoryProgressBar.update_value(value=float(currentSample)/row['samples'])
        
        if self.LedBarometer != None:   
            wifiled = led_color.VisualOutput()
            wifiled.color = colorSwitch.getString(str(mode))
            wifiled.duration = 0.0
            self.LedBarometer.pub.publish(wifiled)
        




    def toggleCutlistInScrollview(self,dt=0,instance=0):
        """ Enables rendering of Cutlist at the Top of every Button inside teh kivy Scrollview """
        if self.cutlist_enabled == False:
            self.cutlist_enabled = True
            for i in range(len(self.trajectoryButtons)):
                self.updateCutlistinScrollview(i)
        else:
            self.cutlist_enabled = False
            for TrajectoryNumber in range(len(self.trajectoryButtons)):
                self.trajectoryButtons[TrajectoryNumber].hide_status()        
                self.trajectoryButtons[TrajectoryNumber].width = self.cutlistbtn.width

    def setCursorStart(self,start=0):
        """ param arg2:  passes relative cursor position to Controller""" 
        if type(self.UpdateCursorEvent) != None:
            Clock.unschedule(self.UpdateCursorEvent)     #delete previous Event         
        self.UpdateCursorEvent = Clock.schedule_once(partial(self.extern_updateTrajectorySlider,SliderStart=start), 0.05) #2Hz
        self.debouncer = self.debouncer + 1        

    def setCursorEnd(self,end=1):
        """ param arg2: passes relative cursor position to Controller """
        if type(self.UpdateCursorEvent) != None:
            Clock.unschedule(self.UpdateCursorEvent)     #delete previous Event  
        self.UpdateCursorEvent = Clock.schedule_once(partial(self.extern_updateTrajectorySlider,SliderEnd=end), 0.05) #2Hz
        self.debouncer = self.debouncer + 1   #command_queue     

    def extern_updateTrajectorySlider(self,dt=0,value_=None,SliderStart=None,SliderEnd=None):
        """Private Function is called to update PublisherSampleCounter and Slider-START from Kivy Interface
        @param dt --> needed by timer for debouncing kivycallbacks 
        """ 
        value = 0
        if SliderStart != None:
            self.StartCursorpos = SliderStart 
            self.playbackController.setInPointofCurrentTrajectory(self.StartCursorpos)
            value=SliderStart
        if SliderEnd != None:
            self.EndCursorpos = SliderEnd
            self.playbackController.setOutPointofCurrentTrajectory(self.EndCursorpos)
            value=SliderEnd
        if value_ != None:
            value = value_    

        i = self.playbackController.currentlyActiveTrajectoryNumber
        self.updateCutlistinScrollview(i,self.StartCursorpos,self.EndCursorpos)
        self.TrajectoryProgressBar.update_value(value=value, Slider1=self.StartCursorpos,Slider2=self.EndCursorpos)      

#        if self.debouncer < 2:  # less than two callbacks in command_queue
#            self.playbackController.updateRobotStateOnce()
        self.debouncer = 0    #count again    


    def updateCutlistinScrollview(self,TrajectoryNumber,rel_start=None,rel_end=None):
        """ Updates cutarea inside the Trajectory Button widget and sets the slider's accordingly"""      
        success = False
        if len(self.trajectoryButtons) <= TrajectoryNumber or TrajectoryNumber < 0:
            return False
        if rel_start == None or rel_end == None:
            rel_start,rel_end = self.playbackController.getRelativeSliderTimestamps(TrajectoryNumber)

        if self.cutlist_enabled == True:    
            if rel_start >= 0 and rel_start <= 1 and rel_end >= 0 and rel_end <=1:
                # if we have valid rel_start and stop stamp, set Status
                btn = self.trajectoryButtons[TrajectoryNumber]
                btn.updateSliderStatus(rel_start,rel_end)
                success = True
            else:
                print("Sliders not valid. rel_start: {0} rel_end: {1}".format(rel_start,rel_end))
                rel_start=0
                rel_end=1
                btn = self.trajectoryButtons[TrajectoryNumber]
                btn.updateSliderStatus(rel_start,rel_end)    
                success = True

        self.trajectoryButtons[TrajectoryNumber].width = self.cutlistbtn.width
        return success


    def updateTrajectoryButtonColorAndText(self, i, btntext = None):
        if i >= self.playbackController.getObservationsCount():
            return
        if btntext is None:
            btntext = self.playbackController.metadata['label'].iloc[i]
        stiffness = self.playbackController.metadata['playback_stiffness'].iloc[i]

        #update scrollview button:
        btn = self.trajectoryButtons[i]
        btn.text = ' ' + str(i).zfill(3) + ': ' + str(round(stiffness,1)) + ' | '+ str(btntext) 
        btn.height = 24
        btn.width = self.cutlistbtn.width 
        btn.text_size = (btn.width, btn.height)
        self.updateColorOfButton(btn)
        self.updateCutlistinScrollview(i)

    def registerTrajectory(self,i,label="",certainty = 0):
        """
        Call this to tell the GUI about the existence of a new observation / trajectory, or when the label of a trajectory is changed
        
        New observations can only be added consecutively, i.e. at i==len(self.trajectoryButtons)
        """
        if i > len(self.trajectoryButtons):
            raise ValueError("Can only create new button at index {0}, but {1} was requested.".format(i,len(self.trajectoryButtons)))

        if i < len(self.trajectoryButtons): #update an existing button
            btn = self.trajectoryButtons[i]
            self.updateColorOfButton(btn)
            return

        # else, create a new button:
        btn = TooltipButton( 
            text =  '', 
            tooltip_config = TooltipConfig(text="Recorded Trajectory with Number: {0}".format(str(i).zfill(3))),
            halign='left',
            text_size=(300, None),
        )
        btn.id = str(i)
        #force btn size at startup
        btn.size_hint=(None, None)
        btn.bind(on_press=self.setTrajectory)
        self.updateColorOfButton(btn)     
        self.TrajectoryButtonsLayout.add_widget(btn)
        self.trajectoryButtons.append(btn) # grants global access for other controlls
        self.updateTrajectoryButtonColorAndText(i, btntext=label)
        self.resize_scrollview() #trigger a recalculation of the scrollview

    def updateColorOfButton(self,btn):
        if btn.text != None:
            if btn.text == "" or 'unknown' in btn.text:
                btn.background_normal = 'atlas://data/images/defaulttheme/button_pressed' 
                btn.background_color = KivyColors.grey
                btn.color = KivyColors.white
            else:
                btn.background_normal = 'atlas://data/images/defaulttheme/button' 
                btn.background_color = KivyColors.grey_dark
                btn.color = KivyColors.grey

            if 'discarded' in btn.text:
                btn.background_color = KivyColors.black
                btn.color = KivyColors.grey_dark
            if int(btn.id) == self.playbackController.currentlyActiveTrajectoryNumber:
                btn.background_color = KivyColors.grey
                btn.color = KivyColors.white
                #update button that got deselected:
                if not self.highlightedTrajectoryButtonHandle is None:
                    if self.playbackController.currentlyActiveTrajectoryNumber != int(self.highlightedTrajectoryButtonHandle.id):
                        self.updateColorOfButton(self.highlightedTrajectoryButtonHandle)
                self.highlightedTrajectoryButtonHandle = btn


    def create_available_default_backgrounds(self,backgroundmap=None):
        """ creates a dictionary containing default background rgb-values """
        if backgroundmap == None: #default
            self.backgrounds = {"grey": [float(152)/255,float(138)/255,float(141)/255,float(30)/255] ,"none": (0.,0.,0.0,1),} 
        else:
            self.available_background_path = backgroundmap

    def switchBackgroundcallback(self,instance=0,state=0):
        """ Callback for changing Bakgroundtbn """
        if type(instance) == TooltipActionButton:
            self.setBackground(name=instance.text)
        else:
            pass
            #print("Warning - Btn instance is no TooltipActionButton")

    def setBackground(self,name=None,path=None):
        """ Function for changing the kivy-background.
            @param: path 
            If a path is set, a backgroudn app tries to load an image from taht path        
        
            @param: name 
            If is set app tries to select that color from the defaults     
        
        
        """

        #Set the background via Image-Path OR choose a deafult-color
        if path != None:
            if os.path.isfile(path): 
                Window.clearcolor = (0,0,0,1)
                with self.canvas.before: 
                    Rectangle(source=path,pos_hint=self.pos,size=(1920,1020)) 
                    return True


        if name != None:
            if not "available_background_path" in self.__dict__ : 
                self.create_available_default_backgrounds() #defaults not created yet -> let's create them

            if name in self.backgrounds:
                Window.clearcolor = (0,0,0,1)
                with self.canvas.before:
                    Color(*self.backgrounds[name])
                    Rectangle(pos_hint=self.pos,size=(1920,1020))
                    return True
    
        with self.canvas.before:
                Color(0.,0.,0.0,1) #just the default background 
        return False



# ------------------------------------------------ #
# -------- LabelApp Auxilliary Classes ----------  #
# ------------------------------------------------ #




class KivyColors():
    # default (Button-)Colors in kivy
    red = [1,0,0,1]             # discarded
    red_dark = [1,0,0,0.6] 
    green = [0,1,0,1]           # labled
    green_dark = [0,1,0,0.6]
    blue =  [0,0,1,1]           # active
    blue_dark =  [0,0,1,0.6]    # active
    purple = [1,0,1,1]    
    purple_dark = [1,0,1,0.6]
    yellow = [1,1,0,1]
    yellow_dark = [1,1,0,0.6]
    orange = [1,.45, 0,1]
    orange_dark =[1,.45,0,0.6]
    black = [0,0,0,0]     
    white = [1,1,1,1]     
    grey = [0.5,0.5,0.5,1]
    grey_dark = [0.3,0.3,0.3,1]
    nac = [.45,0,1,1]


class PandaPublisherColorSwitch():

    def getKivyColor(self,publisherMode):
        switch = {'PandaPublisherModeEnum.replay_requested': KivyColors.red_dark,
                                'PandaPublisherModeEnum.paused_trajectory': KivyColors.purple,
                                'PandaPublisherModeEnum.replaying_trajectory': KivyColors.blue,
                                'PandaPublisherModeEnum.idle': KivyColors.green_dark,
                                'PandaPublisherModeEnum.prepare_for_publishing': KivyColors.grey,
                                'PandaPublisherModeEnum.advance_initial_position': KivyColors.orange_dark,
                                'PandaPublisherModeEnum.publishSample': KivyColors.nac,
                                'PandaPublisherModeEnum.wait_for_behavior': KivyColors.yellow_dark,
                                'PandaPublisherModeEnum.endPublishing': KivyColors.nac,
                                'PandaPublisherModeEnum.recording': KivyColors.red,
                                }
        return switch[str(publisherMode)]

    def getString(self,publisherMode):
        switch = {'PandaPublisherModeEnum.replay_requested': 'red',
                            'PandaPublisherModeEnum.paused_trajectory': 'purple',
                            'PandaPublisherModeEnum.replaying_trajectory': 'blue',
                            'PandaPublisherModeEnum.idle': 'green',
                            'PandaPublisherModeEnum.prepare_for_publishing': 'white',
                            'PandaPublisherModeEnum.advance_initial_position': 'orange',
                            'PandaPublisherModeEnum.publishSample': 'nac',
                            'PandaPublisherModeEnum.wait_for_behavior': 'yellow',
                            'PandaPublisherModeEnum.endPublishing': 'nac',
                            'PandaPublisherModeEnum.recording': 'red',
                            }     
        return switch[str(publisherMode)]


class PandaLedBarometer():

    def __init__(self):
        self.topic = rospy.get_param('visual_output_topic', False)
        if(self.topic == False):
            print("Error getting topic for LedBarometer! Set to /led_color")
            self.topic = "led_color"            
        self.pub = rospy.Publisher(self.topic, led_color.VisualOutput, queue_size=3)
        print("Start publishing LED-Colors")


class ProgressCounter(ProgressBar):

    def __init__(self, **kwargs):
        super(ProgressCounter, self).__init__(**kwargs)
        self.radius = 40
        self.text_size = (100,100)
        text="0"
        self.counter = 0
        self.updateBar(text)

    def updateBar(self,text,width = 50,size_=(80,80)):
        " text: Status Text in the center |width: width of moving zone | angle: is set acording to self.value"
        with self.canvas:
            self.canvas.clear()
            """ Draw the circle """
            Color(0.2, 0.2, 0.2)
            Ellipse(pos=self.pos, size=size_)

            """Draw highlighted area"""
            Color(1, 0, 0)
            progress = (0.0005 if self.value_normalized == 0 else self.value_normalized*360)
            Ellipse(pos=self.pos, size=size_,
                    angle_end=progress,angle_start=progress - width)        #second Ellipse
            Ellipse(pos=self.pos, size=size_,   
                angle_end=progress+180,angle_start=progress - width + 180)  #first Ellipse       
            """Draw Center"""
            Color(0, 0, 0)
            Ellipse(pos=(self.pos[0] + self.radius / 2, self.pos[1] + self.radius / 2),
                    size=(size_[0] - self.radius, size_[1] - self.radius))
            Color(1, 1, 1, 1)
            Label(text=text, font_size=self.radius,pos=(self.pos[0] + (size_[0]/2 - self.text_size[0]/2),self.pos[1] +(size_[1]/2 - self.text_size[1]/2)))

    def set_value(self, value):
        self.value = value%100
        self.counter = value        
        text = str(int(self.counter))
        self.updateBar(text)



class TooltipConfig(object):
    """ Subclass storeing basic Configuration of the displayed Tooltip
        Text: Text to Display
        Color: Tooltip Color
    """
    def __init__(self, text = "Tooltip", color=KivyColors.grey):
        self.text = text
        self.color = color

class Tooltip(Label):
    """ Tooltip displayed in Root App """
    def __init__(self,text=None,color=None, **kwargs):
        super(Label, self).__init__(**kwargs)
        with self.canvas.before:
            app = App.get_running_app()
            if app.root == None:
                return 
            size_ = np.array(app.root.size)
            tl = Button(text=text,size=(max(200,0.2*size_[0]),0.2*size_[1]), pos=(0.73*size_[0],0.75*size_[1]),valign="top",halign="left",background_color=color,markup=True)  
            tl.text_size = tl.size

class CustomTrajectoryStatus(object):

    def hide_status(self):
        self.canvas.after.clear()

    def updateSliderStatus(self,value1,value2,thickness=3,highlight_color=None):
        with self.canvas.after:
            x_margin = 5 # small margin at the edges
            totalwidth = self.size[0] - 2*x_margin
            value1 = value1*totalwidth
            value2 = value2*totalwidth

            Cutwidth = value2 -value1

            y_edge = 2
            y_offset = self.pos[1]+self.size[1]-thickness-y_edge

            pos1 = (x_margin,y_offset) 
            size1 = (value1,thickness)

            pos2 = (value1+x_margin,y_offset) 
            size2 = (Cutwidth,thickness)

            pos3 = (value2+x_margin,y_offset) 
            size3 = (totalwidth-value2,thickness)
            
            color = KivyColors.grey
            Color(color[0],color[1],color[2],color[3])
            Rectangle(pos = pos1,size = size1)

            if highlight_color != None:
                if isinstance(highlight_color,KivyColors):
                    color = highlight_color
            else:
                color = KivyColors.green
            Color(color[0],color[1],color[2],color[3])            
            Rectangle(pos = pos2,size = size2)

            color = KivyColors.grey
            Color(color[0],color[1],color[2],color[3])
            Rectangle(pos = pos3,size = size3)



class Tooltipaction(object):

    def __init__(self, **kwargs):
        self.tooltip_config = None
        self.parent_widget = None
        self.active_tooltip = None
        Window.bind(mouse_pos=self.on_mouse_pos) 

    def on_mouse_pos(self, *args):
        if self.parent_widget == None:
            self.parent_widget = App.get_running_app().root
        #position is given by cursorposition 
        cursor_posx = args[1][0]
        cursor_posy = args[1][1]
        pos = self.to_window(*self.pos)

        success = True
        if cursor_posx < pos[0] or cursor_posx > pos[0] + self.size[0]:
            success = False
        if cursor_posy < pos[1]  or cursor_posy > pos[1] + self.size[1]:
            success = False
   
        #success = self.collide_point(cursor_posx,cursor_posy) #kivy's onboard feature does for some reason not work ...
        if success == False:
            if self.active_tooltip != None: # This entity has an active Tooltip
                self.close_tooltip() # we just left that area 
            return 

        if self.active_tooltip == None:
            if self.tooltip_config == None: #not set yet
                return False  
            self.active_tooltip = Tooltip(text=self.tooltip_config.text,color = self.tooltip_config.color,markup = True)
            self.parent_widget.add_widget(self.active_tooltip)
        

    def close_tooltip(self, *args): 
        if self.active_tooltip != None:  
            self.parent_widget.remove_widget(self.active_tooltip) 
            self.active_tooltip = None 

class Features(Tooltipaction):
    def init(self,parent = None,tooltip_config = None, **kwargs):
        self.set_tooltip_config(tooltip_config)
        super(Tooltipaction,self).__init__(**kwargs)
        self.setparent(parent)

    def setparent(self,parent_): 
        if parent_ == None:
            app = App.get_running_app()
            self.parent_widget = app.root

        else:
            self.parent_widget = parent_

    def set_tooltip_config(self,tooltip_config_=None):    
        if tooltip_config_ != None: 
            self.tooltip_config = tooltip_config_  
        else: 
            self.tooltip_config = Tooltip(text="Tooltip") # text from Button



class TooltipContainer(RelativeLayout):
    def __init__(self,parent = None,tooltip_config = None,size_hint=None,pos_hint=None,**kwargs):
        super(RelativeLayout, self).__init__(**kwargs)
        self.shape_tooltip_container(parent,tooltip_config,size_hint,pos_hint)

    def shape_tooltip_container(self,parent,tooltip_config,size_hint,pos_hint): #TODO
        # for having the tooltips
        tooltip_overlay = TooltipButton(parent=parent,tooltip_config=tooltip_config,size_hint=size_hint,pos_hint=pos_hint)
        tooltip_overlay.canvas.clear() # remove canvas
        #for having the status
        self.add_widget(tooltip_overlay)      


class TooltipBox(RelativeLayout):
    def __init__(self,parent = None,tooltip_config = None,size_hint=None,pos_hint=None,**kwargs):
        super(RelativeLayout, self).__init__(**kwargs)
        self.shape_tooltip_container(parent,tooltip_config,size_hint,pos_hint)

    def shape_tooltip_container(self,parent,tooltip_config,size_hint,pos_hint): #TODO
        # for having the tooltips
        tooltip_overlay = TooltipButton(parent=parent,tooltip_config=tooltip_config,size_hint=size_hint,pos_hint=pos_hint)
        tooltip_overlay.canvas.clear() # remove canvas

        #for having the status
        self.layout_helper = BoxLayout(orientation='vertical',size_hint=size_hint,pos_hint=pos_hint) # just for having the right layout       

        self.add_widget(tooltip_overlay)      
        self.add_widget(self.layout_helper)
    
class TooltipTextInput(RelativeLayout):
    def __init__(self,text="",parent = None,tooltip_config = None,size_hint=None,pos_hint=None,**kwargs):
        super(RelativeLayout, self).__init__(**kwargs)
        self.shape_tooltip_container(parent,tooltip_config,size_hint,pos_hint,text)

    def shape_tooltip_container(self,parent,tooltip_config,size_hint,pos_hint,text): #TODO
        # for having the tooltips #TODOMLKMOMLMLM
        tooltip_overlay = TooltipButton(text="",parent=parent,tooltip_config=tooltip_config,size_hint=size_hint,pos_hint=pos_hint)
        tooltip_overlay.canvas.clear() # remove canvas
 
        #for having the Textinput
        self.textinput = TextInput(text=text,multiline=False) # just for having the right layout       
        self.add_widget(tooltip_overlay)
        self.add_widget(self.textinput)

    def bind(self,**args):
        self.textinput.bind(**args)

class TooltipLabelButton(RelativeLayout,Label,Features):

    def __init__(self,text="",parent = None,tooltip_config = None,size_hint=None,pos_hint=None, playbackController=None, **kwargs):
        super(RelativeLayout, self).__init__(**kwargs)
        self.shape_tooltip_container(text,parent,tooltip_config,size_hint,pos_hint)
        self.playbackController=playbackController
        self.text=text

    def shape_tooltip_container(self,text,parent,tooltip_config,size_hint,pos_hint): #TODO
        if parent == None:
            raise Exception("Error -  Parent not set")
            return False
        self.btn = TooltipButton(text=text,parent=parent,tooltip_config=tooltip_config,size_hint=size_hint,pos_hint=pos_hint)
        remove_option = TooltipButton(text="X",parent=parent,tooltip_config=parent.Tooltip_Config_List['RemoveLabel'],size_hint=(0.1,1),pos_hint=pos_hint,background_color=KivyColors.red)    
        remove_option.bind(on_press=self.remove_self)
        self.add_widget(self.btn)
        self.add_widget(remove_option)


    def bind(self,**args):
        self.btn.bind(**args)

    def remove_self(self,instance=0,state=0):
        if self.playbackController is not None:
            self.playbackController.deleteFromAvailableLabels(self.text)
        self.parent.remove_widget(self)


class TooltipSwitch(Switch,Features,CustomTrajectoryStatus):

    def __init__(self,parent = None,tooltip_config = None, **kwargs):
        super(Switch, self).__init__(**kwargs)
        self.init(parent,tooltip_config, **kwargs)


class TooltipButton(Button,Features,CustomTrajectoryStatus):

    def __init__(self,parent = None,tooltip_config = None, **kwargs):
        super(Button, self).__init__(**kwargs)
        self.init(parent,tooltip_config, **kwargs)

class TooltipImageButton(RelativeLayout):
    """ Button with Custom Tooltip and background Image """ 
    def __init__(self,parent = None,tooltip_config = None,image_source="", **kwargs):
        super(RelativeLayout, self).__init__(**kwargs)
        self.TButton = TooltipButton(parent = parent,tooltip_config=tooltip_config)
        self.add_widget(self.TButton)
        if image_source != "":
            self.Image = Image(source=image_source,size=self.TButton.size,pos=self.TButton.pos,allow_stretch='True')
            self.add_widget(self.Image)
    
    def bind(self,**args):
        self.TButton.bind(**args)

    def change_image(self,image_source = None):
        """ Function for changing the image source of this button """
        if image_source != None:
            self.Image.source = image_source       

    def set_tooltip_config(self,tooltip_config = None):
        """ Function for changing the Tooltipconfiguration"""
        if tooltip_config != None:
            self.TButton.set_tooltip_config(tooltip_config)
        else:
            return False


class TooltipToggleButton(ToggleButton,Features):
    def __init__(self,parent = None,tooltip_config = None, **kwargs):
        super(ToggleButton, self).__init__(**kwargs)
        self.init(parent,tooltip_config, **kwargs)

class TooltipSlider(Slider,Features):
    def __init__(self,parent = None,tooltip_config = None,**kwargs):
        super(Slider,self).__init__(**kwargs)
        self.init(parent,tooltip_config, **kwargs)


class TooltipActionButton(ActionButton,Features):

    def __init__(self,parent = None,tooltip_config = None, **kwargs):
        super(Button, self).__init__(**kwargs)
        self.init(parent,tooltip_config, **kwargs)

class TooltipActionDropDown(ActionDropDown,Features):
    """ Creates a Tooltip for an ActionDropDown Button from Kivy ActionView"""
    def __init__(self,parent = None,tooltip_config = None, **kwargs):
        super(ActionDropDown, self).__init__(**kwargs)
        self.init(parent,tooltip_config, **kwargs)


class CustomSliderProgressbar(RelativeLayout):
    """ Custom Trajectory Progressbar with Sliderwidget to set the Cutlist Start and End for one Trajectory """
    def __init__(self,**kwargs):
        super(RelativeLayout,self).__init__(**kwargs)
        self.ProgressBar = CustomSliderProgress()
        self.add_widget(self.ProgressBar)

        image_width = (30,30)
        pos = (-2*image_width[0],-image_width[1]/2)
        self.Slider_Start = CustomSlider(id="Slider_Start",pos=pos,image_pos=(0,0),image_size = image_width,x_offset=-image_width[0], on_motion=self.Slider_Start_moved, on_touch_up=self.finalizeSliderChanges)
        self.add_widget(self.Slider_Start)

        pos = (0,-image_width[1]/2)
        self.Slider_End = CustomSlider(id="Slider_End",source_=global_path_to_kivy_images+ 'SliderEnd.png',pos=pos,value=1,image_pos=(0,0),image_size = image_width,x_offset=15, on_motion=self.Slider_End_moved, on_touch_up=self.finalizeSliderChanges)
        self.add_widget(self.Slider_End)

    def update_value(self,value=None,Slider1=None,Slider2=None):
        Success, Sliderstart, Sliderend = self.ProgressBar.update_value(value=value,Slider1 = Slider1,Slider2=Slider2)
        if Success:
            self.Slider_Start.extern_set_value(Sliderstart)   
            self.Slider_End.extern_set_value(Sliderend)

    def render_widget(self):
        self.Slider_Start.update_pos()
        self.Slider_End.update_pos()
        self.ProgressBar.render_widget()

    def Slider_Start_moved(self,value):
        try:
            kivyinterface = App.get_running_app().kivyinterface
        except:
            return
        kivyinterface.setCursorStart(value)

    def Slider_End_moved(self,value):
        try:
            kivyinterface = App.get_running_app().kivyinterface
        except:
            return    
        kivyinterface.setCursorEnd(value)
    
    def finalizeSliderChanges(self, value):
        try:
            pb = App.get_running_app().playbackController
        except:
            return    
        pb.commitMetaData()
        

class CustomSlider(RelativeLayout):
    def __init__(self,id = None,source_= None,x_offset=0,image_pos=(0,0),value= 0,image_size = (40,40),on_motion = None,on_touch_up = None,**kwargs):
        super(RelativeLayout,self).__init__(**kwargs)
        self.x_offset = 0
        if id != None:
                self.x_offset = x_offset
        if source_ == None:
            source_ = global_path_to_kivy_images + 'SliderStart.png'
        self.callback_on_motion = on_motion
        self.callback_on_touch_up = on_touch_up
        self.scatter_ = Scatter(size=image_size,do_rotation=False,do_scale=False,do_translation_y=False,auto_bring_to_front=True)
        self.scatter_.bind(pos=self.on_motion)
        self.image = Image(source=source_,size=image_size,pos=image_pos) 
        self.scatter_.add_widget(self.image)
        self.add_widget(self.scatter_)
        self.extern_set_value(value)

    def bind_(self,callback):    
        self.callback_on_motion = callback

    def extern_set_value(self,value=None):
        if type(value == float):
            self.value = max(0.0,min(value,1.0)) 
            self.update_pos()

    def update_pos(self):
            posx= self.value*self.size[0]-self.x_offset #new position
            self.scatter_.pos= (posx,self.scatter_.pos[1]) #callback triggered

    def on_motion(self,instance=0,pos_value=0): # etype, motionevent):
        self.value = max(0,min(1,(pos_value[0]+self.x_offset)/(self.size[0])))
        if self.callback_on_motion != None:
            self.callback_on_motion(self.value)
            
    def on_touch_up(self,instance=0,pos_value=0): # etype, motionevent):
        if self.callback_on_touch_up != None:
            self.callback_on_touch_up(pos_value)


class CustomSliderProgress(RelativeLayout):
    def __init__(self,parent=None,TooltipConfig_List=None,**kwargs):
        super(RelativeLayout,self).__init__(**kwargs)
        self.value = -1 # Startup 
        self.line_width = 15
        self.background_rendered = False
        self.SliderStart = 0.0
        self.SliderEnd = 0.0

        
    def render_background(self):
        try:
            with self.canvas:
                Color(float(232)/255,float(255)/255,float(229)/255) #beige
                Line(points=[self.pos[0],self.pos[1],self.pos[0]+self.size[0],self.pos[1]],cap="square",width=self.line_width+2)#alternative
                if self.background_rendered == False:
                    self.background_rendered = True
        except:
            print("ProgressBarBackground not rendered")

    def render_widget(self):
        """ Function in case the Progressbar has to be rendered again """

        self.canvas.clear()
        self.render_background()

        with self.canvas:

            if self.SliderStart < self.SliderEnd:
                Color(float(140)/255,float(255)/255,float(25)/255) #green 
                Line(points=[self.pos[0]+self.SliderStart*self.size[0],self.pos[1],self.pos[0]+self.SliderEnd*(self.size[0]-self.line_width),self.pos[1]],cap="square",width=self.line_width) #render the Cutout area

            #Color(float(255)/255,float(50)/255,float(50)/255)#red
            Color(float(255)/255,float(111)/255,float(25)/255)#orange
            Line(points=[self.pos[0]+self.SliderStart*self.size[0],self.pos[1],self.pos[0]+(self.value*(self.size[0]-20)),self.pos[1]],cap="round",width=self.line_width*0.8) #renders the progress

            Color(float(0)/255,float(76)/255,float(76)/255) # dark green 
            Line(points=[self.pos[0],self.pos[1],self.pos[0],self.pos[1]],cap="square",width=self.line_width)#render the edges
            Line(points=[self.pos[0]+self.size[0],self.pos[1],self.pos[0]+self.size[0],self.pos[1]],cap="square",width=self.line_width)
        
        return True,self.SliderStart,self.SliderEnd


    def update_value(self,value=None,Slider1=None,Slider2=None):
            """ value: relative value of progressbar """

            
            if self.background_rendered == False: #do that once
                self.render_background()
          

            if value == self.value:
                """ don't render if value hasn't changed """
                return False,0,1

            """ Checking the arguments here """
            if value != None:
                value = max(min(1,value),0) #between 0 and 1
                self.value = value

            if Slider1 != None and Slider1 >=0 and Slider1 <= 1 :
                self.SliderStart = float(Slider1)
                
            if Slider2 != None and Slider2 >=0 and Slider2 <= 1 :
                self.SliderEnd = float(Slider2)
     
            return self.render_widget()



class LabelAppRvizPlayer(RelativeLayout):

    def __init__(self,parent=None,TooltipConfig_List=None,**kwargs):
        """ Container Class for LabelAppPlayer with custom images"""    
        super(RelativeLayout,self).__init__(**kwargs)
        """ pos_hint_ needed for Inserting the image at correct position """    
        # check if Images are present
        if not os.path.isfile(global_path_to_kivy_images + 'playbtn.png'): # check for first Image
            print("Warning - Put images for Action Btns in ROS_DATA/images for having Actionview\n".format())
            raise SystemExit("Warning - Put images for Player Btns in {} for having Actionview\n".format(global_path_to_kivy_images))

        if parent == None:
            raise RuntimeError("Error - No Parent for Callbacks")

        if TooltipConfig_List == None:
            RuntimeError("Error - Tooltips for RVIZ-Player not set.")

        layout_helper = BoxLayout(orientation = 'horizontal')    



        #btn = TooltipImageButton(tooltip_config=TooltipConfig_List['previousLabel'], image_source = global_path_to_kivy_images+ 'prevbtn.png' )
        #btn.bind(on_press=parent.prevcallback)      
        #layout_helper.add_widget(btn)   

        btn =  TooltipImageButton(tooltip_config=TooltipConfig_List['JumpBack'],image_source = global_path_to_kivy_images+ 'jumpBackbtn.png' )
        btn.bind(on_press=parent.jumpBack)      
        layout_helper.add_widget(btn)        

        btn = TooltipImageButton(tooltip_config=TooltipConfig_List['Repeat'],text= '',image_source = global_path_to_kivy_images+ 'jumpBack_Play_btn.png' )
        btn.bind(on_press=parent.jumpBackAndRepeat) 
        layout_helper.add_widget(btn)   

        btn =  TooltipImageButton(tooltip_config=TooltipConfig_List['goBack'],text= '',image_source = global_path_to_kivy_images+ 'playrevbtn.png'  )
        btn.bind(on_press=parent.togglePlayBackwards)  
        layout_helper.add_widget(btn)

        parent.togglereplaybtn =  TooltipImageButton(tooltip_config=TooltipConfig_List['Play'],text= '',image_source = global_path_to_kivy_images+ 'playbtn.png' )
        parent.togglereplaybtn.bind(on_press=parent.toggleReplaying)
        layout_helper.add_widget(parent.togglereplaybtn)
        
        btn =  TooltipImageButton(tooltip_config=TooltipConfig_List['Record'],image_source = global_path_to_kivy_images+ 'recordbtn.png' )
        btn.bind(on_press=parent.toggleRecording)      
        layout_helper.add_widget(btn)  

        btn =  TooltipImageButton(tooltip_config=TooltipConfig_List['PlayAndRecord'],image_source = global_path_to_kivy_images+ 'play_and_recordbtn.png' )
        btn.bind(on_press=parent.toggleReplayRecord)      
        layout_helper.add_widget(btn)  

        #btn = TooltipImageButton(tooltip_config=TooltipConfig_List['nextLabel'],image_source = global_path_to_kivy_images+ 'nextbtn.png')
        #btn.bind(on_press=parent.nxtcallback)      
        #layout_helper.add_widget(btn)   

        self.add_widget(layout_helper) 


class ViewerOptionsCreator(object):

    def __init__(self):
        pass
    # -------------------------------------------- #
    # -------- viewer config buttons ------------  #
    # -------------------------------------------- #
    def create_widgets(self,parent=None,TooltipConfig_List = None):

        if parent == None:
            print("Error - Creating widgets!")
            return False


        if TooltipConfig_List == None:
            print("Error - Tooltips for ViewerOptions not set.")    
            return False
           

        Vieweroptions = BoxLayout(orientation='vertical',size_hint=(0.1,0.1),pos_hint={'x': 0.93, 'y': 0.6})
        Vieweroptions2 = BoxLayout(orientation='vertical',size_hint=(0.5,0.4),pos_hint={'x': 0.7, 'y': 0.04})

        btnsize=(0.4,0.3)
        btnpos={'x': 0.35, 'y': 0}
        
        openbtn = TooltipButton(
                    text = "Toggle Gripper", 
                    pos_hint=btnpos, size_hint = btnsize,
                    tooltip_config = TooltipConfig_List['OpenGripper']
        )
        openbtn.bind(on_press=parent.setGripper)
        Vieweroptions2.add_widget(openbtn)

        learnBtn = TooltipButton(
                parent=parent,
                text = "Learn behavior",
                pos_hint=btnpos, size_hint = btnsize,
                tooltip_config = TooltipConfig_List['LearnBehavior']
        )
                    
        learnBtn.bind(on_press=parent.generateBehavior)
        Vieweroptions2.add_widget(learnBtn)


        simulationbtn = TooltipButton(
                    text = "Run Behavior",
                    pos_hint=btnpos, size_hint = btnsize,
                    tooltip_config = TooltipConfig_List['RunBehavior']
        )


        simulationbtn.bind(on_press=parent.runBehaviorSimulation)
        Vieweroptions2.add_widget(simulationbtn)


        quitBtn = TooltipButton(
                    text = "Quit",
                    pos_hint=btnpos, size_hint = btnsize,
                    tooltip_config = TooltipConfig_List['Quit']
        )

        quitBtn.bind(on_press=parent.quitApplicationCallback)
        Vieweroptions2.add_widget(quitBtn)



        return Vieweroptions,Vieweroptions2, simulationbtn





class TooltipConfigListCreatorTool(object):

    def __init__(self):
        pass

    def create_TooltipConfigList(self,parent=None):

        # --------------------------------------------  #
        # -------- Tooltip Markup Option------------    #
        # --------------------------------------------  #
        mup_open  = "[size=18][b][color=00ff00]"        #color = green, size = 14, bold
        mup_close = "[/size][/b][/color]"
        # --------------------------------------------  #

        if parent == None:
            parent = App.get_running_app()

        TooltipConfig_List = {}
        TooltipConfig_List['OpenGripper'] = TooltipConfig(text=
            "Open the Gripper manually. MTI Panda Controller Software has to be running for that.")

        TooltipConfig_List['Repeat'] = TooltipConfig(text= "Triggers a jump Back to Start and an immediately repeat of the Trajectory.".format(mup_open,mup_close))

        TooltipConfig_List['Pause'] = TooltipConfig(text= "Pause actual Replay. Key: >>{0}Back space{1}<<".format(mup_open,mup_close))

        TooltipConfig_List['Play'] = TooltipConfig(text= "Plays actual Trajectory to Panda, but doens't start recording. Key: >>{0}Back space{1}<<".format(mup_open,mup_close))

        TooltipConfig_List['goBack'] = TooltipConfig(text= "Move arm Back to Beginning of Trajectory. Key: >>{0}Minus |-{1}<<".format(mup_open,mup_close))

        TooltipConfig_List['JumpBack'] = TooltipConfig(text= "Jump back directly to Start of Trajectory.Key: >>{0}Plus | +{1}<<".format(mup_open,mup_close))

        TooltipConfig_List['Record'] = TooltipConfig(text= "Start recording Samples from Panda. Key: >>{0}Enter{1}<<".format(mup_open,mup_close))

        TooltipConfig_List['PlayAndRecord'] = TooltipConfig(text= "Play actual Trajectory to Panda and start with recording immediately. Key: >>{0}CenterPad | 5{1}<< ".format(mup_open,mup_close))

        TooltipConfig_List['LearnBehavior'] = TooltipConfig(text= "Learn the behavior from all labeled Trajectories.For each Label / Segment there will be generated a specific Promp in ./behavior and useful additional evaluation information in ./illustration inside ROS_Data Directory:{0}".format(parent.sessionConfigYamlpath))

        TooltipConfig_List['RunBehavior'] = TooltipConfig(
            text="Run this after successful generation of Promp-Data inside behavior.It will launch an Simulation of the Promp-Behavior and vizualize the result."
        )
               
        TooltipConfig_List['Quit'] = TooltipConfig(text= "Quit this application.")

        TooltipConfig_List['SliderStart'] = TooltipConfig(text = "Slider to set the Start of the Trajectory. Everything on the left will be cut out.")

        TooltipConfig_List['SliderStop'] = TooltipConfig(text = "Slider to set the Stop of the Trajectory. Everything on the right will be cut out.") 

        TooltipConfig_List['previousLabel'] = TooltipConfig(text="Switch to previous Label in Labellist. Key: >>{0}Arrow Up | 4 {1}<<".format(mup_open,mup_close))

        TooltipConfig_List['nextLabel'] = TooltipConfig(text="Switch to next Label in Labellist. Key: >>{0}Arrow Down|6{1}<<".format(mup_open,mup_close))

        TooltipConfig_List['EnterNewLabel'] = TooltipConfig(text="Enter a Label Name here. It will be available below.".format(mup_open,mup_close))

        TooltipConfig_List['discarded'] = TooltipConfig(text= "Mark actual Label as discarded.")

        TooltipConfig_List['Undo'] = TooltipConfig(text="Undo last Labelling.")

        TooltipConfig_List['ActiveTrajectory'] = TooltipConfig(text="Currently active Trajectory Number")
 
        TooltipConfig_List['StatusField'] = TooltipConfig(text="Status Field with 3 rows.\n1. Controller Status\n2. Recorded Samples\n3. Actual Sample/MaxSample of Trajectory")

        TooltipConfig_List['RemoveLabel'] = TooltipConfig(text="Click here to remove the Label from List")

        TooltipConfig_List['Background'] = TooltipConfig(text="Click here to switch between backgrounds")


        return TooltipConfig_List
