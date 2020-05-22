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
    from kivy.uix.behaviors import ButtonBehavior      
    from kivy.uix.togglebutton import ToggleButton
    from kivy.uix.textinput import TextInput
    from kivy.uix.scrollview import ScrollView
    from kivy.uix.scatter import Scatter
    from kivy.uix.gridlayout import GridLayout
    from kivy.properties import ListProperty, StringProperty, NumericProperty
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

mup_open  = "[size=18][b][color=00ff00]"        #color = green, size = 14, bold
mup_close = "[/size][/b][/color]"
Tooltip_Config_List ={
            'OpenGripper': "Open the Gripper manually. MTI Panda Controller Software has to be running for that.",
            'Repeat': "Triggers a jump Back to Start and an immediately repeat of the Trajectory.",
            'Pause': "Pause actual Replay. Key: >>{0}Back space{1}<<".format(mup_open,mup_close),
            'Play': "Plays actual Trajectory to Panda, but doens't start recording. Key: >>{0}Back space{1}<<".format(mup_open,mup_close),
            'goBack': "Move arm Back to Beginning of Trajectory. Key: >>{0}Minus |-{1}<<".format(mup_open,mup_close),
            'JumpBack': "Jump back directly to Start of Trajectory.Key: >>{0}Plus | +{1}<<".format(mup_open,mup_close),
            'Record': "Start recording Samples from Panda. Key: >>{0}Enter{1}<<".format(mup_open,mup_close),
            'PlayAndRecord': "Play actual Trajectory to Panda and start with recording immediately. Key: >>{0}CenterPad | 5{1}<< ".format(mup_open,mup_close),
            'LearnBehavior': "Learn the behavior from all labeled Trajectories.For each Label / Segment there will be generated a specific Promp in ./behavior and useful additional evaluation information in ./illustration inside ROS_Data Directory",
            'RunBehavior': "Run this after successful generation of Promp-Data inside behavior.It will launch an Simulation of the Promp-Behavior and vizualize the result.",
            'Quit': "Quit this application.",
            'SliderStart': "Slider to set the Start of the Trajectory. Everything on the left will be cut out.",
            'SliderStop': "Slider to set the Stop of the Trajectory. Everything on the right will be cut out.",
            'previousLabel':"Switch to previous Label in Labellist. Key: >>{0}Arrow Up | 4 {1}<<".format(mup_open,mup_close),
            'nextLabel':"Switch to next Label in Labellist. Key: >>{0}Arrow Down|6{1}<<".format(mup_open,mup_close),
            'EnterNewLabel': "Enter a Label Name here. It will be available below.".format(mup_open,mup_close),
            'discarded': "Mark actual Label as discarded.",
            'Undo':"Undo last Labelling.",
            'ActiveTrajectory':"Currently active Trajectory Number",
            'StatusField':"Status Field with 3 rows.\n1. Controller Status\n2. Recorded Samples\n3. Actual Sample/MaxSample of Trajectory",
            'RemoveLabel':"Click here to remove the Label from List",
            'Background':"Click here to switch between backgrounds",
            'cutlistbtn': "Press this button to hide / update all cuts from Trajectory",

}


Builder.load_string("""
<BoxLayout>:
    padding: 2
<Button>:  
    color: 0.1, 0.3, 0.4  

<BoxLayout>
    canvas.before:
        Color:
            rgba: 0.1, 0.1, 0.1, 1
        Rectangle:
            pos: self.pos
            size: self.size

""")  



class KivyRootWidget(BoxLayout):

    def __init__(self, sessionConfigYamlpath, playbackController, **kwargs):
        """ Function executes the complete setup for Kivy-Graphics of LabelApp """
        self.status_canvas_created = False # lock callback until canvas has been created
        kwargs['orientation'] = 'vertical'
        kwargs['background_color'] = (0.1,0.1,0.1,1)
        super(KivyRootWidget, self).__init__(**kwargs)        


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
        self.sessionConfig['data_directory'] = os.path.abspath(os.path.expanduser(self.sessionConfig['data_directory']))
        Config.set('input', 'mouse', 'mouse,multitouch_on_demand') #remove red Dot's when doing right click

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

    def onImpedanceSliderValue(self, instance, value):
        impedance_proposed = self.playbackController.maxStiffness * value**3 #stiffness 0 to 10
        impedance_set = self.playbackController.setStiffnessOfCurrentActiveTrajectory(impedance_proposed)
        self.impedanceDisplayLabel.text = "{:4.2f}".format(impedance_set)
        self.impedanceDisplayLabel.color = [1.0, 1.0-value, 1.0-value, 1]

    def presetPlaybackImpedanceDisplay(self, impedance):
        if impedance is None:
            self.impedanceSlider.disabled= True
            self.impedanceDisplayLabel.disabled = True
        else:
            self.impedanceSlider.disabled = False
            self.impedanceDisplayLabel.disabled = False
            value  = float( (impedance / self.playbackController.maxStiffness)**(1./3)) #cast because kivy doesnt handle numpy floats
            self.impedanceSlider.value = value 
            self.impedanceDisplayLabel.text = "{:4.2f}".format(impedance)
            self.impedanceDisplayLabel.color = [1.0, 1.0-value, 1.0-value, 1]

    def init_kivy_elements(self):
        """ Initializes and orders all Kivy-Interface-Elements that are visible to the user."""   

        # -------------------------------------------- #
        # -------- Setup the KivyGui-Layout ---------- #
        # -------------------------------------------- #

        #general UI structure:
        rootPane = self
        mainPane = BoxLayout(orientation='horizontal')
        statusPane = BoxLayout(orientation='horizontal', size_hint_y=0.05,  background_color=(0,1,0))
        rootPane.add_widget(mainPane)
        rootPane.add_widget(statusPane)

        leftPane = BoxLayout(orientation='vertical',background_color=(0,0.3,0.1), size_hint_x=0.4, padding=20)
        mainPane.add_widget(leftPane)

        middlePane = BoxLayout(orientation='vertical',background_color=(0,0.3,0.2),size_hint_x=0.4, padding=20)
        mainPane.add_widget(middlePane)

        rightPane= BoxLayout(orientation='vertical',background_color=(0,0.3,0.3), size_hint_x=0.2,padding=20)
        mainPane.add_widget(rightPane)

        self.Tooltip_Config_List = Tooltip_Config_List


        self.modePane = ModePaneLayout(orientation='horizontal', size_hint_y=0.1)
        middlePane.add_widget(self.modePane)




        playerPane = BoxLayout(orientation='vertical', padding=2, space=2, size_hint_y=0.2) #to keep controls and editor bar together
        middlePane.add_widget(playerPane)

        self.trajectoryEditorBar = TrajectoryEditorBar(playback_controller = self.playbackController, height=50)
        playerPane.add_widget(self.trajectoryEditorBar)

        for name in ('inpoint', 'currentsample', 'outpoint'):
            self.modePane.add_widget(self.trajectoryEditorBar.currentTrajectoryParameterDisplays[name])

            
        ##create the player bar:
        
        # check if Images are present
        if not os.path.isfile(global_path_to_kivy_images + 'playbtn.png'): # check for first Image
            print("Warning - Put images for Action Btns in ROS_DATA/images for having Actionview\n".format())
            raise SystemExit("Warning - Put images for Player Btns in {} for having Actionview\n".format(global_path_to_kivy_images))

        actionbar_ = BoxLayout(orientation='horizontal', padding=0, space=0)

        # btn = PlayerButton(
        #     id='JumpBack',
        #     source=global_path_to_kivy_images+ 'jumpBackbtn.png' ,
        #     allow_stretch='True',
        #     on_press=self.jumpBack,
        # )       
        # actionbar_.add_widget(btn)        
        # btn = PlayerButton(
        #     id='Repeat',
        #     source=global_path_to_kivy_images+ 'jumpBack_Play_btn.png' ,
        #     on_press=self.jumpBackAndRepeat,
        # )       
        #actionbar_.add_widget(btn)        

        btn = PlayerButton(
            id='goBack',
            source=global_path_to_kivy_images+ 'playrevbtn.png' ,
            on_press=self.togglePlayBackwards,
        )       
        actionbar_.add_widget(btn)        
        btn = PlayerButton(
            id='Play',
            source=global_path_to_kivy_images+ 'playbtn.png' ,
            on_press=self.togglePlay,
        )       
        actionbar_.add_widget(btn)      
        
        btn = PlayerButton(
            id='Record',
            source=global_path_to_kivy_images+ 'recordbtn.png' ,
            on_press=self.toggleRecording,
        )       
        actionbar_.add_widget(btn)      
        
        btn = PlayerButton(
            id='PlayAndRecord',
            source=global_path_to_kivy_images+ 'play_and_recordbtn.png' ,
            on_press=self.toggleReplayRecord,
        )       
        actionbar_.add_widget(btn)      
        playerPane.add_widget(actionbar_)  



        ####Right pane UI elements:

        ###Create the right-side pane:


        Vieweroptions = BoxLayout(orientation='vertical', background_color=KivyColors.green)

        Vieweroptions.add_widget(Label(text='Playback\nImpedance', size_hint_y = 0.1, halign='center'))
        sliderbox_horizontal = BoxLayout(orientation='horizontal')
        Vieweroptions.add_widget(sliderbox_horizontal)
        self.impedanceDisplayLabel = Label(text ='0', valign='middle')
        self.impedanceSlider = Slider(
            min = 0, 
            max = 1., 
            orientation ='vertical', 
            value_track = True, 
            value_track_color =[1, 0, 0, 1], 
            on_value=self.onImpedanceSliderValue,
        ) 
        self.impedanceSlider.bind(value=self.onImpedanceSliderValue)

        sliderbox_horizontal.add_widget(Label(text=" "))
        sliderbox_horizontal.add_widget(self.impedanceSlider)
        sliderbox_horizontal.add_widget(self.impedanceDisplayLabel)

        

        Vieweroptions2 = BoxLayout(orientation='vertical')


        btn = Button(
                    id='Undo',
                    text = "Undo",
                    on_press=self.undo_label,
        )
        Vieweroptions2.add_widget(btn)

        openbtn = Button(
                    id='OpenGripper',
                    text = "Open/Close Gripper", 
                    on_press=self.setGripper,
        )
        Vieweroptions2.add_widget(openbtn)

        #self.cutlistbtn = Button( # might be needed for resize_scrollview()
        #    id='cutlistbtn',
        #    text =  'Show/Hide Crop',
        #    on_press=self.toggleCutlistInScrollview,
        #)
        #Vieweroptions2.add_widget(self.cutlistbtn)


        learnBtn = Button(
                id='LearnBehavior',
                text = "Learn behavior",
                on_press=self.generateBehavior,
        )           
        Vieweroptions2.add_widget(learnBtn)


        self.simulationbtn = Button(
                    id='RunBehavior',
                    text = "Run Behavior",
                    on_press=self.runBehaviorSimulation,                    
        )
        Vieweroptions2.add_widget(self.simulationbtn)


        quitBtn = Button(
                    id='Quit',
                    text = "Quit",
                    on_press=self.quitApplicationCallback,
        )
        Vieweroptions2.add_widget(quitBtn)


        self.statusBar = Label(id='statusbar', text="status", halign='left', minimum_size=300)
        statusPane.add_widget(self.statusBar)
        statusPane.add_widget(Label(text="Configuration: "+os.getcwd(), halign='right'))

        rightPane.add_widget(Vieweroptions)
        rightPane.add_widget(Vieweroptions2)




        # -------------------------------------------- #
        # -------- Slider to cut Trajectories ------------  #
        # -------------------------------------------- #


        self.innerLabels = BoxLayout(orientation='vertical', size_hint_y = 0.5)
        middlePane.add_widget(self.innerLabels)


        self.StartCursorpos = None 
        self.EndCursorpos = None # Last CursorSample


        # -------------------------------------------- #
        # -------- create labelname buttons ----------  #
        # -------------------------------------------- #


        btnsize=(0.3,0.3)

        btnsize=(1,1)
        self.list_of_disabled_widgets = []
        textinput = TextInput(
            id='EnterNewLabel',
            text="Enter new label here",
            on_enter=self.on_enter,
            multiline=False, 
        )
        textinput.bind(on_text_validate=self.on_enter)
        self.innerLabels.add_widget(textinput)
        self.list_of_disabled_widgets.append(textinput)
        discardedButton = Button(
                    id='discard',
                    text = "discard",
                    size_hint = btnsize,
                    disabled=True,
                    on_press=partial(self.setLabel, 'discarded'),
        )

        self.innerLabels.add_widget(discardedButton)
        self.list_of_disabled_widgets.append(discardedButton)
        
        self.labelRows = {}
        for i, label in enumerate(self.playbackController.availableLabels):
            self.addLabelButton(label, disabled=True)

        self.currentTrajectoryNumberWidget = Label(id='ActiveTrajectory', text='nothing\nselected')
        self.currentTrajectoryNumberWidget.font_size=50
        self.modePane.add_widget(self.currentTrajectoryNumberWidget)
    
        self.statusBox = BoxLayout(id='StatusField', orientation='vertical',size_hint=(1,0.7))     

        self.status_canvas_created = True # canvas has been created and can be accessed in callback

        self.statusfield = Label(text = 'statusfield',font_size=18)
        self.statusBox.add_widget(self.statusfield)

        self.samplecounter = Label(text = ' (record a trajectory)',font_size=18)
        self.statusBox.add_widget(self.samplecounter)

        self.goalstatus = Label(text = '',font_size=18)
        self.statusBox.add_widget(self.goalstatus)

            
        # ========================= scrollview of LabelList ================================================#




        self.TrajectoryButtonsLayout = GridLayout(cols=1, padding=0.01, spacing=0)
        self.TrajectoryButtonsLayout.bind(minimum_height=self.TrajectoryButtonsLayout.setter('height'))
             
        #store all trajectory buttons in one array for global access
        self.trajectoryButtons = []
        #initial populate:
        for i in range(self.playbackController.getObservationsCount()):
            row = self.playbackController.metadata.iloc[i]
            self.registerTrajectory(i,row['label'],row['playback_stiffness'])

        #make scrollview containing gridlayout
        scrollview = ScrollView(do_scroll_x=False, bar_margin=10, bar_pos_y='left')
        scrollview.add_widget(self.TrajectoryButtonsLayout)

        #make Boxlayout containing scroll view
        vbox = BoxLayout(orientation='vertical')
        vbox.add_widget(Label(text="Recorded Observations:",size_hint_y=0.08))
        vbox.add_widget(scrollview)
        leftPane.add_widget(vbox)

        #make BoxLayout containing BoxLayout
        Window.bind(on_resize=self.labelapp_resize)


    def periodic(self, dt):
        """ function for retriggering everything that is updated regularly
        --> priodic Call for central_state_handler()
        """
        self.playbackController.periodic()
        Clock.schedule_once(self.periodic, self.playbackController.getUpdateTimePeriod())


    def displayTooltip(self, id):
        return
        global Tooltip_Config_List
        if id in Tooltip_Config_List:
            self.statusBar.text  = Tooltip_Config_List[id]
            print(self.statusBar.text)
        else:
            self.statusBar.text  = ""
            print(self.statusBar.text)

    def on_mouse_pos(self, *args):
        """ update the statusbar text based on where we hover"""

        #position is given by cursorposition 
        cursor_posx = args[1][0]
        cursor_posy = args[1][1]
        pos = self.to_window(*self.pos)

        mouse_over = True
        if cursor_posx < pos[0] or cursor_posx > pos[0] + self.size[0]:
            mouse_over = False
        if cursor_posy < pos[1]  or cursor_posy > pos[1] + self.size[1]:
            mouse_over = False
    
        global Tooltip_Config_List
        if mouse_over:
            root = App.get_running_app().root
            root.displayTooltip(self.id)
        
        return False


    def addTrajectoryButton(self, label):
        """
        create and append a new trajectory button to the scroll view
        Is called by the controller if observations are loaded or new ones stored 
        """
        


    def setTrajectoryCursorParameters(self, rel_start, samplevalue, rel_stop):
        self.trajectoryEditorBar.set_value_to_cursor('inpoint', rel_start)
        self.trajectoryEditorBar.set_value_to_cursor('outpoint', rel_stop)
        self.trajectoryEditorBar.set_value_to_cursor('currentsample', samplevalue)


    def labelapp_resize(self,window=None, width=0, height=0):
        self.resize_scrollview()

    def resize_scrollview(self,dt=0):
        """ Function that scales Scollview in Kivy Interface according to the window-size (bug fix)."""
        if self.cutlist_enabled:
            self.toggleCutlistInScrollview()
            self.toggleCutlistInScrollview()
        else:    
            for i,btn in enumerate(self.trajectoryButtons):
                self.updateTrajectoryButtonColorAndText(i)


    def increaseImpedance(self):
        self.impedanceSlider.value = min(1.0, self.impedanceSlider.value + 0.1)
 
    def decreaseImpedance(self):
        self.impedanceSlider.value = max(0.0, self.impedanceSlider.value - 0.1)

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
                'insert':           "publish", #same key but with numlock

                'numpad2':          "next Trajectory",
                'down':             "next Trajectory",

                'numpad8':          "previous Trajectory", 
                'up':               "previous Trajectory", 

                '+':                "increase impedance",
                '-':                "decrease impedance",

                'numpadadd':        "increase impedance",
                'numpadsubstract':  "decrease impedance",

                'enter':            "record",

                'space':            "Play",    
                'backspace':        "PlayBackwards",    

                #keypad-only interface:
                'numpad0':          "Play",
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
    
            "increase impedance":  self.increaseImpedance,
            "decrease impedance":    self.decreaseImpedance,

            "Play" :               self.togglePlay, #request Replay of Trajectory
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

 
    def addLabelButton(self, label, disabled=False):
        row  = BoxLayout(orientation='horizontal')
        size_hint_y = 0.1
        size_y = 15
        button = Button(
            id='labelbutton_'+label,
            text=label,
            size_hint=(1,1),
            size_y=size_y,
            on_press=partial(self.setLabel,label),
            disabled=disabled,
        )
        row.add_widget(button)
        delbutton = Button(
            id='RemoveLabel',
            text='x',
            background_color=KivyColors.red,
            size_hint=(0.1,1),
            size_y=size_y,
            on_press=partial(self.removeLabelAndButton, label),
        )   
        row.add_widget(delbutton)
        self.text = button.text
        self.innerLabels.add_widget(row)
        self.list_of_disabled_widgets.append(button)
        self.labelRows[label] = row
        self.innerLabels.minimum_height = (len(self.labelRows)+2)*15



    def removeLabelAndButton(self, label, buttonInstance):
        self.playbackController.deleteFromAvailableLabels(label)        
        self.innerLabels.remove_widget(self.labelRows[label])
        del self.labelRows[label]
        self.innerLabels.minimum_height = (len(self.labelRows)+2)*15
   

    def on_enter(self,instance):
        """ Callback: Textinput in KivyInterface for naming labels at runtime """
        print("on_enter")
        label = str(instance.text)
        if label in self.playbackController.availableLabels: #don't add another label buttin if label is already there
            return
        self.playbackController.addToAvailableLabels(label)
        self.addLabelButton(label)


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

        if self.keyboard_lock == False:

            key = str(keycode[1])
            if(key == "numlock" or key not in self.keyboard_mapping):
                return  

            self.active_key = self.keyboard_mapping[key]

            if self.active_key != "increase Certainty"  and self.active_key != "reduce Certainty" and self.active_key != "next Trajectory" and self.active_key != "previous Trajectory":
                #self.keyboard_lock = True #lock
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
        if self.playbackController.currentlyActiveTrajectoryNumber is None:
            return
        self.undoStack.append((self.playbackController.currentlyActiveTrajectoryNumber,self.playbackController.activeLabel))
        self.playbackController.setLabelOfCurrentlyActiveTrajectory(label) #update LabelTag for specific trajectory
        self.playbackController.activeLabel = label
        self.updateTrajectoryButtonColorAndText(self.playbackController.currentlyActiveTrajectoryNumber,label)

    
    def enableButtonsManipulatingTrajectories(self):
        for widget in self.list_of_disabled_widgets:
            widget.disabled=False


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
        else:
            self.togglereplaybtn.change_image(image_source = global_path_to_kivy_images + "playbtn.png")


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


    def togglePlay(self,instance=0,state=0):
        """
        Move Panda back step by step
        """
        self.playbackController.toggleReplaying(direction=1)


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


    # we might take less here?
    def generateBehavior(self, instance=0):
        """ Start generating promps and illustration data from labeled Trajectories """
        self.learnGraphPopupWidget = BoxLayout(orientation='vertical')
        helper = BoxLayout(orientation='vertical')
        
        self.stopprogressCounter = False
        self.progresscircle = ProgressCounter(max=100)
        helper.add_widget(self.progresscircle)
        helper.add_widget(Label(text="Generating behavior graph (state graph, ProMPs and state controllers)\nfrom the labeled segments..\n\n(This may take some time)", size_hint_y=0.5))
        self.learnGraphPopupWidget.add_widget(helper)

    
       
        self.learnGraphPopupWidgetProgressText = TextInput(text="")
        self.learnGraphPopupWidget.add_widget(self.learnGraphPopupWidgetProgressText)
            


        self.learnGraphPopup = Popup(title='Generating behavior graph', content=self.learnGraphPopupWidget, auto_dismiss=False)
        self.learnGraphPopup.open()

        # Prevents Access on Datahandler and stop all actions - undo this after Thread finished (Dissmissbtn)
        self.playbackController.PublisherMode = PandaPublisherModeEnum.wait_for_behavior 
 
        self.learnGraphSubprocess = subprocess.Popen(["rosrun", "phastapromep", "learnGraph.py","session.yaml"], stdout=subprocess.PIPE, universal_newlines=True)
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
            self.statusBox.canvas.before.clear()
            with self.statusBox.canvas.before:

                Color(color[0],color[1],color[2],color[3]) # green; colors range from 0-1 instead of 0-255
                Rectangle(size=self.statusBox.size,
                                    pos=self.statusBox.pos)
   
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
            self.goalstatus.text = "Playback Stiffness: {0:4.2f}".format(row['playback_stiffness'])
        



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
                #btn.updateSliderStatus(rel_start,rel_end)
                success = True
            else:
                print("Sliders not valid. rel_start: {0} rel_end: {1}".format(rel_start,rel_end))
                rel_start=0
                rel_end=1
                btn = self.trajectoryButtons[TrajectoryNumber]
                #btn.updateSliderStatus(rel_start,rel_end)    
                success = True

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
        btn = Button( 
            id ='trajectorybutton',
            text =  '', 
            halign='left',
        )
        btn.id = str(i)
        #force btn size at startup
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

            if 'discard' in btn.text:
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


class TrajectoryEditorBar(RelativeLayout):
    """ creates a progress bar with movable inpoint and outpoint cursors"""
    def __init__(self,**kwargs):
        super(TrajectoryEditorBar,self).__init__(**kwargs)

        self.base_height = 48
        self.playbackController = kwargs['playback_controller']
        self.playbackbar_hspace = 10

        with self.canvas.before:
            Color(0.2, 0.2, 0.2, 1)  
            self.bg_rect = Rectangle(width=self.width, height=self.height)
            Color(0.7, 0.7, 1.0, 1)  
            self.bg_bar = Rectangle(width=self.width, height=10)
        self.bind(size=self._update_pos_size_change, pos=self._update_pos_size_change)

        self.dragged_cursor = None

        image_size = (self.base_height,self.base_height)

        self.cursor_images = {'inpoint': 'CursorStart.png', 'outpoint': 'CursorEnd.png', 'currentsample':'CursorCurrent.png'}
        self.cursor_values = {'inpoint': 0.0, 'outpoint': 1.0, 'currentsample':0.5}
        self.center_line_offsets = {'inpoint': 10, 'outpoint': -10, 'currentsample':0}
        
        self.currentTrajectoryParameterDisplays = {}
        self.cursors = {}
        for name in ('inpoint', 'outpoint', 'currentsample'):          
            self.cursors[name] = Image(
                source=global_path_to_kivy_images+ self.cursor_images[name],
                size=(self.base_height,self.base_height),
                pos=(int(self.width * (self.cursor_values[name]-0.5)),0),
                )
            self.add_widget(self.cursors[name])
            self.currentTrajectoryParameterDisplays[name] = Label(id=name+'_value', text='', bcolor=(0,0,1,1))

        for name in self.cursors:
            self.set_value_to_cursor(name, self.cursor_values[name]) #recompute pixel positions
        


    def on_touch_down(self, event):
        distance_closest = 10000000
        self.dragged_cursor = None
        for name in ('inpoint', 'outpoint'):
            inc_x =  event.ox - self.cursors[name].x - self.x - self.cursors[name].width//2
            inc_y =  event.oy - self.cursors[name].y - self.y - self.cursors[name].height//2

            #print("{} dist:{}, {}".format(name, inc_x, inc_y))
            delta = self.base_height//2
            if abs(inc_x) > delta or abs(inc_y) > delta: #not close enough to consider
                continue
            if inc_x < distance_closest:                
                distance_closest = inc_x
                self.dragged_cursor = name

        return (self.dragged_cursor is not None) #eat event if we use it
        #print("touching:{}, {}".format(self.dragged_cursor, distance_closest))

    def on_touch_up(self, event):
        """
        on release, inform the controller the new in/outpoints:
        """
        if self.dragged_cursor == 'inpoint':
            self.playbackController.setInPointofCurrentTrajectory(self.cursor_values['inpoint'])
        elif self.dragged_cursor == 'outpoint':
            self.playbackController.setOutPointofCurrentTrajectory(self.cursor_values['outpoint'])
        eat_event = (self.dragged_cursor is not None) 
        self.dragged_cursor = None
        return eat_event
    
    def on_touch_move(self, event):
        if self.dragged_cursor is None:            
            return False
        proposed_x = self.cursors[self.dragged_cursor].x + event.dx

        left_limit, right_limit = self.get_cursor_limits_pixels(self.dragged_cursor)
        proposed_x = min(max(left_limit, proposed_x), right_limit)

        if self.dragged_cursor == 'inpoint':
            proposed_x = min(proposed_x, self.cursors['outpoint'].x + self.center_line_offsets['outpoint'] - self.center_line_offsets['inpoint'])
        if self.dragged_cursor == 'outpoint':
            proposed_x = max(proposed_x, self.cursors['inpoint'].x  - self.center_line_offsets['outpoint'] + self.center_line_offsets['inpoint'])

        self.cursor_values[self.dragged_cursor] = (proposed_x - left_limit) / (right_limit- left_limit)

        self.cursors[self.dragged_cursor].x = proposed_x
        self.update_parameter_displays(self.dragged_cursor)        
        return True

    def get_cursor_limits_pixels(self, name):
        left_limit = -self.width//2  - self.center_line_offsets[name] + self.playbackbar_hspace
        right_limit = self.width//2  - self.center_line_offsets[name] - self.playbackbar_hspace
        return left_limit, right_limit


    def set_value_to_cursor(self, name, value):
        """
        set a value to a cursor:
        """
        if value is None:
            self.cursors[name].color = (1,1,1,0.1)
        else:
            self.cursors[name].color = (1,1,1,1)
            left_limit, right_limit = self.get_cursor_limits_pixels(name)
            x = int(0.5 + (right_limit-left_limit) * value) + left_limit
            self.cursor_values[name] = value
            self.cursors[name].x = x
        self.update_parameter_displays(name)

    def update_parameter_displays(self, name):
        self.currentTrajectoryParameterDisplays[name].text = "{:3.2f}".format(self.cursor_values[name])

    def _update_pos_size_change(self, instance, value):
        self.bg_rect.size = (instance.size[0], self.base_height)
        self.bg_rect.pos = (0, (instance.height- self.base_height)//2)
        self.bg_bar.size = (instance.size[0]-2*self.playbackbar_hspace,10)
        self.bg_bar.pos = (self.playbackbar_hspace, (instance.height-10)//2)
        for name in self.cursors:
            self.set_value_to_cursor(name, self.cursor_values[name]) #recompute pixel positions


class PlayerButton(ButtonBehavior, Image):  
    pass


class ModePaneLayout(BoxLayout):
    background_color = ListProperty([1,1,0.5,1])
    colormap = {
        PandaRobotMode.kOther: KivyColors.blue_dark,
        PandaRobotMode.kIdle: KivyColors.blue,
        PandaRobotMode.kMove: KivyColors.green,
        PandaRobotMode.kReflex: KivyColors.red,
        PandaRobotMode.kGuiding: KivyColors.red,
        PandaRobotMode.kUserStopped: KivyColors.yellow_dark,
        PandaRobotMode.kAutomaticErrorRecovery: KivyColors.red_dark,
    }

    def __init__(self, **kwargs):
        super(ModePaneLayout, self).__init__(**kwargs)
        with self.canvas.before:
            Color(*self.background_color)
            self.bg_rect = Rectangle(pos=self.pos, size=self.size)
        self.bind(size=self._update_size, pos=self._update_pos)


    def setInfo(self, pdcontrollermode, memode):
        self.background_color = self.colormap[pdcontrollermode]
        self.on_color_change(self)
    

    def on_color_change(self, instance):
        self.canvas.before.clear()
        self.canvas.before.add(Color(*self.background_color))
        self.bg_rect = Rectangle(pos=self.pos, size=self.size)
        self.canvas.before.add(self.bg_rect)

    def _update_pos(self, instance, value):
        self.bg_rect.pos = instance.pos

    def _update_size(self, instance, value):
        self.bg_rect.size = instance.size