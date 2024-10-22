#!/usr/bin/env python
# -*- coding: utf-8 -*-
#Copyright 2018 Raphael Deimel
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

This script takes the description of a state graph with associated ProMP trajectories and controllers, and instantiates them

"""	

import numpy as _np
import os
import sys
import yaml


import phasestatemachine
import promp
import phastapromep



if len(sys.argv) != 2:
    raise SystemExit("Please provide the session configuration file as the only argument!")
with open(sys.argv[1]) as f:
    config = yaml.load(f)

DefinitionsDirectory = os.path.join(os.path.expanduser(config['data directory']), config['output directory'])

phasta = phastapromep.createPhaseStateMachine(DefinitionsDirectory)

prompmixer = phastapromep.createMixer(DefinitionsDirectory)
    

