#!/usr/bin/env python
# -*- coding: utf-8 -*-
#Copyright 2020 Raphael Deimel
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

This script takes a recorded session of demonstrated motions, segments them according to a labeled cutlist, learns promps for each labeled transition and constructs a state graph for sequencing the promps.

HDf5 can be created with teaching_app.py

"""

# #Comment in for Debugging within Visual Studio Code
# import ptvsd; ptvsd.enable_attach(address = ('localhost', 5678)); ptvsd.wait_for_attach()

import os
import sys
import yaml

import matplotlib
import phastapromp

    
if len(sys.argv) == 1:
    filename = 'session.yaml'
elif len(sys.argv) != 2:
    raise SystemExit("Please provide only one argument!")
else:
    filename = sys.argv[1]
with open(filename) as f:
    config = yaml.load(f, loader=yaml.SafeLoader)

import pandas

store = pandas.HDFStore(config['hdf5 filename'])

graphconfig, promps = phastapromp.learnGraphFromDemonstration(config, observationsHDFStore=store)


#save all data neeeded to instantiate the learned graph into a single directy
dataDirectory = os.path.expanduser(config['data directory'])
DefinitionsDirectory = os.path.join(dataDirectory, config['output directory'])


print("Saving learnt graph and movement primitives to {0}".format(os.path.abspath(DefinitionsDirectory)))
phastapromp.saveGraphDefinitionsToDirectory(graphconfig, promps, DefinitionsDirectory)
# do some illustrations for checking/publication
IllustrationsDirectory = os.path.join(dataDirectory,  config['output illustrations directory'])
print("Saving illustrations to {0}".format(os.path.abspath(IllustrationsDirectory)))
phastapromp.illustrateGraph(graphconfig, promps, IllustrationsDirectory)





