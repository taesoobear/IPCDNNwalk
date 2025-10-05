# sample_python aims to allow seamless integration with lua.
# see examples below
import os
import sys
import platform
from libcalab_ogre3d import *

import numpy as np 


# simply forward UI events to lua
def onCallback(mid, userdata):
    lua.onCallback(mid, userdata)

def onFrameChanged(currFrame):
    lua.onFrameChanged(currFrame)

def frameMove(fElapsedTime):
    lua.frameMove(fElapsedTime)
def handleRendererEvent(ev, button, x,y):
    return lua.handleRendererEvent(ev, button, x,y)


this=RE.createMainWin()

lua.dostring('spec_id="walk2-v1"')
#lua.dostring('spec_id="walk2-v1";random_walk=true')
lua.dofile('gym_cdm2/test_cdm_deepmimic.lua')
# calls ctor
lua.F('ctor')
print("ctor finished")

while True:
    if not RE.renderOneFrame(True): break
