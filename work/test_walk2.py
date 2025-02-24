# sample_python aims to allow seamless integration with lua.
# see examples below
import os
import sys
import platform
import pdb # use pdb.set_trace() for debugging
sys.path.append(os.getcwd())
import libmainlib as m   
import luamodule as lua  # see luamodule.py
import numpy as np 


import torch
from torch.autograd import Variable
import torch.nn.functional as F
import torch.utils.data as Data

# simply forward events to lua
def onCallback(mid, userdata):
    lua.onCallback(mid, userdata)

def onFrameChanged(currFrame):
    lua.onFrameChanged(currFrame)

def frameMove(fElapsedTime):
    lua.frameMove(fElapsedTime)
def handleRendererEvent(ev, button, x,y):
    print(ev, button)
    return lua.handleRendererEvent(ev, button, x,y)

_globals=[None]*5
def mapping(vfeature, vdata):
    global _globals
    x=vfeature.ref()[:] # copy
    x=torch.tensor(x.astype(np.float32))
    assert(vdata.size()==0)

    net=_globals[0]
    pred=net(x)
    pred_np=pred.detach().numpy()
    vdata.setSize(pred_np.shape[0])
    vdata.ref()[:]=pred_np
def dotmapping(vfeature, vdotdata):
    global _globals
    x=vfeature.ref()[:] # copy
    x=torch.tensor(x.astype(np.float32))
    assert(vdotdata.size()==0)

    net=_globals[1]
    pred=net(x)
    pred_np=pred.detach().numpy()
    vdotdata.setSize(pred_np.shape[0])
    vdotdata.ref()[:]=pred_np

def loadNN( fn, useDot):
    global _globals
    #matfeature=lua.getglobal_mat('g_dataset', 1)
    #matdataall=lua.getglobal_mat('g_dataset', 2)

    net=torch.load(fn, weights_only=False)
    _globals[0]=net
    if useDot:
        net2=torch.load('dot'+fn, weights_only=False)
        _globals[1]=net2

# note that the above functions are called from lua using a different python module (test_walk2 !=  __main__)
# so _globals will also be different between main() and loadNN() 
# to avoid this issue, you can put global variables in a seperate module (for example, test_walk2_globals.py)
# see test_globals.py also.

def main():
    #scriptFile= 'gym_walk/testPendulumOnlineControlWalkObstacle.lua'
    scriptFile= 'gym_walk/testPendulumOnlineControlWalkEnv.lua'
    #scriptFile= 'gym_walk/testPendulumOnlineControlWalk.lua'
    option=''
    if len(sys.argv)==2:
        scriptFile=sys.argv[1]
    elif len(sys.argv)==3:
        option=sys.argv[1]
        scriptFile=sys.argv[2]
    uiscale=1
    if m.getOgreVersionMinor()==2 and platform.system()!='Darwin':
        rw=1920
        rh=1080
        uiscale=1.5
        m.createMainWin(int(rw+180*uiscale),int(rh+100*uiscale), int(rw), int(rh),uiscale)
    else:
        m.createMainWin(int((1024+180)*uiscale),int((600+100)*uiscale), int(1024*uiscale), int(600*uiscale),uiscale)
    m.showMainWin()
    m.getPythonWin().loadEmptyScript()
    m.getPythonWin().dofile(scriptFile)
    lua.dostring('useNN=true ctor()')

    print("ctor finished")


    m.startMainLoop() # this finishes when program finishes

if __name__=="__main__":
    main()
