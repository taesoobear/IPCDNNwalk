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


# simply forward UI events to lua
def onCallback(mid, userdata):
    lua.onCallback(mid, userdata)

def onFrameChanged(currFrame):
    lua.onFrameChanged(currFrame)

def frameMove(fElapsedTime):
    lua.frameMove(fElapsedTime)
def handleRendererEvent(ev, button, x,y):
    return lua.handleRendererEvent(ev, button, x,y)

def main():
    scriptFile="../lua/testPoses.lua"
    option=''
    if len(sys.argv)==2:
        scriptFile=sys.argv[1]
    elif len(sys.argv)==3:
        option=sys.argv[1]
        scriptFile=sys.argv[2]
    uiscale=1
    if option=='--sep':
        if platform.system()=='Darwin':
            m.createMainWin(int((10+220)*uiscale),int((400+100)*uiscale), int(10*uiscale), int(400*uiscale),uiscale, "../Resource/ogreconfig_linux_sepwin.txt", "plugins_mac.cfg", "ogre_mac.cfg")
        else:
            m.createMainWin(int((10+220)*uiscale),int((400+100)*uiscale), int(10*uiscale), int(400*uiscale),uiscale, "../Resource/ogreconfig_linux_sepwin.txt", "plugins_linux.cfg", "ogre_linux.cfg")
    else:
        m.createMainWin(int((1024+180)*uiscale),int((600+100)*uiscale), int(1024*uiscale), int(600*uiscale),uiscale)
    m.showMainWin()
    if scriptFile[0:1]!='.':
        scriptFile=os.path.relpath(scriptFile, os.getcwd())
    if scriptFile[-3:]!='lua':
        scriptFile=scriptFile+'.lua'
    print('loading', scriptFile)

    #m.getPythonWin().loadScript(scriptFile)
    # you can use the following lines instead of the above line (loadScript).
    m.getPythonWin().loadEmptyScript()
    if option[0:10]=='--dostring':
        m.getPythonWin().dostring(option[11:])
    m.getPythonWin().dofile(scriptFile)
    #m.getPythonWin().dostring('ctor()')
    m.getPythonWin().getglobal('ctor')
    m.getPythonWin().call(0,0)

    print("ctor finished")

    # test lua-python interfacing
    lua.out(3)
    lua.out('asdf')
    lua.out(m.vector3(3,4,5))
    a=m.vectorn()
    a.assign([1,2,3,4,5])
    lua.out(a)
    b=m.intvectorn()
    b.assign([1,2,3,4,5])
    lua.out(b)
    c=np.asmatrix('1 0 0; 0 1 0; 0 0 1')
    print(c)

    m.startMainLoop() # this finishes when program finishes

if __name__=="__main__":
    main()
