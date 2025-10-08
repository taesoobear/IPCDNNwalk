# sample_python aims to allow seamless integration with lua.
# see examples below
import os
import sys
import platform
import pdb # use pdb.set_trace() for debugging
sys.path.append(os.getcwd())
import libmainlib as m   
import luamodule as lua  # see luamodule.py
import rendermodule as RE
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
    scriptFile="../../taesooLib/Samples/scripts/RigidBodyWin/GUI_tools/WRLviewer.lua"
    option=''
    if len(sys.argv)==1:
        print("Examples of usage: ")
        print(" python3 runLua.py ../../taesooLib/Samples/scripts/RigidBodyWin/GUI_tools/WRLviewer.lua")
        print("\nOther examples: ")
        print("1: using a seperate rendering window. (Higher-quality rendering is possible. Settings can be adjusted by deleting 'work/Ogre.cfg')")
        print(" python3 runLua.py --sep ../../taesooLib/Samples/scripts/RigidBodyWin/GUI_tools/WRLviewer.lua")
        print("2: using a different workpath")
        print(" python3 runLua.py --work ../../taesooLib/work ../../taesooLib/Samples/scripts/RigidBodyWin/GUI_tools/WRLviewer.lua")
        return
    elif len(sys.argv)==2:
        scriptFile=sys.argv[1]
    elif len(sys.argv)==3:
        option=sys.argv[1]
        scriptFile=sys.argv[2]
    elif len(sys.argv)==4:
        option=sys.argv[1]
        workPath=sys.argv[2]
        os.chdir(workPath)
        scriptFile=sys.argv[3]
    uiscale=1
    if m.getOgreVersionMinor()<=3 and platform.system()!='Darwin':
        try:
            # try using ogreConfig_personal.txt or ogreConfig*.txt
            if not m.rendererValid():
                renderer=m._createRenderer(0, 0); 
                RENDERER_WIDTH=renderer.getConfig("renderer_width");
                RENDERER_HEIGHT=renderer.getConfig("renderer_height");
                WIDTH=RENDERER_WIDTH+renderer.getConfig("right_panel_width");
                HEIGHT=RENDERER_HEIGHT+renderer.getConfig("right_panel_width");
                UIsf= renderer.getConfigFloat("UI_scale_factor");
                m._createMainWin(WIDTH, HEIGHT, RENDERER_WIDTH, RENDERER_HEIGHT, UIsf, renderer);

        except:
            rw=1920
            rh=1080
            uiscale=1.5
            m.createMainWin(int(rw+180*uiscale),int(rh+100*uiscale), int(rw), int(rh),uiscale)
    elif option=='--sep':
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
    m.getPythonWin().dostring('ctor()')

    print("ctor finished")


    m.startMainLoop() # this finishes when program finishes

if __name__=="__main__":
    main()
