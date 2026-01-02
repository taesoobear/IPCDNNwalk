# this file contains a line-by-line python port of /Users/taesookwon/d/taesooLib/Resource/scripts/RigidBodyWin/GUI_tools/SceneEditor.lua
import os,sys, pdb, math, random, copy 
from libcalab_ogre3d import RE, m, lua, control # see rendermodule.py
import numpy as np


sceneGraph=None

def handleRendererEvent(ev,button,x,y):
    return sceneGraph.handleRendererEvent(ev, button, x, y)

def rotateY(scene_item, elapsedTime):
    #scene_item.orientation=scene_item.orientation*m.quater(1*elapsedTime, m.vector3(0,1,0))
    # without bbox rotation and events
    scene_item.localOrientation=m.quater(1*elapsedTime, m.vector3(0,1,0))*scene_item.localOrientation

def sceneEventFunction(scene_item, ev):
    print(scene_item.nodeId, ev)

def onCallback(w,userData):

    global sceneGraph

    if (w.id()=="global operations") :
        id= w.menuText()

        if (id=="create arrow") :
            sceneGraph.addEntity('arrow2.mesh', localScale=m.vector3(2,2,2), localPosition=m.vector3(0, 50, 0))
        elif (id=="create smiley") :
            sceneGraph.addEntity('Smiley.mesh', localScale=40).material='green'
        elif (id=="create rotating smiley") :
            scene_item=sceneGraph.addEntity('Smiley.mesh', localScale=40, localOrientation=m.quater(math.radians(90),m.vector3(1,0,0)))
            scene_item.material='red'
            scene_item.handleFrameMove=rotateY
            scene_item.eventFunction=sceneEventFunction

    elif sceneGraph.onCallback(w, userData):
        pass
    else:
        pass




this=RE.createMainWin(sys.argv)

this.addText("try shift-L-drag \nor shift-R-drag")

this.create("Choice", "global operations","global operations")
this.widget(0).menuItems(["global operations", 'create arrow', 'create smiley', 'create rotating smiley'])
this.widget(0).menuValue(0)


sceneGraph=RE.SceneGraph()
sceneGraph.createUI(this)

sceneGraph.addEntity('penguin.mesh')

this.updateLayout()
print('ctor finished')
m.startMainLoop() # this finishes when program finishes
