import os,sys, pdb, math, random
import numpy as np
from libcalab_ogre3d import RE,m, lua,control



def frameMove(fElapsedTime):
    RE.output('vpos', RE.viewpoint().vpos)
    return 1;



def handleRendererEvent(ev, button, x, y):
    #print(ev, x,y,button)
    if ev=="PUSH" :
        return 1
    elif ev=="DRAG" :
        return 1
    elif ev=="RELEASE" :
        return 1
    elif ev=="MOVE" :
        out=m.vector3()
        ray=m.Ray()
        m.FltkRenderer().screenToWorldRay(x, y,ray)
        ray.scale(0.01) # to meter
        normal=m.vector3()
        out=g_terrain.pickTerrain(ray, normal)

        RE.draw('SphereM', out, "cursor_on_terrain", "red",0.2)
        return 1

    return 0

this=RE.createMainWin(sys.argv)
this.addText('shift-drag on terrain', 'Usage. shift-drag on terrain')
s=18
h=1

# VRMLloader assumes 256x256 for raw images.
loader=RE.WRLloader("work/taesooLib/Resource/terrain/height_field.png", 40, 40, 3, 1,1)
loader.setPosition(m.vector3(-5, 0.1,-5))
g_terrain=loader

skin=RE.createSkin(loader)
skin.setScale(100)

this.updateLayout()

RE.viewpoint().setFOVy(45.000000)
RE.viewpoint().setZoom(1.000000)
RE.viewpoint().vpos.set(-293.561471, 1173.843322, 2852.874747)
RE.viewpoint().vat.set(-290.435846, 183.892835, -117.509685)
RE.viewpoint().update()
m.startMainLoop() # this finishes when program finishes

