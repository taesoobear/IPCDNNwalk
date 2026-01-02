import os, sys, pdb, math, random


from libcalab_ogre3d import RE,m,lua,control
MujocoSim=control.MujocoSim

import numpy as np
from easydict import EasyDict as edict # pip3 install easydict


# bullet physics engine cannot go lower than 1000hz. 
timestep=1/360 
rendering_step=1/30
numBalls=40
skinScale=100


# event handlers
def dtor():
    del mTimeline

def onFrameChanged( iframe):
    niter=int(rendering_step/timestep)
    for i in range(1, niter +1) :
        sim.stepSimulation()

    skins.setSamePose(sim) # similar to viewer.sync()

def handleRendererEvent(ev, button, x,y):
    return 0


RE.createMainWin(sys.argv)

mTimeline=RE.Timeline('timeline', 10000, rendering_step)

this=m.getPythonWin()
this.create("Check_Button", "draw skeleton", "draw skeleton", 0, 3,0)
this.widget(0).checkButtonValue(0)

this.create("Box", 'help msg', "click the play button (or ctrl+p)",0);
this.create("Value_Slider", "debug", "debug",1);
this.widget(0).sliderRange(-1,1);

this.updateLayout()
this.redraw()

RE.viewpoint().vpos.assign(m.vector3(330.411743, 69.357635, 0.490963))
RE.viewpoint().vat.assign(m.vector3(-0.554537, 108.757057, 0.477768))
RE.viewpoint().update()
RE.viewpoint().TurnRight(math.radians(0))

print("start")

floorBox=m.Geometry()
floorBox.initBox(m.vector3(2, 0.2, 2))


wrl_file='work/taesooLib/Resource/mesh/smallSphere.wrl';
wrl_file2='work/taesooLib/Resource/mesh/basket.wrl';

if os.path.isfile(wrl_file) :
    obj_loader=lua.dynamic_list();
    for i in range(1,numBalls +1) :
        obj_loader[i]=RE.WRLloader(wrl_file);
        obj_loader[i].setName("ball"+str(i))


if os.path.isfile(wrl_file2) :
    basketLoader=RE.WRLloader(wrl_file2);
    basketLoader.setName("basket")

    # adjust basket position
    basketLoader.setPosition(m.vector3(0,0.8,0))

simLoaders=[]
simLoaders.append(basketLoader)


for i in range(1,numBalls +1) :
    simLoaders.append(obj_loader[i]) # character 1~numBalls


floorLoader=m.VRMLloader(floorBox, True) # useFixedJoint==True

simLoaders.append(floorLoader)  # character numBalls+1

swing=RE.WRLloader('work/taesooLib/Resource/mesh/swing.wrl.lua')

simLoaders.append(swing)

numObjects=len(simLoaders)


# compile an xml file from loaders.
sim=MujocoSim(simLoaders,'__temp_ragdoll_scene.xml', timestep)
sim.setGVector(m.vector3(0,9.8,0))

zero=lua.zeros(7) # position(3), orientation(4)
if zero.size()>0 :
    zero.set(3, 1) # m.quater(1,0,0,0)

basketpos=m.vectorn(7);
basketpos.setAllValue(0);
basketpos.set(3,1);

for i in range(1, numBalls +1) :
    zero.setAllValue(0);
    zero.set(3,1);
    zero.set(1, i*0.2+0.3)
    q=m.quater()
    q.identity()
    q.x=random.uniform(0,1)*0.1
    q.y=random.uniform(0,1)*0.1
    q.z=random.uniform(0,1)*0.1
    q.normalize()
    zero.setQuater(3,q)
    sim.setLinkPos(i, zero)
    zero.set(0,random.uniform(0,1)*0.1)
    zero.set(2,random.uniform(0,1)*0.1)
    sim.setLinkVel(i, zero)


swingIndex=sim.numSkeleton()-1
sim.setLinkPos(swingIndex,lua.vec(math.radians(45),math.radians(60)))

skins=RE.createSkin(simLoaders)
skins.setScale(100)
print('ctor finished')

while True:
    if not RE.renderOneFrame(True): break
