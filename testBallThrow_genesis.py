import os, sys, pdb, math, random, time

import libcalab_ogre3d
from libcalab_ogre3d import m, RE, lua
import genesis as gs # used 0.3.4 version
import work.GenesisAddon as GenesisAddon

import numpy as np
import torch, platform



timestep=1/360 
rendering_step=1/30
numBalls=10
skinScale=100


# event handlers
def dtor():
    del mTimeline

def onFrameChanged( iframe):
    sim.stepSimulation(rendering_step)
    skins.setSamePose(sim)

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
floorLoader=m.VRMLloader(floorBox, True)


wrl_file='work/taesooLib/Resource/mesh/smallSphere.wrl';
wrl_file2='work/taesooLib/Resource/mesh/basket.wrl';

assert( os.path.isfile(wrl_file) )
sphereLoader=RE.WRLloader(wrl_file);

assert( os.path.isfile(wrl_file2) )
basketLoader=RE.WRLloader(wrl_file2);
basketLoader.setName("basket")
# adjust basket position
basketLoader.setPosition(m.vector3(0,0.8,0))


simLoaders=[]
simLoaders.append(basketLoader)


for i in range(1,numBalls +1) :
    simLoaders.append(sphereLoader) # character 1~numBalls



simLoaders.append(floorLoader)  # character numBalls+1

swing=RE.WRLloader('work/taesooLib/Resource/mesh/swing.wrl.lua')

simLoaders.append(swing)

numObjects=len(simLoaders)





# compile an xml file from loaders.
sim=GenesisAddon.GenesisSim(simLoaders,'temp_scene.xml', timestep, m.vector3(0,9.8,0), libcalab_ogre3d)

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
