import os, sys, pdb, math, random


from libcalab_ogre3d import m, RE, lua, control

import numpy as np
from easydict import EasyDict as edict # pip3 install easydict


timestep=1/360 
rendering_step=1/30
numBalls=40
skinScale=100


activeBalls=[]

# event handlers
def dtor():
    del mTimeline

def onFrameChanged( iframe):
    global skins
    niter=int(rendering_step/timestep)
    for i in range(1, niter +1) :
        sim.stepSimulation()

    for i, skin in enumerate(skins.skins):
        skin.setPose(sim, i)

def handleRendererEvent(ev, button, x,y):
    return 0

def onCallback(w, id):
    if w.id()=='throw a ball':
        for i in range(1, numBalls+1): # ball index starts at 1
            if i not in activeBalls:
                activeBalls.append(i) 
                zero=lua.zeros(7)
                zero.set(3, 1)
                zero.set(1, 2)
                zero.set(2, 2+random.uniform(-0.2, 0.4))

                sim.setLinkData(i, m.Physics.JOINT_VALUE, zero)
                zero.setVec3(0, m.vector3(0,0,-5))
                sim.setLinkData(i, m.Physics.JOINT_VELOCITY, zero)
                sim.initSimulation()
                return
        print('no balls left')


RE.createMainWin(sys.argv)

mTimeline=RE.Timeline('timeline', 10000, rendering_step)

m.motionPanel().motionWin().playFrom(0)
this=m.getPythonWin()
this.create("Check_Button", "draw skeleton", "draw skeleton", 0, 3,0)
this.widget(0).checkButtonValue(0)

this.create("Box", 'help msg', "click the play button (or ctrl+p)",0);
this.create("Value_Slider", "debug", "debug",1);
this.widget(0).sliderRange(-1,1);

this.addButton("throw a ball")
this.widget(0).buttonShortcut('FL_CTRL+t')

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

sim=m.DynamicsSimulator_TRL_LCP("libccd") #sim.setParam_R_B_MA(0, 0, 0.05); # for large timestep (ARMATURE) , R adjusts rebounds from the ground. B is not implemented yet.
#sim=m.DynamicsSimulator_Trbdl_LCP("libccd") #sim.setParam_R_B_MA(0, 0, 0.05); # for large timestep (ARMATURE) , R adjusts rebounds from the ground. B is not implemented yet.
#sim=m.DynamicsSimulator_TRL_softbody() #sim.setParam_R_B_MA(0, 0, 0.05); # for large timestep (ARMATURE) , R adjusts rebounds from the ground. B is not implemented yet.
sim.setGVector(m.vector3(0,9.8,0))

for  loader in simLoaders:
    sim.registerCharacter(loader) 

numObjects=len(simLoaders)
for i in range(0, numObjects) :
    for j in range(i+1, numObjects) :
        control.registerContactPairAll(None, sim.skeleton(i),sim.skeleton(j), sim)

sim.init(timestep)

zero=lua.zeros(7) # position(3), orientation(4)
if zero.size()>0 :
    zero.set(3, 1) # m.quater(1,0,0,0)

basketpos=m.vectorn(7);
basketpos.setAllValue(0);
basketpos.set(3,1);
for i in range(1, numBalls +1) :
    zero.setAllValue(0);
    zero.set(3,1); # quternion w==1 (identity)
    zero.set(1, -100)
    sim.setLinkData(i, m.Physics.JOINT_VALUE, zero)
    sim.setLinkData(i, m.Physics.JOINT_VELOCITY, zero)

swingIndex=sim.numSkeleton()-1
sim.setQ(swingIndex,lua.vec(math.radians(45),math.radians(60)))
sim.initSimulation()

skins=RE.createSkin(simLoaders)
skins.setScale(100)
print('ctor finished')

while True:
    if not RE.renderOneFrame(True): break
