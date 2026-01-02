import os, sys, pdb, math, random


from libcalab_ogre3d import RE,m,lua,control
import numpy as np
from easydict import EasyDict as edict # pip3 install easydict


#[[ parameters ]]#

# bullet physics engine cannot go higher than 1/1000. I do not understand why.
timestep=1/360 # 1/80000 works perfectly but too slow. Maybe, we need to employ a game-physics engine like Bullet.
rendering_step=1/30
numBoxes=10
skinScale=100
DEBUG_DRAW=False



def dtor():
    del mTimeline

    dtor_sub()

def dtor_sub():
    global mSkin, mSkin2
    # remove objects that are owned by C++
    del mSkin
    del mSkin2


def _start():
    global smallBoxMesh, floorBox, sim, skins, simLoaders
    dtor_sub()
    print("start")
    smallBoxMesh=m.Geometry()
    smallBoxMesh.initBox(m.vector3(0.2,0.1, 0.2)) # works well.
    #smallBoxMesh.initEllipsoid(m.vector3(0.05,0.05, 0.05)) # balls works well too.
    smallBoxLoader=m.VRMLloader(smallBoxMesh, False) # free root

    floorBox=m.Geometry()
    floorBox.initBox(m.vector3(20, 0.2, 40))
    floorLoader=m.VRMLloader(floorBox, True) # fixed root
    #floorBox.initPlane(20, 40) floorBox.rigidTransform(transf(m.quater(1,0,0,0),m.vector3(0,0,0)))
    #floorBox.initBox(m.vector3(0.2, 0.2, 0.2)) # libccd works only for small boxes or plane. do not use large boxes when using libccd.

    simLoaders=[]

    for i in range(numBoxes ) :
        simLoaders.append(smallBoxLoader)  # character 0 ~ numBoxes-1

    simLoaders.append(floorLoader)  # character numBoxes

    sim=m.DynamicsSimulator_Trbdl_impulse("gjk") 
    sim.setParam_restitution_MA(0,0.001)
    sim.setGVector(m.vector3(0,9.8,0))

    for i in range(len(simLoaders)):
        sim.registerCharacter(simLoaders[i])

    def registerPair(iloader1, iloader2):
        param=lua.vec([0.5,0.5, 10000,1000]) # unused
        loader1=sim.skeleton(iloader1)
        loader2=sim.skeleton(iloader2)
        bone_i=loader1.VRMLbone(1)
        bone_j=loader2.VRMLbone(1)
        sim.registerCollisionCheckPair(iloader1,bone_i.name(), iloader2, bone_j.name(), param)


    for i in range(0, numBoxes +1) :
        for j in range(i+1, numBoxes +1) :
            registerPair(i,j)

    # call init after all collisionCheckPairs are registered
    sim.init(timestep)

    zero=lua.zeros(7) # position(3), orientation(4)
    if zero.size()>0 :
        zero.set(3, 1) # quater(1,0,0,0)

    for i in range(1, numBoxes +1) :
        zero.set(0,random.uniform(0,1)*0.10)
        zero.set(1, i*0.2)
        zero.set(2,random.uniform(0,1)*0.05)
        q=m.quater()
        q.identity()
        q.x=random.uniform(0,1)*0.0
        q.y=random.uniform(0,1)*0.0
        q.z=random.uniform(0,1)*0.0
        q.normalize()
        zero.setQuater(3,q)
        sim.setLinkData(i-1, m.Physics.JOINT_VALUE, zero)
        sim.setLinkData(i-1, m.Physics.JOINT_VELOCITY, zero)


    sim.initSimulation()

    skins=RE.createSkin(simLoaders)
    skins.setScale(100)
    if DEBUG_DRAW:
        for i in range(len(skins) ) :
            skins[i].setMaterial('lightgrey_transparent')



def onCallback(w, userData):
    if w.id()=="Start" :
        _start()





def onFrameChanged(iframe):
    global smallBoxMesh, floorBox, sim, skins
    niter=int(rendering_step/timestep)
    for i in range(1, niter +1) :
        sim.stepSimulation()

        if DEBUG_DRAW :
            sim.drawDebugInformation()

    for i, skin in enumerate(skins):
        skin.setPoseDOF(sim.getLastSimulatedPose(i))



RE.createMainWin(sys.argv)
this=m.getPythonWin()
mSkin=None
mSkin2=None

mTimeline=RE.Timeline('timeline', 10000, rendering_step)


this.create("Check_Button", "draw skeleton", "draw skeleton", 0, 3,0)
this.widget(0).checkButtonValue(0)

this.updateLayout()
this.redraw()

RE.viewpoint().vpos.assign(m.vector3(330.411743, 69.357635, 0.490963))
RE.viewpoint().vat.assign(m.vector3(-0.554537, 108.757057, 0.477768))
RE.viewpoint().update()
RE.viewpoint().TurnRight(math.radians(0))
_start()
print('ctor finished')

while True:
    if not RE.renderOneFrame(True): break

