# this file contains a line-by-line python port of testDebugDraw.lua (taesooLib/Samples/classification/lua/)
import os
import sys
import pdb # use pdb.set_trace() for debugging
#import code # or use code.interact(local=dict(globals(), **locals())) for debugging. see below.
import math
import random


from libcalab_ogre3d import *
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
    global smallBoxMesh, floorBox, sim, mBoxSkins
    dtor_sub()
    print("start")
    smallBoxMesh=m.Geometry()
    smallBoxMesh.initBox(m.vector3(0.2,0.1, 0.2)) # works well.
    #smallBoxMesh.initEllipsoid(m.vector3(0.05,0.05, 0.05)) # balls works well too.
    #smallBoxMesh.initCylinder(0.1,0.1, 20) # does not work with libccd. strange... 

    floorBox=m.Geometry()
    floorBox.initBox(m.vector3(20, 0.2, 40))
    #floorBox.initPlane(20, 40) floorBox.rigidTransform(transf(m.quater(1,0,0,0),m.vector3(0,0,0)))
    #floorBox.initBox(m.vector3(0.2, 0.2, 0.2)) # libccd works only for small boxes or plane. do not use large boxes when using libccd.

    sim=m.DynamicsSimulator_Trbdl_impulse("gjk") 
    sim.setParam_restitution_MA(0,0.001)
    sim.setGVector(m.vector3(0,9.8,0))

    for i in range(1,numBoxes +1) :
        sim.createFreeBody(smallBoxMesh)

    sim.createObstacle(floorBox)

    def registerPair(iloader1, iloader2):
        param=m.vectorn()
        param.assign([0.5,0.5, 10000,1000])
        loader1=sim.skeleton(iloader1)
        loader2=sim.skeleton(iloader2)
        bone_i=loader1.VRMLbone(1)
        bone_j=loader2.VRMLbone(1)
        sim.registerCollisionCheckPair(loader1.name(),bone_i.name(), loader2.name(), bone_j.name(), param)


    if True :
        for i in range(0, numBoxes +1) :
            for j in range(i+1, numBoxes +1) :
                registerPair(i,j)


    else:
        for i in range(0, numBoxes +1) :
            for j in range(0, numBoxes +1) :
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

    numSkins=numBoxes+1
    mBoxSkins=lua.dynamic_list() # use lua.dynamic_list() instead for 0-indexed array.
    for i in range(0, numSkins ) :
        loader=sim.skeleton(i)
        mBoxSkins[i]=RE.createVRMLskin(loader, False)
        s=skinScale
        mBoxSkins[i].scale(s,s,s)
        mBoxSkins[i].setPose(sim,i)
        if DEBUG_DRAW :
            mBoxSkins[i].setMaterial('lightgrey_transparent')





def onCallback(w, userData):
    if w.id()=="Start" :
        _start()





def onFrameChanged(iframe):
    global smallBoxMesh, floorBox, sim, mBoxSkins
    numSkins=numBoxes+1
    niter=int(rendering_step/timestep)
    for i in range(1, niter +1) :
        sim.stepSimulation()

        if DEBUG_DRAW :
            sim.drawDebugInformation()

        for j in range(0,numSkins ) :
            mBoxSkins[j].setPose(sim,j)


    #sim.drawDebugInformation()
#main


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

