# this file contains a line-by-line python port of RagdollFallLCP_simple_v2.lua (taesooLib/Samples/classification/lua/)
import os
import sys
import pdb # use pdb.set_trace() for debugging
#import code # or use code.interact(local=dict(globals(), **locals())) for debugging. see below.
import math
import random
import copy

from libcalab_ogre3d import *
import numpy as np
from easydict import EasyDict as edict # pip3 install easydict


model=edict() # use lua.dynamic_list() instead for 0-indexed array.
#model.file_name= "../Resource/motion/locomotion_hyunwoo/hyunwoo_lowdof_T_boxfoot.wrl" 
model.file_name= "work/taesooLib/Resource/motion/MOB1/hanyang_lowdof_T.wrl" 
#model.mot_file= "../Resource/motion/locomotion_hyunwoo/hyunwoo_lowdof_T_wd2_all.dof"
#model.mot_file= "../Resource/motion/locomotion_hyunwoo/hyunwoo_lowdof_T_MOB1_Run_F_Jump.dof" model.initialHeight=0.13 # meters
model.mot_file= None 
model.initialHeight=1.3 # meters
model.k_p_PD=300 # Nm/rad
model.k_d_PD=10 #  Nms/rad. 
#model.mot_file= "../Resource/motion/locomotion_hyunwoo/hyunwoo_lowdof_T_locomotion_hl.dof"
model.frame_rate=30 # mocap frame rate
model.start=0
model.timestep=1/240
model.rendering_step=1/30

# convert posedof to sphericalQ and sphericalDQ format. 

def onFrameChanged( iframe):
    global mLoader, mRagdoll
    this=m.getPythonWin()
    if mLoader and this.findWidget("simulation").checkButtonValue() :
        niter=math.floor(model.rendering_step/model.timestep+0.5)
        mRagdoll.frameMove(niter)




def dtor():
    global mSkin, mSkin2, mTimeline
    # remove objects that are owned by C++
    mSkin=None
    mSkin2=None
    mTimeline=None
 


def _start(this):
    global mLoader, mFloor, simulator, mRagdoll, mSkin, mSkin2, container, mMotionDOF, mTimeline
    dtor()
    mTimeline=RE.Timeline("Timeline", 1000000, 1/30)
    RE.motionPanel().motionWin().playFrom(0)
    print("start")
    mLoader=RE.WRLloader(model.file_name)

    mLoader.printHierarchy()
    if model.mot_file:
        container=m.MotionDOFcontainer(mLoader.dofInfo, model.mot_file)
        mMotionDOF=container.mot
    else:
        mMotionDOF=None

    mFloor=RE.WRLloader("work/taesooLib/Resource/mesh/floor_y.wrl")

    drawSkeleton=this.findWidget("draw skeleton").checkButtonValue()

    simulator= m.DynamicsSimulator_Trbdl_LCP("libccd")  # now works well.
    #simulator= m.DynamicsSimulator_TRL_LCP("libccd")  #  works after reducing k_d and timestep
    #self.simulator.setParam_Epsilon_Kappa(0.1, 0.05) # for soft contact
    simulator.setParam_R_B_MA(0, 0, 0.05); # for large timestep (ARMATURE) , R adjusts rebounds from the ground. B is not implemented yet.

    simulatorParam=edict()
    simulatorParam.timestep=model.timestep
    simulatorParam.debugContactParam=[10, 0, 0.01, 0, 0] # size, tx, ty, tz, tfront
    mRagdoll= RagdollSim(mLoader, drawSkeleton, mMotionDOF, simulator, simulatorParam)
    mRagdoll.drawDebugInformation=False

    mSkin2=RE.createVRMLskin(mFloor, False)
    mSkin2.scale(100,100,100)


def onCallback(w, userData):
    if w.id()=="Start" :
        _start(m.getPythonWin())



class RagdollSim:
    def __init__(self, loader, drawSkeleton, motdof, simulator, simulatorParam):
        if drawSkeleton==None:
            drawSkeleton = True 
        self.simulator=simulator
        self.skin=RE.createVRMLskin(loader, drawSkeleton)
        self.skin.setThickness(0.03)
        self.skin.scale(100,100,100)

        self.simulator.registerCharacter(loader)
        floor=mFloor or RE.WRLloader("../Resource/mesh/floor_y.wrl")
        self.simulator.registerCharacter(floor)

        param=m.vectorn()
        param.assign([0.5,0.5, 30000, 3000]) # unused
        for i in range(1,loader.numBone()-1 +1) :
            bone_i=loader.VRMLbone(i)
            self.simulator.registerCollisionCheckPair(loader.name(),bone_i.name(), floor.name(), floor.bone(1).name(), param)


        self.simulator.init(simulatorParam.timestep)

        kp=lua.zeros(loader.dofInfo.numDOF())
        kd=lua.zeros(loader.dofInfo.numDOF())
        kp.slice(7,0).setAllValue(model.k_p_PD)
        kd.slice(7,0).setAllValue(model.k_d_PD)
        self.simulator.setStablePDparam_dof(0, kp, kd)
        # adjust initial positions

        self.motionDOF=motdof
        self.simulationParam=simulatorParam

        if self.motionDOF :
            for i in range(0, self.motionDOF.numFrames()-1 +1) :
                self.motionDOF.row(i).set(1,self.motionDOF.row(i).get(1)+(model.initialHeight or 0) )


        else:
            motdofc=m.MotionDOFcontainer(loader.dofInfo)
            motdofc.resize(10)
            for i in range(0, 9 +1) :
                motdofc.mot(i).setAllValue(0)
                motdofc.mot(i).set(3, 1) # assuming quaternion (free root joint)

            self.motionDOF=motdofc.mot
            for i in range(0, self.motionDOF.numFrames()-1 +1) :
                self.motionDOF.row(i).set(1,self.motionDOF.row(i).get(1)+(model.initialHeight or 0) )

        self.DMotionDOF=calcDerivative(self.motionDOF)
        self.DDMotionDOF=self.DMotionDOF.derivative(120)

        if self.motionDOF :
            model.start=min(model.start, self.motionDOF.numFrames()-1)
            initialState=m.vectorn()
            initialState.assign(self.motionDOF.row(model.start))

            print("initialState=",initialState)
            self.simulator.setLinkData(0, m.Physics.JOINT_VALUE, initialState)

            if self.DMotionDOF :
                initialVel=self.DMotionDOF.row(model.start).copy()
                self.simulator.setLinkData(0, m.Physics.JOINT_VELOCITY, initialVel)

            self.simulator.initSimulation()
        else:
            assert(False)

        self.referenceFrame=model.start

        #    debug.debug()
        self.skin.setPose(self.simulator,0)

        self.skin.setMaterial("lightgrey_transparent")

        #self.simulator.setGVector(m.vector3(0,0,9.8))
        self.simulator.setGVector(m.vector3(0,9.8,0))
        self.simulator.initSimulation()
        self.loader=loader
        self.floor=floor # have to be a member to prevent garbage collection


    def dtor(self ):
        # remove objects that are owned by C++
        if self.skin:
            self.skin=None

        self.simulator=None


    def frameMove(self, niter):
        global model

        for iter in range(1,niter +1) :

            refFrame=self.referenceFrame+model.timestep*model.frame_rate
            print(refFrame)
            if refFrame>self.motionDOF.numFrames()-1 :
                refFrame=0
            self.referenceFrame=refFrame
            pose_d=self.motionDOF.row(math.floor(refFrame))
            dpose_d=self.DMotionDOF.row(math.floor(refFrame))
            spd_tau=m.vectorn()
            simulator.calculateStablePDForces_dof(0, pose_d,dpose_d, spd_tau)
            simulator.setTau(0, spd_tau)
            simulator.stepSimulation()

        self.skin.setPose(self.simulator,  0)                


        if self.drawDebugInformation:
            forces=m.vector3N()
            torques=m.vector3N()
            for i in range(1, self.loader.numBone()-1 +1) :
                bone=self.loader.VRMLbone(i)

                force=self.simulator.getCOMbasedContactForce(0, i)
                if force.F().length()>0 :
                    pos=self.simulator.getWorldState(0).globalFrame(i)*bone.localCOM()
                    forces.pushBack(pos*100)
                    forces.pushBack(pos*100+force.F())
                    torques.pushBack(pos*100)
                    torques.pushBack(pos*100+force.M())


            RE.namedDraw('Traj', forces.matView(), 'contactforces', 'solidred', 0, 'LineList' )
            RE.namedDraw('Traj', torques.matView(), 'contacttorques', 'solidblue', 0, 'LineList' ) 



def calcDerivative(motionDOF):
    dmotionDOF=m.matrixn()

    dmotionDOF.setSize(motionDOF.numFrames(), motionDOF.numDOF())

    for i in range(1, motionDOF.rows()-2 +1) :
        calcDerivative_row(i,dmotionDOF, motionDOF)


    # fill in empty rows
    dmotionDOF.row(0).assign(dmotionDOF.row(1))
    dmotionDOF.row(dmotionDOF.rows()-1).assign(dmotionDOF.row(dmotionDOF.rows()-2))
    return dmotionDOF


def calcDerivative_row(i, dmotionDOF, motionDOF):
    global model
    dmotionDOF_i=dmotionDOF.row(i);
    dmotionDOF_i.sub(motionDOF.row(i+1), motionDOF.row(i)) # forward difference

    frameRate=120
    if model :
        frameRate=model.frame_rate 
    dmotionDOF_i.rmult(frameRate)

    assert(motionDOF.dofInfo.numSphericalJoint()==1) 
    # otherwise following code is incorrect
    T=motionDOF.row(i).toTransf(0)
    V=T.twist( motionDOF.row(i+1).toTransf(0), 1/frameRate)
    dmotionDOF_i.setVec3(0, V.V())
    dmotionDOF_i.setVec3(4, V.W())



# main
RE.createMainWin(sys.argv)
mSkin=None
mSkin2=None
mTimeline=None
this=m.getPythonWin()
    
#    this.create("Button", "Start", "Start")
#    this.widget(0).buttonShortcut("FL_ALT+s")

this.create("Check_Button", "simulation", "simulation", 0, 2,0)
this.widget(0).checkButtonValue(1) # 1 for imediate start
this.widget(0).buttonShortcut("FL_ALT+s")

this.create("Button", "single step", "single step", 2, 3,0)

this.create("Check_Button", "draw skeleton", "draw skeleton", 0, 3,0)
this.widget(0).checkButtonValue(0)


this.updateLayout()
this.redraw()

RE.viewpoint().vpos.assign(m.vector3(330.411743, 69.357635, 0.490963))
RE.viewpoint().vat.assign(m.vector3(-0.554537, 108.757057, 0.477768))
RE.viewpoint().update()
RE.viewpoint().TurnRight(math.radians(0))
_start(this)


while True:
    if not RE.renderOneFrame(True): break

