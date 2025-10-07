# this file contains a line-by-line python port of showLafan1Motions_skinned.lua 
import os
import sys
import pdb # use pdb.set_trace() for debugging
#import code # or use code.interact(local=dict(globals(), **locals())) for debugging. see below.
import math
import random
from pathlib import Path
import glob

if not os.path.exists('work'):
    print('symlink not found: create the work folder first')
    exit();

import work.libmainlib as m   
import work.luamodule as lua  # see luamodule.py
import work.rendermodule as RE # see rendermodule.py
import work.controlmodule as control
import numpy as np
import copy
from easydict import EasyDict as edict # pip3 install easydict

scriptPath=os.getcwd()

#lua require should be in the ctor function

def dtor():
    global mTimeline, mTargetLoader, mLafanLoader, toOriginalPose, mTargetSkin, mSourceSkin
    # cleanup lua instances 
    mTimeline=None
    mTargetSkin=None
    mSourceSkin=None



def onCallback(w, userData):
    global mTargetLoader, mTargetSkin,mTimeline,mTargetMotion
    this=m.getPythonWin()
    if w.id()=="attach camera" :
        if w.checkButtonValue() :
            mTimeline.attachCameraToMotion(mTargetLoader, mTargetMotion)
        else:
            mTimeline.detachCamera()

    elif w.id()=="2X playback" :
        attachTimer(w.checkButtonValue())

    return 0

def attachTimer(play_x2):
    global mTargetMotion,info
    if play_x2 :
        mTimeline.attachTimer(0.5/info.frameRate, mTargetMotion.numFrames())
    else:
        mTimeline.attachTimer(1/mTargetMotion.frameRate(), mTargetMotion.numFrames())



def onFrameChanged( iframe):
    mTargetSkin.setPoseDOF(mTargetMotion(iframe))
    mTargetSkin2.setPoseDOF(mTargetMotion2(iframe))
    mRobotSkin.setPoseDOF(mTargetMotion2(iframe))

    chest=mTargetLoader.getBoneByName('chest')
    RE.draw('Axes',mTargetSkin.getState().globalFrame(chest.treeIndex()),'chest',100)
    root=mTargetLoader.getBoneByName('root')
    RE.draw('Axes',mTargetSkin.getState().globalFrame(root.treeIndex()),'root',100)
    #lua.M('mTargetSkin3', 'setPoseDOF', mTargetMotion2(iframe))
    mSourceSkin.setPose(mLafanLoader.mMotion.pose(iframe))
    mTimeline.moveCamera(iframe)

def frameMove(fElapsedTime):
    return 0

def handleRendererEvent(ev, x,y,button):
    return 0



# main

RE.createMainWin(sys.argv)
this=m.getPythonWin()

skinScale=1

#mujocofile='mujoco/humanoid_deepmimic_Tpose.xml'
mujocofile='../Characters/humanoid_deepmimic_withpalm_Tpose.xml'

#skelFile='../lafan1/sprint1_subject2.bvh'
skelFile='../lafan1/walk1_subject5.bvh'
#skelFile='../lafan1/fallAndGetUp1_subject1.bvh'

mTargetLoader=RE.MujocoLoader(mujocofile, {'convertToYUP':True})
mTargetLoader2=RE.WRLloader('taesooLib/hanyang_lowdof_T_sh.wrl')

# this loads both a bvh skeleton and the motion
mLafanLoader=m.createMotionLoaderExt_cpp(skelFile)

if False:
    # draw spine positions
    mLafanLoader.setPose(mLafanLoader.mMotion.pose(0))
    RE.draw('Sphere', mLafanLoader.getBoneByName('Spine1').getFrame().translation+m.vector3(-100,0,0), 'spine1 position', 'blue')
    RE.draw('Sphere', mLafanLoader.getBoneByName('Spine2').getFrame().translation+m.vector3(-100,0,0), 'spine2 position', 'red')


lua.addPackagePath(Path(scriptPath).joinpath('mujoco'))
lua.dostring("""
    lafan=require('LafanRetarget')
    require('work/gym_cdm2/module/hanyangToRobot')
    """) 


# create a retargeting module. (this should be reused for all bvh files)
# mConvertToMujoco=Mujoco.createAngleRetargetFromLafan(mLafanLoader, mTargetLoader) in lua.
# see the actual definition (Mujoco.lua) for details
lua.F_lua('mConvertToMujoco', "lafan.toMujoco.createAngleRetarget", mLafanLoader, mTargetLoader)
lua.F_lua('mConvertToHanyang', "lafan.toHanyang.createAngleRetarget", mLafanLoader, mTargetLoader2)
#lua.F_lua('mRobotInfo', 'RE.createRobotFromHanyangLowDOF', mTargetLoader2, True)
robotLoader=RE.FBXloader('../../../../FBX/fbxModels/robot/Robot_TPOSE.fbx', {'useTexture':True, 'newRootBone':'pelvis', 'scale':1.02/100.0, 'boneScale':{'neck_01':0.8, 'ball_l':0.01, 'ball_r':0.01} })
robotLoader.fbxInfo[1].material='Body_robot_low_BaseColor.tga' # use hand-designed material instead of auto-gen one
lua.F_lua('mHanyangToRobot', 'RE.createPoseTransferFromHanyangLowDOFtoRobot', mTargetLoader2, robotLoader, True)



mRobotSkin=RE.createSkin(robotLoader, {'adjustable':True})
mRobotSkin.setPose(robotLoader.loader.pose())
mRobotSkin.setScale(100)
mRobotSkin.setTranslation(200,0,0)
lua.M(mRobotSkin, 'setPoseTransfer', lua.instance('mHanyangToRobot.pt'))

# render using the original fbx loader without finger joints
mTargetSkin=RE.createVRMLskin(mTargetLoader, True)
mTargetSkin.setScale(100,100,100)
mTargetSkin.setMaterial('lightgrey_transparent')

mTargetSkin2=RE.createVRMLskin(mTargetLoader2, True)
mTargetSkin2.setScale(100,100,100)
mTargetSkin2.setTranslation(100,0,0)
#lua.F_lua('mTargetSkin3', 'RE.createRobotSkinForHanyangMotion', lua.instance('mRobotInfo'))
#lua.M('mTargetSkin3', 'setTranslation', 200,0,0)

mSourceSkin=RE.createSkin(mLafanLoader)
mSourceSkin.setScale(1,1,1)
mSourceSkin.setTranslation(-100,0,0)

# if you want to use 
# tempLoader=m.createMotionLoaderExt_cpp(bvhfile)
# mot=tempLoader.mMotion

mot=mLafanLoader.mMotion


# cannot use the reference to a lua temporary instance. so .copy is necesary
#mTargetMotion=lua.M('mConvertToMujoco','_convertMotion',mot).copy() # returns a Motion
mTargetMotion=lua.M('mConvertToMujoco','convertMotion',mot).copy() # returns a MotionDOF for mujoco
mTargetMotion2=lua.M('mConvertToHanyang','convertMotion',mot).copy() # returns a MotionDOF for hanyang

lua.M(RE.viewpoint(), 'lookAt', mTargetMotion(0))


if False:
    taesooLib_pose=mTargetMotion(0)
    mujoco_pose=taesooLib_pose.copy()
    mujoco_pose.setVec3(0, taesooLib_pose.toVector3(0).YtoZ())
    # root quater x,y,z
    mujoco_pose.setVec3(4, mujoco_pose.toVector3(4).YtoZ())
    print(mujoco_pose.ref())

# edict can be used in a way similar to a lua table.
info=edict()
info.frameRate=mot.frameRate()

mTimeline=RE.Timeline("Timeline", mTargetMotion.numFrames(), 1/info.frameRate)
mTimeline.attachCameraToMotion(mTargetLoader, mTargetMotion)

this.addCheckButton("2X playback", False)
this.addCheckButton("attach camera", True)

this.updateLayout()
m.startMainLoop() # this finishes when program finishes
dtor()
