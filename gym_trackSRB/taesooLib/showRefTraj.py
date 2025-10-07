import time, math, pdb, sys, os
import numpy as np
# This script assumes that there exists "./work". (ln -s ~/sample_python/work ./work)
import work.luamodule as lua  # see luamodule.py
import work.rendermodule as RE # see rendermodule.py
import work.controlmodule as control
from easydict import EasyDict as edict
from pathlib import Path
import DeltaExtractor as ED 
m=lua.taesooLib()


# 加载环境
mocap_filename='walk1_subject5'
mocap_path = "motiondata_mujoco_refined/"+mocap_filename+".txt"
mocap_path2='../lafan1/'+mocap_filename+'.bvh'
mocap_frame_rate=30




def onCallback(w, userdata):
    if w.id()=='align to sim':

        refFrame_mocap=int(refTraj.refFrames(RE.motionPanel().motionWin().getCurrFrame(),1))
        if w.checkButtonValue():
            mRobotSkin.setPoseDOF( mAlignedLafanMotion(refFrame_mocap))
            mRobotSkin.setTranslation( 0,0,0)
        else:
            mRobotSkin.setPoseDOF(mLafanMotion(refFrame_mocap))
            mRobotSkin.setTranslation(100,0,0)


def onFrameChanged(i):
    # sim srb
    mSkin.setPoseDOF(refTraj.globalTraj.row(i))

    refFrame=int(refTraj.refFrames(i,0))
    refFrame_mocap=int(refTraj.refFrames(i,1))

    # mocap srb 
    mSkin_mocap.setPoseDOF(refTraj.mocap_qpos.row(refFrame))

    this=m.getPythonWin()
    if this.findWidget('align to sim').checkButtonValue():
        mRobotSkin.setLengthAndPoseDOF(mLenAdjust.row(refFrame_mocap), mAlignedLafanMotion(refFrame_mocap))
        RE.output('lenadjust', mLenAdjust.row(refFrame_mocap).maximum())
        for l in range(4): #ltoe, lheel, rtoe, rheel
            RE.draw('SphereM',feetTraj[l](i),'con'+str(l),'red',0.02)
    else:
        mRobotSkin.setPoseDOF(mLafanMotion(refFrame_mocap))
    mHanyangSkin.setPoseDOF( mLafanMotion(refFrame_mocap))

    RE.draw('SphereM',mHanyangSkin.getState().globalFrame(mConConfig.ankles[1])*mConConfig.ankles_lpos[1]+m.vector3(1,0,0),'ankleR','blue',0.05)
    RE.draw('SphereM',mHanyangSkin.getState().globalFrame(mConConfig.toes[1])*mConConfig.toes_lpos[1]+m.vector3(1,0,0),'toeR','blue',0.05)
    RE.draw('SphereM',mHanyangSkin.getState().globalFrame(mConConfig.ankles[0])*mConConfig.ankles_lpos[0]+m.vector3(1,0,0),'ankleL','red',0.05)
    RE.draw('SphereM',mHanyangSkin.getState().globalFrame(mConConfig.toes[0])*mConConfig.toes_lpos[0]+m.vector3(1,0,0),'toeL','red',0.05)

    lfootpos=refTraj.globalTraj.row(i).toVector3(7)
    rfootpos=refTraj.globalTraj.row(i).toVector3(10)
    RE.draw('SphereM',lfootpos,'lfootpos','red',0.1)
    RE.draw('SphereM',rfootpos,'rfootpos','blue',0.1)

    RE.draw('Axes',m.transf(m.quater(refTraj.globalTraj.row(i)(13+6), m.vector3(0,1,0)),lfootpos) ,'lfootpos',100)
    RE.draw('Axes',m.transf(m.quater(refTraj.globalTraj.row(i)(13+6+1), m.vector3(0,1,0)),rfootpos) ,'rfootpos',100)


RE.createMainWin(sys.argv)

this=m.getPythonWin()
this.addCheckButton('align to sim', True)
this.updateLayout()


model_path = '../Characters/SRB4.xml'

# skeletons
mLoader=RE.MujocoLoader(model_path, {'convertToYUP':True})
mLoader_full=RE.WRLloader('taesooLib/hanyang_lowdof_T_sh.wrl')

lua.addPackagePath(Path(os.getcwd()).joinpath('mujoco'))
lua.require_as('LafanRetarget', 'lafan')
lua.require('work/gym_cdm2/module/hanyangToRobot')

mLafanLoader=m.createMotionLoaderExt_cpp(mocap_path2)
lua.F_lua('mConvertToHanyang', "lafan.toHanyang.createAngleRetarget", mLafanLoader, mLoader_full)
mLafanMotion=lua.M('mConvertToHanyang','convertMotion',mLafanLoader.mMotion).copy() # returns a MotionDOF for hanyang_lowdof_T
mLafanConstraints=lua.F('lafan.markConstraints', mLafanLoader.mMotion) # left_heel, leftToe, rightHeel, rightToe
mLafanLoader=None # no longer necessary


mHanyangSkin=RE.createSkin(mLoader_full)
mHanyangSkin.setScale(100)



mHanyangSkin=RE.createSkin(mLoader_full)
mHanyangSkin.setScale(100)
mHanyangSkin.setTranslation(100,0,0)
mHanyangSkin.setMaterial('lightgrey_verytransparent')

mConConfig=lua.F('dofile', 'work/taesooLib/Resource/motion/MOB1/hanyang_lowdof_T.con_config.lua')
mConConfig.ankles[0]=mLoader_full.getBoneByName(mConConfig.ankles[0])
mConConfig.ankles[1]=mLoader_full.getBoneByName(mConConfig.ankles[1])
mConConfig.toes[0]=mLoader_full.getBoneByName(mConConfig.toes[0])
mConConfig.toes[1]=mLoader_full.getBoneByName(mConConfig.toes[1])

#lua.F_lua('mRobotInfo', 'RE.createRobotFromHanyangLowDOF', mLoader_full, False)
_robotLoader=RE.FBXloader('../../../../FBX/fbxModels/robot/Robot_TPOSE.fbx', {'useTexture':True, 'newRootBone':'pelvis', 'scale':1.02/100.0, 'boneScale':{'neck_01':0.8, 'ball_l':0.01, 'ball_r':0.01} })
_robotLoader.fbxInfo[1].material='Body_robot_low_BaseColor.tga' # use hand-designed material instead of auto-gen one
lua.F_lua('mHanyangToRobot', 'RE.createPoseTransferFromHanyangLowDOFtoRobot', mLoader_full, _robotLoader, True)

mRobotSkin=RE.createSkin(_robotLoader, {'adjustable':True})
mRobotSkin.setPose(_robotLoader.loader.pose())
mRobotSkin.setScale(100)
mRobotSkin.setTranslation(0,0,0)
lua.M(mRobotSkin, 'setPoseTransfer', lua.instance('mHanyangToRobot.pt'))

mSkin=RE.createSkin(mLoader)
mSkin.setScale(100)
mSkin.setMaterial('lightgrey_verytransparent')

mSkin_mocap=RE.createSkin(mLoader)
mSkin_mocap.setScale(100)
mSkin_mocap.setMaterial('red_transparent')
mSkin_mocap.setTranslation(100,0,0)

refTraj=lua.F('util.loadTable' , '../refTraj_'+mocap_filename+'.dat')

if False:
    ED.verifyDerivative(refTraj)

mAlignedLafanMotion=ED.alignMocapToSimTrajectory(mLafanMotion, refTraj)   
feetTraj, mLenAdjust, d_feetTraj=ED.removeFootSliding(mLoader_full, mAlignedLafanMotion, refTraj, mConConfig, mLafanConstraints)

# finally, let's calculate the deltas between sim and (aligned) mocap
mDelta=ED.computeSimMocapDelta(mLoader_full, mAlignedLafanMotion, feetTraj, d_feetTraj, mLoader, refTraj)
mDelta.con=[mLafanConstraints.leftToe, mLafanConstraints.leftHeel, mLafanConstraints.rightToe,mLafanConstraints.rightHeel ]



ED.set_dx_ddx_v2(mDelta, mLoader_full, mLafanMotion, refTraj)







# use extractReftraj.py instead
#RE.saveTable(mDelta, '../delta_'+mocap_filename+'.dat')




#mTimeline=RE.Timeline("Timeline", refTraj.globalTraj.rows(), 1/30)
mTimeline=RE.Timeline("Timeline", refTraj.cycle_length, 1/30)

# visual setting & camera setting
lua.M(RE.viewpoint(), 'lookAt', refTraj.globalTraj.row(0).toVector3(0))
while True:
    if not RE.renderOneFrame(True): break
