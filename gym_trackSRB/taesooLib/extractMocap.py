import time

import numpy as np
import torch,pdb, os
from taesooLib.FullbodyVAE import *
from torch.utils.tensorboard import SummaryWriter
from taesooLib.train_utils import train_FullbodyVAE, findDevice
#from Models.testSRB_mujoco_original_viewer import *

import work.luamodule as lua  # see luamodule.py
import work.rendermodule as RE # see rendermodule.py
import work.controlmodule as control
from easydict import EasyDict as edict
from pathlib import Path
import taesooLib.DeltaExtractor as ED 
m=lua.taesooLib()
mocap_frame_rate=30
def extract(refTraj_all):
    model_path = 'Characters/SRB4.xml'
    mLoader=RE.MujocoLoader(model_path, {'convertToYUP':True})

    mLoader_full=RE.WRLloader('taesooLib/hanyang_lowdof_T_sh.wrl')

    lua.addPackagePath(Path(os.getcwd()).joinpath('mujoco'))
    lua.require_as( 'taesooLib/LafanRetarget', 'lafan')
    lua.require_as('RigidBodyWin/subRoutines/CollisionIK', 'CA')
    lua.require('work/gym_cdm2/module/hanyangToRobot')
    mocap_path2='lafan1/walk1_subject5.bvh'
    mLafanLoader=m.createMotionLoaderExt_cpp(mocap_path2)
    lua.F_lua('mConvertToHanyang', "lafan.toHanyang.createAngleRetarget", mLafanLoader, mLoader_full)
    mLafanMotion=lua.M('mConvertToHanyang','convertMotion',mLafanLoader.mMotion).copy() # returns a MotionDOF for hanyang_lowdof_T

    mAttentionPose=mLafanMotion.row(82).copy() # 차렸자세. 
    mLafanLoader=None # no longer necessary
    mLafanMotion=None

    mConConfig=lua.F('dofile', 'work/taesooLib/Resource/motion/MOB1/hanyang_lowdof_T.con_config.lua')
    mConConfig.ankles[0]=mLoader_full.getBoneByName(mConConfig.ankles[0])
    mConConfig.ankles[1]=mLoader_full.getBoneByName(mConConfig.ankles[1])
    mConConfig.toes[0]=mLoader_full.getBoneByName(mConConfig.toes[0])
    mConConfig.toes[1]=mLoader_full.getBoneByName(mConConfig.toes[1])

    mLegBones=lua.F('CA.checkAllChildren', mLoader_full, mConConfig.ankles[0].parent().parent().name())|lua.F('CA.checkAllChildren', mLoader_full, mConConfig.ankles[1].parent().parent().name())

    # regenerate
    mMocapDeltaAll={}
    if 'hash' in refTraj_all:
        mMocapDeltaAll['hash']=refTraj_all['hash']
    for k in refTraj_all:
        #if k!='var_name' and not k.endswith('mirrored'):
        if k!='var_name' and k!='hash':
            mocap_filename=k
            refTraj=refTraj_all[k]

            motionId=mocap_filename

            mLenAdjust, mLafanMotion, mAlignedLafanMotion, feetTraj, d_feetTraj, mDelta= ED.switchRefMotion(mLoader,mLoader_full, mConConfig,motionId, refTraj)
            mMocapDeltaAll[k]=mDelta
            print('done extracting '+'delta_'+mocap_filename)
            _fullbody_obs_dim = mDelta.toPelvis.cols()+mDelta.toFeet.cols()+mDelta.toMomentum.cols()+mDelta.poses.cols()+mDelta.dx.cols()*2+4


    return mMocapDeltaAll
