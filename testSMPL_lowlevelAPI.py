# this file contains a line-by-line python port of /Users/taesookwon/d/sample_SAMP/lua/SAMP/testSkinToWrl_smpl.lua
import os,sys, pdb, math, random, copy 
import numpy as np
from easydict import EasyDict as edict # pip3 install easydict

from libcalab_ogre3d import m, lua, RE, control
from pathlib import Path

this=RE.createMainWin(sys.argv)

def onFrameChanged( iframe):
    global skin, mot, fbx, this
    if iframe<mot.numFrames() :
        skin.setPose(mot.pose(iframe))


def loadSMPLmotion(fbx, joint_names, frame_rate, trans, poses, root_offset=None, ZtoY=True):
    motion=m.Motion(fbx.loader)
    num_keyframes=poses.shape[0]
    motion.resize(num_keyframes)

    jointIndices=[None]*len(joint_names)
        
    for i,name in enumerate(joint_names):
        jointIndices[i]= fbx.loader.getRotJointIndexByName(name)

    for iframe in range(num_keyframes):
        pose=motion.pose(iframe)
        for smpl_index, rotJointIndex in enumerate(jointIndices):
            assert(rotJointIndex!=-1)

            rot=RE.toVector3(poses[iframe,smpl_index*3:])
            pose.rotations(rotJointIndex).setRotation(rot) # set from rodrigues
        pose.translations(0).assign(RE.toVector3(trans[iframe]))

    if ZtoY:
        tf=m.transf(m.quater(math.radians(-90), m.vector3(1,0,0)))
        motion.transform(tf)
    if root_offset is not None:
        motion.translate(RE.toVector3(root_offset))
    return motion


if False:
    skelFile = '../sample_SAMP/fbx/armchair.fbx'

    convYUP=False

    fbx=RE.FBXloader(skelFile, edict(toYUP=convYUP, newRootBone='pelvis', cleanupBones= ['left_wrist', 'right_wrist', 'left_foot', 'right_foot'] ))
    mot=fbx.loader.mMotion
else:
    SAMP_PATH='../../d/sample_SAMP'
    zfile=np.load(SAMP_PATH+'/AMASS/GRAB/s1/airplane_fly_1_stageii.npz') 

    poses=zfile['poses']
    if poses.shape[1]==52*3 : # smpl-h
        bm_path=SAMP_PATH+'/SMPLH/'+str(zfile['gender'])+'/model.npz'
    else:
        bm_path=SAMP_PATH+'/SMPL-X/models_lockedhead/smplx/SMPLX_'+str(zfile['gender']).upper()+'.npz'

    dd=np.load(bm_path)
    # vertices
    v_shaped=dd['shapedirs']@zfile['betas']+dd['v_template']
    # joint locations
    J_pos=dd['J_regressor']@v_shaped
    parent_joint_id=dd['kintree_table'][0]
    SMPLX_JOINT_NAMES = [ "pelvis", "left_hip", "right_hip", "spine1", "left_knee", "right_knee", "spine2", "left_ankle", "right_ankle", "spine3", "left_foot", "right_foot", "neck", "left_collar", "right_collar", "head", "left_shoulder", "right_shoulder", "left_elbow", "right_elbow", "left_wrist", "right_wrist", "jaw", "left_eye_smplhf", "right_eye_smplhf", "left_index1", "left_index2", "left_index3", "left_middle1", "left_middle2", "left_middle3", "left_pinky1", "left_pinky2", "left_pinky3", "left_ring1", "left_ring2", "left_ring3", "left_thumb1", "left_thumb2", "left_thumb3", "right_index1", "right_index2", "right_index3", "right_middle1", "right_middle2", "right_middle3", "right_pinky1", "right_pinky2", "right_pinky3", "right_ring1", "right_ring2", "right_ring3", "right_thumb1", "right_thumb2", "right_thumb3", ]
    SMPLH_JOINT_NAMES = [ "pelvis", "left_hip", "right_hip", "spine1", "left_knee", "right_knee", "spine2", "left_ankle", "right_ankle", "spine3", "left_foot", "right_foot", "neck", "left_collar", "right_collar", "head", "left_shoulder", "right_shoulder", "left_elbow", "right_elbow", "left_wrist", "right_wrist", "left_index1", "left_index2", "left_index3", "left_middle1", "left_middle2", "left_middle3", "left_pinky1", "left_pinky2", "left_pinky3", "left_ring1", "left_ring2", "left_ring3", "left_thumb1", "left_thumb2", "left_thumb3", "right_index1", "right_index2", "right_index3", "right_middle1", "right_middle2", "right_middle3", "right_pinky1", "right_pinky2", "right_pinky3", "right_ring1", "right_ring2", "right_ring3", "right_thumb1", "right_thumb2", "right_thumb3"]

    fbx=RE.createFBXskeletonFromVertexData(v_shaped, dd['f'], dd['weights'], J_pos,  parent_joint_id, SMPLH_JOINT_NAMES)

    #mot=fbx.loadSMPLmotion(float(zfile['mocap_frame_rate']), zfile['trans'], zfile['poses'], True) # same as below
    mot=loadSMPLmotion(fbx, SMPLH_JOINT_NAMES, float(zfile['mocap_frame_rate']), zfile['trans'], zfile['poses'],  ZtoY=True)




    # 위 함수는 smpl.lua에 구현되어 있음. 
    # python에서 smpl 모션 로딩하는 구현은 sample_objectInteraction.git의 trumans.py나 lingo.py 참고. 
    # 참고로 smpl모션은 dataset마다 모두 다른 형태로 encoding 되어 있음. trumans 데이타셋은 euler angle쓰고 lingo 데이터 셋은 rodrigues 쓰고.. 등.
    # 위 함수는 amass데이타셋에 맞는 구현.


mTimeline=RE.Timeline("Timeline", mot.numFrames(), 1/mot.frameRate())
#fbx.scale(0.01) # convert to meter unit!!!

skinScale=100 # rendering in cm unit.


skin=RE.createFBXskin(fbx, True)
skin.setScale(skinScale, skinScale, skinScale)

print('ctor finished')

this.updateLayout()
m.startMainLoop() # this finishes when program finishes

