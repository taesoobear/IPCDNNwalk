import os,sys, pdb, math, random, copy 
from libcalab_ogre3d import RE, m, lua, control
import numpy as np

datasetRoot=RE.path('../../d/sample_SAMP') # RE.path returns pathlib.Path (normalized)
AMASS_motion_path=datasetRoot/'AMASS/GRAB/s1/airplane_fly_1_stageii.npz' 

zfile=np.load(str(AMASS_motion_path))
model_path=datasetRoot/'SMPLH'  # path to SMPL models
bm_path=model_path/str(zfile['gender'])/'model.npz'  # path to model.npz (or SMPLX_MALE.npz)


ui=RE.createMainWin()
ui.updateLayout()

def onFrameChanged( iframe):
    global skin, mot
    if iframe<mot.numFrames() :
        skin.setPose(mot.pose(iframe))

if 'betas' in zfile and zfile['betas'].size>0:
    fbx=RE.createSMPLskeleton(str(bm_path), zfile['betas'])
else:
    fbx=RE.createSMPLskeleton(str(bm_path))
pelvis_offset=fbx.loader.bone(1).getFrame().translation.copy()

convertYUP=True

poses=zfile['poses']
mot=fbx.loadSMPLmotion(float(zfile['mocap_frame_rate']), zfile['trans'], zfile['poses'], convertYUP)

mTimeline=RE.Timeline("Timeline", mot.numFrames(), 1/mot.frameRate())
skinScale=100 # rendering in cm unit.

drawSkeleton=True
skin=RE.createFBXskin(fbx, drawSkeleton)
skin.setScale(skinScale, skinScale, skinScale)

while True:
    checkEvents=True
    if not RE.renderOneFrame(checkEvents): break

