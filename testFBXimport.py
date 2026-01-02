import os,sys,pdb,math
from libcalab_ogre3d import RE,m,lua,control
import numpy as np


def onFrameChanged(iframe):
    global skin1, skin2
    print(iframe)
    skin1.setPose(fbx.loader.mMotion.pose(iframe))
    skin2.setPose(fbx.loader.mMotion.pose(iframe))


            

this=RE.createMainWin(sys.argv)
skelFile='../../Mixamo/fbx_withSkin/bigvegas_Walking.fbx' 
skinScale=1

fbx=RE.FBXloader(skelFile, skinScale=skinScale, useTexture=True, simplifyMesh=False)

# draw skeleton
skin1=RE.createSkin(fbx.loader)
skin1.setScale(skinScale, skinScale, skinScale)

# draw mesh
skin2=RE.createFBXskin(fbx)
skin2.setScale(skinScale, skinScale, skinScale)


g_motion2=fbx.loader.mMotion
mTimeline=RE.Timeline("Timeline", g_motion2.numFrames(), 1/g_motion2.frameRate())
m.startMainLoop() # this finishes when program finishes
