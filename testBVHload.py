import os, sys, pdb, math, random
from libcalab_ogre3d import RE,m,lua,control
import numpy as np

elapsedTime=0
            
def frameMove(fElapsedTime):
    global elapsedTime, mLoader, mMotion,mSkin

    elapsedTime=elapsedTime+fElapsedTime
    currFrame=round(elapsedTime*30) # assuming 120hz

    if currFrame>=mMotion.numFrames():
        return
    mLoader.setPose(mMotion.pose(currFrame))
    leftwristpos=mLoader.getBoneByName('LeftHand').getFrame().translation
    RE.draw("Sphere", leftwristpos*100, "wrist1", "red", 6)

    mSkin.setPose(mMotion.pose(currFrame))

def onCallback(w, uid):
    print(w.id())


this=RE.createMainWin(sys.argv)

this.addButton('hihi')
this.updateLayout()

mLoader=RE.createMotionLoaderExt('work/taesooLib/Resource/motion/wd2_2foot_walk_turn2.bvh')
mMotion=mLoader.mMotion

mSkin= RE.createSkin(mLoader);    # to create character
mSkin.scale(1,1,1);                    # this motion data is in cm unit 
mSkin.setPose(mMotion.pose(0));

RE.viewpoint().vpos.set(0, 50, 300)
RE.viewpoint().vat.set(0,40,0)
RE.viewpoint().update()

while True:
    if not RE.renderOneFrame(True):
        break
