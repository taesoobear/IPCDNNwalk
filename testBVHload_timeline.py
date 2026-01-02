import os, sys, pdb, math, random
from libcalab_ogre3d import RE,m,lua,control
import numpy as np

elapsedTime=0
            

def onCallback(w, uid):
    print(w.id())


this=RE.createMainWin(sys.argv)

this.addButton('hihi')
this.updateLayout()

mLoader=RE.createMotionLoaderExt('work/taesooLib/Resource/motion/wd2_2foot_walk_turn2.bvh')
mMotion=mLoader.mMotion

mSkin= RE.createSkin(mLoader);    # to create character
mSkin.scale(1,1,1);                    # this motion data is in cm unit 
mSkin.applyAnim(mMotion);

# attach to timeline
RE.motionPanel().motionWin().addSkin(mSkin)

RE.viewpoint().vpos.set(0, 50, 300)
RE.viewpoint().vat.set(0,40,0)
RE.viewpoint().update()

while True:
    if not RE.renderOneFrame(True):
        break
