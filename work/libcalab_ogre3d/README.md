# Example code 1 (fbx rendering using timeline). 
```
import os,sys,pdb,math
from libcalab_ogre3d import *
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
```
# example code 2 (bvh loading without using timeline).
```
import os, sys, pdb, math, random
from libcalab_ogre3d import *
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

mLoader=RE.createMotionLoaderExt('../Resource/motion/woody/wd2_2foot_walk_turn2.bvh')
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

```


# Other examples
```
https://github.com/taesoobear/IPCDNNwalk
```

# Changelog 

## Version 0.1.5
- Automatically creates the `work` resource folder.

## Version 0.1.4
- Improved the SceneGraph UI.

## Version 0.1.3
- Fixed null pointer errors related to `Ogre::SceneNode`.
