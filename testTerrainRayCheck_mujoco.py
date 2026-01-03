
import os,sys, pdb, math, random
import numpy as np
from libcalab_ogre3d import RE,m, lua,control

def dtor():
    RE.removeEntity(RE.getSceneNode("mesh_node"))

this=RE.createMainWin()
this.addText("blue dots are computed\nfrom the mujoco simulator.")
this.updateLayout()

RE.viewpoint().setFOVy(45.000000)
RE.viewpoint().setZoom(1.000000)
RE.viewpoint().vpos.set(-293.561471, 1173.843322, 2852.874747)
RE.viewpoint().vat.set(-290.435846, 183.892835, -117.509685)
RE.viewpoint().update()

mLoader=RE.WRLloader( "work/taesooLib/Resource/motion/MOB1/hanyang_lowdof_T.wrl" )
mFloor=RE.WRLloader("work/taesooLib/Resource/terrain/height_field.png", 40, 40, 3, 1,1)
mFloor.setPosition(m.vector3(-5,0,-5))


loaders=[]
loaders.append(mLoader)
loaders.append(mFloor)

skins=RE.createSkin(loaders)
skins.setScale(100)

# a mujoco xml file will be generated
sim=control.MujocoSim(loaders, '__temp_ragdoll_scene.xml', 1.0/240.0)
sim.setGVector(m.vector3(0,9.8,0))
# without this, ray-pick doesn't work
sim.stepSimulation()



if True:

    numSample=160
    range_x=30
    points=m.matrixn(numSample*numSample,3)
    c=0
    for i in range(numSample):
        for j in range(numSample):
            p=m.vector3()
            p.x=m.map(i, 0, numSample, -range_x, range_x) # when i ==0,  p.x ==-range_x
            p.z=m.map(j, 0, numSample, -range_x, range_x)
            
            p=sim.getTerrainHeight(p) # see work/controlmodule.py for the definition
            points.row(c).setVec3(0, p)
            c+=1
            
    assert(c==numSample*numSample)
    thickness=20

    RE.draw('SphereM',points.row(0).toVector3(),'p0','red',0.1)
    RE.drawBillboard(points*100,'predicted','blueCircle',thickness,'QuadListV')

def frameMove(fElapsedTime):
    RE.updateBillboards(fElapsedTime) # redraws billboards occasionally

m.startMainLoop() # this finishes when program finishes
