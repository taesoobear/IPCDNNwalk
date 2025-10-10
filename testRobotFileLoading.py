import os,sys, pdb, math, random, copy
# 현재 경로 안에 work폴더가 있어야 함. (ln -s ~/sample_python/work ./work)
from libcalab_ogre3d import RE, m, lua, control

import numpy as np
from easydict import EasyDict as edict # pip3 install easydict




robot_files=[
    # libcalab supports many different formats
    # all these files can be converted to mujoco xml.
    edict( title='pi2_urdf', file='work/robots/pi2_robot/pi_12dof_release_v1.urdf' ),
    edict( title='pi2_urdf_collision', file=['work/robots/pi2_robot/pi_12dof_release_v1.urdf',{'useVisualMesh':False}] ),
    #edict( title='pi2_urdf_dae ', file='work/robots/pi2_robot/pi_12dof_release_v1_rl_dae.urdf' ),
    edict( title='swing', file="work/taesooLib/Resource/mesh/swing.wrl.lua" ), # wrl.lua format is convenient because robots can be generated procedurally
    edict( title='humanoid', file='gym_trackSRB/Characters/humanoid_deepmimic_withpalm_Tpose.xml'),
    edict( title='humanoid_YUP', file=['gym_trackSRB/Characters/humanoid_deepmimic_withpalm_Tpose.xml' ,{'convertToYUP':True}]),
	edict( title='hanyang_T', file='gym_trackSRB/taesooLib/hanyang_lowdof_T_sh.wrl'),  # T-pose == identity pose
    edict( title='hyunwoo', file='work/taesooLib/Resource/motion/locomotion_hyunwoo/hyunwoo_lowdof_T_boxfoot.wrl' ),
]

#lua require should be in the ctor function

mLoader=None
mSkin=None
taesooLib_pose=None
mLastFileName=None

def start(robotfile):
    global mLoader,mSkin, taesooLib_pose, mLastFileName


    if isinstance(robotfile, list):
        if robotfile[0][-3:]=='xml' :
            mLoader=RE.MujocoLoader(robotfile[0], robotfile[1])
        elif robotfile[0][-4:]=='urdf' :
            mLoader=RE.URDFloader(robotfile[0], robotfile[1])
        else:
            assert(False)
        mLastFileName=robotfile[0]
    else:
        #mLoader=MujocoLoader(robotfile, [ useVisualMesh=True, convertToYUP=False, exportToMemory=True]) # todo. .obj rotation when convertToYUP==true. set exportToMemory False to generate .xml.wrl (which is a little slow)
        
        if robotfile[-3:]=='xml' :
            mLoader=RE.MujocoLoader(robotfile)
        elif robotfile[-4:]=='urdf' :
            mLoader=RE.URDFloader(robotfile)
        else:
            print(robotfile)
            mLoader=RE.WRLloader(robotfile)
            mLoader=mLoader.YtoZ()  # this file (showRobotFile.py) uses RE.setViewYup(False)
            if mLoader.VRMLbone(1).HRPjointType(0)==control.HRP_JOINT_FREE and taesooLib_pose!=None:
                taesooLib_pose.setTransf(0, taesooLib_pose.toTransf(0).YtoZ())
        mLastFileName=robotfile

    mLoader.updateInitialBone()


    this.create("Box", "bones", "bones")
    this.addFloatSlider('root_height',0.5,0,1.5)
    for i in range(2, mLoader.numBone()-1 +1):
        ndof=mLoader.dofInfo.numDOF(i)
        if ndof>1:
            for j in range(ndof):
                this.addFloatSlider(mLoader.bone(i).name()+"_dof"+str(j),0,-2,2)
        elif ndof>0:
            this.addFloatSlider(mLoader.bone(i).name(),0,-2,2)

    RE.draw("Axes",m.transf(m.quater(1,0,0,0),m.vector3(0,0,0)), 'axes', 100)

    hasFreeJoint=False
    if mLoader.VRMLbone(1).HRPjointType(0)==control.HRP_JOINT_FREE:
        hasFreeJoint=True
        this.findWidget('root_height').activate()
    else:
        this.findWidget('root_height').deactivate()

    if not taesooLib_pose :
        taesooLib_pose=mLoader.getPoseDOF()
        if hasFreeJoint:
            taesooLib_pose.set(2,0.5)

    this.updateLayout()

    mSkin=RE.createVRMLskin(mLoader, True)
    mSkin.scale(100,100,100)
    #mSkin.setMaterial('lightgrey_transparent')


    RE.draw("Axes",m.transf(m.quater(1,0,0,0),m.vector3(0,0,0)), 'axes', 100)


    mSkin.setPoseDOF(taesooLib_pose)



def dtor():
    global taesooLib_pose

    this.removeWidgets(this.widgetIndex('bones'));
    mLoader=None
    mSkin=None
    taesooLib_pose=None
    lua.collectgarbage()


def handleRendererEvent():
    return 0

def onCallback(w, userData):
    global titleMap, taesooLib_pose
    if w.id()=='file' :
        dtor()
        robotfile= titleMap[w.menuText()]
        if 'posedof' in robotfile:
            taesooLib_pose=robotfile.posedof.copy()
        else:
            taesooLib_pose=None
        start(robotfile.file)
    elif w.id()=='root_height':
        taesooLib_pose.set(2, w.sliderValue())
        mSkin.setPoseDOF(taesooLib_pose)
    elif w.id()=='export MJCF':
        print('exporting to '+mLastFileName+'.xml')
        mLoader.exportXML(mLastFileName+'.xml')
    else:
        ti=mLoader.getTreeIndexByName(w.id())
        if ti!=-1 :
            taesooLib_pose.set(mLoader.dofInfo.startT(ti), w.sliderValue())
        else:
            id=w.id()[:-5]
            idof=int(w.id()[-1:])
            ti=mLoader.getTreeIndexByName(id)
            taesooLib_pose.set(mLoader.dofInfo.startT(ti)+idof, w.sliderValue())
        mSkin.setPoseDOF(taesooLib_pose)
        print(taesooLib_pose)

    return 0

def frameMove(fElapsedTime):
    pass

this=RE.createMainWin(sys.argv)

items=[]
titleMap={}
for i, v in enumerate(robot_files) :
    items.append(v.title)
    titleMap[v.title]=v

this.addMenu('file', items)
this.addButton('export MJCF')
this.updateLayout()


RE.setViewYup(False)
robotfile= robot_files[0].file
if 'posedof' in robot_files[0]:
    taesooLib_pose=robot_files[0].posedof 
start(robotfile)

m.startMainLoop() # this finishes when program finishes

dtor()
