# this file contains a line-by-line python port of testDebugDraw.lua (taesooLib/Samples/classification/lua/)
import os
import sys
import pdb # use pdb.set_trace() for debugging
import code # or use code.interact(local=dict(globals(), **locals())) for debugging. see below.
import math

from libcalab_ogre3d import m, lua, RE, control
from pathlib import Path

import numpy as np
from easydict import EasyDict as edict # pip3 install easydict
import pdb


config_hyunwoo=edict({
    'skel':"work/taesooLib/Resource/motion/locomotion_hyunwoo/hyunwoo_lowdof_T.wrl",
    'motion':"work/taesooLib/Resource/motion/locomotion_hyunwoo/hyunwoo_lowdof_T_locomotion_hl.dof",
    'IKconfig':[
        {'effector':['LeftKnee', 'LeftAnkle',m.vector3(0, -0.06, 0.08)], 'reversed':False},
        {'effector':['RightKnee', 'RightAnkle',m.vector3(0, -0.06, 0.08)], 'reversed':False},
        {'effector': ['LeftElbow', 'LeftElbow', m.vector3(0.33, 0, 0)], 'reversed': False},
        {'effector': ['RightElbow', 'RightElbow', m.vector3(-0.33, 0, 0)], 'reversed': False},
    ],
    'skinScale':100,
    'initialHeight':0,
})


config=config_hyunwoo


def eventFunction(ev, i):
    limbik()

def createIKsolver(loader, config):
    out=edict()
    mEffectors=m.Effectors()
    numCon=len(config)
    mEffectors.resize(numCon);
    out.effectors=mEffectors
    out.numCon=numCon

    for i in range(0, numCon) :
        conInfo=config[i].effector
        kneeInfo=0
        lknee=loader.getBoneByName(conInfo[kneeInfo])
        mEffectors(i).init(loader.getBoneByName(conInfo[kneeInfo+1]), conInfo[kneeInfo+2])

    out.solver=m.createFullbodyIk_MotionDOF_MultiTarget_lbfgs(loader.dofInfo);
    return out


def loadMotion(skel, motion, skinScale=None):
    mot=edict()
    mot.loader=RE.WRLloader(skel)
    mot.motionDOFcontainer=m.MotionDOFcontainer(mot.loader.dofInfo, motion)
    if skinScale :
        mot.skin=createSkin(skel, mot.loader, skinScale)
        mot.skin.applyMotionDOF(mot.motionDOFcontainer.mot)
        mot.skin.setMaterial('lightgrey_transparent')

    return mot


def ctor(this):
    global mSkin, mMotionDOF, mLoader, mMotionDOFcontainer, mSolverInfo, mCON, mPose
    this.create("Button", "rotate light", "rotate light",1)
    this.create("Value_Slider", "arm ori y", "arm ori y",1)
    this.widget(0).sliderRange(-math.radians(90),math.radians(90))
    this.widget(0).sliderValue(0)
    

    this.updateLayout();

    #RE.console();
    mMot=loadMotion(config.skel, config.motion)
    mLoader=mMot.loader

    #lua.setGlobal('mLoader', mLoader)
    #print(lua.M1_int('mLoader', 'numBone'))

    #max_iter=mLoader.numBone()
    for i in range(1, mLoader.numBone()-1 +1) :
        if mLoader.VRMLbone(i).numChannels()==0 :
            mLoader.removeAllRedundantBones()
            #mLoader.removeBone(mLoader.VRMLbone(i))
            #mLoader.export(config[1]..'_removed_fixed.wrl')
            break


    mLoader._initDOFinfo()


    mMotionDOFcontainer=mMot.motionDOFcontainer
    mMotionDOF=mMotionDOFcontainer.mot

    # in meter scale
    for i in range(0, mMotionDOF.rows()-1 +1) :
        mMotionDOF.matView().set(i, 1, mMotionDOF.matView()(i,1)+(config.initialHeight or 0))


    # rering is done in cm scale
    mSkin= RE.createVRMLskin(mLoader, True);
    mSkin.setMaterial("lightgrey_transparent")
    s=config.skinScale
    mSkin.scale(s,s,s); # motion data often is in meter unit while visualization uses cm unit.
    mSkin.setThickness(3/s)
    mPose=m.vectorn()
    mPose.assign(mMotionDOF.row(0));
    mSkin.setPoseDOF(mPose);


    mSolverInfo=createIKsolver(mLoader, config.IKconfig)
    effectors=mSolverInfo.effectors
    numCon=mSolverInfo.numCon


    mLoader.setPoseDOF(mPose)
    originalPos=lua.dynamic_list()
    for i in range(0,numCon ) :
        opos=effectors(i).bone.getFrame().toGlobalPos(effectors(i).localpos)
        originalPos[i]=opos*config.skinScale

    originalPos.append( mLoader.bone(1).getFrame().translation*config.skinScale) # desired COM initially unused.
    mCON=RE.Constraints(originalPos)
    #mCON.setOption(1*config.skinScale)
    mCON.connect(__name__, "eventFunction")  # 현재 모듈(__name__)의 eventFunction 함수(위에 정의됨)에 event 연결.

def limbik():
    global mSkin, mPose, mLoader, mMotionDOF, mSolverInfo
    this=m.getPythonWin()
    mEffectors=mSolverInfo.effectors
    numCon=mSolverInfo.numCon
    mIK=mSolverInfo.solver
    mPose.assign(mMotionDOF.row(0));
    mLoader.setPoseDOF(mPose);
    hasCOM=0
    hasRot=1
    hasMM=0
    COM=mCON.conPos(2)/config.skinScale
    mIK._changeNumEffectors(numCon)
    mIK._changeNumConstraints(hasCOM+hasRot+hasMM)
    footPos=m.vector3N(numCon);
    # pos to global pos
    for i in range(0,numCon) :
        mIK._setEffector(i, mEffectors(i).bone, mEffectors(i).localpos)

        originalPos=mCON.conPos(i)/config.skinScale
        footPos(i).assign(originalPos)
        print(mEffectors(i).bone)
        print(mEffectors(i).localpos)

    
    if hasCOM==1 :
        mIK._setCOMConstraint(0, COM)

    if hasRot==1 :
        bone=mLoader.getBoneByName('LeftElbow')

        mIK._setOrientationConstraint(hasCOM, bone,m.quater(this.findWidget('arm ori y').sliderValue(),m.vector3(0,1,0)));

    if hasMM==1 :
        mIK._setMomentumConstraint(hasCOM+hasRot,m.vector3(0,0,0),m.vector3(0,0,0));

    mIK._effectorUpdated()

    import time
    t0 = time.perf_counter()
    mIK.IKsolve(mPose, footPos)
    elapsed = time.perf_counter() - t0

    print( f"IK {elapsed * 1000:.1f} ms. ")
    mSkin.setPoseDOF(mPose);


def onCallback(w, userData)  :

    if w.id()=="button1" :
        print("button1\n");
    elif w.id()=='rotate light' :
        osm=RE.ogreSceneManager()
        if osm.hasSceneNode("LightNode") :
            lightnode=osm.getSceneNode("LightNode")
            lightnode.rotate(m.quater(math.radians(30),m.vector3(0,1,0)))

    limbik()


def handleRendererEvent(ev, button, x,y) :
    global mCON
    if mCON :
        return mCON.handleRendererEvent(ev, button, x,y)

    return 0

def main():

    RE.createMainWin(sys.argv)
    ctor(m.getPythonWin())
    print('ctor finished')
    m.startMainLoop() # this finishes when program finishes

if __name__=="__main__":
    main()
