import work.luamodule as lua  # see luamodule.py
import work.rendermodule as RE # see rendermodule.py
import numpy as np
from easydict import EasyDict as edict
import math,pdb
import torch
m=lua.taesooLib()

try:
    from . import mmsto_config
except:
    import mmsto_config
import heapq
from scipy.spatial.transform import Rotation as R
debugMode=False

toPelvis_cols=6
toFeet_cols=12+12
toMomentum_cols=9
nDOF=59
#poses_cols=mLoader_full.dofInfo.numDOF()-7
poses_cols=59-6

def convert_matrix_to_6d(rot_matrix):
    # Extract the first and second columns of rotation matrix
    rot_6d = np.concatenate([rot_matrix[:, 0], rot_matrix[:, 1]], axis=-1)
    return rot_6d
def convertTo6D(obs):
    SRB_quat = obs[1:5]
    SRB_mat = R.from_quat(SRB_quat).as_matrix()
    SRB_mat_6d = convert_matrix_to_6d(SRB_mat)

    obs_6d = np.concatenate([obs[0:1], SRB_mat_6d, obs[5:23]])
    return obs_6d

def alignMocapToSimTrajectory(mLafanMotion, refTraj):
    # align sim-mocap trajectories
    alignTransform=lua.dynamic_list()
    if True:
        # widely spread markers help matching orientations correctly.
        lpos=[]
        lpos.append(m.vector3(0.9,0,0))
        lpos.append(m.vector3(-0.9,0,0))
        lpos.append(m.vector3(0,0.9,0))
        lpos.append(m.vector3(0,-0.9,0))
        lpos.append(m.vector3(0,0.9,0))
        lpos.append(m.vector3(0,-0.9,0))
        lpos2=[]
        lpos2.append(m.vector3(0,0.9,0))
        lpos2.append(m.vector3(0,-0.9,0))
        lpos2.append(m.vector3(0,0,0.9))
        lpos2.append(m.vector3(0,-0,-0.9))
        lpos2.append(m.vector3(0.9,0,0))
        lpos2.append(m.vector3(-0.9,0,0))
        nmarkers=len(lpos)+2*len(lpos2)
        nf=refTraj.globalTraj.rows()
        mocapTraj=m.matrixn(nf, nmarkers*3)
        simTraj=m.matrixn(nf, nmarkers*3)
        
        for i in range(nf):
            refFrame=int(refTraj.refFrames(i,0))
            mocapCOM=refTraj.mocap_qpos.row(refFrame).toTransf(0)
            simCOM=refTraj.globalTraj.row(i).toTransf(0)

            c=0
            for localpos in lpos:
                mocapTraj.row(i).setVec3(c, mocapCOM*localpos)
                simTraj.row(i).setVec3(c, simCOM*localpos)
                c+=3

            lfootpos=refTraj.globalTraj.row(i).toVector3(7)
            rfootpos=refTraj.globalTraj.row(i).toVector3(10)
            lfootori=m.quater(refTraj.globalTraj(i, 13+6), m.vector3(0,1,0))
            rfootori=m.quater(refTraj.globalTraj(i, 14+6), m.vector3(0,1,0))
            simFeetL=m.transf(lfootori, lfootpos)
            simFeetR=m.transf(rfootori, rfootpos)

            mocapFeetL=m.transf( m.quater(refTraj.mocap_feet(refFrame, 6), m.vector3(0,1,0)), refTraj.mocap_feet.row(refFrame).toVector3(0))
            mocapFeetR=m.transf( m.quater(refTraj.mocap_feet(refFrame, 7), m.vector3(0,1,0)), refTraj.mocap_feet.row(refFrame).toVector3(3))

            for localpos in lpos2:
                mocapTraj.row(i).setVec3(c, mocapFeetL*localpos)
                simTraj.row(i).setVec3(c, simFeetL*localpos)
                c+=3

            for localpos in lpos2:
                mocapTraj.row(i).setVec3(c, mocapFeetR*localpos)
                simTraj.row(i).setVec3(c, simFeetR*localpos)
                c+=3


        windowSize=21
        skip=1
        index=m.intvectorn()
        index.colon(0, windowSize*2, skip)
        tempMat=m.matrixn()
        mocapFeature=m.matrixn()
        for i in range(nf):
            if i%100==0:
                print('.', end='')
            sf=i-windowSize
            ef=i+windowSize
            if sf<0:
                ef-=sf
                sf=0
            if ef>nf:
                sf-=ef-nf
                ef=nf

            tempMat.extractRows(mocapTraj, index+sf)
            mocapFeature=tempMat.toVector()
            tempMat.extractRows(simTraj, index+sf)
            simFeature=tempMat.toVector()

            metric=m.KovarMetric(False)
            dist=metric.calcDistance(simFeature, mocapFeature)
            alignTransform[i]=m.transf(metric.m_transfB)
         

    mAlignedLafanMotion=mLafanMotion.copy()


    refFrame_mocap0=0
    for i in range(refTraj.refFrames.rows()):
        refFrame=int(refTraj.refFrames(i,0)) # 
        refFrame_mocap=int(refTraj.refFrames(i,1))
        if i==0:
            refFrame_mocap0=refFrame_mocap
        assert(refFrame_mocap==refFrame_mocap0+i)
        lafanPose=mAlignedLafanMotion(refFrame_mocap)

        # remove error between mocap full vs sim srb (smoothly using kovar metric)
        lafanPose.setTransf(0, alignTransform[i]*lafanPose.toTransf(0))

    return mAlignedLafanMotion 

def computeSimMocapDelta(mLoader_full, mAlignedLafanMotion, feetTraj, d_feetTraj, mLoader, refTraj):
    cdm_tree=m.LoaderToTree(mLoader, False, False)
    full_tree=m.LoaderToTree(mLoader_full, False, False)

    mMotionDOF=mAlignedLafanMotion
    CDMtoRoot=m.matrixn(mMotionDOF.rows(), 6)
    CDMtoRoot.setAllValue(0)
    CDMfeetToHumanFeetPositions=m.matrixn(mMotionDOF.rows(), 12+12) # lfoot*2, rfoot *2+ vel
    CDMfeetToHumanFeetPositions.setAllValue(0)
    comdof_error=m.matrixn(mMotionDOF.rows(), 9) # com position error, momentum error.
    comdof_error.setAllValue(0)


    sframe=int(refTraj.refFrames(0,1))
    assert(refTraj.refFrames(0,0)==0)
    eframe=int(sframe+refTraj.cycle_length-2)

    dmotdof_full=lua.F('MotionDOF.calcDerivative', mMotionDOF.range(sframe, eframe), 30)

    for refFrame in range(sframe, eframe):
        i=refFrame-sframe

        mocapRoot=mMotionDOF.row(refFrame).toTransf(0)
        simCOM=refTraj.globalTraj.row(i).toTransf(0)
        sim_dq=refTraj.globalTraj.row(i).slice(13,13+6) # unlike mujoco qvel, dq contains[world angvel, world linvel]
        cdm_tree.setPoseDOF(mLoader.dofInfo, refTraj.globalTraj.row(i))
        cdm_tree.setDQ(sim_dq)
        cdm_momentum=cdm_tree.calcMomentumCOM(mLoader)

        full_tree.setLinkData(mMotionDOF.row(refFrame),dmotdof_full.row(i))
        full_momentum=full_tree.calcMomentumCOM(mLoader_full)
        full_com=full_tree.calcCOM(mLoader_full)


        simToFull=simCOM.inverse()*mocapRoot
        simToFull.rotation.align(m.quater(1,0,0,0))

        CDMtoRoot.row(refFrame).assign(simToFull.toLogVec())

        simR=simCOM.rotation
        invR=simR.inverse()

        if True:
            # comdof error
            com_error=comdof_error.row(refFrame)

            com_error.setVec3(0, simR.inverse()*(full_com-simCOM.translation))

            simBodyMomentum=m.vectorn(6)
            #invR=mocapRoot.rotation.inverse()
            simBodyMomentum.setVec3(0, invR*cdm_momentum.M()) #body M
            simBodyMomentum.setVec3(3, invR*cdm_momentum.F()) #body F
            
            #fullGlobalVel=invI*Hcom computed from the fullbody motion
            fullBodyMomentum=m.vectorn(6)
            fullBodyMomentum.setVec3(0, invR*full_momentum.M())
            fullBodyMomentum.setVec3(3, invR*full_momentum.F())

            #print('full_wv:', fullBodyVel, 'sim:', simBodyVel)
            com_error.slice(3,0).assign(fullBodyMomentum-simBodyMomentum)




        feetinfo=CDMfeetToHumanFeetPositions.row(refFrame)
        for icontact in range(4):
            ifoot=math.floor(icontact/2)
            cdm_footcenter=refTraj.globalTraj.row(i).toVector3(7+3*ifoot)
            refCoord=m.transf(m.quater(refTraj.globalTraj.row(i)(13+6+ifoot), m.vector3(0,1,0)), cdm_footcenter)
            mocap_footpos=feetTraj[icontact](i)
            feetinfo.setVec3(icontact*3, refCoord.toLocalPos(mocap_footpos))
        for icontact in range(4):
            vel=d_feetTraj.row(i).toVector3(icontact*3)
            feetinfo.setVec3(12+icontact*3, invR*(vel- sim_dq.toVector3(3))*0.1)



    out= edict(toPelvis=CDMtoRoot, toFeet=CDMfeetToHumanFeetPositions, toMomentum=comdof_error, poses=mMotionDOF.sub(0,0, 7,0).copy())

    if debugMode:
        out.mAlignedLafanMotion=mAlignedLafanMotion
    return out

def set_dx_ddx_unused(mDelta, mLoader_full, mLafanMotion):

    lua.require_as('work/gym_cdm2/shortTermOptimizer', 'ShortTrajOpt')
    mTrajOpt=lua.new('ShortTrajOpt', mLoader_full, mLafanMotion)  
    x_original=mTrajOpt.get('x_original')
    nDOF=mTrajOpt.get('loaderToTree').nDOF()
    eulerMotion=x_original.matView(nDOF)
    dx=mTrajOpt.get('dx').matView(nDOF).copy()
    ddx=mTrajOpt.get('ddx').matView(nDOF).copy()

    #RE.motionPanel().scrollPanel().addPanel(eulerMotion.column(3))
    # to local
    for i in range(dx.rows()):
        s=i-5
        e=i
        if s<0: s=0
        if s==e: e+=1

        qy=eulerMotion.column(3).range(s,e).avg()
        invQ=m.quater(qy, m.vector3(0,1,0)).inverse()

        dx.row(i).setVec3(0, invQ*dx.row(i).toVector3(0))
        #if (i==10): print(qy, dx.row(i))
    for i in range(ddx.rows()):
        s=i-5
        e=i
        if s<0: s=0
        if s==e: e+=1

        qy=eulerMotion.column(3).range(s,e).avg()
        invQ=m.quater(qy, m.vector3(0,1,0)).inverse()

        ddx.row(i).setVec3(0, invQ*ddx.row(i).toVector3(0))

    # less important than x_original
    mDelta.dx=dx*0.1
    mDelta.ddx=ddx*0.1

    # motion-dependent  normalization is inconvenient
    #dx_mean=np.mean(dx.ref(), axis=0)
    #dx_ref= dx.ref()
    #dx_ref-=dx_mean
    #dx_std=np.std(dx.ref(), axis=0)
    #dx_ref/=dx_std
    #ddx_mean=np.mean(ddx.ref(), axis=0)
    #ddx_ref=ddx.ref()
    #ddx_ref-=ddx_mean
    #ddx_std=np.std(ddx.ref(), axis=0)
    #ddx_ref/=ddx_std

    #mDelta.norm_info=m.matrixn(4, dx.cols())
    #mDelta.norm_info.row(0).ref()[:]=dx_mean
    #mDelta.norm_info.row(1).ref()[:]=dx_std
    #mDelta.norm_info.row(2).ref()[:]=ddx_mean
    #mDelta.norm_info.row(3).ref()[:]=ddx_std
    
def set_dx_ddx_v3(mDelta, mLoader_full, mAlignedLafanMotion, refTraj):

    lua.require_as('work/gym_cdm2/shortTermOptimizer', 'ShortTrajOpt')

    sframe=int(refTraj.refFrames(0,1))
    assert(refTraj.refFrames(0,0)==0)
    eframe=int(sframe+refTraj.cycle_length-2)

    mTrajOpt=lua.new('ShortTrajOpt', mLoader_full, mAlignedLafanMotion.range(sframe, eframe))  
    x_original=mTrajOpt.get('x_original')
    nDOF=mTrajOpt.get('loaderToTree').nDOF()
    eulerMotion=x_original.matView(nDOF)
    dx=mTrajOpt.get('dx').matView(nDOF).copy()
    ddx=mTrajOpt.get('ddx').matView(nDOF).copy()

    #RE.motionPanel().scrollPanel().addPanel(eulerMotion.column(3))
    # to local
    for i in range(dx.rows()):
        sim_q=refTraj.globalTraj.row(i).toQuater(3)
        sim_dq=refTraj.globalTraj.row(i).slice(13,13+6) # unlike mujoco qvel, dq contains[world angvel, world linvel]
        invQ=sim_q.inverse()

        dx.row(i).setVec3(0, invQ*(dx.row(i).toVector3(0)-sim_dq.toVector3(3)))
        #if (i==10): print(qy, dx.row(i))
    for i in range(ddx.rows()):
        sim_q=refTraj.globalTraj.row(i).toQuater(3)
        invQ=sim_q.inverse()
        ddx.row(i).setVec3(0, invQ*ddx.row(i).toVector3(0))


    nf=mAlignedLafanMotion.numFrames()
    mDelta.dx=lua.zeros(nf, dx.cols())
    mDelta.ddx=lua.zeros(nf, dx.cols())
    # less important than x_original
    mDelta.dx.sub(sframe,eframe-1,0,0).assign(dx*0.1)
    mDelta.ddx.sub(sframe,eframe-2,0,0).assign(ddx*0.1)


def set_dx_ddx_v2_unused(mDelta, mLoader_full, mAlignedLafanMotion, refTraj):

    lua.require_as('work/gym_cdm2/shortTermOptimizer', 'ShortTrajOpt')

    sframe=int(refTraj.refFrames(0,1))
    assert(refTraj.refFrames(0,0)==0)
    eframe=int(sframe+refTraj.cycle_length-2)

    mTrajOpt=lua.new('ShortTrajOpt', mLoader_full, mAlignedLafanMotion.range(sframe, eframe))  
    x_original=mTrajOpt.get('x_original')
    nDOF=mTrajOpt.get('loaderToTree').nDOF()
    eulerMotion=x_original.matView(nDOF)
    dx=mTrajOpt.get('dx').matView(nDOF).copy()
    ddx=mTrajOpt.get('ddx').matView(nDOF).copy()

    #RE.motionPanel().scrollPanel().addPanel(eulerMotion.column(3))
    # to local
    for i in range(dx.rows()):
        s=i-5
        e=i
        if s<0: s=0
        if s==e: e+=1

        qy=eulerMotion.column(3).range(s,e).avg()
        invQ=m.quater(qy, m.vector3(0,1,0)).inverse()

        dx.row(i).setVec3(0, invQ*dx.row(i).toVector3(0))
        #if (i==10): print(qy, dx.row(i))
    for i in range(ddx.rows()):
        s=i-5
        e=i
        if s<0: s=0
        if s==e: e+=1

        qy=eulerMotion.column(3).range(s,e).avg()
        invQ=m.quater(qy, m.vector3(0,1,0)).inverse()

        ddx.row(i).setVec3(0, invQ*ddx.row(i).toVector3(0))


    nf=mAlignedLafanMotion.numFrames()
    mDelta.dx=lua.zeros(nf, dx.cols())
    mDelta.ddx=lua.zeros(nf, dx.cols())
    # less important than x_original
    mDelta.dx.sub(sframe,eframe-1,0,0).assign(dx*0.1)
    mDelta.ddx.sub(sframe,eframe-2,0,0).assign(ddx*0.1)


    # motion-dependent  normalization is inconvenient
    #dx_mean=np.mean(dx.ref(), axis=0)
    #dx_ref= dx.ref()
    #dx_ref-=dx_mean
    #dx_std=np.std(dx.ref(), axis=0)
    #dx_ref/=dx_std
    #ddx_mean=np.mean(ddx.ref(), axis=0)
    #ddx_ref=ddx.ref()
    #ddx_ref-=ddx_mean
    #ddx_std=np.std(ddx.ref(), axis=0)
    #ddx_ref/=ddx_std

    #mDelta.norm_info=m.matrixn(4, dx.cols())
    #mDelta.norm_info.row(0).ref()[:]=dx_mean
    #mDelta.norm_info.row(1).ref()[:]=dx_std
    #mDelta.norm_info.row(2).ref()[:]=ddx_mean
    #mDelta.norm_info.row(3).ref()[:]=ddx_std
    

def removeFootSliding(mLoader_full, mAlignedLafanMotion, refTraj, mConConfig, mLafanConstraints):
    if True:
        # remove foot sliding
        sframe=int(refTraj.refFrames(0,1))
        eframe=int(sframe+refTraj.cycle_length)-2
        validMotion=mAlignedLafanMotion.range(sframe, eframe)
        con=edict(
                head=edict( bone='Head', lpos=m.vector3(0,0,0)),
                leftToe=edict( bone=mConConfig.toes[0].name(), lpos=mConConfig.toes_lpos[0]),
                leftHeel=edict( bone=mConConfig.ankles[0].name(), lpos=mConConfig.ankles_lpos[0]),
                rightToe=edict( bone=mConConfig.toes[1].name(), lpos=mConConfig.toes_lpos[1]),
                rightHeel=edict( bone=mConConfig.ankles[1].name(), lpos=mConConfig.ankles_lpos[1])
                )
        con.leftToe.con=mLafanConstraints.leftToe.range(sframe, eframe)
        con.leftHeel.con=mLafanConstraints.leftHeel.range(sframe, eframe)
        con.rightToe.con=mLafanConstraints.rightToe.range(sframe, eframe)
        con.rightHeel.con=mLafanConstraints.rightHeel.range(sframe, eframe)

        RE.motionPanel().scrollPanel().addPanel(con.leftToe.con, m.CPixelRGB8(255,0,0))
        RE.motionPanel().scrollPanel().addPanel(con.leftHeel.con, m.CPixelRGB8(255,0,0))
        RE.motionPanel().scrollPanel().addPanel(con.rightToe.con, m.CPixelRGB8(0,255,255))
        RE.motionPanel().scrollPanel().addPanel(con.rightHeel.con, m.CPixelRGB8(0,255,255))

        # footskate cleanup
        t=m.Timer()
        # actually doesn't remove sliding (skipSlidingRemoval)
        LenAdjust, feetTraj=lua.F('fc.removeFeetSlidingHeelAndToe', con, mLoader_full, validMotion, {'useLimbIK':True, 'skipSlidingRemoval':True})
        #LenAdjust, feetTraj=lua.F('fc.removeFeetSlidingHeelAndToe', con, mLoader_full, validMotion, {'useLimbIK':True})

        d_feetTraj= feetTraj[0].matView().derivative_forward(30)| feetTraj[1].matView().derivative_forward(30)| feetTraj[2].matView().derivative_forward(30)| feetTraj[3].matView().derivative_forward(30)

        print(t.stop2()/1e6)
        mLenAdjust=lua.ones(mAlignedLafanMotion.numFrames(), LenAdjust.cols())
        mLenAdjust.sub(sframe, eframe,0,0).assign(LenAdjust)


        for i in range(eframe, mAlignedLafanMotion.numFrames()):
            mAlignedLafanMotion.row(i).set(1,0)
    return feetTraj, mLenAdjust, d_feetTraj

def removeCOMsliding_online(mLoader_full, _com_srb, _desired_com, output, fix_y_also=True, projectToGround=None):
    com_srb=_com_srb.matView()
    desired_com=_desired_com.matView()
    currCOM=m.matrixn(output.rows(), 3)
    optCOM=m.matrixn(output.rows(), 3)

    nvar=output.rows()
    for i in range(output.rows()):
        mLoader_full.setPoseDOF(output.row(i))
        currCOM.row(i).setVec3(0, mLoader_full.calcCOM())


    numCon=3 # 첫 두프레임은 잡아준다. 
    initialPos=currCOM[0]
    initialPos2=currCOM[1]

    index=m.intvectorn()
    startPosCon=1
    index.colon(startPosCon, nvar,1)
    coef=m.vectorn(index.size()+1)

    for dim in range(3):
        if not fix_y_also and dim==1:
            pass
        h=lua.new('QuadraticFunctionHardCon',nvar, numCon);
        #for v in range(1, nvar):
        #  minimize velocity
        #    h.add( 1, v, -1, v-1,  -desiredVel(v-1, imarker*3+ dim)/30.0)
        for v in range(1, nvar-1):
            # minimize accelerations : ( -1, 2, -1 ) kernel
            # (-1* y_{i-1} + 2* y_i - 1*y_{i+1})^2
            h.addWeighted(1.5, -1, v-1, 2, v, -1, v+1, -1*(-1*com_srb[v-1][dim]+2*com_srb[v][dim]-com_srb[v+1][dim]))

        h.con( 1, 0, -initialPos(dim))
        h.con( 1, 1, -initialPos2(dim))

        # hard constraints
        coef.setAllValue(1.0)
        coef[-1]= -1*desired_com.column(dim).slice(startPosCon-1, nvar-1).sum()
        h.addCon( index, coef)

        x=h.solve()
        for f in range(x.size()):
            optCOM.row(f).set(dim,x[f])

    if not fix_y_also :
        for f in range(nvar):
            optCOM.row(f).set(1, currCOM(f, 1))

    if False:
        g=-9.8
        A=0.
        B=0.
        C=0.
        D=0.
        E=0.

        zmp=m.vector3N(com_srb.rows()-2)

        inv_dt=30.0
        ri=com_srb.sub(1, -1, 0, 0)
        ri_=(com_srb.sub(2,0,0,0)-com_srb.sub(0,-2,0,0))*inv_dt
        ri__=(com_srb.sub(2,0,0,0)*-1.0+com_srb.sub(1,-1,0,0)*2.0-com_srb.sub(0,-2,0,0))*inv_dt
        mi=60.0
        A=(ri__.column(1)-g  )*ri.column(0)*mi
        B=(ri__.column(0)-0.0)*ri.column(1)*mi
        C=(ri__.column(1)-g  )*mi
        D=(ri__.column(1)-g  )*ri.column(2)*mi
        E=(ri__.column(2)-0.0)*ri.column(1)*mi
        zmp.x().assign((A-B)/C)
        zmp.y().setAllValue(0)
        zmp.z().assign((D-E)/C)

        refTraj.zmp=zmp

    for i in range(output.rows()):
        minusCOM=m.transf(currCOM.row(i).toVector3(0)*-1)
        delta=optCOM.row(i).toVector3(0)-currCOM.row(i).toVector3(0)
        delta.y=0
        axis=delta.cross(m.vector3(0,1,0))
        axis.normalize()

        
        cy=currCOM(i,1)
        if projectToGround!=None:
            cc=currCOM.row(i).toVector3(0)
            projectToGround(cc)
            cy-=cc.y
        R=m.transf(m.quater(-delta.length()*cy*0.5, axis))
        plusCOM=m.transf(optCOM.row(i).toVector3(0))

        output.row(i).setTransf(0, (plusCOM*R*minusCOM)*output.row(i).toTransf(0))

def verifyDerivative(refTraj):
    # verify mocap derivative
    if True:
        qvel=refTraj.mocap_vel_com.ref().copy() # mujoco qvel. contains[world linvel, body angvel]

        dmotdof=lua.F('MotionDOF.calcDerivative', refTraj.mocap_qpos, 30)
        qvel2=np.zeros((dmotdof.rows(), 6))
        for i in range(dmotdof.rows()):
            vel=dmotdof.row(i)
            RE.setVec3(qvel2[i,0:],  refTraj.mocap_qpos.row(i).toQuater(3)*vel.toVector3(0))  # put world lin vel at column 0
            RE.setVec3(qvel2[i,3:],  vel.toVector3(4))  # put body ang vel at column 3

        print('linvel diff', ((qvel[:,0:3] - qvel[:,0:3])**2).mean(axis=0))
        print('angvel diff', ((qvel[:,3:6] - qvel[:,3:6])**2).mean(axis=0))

    if True:
        # verify sim derivative
        dq=refTraj.globalTraj.ref()[:,13:13+6].copy() # unlike mujoco qvel, dq contains[world angvel, world linvel]

        dmotdof=lua.F('MotionDOF.calcDerivative', refTraj.globalTraj.sub(0,0, 0,7), 30)
        dq2=np.zeros((dmotdof.rows(), 6))
        for i in range(dmotdof.rows()):
            vel=dmotdof.row(i)
            RE.setVec3(dq2[i,0:],  refTraj.globalTraj.row(i).toQuater(3)*vel.toVector3(4))  # put world ang vel at column 0
            RE.setVec3(dq2[i,3:],  refTraj.globalTraj.row(i).toQuater(3)*vel.toVector3(0))  # put world lin vel at column 3

        print('angvel diff', ((dq[:,0:3] - dq2[:,0:3])**2).mean(axis=0))
        print('linvel diff', ((dq[:,3:6] - dq2[:,3:6])**2).mean(axis=0))




# decoder --------------------------------------------------------------------
def predictSingleFrameFullbody(device,srb_tree, srb_loader,  full_pred, obs_global_history, _old_mmsto_config_unused): 
    SRB_obs_dim=mmsto_config.SRB_obs_dim
    obs_history=obs_global_history.sub(0,0,0,SRB_obs_dim) # local part.

    fullbody_obs_dim = mmsto_config.fullbody_obs_dim

    num_output_frames=mmsto_config.planningHorizon-2


    SRB_obs=obs_history.ref()[0: :mmsto_config.obs_history_buffer_skip, :].flatten()
    obs_th=torch.tensor(SRB_obs, dtype=torch.float32, device=device)


    fullbodyvae_encoder, fullbodyvae_decoder=full_pred
    latent_code = fullbodyvae_encoder.prior_encoder(obs_th.unsqueeze(0))
    curr_reconstructed_fullbody_obs = fullbodyvae_decoder(obs_th.unsqueeze(0), latent_code)

    
    tl_curr_reconstructed_fullbody_obs_stacked = lua.vec(curr_reconstructed_fullbody_obs[0])
    part1=tl_curr_reconstructed_fullbody_obs_stacked.slice(0, (fullbody_obs_dim-4)*num_output_frames).matView(fullbody_obs_dim-4)
    part2_con=tl_curr_reconstructed_fullbody_obs_stacked.slice((fullbody_obs_dim-4)*num_output_frames, 0).matView( 4)
    
    out=[None]*num_output_frames

    for i in range(-num_output_frames, 0):
        pose_tl, dq, footRefCoords=unpack_obs_extra(obs_global_history[i].slice(SRB_obs_dim,0)) # global part
        tl_curr_reconstructed_fullbody_obs=part1[i]|part2_con[i]
        out[i+num_output_frames]= _extractSingleFrameFullbody(srb_tree, srb_loader, pose_tl, dq, footRefCoords, tl_curr_reconstructed_fullbody_obs, mmsto_config)
    return out

def _extractSingleFrameFullbody(srb_tree, srb_loader,  pose_tl, dq, footRefCoords, tl_curr_reconstructed_fullbody_obs, _old_mmsto_config_unused): 
    SRBroot=pose_tl.toTransf()
    rootq=SRBroot.rotation

    c=0
    toPelvis=tl_curr_reconstructed_fullbody_obs .range(c,toPelvis_cols)
    SRBtoRoot=toPelvis.to_se3().exp()
    c+=toPelvis_cols
    toHumanFeet=tl_curr_reconstructed_fullbody_obs.range(c, c+toFeet_cols)
    c+=toFeet_cols
    toHumanMomentum=tl_curr_reconstructed_fullbody_obs.range(c, c+toMomentum_cols)
    c+=toMomentum_cols
    humanPose=lua.zeros(7)|tl_curr_reconstructed_fullbody_obs.slice(c,c+poses_cols)
    c+=poses_cols
    humanPose.setTransf(0, SRBroot*SRBtoRoot)

    srb_tree.setPoseDOF(srb_loader.dofInfo, pose_tl)
    srb_tree.setDQ(dq)
    srb_momentum=srb_tree.calcMomentumCOM(srb_loader)

    
    simR=rootq
    com_error=simR*toHumanMomentum.toVector3(0)

    mocapR=humanPose.toQuater(3)
    full_momentum=m.vectorn(6)
    momentum_error_M=simR*toHumanMomentum.toVector3(3)
    momentum_error_F=simR*toHumanMomentum.toVector3(6)
    full_momentum.setVec3(0, srb_momentum.M()+momentum_error_M)
    full_momentum.setVec3(3, srb_momentum.F()+momentum_error_F)




    dx=tl_curr_reconstructed_fullbody_obs.slice(c, c+nDOF)
    c+=nDOF


    ddx=tl_curr_reconstructed_fullbody_obs.slice(c, c+nDOF)
    c+=nDOF
    con=tl_curr_reconstructed_fullbody_obs.slice(c, c+4)
    c+=4
    assert(c==tl_curr_reconstructed_fullbody_obs.size())

    feetpos=m.vectorn(3*4+3*4)
    for icontact in range(4):
        ifoot=math.floor(icontact/2)
        refCoord=footRefCoords[ifoot]
        # 0: lfoot
        # 1: rfoot
        feetpos.setVec3(icontact*3, refCoord.toGlobalPos(toHumanFeet.toVector3(icontact*3)))
    # unnormalize feetvel
    for icontact in range(4):
        feetpos.setVec3(12+icontact*3, simR*(toHumanFeet.toVector3(12+icontact*3))*10.0+dq.toVector3(3) )


    # unnormalize dx and ddx
    dx_ref=dx.ref()
    dx_ref*=10.0

    ddx_ref=ddx.ref()
    ddx_ref*=10.0

    # set_dx_ddx_v3
    dx.setVec3(0, simR*dx.toVector3(0)+dq.toVector3(3))
    ddx.setVec3(0, simR*ddx.toVector3(0))
    return humanPose, dx|ddx, feetpos, pose_tl.toVector3(0)+com_error, full_momentum, lua.vec((con.ref()>0).astype(float)), con




# markerTraj: smooth trajectory for extracting desired velocity
# marker : jerky position prediction (used for predicting contact positions and guiding overall feet positions)
def removeFootSliding_online(footLen, conAll, initialFeet, markerTraj, marker, desiredVel=None, projectToGround=None): 
    planningHorizon=conAll.rows()
    optMarkerTraj=m.matrixn(planningHorizon, 4*3)
    assert(markerTraj[0].rows()==planningHorizon)

    if desiredVel==None:
        matMarkerTraj= markerTraj[0].matView()| markerTraj[1].matView()| markerTraj[2].matView()| markerTraj[3].matView()
        desiredVel=matMarkerTraj.derivative_forward(30)
    for ifoot in range(2):

        conToe=lua.ivec(conAll.column(ifoot*2).ref()>0.5)
        conHeel=lua.ivec(conAll.column(ifoot*2+1).ref()>0.5)

        #conPos, conDir, ii=lua.F('fc.getContactPositions_v2', footLen, conToe, conHeel, markerTraj[ifoot*2].slice(-planningHorizon,0), markerTraj[ifoot*2+1].slice(-planningHorizon,0))
        conPos, conDir, ii=lua.F('fc.getContactPositions_v2', footLen, conToe, conHeel, marker.sub(-planningHorizon,0, (ifoot*2)*3, (ifoot*2)*3+3), marker.sub(-planningHorizon,0, (ifoot*2)*3+3, (ifoot*2)*3+6))


        #for i in range(conPos.rows()):
        #    RE.draw('SphereM',conPos(i),'con'+str(i)+str(ifoot),'red',0.1)


        optMarkerTraj.sub(0,0,(ifoot*2)*3,(ifoot*2+1)*3).assign(markerTraj[ifoot*2].matView())
        optMarkerTraj.sub(0,0,(ifoot*2+1)*3, (ifoot*2+2)*3).assign(markerTraj[ifoot*2+1].matView())

        if conToe(0):
            toePos=initialFeet.toVector3(ifoot*2*3)
            conPos(0).assign(toePos-conDir(0)*footLen*0.5)
        elif conHeel(0):
            heelPos=initialFeet.toVector3((ifoot*2+1)*3)
            conPos(0).assign(heelPos+conDir(0)*footLen*0.5)

        for i in range(2):
            # toe, heel
            imarker=ifoot*2+i
            nvar=planningHorizon
            current_traj=markerTraj[imarker].slice(-planningHorizon,0).matView()
            if i==0:
                con=conToe
                markerConPos=conPos+conDir*footLen*0.5
                otherConPos=conPos-conDir*footLen*0.5
                markerDir=current_traj-markerTraj[ifoot*2+1].slice(-planningHorizon,0).matView()
            else:
                con=conHeel
                markerConPos=conPos-conDir*footLen*0.5
                otherConPos=conPos+conDir*footLen*0.5
                markerDir=current_traj-markerTraj[ifoot*2].slice(-planningHorizon,0).matView()

            if projectToGround:
                for j in range(markerConPos.size()):
                    projectToGround(markerConPos(j))
                    projectToGround(otherConPos(j))

            # project

            for f in range(markerDir.rows()):
                markerDir.row(f).setVec3(0, markerDir.row(f).toVector3(0).normalized())

            #numCon=con.slice(2,0).count()+2 # 첫 두프레임은 잡아준다. 
            numCon=(conToe.slice(1,0)|conHeel.slice(1,0)).count()+1 # 첫 한프레임은 잡아준다. 
            initialPos=initialFeet.toVector3((ifoot*2+i)*3)

            index=m.intvectorn()
            startPosCon=2
            index.colon(startPosCon, nvar,1)
            coef=m.vectorn(index.size()+1)

            for dim in range(3):
                h=lua.new('QuadraticFunctionHardCon',nvar, numCon);
                for v in range(1, nvar):
                    h.add( 1, v, -1, v-1,  -desiredVel(v-1, imarker*3+ dim)/30.0)
                for v in range(1, nvar-1):
                    # minimize accelerations : ( -1, 2, -1 ) kernel
                    # (-1* y_{i-1} + 2* y_i - 1*y_{i+1})^2
                    h.addWeighted(1.5, -1, v-1, 2, v, -1, v+1, 0)


                coef.setAllValue(1.0)
                coef[-1]= -1*marker.sub(-planningHorizon,0, imarker*3,(imarker+1)*3).column(dim).slice(startPosCon, nvar).sum()
                if dim!=1:
                    coef.rmult(0.1) # y is more important

                h.addSquared( index, coef)

                h.con( 1, 0, -initialPos(dim))

                # hard constraints
                for icon in range(ii.size()):
                    s=ii.startI(icon)
                    e=ii.endI(icon)
                    cpos=markerConPos(icon)
                    if s==0:
                        #cpos=initialPos (already same)
                        s+=1
                    
                    for f in range(s,e):
                        if con(f):
                                h.con( 1, f, -cpos(dim))
                        else:
                            h.con( 1, f, -(otherConPos(icon)(dim)+footLen*markerDir(f, dim)))

                x=h.solve()
                for f in range(x.size()):
                    optMarkerTraj.row(f).set((ifoot*2+i)*3+dim,x[f])

    for i in range(4):
        for f in range(optMarkerTraj.rows()):
            markerPos=optMarkerTraj.row(f).toVector3(i*3)
            pMpos=markerPos.copy()
            projectToGround(pMpos)
            if markerPos.y<pMpos.y:
                optMarkerTraj.set(f, i*3+1, pMpos.y)

    return optMarkerTraj

def removeFootSliding_online_stair(footLen, conAll, initialFeet, markerTraj, marker, footCenterTraj, desiredVel=None, projectToGround=None): 
    planningHorizon=conAll.rows()
    optMarkerTraj=m.matrixn(planningHorizon, 4*3)
    assert(markerTraj[0].rows()==planningHorizon)

    if desiredVel==None:
        matMarkerTraj= markerTraj[0].matView()| markerTraj[1].matView()| markerTraj[2].matView()| markerTraj[3].matView()
        desiredVel=matMarkerTraj.derivative_forward(30)
    for ifoot in range(2):

        conToe=lua.ivec(conAll.column(ifoot*2).ref()>0.5)
        conHeel=lua.ivec(conAll.column(ifoot*2+1).ref()>0.5)

        #conPos, conDir, ii=lua.F('fc.getContactPositions_v2', footLen, conToe, conHeel, markerTraj[ifoot*2].slice(-planningHorizon,0), markerTraj[ifoot*2+1].slice(-planningHorizon,0))
        __, conDir, ii=lua.F('fc.getContactPositions_v2', footLen, conToe, conHeel, marker.sub(-planningHorizon,0, (ifoot*2)*3, (ifoot*2)*3+3), marker.sub(-planningHorizon,0, (ifoot*2)*3+3, (ifoot*2)*3+6))
        conPos, _=lua.F('fc.getContactPositions_stair', conToe, conHeel, footCenterTraj[ifoot])

        #for i in range(conPos.rows()):
        #    RE.draw('SphereM',conPos(i),'con'+str(i)+str(ifoot),'red',0.1)


        optMarkerTraj.sub(0,0,(ifoot*2)*3,(ifoot*2+1)*3).assign(markerTraj[ifoot*2].matView())
        optMarkerTraj.sub(0,0,(ifoot*2+1)*3, (ifoot*2+2)*3).assign(markerTraj[ifoot*2+1].matView())


        if conToe(0):
            toePos=initialFeet.toVector3(ifoot*2*3)
            conPos(0).assign(toePos-conDir(0)*footLen*0.5)
        elif conHeel(0):
            heelPos=initialFeet.toVector3((ifoot*2+1)*3)
            conPos(0).assign(heelPos+conDir(0)*footLen*0.5)

        for i in range(2):
            # toe, heel
            imarker=ifoot*2+i
            nvar=planningHorizon
            current_traj=markerTraj[imarker].slice(-planningHorizon,0).matView()
            if i==0:
                con=conToe
                markerConPos=conPos+conDir*footLen*0.5
                otherConPos=conPos-conDir*footLen*0.5
                markerDir=current_traj-markerTraj[ifoot*2+1].slice(-planningHorizon,0).matView()
            else:
                con=conHeel
                markerConPos=conPos-conDir*footLen*0.5
                otherConPos=conPos+conDir*footLen*0.5
                markerDir=current_traj-markerTraj[ifoot*2].slice(-planningHorizon,0).matView()

            #if projectToGround:
            #    for j in range(markerConPos.size()):
            #        projectToGround(markerConPos(j))
            #        projectToGround(otherConPos(j))

            # project

            for f in range(markerDir.rows()):
                markerDir.row(f).setVec3(0, markerDir.row(f).toVector3(0).normalized())

            #numCon=con.slice(2,0).count()+2 # 첫 두프레임은 잡아준다. 
            numCon=(conToe.slice(1,0)|conHeel.slice(1,0)).count()+1 # 첫 한프레임은 잡아준다. 
            initialPos=initialFeet.toVector3((ifoot*2+i)*3)

            index=m.intvectorn()
            startPosCon=2
            index.colon(startPosCon, nvar,1)
            coef=m.vectorn(index.size()+1)

            for dim in range(3):
                h=lua.new('QuadraticFunctionHardCon',nvar, numCon);
                for v in range(1, nvar):
                    h.add( 1, v, -1, v-1,  -desiredVel(v-1, imarker*3+ dim)/30.0)
                for v in range(1, nvar-1):
                    # minimize accelerations : ( -1, 2, -1 ) kernel
                    # (-1* y_{i-1} + 2* y_i - 1*y_{i+1})^2
                    h.addWeighted(1.5, -1, v-1, 2, v, -1, v+1, 0)


                coef.setAllValue(1.0)
                coef[-1]= -1*marker.sub(-planningHorizon,0, imarker*3,(imarker+1)*3).column(dim).slice(startPosCon, nvar).sum()
                if dim!=1:
                    coef.rmult(0.1) # y is more important

                h.addSquared( index, coef)

                h.con( 1, 0, -initialPos(dim))

                # hard constraints
                for icon in range(ii.size()):
                    s=ii.startI(icon)
                    e=ii.endI(icon)
                    cpos=markerConPos(icon)
                    if s==0:
                        #cpos=initialPos (already same)
                        s+=1
                    
                    for f in range(s,e):
                        if con(f):
                                h.con( 1, f, -cpos(dim))
                        else:
                            h.con( 1, f, -(otherConPos(icon)(dim)+footLen*markerDir(f, dim)))

                x=h.solve()
                for f in range(x.size()):
                    optMarkerTraj.row(f).set((ifoot*2+i)*3+dim,x[f])

    avoidHull(optMarkerTraj, projectToGround, 0.5)
    avoidHull(optMarkerTraj, projectToGround, 0.75)
    avoidHull(optMarkerTraj, projectToGround, 0.25)

    return optMarkerTraj


def avoidHull(optMarkerTraj, projectToGround, toeWeight):
    for ii in range(2):

        tpos=m.vector3N(optMarkerTraj.rows())
        for f in range(optMarkerTraj.rows()):

            markerPos=optMarkerTraj.row(f).toVector3((ii*2)*3)*toeWeight+ optMarkerTraj.row(f).toVector3((ii*2+1)*3)*(1.0-toeWeight)
            projectToGround(markerPos)
            tpos(f).assign(markerPos)

        res=lua.F('math.projectTrajectoryToItsHull', tpos)

        for f in range(optMarkerTraj.rows()):

            markerPos=optMarkerTraj.row(f).toVector3((ii*2)*3)*toeWeight+ optMarkerTraj.row(f).toVector3((ii*2+1)*3)*(1.0-toeWeight)

            pMpos=markerPos.copy()
            pMpos.y=tpos(f).y
            if markerPos.y<pMpos.y:
                d=pMpos.y-markerPos.y
                optMarkerTraj.set(f, ii*2*3+1, optMarkerTraj(f, ii*2*3+1)+d)
                optMarkerTraj.set(f, (ii*2+1)*3+1, optMarkerTraj(f, (ii*2+1)*3+1)+d)

def create_fullbody_io_buffers(mLoader_full, mmsto_config, outputMotionBuffer=None):
    desired_fullbody_pose_buffer=m.matrixn()
    desired_euler_dx_ddx=m.matrixn()
    markerTarget=m.matrixn()
    com_target=m.vector3N()
    momentum_target=m.matrixn()
    con_target=m.matrixn()
    feet_traj=m.matrixn()
    if not outputMotionBuffer:
        outputMotionBuffer=m.MotionDOF(mLoader_full.dofInfo)
        outputMotionBuffer.resize(mmsto_config.planningHorizon+1)
        for i in range(mmsto_config.planningHorizon+1):
            outputMotionBuffer.row(i).assign(mLoader_full.getPoseDOF())

    
    return desired_fullbody_pose_buffer, desired_euler_dx_ddx, markerTarget, com_target, momentum_target, con_target, feet_traj,  outputMotionBuffer

def combineSimAndPredictedCon(obs_buffer_row, con, con_target, con_float):
    # heel and toe contact state
    lfootcon=obs_buffer_row[-2]>0.5
    rfootcon=obs_buffer_row[-1]>0.5
    if lfootcon :
        if con.range(0,2).sum()<0.4:
            if con_target.rows()>0 and con_target[-1].range(0,2).sum()>0.4:
                # use the previous contact state
                con.range(0,2).assign(con_target[-1].range(0,2))
            else: 
                if con_float(0)>con_float(1):
                    con.set(0, True)
                else:
                    con.set(1, True)
    else:
        con.set(0, False)
        con.set(1, False)
    if rfootcon:  # if no foot is on the floor, pass
        #assert(rfootcon==env.right_foot_in_contact )
        if con.range(2,4).sum()<0.4:
            if con_target.rows()>0 and con_target[-1].range(2,4).sum()>0.4:
                # use the previous contact state
                con.range(2,4).assign(con_target[-1].range(2,4))
            else: 
                if con_float(2)>con_float(3):
                    con.set(2, True)
                else:
                    con.set(3, True)
    else:
        con.set(2, False)
        con.set(3, False)

# if w==0 -> uses only b (no filtering)
# if w==1 -> uses only a (constant)
def expFilter(a, b, w):
    a.assign(a*w+b*(1.0-w))
def updateFullbodyBuffers(fullbody_io_buffers, allPredictions_stacked, mmsto_config, obs_buffer):
    enqueueToFullbodyBuffers(*fullbody_io_buffers, *allPredictions_stacked[-1], mmsto_config, obs_buffer)


    #testBuffers(fullbody_io_buffers, *allPredictions_stacked[-1])

    desired_fullbody_pose_buffer, desired_euler_dx_ddx, markerTarget, com_target, momentum_target, con_target, feet_traj,  _=fullbody_io_buffers

    if desired_fullbody_pose_buffer.rows()>=len(allPredictions_stacked):
        for i in range(-len(allPredictions_stacked), -1):
            humanPose, dx_ddx, feetpos, com, momentum, con, con_float=allPredictions_stacked[i]
            alpha=0.81 # exp (-1/5)
            expFilter(desired_fullbody_pose_buffer[i],humanPose, alpha)
            expFilter(desired_euler_dx_ddx[i],dx_ddx, alpha)
            expFilter(markerTarget[i],feetpos, alpha)
            expFilter(com_target[i],com, alpha)
            expFilter(momentum_target[i],momentum, alpha)

def reloadFullbodyBuffers(fullbody_io_buffers, allPredictions_stacked, mmsto_config, obs_buffer):

    #testBuffers(fullbody_io_buffers, *allPredictions_stacked[-1])

    desired_fullbody_pose_buffer, desired_euler_dx_ddx, markerTarget, com_target, momentum_target, con_target, feet_traj,  _=fullbody_io_buffers

    if desired_fullbody_pose_buffer.rows()>=len(allPredictions_stacked):
        for i in range(-len(allPredictions_stacked), 0):
            humanPose, dx_ddx, feetpos, com, momentum, con, con_float=allPredictions_stacked[i]
            desired_fullbody_pose_buffer[i].assign(humanPose)
            desired_euler_dx_ddx[i].assign(dx_ddx)
            markerTarget[i].assign(feetpos)
            com_target[i].assign(com)
            momentum_target[i].assign(momentum)

def testBuffers(fullbody_io_buffers, humanPose, dx_ddx, feetpos, com, momentum, con, con_float):
    desired_fullbody_pose_buffer, desired_euler_dx_ddx, markerTarget, com_target, momentum_target, con_target, feet_traj,  _=fullbody_io_buffers

    print(desired_fullbody_pose_buffer[-1]-humanPose)
    print(desired_euler_dx_ddx[-1]-dx_ddx)
    print(markerTarget[-1]-feetpos)
    print(com_target[-1]-com)
    print(momentum_target[-1]-momentum)
    pdb.set_trace()



# input: fullbody_io+allPredictions
def enqueueToFullbodyBuffers(desired_fullbody_pose_buffer, desired_euler_dx_ddx, markerTarget, com_target, momentum_target, con_target, feet_traj, outputMotionBuffer, humanPose, dx_ddx, feetpos, com, momentum, con, con_float, _old_mmsto_config_unused, obs_buffer):

    # con_target: previously set fullbody constraints  (excluding the latest)
    # con, con_float: constraint prediction for the latest pose
    combineSimAndPredictedCon(obs_buffer[-1], con, con_target, con_float)

    desired_fullbody_pose_buffer.enqueue(humanPose, mmsto_config.inputMotionBufferSize)
    desired_euler_dx_ddx.enqueue(dx_ddx, mmsto_config.inputMotionBufferSize)
    planningHorizon=mmsto_config.planningHorizon
    con_target.enqueue(con, planningHorizon)
    com_target.enqueue(com, planningHorizon)
    momentum_target.enqueue(momentum, planningHorizon)
    markerTarget.enqueue(feetpos, planningHorizon)
    outputMotionBuffer.enqueue(humanPose, mmsto_config.outputMotionBufferSize)

    # output motion buffer contains initial euler motion (but the first two frames are already fixed. )

def pack_obs_extra(env):
    pose_tl=lua.vec(env.data.qpos)
    pose_tl.setTransf(0, pose_tl.toTransf(0).ZtoY())

    dq=m.vectorn(6)
    dq.setVec3(0, pose_tl.toQuater(3)*RE.toVector3(env.data.qvel[3:]).ZtoY()) # global w
    dq.setVec3(3, RE.toVector3(env.data.qvel[:3]).ZtoY()) # global v

    offset=5
    footRefCoords=[None, None]
    for ifoot in range(2):
        srb_footcenter=RE.toVector3(env.data.site_xpos[offset*ifoot]).ZtoY()
        srb_footori=RE.toQuater(env.data.site_xmat[offset*ifoot]).ZtoY()
        footRefCoords[ifoot]=m.transf(srb_footori, srb_footcenter)
    env_con=lua.zeros(2)
    if env.left_foot_in_contact :
        env_con.set(0,1)
    if env.right_foot_in_contact :
        env_con.set(1,1)
    return pose_tl|dq| footRefCoords[0].toVector()|footRefCoords[1].toVector()|env_con


def unpack_obs_extra(obs_extra):
    si=7+6
    footRefCoords=[None, None]
    footRefCoords[0]=obs_extra.toTransf(si)
    footRefCoords[1]=obs_extra.toTransf(si+7)
    return obs_extra.slice(0,7), obs_extra.slice(7,7+6), footRefCoords

# mAttentionPose is optional.
def removeTpose(mLegBones, mLafanMotion, mAttentionPose, mLoader_full=None):
    if mLoader_full==None:
        mLoader_full=mAttentionPose
        mAttentionPose=lua.vec([-2.33147380e+00,  9.80776793e-01,  3.95812437e+00,  9.97210782e-01, 7.12743227e-02, -1.60809146e-02, -1.52326006e-02,  4.67584624e-02, 3.05888026e-02, -1.89349676e-01,  2.13954645e-01, -2.45247696e-02, 5.25737820e-02, -2.08184347e-01, -7.75900028e-02, -3.24809975e-02, -2.69185951e-01,  3.78271853e-01, -1.87095190e-02,  6.97515564e-02, -2.54500649e-01, -2.16114696e-03,  3.53843107e-03, -1.46961525e-02, -8.70538078e-03,  1.40944048e-02, -5.87480483e-02,  4.58422163e-18, 1.14035947e-18, -7.45156212e-18,  1.13322295e-01, -2.77652208e-01, 8.69976709e-03,  8.19256656e-02, -1.08730916e+00, -1.60585140e-02, 1.00714017e-01,  7.30960536e-02, -3.01383084e-01,  1.17998168e-01, -1.08051749e-01, -2.95323304e-02,  1.12398556e-02,  3.08266970e-01, -1.56807461e-02, -3.19546968e-01,  9.32209212e-01, -3.83523095e-01, 3.96843531e-02,  9.11666379e-02,  7.03423632e-01,  1.32586815e-02, -5.34794226e-02, -3.35863202e-02,  1.21444687e-01, -2.29374074e-03, -1.64527214e-01,  1.28192701e-01, -8.63161431e-02,  3.80252660e-01])
        
        ti=mLoader_full.getTreeIndexByVoca(m.Voca.LEFTSHOULDER)
        if ti!=-1 and mLoader_full.bone(ti).getRotationalChannels()=='XYZ':
            mAttentionPose=lua.vec([ -2.3314738, 0.980776793, 3.95812437, 0.997210782, 0.0712743227, -0.0160809146, -0.0152326006, 0.046758462437381, 0.030588802619692, -0.1893496761435, 0.21395464523849, -0.024524769628761, 0.05257378208306, -0.20818434731422, -0.07759000286203, -0.032480997516694, -0.26918595120582, 0.37827185341482, -0.018709519014936, 0.069751556509023, -0.25450064938654, -0.0021611469615989, 0.0035384310726616, -0.014696152511002, -0.0087053807893184, 0.014094404816126, -0.058748048365974, 1.6090216605294e-17, 4.9077015475461e-18, -2.6735987312075e-17, 0.11332229522569, -0.27765220852826, 0.0086997671379807, 0.067707526872112, -0.0074648613440102, -1.0873622324379, 0.10071401729748, 0.07309605375404, -0.30138308481781, 0.11799816835232, -0.10805174933029, -0.029532330470037, 0.011239855627136, 0.30826697058874, -0.015680746134611, -0.006230034451479, -0.22493217147029, 0.96788464058571, 0.039684353123567, 0.091166638193921, 0.70342363404508, 0.013258681537096, -0.053479422760973, -0.033586320299742, 0.12144468722847, -0.0022937407630844, -0.16452721431005, 0.12819270133283, -0.086316143242548, 0.38025266089755,])
    # remove T-pose using mAttentionPose
    dofInfo=mLoader_full.dofInfo
    for i in range(0, 93):
        pose=mLafanMotion.pose(i)
        for ti in range(2, mLoader_full.numBone()):
            if not mLegBones(ti):
                s=dofInfo.startT(ti)
                e=dofInfo.endR(ti)
                w=m.clampMap(i, 73, 93, 0, 1)
                w=lua.F('math.smoothTransition', w)
                pose.slice(s,e).assign(pose.slice(s,e)*w+mAttentionPose.slice(s,e)*(1-w))
    for i in range(-93, 0):
        pose=mLafanMotion.pose(i)
        for ti in range(2, mLoader_full.numBone()):
            if not mLegBones(ti):
                s=dofInfo.startT(ti)
                e=dofInfo.endR(ti)
                w=m.clampMap(i, -93, -73, 1, 0)
                w=lua.F('math.smoothTransition', w)
                pose.slice(s,e).assign(pose.slice(s,e)*w+mAttentionPose.slice(s,e)*(1-w))




_robotLoader=None
# wrl_path: 'taesooLib/hanyang_lowdof_T_sh.wrl' or MOB1/hanyang_lowdof_T.wrl
def createHanyangLoader():
    mLoader_full=RE.WRLloader('work/taesooLib/Resource/motion/humanHanyang/hanyang_lowdof_T_sh.wrl')
    mLoader_full._changeVoca(m.Voca.LEFTSHOULDER,mLoader_full.findBone("LeftShoulder"))
    mLoader_full._changeVoca(m.Voca.RIGHTSHOULDER,mLoader_full.findBone("RightShoulder"))

    #mLoader_full.setPoseDOF(mAttentionPose)
    #bpose=mLoader_full.pose()
    #lua.require('moduleGraph')
    #lua.F('fixEulerContinuity',mLoader_full)

    #mLoader_full.setPose(bpose)
    #lua.M(mLoader_full,'printPoseDOF')
    #mLoader_full.export('temp.wrl')
    return mLoader_full
def createRobotSkin(mLoader_full):
    global _robotLoader
    if _robotLoader==None:
        lua.require('work/gym_cdm2/module/hanyangToRobot')
        _robotLoader=RE.FBXloader('../../../FBX/fbxModels/robot/Robot_TPOSE.fbx', {'useTexture':True, 'newRootBone':'pelvis', 'scale':1.02/100.0, 'boneScale':{'neck_01':0.8, 'ball_l':0.01, 'ball_r':0.01} })
        _robotLoader.fbxInfo[1].material='Body_robot_low_BaseColor.tga' # use hand-designed material instead of auto-gen one
        lua.F_lua('mHanyangToRobot', 'RE.createPoseTransferFromHanyangLowDOFtoRobot', mLoader_full, _robotLoader, True)
    mRobotSkin=RE.createSkin(_robotLoader, {'adjustable':True})
    mRobotSkin.setPose(_robotLoader.loader.pose())
    mRobotSkin.setScale(100)
    lua.M(mRobotSkin, 'setPoseTransfer', lua.instance('mHanyangToRobot.pt'))
    return mRobotSkin

def getConConfig(mLoader_full):
    lua.require_as('RigidBodyWin/subRoutines/CollisionIK', 'CA')
    mConConfig=lua.F('dofile', 'work/taesooLib/Resource/motion/MOB1/hanyang_lowdof_T.con_config.lua')
    mConConfig.ankles[0]=mLoader_full.getBoneByName(mConConfig.ankles[0])
    mConConfig.ankles[1]=mLoader_full.getBoneByName(mConConfig.ankles[1])
    mConConfig.toes[0]=mLoader_full.getBoneByName(mConConfig.toes[0])
    mConConfig.toes[1]=mLoader_full.getBoneByName(mConConfig.toes[1])

    mLegBones=lua.F('CA.checkAllChildren', mLoader_full, mConConfig.ankles[0].parent().parent().name())|lua.F('CA.checkAllChildren', mLoader_full, mConConfig.ankles[1].parent().parent().name())
    return mConConfig, mLegBones

def mirrorMotion(mTargetLoader, mMotion):
    # mirror
    Lindices=m.intvectorn()
    Rindices=m.intvectorn()
    for i in range(1, mTargetLoader.numBone()):
        bname=mTargetLoader.bone(i).name()
        if bname[0:4]=='Left':
            Lindices.pushBack(i)
            Rindices.pushBack(mTargetLoader.getTreeIndexByName('Right'+bname[4:]))

    mTargetMotionOriginal=m.Motion(mMotion)
    _mTargetMotion=mTargetMotionOriginal.copy()
    mirrorQ=lambda q : m.quater(q.w, q.x, -q.y, -q.z)
    mirrorP=lambda p : m.vector3( -p.x, p.y,p.z)
    for i in range(_mTargetMotion.numFrames()):
        inputP=mTargetMotionOriginal.pose(i)
        inputP.translations(0).assign(mirrorP(inputP.translations(0)))
        mTargetLoader.setPose(inputP)

        for j in range(1, mTargetLoader.numBone()):
            # mirror global Q
            Rglobal=mTargetLoader.bone(j).getFrame().rotation.copy()
            mTargetLoader.bone(j).getFrame().rotation.assign(mirrorQ(Rglobal))

        for j in range(Lindices.size()):
            # swap global orientation
            Rglobal=mTargetLoader.bone(Rindices(j)).getFrame().rotation.copy()
            Lglobal=mTargetLoader.bone(Lindices(j)).getFrame().rotation.copy()
            mTargetLoader.bone(Lindices(j)).getFrame().rotation.assign(Rglobal)
            mTargetLoader.bone(Rindices(j)).getFrame().rotation.assign(Lglobal)

        mTargetLoader.getPose(_mTargetMotion.pose(i))
    out=m.MotionDOF(mTargetLoader.dofInfo, _mTargetMotion)
    return out


def drawOBS(obs_buffer_row, mSkin, draw_offset_SRB):
    SRB_obs_dim=mmsto_config.SRB_obs_dim
    pose_tl=obs_buffer_row.toTransf(SRB_obs_dim)
    c=SRB_obs_dim+7
    mSkin.setPoseDOF(obs_buffer_row.slice(SRB_obs_dim, c))
    #RE.draw('Axes',pose_tl.translate(draw_offset_SRB),'srb_roottf',100)


    dq=obs_buffer_row.slice(c, c+6)
    c+=6

    offset=5
    footRefCoords=[None, None]
    footRefCoords[0]=obs_buffer_row.toTransf(c)
    c+=7
    footRefCoords[1]=obs_buffer_row.toTransf(c)
    c+=7
    env_con=obs_buffer_row.slice(c,0).ref()>0.5

    def getSitePos(site_id, refCoord):
        if site_id>=offset:
            site_id-=offset
        rel=[None]*5
        rel[0]= np.array([0., 0., 0.])
        rel[1] = np.array([0.12, 0.06, 0])
        rel[2] = np.array([0.12, -0.06, 0])
        rel[3] = np.array([-0.12, 0.06, 0])
        rel[4] = np.array([-0.12, -0.06, 0])

        return refCoord*(RE.toVector3(rel[site_id]).ZtoY())

    for site_id in range(offset*2):
        ifoot=int(site_id/offset)
        nameid='f'+str(site_id)
        RE.erase('SphereM',nameid+'spprt')

        # draw swing foot
        material='blue'
        if site_id>=offset:
            material='red'
        pos=getSitePos(site_id, footRefCoords[ifoot])
        RE.draw('SphereM', pos+draw_offset_SRB,nameid+'swing',material,0.02)

        if env_con[ifoot]:
            RE.draw('SphereM', pos+draw_offset_SRB,nameid+'spprt',material,0.05)

def getFullbodyOBS_GT(mMocapDelta, refFrame_mocap):

    con=mMocapDelta.con
    con_bool=lua.vec(con[0](refFrame_mocap), con[1](refFrame_mocap), con[2](refFrame_mocap), con[3](refFrame_mocap))
    # dx [i-1] contains (x[i]-x[i-1])/dt, which is the latest available assuming i+1 is unknown.
    # ddx [i-1] contains (x[i]-2*x[i-1]+x[i-2])/dt, which is the latest available assuming i+1 is unknown.
    fullbody_obs = mMocapDelta.toPelvis.row(refFrame_mocap)|mMocapDelta.toFeet.row(refFrame_mocap)| mMocapDelta.toMomentum.row(refFrame_mocap)|mMocapDelta.poses.row(refFrame_mocap)|mMocapDelta.dx.row(refFrame_mocap-1)|mMocapDelta.ddx.row(refFrame_mocap-1)|con_bool
    return fullbody_obs


def switchRefMotion(mLoader, mLoader_full, mConConfig, motionId_original, refTraj):

    lua.require_as('retargetting/module/footSkateCleanup', 'fc')
    mirrored=False
    motionId=motionId_original
    if motionId.endswith('mirrored'):
        motionId=motionId[:-9]
        mirrored=True
    mocap_filename='walk1_subject5'
    mocap_path = "motiondata_mujoco_refined/"+motionId+".txt"
    mocap_path2='lafan1/'+motionId+'.bvh'


    if motionId[-7:]=='.motion' :

        mocap_path='ref_contact/'+motionId+'.robot.npy.npz'
        mocap_data=np.load(mocap_path)['arr_0']
        con_path='ref_contact/'+motionId[:-7]+'.constraint.npz'
        con_data=np.load(con_path)

        mLafanMotion=m.MotionDOF(mLoader_full.dofInfo)
        mLafanMotion.resize(mocap_data.shape[0])
        mLafanMotion.ref()[:,:]=mocap_data

        mLafanConstraints=edict()

        conNames=['leftToe', 'leftHeel', 'rightToe', 'rightHeel']
        for c in conNames:
            mLafanConstraints[c]=lua.ivec(con_data[c])
    else:
        tempLafanLoader=m.createMotionLoaderExt_cpp(mocap_path2)
        mLafanMotion=lua.M('mConvertToHanyang','convertMotion',tempLafanLoader.mMotion).copy() # returns a MotionDOF for hanyang_lowdof_T
        mLafanConstraints=lua.F('lafan.markConstraints', tempLafanLoader.mMotion) # left_heel, leftToe, rightHeel, rightToe
        

    if mirrored:
        mTargetLoader=mLoader_full
        # mirror
        Lindices=m.intvectorn()
        Rindices=m.intvectorn()
        for i in range(1, mTargetLoader.numBone()):
            bname=mTargetLoader.bone(i).name()
            if bname[0:4]=='Left':
                Lindices.pushBack(i)
                Rindices.pushBack(mTargetLoader.getTreeIndexByName('Right'+bname[4:]))

        mTargetMotionOriginal=m.Motion(mLafanMotion)
        _mTargetMotion=mTargetMotionOriginal.copy()
        mirrorQ=lambda q : m.quater(q.w, q.x, -q.y, -q.z)
        mirrorP=lambda p : m.vector3( -p.x, p.y,p.z)
        for i in range(_mTargetMotion.numFrames()):
            inputP=mTargetMotionOriginal.pose(i)
            inputP.translations(0).assign(mirrorP(inputP.translations(0)))
            mTargetLoader.setPose(inputP)

            for j in range(1, mTargetLoader.numBone()):
                # mirror global Q
                Rglobal=mTargetLoader.bone(j).getFrame().rotation.copy()
                mTargetLoader.bone(j).getFrame().rotation.assign(mirrorQ(Rglobal))

            for j in range(Lindices.size()):
                # swap global orientation
                Rglobal=mTargetLoader.bone(Rindices(j)).getFrame().rotation.copy()
                Lglobal=mTargetLoader.bone(Lindices(j)).getFrame().rotation.copy()
                mTargetLoader.bone(Lindices(j)).getFrame().rotation.assign(Rglobal)
                mTargetLoader.bone(Rindices(j)).getFrame().rotation.assign(Lglobal)

            mTargetLoader.getPose(_mTargetMotion.pose(i))
        mLafanMotion=m.MotionDOF(mLoader_full.dofInfo, _mTargetMotion)
        a=mLafanConstraints.leftToe
        b=mLafanConstraints.leftHeel
        mLafanConstraints.leftToe=mLafanConstraints.rightToe
        mLafanConstraints.leftHeel=mLafanConstraints.rightHeel
        mLafanConstraints.rightToe=a
        mLafanConstraints.rightHeel=b





    mAlignedLafanMotion=alignMocapToSimTrajectory(mLafanMotion, refTraj)   
    #temp=mAlignedLafanMotion.copy()
    feetTraj, mLenAdjust, d_feetTraj=removeFootSliding(mLoader_full, mAlignedLafanMotion, refTraj, mConConfig, mLafanConstraints)
    #mAlignedLafanMotion=temp

    # finally, let's calculate the deltas between sim and (aligned) mocap
    mDelta=computeSimMocapDelta(mLoader_full, mAlignedLafanMotion, feetTraj, d_feetTraj, mLoader, refTraj)
    mDelta.con=[mLafanConstraints.leftToe, mLafanConstraints.leftHeel, mLafanConstraints.rightToe,mLafanConstraints.rightHeel ]



    set_dx_ddx_v3(mDelta, mLoader_full, mLafanMotion, refTraj)

    return mLenAdjust, mLafanMotion, mAlignedLafanMotion, feetTraj, d_feetTraj, mDelta
