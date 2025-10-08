from . import luamodule as lua
from . import rendermodule as RE

import numpy as np
m=lua.taesooLib()
import math,pdb

# defined in VRMLloader.h
HRP_JOINT_FREE=0			# "XYZ", "ZXY" (jointAxis will be ignored, ball joint will be used for the rotational channels)
HRP_JOINT_BALL=1			# quaternion. (jointAxis will be ignored)
HRP_JOINT_ROTATE=2			# ""	, jointAxis
HRP_JOINT_FIXED=3			# ""	, ""	(jointAxis will be ignored)
HRP_JOINT_GENERAL=4		# "Z", "XYZ" when jointAxis== "Z_XYZ"
HRP_JOINT_SLIDE=5			# jointAxis, ""

# to control mujoco characters, install actuators. by default, actuators are installed for all non-rigid-body characters.
# You can also set sim.data.xfrc_applied or sim.data.qfrc_applied
# xfrc_applied is (nbody, 6) 3D force and 3D torque
# qfrc_applied is (nv, 6) 3D force and 3D torque applied at generalized position
# 
# To set these variables you need to update the matrix values since they are not explicitly writeable:
# sim.data.xfrc_applied[:] = new_xfrc # Or something like that
# https://roboti.us/forum/index.php?threads/applying-external-force-at-a-position.2199/

# a simple taesooLib style wrapper. assumes Euler joints (except for the free root joint)
# this class will be moved to controlmodule.py later
class MujocoSim:
    def __init__(self, simLoaders, scene_file=None, timestep=None):
        """ input: 1. simLoaders (a list of loaders or an mjcf xml file name)
                   2. timestep
            output: scene_file (can be None)
        """ 
        import mujoco
        self.mujoco=mujoco
        self.mj_ray=mujoco.mj_ray
        if isinstance(simLoaders, list):
            if scene_file==None:
                scene_file='__temp_ragdoll_scene.xml'
            if timestep==None:
                timestep=1.0/300.0
            RE.writeMujocoXML(simLoaders, scene_file, {'groundPlane':False})
        else:
            assert(scene_file==None)
            scene_file=simLoaders
            simLoaders=RE.MujocoLoader(scene_file)
            if isinstance(simLoaders, m.VRMLloader):
                simLoaders=[simLoaders]

        self.model = mujoco.MjModel.from_xml_path(scene_file)
        self.data = mujoco.MjData(self.model)
        self.loaders=simLoaders
        self.fkSolvers=[] # only for visualization
        self.startq=[]
        self.startv=[]
        self.startctrl=[]
        self.start_body=[]
        startq=0
        startv=0
        startctrl=0
        startb=1
        for loader in simLoaders:
            self.fkSolvers.append(m.BoneForwardKinematics(loader))
            self.startq.append(startq)
            self.startv.append(startv)
            self.startctrl.append(startctrl)
            self.start_body.append(startb)
            startq+=loader.dofInfo.numDOF()
            startv+=loader.dofInfo.numActualDOF()

            #print(mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY,startb), loader.bone(1).name())
            
            if loader.VRMLbone(1).HRPjointType(0)==HRP_JOINT_FREE:
                startctrl+=loader.dofInfo.numActualDOF()-6
                startb+=loader.numBone()-1
            else:
                startctrl+=loader.dofInfo.numActualDOF()
                if loader.VRMLbone(1).HRPjointType(0)!=HRP_JOINT_FIXED:
                    startb+=loader.numBone()-1
                elif loader.dofInfo.numActualDOF()>0:
                    startb+=loader.numBone()-1

        self.startctrl.append(startctrl)
        self.start_body.append(startb)

        assert(startq==len(self.data.qpos))
        assert(startv==len(self.data.qvel))
        assert(startctrl==len(self.data.ctrl))
        if (startb!=self.model.nbody) :
            print("warning! nbody!=nloaders. let's debug later... most codes would work anyway.")
            print("--- printing debug information")
            self.printAll()
            for i,l in enumerate(self.loaders):
                print(i, l.name())

        if timestep is not None:
            self.model.opt.timestep=timestep
        self.servo_settings=lua.dynamic_list()

    def setQpos(self, iloader, qpos):
        self.setLinkPos(iloader, qpos)
    def getQpos(self, iloader):
        startq=self.startq[iloader]
        ndof=self.loaders[iloader].dofInfo.numDOF()
        return self.data.qpos[startq:startq+ndof]
    def setQvel(self, iloader, qvel):
        startv=self.startv[iloader]
        ndof=loader.dofInfo.numActualDOF()
        self.data.qvel[startv:startv+ndof]=qvel
    def getQvel(self, iloader):
        startv=self.startv[iloader]
        ndof=loader.dofInfo.numActualDOF()
        return self.data.qvel[startv:startv+ndof]
    def setCtrl(self, iloader, ctrl):
        s=self.startctrl[iloader]
        e=self.startctrl[iloader+1]
        self.data.ctrl[s:e]=ctrl.ref()

    def getBodyId(self, iloader, treeIndex):
        return self.start_body[iloader]+treeIndex-1
    def getCharacterAndTreeIndex(self, bodyid):
        for i in range(len(self.loaders)):
            if int(bodyid)<self.start_body[i+1]:
                loaderIndex=i
                treeindex=int(bodyid)-self.start_body[i]+1
                return loaderIndex, treeindex

        return None, None

    def getTerrainHeight(self, p):
        model=self.model
        data=self. data
        x=p.x
        z=p.z
        max_y=100.0
        # I put static objects (terrain, floor, ...) in group 1.  Moving objects are in group 0.
        geomgroup=np.array([0, 1, 0, 0, 0, 0], dtype=np.uint8)
        pnt = np.array([x, max_y, z], dtype=np.float64)
        vec = np.array([0.0, -1.0, 0.0], dtype=np.float64)  
        geomid = np.array([-1], dtype=np.int32)
        #bodyexclude=-1 can be used to indicate that all bodies are included.
        distance = self.mj_ray(model, data, pnt, vec, geomgroup=geomgroup, flg_static=1, bodyexclude=-1, geomid=geomid)

        if geomid[0] == -1:
            return m.vector3(x,-100.0, z)
        else:
            return m.vector3(x,max_y-distance, z)

    def printAll(self):
        nbodies=self.model.nbody
        mujoco=self.mujoco
        # all dynamic bodies
        for i in range(nbodies):
            print(i, mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY,i))
        # all static meshes are stored in geom.

    def setGVector(self,g):
        opt=self.model.opt.gravity
        opt[0]=-g.x
        opt[1]=-g.y
        opt[2]=-g.z
    def stepSimulation(self):
        self.mujoco.mj_step(self.model, self.data)
        for iloader in range(0, len(self.loaders)):
            posedof=self.getLinkPos(iloader)
            self.fkSolvers[iloader].setPoseDOF(posedof)

    # for forward kinematics (bone positions and orientations)
    def pose(self, iloader):
        return self.fkSolvers[iloader]

    def numSkeleton(self):
        return len(self.loaders)

    # all member functions below are for compatibility with trbdl_LCP simulator.
    def setStablePDparam_dof(self, iloader, kp, kd):
        self.servo_settings[iloader]=[kp, kd]

    # uses taesooLib-style packing for all inputs and outpus.
    def calculateStablePDForces_dof(self, iloader, pose_d, dpose_d, tau):
        l=self.loaders[iloader]
        nquat=l.dofInfo.numSphericalJoint()
        ndof=l.dofInfo.numDOF()
        nActualDof = ndof-nquat 

        proportionalTorquePlusQDotDeltaT=m.vectorn(nActualDof)
        proportionalTorquePlusQDotDeltaT.setAllValue(0.0)
        derivativeTorque=m.vectorn(nActualDof)
        derivativeTorque.setAllValue(0.0)

        # mujoco indices
        qindex=self.startq[iloader] 

        startv=self.startv[iloader]
        dqindex=startv

        # taesooLib index (for both pos and vel)
        dofindex=0

        # call setStablePDparam first
        assert(self.servo_settings[iloader])
        kp=self.servo_settings[iloader][0]
        kd=self.servo_settings[iloader][1]
        
        timestep=self.model.opt.timestep;
        for ibone in range(1, l.numBone()):
            b=l.VRMLbone(ibone)
            bt=b.HRPjointType(0)
            if bt==HRP_JOINT_FIXED:
                pass
            elif bt==HRP_JOINT_FREE:
                qindex+=7
                dqindex+=6
                dofindex+=7
            elif bt==HRP_JOINT_BALL:
                print("not supported yet")
                assert(False)
            elif bt==HRP_JOINT_ROTATE:
                nq=l.dofInfo.numDOF(ibone);
                for i in range(nq):
                    proportionalTorquePlusQDotDeltaT.set(dqindex-startv, kp[dofindex] * ( pose_d[dofindex] - self.data.qpos[qindex] - timestep * self.data.qvel[dqindex]))
                    derivativeTorque.set(dqindex-startv, kd[dofindex] * (dpose_d[dofindex]- self.data.qvel[dqindex]))
                    qindex+=1
                    dqindex+=1
                    dofindex+=1
            else:
                print("not supported yet")
                assert(False)

        assert(dqindex==ndof-nquat+startv);
        assert(dofindex==ndof);
        assert(qindex==self.startq[iloader]+ndof)

        tau.add( proportionalTorquePlusQDotDeltaT , derivativeTorque);
        tau.range(0,6).setAllValue(0.0);

    def setTau(self, iloader, spd_tau):
        # for compatibility with taesoo Lib
        s=self.startctrl[iloader]
        e=self.startctrl[iloader+1]
        self.data.ctrl[s:e]=spd_tau.ref()[6:]


    def setTorque(self, iloader, spd_tau):
        # different packing from setTorque
        s=self.startctrl[iloader]
        e=self.startctrl[iloader+1]
        self.data.ctrl[s:e]=spd_tau.ref()[7:]

    def setLinkData(self, iloader, data_type, v):
        if data_type==m.Physics.JOINT_VALUE:
            self.setLinkPos(iloader, v)
        elif data_type==m.Physics.JOINT_VELOCITY:
            self.setLinkVel(iloader, v)
        else:
            self.setCtrl(iloader, v.slice(7,0))

    def getPoseDOF(self, iloader):
        return self.getLinkPos(iloader)
    def setPoseDOF(self, iloader, pose):
        return self.setLinkPos(iloader, pose)
    def getLinkData(self, iloader, data_type, v):
        if data_type==m.Physics.JOINT_VALUE:
            v.assign(self.getLinkPos(iloader))
        elif data_type==m.Physics.JOINT_VELOCITY:
            v.assign(self.getDPoseDOF(iloader))
        else:
            assert(False)
    def setLinkPos(self, iloader, posedof):
        startq=self.startq[iloader]
        ndof=self.loaders[iloader].dofInfo.numDOF()
        self.data.qpos[startq:startq+ndof]=posedof.ref()
        self.fkSolvers[iloader].setPoseDOF(posedof)

    def initSimulation(self):
        # only for taesooLib style API.
        pass
    def setQ(self, iloader, q):
        # 이거 루트가 free가 아닐때 만 정상적으로 taesooLib처럼 동작함.
        self.setLinkPos(iloader, q)

    def getLinkPos(self, iloader):
        startq=self.startq[iloader]
        ndof=self.loaders[iloader].dofInfo.numDOF()
        return lua.vec(self.data.qpos[startq:startq+ndof])

    def setDPoseDOF(self, iloader, dposedof):
        self.setLinkVel(iloader, dposedof)

    def setLinkVel(self, iloader, dposedof):
        startv=self.startv[iloader]
        loader=self.loaders[iloader]
        ndof=loader.dofInfo.numActualDOF()

        qvel=None

        # taesooLib uses body linear velocity and body angular velocity.
        # mujoco uses a strange combination: (world linear velocity, body angular velocity)
        if loader.VRMLbone(1).HRPjointType(0)==0 :
            # free joint 
            sq=self.startq[iloader]
            R=RE.toQuater(self.data.qpos[sq+3:sq+7])
            qvel=np.array(dposedof.ref()[1:])
            # world linear vel
            RE.setVec3(qvel[0:3], R*dposedof.toVector3(0))
            # body ang vel
            qvel[3:ndof]=dposedof.ref()[4:ndof+1]
        else:
            qvel=np.array(dposedof.ref())

        self.data.qvel[startv:startv+ndof]=qvel

    # getLinkVel
    def getDPoseDOF(self, iloader):
        startv=self.startv[iloader]
        loader=self.loaders[iloader]
        ndof=loader.dofInfo.numActualDOF()
        dposedof=m.vectorn(loader.dofInfo.numDOF())

        # taesooLib uses body linear velocity and body angular velocity.
        # mujoco uses a strange combination: (world linear velocity, body angular velocity)
        if loader.VRMLbone(1).HRPjointType(0)==0 :
            # free joint 
            sq=self.startq[iloader]
            R=RE.toQuater(self.data.qpos[sq+3:sq+7])
            # body lin vel
            dposedof.setVec3(0, R.inverse()*RE.toVector3(self.data.qvel[startv: startv+3]))

            dposedof.set(3,0) # unused w==0

            # body ang vel
            dposedof.ref()[4:]=self.data.qvel[startv+3:startv+ndof]
        else:
            dposedof.ref()[:]=self.data.qvel[startv:startv+ndof]
        return dposedof
    def getLinkVel(self, iloader):
        return self.getDPoseDOF( iloader)




# ported from PDservo_spd.lua
class PoseMaintainer:
    def __init__(self, skeletonIndex):
        assert(skeletonIndex!=None)
        self.theta=m.vectorn()
        self.dtheta=m.vectorn()
        self.theta_d=m.vectorn() # desired q
        self.dtheta_d=m.vectorn() # desired dq
        self.controlforce=m.vectorn()

        self.skeletonIndex=skeletonIndex or 0
        # followings are temporaries
        self.kp=m.vectorn()
        self.kd=m.vectorn()


    def init(self, skel, simulator, k_p, k_d, k_p_slide=None, k_d_slide=None):
        si=self.skeletonIndex
        simulator.getLinkData(si, m.Physics.JOINT_VALUE, self.theta_d)
        simulator.getLinkData(si, m.Physics.JOINT_VELOCITY, self.dtheta_d)

        dofInfo=skel.dofInfo
        self.kp.setSize(dofInfo.numDOF())
        self.kp.setAllValue(k_p)
        self.kd.setSize(dofInfo.numDOF())
        self.kd.setAllValue(k_d)
        
        if skel.VRMLbone(1).HRPjointType(0)==HRP_JOINT_FREE :
            # exclude free root joint
            self.kp.range(0,7).setAllValue(0)
            self.kd.range(0,7).setAllValue(0)

        
        if k_p_slide==None :
           k_p_slide=k_p*10


        if k_d_slide==None :
           k_d_slide=k_d*500


        for i in range(1,skel.numBone()-1 +1) :
            vbone=skel.VRMLbone(i)
            nJoint=vbone.numHRPjoints()
            for j in range(0, nJoint-1 +1) :
                if vbone.HRPjointType(j)==HRP_JOINT_SLIDE :
                    self.kp.set(vbone.DOFindex(j), k_p_slide)
                    self.kd.set(vbone.DOFindex(j), k_d_slide)
        simulator.setStablePDparam_dof(self.skeletonIndex, self.kp, self.kd)

    def applyPDtorque(self, sim, theta_d, dtheta_d=None):
        self.theta_d.assign(theta_d)
        if dtheta_d:
            self.dtheta_d.assign(dtheta_d)
        else:
            self.dtheta_d.zero()
        self.generateTorque(sim)
        sim.setTau(self.skeletonIndex, self.controlforce)

    def generateTorque(self, simulator):
        si=self.skeletonIndex
        simulator.getLinkData(si, m.Physics.JOINT_VALUE, self.theta)
        simulator.getLinkData(si, m.Physics.JOINT_VELOCITY, self.dtheta)

        tau=m.vectorn()
        simulator.calculateStablePDForces_dof(self.skeletonIndex, self.theta_d, self.dtheta_d, tau)
        self.controlforce=tau

    def resetParam(self, kp, kd, theta_d):
        self.kp.setAllValue(kp)
        self.kd.setAllValue(kd)
        self.theta_d.assign(theta_d)


class PDservo:
    def setCoef(self, dofInfo,kp, kd, tgtVelScale, k_scale, model):
        assert(dofInfo.numSphericalJoint()==1)
        # spherical joint가 있는 경우 PDservo_spherical 사용할 것!
        kp.setSize(dofInfo.numDOF())
        k_p=model.k_p_PD
        k_d=model.k_d_PD
        kp.setAllValue(k_p)
        kd.setSize(dofInfo.numDOF())
        kd.setAllValue(k_d)

        tgtVelScale.setSize(dofInfo.numDOF())
        tgtVelScale.setAllValue(model.k_d_PD)

        # exclude root joint
        kp.range(0,7).setAllValue(0)
        kd.range(0,7).setAllValue(0)
        tgtVelScale.range(0,7).setAllValue(0)

        print("initPDservo."+dofInfo.skeleton().bone(1).name())
        for i in range(2,dofInfo.skeleton().numBone()-1 +1) :
            bone=dofInfo.skeleton().bone(i)
            vbone=bone.treeIndex()
            nJoint=dofInfo.numDOF(vbone)
            #      print("initPDservo."..bone.name())
            for j in range(0, nJoint-1 +1) :

                dofIndex=dofInfo.DOFindex(vbone,j)

                kp.set(dofIndex, k_p*k_scale.default[1])
                kd.set(dofIndex, k_d*k_scale.default[2])
                tgtVelScale.set(dofIndex, k_scale.default[3])

                if bone.voca()==m.Voca.LEFTANKLE or bone.voca()==m.Voca.RIGHTANKLE :
                    if k_scale.ankle :
                        kp.set(dofIndex, k_p*k_scale.ankle[1])
                        kd.set(dofIndex, k_d*k_scale.ankle[2])
                        tgtVelScale.set(dofIndex, k_scale.ankle[3])

                elif bone.voca()==m.Voca.LEFTCOLLAR or bone.voca()==m.Voca.RIGHTCOLLAR :
                    if k_scale.collar :
                        kp.set(dofIndex, k_p*k_scale.collar[1])
                        kd.set(dofIndex, k_d*k_scale.collar[2])
                        tgtVelScale.set(dofIndex, k_scale.collar[3])

                elif bone.voca()==m.Voca.LEFTSHOULDER or bone.voca()==m.Voca.RIGHTSHOULDER :
                    if k_scale.shoulder :
                        kp.set(dofIndex, k_p*k_scale.shoulder[1])
                        kd.set(dofIndex, k_d*k_scale.shoulder[2])
                        tgtVelScale.set(dofIndex, k_scale.shoulder[3])

                elif bone.voca()==m.Voca.LEFTELBOW or bone.voca()==m.Voca.RIGHTELBOW :
                    if k_scale.elbow :
                        kp.set(dofIndex, k_p*k_scale.elbow[1])
                        kd.set(dofIndex, k_d*k_scale.elbow[2])
                        tgtVelScale.set(dofIndex, k_scale.elbow[3])

                elif bone.voca()==m.Voca.LEFTKNEE or bone.voca()==m.Voca.RIGHTKNEE :
                    if k_scale.knee :
                        kp.set(dofIndex, k_p*k_scale.knee[1])
                        kd.set(dofIndex, k_d*k_scale.knee[2])
                        tgtVelScale.set(dofIndex, k_scale.knee[3])

                elif bone.voca()==m.Voca.LEFTHIP or bone.voca()==m.Voca.RIGHTHIP :
                    if k_scale.hip :
                        kp.set(dofIndex, k_p*k_scale.hip[1])
                        kd.set(dofIndex, k_d*k_scale.hip[2])
                        tgtVelScale.set(dofIndex, k_scale.hip[3])

                elif bone.voca()==m.Voca.CHEST :
                    if k_scale.chest :
                        kp.set(dofIndex, k_p*k_scale.chest[1])
                        kd.set(dofIndex, k_d*k_scale.chest[2])
                        tgtVelScale.set(dofIndex, k_scale.chest[3])

                elif bone.voca()==m.Voca.CHEST2 :
                    if k_scale.chest2 :
                        kp.set(dofIndex, k_p*k_scale.chest2[1])
                        kd.set(dofIndex, k_d*k_scale.chest2[2])
                        tgtVelScale.set(dofIndex, k_scale.chest2[3])

                elif bone.voca()==m.Voca.NECK :
                    if k_scale.neck :
                        kp.set(dofIndex, k_p*k_scale.neck[1])
                        kd.set(dofIndex, k_d*k_scale.neck[2])
                        tgtVelScale.set(dofIndex, k_scale.neck[3])

                elif bone.voca()==m.Voca.HEAD :
                    if k_scale.head :
                        kp.set(dofIndex, k_p*k_scale.head[1])
                        kd.set(dofIndex, k_d*k_scale.head[2])
                        tgtVelScale.set(dofIndex, k_scale.head[3])


                if "toes" in bone.name() :
                    dofIndex=dofInfo.DOFindex(vbone,j)
                    if k_scale.toes :
                        kp.set(dofIndex, k_p*k_scale.toes[1])
                        kd.set(dofIndex, k_d*k_scale.toes[2])
                        tgtVelScale.set(dofIndex, k_scale.toes[3])




                if dofInfo.DOFtype(vbone, j)==m.DOFtype.SLIDE :
                    dofIndex=dofInfo.DOFindex(vbone,j)
                    kp.set(dofIndex, model.k_p_slide)
                    kd.set(dofIndex, model.k_d_slide)
                    tgtVelScale.set(dofIndex, 0)




    def __init__(self, dofInfo, model):
        assert(dofInfo.numSphericalJoint()==1) # otherwise, use PDservo_spherical instead.
        self.model=model
        self.theta=m.vectorn()
        self.dtheta=m.vectorn()
        self.theta_d=m.vectorn() # desired q
        self.dtheta_d=m.vectorn() # desired dq
        self.controlforce=m.vectorn()
        self.kp=m.vectorn()
        self.kd=m.vectorn()
        self.tgtVelScale=m.vectorn()
        self.mask_slide=m.vectorn()
        self.muscleActiveness=0.3
        self.mask_slide.setSize(dofInfo.numDOF())
        self.mask_slide.setAllValue(0)
        self.dofInfo=dofInfo
        self.updateCoef(model)
        print ("kp=",self.kp)
        print ("kd=",self.kd)

        clampTorque=800
        clampForce=8000

        if 'clampTorque' in model :
            clampTorque=model.clampTorque


        if 'clampForce' in model :
            clampForce=model.clampForce


        self.clampMax=m.vectorn(dofInfo.numDOF())
        self.clampMax.setAllValue(clampTorque)
        for i in range(2,dofInfo.skeleton().numBone()-1 +1) :
            bone=dofInfo.skeleton().bone(i)
            vbone=bone.treeIndex()
            nJoint=dofInfo.numDOF(vbone)
            for j in range(0, nJoint-1 +1) :
                dofIndex=dofInfo.DOFindex(vbone,j)
                if dofInfo.DOFtype(vbone, j)==m.DOFtype.SLIDE :
                    dofIndex=dofInfo.DOFindex(vbone,j)
                    self.mask_slide.set(dofIndex, 1)
                    self.clampMax.set(dofIndex, clampForce)
                else:
                    self.clampMax.set(dofIndex, clampTorque)




        self.clampMin=self.clampMax*-1

    def updateCoef(self, model):
        dofInfo=self.dofInfo
        k_scale_active=model.k_scale_active_pd

        self.setCoef(dofInfo,self.kp, self.kd, self.tgtVelScale, k_scale_active, model)


    def dposeToDQ(dpose):
        return dpose.slice(0,3)|dpose.slice(7,0)|dpose.slice(4,7)

    def DQtoDpose(dq):
        return dq.slice(0,3)| lua.zeros(1)|dq.slice(-3,0)|dq.slice(3,-3)

    def poseToQ(pose):
        return pose.slice(0,3)|pose.slice(7,0)|pose.slice(3,7)


    def initPDservo(self, startf, endf,motionDOF, dmotionDOF, simulator):
        self.startFrame=startf
        self.endFrame=endf
        self.currFrame=startf
        self.deltaTime=0
        self.motionDOF=motionDOF
        self.dmotionDOF=dmotionDOF

        dofInfo=self.dofInfo
        assert(dofInfo.numSphericalJoint()==1)

        assert(self.kp.size()==dofInfo.numDOF())


        self.skeletonIndex=0


        simulator.setStablePDparam_dof(self.skeletonIndex, self.kp, self.kd)

    # generate FBtorque
    def generateTorque(self, simulator):
        model=self.model
       
        self.currFrame=(simulator.currentTime()+self.deltaTime)*model.frame_rate+self.startFrame
        #print(self.currFrame) # extremely slow.
        if self.currFrame>self.endFrame-1 :
            simulator.getLinkData(0, m.Physics.JOINT_VALUE, self.theta)
            simulator.getLinkData(0, m.Physics.JOINT_VELOCITY, self.dtheta)
            return False


        self._generateTorque(simulator, self.currFrame)
        return True


    #gTimer=util.Timer()
    def stepSimul(self, simulator, drawDebugInformation=None):
        simulator.setLinkData(0, m.Physics.JOINT_TORQUE, self.controlforce)
        if drawDebugInformation :
            simulator.drawDebugInformation()

        #gTimer.start()
        simulator.stepSimulation()
        #print(gTimer.stop2())


    def _generateTorque(self, simulator, frame, target_delta=None):
       
        simulator.getLinkData(0, m.Physics.JOINT_VALUE, self.theta)
        simulator.getLinkData(0, m.Physics.JOINT_VELOCITY, self.dtheta)

        #[[ continuous sampling ]]#
        #   print("theta",self.theta)

        # desired (target) pose
        self.motionDOF.samplePose(frame, self.theta_d)
        if target_delta :
            self.theta_d.radd(target_delta) 
        self.dmotionDOF.sampleRow(frame, self.dtheta_d)

        #   self.dtheta_d.setAllValue(0)

        self.dtheta_d.rmult(self.muscleActiveness) # this corresponds to muscle activeness

        self.controlforce.setSize(self.motionDOF.numDOF())

        #delta=self.theta_d-self.theta
        #MainLib.VRMLloader.projectAngles(delta) # [-pi, pi]
        #self.controlforce.assign(self.kp*delta + self.kd*(self.dtheta_d*self.tgtVelScale-self.dtheta))


        tau=m.vectorn()
        simulator.calculateStablePDForces(self.skeletonIndex, PDservo.poseToQ(self.theta_d),tau)

        self.tau=tau
        self.controlforce=PDservo.DQtoDpose(tau)


        self.controlforce.clamp(self.clampMin, self.clampMax)


    def rewindTargetMotion(self, simulator):
        self.deltaTime=-1*simulator.currentTime()




class PDservo_spherical:

    # returns motQ, motDQ which are compatible with loader_spherical
    def convertMotionState(loader_euler, loader_spherical, motionDOF_euler, frame_rate):

        DMotionDOF_euler=lua.M1_matrixn( motionDOF_euler, 'calcDerivative', frame_rate)

        # important!!!
        # convert loader, motionDOF, and its time-derivative to new formats.

        nf=motionDOF_euler.numFrames()
        motQ=m.matrixn(nf, loader_spherical.dofInfo.numDOF())
        motDQ=m.matrixn(nf, loader_spherical.dofInfo.numActualDOF())

        tree=m.LoaderToTree(loader_euler, False,False)

        euler_dofInfo=loader_euler.dofInfo
        spherical_dofInfo=loader_spherical.dofInfo

        for i in range(0, nf-1 +1) :
            tree.setPoseDOF(euler_dofInfo, motionDOF_euler.row(i))
            tree.setVelocity(euler_dofInfo, DMotionDOF_euler.row(i))

            tree.getSphericalState(spherical_dofInfo, motQ.row(i), motDQ.row(i))

        return motQ, motDQ


    def __init__(self, dofInfo, _model):
        self.theta=m.vectorn()
        self.dtheta=m.vectorn()
        self.theta_d=m.vectorn() # desired q
        self.dtheta_d=m.vectorn() # desired dq
        self.controlforce=m.vectorn()
        self.dofInfo=dofInfo
        self.skeletonIndex=0
        self.model=_model or model


    def initPDservo(self, startf, endf,motQ, motDQ, simulator, ichara):
        csize=m.vector3()
        dofInfo=simulator.skeleton(ichara).dofInfo
        model=self.model
        csize.y=simulator.numSphericalJoints(ichara)
        csize.x=dofInfo.numDOF() -csize.y*4

        # for the root joint and other 1-DOF joints
        self.kp=m.vectorn(int(csize.x+csize.y*3))
        self.kd=m.vectorn(int(csize.x+csize.y*3))
        self.kp.setAllValue(self.model.k_p_PD)
        self.kd.setAllValue(self.model.k_d_PD)
        
        # exclude root translation
        self.kp.range(0,3).setAllValue(0)
        self.kp.range(0,3).setAllValue(0)


        clampTorque=800
        clampForce=8000

        if 'clampTorque' in model:
            clampTorque=model.clampTorque


        if 'clampForce' in model:
            clampForce=model.clampForce

        self.clampMax=m.vectorn(int(csize.x+csize.y*3))
        self.clampMax.setAllValue(clampTorque)
        self.clampMin=self.clampMax*-1

        self.startFrame=startf
        self.endFrame=endf
        self.currFrame=startf
        self.deltaTime=0

        q=m.vectorn()
        dq=m.vectorn()
        simulator.initSimulation()
        simulator.getSphericalState(ichara, q, dq)

        self.nonQsize=csize.x
        self.motions=[motQ, motDQ]
        simulator.setStablePDparam(self.skeletonIndex, self.kp, self.kd)


    def generateTorque(self, simulator):

        model=self.model
        self.currFrame=(simulator.currentTime()+self.deltaTime)*model.frame_rate+self.startFrame
        #print(self.currFrame) # extremely slow.
        if self.currFrame>self.endFrame-1 :
            return False


        self._generateTorque(simulator, self.currFrame)
        return True


    def stepSimul(self, simulator, drawDebugInformation=None):
        simulator.setTau(0, self.controlforce)
        if drawDebugInformation :
            simulator.drawDebugInformation()

        simulator.stepSimulation()


    def _generateTorque(self, simulator, frame):

        self.motions[0].sampleRow(frame, self.theta_d)
        self.motions[1].sampleRow(frame, self.dtheta_d)

        simulator.calculateStablePDForces(self.skeletonIndex, self.theta_d, self.controlforce)

        self.controlforce.clamp(self.clampMin, self.clampMax)


    def rewindTargetMotion(self, simulator):
        self.deltaTime=-1*simulator.currentTime()



class PoseMaintainer_spherical:

    def __init__(self, skeletonIndex):
        self.theta=m.vectorn()
        self.dtheta=m.vectorn()
        self.theta_d=m.vectorn() # desired q
        self.dtheta_d=m.vectorn() # desired dq
        self.controlforce=m.vectorn()

        self.skeletonIndex=skeletonIndex or 0




    def init(self, skel, simulator, k_p, k_d):
        csize=m.vector3()
        ichara=self.skeletonIndex
        dofInfo=simulator.skeleton(ichara).dofInfo
        csize.y=simulator.numSphericalJoints(ichara)
        csize.x=dofInfo.numDOF() -csize.y*4

        # for the root joint and other 1-DOF joints
        self.kp=m.vectorn(csize.x+csize.y*3)
        self.kd=m.vectorn(csize.x+csize.y*3)
        self.kp.setAllValue(k_p)
        self.kd.setAllValue(k_d)
        
        if dofInfo.hasTranslation(1) :
            # exclude root translation
            self.kp.range(0,3).setAllValue(0)
            self.kd.range(0,3).setAllValue(0)
            self.freeRoot=True


        print ("kp=",self.kp)
        print ("kd=",self.kd)

        self.startFrame=startf
        self.endFrame=endf
        self.currFrame=startf
        self.deltaTime=0

        simulator.initSimulation()
        q=m.vectorn()
        dq=m.vectorn()
        simulator.getSphericalState(self.skeletonIndex, q, dq)

        self.nonQsize=q.size()-csize.y*4
        assert(q.toQuater(self.nonQsize).length()>0.99)
        self.theta_d=q
        self.dtheta_d=dq
        self.dtheta_d.zero()

        simulator.setStablePDparam(self.skeletonIndex, self.kp, self.kd)


    def stepSimul(self, simulator, drawDebugInformation=None):
        simulator.setTau(self.skeletonIndex, self.controlforce)
        if drawDebugInformation :
            simulator.drawDebugInformation()

        simulator.stepSimulation()


    def generateTorque(self, simulator):

        simulator.getSphericalState(self.skeletonIndex, self.theta, self.dtheta)
        simulator.calculateStablePDForces(self.skeletonIndex, self.theta_d, self.controlforce)

        # self.controlforce.clamp(self.clampMin, self.clampMax)


def registerContactPairAll(model, loader, floor, simulator):
    param=m.vectorn()
    if model and 'penaltyForceStiffness' in model:
        param.assign([0.5,0.5, model.penaltyForceStiffness, model.penaltyForceDamp])
    else:
        param.assign([0.5,0.5, 10000, 1000])
    for i in range(1,loader.numBone()-1 +1) :
        for j in range(1, floor.numBone()-1+1):
            bone_i=loader.VRMLbone(i)
            simulator.registerCollisionCheckPair(loader.name(),bone_i.name(), floor.name(), floor.bone(j).name(), param)


class NonlinearController:
    def __init__(self, dim, cdim):
        # dim: # of generalized coordinates
       # D ddot(theta) + C dot(theta) + G (theta)= H u
       self.D=m.matrixn(dim,dim)
       self.C=m.matrixn(dim,dim)
       self.G=m.vectorn(dim)
       self.H=m.matrixn(dim,cdim)
       self.dim=dim
       self.cdim=cdim
       self.dGdTheta_0=m.vectorn(dim)
       self.Gsd=lua.zeros(dim, dim)

       # outputs of update functions
       #: dot(x)=Ax+Bu
       self.invD=m.matrixn(dim,dim)
       self.A=m.matrixn(dim*2,dim*2)
       self.B=m.matrixn(dim*2,cdim)
       self.A.range(0, dim, 0, dim*2).setAllValue(0)
       self.A.range(0, dim, dim, dim*2).diag().setAllValue(1)
       self.B.range(0, dim, 0, cdim).setAllValue(0)

    def _updateSDRE(self, theta):
        # assumes that D, C, G is already calculated.
        #--   self.invD:pseudoInverse(self.D)
        self.invD.ref()[:,:]=np.linalg.inv(self.D.ref())
        dim=self.dim
        cdim=self.cdim

        Gsd=self.Gsd
        G=self.G
        dGdTheta_0=self.dGdTheta_0

        thr=0.0001
        for i in range(Gsd.rows()):
            if theta(i)<thr :
                Gsd.set(i, i, dGdTheta_0(i))
            else:
                Gsd.set(i, i, G(i)/theta(i))

        a3=self.A.range(dim, dim*2, 0, dim)
        a3.mult(self.invD, Gsd)
        a3.radd(-1.0)

        a4=self.A.range(dim, dim*2, dim, dim*2)
        a4.mult(self.invD, self.C)
        a4.radd(-1.0)

        b2=self.B.range(dim, dim*2,0, cdim)
        b2.mult(self.invD, self.H)

    def oneStep(self, x,U,dt,Q,R,K, xd, tau=None):
        dim=self.dim
        self.cf=self.calcControlForce(x,U,Q,R,K,xd)
        cf=self.cf
        return self.oneStepRaw(x,cf,dt,Q,R,K, xd, tau)
    def oneStepRaw(self, x,cf,dt,Q,R,K, xd, tau):

        dim=self.dim

        #- use D, C, G : the most accurate
        #- using D ddtheta + C dtheta + G = Hu
        theta=x.range(0, dim,0,1)
        dtheta=x.range(dim, dim*2,0,1)

        Dddtheta=None
        Dddtheta=self.H*cf  - self.G.column() - self.C*dtheta

        if tau !=None:
            Dddtheta.radd(tau.column())
        ddtheta=self.invD*Dddtheta
        dtheta.radd(ddtheta*dt)
        theta.radd(dtheta*dt)

        return ddtheta

    def calcControlForce(self, x,U,Q,R,K, xd):
        dim=self.dim
        self.updateSDRE(x.column(0).range(0,dim), x.column(0).range(dim, dim*2))

        #useSDRE=False
        #if useSDRE :
        #      K.assign(LQR_wrap(self.A,self.B,Q,R))


        U.assign(K*xd)

        return U-K*x
class SDS(NonlinearController):
    def __init__(self, mass, B, k, q, qd,dt, usePreset=False):
        self.M=mass
        self.b=B
        self.k=k

        self.t=0
        self.dt=dt
        # D ddot(theta) + C dot(theta) + G (theta)= H u
        super(SDS, self).__init__(1,1)
        self.dGdTheta_0.set(0, self.k/self.M)
        unused=lua.vec([0])
        self.updateSDRE(unused, unused)

        self.Q=m.matrixn(2,2)
        self.Q.setAllValue(0)
        self.Q.diag().assign([q,qd])

        self.R=lua.eye(1);

        # python-control
        try:
            if usePreset:
                self.ct=None
                self.useKpreset(q)
            else:
                import control as ct
                self.ct=ct
                self.K, S, E=ct.lqr(self.A.ref(),self.B.ref(),self.Q.ref(),self.R.ref())
                self.K=lua.mat(self.K)
        except:
            assert(mass==60)
            assert(B==0.001)
            assert(k==1)
            assert(qd==0.0)
            self.useKpreset(q)

        self.x=m.matrixn(2,1)
        self.x.setAllValue(0)
        self.xd=m.matrixn(2,1)
        self.xd.setAllValue(0)
        self.U=lua.mat(1,1,0)
    def useKpreset(self, q):

        src=[math.pow(10,i) for i in range(5,13)]
        tgt=[float(i) for i in range(0,8)]
        index=lua.F('sop.piecewiseLinearMap', q, src, tgt)
        
        K=lua.mat(7,2,
                  262.685, 127.410,  # 1e5
                  942.739, 281.657,  # 1e6
                  3103.828, 553.238, # 1e7
                  9941.174, 1033.866, #1e8
                  31563.832, 1887.117, #1e9
                  99941.017, 3403.601, #1e10
                  316168.772, 6099.859)

        self.K=lua.zeros(1,2)
        K.sampleRow(index, self.K.row(0))

    def updateSDRE(self, theta, dtheta):
        invM=1/self.M
        self.D.set(0,0,1)
        self.C.set(0,0,self.b*invM)
        self.G.set(0,self.k*invM)
        self.H.set(0,0,1*invM)
        self._updateSDRE(theta)
	
    def updateCoef(self, B, k,Q):
        self.b=B
        self.k=k
        unused=lua.vec([0])
        self.updateSDRE(unused, unused)
        self.dGdTheta_0.set(0, self.k/self.M)

        self.Q.set(0,0,Q)

        try:
            self.K, S, E=self.ct.lqr(self.A.ref(),self.B.ref(),self.Q.ref(),self.R.ref())
            self.K=lua.mat(self.K)
        except:
            assert(B==0.001)
            assert(k==1)
            self.useKpreset(Q)

    def singleStep(self):
        self.oneStep(self.x, self.U, self.dt, self.Q, self.R, self.K, self.xd)
        self.t=self.t+self.dt
