import work.luamodule as lua  # see luamodule.py
import work.rendermodule as RE # see rendermodule.py
import work.controlmodule as control
import random
import mujoco, pdb
import numpy as np
m=lua.taesooLib()

from . import mmsto_config

class BallSim:
    def startPassiveMode(self, mLoader_full):
        # ragdoll 시작하면 gain 늘리기. 
        self.pdservo.init(mLoader_full, self.sim, 50, 5)

    def __init__(self, mLoader_full, otherLoaders=None):
        self.simFrameRate=360 # for balls
        self.numBalls=40
        self.loader=mLoader_full
        self.activeBalls=[]
        self.n_contact=0
        self._lastPose=None
        self._lastDPose=None
        self._passiveModeStarted=False
        self.debugMode=False
        self.mocap_frame_rate=30
        self.pdservo=None
        self.ballSkins=None
        self.simLoaders=[]
        self.floorBox=None
        self.sim=None
        self.characterIndex=None
        self.ballBodies=None
        self.humanBodies=None
        self.useTerrain=otherLoaders!=None
        self.ballLoader=RE.WRLloader("""
                                return {
                                    body ={
                                        jointType ="free",
                                        mass =2,
                                        name ="root",
                                        translation =vector3(0,0,0),
                                        geometry ={
                                            {
                                                "Ellipsoid",
                                                rotation =quater(1,0,0,0),
                                                size =vector3(0.1,0.1,0.1),
                                                translation =vector3(0,0,0),
                                            },
                                        },
                                    },
                                    name ="small_sphere",
                                }
                                """)
        self.ballSkins=lua.dynamic_list() # use edict() instead for a named array (dictionary).

        self.simLoaders=[]

        simLoaders=self.simLoaders

        floorBox=m.Geometry()
        floorBox.initBox(m.vector3(100, 0.2, 100))
        floorBoxHeight=-0.1+0.02
        floorLoader=m.VRMLloader(floorBox, True)
        floorLoader.setPosition(m.vector3(0,floorBoxHeight,0))
        simLoaders.append(floorLoader) # character 0

        ballSkins=self.ballSkins
        numBalls=self.numBalls
        ballLoader=self.ballLoader
        for i  in range(1, numBalls +1):
            simLoaders.append(ballLoader) 
            ballSkins[i]=RE.createSkin(ballLoader)
            ballSkins[i].setScale(100)
            ballSkins[i].setMaterial('white')
            ballSkins[i].setVisible(False)

        simLoaders.append(mLoader_full)
        self.characterIndex=len(simLoaders)-1

        if otherLoaders!=None:
            for i in range(1, len(otherLoaders)):
                print(otherLoaders[i].name())
                print(otherLoaders[i].bone(1).name())
                simLoaders.append(otherLoaders[i])
            


        self.pdservo=control.PoseMaintainer(self.characterIndex)
        self.sim=control.MujocoSim(simLoaders, '__temp_ragdoll_scene.xml', 1.0/self.simFrameRate)
        self.sim.setGVector(m.vector3(0,9.8,0))
        self.pdservo.init(mLoader_full, self.sim, 200, 15)
        sim=self.sim

        self.ballBodies=range(sim.start_body[1], sim.start_body[numBalls+1])
        self.humanBodies=range(sim.start_body[self.characterIndex], sim.start_body[self.characterIndex+1])
        if self.debugMode:
            mSkinCollider=RE.createSkin(mLoader_full)
            mSkinCollider.setScale(100)
            mSkinCollider.setMaterial('lightgrey_transparent')
            mSkinCollider.setVisible(True)
            mSkinCollider.setTranslation(-100,0,0)
            self.mSkinCollider=mSkinCollider
        else:
            self.mSkinCollider=None



    def simulateBalls(self, currPredictedPose, kinematic=True, _desiredPose=None, srb_obs=None):
        _lastPose=self._lastPose
        _lastDPose=self._lastDPose
        sim=self.sim 
        ballBodies=self.ballBodies 
        humanBodies=self.humanBodies
        activeBalls=self.activeBalls
        sim=self.sim

        force=np.array([0,0,0,0,0,0]) # force_ZUP to feed back to SRB
        dpose=lua.zeros(currPredictedPose.size())
        if _lastPose :
            mot=m.matrixn(2, _lastPose.size())
            mot.row(0).assign(_lastPose)
            mot.row(1).assign(currPredictedPose)
            dmot=lua.F('MotionDOF.calcDerivative', mot, self.mocap_frame_rate)
            dpose.assign(dmot.row(0))
            _lastDPose=dpose.slice(0,7).copy()

        if kinematic:
            sim.setPoseDOF(self.characterIndex, currPredictedPose)
            sim.setDPoseDOF(self.characterIndex, dpose)
            sim.initSimulation()
        niter=int(self.simFrameRate/self.mocap_frame_rate)

        loader=self.loader
        hasForce=m.boolN(loader.numBone())
        for i in range(niter ):
            self.pdservo.applyPDtorque(sim, _desiredPose or currPredictedPose)
            sim.stepSimulation()

            if srb_obs!=None:

                SRB_obs_dim=mmsto_config.SRB_obs_dim
                # YUP
                srb_root=srb_obs.toTransf(SRB_obs_dim)
                inv_srb_root=srb_root.inverse()
                dq=srb_obs.ref()[SRB_obs_dim+7:SRB_obs_dim+7+6]

                # print contact information
                #print('number of contacts', sim.data.ncon)

                for i in range(self.n_contact):
                    RE.erase('Arrow', f'contact{i}')

                self.n_contact=sim.data.ncon
                complianceFactor=0.5


                for i in range(sim.data.ncon):

                    # Note that the contact array has more than `ncon` entries,
                    # so be careful to only read the valid entries.
                    contact = sim.data.contact[i]
                    if contact.dist<0:
                        #print('contact', i, contact)
                        #print('dist', contact.dist)
                        body_id = sim.model.geom_bodyid[contact.geom1]
                        body_id2 = sim.model.geom_bodyid[contact.geom2]

                        if body_id in ballBodies and body_id2 in humanBodies:
                            body_name = mujoco.mj_id2name(sim.model, mujoco.mjtObj.mjOBJ_BODY, body_id)
                            body_name2 = mujoco.mj_id2name(sim.model, mujoco.mjtObj.mjOBJ_BODY, body_id2)


                            ichara, tree_index=sim.getCharacterAndTreeIndex(body_id2)
                            hasForce.set(tree_index, True)


                            # There's more stuff in the data structure
                            # See the mujoco documentation for more info!
                            geom2_body = sim.model.geom_bodyid[sim.data.contact[i].geom2]
                            if self.debugMode:
                                print( body_name, body_name2)
                                print(' Contact force on geom2 body', sim.data.cfrc_ext[geom2_body])
                                print('norm', np.sqrt(np.sum(np.square(sim.data.cfrc_ext[geom2_body]))))
                            # Use internal functions to read out mj_contactForce
                            c_array = np.zeros(6, dtype=np.float64)
                            mujoco.mj_contactForce(sim.model, sim.data, i, c_array)
                            if self.debugMode:
                                print('c_array', c_array)
                                print(contact.frame)
                            CF=RE.toQuater(contact.frame)

                            p=RE.toVector3(contact.pos)
                            f=CF.inverse()*RE.toVector3(c_array)

                            lpos=(inv_srb_root*p)*complianceFactor

                            RE.setVec3(force[0:3], RE.toVector3(force[0:3])+f.YtoZ())
                            RE.setVec3(force[3:6], RE.toVector3(force[3:6])+lpos.YtoZ().cross(f.YtoZ()))
                            
                            if self.debugMode:
                                RE.draw('Arrow', p*100, p*100+f, f'ballcontact{i}')
        for v in activeBalls:
            ballPose=sim.getPoseDOF(v)
            self.ballSkins[v].setPoseDOF(ballPose)

            d=ballPose.toVector3(0).distance(currPredictedPose.toVector3(0))

            ty=0
            if self.useTerrain:
                bt=sim.getTerrainHeight(ballPose.toVector3(0))
                ty=bt.y

            if ballPose(1)<ty-0.2 and d>3:
                activeBalls.remove(v)
                self.ballSkins[v].setVisible(False)
            elif d>5:
                activeBalls.remove(v)
                self.ballSkins[v].setVisible(False)
            elif ballPose(1)<ty+0.2 and d>2.9:
                self.ballSkins[v].setMaterial('lightgrey_verytransparent')
            elif ballPose(1)<ty+0.2 and d>2.8:
                self.ballSkins[v].setMaterial('lightgrey_transparent')

        velCon=[]

        if float(force.dot(force))>1e-3:

            for i in range(1, self.loader.numBone()):
                if hasForce(i):
                    body_id=sim.getBodyId(self.characterIndex, i)
                    p=RE.toVector3(sim.data.xpos[body_id])
                    v_array = np.zeros(6, dtype=np.float64)
                    mujoco.mj_objectVelocity(sim.model, sim.data,mujoco.mjtObj.mjOBJ_BODY,body_id ,v_array, 0)
                    v=RE.toVector3(v_array[3:6])
                    velCon.append([i, self.loader.VRMLbone(i).localCOM(), v])
                    #RE.draw('Arrow', p*100, p*100+v*100, f'ballcontact{i}')

        self.velCon=velCon
        return force/float(niter)

    def throwBall(self, projectToGround=None):
        _lastPose=self._lastPose 
        _lastDPose=self._lastDPose
        activeBalls=self.activeBalls
        ballSkins=self.ballSkins

        numBalls=self.numBalls
        for i  in range(1, numBalls +1): # ball index starts at 1
            if i not in activeBalls:
                activeBalls.append(i) 
                zero=lua.zeros(7)
                zero.set(0, 0+random.uniform(-0.2, 0.2)) 
                zero.set(1, 1.5)
                zero.set(2, 3+random.uniform(-0.2, 0.4))
                zero.set(3, 1) # quater w

                if _lastPose :
                    rootPos=_lastPose.toVector3(0)
                    if projectToGround!=None:
                        projectToGround(rootPos)
                    else:
                        rootPos.y=0
                    zero.setVec3(0, zero.toVector3(0)+rootPos)

                # ball
                self.sim.setPoseDOF(i,  zero)
                ballSkins[i].setPoseDOF(zero)
                ballSkins[i].setVisible(True)
                ballSkins[i].setMaterial('white')

                # ball velocity
                if _lastPose and _lastDPose:
                    rootVel=_lastPose.toQuater(3)*_lastDPose.toVector3(0)
                else:
                    rootVel=m.vector3(0,0,0)
                
                RE.output('rootVel', rootVel)
                zero.setVec3(0,m.vector3(0,0,-10)+rootVel)
                self.sim.setDPoseDOF(i, zero)

                self.sim.initSimulation()
                return
        print('no balls left')



    def stepSimul(self, mRobotSkin, currPredictedPose, srb_obs, passive=None):

        if passive==None:
            passive=srb_obs
            srb_obs=None

        f=None
        if passive:
            if not self._passiveModeStarted:
                f=self.simulateBalls(currPredictedPose, True, None, srb_obs)
                self._passiveModeStarted=True
            else:
                f=self.simulateBalls(currPredictedPose, False, None, srb_obs)
            pose=self.sim.getPoseDOF( self.sim.numSkeleton()-1)
            mRobotSkin.setPoseDOF(pose)
            self._lastPose=pose.copy()

        else:
            f=self.simulateBalls(currPredictedPose, True, None, srb_obs)
            self._lastPose=currPredictedPose.copy()
        if self.mSkinCollider:
            self.mSkinCollider.setPoseDOF(_lastPose)
        return f

