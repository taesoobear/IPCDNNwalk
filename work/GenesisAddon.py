libcalab=None
import genesis as gs
import numpy as np
import time,pdb
# a simple taesooLib style wrapper. assumes Euler joints (except for the free root joint)
# this class will be moved to controlmodule.py later
# all funtions runs on cpu so this class should not be used for training purposes.
class GenesisSim:
    def __init__(self, simLoaders, scene_file, timestep, g, _libcalab):
        """ input: 1. simLoaders (a list of loaders or an mjcf xml file name)
                   2. timestep
            output: scene_file (can be None)
        """ 

        global libcalab
        libcalab=_libcalab
        if isinstance(simLoaders, list):
            libcalab.RE.writeMujocoXML(simLoaders, scene_file, {'groundPlane':False})
        else:
            assert(scene_file==None)
            scene_file=simLoaders
            simLoaders=libcalab.RE.MujocoLoader(scene_file)
            if isinstance(simLoaders, libcalab.m.VRMLloader):
                simLoaders=[simLoaders]
        #if torch.cuda.is_available() or platform.system()=='Darwin':
        #    gs.init(backend=gs.gpu)
        #else:
        gs.init(backend=gs.cpu)  # cpu runs much faster when a render-window exists.

        self.scene=gs.Scene(
            sim_options=gs.options.SimOptions( 
                dt=timestep,
                gravity=(-g.x, -g.y,-g.z),
            ),
        )
        self.entityAll=self.scene.add_entity(gs.morphs.MJCF(file=scene_file))
        self.scene.build()

        self.loaders=simLoaders
        self.fkSolvers=[] # only for visualization
        self.startq=[]
        self.startv=[]
        self.startLinkIndex=[]
        self.indexMap=[]
        self.timestep=timestep
        startq=0
        startv=0
        self.startLinkIndex=[None]*len(simLoaders)
        startLinkIndex=0
        for iloader, loader in enumerate(simLoaders):
            if loader.dofInfo.numDOF()==0:
                self.startLinkIndex[iloader]=0 # world
        startLinkIndex=1

        for iloader, loader in enumerate(simLoaders):
            if loader.dofInfo.numDOF()>0:
                self.startLinkIndex[iloader]=startLinkIndex
                startLinkIndex+=loader.numBone()-1

        for iloader, loader in enumerate(simLoaders):
            print(loader.name(), self.entityAll.links[self.startLinkIndex[iloader]].name)

        for loader in simLoaders:
            self.fkSolvers.append(libcalab.m.BoneForwardKinematics(loader))
            self.startq.append(startq)
            self.startv.append(startv)
            iloader=len(self.startq)-1
            self.indexMap.append(self.getIndexMap(self.entityAll, iloader, loader))
            print(iloader, loader.name(), loader.dofInfo.numDOF(), self.indexMap[iloader])
            if loader.dofInfo.numDOF()>0:
                assert(startq==self.indexMap[iloader][0][0])
                assert(startv==self.indexMap[iloader][1][0])
            startq+=loader.dofInfo.numDOF()
            startv+=loader.dofInfo.numActualDOF()
        self.qpos=libcalab.lua.vec(self.entityAll.get_qpos())
        self.qvel=libcalab.lua.vec(self.entityAll.get_dofs_velocity())
        self.ctrl=libcalab.lua.zeros(self.qvel.size())
        self.qposDirty=False
        self.qvelUpToDate=True
        self.qvelModified=False
        assert(startq==self.qpos.size())
        assert(startv==self.qvel.size())
    def setCtrl(self, iloader, torque):
        loader=self.loaders[iloader]
        if loader.VRMLbone(1).HRPjointType(0)==0 :
            self.ctrl.ref()[self.indexMap[iloader][1][6:]]=torque.array
        else:
            self.ctrl.ref()[self.indexMap[iloader][1]]=torque.array

    def stepSimulation(self, rendering_step=None):

        if self.qposDirty:
            self.entityAll.set_qpos(self.qpos.ref())
            self.qposDirty=False
        if self.qvelModified:
            self.entityAll.set_dofs_velocity(self.qvel.ref())
            self.qvelModified=False

        if rendering_step is None:
            niter=1
        else:
            niter=int(rendering_step/self.timestep)

        timer=libcalab.m.Timer()
        timer.start()
        self.entityAll.control_dofs_force(self.ctrl.ref())
        self.ctrl.zero()
        for i in range(0, niter ) :
            self.scene.step()

        libcalab.RE.output2("sim time(ms)", timer.stop2()/1e3)

        timer.start()
        a=time.perf_counter()
        self.qpos=self.entityAll.get_qpos()  # extremely slow here (only when using gpu), probably because of rendering.
        b=time.perf_counter()
        libcalab.RE.output2("get state time(ms)", timer.stop2()/1e3)
        print('get_qpos (ms)', (b-a)*1000.0)
        self.qpos=libcalab.lua.vec(self.qpos)   # not slow at all.
        #self.qvel=lua.vec(self.entityAll.get_dofs_velocity()) slowwwww
        self.qvelUpToDate=False

        for iloader in range(0, len(self.loaders)):
            posedof=self.getPoseDOF(iloader)
            self.fkSolvers[iloader].setPoseDOF(posedof)


    def pose(self, iloader):
        return self.fkSolvers[iloader]

    def setPoseDOF(self, iloader, TLpose): # setTLpose(robot, taesooLibPose.ref()) 여기서 .ref()는 taesooLibPose의 타입이 vectorn인 경우 필요. 
        self.qpos.ref()[self.indexMap[iloader][0]]=TLpose.ref()
        self.fkSolvers[iloader].setPoseDOF(TLpose)
        self.qposDirty=True

    def getPoseDOF(self, iloader):
        return libcalab.lua.vec(self.qpos.ref()[self.indexMap[iloader][0]])
    def setLinkPos(self, iloader, TLpose):
        self.setPoseDOF( iloader,TLpose)
    def getLinkPos(self, iloader, TLpose):
        return self.getPoseDOF(self, iloader)

    def getLinkVel(self, iloader):
        startv=self.startv[iloader]
        loader=self.loaders[iloader]
        ndof=loader.dofInfo.numActualDOF()

        qvel=self.qvel.ref()
        dposedof=libcalab.m.vectorn(ndof+1)
        if loader.VRMLbone(1).HRPjointType(0)==0 :
            # free joint 
            dposedof.ref()[1:]=qvel[self.indexMap[iloader][1]]
            dposedof.ref()[0:3]=qvel[startv:startv+3]
            dposedof.ref()[4:7]=qvel[startv+3:startv+6]
        else:
            dposedof.array[:]=qvel[self.indexMap[iloader][1]]
        return dposedof

    def setLinkVel(self, iloader, dposedof):
        startv=self.startv[iloader]
        loader=self.loaders[iloader]
        ndof=loader.dofInfo.numActualDOF()

        qvel=self.qvel.array
        if loader.VRMLbone(1).HRPjointType(0)==0 :
            # free joint 
            qvel[self.indexMap[iloader][1]]=dposedof.ref()[1:]
            qvel[startv:startv+3]=dposedof.ref()[0:3]
            qvel[startv+3:startv+6]=dposedof.ref()[4:7]
        else:
            qvel[self.indexMap[iloader][1]]=dposedof.ref()

        self.qvelModified=True
    def numSkeleton(self):
        return len(self.loaders)
    def sync(self, skins):
        for i in range(0, self.numSkeleton()):
            skins[i].setSamePose(self.pose(i))

    def flatten(self, dofs_idx2d):
        out=[]
        for e in dofs_idx2d:
            if isinstance(e, int):
                out+=[e]
            else:
                out+=e
        return out
    def getIndexMap(self,robot, iloader, mLoader):
        # 가끔 genesis가 xml에 적혀있는 본의 순서를 무시하고 로딩하는 경우가 있어서 아래와 같은 index매핑 필요.
        dofs_TL_qidx2d=[]
        dpose_dofs_TL_qidx2d=[]
        for i in range(1, mLoader.numBone()):
            if mLoader.dofInfo.numDOF(i)>0:
                if mLoader.numBone()>2:
                    link=robot.get_link(mLoader.bone(i).name())
                else:
                    link=robot.links[self.startLinkIndex[iloader]]

                print('same?',iloader, self.startLinkIndex[iloader], mLoader.bone(i).name(), link.name)
                for j in link.joints:
                    dofs_TL_qidx2d.append(j.qs_idx_local)
                    dpose_dofs_TL_qidx2d.append(j.dofs_idx_local)

        #print(dofs_TL_qidx2d)
        #print(dpose_dofs_TL_qidx2d)
        #pdb.set_trace()
        # quaternion dofs (qpos) ordered according to taesooLib convention 
        return self.flatten(dofs_TL_qidx2d), self.flatten(dpose_dofs_TL_qidx2d)

