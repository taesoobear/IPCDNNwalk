libcalab=None
import genesis as gs
import numpy as np
import time
# a simple taesooLib style wrapper. assumes Euler joints (except for the free root joint)
# this class will be moved to controlmodule.py later
class GenesisSim:
    def __init__(self, simLoaders, scene_file, timestep, g, _libcalab):
        global libcalab
        libcalab=_libcalab
        libcalab.RE.writeMujocoXML(simLoaders, scene_file, {'groundPlane':False})
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
        self.timestep=timestep
        startq=0
        startv=0
        for loader in simLoaders:
            self.fkSolvers.append(libcalab.m.BoneForwardKinematics(loader))
            self.startq.append(startq)
            self.startv.append(startv)
            startq+=loader.dofInfo.numDOF()
            startv+=loader.dofInfo.numActualDOF()
        self.qpos=libcalab.lua.vec(self.entityAll.get_qpos())
        self.qvel=libcalab.lua.vec(self.entityAll.get_dofs_velocity())
        self.qposDirty=False
        self.qvelUpToDate=True
        self.qvelModified=False
        assert(startq==self.qpos.size())
        assert(startv==self.qvel.size())

    def stepSimulation(self, rendering_step):

        if self.qposDirty:
            self.entityAll.set_qpos(self.qpos.ref())
            self.qposDirty=False
        if self.qvelModified:
            self.entityAll.set_dofs_velocity(self.qvel.ref())
            self.qvelModified=False

        niter=int(rendering_step/self.timestep)

        timer=libcalab.m.Timer()
        timer.start()
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
            posedof=self.getLinkPos(iloader)
            self.fkSolvers[iloader].setPoseDOF(posedof)

    def pose(self, iloader):
        return self.fkSolvers[iloader]
    def setLinkPos(self, iloader, posedof):
        startq=self.startq[iloader]
        ndof=self.loaders[iloader].dofInfo.numDOF()
        self.qpos.ref()[startq:startq+ndof]=posedof.ref()
        self.fkSolvers[iloader].setPoseDOF(posedof)
        self.qposDirty=True
    def getLinkPos(self, iloader):
        startq=self.startq[iloader]
        ndof=self.loaders[iloader].dofInfo.numDOF()
        return self.qpos.range(startq,startq+ndof)

    def setLinkVel(self, iloader, dposedof):
        startv=self.startv[iloader]
        loader=self.loaders[iloader]
        ndof=loader.dofInfo.numActualDOF()

        qvel=None
        if loader.VRMLbone(1).HRPjointType(0)==0 :
            # free joint 
            qvel=np.array(dposedof.ref()[1:])
            qvel[0:3]=dposedof.ref()[0:3]
            qvel[3:6]=dposedof.ref()[4:7]
        else:
            qvel=np.array(dposedof.ref())

        self.qvel.ref()[startv:startv+ndof]=qvel
        self.qvelModified=True
    def numSkeleton(self):
        return len(self.loaders)
    def sync(self, skins):
        for i in range(0, self.numSkeleton()):
            skins[i].setSamePose(self.pose(i))
