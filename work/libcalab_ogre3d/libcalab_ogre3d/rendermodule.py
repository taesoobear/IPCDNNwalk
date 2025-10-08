
import os
import pdb # use pdb.set_trace() for debugging
#import code # or use code.interact(local=dict(globals(), **locals())) for debugging. see below.
import numpy as np
from easydict import EasyDict as edict # pip3 install easydict
try:
    import settings
except:
    try:
        import work.settings as settings
        settings.relativeMode=True
    except:
        from . import default_settings as settings
        settings.relativeMode=True

m=settings.mlib
lua=settings.lua
assert(m is not None)
assert(lua is not None)
doNotGarbageCollect=[] # list of instances which should not be garbage collected. (e.g. loaders)

# import all three in a line.
# m,lua,control=RE.defaultModules()
def defaultModules():
    return settings.mlib, settings.lua, settings.control

def addPanel(signal):
    if isinstance(signal, m.boolN):
        m.motionPanel().scrollPanel().addPanel(signal, m.CPixelRGB8(255,255,0))
def lookAt(pose, options=None, **kwargs):
    if options is not None:
        lua.M(m.viewpoint(), 'lookAt', pose, options)
    else:
        lua.M(m.viewpoint(), 'lookAt', pose, kwargs)

def filename(full_path):
    return lua.F('os.filename', full_path)
def createLoader(filename ,_options=None):
    if filename[-3:]=='wrl' :
        return edict({ 'loader':m.VRMLloader(filename)})

def toTransf(anyvec_ori, anyvec_pos=None):
    if len(anyvec_ori)==7:
        return m.transf(toQuater(anyvec_ori[3:]), toVector3(anyvec_ori[0:3]))
    return m.transf(toQuater(anyvec_ori), toVector3(anyvec_pos))
def toQuater(anyvec):
    if len(anyvec)==9:
        mat=m.matrix3()
        mat.ref1D()[:]=anyvec
        return mat.toQuater()
    elif isinstance(anyvec, np.ndarray) and len(anyvec.shape)==2:
        mat=m.matrix3()
        mat.ref()[:,:]=anyvec
        return mat.toQuater()
    return m.quater(anyvec[0], anyvec[1], anyvec[2], anyvec[3])
# (x,y,z,w) format
def V4toQuater(anyvec):
    return m.quater(anyvec[3], anyvec[0], anyvec[1], anyvec[2])
def toVector3(anyvec):
    return m.vector3(anyvec[0], anyvec[1], anyvec[2])

def isFileExist(fname):
    import os.path
    return os.path.isfile(fname)
def setQuater(anyvec,q ):
    anyvec[0]=q.w
    anyvec[1]=q.x
    anyvec[2]=q.y
    anyvec[3]=q.z
def setVec3(anyvec,v ):
    anyvec[0]=v.x
    anyvec[1]=v.y
    anyvec[2]=v.z
def setTransf(anyvec, tf):
    setVec3(anyvec, tf.translation)
    setQuater(anyvec[3:], tf.rotation)
def saveTable(tbl=dict, filename=str):
    lua.F('util.saveTable', tbl, filename)
def loadTable(filename=str):
    return lua.F('util.loadTable', filename)

# ZUP to YUP
def quater_ZtoY(q):
    return m.quater(q.w, q.y, q.z, q.x)
def vector3_ZtoY(v):
    return m.vector3(v.y, v.z, v.x)
def transf_ZtoY(t):
    return m.transf(t.rotation.ZtoY(), t.translation.ZtoY())

def ZtoY(v):
    mat=v
    if isinstance(v, m.matrixn):
        mat=v.ref()

    if mat.shape[1]==3:
        mat[:, [0,1,2]]=mat[:, [1,2,0]]
    elif mat.shape[1]==4:
        mat[:, [1,2,3]]=mat[:, [2,3,1]]
    else:
        pdb.set_trace()


# YUP to ZUP
def quater_YtoZ(q):
    return m.quater(q.w, q.z, q.x, q.y)
def vector3_YtoZ(v):
    return m.vector3(v.z, v.x, v.y)
def transf_YtoZ(t):
    return m.transf(t.rotation.YtoZ(), t.translation.YtoZ())

def boolN_numpy(self):
    out=np.ones((self.size()), dtype=bool)
    for i in range(self.size()):
        out[i]=self(i)
    return out
m.boolN.numpy=boolN_numpy
def tempFunc(self):
    out=m.intvectorn(self.size())
    out.setAllValue(0)
    for i in range(self.size()):
        if self(i):
            out.set(i,1)
    return out
m.boolN.asInt=tempFunc
m.quater.ZtoY=quater_ZtoY
m.vector3.ZtoY=vector3_ZtoY
m.transf.ZtoY=transf_ZtoY
m.quater.YtoZ=quater_YtoZ
m.vector3.YtoZ=vector3_YtoZ
m.transf.YtoZ=transf_YtoZ


def tempFunc(self, b):
    out=m.boolN()
    out._or(self,b)
    return out
m.boolN.bitwiseOR=tempFunc

def tempFunc(self, prevFrameRate, newFrameRate):
    lua.M(self, 'resample', prevFrameRate, newFrameRate)
m.MotionDOF.resample=tempFunc

def tempFunc(self, b):
    out=m.boolN()
    out._and(self,b)
    return out
m.boolN.bitwiseAND=tempFunc
def tempFunc(self):
    out=m.intIntervals()
    out.runLengthEncode(self)
    return out
m.boolN.runLengthEncode=tempFunc


def changeBoolChartPrecision(n):
    lua.F('Imp.ChangeBoolChartPrecision',n)

def createMotionLoaderExt(fn):
    loader=m.createMotionLoaderExt_cpp(fn)
    doNotGarbageCollect.append(loader)
    return loader
def setViewZup():
    lua.F('setViewYUp', False)
def setViewYup(yup):
    lua.F('setViewYUp', yup)
def viewpoint():
    return m.viewpoint()
def console():
    lua.dostring('dbg.console()')
def ogreSceneManager():
    return m.ogreSceneManager()
def ogreRootSceneNode():
    return m.ogreRootSceneNode()
def createChildSceneNode(a, b):
    return m.createChildSceneNode(a,b)
def createEntity(*args):
    return m.createEntity(*args)
def removeEntity(a):
    return m.removeEntity(a)
def namedDraw(*args):
    lua.F(('dbg', 'namedDraw'),*args) 
def msgBox(msg):
    lua.F(('util','msgBox'),msg)
def draw(*args):
    if isinstance(args[1], m.vector3N):
        args=list(args)
        v=args[1] # to prevent immediate garbage collection 
        args[1]=v.matView()
        lua.F(('dbg', 'draw'),*args) 
        return
    lua.F(('dbg', 'draw'),*args) 
# also call delayedDrawTick once every frame
def delayedDraw(*args):
    lua.F(('dbg', 'delayedDraw'),*args) 
def delayedErase(*args):
    lua.F(('dbg', 'delayedErase'),*args) 
def delayedSetPoseDOF(skin, delay, theta):
    lua.F(('dbg', 'delayedSetPoseDOF'),skin, delay, theta)
def delayedDrawTick():
    lua.F(('dbg', 'delayedDrawTick'))
def erase(*args):
    lua.F(('dbg', 'erase'),*args) 
def drawTraj(objectlist, matrix, nameid=None, color='solidred', thickness=0, linetype="LineList"):
    if not nameid: nameid=RE.generateUniqueName()
    objectlist.registerObject(nameid, linetype, color, matrix, thickness)
def timedDraw(*args):
    lua.F(('dbg', 'timedDraw'),*args) 
def drawBillboard(*args):
    if isinstance(args[0], m.vector3N):
        args=list(args)
        v=args[0] # to prevent immediate garbage collection 
        args[0]=v.matView()
        lua.F(('dbg', 'drawBillboard'),*args) 
        return
    lua.F(('dbg', 'drawBillboard'),*args) 
def eraseAllDrawn():
    lua.F(('dbg', 'eraseAllDrawn'))
def updateBillboards(fElapsedTime):
    lua.F(('dbg', 'updateBillboards'),fElapsedTime)
def createColorBuffer(mesh):
    lua.require("Kinematics/meshTools")
    lua.M0(mesh,'createColorBuffer')
def createTextureBuffer(mesh):
    lua.require("Kinematics/meshTools")
    lua.M0(mesh,'createTextureBuffer')
m.Mesh.createColorBuffer=createColorBuffer # register
m.Mesh.createTextureBuffer=createTextureBuffer # register
def motionPanel():
    return m.motionPanel()
def removeEntityByName(name):
    n=m.getSceneNode(name)
    if n:
        m.removeEntity(n)
def turnOffSoftShadows():
    lua.F(("RE","turnOffSoftShadows"))
def renderOneFrame(check):
    return m.renderOneFrame(check)
def output(key, *args):
    m._output(key,str(args),1)

def output2(key, *args):
    m._output(key,str(args),2)


class Skins:
    def __init__(self, loaders, options=None):
        self.skins=[]
        for loader in loaders :
            self.skins.append(createSkin(loader, options))
    def __len__(self):
        return len(self.skins)
    def __getitem__(self, i):
        return self.skins[i]
    def setScale(self, x,y=None,z=None):
        if y!=None:
            for skin in self.skins :
                skin.setScale(x,y,z)
        else:
            for skin in self.skins :
                skin.setScale(x)
    def setTranslation(self, x,y=None,z=None):
        if y!=None:
            for skin in self.skins :
                skin.setTranslation(x,y,z)
        else:
            for skin in self.skins :
                skin.setTranslation(x)
    def setMaterial(self, mat):
        for skin in self.skins :
            skin.setMaterial(mat)
    def setSamePose(self, sim):
        for i, skin in enumerate(self.skins):
            skin.setSamePose(sim.pose(i))



def createSkin(loader, options=None):
    if isinstance(loader, m.VRMLloader):
        return m.createVRMLskin(loader, False)
    elif isinstance(loader, edict):
        return createSkin(loader.loader)
    elif isinstance(loader, FBXloader):
        return createFBXskin(loader, options)
    elif isinstance(loader, list):
        return Skins(loader, options)

    return m.createSkin(loader)
def loadMotionDOF(loader, motionDOFfile):
    if isinstance(loader, edict):
        loader=loader.loader
    return m.MotionDOFcontainer(loader.dofInfo, motionDOFfile)

def tempFunc(self, motdof):
    pn=self.numFrames()
    self.resize(self.numFrames()+motdof.numFrames())
    self.mot.range(pn, self.numFrames()).assign(motdof)
    self.discontinuity.resize(self.numFrames())
    self.discontinuity.set(pn, True)
m.MotionDOFcontainer.concat=tempFunc

def tempFunc(self, a, b):
    lua.M(self,'gsub', a, b)
m.TStrings.gsub=tempFunc

def tempFunc(self, pose6D):
    lua.M(self, 'setPose6D', pose6D)
m.MotionLoader.setPose6D=tempFunc
def tempFunc(self):
    return lua.M(self, 'getPose6D')
m.MotionLoader.getPose6D=tempFunc

def tempFunc(self):
    return lua.M(self, 'ZtoY') 
m.VRMLloader.ZtoY=tempFunc
def tempFunc(self):
    return lua.M(self, 'YtoZ') 
m.VRMLloader.YtoZ=tempFunc
def tempFunc(self, posemap):
    lua.M(self, 'setPoseMap', posemap)
m.MotionLoader.setPoseMap=tempFunc

def tempFunc(self):
    return lua.M(self, 'getPoseMap')
m.MotionLoader.getPoseMap=tempFunc

def createFBXskin(fbx, drawSkeleton=None, **kwargs):
    if drawSkeleton:
        return FBXskin(fbx, drawSkeleton)
    else:
        return FBXskin(fbx, kwargs)
def createVRMLskin(loader, drawSkeleton):
    return m.createVRMLskin(loader, drawSkeleton)

class CollisionDetector(lua.instance):
    def __init__(self, **kwargs):
        if 'var_name' in kwargs:
            self.var_name=kwargs['var_name']
    def addModel(self, model):
        self('addModel', model)
    def isSignedDistanceSupported(self):
        return self('isSignedDistanceSupported')
    def calculateNearestSurfacePoint(self, icharacter, ibone, position):
        return self('calculateNearestSurfacePoint', icharacter, ibone, position)
    def addModel(self,v):
        self('addModel', v)
    def addObstacle(self,v):
        self('addObstacle', v)
    def getModel(self, i):
        return self.call_nocopy('getModel', i)

class Timeline(lua.instance):
    def __init__(self, title, numFrames, frameTime=None):
        lua.require('subRoutines/Timeline')
        lua.dostring("""
        if not mEventReceiver then mEventReceiver=EVR() end
        function EVR:onFrameChanged(win, iframe)
        end
        """)
        self.var_name='mTimeline'+m.generateUniqueName()
        if not frameTime:
            frameTime=1.0/30.0
        lua.F_lua(self.var_name,'Timeline',title,numFrames,frameTime)
    def __del__(self):# this is called when garbage collected
        if lua:
            lua.M(self.var_name,'dtor')
            lua.dostring(self.var_name+'=nil')
    def attachCameraToMotion(self, loader, motion):
        lua.M('mEventReceiver', 'attachCameraToMotion', loader, motion)
    def detachCamera(self):
        lua.M('mEventReceiver', 'detachCamera')
    def moveCamera(self, iframe):
        lua.M('mEventReceiver', 'moveCamera', iframe)
    def attachTimer(self, frameRate, numFrames):
        lua.M(self.var_name, 'attachTimer', frameRate, numFrames)
    def reset(self, totalTime, frameTime):
        lua.M(self.var_name, 'reset', totalTime, frameTime)
class OnlineFilter(lua.instance):
    def __init__(self, filterSize, loader=None):
        lua.require('subRoutines/VelocityFields')
        self.var_name='mFilter'+m.generateUniqueName()
        lua.F_lua(self.var_name,'OnlineFilter',loader,None,filterSize)
    def __del__(self):# this is called when garbage collected
        if lua:
            lua.M(self.var_name,'dtor')
            lua.dostring(self.var_name+'=nil')
    def setCurrPose(self, posedof):
        self('setCurrPose', posedof)
    def getFiltered(self):
        return self('getFiltered')
class MaskedOnlineFilter:
    def __init__(self, filterSize, mask_for_filtered, loader=None):
        self.mask=mask_for_filtered
        self.filter=OnlineFilter(filterSize, loader)
    def setCurrPose(self, posedof):
        self.filter.setCurrPose(posedof)
    def getFiltered(self):
        unfiltered, filtered=self.filter('getCenterAndFiltered')
        return filtered*self.mask - unfiltered*(self.mask-1.0)


class OnlineSingleLimbIK:
    def __init__(self, loader, hipbone, anklebone, windowSize):
        self.loader=loader
        self.weightMap=lua.zeros(windowSize)
        for i in range(windowSize):
            self.weightMap[i]=lua.F('sop.mapSin', i, 0, 5, 1,0)
        self.weightMap[0]=0 # firstframe delta has already been applied.

        dofInfo=loader.dofInfo
        self.mask=lua.zeros(dofInfo.numDOF())
        self.mask.range( dofInfo.startT(hipbone.treeIndex()), dofInfo.endR(anklebone.treeIndex())).setAllValue(1) 
        self.mask_leglen=lua.zeros(loader.numBone())
        self.mask_leglen.range(hipbone.treeIndex(), anklebone.treeIndex()).setAllValue(1.0)
        self.i=None

    #limbIKsolver=lua.new('createLimbIksolverToeAndHeel', mLoader_full, markers)
    # returns delta, deltaLeglen
    def calcIKdelta(limbIKsolver, inputpose, marker_pos, importance):
        inputpose_orig=inputpose.copy()
        leglen=limbIKsolver('IKsolve', inputpose, marker_pos, importance)
        delta=inputpose-inputpose_orig
        return delta, leglen-1

    def startReducingDelta(self, initialDelta, initialDeltaLegLen):
        self.initialDelta=initialDelta.copy()
        self.initialDeltaLegLen=initialDeltaLegLen.copy()
        self.i=0
    def getCurrentDelta(self):
        if self.i!=None:
            d= self.initialDelta*self.weightMap[self.i]*self.mask
            d2= self.initialDeltaLegLen*self.weightMap[self.i]*self.mask_leglen
            self.i+=1
            if self.i>=self.weightMap.size()-1:
                self.i=None
            return d, d2
        return self.mask*0, self.mask_leglen*0
class CollisionChecker_pose:
    def __init__(self, checker):
        self.checker=checker
    def __getitem__(self, iloader):
        return self.checker._getPose(iloader)

class CollisionChecker(lua.instance):
    def __init__(self, list_of_obj, **kwargs):
        colType='libccd'
        if isinstance(list_of_obj, str):
            colType=list_of_obj
            list_of_obj=[]

        lua.require("RigidBodyWin/subRoutines/CollisionChecker")
        self.var_name='mChecker'+m.generateUniqueName()
        lua.F_lua(self.var_name, 'CollisionChecker', colType)
        for v in list_of_obj:
            lua.M0(self.var_name, "addObject", v)
        lua._getGlobal((self.var_name, 'collisionDetector'))
        try:
            self.collisionDetector=m.getPythonWin().popCollisionDetector()
        except:
            self.collisionDetector=m.getPythonWin().pop()
            self.collisionDetector=CollisionDetector(var_name=(self.var_name, 'collisionDetector'))

        self.pose=CollisionChecker_pose(self)
        self.collisionSequence=m.CollisionSequence()
    def checkCollision(self):
        self.collisionDetector.testIntersectionsForDefinedPairs(self.collisionSequence)
        return self.collisionSequence
    def setPoseDOF(self, iloader, dof):
        assert(iloader>=0)
        lua.M0(self.var_name, 'setPoseDOF', iloader, dof)
    def registerPair(self, iloader1, iloader2):
        assert(iloader1>=0)
        assert(iloader2>=0)
        lua.M0(self.var_name, 'registerPair', iloader1, iloader2)
    def _getPose(self, iloader):
        assert(iloader>=0)
        return lua.G_vec((self.var_name, 'pose', iloader))

        
class FBXloader_fbxInfo:
    def __init__(self, fbxloader, key):
        self.fbxloader=fbxloader
        self.key=key
    def _set_material(self,mat):
        lua.dostring(self.fbxloader.var_name+'.fbxInfo['+str(self.key)+'].material="'+mat+'"')

    def _get_material(self):
        return lua.G_str((self.fbxloader.var_name,"fbxInfo",self.key, 'material'))


    material = property(
            fget=_get_material,
            fset=_set_material,
            )

class FBXloader_fbxInfoArray:
    def __init__(self, fbxloader,n):
        self.fbxloader=fbxloader
        self.n=n
    def __getitem__(self, i):
        assert(i>0) # lua : 1-indexing
        return FBXloader_fbxInfo(self.fbxloader, i) 
    def __len__(self):
        return self.n


class FBXloader(lua.instance):
    def __init__(self, filename=None, options=None, **kwargs):
        lua.dostring( 'if not FBXloader then FBXloader=require("FBXloader") end')
        if 'var_name' in kwargs:
            self.var_name=kwargs['var_name']
        else:
            self.var_name='mFBXloader'+m.generateUniqueName()
        if filename:
            if options:
                lua.F_lua(self.var_name,'FBXloader', filename, options)
            else:
                lua.F_lua(self.var_name,'FBXloader', filename, kwargs)
        n=lua.F1_int('table.getn', lua.instance(self._addToVarName('fbxInfo')))
        self.fbxInfo=FBXloader_fbxInfoArray(self,n)
    def _get_loader(self):
        return lua.G_loader(self._addToVarName("loader")) # fbx.loader
    def loadSMPLmotion(self, mocap_framerate, trans, poses,_convertYUP=False):
        return lua.F('SMPL.loadMotion', lua.instance((self.var_name,'dd')), lua.instance((self.var_name,'loader')), mocap_framerate, trans, poses, _convertYUP).copy()
    loader=property( fget=_get_loader,)
    def createSkinningInfo(self):
        return self('createSkinningInfo')
    def getPoseDOF(self):
        return self('getPoseDOF')

    def _get_bindpose(self):
        return lua.G(self.var_name,'bindpose')
    def _get_currentPose(self):
        return lua.G(self.var_name,'currentPose')
    def toVRMLloader(self,*args):
        return lua.M(self,'toVRMLloader', *args)
    def _set_cacheCollisionMesh(self,mat):
        self.set('cacheCollisionMesh', mat)

    cacheCollisionMesh=property(
            fset=_set_cacheCollisionMesh
            )
    bindpose = property(
            fget=_get_bindpose,
            )
    currentPose = property(
            fget=_get_currentPose,
            )
# options: {'groundPlane':False} 
class OsimLoader(lua.instance):
    def __init__(self, filename=None, options=None, **kwargs):
        lua.dostring( 'if not OsimLoader then OsimLoader=require("subRoutines/OsimLoader") end')
        if 'var_name' in kwargs:
            self.var_name=kwargs['var_name']
        else:
            self.var_name='mOsimLoader'+m.generateUniqueName()
        if filename:
            if options:
                lua.F_lua(self.var_name,'OsimLoader', filename, options)
            else:
                lua.F_lua(self.var_name,'OsimLoader', filename, kwargs)
    def _get_loader(self):
        return lua.G_loader((self.var_name, "loader")) # fbx.loader
    loader=property( fget=_get_loader,)

    def _get_actuated(self):
        return lua.G((self.var_name, 'actuated'))
    def drawMuscles(self):
        lua.M(self, 'drawMuscles')

    actuated = property(
            fget=_get_actuated,
            )

# options: {'groundPlane':False} 
#see MujocoLoader.lua for more details. 
def writeMujocoXML(self, xmlfile, options=None,**kwargs):
    if options is None and kwargs is None:
        options={'groundPlane':False}
    lua.dostring('if not MujocoLoader then require("RigidBodyWin/subRoutines/MujocoLoader") end')
    if kwargs is None:
        lua.F('writeMujocoXML', self, xmlfile, options)
    else:
        lua.F('writeMujocoXML', self, xmlfile, kwargs)
m.VRMLloader.exportXML=writeMujocoXML

def MujocoLoader(filename, options=None):
    if filename[-4:]=='.wrl' :
        loader=m.VRMLloader(filename)
        xmlfile=filename[:-4]+'.mujoco.xml'
        writeMujocoXML(loader, xmlfile, {'groundPlane':False})
        return loader
    else:
        # creates a temporary file
        wrlfile=filename+'.wrl'
        import os.path
        lua.dostring('if not MujocoLoader then require("RigidBodyWin/subRoutines/MujocoLoader") end')
        if options:
            options['exportToMemory']=True
            if 'useVisualMesh' not in options:
                options['useVisualMesh']=True
            parser=lua.new('MujocoParser', filename,wrlfile, options)
        else:
            parser=lua.new('MujocoParser', filename,wrlfile, { 'useVisualMesh':True, 'exportToMemory':True})
        if parser.get("bodies"):
            loaders=parser.copyget('loaders')
            for iloader in range(len(loaders)):
                l=loaders[iloader]
                if l.getURL()[0]=='{':
                    root=l.VRMLbone(1)
                    pos, ori= lua.F('table.fromstring2', l.getURL())
                    root.setJointPosition(pos)
                    root.getLocalFrame().translation.assign(pos);
                    root.getLocalFrame().rotation.assign(ori);
                    l.fkSolver().forwardKinematics()
            return loaders
        else:
            loader=parser.copyget('loader')
            return loader

def URDFloader(filename, options=None):
    # creates a temporary file
    wrlfile=filename+'.wrl'
    import os.path
    lua.dostring('if not URDFloader then require("RigidBodyWin/subRoutines/URDFloader") end')
    if options==None:
        lua.dostring('URDFparser("'+ filename+'","'+ wrlfile+'", { useVisualMesh=true})')
    else:
        lua.F_lua('tempParser','URDFparser', filename, wrlfile, options)
    return m.VRMLloader(wrlfile)
def flatten(dofs_idx2d):
    out=[]
    for e in dofs_idx2d:
        if isinstance(e, int):
            out+=[e]
        else:
            out+=e
    return out
def genesisIndexMap(robot, mLoader):
    # 가끔 genesis가 xml에 적혀있는 본의 순서를 무시하고 로딩하는 경우가 있어서 아래와 같은 index매핑 필요.
    actuated_bones=[]
    for i in range(1, mLoader.numBone()):
        if mLoader.dofInfo.numDOF(i)>0:
            actuated_bones.append(i)

    dofs_TL_qidx2d= [robot.get_link(mLoader.bone(i).name()).joint.q_idx_local for i in actuated_bones] 
    # quaternion dofs (qpos) ordered according to taesooLib convention 
    return flatten(dofs_TL_qidx2d)
def getTLpose(robot, dofs_TL_qidx, irow=None):
    qpos=robot.get_qpos(qs_idx_local=dofs_TL_qidx)
    if irow!=None:
        return lua.vec(qpos[irow])
    else:
        return lua.vec(qpos)
def setTLpose(robot, TLpose): # setTLpose(robot, taesooLibPose.ref()) 여기서 .ref()는 taesooLibPose의 타입이 vectorn인 경우 필요. 
    robot.set_qpos(TLpose, qs_idx_local=dofs_TL_qidx)
def _setGenesisPose(skin, robot, dofs_TL_qidx, irow=None):
    skin.setPoseDOF(getTLpose(robot, dofs_TL_qidx, irow))

m.PLDPrimVRML.setGenesisPose=_setGenesisPose

# you may need lua.addPackagePath('../../sample_SAMP/lua')
def createSMPLskeleton(bm_path:str, betas=None):
    uid=m.generateUniqueName()
    lua.dostring( 'if not FBXloader then FBXloader=require("FBXloader") end')
    var_name='mFBXloader'+uid
    lua.dostring("""
    SMPL=require("smpl/smpl")
    """)
    dd=lua.new('SMPL.init', bm_path)
    if betas is not None:
        dd.get('betas').range(0,len(betas)).ref()[:]=betas # change bodyshape (first 16 components)

    lua.F_lua(var_name,'SMPL.createFBXskeleton', dd)
    dd.collect()
    return FBXloader(var_name=var_name)

class Constraints(lua.instance):
    def __init__(self, originalPos, **kwargs):
        self.var_name='mCON'+m.generateUniqueName()
        lua.require("RigidBodyWin/subRoutines/Constraints")
        lua.F_lua( self.var_name,'Constraints')
        self.conPos=lua.G_vector3N((self.var_name, 'conPos')) # reference to mCON.conPos
        if isinstance(originalPos, list):
            for v in originalPos:
                self.conPos.pushBack(v)
        else:
            self.conPos.assign(originalPos)
        lua.M0(self.var_name, 'setCON', self.conPos)  # mCON:setCON(self.conPos)

    def connect(self, moduleName, eventFunctionName ):
        # event 루아에서 받아서 다시 파이썬으로 던지기
        self.eventFunctionName=eventFunctionName
        # lua code:
        # eventFunction_mCON=function(ev, i) python.F('__main__','eventFunction', ev, i) end
        # mCON:connect(eventFunction_mCON)
        lua.dostring("""
        eventFunction_"""+self.var_name+"""=function(ev, i)
            python.F('"""+moduleName+"','"+eventFunctionName+"""', ev, i)
        end
        """+self.var_name+""":connect(eventFunction_"""+self.var_name+""")
        """)
    def handleRendererEvent(self, ev, button, x, y):
        return int(lua.M1_dbl(self.var_name, 'handleRendererEvent', ev, button, x, y))

def WRLloader(filename, *kwargs):
    if filename[-4:]=='.wrl':
        loader=m.VRMLloader(filename)
        doNotGarbageCollect.append(loader)
        return loader
    elif filename[-4:]=='.png' or filename[-4:]=='.raw':
        var_name='mLoader'+m.generateUniqueName()
        lua.F_lua(var_name, "MainLib.VRMLloader", filename, *kwargs)
        return lua.G_VRMLloader(var_name)

    var_name='mLoader'+m.generateUniqueName()
    lua.require( "subRoutines/WRLloader")
    if filename[-8:]=='.wrl.lua' or filename[-8:]=='.wrl.dat' :
        lua.F_lua(var_name, "MainLib.WRLloader", filename)
    else:
        lua.F_lua(var_name, "MainLib.WRLloader", lua.toTable(filename))
    info={} # todo : return this when requested through an option
    info['var_name']=var_name
    return lua.G_VRMLloader(var_name)

# FBXskin class can be used for any skin-type instance in the lua side.
class FBXskin(lua.instance):
    def __init__(self, fbx=None, options=None, **kwargs):
        self.var_name='mFBXskin'+m.generateUniqueName()
        self.persistent=False
        if fbx==None:
            self.var_name=options
            self.persistent=True
            return
        if options:
            lua.F_lua(self.var_name,'RE.createFBXskin',fbx, options)
        else:
            lua.F_lua(self.var_name,'RE.createFBXskin',fbx, *kwargs)
    def setTranslation(self, *args):
        lua.M0(self.var_name, 'setTranslation', *args)
    def setVisible(self, b):
        lua.M0(self.var_name, 'setVisible', b)
    def setScale(self, x,y=None,z=None):
        if y:
            lua.M0(self.var_name, 'setScale', x,y,z)
        else:
            lua.M0(self.var_name, 'setScale', x)
    def getState(self):
        return self.get('fkSolver')

    def setPose(self, pose):
        lua.M0(self.var_name, 'setPose', pose)
    def setPoseDOF(self, pose):
        lua.M0(self.var_name, 'setPoseDOF', pose)
    def setLengthAndPose(self, length_scale, pose):
        lua.M0(self.var_name, 'setLengthAndPose', length_scale, pose)
    def setLengthAndPoseDOF(self, length_scale, pose):
        lua.M0(self.var_name, 'setLengthAndPoseDOF', length_scale, pose)
    def applyAnim(self, motion):
        lua.M0(self.var_name, 'applyAnim', motion)
    def setPoseTransfer(self, pt):
        lua.M0(self.var_name, 'setPoseTransfer', pt)
    def setMaterial(self, name):
        lua.M0(self.var_name,'setMaterial', name)
    def __del__(self): # this is called when garbage collected
        if lua and not self.persistent:
            lua.M(self.var_name,'dtor')
            lua.dostring(self.var_name+'=nil')

def tempFunc(self, length_scale, pose):
    lua.M0(self, 'setLengthAndPoseDOF', length_scale, pose)
m.PLDPrimVRML.setLengthAndPoseDOF=tempFunc
m.PLDPrimSkin.setLengthAndPoseDOF=tempFunc

def tempFunc(self, *args):
    lua.M0(self, 'clamp', *args)

m.vectorn.clamp=tempFunc

def drawMesh(mesh, materialName, _optionalNodeName=None, _optionalDoNotUseNormal=None):
    lua.require("Kinematics/meshTools")
    if _optionalNodeName is None:
        _optionalNodeName='node_name'

    useTexCoord=False
    useColor=False

    if mesh.numNormal()==0 :
        _optionalDoNotUseNormal=True
    if mesh.numTexCoord()>0 :
        useTexCoord=True
    if mesh.numColor()>0 :
        useColor=True
    # scale 100 for rendering 
    meshToEntity=m.MeshToEntity(mesh, 'meshName'+_optionalNodeName, False, True, not _optionalDoNotUseNormal, useTexCoord, useColor)
    entity=meshToEntity.createEntity('entityName'+_optionalNodeName )
    if entity :
        if materialName :
            entity.setMaterialName(materialName)
        else:
            entity.setMaterialName("CrowdEdit/Terrain1")
        node=createChildSceneNode(ogreRootSceneNode(), _optionalNodeName)
        node.attachObject(entity)
        return meshToEntity,node
    return None
m.Mesh.drawMesh=drawMesh # register

class SegmentFinder:

    def __init__(self, discontinuity):
        self.vStart=m.intvectorn()
        self.vEnd=m.intvectorn()
        self.vStart.pushBack(0)
        for i in range(1, discontinuity.size()):
            if discontinuity(i) :
                self.vEnd.pushBack(i)
                self.vStart.pushBack(i)
        self.vEnd.pushBack(discontinuity.size())
        assert(self.vStart.size()==self.vEnd.size())

    def numSegment(self):
        return self.vStart.size()

    def startFrame(self,iseg):
        return self.vStart(iseg)

    def endFrame(self,iseg):
        return self.vEnd(iseg)

def derivPose(pose, nextPose, frameRate):
    dmotionDOF_i=m.vectorn()
    dmotionDOF_i.sub(nextPose, pose) # forward difference
    dmotionDOF_i.rmult(frameRate)

    T=pose.toTransf(0)
    # body velocity
    V=T.twist( nextPose.toTransf(0), 1.0/frameRate)
    dmotionDOF_i.setVec3(0, V.v)
    dmotionDOF_i.set(3,0) # unused
    dmotionDOF_i.setVec3(4, V.w)
    return dmotionDOF_i

def _MotionDOF_translate(motiondof, v):
    for i in range(motiondof.numFrames()):
        motiondof.row(i).setVec3(0, motiondof.row(i).toVector3(0)+v)

m.MotionDOF.translate=_MotionDOF_translate
def tempFunc(self, newmot, _optional=None):
    return lua.M(self,'blendStitch', newmot, _optional)
m.MotionDOF.blendStitch=tempFunc

def _MotionDOF_pose(self, i):
    if i<0:
        return self.row(self.rows()+i)
    return self.row(i)

m.MotionDOF.pose=_MotionDOF_pose


def toLuaString(v):
    if isinstance(v, bool):
        if v:
            return "true"
        return "false"
    elif isinstance(v, (int, float)):
        return str(v)
    elif isinstance(v, dict):
        out="{";
        for k, vv in v.items():
            out=out+k+"="+toLuaString(vv)+",\n"
        return out+"} "
    else:
        assert(False)
def loadMotion(loader, motionFile):
    return lua.F('RE.loadMotion',loader, motionFile)
def loadMotions(loader, motionFile):
    return lua.F('RE.loadMotions',loader, motionFile)
def loadRawMotions(*args):
    l=m.getPythonWin()
    lua._F(2, 'RE.loadRawMotions', *args)
    nameid=m.generateUniqueName()
    # save results in lua globals "info_..."
    l.insert(-2)
    l.push('motion')
    l.insert(-2)
    l.settable(-3)
    l.set('info_'+nameid)

    mot=lua.G('info_'+nameid)
    mot.var_name='info_'+nameid
    mot.info=mot.array

    return mot.motion, mot

def createLuaEnvOnly():
    # here, you cannot use renderer!!! but all other non-rendering-related classes can be used.
    # this is different from console.mainlib in that console.mainlib supports skins and renderer too.
    uiscale=1
    taesooLibPath=''
    if not os.path.exists(taesooLibPath+ "../Resource/ogreconfig_linux_sepwin.txt"):
        taesooLibPath= 'work/taesooLib'

    m._createInvisibleMainWin()
    m.getPythonWin().loadEmptyScript()

def releaseLuaEnv():
    m.releaseMainWin()

def createMainWin(argv=None):
    if not argv:
        argv=[]
    import platform
    options=[]
    scriptFile=[]
    dostring=None
    for i in range(1, len(argv)):
        if argv[i][0:2]=='--':
            if option[0:10]=='--dostring':
                dostring=option[11:]
            else:
                options.append(argv[i])

    uiscale=1
    taesooLibPath=''
    if not os.path.exists(taesooLibPath+ "../Resource/ogreconfig_linux_sepwin.txt"):
        taesooLibPath= 'work/taesooLib'

    if m.getOgreVersionMinor()<=3 and platform.system()!='Darwin':
        try:
            if not m.rendererValid():
                renderer=m._createRenderer(0, 0); 
                RENDERER_WIDTH=renderer.getConfig("renderer_width");
                RENDERER_HEIGHT=renderer.getConfig("renderer_height");
                WIDTH=RENDERER_WIDTH+renderer.getConfig("right_panel_width");
                HEIGHT=RENDERER_HEIGHT+renderer.getConfig("right_panel_width");
                UIsf= renderer.getConfigFloat("UI_scale_factor");
                m._createMainWin(WIDTH, HEIGHT, RENDERER_WIDTH, RENDERER_HEIGHT, UIsf, renderer);

        except:
            rw=1920
            rh=1080
            uiscale=1.5
            m.createMainWin(int(rw+180*uiscale),int(rh+100*uiscale), int(rw), int(rh),uiscale)
    else:
        m.createMainWin(int((1024+180)*uiscale),int((600+100)*uiscale), int(1024*uiscale), int(600*uiscale),uiscale)
    m.showMainWin()
    m.getPythonWin().loadEmptyScript()
    if dostring:
        m.getPythonWin().dostring(dostring)
    return m.getPythonWin()
def ogreVersion():
    if m.getOgreVersionMinor()>=12 :
        return 1
    elif m.getOgreVersionMinor()<=3 :
        return 2
    return 0
float_options=None
def tempFunc(self, title, val, vmin, vmax):
    global float_options
    if float_options is None:
        float_options=lua.Table()
    float_options[title]=val
    self.create("Value_Slider", title, title,1)
    self.widget(0).sliderRange(vmin, vmax)
    self.widget(0).sliderStep((vmax-vmin)/20)
    self.widget(0).sliderValue(val)

m.FlLayout.addFloatSlider=tempFunc
def tempFunc(self, w_id, tbl):
    lua.M(self, 'addMenu', w_id, tbl)
m.FlLayout.addMenu=tempFunc
def tempFunc(self, tbl):
    lua.M(self, 'menuItems', tbl)
m.Widget.menuItems=tempFunc

def tempFunc():
    while True:
        if not renderOneFrame(True): break
m.startMainLoop=tempFunc
def tempFunc(self, pose, options=None, **kwargs):
    if options is not None:
        lua.M(self, 'lookAt',pose, options)
    else:
        lua.M(self, 'lookAt',pose, kwargs)
m.Viewpoint.lookAt=tempFunc

def tempFunc(self, *args):
    name=None
    if len(args)==0:
        name=m.generateUniqueName()
        lua.M0(self, 'draw', name)
    else:
        name=args[0]
        lua.M0(self, 'draw', *args)
    return FBXskin(None, ('dbg','g_skins', name))

m.VRMLloader.draw=tempFunc

def tempFunc(self, treeIndex):
    if isinstance(treeIndex, str):
        treeIndex=self.getTreeIndexByName(treeIndex)
        assert(treeIndex!=-1)
    return self.dofInfo.startT(treeIndex),self.dofInfo.endR(treeIndex)
m.MotionLoader.dofIndex=tempFunc

def changeChartPrecision(x):
    lua.F('math.changeChartPrecision', x)
def drawSignals(*args):
    lua.F('math.drawSignals', *args)

def tempFunc(self, *args):
    lua.M(self,'addText', *args)
m.PythonExtendWin.addText=tempFunc

# posetransfer('convertPose', pose)
def tempFunc(self, name, pose):
    self.setTargetSkeleton(pose)
    return self.target().pose()
m.PoseTransfer.__call__=tempFunc

del tempFunc
