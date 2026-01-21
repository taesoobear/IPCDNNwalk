
import os
import pdb # use pdb.set_trace() for debugging
#import code # or use code.interact(local=dict(globals(), **locals())) for debugging. see below.
import numpy as np
from easydict import EasyDict as edict # pip3 install easydict
from pathlib import Path
import subprocess
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
_sceneGraphs=[]

def path(path):
    from pathlib import Path
    path=os.path.normpath(path)
    return Path(path)

# import all three in a line.
# m,lua,control=RE.defaultModules()
def defaultModules():
    return settings.mlib, settings.lua, settings.control
def _compressVoxels(scene, optional_filename=None):
    sceneCompressed=m.boolN(scene.shape[0]*scene.shape[1]*scene.shape[2])
    sceneCompressed.setAllValue(False)
    for i in range(scene.shape[0]):
        for j in range(scene.shape[1]):
            for k in range(scene.shape[2]):
                occ=scene[i,j,k]
                if occ:
                    sceneCompressed.set(i*scene.shape[1]*scene.shape[2]+j*scene.shape[2]+k, True)
    info={'shape':scene.shape, 'bits':sceneCompressed}
    if optional_filename is not None:
        saveTable(info, optional_filename)

def create_cache_folder(path: str | Path, suffix=".cached", create=True) -> Path:
    src = Path(path)

    if not src.is_file():
        raise FileNotFoundError(src)

    parent = src.parent                      # aaa
    cached_parent = parent.with_name(parent.name + suffix)  # aaa.cached
    if create:
        cached_parent.mkdir(parents=True, exist_ok=True)
    return cached_parent, src.name
class Voxels(lua.instance):
    def __init__(self, filename_or_info):
        lua.require("Kinematics/meshTools")
        var_name='Voxels'+m.generateUniqueName()

        if isinstance(filename_or_info, np.ndarray):
            info=_compressVoxels(filename_or_info)
            self.array=filename_or_info
            lua.F_lua(var_name,'Voxels', info)
        elif isinstance(filename_or_info, str):
            if filename_or_info[-4:]=='.npy':
                scene= np.load(filename_or_info)

                cache_folder, filename=create_cache_folder(filename_or_info)
                cache_file=cache_folder.joinpath(filename+'.compressed')

                if not cache_file.is_file():
                    info=_compressVoxels(scene, str(cache_file))
                self.array=scene
                lua.F_lua(var_name,'Voxels', str(cache_file))
            else:
                lua.F_lua(var_name,'Voxels', filename_or_info)
        super().__init__(var_name)


def addPanel(signal):
    if isinstance(signal, m.boolN):
        m.motionPanel().scrollPanel().addPanel(signal, m.CPixelRGB8(255,255,0))
def lookAt(pose, options=None, **kwargs):
    if options is not None:
        lua.M(m.viewpoint(), 'lookAt', pose, options)
    else:
        lua.M(m.viewpoint(), 'lookAt', pose, kwargs)

def filename(full_path): return lua.F('os.filename', full_path)
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
def tempFunc(self):
    return max(self.x, self.y, self.z)
m.vector3.maximum=tempFunc


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
import time

start_time = time.time()

def chooseFile(title, path, mask, write):
    res= lua.F('Fltk.ChooseFile', title, path, mask, write)
    if isinstance(res, tuple):
        return None
    return res

def renderOneFrame(check):
    global start_time 
    ctime=time.time()
    elapsed =  ctime- start_time
    start_time=ctime
    if check:
        if elapsed>1.0/30:
            elapsed=1.0/30
        if _sceneGraphs is not None:
            for i, v in enumerate(_sceneGraphs):
                for k, vv in v.objects.items():
                    if vv.handleFrameMove is not None:
                        vv.handleFrameMove(vv, elapsed)
    return m.renderOneFrame(check)
def output(key, *args):
    m._output(key,str(args),1)

def output2(key, *args):
    m._output(key,str(args),2)


class Skins:
    def __init__(self, loaders, options=None):
        self.skins=[]
        for loader in loaders :
            if loader is None:
                self.skins.append(None)
            else:
                self.skins.append(createSkin(loader, options))
    def __len__(self):
        return len(self.skins)
    def __getitem__(self, i):
        return self.skins[i]
    def setScale(self, x,y=None,z=None):
        if y!=None:
            for skin in self.skins :
                if skin is not None:
                    skin.setScale(x,y,z)
        else:
            for skin in self.skins :
                if skin is not None:
                    skin.setScale(x)
    def setTranslation(self, x,y=None,z=None):
        if y!=None:
            for skin in self.skins :
                if skin is not None:
                    skin.setTranslation(x,y,z)
        else:
            for skin in self.skins :
                if skin is not None:
                    skin.setTranslation(x)
    def setMaterial(self, mat):
        for skin in self.skins :
            if skin is not None:
                skin.setMaterial(mat)
    def setSamePose(self, sim):
        for i, skin in enumerate(self.skins):
            if skin is not None:
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
class OnlineFilter6D(lua.instance):
    def __init__(self, filterSize, loader=None):
        lua.require('subRoutines/VelocityFields')
        self.var_name='mFilter'+m.generateUniqueName()
        lua.F_lua(self.var_name,'OnlineFilter6D',loader,None,filterSize)
    def __del__(self):# this is called when garbage collected
        if lua:
            lua.M(self.var_name,'dtor')
            lua.dostring(self.var_name+'=nil')
class MaskedOnlineFilter:
    def __init__(self, filterSize, mask_for_filtered, loader=None):
        self.mask=mask_for_filtered
        self.filter=OnlineFilter(filterSize, loader)
    def setCurrPose(self, posedof):
        self.filter.setCurrPose(posedof)
    def getFiltered(self):
        unfiltered, filtered=self.filter.getCenterAndFiltered()
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
    def _get_bindpose(self):
        return lua.G(self.var_name,'bindpose')
    def _get_currentPose(self):
        return lua.G(self.var_name,'currentPose')
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
def createMujocoXMLstring(self, **kwargs):
    if kwargs is None:
        kwargs={'groundPlane':False}
    lua.dostring('if not MujocoLoader then require("RigidBodyWin/subRoutines/MujocoLoader") end')
    return lua.F('createMujocoXMLstring', self, kwargs)

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

    # from genesis 0.3, link.joint became link.joints
    dofs_TL_qidx2d= [robot.get_link(mLoader.bone(i).name()).joints[0].q_idx_local for i in actuated_bones] 
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
    assert(path(bm_path).is_file())
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
        """)
        lua.M(self,'connect',  lua.instance('eventFunction_'+self.var_name))

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

def clone_git_to_cache(
    repo_url: str,
    repo_name: str | None = None,
    cache_dir_name: str = ".cache",
    pull_if_exists: bool = False,
) -> Path:
    """
    Clone a git repository into a cache directory (platform-independent).

    Args:
        repo_url: Git repository URL
        repo_name: Folder name for the repo (defaults to repo URL name)
        cache_dir_name: Cache directory name under user home
        pull_if_exists: If True, run 'git pull' when repo already exists

    Returns:
        Path to the cloned repository
    """

    # ~/.cache (or custom name)
    cache_root = Path.home() / cache_dir_name
    cache_root.mkdir(parents=True, exist_ok=True)

    if repo_name is None:
        repo_name = Path(repo_url.rstrip("/")).stem

    repo_path = cache_root / repo_name

    if repo_path.exists():
        if pull_if_exists:
            print(f"[INFO] Repository exists, pulling: {repo_path}")
            subprocess.run(
                ["git", "-C", str(repo_path), "pull"],
                check=True,
            )
        else:
            print(f"[INFO] Repository already cached: {repo_path}")
    else:
        print(f"[INFO] Cloning repository to cache: {repo_path}")
        subprocess.run(
            ["git", "clone", repo_url, str(repo_path)],
            check=True,
        )

    return repo_path
def run_mklink_as_admin(src, tgt, bat_path=None):
    src = os.path.abspath(src)
    tgt = os.path.abspath(tgt)

    if bat_path is None:
        bat_path = Path.cwd() / "gitscript.bat"
    else:
        bat_path = Path(bat_path)

    # 1. gitscript.bat 생성
    bat_content = f"""@echo off
echo Creating symbolic link...
mklink /d "{tgt}" "{src}"
if %errorlevel% neq 0 (
    echo Failed to create symlink.
) else (
    echo Symlink created successfully.
)
set /p res=Press Enter to continue.
"""

    bat_path.write_text(bat_content, encoding="utf-8")

    # 2. PowerShell로 관리자 권한 실행
    ps_cmd = [
        "powershell",
        "-NoProfile",
        "-Command",
        f"Start-Process '{bat_path}' -Verb RunAs"
    ]

    subprocess.run(ps_cmd, check=True)

def createMainWin(argv=None):

    if not os.path.exists('./work'):
        print("Ogre3D resource folder ('work') not found. Creating it from GitHub taesoobear/IPCDNNwalk/work.")
        cache_root = Path.home() / '.cache'

        repo_path = clone_git_to_cache(
            repo_url="https://github.com/taesoobear/IPCDNNwalk.git",
            pull_if_exists=True,
        )

        if os.name == "nt":
            print('Creating symbolic link')
            run_mklink_as_admin(str(cache_root/'IPCDNNwalk'/'work'), 'work')
        else:
            os.symlink(cache_root/'IPCDNNwalk'/'work', 'work')

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



class SceneGraph:
    def __init__(self):
        global _sceneGraphs
        lua.require('subRoutines/RayBoxCheck')
        self.objects={}
        self.objectList=m.ObjectList()
        self._lastCursorPosOnMove=None
        _sceneGraphs.append(self)
    def __del__(self):# this is called when garbage collected
        global _sceneGraphs
        if _sceneGraphs is not None:
            _sceneGraphs.remove(self)

    def createUI(self, this, addHouseBuilder=False, snapToGrid=True):
        self.layout=this

        this.setWidgetHeight(80)
        this.create("Multi_Browser","clip board","components",0)



        this.resetToDefault()
        this.newLine()
        this.setWidgetHeight(80)

        this.create("Multiline_Input",'sceneComponent_script')

        this.resetToDefault()

        this.create("Button","Run script","change property")
        #this.newLine()

        this.create("Check_Button","snap to grid","snap to grid")
        if snapToGrid:
            this.widget(0).checkButtonValue(1)
        else:
            this.widget(0).checkButtonValue(0)

        this.create("Choice", "edit mode","edit mode",1)
        this.widget(0).menuSize(12)
        this.widget(0).menuItem(0,"none",'n')
        this.widget(0).menuItem(1,"translate",'t')
        this.widget(0).menuItem(2,"rotate",'r')
        this.widget(0).menuItem(3,"scale",'s')
        this.widget(0).menuItem(4,"translate Y")
        this.widget(0).menuItem(5,"rotate Z")
        this.widget(0).menuItem(6,"rotate XY")
        this.widget(0).menuItem(7,"CLICK. remove",'q')
        this.widget(0).menuItem(8,"CLICK. duplicate",'y')
        this.widget(0).menuItem(9,"CLICK. rotate 90 about X",'z')
        this.widget(0).menuItem(10,"CLICK. rotate 90 about Y",'x')
        this.widget(0).menuItem(11,"CLICK. rotate 90 about Z",'c')
        self.editModes=[ "none", "translate", "rotate", "scale", "translate Y" ,"rotate Z", "rotate XY"]

        this.widget(0).menuValue(1)

        if addHouseBuilder :
            this.create("Choice", "change material","material",1)
            this.widget(0).menuItems( [
                ["Crowd materials", "Crowd/Blue","cagotruck.material", "Crowd/Red", "Crowd/Green", "Crowd/Red", "Crowd/Dirt", 
                "Crowd/Dirt01", "Crowd/EgyptRocky", "Crowd/MtlPlat2", "Crowd/RustedMetal", "Crowd/TerrRock", "Crowd/Terrain", 
                "CrowdEdit/Terrain1"],
                ["Solid colors", "solidblue", "solidblack", "solidred", "solidlightgreen", "solidgreen", "solidwhite"],
                ["Colors", "green", "white", "black","blue","red","lightgrey"],
                ["Icons", "blueCircle", "redCircle", "icons/add", "icons/refresh","LAKEREM"]
            ])
            this.create("Choice", "change source","source",1)
            this.widget(0).menuItems( [
                ["Simple polygons", "sphere1010.mesh", "arrow.mesh", "cone.mesh", "axes.mesh", "cylinder.mesh"],
                ["Static objects", "pave.mesh","cagotruck.mesh", "ogrehead.mesh"],
                ["Ogre default meshes", "ogrehead.mesh", "athene.mesh", "Barrel.mesh", "column.mesh", "cube.mesh", "facial.mesh", "fish.mesh", "geosphere4500.mesh",
                "geosphere8000.mesh", "knot.mesh", "ninja.mesh", "razor.mesh", "RZR-002.mesh", "sphere.mesh", "WoodPallet.mesh"],
                ["Dynamic objects",  "muaythai1.mesh"],
            ])

            this.create("Choice","house","add house")
            this.widget(0).menuSize(6)
            this.widget(0).menuItem(0,"type 1",'6')
            this.widget(0).menuItem(1,"type 2",'7')
            this.widget(0).menuItem(2,"type 3",'8')
            this.widget(0).menuItem(3,"type 4",'9')
            this.widget(0).menuItem(4,"type 5",'0')
            this.widget(0).menuItem(5,"type 6",'-')

            this.create("Choice","create item")
            this.widget(0).menuSize(8)
            this.widget(0).menuItem(0,"choose item")
            this.widget(0).menuItem(1,"item 1",'6')
            this.widget(0).menuItem(2,"item 2",'7')
            this.widget(0).menuItem(3,"item 3",'8')
            this.widget(0).menuItem(4,"item 4",'9')
            this.widget(0).menuItem(5,"item 5",'0')
            this.widget(0).menuItem(6,"item 6",'-')
            this.widget(0).menuItem(7,"item 7",'=')

            this.create("Button", "build all houses", "build all houses")
        this.updateLayout()
    def handleEditingEvent(self, pushed,cur,ev,editMode)    :
        this=self.layout

        menu=this.findWidget("edit mode")

        if (ev=='PUSH') :

            menu.deactivate()
            bSnapToGrid=this.findWidget("snap to grid").checkButtonValue()
            browser=this.findWidget('clip board')
            for i in range(1,browser.browserSize() +1):
                if (browser.browserSelected(i)) :
                    graph=self.getInfo(browser, i)
                    pushed.selectedItems.append(graph)
                    graph.push_state=self.copyInfo(graph)

            self.drawIcon(editMode, pushed, pushed.cursorPos)
            return 1
        elif (ev=='DRAG') :
            import math
            if (editMode[:9]=='translate') :
                self.drawIcon(editMode, pushed, cur.cursorPos)


            bSnapToGrid=this.findWidget("snap to grid").checkButtonValue()
            browser=this.findWidget('clip board')
            for i in range(1,browser.browserSize() +1):

                if (browser.browserSelected(i)) :
                    graph=self.getInfo(browser, i)
                    if graph.push_state is None:
                        continue

                    if(editMode=='translate' or editMode=='translate Y') :
                        delta=cur.cursorPos-pushed.cursorPos

                        if(editMode=='translate Y') :
                            if(bSnapToGrid) :
                                delta.x=(math.floor(delta.x/50.0))*50
                                delta.y=(math.floor(delta.y/50.0))*50
                                delta.z=(math.floor(delta.z/50.0))*50
                        else:
                            if(bSnapToGrid) :
                                delta.x=(math.floor(delta.x/50.0))*50
                                delta.y=0
                                delta.z=(math.floor(delta.z/50.0))*50


                        graph._pos.assign(graph.push_state.pos+delta)
                        graph.setTransform()
                    elif (editMode[:6]=='rotate' ) :

                        if editMode=='rotate XY':
                            amt_x=(pushed.x-cur.x)/-100.0
                            amt_y=(pushed.y-cur.y)/-100.0

                            graph._ori.assign(m.quater(amt_y,m.vector3(1,0,0))*m.quater(amt_x,m.vector3(0,1,0))*graph.push_state.ori)
                        else:
                            amt=(pushed.x-cur.x)/-100.0

                            if(bSnapToGrid) :
                                amt=math.floor(amt/math.radians(15))*math.radians(15)

                            if editMode=='rotate Z':
                                amt=(pushed.x - cur.x + pushed.y-cur.y)/50.0
                                graph._ori.assign(m.quater(amt,m.vector3(0,0,1))*graph.push_state.ori)
                            else:
                                graph._ori.assign(m.quater(amt,m.vector3(0,1,0))*graph.push_state.ori)
                        graph.setTransform()
                    elif (editMode=='scale') :
                        if(pushed.y-cur.y>0) :
                            scale=1.0+(pushed.y-cur.y)/100.0
                        else:
                            scale=1.0/(1.0+(cur.y-pushed.y)/100.0)

                        graph._scale.assign(graph.push_state.scale*scale)
                        graph.setTransform()



        elif (ev=='RELEASE') :
            browser=this.findWidget('clip board')
            for i in range(1,browser.browserSize() +1):
                if (browser.browserSelected(i)) :
                    graph=self.getInfo(browser, i)
                    graph.push_state=None


            self.objectList.erase("EDIT_MODE")
            menu.activate()
            self.updateScript()

            #this.findWidget("edit mode").menuValue(0)
            this.redraw()
            return 1

        return 0


    def drawIcon(self, editMode, pushed, cursorPos):
        if editMode[:5]=='CLICK' :
            return 


        pos=m.vector3N()
        if(editMode[:6]=='rotate') :
            thickness=250
            mat= "icons/refresh"
        else:
            thickness=200
            mat= "icons/add"

        if len(pushed.selectedItems)==1:
            item=pushed.selectedItems[0]
            thickness*=((item.bbmax-item.bbmin).maximum()+10)/110.0
            cursorPos=item.position

        pos.pushBack(cursorPos)

        if len(pushed.selectedItems)==0: 
            return 
        if editMode=='translate Y' or editMode=='rotate Z':
            self.objectList.registerObject('EDIT_MODE', 'QuadListV', mat, pos.matView(), thickness)
        else:
            self.objectList.registerObject('EDIT_MODE', 'QuadListY', mat, pos.matView(), thickness)

    def _removeBrowserItem(self, item, child,browser, i) :
        item.__finalize()
        self.objects[item.nodeId]=None
        browser.browserRemove(i) 

    def dtor():
        this=self.layout
        browser=this.findWidget('clip board')
        for i in range(1,browser.browserSize() +1):
            self.objects[browser.browserText(i)]=None

        self.objectList.clear()

        lua.collectgarbage()
    def _addBrowserItem(self, item, child,browser, i) :
        info=SceneComponent(item.scType)
        ci=self.copyInfo(item)
        for k, v in ci.items() :
            info[k]=v
        self.add(info)
    def foreachSelected(self, fcn, bBreak):
        this=self.layout
        browser=this.findWidget('clip board')
        for i in range(1,browser.browserSize() +1):

            if (browser.browserSelected(i)) :
                graph=getInfo(browser, i)
                child=RE.ogreSceneManager().getSceneNode(graph.nodeId)
                fcn(graph, child, browser, i)
                if bBreak :
                    break

                graph.setTransform()


    def copyInfo(self, v):
        return lua.Table( scType=v.scType,
            nodeId=v.nodeId,
            options=v.options.copy(),
            pos=v._pos.copy(),
            ori=v._ori.copy(),
            scale=v._scale.copy(),
            material=v.material,
            bNormaliseNormals=v.bNormaliseNormals,
            source=v.source,
        )

    def getInfo(self, browser, i):
        id=browser.browserText(i)
        pInfo=self.objects[id]
        return pInfo

    def updateScript(self):
        this=self.layout
        browser=this.findWidget('clip board')
        _input=this.findWidget('sceneComponent_script')

        count=0
        for i in range(1,browser.browserSize() +1):
            if browser.browserSelected(i) :
                count+=1

        if count ==1:
            for i in range(1,browser.browserSize() +1):
                if browser.browserSelected(i) :
                    pInfo=self.getInfo(browser, i)
                    _input.inputValue(pInfo.getScript())
                    _input.redraw()
                    return
        else:
            _input.inputValue("")
            this.redraw()
    def dbgDraw(self, name, pos, color, size=5):
        color = color or 'blue'
        #namedDraw('Sphere', pos, name, color, size)
    def handleRendererEvent(self, ev, button, x, y):

        output("event", ev, x,y, button);
        this=self.layout

        ray = m.Ray()
        m.FltkRenderer().screenToWorldRay(x,y,ray)

        if ev =="PUSH" or ev=="MOVE" or ev=="DRAG" or ev=="RELEASE" :



            PLANE_HEIGHT=1.0;
            tt=m.Plane (m.vector3(0,1,0), PLANE_HEIGHT);
            if (ev=='PUSH' or ev=='RELEASE' or ev=='DRAG') :

                editMode=this.findWidget('edit mode').menuText()
                if (self._lastCursorPosOnMove is not None):
                    normal=m.vector3(0,1,0)
                    if editMode[-1:]=='Y':
                        normal=m.viewpoint().vat-m.viewpoint().vpos
                        normal.y=0
                        normal.normalize()
                    tt=m.Plane (normal, self._lastCursorPosOnMove);

                self.dbgDraw("cursorPos",self._lastCursorPosOnMove,'red', 6)
                r=ray.intersects(tt)
                currentcursorPos=ray.getPoint(r(1))
                self.dbgDraw("cursor",currentcursorPos,'blue')

                if editMode[:5]=='CLICK' :
                    editMode=editMode[8:]
                    if ev=='RELEASE' :
                        if editMode=='remove' :
                            self.foreachSelected(_remove, True)
                        elif editMode=='duplicate' :
                            self.foreachSelected(_addBrowserItem , True)
                        elif editMode[:9] =='rotate 90' :
                            if editMode[-1:]=='X' :
                                q=m.quater(math.radians(90),m.vector3(1,0,0))
                            elif editMode[-1:]=='Y' :
                                q=m.quater(math.radians(90),m.vector3(0,1,0))
                            else:
                                q=m.quater(math.radians(90),m.vector3(0,0,1))

                            rotate_q=lambda graph, child: _rotate(graph, child, q)
                            self.foreachSelected(rotate_q)
                        else:
                            print('not implmented yet '+editMode)


                    return 1;


                if ev=='PUSH' :
                    self._pushed=lua.Table( x=x, y=y, cursorPos=currentcursorPos.copy(), selectedItems=[])


                if editMode!='none' :

                    if ev=='PUSH':
                        menu=this.findWidget('edit mode')
                        editMode=menu.menuText()
                        if button==3:
                            if(editMode=='translate' or editMode=='translate Y'):
                                this.findWidget('edit mode').menuValue(self.editModes.index('translate Y'))
                            elif editMode[:6]=='rotate':
                                this.findWidget('edit mode').menuValue(self.editModes.index('rotate XY'))
                        elif button==2:
                            if editMode[:6]=='rotate':
                                this.findWidget('edit mode').menuValue(self.editModes.index('rotate Z'))
                        else:
                            if(editMode=='translate' or editMode=='translate Y'):
                                this.findWidget('edit mode').menuValue(self.editModes.index('translate'))
                            elif editMode[:6]=='rotate':
                                this.findWidget('edit mode').menuValue(self.editModes.index('rotate'))
                        menu.redraw()
                        editMode=menu.menuText()

                    return self.handleEditingEvent(self._pushed, lua.Table(x=x, y=y, cursorPos=currentcursorPos.copy()), ev, editMode )

                return 1;

            elif ev=='MOVE' :

                menu=this.findWidget("edit mode")
                menu.activate()

                pInfo,bret,o=self.FindCollisionEntity(ray)

                self.deselectAll()
                if bret :
                    self._lastCursorPosOnMove=ray.getPoint(o)
                    self.selectfunc(pInfo.nodeId)
                else:
                    self._lastCursorPosOnMove=None
                return 1
        return 0

    def getNodeTransform(self, pNode):
        tt=m.matrix4()
        tt.setTransform(pNode.getPosition(), pNode.getScale(), pNode.getOrientation())
        return tt
    def FindCollisionEntity(self, ray):
        for i,v in self.objects.items():
            v.showBoundingBox(False)

        min_o=1e9
        argMin=None
        for i,pInfo in self.objects.items():
            childnode=ogreSceneManager().getSceneNode(pInfo.nodeId)
            MatrixInfo=self.getNodeTransform(childnode)
            # bbox margin ==5
            bbmin=pInfo.bbmin*pInfo._scale-m.vector3(5,5,5)
            bbmax=pInfo.bbmax*pInfo._scale+m.vector3(5,5,5)

            bret, o=lua.F('rayIntersectsBox', ray, bbmin, bbmax, MatrixInfo)
            #childnode.setPosition(currentcursorPos.x,currentcursorPos.y,currentcursorPos.z)
            if bret and o<min_o :
                min_o=o
                argMin=(pInfo, bret, o)


        if argMin :
            return argMin

        return None, False, 0



    def onCallback(self, w, userData):
        this=self.layout
        if (w.id()=="clip board") :
            browser=this.findWidget('clip board')
            for i in range(1,browser.browserSize() +1):
                if browser.browserSelected(i) :
                    self.selectfunc(browser.browserText(i))
                else:
                    self.deselect(browser.browserText(i))


            self.updateScript()
        elif (w.id()=="operations") :
            op=this.menuText()
            if op.left(6)=="rotate":
                pass
            else:
                pass

        elif(w.id()=="create item") :
            if not (w.menuValue() ==0) :

                i= this.findWidget("house").menuValue()+1
                j= w.menuValue()

                self.buildHouse(i,j)

        elif (w.id()=="build all houses") :
            for i in range(1,6 +1):
                for j in range(1,7 +1):
                    self.buildHouse(i,j)


        elif (w.id()=="change material") :
            browser=this.findWidget('clip board')
            for i in range(1,browser.browserSize() +1):
                if browser.browserSelected(i) :
                    id=browser.browserText(i)
                    pInfo=self.objects[id]
                    pInfo.material=w.menuText()
                    pInfo.redraw() 


            self.updateScript()
        elif (w.id()=="change source") :
            browser=this.findWidget('clip board')
            for i in range(1,browser.browserSize() +1):
                if browser.browserSelected(i) :
                    id=browser.browserText(i)
                    pInfo=self.objects[id]
                    if (pInfo.scType == SceneComponent.ENTITY) :
                        pInfo.source=w.menuText()
                        pInfo.redraw()



            self.updateScript()





        elif (w.id()=="Run script") : # change property button pressed

            browser=this.findWidget('clip board')
            _input=this.findWidget('sceneComponent_script')
            for i in range(1,browser.browserSize() +1):
                if browser.browserSelected(i) :
                    id=browser.browserText(i)
                    pInfo=self.objects[id]

                    localSpace={
                    'pos':pInfo._pos,
                    'scale':pInfo._scale,
                    'ori':pInfo._ori,
                    'material':pInfo.material,
                    'source':pInfo.source,
                    'options':pInfo.options,
                    }

                    f=_input.inputValue()
                    try:
                        exec(f, {}, localSpace)
                    except Exception as e:
                        print(f"Error: {e}\n{f}")

                    #if res==nil :
                    if 'material' in localSpace:
                        pInfo.material=localSpace['material']
                    if 'source' in localSpace:
                        pInfo.source=localSpace['source']
                    #pInfo.options.setValues(4000,4000,40,40)
                    #end


                    pInfo.redraw()



    def findSceneComponent(self, node_name):
        if self.objects.get(node_name) is not None:
            return True

        return False


    def findCollidingSceneComponent(self, pgraph):
        for k, v in self.objects.items():
            if v.nodeId!=pgraph.nodeId :
                if pgraph._pos.distance(v._pos)<60 :
                    return True
        return False

        
    def _rotate(self, item, child, q):
        item.ori.leftMult(q) 
    def addEntity(self, mesh_name, bbox_size=None, nodeId=None, localPosition=None, localScale=None, localOrientation=None):
        entity=SceneComponent(SceneComponent.ENTITY, localPosition=localPosition, localScale=localScale, localOrientation=localOrientation)
        if nodeId is not None:
            entity.nodeId=nodeId
        else:
            entity.nodeId=mesh_name[:-5]+"_000"
        entity.source=mesh_name
        if bbox_size is not None:
            entity.bbmin=m.vector3(-bbox_size/2.0)
            entity.bbmax=m.vector3(bbox_size/2.0)


        self._add(entity)
        return entity

    def _add(self, pitem):
        this=self.layout
        while(self.findSceneComponent(pitem.nodeId)):
            try:
                pitem.nodeId=pitem.nodeId[:-3]+'%03d'%(int(pitem.nodeId[-3:])+1)
            except:
                pitem.nodeId=pitem.nodeId+'_001'

        import random
        while(self.findCollidingSceneComponent(pitem)):
            if random.uniform(0,1)<0.5 :
                pitem._pos.x+=100
            else:
                pitem._pos.z+=100

        assert(pitem.nodeId not in self.objects)

        self.objects[pitem.nodeId]=pitem
        pitem.redraw()
        browser=this.findWidget('clip board')
        browser.browserAdd(pitem.nodeId)
        browser.redraw()

    def selectfunc(self, id):
        this=self.layout
        browser=this.findWidget('clip board')
        for i in range(1,browser.browserSize() +1):
            if (id==browser.browserText(i)) :
                if not (browser.browserSelected(i)) :
                    browser.browserSelect(i)




        item=self.objects[id]
        item.showBoundingBox(True)
        self.updateScript()


    def deselect(self, id):
        this=self.layout
        browser=this.findWidget('clip board')

        pInfo=self.objects[id]

        for i in range(1,browser.browserSize() +1):
            if id==browser.browserText(i) :
                if browser.browserSelected(i) :
                    browser.browserSelect(i)
                    break


        pInfo.showBoundingBox(False)
        
    def deselectAll(self):
        this=self.layout
        browser=this.findWidget('clip board')
        for i in range(1, browser.browserSize() +1):
            self.deselect(browser.browserText(i))

        browser.browserDeselect()
        this.redraw();





    def buildHouse(self, i,j):
        entity=SceneComponent(SceneComponent.ENTITY)
        if i==1 or i==6 :
            if j>5 :
                # mesh does not exist.
                return 


        id=f"{i}{j}"
        entity.source=f"h{id}.mesh"
        entity.nodeId=f"h{id}_000" 
        entity._pos.y=50

        if((i==1 and j ==5) or (i==3 and j == 4) or (i==3 and j == 6) or (i==3 and j ==5) or (i==4 and j ==5) or (i==4 and j==2)) :
            entity._pos.x=50    
            entity.bbmin.x=-100
            entity.bbmax.x=100

        # adjust bbox size
        if (i==5 and j==5):
            entity.bbmin.x=-100
            entity.bbmax.x=100

        if (i==3 and j==6) or (i==3 and j==7):
            entity.bbmin.y=-100
            entity.bbmax.y=100
        if (i==3 and j==5):
            entity.bbmin.y=-100
            entity.bbmax.y=100
            entity.bbmin.x=-100
            entity.bbmax.x=100
        if (i==5 and j==1):
            entity.bbmin.z=-100
            entity.bbmax.z=100
        if (i==4 and j==7):
            entity.bbmin.y=-100
            entity.bbmax.y=100
            entity.bbmin.z=-100
            entity.bbmax.z=100
        if (i==5 and j==2):
            entity.bbmin.z=-100
            entity.bbmax.z=100
        if (i==5 and j==6):
            entity.bbmin.x=-100
            entity.bbmax.x=100
            entity.bbmin.y=-100
            entity.bbmax.y=100
            entity.bbmin.z=-100
            entity.bbmax.z=100
        if (i==5 and j==7):
            entity.bbmin.y=-100
            entity.bbmax.y=100
        if (i==5 and j==3) or (i==5 and j==4):
            entity.bbmin.x=-100
            entity.bbmax.x=100
            entity.bbmin.z=-100
            entity.bbmax.z=100
        entity._pos.y=entity.bbmax.y


        self._add(entity)
        self.selectfunc(entity.nodeId)

class SceneComponent:
    PLANE=1
    TERRAIN=2
    ENTITY=3
    NONE=4
    @property
    def position(self):
        return self._pos

    @position.setter
    def position(self, value):
        self._pos .assign( value)
        self.setTransform()
    @property
    def scale(self):
        return self._scale

    @scale.setter
    def scale(self, value):
        self._scale .assign( value)
        self.setTransform()
    @property
    def orientation(self):
        return self._ori
    @orientation.setter
    def orientation(self, value):
        self._ori .assign( value)
        self.setTransform()
    @property
    def material(self):
        return self._material
    @material.setter
    def material(self, value):
        self._material=value
        self.redraw()

    @property
    def localOrientation(self):
        if hasattr(self,'pChildNode'):
            return self.pChildNode.getOrientation()
        return m.quater(1,0,0,0)
    @localOrientation.setter
    def localOrientation(self, v):
        if not hasattr(self,'pChildNode'):
            print("this case has not been implemented yet")
            assert(False)
        self.pChildNode.setOrientation(v)

    def __init__(self, t, localPosition=None, localScale=None, localOrientation=None):
        self.scType=t
        self._pos=m.vector3(0,50,0)
        self._scale=m.vector3(1,1,1)
        self._ori=m.quater(1,0,0,0)
        self.bNormaliseNormals=True
        self.options=m.intvectorn()
        self.bbmin= m.vector3(-50,-50,-50)
        self.bbmax= m.vector3(50,50,50)
        self.show_bbox=False
        self._localPosition=localPosition
        self._localOrientation=localOrientation
        self._localScale=localScale
        self.handleFrameMove=None
        self.eventFunction=None
        
        if localScale is not None and isinstance(localScale, (float, int)):
            self._localScale=m.vector3(localScale)


        if (self.scType==SceneComponent.PLANE) :
            self.nodeId="plane_000"
            self._material="Crowd/Board"
            self.options=lua.ivec([4000,4000,20,20])
        elif (self.scType==SceneComponent.ENTITY) :
            self.nodeId='entity_000'
            self._material=""
            self.source='h11.mesh'


    

    def __del__(self):
        if m is not None:
            mgr=ogreSceneManager()
            child=mgr.getSceneNode(self.nodeId)
            if child is not None:
                removeEntity(child)
    def _redrawBBOX(self):
        pNode=self.pNode
        if self.show_bbox:
            # bbox margin ==5
            draw('WireBox', self.getTransform(pNode), 'bbox2'+self.nodeId, (self.bbmax-self.bbmin)*self._scale+m.vector3(10,10,10), 1, 'solidwhite')
        else:
            erase('WireBox', 'bbox2'+self.nodeId)

    def setTransform(self):
        pNode=self.pNode
        pNode.setPosition(self._pos)
        pNode.setScale(self._scale)
        pNode.setOrientation(self._ori)

        self._redrawBBOX()

        if self.eventFunction is not None:
            self.eventFunction(self, 'transform modified')
    def getScript(self):
        out=[]
        out.append('pos.set(%.3f, %.3f, %.3f)\n'%(self._pos.x,self._pos.y,self._pos.z))
        out.append('scale.set(%.3f, %.3f, %.3f)\n'%(self._scale.x,self._scale.y,self._scale.z))
        out.append(f'ori.setValue({self._ori.w},{self._ori.x},{self._ori.y},{self._ori.z})\n')
        if self.scType==SceneComponent.ENTITY :
            out.append("material=''\n")
        else:
            out.append("material='"+self._material+"'\n")

        if self.scType==SceneComponent.ENTITY  :
            out.append("source= '"+self.source+"'\n")
        else:
            out.append("source=''\n")



        if(self.options.size()==4) :
            out.append(f"options.setValues({self.options(0)},{self.options(1)},{self.options(2)},{self.options(3)})\n")
        elif (self.options.size()==6) :
            out.append(f"options.setValues({self.options(0)},{self.options(1)},{self.options(2)},{self.options(3)},{self.options(4)},{self.options(5)})\n")
        elif (self.options.size()==7) :
            out.append(f"options.setValues({self.options(0)},{self.options(1)},{self.options(2)},{self.options(3)},{self.options(4)},{self.options(5)},{self.options(6)})\n")

        return ''.join(out)[:-1]
    def _getSource(self):
        if(self.scType==SceneComponent.PLANE) :
            return self.options
        return self.source

    def redraw(self):
        rootnode=ogreRootSceneNode()
        pNode=None
        if not hasattr(self, 'lastSource') or self.lastSource!=self._getSource() :
            removeEntity(self.nodeId)
            if(self.scType==SceneComponent.PLANE) :
                if((self.options.size())!=4 and (self.options.size())!=6) :
                    Msg.MsgBox("error! plane should have 4 or 6 parameters.(width,height,nx,ny,ntx=1,nty=1)")
                    return

                pNode=createChildSceneNode(rootnode,self.nodeId)
                if(self.options.size()==4) :
                    entity=createPlane("_entity_"+self.nodeId,self.options(0),self.options(1),self.options(2),self.options(3),1,1)
                else:
                    entity=createPlane("_entity_"+self.nodeId,self.options(0),self.options(1),self.options(2),self.options(3),self.options(4),self.options(5))
                self.lastSource=self.options
            else:
                pNode=createChildSceneNode(rootnode,self.nodeId)
                entity=ogreSceneManager().createEntity("_entity_"+self.nodeId,self.source)
                self.lastSource=self.source

            if self._localPosition is not None or self._localOrientation is not None or self._localScale is not None:
                pCnode=createChildSceneNode(pNode, self.nodeId+'_c0')
                if self._localPosition is not None: pCnode.setPosition(self._localPosition)
                if self._localScale is not None: pCnode.setScale(self._localScale)
                if self._localOrientation is not None: pCnode.setOrientation(self._localOrientation)
                # these attributes are not for editing
                self._localPosition=None
                self._localScale=None
                self._localOrientation=None
                pCnode.attachObject(entity)
                self.pChildNode=pCnode
            else:
                pNode.attachObject(entity)    
            self.entity=entity
            self.pNode=pNode
            assert(pNode is not None)
        else:
            pNode=m.getSceneNode(self.nodeId)
            entity=self.entity
        if pNode is None:
            print('error!', self.nodeId, pNode)
            pdb.set_trace()
            assert(False)

        self.setTransform()

        if len(self._material)>0 :
            entity.setMaterialName(self._material)



    def getTransform(self, pNode):
        tt=m.transf(pNode.getOrientation(), pNode.getPosition())
        return tt

    def showBoundingBox(self, value):
        self.show_bbox=value
        self._redrawBBOX()



