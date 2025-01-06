
import os
import pdb # use pdb.set_trace() for debugging
#import code # or use code.interact(local=dict(globals(), **locals())) for debugging. see below.
import numpy as np

try:
    import luamodule as lua
    import settings
except:
    import work.luamodule as lua
    import work.settings as settings

m=lua.taesooLib()
doNotGarbageCollect=[] # list of instances which should not be garbage collected. (e.g. loaders)

def createMotionLoaderExt(fn):
    loader=m.createMotionLoaderExt_cpp(fn)
    doNotGarbageCollect.append(loader)
    return loader
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
    lua.F(('dbg', 'draw'),*args) 
def erase(*args):
    lua.F(('dbg', 'erase'),*args) 
def drawTraj(objectlist, matrix, nameid=None, color='solidred', thickness=0, linetype="LineList"):
    if not nameid: nameid=RE.generateUniqueName()
    objectlist.registerObject(nameid, linetype, color, matrix, thickness)
def timedDraw(*args):
    lua.F(('dbg', 'timedDraw'),*args) 
def drawBillboard(*args):
    lua.F(('dbg', 'drawBillboard'),*args) 
def eraseAllDrawn():
    lua.F(('dbg', 'eraseAllDrawn'))
def updateBillboards(fElapsedTime):
    lua.F(('dbg', 'updateBillboards'),fElapsedTime)
def createColorBuffer(mesh):
    lua.require("Kinematics/meshTools")
    lua.M0(mesh,'createColorBuffer')
m.Mesh.createColorBuffer=createColorBuffer # register
def drawMesh(mesh, materialName, nodeName):
    lua.require("Kinematics/meshTools")
    lua.M0(mesh,'drawMesh', materialName, nodeName)
m.Mesh.drawMesh=drawMesh # register
def motionPanel():
    return m.motionPanel()
def removeEntityByName(name):
    n=m.getSceneNode(name)
    if n:
        m.removeEntity(n)
def turnOffSoftShadows():
    lua.F(("RE","turnOffSoftShadows"))
def renderOneFrame(check):
    m.renderOneFrame(check)
def output(key, *args):
    m._output(key,str(args),1)

def output2(key, *args):
    m._output(key,str(args),2)

def createSkin(loader):
    return m.createSkin(loader)
def createFBXskin(fbx, drawSkeleton=None):
    return FBXskin(fbx, drawSkeleton)
def createVRMLskin(loader, drawSkeleton):
    return m.createVRMLskin(loader, drawSkeleton)

class Timeline(lua.instance):
    def __init__(self, title, numFrames, frameTime=None):
        lua.require('subRoutines/Timeline')
        lua.dostring("if not mEventReceiver then mEventReceiver=EVR() end")
        self.var_name='mTimeline'+m.generateUniqueName()
        if not frameTime:
            frameTime=1/30
        lua.F_lua(self.var_name,'Timeline',title,numFrames,frameTime)
    def __del__(self):# this is called when garbage collected
        if lua:
            lua.M(self.var_name,'dtor')
            lua.dostring(self.var_name+'=nil')
    def attachCameraToMotion(self, loader, motion):
        lua.M('mEventReceiver', 'attachCameraToMotion', loader, motion)
    def deattachCamera(self):
        lua.M('mEventReceiver', 'detachCamera')
    def moveCamera(self, iframe):
        lua.M('mEventReceiver', 'moveCamera', iframe)
    def attachTimer(self, frameRate, numFrames):
        lua.M(self.var_name, 'attachTimer', frameRate, numFrames)
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
        self.collisionDetector=m.getPythonWin().popCollisionDetector()
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
    def __init__(self, fbxloader):
        self.fbxloader=fbxloader
    def __getitem__(self, i):
        return FBXloader_fbxInfo(self.fbxloader, i) 

class FBXloader(lua.instance):
    def __init__(self, filename=None, options=None, **kwargs):
        lua.dostring( 'if not FBXloader then FBXloader=require("FBXloader") end')
        if 'var_name' in kwargs:
            self.var_name=kwargs['var_name']
        else:
            self.var_name='mFBXloader'+m.generateUniqueName()
        if filename:
            lua.F_lua(self.var_name,'FBXloader', filename, kwargs)
        self.fbxInfo=FBXloader_fbxInfoArray(self)
    def _get_loader(self):
        return lua.G_loader((self.var_name, "loader")) # fbx.loader
    def loadSMPLmotion(self, mocap_framerate, trans, poses,_convertYUP=False):
        return lua.F('SMPL.loadMotion', lua.instance((self.var_name,'dd')), lua.instance((self.var_name,'loader')), mocap_framerate, trans, poses, _convertYUP).copy()
    loader=property( fget=_get_loader,)

def createSMPLskeleton(bm_path:str, betas=None):
    uid=m.generateUniqueName()
    lua.dostring( 'if not FBXloader then FBXloader=require("FBXloader") end')
    var_name='mFBXloader'+uid
    lua.dostring("""
    SMPL=require("smpl/smpl")
    """)
    lua.F_lua('dd'+uid, 'SMPL.init', bm_path)
    if betas.any():
        lua.G(('dd'+uid,'betas')).range(0,len(betas)).ref()[:]=betas # change bodyshape (first 16 components)

    lua.F_lua(var_name,'SMPL.createFBXskeleton', lua.instance('dd'+uid))
    lua.dostring('dd'+uid+'=nil') # because it is backed up in fbxloader
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

    var_name='mLoader'+m.generateUniqueName()
    lua.require( "subRoutines/WRLloader")
    if filename[-8:]=='.wrl.lua' or filename[-8:]=='.wrl.dat' :
        lua.F_lua(var_name, "MainLib.WRLloader", filename)
    else:
        lua.F_lua(var_name, "MainLib.WRLloader", lua.toTable(filename))
    info={} # todo : return this when requested through an option
    info['var_name']=var_name
    return lua.G_VRMLloader(var_name)

class FBXskin(lua.instance):
    def __init__(self, fbx, drawSkeleton=None):
        self.var_name='mFBXskin'+m.generateUniqueName()
        if drawSkeleton:
            lua.F_lua(self.var_name,'RE.createFBXskin',lua.instance(fbx.var_name), True)
        else:
            lua.F_lua(self.var_name,'RE.createFBXskin',lua.instance(fbx.var_name))
    def setScale(self, x,y,z):
        lua.M0(self.var_name, 'setScale', x,y,z)
    def setPose(self, pose):
        lua.M0(self.var_name, 'setPose', pose)
    def setPoseDOF(self, pose):
        lua.M0(self.var_name, 'setPoseDOF', pose)
    def setPoseTransfer(self, pt):
        lua.M0(self.var_name, 'setPoseTransfer', pt)
    def setMaterial(self, name):
        lua.M0(self.var_name,'setMaterial', name)
    def __del__(self): # this is called when garbage collected
        if lua:
            lua.M(self.var_name,'dtor')
            lua.dostring(self.var_name+'=nil')



def drawMesh(mesh, materialName, _optionalNodeName=None, _optionalDoNotUseNormal=None):
    _optionalNodeName=_optionalNodeName or 'node_name'

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
    mot[1].var_name='info_'+nameid
    mot[1].info=mot[0]

    return mot[1].motion, mot[1]

def createMainWin(argv):
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

    m.createMainWin(int((1024+180)*uiscale),int((600+100)*uiscale), int(1024*uiscale), int(600*uiscale),uiscale)
    m.showMainWin()
    m.getPythonWin().loadEmptyScript()
    if dostring:
        m.getPythonWin().dostring(dostring)


