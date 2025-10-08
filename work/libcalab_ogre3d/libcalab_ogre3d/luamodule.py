"""
# usage
import luamodule as lua
m=lua.taesooLib()

# G : get global varibles. ex) mLoader=lua.G('mLoader_in_lua')
# F : function calls. ex) numBone=lua.F('dbg.draw','Sphere', m.vector3(1,1,1), "ball3", "green", 10)
# M : member fucntion calls. ex) numBone=lua.M('mLoader_in_lua','numBone')
"""
import pdb, re # use pdb.set_trace() for debugging
#import code # or use code.interact(local=dict(globals(), **locals())) for debugging. see below.
import numpy as np
import platform, os
hasTorch=True
try:
    import torch
except:
    print("Warning! install torch. Otherwise, many functionalities won't work")
    hasTorch=False

try:
    import settings
except:
    try:
        import work.settings as settings
        settings.relativeMode=True
    except:
        from . import default_settings as settings
        settings.relativeMode=True

mlib=settings.mlib
assert(mlib is not None)
import importlib
from easydict import EasyDict as edict # pip3 install easydict

def taesooLib():
    return mlib
def zeros(n,m=None):
    if m:
        out=mlib.matrixn(n,m)
        out.setAllValue(0)
        return out
    out=mlib.vectorn()
    out.resize(n);
    out.setAllValue(0)
    return out
def ones(n,m=None):
    if m:
        out=mlib.matrixn(n,m)
        out.setAllValue(1.0)
        return out
    out=mlib.vectorn()
    out.resize(n);
    out.setAllValue(1.0)
    return out
def eye(n):
    out=mlib.matrixn(n,n)
    out.setAllValue(0)
    out.diag().setAllValue(1)
    return out
def rand(m,n=None):
    if not n:
        return vec(np.random.rand(m))
    return mat(np.random.rand(m,n))

def require(filename):
    dostring('require("'+filename+'")')
def require_as(filename, var_name):
    dostring(var_name+'=require("'+filename+'")')
def module(filename):
    var_name= mlib.generateUniqueName()
    dostring(var_name+'=require("'+filename+'")')
    return instance(var_name)

def collectgarbage(obj=None):
    if not obj:
        dostring('collectgarbage()')
    elif 'var_name' in obj:
        dostring(obj.var_name+'=nil' )
def new(typename, *args):
    var_name=re.sub(r'\W+', '', typename)+mlib.generateUniqueName()
    F_lua(var_name, typename, *args)
    return instance(var_name)

def out(*args):
    l=mlib.getPythonWin()
    if not l.isLuaReady() : return
    # lua function call
    l.getglobal("print")
    n=len(args)
    for i in range(n):
        l.push(args[i])
    l.call(n,0)

def dostring(arg):
    l=mlib.getPythonWin()
    if not l.isLuaReady() : return
    l.dostring(arg)


def getglobal_mat(*args):
    varname=args[0]
    l=mlib.getPythonWin()
    if not l.isLuaReady() : return
    l.getglobal(varname)

    for i in range(1,len(args)):
        l.replaceTop(args[i])
    mat=l.popmatrixn()
    return mat.ref()

# call python from lua
# python function call "python.F(funcname, args)" from lua is forwarded to this function.
# python function call "python.FC(funcname, args)" from lua is forwarded to this function.
# python.F and python.FC are defined in modules.lua
def pycallFromLua(a):
    # print(a)
    # print type name
    # for i in range(len(a)): print( type(a[i]).__name__)
    try:
        if a[0]=="F":
            m=importlib.import_module(a[1])
            getattr(m, a[2])(*a[3:])
        elif a[0]=="FC":
            b=[]
            for i in range(3, len(a)): 
                tn=type(a[i]).__name__[0:7]
                if tn=='vectorn' or tn=='matrixn' or tn=='hyperma': # this includes hypermatrixn, vectornView, matrixnView
                    b.append(a[i].ref())
                else:
                    b.append(a[i])

            m=importlib.import_module(a[1])
            getattr(m, a[2])(*b)
        else:
            print('??? unknown format')
            print(a)
            pdb.set_trace()
    except Exception as e:
        print(e)
        pdb.set_trace()


# call lua from python
# lua.F and lua.F1_* assumes that all the arguments are constant (read-only).
# if you want to use the call-by-reference convention instead, use Fref*
# e.g. lua.F('print', [1,2,3])
#      lua.F('drawCNN_row', dataconv*1, mlib.vector3(200,0,0), "kinect")
#      lua.F('dbg.draw', 'Sphere',mlib.vector3(-100,20,0), 'ball1', 'red', 5)
#      a=lua.F1_vec('getCNN_row', dataconv*1, mlib.vector3(200,0,0), "kinect")
#      b=lua.F1_mat('getCNN_matrix', dataconv*1, mlib.vector3(200,0,0), "kinect")
#      b=lua.F1_dbl('getCNN_matrix', dataconv*1, mlib.vector3(200,0,0), "kinect")

# supported types: ndarray (up to 3D), tuple, mlib.vector3, mlib.quater, mlib.*...

# On the other hand, lua.Fref assumes all the arguments are read-writable (reference).
# actual implmentation uses multiple-copies. If you want to use true referencing, use mlib.matrixn() or mlib.vectorn() and their ref() functions, instead.
# e.g. 
# a=np.array([[1,2,3],[4,5,6]])
# lua.Fref('modifyMat', a)
# print(a) --> a can be modified inside lua. (but can be slow).
# b=lua.Fref1_vec('modifyMat1', a)
# b=lua.Fref1_mat('modifyMat2', a)
# b=lua.Fref1_dbl('modifyMat3', a)

# member function call
# numBone=lua.m1_int('mLoader', 'numBone')
# nf=lua.M1_int(('mDOFcontainer','mot'), 'numFrames')
def m(varname, funcname, *args):
    return F(('python', 'luaCall'), varname, funcname, *args)
def M(varname, funcname, *args):
    return F(('python', 'luaCall'), varname, funcname, *args)
def M_nocopy_userdata(varname, funcname, *args):
    return F_nocopy_userdata(('python', 'luaCall'), varname, funcname, *args)
def M_lua(out_name, varname, funcname, *args):
    F_lua(out_name, ('python', 'luaCall'), varname, funcname, *args)

def call(funcname, *args):
    out_varname=mlib.generateUniqueName()
    F_lua(out_varname, funcname, *args)
    return instance(out_varname)

def callM(var, funcname, *args):
    out_varname=mlib.generateUniqueName()
    M_lua(out_varname, var, funcname, *args)
    return instance(out_varname)


def M0(varname, funcname, *args):
    _m(0, varname, funcname, *args)
def M1_dbl(varname, funcname, *args):
    _m(1, varname, funcname, *args)
    return _popnumber();
def M1_float(varname, funcname, *args):
    _m(1, varname, funcname, *args)
    return _popnumber();
def M1_int(varname, funcname, *args):
    return int(M1_dbl( varname, funcname, *args))
def M1_mat(varname, funcname, *args):
    _m(1, varname, funcname, *args)
    return _popMat(); # list
def M1_matrixn(varname, funcname, *args):
    _m(1, varname, funcname, *args)
    return _popMatn(); #matrixn (copy)

def _popnumber():
    l=mlib.getPythonWin()
    if not l.isnil(l.gettop()):
        v=l.popnumber()
        return v
    return None
def _popstring():
    l=mlib.getPythonWin()
    if not l.isnil(l.gettop()):
        v=l.popstring()
        return v
    return None
def _popboolean():
    l=mlib.getPythonWin()
    if not l.isnil(l.gettop()):
        v=l.popboolean()
        return v
    return None

# lua.setGlobal('mLoader', loader)
def setGlobal(varname, value):
    F(("python","setGlobal"), varname, value)


# G (getGlobal) functions returns a reference (==native taesooLib class).
def G(*args):
    l=mlib.getPythonWin()
    _getGlobal(*args)
    return _popAuto(l)

def G_copy(*args):
    l=mlib.getPythonWin()
    _getGlobal(*args)
    return _popAutoCopy(l)
def G_int(varname):
    _getGlobal(varname)
    return int(_popnumber())
def G_str(varname):
    _getGlobal(varname)
    return _popstring()
def G_bool(varname):
    _getGlobalNoCheck(varname)
    return _popboolean()
def G_float(varname):
    _getGlobal(varname)
    return _popnumber()
def G_vectorn(varname): # reference
    return G_vec(varname)
def G_intvectorn(varname): # reference
    return G_ivec(varname)
def G_matrixn(varname): # reference
    return G_mat(varname)
def G_MotionLoader(varname): # reference
    return G_loader(varname)
def G_vector3N(varname): # reference
    _getGlobal(varname)
    l=mlib.getPythonWin()
    if not l.isLuaReady() : return
    return l.popVector3N()
def G_quaterN(varname): # reference
    _getGlobal(varname)
    l=mlib.getPythonWin()
    if not l.isLuaReady() : return
    return l.popQuaterN()
def G_VRMLloader(varname):
    _getGlobal(varname)
    l=mlib.getPythonWin()
    if not l.isLuaReady() : return
    return l.popVRMLloader()
def G_boolN(varname):
    _getGlobal(varname)
    l=mlib.getPythonWin()
    if not l.isLuaReady() : return
    return l.popboolN()

def G_dbl(varname):
    _getGlobal(varname)
    return _popnumber()

def G_vec(varname):
    _getGlobal(varname)
    l=mlib.getPythonWin()
    if not l.isLuaReady() : return
    return l.popvectorn()

def G_mat(varname):
    _getGlobal(varname)
    l=mlib.getPythonWin()
    if not l.isLuaReady() : return
    return l.popmatrixn()

def G_ivec(varname):
    _getGlobal(varname)
    l=mlib.getPythonWin()
    if not l.isLuaReady() : return
    return l.popintvectorn()


def G_loader(varname):
    _getGlobal(varname)
    l=mlib.getPythonWin()
    if not l.isLuaReady() : return
    return l.poploader()
def G_motion(varname):
    _getGlobal(varname)
    l=mlib.getPythonWin()
    if not l.isLuaReady() : return
    return l.popMotion()
def G_motionDOF(varname):
    _getGlobal(varname)
    l=mlib.getPythonWin()
    if not l.isLuaReady() : return
    return l.popMotionDOF()


# you probably want to use G  instead of _getGlobal
# _getGlobal don't bring the results to python-side. 
# it just leaves them in the lua stack.
def _getGlobal(*args):
    l=mlib.getPythonWin()
    if not l.isLuaReady() : return

    if len(args)==1:
        if isinstance(args[0], instance):
            args=[args[0].var_name]
        if isinstance(args[0], tuple) or isinstance(args[0], list):
            args=args[0]
        else:
            assert(isinstance(args[0], str))
            if '.' in args[0]:
                args=args[0].split('.')

    l.getglobal(args[0])
    for i in range(1, len(args)):
        l.replaceTop(args[i])

def _getGlobalNoCheck(varname):
    l=mlib.getPythonWin()
    if not l.isLuaReady() : return
    if isinstance(varname,tuple):
        l.getglobalNoCheck(*varname)
    else:
        l.getglobalNoCheck(varname)

# member function call (fixed # of return values
def _m(numout, varname, funcname, *args):
    _F(numout,('python', 'luaCall'), varname, funcname, *args)


# this is the most convenient version to use. (but can be a little slower)
# global function call (variable-length return) 
def F(*args):
    funcname=args[0]
    _getGlobal(funcname)
    l=mlib.getPythonWin()
    ctop=l.gettop()-1

    tempvar=[] # this list prevents immediate garbage collection of temporary python-variables during a single lua function call.
    for i in range(1,len(args)):
        _push(l, args[i], tempvar)
    stack=l.call(len(args)-1)

    numOut=l.gettop()-ctop
    if numOut>0:
        out=[]
        for i in range(numOut):
            tid=l.luaType(-1)
            if tid==7: # userdata -> copy
                tn=l.lunaType(-1)
                out.insert(0, _popUserdata(l,tn).copy())
            elif tid==0: #nil
                # simply ignore
                pass
            elif tid==1: #bool
                out.insert(0, _popboolean())
            elif tid==4: # string
                out.insert(0, _popstring())
            elif tid==3: #number
                out.insert(0, _popnumber())
            elif tid==5: # table -> make a global backup in lua, and return a reference
                res=_popAuto(l, False) # no stack cleanup for backup
                out.insert(0, res) # reference
                var_name='out_'+mlib.generateUniqueName()
                # backup to lua global to prevent garbage collection
                if isinstance(res, dict):
                    res['var_name']=var_name 
                l.set(var_name)
            else:
                print('other cases not implemented yet')
                l.printStack()
                pdb.set_trace()
        if len(out)==1:
            return out[0]
        return tuple(out)


# do not use unless you know what you are doing
def F_nocopy_userdata(*args):
    funcname=args[0]
    _getGlobal(funcname)
    l=mlib.getPythonWin()
    ctop=l.gettop()-1

    tempvar=[] # this list prevents immediate garbage collection of temporary python-variables during a single lua function call.
    for i in range(1,len(args)):
        _push(l, args[i], tempvar)
    stack=l.call(len(args)-1)

    numOut=l.gettop()-ctop
    if numOut>0:
        out=[]
        for i in range(numOut):
            tid=l.luaType(-1)
            if tid==7: # userdata -> no copy
                tn=l.lunaType(-1)
                out.insert(0, _popUserdata(l,tn))
            elif tid==0: #nil
                # simply ignore
                pass
            elif tid==1: #bool
                out.insert(0, _popboolean())
            elif tid==4: # string
                out.insert(0, _popstring())
            elif tid==3: #number
                out.insert(0, _popnumber())
            elif tid==5: # table -> make a global backup in lua, and return a reference
                res=_popAuto(l, False) # no stack cleanup for backup
                out.insert(0, res) # reference
                var_name='out_'+mlib.generateUniqueName()
                # backup to lua global to prevent garbage collection
                if isinstance(res, dict):
                    res['var_name']=var_name 
                l.set(var_name)
            else:
                print('other cases not implemented yet')
                l.printStack()
                pdb.set_trace()
        if len(out)==1:
            return out[0]
        return tuple(out)

def F_lua(out_name, *args):
    _F(1, *args)
    l=mlib.getPythonWin()
    if not l.isnil(l.gettop()): l.set(out_name)

#assumes that the lua function returns a matrix
def F1_mat(*args):
    _F(1, *args)
    return _popMat()
def F1_npmat(*args):
    _F(1, *args)
    return _popNumpyMat()
def F0(*args): # discard return values
    _F(0, *args)

def _popMat():
    l=mlib.getPythonWin()
    mat=l.checkmatrixn()
    out=mat.ref().tolist() # copy (because "mat" is a temporary variable that cannot go beyond its lua stack frame)
    l.pop()
    return out
def _popMatn():
    l=mlib.getPythonWin()
    mat=l.checkmatrixn()
    # copy (because "mat" is a temporary variable that cannot go beyond its lua stack frame)
    mat2=mlib.matrixn()
    mat2.assign(mat)
    l.pop()
    return mat2
def _popNumpyMat():
    l=mlib.getPythonWin()
    mat=l.checkmatrixn()
    out=mat.ref().copy() # copy
    l.pop()
    return out

def _popVec():
    l=mlib.getPythonWin()
    if not l.isnil(l.gettop()):
        out=l.checkvectorn().ref().tolist()#copy
        l.pop()
        return out
    return None

def _popIVec():
    l=mlib.getPythonWin()
    if not l.isnil(l.gettop()):
        out=l.checkintvectorn().ref().tolist() #copy
        l.pop()
        return out
    return None
def printStack():
    l=mlib.getPythonWin()
    if not l.isnil(l.gettop()):
        l.printStack()

#assumes that the lua function returns a vector
def F1_vec(*args):
    #l.printStack()
    _F(1, *args)
    return _popVec()
def F1_ivec(*args):
    #l.printStack()
    _F(1, *args)
    return _popIVec()
    #l.printStack()

def F1_dbl(*args):
    #l.printStack()
    _F(1, *args)
    #l.printStack()
    l=mlib.getPythonWin()
    if not l.isnil(l.gettop()):
        v=l.popnumber()
        return v
    return None
def F1_int(*args):
    return int(F1_dbl(*args))

def Fref(*args):
    _Fref(0, *args)
def Fref1_mat(*args):
    _Fref(1, *args)
    return _popMat()

#assumes that the lua function returns a vector
def Fref1_vec(*args):
    #l.printStack()
    _Fref(1, *args)
    #l.printStack()
    l=mlib.getPythonWin()
    if not l.isnil(l.gettop()):
        vec=l.popvectorn()
        return vec.ref().tolist()
    return None

def Fref1_dbl(*args):
    #l.printStack()
    _Fref(1, *args)
    #l.printStack()
    l=mlib.getPythonWin()
    if not l.isnil(l.gettop()):
        v=l.popnumber()
        return v
    return None
# converts numpy array to vectorn, matrixn or hypermatrixn.
def array(ndarr):
    if len(ndarr.shape)==1:
        temp=mlib.vectorn()
        temp.setSize(ndarr.shape[0])
        temp.ref()[:]=ndarr
        return temp
    elif len(ndarr.shape)==2:
        temp=mlib.matrixn(ndarr.shape[0], ndarr.shape[1])
        temp.ref()[:,:]=ndarr
        return temp
    elif len(ndarr.shape)==3:
        temp=mlib.hypermatrixn(ndarr.shape[0], ndarr.shape[1], ndarr.shape[2])
        for j in range(temp.pages()):
            temp.page(j).ref()[:,:]=ndarr[j]
        return temp
    else:
        assert(False)
    return None

# the return values will be inside the lua stack. See F1_mat and F1_vec for retrieving the return values.
def _F(numout, *args):
    funcname=args[0]
    _getGlobal(funcname)
    l=mlib.getPythonWin()

    tempvar=[]
    for i in range(1,len(args)):
        _push(l, args[i], tempvar)
    l.call(len(args)-1,numout)

# assumes all the arguments are read-writable (reference).
# e.g. 
# a=np.array([[1,2,3],[4,5,6]])
# lua.Fref('modifyMat', a)
# print(a)

def _Fref(numout, *args):
    funcname=args[0]
    l=mlib.getPythonWin()
    if not l.isLuaReady() : return
    if isinstance(funcname,tuple):
        l.getglobal(*funcname)
    else:
        l.getglobal(funcname)

    out=[None] * len(args)
    for i in range(1,len(args)):
        if isinstance(args[i], np.ndarray):
            temp=array(args[i])
            l.push(temp)
            out[i]=temp   # will be updated after the lua function finishes.
        else:
            l.push(args[i])
    l.call(len(args)-1,numout)
    for i in range(1,len(args)):
        if out[i] and args[i].flags["WRITEABLE"]:
            if len(args[i].shape)==1:
                vec=args[i]
                vec[:]= out[i].ref()
            elif len(args[i].shape)==2:
                mat=args[i]
                mat[:,:]= out[i].ref()
            else:
                assert(False)
def init_console():
    # here, you cannot use renderer!!! but all other non-rendering-related classes can be used.
    # this is different from console.mainlib in that console.mainlib supports skins and renderer too.
    uiscale=1
    taesooLibPath=''
    if not os.path.exists(taesooLibPath+ "../Resource/ogreconfig_linux_sepwin.txt"):
        taesooLibPath= 'work/taesooLib'
    mlib._createInvisibleMainWin()
    mlib.getPythonWin().loadEmptyScript()

# simply forward events to lua
def onCallback(mid, userdata):
    l=mlib.getPythonWin()

    if not l.isLuaReady() : return
    # lua function call
    #l.dostring("function test(a,b) print(a,b) end")
    #l.getglobal("test")
    l.getglobal("onCallback")
    l.push(mid)
    l.push(userdata)
    #l.printStack()
    l.call(2,0)

def handleRendererEvent(ev, button, x, y):
    #print( ev, button, x, y)
    l=mlib.getPythonWin()


    #l.printStack()
    if not l.isLuaReady() : return 0
    l.getglobalNoCheck("handleRendererEvent")
    if not l.isnil(l.gettop()):
        l.push(ev)
        l.push(button)
        l.push(x)
        l.push(y)
        l.call(4,1)
        return l.popnumber()
    else:
        l.pop()
    return 0


def onFrameChanged(currFrame):
    l=mlib.getPythonWin()
    if not l.isLuaReady() : return
    # lua function call
    l.getglobalNoCheck("onFrameChanged")
    if l.isnil(l.gettop()):
        l.pop()
    else:
        l.push(currFrame)
        l.call(1,0)

def frameMove(fElapsedTime):
    l=mlib.getPythonWin()
    if not l.isLuaReady() : return
    # lua function call
    l.getglobalNoCheck("frameMove")
    if l.isnil(l.gettop()):
        l.pop()
    else:
        l.push(fElapsedTime)
        l.call(1,0)

def dofile(fn):
    mlib.getPythonWin().dofile(fn)


def vec(*args): # fot float arrays
    out=mlib.vectorn()
    if len(args)==1:
        v=args[0]
        if isinstance(v, np.ndarray):
            out.setSize(v.shape[0])
            if(out.size()>0):
                out.ref()[:]=v
        elif isinstance(v, torch.Tensor):
            #return vec(v.detach().cpu().numpy())
            v=v.detach().cpu().numpy()
            out.setSize(v.shape[0])
            if(out.size()>0):
                out.ref()[:]=v
        elif not isinstance(v, float):
            out.assign(v)
        return out
    out.setSize(len(args))
    out.ref()[:]=args
    return out
def ivec(*args): # for int or boolean arrays
    if len(args)==1:
        v=args[0]
        if isinstance(v, np.ndarray):
            if v.size==0:
                return mlib.intvectorn()
            elif isinstance(v[0],np.bool_):
                out=mlib.boolN(v.size)
                out.assign(v)
                return out
            else:
                out=mlib.intvectorn(v.size)
                out.ref()[:]=v
                return out
        elif isinstance(v, list):
            out=mlib.intvectorn(len(v))
            out.assign(v)
            return out
        elif isinstance(v, torch.Tensor):
            #return vec(v.detach().cpu().numpy())
            return ivec(v.detach().cpu().numpy())

    out=mlib.intvectorn()
    out.setSize(len(args))
    out.ref()[:]=args
    return out
def mat(*args):
    if len(args)==1:
        v=args[0]
        out=mlib.matrixn()
        if isinstance(v, np.ndarray):
            out.setSize(v.shape[0], v.shape[1])
            out.ref()[:,:]=v
        elif isinstance(v, torch.Tensor):
            v=v.detach().cpu().numpy()
            out.setSize(v.shape[0], v.shape[1])
            out.ref()[:,:]=v
        else:
            ncol=len(v[0])
            out.setSize(len(v), ncol)
            for i in range(len(v)):
                out.row(i).assign(v[i])
        return out

    else:
        out=mlib.matrixn()
        n=args[0]
        m=args[1]
        out.setSize(n,m)
        assert(len(args)==n*m+2)
        c=2
        for i in range(n):
            for j in range(m):
                out.set(i,j,args[c])
                c+=1
        return out

def tensor(v):
    if len(v.shape)==1:
        return vec(v)
    elif len(v.shape)==2:
        return mat(v)
    else:
        if isinstance(v, np.ndarray):
            out=mlib.Tensor(*v.shape)
            np.copyto(out.ref(),v)
            return out
        elif isinstance(v, torch.Tensor):
            v=v.detach().cpu().numpy()
            out=mlib.Tensor(*v.shape)
            np.copyto(out.ref(),v)
            return out
        else:
            pdb.set_trace()

def tempFunc(self):
   rot_y=mlib.quater()
   offset=mlib.quater()
   self.decompose(rot_y, offset)
   return offset
mlib.quater.offsetQ=tempFunc
def tempFunc(self, xPos, out):
    return M(self, 'getTerrainHeight', xPos, out)
mlib.VRMLloader.getTerrainHeight=tempFunc
def tempFunc(self, ray, normal):
    return M(self, 'pickTerrain', ray, normal)
mlib.VRMLloader.pickTerrain=tempFunc

def tempFunc(self, bones=None):
    if bones is None :
        # automatic annotation
        require_as('retargetting/module/retarget_common', 'RET')
        F('RET.setVoca', self)
        return

    strToVoca={     
            "hips":mlib.Voca.HIPS,
            "left_hip":mlib.Voca.LEFTHIP,
            "left_knee":mlib.Voca.LEFTKNEE,
            "left_heel":mlib.Voca.LEFTANKLE,
            "left_ankle":mlib.Voca.LEFTANKLE,
            "left_ball":mlib.Voca.LEFTTOES,
            "left_collar":mlib.Voca.LEFTCOLLAR,
            "left_shoulder":mlib.Voca.LEFTSHOULDER,
            "left_elbow":mlib.Voca.LEFTELBOW,
            "left_wrist":mlib.Voca.LEFTWRIST,
            "right_hip":mlib.Voca.RIGHTHIP,
            "right_knee":mlib.Voca.RIGHTKNEE,
            "right_heel":mlib.Voca.RIGHTANKLE,
            "right_ankle":mlib.Voca.RIGHTANKLE,
            "right_ball":mlib.Voca.RIGHTTOES,
            "right_collar":mlib.Voca.RIGHTCOLLAR,
            "right_shoulder":mlib.Voca.RIGHTSHOULDER,
            "right_elbow":mlib.Voca.RIGHTELBOW,
            "right_wrist":mlib.Voca.RIGHTWRIST,
            "chest":mlib.Voca.CHEST,
            "chest2":mlib.Voca.CHEST2,
            "neck":mlib.Voca.NECK,
            "head":mlib.Voca.HEAD,
            }
    for k,v in bones.items():
        print(k,v)
        if self.getTreeIndexByName(v)>0:
            if isinstance(k, str):
                if strToVoca[k]!=None :
                    self._changeVoca(strToVoca[k], self.getBoneByName(v))
                else:
                    print("error?", k)
                    pdb.set_trace()
            else:
                self._changeVoca(v, self.getBoneByName(v))
        else:
            print("can't find", v)
            pdb.set_trace()

    # automatic vocaburary assignment of child bones
    def derive(voca_parent, voca_child):

        if self.getTreeIndexByVoca(voca_parent)!=-1 and self.getTreeIndexByVoca(voca_child)==-1 :
            pbone=self.getBoneByVoca(voca_parent)
            pchildbone=pbone.childHead()
            if pchildbone :
                assert(pchildbone.voca()==-1)
                self._changeVoca(voca_child, pchildbone)

    derive(mlib.Voca.LEFTCOLLAR, mlib.Voca.LEFTSHOULDER)
    derive(mlib.Voca.LEFTSHOULDER, mlib.Voca.LEFTELBOW)
    derive(mlib.Voca.LEFTELBOW, mlib.Voca.LEFTWRIST)
    derive(mlib.Voca.RIGHTCOLLAR, mlib.Voca.RIGHTSHOULDER)
    derive(mlib.Voca.RIGHTSHOULDER, mlib.Voca.RIGHTELBOW)
    derive(mlib.Voca.RIGHTELBOW, mlib.Voca.RIGHTWRIST)


mlib.MotionLoader.setVoca=tempFunc
tempFunc=None
def tempFunc(self , row, minimumNumRows=None):
    if minimumNumRows and self.rows()<minimumNumRows:
        self.pushBack(row)
        return
    self.ref()[:-1, :]=self.ref()[1:,:]
    self.ref()[-1]=row.ref()
mlib.matrixn.enqueue=tempFunc # rows는 그대로 둔채 앞에서 하나 빠지고 뒤로 하나 추가됨. 
mlib.vector3N.enqueue=tempFunc # rows는 그대로 둔채 앞에서 하나 빠지고 뒤로 하나 추가됨. 

def tempFunc(self, frame_rate, discontinuity=None):
    return M(self, 'derivative', frame_rate, discontinuity)
mlib.matrixn.derivative=tempFunc

def tempFunc(self,other):
    self_ref=self.ref()
    if isinstance(other, float):
        self_ref+=other
    else:
        self_ref+=other.ref()
mlib.matrixn.radd=tempFunc

def tempFunc(self , nrows):
    if nrows<=self.rows():
        self.resize(self.rows()-nrows,self.cols())
    else:
        self.resize(0,self.cols())
mlib.matrixn.popBack=tempFunc # rows는 그대로 둔채 앞에서 하나 빠지고 뒤로 하나 추가됨. 

def tempFunc(self , nrows=1):
    out=mat(self.ref()[:nrows,:])
    self.ref()[:-nrows, :]=self.ref()[nrows:,:]
    self.resize(self.rows()-nrows, self.cols())
    return out
mlib.matrixn.dequeue=tempFunc # 앞에서 nrows개 빠짐. 

tempFunc=None
def tempFunc(self ):
    out=None
    m=mlib
    typen=self._unpackInt()
    if typen==0: #util.BinaryFile.TYPE_INT :
        out=self._unpackInt()
    elif typen==1: #util.BinaryFile.TYPE_FLOAT :
        out=self._unpackFloat()
    elif typen==2: #util.BinaryFile.TYPE_FLOATN :
        v=m.vectorn()
        self._unpackVec(v)
        out=v
    elif typen==5 : # util.BinaryFile.TYPE_FLOATMN
        m=m.matrixn()
        self._unpackMat(m)
        out=m
    elif typen==-5 :
        m=m.hypermatrixn()
        self._unpackMat(m)
        out=m
    elif typen==13 :
        v=m.vectorn()
        self._unpackSPVec(v)
        return v
    elif typen==14 :
        v=m.matrixn()
        self._unpackSPMat(v)
        return v
    elif typen==8 : # util.BinaryFile.TYPE_STRING 
        out=self._unpackStr()
    elif typen==11 :#util.BinaryFile.TYPE_EOF :
        return nil
    elif typen==4:#util.BinaryFile.TYPE_BITN :
        bb=m.boolN()
        self._unpackBit(bb)
        out=bb
    elif typen==3 : # TYPE_INTN
        bb=m.intvectorn()
        self._unpackVec(bb)
        out=bb
    elif typen==9 :
        n=self._unpackInt();
        out=m.TStrings()
        out.resize(n)
        for i in range(0, n-1 +1):
            out.set(i, self.unpackStr())

    else:
        print('unpackAny ' +str(typen)+' has not been implemented yet')
        RE.console()

    return out


mlib.BinaryFile.unpackAny=tempFunc
tempFunc=None
def tempFunc(self):
    v=mlib.vectorn(6)
    V=mlib.se3()
    V.log(self)
    v.setVec3(0, V.W())
    v.setVec3(3, V.V())
    return v
mlib.transf.toLogVec=tempFunc
def tempFunc(self, t):
    out=self.copy()
    out.translation.radd(t)
    return out
mlib.transf.translate=tempFunc
tempFunc=None
def tempFunc(self):
    V=mlib.se3()
    V.W().assign(self.toVector3(0))
    V.V().assign(self.toVector3(3))
    return V
mlib.vectorn.to_se3=tempFunc
tempFunc=None
def tempFunc(self):
    out=mlib.vectorn(7)
    out.setTransf(0, self)
    return out
mlib.transf.toVector=tempFunc
tempFunc=None

# a=dynamic_list()   a[3]=7   이런식으로 초기화 없이 즉시 사용가능한 list. 0-indexed
class dynamic_list(list):
    def __init__(self,num_gen=0):
        self._num_gen = num_gen
    def __getitem__(self,index):
        if isinstance(index, int):
            self.expandfor(index)
            return super(dynamic_list,self).__getitem__(index)

        elif isinstance(index, slice):
            if index.stop<index.start:
                return super(dynamic_list,self).__getitem__(index)
            else:
                self.expandfor(index.stop if abs(index.stop)>abs(index.start) else index.start)
            return super(dynamic_list,self).__getitem__(index)

    def __setitem__(self,index,value):
        if isinstance(index, int):
            self.expandfor(index)
            return super(dynamic_list,self).__setitem__(index,value)

        elif isinstance(index, slice):
            if index.stop<index.start:
                return super(dynamic_list,self).__setitem__(index,value)
            else:
                self.expandfor(index.stop if abs(index.stop)>abs(index.start) else index.start)
            return super(dynamic_list,self).__setitem__(index,value)

    def expandfor(self,index):
            rng = []
            if abs(index)>len(self)-1:
                if index<0:
                    rng = range(abs(index)-len(self))
                else:
                    rng = range(abs(index)-len(self)+1)
            for i in rng:
                self.append(None)
def _popAuto(l,stackCleanup=True): # reference
    tid=l.luaType(-1)
    if tid==7:
        tn=l.lunaType(-1)
        return _popUserdata(l, tn)
    elif tid==4:
        return l.popstring()
    elif tid==3:
        return l.popnumber()
    elif tid==0:
        return None
    elif tid==1:
        return l.popboolean()
    elif tid==2:
        l.pop()
        return "lightuserdata" # lightuserdata
    elif tid==6:
        l.pop()
        return "function" # function
    elif tid==5:
        # nested table? not yet.
        l.pushnil();
        out2=Table()
        out=[]
        isDict=False
        while (l.next(-2)):
            # stack now contains: -1 => value; -2 => key; -3 => table
            # copy the key so that lua_tostring does not modify the original
            l.pushvalue(-2);
            # stack now contains: -1 => key; -2 => value; -3 => key; -4 => table
            key=_popAuto(l)
            value=_popAuto(l)
            if not isDict and isinstance(key, str):
                try:
                    out2[key]=value
                except:
                    isDict=True
                    out2=dict(out2)
                    out2[key]=value
            elif out!=None and key==len(out)+1:
                out.append(value)
            else: # easydict only supports string keys so switch to native dict.
                isDict=True
                out2=dict(out2)
                out2[key]=value
            if isDict and out:
                for ii in range(len(out)):
                    out2[ii+1]=out[ii] # +1 so that it is compatible with other keys. (lua uses 1-indexing)
                out=None
        if stackCleanup:
            l.pop()
        if out!=None and len(out)>0:
            if len(out2)>0:
                out2['_Table__list']=out
                return out2
            return out
        return out2
    else:
        print('other cases not implemented yet')
        l.printStack()
        pdb.set_trace()
def _popAutoCopy(l,stackCleanup=True): # reference
    tid=l.luaType(-1)
    if tid==7:
        tn=l.lunaType(-1)
        return _popUserdata(l, tn).copy()
    elif tid==4:
        return l.popstring()
    elif tid==3:
        return l.popnumber()
    elif tid==0:
        return None
    elif tid==1:
        return l.popboolean()
    elif tid==2:
        l.pop()
        return "lightuserdata" # lightuserdata
    elif tid==6:
        l.pop()
        return "function" # function
    elif tid==5:
        # nested table? not yet.
        l.pushnil();
        out2=edict()
        out=[]
        while (l.next(-2)):
            # stack now contains: -1 => value; -2 => key; -3 => table
            # copy the key so that lua_tostring does not modify the original
            l.pushvalue(-2);
            # stack now contains: -1 => key; -2 => value; -3 => key; -4 => table
            key=_popAutoCopy(l)
            value=_popAutoCopy(l)
            if isinstance(key, str):
                out2[key]=value
            else:
                assert(key==len(out)+1)
                out.append(value)
        if stackCleanup:
            l.pop()
        if len(out)>0:
            if len(out2)>0:
                return (out, out2)
            return out
        return out2
    else:
        print('other cases not implemented yet')
        l.printStack()
        pdb.set_trace()

def _popUserdata(l, tn):
    if tn[0:7]=='matrixn' :
        return l.popmatrixn()
    elif tn=='Pose':
        return l.popPose()
    elif tn[0:7]=='vectorn':
        return l.popvectorn()
    elif tn[0:10]=='intvectorn':
        return l.popintvectorn()
    elif tn[0:5]=='boolN':
        return l.popboolN()
    elif tn[0:9]=='MotionDOF':
        return l.popMotionDOF()
    elif tn[0:8]=='vector3N':
        return l.popVector3N()
    elif tn[0:7]=='quaterN':
        return l.popQuaterN()
    elif tn=='vector3':
        return l.popvector3()
    elif tn=='vector2':
        return l.popvector2()
    elif tn=='quater':
        return l.popquater()
    elif tn=='matrix4':
        return l.popmatrix4()
    elif tn=='TStrings':
        return l.popTStrings()
    elif tn[-12:]=='MotionLoader':
        return l.poploader()
    elif tn=='Motion':
        return l.popMotion()
    elif tn[-10:]=='VRMLloader':
        return l.popVRMLloader()
    elif tn[-12:]=='LoaderToTree':
        return l.popLoaderToTree()
    elif tn=='transf':
        return l.poptransf()
    elif tn=='Tensor':
        return l.popTensor()
    elif tn=='matrix3':
        return l.popMatrix3()
    elif tn=='intIntervals':
        return l.popIntIntervals()
    elif tn[-20:]=="ScaledBoneKinematics":
        return l.popScaledBoneKinematics()
    elif tn[-14:]=="BoneForwardKinematics":
        return l.popBoneForwardKinematics()
    else:
        print('warning! luamodule._popUserdata: this case not implemented yet:', tn)
        l.pop()
        return tn
def _pushList(l, arg, tempvar):
    n=len(arg)
    for j in range(n):
        l.push(j+1) # convert to 1-indexing
        _push(l, arg[j], tempvar)
        l.settable(-3)

def _push(l, arg, tempvar):
    if isinstance(arg,np.ndarray):
        t=array(arg)
        tempvar.insert(0, t) # to prevent immediate garbage collection
        l.push(t)
    elif isinstance(arg, ArgumentProcessor):
        arg.push(l)
    elif isinstance(arg, bool):
        l.pushBoolean(arg)
    elif isinstance(arg, dict):
        l.newtable()  # push a lua table
        for k, v in arg.items():
            if k=='_Table__list' :
                if v is not None:
                    _pushList(l, v, tempvar)
            else:
                l.push(k)
                _push(l,v, tempvar)
                l.settable(-3)
    elif isinstance(arg, list) or isinstance(arg, tuple):
        l.newtable()  # push a lua table
        _pushList(l, arg, tempvar)
    elif arg is None:
        l.pushnil()
    else:
        try:
            l.push(arg)
        except:
            if arg!=None:
                print('push: unknown type.', arg)
            l.pushnil()
class ArgumentProcessor:
    def __init__(self):
        pass
    def push(self):
        pass

# F('printTable', lua.toTable("return { a=3, b=4}"))
class toTable(ArgumentProcessor):
    def __init__(self, python_string: str):
        self.tableString=python_string
    def push(self, l):
        dostring(self.tableString)
        l.insert(-2)
        l.pop()

"""
a wrapper class of a lua variable (any type)
"""
class instance(ArgumentProcessor):
    # lua.instance("lua variable name") can also be used as an lua.F(*) argument.
    def __init__(self, python_var):
        if isinstance(python_var, str):
            self.var_name=python_var
        elif isinstance(python_var, tuple):
            self.var_name=list(python_var)
        elif isinstance(python_var, list):
            self.var_name=python_var
        else:
            self.var_name=python_var.var_name
        self.dependent=[]
    def push(self, l):
        _getGlobal(self.var_name)
    def _addToVarName(self, str_name):
        var_name=self.var_name
        if isinstance(var_name, list):
            return var_name+[str_name]
        elif isinstance(var_name, tuple):
            return list(var_name)+[str_name]
        else:
            return [var_name, str_name]
    # member access functions
    def len(self):
        return F('table.getn', self)
    def __getattr__(self, name):
        return instance(self._addToVarName(name))
    def __getitem__(self, index):
        return instance(self._addToVarName(index))

    def get(self, propertyName): # reference
        #return G(self.var_name, propertyName)
        return F_nocopy_userdata('python.getGlobal', self._addToVarName( propertyName))
    def copyget(self, propertyName): # copy
        return G_copy(self.var_name, propertyName)
    def set(self, propertyName, value):
        return setGlobal(self._addToVarName(propertyName), value)
    def hasKey(self, key):
        return F('python.hasKey', self, key)
    # function calls
    # e.g. variable('functionName', arg1, ...)
    def __call__(self, funcname, *args):   
        # copies user data
        return M(self, funcname, *args)
    def call_nocopy(self, *args):
        return M_nocopy_userdata(self, *args)

    # use when return value has to be a lua instance. otherwise, use __call__
    def call(self, funcname, *args):
        # hashing
        var_name=('_'.join(self.var_name)+funcname).replace(".", "_")
        if var_name not in self.dependent:
            self.dependent.append(var_name)
        M_lua(var_name, self.var_name, funcname, *args)
        return instance(var_name)

    def __repr__(self):
        return 'lua instance :\n'+F('dbg.tostring', self)
    def collect(self): # manual collection. (automatic collection is possible, but often is dangerous, so I didn't implement it.)
        dostring(self.var_name+'=nil')
        for i in self.dependent:
            dostring(d+'=nil')
def toDict(python_str):
    if isinstance(python_str , str):
        return F('python.identityFunction', toTable('return '+python_str))
    elif isinstance(python_str, list):
        out={}
        for i in range(len(python_str)):
            v=python_str[i]
            out[i+1]=v # lua uses 1-indexing.
        return out
        
def escapeString(a_string):
    return a_string.translate(str.maketrans({"-":  r"\-",
                                          "]":  r"\]",
                                          "\\": r"\\",
                                          "^":  r"\^",
                                          "$":  r"\$",
                                          "*":  r"\*",
                                          ".":  r"\."}))
def console():
    dostring('dbg.console()')
def addPackagePath(path):
    script="""
    package.path=package.path..';"""+ str(path)+"/?.lua'"
    if platform.system()=='Windows':
        script="""
        package.path=package.path..';"""+ escapeString(str(path))+"\\\\?.lua'"
    print(script)
    dostring(script)

    
def vector3N_slice(self, s, e):
    if s<0:
        s=self.size()+s
    if e<=0:
        e=self.size()+e
        return self.range(s,e)
mlib.vector3N.slice=vector3N_slice
del vector3N_slice
mlib.matrixn.slice=mlib.matrixn.sub

def convertFloatTableToInt(tbl):
    out=None
    if isinstance(tbl, edict):
        out=edict()
    elif isinstance(tbl, dict):
        out={}
    elif isinstance(tbl, list):
        out=[None]*len(tbl)
        for i , v in enumerate(tbl):
            if isinstance(v, float):
                if v.is_integer():
                    out[i]=int(v)
                    continue
            out[i]=convertFloatTableToInt(v)
        return out
    elif isinstance(tbl, float) and tbl.is_integer():
        return int(tbl)
    else:
        return tbl

    for k , v in tbl.items():
        if isinstance(k, float):
            if k.is_integer():
                out[int(k)]=convertFloatTableToInt(v)
                continue
        out[k]=convertFloatTableToInt(v)
    return out

def printVec(v):
    print('lua.vec(np.'+ np.array_repr(v.ref())+')')
mlib.VRMLloader.findBone=mlib.VRMLloader.getBoneByName
def tempFunc(self, n):
    pn=self.size()
    self.resize(max(n,pn))
    self.resize(pn)
mlib.intvectorn.reserve=tempFunc

def tempFunc(self,value):
    indices=mlib.intvectorn()
    indices.findIndex(self, value)
    out=mlib.boolN(self.size())
    for i in range(indices.size()):
        out.set(indices(i), True)
    return out
mlib.intvectorn.getMask=tempFunc

def tempFunc(self, w_id, items):
    self.create("Choice", w_id,'', 0)
    self.widget(0).menuSize(len(items))
    for i in range(len(items)):
        self.widget(0).menuItem(i, items[i])
    self.widget(0).menuValue(0)
mlib.FlLayout.addMenu=tempFunc
def tempFunc(self):
    return self.ref().copy()
mlib.vectorn.numpy=tempFunc
mlib.matrixn.numpy=tempFunc
mlib.vector3.numpy=tempFunc
mlib.quater.numpy=tempFunc
mlib.vector3N.numpy=tempFunc
mlib.quaterN.numpy=tempFunc
mlib.Tensor.numpy=tempFunc

def tempFunc(self, fcn, *args):
    return M(self, fcn, *args)

mlib.LuaScript.__call__=tempFunc
mlib.ThreadScriptPool.__call__=tempFunc

del tempFunc

# dict + list. see testTable.py
class Table(dict):
    def __init__(self,*args, **kwargs):
        for key in kwargs:
            self[key]=kwargs[key]
        if len(args)>0:
            self.__list=list(args)

    def __getattr__(self, key):
        if key in self:
            return self[key]
        return None
    def __setattr__(self, key, value):
        self[key]=value
    def __call__(self, key):
        return self.__list[key-1]  # 1-indexing
    def set(self, i, v):
        self.__list[i-1]=v  # 1-indexing

    def get_array(self):
        return self.__list

    def set_array(self, new_value):
        self.__list = new_value

    def del_array(self):
        del self.__list
    def __repr__(self):
        out='L('
        if self.array is not None and len(self.array)>0:
            out+=list.__repr__(self.array)[1:-1]+', '
        for k, v in self.items():
            if k=='_Table__list' :
                pass
            else:
                out+=f"{k}={v}, "
        out+=')'
        return out


    array = property(get_array, set_array, del_array, "This is an array property")
