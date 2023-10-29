
import pdb # use pdb.set_trace() for debugging
#import code # or use code.interact(local=dict(globals(), **locals())) for debugging. see below.
import numpy as np
import settings

if hasattr(settings,'useConsole') and settings.useConsole: 
    import console.libmainlib as mlib
else:
    import libmainlib as mlib
import importlib
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


# call lua from python
# lua.F and lua.F1_* assumes that all the arguments are constant (read-only).
# if you want to use the call-by-reference convention instead, use Fref*
# e.g. lua.F('print', [1,2,3])
#      lua.F('drawCNN_row', dataconv*1, mlib.vector3(200,0,0), "kinect")
#      lua.F(('dbg','draw'), 'Sphere',mlib.vector3(-100,20,0), 'ball1', 'red', 5)
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
    _m(0, varname, funcname, *args)

def M1_dbl(varname, funcname, *args):
    _m(1, varname, funcname, *args)
    return _popnumber();

def _popnumber():
    l=mlib.getPythonWin()
    if not l.isnil(l.gettop()):
        v=l.popnumber()
        return v
    return None
def _popboolean():
    l=mlib.getPythonWin()
    if not l.isnil(l.gettop()):
        v=l.popboolean()
        return v
    return None
def M1_int(varname, funcname, *args):
    return int(M1_dbl( varname, funcname, *args))

# G (getGlobal) functions returns a reference (==native taesooLib class).
def G_int(varname):
    _getGlobal(varname)
    return int(_popnumber())
def G_bool(varname):
    _getGlobalNoCheck(varname)
    return _popboolean()
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


def _getGlobal(varname):
    l=mlib.getPythonWin()
    if not l.isLuaReady() : return
    if isinstance(varname,tuple):
        l.getglobal(*varname)
    else:
        l.getglobal(varname)
def _getGlobalNoCheck(varname):
    l=mlib.getPythonWin()
    if not l.isLuaReady() : return
    if isinstance(varname,tuple):
        l.getglobalNoCheck(*varname)
    else:
        l.getglobalNoCheck(varname)

def _m(numout, varname, funcname, *args):
    _getGlobal(varname)
    l=mlib.getPythonWin()
    l.getMemberFunc(funcname)
    tempvar=[]
    for i in range(len(args)):
        if isinstance(args[i],np.ndarray):
            t=array(args[i])
            l.push(t)
            tempvar.append(t) # to prevent garbage collection
        else:
            l.push(args[i])
    l.call(len(args)+1,numout)


def F(*args):
    _F(0, *args)

#assumes that the lua function returns a matrix
def F1_mat(*args):
    _F(1, *args)
    return _popMat()
def F1_npmat(*args):
    _F(1, *args)
    return _popNumpyMat()

def _popMat():
    l=mlib.getPythonWin()
    mat=l.checkmatrixn()
    out=mat.ref().tolist() # copy (because "mat" is a temporary variable that cannot go beyond its lua stack frame)
    l.pop()
    return out
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
        if isinstance(args[i],np.ndarray):
            t=array(args[i])
            l.push(t)
            tempvar.append(t) # to prevent garbage collection
        else:
            l.push(args[i])
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
    # absolutely necessary. The mainWin won't be shown without m.showMainWin() though.
    uiscale=1.5
    mlib.createMainWin(int((1024+180)*uiscale),int((600+100)*uiscale), int(1024*uiscale), int(600*uiscale),uiscale)
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
    l.getglobal("handleRendererEvent")
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
    l.getglobal("onFrameChanged")
    l.push(currFrame)
    l.call(1,0)

def frameMove(fElapsedTime):
    l=mlib.getPythonWin()
    if not l.isLuaReady() : return
    # lua function call
    l.getglobal("frameMove")
    l.push(fElapsedTime)
    l.call(1,0)

def dofile(fn):
    mlib.getPythonWin().dofile(fn)
