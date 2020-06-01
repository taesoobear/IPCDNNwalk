
import pdb # use pdb.set_trace() for debugging
#import code # or use code.interact(local=dict(globals(), **locals())) for debugging. see below.
import numpy as np
import libmainlib as m 
import importlib
def out(*args):
    l=m.getPythonWin()
    if not l.isLuaReady() : return
    # lua function call
    l.getglobal("print")
    n=len(args)
    for i in range(n):
        l.push(args[i])
    l.call(n,0)

def dostring(arg):
    l=m.getPythonWin()
    if not l.isLuaReady() : return
    l.dostring(arg)


def getglobal_mat(*args):
    varname=args[0]
    l=m.getPythonWin()
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
# e.g. lua.F('print', [1,2,3])
#      lua.F('drawCNN_row', dataconv*1, m.vector3(200,0,0), "kinect")
#      lua.F(('dbg','draw'), 'Sphere',m.vector3(-100,20,0), 'ball1', 'red', 5)
#      a=lua.F1_vec('getCNN_row', dataconv*1, m.vector3(200,0,0), "kinect")
#      b=lua.F1_mat('getCNN_matrix', dataconv*1, m.vector3(200,0,0), "kinect")
# supported types: ndarray (up to 2D), tuple, m.vector3, m.quater, m.*...

# On the other hand, lua.Fref assumes all the arguments are read-writable (reference).
# e.g. 
# a=np.array([[1,2,3],[4,5,6]])
# lua.Fref('modifyMat', a)
# print(a)

def F(*args):
    _F(0, *args)

#assumes that the lua function returns a matrix
def F1_mat(*args):
    _F(1, *args)
    l=m.getPythonWin()
    mat=l.popmatrixn()
    return mat.ref().tolist()

#assumes that the lua function returns a vector
def F1_vec(*args):
    #l.printStack()
    _F(1, *args)
    #l.printStack()
    l=m.getPythonWin()
    if not l.isnil(l.gettop()):
        vec=l.popvectorn()
        return vec.ref().tolist()
    return None

# converts numpy array to vectorn, matrixn or hypermatrixn.
def array(ndarr):
    if len(ndarr.shape)==1:
        temp=m.vectorn()
        temp.setSize(ndarr.shape[0])
        temp.ref()[:]=ndarr
        return temp
    elif len(ndarr.shape)==2:
        temp=m.matrixn(ndarr.shape[0], ndarr.shape[1])
        temp.ref()[:,:]=ndarr
        return temp
    elif len(ndarr.shape)==3:
        temp=m.hypermatrixn(ndarr.shape[0], ndarr.shape[1], ndarr.shape[2])
        for j in range(temp.pages()):
            temp.page(j).ref()[:,:]=ndarr[j]
        return temp
    else:
        assert(False)
    return None

# the return values will be inside the lua stack. See F1_mat and F1_vec for retrieving the return values.
def _F(numout, *args):
    funcname=args[0]
    l=m.getPythonWin()
    if not l.isLuaReady() : return
    if type(funcname).__name__=='tuple':
        l.getglobal(*funcname)
    else:
        l.getglobal(funcname)

    for i in range(1,len(args)):
        if type(args[i]).__name__=='ndarray':
            l.push(array(args[i]))
        else:
            l.push(args[i])
    l.call(len(args)-1,numout)

# assumes all the arguments are read-writable (reference).
# e.g. 
# a=np.array([[1,2,3],[4,5,6]])
# lua.Fref('modifyMat', a)
# print(a)

def Fref(*args):
    funcname=args[0]
    l=m.getPythonWin()
    if not l.isLuaReady() : return
    if type(funcname).__name__=='tuple':
        l.getglobal(*funcname)
    else:
        l.getglobal(funcname)

    out=[None] * len(args)
    for i in range(1,len(args)):
        if type(args[i]).__name__=='ndarray':
            if len(args[i].shape)==1:
                temp=m.vectorn()
                temp.assign(args[i])
                l.push(temp)
                out[i]=temp
            elif len(args[i].shape)==2:
                temp=m.matrixn(args[i].shape[0], args[i].shape[1])
                for j in range(temp.rows()):
                    temp.row(j).assign(args[i][j])
                l.push(temp)
                out[i]=temp
            else:
                assert(False)
        else:
            l.push(args[i])
    l.call(len(args)-1,0)
    for i in range(1,len(args)):
        if type(args[i]).__name__=='ndarray':
            if len(args[i].shape)==1:
                args[i][:]= out[i].tolist()
            elif len(args[i].shape)==2:
                args[i][:]= out[i].tolist()
            else:
                assert(False)
def init_console():
    # absolutely necessary. The mainWin won't be shown without m.showMainWin() though.
    uiscale=1.5
    m.createMainWin(int((1024+180)*uiscale),int((600+100)*uiscale), int(1024*uiscale), int(600*uiscale),uiscale)
    m.getPythonWin().loadEmptyScript()

# simply forward events to lua
def onCallback(mid, userdata):
    l=m.getPythonWin()

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
    l=m.getPythonWin()


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
    l=m.getPythonWin()
    if not l.isLuaReady() : return
    # lua function call
    l.getglobal("onFrameChanged")
    l.push(currFrame)
    l.call(1,0)

def frameMove(fElapsedTime):
    l=m.getPythonWin()
    if not l.isLuaReady() : return
    # lua function call
    l.getglobal("frameMove")
    l.push(fElapsedTime)
    l.call(1,0)

def dofile(fn):
    m.getPythonWin().dofile(fn)
