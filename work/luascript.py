import pdb
import numpy as np
relative=False
try:
    import settings
except:
    import work.settings as settings
    settings.relativeMode=True

settings._loadMlib()
mlib=settings.mlib
lua=settings.lua
ArgumentProcessor=lua.ArgumentProcessor
def require(self, modulename):
    self.getglobal('require')
    self.push(modulename)
    self.call(1, 0)

def setGlobal(self, varname, value):
    blockingCall(self, 'python.setGlobal', varname, value)

def _getGlobal(l, varname):
    if '.' in varname:
        varname=varname.split('.')
        l.getglobal(*varname)
    elif isinstance(varname,tuple):
        l.getglobal(*varname)
    else:
        l.getglobal(varname)

def _push(l, arg, tempvar):
    if isinstance(arg,np.ndarray):
        t=lua.array(arg)
        tempvar.insert(0, t) # to prevent immediate garbage collection
        l.push(t)
    elif isinstance(arg, ArgumentProcessor):
        arg.push(l)
    elif isinstance(arg, dict):
        l.newtable()  # push a lua table
        for k, v in arg.items():
            l.push(k)
            _push(l,v, tempvar)
            l.settable(-3)
    elif isinstance(arg, list) or isinstance(arg, tuple):
        l.newtable()  # push a lua table
        n=len(arg)
        for j in range(n):
            l.push(j+1) # convert to 1-indexing
            _push(l,arg[j], tempvar)
            l.settable(-3)
    elif arg!=None:
        try:
            l.push(arg)
        except:
            print('push: unknown type', arg)
    else:
        l.pushnil()
def _getResults(l):
    numOut=l.gettop()-l.getPreviousTop()+1
    if numOut>0:
        out=[]
        for i in range(numOut):
            tid=l.luaType(-1)
            if tid==7: # userdata -> copy
                tn=l.lunaType(-1)
                out.insert(0, l.popUserdata(tn).copy())
            elif tid==0: #nil
                # simply ignore
                pass
            elif tid==1: #bool
                out.insert(0, l.popboolean())
            elif tid==4: # string
                out.insert(0, l.popstring())
            elif tid==3: #number
                out.insert(0, l.popnumber())
            elif tid==5: # table -> make a global backup in lua, and return a reference
                res=lua._popAuto(l, False) # no stack cleanup for backup
                out.insert(0, res) # reference
                var_name='out_'+mlib.generateUniqueName()
                # backup to lua global to prevent garbage collection
                if isinstance(res, dict):
                    res.var_name=var_name 
                l.set(var_name)
            else:
                print('other cases not implemented yet')
                l.printStack()
                pdb.set_trace()
        if len(out)==1:
            return out[0]
        return tuple(out)

def getResults(l):
    l.waitUntilFinished()
    return _getResults(l)

def blockingCall(l,*args):
    funcname=args[0]
    _getGlobal(l,funcname)
    l.saveCurrentTop()
    tempvar=[]
    for i in range(1,len(args)):
        l.pushAuto( args[i], tempvar)
    l.call(len(args)-1)
    return l._getResults()

def nonblockingCall(l,*args):
    funcname=args[0]
    _getGlobal(l,funcname)
    tempvar=[]
    for i in range(1,len(args)):
        l.pushAuto( args[i], tempvar)
    l.threadedCall(len(args)-1)


def batchSetGlobal(self, varname, value):
    for i in range(self.numThreads()):
        self.env(i).setGlobal(varname, value)
def waitUntilAllJobsFinish(pool):
    while pool.busy(): 
        mlib.usleep(100) 

mlib.LuaScript.require=require
mlib.LuaScript.setGlobal=setGlobal
mlib.LuaScript.pushAuto=_push
mlib.LuaScript._getResults=_getResults
mlib.LuaScript.blockingCall=blockingCall
mlib.LuaScript.popUserdata=lua._popUserdata
mlib.ThreadedScript.nonblockingCall=nonblockingCall
mlib.ThreadedScript.getResults=getResults
mlib.ThreadScriptPool.waitUntilAllJobsFinish=waitUntilAllJobsFinish
mlib.ThreadScriptPool.setGlobal=batchSetGlobal


