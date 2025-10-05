import os
import sys
import platform
import pdb # use pdb.set_trace() for debugging
import luamodule as lua  # see luamodule.py
m=lua.taesooLib()
import numpy as np

import math
import gymnasium
import settings
from gymnasium import spaces, logger
from gymnasium.utils import seeding

            
import rendermodule as RE
def createMainWin(spec_id, do_string, args=None):
    scriptFile="gym_deepmimic/configmotions.lua"
    if args:
        scriptFile=args.scriptFile

    _createMainWin(spec_id, scriptFile, do_string)
def _createMainWin(spec_id, scriptFile, do_string):
    if len(sys.argv)==2:
         scriptFile=sys.argv[1]
    uiscale=1
    #uiscale=1.5 doesn't work on mac
    m.createMainWin(int((1024+180)*uiscale),int((600+100)*uiscale), int(1024*uiscale), int(600*uiscale),uiscale)
    m.showMainWin()
    l=m.getPythonWin()
    l.loadEmptyScript()
    if os.path.exists(scriptFile):
        l.dofile(scriptFile)
        l.dostring(do_string+' ctor();param={}')
        l.getglobal('prepareEnv')
        l.push(spec_id)
        l.call(1,0)




class LuaEnv(gymnasium.Env):    
    def _render(self):
        m.usleep(int(1e6/30))
        RE.renderOneFrame(True)

    def __init__(self, args, render_mode=None):
        if args.testMode==False:
            self._init_taesooLib_Ogre(args)
        self.renderMode=render_mode
        self._init_taesooLib()
        self._init_sim() # taesoo adjusted the order
        self.args=args
        lua.dostring('assert(not envInitilized)')
        lua.setGlobal('envInitilized', True)

    def _init_sim(self):
        info=lua.F('get_dim')
        state_dim=int(info.get(0))
        action_dim=int(info.get(1))

        max_state=np.ones(state_dim, dtype=np.float32)*100
        self.observation_space = spaces.Box(-max_state, max_state)
        self.action_space = spaces.Box(low =-1.0, high = 1.0, shape=(action_dim,))
        self.actionSteps = 0
        self.step_counter = 0
        self.episodeTotalReward = 0


    def _init_taesooLib(self):
        lua.F("init_env")

    def _init_taesooLib_Ogre(self, args=None):
        createMainWin(args.env_name,'', args)


    def get_observation(self):
        return np.array([])

    def step(self, paction):
        actionvec=m.vectorn()
        actionvec.setSize(len(paction))
        actionvec.ref()[:]=paction

        lua_state, lua_done, reward, lua_truncated=lua.F('step', 0, actionvec)
        thisState=lua_state.ref().astype(np.float32)
        if lua_done ==1:
            done=True
        else:
            done=False

        if lua_truncated==1:
            truncated=True
        else:
            truncated=False

        self.step_counter+=1
        self.actionSteps+=1
        self.episodeTotalReward += reward
        if done is True and not truncated:
            self.reset()

        if self.renderMode=='taesooLib':
            self._render()
        return thisState, reward, done, truncated, {}


    #def render(self, mode='human', close=False):
    def render(self):
        l=m.getPythonWin()
        if not l.isLuaReady() : return
        l.getglobal("python_render")
        l.push(renderBool)
        l.call(1,0)

    def reset(self, seed=None, options=None):
        self.step_counter=0
        self.actionSteps=0
        self.episodeTotalReward = 0
        out=lua.F("reset",0)
        if seed!=None:
            lua.F("math.randomseed", seed)

        resetInfo={}
        return out.ref().astype(np.float32), resetInfo
