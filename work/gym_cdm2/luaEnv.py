import os
import sys
import platform
import pdb # use pdb.set_trace() for debugging
import settings
if hasattr(settings,'useConsole') and settings.useConsole: 
    import console.libmainlib as m 
else:
    import libmainlib as m
import luamodule as lua  # see luamodule.py
import numpy as np

import math
import gym
import settings
from gym import spaces, logger
from gym.utils import seeding

            
def createMainWin(spec_id, do_string):
    scriptFile="gym_cdm2/spec/"+spec_id+".lua"
    _createMainWin(spec_id, scriptFile, do_string, None)

def _createMainWin(spec_id, scriptFile, do_string, args):
    if len(sys.argv)==2:
         scriptFile=sys.argv[1]
    uiscale=1
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




class LuaEnv(gym.Env):
    def lua_getdim(self):
        l=m.getPythonWin()
        if not l.isLuaReady() : return
        l.getglobal("get_dim")
        l.call(0,1)
        return l.popvectorn()
    def init_sim(self):
        info=self.lua_getdim()
        state_dim=int(info.get(0))
        action_dim=int(info.get(1))

        max_state=np.ones(state_dim)*100
        self.observation_space = spaces.Box(max_state, -max_state)
        self.action_space = spaces.Box(low =-1.0, high = 1.0, shape=(action_dim,))
        self.actionSteps = 0
        self.step_counter = 0
        self.episodeTotalReward = 0


    def init_taesooLib(self):
        l=m.getPythonWin()
        l.getglobal("init_env")
        l.call(0,0)

    def init_taesooLib_Ogre(self):
        createMainWin(self.spec.id,'')

    def lua_reset(self):
        l=m.getPythonWin()
        l.getglobal("reset")
        l.push(0) # unused
        l.call(1,1)
        return l.popvectorn()

    def lua_step(self,frameNum,paction):
        l=m.getPythonWin()
        if not l.isLuaReady() : return
        l.getglobal("step")
        actionvec=m.vectorn()
        actionvec.setSize(len(paction))
        actionvec.ref()[:]=paction
        l.push(frameNum)
        l.push(actionvec)
        l.call(2,3)
        return l.popnumber(), l.popnumber(), l.popvectorn()

    def vectorTostate(self):
        return np.array(self.initial_pose.ref())
    def vectorTonumpy(self,luaVec):
        return np.array(luaVec.ref())
    def get_observation(self):
        return np.array([])

    def step(self, action):
        reward, lua_done, lua_state =self.lua_step(self.step_counter,action)
        thisState=self.vectorTonumpy(lua_state)
        if lua_done ==1:
            done=True
        else:
            done=False

        self.step_counter+=1
        self.actionSteps+=1
        self.episodeTotalReward += reward
        if done is True:
            self.reset()
        return thisState, reward, done, {}


    #def render(self, mode='human', close=False):
    def render(self):
        l=m.getPythonWin()
        if not l.isLuaReady() : return
        l.getglobal("python_render")
        l.push(renderBool)
        l.call(1,0)

    def reset(self):
        self.step_counter=0
        self.actionSteps=0
        self.episodeTotalReward = 0
        self.initial_pose=self.lua_reset()
        return self.vectorTostate()
