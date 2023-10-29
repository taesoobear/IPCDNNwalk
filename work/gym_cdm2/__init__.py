import sys
if 'gym' in sys.modules:
    from gym.envs.registration import register
    import gym_cdm2.luaEnv 

    # todo: just glob *.lua in the spec folder, and register them all.
    register( id='walk2-v1', entry_point='gym_cdm2.luaEnv:LuaEnv')
    register( id='walkterrain-v1', entry_point='gym_cdm2.luaEnv:LuaEnv')
    register( id='run2-v1', entry_point='gym_cdm2.luaEnv:LuaEnv')
    register( id='run180-v1', entry_point='gym_cdm2.luaEnv:LuaEnv')
    register( id='run180_1-v1', entry_point='gym_cdm2.luaEnv:LuaEnv')
    register( id='run180_2-v1', entry_point='gym_cdm2.luaEnv:LuaEnv')
    register( id='run180_3-v1', entry_point='gym_cdm2.luaEnv:LuaEnv')
