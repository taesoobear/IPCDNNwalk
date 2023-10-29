
require("config")

require("common")
require("subRoutines/RagdollSim")
require("subRoutines/Timeline")
require("subRoutines/WRLloader")
--[[ parameters ]]--

g_env={
	env_name=spec_id or 'run3-v5',
}
spec_id=g_env.env_name
g_env.scriptFile=script_file or "gym_cdm2/spec/"..spec_id..".lua"

require('gym_cdm2/torchModule')

dofile(g_env.scriptFile)

ctor_original=ctor -- defined in configmotions

function ctor()
	isMainloopInLua =true
	ctor_original()
	prepareEnv(spec_id) -- loads deepmimic.lua

	if not param then
		param={}
	end
	
	param.globalTime=0
	

	-- override onFrameChanged defined in deepmimic.lua
	EVR.onFrameChanged=function(self, win, iframe)
		param.globalTime=param.globalTime+1/30
		-- actual time advances 1/60 (to capture at 60hz)
		envStep()
		if param then
			local target=param.exitAt 
			if target and target<param.globalTime then this('exit!',0) end
		end
		return 0
	end

	init_env()

	local filename="trained_models/ppo/"..spec_id..".pt"
	if not random_walk then
		g_agent=TorchAgent(filename, spec_id)
	end
    print('reset started')
	g_env.obs=reset(0) -- to compare with pytorch's obs, use g_agent.vec_norm:process(obs)
	for i=1, 15 do
		envStep()
	end

    print('reset finished')

	if param.capture then
		RE.renderer():screenshot(true)
		RE.motionPanel():motionWin():playFrom(0)
	elseif param.collectRefTraj then
		RE.motionPanel():motionWin():playFrom(0)
	end
	
    --만약 루프를 frameMove가 아닌 여기서 돌려면 아래 uncomment
    --while True:
    --    envStep() 
    --    if render_func is not None:
    --        render_func()
    --pdb.set_trace()
	--require('gym_cdm2/deepmimic_full')
end
function envStep()
	local action
	if random_walk then
		action=CT.zeros(get_dim()(1))
	else
		action =g_agent:getAction(g_env.obs)
	end
	local _obs, done, reward=step(0, action)  
	g_env.obs=_obs

	if done==1 then
		g_env.obs=reset(0)
	end
end
