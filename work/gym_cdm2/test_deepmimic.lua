
require("config")

package.projectPath='../Samples/QP_controller/'
package.path=package.path..";../Samples/QP_controller/lua/?.lua" --;"..package.path
package.path=package.path..";../Samples/classification/lua/?.lua" --;"..package.path
package.path=package.path..";../Resource/classification/lua/?.lua" --;"..package.path

require("common")
require("subRoutines/Timeline")
require("subRoutines/WRLloader")
require("RigidBodyWin/subRoutines/PDservo_spd")
require("tl")
require("gym_deepmimic/module/RetargetConfigPreset")
require("gym_cdm2/deepmimic_full")
--[[ parameters ]]--

g_env_full={
	env_name=spec_id_full or 'walk-v1',
}

require('gym_cdm/torchModule')

function generateRandomNumbers()
	pseudoRandom=vectorn(1000)
	math.randomseed(0)
	for i=0, 1000-1 do
		pseudoRandom:set(i, math.random())
	end
end

function ctor()
	isMainloopInLua =true
	RagdollSimFull._prepareEnv(g_env_full.env_name) -- loads deepmimic.lua

	ctor2()
	if not param then
		param={}
	end

	param.globalTime=0


	mEventReceiver=EVR()
	-- override onFrameChanged defined in deepmimic.lua
	EVR.onFrameChanged=function(self, win, iframe)
		param.globalTime=param.globalTime+1/30
		envStep()
		if param then
			local target=param.exitAt 
			if target and target<param.globalTime then this('exit!',0) end
		end
		return 0
	end


	local filename="trained_models/ppo/"..g_env_full.env_name..".pt"
	if not random_walk then
		g_agent_full=TorchAgent(filename, g_env_full.env_name)
	end
	print('reset started')
	g_env_full.obs=reset(0) -- to compare with pytorch's obs, use g_agent.vec_norm:process(obs)
	print('reset finished')

	if param.capture then
		RE.renderer():screenshot(true)
		RE.motionPanel():motionWin():playFrom(0)
	end
	--만약 루프를 frameMove가 아닌 여기서 돌려면 아래 uncomment
	--while True:
	--    envStep() 
	--    if render_func is not None:
	--        render_func()
	--pdb.set_trace()
	--
end
function ctor2()
	RE.viewpoint():setFOVy(40)
	RE.viewpoint().vpos:assign(vector3(370, 110, 160))
	RE.viewpoint().vat:assign(vector3(0,110,0))
	RE.viewpoint():update()
	generateRandomNumbers()
	if isMainloopInLua then
		hasGUI=true
	end
	--mEventReceiver=EVR()

	if hasGUI and isMainloopInLua then
		require('subRoutines/Timeline')
		mTimeline=Timeline("Timeline", 10000)
	end

	this:create("Button", "viewPoint", "viewPoint")
	this:create("Check_Button", "Simulation", "Simulation", 0, 2,0)
	this:widget(0):checkButtonValue(0) -- 1 for imediate start
	this:widget(0):buttonShortcut("FL_CTRL+s")
	this:create("Button", "capture", "capture")
	this:create("Button", "viewpoint", "viewpoint")
	this:create("Button", "start randomRestart", "start randomRestart",0,3,0)
	this:create("Button", "start randomInitial", "start randomInitial")
	this:create("Button", "start zero action", "start zero action")
	this:create("Button", "show reference motion", "show reference motion")
	this:create("Button", "show original motion", "show original motion")

	for k, v in pairs(float_options) do
		this:create("Value_Slider"	, k, k,1)
		this:widget(0):sliderRange(v.min, v.max)
		this:widget(0):sliderValue(v.val)
	end

	this:create("Button", "push", "push",1);

	this:updateLayout()

end
function envStep()
	local action
	if random_walk then
		action=CT.zeros(get_dim()(1))
	else
		action =g_agent_full:getAction(g_env_full.obs)
	end
	local _obs, done, reward=step(0, action)  
	g_env_full.obs=_obs

	if done==1 then
		g_env_full.obs=reset(0)
	end
end


function dtor()
end
if EventReceiver then
	EVR=LUAclass(EventReceiver)
	function EVR:__init(graph)
		self.cameraInfo={}
	end
end

function onCallback(w, userData)
	if w:id()=='viewPoint' then
		print('view pos',RE.viewpoint().vpos)
		print('view at',RE.viewpoint().vat)
	elseif w:id()=='push' then
		local simulationFrameRate=1/model.timestep

		simulator.impulse=float_options.impulseDuration.val*simulationFrameRate
		simulator.impulseDir=vector3(1,0,0)*float_options.impulseMagnitude.val
		simulator.impulseGizmo=simulator.objectList:registerEntity("arrow2", "arrow2.mesh")
		simulator.impulseGizmo:setScale(2,2,2)
		RE.output("impulse", tostring(simulator.impulse))
	elseif w:id()=="show reference motion" then
		if not simulator then
			init_env()
		end

		simulator.skin_ref:applyMotionDOF(simulator.motionDOF_euler)
		RE.motionPanel():motionWin():addSkin(simulator.skin_ref)
		simulator.skin_ref:setVisible(true)
	elseif w:id()=="show original motion" then

		if not simulator then
			init_env()
		end

		simulator.skin_ref:applyMotionDOF(mMotionDOF_euler)

		RE.motionPanel():motionWin():addSkin(simulator.skin_ref)
		simulator.skin_ref:setVisible(true)


	elseif w:id()=='start randomRestart' then
		generateRandomNumbers()
		randomRestart=true
		randomRestartInfo.freq=1
	elseif w:id()=="start randomInitial" then
		randomInitial=true
	elseif w:id()=='capture' then
		RE.renderer():screenshot(true)
	elseif w:id()=='viewpoint' then
		print(RE.viewpoint().vpos)
		print(RE.viewpoint().vat)
	elseif w:id()=="start zero action" then
		random_action_environment=true
		random_action_max=0
		hasGUI=true
		g_stepCount=0
		init_env()

		if true then
			-- print debug information
			___reset_old=reset
			reset=function()
				local return_initial_state=___reset_old()
				-- if you want to test the step function outside of render loop.
				print ('reset:', return_initial_state)
				local dims=get_dim()
				local adim=dims(1)
				local action=CT.zeros(adim)
				out=step(model.start, 	action)

				g_stepCount=g_stepCount+1
				print('step1:', out)
				return_initial_state=out
				return return_initial_state
			end
		end
		reset()
	else
		for k, v in pairs(float_options) do
			if w:id()==k then
				float_options[k].val=w:sliderValue()
				break
			end
		end
	end
end


function get_dim()
	--local nball=mLoader.dofInfo:numSphericalJoint()
	local ndof=mLoader.dofInfo:numDOF()
	local n_actual_dof=mLoader.dofInfo:numActualDOF()
	-- numDOF of hyunwoo_low_dof_T : 35
	-- numDOF of hyunwoo_low_dof_T (spherical) : 43 == 3 + 4 (knees/elbows) + 9*4 
	-- numDOF of hanyang_low_dof_T (spherical) : 77 == 3 + 2 (knees) + 18*4
	--if useHanyangInsteadOfHyunwoo then
	--	return CT.vec(77+12+6+3,2+(18-1)*3)

	return CT.vec(ndof+12+6+3,n_actual_dof-6)
end


function frameMove(fElapsedTime)
	if isMainloopInLua then
		-- moved to onFrameChanged
	elseif random_action_environment then
		-- python에서 초출 되는 순서로 호출함

		local dims=get_dim()
		local adim=dims(1)
		local action=(CT.rand(adim)-CT.ones(adim)*0.5)*(random_action_max*2)

		local step_state, episode_done, step_reward=step(g_stepCount, action)


		g_stepCount=g_stepCount+1
		if episode_done==1 then
			g_stepCount=0
			reset()
		end
	end

end


function EVR:onFrameChanged(win, iframe)
	if pyModuleName then
		python.F(pyModuleName, 'envStep')
	else
		assert(false)
	end
end

function step(_python_iframe_unused, action)
	local iframe=simulator.RLenv.iframe
	--assert(randomRestart or (_python_iframe_unused==iframe))

	if action:isnan() then
		print('action nan??? .. using random action')
		for i=0, action:size()-1 do
			action:set(i, math.random()-0.5)
		end
	end
	local targetAction=vectorn()
	targetAction=actionScaling(action)

	local res, step_reward, step_state, done_
	res, step_reward, step_state, done_= pcall(RagdollSimFull.frameMove, simulator, iframe, targetAction)
	if type(step_reward)=='string' then
		print('error!', step_reward) -- error!
		if select(1, string.find(step_reward, 'projectAngle')) then
			return CT.zeros(get_dim()(0)),1 , 0
		else
			assert(false)
		end
	end

	local model=model_full
	local mocapstep_per_controlstep=model.frame_rate*model.RL_step
	simulator.RLenv.iframe=iframe+mocapstep_per_controlstep

	if isMainloopInLua and model.loopMotion then
		local maxHistory=model.frame_rate/2
		if iframe+mocapstep_per_controlstep-maxHistory>simulator.motionDOF_original:numFrames()-1 then
			-- transition
			simulator.RLenv.iframe=simulator.RLenv.iframe-
			(simulator.motionDOF_original:numFrames()-1)
		end
	end
	--
	-- ctrl+alt+o to see outputs
	RE.output2('RL step', simulator.RLenv.iframe,done_, step_reward, step_state, action)

	if done_ then		
		return step_state,1 , step_reward
	else
		return step_state,0 , step_reward
	end
end

function python_render(renderBool)
	RE.renderOneFrame(true)
end

function reset()
	local res, return_initial_state= pcall(simulator.reset, simulator)
	if not res then
		print(return_initial_state)
		dbg.console()
	end

	return return_initial_state 
end

function render()
	RE.renderOneFrame(true)
	RE.renderer():screenshot(true)
end

function state_reScaling(step_state)
	local rescale_state=vectorn(step_state:size())
	for i=0, step_state:size()-1 do	 
		rescale_state:range(i,i+1):assign((step_state:range(i,i+1)/5))
	end
	return rescale_state
end

function actionScaling(actions)
	return actions*0.2
end

function getDofIndex(skel)
	local DoFIndexData={}
	local totalDoFNum=0	
	print("Treeindex","BoneName","NumDof","startDoFIndex")
	for j=1, skel:numBone()-1  do
		DoFIndexData.BoneName = skel:getBoneByTreeIndex(j)

		DoFIndexData.NumDoF = skel.dofInfo:numDOF(j)

		DoFIndexData.startDoFIndex = skel.dofInfo:DOFindex(j,0)
		print(j,"   ",DoFIndexData.BoneName,"		",DoFIndexData.NumDoF,"        ",DoFIndexData.startDoFIndex)	
		totalDoFNum=totalDoFNum+DoFIndexData.NumDoF
	end

	print("\n total joint DoF Number is ", totalDoFNum)
	--return dofIndex
end

---------------------------------after this line, simulation class--------------------------------








function handleRendererEvent(ev, button, x, y)
	return 0
end





