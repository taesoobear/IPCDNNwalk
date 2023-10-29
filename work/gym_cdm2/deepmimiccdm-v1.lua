-- v6에 기반해서, 필터된 위치에 디디도록 수정해보자.
require("config")
require("common")

package.projectPath='../Samples/QP_controller/'
package.path=package.path..";../Samples/QP_controller/lua/?.lua" --;"..package.path
package.path=package.path..";../Samples/classification/lua/?.lua" --;"..package.path
package.path=package.path..";../Resource/classification/lua/?.lua" --;"..package.path
require("tl")
require("subRoutines/WRLloader")
require("subRoutines/VelocityFields")
require("gym_cdm2/module/collisionAvoid")
require("gym_cdm2/module/RetargetConfigPreset")
require("gym_cdm2/module/QPservo_v3")
fc=require("gym_cdm2/module/CDMTraj")
require('gym_cdm2/RagdollSim')
require('Kinematics/meshTools')
require("gym_cdm2/ConvexHull2D")
timestep_qp=1/120

-- shared parameters for all motions
debug_mode=false
debug_draw=false
draw_option={
	contactpos=true,
	swingpos=true,
	contactforce=true,
	--draw_offset=vector3(0,0,0)
}
RL_step=1/60
CHANGE_TERM=120

ctrl_cost_weight=4.25*0.006*5e-3 
startFromStanding=false
numLegs=2
--costs_weight=0.01 -- works but unnatural
costs_weight=0.03 -- 
--costs_weight=0.1 -- works but does not move
healthy_reward=5.0
terminate_when_unhealthy=true
healthy_y_range={0.8, 2.0} -- actually y (height)
fall_angle=math.rad(20)
--fall_angle=math.rad(90)
reset_noise_scale=5e-2
reset_noise_scale_velxz=5e-2
exclude_current_positions_from_observation=true
attachCamera = true
alignCameraForwardDir=false

CDMdrawOffset=vector3(-1,0,0)

if not util.PerfTimer2.stopMsg then
	function util.PerfTimer2:stopMsg(str)
		 RE.output(str, tostring(self:stop2()))
	end
end
timer1=util.PerfTimer2()--(1,"timer3")
timer2=util.PerfTimer2()--(1,"timer3")
timer3=util.PerfTimer2()--(1,"timer3")



float_options={}
float_options.menu=
{
	'impulseMagnitude',
	'impulseDuration',
	'impulseDir',
	'actionModFZ',
	'actionModX',
	'actionModY',
	'actionModZ',
	'actionModAX',
	'actionModAY',
	'actionModAZ',
	'actionModTiming',
}
float_options.impulseMagnitude={ val=200, min=10, max=1500}
float_options.impulseDuration={ val=0.2, min=0, max=1}
float_options.impulseDir={ val=0, min=math.rad(-90), max=math.rad(90)}
float_options.actionModX={ val=0, min=-3, max=3}
float_options.actionModY={ val=0, min=-0.3, max=0.2}
float_options.actionModZ={ val=0, min=-3, max=3}
float_options.actionModAX={ val=0, min=-3, max=3}
float_options.actionModAY={ val=0, min=-0.5, max=0.5}
float_options.actionModAZ={ val=0, min=-3, max=3}
float_options.actionModFZ={ val=0, min=-0.14, max=0.14}
float_options.actionModTiming={ val=0, min=-3, max=3}

drawMode=true
nanchecker=false

hasGUI=false

-- RL environment functions:
-- reset, init_env, step


function ctor()
	this:create("Box", "box0", "do not change the steering\n direction below too abruptly")
		this:create("Value_Slider"	, "target angle", "target angle",1)
		this:widget(0):sliderRange(-math.pi,math.pi)
		this:widget(0):sliderValue(0)
RE.viewpoint():setFOVy(40)
RE.viewpoint().vpos:assign(vector3(370, 110, 160))
RE.viewpoint().vat:assign(vector3(0,110,0))
RE.viewpoint():update()
	if isMainloopInLua and RE.ogreSceneManager() then
		hasGUI=true
	end
	if not hasGUI then
		-- disable debug console
		function fineLog(...)
			local fn
			if rank==nil then
				fn="optimize_finelog.txt"
			else
				fn="optimize_finelog"..rank..".txt"
			end
			print("error occurred! see optimize_finelog.txt")

			--   util.outputToFile(fn, os.date()..util.mergeString({...}))
			dbg.outputToFile(fn, os.date()..util.mergeString({...}))
		end
		fineLog=nil
	end
	mEventReceiver=EVR()

	if hasGUI and isMainloopInLua then
		require('subRoutines/Timeline')
		mTimeline=Timeline("Timeline", 10000)
	end

	this:create("Button", "viewPoint", "viewPoint")
	this:create("Button", "rotate light", "rotate light")
	this:create("Check_Button", "Simulation", "Simulation", 0, 2,0)
	this:widget(0):checkButtonValue(0) -- 1 for imediate start
	this:widget(0):buttonShortcut("FL_CTRL+s")
	this:create("Button", "capture", "capture")
	this:create("Button", "viewpoint", "viewpoint")
	this:create("Button", "start randomRestart", "start randomRestart",0,3,0)
	this:create("Button", "start randomInitial", "start randomInitial")
	this:create("Button", "start zero action", "start zero action")
	this:create("Button", "show reference motion", "show reference motion")
	this:create("Button", "show only fullbody", "1. show only fullbody")
	this:create("Button", "show only SRB", "2. show only SRB")
	this:create("Button", "show both", "3. show both")

	for i, k in ipairs(float_options.menu) do
		local v=float_options[k]
		this:create("Value_Slider"	, k, k,1)
		this:widget(0):sliderRange(v.min, v.max)
		this:widget(0):sliderValue(v.val)
	end

	this:create("Button", "push", "push",1);
	this:create('Button', 'push head', 'push head')
	this:create('Button', 'push arm', 'push arm')
	this:create('Button', 'push leg', 'push leg')

	this:updateLayout()
	collisionTestOnlyAnkle=false
	debugContactParam={10, 0, 0.01, 0, 0}-- size, tx, ty, tz, tfront
	
end

function dtor()
	if mTimeline then
		mTimeline:dtor()
		mTimeline=nil
	end
end

if EventReceiver then
	EVR=LUAclass(EventReceiver)
	function EVR:__init(graph)
		self.cameraInfo={}
	end
end

function upv() 
	return vector3(0,1,0)
end

function onCallback(w, userData)
	if w:id()=='target angle' then
		simulator.change_count=0
		simulator.target_Y_UI=(quater(w:sliderValue(),upv()))
	elseif w:id()=='viewPoint' then
		print('view pos',RE.viewpoint().vpos)
		print('view at',RE.viewpoint().vat)
	elseif w:id()=='show only SRB' then
		simulator.CDM.skin:setVisible(true)
		simulator.skin_fullbody:setVisible(false)
		CDMdrawOffset.x=0
		simulator.CDM.skin:setTranslation(CDMdrawOffset*100)
		draw_option.contactpos=true
		draw_option.contactforce=true
	elseif w:id()=='show only fullbody' then
		simulator.CDM.skin:setVisible(false)
		simulator.skin_fullbody:setVisible(true)
		draw_option.contactpos=false
		draw_option.contactforce=false
	elseif w:id()=='show both' then
		simulator.CDM.skin:setVisible(true)
		simulator.skin_fullbody:setVisible(true)
		--CDMdrawOffset.x=-1
		CDMdrawOffset.x=0
		simulator.CDM.skin:setTranslation(CDMdrawOffset*100)
		draw_option.contactpos=true
		draw_option.contactforce=true
  elseif w:id()=='rotate light' then
	  local osm=RE.ogreSceneManager()
	  if osm:hasSceneNode("LightNode") then
		  local lightnode=osm:getSceneNode("LightNode")
		  lightnode:rotate(quater(math.rad(30), vector3(0,1,0)))
	  end
	elseif w:id()=='push' then
		local simulationFrameRate=1/timestep_qp
		simulator.impulse=float_options.impulseDuration.val*simulationFrameRate
		simulator.impulseDir=(quater(float_options.impulseDir.val,vector3(0,1,0))*vector3(-1,0,0))*float_options.impulseMagnitude.val
		simulator.impulse_lpos=vector3(0,0,0)
		simulator.gizmo={'Hips', vector3(0, 0.2, 0), false}
		RE.output("impulse", tostring(simulator.impulse))
	elseif w:id()=='push head' then
		local simulationFrameRate=1/timestep_qp
		simulator.impulse=0.2*simulationFrameRate -- duration
		simulator.impulseDir=(quater(float_options.impulseDir.val,vector3(0,1,0))*vector3(-1,0,0))*float_options.impulseMagnitude.val
		simulator.impulse_lpos=vector3(0,0.65,0.03)
		simulator.gizmo={'Neck', vector3(0, 0.09, 0), false}
	elseif w:id()=='push arm' then
		local simulationFrameRate=1/timestep_qp
		simulator.impulse=0.2*simulationFrameRate -- duration
		simulator.impulseDir=(quater(float_options.impulseDir.val,vector3(0,1,0))*vector3(-1,0,0))*float_options.impulseMagnitude.val
		simulator.impulse_lpos=vector3(-0.18,0.18,0.13)
		simulator.gizmo={'RightElbow', vector3(-0.27, 0, 0), false}
	elseif w:id()=='push leg' then
		local simulationFrameRate=1/timestep_qp
		simulator.impulse=0.2*simulationFrameRate -- duration
		simulator.impulseDir=(quater(float_options.impulseDir.val,vector3(0,1,0))*vector3(-1,0,0))*float_options.impulseMagnitude.val
		simulator.impulse_lpos=vector3(-0.18,-0.22,0.13)
		simulator.gizmo={'RightHip', vector3(-0.08, -0.25, 0), false}


	elseif w:id()=="show reference motion" then
		if not simulator then
			init_env()
		end
		if EVR and EVR.onFrameChanged then
			-- override onFrameChanged defined in test_cdm_deepmimic.lua
			EVR.onFrameChanged=function(self, win, iframe)
				return 0
			end
		end

		local info=model.motionData
		simulator.skin_ref:applyMotionDOF(info.motionDOF_fullbody)
		RE.motionPanel():motionWin():addSkin(simulator.skin_ref)
		RE.motionPanel():scrollPanel():addPanel(info.touchDown[1], CPixelRGB8(255,255,255))
		RE.motionPanel():scrollPanel():setLabel('R touchDown')
		RE.motionPanel():scrollPanel():addPanel(info.touchOff[1], CPixelRGB8(255,255,255))
		RE.motionPanel():scrollPanel():setLabel('R touchOff')

		RE.motionPanel():scrollPanel():addPanel(info.touchDown[2], CPixelRGB8(255,255,255))
		RE.motionPanel():scrollPanel():addPanel(info.touchOff[2], CPixelRGB8(255,255,255))

		local mi=model._motionInfo
		local conL=fc.buildContactConstraints(info.motionDOF_original:numFrames(), mi.contact.touchDown, mi.contact.touchOff)
		RE.motionPanel():scrollPanel():addPanel(conL[1], CPixelRGB8(255,255,255))
		RE.motionPanel():scrollPanel():addPanel(conL[2], CPixelRGB8(255,255,255))
	elseif w:id()=='start randomRestart' then
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

function prepareEnv(spec_id)
	if model.showHuman then
		require("gym_cdm2/module/showHuman") 
	end
end

function init_env()
	print('lua initenv')
	debugContactParam={10, 0, 0.01, 0, 0}-- size, tx, ty, tz, tfront
	--[[ implementations ]]--

	niter=math.floor(RL_step/timestep_qp+0.5)

	mLoader= MainLib.WRLloader(
	{
		name='CDM',
		body={
			name='box',
			jointType="free",
			translation =vector3(0.0,0.0,0.0 ), -- joint의 위치
			geometry ={ 
				{
					'Box',
					translation=vector3(0,0,0),
					size=VRMLexporter.boxSizeFromInertia(60, model.inertia.x, model.inertia.y, model.inertia.z),
					mass=60,
				},
			},
		}
	}
	)
	print(mLoader:VRMLbone(1):mass())

	--mFloorLoader=MainLib.VRMLloader("../Resource/mesh/floor_y.wrl")

	if RE.ogreSceneManager() then
		-- adjust fog depending on the box size
		--	RE.ogreSceneManager():setFog( 0.8,0.8,0.8, 0.0,200, 1800 )
		RE.ogreSceneManager():setFog( 0.77,0.92,1, 0.0,1200, 2400 )
	end
	--to check collision between floor and character
	--mFloorSkin=RE.createVRMLskin(mFloorLoaderForDrawing, false)
	--mFloorSkin:scale(100,100,100)
	--mFloorSkin:setMaterial('lightgrey')
    --mFloorSkin:setVisible(true)

	--math.randomseed(os.time())
	local simulatorParam={
		timestep=timestep_qp,
		floor=mFloorLoader,
	}
	if model.useTerrain then
		--local s=48 -- meter
		local s=48
		local h=model.terrainHeight or 1.5
		local min_h=model.terrainMinHeight or 0.1
		local mesh=OBJloader.Terrain("../Resource/crowdEditingScripts/terrain/heightmap1_256_256_2bytes.raw", 256, 256, s*100, s*100, h*100,1,1, true)
		if false then
			mesh=Geometry()
			local numSegX=1
			local numSegZ=1
			--mesh:initPlane(numSegX, numSegZ, 20, 20) -- sizeX, sizeY
			mesh:initPlane(800, 800) -- sizeX, sizeY
		end
		g_terrain=mesh
		g_terrainPos=vector3(-s/2,min_h,-s/2)
		if hasGUI then
			local _, node=g_terrain:drawMesh('CrowdEdit/Terrain1', "terrain_node")
			local _, node2=g_terrain:drawMesh('CrowdEdit/Terrain1', "terrain_node2")
			local _, node3=g_terrain:drawMesh('CrowdEdit/Terrain1', "terrain_node3")
			node:translate(g_terrainPos*100)
			local delta=vector3(0,0,s*100)
			node2:translate(g_terrainPos*100+delta)
			node3:translate(g_terrainPos*100+2*delta)
			bgnode=RE.ogreSceneManager():getSceneNode("BackgroundNode")
			bgnode:flipVisibility()
		end
		g_terrainSize=s



		--dbg.draw('Sphere', g_terrain:getTerrainPos(vector3(0,0,0))*100,'orisin', 'blue', 20)
		--dbg.draw('Sphere', g_terrain:getTerrainPos(vector3(0,0,g_terrainSize))*100,'origin2', 'red', 40)
	end



	simulator=RagdollSim(mLoader, false, simulatorParam)

	if placeObstacle then
		ObstacleSim=createObstacleSim(self)
	end
	if attachCamera then
		mCOMfilter=OnlineLTIFilter(CT.vec(vector3(0,0,0)), 10)
	end
	simulator:reset() -- 이거 해주면 첫 한번이 이상한 문제 사라짐.. -.-
end

function OBJloader.Terrain:getTerrainPos(xzpos)

	local terrain=g_terrain
	assert(terrain.height )

	local xz_local=(xzpos-g_terrainPos)*100
	local h=terrain:height(vector2(xz_local.x, xz_local.z))

	return vector3(xzpos.x, h/100+g_terrainPos.y, xzpos.z)
end

-- state: 
-- curr foot pos, curr foot vel (locally represented)
-- action (per limb):
-- (ax,az)
-- action (global):
-- dv,dw
-- cos(delta_y), sin(delta_y)

function get_dim()
	return CT.vec(11+6*numLegs+2+2,2*numLegs+6)
end
function RagdollSim:getOBSdim()
	return 11+6*numLegs+2+2
end

function RagdollSim:startSwing(gf,rotY, li)
	li.isSwing=true
	li.swingPhase=0
	--li.prevContact=li.contactPos:copy()
	li.prevContact=li.contactFilter:copy()

	self:filterContact(li, rotY, 'ssw')

	local ileg=1
	if li.isLeft then
		ileg=2
	end
	local simulator=self:getCDMsimulator()
	local dq=simulator:getDQ(0) -- [ global w, global v ]
	local gv=dq:toVector3(3)
	local rt=model:remainingTimeToTouchDown(ileg)
	local footOffset=model:defaultFeetOffset(ileg)
	--RE.output2('footAction'..ileg, action(si), action(si+1), footOffset, rotY:rotationAngleAboutAxis(vector3(0,1,0)))
	local desiredPos=gf.translation+rotY*footOffset
	desiredPos:radd(rt/model.frame_rate*gv)
	self:projectToGround(desiredPos)

	if model.useTerrain and hasGUI then
		local tpos=vector3N(60)
		for i=0, tpos:size()-1 do
			local w=sop.map(i,0, tpos:size()-1, 1,0)
			local opos=li.prevContact*w + desiredPos*(1-w)
			self:projectToGround(opos)
			tpos(i):assign(opos)
		end

		local res=math.projectTrajectoryToItsHull(tpos)

		if debug_draw then
			dbg.draw('Sphere', li.prevContact*100, 'sp1'..ileg)
			dbg.draw('Sphere', desiredPos*100, 'sp2'..ileg)
			if res then
				if ileg==1 then
					dbg.namedDraw('Traj',tpos:matView()*100, 'projtpos', 'solidblue', 12, 'BillboardLineList')
				else
					dbg.namedDraw('Traj',tpos:matView()*100, 'projtpos2', 'solidred', 12, 'BillboardLineList')
				end
			end
		end
		li.hull={tpos, rt}

	end
end
function RagdollSim:startSpprt(gf,rotY, legInfo)
	legInfo.isSwing=false
	legInfo.actualTouchDown=false
	legInfo.spprtPhase=0
	legInfo.rotY=rotY:copy()

	self:filterContact(legInfo, rotY, 'ssp')

end
function RagdollSim:initContactFilter(li)
	local B=0.001
	local k=1
	local logQ=model.cf_option.logQ or 7 -- 8 works. let's try 7.
	local logQrot=logQ
	li.logQ=logQ
	require('control/SDRE')
	local ball=MassParticle3D(30, B, k, math.pow(10, logQ), 0, RL_step/2)
	ball:setState(li.contactPos)
	li.lqrFilter=ball
	li.contactPreFilter=li.contactPos:copy()
	li.contactFilter=li.contactPos:copy()
	li.dotContactFilter=vector3(0,0,0)

	-- delta between initial and filtered footRotY 
	li.lqrFilterQ=SDS(30, B, k, math.pow(10, logQrot), 0, RL_step/2)
	li.lqrFilterQ:setState(0, 0)
	li.lqrFilterQorigin=li.rotY:copy()
	li.contactFilterQ=quater(1,0,0,0)
end

function RagdollSim:filterContact(li, rotY, mode, gv)
	local B=0.001
	local k=1
	local logQ=li.logQ
	local logQrot=li.logQ
	local filterEndPhase=0.4

	if mode=='ssw' or mode=='ssp' then

		if mode=='ssw' then
			li.contactPreFilter=li.prevContact
			li.lqrFilter:setState(li.prevContact)

			li.lqrFilterQ:setState(0, 0)
			li.contactFilterQ:assign(quater(1,0,0,0))
		else
			local ball=li.lqrFilter
			li.contactFilter=ball:getPosition()
			li.dotContactFilter=vector3(0,0,0)

			li.lqrFilterQorigin=li.rotY:copy()
			li.contactFilterQ:assign(quater(1,0,0,0))
		end
	elseif mode=='sw' then
		local maxFootVel=model.maxFootVel 
		local ball=li.lqrFilter
		local w=li.swingPhase
		if not li.isSwing then
			w=filterEndPhase
		end
		local vel=li.contactPos-li.contactPreFilter --global vel
		local alpha2=sop.mapSin(w, 0, filterEndPhase, 0, 1)
		vel=math.smoothClampVec3(vel, maxFootVel*alpha2)
		li.contactPreFilter=li.contactPreFilter+vel


		--if g_terrain then
		--	li.contactPreFilter.y=math.max(li.contactPreFilter.y, g_terrain:getTerrainPos(li.contactPreFilter).y)
		--end

		local Q=math.pow(10, logQ)
		local k=1
		ball:updateCoef(B, k,Q)

		local v=gv:copy()
		v.y=0
		ball:setDesiredState(li.contactPreFilter, v)
		ball:singleStep()
		ball:singleStep()
		li.contactFilter=ball:getPosition()
		li.dotContactFilter=ball:getVelocity()

		local filterQ=li.lqrFilterQ
		Q=math.pow(10, logQrot)

		local delta=quater()
		delta:difference( li.lqrFilterQorigin,rotY)
		delta:align(quater(1,0,0,0))
		delta=delta:rotationAngleAboutAxis(vector3(0,1,0))
		filterQ:updateCoef(B, k, Q)

		filterQ:setDesiredState(delta,0)
		filterQ:singleStep()
		filterQ:singleStep()

		li.contactFilterQ:setRotation( vector3(0,1,0),filterQ.x(0,0))
		if li.isLeft then
			RE.output2('angle', li.lqrFilterQorigin:rotationAngleAboutAxis(vector3(0,1,0)), delta, filterQ.x(0,0))
		end

		if debug_draw then
			dbg.draw('Axes', transf(li.contactFilterQ*li.lqrFilterQorigin, ball:getPosition()), 'filterq'..tostring(li.isLeft), 100)
			RE.output2("cfQ"..tostring(li.isLeft), li.swingPhase, delta, filterQ.x(0,0))
			RE.output2("swing"..tostring(li.isLeft), li.swingPhase,filterEndPhase, logQ)
			dbg.draw('Sphere', (li.contactPreFilter)*100, 'swingFootPrefilter'..tostring(li.isLeft),'blue')
			dbg.draw('Sphere', (li.contactFilter)*100, 'swingFootFiltered'..tostring(li.isLeft),'green')
			dbg.draw('Sphere', (li.contactPos)*100, 'swingFootUnfiltered'..tostring(li.isLeft),'red')
		end

	end
end


function RagdollSim:calcContactPos( ileg, gf, gv, rotY , action) 
	local info=model.motionData
	local li=self.legInfo[ileg]
	local contactPosL=li.contactPos
	local hipPos=gf*li.footOffset
	local legLen= contactPosL:distance(hipPos)
	RE.output2('leglen'.. ileg, legLen)
	local maxLegLen=model.maxLegLen 

	local penalty=0


	local isSwing=li.isSwing
	local minSwingPhase=model.minSwingPhase or 0.4
	local function updateSwingFoot(contactPosL,li)
		local localContactPos=gf:inverse()*contactPosL
		local slack=-0.01 --math.min(gf.translation.y-1.0, 0)
		if localContactPos:length()>maxLegLen + slack then
			localContactPos:normalize()
			localContactPos:rmult(maxLegLen+slack)
		end
		--if localContactPos.x>0.5 then localContactPos.x=0.5 end
		--if localContactPos.x<-0.5 then localContactPos.x=-0.5 end
		li.localContactPos=localContactPos
		li.contactPos:assign(gf*localContactPos)

		self:filterContact(li, rotY, 'sw', gv)
	end

	if not isSwing then
		--assert(ileg==1)
		-- detect transition to a swing phase
		self.legInfo[ileg].spprtPhase=self.legInfo[ileg].spprtPhase+RL_step/0.73 -- max spprt dur=0.73
		--if self.legInfo[ileg].actualTouchDown and (legLen>maxLegLen or self.legInfo[ileg].spprtPhase>1) then
		if model:remainingTimeToTouchOff(ileg)<=0 then
		--	--penalty=penalty+600*600
			self:startSwing(gf, rotY, self.legInfo[ileg])
			updateSwingFoot(contactPosL,li)
		else
			self:filterContact(li, nil, 'sp')
		end
	end

	if isSwing then
		local acc=vector3(0,0,0)

		local pgv=gv:copy()
		pgv.y=0
		do
			local si=0
			-- global desired pos
			--
			local footOffset=model:defaultFeetOffset(ileg)
			--RE.output2('footAction'..ileg, action(si), action(si+1), footOffset, rotY:rotationAngleAboutAxis(vector3(0,1,0)))
			local desiredPos=gf.translation+rotY*(vector3(action(si), 0, action(si+1)+float_options.actionModFZ.val)+footOffset)
		
			self:projectToGround(desiredPos,li)
			--local dt=action(8)*1+0.1
			--if dt>0.2 then
			--	local e=dt-0.2
			--	dt=0.2
			--	penalty=penalty+e*e*600
			--end
			li.swingPhase=li.swingPhase+RL_step
			--local t=li.swingPhase -- 0 to 2

			--RE.output2('swingP'..ileg, li.swingPhase)
			--desiredPos.y=0.02*(1-(t-1)*(t-1))

			--local w=t/2
			--local fixedGamma=1.26
			--w=math.smoothTransition(math.pow(w, fixedGamma))
			--li.contactPos:interpolate(w, li.prevContact, desiredPos)
			--li.contactPos.y=desiredPos.y
			li.contactPos:assign(desiredPos)
		end
		updateSwingFoot(li.contactPos, li)

		local info=model.motionData
		local rt=model:remainingTimeToTouchDown(ileg)

			
		local COM_height=gf.translation.y

		if model.useTerrain then
			-- terrain height relative to landing foot position.
			--local COM_height=gf.translation.y- g_terrain:getTerrainPos(gf.translation).y
			local swingphase=model:swingPhase(ileg)
			local th1=g_terrain:getTerrainPos(gf.translation).y
			local th2= g_terrain:getTerrainPos(li.contactPos).y
			COM_height=COM_height- sop.mapCos(swingphase, 0, 1, th1, th2)

			--dbg.delayedDraw('Sphere', model.delayedVis, vector3(gf.translation.x, th1+COM_height, gf.translation.z)*100, 'desiredCOM', 'red', 30)
			--dbg.delayedDraw('Sphere', model.delayedVis, vector3(gf.translation.x, th1, gf.translation.z)*100, 'desiredCOMorig', 'blue', 30)
		else
			--dbg.delayedDraw('Sphere', model.delayedVis, gf.translation*100, 'desiredCOMorig', 'blue', 30)
		end
		if rt<=0  then
			--print('td', info.loader_ref:bone(1):getFrame().translation.y)
			--if li.contactPos.y<0.03 then
			if model.unlimitedLeglen or COM_height<info.loader_ref:bone(1):getFrame().translation.y+(model.contactThr or 0.05) then

				-- detect transition to a contact phase
				self:projectToGround(li.contactPos)

				self:startSpprt(gf, rotY, self.legInfo[ileg])
			else
				li.lateTouchDown=true
			end
		elseif COM_height<model.touchDownHeight and li.swingPhase>minSwingPhase then
			li.earlyTouchDown=rt
		else
			local iotherLeg=math.fmod(ileg,2)+1
			local oli=self.legInfo[iotherLeg]
			local st=model.strideThr

			if st then
				--if ileg==1 then
				--	dbg.draw("Sphere", (oli.contactFilter+rotY*vector3(0,0,st))*100, 'oli'..ileg)
				--	dbg.draw("Sphere", (li.contactFilter)*100, 'li'..ileg,'red')
				--end
				local function proj(v) return vector3(v.x, 0, v.z) end
				local currentStride=proj(li.contactFilter):distance((proj(oli.contactFilter)+rotY*vector3(0,0,st)))
				if not oli.isSwing and currentStride>st and li.swingPhase>minSwingPhase then
					li.earlyTouchDown=rt
				end
			end
		end

		if numContactPoints==1 then
			return {contactPosL}, penalty
		else
			return {
				self:_calcContactPos(ileg, rotY, 1),
				self:_calcContactPos(ileg, rotY, -1),
			},  penalty
		end
	end
	rotY=self.legInfo[ileg].rotY

	if numContactPoints==1 then
		return {contactPosL}, penalty
	else
		local pos1=self:_calcContactPos(ileg, rotY, 1)
		local pos2=self:_calcContactPos(ileg, rotY, -1)

		return {
			pos1,
			pos2,
		}, 
		penalty
	end
end
function RagdollSim:advanceFootStep(iframe, ileg, rotY, gf)
	local li=self.legInfo[ileg]

	local isSwing=li.isSwing


	if not isSwing then
		--assert(ileg==1)
		-- detect transition to a swing phase
		self.legInfo[ileg].spprtPhase=self.legInfo[ileg].spprtPhase+RL_step/0.73 -- max spprt dur=0.73
		if model:remainingTimeToTouchOff(ileg,iframe)<=0 then
			self:startSwing(gf, rotY, self.legInfo[ileg])
		end
	end

	if isSwing then
		if model:remaininingTimeToTouchDown(ileg, iframe)<=0 then
			-- detect transition to a contact phase
			self:projectToGround(li.contactPos)

			self:startSpprt(gf, rotY, self.legInfo[ileg])
			--if not self.legInfo[2].isSwing then
			--	startSwing(self.legInfo[2])
			--end
		end
	end
end
function RagdollSim:enforceTouchDown(ileg, rotY)
	local li=self.legInfo[ileg]

	assert(li.isSwing)
	self:startSpprt(nil, rotY, li)
	--[[
	-- detect transition to a contact phase
	self:projectToGround(li.contactPos)

	self.legInfo[ileg].isSwing=false
	self.legInfo[ileg].spprtPhase=0
	self.legInfo[ileg].rotY=rotY:copy()
	self.legInfo[ileg].actualTouchDown=false
	]]
end

function RagdollSim:_calcContactPos(ileg, rotY, isToe)
	local li=self.legInfo[ileg]
	local contactPosL=li.contactPos
	return contactPosL+rotY*vector3(0,0, isToe*0.12)
end

function RagdollSim:_calcFilteredContactPos(ileg, rotY, isToe)
	local pos, ori=self:getFilteredFootPos(transf(rotY), ileg)
	return pos+ori*vector3(0,0, isToe*0.12)
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


function handleRendererEvent()
	return 0
end

function EVR:onFrameChanged(win, iframe)
	if not rendering_step then
		rendering_step=RL_step
	end
	local niter=math.round(rendering_step/RL_step)
	niter=1  -- for making high-quality video.
	for i=1, niter do
		if pyModuleName then
			python.F(pyModuleName, 'envStep')
		else
			python.F('gym_cdm2.test_gym', 'envStep')
		end
	end
end

function step(_python_iframe_unused, action)
	timer1:stopMsg('outside step')
	timer2:start()
	--assert(randomRestart or (_python_iframe_unused==iframe))

	if action:isnan() then
		print('action nan??? .. using random action')
		for i=0, action:size()-1 do
			action:set(i, math.random()-0.5)
		end
	end
	local res, step_reward, step_state, done_
    -- pcall corresponds to try-catch in c++
	res, step_reward, step_state, done_= pcall(RagdollSim.step, simulator, action)
	--	 위 문장은 아래 simulator:step과 같은 뜻이지만 에러 발생시 다르게 동작. 
	if debug_mode then
		print(step_state, step_reward)
	end

	-- without pcall, error inside this function will be catched in python (but using dbg.console())
	-- choose whichever way you prefer.
	if not res then
		print('error!', step_reward) -- error!
		if debug_mode then
			dbg.console()
		end
		if not step_reward or select(1, string.find(step_reward, 'projectAngle')) then
			return CT.zeros(get_dim()(0)),1 , 0
		else
			assert(false)
		end
	end
	
	--
	-- ctrl+alt+o to see outputs
	--RE.output2('RL step', simulator.RLenv.iframe,done_, step_reward, step_state, action)

	timer2:stopMsg('step2')
	timer1:start()
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
	--print(reset)
	local res, return_initial_state= pcall(simulator.reset, simulator)
	if not res then
		print('error!!', return_initial_state)
	end

	if debug_mode then
		print('reset', return_initial_state)
		dbg.console()
	end

	if ObstacleSim then
		ObstacleSim:reset()
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

function defaultActionScaling(actions)
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




function RagdollSim:getJointStateFromSimulator(rotY)
	assert(rotY)
	local simulator=self:getCDMsimulator()
	local numBone=self.loader:numBone()
	local sim_state=simulator:getWorldState(0)
	local jointpos=vector3N(8)
	local sg=sim_state:globalFrame(1)
	jointpos(0):assign(sg.translation)
	jointpos(1):assign(self:_calcFilteredContactPos(1, rotY, 1)) -- R toe
	jointpos(2):assign(self:_calcFilteredContactPos(1, rotY, -1))
	jointpos(3):assign(self:_calcFilteredContactPos(2, rotY, 1)) -- L toe
	jointpos(4):assign(self:_calcFilteredContactPos(2, rotY, -1))
	jointpos(5):assign(sg*vector3(0,0.63,0))
	jointpos(6):assign(sg*vector3(0.5,0,0))
	jointpos(7):assign(sg*vector3(-0.5,0,0))
	for i=1,4 do
		self:projectToGround(jointpos(i))
	end

	return { pos=jointpos, } --ori=jointori}
end
function RagdollSim:getJointStateFromRefTree()

	local toeLocalPos=vector3(0.000000,-0.010000,0.140000)
	local heelLocalPos=vector3(0.0000,-0.010000,-0.140000) 
	local additionalmarkers={
		{MotionLoader.RIGHTANKLE, toeLocalPos}, {MotionLoader.RIGHTANKLE, heelLocalPos},
		{MotionLoader.LEFTANKLE, toeLocalPos}, {MotionLoader.LEFTANKLE, heelLocalPos},

		{MotionLoader.NECK, vector3(0,0,0)},
		{MotionLoader.HIPS, vector3(0.5,0,0)}, 
		{MotionLoader.HIPS, vector3(-0.5,0,0)}, 
	}
	local numBone=1
	local jointpos=vector3N(numBone+#additionalmarkers)
	--local jointori=quaterN(numBone)
	local info=model.motionData
	local ref_tree=info.loader_ref
	local sg=ref_tree:bone(1):getFrame()
	jointpos(0):assign(sg.translation)
	local ref_tree_fullbody=info.loader_ref_fullbody

	for i,marker in ipairs(additionalmarkers) do
		local lpos=marker[2]
		jointpos(i):assign(ref_tree_fullbody:getBoneByVoca(marker[1]):getFrame()*lpos)
	end
	for i=1,4 do
		self:projectToGround(jointpos(i))
	end

	return { pos=jointpos, } --ori=jointori}
end

function RagdollSim:step(_action)

	local action=vectorn()
	action=model.actionScaling(_action, self)

	if debug_mode then
		print('frame', iframe, action)
	end
	--print('step')
	if not model.delayedVis then
		model.delayedVis=0
	end

	RE.output2('iframe', model.iframe)
	--local timer=util.PerfTimer2()
	--timer:start()


	local refpose_fullbody=model:createRefpose()
	if refpose_fullbody==nil then
		-- episode done normally.
		if model.collectRefTraj then
			local refTraj=g_refTraj
			util.saveTable(refTraj, 'refTraj_done.dat')
			print('refTraj_done.dat exported... Manually copy this file to refTraj_~.dat')
		end
		local step_state=self:_get_obs()
		return 0, step_state, true
	end
	model:setRefTree()
	if hasGUI then
		self.skin_ref:setPoseDOF(refpose_fullbody)
	end
	

	--os.sleep(1)
	--timer:stopMsg('setPoseDOF') -- type ctrl+alt+o to see the computation time
	
	local episode_done=false


	local simulator=self:getCDMsimulator()
	local prev_com=simulator:calculateCOM(0)

	local ctrl_force=0
	local swing_penalty=0

	local gf=simulator:getWorldState(0):globalFrame(1)
	local rotY=model:getRefRotY(gf.rotation)
	--dbg.namedDraw('Axes',transf(rotY, gf.translation),'rotY',100)
	local dq=simulator:getDQ(0) -- [ global w, global v ]

	if self.target_Y_UI then
		--self.target_Y:assign(self.target_Y_UI)
		-- velocity-dependent clamping
		self.target_Y=getDesiredFinalVel(rotY, self.target_Y_UI, dq:toVector3(3):dotProduct(rotY*vector3(0,0,1)))
	end

	if debug_mode then
		print('gf', gf, dq)
	end
	--dbg.delayedErase('Sphere', model.delayedVis, 'desiredCOMorig')
	local contactPos={}
	for i=1, numLegs do
		local limbAction=action:slice((i-1)*2, i*2)
		local _,  _penalty=self:calcContactPos( i, gf, dq:toVector3(3), rotY,limbAction )

		local li=self.legInfo[i]
		--
		--local contactPosL=li.contactFilter
		--local footrotY=li.rotY
		local contactPosL, footrotY=self:getFilteredFootPos(gf, i)
		_contactPosL=
		{
			contactPosL+footrotY*vector3(0,0,0.12),
			contactPosL+footrotY*vector3(0,0,-0.12)
		}
		contactPos[i]=_contactPosL
		swing_penalty=swing_penalty+_penalty
	end

	if model.collectRefTraj then
		local refTraj=g_refTraj
		if not refTraj then
			g_refTraj={
				globalTraj=matrixn(),
				refFrames=vectorn(),
				rawFootInfo=matrixn(),
			}
			refTraj=g_refTraj
			RE.turnOffSoftShadows() -- for faster rendering
		end

		--local prevFrame=math.max(iframe-1,0) -- compare with half-second before so that long-accumulated errors do not affect later reward
		local sf=model.iframe
		local globalTraj=vectorn(7+3+3+6+2)
		local footInfo=vectorn(2+3*2+2)
		local info=model.motionData
		assert(sf<info.motionDOF_iframe:size())
		refTraj.refFrames:pushBack(info.motionDOF_iframe(sf))
		globalTraj:setVec3(0, gf.translation)
		globalTraj:setQuater(3, gf.rotation)

		local rpos,rori=self:getFilteredFootPos(gf, 1)
		local lpos,lori=self:getFilteredFootPos(gf, 2)
		globalTraj:setVec3(7, lpos)
		globalTraj:setVec3(10, rpos)
		globalTraj:slice(13,13+6):assign(dq)
		globalTraj:set(13+6, rori:rotationAngleAboutAxis(vector3(0,1,0)))
		globalTraj:set(13+6+1, lori:rotationAngleAboutAxis(vector3(0,1,0)))

		for ilimb=1,2 do
			local legInfo=self.legInfo[ilimb]
			if legInfo.isSwing then
				footInfo:set(ilimb-1, 0)
			else
				footInfo:set(ilimb-1, 1)
			end
			footInfo:setVec3(3*(ilimb-1)+2, legInfo.contactPos)
			footInfo:set(8+ilimb-1, legInfo.swingPhase or 0)
		end

		refTraj.globalTraj:pushBack(globalTraj)
		refTraj.rawFootInfo:pushBack(footInfo)

		if math.fmod(refTraj.refFrames:size(), 100)==99 then
			util.saveTable(refTraj, 'refTraj.dat')
			if param and param.exitAt then
				print('refTraj.dat updated.',refTraj.refFrames:size(), '/', param.exitAt*30)
			else
				print('refTraj.dat updated.',refTraj.refFrames:size() )
			end
		end
	end


	if self.legInfo[1].lateTouchDown or self.legInfo[2].lateTouchDown then
		if debug_mode then print('lateTouchDown') end

		self.legInfo[1].lateTouchDown=nil
		self.legInfo[2].lateTouchDown=nil
		self.legInfo[1].earlyTouchDown=nil
		self.legInfo[2].earlyTouchDown=nil

		local mocapstep_per_controlstep=model.frame_rate*RL_step
		-- do not advance reference frame
		model:advanceTime(-mocapstep_per_controlstep)
	else
		local etd1=self.legInfo[1].earlyTouchDown
		local etd2=self.legInfo[2].earlyTouchDown
		if etd1 and etd2 then
			if etd1<etd2 then
				etd2=nil
			else
				etd1=nil
			end
		end

		local etd
		local mocapstep_per_controlstep=model.frame_rate*RL_step


		if etd1 then
			if debug_mode then print('etd1') end
			if etd1> mocapstep_per_controlstep then
				etd1=mocapstep_per_controlstep
			else
				self:enforceTouchDown(1, rotY)
			end
			etd=etd1
		elseif etd2 then
			if debug_mode then print('etd2') end
			if etd2>mocapstep_per_controlstep then
				etd2=mocapstep_per_controlstep
			else
				self:enforceTouchDown(2, rotY)
			end
			etd=etd2
		end
		self.legInfo[1].earlyTouchDown=nil
		self.legInfo[2].earlyTouchDown=nil

		if etd then
			model:advanceTime(etd)
			local simulator=self
		end
	end

	local cdmAction=action:slice(numLegs*2, numLegs*2+6)

	self:sampleTargetPose()
	if model.useTerrain then
		-- sample targetpose and modify theta_d so that it is aligned with theta
		local qpservo=self.CDM.qpservo

		local terrain_y=g_terrain:getTerrainPos(qpservo.theta_d:toVector3()).y
		qpservo.theta_d:set(1, qpservo.theta_d(1)+terrain_y)

		local dq=simulator:getDQ(0) -- [ global w, global v ]
		local gv=dq:toVector3(3)

		--local root_q=qpservo.theta_d:toQuater(3)
		local delta_t=0.5
		local currHeight=g_terrain:getTerrainPos(gf.translation).y
		local nextHeight=g_terrain:getTerrainPos(gf.translation+gv*delta_t).y
		local dy=(nextHeight-currHeight)/delta_t

		local root_q=gf.rotation
		local v_d=root_q*qpservo.dtheta_d:toVector3(0)
		if dy<0 then
			dy=math.max(dy, -5)
			v_d.y=v_d.y+dy
			qpservo.dtheta_d:setVec3(0, root_q:inverse()*v_d)
		end

		self.CDM.refCoord.translation.y=terrain_y


		if debug_mode then
			print('T:', T, '\nT_ref', T_ref)
			print('theta_d', qpservo.theta_d)
		end

		--dbg.namedDraw('Axes',MotionDOF.rootTransformation(qpservo.theta_d),'desiredRoot',100)
	end

	if true then

		local contactpos=vector3N()
		for i=1, numLegs do
			local rotY=self.legInfo[i].rotY
			for j=1, numContactPoints do
				local maxContactLen=model.maxContactLen or 2.0
				if self.legInfo[i].isSwing==false then
					local cpos
					cpos=contactPos[i][j] -- +rotY*vector3(0.05,0,0)
					self:projectToGround(cpos)
					if cpos:distance(gf.translation)<maxContactLen then
						contactpos:pushBack(cpos)
					end
					cpos=contactPos[i][j] -- -rotY*vector3(0.05,0,0)
					self:projectToGround(cpos)
					if cpos:distance(gf.translation)<maxContactLen then
						contactpos:pushBack(cpos)
					end
				end
			end
		end

		cdmAction:set(0, cdmAction(0)+float_options.actionModX.val)
		cdmAction:set(1, cdmAction(1)+float_options.actionModY.val)
		cdmAction:set(2, cdmAction(2)+float_options.actionModZ.val)
		cdmAction:set(3, cdmAction(3)+float_options.actionModAX.val)
		cdmAction:set(4, cdmAction(4)+float_options.actionModAY.val)
		cdmAction:set(5, cdmAction(5)+float_options.actionModAZ.val)

		--self.CDM:frameMove_simple(niter, contactpos, -1, nil, cdmAction, self)
		self.CDM:frameMove(niter, contactpos, -1, nil, cdmAction, self, ObstacleSim)


	end

	for i=1, numLegs do
		for j=1, numContactPoints do
			local contactPosL=contactPos[i][j]

			if hasGUI then
				local color='red'
				if self.legInfo[i].isLeft then color='blue' end

				local size=5
				if self.legInfo[i].isSwing then size=2 end

				if draw_option.contactpos then

					local delay=model.delayedVis
					if self.phaseFilter then
						delay=delay+self.phaseFilter.delay
					end
					if size~=2 or draw_option.swingpos then
						--dbg.delayedDraw('Sphere',model.delayedVis, (contactPosL+rotY*draw_option.draw_offset+CDMdrawOffset)*100, 'contactp'..i..j, color, size)
						dbg.delayedDraw('Sphere',delay, (contactPosL+CDMdrawOffset)*100, 'contactp'..i..j, color, size)
					else
						dbg.delayedErase('Sphere',delay, 'contactp'..i..j)
					end
				end
			end
		end
	end

	--for i=1, numLegs do
	--	local li=self.legInfo[i]
	--	if li.isSwing then
	--		li.swingPhase=li.swingPhase+1
	--	end
	--end
	--RE.output2('cf', action)


	-- 타이밍 diagram. (기준 시간, 즉 delta_frame==0)
	--
	-- after reset |      step |        step |         step |
	--               (after cdm frameMove)    
	--   iframe :              0             0.5            1
	--
	--
	--  before calcRefDif
	--   sim :     0           0             0.5            1
	--  in calcRefDif
	--   sim :     0          0.5            1.0           1.5


	local mimic_cost, poseDif, com_lvdif, ref_comvel=self:calcRefDif2(simulator)

	local avgVel=1.8
	local timingMod=(model.timingModCoef or 0.4)*(math.max(com_lvdif,0.7)/math.max(ref_comvel ,0.7))--action(numLegs*2+6)

	timingMod=timingMod+float_options.actionModTiming.val

	--RE.output2("timingmod", 1+timingMod, ref_comvel)
	local after_com=simulator:calculateCOM(0)
	local com_vel=(after_com-prev_com)/RL_step

        --electricity_cost = self.electricity_cost * float(np.abs(a*self.robot.joint_speeds).mean())  # let's assume we have DC motor with controller, and reverse current braking
        --electricity_cost += self.stall_torque_cost * float(np.square(a).mean())
	local l=swing_penalty
	RE.output2('c s', ctrl_force, l, com_lvdif)
	local ctrl_cost=ctrl_cost_weight*(ctrl_force+ l)

	local function float(boolValue)
		if boolValue then
			return 1.0
		else
			return 0.0
		end
	end
    local function is_healthy(self)
        local min_y, max_y = unpack(healthy_y_range)
		local _fall_angle=fall_angle
		if hasGUI and not param.capture then
			if _fall_angle>0 then
				_fall_angle=math.max(math.rad(70), _fall_angle)
			end
			min_y=0.2
			max_y=2.0
		end
        local is_healthy=false
		local last_sim=simulator:getLastSimulatedPose(0)
		local curr_y=last_sim(1)
		if model.useTerrain then
			curr_y=curr_y-g_terrain:getTerrainPos(last_sim:toVector3(0)).y
		end

		local info=model.motionData
		min_y=math.min(min_y, info.loader_ref:bone(1):getFrame().translation.y*0.7)
		if min_y < curr_y and curr_y < max_y then
			is_healthy=true
		end

		if _fall_angle>0 then
			if last_sim:toQuater(3):offsetQ():rotationAngle()>_fall_angle then
				is_healthy=false
			end
		else
			-- angle-diff method 2:
			local Q=last_sim:toQuater(3)
			local Q_ref=info.loader_ref:bone(1):getFrame().rotation

			local rotY=model:getRefRotY(Q)
			local rotY_ref=model:getRefRotY(Q_ref)

			-- rotY*offset=Q

			local offset1=rotY:inverse()*Q
			local offset2=rotY_ref:inverse()*Q_ref
			if (offset1:inverse()*offset2):rotationAngle()>-_fall_angle then
				is_healthy=false
			end
		end

		local conR=not self.legInfo[1].isSwing
		local conL=not self.legInfo[2].isSwing

        return is_healthy
	end
	local _is_healthy=is_healthy(self)

    local function calc_healthy_reward(self)
        return float(
            _is_healthy
            or terminate_when_unhealthy
        ) * healthy_reward
	end

    local function done(self)
        local done 
		
		if terminate_when_unhealthy then
			done= not _is_healthy
		else
			done= false
		end
        return done
	end

	local rewards = calc_healthy_reward(self)

	RE.output2('healthyR', rewards)
	RE.output2('mimic_cost', mimic_cost*10)
	RE.output2('ctrl_cost', ctrl_cost)
	RE.output2('com', after_com)
	RE.output2('comvel', rotY:inverse()*com_vel)


	if self.target_Y_UI then
		--dbg.draw('Arrow', prev_com*100, (prev_com+self.target_Y*vector3(0,0,1))*100, 'targetY', 10)
		dbg.draw('Arrow', prev_com*100, (prev_com+self.target_Y_UI*vector3(0,0,1))*100, 'targetY', 10)
	end

	--local desired_vel_cost=(forward_velocity-1)*(forward_velocity-1)
	--local costs = ctrl_cost +mimic_cost*10




	local target_delta=rotY:inverse()*self.target_Y
	local delta_y=target_delta:rotationAngleAboutAxis(upv())
	local facing_reward= math.cos(delta_y)-1
	local facingWeight=model.facingWeight or 1.5
	-- poseDif*0.08 works pretty well but motion quality needs to improve 
	-- poseDif*2, -com_lvdif*0.5 -> do not turn well.
	--local reward=math.exp(-poseDif*2)*math.exp(-com_lvdif*2)*math.exp(facing_reward) not bad, but facing is slow.
	--local reward=math.exp(-poseDif*2)*math.exp(-com_lvdif*2)*math.exp(facing_reward*2) better 
	local reward=math.exp(-poseDif*2)*math.exp(-com_lvdif*2)*math.exp(facing_reward*facingWeight)
	if model.doNotUseFacingReward then
		reward=math.exp(-poseDif*2)*math.exp(-com_lvdif*2)
		--local costs = mimic_cost*10
		--reward = rewards - costs*costs_weight
		--if reward<0 then reward=0 end
	end

	--print(delta_y, facing_reward, mimic_cost, math.exp(-0.1*mimic_cost))

	episode_done = done(self)
	if model.maxMimicCost and costs>model.maxMimicCost then
		episode_done=true
	end

	if self.impulse<=0 then
		dbg.delayedErase('Arrow', model.delayedVis, 'impulseGizmo')
	end
	--timer:stopMsg('simulLoop')
	local step_state=self:_get_obs()

	if hasGUI then
		-- debug draw
		local theta=simulator:getLastSimulatedPose(0)
		--self.CDM.skin:setPoseDOF(theta)
		local delay=model.delayedVis
		if self.phaseFilter then
			delay=delay+self.phaseFilter.delay
		end
		dbg.delayedSetPoseDOF(self.CDM.skin, delay, theta)
		if attachCamera then
			if not g_prevRootPos then
				g_prevRootPos=vector3(0,0,0)
			end
			local COM=simulator:calculateCOM(0)
			mCOMfilter:setCurrPose(CT.vec(COM))
			local curPos= mCOMfilter:getFiltered():toVector3(0)*100

			if not model.useTerrain then
				local gridpos=math.floor(curPos.z/400)
				local gridposx=math.floor(curPos.x/400)
				local bgnode=RE.ogreSceneManager():getSceneNode("BackgroundNode")

				bgnode:setPosition(vector3(gridposx*4*100,0,gridpos*4*100))
			end

			curPos.y=0
			RE.viewpoint().vpos:assign(RE.viewpoint().vpos+curPos-g_prevRootPos)
			RE.viewpoint().vat:assign(RE.viewpoint().vat+curPos-g_prevRootPos)
			RE.viewpoint():update()     
			g_prevRootPos=curPos:copy()

			-- floor follows the character on integer grids (meters)
			--mFloorSkin:setTranslation(math.round(curPos.x/100)*100, 0, math.round(curPos.z/100)*100)
			--mFloorSkin:setTranslation(curPos.x, 0, curPos.z)

			if alignCameraForwardDir then
				local function getViewDelta(vdir)
					vdir:normalize()
					local cdir=theta:toQuater(3)*vector3(0,0,1)
					local deltaq=quater()
					deltaq:setAxisRotation(vector3(0,1,0), cdir, vdir)
					deltaq:align(quater(1,0,0,0))
					local delta=deltaq:rotationAngleAboutAxis(vector3(0,1,0))
					return delta
				end
				if not g_prevRootOriDelta then
					local vdir=RE.viewpoint().vat-RE.viewpoint().vpos
					vdir.y=0
					local len=vdir:length()
					local delta=getViewDelta(vdir)
					g_prevRootOriVdir=vdir
					g_prevRootOriDelta=delta +math.rad(alignCameraForwardDirDelta or 0)
					g_prevRootOriLen=len
					mCOMoriFilter=OnlineLTIFilter(CT.vec(delta), 100)
				end

				mCOMoriFilter:setCurrPose(CT.vec(getViewDelta(g_prevRootOriVdir)))
				local curDelta= mCOMoriFilter:getFiltered()(0)

				local newvdir=quater(-g_prevRootOriDelta-curDelta, vector3(0,1,0))*g_prevRootOriVdir
				newvdir:scale(g_prevRootOriLen)

				local vdir=RE.viewpoint().vat-RE.viewpoint().vpos
				newvdir.y=vdir.y

				RE.viewpoint().vpos:assign(RE.viewpoint().vat+newvdir)
				RE.viewpoint():update()     
			end
		end
	end
	if hasGUI then
	else
		self.change_count=self.change_count+1	
		if self.change_count>=1 then
			if self.change_count%CHANGE_TERM==0 then
				if not hasGUI then
					self:change_target_direction()
				end
			end
		end
	end

	if randomRestartInfo.limitLength and (not isMainloopInLua or not model.loopMotion) then
		-- limitlength is used only when training for a loop-motion.
		if self.globalTime*model.frame_rate> randomRestartInfo.limitLength then
			episode_done=true
		end
	end

	if step_state:isnan() then
		step_state:setAllValue(0)
		reward=0
		episode_done =true
	end

	self.lastOBS=step_state:copy()
	self.lastOBS_phase=model:getPhase()
	do
		local simulator=self
		local mocapstep_per_controlstep=model.frame_rate*RL_step* (1+timingMod)
		model:advanceTime(mocapstep_per_controlstep)
		simulator.globalTime=simulator.globalTime+RL_step
	end
	dbg.delayedDrawTick()

	return reward, step_state, episode_done
end



function RagdollSim:reset()
	self.change_count=0
	self.target_Y=quater(1,0,0,0)
	self.target_Y_UI=quater(1,0,0,0)
	timer3:start()

	--math.randomseed(0)
	self.globalTime=0

	
	local initialState, initialVel, startRswing, startLswing=model:getInitialState()

	model:setRefTree()

	-- restart from the origin for easier debugging
	local deltaRootPos=vector3(initialState(0), 0, initialState(2))
	initialState:set(0,0)
	if model.useTerrain then
		initialState:set(1, initialState(1)+g_terrain:getTerrainPos(vector3(0,0,0)).y)
	else
		initialState:set(1, initialState(1))
	end
	initialState:set(2,0)

	local q=initialState:toQuater(3)
	local r90=quater(math.rad(0), vector3(0,1,0)) -- for testing coordinate invariance. for training, set the angle to 0.
	if model.randomInitialOri then
		r90=quater(math.rad(math.random()*360), vector3(0,1,0)) 
	end

	initialState:setQuater(3, r90*q)

	if debug_mode then
		math.randomseed(0)
	end

	if randomInitial then
		local roottf=MotionDOF.rootTransformation(initialState)
		roottf.rotation:assign(quater(math.random(),vector3(0,1,0))*roottf.rotation:offsetQ())
		initialState:setQuater(3, roottf.rotation)
	end

	if hasGUI then
		reset_noise_scale=math.min(reset_noise_scale, 5e-2)
		reset_noise_scale_velxz=math.min(reset_noise_scale_velxz, 5e-2)
	end

	local noise_low=-reset_noise_scale
	local noise_high=reset_noise_scale
	initialState:slice(3,0):radd(CT.randomUniform{low=noise_low, high=noise_high, size=initialState:size()-3})

	if reset_noise_scale_pos then
		local noise_low=-reset_noise_scale_pos
		local noise_high=reset_noise_scale_pos
		initialState:slice(0,3):radd(CT.randomUniform{low=noise_low, high=noise_high, size=3})
	end

	local q=initialState:toQuater(3)
	q:normalize()
	initialState:setQuater(3, q)

	local simulator=self:getCDMsimulator()
	simulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, initialState)

	if simulator.setDQ then
		initialVel=MotionDOF.dposeToDQ(initialState:toQuater(3), initialVel)
		initialVel:radd(CT.randomUniform{low=noise_low, high=noise_high, size=initialVel:size()})
		initialVel:set(3, initialVel(3)+(math.random()-0.5)*reset_noise_scale_velxz)
		initialVel:set(4, initialVel(4)+(math.random()-0.5)*reset_noise_scale)
		initialVel:set(5, initialVel(5)+(math.random()-0.5)*reset_noise_scale_velxz)

		--print('initialVel', initialVel)
		simulator:setDQ(0, initialVel)
	else
		local initialVel=info.DMotionDOF:row(iframe):range(0,7):copy()
		initialVel:radd(CT.randomUniform{low=noise_low, high=noise_high, size=initialVel:size()})
		simulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, initialVel)
	end
	simulator:initSimulation() -- necessary

	--self.simulator:getLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, initialState)

	-- only root configuration is stored.
	self.replayBuffer=matrixn(1, initialState:size())
	self.replayBuffer:row(0):assign(initialState)

	local gf=simulator:getWorldState(0):globalFrame(1)
	local rotY=model:getRefRotY(gf.rotation)
	do

		self.legInfo={
		}

		-- calc desired contactpso
		local desired_foot_pos=self:getJointStateFromRefTree()

		for i=1, numLegs do
			local legInfo={}
			legInfo.rotY=rotY:copy()
			local pos=desired_foot_pos.pos

			legInfo.contactPos=(pos(i*2-1)*0.5+pos(i*2)*0.5)
			legInfo.contactPos.y=0
			legInfo.contactPos:rsub(deltaRootPos)
			legInfo.contactPos:rotate(r90)
			legInfo.prevContact=legInfo.contactPos:copy()

			self:initContactFilter(legInfo)
			if i==1 then
				legInfo.footOffset=vector3(-0.07,0,0) -- local
				legInfo.isLeft=false
				if startRswing then
					self:startSwing(gf, rotY, legInfo)
				else
					self:startSpprt(gf, rotY, legInfo)
				end
			else
				legInfo.footOffset=vector3(0.07,0,0)
				legInfo.isLeft=true
				if startLswing then
					self:startSwing(gf, rotY, legInfo)
				else
					self:startSpprt(gf, rotY, legInfo)
				end
			end
			self.legInfo[i]=legInfo
		end
	end
	self.sim_prev=self:getJointStateFromSimulator(rotY)


	local initialState_env=self:_get_obs()

	self.impulse=0
	self.externalForce=vector3(0,0,0)

	assert(not initialState_env:isnan())

	timer3:stopMsg('reset')
	--print('reset finished')

	self.lastOBS=initialState_env:copy()
	self.lastOBS_phase=model:getPhase()
	return initialState_env
end

function RagdollSim:getRefCoord(rootRotY, theta)
	local rootPos=theta:toVector3(0)
	self:projectToGround(rootPos)
	return transf(rootRotY, rootPos)
end

function RagdollSim:saveFullState(file)
	local simulator=self:getCDMsimulator()
	local position=simulator:getPoseDOF(0) -- [ x, y, z, qw, qx, qy, qz]
	local velocity=simulator:getDQ(0) -- [ global w, global v ]
	file:pack(position)
	file:pack(velocity)

	for i=1, numLegs do
		local li=self.legInfo[i]
		if li.isSwing then
			file:packInt(1)
		else
			file:packInt(0)
		end
		file:pack(li.contactPos)
		file:pack(li.rotY)
	end
	file:packInt(model.iframe)
	file:pack(self.replayBuffer)
end

function RagdollSim:restoreFullState(file)

	local position=vectorn()
	local velocity=vectorn()
	file:unpack(position)
	file:unpack(velocity)

	local simulator=self:getCDMsimulator()
	simulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, position)
	assert( simulator.setDQ )
	simulator:setDQ(0, velocity)

	for i=1, numLegs do
		local li=self.legInfo[i]
		if file:unpackInt()==1 then
			li.isSwing=true
		else
			li.isSwing=false
		end
		file:unpack(li.contactPos)
		file:unpack(li.rotY)
	end
	
	model.iframe=file:unpackInt()
	file:unpack(self.replayBuffer)
end

-- return a reference coordinate
function RagdollSim:_get_obs()
	local simulator=self:getCDMsimulator()
	local position=simulator:getPoseDOF(0) -- [ x, y, z, qw, qx, qy, qz]
	local rotY=model:getRefRotY(position:toQuater(3))
	local velocity=simulator:getDQ(0) -- [ global w, global v ]
	local refCoord=self:getRefCoord(rotY, position)
	--
	-- terrain relative
	position:set(1, position(1)-refCoord.translation.y)
	local offsetQ=rotY:inverse()*position:toQuater(3)
	offsetQ:align(quater(1,0,0,0))
	position:setQuater(3, offsetQ)
	if exclude_current_positions_from_observation then
		local y=position(1)
		position=position:slice(2,0):copy()
		position:set(0, y)
	end
	-- Q=rotY*offset
	--> offset=rotY:inverse()*Q
	velocity:setVec3(0, rotY:inverse()*velocity:toVector3(0))
	velocity:setVec3(3, rotY:inverse()*velocity:toVector3(3))

	--local step_state=position..velocity..com_velocity..external_contact_forces
	
	local step_state=vectorn(position:size()+velocity:size()+6*numLegs+2+2)
	step_state:slice(0, position:size()):assign(position)
	local c=position:size()+velocity:size()
	step_state:slice(position:size(), c):assign(velocity)

	for i=1, numLegs do
		local li=self.legInfo[i]
		--step_state:setVec3(c+(i-1)*4, refCoord:toLocalPos(li.contactPos))
		local cpos=li.contactFilter
		step_state:setVec3(c+(i-1)*6, refCoord:toLocalPos(cpos))

		local v3=li.dotContactFilter 
		local vv=refCoord:toLocalDir(v3)
		step_state:set(c+(i-1)*6+3,vv.x )
		step_state:set(c+(i-1)*6+4,vv.z )
		--step_state:set(c+(i-1)*4+3, li.footOffset.x)
		if li.isSwing then
			step_state:set(c+(i-1)*6+5, 0)
		else
			-- rotY*delta=footRotY
			local deltaq=(rotY:inverse()*li.lqrFilterQorigin)
			deltaq:align(quater(1,0,0,0))
			local delta=deltaq:rotationAngleAboutAxis(vector3(0,1,0))
			step_state:set(c+(i-1)*6+5, delta)
		end
	end
	c=c+6*numLegs
	if true then
		local phase=model:getPhase()
		step_state:set(c, sop.mapCos(phase, 0,0.25, 1,0)) -- phase
		step_state:set(c+1, sop.mapSin(phase, 0,0.25, 0,1)) -- phase
	end

	if true then
		local target_vec=vector3(0,0,1)
		local delta_=rotY:inverse()*self.target_Y

		if model.doNotUseFacingReward then
			delta_:identity()
		end
		target_vec:rotate(delta_)

		step_state:set(c+2, target_vec.x)
		step_state:set(c+3, target_vec.z)
	end

	assert(step_state:size()==self:getOBSdim())


	assert(c+4==step_state:size())


	if debug_mode then
		print('step_state', step_state)
	end
	return step_state
end



-- right foot pos : ilimb==1 , lfoot pos : ilimb==2 or ilimb==0
-- gf -> use only Y component!!!
function RagdollSim:getFilteredFootPos(gf, ilimb)
	if  ilimb==0 then
		ilimb=2
	end
	local legInfo=self.legInfo[ilimb]
	local pos=legInfo.contactFilter:copy()
	local ori=legInfo.lqrFilterQorigin
	--dbg.draw('Sphere', pos*100, 'filtered'..tostring(ilimb ), 'green')
	if self.legInfo[ilimb].isSwing then 
		ori=legInfo.lqrFilterQorigin*legInfo.contactFilterQ

		local olimb=ilimb%2+1
		local olimbInfo=self.legInfo[olimb]
		if not olimbInfo.isSwing then
			-- avoid collision
			local rotY=gf.rotation:rotationY()
			local side=rotY*vector3(1,0,0)
			local delta=pos-olimbInfo.contactFilter
			local depth=delta:dotProduct(side)
			local thr=0.05
			local weight=math.sin(model:swingPhase(ilimb)*math.pi)
			weight=math.pow(weight, 0.5)
			if legInfo.isLeft then
				if depth<thr then
					pos=pos-side*((depth-thr)*weight)
				end
			else
				if -depth<thr then
					pos=pos+side*((-depth-thr)*weight)
				end
			end
		end

		self:projectToGround(pos)
		return pos, ori, false
	end
	self:projectToGround(pos)
	return pos, ori, true
end



function getDesiredFinalVel(initialRotY, fRotY, v)
	v=math.max(0,v)
	local vv=fRotY*vector3(0,0,v)
	local pendRotY

	if true then
		local arc=require("Kinematics/arc")
		-- prevent extreme turning 
		pendRotY=initialRotY
		local initialfront=pendRotY*vector3(0,0,1)
		local finalfront=vv:copy()
		finalfront:normalize()
		local delta=quater()
		delta:setAxisRotation(vector3(0,1,0), initialfront, finalfront)
		delta:align(quater(1,0,0,0))
		local maxangle=
		arc.getLinearSplineABS(v,
		CT.mat(5,2,
		0,math.rad(180),
		1,math.rad(180),
		2.5,math.rad(120),
		5.5,math.rad(50),
		10.5,math.rad(30)))*(model.maxTurning or 0.5)
		local angle=delta:rotationAngleAboutAxis(vector3(0,1,0))
		angle=math.clamp(angle, -maxangle, maxangle)
		finalrotY=pendRotY*quater(angle, vector3(0,1,0))
		vv=finalrotY*vector3(0,0,v)
	end
	return finalrotY, vv, pendRotY
end


function RagdollSim:change_target_direction()
	self.change_count=0
	self.target_Y=(quater((math.random()-0.5)*1.0*math.pi,upv()))
	this:findWidget("target angle"):sliderValue(self.target_Y:rotationAngleAboutAxis(upv()))
end


function RagdollSim:calcRefDif2(pd_sim)
	assert(self.sim_prev and not self.ref_prev)
	local pd_pose=vectorn()

	pd_pose:assign(pd_sim:getLastSimulatedPose(0))

	-- root orientation difference (see local difference)
	local delta_frame=model.frame_rate*RL_step  -- (delta_frame=0.5)
	local prev_delta_frame=delta_frame+model:getComparisonDeltaFrame()
	local sample=self:sampleSimPose(prev_delta_frame) -- sample before updating replaybuffer for correct timing

	if debug_draw and model.iframe then
		RE.output2('sampleTime', 'mocap=',model.iframe+0.5, model.iframe+prev_delta_frame, 
		'sim=', model.iframe+0.5, model.iframe+prev_delta_frame)
	end

	-- update replayBuffer 
	self:pushSimPose( pd_pose)


	local sim_prev=self.sim_prev -- corresponds to model.iframe
	model:setRefTree(0) -- so that timingMod, earlyTouchDown, and so on doesn't mess-up reference velocity computation.
	local ref_prev=self:getJointStateFromRefTree()

	model:setRefTree(delta_frame) -- 시뮬레이션 한 RL_step진행했으니 iframe+0.5과 비교하는게 맞음
	local info=model.motionData
	local ref_pose=vectorn()
	info.loader_ref:getPoseDOF(ref_pose)

	local sim_rotY=model:getRefRotY(pd_pose, delta_frame) -- corresponds to model.iframe+0.5
	local ref_rotY=model:getRefRotY(ref_pose, delta_frame)

	local sim_curr=self:getJointStateFromSimulator(sim_rotY) -- corresponds to model.iframe+0.5
	local ref_curr=self:getJointStateFromRefTree()
	
	local poseDif=0;
	--
	local sampleRef=model:sampleRefPose(prev_delta_frame)

	local function DIFF_METRIC(x)
		return x
	end
	local function dist2D(v1, v2)
		local v1p=v1:copy()
		local v2p=v2:copy()
		v1p.y=0
		v2p.y=0
		return v1p:distance(v2p)
	end
	do
		local prevSimRootOri=model:getRefRotY(sample, prev_delta_frame)
		local prevRefRootOri=model:getRefRotY(sampleRef, prev_delta_frame)

		local deltaSim=prevSimRootOri:inverse()*pd_pose:toQuater(3)
		local deltaRef=prevRefRootOri:inverse()*ref_pose:toQuater(3)


		if model.useWorldCoMvel then
			poseDif=poseDif+ DIFF_METRIC((pd_pose:toQuater(3):inverse()* ref_pose:toQuater(3)):rotationAngle()*5)
		else
			poseDif=poseDif+DIFF_METRIC((deltaSim:inverse()*deltaRef):rotationAngle()*5)
		end


		--local dy=(deltaSimY:inverse()*deltaRefY):rotationAngle()
		--poseDif=poseDif+dy*dy -- y orientation is more important


		deltaSim=prevSimRootOri:inverse()*(pd_pose:toVector3(0)-sample:toVector3(0))
		deltaRef=prevRefRootOri:inverse()*(ref_pose:toVector3(0)-sampleRef:toVector3(0))

		poseDif=poseDif+DIFF_METRIC(deltaSim:distance(deltaRef))


		local EFdif=0
		local refCoordMocap=transf(prevRefRootOri, sampleRef:toVector3(0))
		refCoordMocap.translation.y=0
		local refCoordSim=transf(prevSimRootOri, sample:toVector3(0))
		self:projectToGround(refCoordSim.translation)

		if debug_draw then
			dbg.namedDraw('Axes', MotionDOF.rootTransformation(sample), 'pd_prev', 100)
			dbg.namedDraw('Axes', MotionDOF.rootTransformation(pd_pose), 'pd_curr', 100)
			dbg.namedDraw('Axes', refCoordSim, 'pd_refcoord', 100)

			local function moveT(T, v)
				return transf(T.rotation, T.translation+v)
			end
			dbg.namedDraw('Axes', moveT(MotionDOF.rootTransformation(sampleRef), vector3(1,0,0)), 'ref_prev', 100)
			dbg.namedDraw('Axes', moveT(MotionDOF.rootTransformation(ref_pose), vector3(1,0,0)), 'ref_curr', 100)
			dbg.namedDraw('Axes', moveT(refCoordMocap, vector3(1,0,0)), 'ref_refcoord', 100)
		end

		for ileg=1,2 do
			if not self.legInfo[ileg].isSwing then
				-- toe

				
				EFdif=EFdif+dist2D(refCoordMocap:toLocalPos(ref_curr.pos(ileg*2-1)),refCoordSim:toLocalPos(sim_curr.pos(ileg*2-1)))*3
				-- heel
				EFdif=EFdif+dist2D(refCoordMocap:toLocalPos(ref_curr.pos(ileg*2)),refCoordSim:toLocalPos(sim_curr.pos(ileg*2)))*3
				
				-- neck
				--EFdif=EFdif+refCoordMocap:toLocalPos(ref_curr.pos(5)):distance(refCoordSim:toLocalPos(sim_curr.pos(5)))
				-- com
				EFdif=EFdif+(refCoordMocap:toLocalPos(ref_curr.pos(0)):distance(refCoordSim:toLocalPos(sim_curr.pos(0))))*3

			
				-- toe-to-com (doesn't work very well)
				--EFdif=EFdif+refCoordMocap:toLocalDir(ref_curr.pos(ileg*2-1)-ref_curr.pos(0)):distance(refCoordSim:toLocalDir(sim_curr.pos(ileg*2-1)-sim_curr.pos(0)))*2
				-- heel-to_com (doesn't work very well)
				--EFdif=EFdif+refCoordMocap:toLocalDir(ref_curr.pos(ileg*2)-ref_curr.pos(0)):distance(refCoordSim:toLocalDir(sim_curr.pos(ileg*2)-sim_curr.pos(0)))*10
			else
				-- neck
				--EFdif=EFdif+(refCoordMocap:toLocalPos(ref_curr.pos(5)):distance(refCoordSim:toLocalPos(sim_curr.pos(5))))
				-- com
				EFdif=EFdif+(refCoordMocap:toLocalPos(ref_curr.pos(0)):distance(refCoordSim:toLocalPos(sim_curr.pos(0))))
				--EFdif=EFdif+0.4
			end

			if false and debug_draw and hasGUI then
				if not self.legInfo[ileg].isSwing then
					--dbg.draw('Sphere', ref_curr.pos(ileg*2-1)*100, 'ballr_orig'..(ileg*2-1), 'green')
					--dbg.draw('Sphere', ref_curr.pos(ileg*2)*100, 'ballr_orig'..(ileg*2), 'green')

					-- toe
					dbg.draw('Sphere', (refCoordSim*refCoordMocap:toLocalPos(ref_curr.pos(ileg*2-1)))*100, 'ballr'..(ileg*2-1), 'green')
					dbg.draw('Sphere', (refCoordSim*refCoordSim:toLocalPos(sim_curr.pos(ileg*2-1)))*100, 'balls'..(ileg*2-1),'white')

					-- heel
					dbg.draw('Sphere', (refCoordSim*refCoordMocap:toLocalPos(ref_curr.pos(ileg*2)))*100, 'ballr'..(ileg*2), 'green')
					dbg.draw('Sphere', (refCoordSim*refCoordSim:toLocalPos(sim_curr.pos(ileg*2)))*100, 'balls'..(ileg*2),'white')

					-- neck
					dbg.draw('Sphere', (refCoordSim*refCoordMocap:toLocalPos(ref_curr.pos(5)))*100, 'ballr5'..ileg, 'green')
					dbg.draw('Sphere', (refCoordSim*(refCoordSim:toLocalPos(sim_curr.pos(5))))*100, 'ballc5'..ileg,'white')
					dbg.draw('Sphere', (refCoordSim*refCoordMocap:toLocalPos(ref_curr.pos(0)))*100, 'ballr0'..ileg, 'green')
					dbg.draw('Sphere', (refCoordSim*(refCoordSim:toLocalPos(sim_curr.pos(0))))*100, 'ballc0'..ileg,'white')
				else
					--dbg.erase('Sphere',  'ballr_orig'..(ileg*2-1))
					--dbg.erase('Sphere',  'ballr_orig'..(ileg*2))

					dbg.erase('Sphere',  'ballr'..(ileg*2-1))
					dbg.erase('Sphere',  'balls'..(ileg*2-1))

					dbg.erase('Sphere',  'ballr'..(ileg*2))
					dbg.erase('Sphere',  'balls'..(ileg*2))

					dbg.erase('Sphere',  'ballr0'..ileg)
					dbg.erase('Sphere', 'ballc0'..ileg)
					dbg.erase('Sphere',  'ballr5'..ileg)
					dbg.erase('Sphere', 'ballc5'..ileg)
				end
			end
		end
		if not self.legInfo[1].isSwing and not self.legInfo[2].isSwing then
			-- toe-to-toe 
			EFdif=EFdif+dist2D(refCoordMocap:toLocalDir(ref_curr.pos(1*2-1)-ref_curr.pos(2*2-1)),refCoordSim:toLocalDir(sim_curr.pos(1*2-1)-sim_curr.pos(2*2-1)))*3
			-- heel-to-heel
			EFdif=EFdif+dist2D(refCoordMocap:toLocalDir(ref_curr.pos(1*2)-ref_curr.pos(2*2)),refCoordSim:toLocalDir(sim_curr.pos(1*2)-sim_curr.pos(2*2)))*3
		end

		poseDif=poseDif+EFdif*(model.EFdif_scale or 1.0/8.0)
	end
	--print(poseDif)

	local numBone=self.loader:numBone() -- == 2 (dummy root=0 and CDM=1)

	poseDif=poseDif*10

	local simulator=self:getCDMsimulator()
	local sim_state=simulator:getWorldState(0)
	local pd_COM=sim_state:globalFrame(1).translation
	local ref_COM=info.loader_ref:bone(1):getFrame().translation
	local COM_lvdif= 0
	local sim_rotY=model:getRefRotY(pd_pose)
	local ref_rotY=model:getRefRotY(ref_pose)
	local weight_COMLVy=model.weight_COMLVy or 1
	--for i=1, numBone-1 do -- buggy. sim_curr.pos(1) means RIGHTANKLE.
	do
		local i=0 -- means COM here.
		local target_delta=sim_rotY:inverse()*self.target_Y

		if model.useWorldCoMvel then
			COM_lvdif=COM_lvdif+DIFF_METRIC((sim_curr.pos(i)-sim_prev.pos(i)):distance( ref_curr.pos(i)-ref_prev.pos(i))/RL_step)
		else
			if model.doNotUseFacingReward then
				target_delta:identity()
			end
			local v1=sim_rotY:inverse()*(sim_curr.pos(i)-sim_prev.pos(i))
			local v2=(target_delta*ref_rotY:inverse())*(ref_curr.pos(i)-ref_prev.pos(i))
			RE.output2('v12', v1, v2)

			local dxz=dist2D(v1, v2)
			local dy=math.abs(v1.y-v2.y)*weight_COMLVy
			COM_lvdif=COM_lvdif+(math.sqrt(dxz*dxz+dy*dy))/RL_step
		end
		if debug_draw then
			RE.output2('com_lvdif', sim_rotY:inverse()*(sim_curr.pos(i)-sim_prev.pos(i))/RL_step,
			ref_rotY:inverse()*(ref_curr.pos(i)-ref_prev.pos(i))/RL_step)
			print('lvdif', (sim_curr.pos(i)-sim_prev.pos(i)):length(), (ref_curr.pos(i)-ref_prev.pos(i)):length())
		end
	end
	COM_lvdif=COM_lvdif/(numBone-1)


	self.sim_prev =sim_curr
	
	local step_stand_reward=10

	local weight_pose=model.weight_pose or 0.65*5*0.5
	local weight_endE=0.15*40
	local weight_COM=0.1*40
    local weight_COMLV=model.weight_COMLV or 0.1*100
	local res

    --local cost= (poseDif*weight_pose) +(COMDif*weight_COM) +(COM_lvdif*weight_COMLV)
    local cost= (poseDif*weight_pose) +(COM_lvdif*weight_COMLV) -- unused
    --print((poseDif*weight_pose),
     --     (COM_lvdif*weight_COMLV), step_reward)

    --RE.output2('diff', (poseDif*weight_pose), (COM_lvdif*weight_COMLV), step_reward)

	return cost, poseDif/10, COM_lvdif, ref_curr.pos(0):distance(ref_prev.pos(0))/RL_step
end
function RagdollSim:projectToGround(v)
	if not model.useTerrain then 
		v.y=0
		return 
	end
	v.y=g_terrain:getTerrainPos(v).y
end
-- todo: search this 
function RagdollSim:projectToGroundHull(v, li)
	--v.y=g_terrain:getTerrainPos(v).y
	if not model.useTerrain then return end
	assert(li.hull)
	-- mostly okay
	--assert(v.z>li.hull[1](0).z and v.z<li.hull[1](li.hull[1]:size()-1).z)
	if true then
		local hulltraj=li.hull[1]
		local minDist=1000
		local argMin
		for i=0, hulltraj:size()-1 do
			local vv=hulltraj(i)-v
			vv.y=0
			if vv:length()<minDist then
				minDist=vv:length()
				argMin=i
			end
		end
		v.y=hulltraj(argMin).y
	else
		local hulltraj=li.hull[1]
		local v1=hulltraj(hulltraj:size()-1)-hulltraj(0)
		v1.y=0
		local dist=v1:length()
		v1:normalize()
		local v2=v-hulltraj(0)
		v2.y=0

		local weight=v2:dotProduct(v1)/dist

		v.y=hulltraj:sample((hulltraj:size()-1)*weight).y
	end
end
