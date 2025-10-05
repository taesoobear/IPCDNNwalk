

require("config")
require("module")
require("common")

package.projectPath='./gym_walkCDM'
package.path=package.path..";../../taesooLib/Samples/scripts/?.lua" --;"..package.path
package.path=package.path..";../../taesooLib/Samples/scripts/RigidBodyWin/subRoutines/?.lua" --;"..package.path
package.path=package.path..";../../taesooLib/Samples/scripts/?.lua" --;"..package.path
require("gym_walkCDM/testWalkV2_FA")
require("gym_walkCDM/FDtracker")

ctor_push_FA_orig=ctor

function ctor()
	g_FDtracker=FDTracker( vector3(0,9.8,0))
	--_createTerrain()
	ctor_push_FA_orig(true)

	g_pushDir=vector3(1,0,0)
	this:create('Button', 'change push dir', 'change push dir')
	this:create('Button', 'push', 'push')
	this:create('Button', 'push 200', 'push 200')
	this:create('Button', 'push 400', 'push 400')
	this:create('Button', 'push 600', 'push 600')
	this:create('Button', 'push 800', 'push 800')
	this:create('Button', 'push head 200', 'push head 200')
	this:create('Button', 'push arm 200', 'push arm 200')
	this:create('Button', 'push leg 200', 'push leg 200')

	if false then
		-- run
		this:findWidget('speed'):sliderValue(2.68)
		this:updateLayout()

		initiateChangeMotionType()
	end
		this:updateLayout()

	planner:replan()
end

onCallback_push_FA_orig=onCallback
function onCallback(w, ud)
	g_FDtracker.gizmo=nil
	if w:id()=='use slope' then
		if w:checkButtonValue() then
			planner1.planner:changeTerrainSlope(quater(math.rad(-4), vector3(0,1,0))*vector3(0,0,1), 0)
		end
	elseif w:id()=='change push dir' then
		if g_pushDir.x==1 then
			g_pushDir=vector3(0,-1,0)
		else
			g_pushDir=vector3(1,0,0)
		end
	elseif w:id()=='push' then
		g_FDtracker.force=g_pushDir*100
		g_FDtracker.force_lpos=vector3(0,0,0.03)
	elseif w:id()=='push 200' then
		g_FDtracker.force=g_pushDir*200
		g_FDtracker.force_lpos=vector3(0,0,0.03)
	elseif w:id()=='push 400' then
		g_FDtracker.force=g_pushDir*400
		g_FDtracker.force_lpos=vector3(0,0,0.03)
	elseif w:id()=='push 600' then
		g_FDtracker.force=g_pushDir*600
		g_FDtracker.force_lpos=vector3(0,0,0.03)
	elseif w:id()=='push 800' then
		g_FDtracker.force=g_pushDir*600
		g_FDtracker.force_lpos=vector3(0,0,0.03)
	elseif w:id()=='push head 200' then
		g_FDtracker.force=vector3(200,0,0)
		g_FDtracker.force_lpos=vector3(0,0.65,0.03)
		g_FDtracker.gizmo={'Neck', vector3(0, 0.09, 0), false}
	elseif w:id()=='push arm 200' then
		g_FDtracker.force=vector3(200,0,0)
		g_FDtracker.force_lpos=vector3(-0.18,0.18,0.13)
		g_FDtracker.gizmo={'RightElbow', vector3(-0.27, 0, 0), false}
	elseif w:id()=='push leg 200' then
		g_FDtracker.force=vector3(200,0,0)
		g_FDtracker.force_lpos=vector3(-0.18,-0.22,0.13)
		g_FDtracker.gizmo={'RightHip', vector3(-0.08, -0.25, 0), false}
	else
		onCallback_push_FA_orig(w, ud)
	end
end

function EVR:onFrameChanged(win, iframe)
	local switchFrame=planner.switchTime*framerate
	RE.output('switchFrame', switchFrame, g_iframe, g_global_time)
	local totalTime=planner.totalTime
	if g_iframe>switchFrame then
		planner:setInitialState(planner:getState(planner.switchTime))
		planner:replan()
		g_iframe=g_iframe-switchFrame
	end
	local iframe=g_iframe -- override. fixed frame rate
	planner:drawFrame(g_iframe/framerate)

	RE.output2('gtime', math.round(g_global_time*framerate))
	if math.round(g_global_time*framerate)==70 then
		g_FDtracker.force=g_pushDir*200
		g_FDtracker.force_lpos=vector3(0,0.3,0.03)
	end

	if g_FDtracker.externalForces then
		local dt=g_FDtracker.dt
		local i=(g_iframe/framerate)/dt

		if i<0 then i=0 end	
		local externalForces=g_FDtracker.externalForces
		local pos=g_FDtracker.applicationPoints
		local pi=g_FDtracker
		local f=externalForces:matView():sampleVec(i, 0)
		local p=pos:matView():sampleVec(i, 0)
		print(externalForces:rows(), i)

		if g_FDtracker.gizmo then
			g_FDtracker.gizmo[3]=f
		else
			if f:length()>1e-3 then
				dbg.draw('Arrow', p*100-f, p*100, 'force')
			else
				dbg.erase('Arrow', 'force')
			end
		end
	else
		dbg.erase('Arrow', 'force')
	end

	g_iframe=g_iframe+1
	g_global_time=g_global_time+1.0/framerate
end
