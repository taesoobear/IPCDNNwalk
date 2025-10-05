
require("config")
require("module")

require('gym_walkCDM/testWalkV2_FA')
require("gym_walkCDM/FDtracker")
if true then
	-- banked run
	desiredSpeed=1.15 g_history={filename='bankedRunNew.dat'}
	desiredSpeedRun=5
	g_uiHistory={mode='none', filename='ui_bankedRun2.dat', data=vector3N()}
	--g_uiHistory={mode='load', filename='ui_bankedRun1.dat', data=vector3N()} -- for comparison
	g_useSlopeBasedSpeedControl=true
	--[[

		local origSpeed= this:findWidget("speed"):sliderValue()
		if self.prevGoalPos and g_useSlopeBasedSpeedControl and planner:hasTerrain() then
			local h1=planner:getTerrainHeight(0,0)
			local pgp=pi:toZUP_pos(self.prevGoalPos)
			local h2=planner:getTerrainHeight(pgp.x, pgp.y)

			local slope=math.cos(math.atan2(h2-h1, pgp:length()))
			this:findWidget("speed"):sliderValue(origSpeed*slope)
		end
	]]
else
	desiredSpeed=2.68 g_history={filename='runtest_slow.dat'}
	g_uiHistory={mode='load', filename='ui.dat', data=vector3N()}
end

ctor_FA=ctor
function ctor()
	g_FDtracker=FDTracker( vector3(0,9.8,0))
	if RE.ogreSceneManager().setFog then
		RE.ogreSceneManager():setFog( 0.8,0.8,0.8, 0.0,1200, 3800 )
	end
	_createTerrain()
	ctor_FA()
	-- you can increase upto 5.
	-- after that, it still works but IK causes...
	this:findWidget("speed"):sliderValue(desiredSpeed or 2.6)
	this:create('Button', 'start capture', 'start capture')

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
	this:updateLayout()


	if g_uiHistory.mode=='load' then
		g_uiHistory.data=util.loadTable(g_uiHistory.filename)[1]
		g_uiHistory.currRow=0

		RE.viewpoint():setFOVy(45.000000)
		RE.viewpoint():setZoom(1.000000)
		RE.viewpoint().vpos:set(91.319267, 234.726742, 544.214746)
		RE.viewpoint().vat:set(37.051364, 89.392482, -26.296569)
		RE.viewpoint():update()
	end

end



function EVR:onFrameChanged(win, iframe)
	local switchFrame=planner.switchTime*framerate
	RE.output('switchFrame', switchFrame, g_iframe, g_global_time)
	local totalTime=planner.totalTime
	if g_iframe>switchFrame then
		g_foot=not g_foot

		planner:setInitialState(planner:getState(planner.switchTime))
		local plannerInfo=planner

		if g_uiHistory.mode=='save' then
			local data=g_uiHistory.data
			if g_targetPos then
				data:pushBack(g_targetPos)
			else
				data:pushBack(vector3(0,-100,0))
			end
			if math.fmod(data:size(),2)==0 then
				util.saveTable({data}, g_uiHistory.filename)
			end
		elseif g_uiHistory.mode=='load' then
			if g_uiHistory.currRow >=g_uiHistory.data:size() then
				g_uiHistory.currRow=g_uiHistory.data:size()-1
			end
			local targetPos=g_uiHistory.data(g_uiHistory.currRow)
			if targetPos.y>-99 then
				g_targetPos=targetPos+vector3(0,0,50)
			else
				g_targetPos=nil
			end
			--if g_targetPos then
			--	local node=RE.createEntity('flag', 'flag.mesh', 'yellow')
			--	node:scale(3,3,3)
			--	node:translate(g_targetPos)
			--end
			g_uiHistory.currRow=g_uiHistory.currRow+1
		end


		if g_targetPos then
			local pos=planner:getInitialPos()
			local ori=planner:getInitialRotY()
			local dir=g_targetPos/100-pos
			dir:normalize()
			local finalRotY=quater()
			finalRotY:setAxisRotation(vector3(0,1,0), vector3(0,0,1), dir)
			local angle=finalRotY:rotationAngleAboutAxis(vector3(0,1,0))
			this:findWidget('finalrotY'):sliderValue(angle)
		end

		planner:replan(g_foot)
		--g_iframe=g_iframe-math.floor(switchFrame)
		g_iframe=g_iframe-switchFrame
	end
	local iframe=g_iframe -- override. fixed frame rate
	planner:drawFrame(iframe/framerate)

	if math.fmod(math.round(g_global_time*framerate)+1, 70)==0 then
		g_pushDir=quater(math.random(), vector3(0,1,0))*vector3(1,0,0)
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

onCallback_LR=onCallback
function onCallback(w, uid)
	if w:id()=='start capture' then
		local fn='dump'
		RE.motionPanel():motionWin():playFrom(0)
		RE.renderer():setScreenshotPrefix('../dump/'..fn)
		RE.renderer():screenshot(true)
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
		onCallback_LR(w, uid)
	end
end

function handleRendererEvent(ev, button, x, y)
	if g_uiHistory.mode=='load' then return 0 end
		--print(ev, x,y,button)
	if ev=="PUSH" then
		local xPos=RE.FltkRenderer():screenToWorldXZPlane(x, y)
		--print("PUSH")
		return 1
	elseif ev=="DRAG" then
		local xPos=RE.FltkRenderer():screenToWorldXZPlane(x, y)
		--print(x,y,button)
		return 1
	elseif ev=="RELEASE" then
		return 1
	elseif ev=="MOVE" then

		local out=vector3()
		-- use proper ray pick
		local ray=Ray()
		RE.FltkRenderer():screenToWorldRay(x, y,ray)
		ray:translate(-g_terrainPos)
		local terrain=g_terrain
		local normal=vector3()
		out=terrain:pick(ray, normal)
		out:radd(g_terrainPos)
		--dbg.draw('Sphere', out, "cursor_on_terrain", "bullet",5)
		local node=RE.createEntity('flag', 'flag.mesh', 'yellow')
		node:scale(3,3,3)
		node:translate(out)

		g_targetPos=out
		return 1
	elseif ev=='KEYUP' then
		if button=='w' then
			onCallback(this:findWidget('change motion type'), 0)
			if g_desiredState==2 then
				this:findWidget("speed"):sliderValue(desiredSpeedRun or 2.6)
			end
		elseif button=='e' then
			onCallback(this:findWidget('change motion type'), 0)
			if g_desiredState==1 then
				this:findWidget("speed"):sliderValue(desiredSpeed)
			end
		end
		return 1
	end
	return 0
end
