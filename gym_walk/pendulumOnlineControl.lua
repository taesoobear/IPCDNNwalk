
require("subRoutines/Timeline")
require("gym_walk/footCoord")

if useMonopod then
	TimingOpt=require("gym_walk/timingOptMonopod")
elseif useQuadruped then
	initialSpeed=0.76
	finalRotY=0
	TimingOpt=require("gym_quadCDM/timingOptQuadruped")
	motionType='donkey'
	timeScale=1.5
	isHorse=true
	require('gym_quadCDM/anymal')
	require("gym_quadCDM/getSeq_multilimb")
else
	TimingOpt=require("gym_walk/timingOpt")
	require("gym_walk/getSeq")
	if true then
		input.defaultMotionType='walk'
		input.states={{'walk'}, {'run'}}
		g_currState=1
		g_prevState=1
	else
		input.defaultMotionType='run'
		input.states={{'walk'}, {'run'}}
		g_currState=2
		g_prevState=2
	end
	--input.run.strideWeight=0.5
	--motionType='walk'
	--motionType='run' 
end
framerate=30
totalTime=1.0
height=1.0

function setInput(motionType)
	local out={}
	out.defaultSeq=getSeq(input[motionType])
	out.strength=input[motionType].pendstrength or 8 
	out.input=input[motionType]
	return out
end

function matrixn:sampleVec(t, c)
	local v=vectorn()
	if t>self:rows()-1 then
		t=self:rows()-1
	end
	self:sampleRow(t, v)
	return v:toVector3(c)
end
function matrixn:sampleQuat(t, c)
	local v=vectorn()
	if t>self:rows()-1 then
		t=self:rows()-1
	end
	self:sampleRow(t, v)
	local q=v:toQuater(c)
	q:normalize()
	return q
end
matrixnView.sampleQuat=matrixn.sampleQuat
matrixnView.sampleVec=matrixn.sampleVec
function Constraints:_calcPlaneNormal()
	return vector3(0,1,0)
end

function createUI()
	--[[
	this:create("Button", "debug", "debug", 1)

	this:create("Value_Slider", "initialturning", "initialturning", 1)
	this:widget(0):sliderRange(-2, 2)
	this:widget(0):sliderValue(0)
	--]]


	this:create("Value_Slider", "turning", "turning", 1)
	this:widget(0):sliderRange(-2, 2)
	this:widget(0):sliderValue(0)
	this:widget(0):clearVisible()

	--this:create("Value_Slider", "initialspeed", "initialspeed", 1)
	--this:widget(0):sliderRange(0, 4)
	--this:widget(0):sliderValue(0.2)

	this:create("Value_Slider", "speed", "speed", 1)
	this:widget(0):sliderRange(0.02, 13)
	--this:widget(0):sliderValue(3.6)
	this:widget(0):sliderValue(1.7) --walk
	--this:widget(0):sliderValue(4.2) --trot
	--this:widget(0):sliderValue(7.7) --trot

	--this:create("Value_Slider", "rotY", "rotY", 1)
	--this:widget(0):sliderRange(-4, 4)
	--this:widget(0):sliderValue(0)
	--this:widget(0):clearVisible()

	this:create("Value_Slider", "finalrotY", "finalrotY", 1)
	this:widget(0):sliderRange(-4, 4)
	this:widget(0):sliderValue(0)

	--this:create("Value_Slider", "total time", "total time", 1)
	--this:widget(0):sliderRange(0, 2)
	--this:widget(0):sliderValue(1.5)


	this:create("Check_Button", "draw pend traj", "draw pend traj")
	this:widget(0):checkButtonValue(true)
	this:updateLayout()
end

function ctor()
	RE.FltkRenderer():onCallback('OgreTraceManager')
	mTimeline=Timeline("Timeline", 10000, 1/framerate)
	mEventReceiver=EVR()
	g_iframe=0
	g_global_time=0
	mCameraInfo={
		vpos= RE.viewpoint().vpos:copy(),
		vat=RE.viewpoint().vat:copy(),
	}

	planner=PendPlanner(totalTime) 

	createUI()

	m_pendulum=IPC3d("gym_walk/cart_pole_ball.wrl",0, 9.8,1.0/30, 30000)

	planner:replan()
end


function onCallback(w, userData)
	print('onc')
	if w:id()=="debug" then
		g_debugFlag=true
	end
end

function dtor()
end

function handleRendererEvent(ev, button, x, y)
	return 0
end

			
function frameMove(fElapsedTime)
end


function rangeTest(A,B,control, min)
	local o=(B-A)*min
	if (B-A-o):dotProduct(control-A-o)<0 then return false end
	if (A-B+o):dotProduct(control-B+o)<0 then return false end
	if (A-B):length()<min then return false end
	return true
end

PendPlanner=LUAclass()

function PendPlanner:__init(totalTime, initialEulerVel)
	local offset_z=0.6
	local offset_x=0.17
	local offset_head=0.05
	local offset_head2=0.1
	if not isHorse then
		offset_z=0.4
		offset_x=0.25
		offset_head=0
		offset_head2=0
	end
	local offsets={
		vector3(offset_x,0,-offset_z-offset_head),  
		vector3(-offset_x, 0, -offset_z-offset_head),
		vector3(offset_x,0,offset_z-offset_head2),  
		vector3(-offset_x, 0,offset_z-offset_head2), 
	}
	self.offsets=offsets
	local initialState={
		vector3(0,height,0), -- pos
		vector3(0,0,0), -- vel
		quater(1,0,0,0), --ori
		vector3(0,0,0), -- angvel
		-- spprtCoord
		spprtCoord={
			{
				transf(quater(1,0,0,0), offsets[1]),
				transf(quater(1,0,0,0), offsets[2]),
				transf(quater(1,0,0,0), offsets[3]),
				transf(quater(1,0,0,0), offsets[4]),
			},
			{
				transf(quater(1,0,0,0), offsets[1]),
				transf(quater(1,0,0,0), offsets[2]),
				transf(quater(1,0,0,0), offsets[3]),
				transf(quater(1,0,0,0), offsets[4]),
			},
		},
		lastContactTime=
		{
			CT.vec(0,-11,-16,-4)/30.0*timeScale -defaultSeq[1]:sum()*timeScale,  -- last contact timing
			CT.vec(0,-11,-16,-4)/30.0*timeScale,  -- last contact timing
		},
	}
	self:setInitialState(initialState)
end
function PendPlanner:setInitialState(t)
	self.initialPos=t[1]:copy()
	self.initialVel=t[2]:copy()
	self.initialOri=t[3]:rotationY():copy()
	self.w0=t[4].y
	self.spprtCoord=t.spprtCoord
	self.lastContactTime=t.lastContactTime
end

function PendPlanner:drawFrame(t)

	-- only for quad


	local pendFrameRate=30
	local ii=t*pendFrameRate
	local com_pos=self.traj[1]:sampleVec(ii,0)

	-- translate the floor every 4m
	local gridpos=math.floor(com_pos.z/4)
	bgnode=RE.ogreSceneManager():getSceneNode("BackgroundNode")
	bgnode:setPosition(vector3(0,0,gridpos*4*100))

	local com_vel=self.traj[1]:sampleVec(ii,3)
	local zmp_pos=self.traj[2]:sampleVec(ii,0)
	local zmp_vel=self.traj[3]:sampleVec(ii,0)

	m_pendulum:setState(zmp_pos, com_pos, zmp_vel, com_vel)
	m_pendulum:draw()

	RE.output2("coM_vel", com_vel)
	for ileg=1,4 do
		dbg.erase('Sphere', "toe"..ileg)
	end

	local t=ii/30
	local legs=vector3N()
	local lc=0
	for ileg, timing in ipairs(self.timing) do
		if ileg==3 then
			lc=legs:size()
		end

		local function getColor(ileg)
			if ileg==1 then 
				return 'red' 
			elseif ileg==2 then 
				return 'white' 
			elseif ileg==3 then 
				return 'green' 
			else
				return 'blue' 
			end
		end
		local endi=1
		for i=-2,endi do
			local proj_com=com_pos:copy()
			proj_com.y=0
			local dist_thr=1.3 -- realistic
			--local dist_thr=0.2 -- to better see timing
			local time_thr=self.switchTime/3
			local footPos
			local footTime
			if i==-2 then
				footPos=self.spprtCoord[#self.spprtCoord-1][ileg].translation
			elseif i==-1 then
				footPos=self.spprtCoord[#self.spprtCoord][ileg].translation
			else
				footPos=timing.footPos(i)
			end
			if i==-2 then
				footTime=self.lastContactTime[#self.lastContactTime-1](ileg-1)
			elseif i==-1 then
				footTime=self.lastContactTime[#self.lastContactTime](ileg-1)
			else
				footTime=timing.timing(i)
			end
			if false and ileg==3 then
				print(ileg, i, footTime, t, time_thr,(footPos-self.offsets[ileg]):distance(proj_com))

				local check=false
				if (footPos-self.offsets[ileg]):distance(proj_com)<dist_thr and math.abs(footTime-t)<time_thr then
					check=true
				end
				if footTime>0 then
					if check then
						dbg.namedDraw('Text', self.traj[1]:sampleVec(footTime*30,0)*100, "Leg3"..i, vector3(1,0,0), 15, tostring('O'..i))
					else
						dbg.namedDraw('Text', self.traj[1]:sampleVec(footTime*30,0)*100, "Leg3"..i, vector3(1,0,0), 15, tostring('X'..i))
					end
					dbg.namedDraw('Sphere', self.traj[2]:sampleVec(footTime*30,0)*100+self.offsets[ileg]*100, "F3"..i, 'green', 15)
				end
			end
			if (footPos-self.offsets[ileg]):distance(proj_com)<dist_thr 
				and math.abs(footTime-t)<time_thr
				then
				dbg.draw('Sphere', footPos*100, 'toe'..ileg, getColor(ileg), 7)
				--dbg.draw('Sphere', (footPos-self.offsets[ileg])*100, 'te'..i..'_'..ileg, getColor(ileg), 10)

				legs:pushBack(footPos)
				legs:pushBack(com_pos+self.offsets[ileg])
			end
		end
	end
	local thickness=10 -- 10 cm
	dbg.namedDraw('Traj', legs:matView():sub(0,lc,0,0)*100, 'goal3', 'solidblue', thickness, 'BillboardLineList' )
	dbg.namedDraw('Traj', legs:matView():sub(lc,0,0,0)*100, 'goal4', 'solidred', thickness, 'BillboardLineList' )


	local rootpos=com_pos
	local attachCamera=true
	if attachCamera then
		local curPos= rootpos*100
		curPos.y=0
		if g_prevRootPos then
			RE.viewpoint().vpos:assign(RE.viewpoint().vpos+curPos-g_prevRootPos)
			RE.viewpoint().vat:assign(RE.viewpoint().vat+curPos-g_prevRootPos)
			RE.viewpoint():update()     
		end
		g_prevRootPos=curPos:copy()
	end
end
function PendPlanner:getInitialPos()
	return self.initialPos
end
function PendPlanner:getInitialVel()
	return self.initialVel
end
function PendPlanner:getInitialRotY()
	return self.initialOri
end
function calcTotalTime()
	return defaultSeq[1]:sum()*2.333 -- L
end
function PendPlanner:redrawGoal()
	local pi=self
	local totalTime0=calcTotalTime()
	local T=totalTime0*1.5
	local planner=pi.planner
	local w=this:findWidget("turning"):sliderValue()
	local w0=self.w0
	local startPos=self:getInitialPos()
	local startVel=self:getInitialVel()


	local v0=startVel:copy()
	v0.y=0

	local v=this:findWidget("speed"):sliderValue()
	

	local goal, goal2, cf

	do
		local finalrotY=quater(this:findWidget('finalrotY'):sliderValue(), vector3(0,1,0))
		local vv=finalrotY*vector3(0,0,v)
		local refPendState=pi.refPendState
		if refPendState then
			local _COM,_dotCOM,_ANG,_ANGVEL,_ZMP,_ZMPVEL=unpack(refPendState)
			-- no feedback at all. (unnecesary)
			-- very slightly more robust.
			goal, goal2, cf=arc.getPendTraj_direction(m_pendulum, self:getInitialRotY(), _COM, _ZMP, _dotCOM, _ZMPVEL, vv, w0, T, strength)
		else
			if not startZMP then
				startZMP=vector3(startPos.x, 0, startPos.z)
			end
			goal, goal2, cf=arc.getPendTraj_direction(m_pendulum, self:getInitialRotY(), startPos, startZMP, v0, v0, vv, w0, T, strength)
		end
	end

	if this:findWidget('draw pend traj'):checkButtonValue() then
		local thickness=10 -- 10 cm
		dbg.namedDraw('Traj', goal*100, 'goal3 com', 'blueCircle', thickness, 'QuadListV' )
		dbg.namedDraw('Traj', goal2*100, 'goal2', 'greenCircle', thickness, 'QuadListV' )
		for i=0, goal:rows()-1, 3 do
			local goalPos=goal:row(i):toVector3(0)
			local goalOri=goal2:row(i):toQuater(6)
			dbg.draw('Axes', transf(goalOri, goalPos), 'final ori'..i, 100, 0.5)
		end
	end

	local timing
	local cf_all
	do
		local Cycle=defaultSeq[1]:sum()
		cf_all= goal2:column(13)
		-- center:  L  R LH RH
		--         12 23 18 30
		--local gaitCycles= CT.vec(11, 12, 6, 16)/30.0 -- R-L, RH-LH, LH-L, L-LH
		--local gaitCycles= CT.vec(11, 6, 16)/30.0 -- R-L, RH-LH, LH-L, L-LH
		local gaitCycles= CT.vec(11, 11, 11, 11)/30.0 -- R-L, RH-LH, LH-L, L-LH
		local gaitStrides= CT.vec(1.2, 1.2, 1.2, 1.2)*0.2
		if not isHorse then
			gaitCycles=gaitCycles*0.5
			gaitStrides=gaitStrides*0.5
		end
		local offsets=self.offsets

		local totalTime=1.5
		local cycle=22/30.0
		local stride=22/30 *0.5-- walking stride.
		if not isHorse then
			cycle=cycle*0.5
			stride=stride*0.5
		end
		if true then
			cycle=cycle*timeScale
			gaitCycles=gaitCycles*timeScale
		end
		-- ileg1 comes first
		--{ileg1, ileg2, useTerm, sample_next_position_of_ileg1}
		local opt=TimingOpt(
		cycle,
		stride,
		--{{2,3, true,false},{4,1,true,true}},
		--{{2,1, false,true}, {3,4, false,false}, {4,2, true,false},{2,4,true, true}, {3,1,true, true }, {1,3,true}}, 
		--{{4,2, true,false},{2,4,true, true}, {3,1,true, true }, {1,3,true}}, 
		{{1,3,true},{3,2,true,false,'leq'},{2,4,true},{4,1,true,-1}}, 
		{{1,2, true, ws=2},{3,4,true}, {4,1,true,-2,ws=2}}, 
		--{{2,1, true,true}, {3,4, true,false},}, 
		gaitCycles,
		gaitStrides,
		totalTime, 
		self.lastContactTime,  -- last contact timing
		offsets,
		self.spprtCoord,
		goal, goal2, cf_all)
		--local opt=TimingOpt(Cycle, Cycle*defaultSeq.input.speed, T, pi.spprtCoord[1], goal2,cf_all)
		timing=opt:getResult()

		local timing1=timing[1].timing
		for i, res in ipairs(timing) do
			local timing=res.timing
			assert(timing:size()>=2)
			assert(timing(0)>=0)
			assert(timing(1)>timing(0))
			if i==1 then
				assert(timing(0)>0.1)
			end
		end
		opt:drawResult()

		--opt:drawResult()
	end


	-- COM, ZMP, ANG, ANGVEL
	return goal, goal2, goal2:sub(0,0,6,10), goal2:sub(0,0,3,6), goal2:sub(0,0,10,13), timing, cf_all
end
function PendPlanner:getSwitchTime()
	local timing=self.timing
	return timing[1].timing(0)
end
function PendPlanner:replan()
	local pi=self
	local COM, ZMP, ANG, ANGVEL, ZMPVEL, timing,cf= pi:redrawGoal( isL, startZMP)

	local pendFrameRate=30
	self.timing=timing
	local switchTime=self:getSwitchTime()
	RE.output2('switchTime', switchTime)
	local ii=switchTime*pendFrameRate
	if self.terrain then
		dbg.console()
		local h1pos=self:toZUP_pos(ZMP:row(0):toVector3(0))
		--local h2pos=self:toZUP_pos(ZMP:row(math.round(self.switchTime*30)):toVector3(0))
		local h2pos=self:toZUP_pos(ZMP:row(math.min(ZMP:rows()-1, math.round(t(goalNodeId)*30))):toVector3(0))
		local h1=planner:getTerrainHeight(h1pos.x, h1pos.y)
		local h2=planner:getTerrainHeight(h2pos.x, h2pos.z)
		--local vh=(h2-h1)/self.switchTime
		local vh=(h2-h1)/t(goalNodeId);
		vh=vh*0.5
		for i=0, t:size()-1 do
			desiredDotCOM(i).z= desiredDotCOM(i).z+vh
		end
		goalPos.z=goalPos.z+h2
	end
	pi.refPendState={
		COM:sampleVec(ii, 0),
		COM:sampleVec(ii, 3),
		ANG:sampleQuat(ii, 0),
		ANGVEL:sampleVec(ii, 0),
		ZMP:sampleVec(ii,0),
		ZMPVEL:sampleVec(ii,0),
	}
	self.traj={
		COM:copy(), ZMP:copy(), ZMPVEL:copy()
	}

	self.switchTime=switchTime
	print(g_timer:stop()/1000.0, "ms pend-replanning.")
end
function PendPlanner:getState(t)
	local pendFrameRate=30
	assert(t==self.switchTime)
	local spprtCoord={}

	for ileg, timing in ipairs(self.timing) do
		spprtCoord[ileg]=transf(timing.footOri(0),timing.footPos(0))
	end
	local nleg=#self.timing
	local lastContactTime=vectorn(nleg)
	for ileg, timing in ipairs(self.timing) do
		lastContactTime:set(ileg-1, timing.timing(0)-t)
	end
	self.refPendState.spprtCoord={
		self.spprtCoord[#self.spprtCoord],
		spprtCoord
	}
	self.refPendState.lastContactTime={
		self.lastContactTime[#self.lastContactTime]-t,
		lastContactTime
	}
	return self.refPendState
end


if EventReceiver then
	--class 'EVR'(EventReceiver)
	EVR=LUAclass(EventReceiver)
	function EVR:__init()
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
		g_iframe=g_iframe+1
		g_global_time=g_global_time+1.0/framerate
	end
end
