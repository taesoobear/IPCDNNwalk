require("config")

package.projectPath='../Samples/classification/'
package.path=package.path..";../Samples/classification/lua/?.lua" --;"..package.path

require("module")
arc=require('gym_walk/arc')
require("gym_walk/IPC3d_approx")
require("RigidBodyWin/subRoutines/Constraints")

package.path=package.path..";../Samples/sample_luatorch/lua/?.lua" --;"..package.path
package.path=package.path..";../../taesooLib/Samples/scripts/RigidBodyWin/subRoutines/?.lua" --;"..package.path

useMonopod=false
useQuadruped=false

require("gym_walk/pendulumOnlineControl")
fc=require("gym_walk/footCoord")
require("gym_walk/PlannerInfo")
require("gym_walk/ConvexHull2D")
TE=require("gym_walk/COMtrajEdit")
require('gym_walk/trajOptObstacle')
function handleRendererEvent()
	return 0
end
function frameMove() 
	return 0
end
package.path=package.path..";../Samples/QP_controller/ILIPM/?.lua" --;"..package.path
require("gym_walk/collisionAvoid")

function drawBoxes(boxes, name, y, skinscale, margin)

	y= y or 0
	name=name or 'boxes'
	margin=margin or 0
	local lines=vector3N()
	for i, b in ipairs(boxes) do
		if type(b)=='table' then
			b=b[1]
		end
		local min=b.min:toVector3()
		local max=b.max:toVector3()
		min.x=min.x-margin
		min.z=min.z-margin
		max.x=max.x+margin
		max.z=max.z+margin

		min.y=y
		max.y=y
		lines:pushBack(min)
		lines:pushBack(vector3(min.x,y, max.z))
		lines:pushBack(vector3(min.x,y, max.z))
		lines:pushBack(max)
		lines:pushBack(max)
		lines:pushBack(vector3(max.x,y, min.z))
		lines:pushBack(vector3(max.x,y, min.z))
		lines:pushBack(min)
	end

	--dbg.draw('Traj', lines:matView()*skinscale, name, 'solidgreen', 5, 'BillboardLineList')
	dbg.draw('Traj', lines:matView()*skinscale, name, 'solidgreen', 5, 'LineList')
end
function createBoxes(planner1)
	local function createBox(x, z, maxHalfGapSize, minHalfGapSize)

		minHalfGapSize=minHalfGapSize or 0.1
		local t=sop.map(math.random(),0,1,minHalfGapSize,maxHalfGapSize)
		local p=(math.random()-0.5)*0.3
		return Box2D(vector2(x-0.3, z-t+p), vector2(x+0.3, z+t+p))
	end

	local z=0.5 
	local maxHalfGapSize=0.3
	boxes={}
	for i=0, 40 do
		table.insert(boxes, createBox(math.random(), z, maxHalfGapSize))
		z=z+sop.map(math.random(), 0, 1, 0.8, 1.0)
	end
end
function createAndDrawBoxes(planner1)
	createBoxes(planner1)
	_drawBoxes(planner1)
end
function _drawBoxes(planner1)
	drawBoxes(boxes,'boxes', 0.01, 100)
end
function PendPlanner:createUI()
	createUI()
	this:findWidget("speed"):sliderRange(0.01, defaultSeq.input.maxspeed or 10)
	this:findWidget("speed"):sliderValue(defaultSeq.input.speed)
	this:findWidget('draw pend traj'):checkButtonValue(true)
	this:create('Check_Button', 'solve ik', 'solve ik')
	this:widget(0):checkButtonValue(true)
	this:create("Check_Button", "use terrain", "use terrain", 1)
	this:widget(0):checkButtonValue(false)
	this:updateLayout()
end

BoundedTerrain=LUAclass()
function BoundedTerrain:__init( filename, imageSizeX, imageSizeY, sizeX, sizeZ, minh, h)
	self.terrain=OBJloader.Terrain(
	filename, imageSizeX, imageSizeY,
	sizeX, sizeZ, h,1,1)
	self.minh=minh
end
function BoundedTerrain:height(x, normal)
	assert(not normal)

	if self.terrain:isInsideTerrain(x) then
		return self.terrain:height(x)+self.minh
	else
		return 0
	end
end

function PendPlanner:changeTerrain(draw, s, h, minh)
	local planner=self
	s=s or 18
	h=h or 1.3

	if minh then
		self.terrain=BoundedTerrain(
		"../Resource/crowdEditingScripts/terrain/heightmap1_256_256_2bytes.raw", 256, 256, 
		s, s, minh,h)
	else
		self.terrain=OBJloader.Terrain(
		"../Resource/crowdEditingScripts/terrain/heightmap1_256_256_2bytes.raw", 256, 256, 
		s, s, h,1,1)
	end

	bgnode=RE.ogreSceneManager():getSceneNode("BackgroundNode")
	bgnode:setVisible(false)

	if draw then
		g_mesh=OBJloader.Terrain("../Resource/crowdEditingScripts/terrain/heightmap1_256_256_2bytes.raw", 256, 256, s*100,s*100,h*100,1,1)
		-- scale 100 for rendering 
		g_meshToEntity=MeshToEntity(g_mesh, 'meshName')
		--meshToEntity:updatePositionsAndNormals()
		local entity=g_meshToEntity:createEntity('entityName' )
		entity:setMaterialName("CrowdEdit/Terrain1")
		--entity:setMaterialName("red")
		--entity:setMaterialName("checkboard/crowdEditink")
		local node=RE.createChildSceneNode(RE.ogreRootSceneNode(), "mesh_node")
		node:attachObject(entity)
		node:translate(-s*50,-0.1,-s*50)
	end

end
function ctor()

	createAndDrawBoxes(planner)
	--RE.FltkRenderer():onCallback('OgreTraceManager')

	planner=PendPlanner(totalTime) 
	planner:createUI()


	m_pendulum=IPC3d("gym_walk/cart_pole_ball.wrl",0, 9.8,1.0/30, 30000)
	m_pendulum.skin:setVisible(false)

	planner:initializeGlobal()

	planner:replan()
	this:redraw()

end

function PendPlanner:initializeGlobal()
	mTimeline=Timeline("Timeline", 10000, 1/framerate)
	local limbs=input.limbs
	if input.motionType and input[motionType].limbs then
		limbs=input[motionType].limbs 
	end
	mEventReceiver=EVR()
	g_iframe=0
	g_global_time=0
	mCameraInfo={
		vpos= RE.viewpoint().vpos:copy(),
		vat=RE.viewpoint().vat:copy(),
	}
	_mMot=fc.loadMotion(input,true)
	_mSolverInfo=fc.createIKsolver(_mMot.loader, limbs)
	_mSampler=fc.PoseSampler(_mSolverInfo.effectors, _mMot.loader, _mMot.motionDOFcontainer.mot, input, motionType)

	--mMot.skin:setMaterial('lightgrey_transparent')
	--mSampleSkin=fc.createSkin(mMot.loader, input.skinScale)
	--mSampleSkin:setTranslation(-150,0,0)
	--local pose=mSampler:samplePose(1.5, 3.5)
	--mSampleSkin:setPoseDOF(pose)

	mCOMfilter=OnlineLTIFilter(CT.vec(vector3(0,0,0)), defaultSeq.input.filterSize+3)
end
function PendPlanner:__init(totalTime, initialEulerVel)
	local fo=0.05
	local height=0.95
	local offsets={
		vector3(fo,0,0),
		vector3(-fo,0,0)
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
				transf(quater(1,0,0,0), offsets[1]+vector3(0.03,0,0)),
				transf(quater(1,0,0,0), offsets[2]-vector3(0.03,0,0)),
			},
			{
				transf(quater(1,0,0,0), offsets[1]+vector3(0.03,0,0)),
				transf(quater(1,0,0,0), offsets[2]-vector3(0.03,0,0)),
			},
		},
		lastContactTime=
		{
			CT.vec(0,-18)/30.0 -36/30.0,  -- last contact timing
			CT.vec(0,-18)/30.0,  -- last contact timing
		},
	}
	self:setInitialState(initialState)
end
function calcTotalTime()
	return defaultSeq.touchOff:sum()*2.333 -- L
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

		if true then
			local pendRotY=self:getInitialRotY()
			local initialfront=pendRotY*vector3(0,0,1)
			local finalfront=vv:copy()
			finalfront:normalize()
			local delta=quater()
			delta:setAxisRotation(vector3(0,1,0), initialfront, finalfront)
			delta:align(quater(1,0,0,0))
			local maxangle=
			fc.getLinearSpline(v,
			CT.mat(6,2,
			0,0,
			1,math.rad(180),
			1.5,math.rad(120),
			3,math.rad(50),
			5,math.rad(30),
			10,math.rad(20)))
			local angle=delta:rotationAngleAboutAxis(vector3(0,1,0))
			angle=math.clamp(angle, -maxangle, maxangle)
			finalrotY=pendRotY*quater(angle, vector3(0,1,0))
			vv=finalrotY*vector3(0,0,v)

		end

		local refPendState=pi.refPendState
		local _dotCOM, _ZMPVEL
		if refPendState then
			local _COM,_ZMP
			_COM,_dotCOM,_ANG,_ANGVEL,_ZMP,_ZMPVEL=unpack(refPendState)
			startPos=_COM
			startZMP=_ZMP
			-- no feedback at all. (unnecesary)
			-- very slightly more robust.
		else
			if not startZMP then
				startZMP=vector3(startPos.x, 0, startPos.z)
			end
			_dotCOM=v0
			_ZMPVEL=v0
		end
		if true then
			goal, goal2, cf=arc.getPendTraj_direction(m_pendulum, self:getInitialRotY(), startPos, startZMP, _dotCOM, _ZMPVEL, vv, w0, T, strength)
		else

			local rotY=self:getInitialRotY()
			local opt=TrajOptObs(m_pendulum, T, rotY, startPos, startZMP,  _dotCOM, _ZMPVEL, this:findWidget('finalrotY'):sliderValue(), v,  boxes, strength)


			local vn=opt:getResult()
			local vn= quater(vn.x, vector3(0,1,0)) *vector3(0,0,vn.z)

			print('vv', vv, vn)
			goal, goal2, cf=arc.getPendTraj_direction(m_pendulum, rotY, startPos, startZMP, _dotCOM, _ZMPVEL, vn, w0, T, strength)

			dbg.namedDraw('Traj', goal*100, 'goal', 'blueCircle', thickness, 'QuadListY' )
			dbg.namedDraw('Traj', goal2*100, 'goal2', 'greenCircle', thickness, 'QuadListV' )
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

	local timing, pi
	local cf_all
	local dist_z
	do
		local halfCycle=defaultSeq.touchOff:sum()
		cf_all= goal2:column(13)

		timing=timingOpt(self, halfCycle, halfCycle*defaultSeq.input.speed, T, self.lastContactTime, self.spprtCoord, goal, goal2,cf_all)
		--opt:drawResult()
	end

	-- COM, ZMP, ANG, ANGVEL
	return goal, goal2, goal2:sub(0,0,6,10), goal2:sub(0,0,3,6), goal2:sub(0,0,10,13), timing, cf_all
end
function PendPlanner:replan()
	local pi=self
	local COM, ZMP, ANG, ANGVEL, ZMPVEL, timing,cf= pi:redrawGoal( )


	local t=vectorn(20)
	t:linspace(0, totalTime)
	self.t=t
	self.COMheight_offset=vectorn(t:size()) -- only for drawing


	local pendFrameRate=30

	for i=0, t:size()-1 do
		local ii=t(i)*pendFrameRate

		local desiredDotCOM=COM:sampleVec(ii, 3)
		local pendOri=ANG:sampleQuat(ii, 0)
		-- proportional to control force, speed and leaning angle
		local speedw=sop.clampMap(desiredDotCOM:length(), 1, 5, 0,1)
		local angle=sop.clampMap(pendOri:offsetQ():rotationAngle(), 0, math.rad(10), 0, 1)
		local cf=sop.clampMap(cf(math.min(ii, cf:size()-1)), 300, 1000, 0, 1)
		COMheight_offset=-speedw*0.04*cf*angle
		self.COMheight_offset:set(i, COMheight_offset)

	end
	print('timing', g_global_time, timing)


	self.timing=timing
	local switchTime=math.min(timing[1].timing(0), timing[2].timing(0))
	RE.output2('switchTime', switchTime)
	local ii=switchTime*pendFrameRate
	pi.refPendState={
		COM:sampleVec(ii, 0),
		COM:sampleVec(ii, 3),
		ANG:sampleQuat(ii, 0),
		ANGVEL:sampleVec(ii, 0),
		ZMP:sampleVec(ii,0),
		ZMPVEL:sampleVec(ii,0),
		pi.plannerInfo.durAllLong
	}
	self.traj={
		COM:copy(), ZMP:copy(), ZMPVEL:copy()
	}

	self.switchTime=switchTime
end

g_count=0


function getSpprtCoord(self)
	local lastContactTime=self.lastContactTime
	local spprtCoord=self.spprtCoord
	local isL=false
	if math.abs(lastContactTime[#lastContactTime](0)-0)<1e-3 then
		isL=true
	end

	local spprtCoordL, spprtCoordR
	
	if isL then
		spprtCoordL=spprtCoord[#spprtCoord][1]
		spprtCoordR=spprtCoord[#spprtCoord-1][2]
	else
		spprtCoordR=spprtCoord[#spprtCoord][2]
		spprtCoordL=spprtCoord[#spprtCoord-1][1]
	end
	return isL, spprtCoordL, spprtCoordR
end

function convertTimingToV2(self, _timing, isL, spprtCoord, spprtCoordL, spprtCoordR, COM, ZMP)
	-- convert to the new timing format.
	local timing={{},{}}
	timing[1].timing=vectorn()
	timing[2].timing=vectorn()
	timing[1].footPosIndex=intvectorn()
	timing[2].footPosIndex=intvectorn()
	-- timing[1].timing=[1.15,1.8,2.95] -> 0생략.
	-- timing[2].timing=[0.28,1.4] -> 0생략.

	local cleg=0
	if isL then cleg=1 end
	
	local _footPos=vector3N()
	local _footOri=quaterN()
	if isL then
		_footPos:pushBack(spprtCoordL.translation)
		_footOri:pushBack(spprtCoordL.rotation)
	else
		_footPos:pushBack(spprtCoordR.translation)
		_footOri:pushBack(spprtCoordR.rotation)
	end

	for i=0, _timing:size()-1 do
		timing[cleg+1].timing:pushBack(_timing(i))
		timing[cleg+1].footPosIndex:pushBack(_footPos:size())
		local t=_timing(i)
		local footOri=sampleQuatCol(ZMP, t,6)
		local _isL=false
		if cleg==0 then _isL=true end
		_footOri:pushBack(footOri)
		_footPos:pushBack(getPosition(sampleBaseTraj(ZMP, t), footOri, _isL))
		cleg=math.fmod(cleg+1,2)
	end

	local function distance(i)
		local p={}
		p[1]=_footPos(i)
		local o=_footOri(i):rotationY()*vector3(0,0,0.12)
		p[2]=_footPos(i)+o
		p[3]=_footPos(i)-o
		local p2={}
		p2[1]=_footPos(i-1)
		o=_footOri(i-1):rotationY()*vector3(0,0,0.12)
		p2[2]=_footPos(i-1)+o
		p2[3]=_footPos(i-1)-o

		local d=1000
		for i=1,3 do
			for j=1,3 do
				d=math.min(d, p[i]:distance(p2[j]))
			end
		end

		return d
	end
	local c=1
	while c>0 do
		-- collision avoid
		c=0
		local _isL=not isL
		local thr=0.2
		for i=1, _footPos:size()-1 do
			if distance(i)<thr then
				local offsetX=0.005
				if not _isL then offsetX=-offsetX end

				_footPos(i):radd(_footOri(i):rotationY()*vector3(offsetX,0,0))
				if i>1 then
					_footPos(i-1):radd(_footOri(i-1):rotationY()*vector3(-offsetX,0,0))
				end
				c=c+1
			end

			_isL=not _isL
		end
	end

	for ileg, timing in ipairs(timing) do
		local isL=true
		if ileg==2 then
			isL=false
		end
		local footPos=vector3N(timing.timing:size())
		local footOri=quaterN(timing.timing:size())

		for i=0, footPos:size()-1 do
			local t=timing.timing(i)
			footOri(i):assign(sampleQuatCol(ZMP, t,6))
			--footPos(i):assign(getPosition(sampleBaseTraj(ZMP, t), footOri(i), isL))
			footPos(i):assign(_footPos(timing.footPosIndex(i)))
			--print(getPosition(sampleBaseTraj(ZMP, t), footOri(i), isL), footPos(i))
			--if ileg==2 then dbg.namedDraw("Sphere", footPos(i)*100+vector3(0,6,0), 'f'..i) end
		end
		timing.footPos=footPos
		timing.footOri=footOri
	end
	local finalrotY=quater(this:findWidget('finalrotY'):sliderValue(), vector3(0,1,0))
	--TimingOpt.updateStopMode(self, timing,COM,ZMP, spprtCoordL, spprtCoordR, self:getInitialRotY(),finalrotY)
	local stopMode=self.stopMode

	RE.output("stopmode", isL, self.stopMode, self.prevStopMode)
	if stopMode then
		for ileg, timing in ipairs(timing) do
			for i=0, timing.footPos:size()-1 do
				timing.footPos(i):assign(spprtCoord[#spprtCoord][ileg].translation)
			end
		end
	end

	local plannerInfo
	do
		-- set phase duration
		plannerInfo={
			planner={
				dur={},
				setDurationBasePolynomial=function () end,
				setNumForcePolynomials=function() end,
				setBasePolynomial=function() end,
				getDurationBasePolynomial=function () return 0 end,
				setPhaseDurations=function (self, i_constraint, startCon, dur) 
					RE.output2("con"..i_constraint, startCon, dur)
					self.dur[i_constraint]={startCon, dur}
				end,
				_getFootCenterPos=function (self, i_constraint, t)
					local ifoot=math.floor(i_constraint/2)
					local dur=self.dur[i_constraint]
					local id=BipedPlanner._getSegmentID(t, dur[2])
					if id(0)==0 then 
						local tf=self.pi.spprtCoord[ifoot+1]
						return tf.translation:copy(), tf.rotation:copy()
					end
					if id(0)==-1 then
						id:set(0, dur[2]:size()-1)
					end
					local i
					if math.fmod(id(0),2)==1 then
						assert(not dur[1])
						i=math.floor(id(0)/2)
					else
						assert(dur[1])
						i=math.floor((id(0)-1)/2)
					end
					local timing=self.pi.timingPerLimb[ifoot+1]
					assert(i<timing.footPos:size())
					return timing.footPos(i):copy(), timing.footOri(i):copy()
				end,
				getFootCenterPos=function (self, i_constraint, t)
					return select(1,self:_getFootCenterPos( i_constraint,t))
				end
			},
			getFootSeq=function (self)
				local footindex=intvectorn(4)
				if self.isL then
					footindex:setValues(2,0,2,0) -- RLRL
				else
					footindex:setValues(0,2,0,2) -- LRLR
				end
				return footindex
			end,
			spprtCoord={spprtCoordL, spprtCoordR},
			desiredPendOri=ZMP:sub(0,0,6,10):copy(),
			toYUP_pos=function (self, p) return p end
		}
		plannerInfo.planner.pi=plannerInfo
		local R0 
		if isL then
			R0=spprtCoordR.translation:copy()
			L1=spprtCoordL.translation:copy()
			R1=timing[2].footPos(0):copy()
			L2=timing[1].footPos(0):copy()
		else
			R0=spprtCoordL.translation:copy()
			L1=spprtCoordR.translation:copy()
			R1=timing[1].footPos(0):copy()
			L2=timing[2].footPos(0):copy()
		end
		local totalTime
		totalTime=PlannerInfo.setPhaseDurations(plannerInfo, isL, _timing, {R0,L1,R1,L2}, COM, self.prevStopMode, self.stopMode)

		if true then
			-- adjust desired COM
			local durAll=plannerInfo.durAll
			local p7
			if self.refPendState then
				p7=self.refPendState[7]
			end
			adjustDesiredCOM(durAll, p7, COM)


			if false then
				Imp.ChangeChartPrecision(50)
				if not g_signal then g_signal=vectorn() end
				local nf=math.round(_timing(0)*30)+1
				local gnf=math.round(g_global_time*30)
				g_signal:resize(gnf+nf)
				g_signal:range(gnf, gnf+nf):assign(COM:column(1):range(0, nf))
				RE.motionPanel():scrollPanel():addPanel(g_signal)
			end
		end

		plannerInfo.timing=_timing
		plannerInfo.timingPerLimb=timing
		plannerInfo.isL=isL

		PlannerInfo.drawTrajectory(plannerInfo)
		plannerInfo.dist_z=math.min(L1:distance(R1), R1:distance(L2))
	end
	

	self.plannerInfo=plannerInfo
	self.dist_z=plannerInfo.dist_z

	return timing
end

function timingOpt(self, halfCycle, halfStride, T, lastContactTime, spprtCoord, COM, ZMP, cf)
	
	local isL, spprtCoordL, spprtCoordR=getSpprtCoord(self)

	if false then
		-- test foot skipping
		g_count=g_count+1
		if math.fmod(g_count,2)==0 then
			isL=not isL
		end
	end

	--dbg.namedDraw('Axes', spprtCoordL, 'p1', 100)
	--dbg.namedDraw('Axes', spprtCoordR, 'p2', 100)
	--local opt=TimingOpt(halfCycle, halfStride, T, isL, spprtCoordL, spprtCoordR, ZMP,cf, nil, defaultSeq.input)
	--local _timing=opt:getResult()
	--
	_timing=CT.vec(0.78,1.43, 2.03, 2.61, 3.19,3.77)
	assert(_timing:size()>=3)
	assert(_timing(0)>0)
	assert(_timing(1)>_timing(0))
	--assert(_timing(2)>_timing(1))
	--opt:drawResult()
	local timing=convertTimingToV2(self, _timing, isL, spprtCoord, spprtCoordL, spprtCoordR, COM, ZMP)


	--timing[1]k
	return timing
end
drawFrame_old=PendPlanner.drawFrame
function PendPlanner:drawFrame(t)


	if false then
		--local spprtCoord=self.spprtCoord[#self.spprtCoord]
		--local spprtCoord=self.spprtCoord[#self.spprtCoord-1]
		local spprtCoord=self.plannerInfo.spprtCoord
		dbg.namedDraw('Axes', spprtCoord[1], 'p1', 100)
		dbg.namedDraw('Axes', spprtCoord[2], 'p2', 100)
	end
	--drawFrame_old(self,t)

	local pendFrameRate=30
	local ii=t*pendFrameRate
	local COM=self.traj[1]:sampleVec(ii,0)
	local ROOTQ=self.traj[2]:sampleQuat(ii,6)
	local COMVEL=self.traj[1]:sampleVec(ii,3)
	local ANGVEL=self.traj[2]:sampleVec(ii,3)

	local gridpos=math.floor(COM.z/4)
	local gridposx=math.floor(COM.x/4)
	bgnode=RE.ogreSceneManager():getSceneNode("BackgroundNode")
	bgnode:setPosition(vector3(gridposx*4*100,0,gridpos*4*100))
	if true then
		local pi=self.plannerInfo
		local lspline,rspline=unpack(pi.phases)
		local phaseL=fc.getLinearSpline(t, lspline)
		local phaseR=fc.getLinearSpline(t, rspline)
		--mSampleSkin:setPoseDOF(pose)

		local numCon=_mSolverInfo.numCon
		local footPos=vector3N (numCon);


		-- todo
		local Ldist_z=self.dist_z*2.0
		local Rdist_z=self.dist_z*2.0

		RE.output2('dist_z', self.dist_z, phaseL, phaseR)

		local _, lswing, rswing=_mSampler:samplePose(phaseL, phaseR, Ldist_z, Rdist_z)

		local COMy_down_pose=defaultSeq.input.COMy_down_pose
		if COMy_down_pose then
			local COMheight_offset=self.COMheight_offset:sample(sop.map(t, 0, self.t(self.t:size()-1), 0, self.t:size()-1))
			pose=fc.addPose(pose, fc.scalePose(COMy_down_pose[1], 
			-COMheight_offset/COMy_down_pose[2]))
		end

		local swing={lswing, rswing}

		local pi=self.plannerInfo
		local timing=pi.timing
		local T1,T2, OT
		T1=0
		T2=timing(1)
		OT=timing(0)
		local phases={phaseL, phaseR}

		-- local pos to global pos
		for i=0,numCon-1,2 do

			local ifoot=math.floor(i/2)

			local footMidPos, rotY
			local phase=phases[ifoot+1]


			local mNormal=vector3(0,1,0)
			local g_footLen=0.20
			local offset=swing[ifoot+1]
			local swing=offset[1]
			if swing==1.0  then
				local io=-1
				if (pi.isL and i==0) or (not pi.isL and i~=0) then
					io=0
				end
				
				local p1=pi.footsteps[1](io+1)
				local op=pi.footsteps[1](io+2)
				local p2=pi.footsteps[1](io+3)


				--if i==2 then
				--	dbg.namedDraw('Sphere', p1*100+vector3(0,15,0), 'p1')
				--	dbg.namedDraw('Sphere', p2*100+vector3(0,15,0), 'p2')
				--	dbg.namedDraw('Sphere', op*100+vector3(0,15,0), 'op')
				--end


				local rotY1=pi.footsteps[2](io+1)
				local rotYO=pi.footsteps[2](io+2)
				local rotY2=pi.footsteps[2](io+3)
				local weight=offset[2]
				RE.output("swing"..i, weight )
				local function sc(p,q)
					local rotY=q:rotationY()
					-- q_offset*rotY=q
					q_offset=q*rotY:inverse()
					return fc.composeSupportCoordinate(p, q_offset, rotY)
				end
				--dbg.namedDraw('Axes', transf(rotY1, p1), 'p1'..i, 100)
				--dbg.namedDraw('Axes', transf(rotY2, p2), 'p2'..i, 100)
				local p, shear_q, q=fc.calcSwingFootCoord(sc(p1,rotY1), sc(p2, rotY2), sc(op, rotYO), weight, (pi.isL and i==0) or (not pi.isL and i==0) )
				--local p, shear_q, q=fc.calcSwingFootCoord(sc(p1,rotY1), sc(p2, rotY2), sc(op, rotYO), weight, nil)
				footMidPos=p
			assert(footMidPos.x==footMidPos.x)
				rotY=shear_q*q
			else
				if phase>=2 and phase<=3 then
					-- toe support
					footMidPos,rotY=pi.planner:_getFootCenterPos(i+1, t)
				else
					--if t~=0 then dbg.console() end
					footMidPos, rotY=pi.planner:_getFootCenterPos(i, t)
					--if t==0 then
					--	dbg.console()
					--end
				end
				RE.output("swing"..i, 0)
			end
					--dbg.namedDraw('Sphere', footMidPos*100+vector3(0,15,0), 'p'..i)

			local isHeel, heelPos, toePos=fc.getHeelAndToePosition(rotY:rotationY(), COM, footMidPos, mNormal, swing, g_footLen)
			local mOffset=offset[3]



			local shearM=fc.shearM(rotY:rotationY()*rotY:inverse())

			footPos(i):assign(toePos);
			footPos(i+1):assign(heelPos);
			--dbg.draw('Sphere', toePos*100, 'toe'..i, 'blue')
			--dbg.draw('Sphere', heelPos*100, 'heel'..i, 'green')
		end

		local attachCamera=true
		if attachCamera then
			mCOMfilter:setCurrPose(CT.vec(COM))
			local curPos= mCOMfilter:getFiltered():toVector3(0)*100
			curPos.y=0
			if g_prevRootPos then
				RE.viewpoint().vpos:assign(RE.viewpoint().vpos+curPos-g_prevRootPos)
				RE.viewpoint().vat:assign(RE.viewpoint().vat+curPos-g_prevRootPos)
				RE.viewpoint():update()     
			end
			g_prevRootPos=curPos:copy()
		end
		--mMot.skin:setTranslation(50,0,0)

	end
end

