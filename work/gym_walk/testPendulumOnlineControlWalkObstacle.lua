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
require('gym_walk/pendulumOnlineWalk')
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
		return Box2D(vector2(x-0.2, z-t+p), vector2(x+0.2, z+t+p))
	end

	local z=0.5 
	local maxHalfGapSize=0.2
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
	this:findWidget('draw pend traj'):checkButtonValue(false)
	this:create('Check_Button', 'solve ik', 'solve ik')
	this:widget(0):checkButtonValue(true)
	this:create("Check_Button", "use terrain", "use terrain", 1)
	this:widget(0):checkButtonValue(false)
	this:updateLayout()
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
	mMot=fc.loadMotion(input,true)
	local limbs=input.limbs
	if input.motionType and input[motionType].limbs then
		limbs=input[motionType].limbs 
	end
	mSolverInfo=fc.createIKsolver(mMot.loader, limbs)
	mEventReceiver=EVR()
	g_iframe=0
	g_global_time=0
	mCameraInfo={
		vpos= RE.viewpoint().vpos:copy(),
		vat=RE.viewpoint().vat:copy(),
	}
	mSampler=fc.PoseSampler(mSolverInfo.effectors, mMot.loader, mMot.motionDOFcontainer.mot, input, motionType)
	local initialPose=mSampler:samplePose(1.5,3.5)
	initialPose:set(1, 1.0)
	g_prevPose=initialPose:copy()
	g_prevDesiredPose=initialPose:copy()

	--mMot.skin:setMaterial('lightgrey_transparent')
	--mSampleSkin=fc.createSkin(mMot.loader, input.skinScale)
	--mSampleSkin:setTranslation(-150,0,0)
	--local pose=mSampler:samplePose(1.5, 3.5)
	--mSampleSkin:setPoseDOF(pose)

	if true and defaultSeq.input.filterSize~=0 then
		mFilter=OnlineFilter(mMot.loader, initialPose, defaultSeq.input.filterSize+2)
		mCOMfilter=OnlineLTIFilter(CT.vec(vector3(0,0,0)), defaultSeq.input.filterSize+3)
	else
		mCOMfilter=OnlineLTIFilter(CT.vec(vector3(0,0,0)), 5)
	end
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
		if false then
			local finalrotY, vv=self:getDesiredFinalVel(this:findWidget('finalrotY'):sliderValue(), v)
			goal, goal2, cf=arc.getPendTraj_direction(m_pendulum, self:getInitialRotY(), startPos, startZMP, _dotCOM, _ZMPVEL, vv, w0, T, strength)
		else
			local finalrotY, vv=self:getDesiredFinalVel(this:findWidget('finalrotY'):sliderValue(), v)

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

