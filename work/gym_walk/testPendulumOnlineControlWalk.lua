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
useWoody=false

require("gym_walk/pendulumOnlineControl")
fc=require("gym_walk/footCoord")
require("gym_walk/PlannerInfo")
require("gym_walk/ConvexHull2D")
TE=require("gym_walk/COMtrajEdit")
require('gym_walk/pendulumOnlineWalk')
function handleRendererEvent()
	return 0
end
function frameMove() 
	return 0
end
package.path=package.path..";../Samples/QP_controller/ILIPM/?.lua" --;"..package.path
require("gym_walk/collisionAvoid")

function PendPlanner:createUI()
	createUI()
	local maxSpeed=1.1
	for i,v in ipairs(input.states) do
		maxSpeed=math.max(maxSpeed, input[v[1]].maxspeed or 1)
	end
	this:findWidget("speed"):sliderRange(0.01, maxSpeed or 10)
	this:findWidget('draw pend traj'):checkButtonValue(false)
	this:create('Check_Button', 'solve ik', 'solve ik')
	this:widget(0):checkButtonValue(true)
	this:create("Check_Button", "use terrain", "use terrain", 1)
	this:widget(0):checkButtonValue(false)
	this:create("Button", "change motion type", "change motion type", 1)
	this:updateLayout()
end
function ctor()

	--RE.FltkRenderer():onCallback('OgreTraceManager')

	planner=PendPlanner(totalTime) 
	planner:createUI()

	--this:create("Button", 'save state', 'save state')
	--this:create("Button", 'load state', 'load state')

	this:redraw()

	m_pendulum=IPC3d("gym_walk/cart_pole_ball.wrl",0, 9.8,1.0/30, 30000)
	m_pendulum.skin:setVisible(false)

	planner:initializeGlobal()

	planner.refPendState={
		vector3(0,0.948, 0), -- com
		vector3(0,0,0), -- dotcom
		quater(1,0,0,0), -- ang
		vector3(0,0,0), -- angvel
		vector3(0,0,0), -- zmp
		vector3(0,0,0), -- zmpvel
	}
	planner:replan()

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
	for i, state in ipairs(input.states) do
		local motionType=state[1]
		state.mSampler=fc.PoseSampler(mSolverInfo.effectors, mMot.loader, mMot.motionDOFcontainer.mot, input, motionType)
		state.mSeq=setInput(motionType)
	end
	mDiscontinuityRemover=fc.DiscontinuityRemover(mMot.loader)
	mCOMDiscontinuityRemover=fc.DiscontinuityRemoverLin(3)
	local initialPose=input.states[g_currState].mSampler:samplePose(1.5,3.5)
	initialPose:set(1, 1.0)
	g_prevPose=initialPose:copy()
	g_prevDesiredPose=initialPose:copy()

	--mMot.skin:setMaterial('lightgrey_transparent')
	--mSampleSkin=fc.createSkin(mMot.loader, input.skinScale)
	--mSampleSkin:setTranslation(-150,0,0)
	--local pose=mSampler:samplePose(1.5, 3.5)
	--mSampleSkin:setPoseDOF(pose)
	if false then
		mTestSkin=fc.createSkin(mMot.loader, input.skinScale)
		local pose=input.run.COMy_down_pose_z[1]:copy()
		pose:set(1, pose(1)+1.0)
		pose:set(0, pose(0)+1.0)
		mTestSkin:setPoseDOF(pose)
		mMot.loader:setPoseDOF(pose)
		local com=mMot.loader:calcCOM() print(com)
		pose:setAllValue(0.0) pose:set(3,1) mMot.loader:setPoseDOF(pose)
		local com=mMot.loader:calcCOM() print(com)
	end

	mCOMfilter=OnlineLTIFilter(CT.vec(vector3(0,0,0)), 5)
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
