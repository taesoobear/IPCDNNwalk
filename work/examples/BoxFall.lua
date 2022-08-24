
require("config")

require("common")
require("subRoutines/RagdollSim")
require("subRoutines/Timeline")
--[[ parameters ]]--

-- bullet physics engine cannot go higher than 1/1000. I do not understand why.
timestep=1/360 -- 1/80000 works perfectly but too slow. Maybe, we need to employ a game-physics engine like Bullet.
rendering_step=1/30
numBoxes=10
skinScale=100
DEBUG_DRAW=false

function ctor()
	mEventReceiver=EVR()
	mTimeline=Timeline('timeline', 10000, rendering_step)

   
   this:create("Check_Button", "draw skeleton", "draw skeleton", 0, 3,0)
   this:widget(0):checkButtonValue(0)

   this:updateLayout()
   this:redraw()

   RE.viewpoint().vpos:assign(vector3(330.411743, 69.357635, 0.490963))
   RE.viewpoint().vat:assign(vector3(-0.554537, 108.757057, 0.477768))
   RE.viewpoint():update()
   RE.viewpoint():TurnRight(math.rad(viewRotate or 0))
   _start()
end

function dtor()
	if  mTimeline then
		mTimeline:dtor()
		mTimeline=nil
	end
	dtor_sub()
end
function dtor_sub()
   -- remove objects that are owned by C++
   if mSkin~=nill then
      RE.remove(mSkin)
      mSkin=nil
   end
   if mSkin2~=nill then
      RE.remove(mSkin2)
      mSkin2=nil
   end
   -- remove objects that are owned by LUA
   collectgarbage()
end

function _start()
	dtor_sub()
	print("start")
	local smallBoxMesh=Geometry()
	smallBoxMesh:initBox(vector3(0.2,0.1, 0.2)) -- works well.
	--smallBoxMesh:initEllipsoid(vector3(0.05,0.05, 0.05)) -- balls works well too.
	--smallBoxMesh:initCylinder(0.1,0.1, 20) -- does not work with libccd. strange... 

	local floorBox=Geometry()
	floorBox:initBox(vector3(20, 0.2, 40))
	--floorBox:initPlane(20, 40) floorBox:rigidTransform(transf(quater(1,0,0,0), vector3(0,0,0)))
	--floorBox:initBox(vector3(0.2, 0.2, 0.2)) -- libccd works only for small boxes or plane. do not use large boxes when using libccd.

	--sim=Physics.DynamicsSimulator_TRL_LCP("libccd") -- taesoo's simulator
	--sim=Physics.DynamicsSimulator_TRL_LCP("OPCODE") -- taesoo's simulator 
	--sim=Physics.DynamicsSimulator_TRL_LCP("libccd_LBS") -- taesoo's simulator, more accurate collision detector. but not accurate enough. needs to add EE detector
	--sim=Physics.DynamicsSimulator_TRL_LCP("bullet") -- taesoo's simulator, 
	sim=Physics.DynamicsSimulator_TRL_LCP("gjk") 
	--sim=Physics.DynamicsSimulator_physX() 
	--sim=Physics.DynamicsSimulator_TRL_softbody() -- taesoo's simulator. 
	--sim=Physics.DynamicsSimulator_Bullet("bullet")--불릿물리엔진.
	--sim=Physics.DynamicsSimulator_AIST("bullet") -- aist simulator with bullet collision detector
	--sim=Physics.DynamicsSimulator_AIST("libccd")-- aist simulator with libccd collision detector -- needs to fix.
	--sim=Physics.DynamicsSimulator_AIST("opcode")-- aist simulator with opcode collision detector -- needs to fix.
	--sim=Physics.DynamicsSimulator_UT("bullet")
	--sim=Physics.DynamicsSimulator_AIST("opcode")
	--sim=Physics.DynamicsSimulator_UT("opcode")
	-- default로는 velocity damping이 꺼져있음. damping을 키면 timestep크게 써도 좀더 안정된 결과가 나옴.
	--sim:setDamping(0.05, 0.85) -- bullet물리엔진의 기본 값. (bullet은 default로 켜져있음)
	--sim:setDamping(15, 300) -- 떨림이 없어지지만 이쯤되면 물리엔진이 아니라고 봐야함.
	sim:setGVector(vector3(0,9.8,0))

	for i=1,numBoxes do
		sim:createFreeBody(smallBoxMesh)
	end
	sim:createObstacle(floorBox)

	function registerPair(iloader1, iloader2)
		local param=vectorn ()
		--param:setValues(0.5,0.5, 10000,1000)
		param:setValues(0.5,0.5, 10000,1000)
		local loader1=sim:skeleton(iloader1)
		local loader2=sim:skeleton(iloader2)
		local bone_i=loader1:VRMLbone(1)
		local bone_j=loader2:VRMLbone(1)
		sim:registerCollisionCheckPair(loader1:name(),bone_i.NameId, loader2:name(), bone_j.NameId, param)
	end

	if true then
		for i=0, numBoxes do
			for j=i+1, numBoxes do
				registerPair(i,j)
			end
		end
	else
		for i=0, numBoxes do
			for j=0, numBoxes do
				registerPair(i,j)
			end
		end
	end
	-- call init after all collisionCheckPairs are registered
	sim:init(timestep, Physics.DynamicsSimulator.EULER)


	local zero=CT.zeros(7) -- position(3), orientation(4)
	if zero:size()>0 then
		zero:set(3, 1) -- quater(1,0,0,0)
	end
	for i=1, numBoxes do
		zero:set(0, math.random()*0.10)
		zero:set(1, i*0.2)
		zero:set(2, math.random()*0.05)
		local q=quater()
		q:identity()
		q.x=math.random()*0.0
		q.y=math.random()*0.0
		q.z=math.random()*0.0
		q:normalize()
		zero:setQuater(3,q)
		sim:setLinkData(i-1, Physics.DynamicsSimulator.JOINT_VALUE, zero)
		sim:setLinkData(i-1, Physics.DynamicsSimulator.JOINT_VELOCITY, zero)
	end

	sim:initSimulation()

	numSkins=numBoxes+1
	mBoxSkins={}
	for i=1, numSkins do
		local loader=sim:skeleton(i-1)
		mBoxSkins[i]=RE.createVRMLskin(loader, false)
		local s=skinScale
		mBoxSkins[i]:scale(s,s,s)
		mBoxSkins[i]:setPose(sim,i-1)
		if DEBUG_DRAW then
			mBoxSkins[i]:setMaterial('lightgrey_transparent')
		end
	end

end

function onCallback(w, userData)
   if w:id()=="Start" then
      _start()
   end
end


function frameMove(fElapsedTime)
end
function EVR:onFrameChanged(win, iframe)
	local niter=rendering_step/timestep
	for i=1, niter do
		sim:stepSimulation()

		if DEBUG_DRAW then
			sim:drawDebugInformation()
		end
		for j=1,numSkins do
			mBoxSkins[j]:setPose(sim,j-1)
		end
	end
	--sim:drawDebugInformation()
end
