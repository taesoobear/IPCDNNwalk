
require("config")

require("common")
require("subRoutines/RagdollSim")
require("subRoutines/Timeline")
--[[ parameters ]]--

-- bullet physics engine cannot go higher than 1000hz. I do not understand why.
timestep=1/360 -- for other simulators 1/10000 works perfectly but is too slow. Maybe, we need to employ another game-physics engine.
rendering_step=1/30
numBalls=10
skinScale=100

function ctor()

	mEventReceiver=EVR()
	mTimeline=Timeline('timeline', 10000, rendering_step)
   
   this:create("Check_Button", "draw skeleton", "draw skeleton", 0, 3,0)
   this:widget(0):checkButtonValue(0)

	this:create("Box", 'help msg', "click the play button (or ctrl+p)",0);
	this:create("Value_Slider", "debug", "debug",1);
	this:widget(0):sliderRange(-1,1);

   this:updateLayout()
   this:redraw()

   RE.viewpoint().vpos:assign(vector3(330.411743, 69.357635, 0.490963))
   RE.viewpoint().vat:assign(vector3(-0.554537, 108.757057, 0.477768))
   RE.viewpoint():update()
   RE.viewpoint():TurnRight(math.rad(viewRotate or 0))
   _start()
   --print('bygu test')
   --aa=Physics.CollisionDetector_libccd
   --dbg.console()
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
function dtor()
	if  mTimeline then
		mTimeline:dtor()
		mTimeline=nil
	end
	dtor_sub()
end

function _start()
	dtor_sub()
	print("start")
	--local smallBoxMesh=Geometry()
	--smallBoxMesh:initBox(0.2,0.1, 0.2)

	local floorBox=Geometry()
	floorBox:initBox(vector3(2, 0.2, 2))

	--local bottomBox=Geometry()
	--bottomBox:initBox(500, 6, 500)

	local wrl_file='../Resource/mesh/smallSphere.wrl';
	local wrl_file2='../Resource/mesh/basket.wrl';

	if os.isFileExist(wrl_file) then
		obj_loader={};
		for i=1,numBalls do
			obj_loader[i]=MainLib.VRMLloader(wrl_file);
			obj_loader[i]:setName("ball"..i)
		end
	end
	if os.isFileExist(wrl_file2) then
		basketLoader=MainLib.VRMLloader(wrl_file2);
		basketLoader:setName("basket")

		-- adjust basket position
		basketLoader:setPosition(vector3(0,0.8,0))
	end
	
	--sim=Physics.DynamicsSimulator_TRL_LCP("libccd") 
	sim=Physics.DynamicsSimulator_TRL_LCP("gjk") 
	--sim=Physics.DynamicsSimulator_Trbdl_LCP("gjk") 
	--sim=Physics.DynamicsSimulator_TRL_LCP("opcode") 
	--sim=Physics.DynamicsSimulator_Bullet("bullet") 
	--sim=Physics.DynamicsSimulator_Bullet("bullet") 
	--sim=Physics.DynamicsSimulator_AIST("bullet") 
	--sim=Physics.DynamicsSimulator_AIST("libccd") 
	--sim=Physics.DynamicsSimulator_UT("bullet") 
	--sim=Physics.DynamicsSimulator_AIST("opcode")
	--sim=Physics.DynamicsSimulator_UT("opcode")
	sim:setGVector(vector3(0,9.8,0))

	--sim:createFreeBody(bottomBox);
	sim:registerCharacter(basketLoader) -- character 0

	for i=1,numBalls do
		sim:registerCharacter(obj_loader[i]) -- character 1~numBalls
	end


	floorLoader=MainLib.VRMLloader(floorBox, true) -- fixed joint: true
	floorLoader:setPosition(vector3(0,-0.09,0))
	sim:registerCharacter(floorLoader)  -- character numBalls+1
	floorSkin=RE.createVRMLskin(sim:skeleton(sim:numSkeleton()-1), false)
	floorSkin:scale(100,100,100)

	basket2=basketLoader:copy()
	basket2:setName('basket2') -- has to have a different name
	basket2:setPosition(vector3(0,0,0.5))
	basketSkin2=RE.createVRMLskin(basket2, false)
	basketSkin2:scale(100,100,100)
	sim:registerCharacter(basket2)

	function registerPair(iloader1, iloader2)
		local param=vectorn ()
		--param:setValues(0.5,0.5, 10000,1000)
		param:setValues(0.6,0.6, 10000,1000)
		local loader1=sim:skeleton(iloader1)
		local loader2=sim:skeleton(iloader2)
		local bone_i=loader1:VRMLbone(1)
		local bone_j=loader2:VRMLbone(1)
		--print(loader1:name())
		sim:registerCollisionCheckPair(loader1:name(),bone_i.NameId, loader2:name(), bone_j.NameId, param)
	end
	local numObjects=numBalls+3; -- basket, balls, floor, basket2 in that order.

	for i=0, numObjects-1 do
		for j=i+1, numObjects-1 do
			registerPair(i,j)
		end
	end

	sim:init(timestep, Physics.DynamicsSimulator.EULER)

	local zero=CT.zeros(7) -- position(3), orientation(4)
	if zero:size()>0 then
		zero:set(3, 1) -- quater(1,0,0,0)
	end
	basketpos=vectorn(7);
	basketpos:setAllValue(0);
--	basketpos:set(1,-0.01);
--	basketpos:set(2,3);
	basketpos:set(3,1);
	-- 000 1000
	--sim:setLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, basketpos)
	zero:setAllValue(0);
	zero:set(3,1);
	--sim:setLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, zero)

	--dbg.draw('Sphere', vector3(0,0.2, 0), "balll3", "red", 10)
	for i=1, numBalls do
		zero:set(1, i*0.2+0.3)
		local q=quater()
		q:identity()
		q.x=math.random()*0.1
		q.y=math.random()*0.1
		q.z=math.random()*0.1
		q:normalize()
		zero:setQuater(3,q)
		--sim:setLinkData(i-1, Physics.DynamicsSimulator.JOINT_VALUE, zero)
		--sim:setLinkData(i-1, Physics.DynamicsSimulator.JOINT_VELOCITY, zero)
		sim:setLinkData(i, Physics.DynamicsSimulator.JOINT_VALUE, zero)
		sim:setLinkData(i, Physics.DynamicsSimulator.JOINT_VELOCITY, zero)
	end

	sim:initSimulation()

	mBasketSkin=RE.createVRMLskin(basketLoader, false)
	mBasketSkin:scale(100,100,100)
	mBasketSkin:setPose(sim,0)
	mBallSkins={}
	for i=1, numBalls do
		mBallSkins[i]=RE.createVRMLskin(obj_loader[i], false)
		local s=skinScale
		mBallSkins[i]:scale(s,s,s)
		--mBallSkins[i]:setPose(sim,i-1)
		mBallSkins[i]:setPose(sim,i)
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
	end
	mBasketSkin:setPose(sim,0)
	for j=1,numBalls do
		--mBallSkins[j]:setPose(sim,j-1)
		mBallSkins[j]:setPose(sim,j)
	end
end
