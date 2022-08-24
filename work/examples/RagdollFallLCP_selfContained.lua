require("config")

package.projectPath='../Samples/QP_controller/'
package.resourcePath='../Resource/motion/'
package.path=package.path..";../Samples/QP_controller/lua/?.lua" --;"..package.path
package.path=package.path..";../Samples/classification/lua/?.lua" --;"..package.path
package.path=package.path..";../Resource/classification/lua/?.lua" --;"..package.path
require("module")
package.path=package.path..";../Samples/scripts/RigidBodyWin/subRoutines/?.lua" --;"..package.path
require("Timeline")


model_files={}
model_files.default={}
-- Default PD-servo settings
model_files.default.k_p_PD=500 -- Nm/rad
model_files.default.k_d_PD=10 --  Nms/rad. 
model_files.default.k_p_slide=2000-- 
model_files.default.k_d_slide=50 --   
model_files.default.timestep=1/8000
--model_files.default.rendering_step=1/30
model_files.default.rendering_step=1/60 -- 0.5x real-time speed
model_files.default.frame_rate=120 -- mocap frame rate
model_files.default.initialHeight=0.01 -- meters
model_files.default.penaltyDepthMax={0.0005}
model_files.default.penaltyForceStiffness=30000
model_files.default.penaltyForceDamp=3000
model_files.default.start=0
model_files.default.k_scale_active_pd={ default={1,4,0}, hip={1,1,1}, knee={1,1,1}, elbow= {0.3,2,0}, shoulder={0.3,2,1}, collar={0.3,2,0}, ankle={1,2,1}, toes={0.3,0.3,0}, }
model_files.default.k_scale_passive_pd=deepCopyTable(model_files.default.k_scale_active_pd)

model_files.hyunwoo=deepCopyTable(model_files.default)
do
	local model=model_files.hyunwoo
	model.file_name= "../Resource/motion/locomotion_hyunwoo/hyunwoo_lowdof_T_boxfoot.wrl" 
	--model.mot_file= "../Resource/motion/locomotion_hyunwoo/hyunwoo_lowdof_T_wd2_all.dof"
	--model.mot_file= "../Resource/motion/locomotion_hyunwoo/hyunwoo_lowdof_T_MOB1_Run_F_Jump.dof" model.initialHeight=0.13 -- meters
	model.mot_file= nil model.initialHeight=1.3 -- meters
	model.k_p_PD=500 -- Nm/rad
	model.k_d_PD=5 --  Nms/rad. 
	--model.mot_file= "../Resource/motion/locomotion_hyunwoo/hyunwoo_lowdof_T_locomotion_hl.dof"
	model.frame_rate=30 -- mocap frame rate
	model.start=0
	model.timestep=1/240
	model.bones={
		chest='Chest',
		left_heel='LeftAnkle',
		left_ball='LeftAnkle',
		left_knee='LeftKnee',
		left_hip='LeftHip',
		right_heel='RightAnkle',
		right_ball='RightAnkle',
		right_knee='RightKnee',
		right_hip='RightHip',
		left_shoulder='LeftShoulder',
		left_elbow='LeftElbow',
		right_shoulder='RightShoulder',
		right_elbow='RightElbow',
		neck='Neck',
		hips='Hips',
	}
end

--model=model_files.gymnist
model=model_files.hyunwoo 
--model=model_files.bipV3
--model=model_files.lowerbody
--model=model_files.chain
--model=model_files.chain2
--model=model_files.car




EVR=LUAclass(EventReceiver)
function EVR:__init()
end
function EVR:onFrameChanged(win, iframe)
	if mLoader~=nill and this:findWidget("simulation"):checkButtonValue() then
		local niter=math.floor(model.rendering_step/model.timestep+0.5)
		mRagdoll:frameMove(niter)
	end
end
function ctor()
	mEventReceiver=EVR()
   --	this:create("Button", "Start", "Start")
   --	this:widget(0):buttonShortcut("FL_ALT+s")

   this:create("Check_Button", "simulation", "simulation", 0, 2,0)
   this:widget(0):checkButtonValue(1) -- 1 for imediate start
   this:widget(0):buttonShortcut("FL_ALT+s")
   
   this:create("Button", "single step", "single step", 2, 3,0)
   
   this:create("Check_Button", "draw skeleton", "draw skeleton", 0, 3,0)
   this:widget(0):checkButtonValue(0)

   this:create("Button", "debug LoaderToTree class", "debug LoaderToTree class")
   this:create("Button", "debug get/setlink", "debug get/setlink")
   this:create("Button", "debug get/setlink2", "debug get/setlink2")

   this:updateLayout()
   this:redraw()
   
   RE.viewpoint().vpos:assign(vector3(330.411743, 69.357635, 0.490963))
   RE.viewpoint().vat:assign(vector3(-0.554537, 108.757057, 0.477768))
   RE.viewpoint():update()
   RE.viewpoint():TurnRight(math.rad(viewRotate or 0))
   _start()
end

function dtor()
   -- remove objects that are owned by C++
   if mSkin~=nill then
      RE.remove(mSkin)
      mSkin=nil
   end
   if mSkin2~=nill then
      RE.remove(mSkin2)
      mSkin2=nil
   end
   if mTimeline then
	   mTimeline:dtor()
       mTimeline=nil
   end
   -- remove objects that are owned by LUA
   collectgarbage()
end

function _start()
	dtor()
	mTimeline=Timeline("Timeline", 1000000, 1/30)
	RE.motionPanel():motionWin():playFrom(0)
	print("start")
	mLoader=MainLib.VRMLloader(model.file_name)

	if model.cleanup then
		require('subRoutines/VRMLexporter')
		VRMLexporter.cleanupWRL(mLoader, model.file_name..'cleanup.wrl', mLoader:name())
		mLoader=MainLib.VRMLloader(model.file_name..'cleanup.wrl')
		--mLoader:setTotalMass(65)
	end
	mLoader:printHierarchy()
	if model.mot_file~=nill then
		local container=MotionDOFcontainer(mLoader.dofInfo, model.mot_file)
		mMotionDOF=container.mot
	end
	mFloor=MainLib.VRMLloader("../Resource/mesh/floor_y.wrl")

	drawSkeleton=this:findWidget("draw skeleton"):checkButtonValue()

	local simulator= Physics.DynamicsSimulator_TRL_LCP("gjk")  -- now works well.

	local simulatorParam={
		timestep=model.timestep,
		integrator=Physics.DynamicsSimulator.EULER,
		debugContactParam={10, 0, 0.01, 0, 0}, -- size, tx, ty, tz, tfront
	}
	mRagdoll= RagdollSim(mLoader, drawSkeleton, mMotionDOF, simulator, simulatorParam)
	mRagdoll.drawDebugInformation=true

	mSkin2=RE.createVRMLskin(mFloor, false)
	mSkin2:scale(100,100,100)
end

function onCallback(w, userData)
	if w:id()=="Start" then
		_start()
	elseif w:id()=="debug get/setlink" then
		-- just for testing and debugging.
		local theta=vectorn()
		local dtheta=vectorn()
		local sim=mRagdoll.simulator
		sim:getLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, theta);
		sim:getLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, dtheta);
		sim:setLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, theta);
		sim:setLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, dtheta);
		sim:initSimulation()
		local theta2=vectorn()
		local dtheta2=vectorn()
		sim:getLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, theta2);
		sim:getLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, dtheta2);
		local delta=dtheta-dtheta2
		assert(delta:maximum()<0.0001)
		assert(delta:minimum()>-0.0001)
		print('delta1:', delta)
		sim:setLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, theta2);
		sim:setLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, dtheta2);
		sim:getLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, theta);
		sim:getLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, dtheta);
		local delta=dtheta-dtheta2
		assert(delta:maximum()<0.0001)
		assert(delta:minimum()>-0.0001)
		print('test passed.')
	elseif w:id()=="debug LoaderToTree class" then
		-- just for testing and debugging.
		local theta=vectorn()
		local dtheta=vectorn()
		local theta2=vectorn()
		local dtheta2=vectorn()
		local sim=mRagdoll.simulator
		sim:getLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, theta);
		sim:getLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, dtheta);

		sim:setLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, theta);
		sim:setLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, dtheta);

		sim:getLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, theta2);
		sim:getLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, dtheta2);

		sim:setLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, theta2);
		sim:setLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, dtheta2);

		local e=MotionUtil.Effectors()
		local c=MotionUtil.Constraints()
		local tree=MotionUtil.LoaderToTree(mLoader, e,c, false,false)
		tree:setPoseDOF(mLoader.dofInfo, theta)
		print('hihi')
		print(theta)
		print(dtheta)
		tree:setVelocity(mLoader.dofInfo, dtheta)
		tree:Print()

		local testPassed=true
		for i=1, mLoader:numBone()-1 do
			local wldst=sim:getWorldState(0):globalFrame(i)
			local wldst2=tree:getLastNode(i)._global
			if (wldst.translation- wldst2.translation):length()>0.0001 then
				testPassed=false
				print('test1 failed')
				dbg.console()
			end
		end
		for i=1, mLoader:numBone()-1 do
			local wldst=sim:getWorldState(0):globalFrame(i)
			local wldst2=tree:getLastNode(i)._global
			local angVel=sim:getWorldAngVel(0, mLoader:VRMLbone(i))
			local linVel=sim:getWorldVelocity(0, mLoader:VRMLbone(i), vector3(0,0,0))
			local bangVel=tree:getLastNode(i):bodyAngVel():copy()
			local blinVel=tree:getLastNode(i):bodyLinVel():copy()
			angVel:rotate(wldst2.rotation:inverse())
			linVel:rotate(wldst2.rotation:inverse())
			print(i)
			print(wldst)
			print(wldst2)
			print(angVel, bangVel)
			print(linVel, blinVel)
			if( angVel- bangVel):length()>0.0001 then
				testPassed=false
				dbg.console()
			end
			if( linVel- blinVel):length()>0.0001 then
				testPassed=false
				dbg.console()
			end
		end
		local dtheta2=vectorn(dtheta:size())
		tree:getVelocity(mLoader.dofInfo, dtheta2)
		local delta=dtheta-dtheta2
		assert(delta:maximum()<0.0001)
		assert(delta:minimum()>-0.0001)
		if testPassed then
			print("Test passed")
		end
	elseif w:id()=='debug get/setlink2' then
		testGetSetLink()
	end
end

function frameMove(fElapsedTime)
end

RagdollSim=LUAclass ()

function RagdollSim:__init(loader, drawSkeleton, motdof, simulator, simulatorParam)
	if drawSkeleton==nil then drawSkeleton = true end
	self.simulator=simulator
	self.skin=RE.createVRMLskin(loader, drawSkeleton)
	self.skin:setThickness(0.03)
	self.skin:scale(100,100,100)

	self.simulator:registerCharacter(loader)
	local floor=mFloor or VRMLloader("../Resource/mesh/floor_y.wrl")
	self.simulator:registerCharacter(floor)

	--self.simulator:setParam_Epsilon_Kappa(0.1, 0.05) -- for soft contact
	self.simulator:setParam_R_B_MA(0, 0, 0.05); -- for large timestep (ARMATURE) , R adjusts rebounds from the ground. B is not implemented yet.
	local param=vectorn ()
	param:setValues(0.5,0.5, model.penaltyForceStiffness, model.penaltyForceDamp)
	for i=1,loader:numBone()-1 do
		local bone_i=loader:VRMLbone(i)
		self.simulator:registerCollisionCheckPair(loader:name(),bone_i.NameId, floor:name(), floor:bone(1):name(), param)
	end

	self.simulator:init(simulatorParam.timestep, simulatorParam.integrator)

	self.simulator:setSimulatorParam("debugContact", simulatorParam.debugContactParam) 
	self.simulator:setSimulatorParam("contactForceVis", {0.001,0.001,0.001})
	self.simulator:setSimulatorParam("penaltyDepthMax", {0.0005})
	-- adjust initial positions

	self.motionDOF=motdof
	self.simulationParam=simulatorParam

	self.controlforce=vectorn(loader.dofInfo:numDOF())
	if self.motionDOF then
		for i=0, self.motionDOF:numFrames()-1 do
			self.motionDOF:row(i):set(1,self.motionDOF:row(i):get(1)+(model.initialHeight or 0) )

			if false then
				-- test get, setlinkdata
				self.simulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE,self.motionDOF:row(i))
				local out=vectorn()
				self.simulator:getLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE,out);
				local errorv=out-self.motionDOF:row(i)
				if errorv:maximum()>0.00001 or errorv:minimum()<-0.00001 then
					print(i, errorv)
					dbg.console()
				end
			end
		end
	else
		motdofc=MotionDOFcontainer(loader.dofInfo)
		motdofc:resize(10)
		for i=0, 9 do
			motdofc.mot(i):setAllValue(0)
			motdofc.mot(i):set(3, 1) -- assuming quaternion (free root joint)
		end
		self.motionDOF=motdofc.mot
		for i=0, self.motionDOF:numFrames()-1 do
			self.motionDOF:row(i):set(1,self.motionDOF:row(i):get(1)+(model.initialHeight or 0) )
		end

	end
	if model.initialAngle then
		local delta= quater(model.initialAngle, vector3(1,0,0) )
		for i=0, self.motionDOF:numFrames()-1 do
			self.motionDOF:row(i):setQuater(3, self.motionDOF:row(i):toQuater(3)*delta)
		end
	end

	self.DMotionDOF=calcDerivative(self.motionDOF)
	self.DDMotionDOF=self.DMotionDOF:derivative(120)

	require("RigidBodyWin/subRoutines/PDservo")
	k_p=model.k_p_PD	-- Nm/rad
	k_d=model.k_d_PD --  Nms/rad. worked in range [0, 1]
	self.pdservo=PDservo(loader.dofInfo)
	self.pdservo:initPDservo(model.start, self.motionDOF:numFrames(),
	self.motionDOF, self.DMotionDOF)

	if self.motionDOF then
		model.start=math.min(model.start, self.motionDOF:numFrames()-1)
		initialState=vectorn()
		initialState:assign(self.motionDOF:row(model.start))

		print("initialState=",initialState)
		self.simulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, initialState)

		if self.DMotionDOF then
			local initialVel=self.DMotionDOF:row(model.start):copy()
			self.simulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, initialVel)
		end
		self.simulator:initSimulation()
	else
		local wldst=self.simulator:getWorldState(0)
		wldst:localFrame(loader:getBoneByTreeIndex(1)).rotation:identity()
		wldst:localFrame(loader:getBoneByTreeIndex(1)).translation:radd(vector3(0, model.initialHeight, 0))
		wldst:forwardKinematics()
		self.simulator:setWorldState(0)
	end
	--	debug.debug()
	self.simulator.setPose(self.skin,self.simulator,0)

	self.skin:setMaterial("lightgrey_transparent")

	--self.simulator.setGVector(vector3(0,0,9.8))
	self.simulator:setGVector(vector3(0,9.8,0))
	self.simulator:initSimulation()
	self.loader=loader
	self.floor=floor -- have to be a member to prevent garbage collection
end
function RagdollSim:setFrame(iframe, initialHeight)
	self.pdservo.startFrame=iframe
	self.pdservo:rewindTargetMotion(self.simulator)
	local initialState=vectorn()
	initialState:assign(self.motionDOF:row(iframe))
	-- set global position
	initialState:set(1,initialState:get(1)+initialHeight)

	self.simulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, initialState)

	if self.DMotionDOF then
		local initialVel=self.DMotionDOF:row(iframe):copy()
		self.simulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, initialVel)
	end
	self.simulator:initSimulation()
end
function RagdollSim:__finalize()
	-- remove objects that are owned by C++
	if self.skin~=nill then
		RE.remove(self.skin)
		self.skin=nil
	end
	self.simulator=nil

end
function RagdollSim:frameMove(niter)
	--assert(math.floor(niter)==niter)
	--		debug.debug()
	temp=vectorn()
	self.controlforce:zero()
	self.simulator:getLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, temp)

	for iter=1,niter do
		do
			local maxForce=9.8*80
			if not self.pdservo:generateTorque(self.simulator, maxForce) then
				self.pdservo:rewindTargetMotion(self.simulator)
				self.pdservo:generateTorque(self.simulator, maxForce)
			end

			local controlforce=self.pdservo.controlforce

			if self.pdservo.stepSimul then
				self.pdservo:stepSimul(self.simulator)
			else
				--testGetSetLink()
				self.simulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_TORQUE, controlforce)
				if self.drawDebugInformation then
					self.simulator:drawDebugInformation()
				end
				self.simulator:stepSimulation()
			end
			self.controlforce:radd(controlforce)
		end
		self.simulator.setPose(self.skin, self.simulator, 0)				
	end

	self.controlforce:rdiv(niter)

	--[[
	local forces=vector3N()
	local torques=vector3N()
	for i=1, self.loader:numBone()-1 do
		local bone=self.loader:VRMLbone(i)

		local force=self.simulator:getCOMbasedContactForce(0, i)
		if force:F():length()>0 then
			local pos=self.simulator:getWorldState(0):globalFrame(i)*bone:localCOM()
			forces:pushBack(pos*100)
			forces:pushBack(pos*100+force:F())
			torques:pushBack(pos*100)
			torques:pushBack(pos*100+force:M())
		end
	end
	dbg.namedDraw('Traj', forces:matView(), 'contactforces', 'solidred', 0, 'LineList' )
	dbg.namedDraw('Traj', torques:matView(), 'contacttorques', 'solidblue', 0, 'LineList' )
	]]

end

function testGetSetLink()
	local self=mRagdoll
	
	local v1=vectorn()
	local v2=vectorn()
	self.simulator:getLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, v1)
	self.simulator:getLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, v2)
	local zero=vectorn(v1:size())
	zero:setAllValue(0)
	zero:set(3,1)
	self.simulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, zero)
	self.simulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, zero)
	self.simulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, v1)
	--v2:range(3,v2:size()):setAllValue(0)
	self.simulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, v2)
end
function calcDerivative(motionDOF)
	assert(motionDOF~=nil)
	local dmotionDOF=matrixn()

	dmotionDOF:setSize(motionDOF:numFrames(), motionDOF:numDOF())

	for i=1, motionDOF:rows()-2 do
		calcDerivative_row(i,dmotionDOF, motionDOF)
	end

	-- fill in empty rows
	dmotionDOF:row(0):assign(dmotionDOF:row(1))
	dmotionDOF:row(dmotionDOF:rows()-1):assign(dmotionDOF:row(dmotionDOF:rows()-2))
	return dmotionDOF
end

function calcDerivative_row(i, dmotionDOF, motionDOF)
   local dmotionDOF_i=dmotionDOF:row(i);
   dmotionDOF_i:sub(motionDOF:row(i+1), motionDOF:row(i)) -- forward difference

   local frameRate=120
   if model then frameRate=model.frame_rate end
   dmotionDOF_i:rmult(frameRate)
   
   assert(motionDOF.dofInfo:numSphericalJoint()==1) 
   -- otherwise following code is incorrect
   local T=MotionDOF.rootTransformation(motionDOF:row(i))
   local V=T:twist( MotionDOF.rootTransformation(motionDOF:row(i+1)), 1/frameRate)
   dmotionDOF_i:setVec3(0, V.v)
   dmotionDOF_i:setVec3(4, V.w)
end

