-- PDServo class
--class 'PDservo'
PDservo=LUAclass()


function PDservo:setCoef(dofInfo,kp, kd, tgtVelScale, k_scale)
	assert(dofInfo:numSphericalJoint()==1)
	-- spherical joint가 있는 경우 PDservo_spherical 사용할 것!
   kp:setSize(dofInfo:numDOF())
   kp:setAllValue(k_p)
   kd:setSize(dofInfo:numDOF())
   kd:setAllValue(k_d)
   tgtVelScale:setSize(dofInfo:numDOF())
   tgtVelScale:setAllValue(k_d)
   
   -- exclude root joint
   kp:range(0,7):setAllValue(0)
   kd:range(0,7):setAllValue(0)
   tgtVelScale:range(0,7):setAllValue(0)
  
   print("initPDservo:"..dofInfo:skeleton():bone(1):name())
   for i=2,dofInfo:skeleton():numBone()-1 do
      local bone=dofInfo:skeleton():bone(i)
      local vbone=bone:treeIndex()
      local nJoint=dofInfo:numDOF(vbone)
--      print("initPDservo:"..bone:name())
      for j=0, nJoint-1 do
	 
	 local dofIndex=dofInfo:DOFindex(vbone,j)
	 
	 kp:set(dofIndex, k_p*k_scale.default[1])
	 kd:set(dofIndex, k_d*k_scale.default[2])
	 tgtVelScale:set(dofIndex, k_scale.default[3])

	 if bone:voca()==MotionLoader.LEFTANKLE or bone:voca()==MotionLoader.RIGHTANKLE then
	    if k_scale.ankle~=nil then
	       kp:set(dofIndex, k_p*k_scale.ankle[1])
	       kd:set(dofIndex, k_d*k_scale.ankle[2])
	       tgtVelScale:set(dofIndex, k_scale.ankle[3])
	    end
	 elseif bone:voca()==MotionLoader.LEFTCOLLAR or bone:voca()==MotionLoader.RIGHTCOLLAR then
	    if k_scale.collar~=nil then
	       kp:set(dofIndex, k_p*k_scale.collar[1])
	       kd:set(dofIndex, k_d*k_scale.collar[2])
	       tgtVelScale:set(dofIndex, k_scale.collar[3])
	    end
	 elseif bone:voca()==MotionLoader.LEFTSHOULDER or bone:voca()==MotionLoader.RIGHTSHOULDER then
	    if k_scale.shoulder~=nil then
	       kp:set(dofIndex, k_p*k_scale.shoulder[1])
	       kd:set(dofIndex, k_d*k_scale.shoulder[2])
	       tgtVelScale:set(dofIndex, k_scale.shoulder[3])
	    end
	 elseif bone:voca()==MotionLoader.LEFTELBOW or bone:voca()==MotionLoader.RIGHTELBOW then
	    if k_scale.elbow~=nil then
	       kp:set(dofIndex, k_p*k_scale.elbow[1])
	       kd:set(dofIndex, k_d*k_scale.elbow[2])
	       tgtVelScale:set(dofIndex, k_scale.elbow[3])
	    end
	 elseif bone:voca()==MotionLoader.LEFTKNEE or bone:voca()==MotionLoader.RIGHTKNEE then
	    if k_scale.knee~=nil then
	       kp:set(dofIndex, k_p*k_scale.knee[1])
	       kd:set(dofIndex, k_d*k_scale.knee[2])
	       tgtVelScale:set(dofIndex, k_scale.knee[3])
	    end
	 elseif bone:voca()==MotionLoader.LEFTHIP or bone:voca()==MotionLoader.RIGHTHIP then
	    if k_scale.hip~=nil then
	       kp:set(dofIndex, k_p*k_scale.hip[1])
	       kd:set(dofIndex, k_d*k_scale.hip[2])
	       tgtVelScale:set(dofIndex, k_scale.hip[3])
	    end
	 elseif bone:voca()==MotionLoader.CHEST then
	    if k_scale.chest~=nil then
	       kp:set(dofIndex, k_p*k_scale.chest[1])
	       kd:set(dofIndex, k_d*k_scale.chest[2])
	       tgtVelScale:set(dofIndex, k_scale.chest[3])
	    end
	 elseif bone:voca()==MotionLoader.CHEST2 then
	    if k_scale.chest2~=nil then
	       kp:set(dofIndex, k_p*k_scale.chest2[1])
	       kd:set(dofIndex, k_d*k_scale.chest2[2])
	       tgtVelScale:set(dofIndex, k_scale.chest2[3])
	    end
	 elseif bone:voca()==MotionLoader.NECK then
	    if k_scale.neck~=nil then
	       kp:set(dofIndex, k_p*k_scale.neck[1])
	       kd:set(dofIndex, k_d*k_scale.neck[2])
	       tgtVelScale:set(dofIndex, k_scale.neck[3])
	    end
	 elseif bone:voca()==MotionLoader.HEAD then
	    if k_scale.head~=nil then
	       kp:set(dofIndex, k_p*k_scale.head[1])
	       kd:set(dofIndex, k_d*k_scale.head[2])
	       tgtVelScale:set(dofIndex, k_scale.head[3])
	    end
	 end
	 if str_include(bone:name(), "toes") then
	    local dofIndex=dofInfo:DOFindex(vbone,j)
	    if k_scale.toes~=nil then
	       kp:set(dofIndex, k_p*k_scale.toes[1])
	       kd:set(dofIndex, k_d*k_scale.toes[2])
	       tgtVelScale:set(dofIndex, k_scale.toes[3])
	    end

	 end

	 if dofInfo:DOFtype(vbone, j)==MotionDOFinfo.SLIDE then
	    local dofIndex=dofInfo:DOFindex(vbone,j)
	    kp:set(dofIndex, model.k_p_slide)
	    kd:set(dofIndex, model.k_d_slide)
	    tgtVelScale:set(dofIndex, 0)
	 end
      end
   end
end

function PDservo:updateCoef()
   local dofInfo=self.dofInfo
   local k_scale_active=model.k_scale_active_pd

   self:setCoef(dofInfo,self.kp_active, self.kd_active, self.tgtVelScale_active, k_scale_active)

   local k_scale_passive=model.k_scale_passive_pd

   self:setCoef(dofInfo,self.kp_passive, self.kd_passive, self.tgtVelScale_passive, k_scale_passive)
end

function PDservo:__init(dofInfo)
   self.theta=vectorn()
   self.dtheta=vectorn()
   self.theta_d=vectorn() -- desired q
   self.dtheta_d=vectorn() -- desired dq
   self.controlforce=vectorn()
   self.kp=vectorn()
   self.kd=vectorn()
   self.tgtVelScale=vectorn()
   self.kp_active=vectorn()
   self.kd_active=vectorn()
   self.tgtVelScale_active=vectorn()
   self.kp_passive=vectorn()
   self.kd_passive=vectorn()
   self.tgtVelScale_passive=vectorn()
   self.mask_slide=vectorn()
   self.muscleActiveness=0.3
   self.kp_weight=1.0 -- use kp_active(1) or kp_passive(0)
   self.kd_weight=1.0 -- use kd_active(1) or kd_passive(0)
   self.mask_slide:setSize(dofInfo:numDOF())
   self.mask_slide:setAllValue(0)
   self.dofInfo=dofInfo
   self:updateCoef()
   print ("kp=",self.kp)
   print ("kd=",self.kd)

   local clampTorque=800
   local clampForce=8000

   if model.clampTorque~=nil then
      clampTorque=model.clampTorque
   end

   if model.clampForce~=nil then
      clampForce=model.clampForce
   end

   self.clampMax=vectorn(dofInfo:numDOF())
   for i=2,dofInfo:skeleton():numBone()-1 do
      local bone=dofInfo:skeleton():bone(i)
      local vbone=bone:treeIndex()
      local nJoint=dofInfo:numDOF(vbone)
      for j=0, nJoint-1 do
	 local dofIndex=dofInfo:DOFindex(vbone,j)
	 if dofInfo:DOFtype(vbone, j)==MotionDOFinfo.SLIDE then
	    local dofIndex=dofInfo:DOFindex(vbone,j)
	    self.mask_slide:set(dofIndex, 1)
	    self.clampMax:set(dofIndex, clampForce)
	 else
	    self.clampMax:set(dofIndex, clampTorque)
	 end
      end
   end   

   self.clampMin=self.clampMax*-1
   return o
end

function PDservo:initPDservo(startf, endf,motionDOF, dmotionDOF)
   self.startFrame=startf
   self.endFrame=endf
   self.currFrame=startf
   self.deltaTime=0
   self.motionDOF=motionDOF
   self.dmotionDOF=dmotionDOF
end

-- generate FBtorque
function PDservo:generateTorque(simulator)
   
	self.currFrame=(simulator:currentTime()+self.deltaTime)*model.frame_rate+self.startFrame
	--print(self.currFrame) -- extremely slow.
	if self.currFrame>self.endFrame-1 then
		simulator:getLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, self.theta)
		simulator:getLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, self.dtheta)
		return false
	end

	self:_generateTorque(simulator, self.currFrame)
	return true
end

--gTimer=util.Timer()
function PDservo:stepSimul(simulator, drawDebugInformation)
	simulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_TORQUE, self.controlforce)
	if drawDebugInformation then
		simulator:drawDebugInformation()
	end
	--gTimer:start()
	simulator:stepSimulation()
	--print(gTimer:stop2())
end

function PDservo:_generateTorque(simulator, frame)
   
   simulator:getLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, self.theta)
   simulator:getLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, self.dtheta)
   
   --[[ continuous sampling ]]--
--   print("theta",self.theta)
   
   -- desired (target) pose
   self.motionDOF:samplePose(frame, self.theta_d)
   self.dmotionDOF:sampleRow(frame, self.dtheta_d)
   
--   self.dtheta_d:setAllValue(0)

   self.dtheta_d:rmult(self.muscleActiveness) -- this corresponds to muscle activeness

   self.controlforce:setSize(self.motionDOF:numDOF())
   
--   self.controlforce:assign(self.kp*(self.theta_d-self.theta)+
--			 self.kd*(self.dtheta_d-self.dtheta))

   local delta=self.theta_d-self.theta
   MainLib.VRMLloader.projectAngles(delta) -- [-pi, pi]

   self.kp:interpolate(self.kp_weight, self.kp_passive, self.kp_active)
   self.kd:interpolate(self.kd_weight, self.kd_passive, self.kd_active)
   self.tgtVelScale:interpolate(self.kd_weight, self.tgtVelScale_passive, self.tgtVelScale_active)

   self.controlforce:assign(self.kp*delta +
			    self.kd*(self.dtheta_d*self.tgtVelScale-self.dtheta))

   self.controlforce:clamp(self.clampMin, self.clampMax)

end

function PDservo:rewindTargetMotion(simulator)
   self.deltaTime=-1*simulator:currentTime()
end

-- following functions are for feedback error learning.

-- segment={ mot, dmot, offsetTable}
function updateFFtorque(segment, debugInfo)
   
   
   dtor_loop()
   
   
   
   mPDservo=PDservo:new(segment.mot.dofInfo)
   mPDservo:initPDservo(0, segment.mot:numFrames(), segment.mot, segment.dmot)
   
   
   if segment.offsetTable:rows()==0 then
      segment.offsetTable:resize(segment.mot:numFrames(), segment.mot.dofInfo:numDOF())
      segment.offsetTable:setAllValue(0)
   end
   
   local tempOffsetTable=matrixn()
   local tempOffsetCount=vectorn()
   
   
   tempOffsetCount:setSize(segment.mot:numFrames())
   tempOffsetCount:setAllValue(0)
   tempOffsetTable:setSize(segment.mot:numFrames(), segment.mot.dofInfo:numDOF())
   tempOffsetTable:setAllValue(0)
	
   --mFloor=VRMLloader("../Resource/mesh/floor.wrl")
   mFloor=MainLib.VRMLloader("../Resource/mesh/floor_y.wrl")
	
   drawSkeleton=false
	
   assert(mSkel~=nil, "skel nil")
	
	
	mSkin=RE.createVRMLskin(mSkel, drawSkeleton)
	mSkin:setThickness(0.03)
	
	if showDesiredPose then
		mSkin2=RE.createVRMLskin(mSkel, drawSkeleton)
		mSkin2:setThickness(0.03)
		mSkin2:scale(100,100,100)
		mSkin2:setVisible(false)
	end
	
	mSkinFloor=RE.createVRMLskin(mFloor, false)

	mSkin:scale(100,100,100)
	mSkinFloor:scale(100,100,100)
	mSimulator=createSimulator(simulator)
	mSimulator:registerCharacter(mSkel)
	mSimulator:registerCharacter(mFloor)

	registerContactPair(model, mSkel, mFloor, mSimulator)
	mSimulator:init(timestep, integrator)

	mSimulator:setSimulatorParam("debugContact", debugContactParam) 
	mSimulator:setSimulatorParam("penaltyDepthMax", model.penaltyDepthMax )
	
	initialState=vectorn()
	initialState:assign(mPDservo.motionDOF:row(0))
	
	-- set global position
	initialState:set(1,initialState:get(1)+initialHeight)	
	
	mSimulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, initialState)
	mSimulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, mPDservo.dmotionDOF:row(0))
	mSkin:setPoseDOF(initialState)
	
	showDesiredPose(initialState)
	--mSimulator.setGVector(vector3(0,0,9.8))
	mSimulator:setGVector(vector3(0,9.8,0))
	mSimulator:initSimulation()
	
	local iter=0
	local offset=vectorn()
	local error=0
	
	while true do
		assert(mSkel~=nill)
	
		mSimulator:stepSimulation()
		
	
		if mPDservo:generateTorque(mSimulator) ==false then
		
			for ii=0, tempOffsetTable:rows()-1 do
				tempOffsetTable:row(ii):rdiv(tempOffsetCount:get(ii))	-- average FBtorque per bin.
			end
			
			if debugInfo~=nil then
				mm=tempOffsetTable:minimum()
				MM=tempOffsetTable:maximum()
				tempOffsetTable:draw(debugInfo.."before.bmp")
			end
			
			-- about half second kernel.
			math.gaussFilter(mPDservo.motionDOF.dofInfo:frameRate()*0.5, tempOffsetTable)
			
			if debugInfo~=nil then
				tempOffsetTable:draw(debugInfo.."after.bmp", mm, MM)
			end
			
			-- update Feed-forward torques
			segment.offsetTable:assign(segment.offsetTable*0.9+tempOffsetTable*0.1)
			break
		end

		local controlforce=vectorn()
		local FFforce=vectorn()
		-- FF torque
		segment.offsetTable:sampleRow(mPDservo.currFrame, FFforce)
		-- FB torque + FF torque
		controlforce:add(mPDservo.controlforce, FFforce)
		
		mSimulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_TORQUE, controlforce)
		--mSimulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_TORQUE, mPDservo.controlforce)

		local binIndex=math.floor(mPDservo.currFrame+0.5)
		
		-- gather Feed-back torques.
		
		mPDservo.controlforce:rsub(mPDservo.controlforce*mPDservo.mask_slide)	-- discard slide joints.
		tempOffsetTable:row(binIndex):radd(mPDservo.controlforce)
		tempOffsetCount:set(binIndex, tempOffsetCount:get(binIndex)+1)
		error=error+mPDservo.controlforce:length()
		
		if math.mod(iter, niter)==0 then		
			-- rendering

			--subtitlebar:setCaption("pos="..pos:__tostring().." dist="..dist)
			
			if g_signal~=nill then
				local controlforce=mPDservo.controlforce
				g_signal:resize(g_signal:rows()+1, 4)
				g_signal:row(g_signal:rows()-1):assign({controlforce:get(7), controlforce:get(8), controlforce:get(9), controlforce:get(10)})
			end
			
			
			showDesiredPose(mPDservo.theta_d)
			
			
			mSimulator:drawDebugInformation()
			mObjectList:clear()

			if drawCOM then
				local com=mSimulator:calculateCOM(0)
				local comDir=mSimulator:calculateCOMvel(0)
				com:assign(com*100)
				comDir:assign(comDir*100)
				comEntity=mObjectList:registerEntity("COM", "sphere1010.mesh")
				comEntity:setScale(20,20,20)
				comEntity:setPosition(com.x, com.y, com.z)
				lines=vector3N()
				lines:setSize(2)
				lines:at(0):assign(com)
				lines:at(1):assign(com+comDir*10)
				--mObjectList:registerObject("Comdir", "LineList", "solidred", lines,0)
			end
			if drawZMP then
				local zmp=mSimulator:calculateZMP(0)
				zmp:assign(zmp*100)
				print("ZMP=",zmp.x, zmp.y, zmp.z)
				zmpEntity=mObjectList:registerEntity("ZMP", "sphere1010.mesh")
				zmpEntity:setScale(10,10,10)
				zmpEntity:setPosition(zmp.x, zmp.y, zmp.z)
			end
							
			mSkin:setPose(mSimulator,0)

	
			RE.renderOneFrame(true)
			

		end
		
		iter=iter+1
	end
	
	
	
	return error
end

function dtor_loop()
	-- remove objects that are owned by C++
	if mSkin~=nill then
		RE.remove(mSkin)
		mSkin=nil
	end
	-- remove objects that are owned by C++
	if mSkin2~=nill then
		RE.remove(mSkin2)
		mSkin2=nil
	end
	if mSkinFloor~=nill then
		RE.remove(mSkinFloor)
		mSkinFloor=nil
	end
	mObjectList:clear()
	-- remove objects that are owned by LUA
	mPDservo=nil
	collectgarbage()
end



--class 'PoseMaintainer'
PoseMaintainer=LUAclass()

function PoseMaintainer:__init(skeletonIndex)
	assert(skeletonIndex)
	self.theta=vectorn()
	self.dtheta=vectorn()
	self.theta_d=vectorn() -- desired q
	self.dtheta_d=vectorn() -- desired dq
	self.controlforce=vectorn()

	self.skeletonIndex=skeletonIndex or 0
	-- followings are temporaries
	self.kp=vectorn()
	self.kd=vectorn()
end

function PoseMaintainer:init(skel, simulator, k_p, k_d, k_p_slide, k_d_slide)
	local si=self.skeletonIndex
	simulator:getLinkData(si, Physics.DynamicsSimulator.JOINT_VALUE, self.theta_d)
	simulator:getLinkData(si, Physics.DynamicsSimulator.JOINT_VELOCITY, self.dtheta_d)

	local dofInfo=skel.dofInfo
	self.kp:setSize(dofInfo:numDOF())
	self.kp:setAllValue(k_p)
	self.kd:setSize(dofInfo:numDOF())
	self.kd:setAllValue(k_d)
	
	if skel:VRMLbone(1):HRPjointType(0)==MainLib.VRMLTransform.FREE then
		-- exclude free root joint
		self.kp:range(0,7):setAllValue(0)
		self.kd:range(0,7):setAllValue(0)
	end
	
	if k_p_slide==nil then
	   k_p_slide=k_p*10
	end

	if k_d_slide==nil then
	   k_d_slide=k_d*500
	end

	for i=1,skel:numBone()-1 do
		vbone=skel:VRMLbone(i)
		nJoint=vbone:numHRPjoints()
		for j=0, nJoint-1 do
			if vbone:HRPjointType(j)==MainLib.VRMLTransform.SLIDE then
				self.kp:set(vbone:DOFindex(j), k_p_slide)
				self.kd:set(vbone:DOFindex(j), k_d_slide)
			end
		end
	end
end

function PoseMaintainer:generateTorque(simulator)
	local si=self.skeletonIndex
	simulator:getLinkData(si, Physics.DynamicsSimulator.JOINT_VALUE, self.theta)
	simulator:getLinkData(si, Physics.DynamicsSimulator.JOINT_VELOCITY, self.dtheta)
	self.controlforce:setSize(simulator:skeleton(si).dofInfo:numDOF())
	self.controlforce:setAllValue(0)

	self.controlforce:assign(self.kp*(self.theta_d-self.theta)+ self.kd*(self.dtheta_d-self.dtheta))

end
function PoseMaintainer:resetParam(kp, kd, theta_d)
	self.kp:setAllValue(kp)
	self.kd:setAllValue(kd)
	self.theta_d:assign(theta_d)
end

SimplePDservo=LUAclass()


function SimplePDservo:setCoef(dofInfo, kp, kd, k_p, k_d, k_scale)
	kp:setSize(dofInfo:numDOF())
	kp:setAllValue(k_p)
	kd:setSize(dofInfo:numDOF())
	kd:setAllValue(k_d)

	-- exclude root joint
	kp:range(0,7):setAllValue(0)
	kd:range(0,7):setAllValue(0)

	for i=2,dofInfo:skeleton():numBone()-1 do
		local bone=dofInfo:skeleton():bone(i)
		local vbone=bone:treeIndex()
		local nJoint=dofInfo:numDOF(vbone)
		--      print("initPDservo:"..bone:name())
		for j=0, nJoint-1 do

			local dofIndex=dofInfo:DOFindex(vbone,j)

			kp:set(dofIndex, k_p*k_scale.default[1])
			kd:set(dofIndex, k_d*k_scale.default[2])

			if bone:voca()==MotionLoader.LEFTANKLE or bone:voca()==MotionLoader.RIGHTANKLE then
				if k_scale.ankle~=nil then
					kp:set(dofIndex, k_p*k_scale.ankle[1])
					kd:set(dofIndex, k_d*k_scale.ankle[2])
				end
			elseif bone:voca()==MotionLoader.LEFTCOLLAR or bone:voca()==MotionLoader.RIGHTCOLLAR then
				if k_scale.collar~=nil then
					kp:set(dofIndex, k_p*k_scale.collar[1])
					kd:set(dofIndex, k_d*k_scale.collar[2])
				end
			elseif bone:voca()==MotionLoader.LEFTSHOULDER or bone:voca()==MotionLoader.RIGHTSHOULDER then
				if k_scale.shoulder~=nil then
					kp:set(dofIndex, k_p*k_scale.shoulder[1])
					kd:set(dofIndex, k_d*k_scale.shoulder[2])
				end
			elseif bone:voca()==MotionLoader.LEFTELBOW or bone:voca()==MotionLoader.RIGHTELBOW then
				if k_scale.elbow~=nil then
					kp:set(dofIndex, k_p*k_scale.elbow[1])
					kd:set(dofIndex, k_d*k_scale.elbow[2])
				end
			elseif bone:voca()==MotionLoader.LEFTKNEE or bone:voca()==MotionLoader.RIGHTKNEE then
				if k_scale.knee~=nil then
					kp:set(dofIndex, k_p*k_scale.knee[1])
					kd:set(dofIndex, k_d*k_scale.knee[2])
				end
			elseif bone:voca()==MotionLoader.LEFTHIP or bone:voca()==MotionLoader.RIGHTHIP then
				if k_scale.hip~=nil then
					kp:set(dofIndex, k_p*k_scale.hip[1])
					kd:set(dofIndex, k_d*k_scale.hip[2])
				end
			elseif bone:voca()==MotionLoader.CHEST then
				if k_scale.chest~=nil then
					kp:set(dofIndex, k_p*k_scale.chest[1])
					kd:set(dofIndex, k_d*k_scale.chest[2])
				end
			elseif bone:voca()==MotionLoader.CHEST2 then
				if k_scale.chest2~=nil then
					kp:set(dofIndex, k_p*k_scale.chest2[1])
					kd:set(dofIndex, k_d*k_scale.chest2[2])
				end
			elseif bone:voca()==MotionLoader.NECK then
				if k_scale.neck~=nil then
					kp:set(dofIndex, k_p*k_scale.neck[1])
					kd:set(dofIndex, k_d*k_scale.neck[2])
				end
			elseif bone:voca()==MotionLoader.HEAD then
				if k_scale.head~=nil then
					kp:set(dofIndex, k_p*k_scale.head[1])
					kd:set(dofIndex, k_d*k_scale.head[2])
				end
			end
		end
	end
end

function SimplePDservo:__init(dofInfo, k_p, k_d, k_scale)
	self.controlforce=vectorn()
	self.kp=vectorn()
	self.kd=vectorn()
	self.mask_slide=vectorn()
	self.mask_slide:setSize(dofInfo:numDOF())
	self.mask_slide:setAllValue(0)
	self.dofInfo=dofInfo
	self:setCoef(dofInfo, self.kp, self.kd, k_p, k_d, k_scale)
	self.controlforce:setSize(dofInfo:numDOF())

	local clampTorque=800
	local clampForce=8000

	self.clampMax=vectorn(dofInfo:numDOF())
	for i=2,dofInfo:skeleton():numBone()-1 do
		local bone=dofInfo:skeleton():bone(i)
		local vbone=bone:treeIndex()
		local nJoint=dofInfo:numDOF(vbone)
		for j=0, nJoint-1 do
			local dofIndex=dofInfo:DOFindex(vbone,j)
			if dofInfo:DOFtype(vbone, j)==MotionDOFinfo.SLIDE then
				local dofIndex=dofInfo:DOFindex(vbone,j)
				self.mask_slide:set(dofIndex, 1)
				self.clampMax:set(dofIndex, clampForce)
			else
				self.clampMax:set(dofIndex, clampTorque)
			end
		end
	end   

	self.clampMin=self.clampMax*-1
end

function SimplePDservo:_generateTorque(simulator, theta, dtheta, theta_d, dtheta_d)
   local delta=theta_d-theta
   MainLib.VRMLloader.projectAngles(delta) -- [-pi, pi]

   self.controlforce:assign(self.kp*delta + self.kd*(dtheta_d-dtheta))
   self.controlforce:clamp(self.clampMin, self.clampMax)
end

GPDservo=LUAclass()

function GPDservo:__init(loader, model)
	self.integrator= MotionUtil.LoaderToTree(loader, false, false)
	self.loader=loader
	local dofInfo=loader.dofInfo
	self.theta=vectorn()
	self.dtheta=vectorn()
	self.theta_d=vectorn() -- desired q
	self.dtheta_d=vectorn() -- desired dq
	self.controlforce=vectorn()
	self.kp=vectorn()
	self.kd=vectorn()
	self.mask_slide=vectorn()
	self.mask_slide:setSize(dofInfo:numBone())
	self.mask_slide:setAllValue(0)
	self.dofInfo=dofInfo
	self:setCoef(dofInfo, model.k_p_PD, model.k_d_PD, model.k_scale_active_pd)
	self.jointSpacePDservo=SimplePDservo(dofInfo, model.k_p_PD*0.25, model.k_d_PD, model.k_scale_active_pd)

	local clampTorque=800
	local clampForce=8000

	if model.clampTorque~=nil then
		clampTorque=model.clampTorque
	end

	if model.clampForce~=nil then
		clampForce=model.clampForce
	end

	self.clampMax=vectorn(dofInfo:numBone()*3)
	self.clampMax:setVec3(0,vector3(0,0,0)) -- unused index
	for i=1,dofInfo:skeleton():numBone()-1 do
		local bone=dofInfo:skeleton():bone(i)
		local vbone=bone:treeIndex()
		self.clampMax:setVec3(vbone*3, vector3(clampTorque, clampTorque, clampTorque))
	end   

	self.clampMin=self.clampMax*-1
end
function GPDservo:setCoef(dofInfo,_k_p, _k_d, k_scale)
	local k_p=_k_p 
	local k_d=_k_d 
	local kp=self.kp
	local kd=self.kd
	kp:setSize(dofInfo:numDOF())
	kp:setAllValue(k_p)
	kd:setSize(dofInfo:numDOF())
	kd:setAllValue(k_d)

	-- exclude root joint
	-- -> root 정보도 hip torque계산시 필요함.
	--kp:range(0,7):setAllValue(0)
	--kd:range(0,7):setAllValue(0)

	print("initPDservo:"..dofInfo:skeleton():bone(1):name())
	for i=2,dofInfo:skeleton():numBone()-1 do
		local bone=dofInfo:skeleton():bone(i)
		local vbone=bone:treeIndex()

		if k_scale then
			kp:set(vbone, k_p*k_scale.default[1])
			kd:set(vbone, k_d*k_scale.default[2])
			if bone:voca()==MotionLoader.LEFTANKLE or bone:voca()==MotionLoader.RIGHTANKLE then
				if k_scale.ankle~=nil then
					kp:set(vbone, k_p*k_scale.ankle[1])
					kd:set(vbone, k_d*k_scale.ankle[2])
				end
			elseif bone:voca()==MotionLoader.LEFTCOLLAR or bone:voca()==MotionLoader.RIGHTCOLLAR then
				if k_scale.collar~=nil then
					kp:set(vbone, k_p*k_scale.collar[1])
					kd:set(vbone, k_d*k_scale.collar[2])
				end
			elseif bone:voca()==MotionLoader.LEFTSHOULDER or bone:voca()==MotionLoader.RIGHTSHOULDER then
				if k_scale.shoulder~=nil then
					kp:set(vbone, k_p*k_scale.shoulder[1])
					kd:set(vbone, k_d*k_scale.shoulder[2])
				end
			elseif bone:voca()==MotionLoader.LEFTELBOW or bone:voca()==MotionLoader.RIGHTELBOW then
				if k_scale.elbow~=nil then
					kp:set(vbone, k_p*k_scale.elbow[1])
					kd:set(vbone, k_d*k_scale.elbow[2])
				end
			elseif bone:voca()==MotionLoader.LEFTKNEE or bone:voca()==MotionLoader.RIGHTKNEE then
				if k_scale.knee~=nil then
					kp:set(vbone, k_p*k_scale.knee[1])
					kd:set(vbone, k_d*k_scale.knee[2])
				end
			elseif bone:voca()==MotionLoader.LEFTHIP or bone:voca()==MotionLoader.RIGHTHIP then
				if k_scale.hip~=nil then
					kp:set(vbone, k_p*k_scale.hip[1])
					kd:set(vbone, k_d*k_scale.hip[2])
				end
			elseif bone:voca()==MotionLoader.CHEST then
				if k_scale.chest~=nil then
					kp:set(vbone, k_p*k_scale.chest[1])
					kd:set(vbone, k_d*k_scale.chest[2])
				end
			elseif bone:voca()==MotionLoader.CHEST2 then
				if k_scale.chest2~=nil then
					kp:set(vbone, k_p*k_scale.chest2[1])
					kd:set(vbone, k_d*k_scale.chest2[2])
				end
			elseif bone:voca()==MotionLoader.NECK then
				if k_scale.neck~=nil then
					kp:set(vbone, k_p*k_scale.neck[1])
					kd:set(vbone, k_d*k_scale.neck[2])
				end
			elseif bone:voca()==MotionLoader.HEAD then
				if k_scale.head~=nil then
					kp:set(vbone, k_p*k_scale.head[1])
					kd:set(vbone, k_d*k_scale.head[2])
				end
			end
		else
			kp:set(vbone, k_p)
			kd:set(vbone, k_d)
		end
	end
end
function GPDservo:computeHipTorques(simulator, torque, swingHipIndex, stanceHipIndex,   rootStrength, stanceHipToSwingHipRatio)
	
	--compute the total torques that should be applied to the root and swing hip, keeping in mind that
	--the desired orientations are expressed in the character frame

	-- qRootDW is the desired orientation in world coordinates

	if (rootStrength < 0) then
		rootStrength = 0;
	end
	if (rootStrength > 1) then
		rootStrength = 1;
	end

	local function getTorque(treeindex)
		return torque:toVector3(treeindex*3)
	end

	--so this is the net torque that the root wants to see, in world coordinates
	local rootTorque = getTorque(1)

	--we need to compute the total torque that needs to be applied at the hips so that the final net torque on the root is rootTorque
	local rootMakeupTorque=vector3(0,0,0);
	local loader=self.loader
	local root=self.loader:bone(1)
	-- chest, lhip, rhip
	for i=1, self.loader:numBone()-1 do
		if (loader:bone(i):parent()==root) then
			rootMakeupTorque :rsub(torque:toVector3(i*3))
		end
	end
	rootMakeupTorque :rsub(rootTorque);

	--TODO//add to the root makeup torque the predictive torque as well (only consider the effect of the torque in the lateral plane).
	--Vector3d rootPredictiveTorque(0, 0, rootPredictiveTorqueScale*9.8*d.x);
	--rootMakeupTorque += characterFrame.rotate(rootPredictiveTorque);

	--assume the stance foot is in contact...
	local stanceHipTorque = getTorque(stanceHipIndex);
	local swingHipTorque= getTorque(swingHipIndex)

	--now, based on the ratio of the forces the feet exert on the ground, and the ratio of the forces between the two feet, we will compute the forces that need to be applied
	--to the two hips, to make up the root makeup torque - which means the final torque the root sees is rootTorque!
	stanceHipTorque :radd( rootMakeupTorque * stanceHipToSwingHipRatio * rootStrength);
	swingHipTorque :radd( rootMakeupTorque * (1-stanceHipToSwingHipRatio) * rootStrength);


	local stanceHipDamping=0
	if( stanceHipDamping > 0 ) then 
		local stanceHipMaxVelocity=50
		local rootAngVel=simulator:getWorldAngVel(0, self.loader:bone(1))
		local stanceHipAngVel=simulator:getWorldAngVel(0, self.loader:bone(stanceHipIndex))

		local wRely = rootAngVel.y - stanceHipAngVel.y;
		local function SGN(v)
			if v>0 then
				return 1.0
			else
				return -1.0
			end
		end

		if (math.abs(wRely) > stanceHipMaxVelocity ) then wRely = stanceHipMaxVelocity * SGN(wRely) end;
		stanceHipTorque.y = stanceHipTorque.y- stanceHipDamping * wRely * wRely * SGN(wRely);
	end

	torque:setVec3(stanceHipIndex*3, stanceHipTorque);
	torque:setVec3(swingHipIndex*3, swingHipTorque);
end
function GPDservo:getWorldPDTorque(idx, qOri, ori_d, angvel, angvel_d)
	local torque = vector3(0,0,0)

	qOri:align(ori_d)

	local scale=1.0
	if true then
		-- PoseController::computePDTorque
		local qErr = qOri:inverse() *ori_d
		--local qErr = ori_d*qOri:inverse() 
		qErr:align(quater(1,0,0,0))

		local erroramount=math.abs(qErr:dotProduct(quater(1,0,0,0)))
		if erroramount<0.1 then
			--dbg.console()
			error('too much error! fallback to joint-space pd servo')
		end

		local vErr=qErr:rotationVector() 
		if isnan(vErr:length()) then
			print('nan error')
			dbg.console()
		end
		torque = self.kp(idx)*vErr-- p torque
		--qErr represents the rotation from the desired child frame to the actual child frame, which
		--means that the torque is now expressed in child coordinates. We need to express it in parent coordinates!
		torque:rotate(qOri)
	else
		--저 위 수식. 솔직히 이해는 안감. 아래처럼 구현하면 맞는 수식. 결과가 둘이 비슷하긴 한듯.
		--where v : body velocity (refer to murray formulation)
		--		- w_ab=invR_ab*dR_ab (eqn 2.48)
		local m1=matrix3()
		m1:setFromQuaternion(qOri)
		local m2=matrix3()
		m2:setFromQuaternion(ori_d)
		local dotM=m2-m1

		local invR=matrix3()
		invR:setFromQuaternion(qOri:inverse())
		
		local w_ab=invR*dotM
		local function unskew(v)
			return vector3(v._23*-1, v._13, v._12*-1)
		end
		torque=self.kp(idx)*(qOri*unskew(w_ab)) -- excluding root
		--torque=500*(qOri*unskew(w_ab)) -- including root
	end

	torque = torque + ( (angvel_d - angvel) * self.kd(idx)) --[[self.kd)]]
	--torque = torque+ ( (angvel_d - angvel) * 5) --[[self.kd)]]
	--torque = torque * strength???? 

	return torque
end

function GPDservo:generateTorque(simulator)
   
   self.currFrame=(simulator:currentTime()+self.deltaTime)*model.frame_rate+self.startFrame
   --print(self.currFrame) -- extremely slow.
   if self.currFrame>self.endFrame-1 then
      return false
   end
   
   self:sampleDesired(simulator,  self.currFrame)
   local ok=true
   local worldTorque
   if not self.errorOccurred then
	   ok, worldTorque=pcall( self.generateWorldTorque, self, simulator) 
   end
   --local worldTorque= self:generateWorldTorque( simulator)  local ok=true -- to debug inside the function

   local torque=self.controlforce
   torque:setSize(simulator:dof(0))
   torque:setAllValue(0)
   if not ok or self.errorOccurred then
	   if not ok then print(worldTorque) end
	   -- fall back to jointSpacePDservo
	   simulator:getLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, self.theta)
	   simulator:getLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, self.dtheta)
	   self.jointSpacePDservo:_generateTorque(simulator, self.theta, self.dtheta, self.theta_d, self.dtheta_d)
	   local cf=self.jointSpacePDservo.controlforce -- JOINT_TORQUE format (root joint: 7 numbers). 
	   torque:radd(cf:range(1, cf:size()))  -- U format (root joint :6 numbers)
	   self.errorOccurred=true
   elseif simulator.addTauextToLinks then
	   local out=vectorn()	
	   simulator:addTauextToLinks(0,worldTorque:range(6, worldTorque:size())*-1, out) 
   elseif false then
	   -- slow
	   local tau=matrixn(1,3)
	   for j=2, self.loader:numBone()-1 do
		   local J=matrixn()
		   local JP=matrixn()
		   tau:row(0):setVec3(0, worldTorque:toVector3(j*3))

		   simulator:calcJacobianAt(0, j, J, vector3(0,0,0))
		   simulator:calcJacobianAt(0, self.loader:bone(j):parent():treeIndex(), JP, vector3(0,0,0))
		   local cf_j=tau*J:sub(3,6)-tau*JP:sub(3,6)
		   torque:radd(cf_j:row(0))
	   end
	   torque:range(0,6):setAllValue(0)
	   simulator:setU(0, torque)
   elseif true then
	   -- fast (위의 코드에서 중복되는 연산 제거함)
	   local tau=matrixn(1,3)
	   local S=matrixn()
	   for j=2, self.loader:numBone()-1 do
		   tau:row(0):setVec3(0, worldTorque:toVector3(j*3))
		   local dqIndex=simulator:calcS(0, j, S)
		   local cf_j2=tau*S
		   torque:range(dqIndex, dqIndex+cf_j2:cols()):radd(cf_j2:row(0))
	   end
   end
   torque:range(0,6):setAllValue(0)

   return true
end
function GPDservo:stepSimul(simulator, drawDebugInformation)
	simulator:setU(0, self.controlforce)
	simulator:stepSimulation()
end
function GPDservo:sampleDesired(simulator, frame)
	--simulator:getLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, self.theta)
	--simulator:getLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, self.dtheta)

	-- desired (target) pose
	self.motionDOF:samplePose(frame, self.theta_d)
	self.dmotionDOF:sampleRow(frame, self.dtheta_d)
	self.controlforce:setSize(self.motionDOF:numDOF())
end
function GPDservo:generateWorldTorque(simulator)

	self.orientation=quater()
	self.angvel=vector3()
	
	local integrator =self.integrator

	local dofInfo=self.dofInfo
	-- compute desired state.
	integrator:setPoseDOF(dofInfo, self.theta_d)
	integrator:setVelocity(dofInfo, self.dtheta_d)
	
	local torque = vectorn()
	torque:resize(self.dofInfo:skeleton():numBone()*3)
	torque:setAllValue(0)

	for i=1, self.dofInfo:skeleton():numBone()-1 do
		self.orientation:assign(simulator:getWorldState(0):globalFrame(i).rotation)
		self.angvel:assign(simulator:getWorldAngVel(0, self.loader:bone(i)))

		local node=integrator:getLastNode(i)
		local orientation_d =node:globalFrame().rotation
		local linvel_d=vector3()
		local angvel_d=vector3()
		integrator:getLastNode(i):GetJointVel(linvel_d, angvel_d)

		local Wtorque=self:getWorldPDTorque(i, self.orientation, orientation_d, self.angvel, angvel_d)
		torque:setVec3(3*i,Wtorque)
	end
	
	-- control parameters!!!
	local rootStrength=1.0
	local swingHipIndex=self.loader:getTreeIndexByVoca(MotionLoader.LEFTHIP)
	local stanceHipIndex=self.loader:getTreeIndexByVoca(MotionLoader.RIGHTHIP)
	--local stanceHipToSwingHipRatio=0.5 -- both support
	local stanceHipToSwingHipRatio=0 -- L support
	--local stanceHipToSwingHipRatio=1 -- R support
	local useRootTorque=false
	if not useRootTorque then
		self:computeHipTorques(simulator, torque, swingHipIndex, stanceHipIndex,   rootStrength, stanceHipToSwingHipRatio)
		torque:setVec3(1, vector3(0,0,0))
	end

	torque:clamp(self.clampMin, self.clampMax)
	return torque
end
function GPDservo:initPDservo(startf, endf,motionDOF, dmotionDOF)
   self.startFrame=startf
   self.endFrame=endf
   self.currFrame=startf
   self.deltaTime=0
   self.motionDOF=motionDOF
   self.dmotionDOF=dmotionDOF
end
