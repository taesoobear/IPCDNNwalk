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
	local kp=PDservo.poseToQ(self, self.kp)
	local kd=PDservo.dposeToDQ(self, self.kd)
	simulator:setStablePDparam(self.skeletonIndex, kp, kd)
end

function PoseMaintainer:applyPDtorque(sim, theta_d, dtheta_d)
	self.theta_d:assign(theta_d)
	if dtheta_d then
		self.dtheta_d:assign(dtheta_d)
	else
		self.dtheta_d:zero()
	end
	self:generateTorque(sim)
	sim:setLinkData(self.skeletonIndex, Physics.DynamicsSimulator.JOINT_TORQUE, self.controlforce)
end

function PoseMaintainer:generateTorque(simulator)
	local si=self.skeletonIndex
	simulator:getLinkData(si, Physics.DynamicsSimulator.JOINT_VALUE, self.theta)
	simulator:getLinkData(si, Physics.DynamicsSimulator.JOINT_VELOCITY, self.dtheta)

	local tau=vectorn()
	simulator:calculateStablePDForces(self.skeletonIndex, self:poseToQ(self.theta_d), self:dposeToDQ(self.dtheta_d), tau)

	self.controlforce=self:DQtoDpose(tau)
end
function PoseMaintainer:resetParam(kp, kd, theta_d)
	self.kp:setAllValue(kp)
	self.kd:setAllValue(kd)
	self.theta_d:assign(theta_d)
end

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
function PDservo:__init(dofInfo)
	assert(dofInfo:numSphericalJoint()==1) -- otherwise, use PDservo_spherical instead.
	self.theta=vectorn()
	self.dtheta=vectorn()
	self.theta_d=vectorn() -- desired q
	self.dtheta_d=vectorn() -- desired dq
	self.controlforce=vectorn()
	self.kp=vectorn()
	self.kd=vectorn()
	self.tgtVelScale=vectorn()
	self.mask_slide=vectorn()
	self.muscleActiveness=0.3
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
	self.clampMax:setAllValue(clampTorque)
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
function PDservo:updateCoef()
	local dofInfo=self.dofInfo
	local k_scale_active=model.k_scale_active_pd

	self:setCoef(dofInfo,self.kp, self.kd, self.tgtVelScale, k_scale_active)
end

function PDservo:dposeToDQ(dpose)
	return dpose:slice(0,3)..dpose:slice(7,0)..dpose:slice(4,7)
end
function PDservo:DQtoDpose(dq)
	return dq:slice(0,3).. CT.vec(0)..dq:slice(-3,0)..dq:slice(3,-3)
end
function PDservo:poseToQ(pose)
	assert(pose:size()>7)
	return pose:slice(0,3)..pose:slice(7,0)..pose:slice(3,7)
end

PoseMaintainer.poseToQ=PDservo.poseToQ
PoseMaintainer.dposeToDQ=PDservo.dposeToDQ
PoseMaintainer.DQtoDpose=PDservo.DQtoDpose
function PDservo:initPDservo(startf, endf,motionDOF, dmotionDOF, simulator)
	self.startFrame=startf
	self.endFrame=endf
	self.currFrame=startf
	self.deltaTime=0
	self.motionDOF=motionDOF
	self.dmotionDOF=dmotionDOF

	local dofInfo=self.dofInfo
	assert(dofInfo:numSphericalJoint()==1)

	assert(self.kp:size()==dofInfo:numDOF())


	self.skeletonIndex=0


	local kp=PDservo.poseToQ(self, self.kp)
	local kd=PDservo.dposeToDQ(self, self.kd)
	simulator:setStablePDparam(self.skeletonIndex, kp, kd)
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

function PDservo:_generateTorque(simulator, frame, target_delta)
   
	simulator:getLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, self.theta)
	simulator:getLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, self.dtheta)

	--[[ continuous sampling ]]--
	--   print("theta",self.theta)

	-- desired (target) pose
	self.motionDOF:samplePose(frame, self.theta_d)
	if target_delta then self.theta_d:radd(target_delta) end
	self.dmotionDOF:sampleRow(frame, self.dtheta_d)

	--   self.dtheta_d:setAllValue(0)

	self.dtheta_d:rmult(self.muscleActiveness) -- this corresponds to muscle activeness

	self.controlforce:setSize(self.motionDOF:numDOF())

	--local delta=self.theta_d-self.theta
	--MainLib.VRMLloader.projectAngles(delta) -- [-pi, pi]
	--self.controlforce:assign(self.kp*delta + self.kd*(self.dtheta_d*self.tgtVelScale-self.dtheta))


	local tau=vectorn()
	simulator:calculateStablePDForces(self.skeletonIndex, self:poseToQ(self.theta_d),tau)

	self.tau=tau
	self.controlforce=self:DQtoDpose(tau)


	self.controlforce:clamp(self.clampMin, self.clampMax)
end

function PDservo:rewindTargetMotion(simulator)
	self.deltaTime=-1*simulator:currentTime()
end



PDservo_spherical=LUAclass()

-- returns motQ, motDQ which are compatible with loader_spherical
function PDservo_spherical.convertMotionState(loader_euler, loader_spherical, motionDOF_euler, frame_rate)

	local DMotionDOF_euler=motionDOF_euler:calcDerivative(frame_rate)

	-- important!!!
	-- convert loader, motionDOF, and its time-derivative to new formats.

	local nf=motionDOF_euler:numFrames()
	local motQ=matrixn(nf, loader_spherical.dofInfo:numDOF())
	local motDQ=matrixn(nf, loader_spherical.dofInfo:numActualDOF())

	local tree=MotionUtil.LoaderToTree(loader_euler, false,false)

	local euler_dofInfo=loader_euler.dofInfo
	local spherical_dofInfo=loader_spherical.dofInfo

	for i=0, nf-1 do
		tree:setPoseDOF(euler_dofInfo, motionDOF_euler:row(i))
		tree:setVelocity(euler_dofInfo, DMotionDOF_euler:row(i))

		tree:getSphericalState(spherical_dofInfo, motQ:row(i), motDQ:row(i))
	end
	return motQ, motDQ
end

function PDservo_spherical:__init(dofInfo, _model)
	self.theta=vectorn()
	self.dtheta=vectorn()
	self.theta_d=vectorn() -- desired q
	self.dtheta_d=vectorn() -- desired dq
	self.controlforce=vectorn()
	self.dofInfo=dofInfo
	self.skeletonIndex=0
	self.model=_model or model
end

function PDservo_spherical:initPDservo(startf, endf,motQ, motDQ, simulator, ichara)
	local csize=vector3()
	local dofInfo=simulator:skeleton(ichara).dofInfo
	local model=self.model
	csize.y=simulator:numSphericalJoints(ichara)
	csize.x=dofInfo:numDOF() -csize.y*4

	-- for the root joint and other 1-DOF joints
	self.kp=vectorn(csize.x+csize.y*3)
	self.kd=vectorn(csize.x+csize.y*3)
	self.kp:setAllValue(self.model.k_p_PD)
	self.kd:setAllValue(self.model.k_d_PD)
	
	-- exclude root translation
	self.kp:range(0,3):setAllValue(0)
	self.kp:range(0,3):setAllValue(0)


	local clampTorque=800
	local clampForce=8000

	if model.clampTorque~=nil then
		clampTorque=model.clampTorque
	end

	if model.clampForce~=nil then
		clampForce=model.clampForce
	end
	self.clampMax=vectorn(csize.x+csize.y*3)
	self.clampMax:setAllValue(clampTorque)
	self.clampMin=self.clampMax*-1

	self.startFrame=startf
	self.endFrame=endf
	self.currFrame=startf
	self.deltaTime=0

	local q=vectorn()
	local dq=vectorn()
	simulator:initSimulation()
	simulator:getSphericalState(ichara, q, dq)

	self.nonQsize=csize.x
	self.motions={motQ, motDQ}
	simulator:setStablePDparam(self.skeletonIndex, self.kp, self.kd)
end

-- generate FBtorque
function PDservo_spherical:generateTorque(simulator)

	self.currFrame=(simulator:currentTime()+self.deltaTime)*model.frame_rate+self.startFrame
	--print(self.currFrame) -- extremely slow.
	if self.currFrame>self.endFrame-1 then
		return false
	end

	self:_generateTorque(simulator, self.currFrame)
	return true
end

function PDservo_spherical:stepSimul(simulator, drawDebugInformation)
	simulator:setTau(0, self.controlforce)
	if drawDebugInformation then
		simulator:drawDebugInformation()
	end
	simulator:stepSimulation()
end

function PDservo_spherical:_generateTorque(simulator, frame)

	self.motions[1]:sampleRow(frame, self.theta_d)
	self.motions[2]:sampleRow(frame, self.dtheta_d)

	simulator:calculateStablePDForces(self.skeletonIndex, self.theta_d, self.controlforce)

	self.controlforce:clamp(self.clampMin, self.clampMax)
end

function PDservo_spherical:rewindTargetMotion(simulator)
	self.deltaTime=-1*simulator:currentTime()
end


PoseMaintainer_spherical=LUAclass()

function PoseMaintainer_spherical:__init(skeletonIndex)
	self.theta=vectorn()
	self.dtheta=vectorn()
	self.theta_d=vectorn() -- desired q
	self.dtheta_d=vectorn() -- desired dq
	self.controlforce=vectorn()

	self.skeletonIndex=skeletonIndex or 0


end

function PoseMaintainer_spherical:init(skel, simulator, k_p, k_d)
	local csize=vector3()
	local ichara=self.skeletonIndex
	local dofInfo=simulator:skeleton(ichara).dofInfo
	csize.y=simulator:numSphericalJoints(ichara)
	csize.x=dofInfo:numDOF() -csize.y*4

	-- for the root joint and other 1-DOF joints
	self.kp=vectorn(csize.x+csize.y*3)
	self.kd=vectorn(csize.x+csize.y*3)
	self.kp:setAllValue(k_p)
	self.kd:setAllValue(k_d)
	
	if dofInfo:hasTranslation(1) then
		-- exclude root translation
		self.kp:range(0,3):setAllValue(0)
		self.kd:range(0,3):setAllValue(0)
		self.freeRoot=true
	end

	print ("kp=",self.kp)
	print ("kd=",self.kd)

	self.startFrame=startf
	self.endFrame=endf
	self.currFrame=startf
	self.deltaTime=0

	simulator:initSimulation()
	local q=vectorn()
	local dq=vectorn()
	simulator:getSphericalState(self.skeletonIndex, q, dq)

	self.nonQsize=q:size()-csize.y*4
	assert(q:toQuater(self.nonQsize):length()>0.99)
	self.theta_d=q
	self.dtheta_d=dq
	self.dtheta_d:zero()

	simulator:setStablePDparam(self.skeletonIndex, self.kp, self.kd)
end

function PoseMaintainer_spherical:stepSimul(simulator, drawDebugInformation)
	simulator:setTau(self.skeletonIndex, self.controlforce)
	if drawDebugInformation then
		simulator:drawDebugInformation()
	end
	simulator:stepSimulation()
end

function PoseMaintainer_spherical:generateTorque(simulator)

	simulator:getSphericalState(self.skeletonIndex, self.theta, self.dtheta)
	simulator:calculateStablePDForces(self.skeletonIndex, self.theta_d, self.controlforce)

	-- self.controlforce:clamp(self.clampMin, self.clampMax)
end

