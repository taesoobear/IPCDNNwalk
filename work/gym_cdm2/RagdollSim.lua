
RagdollSim=LUAclass()
function RagdollSim:__init(loader, drawSkeleton, simulatorParam)
	if drawSkeleton==nil then drawSkeleton = true end

	self.loader=loader
	self.objectList=Ogre.ObjectList ()

	self.externalForce={remainingFrames=0,}

	self:prepareMotionForSim()

end
function RagdollSim:projectToGround(v)
	v.y=0
end
function RagdollSim:prepareMotionForSim()

	model:loadMotion(self)

	local option={
		bones=input.bones,
		debugDraw=false,
	}
	if model.removeAccumulatedErrorInRefCDMtrajectory then
		option.removeAccumulatedError=true
	end
	if model.option_refCDMtraj then
		table.mergeInPlace(option, model.option_refCDMtraj, true)
	end
	local info=model.motionData
	info.loader:setVoca(option.bones)
	--option.debugDraw=true

	-- generate a reference CDM trajectory
	info.CDMtraj=fc.CDMTraj(info.loader, info.mot, option)
	--self.CDMtraj:exportTraj("refCDMtraj_runjump.dat")

	info.loader_ref_fullbody=info.loader
	info.loader_ref=mLoader:copy()

	local loader=info.loader_ref_fullbody
	loader:setVoca(model.bones)
	if model.fixDOF_part2 then		
		model.fixDOF_part2(info.loader, info.mot)
	end

	fc.prepareMotionsForSim(info, initial_height, model)

	local drawSkeleton=false
	self.skin_ref=RE.createVRMLskin(loader, drawSkeleton)
	self.skin_ref:setThickness(0.03)
	self.skin_ref:scale(100,100,100)
	self.skin_ref:setTranslation(100,0,0)
	if debug_draw then
		self.skin_ref:setMaterial('lightgrey_transparent')
	end
	

	local sp={}
	sp.timestep=timestep_qp
	sp.floor=mFloor
	sp.drawDebugInformation=true
	sp.drawSkeleton=false


	local info=model.motionData
	info.CDMmot=MotionDOF(self.loader.dofInfo)
	info.CDMmot:resize(info.motionDOF_cdm:numFrames())
	info.CDMmot:matView():assign(info.motionDOF_cdm:matView():sub(0,0,0,7))

	if model.useApproxRotY then
		local nf=info.motionDOF_cdm:numFrames()
		info.rotYref=matrixn(nf,4)

		local rotY1=info.motionDOF_cdm:row(0):toQuater(3)
		local rotY2=info.motionDOF_cdm:row(nf-1):toQuater(3)
		for i=0, nf-1 do
			local q=quater()
			q:safeSlerp(rotY1, rotY2, sop.map(i, 0, nf-1, 0, 1))

			-- find qo such that   q==rot*qo

			info.rotYref:row(i):setQuater(0, info.motionDOF_cdm:row(i):toQuater(3):inverse()*q)
		end
	end

	info.CDM_DMotionDOF=info.CDMmot:calcDerivative(model.frame_rate)
	info.CDM_DDMotionDOF=info.CDM_DMotionDOF:derivative(model.frame_rate)

	self.CDM= QPsim(mLoader,info, sp)
	self.CDM.skin:setTranslation(CDMdrawOffset*100)
end

-- rpos : ilimb==1 , lpos : ilimb==2 or ilimb==0
function RagdollSim:getFilteredFootPos(gf, ilimb)
	if  ilimb==0 then
		ilimb=2
	end
	local legInfo=self.legInfo[ilimb]
	local pos=legInfo.contactPos
	local ori=legInfo.rotY
	--dbg.draw('Sphere', pos*100, 'filtered'..tostring(ilimb ), 'green')
	if self.legInfo[ilimb].isSwing then 
		--pos=gf*legInfo.contactFilter  -- filtering in a local frame
		pos=legInfo.contactFilter 
		ori=gf.rotation:rotationY()*legInfo.contactFilterQ
		return pos, ori, false
	end
	if model.usefilteredPosForSpportPhaseToo then
		pos=legInfo.contactFilter 
		ori=gf.rotation*legInfo.contactFilterQ
	end
	return pos, ori, true
end
function RagdollSim:getCon(ilimb)
	if  ilimb==0 then
		ilimb=2 -- L
	end
	local legInfo=self.legInfo[ilimb]
	--dbg.draw('Sphere', pos*100, 'filtered'..tostring(ilimb ), 'green')
	if self.legInfo[ilimb].isSwing then 
		return false
	end
	return true
end
--version 2
function RagdollSim:pushSimPose( pd_pose)
	self.replayBuffer:pushBack( pd_pose:range(0,7))
end

function RagdollSim:getCDMsimulator()
	return self.CDM.simulator
end
function RagdollSim:sampleSimPose(deltaFrame)
	-- deltaFrame 이 0일때 마지막 push된 포즈 리턴.
	local sf=deltaFrame*2+self.replayBuffer:size() -1
	if sf>self.replayBuffer:size()-1 then
		sf=self.replayBuffer:size()-1
	end

	return self.replayBuffer:sample(math.max(0, sf))
end

function RagdollSim:sampleTargetPose()
	local function transformTheta(tf, theta)
		MotionDOF.setRootTransformation(theta, tf*MotionDOF.rootTransformation(theta))
	end
	do
		-- sample targetpose and modify theta_d so that it is aligned with theta
		local qpservo=self.CDM.qpservo
		model:sampleTargetPoses(qpservo)

		local delta_frame=model:getComparisonDeltaFrame()
		local T=MotionDOF.rootTransformation(self:sampleSimPose(delta_frame))
		T.rotation:assign(model:getRefRotY(T.rotation, delta_frame))
		local T_ref=MotionDOF.rootTransformation(model:sampleRefPose(delta_frame))
		T_ref.rotation:assign(model:getRefRotY(T_ref.rotation, delta_frame))
		if debug_mode then
			print('T:', T, '\nT_ref', T_ref)
		end
		-- delta* T=T_ref
		local delta=T_ref*T:inverse()
		-- project2D
		delta.translation.y=0
		delta.rotation:assign(delta.rotation:rotationY())
		transformTheta(delta:inverse(), qpservo.theta_d)

		--dbg.namedDraw('Axes',MotionDOF.rootTransformation(qpservo.theta_d),'desiredRoot',100)
	end
	if self.stitchCDMq then
		self.stitchCDMq:update(self.CDM.qpservo.theta_d)
		self.stitchCDMdq:update(self.CDM.qpservo.dtheta_d)
	end

end

function RagdollSim:calcRefDif(pd_sim)
	assert(self.sim_prev and not self.ref_prev)
	local pd_pose=vectorn()

	pd_pose:assign(pd_sim:getLastSimulatedPose(0))

	-- root orientation difference (see local difference)
	local delta_frame=model.frame_rate*RL_step  -- (delta_frame=0.5)
	local prev_delta_frame=delta_frame+model:getComparisonDeltaFrame()
	local sample=self:sampleSimPose(prev_delta_frame) -- sample before updating replaybuffer for correct timing

	if debug_draw and model.iframe then
		RE.output2('sampleTime', 'mocap=',model.iframe+0.5, model.iframe+prev_delta_frame, 
		'sim=', model.iframe+0.5, model.iframe+prev_delta_frame)
	end

	-- update replayBuffer 
	self:pushSimPose( pd_pose)


	local sim_prev=self.sim_prev -- corresponds to model.iframe
	model:setRefTree(0) -- so that timingMod, earlyTouchDown, and so on doesn't mess-up reference velocity computation.
	local ref_prev=self:getJointStateFromRefTree()

	model:setRefTree(delta_frame) -- 시뮬레이션 한 RL_step진행했으니 iframe+0.5과 비교하는게 맞음
	local info=model.motionData
	local ref_pose=vectorn()
	info.loader_ref:getPoseDOF(ref_pose)

	local sim_rotY=model:getRefRotY(pd_pose, delta_frame) -- corresponds to model.iframe+0.5
	local ref_rotY=model:getRefRotY(ref_pose, delta_frame)

	local sim_curr=self:getJointStateFromSimulator(sim_rotY) -- corresponds to model.iframe+0.5
	local ref_curr=self:getJointStateFromRefTree()
	
	local poseDif=0;
	--
	local sampleRef=model:sampleRefPose(prev_delta_frame)

	local function DIFF_METRIC(x)
		return x
	end
	local function dist2D(v1, v2)
		local v1p=v1:copy()
		local v2p=v2:copy()
		v1p.y=0
		v2p.y=0
		return v1p:distance(v2p)
	end
	do
		local prevSimRootOri=model:getRefRotY(sample, prev_delta_frame)
		local prevRefRootOri=model:getRefRotY(sampleRef, prev_delta_frame)

		local deltaSim=prevSimRootOri:inverse()*pd_pose:toQuater(3)
		local deltaRef=prevRefRootOri:inverse()*ref_pose:toQuater(3)

		if model.useWorldCoMvel then
			poseDif=poseDif+ DIFF_METRIC((pd_pose:toQuater(3):inverse()* ref_pose:toQuater(3)):rotationAngle()*5)
		else
			poseDif=poseDif+DIFF_METRIC((deltaSim:inverse()*deltaRef):rotationAngle()*5)
		end


		--local dy=(deltaSimY:inverse()*deltaRefY):rotationAngle()
		--poseDif=poseDif+dy*dy -- y orientation is more important


		deltaSim=prevSimRootOri:inverse()*(pd_pose:toVector3(0)-sample:toVector3(0))
		deltaRef=prevRefRootOri:inverse()*(ref_pose:toVector3(0)-sampleRef:toVector3(0))

		poseDif=poseDif+DIFF_METRIC(deltaSim:distance(deltaRef))


		local EFdif=0
		local refCoordMocap=transf(prevRefRootOri, sampleRef:toVector3(0))
		refCoordMocap.translation.y=0
		local refCoordSim=transf(prevSimRootOri, sample:toVector3(0))
		self:projectToGround(refCoordSim.translation)

		if debug_draw then
			dbg.namedDraw('Axes', MotionDOF.rootTransformation(sample), 'pd_prev', 100)
			dbg.namedDraw('Axes', MotionDOF.rootTransformation(pd_pose), 'pd_curr', 100)
			dbg.namedDraw('Axes', refCoordSim, 'pd_refcoord', 100)

			local function moveT(T, v)
				return transf(T.rotation, T.translation+v)
			end
			dbg.namedDraw('Axes', moveT(MotionDOF.rootTransformation(sampleRef), vector3(1,0,0)), 'ref_prev', 100)
			dbg.namedDraw('Axes', moveT(MotionDOF.rootTransformation(ref_pose), vector3(1,0,0)), 'ref_curr', 100)
			dbg.namedDraw('Axes', moveT(refCoordMocap, vector3(1,0,0)), 'ref_refcoord', 100)
		end

		for ileg=1,2 do
			if not self.legInfo[ileg].isSwing then
				-- toe

				
				EFdif=EFdif+dist2D(refCoordMocap:toLocalPos(ref_curr.pos(ileg*2-1)),refCoordSim:toLocalPos(sim_curr.pos(ileg*2-1)))*3
				-- heel
				EFdif=EFdif+dist2D(refCoordMocap:toLocalPos(ref_curr.pos(ileg*2)),refCoordSim:toLocalPos(sim_curr.pos(ileg*2)))*3
				
				-- neck
				--EFdif=EFdif+refCoordMocap:toLocalPos(ref_curr.pos(5)):distance(refCoordSim:toLocalPos(sim_curr.pos(5)))
				-- com
				EFdif=EFdif+(refCoordMocap:toLocalPos(ref_curr.pos(0)):distance(refCoordSim:toLocalPos(sim_curr.pos(0))))*3

			
				-- toe-to-com (doesn't work very well)
				--EFdif=EFdif+refCoordMocap:toLocalDir(ref_curr.pos(ileg*2-1)-ref_curr.pos(0)):distance(refCoordSim:toLocalDir(sim_curr.pos(ileg*2-1)-sim_curr.pos(0)))*2
				-- heel-to_com (doesn't work very well)
				--EFdif=EFdif+refCoordMocap:toLocalDir(ref_curr.pos(ileg*2)-ref_curr.pos(0)):distance(refCoordSim:toLocalDir(sim_curr.pos(ileg*2)-sim_curr.pos(0)))*10
			else
				-- neck
				--EFdif=EFdif+(refCoordMocap:toLocalPos(ref_curr.pos(5)):distance(refCoordSim:toLocalPos(sim_curr.pos(5))))
				-- com
				EFdif=EFdif+(refCoordMocap:toLocalPos(ref_curr.pos(0)):distance(refCoordSim:toLocalPos(sim_curr.pos(0))))
				--EFdif=EFdif+0.4
			end

			if false and debug_draw and hasGUI then
				if not self.legInfo[ileg].isSwing then
					--dbg.draw('Sphere', ref_curr.pos(ileg*2-1)*100, 'ballr_orig'..(ileg*2-1), 'green')
					--dbg.draw('Sphere', ref_curr.pos(ileg*2)*100, 'ballr_orig'..(ileg*2), 'green')

					-- toe
					dbg.draw('Sphere', (refCoordSim*refCoordMocap:toLocalPos(ref_curr.pos(ileg*2-1)))*100, 'ballr'..(ileg*2-1), 'green')
					dbg.draw('Sphere', (refCoordSim*refCoordSim:toLocalPos(sim_curr.pos(ileg*2-1)))*100, 'balls'..(ileg*2-1),'white')

					-- heel
					dbg.draw('Sphere', (refCoordSim*refCoordMocap:toLocalPos(ref_curr.pos(ileg*2)))*100, 'ballr'..(ileg*2), 'green')
					dbg.draw('Sphere', (refCoordSim*refCoordSim:toLocalPos(sim_curr.pos(ileg*2)))*100, 'balls'..(ileg*2),'white')

					-- neck
					dbg.draw('Sphere', (refCoordSim*refCoordMocap:toLocalPos(ref_curr.pos(5)))*100, 'ballr5'..ileg, 'green')
					dbg.draw('Sphere', (refCoordSim*(refCoordSim:toLocalPos(sim_curr.pos(5))))*100, 'ballc5'..ileg,'white')
					dbg.draw('Sphere', (refCoordSim*refCoordMocap:toLocalPos(ref_curr.pos(0)))*100, 'ballr0'..ileg, 'green')
					dbg.draw('Sphere', (refCoordSim*(refCoordSim:toLocalPos(sim_curr.pos(0))))*100, 'ballc0'..ileg,'white')
				else
					--dbg.erase('Sphere',  'ballr_orig'..(ileg*2-1))
					--dbg.erase('Sphere',  'ballr_orig'..(ileg*2))

					dbg.erase('Sphere',  'ballr'..(ileg*2-1))
					dbg.erase('Sphere',  'balls'..(ileg*2-1))

					dbg.erase('Sphere',  'ballr'..(ileg*2))
					dbg.erase('Sphere',  'balls'..(ileg*2))

					dbg.erase('Sphere',  'ballr0'..ileg)
					dbg.erase('Sphere', 'ballc0'..ileg)
					dbg.erase('Sphere',  'ballr5'..ileg)
					dbg.erase('Sphere', 'ballc5'..ileg)
				end
			end
		end
		if not self.legInfo[1].isSwing and not self.legInfo[2].isSwing then
			-- toe-to-toe 
			EFdif=EFdif+dist2D(refCoordMocap:toLocalDir(ref_curr.pos(1*2-1)-ref_curr.pos(2*2-1)),refCoordSim:toLocalDir(sim_curr.pos(1*2-1)-sim_curr.pos(2*2-1)))*3
			-- heel-to-heel
			EFdif=EFdif+dist2D(refCoordMocap:toLocalDir(ref_curr.pos(1*2)-ref_curr.pos(2*2)),refCoordSim:toLocalDir(sim_curr.pos(1*2)-sim_curr.pos(2*2)))*3
		end

		poseDif=poseDif+EFdif*(model.EFdif_scale or 1.0/8.0)
	end
	--print(poseDif)

	local numBone=self.loader:numBone() -- == 2 (dummy root=0 and CDM=1)

	poseDif=poseDif*10

	local simulator=self:getCDMsimulator()
	local sim_state=simulator:getWorldState(0)
	local pd_COM=sim_state:globalFrame(1).translation
	local ref_COM=info.loader_ref:bone(1):getFrame().translation
	local COM_lvdif= 0
	local sim_rotY=model:getRefRotY(pd_pose)
	local ref_rotY=model:getRefRotY(ref_pose)
	local weight_COMLVy=model.weight_COMLVy or 1
	--for i=1, numBone-1 do -- buggy. sim_curr.pos(1) means RIGHTANKLE.
	do
		local i=0 -- means COM here.
		if model.useWorldCoMvel then
			COM_lvdif=COM_lvdif+DIFF_METRIC((sim_curr.pos(i)-sim_prev.pos(i)):distance( ref_curr.pos(i)-ref_prev.pos(i))/RL_step)
		else
			local v1=sim_rotY:inverse()*(sim_curr.pos(i)-sim_prev.pos(i))
			local v2=ref_rotY:inverse()*(ref_curr.pos(i)-ref_prev.pos(i))
			local dxz=dist2D(v1, v2)
			local dy=math.abs(v1.y-v2.y)*weight_COMLVy
			COM_lvdif=COM_lvdif+(math.sqrt(dxz*dxz+dy*dy))/RL_step
		end
		if debug_draw then
			RE.output2('com_lvdif', sim_rotY:inverse()*(sim_curr.pos(i)-sim_prev.pos(i))/RL_step,
			ref_rotY:inverse()*(ref_curr.pos(i)-ref_prev.pos(i))/RL_step)
			print('lvdif', (sim_curr.pos(i)-sim_prev.pos(i)):length(), (ref_curr.pos(i)-ref_prev.pos(i)):length())
		end
	end
	COM_lvdif=COM_lvdif/(numBone-1)


	self.sim_prev =sim_curr
	
	local step_stand_reward=10

	local weight_pose=model.weight_pose or 0.65*5*0.5
	local weight_endE=0.15*40
	local weight_COM=0.1*40
    local weight_COMLV=model.weight_COMLV or 0.1*100
	local res

    --local cost= (poseDif*weight_pose) +(COMDif*weight_COM) +(COM_lvdif*weight_COMLV)
    local cost= (poseDif*weight_pose) +(COM_lvdif*weight_COMLV)
    --print((poseDif*weight_pose),
     --     (COM_lvdif*weight_COMLV), step_reward)

    --RE.output2('diff', (poseDif*weight_pose), (COM_lvdif*weight_COMLV), step_reward)

	return cost, poseDif/10, COM_lvdif, ref_curr.pos(0):distance(ref_prev.pos(0))/RL_step
end


function dbg.initDelayQueue()
	if not dbg.g_delayinfo then
		dbg.g_delayinfo={
			{}, -- delay0
			{}, -- delay1
			{}, -- delay2
			{}, -- delay3
			{}, -- delay4
			{}, -- delay5
		}
	end
end

function dbg.delayedSetPoseDOF(skin, delay, theta)
	local info={skin, theta:copy()}
	if delay>5 then delay=5 end
	dbg.initDelayQueue()
	table.insert(dbg.g_delayinfo[delay+1], {'others', 'setPoseDOF', info})
end
function dbg.delayedDraw(typeid, delay, ...)
	local info=deepCopyTable({...})
	
	if delay>5 then delay=5 end
	if delay==0 then
		local otherarg={...}
		dbg.draw(typeid, unpack(otherarg))
		return
	end
	dbg.initDelayQueue()

	table.insert(dbg.g_delayinfo[delay+1], {'draw', typeid, info})

end
function dbg.delayedErase(typeid, delay, nameid)
	if delay>5 then delay=5 end
	dbg.initDelayQueue()

	table.insert(dbg.g_delayinfo[delay+1], {'erase', typeid, nameid})
end
function dbg.delayedDrawTick()
	for i,v in ipairs(dbg.g_delayinfo[1]) do
		if v[1]=='draw' then
			dbg.draw(v[2], unpack(v[3]))
		elseif v[1]=='others' then
			if v[2]=='setPoseDOF' then
				local skin, theta=unpack(v[3])
				skin:setPoseDOF(theta)
			end
		else
			dbg.erase(v[2], v[3])
		end
	end
	table.remove(dbg.g_delayinfo, 1)
	table.insert(dbg.g_delayinfo,{})
end
function printReward(reward)
	if hasGUI then
		RE.output2( 'reward', reward)
	else
		print('reward', reward)
	end
end
