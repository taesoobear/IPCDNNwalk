
local createModel=function(motion)
	-- all motion-dependent parameters are stored in a table called model.

	local motionInfos=require('gym_cdm2/module/motionInfo')
	-- set model from motionInfo
	local model=deepCopyTable(model_files.gymnist)
	local motionInfo=motionInfos[motion]

	model._motionInfo=motionInfo

	-- set defaults
	model.frame_rate=30
	model.muscleActiveness=1.0
	model.loopMotion=false
	numContactPoints=2
	-- kinematics
	model.maxLegLen=1.05
	model.touchDownHeight=0.9
	model.inertia=vector3(9.683,2.056,10.209)
	model.delayedVis=2
	model.touchDownHeight=0.7 
	model.minSwingPhase=12.0/30.0
	model.maxFootVel=3 -- meaning 0.03 meter per 1/60s  so this is pretty fast enough


	model.actionScaling=function (actions)
		return actions*0.2
	end

	model.QPparam={}
	QPparam=model.QPparam
	-- only for QP servo
	QPparam.tauObjWeight=0.0001
	QPparam.lambdaObjWeight=0.0001
	QPparam.ddqObjWeight=10000
	QPparam.ddqObjWeight_flight=10000
	QPparam.velMarginStrength=1
	QPparam.excludeRoot=false
	QPparam.useVelocityCone=false
	QPparam.collisionCheck=false

	QPparam.k_p_ID=340
	QPparam.k_d_ID=35



	randomInitial=false  -- do not change here.
	random_action_environment=false
	random_action_max=1

	-- set motion dependent settings
	do
		-- copy settings from motionInfo
		model.option_refCDMtraj= motionInfo.option_refCDMtraj
		model.fixDOF=motionInfo.fixDOF
		model.fixDOF_part2=motionInfo.fixDOF_part2
		--model.fixDOF_part3=motionInfo.fixDOF_part3
		model.file_name=motionInfo.skel_deepmimic or motionInfo.skel
		model.mot_file=motionInfo.fn
		local fn2=motionInfo.fn2
		if spec_id then
			if os.isFileExist('gym_cdm2/spec/refTraj_'..spec_id..'.dat') then
				fn2='gym_cdm2/spec/refTraj_'..spec_id..'.dat'
			end
		end
		model.reftraj_file=fn2..'.processed'
		model.touchDown=motionInfo.contact.touchDown
		model.touchOff=motionInfo.contact.touchOff
		model.start=motionInfo.start
		model.weight_COMLV=motionInfo.weight_COMLV
		model.weight_pose=motionInfo.weight_pose
		model.maxMimicCost=motionInfo.maxMimicCost
		model.doNotUseConWeight=motionInfo.doNotUseConWeight
		model.doNotUseConHeight=motionInfo.doNotUseConHeight
		model.doNotSolveIK=motionInfo.doNotSolveIK
		model.IKconfig=motionInfo.IKconfig
		model.conWeightMap=motionInfo.conWeightMap
		if motionInfo.delayedVis then
			model.delayedVis=motionInfo.delayedVis
		end
		if motionInfo.contactThr then
			model.contactThr=motionInfo.contactThr
		end
		if not motionInfo.noLooping then
			model.loopMotion=true
		end
		if motionInfo.option_contactFilter then
			local ocf=motionInfo.option_contactFilter 
			if ocf.refSwingAlpha then model.refSwingAlpha=ocf.refSwingAlpha end
			if ocf.maxFootVel then model.maxFootVel=ocf.maxFootVel end
			model.usefilteredPosForSpportPhaseToo =ocf.usefilteredPosForSpportPhaseToo  or ocf.useV2

			model.cf_option=ocf
			model.cf_useV2=ocf.useV2
			if model.cf_useV2 then
				model.usefilteredPosForSpportPhaseToo =true
				if not ocf.logQm then
					ocf.logQm=((ocf.logQ1+ocf.logQ2)*0.5)
				end
			end
		end

		model.unlimitedLeglen=motionInfo.unlimitedLeglen
		model.strideThr=motionInfo.strideThr
		model.maxContactLen=motionInfo.maxContactLen
		model.minSwingPhase=motionInfo.minSwingPhase

		assert(RL_step==1/30/motionInfo.downsample)
	end


	-- other settings.
	-- also fix incompatible settings in the motionInfo

	randomRestart=false
	model.getRefPrevFrame=function(iframe)
		--return math.max(iframe-model.frame_rate/2,0) -- compare with half-second before so that long-accumulated errors do not affect later reward
		return math.max(iframe-2,0) -- compare with half-second before so that long-accumulated errors do not affect later reward
	end
	function model:getComparisonDeltaFrame()
		return self.getRefPrevFrame(100)-100
	end
	function model:sampleRefPose(delta_frame)
		local prevFrame=math.max(self.iframe+delta_frame,0)
		return self.motionData.CDMmot:matView():sample(prevFrame)
	end
	function model:loadRefTraj(reftraj_file)
		reftraj_file =reftraj_file or self.reftraj_file
		if reftraj_file and os.isFileExist(reftraj_file) then
			self.refTraj=util.loadTable(reftraj_file)
			self.reftraj_file=reftraj_file
		end
		local refTraj=model.refTraj

		if refTraj then
			local refFrame=refTraj.CDMfeetToHumanFeetPositions:column(refTraj.CDMfeetToHumanFeetPositions:cols()-1)
			local phase=CT.linspace(refFrame(0), refFrame(refFrame:size()-1),refFrame:size())
			self.refFrameToPhase=math.PiecewiseLinearCurve(refFrame, phase)
			require('gym_cdm2/module/CDMphaseFilter')
			--if model.fixDOF_part3 then
			--	model:fixDOF_part3()
			--end
			return true
		end
		return false
	end

	if RE.ogreSceneManager() then
		if os.isFileExist(model.reftraj_file) then
			model.showHuman=true
		end
	end
	model.collectRefTraj=false -- manually set true only once after the controller tracks the reference motion well.
	if param and param.collectRefTraj then
		model.collectRefTraj=true
	end
	if model.collectRefTraj then
		model.showHuman=false
	end
	if debug_draw then
		--model.showHuman=false
		model.delayedVis=0
	end
	--model.contactThr=-0.15

	--reset_noise_scale=5e-2 -- for fair comparisons with original deepmimic
	reset_noise_scale=5e-1   -- for more robustness.
	reset_noise_scale_pos=5e-2
	--reset_noise_scale_velxz=2
	randomRestart=false
	randomRestartInfo= {prevStart=model.start, freq=1 , limitLength=5/RL_step }

	-- sim is an instancs of RagdollSim
	function RagdollSim_loadMotion_default(model, sim)

		local hasGUI=isMainloopInLua and RE.ogreSceneManager()
		local isTraining=not hasGUI

		if isTraining then
			if model.showHuman then
				print('did you forget to "make console"')
				assert(false)
			end
			model.touchDownHeight=0 -- disable early touchdown
			model.unlimitedLeglen=true -- disable late touchDown
		else
			--model.touchDownHeight=0 -- disable early touchdown
			--model.unlimitedLeglen=true -- disable late touchDown
		end

		model.bones=input.bones

		-- export reference CDM trajectory. needed only once after training.
		model.refSwingAlpha=0.5 -- use a smaller number for less smoothing of the swing foot trajectory.
		initial_height=0.0
		local info
		if model.file_name:sub(-4)=='.txt' then
			require('gym_deepmimic/module/DeepMimicLoader')
			info=loadDeepMimicMotionToHanyang(model.file_name, model.mot_file)
			if model.fixDOF then		
				model.fixDOF(info.loader, info.mot)
			end
		else
			info={}
			info.loader=MainLib.VRMLloader(model.file_name)

			local mLoader=info.loader
			mLoader:setVoca(model.bones)
			local container=MotionDOFcontainer(mLoader.dofInfo, model.mot_file)
			local motdof=container.mot
			if model.fixDOF then		
				model.fixDOF(mLoader, motdof)
			end

			if useThreeDOFankles then
				-- convert ankle joints to 3DOFs (YZX)
				local srcMotion=Motion( motdof)

				-- ankle L and R
				mLoader:setChannels(mLoader:getBoneByName(IKconfig[1][2]), '', 'YZX')
				mLoader:setChannels(mLoader:getBoneByName(IKconfig[2][2]), '', 'YZX')

				info.mot=MotionDOF(mLoader.dofInfo)
				info.mot:set(srcMotion)
			else
				info.mot=motdof
			end
		end

		model.motionData=info
		assert(model.motionData)
		print('motiondata set')

	end

	model.loadMotion=RagdollSim_loadMotion_default

	function model:getPhase_iframe() -- phase format 0: frame number
		local iframe=self.iframe
		local info=self.motionData
		local nf=info.motionDOF_original:numFrames()-1 -- actual length.

		if iframe>nf*2 then
			iframe=iframe-nf
			self.iframe=iframe
		end
		if iframe>=info.motionDOF_iframe:size()-1 then
			iframe=iframe-nf
			self.iframe=iframe
		end
		if (iframe>= info.motionDOF_iframe:size() ) then
			error()
		end
		local ifloor=math.floor(iframe)
		if info.motionDOF_iframe(ifloor+1)- info.motionDOF_iframe(ifloor) <0 then

			iframe=sop.map(iframe, ifloor, ifloor+1,
			info.motionDOF_iframe(ifloor),
			info.motionDOF_iframe(ifloor+1)+nf)

			if iframe>nf then
				iframe=iframe-nf
			end
			-- phase=w*p1+(1-w)*(p2+1)
		else
		end
		return info.motionDOF_iframe:sample(iframe)
	end

	function model:getPhase() -- phase format 1: normalized to be in range [0, 1]

		local info=self.motionData
		local nf=info.motionDOF_original:numFrames()-1 -- actual length.
		local iframe=self.iframe
		local phase=self:getPhase_iframe()/nf -- [0,1]

		if debug_mode then
			print(nf, iframe, info.motionDOF_iframe:sample(iframe), phase)
			RE.output2('phase', phase)
		end
		return phase
	end
	function model:setPhase(phase)
		local nf= self.motionData.motionDOF_original:numFrames()-1 -- actual length.
		-- discard accumulatedphase error
		self.iframe=nf*phase
	end

	-- uses frame unit.
	function model:remainingTimeToTouchDown(ileg, _iframe) 
		local iframe=self.iframe
		local info=self.motionData

		if _iframe then
			iframe=_iframe
		end
		if not info.rttTouchDown then
			local nf=info.con[ileg]:size()
			info.rttTouchDown={intvectorn(nf), intvectorn(nf)}
			for _ileg=1,2 do
				local rtt=info.rttTouchDown[_ileg]
				for i=0, nf-1 do
					if info.con[_ileg](i) then
						local rf=info.con[_ileg]:findPrev(i, false)
						rtt:set(i, rf-i+1)
					else
						rtt:set(i, info.touchDown[_ileg]:find(i)-i )
					end
				end
			end
		end
		return info.rttTouchDown[ileg](iframe)
	end
	-- returns swingPhase in [0,1]
	function model:swingPhase(ileg) 
		local iframe=self.iframe
		local info=self.motionData
		if not info.swingDuration then
			local nf=info.con[ileg]:size()
			info.swingDuration={vectorn(nf), vectorn(nf)}
			for _ileg=1,2 do
				local intv=info.con[_ileg]:flip():runLengthEncode()
				for iswing=0, intv:size()-1 do
					local s=intv:startI(iswing)
					local e=intv:endI(iswing)
					local duration=e-s-1
					if iswing==0 and s~=0 then
						s=0
					elseif iswing==intv:size()-1 then
						e=nf
					end
					info.swingDuration[_ileg]:range(s,e):setAllValue(duration)
					if iswing>0 then
						prev_e=intv:endI(iswing-1)

						local prev_dur=info.swingDuration[_ileg](prev_e-1)
						info.swingDuration[_ileg]:range(prev_e, s-1):linspace(prev_dur, duration)
					end
				end
			end
		end
		local rem=self:remainingTimeToTouchDown(ileg)
		local dur=info.swingDuration[ileg](iframe)
		return sop.clampMap(rem, 0, dur,  0, 1)
	end

	function model:createRefpose()
		local iframe=self.iframe
		local out=vectorn()
		local info=model.motionData
		if iframe>=info.motionDOF_fullbody:numFrames() then
			return nil
		end
		info.motionDOF_fullbody:sampleRow(iframe, out) -- iframe: continuous time. can be 5.5 or 7.25 for example
		return out
	end

	function model:defaultFeetOffset(ileg)
		local info=self.motionData
		if info.mocapFeetOffset then
			local iframe=self.iframe
			return info.mocapFeetOffset[ileg](iframe)
		else
			if ileg==1 then
				return vector3(-0.07,0,0) -- local
			else -- ileg==2
				return vector3(0.07,0,0) -- local
			end
		end
	end

	function model:getRefRotY(theta, delta_frame)
		local iframe=self.iframe +( delta_frame or 0)
		assert(iframe) -- forward direction can be pose-dependent so iframe is necessary.
		local rootq
		if theta.w then
			rootq=theta
		else
			rootq=theta:toQuater(3)
		end

		if model.useApproxRotY then
			local info=model.motionData
			local qo=vectorn()
			info.rotYref:sampleRow(iframe, qo)
			rootq=rootq*qo:toQuater(0):Normalize()
		end

		local rootRotY=rootq:rotationY()
		return rootRotY

	end

	function model:remainingTimeToTouchOff(ileg, _iframe)
		local iframe=self.iframe
		if _iframe then
			iframe=_iframe
		end

		local info=self.motionData
		if not info.rttTouchOff then
			local nf=info.con[ileg]:size()
			info.rttTouchOff={intvectorn(nf), intvectorn(nf)}
			for _ileg=1,2 do
				local rtt=info.rttTouchOff[_ileg]
				for i=0, nf-1 do
					if not info.con[_ileg](i) then
						local rf=info.con[_ileg]:findPrev(i, true)
						rtt:set(i, rf-i+1)
					else
						rtt:set(i, info.touchOff[_ileg]:find(i)-i )
					end
				end
			end
		end
		return info.rttTouchOff[ileg](iframe)
	end

	function model:sampleTargetPoses(qpservo)
		local iframe=self.iframe
		local info=self.motionData
		info.CDMmot:samplePose(iframe, qpservo.theta_d)
		info.CDM_DMotionDOF:sampleRow(iframe, qpservo.dtheta_d)
		info.CDM_DDMotionDOF:sampleRow(iframe, qpservo.ddtheta_d)
	end

	function model:setRefTree(delta_frame)
		local iframe=self.iframe+(delta_frame or 0)
		local info=self.motionData
		iframe=math.min(iframe, info.motionDOF_fullbody:numFrames()-1)

		info.loader_ref_fullbody:setPoseDOF(info.motionDOF_fullbody:sample(iframe))
		info.loader_ref:setPoseDOF(info.motionDOF_cdm:sample(iframe))
	end
	function model:getInitialFullbodyPose()
		return self.motionData.motionDOF_fullbody:row(self.start)
	end
	function model:getInitialState()

		local startFrame=self.start  

		if randomRestart then
			local iter_j=randomRestartInfo.iter_j or 0
			local count=math.mod(iter_j, randomRestartInfo.freq+1)
			if isMainloopInLua then
				randomRestartInfo.limitLength =nil
			end
			local limitLength=randomRestartInfo.limitLength or 30
			if isMainloopInLua or count==randomRestartInfo.freq then
				local nf=info.motionDOF_cdm:numFrames()

				if randomRestartFrames then
					local ii=randomRestartFrames:findFirstIndex(randomRestartInfo.prevStart)
					ii=math.fmod(ii+1, randomRestartFrames:size())
					startFrame=randomRestartFrames(ii)
				else
					startFrame=math.round(
					sop.clampMap(math.random(), 0, 1,
					self.start-limitLength*0.5, nf-1-limitLength*0.5)
					)   -- give more probabilty to select frame 0, and nf-1-limitLength

					if startFrame>nf-1-limitLength then
						startFrame=nf-1-limitLength
					end

					if startFrame<self.start then
						startFrame=self.start
					end
				end

				if startFrame~=randomRestartInfo.prevStart then
					randomRestartInfo.prevStart=startFrame
				end
			else
				startFrame=randomRestartInfo.prevStart
			end
			randomRestartInfo.iter_j=iter_j+1
		end

		local info=self.motionData
		local startRswing=not info.con[1](startFrame)
		local startLswing=not info.con[2](startFrame)

		if self.startFromSwing then
			-- backflip only.
			if info.touchDown[1](startFrame) then
				startRswing=true -- so that action can be used immediatly
			end
			if info.touchDown[2](startFrame) then
				startLswing=true -- so that action can be used immediatly
			end
		end
		self.iframe=startFrame
		local iframe=startFrame

		local initialState=info.motionDOF_cdm:row(startFrame):range(0,7):copy()
		local initialVel=info.DMotionDOF:row(iframe):range(0,7):copy()
		return initialState, initialVel, startRswing, startLswing
	end
	function model:advanceTime(mocapstep_per_controlstep)
		self.iframe=self.iframe+mocapstep_per_controlstep

		if  isMainloopInLua and self.loopMotion then
			local info=model.motionData
			local maxHistory=model.frame_rate/2
			if self.iframe+mocapstep_per_controlstep-maxHistory>info.motionDOF_original:numFrames()-1 then
				-- transition
				self.iframe=self.iframe- (info.motionDOF_original:numFrames()-1)
			end
		end

	end

	function model:sampleFullbodyPose(delta_frame)
		local ref_frame=self.iframe+(delta_frame or 0)
		return self:_sampleFullbodyPose(ref_frame)
	end

	function model:_sampleFullbodyPose(ref_frame)
		local local_iframe=ref_frame


		local refTraj=self.refTraj
		local cycle_len=(refTraj.CDMtoRoot:rows() -1)

		while local_iframe> cycle_len do
			local_iframe=local_iframe-cycle_len
		end
		local_iframe=math.max(local_iframe, 0)

		local phase=self.refFrameToPhase:sample(local_iframe) 
		RE.output2('phase', ref_frame, local_iframe, phase)

		local refPose=vectorn()
		local g_info=self.motionData
		--assert(phase<g_info.motionDOF_fullbody:rows()+0.001)
		g_info.motionDOF_fullbody:sampleRow(phase, refPose) -- phase: continuous time. can be 5.5 or 7.25 for example


		local CDMtoRoot=refTraj.CDMtoRoot:sample(phase):to_se3()
		local CDMfeetToHumanFeetPositions=refTraj.CDMfeetToHumanFeetPositions:sample(phase)
		local comdof_error=refTraj.comdof_error:sample(phase)
		local bodyVel=refTraj.bodyVel:sample(phase)


		local info=g_info
		local refDPose=info.CDMtraj.dmot:sample(phase)
		return refPose, CDMtoRoot,refDPose, CDMfeetToHumanFeetPositions, comdof_error, bodyVel, CT.vec(g_info.conWeight[1]:sample(phase), g_info.conWeight[2]:sample(phase))
	end

	function model:filterContact(li, rotY, mode)
		if self.cf_useV2 then
			return model:filterContact_V2(li, rotY, mode)
		else
			assert(false)
		end
		local model=self
		if mode=='sw' then
			local minSwingPhase=model.minSwingPhase or 0.4
			local w=math.clamp(li.swingPhase, 0, minSwingPhase )
			local ref_alpha=model.refSwingAlpha 
			local maxFootVel=model.maxFootVel

			local alpha=sop.mapSin(w, 0, minSwingPhase, 1, ref_alpha)
			local vel=li.contactPos-li.contactFilter --global vel
			local alpha2=sop.mapSin(w, 0, minSwingPhase, 0, 1)
			vel=math.smoothClampVec3(vel, maxFootVel*alpha2)
			li.contactFilter=li.contactFilter*alpha+(li.contactFilter+vel)*(1-alpha)
			li.contactFilterQ:safeSlerp(li.contactFilterQ, quater(1,0,0,0), 1.0-alpha)

			if debug_draw then
				dbg.draw('Sphere', (li.contactFilter)*100, 'swingFootFiltered'..tostring(li.isLeft),'green')
				dbg.draw('Sphere', (li.contactPos)*100, 'swingFootUnfiltered'..tostring(li.isLeft),'red')
			end

		elseif mode=='ssw' then
			-- start swing
			if model.usefilteredPosForSpportPhaseToo and li.contactFilter then
				return
			end

			--li.contactFilter=gf:inverse()*li.prevContact -- filtering in a local frame
			li.contactFilter=li.prevContact
			li.contactFilterQ=rotY:inverse()*li.rotY
			li.contactFilterQ:align(quater(1,0,0,0))
		elseif mode=='ssp' then
			-- start support
			if not li.contactFilter then
				li.contactFilter=li.contactPos:copy()
				li.contactFilterQ=rotY:inverse()*li.rotY
				li.contactFilterQ:align(quater(1,0,0,0))
			else
				local ref_alpha=model.refSwingAlpha 
				local maxFootVel=model.maxFootVel 
				local vel=li.contactPos-li.contactFilter --global vel
				vel=math.smoothClampVec3(vel, maxFootVel)
				li.contactFilter=li.contactFilter*ref_alpha+(li.contactFilter+vel)*(1-ref_alpha)
				li.contactFilterQ=rotY:inverse()*li.rotY
				li.contactFilterQ:align(quater(1,0,0,0))
			end
		end
	end

	function model:filterContact_V2(li, rotY, mode)
		local ocf=self.cf_option
		assert(ocf)
		local B=0.001
		local k=1
		local logQ1=ocf.logQ1 or 4
		local logQ2=ocf.logQ2 or 9

		local logQm=ocf.logQm or (logQ1+logQ2)*0.5
		local filterEndPhase=ocf.filterEndPhase or 0.8
		if mode=='ssw' or mode=='ssp' then
			if not li.lqrFilter then
				require('control/SDRE')
				local ball=MassParticle3D(30, B, k, math.pow(10, logQ1), 0, RL_step/2)

				ball:setState(li.contactPos)
				li.lqrFilter=ball
				li.lqrFilterQ=SDS(30, B, k, math.pow(10, logQ1), 0, RL_step/2)
				li.contactPreFilter=li.contactPos:copy()
				li.contactFilterQ=quater(1,0,0,0)
			end

			if mode=='ssw' then
				li.contactPreFilter=li.prevContact
				li.lqrFilter:setState(li.prevContact)

				li.contactFilterQ=rotY:inverse()*li.rotY
				li.contactFilterQ:align(quater(1,0,0,0))
				li.lqrFilterQ:setState(li.contactFilterQ:rotationAngleAboutAxis(vector3(0,1,0)), 0)
				li.lqrFilterQ:setDesiredState(0,0)
			end
		else
			local model=self
			local maxFootVel=model.maxFootVel 
			local ball=li.lqrFilter
			local w=li.swingPhase
			if not li.isSwing then
				w=filterEndPhase
			end
			local logQ
			if w<filterEndPhase*0.5 then
				logQ= sop.clampMap(w, 0, filterEndPhase *0.5, logQ1, logQm)
			else
				logQ= sop.clampMap(w, 0.5*filterEndPhase, filterEndPhase , logQm, logQ2)
			end
			local vel=li.contactPos-li.contactPreFilter --global vel
			local alpha2=sop.mapSin(w, 0, filterEndPhase, 0, 1)
			vel=math.smoothClampVec3(vel, maxFootVel*alpha2)
			li.contactPreFilter=li.contactPreFilter+vel


			--if g_terrain then
			--	li.contactPreFilter.y=math.max(li.contactPreFilter.y, g_terrain:getTerrainPos(li.contactPreFilter).y)
			--end

			local Q=math.pow(10, logQ)
			local k=1
			ball:updateCoef(B, k,Q)

			ball:setDesiredState(li.contactPreFilter)
			ball:singleStep()
			ball:singleStep()
			li.contactFilter=ball:getPosition()

			local filterQ=li.lqrFilterQ
			filterQ:updateCoef(B, k, Q)
			filterQ:singleStep()
			filterQ:singleStep()

			li.contactFilterQ:setRotation(filterQ.x(0,0), vector3(0,1,0))
			if debug_draw then
				RE.output2("swing"..tostring(li.isLeft), li.swingPhase,filterEndPhase, logQ)
				dbg.draw('Sphere', (li.contactPreFilter)*100, 'swingFootPrefilter'..tostring(li.isLeft),'blue')
				dbg.draw('Sphere', (li.contactFilter)*100, 'swingFootFiltered'..tostring(li.isLeft),'green')
				dbg.draw('Sphere', (li.contactPos)*100, 'swingFootUnfiltered'..tostring(li.isLeft),'red')
			end

		end
	end
	return model
end
return createModel
