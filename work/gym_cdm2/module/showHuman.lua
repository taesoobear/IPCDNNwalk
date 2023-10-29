fc=require("gym_cdm2/module/CDMTraj")
require("gym_cdm2/module/info_hyunwooLowDOF")
require('subRoutines/VelocityFields')

RagdollSim_prepareMotionCDMonly= RagdollSim.prepareMotionForSim -- back the previous definition
RagdollSim_stepCDMonly=RagdollSim.step
RagdollSim_resetCDMonly=RagdollSim.reset
RagdollSim_initCDMonly=RagdollSim.__init
model.collectFinalTraj =false
showCDM=true useOgreSkin=true 
if debug_draw then
	useOgreSkin=false
end
if random_walk then
	return
end

if useOgreSkin then
	package.path=package.path..";../../taesooLib/Samples/classification/lua/?.lua"
	require("subRoutines/RetargetSkin")
	require("gym_cdm2/module/RetargetConfigPreset")
end

function RagdollSim:__init(loader, drawSkeleton, simulatorParam)
	RagdollSim_initCDMonly(self, loader, drawSkeleton, simulatorParam)
	if not showCDM then
		self.CDM.skin:setVisible(false)
	else
		self.CDM.skin:setTranslation((CDMdrawOffset or vector3(0,0,0))*100)
	end
	self.deltaY=0
end
function RagdollSim:createFullbodySkin()
	local g_info=model.motionData
	local loader=g_info.loader
	local drawSkeleton=false

	local skin_fullbody
	if useOgreSkin then

		loader:updateInitialBone()
		local showDebugSkin=false
		local skinConfig=skinConfigHyunwooLowT -- defined in RetargetConfigPreset
		if useHanyangInsteadOfHyunwoo then
			skinConfig=skinConfigHanyangLowT_robot
			if model.ogreSkinStraightFingers then
				skinConfig=skinConfigHanyangLowT_robot_straightFingers
			end
		end
		skin_fullbody=RetargetSkin(loader, skinConfig, showDebugSkin)
	else
		skin_fullbody=RE.createVRMLskin(loader, drawSkeleton)
		skin_fullbody:setThickness(0.03)
		skin_fullbody:scale(100,100,100)
		skin_fullbody:setTranslation(0,0,0)
		skin_fullbody:setMaterial('lightgrey_verytransparent')
	end

	skin_fullbody:setTranslation(0,-2,  0)

	return skin_fullbody
end

function RagdollSim:prepareMotionForSim()
	--dbg.startTrace2()
	RagdollSim_prepareMotionCDMonly(self)
	--assert(model.collectRefTraj==false)

	local g_info=model.motionData

	local allLimbs=copyTable(input.limbs)
	if placeObstacle then
		array.concat(allLimbs, input.hands)
	end
	mSolverInfo=fc.createIKsolver(g_info.loader, allLimbs)


	local effectors=mSolverInfo.effectors
	mLimbEffectors=MotionUtil.Effectors()
	local axes=vector3N()
	if placeObstacle then
		mLimbEffectors:resize(4)
		mLimbEffectors(0):init(effectors(0).bone, effectors(0).localpos+vector3(0,0.02, 0.05))
		mLimbEffectors(1):init(effectors(2).bone, effectors(2).localpos+vector3(0,0, 0.0))
		mLimbEffectors(2):init(effectors(4).bone, effectors(4).localpos)
		mLimbEffectors(3):init(effectors(5).bone, effectors(5).localpos)
		axes:setSize(4)
		axes:setAllValue(vector3(1,0,0))
		axes(2):assign(vector3(0,-1,0))
		axes(3):assign(vector3(0,1,0))
	else
		mLimbEffectors:resize(2)
		mLimbEffectors(0):init(effectors(0).bone, effectors(0).localpos)
		mLimbEffectors(1):init(effectors(2).bone, effectors(2).localpos)
		axes:setSize(2)
		axes:setAllValue(vector3(1,0,0))
	end
	local kneeIndices=intvectorn()
	for i=0, mLimbEffectors:size()-1 do
		kneeIndices:pushBack(mLimbEffectors(i).bone:parent():treeIndex())
	end
	mLimbIKsolver=LimbIKsolver(g_info.loader.dofInfo, mLimbEffectors, kneeIndices, axes)
	if model.IKconfig and model.IKconfig.doNotUseKneeDamping then
		mLimbIKsolver:setOption('useKneeDamping', 0)
	end

	-- mot: humanoid3d_run


	self.skin_fullbody=self:createFullbodySkin()
	self.skin_ref:setVisible(false)

	if model:loadRefTraj() then
		local filterSize=model.delayedVis*2+1 -- has to be an odd number
		mFilter=OnlineFilter(loader, model:getInitialFullbodyPose(), filterSize)
	end
	if model.collectFinalTraj then
		local refTraj=g_refFinalTraj
		if not refTraj then
			g_refFinalTraj={
				poses=matrixn(),
				footdof=matrixn(),
			}
		end
	end
end


function RagdollSim:generatePose( _prevFullbodyPose)
	return self:_generatePose( _prevFullbodyPose)
end

function RagdollSim:_generatePose( _prevFullbodyPose)
	
	local simulator=self:getCDMsimulator()
	do
		-- get cdm state
		local cdm_gf=simulator:getWorldState(0):globalFrame(1) -- global frame
		local cdm_dq=simulator:getDQ(0)
		local footPos1, footOri1, con1=self:getFilteredFootPos(cdm_gf, 1)
		local footPos2, footOri2, con2=self:getFilteredFootPos(cdm_gf, 2)
		self.phaseFilter:setState(cdm_gf, cdm_dq, transf(footOri1, footPos1), transf(footOri2, footPos2),  con1, con2, model:getPhase_iframe())
	end

	local cdm_gf, cdm_dq=self.phaseFilter:getCDMbodyState()
	local refPose, CDMtoRoot, refDPose, CDMfeetToHumanFeetPositions, comdof_error, bodyVel, conWeight=model:_sampleFullbodyPose(self.phaseFilter:getPhase())

	local human_root=cdm_gf*CDMtoRoot:exp()
	MotionDOF.setRootTransformation(refPose, human_root)

	local comdof=CT.zeros(7)
	MotionDOF.setRootTransformation(comdof, cdm_gf)
	do
		local w=cdm_dq:toVector3(0)
		local v=cdm_dq:toVector3(3)
		local R=cdm_gf.rotation
		local invR=R:inverse()
		w:rotate(invR)
		v:rotate(invR)
		w:radd(comdof_error:toVector3(3))
		v:radd(comdof_error:toVector3(6))

		comdof:setVec3(0, comdof:toVector3(0)+R*comdof_error:toVector3(0))

		local fullbodyR=refPose:toQuater(3)
		-- velocity error
		-- cdm_dq should be near zero when closely following mocap.
		cdm_dq:setVec3(0, R*w-fullbodyR*bodyVel:toVector3(0))
		cdm_dq:setVec3(3, R*v-fullbodyR*bodyVel:toVector3(3))
	end
	comdof=comdof..cdm_dq

	if draw_constraints then
		dbg.draw('Axes', comdof:toTransf(0):translate(vector3(-1,0,0)), 'comdof',100)
		dbg.draw('Axes', human_root, 'human_root',100)
	end

	local numCon=#input.limbs
	local footDOF=vectorn(3*numCon)
	local effectors=mSolverInfo.effectors

	local loader=mSolverInfo.loader
	loader:setPoseDOF(refPose)


	local cdm_rotY=cdm_gf.rotation:rotationY()

	local importance=CT.vec(0,0)
	local ignoreFootDelta={ false, false}
	-- update ignoreFootDelta
	for ifoot=0, 1 do
		local footCoord, con=self.phaseFilter:getFootPos(ifoot)
		if not model.ignoreSwingFeetDelta then
			model.ignoreSwingFeetDelta ={}
			model.ignoreSwingFeetDeltaInfo ={}
		end

		if model.ignoreSwingFeetDelta[ifoot+1] then
			local prevCon=model.ignoreSwingFeetDeltaInfo[ifoot+1]
			RE.output2('ignore'..ifoot, 'true :'.. tostring(con)..',' ..tostring(prevCon))
			if con and prevCon==false then
				-- turn off transition.
				model.ignoreSwingFeetDelta[ifoot+1]=false
				model.ignoreSwingFeetDeltaInfo[ifoot+1]=nil -- not same as false
			else
				ignoreFootDelta[ifoot+1]=true
			end
		else
			RE.output2('ignore'..ifoot, 'false ', model.ignoreSwingFeetDelta[ifoot+1])
		end
		model.ignoreSwingFeetDeltaInfo[ifoot+1]=con
	end

	for icontact=0,numCon-1 do
		ifoot=math.floor(icontact/2)

		local refCoord, con=self.phaseFilter:getFootPos(ifoot)
		if con then
			importance:set(ifoot, 1)
		end
		local contactpos=refCoord*CDMfeetToHumanFeetPositions:toVector3(icontact*3)

		if draw_constraints then
			dbg.draw('Sphere', contactpos*100+vector3(-100,0,0), 'sphere'..icontact, 'green',2)
		end
		--dbg.delayedDraw('Sphere', model.delayedVis, contactpos*100, 'cpfoot'..icontact, 'green')

		local importance=sop.clampMap(contactpos.y-refCoord.translation.y, 0.03, 0.05, 1,0)

		-- 만약 contactpos가 땅 믿에 있으면 꺼내라.. 정도가 좋을 듯.
		-- 아니다. contact가 toe 인치 heel인지도 구분해야할 것 같은데.. 
		-- importance?
		if true then
			-- adapt to terrain
			local icon=icontact-ifoot*2

			local delta=vector3(0,0,0)
			if icon==1 then
				delta.z=-1
			end
			delta:rotate(cdm_rotY)
			local pos2=contactpos:copy()
			if hasGUI then
				if not con and self.projectToGroundHull then
					local ilimb=ifoot
					if ilimb==0 then ilimb=2 end
					self:projectToGroundHull(pos2, self.legInfo[ilimb])
				else
					self:projectToGround(pos2)
				end
				contactpos.y=pos2.y*importance+contactpos.y*(1-importance)
			end
		end

		local footPos=refCoord.translation
		if debug_draw then
			dbg.delayedDraw('Sphere', model.delayedVis, footPos*100, 'foot'..icontact, 'red')
			dbg.delayedDraw('Sphere', model.delayedVis, contactpos*100, 'contactpos'..icontact, 'blue')
		end

		local desired_contactpos=effectors(icontact).bone:getFrame():toGlobalPos(effectors(icontact).localpos)

		if debug_draw then
			if ifoot==0 then
				dbg.draw('Sphere', desired_contactpos*100, 'desiredcontact'..(icontact),'green',1)
				dbg.draw('Sphere', footPos*100+vector3(0,7,0), 'desiredcontactfoot'..(icontact),'green',1)
			end
		end

		if true and not model.doNotUseConWeight then
			--footDOF:setVec3(icontact*3, contactpos)
			local w=conWeight(1-ifoot)
			if not con and not model.doNotUseConHeight and (cdm_rotY:inverse()*(desired_contactpos-contactpos)).z>0.1 then
				w=0
			end
			if ignoreFootDelta[ifoot+1] then
				w=0
			end
			RE.output2('conWeight'..icontact..ifoot, w)
			--print(con, 'conWeight'..icontact..ifoot, w)

			footDOF:setVec3(icontact*3, contactpos*w+desired_contactpos*(1-w))
		else
			footDOF:setVec3(icontact*3, contactpos)
		end
	end

	if not model.doNotSolveIK then
		local importance=CT.vec(1,1)
		local renderPose=self:solveIK( mSolverInfo,  _prevFullbodyPose, refDPose, refPose, comdof, footDOF, importance)
		return renderPose
	else
		return refPose
	end
end

function getConHeight(conpos)
	if g_terrain then
		return conpos.y-g_terrain:getTerrainPos(conpos).y
	end
	return conpos.y
end
function RagdollSim:step(_iframe, action)
	local reward, step_state, episode_done=RagdollSim_stepCDMonly(self, _iframe, action)

	if not mFilter then 
		return reward,step_state, episode_done
	end
	-- a transition may have occured
	local mocapstep_per_controlstep=model.frame_rate*RL_step

	--print(iframe)
	if not model.doNotSolveIK then

		local renderPose= self:_generatePose( self._prevFullbodyPose)

		renderPose:set(1, renderPose(1)-self.deltaY)
		if false then -- set true to use only MMIK
			disable_IK_unfiltered =true
			disable_fix_floating_contactFoot =true
			disable_IK_filtered =true
		end
		if not disable_IK_unfiltered then
			-- IK unfiltered
			-- remove feet penetration 
			local loader=mSolverInfo.loader
			local effectors=mSolverInfo.effectors
			loader:setPoseDOF(renderPose)
			local minY=1e5
			local conpos=vector3N(2)
			local conori=quaterN(2)
			local importance=CT.vec(1,1)
			for i=0, 1 do
				conori(i):assign(effectors(i*2).bone:getFrame().rotation)
				conpos(i):assign(effectors(i*2).bone:getFrame()*effectors(i*2).localpos)
				minY=math.min(minY, conpos(i).y- getConHeight(conpos(i)))
				local cp2=effectors(i*2+1).bone:getFrame()*effectors(i*2+1).localpos
				minY=math.min(minY, cp2.y-getConHeight(cp2))
				if minY<0 then
					conpos(i).y=conpos(i).y-minY
				end
				--if i==0 and self.LheightAdjust then
				--	conpos(i).y=conpos(i).y+self.LheightAdjust:generate()
				--end
				local con=self.phaseFilter:getCon( i)
				if con then
					local maxY=0.02
					if minY>maxY then
						conpos(i).y=conpos(i).y-(minY-maxY)
					end
				end
			end

			local rootTF=MotionDOF.rootTransformation(renderPose)
			--if self.heightAdjust then
			--	rootTF.translation.y=rootTF.translation.y+self.heightAdjust:generate()
			--end

			if placeObstacle then
				ObstacleSim:updateLimbIKconstraints(conpos, conori, importance)
			end

			mLimbIKsolver:IKsolve3(renderPose, rootTF, conpos, conori, importance);
		end

		if not disable_fix_floating_contactFoot then
			-- fix contact feet floating above ground (using vertical translation which is dissipated over time)
			local loader=mSolverInfo.loader
			local effectors=mSolverInfo.effectors
			loader:setPoseDOF(renderPose)
			local minY=1e5
			local conpos=vector3N(2)
			local conori=quaterN(2)
			local importance=CT.vec(1,1)
			local deltaY=0
			for i=0, 1 do
				conori(i):assign(effectors(i*2).bone:getFrame().rotation)
				conpos(i):assign(effectors(i*2).bone:getFrame()*effectors(i*2).localpos)
				minY=math.min(minY, getConHeight(conpos(i)))
				minY=math.min(minY, (effectors(i*2+1).bone:getFrame()*effectors(i*2+1).localpos).y)
				if minY<0 then
					conpos(i).y=conpos(i).y-minY
				end
				--if i==0 and self.LheightAdjust then
				--	conpos(i).y=conpos(i).y+self.LheightAdjust:generate()
				--end
				local con=self.phaseFilter:getCon( i)
				if con then
					local maxY=0.02
					if minY>maxY then
						deltaY=math.max(minY-maxY)
					end
					local deltaYrange=model.deltaYrange or {0, 0.06}
					deltaY=math.max(deltaY, deltaYrange[1])
					deltaY=math.min(deltaY, deltaYrange[2])
				end
			end
			renderPose:set(1, renderPose(1)-deltaY)

			self.deltaY=math.max(self.deltaY, deltaY)
			-- dissipated 5mm per frame
			self.deltaY=math.max(self.deltaY-0.005, 0)

			RE.output2('deltaY', self.deltaY)
		end

		mFilter:setCurrPose(renderPose)

		local renderPoseFiltered=mFilter:getFiltered()
		if not disable_IK_filtered then
			-- remove feet penetration due to filtering
			local loader=mSolverInfo.loader
			local effectors=mSolverInfo.effectors
			loader:setPoseDOF(renderPoseFiltered)
			local conpos=vector3N(2)
			local conori=quaterN(2)
			local importance=CT.vec(1,1)
			for i=0, 1 do
				local minY=1e5
				conori(i):assign(effectors(i*2).bone:getFrame().rotation)
				conpos(i):assign(effectors(i*2).bone:getFrame()*effectors(i*2).localpos)
				minY=math.min(minY, getConHeight(conpos(i)))
				--dbg.draw('Sphere', conpos(i)*100, 'con'..i)
				minY=math.min(minY, getConHeight(effectors(i*2+1).bone:getFrame()*effectors(i*2+1).localpos))
				--dbg.draw('Sphere',(effectors(i*2+1).bone:getFrame()*effectors(i*2+1).localpos)*100, 'con2_'..i)
				local minHeight=0.01

				if minY<minHeight then
					conpos(i).y=conpos(i).y+(minHeight-minY)
				end
				if i==0 and self.LheightAdjust then
					conpos(i).y=conpos(i).y+self.LheightAdjust:generate()
				end
				--local con=self.phaseFilter:getCon( i)
				--if con then
				--	if conpos(i).y>0 then
				--		conpos(i).y=0
				--	end
				--end
			end

			local rootTF=MotionDOF.rootTransformation(renderPoseFiltered)
			if self.heightAdjust then
				rootTF.translation.y=rootTF.translation.y+self.heightAdjust:generate()
			end

			if placeObstacle then
				ObstacleSim:updateLimbIKconstraints(conpos, conori, importance)
			end
			mLimbIKsolver:IKsolve3(renderPoseFiltered, rootTF, conpos, conori, importance);
			if g_refFinalTraj then
				local refTraj=g_refFinalTraj 
				refTraj.poses:pushBack(renderPoseFiltered)

				local footDOF=vectorn()
				for ifoot=0, 1 do
					local simulator=self.CDM.simulator
					local cdm_gf=simulator:getWorldState(0):globalFrame(1) -- global frame
					local refCoord, con=self.phaseFilter:getFootPos(ifoot)
					local footPos=refCoord.translatoin
					if con then
						footDOF=footDOF..(CT.vec(1)..CT.vec(footPos))
					else
						footDOF=footDOF..(CT.vec(0)..CT.vec(footPos))
					end
				end
				refTraj.footdof:pushBack(footDOF)
				if math.fmod(refTraj.poses:rows(),50)==0 then
					util.saveTable(refTraj, 'finalTraj.dat')
					print(refTraj.poses:rows(), 'finalTraj.dat exported. Manually copy this file to finalTraj_~.dat')
				end
			end
		end
		self.skin_fullbody:setPoseDOF(renderPoseFiltered)

		if g_exportMot then
			local em=g_exportMot
			local fn=em[1]
			if not em.mot then
				em.mot=g_info.mot:copy()
				em.mot:resize(0)
			end
			em.mot:pushBack(renderPoseFiltered)
			if math.fmod(em.mot:numFrames(),50)==0 then
				print('export ',fn)
				em.mot:exportMot(fn)
			end
		end

	else
		local refPose=self:generatePose( self._prevFullbodyPose)
		self.skin_fullbody:setPoseDOF(refPose)
	end

	return reward,step_state, episode_done
end
function RagdollSim:reset()
	local initialState_env=RagdollSim_resetCDMonly(self)

	if not mFilter  then return initialState_env end

	self.phaseFilter=CDMphaseFilter(CDMphaseFilter.default_delay, model.refFrameToPhase) -- only the fullbody-phase will be filtered so that the timing flows fluently.


	local iframe=model.iframe
	local simulator=self.CDM.simulator
	local cdm_gf=simulator:getWorldState(0):globalFrame(1) -- global frame

	local refPose, CDMtoRoot=model:sampleFullbodyPose()
	local human_root=cdm_gf*CDMtoRoot:exp()
	MotionDOF.setRootTransformation(refPose, human_root)

	self._prevFullbodyPose=refPose

	return initialState_env
end

--	pose: desired human pose with correct root configuration
--	returns renderPose
function RagdollSim:solveIK( mSolverInfo, g_prevPose, dpose, pose, comdof, footDOF, importance)

	local velScale=1
	local IKconfig={
		wVelRoot=0.95,
		wCOMy=1,
		wFullVel=0.1*velScale,
		wFullVelPose=0.02,
		wMM=1,
		wHead_y=0.0001,
		wHead_z=0.1,
		--effWeightsY=2,
		effWeightsY=0.5,
		v2_max=50,
		useHead=1,
	}
	IKconfig.wCOMy=1
	IKconfig.wFullVel=0.2
	IKconfig.wFullVelPose=0.02
		wHead_y=0
		wHead_z=0
	IKconfig.wMM=1
	IKconfig.v2_max=100
	IKconfig.useHead=1
	IKconfig.frameRate=1/RL_step

	IKconfig.removeRemainingSliding=false
	if true then
		IKconfig.hasCOM=1
		IKconfig.hasMM=1
		IKconfig.hasPoseCon=1
		IKconfig.useEffYcon=0
		IKconfig.effWeightsY=0.1  -- 발의 importance==0.1
		--IKconfig.debugDraw=true
	end
	if false then
		-- 속도 계산이 맞는지 test용 세팅. 
		IKconfig.hasCOM=1 
		IKconfig.hasMM=1
		IKconfig.hasPoseCon=1
		IKconfig.useEffYcon=0
		IKconfig.effWeightsY=0
		IKconfig.removeRemainingSliding=false
	end

	if model.IKconfig then
		table.mergeInPlace(IKconfig, model.IKconfig, true)
	end

	if self.impulse>0 and self.gizmo and self.gizmo[3] then
		local f=self.gizmo[3]
		local loader=mSolverInfo.loader
		if f:length()>1e-3 then
			loader:setPoseDOF(g_prevPose)
			local p=loader:getBoneByName(self.gizmo[1]):getFrame()*self.gizmo[2]
			if model.delayedVis then
				dbg.delayedDraw('Arrow', model.delayedVis-2, p*100-f, p*100, 'impulseGizmo')
			else
				dbg.draw('Arrow', p*100-f, p*100, 'impulseGizmo')
			end
			IKconfig.vel_con={{self.gizmo, p, f*0.01}}
		end
	end


	--g_timer2=util.PerfTimer2()
	--g_timer2:start()
	if true then
		fc.solveIK(mSolverInfo, g_prevPose, dpose, pose, comdof, footDOF, nil ,IKconfig)
	else
		g_prevPose:assign(pose)
	end
	--print(g_timer2:stop()/1000.0, "ik.")
	local renderPose=g_prevPose:copy()
	if IKconfig.IKpostprocess then
		fc.solveIK_postprocess(mSolverInfo, renderPose, comdof, footDOF,nil,
		{
			wCOMy=0.3,
			wMM=0.1,
			wHead_y=0.0001,
			effWeightsY=2,
			--v2_max=5,
			useHead=1,
		}
		)
	end

	if IKconfig.removeRemainingSliding then
		RE.output2("remaining", importance)
		-- remove remaining feet sliding due to soft constraints.
		local loader=mSolverInfo.loader
		local effectors=mSolverInfo.effectors
		loader:setPoseDOF(renderPose)
		local conpos=vector3N(2)
		local conori=quaterN(2)
		for i=0, 1 do
			conori(i):assign(effectors(i*2).bone:getFrame().rotation)
			conpos(i):assign(footDOF:toVector3(i*2*3))
			--dbg.namedDraw('Sphere', conpos(i)*100, 'conpos'..i)
		end

		--print(importance)
		if placeObstacle then
			ObstacleSim:updateLimbIKconstraints(conpos, conori, importance)
		end
		mLimbIKsolver:IKsolve3(renderPose, MotionDOF.rootTransformation(renderPose), conpos, conori, importance);
	end
	return renderPose
end
