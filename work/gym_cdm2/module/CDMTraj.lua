
local fc={}

fc.CDMTraj=LUAclass()

function MotionUtil.FullbodyIK_MotionDOF:_setNormalConstraint(i, bone, localpos, normal, gpos)
	local plane=Plane(-1*normal, gpos)
	return self:_setHalfSpaceConstraint(i, bone, localpos, plane.normal, plane.d)
end
-- boolN con, 
-- matrixn traj (global positions : n by 3), 
-- [function adjustHeight(pos) returns y] -- for support positions
-- [function clearGroundTraj(traj, s, e)] -- for swing traj
function fc.removeSliding(con, traj, adjustHeight, clearGroundTraj)
	local ii=intIntervals()
	ii:runLengthEncode(con)
	for icontact=0, ii:size()-1  do
		local si=ii:startI(icontact)
		local ei=ii:endI(icontact)
		local prev_ei=nil
		local next_si=nil
		if icontact > 0 then
			prev_ei=ii:endI(icontact-1)
		end
		if icontact <ii:size()-1 then
			next_si=ii:startI(icontact+1)
		end
		local avgPos=vector3(0,0,0)
		for i=si, ei-1 do
			local data=traj:row(i)
			avgPos:radd(data:toVector3(0))
		end
		avgPos:scale(1.0/(ei-si))
		if adjustHeight then
			avgPos.y=adjustHeight(avgPos)
		end

		local err1=avgPos-traj:row(si):toVector3(0)
		local err2=avgPos-traj:row(ei-1):toVector3(0)

		local minWindow=15
		if prev_ei then
			for i=prev_ei, si-1 do
				local data=traj:row(i)
				local s=sop.smoothMapA(i, prev_ei-1, si, 0,1)
				data:setVec3(0, data:toVector3(0)+err1*s)
			end
			if clearGroundTraj then clearGroundTraj(traj, prev_ei, si) end
		elseif si>0 then
			for i=0, si-1 do
				local data=traj:row(i)
				local s=sop.smoothMapA(i, math.min(0, si-minWindow), si, 0,1)
				data:setVec3(0, data:toVector3(0)+err1*s)
			end
			if clearGroundTraj then clearGroundTraj(traj, 0, si) end
		end

		if nsi then
			for i=ei, nsi-1 do
				local data=traj:row(i)
				local s=sop.smoothMapB(i, ei-1, nsi, 1,0)
				data:setVec3(0, data:toVector3(0)+err2*s)
			end
			if clearGroundTraj then clearGroundTraj(traj, ei, nsi) end
		elseif ei<traj:rows() then
			local nf=traj:rows()
			for i=ei, nf-1 do
				local data=traj:row(i)
				local s=sop.smoothMapB(i, ei-1, math.max(nf, ei+minWindow), 1,0)
				data:setVec3(0, data:toVector3(0)+err2*s)
			end
			if clearGroundTraj then clearGroundTraj(traj, ei, nsi) end

		end
		local nf=traj:rows()
		for i=si, ei-1 do
			local data=traj:row(i)
			data:setVec3(0, avgPos)
		end
	end
end
function fc.removeDeltaSliding(con, traj, trajOrig)
	-- 원래 미끄러지는 정도는 봐주는 버젼.
	local ii=intIntervals()
	ii:runLengthEncode(con)
	for icontact=0, ii:size()-1  do
		local si=ii:startI(icontact)
		local ei=ii:endI(icontact)
		local prev_ei=nil
		local next_si=nil
		if icontact > 0 then
			prev_ei=ii:endI(icontact-1)
		end
		if icontact <ii:size()-1 then
			next_si=ii:startI(icontact+1)
		end
		local avgPos=vector3(0,0,0)
		avgPos:radd(traj:row(si):toVector3(0))
		avgPos:radd(traj:row(ei-1):toVector3(0))
		avgPos:scale(1.0/2.0)

		local delta=trajOrig:row(ei-1):toVector3(0)-trajOrig:row(si):toVector3(0)
		local avgPosOrig=(trajOrig:row(ei-1):toVector3(0)+trajOrig:row(si):toVector3(0))*0.5

		local err1=avgPos-delta*0.5-traj:row(si):toVector3(0)
		local err2=avgPos+delta*0.5-traj:row(ei-1):toVector3(0)

		local minWindow=15
		if prev_ei then
			for i=prev_ei, si-1 do
				local data=traj:row(i)
				local s=sop.smoothMapA(i, prev_ei-1, si, 0,1)
				data:setVec3(0, data:toVector3(0)+err1*s)
			end
		elseif si>0 then
			for i=0, si-1 do
				local data=traj:row(i)
				local s=sop.smoothMapA(i, math.min(0, si-minWindow), si, 0,1)
				data:setVec3(0, data:toVector3(0)+err1*s)
			end
		end

		if nsi then
			for i=ei, nsi-1 do
				local data=traj:row(i)
				local s=sop.smoothMapB(i, ei-1, nsi, 1,0)
				data:setVec3(0, data:toVector3(0)+err2*s)
			end
		elseif ei<traj:rows() then
			local nf=traj:rows()
			for i=ei, nf-1 do
				local data=traj:row(i)
				local s=sop.smoothMapB(i, ei-1, math.max(nf, ei+minWindow), 1,0)
				data:setVec3(0, data:toVector3(0)+err2*s)
			end

		end
		local nf=traj:rows()
		for i=si, ei-1 do
			local data=traj:row(i)
			data:setVec3(0, trajOrig:row(i):toVector3(0)+avgPos-avgPosOrig)
		end
	end
end

-- for an alignment using a rigid transformation, set accumulatedError1==accumulatedError2
function fc.alignFootTraj(rfoot_midPos, con, accumulatedError1, accumulatedError2)
	local ii=intIntervals()
	ii:runLengthEncode(con)
	local nf=rfoot_midPos:size()
	local new_midPos=rfoot_midPos:copy()
	for iinterval=0, ii:size() do
		local prevEnd=nil
		local s=nil
		local e=nil
		local prevError, currError
		local prevMid, currMid
		if iinterval==0 then
			if s~=0 then
				prevMid=0
				prevEnd=0
				prevError=accumulatedError1
			end
		else
			local ps=ii:startI(iinterval-1)
			prevEnd=ii:endI(iinterval-1)
			prevMid=ps*0.5+prevEnd*0.5
			if ps==0 then prevMid=0 end
			prevError=accumulatedError1:Interpolate(sop.map(prevMid, 0, nf-1, 0,1), accumulatedError2)
		end
		if iinterval~=ii:size() then
			s,e=unpack(ii(iinterval))
			currMid=s*0.5+e*0.5
			if s==0 then currMid=0 end
			if e==nf then currMid=nf-1 end
			currError=accumulatedError1:Interpolate(sop.map(currMid, 0, nf-1, 0,1), accumulatedError2)
		else
			s=nil
			if prevEnd~=nf then
				currError=accumulatedError2
				s=nf
				currMid=nf
			end
			e=nil
		end

		if prevEnd and s then
			-- fill swingfoot positions for [prevEnd, s)
			for i=prevEnd, s-1 do
				local rfootsim=rfoot_midPos(i)
				local w1=sop.map(prevMid, 0, nf-1, 0, 1)
				local w2=sop.map(currMid, 0, nf-1, 0, 1)
				local w=sop.map(i, prevEnd, s, w1, w2)
				local errorToRemove=accumulatedError1:Interpolate(w, accumulatedError2)
				new_midPos(i):assign( errorToRemove*rfootsim)
			end
		end

		if e then
			-- fill contact foot positions [s, e)
			for i=s, e-1 do
				local rfootsim=rfoot_midPos(i)
				local w=sop.map(currMid, 0, nf-1, 0, 1)
				local errorToRemove=accumulatedError1:Interpolate(w, accumulatedError2)
				new_midPos(i):assign( errorToRemove*rfootsim)
			end
		end
	end
	return new_midPos
end

function fc.adjustContactLocations(con, traj, newPos)
	local ii=intIntervals()
	ii:runLengthEncode(con)
	assert(ii:size()==newPos:size())
	for icontact=0, ii:size()-1  do
		local si=ii:startI(icontact)
		local ei=ii:endI(icontact)
		local prev_ei=nil
		local next_si=nil
		if icontact > 0 then
			prev_ei=ii:endI(icontact-1)
		end
		if icontact <ii:size()-1 then
			next_si=ii:startI(icontact+1)
		end
		local avgPos=newPos(icontact)

		local err1=avgPos-traj:row(si):toVector3(0)
		local err2=avgPos-traj:row(ei-1):toVector3(0)

		if prev_ei then
			for i=prev_ei, si-1 do
				local data=traj:row(i)
				local s=sop.map(i, prev_ei-1, si, 0,1)
				data:setVec3(0, data:toVector3(0)+err1*s)
			end
		elseif si>0 then
			for i=0, si-1 do
				local data=traj:row(i)
				local s=sop.map(i, 0, si, 0,1)
				data:setVec3(0, data:toVector3(0)+err1*s)
			end
		end

		if nsi then
			for i=ei, nsi-1 do
				local data=traj:row(i)
				local s=sop.map(i, ei-1, nsi, 1,0)
				data:setVec3(0, data:toVector3(0)+err2*s)
			end
		elseif ei<traj:rows() then
			local nf=traj:rows()
			for i=ei, nf-1 do
				local data=traj:row(i)
				local s=sop.map(i, ei-1, nf, 1,0)
				data:setVec3(0, data:toVector3(0)+err2*s)
			end

		end
		local nf=traj:rows()
		for i=si, ei-1 do
			local data=traj:row(i)
			data:setVec3(0, avgPos)
		end
	end
end

-- feet_config= { toe=vector3(0, -0.05, 0.14), -- local pos of toe,
--                  heel=vector3(0, -0.10, 0.06),} -- local pos of heel
-- returns { rtoe, rheel, ltoe, lheel}
function fc.getFeetTraj(feet_config, loader, mMotionDOF)
	local nf=mMotionDOF:numFrames()
	local feetTraj={
		vector3N(nf),  -- R toe
		vector3N(nf), 
		vector3N(nf),  -- L toe
		vector3N(nf), 
	}

	for i=0, mMotionDOF:numFrames()-1 do
		loader:setPoseDOF(mMotionDOF:row(i))

		for c=1, 4 do

			local conpos
			local lpos=feet_config.heel
			if math.fmod(c-1,2)==0 then
				lpos=feet_config.toe
			end
			local gpos
			if c<=2 then
				gpos=loader:getBoneByVoca(MotionLoader.RIGHTANKLE):getFrame()*lpos
			else
				gpos=loader:getBoneByVoca(MotionLoader.LEFTANKLE):getFrame()*lpos
			end
			feetTraj[c](i):assign(gpos)
		end
	end

	return feetTraj
end

-- con={conRfoot, conLfoot}
-- feet_config= { toe=vector3(0, -0.05, 0.14), -- local pos of toe,
--                  heel=vector3(0, -0.10, 0.06),} -- local pos of heel
-- option={debugDraw=false}
-- mLoader should already have "setVoca"-called.
function fc.removeFeetSliding(con, feet_config, mLoader, mMotionDOF, option)
	if not option then
		option={}
	end

	local loader=mLoader
	-- 원본 모캡의 발 미끄럼 잡기

	local feetTraj=fc.getFeetTraj(feet_config, loader, mMotionDOF)
	local adjustHeight=function(x) return 0.0 end
	fc.removeSlidingHeelAndToe(con[1], feetTraj[1], feetTraj[2], adjustHeight)
	fc.removeSlidingHeelAndToe(con[2], feetTraj[3], feetTraj[4], adjustHeight)

	mSolverInfo=fc.createIKsolver(loader, input.limbs)

	for i=0, mMotionDOF:rows()-1 do
		local pose=mMotionDOF:row(i):copy()

		local numCon=mSolverInfo.numCon
		local footPos=vector3N (numCon);

		footPos(0):assign(feetTraj[3](i)) -- L toe (see info_hyunwooLowDOF.lua)
		footPos(1):assign(feetTraj[4](i))
		footPos(2):assign(feetTraj[1](i)) -- R toe
		footPos(3):assign(feetTraj[2](i))

		local mIK=mSolverInfo.solver
		local mEffectors=mSolverInfo.effectors
		--local prev_roottf=MotionDOF.rootTransformation(pose)
		--local toLocal=prev_roottf:inverse()
		local toLocal=MotionDOF.rootTransformation(pose):inverse()
		toLocal.translation.y=0
		toLocal.rotation:assign(toLocal.rotation:rotationY())

		local useHead=1
		mIK:_changeNumEffectors(numCon)
		mIK:_changeNumConstraints(useHead)
		if useHead==1 then
			local loader=mSolverInfo.loader
			--local headRefPose=loader.dofInfo:blendPose( nextPose1, nextPose2, 0.1)
			loader:setPoseDOF(pose)
			headBone=loader:getBoneByName(input.head[1])
			headPos=headBone:getFrame()*input.head[2]
		end

		mIK:setParam('damping_weight', 0.01,0.01)

		for i=0,numCon-1 do
			mIK:_setEffector(i, mEffectors(i).bone, mEffectors(i).localpos)
			footPos(i):assign(toLocal*footPos(i))
		end
		if useHead==1 then
			local i=numCon
			local wHead_y=0.0001
			mIK:_setPositionConstraint(0, headBone, input.head[2], toLocal*headPos, 1,wHead_y ,0.1 );
		end

		mIK:_effectorUpdated()
		MotionDOF.setRootTransformation(pose, toLocal*MotionDOF.rootTransformation(pose))
		mIK:IKsolve(pose, footPos)


		MotionDOF.setRootTransformation(pose, toLocal:inverse()*MotionDOF.rootTransformation(pose))
		mMotionDOF:row(i):assign(pose)
	end
	if option.debugDraw then
		for c=1,4 do
			dbg.namedDraw('Traj', feetTraj[c]:matView()*100, 'feet'..c)
		end
	end
end

local function midPos(toetraj, heeltraj, i)
	local toe=toetraj:row(i):toVector3(0)
	local heel=heeltraj:row(i):toVector3(0)
	local footlen=heel:distance(toe)

	if toe.y < heel.y then
		-- toe contact
		local heelDir=heel-toe
		heelDir.y=0
		heelDir:normalize()
		heel=toe+heelDir*footlen
	else
		-- heel contact
		local toeDir=toe-heel
		toeDir.y=0
		toeDir:normalize()
		toe=heel+toeDir*footlen
	end
	return toe*0.5+heel*0.5
end

-- returns conPos(vector3N), con_intervals(intIntervals), conToe(boolN), conHeel(boolN)
function fc.getContactPositions(con, toetraj, heeltraj)
	if dbg.lunaType(toetraj):sub(1,8)=='vector3N' then
		toetraj=toetraj:matView()
	end
	if dbg.lunaType(heeltraj):sub(1,8)=='vector3N' then
		heeltraj=heeltraj:matView()
	end
	local ii=intIntervals()
	ii:runLengthEncode(con)

	local conToe=boolN(con:size())
	local conHeel=boolN(con:size())
	local conPos=vector3N(ii:size())

	for icontact=0, ii:size()-1  do
		local si=ii:startI(icontact)
		local ei=ii:endI(icontact)
		local avgPos=vector3(0,0,0)


		for i=si, ei-1 do
			avgPos:radd(midPos(toetraj, heeltraj, i))

			local toe=toetraj:row(i):toVector3(0)
			local heel=heeltraj:row(i):toVector3(0)
			local thr=0.01
			if toe.y < heel.y-thr then
				conToe:set(i, true)
			elseif heel.y < toe.y-thr then
				conHeel:set(i, true)
			else
				conToe:set(i, true)
				conHeel:set(i, true)
			end
		end
		avgPos:scale(1.0/(ei-si))
		conPos(icontact):assign(avgPos)
	end
	return conPos, ii, conToe, conHeel
end

function fc.getMidSolePositions(con, toetraj, heeltraj)
	local rfoot_midPos=toetraj:copy()
	for i=0, rfoot_midPos:size()-1 do
		if con(i) then
			rfoot_midPos(i):assign(midPos(toetraj:matView(), heeltraj:matView(), i))
		else
			rfoot_midPos(i):assign(toetraj(i)*0.5+heeltraj(i)*0.5)
		end
	end

	return rfoot_midPos
end

-- input parameters:
-- boolN con, 
-- matrixn toetraj (global positions : n by 3), 
-- matrixn heeltraj (global positions : n by 3), 
-- [function adjustHeight(pos) returns y] -- for support positions
-- [function clearGroundTraj(toetraj, heeltraj, s, e)] -- for swing traj
--
function fc.removeSlidingHeelAndToe(con, toetraj, heeltraj, adjustHeight, clearGroundTraj)

	if dbg.lunaType(toetraj):sub(1,8)=='vector3N' then
		toetraj=toetraj:matView()
	end
	if dbg.lunaType(heeltraj):sub(1,8)=='vector3N' then
		heeltraj=heeltraj:matView()
	end


	if false then
		-- only for debugging
		for i=0, toetraj:rows()-1 do
			toetraj:row(i):setVec3(0, midPos(toetraj, heeltraj, i))
		end
		return 
	end

	local conPos, ii, conToe, conHeel=fc.getContactPositions(con, toetraj, heeltraj)

	for icontact=0, ii:size() -1 do
		local si=ii:startI(icontact)
		local ei=ii:endI(icontact)
		local prev_ei=nil
		local next_si=nil
		if icontact > 0 then
			prev_ei=ii:endI(icontact-1)
		end
		if icontact <ii:size()-1 then
			next_si=ii:startI(icontact+1)
		end
		local avgPos=conPos(icontact)

		if adjustHeight then
			avgPos.y=adjustHeight(avgPos)
		end

		--dbg.draw('Sphere', avgPos*100, RE.generateUniqueName())

		local err1=avgPos-midPos(toetraj, heeltraj, si)
		local err2=avgPos-midPos(toetraj, heeltraj, ei-1)

		local minWindow=15
		if prev_ei then
			for i=prev_ei, si-1 do
				local s=sop.smoothMapA(i, prev_ei-1, si, 0,1)
				local data
				data=toetraj:row(i)
				data:setVec3(0, data:toVector3(0)+err1*s)
				data=heeltraj:row(i)
				data:setVec3(0, data:toVector3(0)+err1*s)
			end
			if clearGroundTraj then clearGroundTraj(toetraj, heeltraj, prev_ei, si) end
		elseif si>0 then
			for i=0, si-1 do
				local s=sop.smoothMapA(i, math.min(0, si-minWindow), si, 0,1)
				local data
				data=toetraj:row(i)
				data:setVec3(0, data:toVector3(0)+err1*s)
				data=heeltraj:row(i)
				data:setVec3(0, data:toVector3(0)+err1*s)
			end
			if clearGroundTraj then clearGroundTraj(toetraj, heeltraj, 0, si) end
		end

		if nsi then
			for i=ei, nsi-1 do
				local s=sop.smoothMapB(i, ei-1, nsi, 1,0)
				local data
				data=toetraj:row(i)
				data:setVec3(0, data:toVector3(0)+err2*s)
				data=heeltraj:row(i)
				data:setVec3(0, data:toVector3(0)+err2*s)
			end
			if clearGroundTraj then clearGroundTraj(toetraj,heeltraj, ei, nsi) end
		elseif ei<toetraj:rows() then
			local nf=toetraj:rows()
			for i=ei, nf-1 do
				local s=sop.smoothMapB(i, ei-1, math.max(nf, ei+minWindow), 1,0)
				local data
				data=toetraj:row(i)
				data:setVec3(0, data:toVector3(0)+err2*s)
				data=heeltraj:row(i)
				data:setVec3(0, data:toVector3(0)+err2*s)
			end
			if clearGroundTraj then clearGroundTraj(toetraj, heeltraj, ei, nsi) end

		end

		local nf=toetraj:rows()
		for i=si, ei-1 do
			local err=avgPos-midPos(toetraj, heeltraj, i)

			local data
			data=toetraj:row(i)
			data:setVec3(0, data:toVector3(0)+err)
			data=heeltraj:row(i)
			data:setVec3(0, data:toVector3(0)+err)
		end
	end

	fc.removeSliding(conToe, toetraj, adjustHeight)
	fc.removeSliding(conHeel, heeltraj, adjustHeight)


end

function fc.buildContactConstraints(nf, touchDown, touchOff)
	local con={}
	local function contains(td, iframe)
		assert(type(td)=='table')
		for i, td_frame in ipairs(td) do
			if td_frame==iframe then
				return true
			end
		end
		return false
	end
	local numCon=#touchDown
	for i_con=1, numCon do
		con[i_con]=boolN()
		local c=con[i_con]
		c:resize(nf)
		c:setAllValue(false)
		local ccon=false
		local td=touchDown[i_con]
		local toff=touchOff[i_con]
		for iframe=0, nf-1 do
			if contains(td, iframe) then
				-- start touchDown
				ccon=true
			elseif contains(toff, iframe) then
				-- start swing
				ccon=false
			end
			c:set(iframe, ccon)
		end
	end
	return con
end

-- input ={ bones ={...}, filterFD=20, useRawRoot=false,  doNotUseOffsetQ=false, debugDraw=false, constraints={conL, conR}}
-- output (see testRefCDMtraj.lua): 
-- self.COMtraj
-- self.rotY
-- self.rootTraj
-- self.offsetQ which can only be accessed using self:getOffsetQ
function fc.CDMTraj:__init(loader, mot, input)
	if not input.startF then input.startF=0 end
	if not input.endF then input.endF=mot:numFrames() end
	
	local defaultOptions={
		filterFD=20,
		m1=input.startF,
		m2=input.endF-1,
		useRawRoot=false,
		doNotUseOffsetQ=false,
		debugDraw=true,
		motionFrameRate=30,
		removeAccumulatedError=true,
	}
	table.mergeInPlace(input, defaultOptions)

	local debugDraw=input.debugDraw
	local NDOF=loader.dofInfo:numDOF()

	self.loader=loader
	self.mot=mot:copy()
	if self.mot:rows()==1 then
		self.mot:resize(100)
		for i=1, 99 do
			self.mot:row(i):assign(self.mot:row(0))
		end
	end
	self.mot:matView():quatViewCol(3):align()
	self.MMhelper={
		LtoT=MotionUtil.LoaderToTree(self.loader, false, false),
		chain=BoneForwardKinematics(self.loader),
	}

	self.MMhelper.chain:init()
	local initialHeight=input.initialHeight 
	if initialHeight then
		for i=0, self.mot:numFrames()-1 do
			self.mot:row(i):set(1, self.mot:row(i)(1)+initialHeight)
		end
	end
	-- convert poses to COM local coord (without actually modifying self.mot)
	local COMtraj,rotY

	if input.bones then
		loader:setVoca(input.bones) -- for drawing spheres. see dbg.drawSphere below.
		mot.dofInfo:skeleton():setVoca(input.bones)
	end

	if debug_mode then
		printTable(input)
	end
	if not input.useRawRoot then
		COMtraj,rotY= fc.extractCOMroot(loader, self.mot, input.bones, input)
	else
		COMtraj,rotY= fc.extractCOMroot_raw(loader, self.mot, input.bones, input)
	end

	if debugDraw then
		local thickness=5
		local COMx100=COMtraj:matView():copy()*100
		COMx100:column(1):setAllValue(1)
		dbg.namedDraw('Traj', COMx100, 'goal2', 'blueCircle', thickness, 'QuadListY')
	end
	self.COMtraj=COMtraj

	self.rotY=rotY

	local motionFrameRate=input.motionFrameRate

	do
		self.input=input

		self.dmot=self.mot:calcDerivative(motionFrameRate)
		
		if input.doNotUseOffsetQ then
			local startF=input.startF local endF=input.endF
			local COMori=quaterN(endF-startF)
			for i=startF, endF-1 do COMori(i-startF):identity() end
			self.offsetQ= {startF, COMori}
		elseif input.useIdentityOri then
			local startF=input.startF local endF=input.endF
			local COMori=quaterN(endF-startF)
			for i=startF, endF-1 do COMori(i-startF):assign(rotY(i):inverse()) end
			self.offsetQ= {startF, COMori}
		elseif input.usePelvisOri then
			local startF=input.startF local endF=input.endF
			local COMori=quaterN(endF-startF)
			for i=startF, endF-1 do 
				COMori(i-startF):assign(self.mot:row(i):toQuater(3)) 
			end
			if input.filterCDMori then
				math.filterQuat(COMori:matView(), input.filterCDMori)
			end
			-- remove rotY component from global COMori
			for i=startF, endF-1 do
				COMori(i-startF):assign(rotY(i):inverse()*COMori(i-startF))
			end
			self.offsetQ= {startF, COMori}
		elseif input.usePendulum then
			local delta=7
			local frames=CT.colon(0, self.COMtraj:rows(), delta)
			function getSpline(frames, mat, calcAcc)
				local out=matrixn(frames(frames:size()-1)+1, mat:cols())
				local acc=_getSpline(frames, mat, out, calcAcc)
				return out, acc
			end
			function _getSpline(frames, mat, matout, calcAcc)

				local timing=frames
				if dbg.lunaType(frames)=='intvectorn' then
					timing=frames:asFloat()
				end
				local curveFit=math.NonuniformSpline(timing, mat)

				local nf=timing(timing:size()-1)+1
				local newTiming=CT.linspace(timing(0), nf-1, nf)
				curveFit:getCurve(newTiming, matout)

				if calcAcc then
					local matAcc=matrixn()
					curveFit:getSecondDeriv(newTiming, matAcc)
					return matAcc
				end
			end
			local controlpoints=matrixn()
			controlpoints:extractRows(self.COMtraj:matView(), frames)
			local newCurve, newAcc=getSpline(frames, controlpoints, true)
			if input.filterCDMacc then
				math.filter(newAcc, input.filterCDMacc)
			end
			local leanweight=40
			if false then
				local lines=vector3N()
				for i=0, newAcc:rows()-1 do
					lines:pushBack(newCurve:row(i):toVector3(0))

					local footpos=newCurve:row(i):toVector3(0)+newAcc:row(i):toVector3(0)*-leanweight
					footpos.y=0
					lines:pushBack(footpos)
				end

				dbg.timedDraw(15, 'Traj', lines:matView()*100,  'solidred', 0, 'LineList' )
			end
			--dbg.draw('Traj', lines:matView()*100,  'pendulum', 'solidred', 0, 'LineList' )

			local startF=input.startF local endF=input.endF
			local COMori=quaterN(endF-startF)
			for i=startF, endF-1 do 
				local lean=quater()
				local ii=math.min(i, newCurve:rows()-1)
				local footpos=newCurve:row(ii):toVector3(0)+newAcc:row(ii):toVector3(0)*-leanweight
				footpos.y=0
				local footToCOM= newCurve:row(ii):toVector3(0)-footpos

				lean:axisToAxis(vector3(0,1,0), footToCOM)
				COMori(i-startF):assign(lean*rotY(i))
			end
			if input.filterCDMori then
				math.filterQuat(COMori:matView(), input.filterCDMori)
			end
			-- remove rotY component from global COMori
			for i=startF, endF-1 do
				COMori(i-startF):assign(rotY(i):inverse()*COMori(i-startF))
			end
			self.offsetQ= {startF, COMori}
		else
			-- momentum-based computation of offset q.
			self.offsetQ=self:computeOffsetQ_MM( input.startF, input.endF, input.m1, input.m2, rotY, COMtraj, input)
		end


		-- remove global rotY and COM position components from self.mot
		for i=0, self.mot:numFrames()-1 do
			local roottf=MotionDOF.rootTransformation(self.mot:row(i))
			local comtf=transf(rotY:row(i), COMtraj:row(i))
			--comtf.translation.y=0
			-- root=com*delta
			local localtf=comtf:inverse()*roottf
			localtf.rotation:align(quater(1,0,0,0))
			MotionDOF.setRootTransformation(self.mot:row(i), localtf)

			if input.offsetPose then
				self.mot:row(i):assign(self.mot:row(i)+input.offsetPose)
			end
		end



	end

	do
		local minf=input.startF
		local maxf=input.endF

		local rootTraj=matrixn(maxf-minf,7)

		local function getOffsetQ(iframe)
			local oq=self.offsetQ
			return oq[2]:row(iframe+oq[1])
		end

		for i=minf, maxf-1 do
			rootTraj:row(i-minf):setVec3(0, self.COMtraj(i))

			local qO=getOffsetQ(i)
			local qY=self.rotY(i)
			rootTraj:row(i-minf):setQuater(3, qY*qO)
		end
		if debugDraw then
			for i=minf, maxf-1 do
				local tf=MotionDOF.rootTransformation(rootTraj:row(i-minf))
				dbg.draw('Axes', tf, 'rootcom_'..i, 100,0.5)
			end
		end
		local dmot=MotionDOF.calcDerivative(rootTraj, input.motionFrameRate)

		if false then
			local tf=MotionDOF.rootTransformation(rootTraj:row(0))
			--tf.translation.z=tf.translation.z+0.05
			for i=minf, maxf-2 do
				dbg.draw('Axes', tf, 'rootcomi_'..i, 100,0.5)
				tf:integrateBodyVel(dmot:row(i-minf):toVector3(4), dmot:row(i-minf):toVector3(0), 1/30)
				--tf:integrate({w=dmot:row(i-minf):toVector3(4), v=dmot:row(i-minf):toVector3(0)}, 1/30)
			end
		end

		self.rootTraj={minf, rootTraj, dmot}
	end
end

function fc.CDMTraj:exportTraj(fn)
	local out={ self.rotY, self.offsetQ, self.COMtraj}
	util.saveTable(out, fn)
end
function fc.CDMTraj:importTraj(fn)
	local tbl=util.loadTable(fn)
	self.rotY=tbl[1]
	self.offsetQ=tbl[2]
	self.COMtraj=tbl[3]
end

function fc.extractCOMroot(loader, mot, bones, inputm)
	local COMtraj=vector3N(mot:numFrames())
	for currFrame=0, mot:numFrames()-1 do
		loader:setPoseDOF(mot:row(currFrame))
		COMtraj(currFrame):assign( loader:calcCOM())
	end
	local global_positions=mot:extractGlobalPositionsFromVoca(
	MotionLoader.LEFTSHOULDER, MotionLoader.RIGHTSHOULDER,
	MotionLoader.LEFTHIP, MotionLoader.RIGHTHIP
	)
    local across = (
        (global_positions:page(0) - global_positions:page(1)) + 
        (global_positions:page(2) - global_positions:page(3)))

	local forwardDir=across:vec3ViewCol(0):copy()

	for i=0, forwardDir:size()-1 do
		local v=forwardDir(i)
		v:cross(v:copy(), vector3(0,1,0))
		v.y=0
		v:normalize()
	end

	math.gaussFilter(inputm.filterFD or 20, forwardDir:matView())

	local rotY=quaterN(mot:numFrames())
	for currFrame=0, mot:numFrames()-1 do
		rotY(currFrame):setAxisRotation(vector3(0,1,0), vector3(0,0,1), forwardDir(currFrame))
	end
	return COMtraj, rotY
end

-- COMtraj [0, nf]
--
-- startF                endF
--   |                     |
--      |               |
--      m1             m2                 m1, m2 에서 COMori는 rotY.
function fc.CDMTraj:computeOffsetQ_MM(startF, endF, m1, m2, rotY, COMtraj, input)
	local loader=self.loader

	loader:setPoseDOF(self.mot:row(startF))

	--local simulator=Physics.DynamicsSimulator_TRL_QP('libccd')
	local simulator=Physics.DynamicsSimulator_TRL_LCP('libccd')
	simulator:registerCharacter(loader)
	simulator:init(1/30, Physics.DynamicsSimulator.EULER)

	local mMotionDOF=self.mot
	local dmot

	if not model or model.useOriginalFullbodyVelocity then
		dmot=self.dmot:copy()
	else
		-- uses model.frame_rate
		dmot=calcDerivative(mMotionDOF)

		-- heuristic. reduce theta_d by factor of 4 for less rotational velocity of the CDM.
		dmot:rmult(1.0/4.0)  

		-- this dmot is used only for baseline-QP-controls which are later modified by actions in RL anyway.
	end

	local tf=transf(quater(1,0,0,0), COMtraj(startF))
	local COMori=quaterN(endF-startF)

	--dbg.draw('Axes', tf, 'root0', 100)
	for i=startF, endF-1 do
		simulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, mMotionDOF:row(i))
		simulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, dmot:row(i))
		simulator:initSimulation() -- very important

		local vel=simulator:calcMomentumCOM(0)

		local I=vectorn()
		simulator:calcInertia(0, mMotionDOF:row(i), I)
		
		local m=I(6)
		local r=vector3(I(7), I(8), I(9))	
		local I6=CT.mat(6,6,I(0),I(3), I(4), 0, -r.z, r.y,
		I(3),I(1), I(5), r.z, 0,  -r.x,
		I(4),I(5), I(2), -r.y, r.x, 0 ,
		0, r.z, -r.y, m,   0 , 0,
		-r.z,0,  r.x, 0,  m,  0,
		r.y, -r.x, 0, 0,  0,  m)

		invI=CT.inverse(I6)
		local M=vectorn(6)
		M:setVec3(0, vel:M())
		M:setVec3(3, vel:F())
		local v=(invI*M:column()):column(0):copy()
		local V=Liegroup.se3(v:toVector3(0), v:toVector3(3))
		local invR=tf.rotation:inverse()
		V.w:rotate(invR)
		V.v:rotate(invR)

		local cav=input.constantAngVel
		if cav and i>=cav[1] and i<cav[2] then
			V.w:assign(cav[3])
			--print(i,  tf.rotation:rotationAngleAboutAxis(vector3(1,0,0)))
		end

		COMori(i-startF):assign(tf.rotation)
		tf:integrate(V, 1/30)
	end

	if input.filterCDMori then
		--dbg.console()
		math.filterQuat(COMori:matView(), input.filterCDMori)
	end

	local q1=COMori(m1-startF):inverse()
	local q2=COMori(m2-startF):inverse()
	q1:align(quater(1,0,0,0))
	q2:align(q1)

	if input.removeAccumulatedError then	
		--dbg.console()
		-- remove accumulated error
		for i=startF, endF-1 do
			local COM=COMtraj(i)

			if i<m1 then
				COMori(i-startF):leftMult(q1)
			elseif i>=m2 then
				COMori(i-startF):leftMult(q2)
			else
				local q=quater()
				q:interpolate(sop.map(i, m1, m2, 0, 1), q1, q2)
				COMori(i-startF):leftMult(q)
			end
			local tf=transf(COMori(i-startF), COM)

			if debugDraw and math.fmod(i,3)==0 then
				dbg.draw('Sphere', COM*100, 'com'..i,'blue', 1 )
				dbg.draw('Axes', tf, 'root_'..i, 100,0.5)
			end
		end
	end
	-- remove rotY component from global COMori
	for i=startF, endF-1 do
		local q=rotY(i)
		-- q*local_oq=COMori ->
		-- local_oq=q:inverse()*COMori
		COMori(i-startF):assign(q:inverse()*COMori(i-startF))
	end
	return {startF, COMori}
end

function fc.CDMTraj:getOffsetQ(i)
	local startF=self.offsetQ[1]
	if i< startF then
		i=startF 
	end
	return self.offsetQ[2](i-startF)
end
function fc.CDMTraj:getCDMtraj(defaultLean)
	if not defaultLean then
		defaultLean=quater(1,0,0,0)
	end
	local out=matrixn(self.rotY:rows(), 7)
	for i=0, out:rows()-1 do
		local COMtraj=self.COMtraj
		MotionDOF.setRootTransformation(out:row(i), transf(self.rotY(i)*self:getOffsetQ(i)*defaultLean, COMtraj:row(i)))
	end
	return out
end

function fc.createIKsolver(loader, config)
	local out={}
	local mEffectors=MotionUtil.Effectors()
	local numCon=#config
	mEffectors:resize(numCon);
	out.loader=loader
	out.effectors=mEffectors
	out.numCon=numCon
	local kneeDofs=vectorn()

	for i=0, numCon-1 do
		local conInfo=config[i+1]
		local kneeInfo=1
		if #conInfo==2 then
			kneeInfo=0
		end
		mEffectors(i):init(loader:getBoneByName(conInfo[kneeInfo+1]), conInfo[kneeInfo+2])
		if kneeInfo~=0 then
			kneeDofs:pushBack(loader.dofInfo:startR(loader:getBoneByName(conInfo[kneeInfo]):treeIndex()))
		end
	end
	if kneeDofs:size()>0 then
		out.kneeDOFs=kneeDofs
	end
	--out.solver=MotionUtil.createFullbodyIk_MotionDOF_MultiTarget_lbfgs(loader.dofInfo, mEffectors,g_con);
	out.solver=MotionUtil.createFullbodyIk_MotionDOF_MultiTarget_lbfgs(loader.dofInfo);
	return out
end

-- prev_pose will be updated to be the next pose.
function fc.solveIK(mSolverInfo, prev_pose, dotpose, desired_pose, comdof, footdof, effWeights, config) 
	-- V: world momentum-mapping velocity error (relative to desired_pose at the current frame. not relative to (prev_pose+dotpose*dt)
	-- when V is zero, MM term has no effects
	local V=Liegroup.se3(comdof:toVector3(7), comdof:toVector3(10))
	
	RE.output2('V0', V.w, V.v)

	local hasCOM=1
	local hasMM=1
	local hasPoseCon=1
	local useEffYcon=1
	local wCOMy=0.0
	local wCOMxz=1.0
	local wFullVel=0.01
	local wFullVelPose=0.001
	local wFullVelPose2=0.005
	local wMM=0.1
	local frameRate=30
	if config then
		wCOMy=config.wCOMy or 0.0
		wCOMxz=config.wCOMxz or 1.0
		if config.wFullVel then wFullVel=config.wFullVel end
		if config.wFullVelPose then wFullVelPose=config.wFullVelPose wFullVelPose2=config.wFullVelPose*5 end
		if config.wFullVelPose2 then wFullVelPose2=config.wFullVelPose2 end
		if config.wMM then wMM=config.wMM end
		if config.hasCOM then hasCOM=config.hasCOM end
		if config.hasMM then hasMM=config.hasMM end
		if config.hasPoseCon then hasPoseCon=config.hasPoseCon end
		if config.useEffYcon then useEffYcon=config.useEffYcon end
		if config.frameRate then frameRate=config.frameRate end
	else
		config={}
	end

	local comtf=MotionDOF.rootTransformation(comdof)


	local pose=prev_pose

	local numCon=mSolverInfo.numCon
	if placeObstacle then
		numCon=numCon-2
	end
	local footPos=vector3N (numCon);

	for i=0, numCon-1 do
		footPos(i):assign(footdof:toVector3(3*i))
	end

	local mIK=mSolverInfo.solver
	local mEffectors=mSolverInfo.effectors


	--
	--local prev_roottf=MotionDOF.rootTransformation(pose)
	--local toLocal=prev_roottf:inverse()
	local toLocal=MotionDOF.rootTransformation(comdof):inverse()

	if not config.noPlannarProjection then
		toLocal.translation.y=0
		toLocal.rotation:assign(toLocal.rotation:rotationY())
	end

	local useHead=config.useHead or 1
	local hasVelCon=0
	if config.vel_con then
		hasVelCon=hasVelCon+#config.vel_con
	end

	if not config.v2_max then
		config.v2_max=math.max(V.v:length(), V.w:length(), 10)
	end
	if not config.effWeightsY then
		config.effWeightsY=0.1
	end
	local v2newpose=MotionDOF.calcVelocity(prev_pose, desired_pose, frameRate)
	v2newpose:clamp(config.v2_max)
	mIK:_changeNumEffectors(numCon)
	if effWeights then
	else
		effWeights=vectorn(numCon)
		effWeights:setAllValue(1.0)
	end
	for i=0, numCon-1 do mIK:_setEffectorWeight(i, math.pow(effWeights(i),config.effwPower or 2)*config.effWeightsY) end
	mIK:_changeNumConstraints(hasCOM+hasMM+useHead+hasPoseCon*3+useEffYcon+hasVelCon)

	local COM=comtf.translation
	if hasCOM==1 then
		mIK:_setCOMConstraint(0, toLocal*COM, wCOMxz,wCOMy,wCOMxz)
	end
	--if g_dbgdbg then dbg.console() end
	if hasMM==1 then
		-- because (initial pose)==pose==prev_pose+dotpose
		-- MM con needs to consider only the remaining V (in global)
		V:Ad_ori(toLocal.rotation)
		local dt=1/frameRate
		--local dt=1
		RE.output2('V', V.w, V.v, toLocal.rotation)
		mIK:_setMomentumConstraint(hasCOM, V.w*dt, V.v*dt, wMM)
	end
	mIK:setParam('damping_weight', 0,0)
	mIK:setParam('root_damping_weight', 0.01,1)

	if false then
		-- 모션 품질 나빠짐.
		-- clamp dotpose
		local thr=math.rad(10)
		for i=7, dotpose:size()-1 do
			if prev_pose(i)> desired_pose(i)+thr and dotpose(i)>0  then
				dotpose:set(i,0)
			end
			if prev_pose(i)< desired_pose(i)-thr and dotpose(i)<0  then
				dotpose:set(i,0)
			end
		end
	end
	local kneeDOFs=mSolverInfo.kneeDOFs
	if kneeDOFs then
		-- knee damping
		local thr=0
		for ii=0, kneeDOFs:size()-1 do
			i=kneeDOFs(ii) 

			if prev_pose(i)< thr and dotpose(i)<0  then
				dotpose:set(i,0)
			end
		end
	end

	local dotpose2=dotpose:copy()
	--dotpose2:setVec3(4, dotpose2:toVector3(4)+desired_pose:toQuater(3):inverse()*V.w)
	--use only y
	dotpose2:set(5, dotpose2(5)+(desired_pose:toQuater(3):inverse()*V.w).y)
	local nextPose1=MotionDOF.integrate(prev_pose, dotpose2, frameRate)
	local nextPose2=MotionDOF.integrate(prev_pose, v2newpose, frameRate)
	if false then
		if not g_debugskin then
			g_debugskin=RE.createVRMLskin(g_info.loader, false)
			g_debugskin:setScale(100,100,100)
			g_debugskin:setTranslation(100,0,0)
			g_debugskin2=RE.createVRMLskin(g_info.loader, false)
			g_debugskin2:setScale(100,100,100)
			g_debugskin2:setTranslation(200,0,0)
			g_debugskin3=RE.createVRMLskin(g_info.loader, false)
			g_debugskin3:setScale(100,100,100)
			g_debugskin3:setTranslation(300,0,0)
		end
		g_debugskin:setPoseDOF(nextPose1)
		g_debugskin2:setPoseDOF(prev_pose)
		g_debugskin3:setPoseDOF(desired_pose)

	end

	--dbg.namedDraw('Axes', prev_pose:toTransf(0), 'prev_pose', 100,0.5)
	--dbg.namedDraw('Axes', nextPose1:toTransf(0), 'nextpose1', 100,0.5)
	--dbg.namedDraw('Axes', nextPose2:toTransf(0), 'nextpose2', 100,0.5)

	-- IK initial solution
	pose:assign(nextPose1)

	local tf=transf()
	local tf1=nextPose1:toTransf(0)
	local tf2=nextPose2:toTransf(0)
	tf:interpolate(0.9, tf1, tf2)
	pose:setTransf(0, tf)

	local  headPos, headBone
	if useHead==1 then
		local loader=mSolverInfo.loader
		--local headRefPose=loader.dofInfo:blendPose( nextPose1, nextPose2, 0.1)
		--loader:setPoseDOF(headRefPose)
		headBone=loader:getBoneByName(input.head[1])
		--headOri=headBone:getFrame().rotation:copy()
		--local COMtoHead=headBone:getFrame()*input.head[2]- loader:calcCOM()
		local COMtf=MotionDOF.rootTransformation(comdof)
		headPos=COMtf*input.head[3]
		--dbg.draw('Sphere', headPos*100, 'head')
	end
	MotionDOF.setRootTransformation(pose, toLocal*MotionDOF.rootTransformation(pose))
	MotionDOF.setRootTransformation(nextPose1, toLocal*MotionDOF.rootTransformation(nextPose1))
	MotionDOF.setRootTransformation(nextPose2, toLocal*MotionDOF.rootTransformation(nextPose2))

	if hasPoseCon==1 then
		--for i=1, mMot.loader:numBone()-1 do print(i,mMot.loader:bone(i):name()) end
		mIK:_setPoseConstraint(hasCOM+hasMM, nextPose1, wFullVel, 2)  -- fullbody velocity constraint
		mIK:_setPoseConstraint(hasCOM+hasMM+1, nextPose2, wFullVelPose, 2)  -- fullbody poseconstraint
		if not input.lowerbody then
			input.lowerbody={2,10000}
		end
		mIK:_setPoseConstraint(hasCOM+hasMM+2, nextPose2, wFullVelPose2, input.lowerbody[1],input.lowerbody[2] ) -- lowerbody pose constraint
	end
	if config.debugDraw then
		for c=0,numCon-1 do
			dbg.namedDraw('Sphere', footPos(c)*100, 'feet'..c)
		end
	end
	for i=0,numCon-1 do
		mIK:_setEffector(i, mEffectors(i).bone, mEffectors(i).localpos)
		footPos(i):assign(toLocal*footPos(i))
	end
	if useHead==1 then
		local i=numCon
		mIK:_setPositionConstraint(hasCOM+hasMM+hasPoseCon*3, headBone, input.head[2], toLocal*headPos, 1,config.wHead_y or 0,config.wHead_z or 0.1 );
	end

	if useEffYcon==1 then
		local ew=effWeights:copy()
		for i=0,ew:size()-1 do ew:set(i, math.pow(ew(i),config.effwPower or 2)) end

		mIK:_setEffectorYConstraint(hasCOM+hasMM+useHead+hasPoseCon*3, 1.0, ew)
	end

	if config.vel_con then
		for i,v in ipairs(config.vel_con) do
			local loader=mSolverInfo.loader
			local bone=loader:getBoneByName(v[1][1])
			local lpos=v[1][2]
			local gpos=v[2]
			local offset=v[3]
			local normal=offset:copy()
			normal:normalize()
			mIK:_setNormalConstraint(hasCOM+hasMM+useHead+hasPoseCon*3+useEffYcon+i-1, bone, lpos, toLocal.rotation*normal, toLocal*(gpos+offset*1/frameRate))
		end
	end

	mIK:_effectorUpdated()
	mIK:IKsolve(pose, footPos)



	MotionDOF.setRootTransformation(pose, toLocal:inverse()*MotionDOF.rootTransformation(pose))
	assert(prev_pose==pose)

	--MotionDOF.setRootTransformation(nextPose1, toLocal:inverse()*MotionDOF.rootTransformation(nextPose1)) pose:assign(nextPose1)
	--MotionDOF.setRootTransformation(nextPose2, toLocal:inverse()*MotionDOF.rootTransformation(nextPose2)) pose:assign(nextPose2)
end
function fc.solveIK_postprocess(mSolverInfo, prev_pose, comdof, footdof, effWeights, config) 
	local pose=prev_pose

	local numCon=mSolverInfo.numCon
	local footPos=vector3N (numCon);

	for i=0, numCon-1 do
		footPos(i):assign(footdof:toVector3(3*i))
	end

	local mIK=mSolverInfo.solver
	local mEffectors=mSolverInfo.effectors
	--local prev_roottf=MotionDOF.rootTransformation(pose)
	--local toLocal=prev_roottf:inverse()
	local toLocal=MotionDOF.rootTransformation(comdof):inverse()
	toLocal.translation.y=0
	toLocal.rotation:assign(toLocal.rotation:rotationY())


	local hasCOM=1
	local hasMM=0
	local wCOMy=0.0
	local wMM=0.1
	local frameRate=30
	if config then
		wCOMy=config.wCOMy or 0.0
		if config.wMM then wMM=config.wMM end
		if config.hasCOM then hasCOM=config.hasCOM end
		if config.hasMM then hasMM=config.hasMM end
		if config.frameRate then frameRate=config.frameRate end
	else
		config={}
	end
	local hasVelCon=0
	if config.vel_con then
		hasVelCon=hasVelCon+#config.vel_con
	end


	local useHead=config.useHead or 1

	if not config.effWeightsY then
		config.effWeightsY=0.1
	end
	mIK:_changeNumEffectors(numCon)
	if effWeights then
	else
		effWeights=vectorn(numCon)
		effWeights:setAllValue(1.0)
	end
	for i=0, numCon-1 do mIK:_setEffectorWeight(i, math.pow(effWeights(i),2)*config.effWeightsY) end
	mIK:_changeNumConstraints(hasCOM+hasMM+useHead+1+hasVelCon)

	local COM
	do
		local loader=mSolverInfo.loader
		loader:setPoseDOF(prev_pose)
		COM=loader:calcCOM()
	end

	if hasCOM==1 then
		mIK:_setCOMConstraint(0, toLocal*COM, 1,wCOMy,1)
	end
	--if g_dbgdbg then dbg.console() end
	if hasMM==1 then
		mIK:_setMomentumConstraint(hasCOM, vector3(0,0,0), vector3(0,0,0), wMM)
	end
	mIK:setParam('damping_weight', 0.01,0.01)

	local  headPos, headBone
	if useHead==1 then
		local loader=mSolverInfo.loader
		--local headRefPose=loader.dofInfo:blendPose( nextPose1, nextPose2, 0.1)
		--loader:setPoseDOF(headRefPose)
		headBone=loader:getBoneByName(input.head[1])
		headPos=headBone:getFrame()*input.head[2]
	end
	MotionDOF.setRootTransformation(pose, toLocal*MotionDOF.rootTransformation(pose))

	for i=0,numCon-1 do
		mIK:_setEffector(i, mEffectors(i).bone, mEffectors(i).localpos)
		footPos(i):assign(toLocal*footPos(i))
	end
	if useHead==1 then
		local i=numCon
		mIK:_setPositionConstraint(hasCOM+hasMM, headBone, input.head[2], toLocal*headPos, 1,config.wHead_y or 0,0.1 );
	end
	local ew=effWeights:copy()
	for i=0,ew:size()-1 do ew:set(i, math.pow(ew(i),2)) end
	
	mIK:_setEffectorYConstraint(hasCOM+hasMM+useHead, 1.0, ew)

	if config.vel_con then
		for i,v in ipairs(config.vel_con) do
			local loader=mSolverInfo.loader
			local bone=loader:getBoneByName(v[1][1])
			local lpos=v[1][2]
			local gpos=v[2]
			local offset=v[3]
			local normal=offset:copy()
			normal:normalize()
			mIK:_setNormalConstraint(hasCOM+hasMM+1+i, bone, lpos, toLocal.rotation*normal, toLocal*(gpos+offset*1/frameRate))
		end
	end

	mIK:_effectorUpdated()
	mIK:IKsolve(pose, footPos)

	MotionDOF.setRootTransformation(pose, toLocal:inverse()*MotionDOF.rootTransformation(pose))
	assert(prev_pose==pose)
end

-- input: self.loader_ref_fullbody, self.loader_ref, self.mot, self.CDMtraj, input, model.touchDown, model.touchOff
-- output: self.motionDOF_*, self.DMotionDOF,  self.touchDown, self.touchOff, self.con
function fc.prepareMotionsForSim(self, initial_height, model) 
	local loader=self.loader_ref_fullbody
	local cdmLoader=self.loader_ref
	self.motionDOF_original=self.mot

	assert(loader)
	assert(cdmLoader)
	assert(self.mot)

	local mMotionDOF=self.motionDOF_original
	for i=0, mMotionDOF:rows()-1 do
		mMotionDOF:matView():set(i,1, mMotionDOF:matView()(i,1)+initial_height)
	end
	self.touchDown={
		boolN(),
		boolN(),
	}

	for i=1,2 do
		local td= self.touchDown[i]
		td:resize(mMotionDOF:rows())
		td:setAllValue(false)
		for k,v in ipairs(model.touchDown[i]) do
			if v<td:size() then td:set(v, true) end
		end
	end

	self.touchOff={
		boolN(),
		boolN(),
	}
	for i=1,2 do
		local td= self.touchOff[i]
		td:resize(mMotionDOF:rows())
		td:setAllValue(false)
		for k,v in ipairs(model.touchOff[i]) do
			if v<td:size() then td:set(v, true) end
		end
	end

	self.con=fc.buildContactConstraints(mMotionDOF:numFrames(), model.touchDown, model.touchOff)

	self.motionDOF_cdm=MotionDOF(cdmLoader.dofInfo)
	self.motionDOF_cdm:resize(mMotionDOF:numFrames())

	local defaultLeaning=model.defaultCDMlean or quater(0.28*0.5,vector3(1,0,0))
	self.motionDOF_cdm:matView():assign( self.CDMtraj:getCDMtraj(defaultLeaning)) -- use CDM trajectory
	--self.motionDOF_cdm:matView():assign( mMotionDOF:matView():sub(0,0,0,7)) -- use root trajectory
	

	self.DMotionDOF=calcDerivative(self.motionDOF_cdm) -- uses model.frame_rate
	if model.loopMotion then
		local motdof=self.motionDOF_cdm:copy()
		local motdof_fullbody=self.motionDOF_original
		local tdo={
			self.touchDown[1]:copy(),
			self.touchDown[2]:copy(),
			self.touchOff[1]:copy(),
			self.touchOff[2]:copy(),
		}
		tdo[5]=self.con[1]:copy()
		tdo[6]=self.con[2]:copy()

		self.motionDOF_cdm=motdof:copy()
		self.motionDOF_fullbody=motdof_fullbody:copy()
		self.motionDOF_iframe=CT.colon(0, motdof:rows(), 1)

		local dmot1=self.DMotionDOF:sub(1,0,0,0):copy()

		local rotYref
		if self.rotYref then
			rotYref=self.rotYref:sub(1,0,0,0):copy()
		end

		local limitLength=randomRestartInfo.limitLength or (motdof:length()-2)
		repeat
			if true then
				local motA=self.motionDOF_fullbody:copy()
				local motB=motdof_fullbody:copy()

				local delta=transf()
				delta:difference(motB:row(0):toTransf():project2D(), motA:row(motA:rows()-1):toTransf():project2D())
				--delta.rotation:assign(delta.rotation:rotationY())
				delta.translation.y=0

				local function align(out, motA, motB, delta)
					out:changeLength(motA:length()+motB:length())
					out:range(0, motA:rows()):assign(motA)
					out:range(motA:rows(), out:rows()):assign(motB:range(1,motB:rows()))
					out:range(motA:rows(), out:rows()):transform(delta)
				end
				align(self.motionDOF_fullbody,motA, motB, delta)
				align(self.motionDOF_cdm, self.motionDOF_cdm:copy(), motdof, delta)
			else
				self.motionDOF_fullbody:align(self.motionDOF_fullbody:copy(), motdof_fullbody)
				self.motionDOF_cdm:align(self.motionDOF_cdm:copy(), motdof)
			end

			self.DMotionDOF=self.DMotionDOF:concat(dmot1)
			self.motionDOF_iframe=self.motionDOF_iframe..CT.colon(1, motdof:rows(), 1)

			if self.rotYref then
				self.rotYref =self.rotYref:concat(rotYref)
			end
			local pnf=self.touchDown[1]:size()
			local tdo1=tdo[1](0) self.touchDown[1]:concat(tdo[1]:range(1, tdo[1]:size())) if tdo1 then self.touchDown[1]:set(pnf, true) end
			local tdo2=tdo[2](0) self.touchDown[2]:concat(tdo[2]:range(1, tdo[2]:size())) if tdo2 then self.touchDown[1]:set(pnf, true) end

			self.touchOff[1]:concat(tdo[3]:range(1, tdo[3]:size()))
			self.touchOff[2]:concat(tdo[4]:range(1, tdo[4]:size()))

			self.con[1]:concat(tdo[5]:range(1,tdo[5]:size()))
			self.con[2]:concat(tdo[6]:range(1,tdo[6]:size()))
		until self.motionDOF_cdm:length()>  (limitLength+2)*3
		assert(self.touchDown[1]:size()==self.motionDOF_cdm:rows())
		assert(self.motionDOF_iframe:size()==self.motionDOF_cdm:rows())
	else
		local motdof_fullbody=self.motionDOF_original
		self.motionDOF_fullbody=motdof_fullbody:copy()
		self.motionDOF_iframe=CT.colon(0, motdof_fullbody:rows(), 1)
	end
	-- convert to vectorn
	do
		local backup=self.motionDOF_iframe
		self.motionDOF_iframe=vectorn()
		self.motionDOF_iframe:assign(backup)
	end

	self.conWeight={}
	local mocapOffset=nil
	if model.option_refCDMtraj and model.option_refCDMtraj.useMocapOffset then
		local nf=self.con[1]:size()
		mocapOffset={ vector3N(nf), vector3N(nf)}
	end
	local feet_config={
		-- this local positions have to be same as those defined in showHuman.lua (input.limbs[1][3] and input.limbs[2][3]))
		toe=input.limbs[1][3], 
		heel=input.limbs[2][3],
	}
	for ilimb=1, 2 do
		local nf=self.con[ilimb]:size()
		self.conWeight[ilimb]=CT.ones(nf)
		local segments=intIntervals()
		segments:runLengthEncode(self.con[ilimb])
		for iseg=0, segments:numInterval()-1 do
			local startSwing=0
			local endSwing=0
			if iseg==segments:numInterval()-1 then
				startSwing=segments:endI(iseg)
				endSwing=segments:startI(0)+nf-1
			else
				startSwing=segments:endI(iseg)
				endSwing=segments:startI(iseg+1)
			end
			for i=startSwing, endSwing-1 do
				local w=sop.map(i, startSwing-2, endSwing, -1, 1) -- w decreases early for better motion style.
				local wmap=model.conWeightMap or function(w, i, startSwing, endSwing) return math.pow(w,4) end
				if i<nf then
					self.conWeight[ilimb]:set(i, wmap(w, i, startSwing, endSwing))
				else
					self.conWeight[ilimb]:set(i-nf, wmap(w, i, startSwing, endSwing))
				end
			end

			if mocapOffset then
				local iframe=segments:startI(iseg)
				loader:setPoseDOF(self.motionDOF_fullbody:row(iframe))
				local gf=self.motionDOF_cdm:row(iframe):toTransf(0):project2D()

				local gpos
				local lpos=feet_config.toe*0.5+feet_config.heel*0.5
				if ilimb==1 then
					gpos=loader:getBoneByVoca(MotionLoader.RIGHTANKLE):getFrame()*lpos
				else
					gpos=loader:getBoneByVoca(MotionLoader.LEFTANKLE):getFrame()*lpos
				end


				local offset=gf:toLocalPos(gpos)
				offset.y=0
				--print('offset', offset, gf.translation, gpos)
				--dbg.draw("Sphere", gpos*100+vector3(100,0,0), 'gpos'..iframe..'_'..ilimb, 'red',10)
				--dbg.draw("Sphere", gf.translation*100+vector3(100,0,0), 'gf'..iframe..'_'..ilimb, 'blue',10)

				mocapOffset[ilimb]:range(iframe, segments:endI(iseg)):setAllValue(offset)

				if iseg==0 then
					for i=0, iframe-1 do
						mocapOffset[ilimb](i):assign(offset)
					end
				else
					--mocapOffset[ilimb]:range(segments:endI(iseg-1), iframe):setAllValue(offset)
					local prevOffset=mocapOffset[ilimb](segments:endI(iseg-1)-1)
					local s=segments:endI(iseg-1)
					local e=iframe
					for i=s,e-1 do
						mocapOffset[ilimb](i):interpolate(sop.smoothMap(i,s, e, 0,1), prevOffset, offset)
					end


					if iseg==segments:numInterval()-1 then
						mocapOffset[ilimb]:range(segments:endI(iseg), nf):setAllValue(offset)
					end
				end

			end
		end
	end
	self.mocapFeetOffset=mocapOffset
	--fc.removeFeetSliding(self.con, feet_config, loader, self.motionDOF_fullbody)
end

local function diffPose(a, b)
	local pose= b-a
	local qd=quater()
	qd:difference(a:toQuater(3), b:toQuater(3))
	pose:setQuater(3, qd)
	return pose
end
local function addPose(a, b)
	local pose= a+b
	local q=b:toQuater(3)*a:toQuater(3)
	pose:setQuater(3, q)
	return pose
end
local function scalePose(a,s)
	local pose= a*s
	local q=a:toQuater(3)
	q:scale(s)
	pose:setQuater(3, q)
	return pose
end
fc.diffPose=diffPose
fc.addPose=addPose
fc.scalePose=scalePose
fc.blendPose=blendPose
fc.midPos=midPos
return fc
