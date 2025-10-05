History={}
function History.saveRoot(g_history,pi, dtheta, roottf)
	if not g_history.rootTrajectory then
		g_history.rootTrajectory=matrixn()
		g_history.dtheta=vector3N()
		g_history.phases=matrixn()
		g_history.dist_z=vectorn()
		g_history.state=vectorn()
		g_history.leginfo={matrixn(), matrixn(), matrixn(), matrixn()}
		g_history.cf={matrixn(), matrixn(), matrixn(), matrixn()}
		g_history.input={matrixn(), matrixn(), matrixn(), matrixn()} -- pose, dpose, footdof, comdof
		g_history.pendstate=matrixn()
	end
	g_history.dtheta:pushBack(pi:toYUP(dtheta:toVector3(0)))
	g_history.state:pushBack(g_currState or 0)
	local pose=vectorn(7)
	MotionDOF.setRootTransformation(pose, roottf)
	g_history.rootTrajectory:pushBack(pose)

	local comdof=vectorn(7+6)
	MotionDOF.setRootTransformation(comdof,roottf)
	if dtheta:size()==3 then
		-- mmpm
		comdof:setVec3(7, vector3(0,0,0))
	else
		comdof:setVec3(7, pi:toYUP(dtheta:toVector3(3)))
	end
	comdof:setVec3(10, pi:toYUP(dtheta:toVector3(0)))
	g_history.input[4]:pushBack(comdof)

	return comdof
end
local function bton(a)
	if a then return 1 end
	return 0
end
function History.saveLegInfo(g_history, ileg, t, pi, footMidPos, rotY, foot)
	local leginfo= g_history.leginfo[ileg]

	local v=vectorn(12)
	local q=vectorn(16)
	if foot then
		v:setVec3(0, pi.footsteps[foot][1](0))
		v:setVec3(3, pi.footsteps[foot][1](1))
		v:setVec3(6, pi.footsteps[foot][1](2))
		q:setQuater(0, pi.footsteps[foot][2](0))
		q:setQuater(4, pi.footsteps[foot][2](1))
		q:setQuater(8, pi.footsteps[foot][2](2))
		if pi.footsteps[foot][1]:size()==3 then
			v:setVec3(9, vector3(0,0,0)) -- unused
			q:setQuater(12, quater(1,0,0,0)) -- unused
		else
			v:setVec3(9, pi.footsteps[foot][1](3))
			q:setQuater(12, pi.footsteps[foot][2](3))
		end
	else
		--assert(pi.footsteps[1]:size()>3)
		--assert(pi.footsteps[2]:size()>3)
		v:setVec3(0, pi.footsteps[1](0))
		v:setVec3(3, pi.footsteps[1](1))
		v:setVec3(6, pi.footsteps[1](2))
		q:setQuater(0, pi.footsteps[2](0))
		q:setQuater(4, pi.footsteps[2](1))
		q:setQuater(8, pi.footsteps[2](2))
		if pi.footsteps[1]:size()==3 then
			v:setVec3(9, vector3(0,0,0)) -- unused
			q:setQuater(12, quater(1,0,0,0)) -- unused
		else
			v:setVec3(9, pi.footsteps[1](3))
			q:setQuater(12, pi.footsteps[2](3))
		end
	end
	local p=vectorn(7)
	p:setVec3(0, footMidPos)
	p:setQuater(3, rotY)
	leginfo:pushBack(CT.vec(t, t, footMidPos.y, bton(pi.isL))..v..q..p)
end

function History.saveCF(g_history, pi, ifoot, cf, contact)
	local cfinfo= g_history.cf[ifoot+1]
	local v=vectorn(4)
	v:setVec3(0, pi:toYUP(cf))
	v:set(3, bton(contact))
	cfinfo:pushBack(v)
end
-- example_config= {dist_z_thr=dist_z_thr, hip_sf=0.6, }
function History.samplePose(g_history, iframe, config)
	local roottf=MotionDOF.rootTransformation(g_history.rootTrajectory:row(iframe))
	local dist_z=g_history.dist_z(iframe)
	local stride_phases={ g_history.stridePhase[1](iframe,0), g_history.stridePhase[2](iframe,0) }
	pose, avgPose=mSampler:sampleStridePose(stride_phases, dist_z, config)

	pose:setVec3(0, roottf.translation+pose:toVector3(0))
	pose:setQuater(3, roottf.rotation*pose:toQuater(3))
	return pose
end

-- returns roottf
function History.loadRootAndLegInfo(g_history, iframe, pi, sepLegInfo)
	local roottf=MotionDOF.rootTransformation(g_history.rootTrajectory:row(iframe))
	pi.dist_z=g_history.dist_z(iframe)
	RE.output2('dist_z', pi.dist_z )
	pi.footsteps={}

	local tt
	for ileg=1,pi.nLimb do
		local leginfo= g_history.leginfo[ileg]:row(iframe)
		--
		--pi.lastContactIndices[ileg]=leginfo(0)
		--leginfo=leginfo:slice(1,0)
		tt=leginfo(0)

		local c=4
		local v=leginfo:range(c, c+12) c=c+12

		local q=leginfo:range(c, c+16) c=c+16
		local v3=vector3N(4)
		v3(0):assign(v:toVector3(0))
		v3(1):assign(v:toVector3(3))
		v3(2):assign(v:toVector3(6))
		v3(3):assign(v:toVector3(9))
		local q3=quaterN(4)
		q3(0):assign(q:toQuater(0))
		q3(1):assign(q:toQuater(4))
		q3(2):assign(q:toQuater(8))
		q3(3):assign(q:toQuater(12))
		if sepLegInfo then
			if not pi.footsteps then
				pi.footsteps={}
			end
			pi.footsteps[ileg]={v3, q3}
		else
			pi.footsteps={v3, q3}
		end
	end

	return roottf,tt
end

function History.checkInput(g_history)
	if not g_history.input then
		g_history.input={matrixn(), matrixn(), matrixn(), matrixn()}
		local dtheta=MotionDOF.calcDerivative(g_history.rootTrajectory,30)
		MotionDOF.convertBodyVelToGlobal(g_history.rootTrajectory, dtheta)

		for i=0, g_history.rootTrajectory:rows()-1 do
			local comdof=vectorn(7+6)
			local roottf=MotionDOF.rootTransformation(g_history.rootTrajectory:row(i))
			MotionDOF.setRootTransformation(comdof,roottf)
			comdof:setVec3(7, dtheta:row(i):toVector3(4))
			comdof:setVec3(10, dtheta:row(i):toVector3(0))
			g_history.input[4]:pushBack(comdof)
		end

	end
end
function History.fixSwingPhase()
	if true then
		-- filter swing phase

		local nlegs=g_history.phases:cols()
		for ileg=0,nlegs-1 do
			local swingStart=-1
			local phases=g_history.phases
			for i=0, phases:rows()-2 do
				local pp=phases(i, ileg)
				local pn=phases(i+1, ileg)
				if pp<3 and pn>=3 then
					swingStart=i+1
				elseif pn<pp then
					swingEnd=i+1
					if swingStart<0 then swingStart=0 end
					local p1=phases(swingStart, ileg)
					local p2=phases(swingEnd-1, ileg)

					if p2>=1.5 and p2<1.5+1e-3 then
						-- standing detected
					else
						assert(swingEnd<=phases:rows())
						if swingEnd>swingStart+1 then
							phases:column(ileg):range(swingStart, swingEnd):linspace(p1,p2)
						end
					end
				end
			end
		end
	end

end
function History.annotateStridePhase(g_history, config)
	if type(config)~='table' then
		config={ containsLongJumps=config,
		containsShortJumps=not config}
	end

	local phases=g_history.phases
	local stridePhase={matrixn(phases:rows(), 5), matrixn(phases:rows(),5)}  -- L,R

	local nlegs=g_history.phases:cols()
	if nlegs==4 then
		stridePhase[3]=matrixn(phases:rows(), 5)
		stridePhase[4]=matrixn(phases:rows(), 5)
	end
	if config.containsLongJumps then
		g_history.anticipation=matrixn(phases:rows(),2)
		g_history.anticipation:setAllValue(0.0) -- w_fb, www
	end
	if config.getPrevStride then
		g_history.prevStride=matrixn(phases:rows(),nlegs)
		g_history.prevStride:setAllValue(0)
		g_history.prevStrideFrame=matrixn(phases:rows(),nlegs)
		g_history.prevStrideFrame:setAllValue(0)
	end
	-- stridePhase, swingStart, swingEnd, swing_y, effw
	local leadingLeg=nil
	local aPrevStride={}
	local aPrevStrideFrame={}
	local aPrevPrevStrideFrame={}
	local aSwingStart={}
	local aSwingEnd={}
	local aIsSpprt={}

	for ileg=0,nlegs-1 do
		aIsSpprt[ileg]=true
		aPrevStride[ileg]=0
		aPrevStrideFrame[ileg]=0
		aSwingStart[ileg]=0
		aSwingEnd[ileg]=nil
		local sp=stridePhase[ileg+1]
		sp:setAllValue(0)
	end
	local leadingLegSwingEnd=nil
	local followingLegSwingStart=nil

	local dist_z=vectorn(phases:rows())
	dist_z:setAllValue(0)
	local lastMidContactFrame=0
	for i=1, phases:rows()-2 do
		for ileg=0,nlegs-1 do

			local isSpprt=aIsSpprt[ileg]
			local prevStride=aPrevStride[ileg]
			local prevStrideFrame=aPrevStrideFrame[ileg]
			local swingStart=aSwingStart[ileg]
			local swingEnd=aSwingEnd[ileg]
			local sp=stridePhase[ileg+1]

			local pp=phases(i, ileg)
			local pn=phases(i+1, ileg)
			if pp<1.5 and pn>=1.5 then
				dist_z:range(lastMidContactFrame, i):setAllValue(g_history.rootTrajectory:row(lastMidContactFrame):toVector3(0):distance(g_history.rootTrajectory:row(i):toVector3(0)))
				lastMidContactFrame=i
				assert(isSpprt==true)

				local strideFrame=sop.map(1.5, pp, pn, i, i+1)
				local function hermite(t, p1, p4, r1, r4)
					return (2*t*t*t-3*t*t+1)*p1+(-2*t*t*t+3*t*t)*p4+(t*t*t-2*t*t+t)*r1+(t*t*t-t*t)*r4;
				end
		
				local function hermite(t, p1, p4, r1, r4)
					return (2*t*t*t-3*t*t+1)*p1+(-2*t*t*t+3*t*t)*p4+(t*t*t-2*t*t+t)*r1+(t*t*t-t*t)*r4;
				end
				local function hermite01(t, r1,r4)
					return (-2*t*t*t+3*t*t)+(t*t*t-2*t*t+t)*r1+(t*t*t-t*t)*r4;
				end
				local function hermite0_1p(t, r1,m, r4, p)
					p = p or 2
					if t<0.5 then
						return 0.5*math.pow(hermite01(t*2, r1,m),p)
					else
						return 0.5*math.pow(hermite01((t-0.5)*2, m, r4),p)+0.5
					end
				end
				local function pow01(t, mid1, mid2, p1, p2)
					if t<mid1 then
						return math.pow(t, p1)
					elseif t<mid2 then
						return sop.map(t, mid1, mid2, math.pow(mid1,p1), math.pow(mid2, p2))
					else
						return math.pow(t, p2)
					end
				end
				local function pow01y(t, mid1y, mid2y, p1, p2)
					-- math.pow(mid1, p1)==mid1y
					local mid1=math.pow(mid1y, 1/p1)
					local mid2=math.pow(mid2y, 1/p2)
					return pow01(t,mid1, mid2, p1,p2)
				end
				local function hermite0_1(t, r1,m, r4, mid, mid_x)
					mid_x = mid_x or 0.5
					if t<mid_x then
						return hermite(t/mid_x,0, mid, r1,m)
					else
						return hermite((t-0.5)/(1-mid_x),mid, 1, m, r4)
					end
				end
				local function arc1(frames)
					local res=CT.zeros(frames)
					for i=0, frames-1 do
						local y=sop.map(i,0, frames-1, -1, 1)
						y=1-y*y
						res:set(i,y)
					end
					return res
				end
				local function arc(frames, des_acc)
					local res=arc1(frames)
					local function calcAcc(res)
						local dt=1.0/30.0
						local acc=((res(2)-res(1))/dt-(res(1)-res(0))/dt)/dt
						return acc
					end
					local acc=calcAcc(res)
					res:rmult(des_acc/acc)
					return res
				end

				local dur=i+1-prevStrideFrame
				if config.containsLongJumps and dur>25 then
					local mid=0.57 -- max knee-up. see testPoseSampler
					if leadingLeg ==nil then
						-- 앞발
						if config.useLinearStrideCurve then
							for j=prevStrideFrame, i+1 do
								sp:set(j, 0, sop.map(j, prevStrideFrame, strideFrame, 0, 1))
							end
						else
							for j=prevStrideFrame, i+1 do
								--sp:set(j, 0, hermite0_1( sop.map(j, prevStrideFrame, strideFrame, prevStride, 1),3,-0.3,1.5, mid))
								--sp:set(j, 0, hermite0_1( sop.map(j, prevStrideFrame, strideFrame, 0, 1),3,-0.3,2, mid))
								sp:set(j, 0, pow01y(sop.map(j, prevStrideFrame, strideFrame, 0, 1),0.57, 0.78, 0.5,2.0))
								--sp:set(j, 0, hermite0_1p( sop.map(j, prevStrideFrame, strideFrame, prevStride, 1),2,0,3))
							end
						end
						leadingLeg=ileg
						leadingLegSwingEnd=i+1
						followingLegSwingStart=aPrevStrideFrame[math.fmod(ileg+1,2)]

						local s=math.ceil(followingLegSwingStart)
						local e=math.floor(swingEnd)
						local res=arc1(e-s+1)
						sp:column(3):range(s,e+1):assign(res)
						sp:column(4):range(s,e+1):assign(res)

					else
						if config.useLinearStrideCurve then
							for j=prevStrideFrame, i+1 do
								sp:set(j, 0, sop.map(j, prevStrideFrame, strideFrame, 0, 1))
							end
						else
							-- 뒷발
							mid=0.3
							mid_x=0.5
							for j=prevStrideFrame, i+1 do
								--sp:set(j, 0, hermite0_1( sop.map(j, prevStrideFrame, strideFrame, prevStride, 1),1,-0.5,1, mid,mid_x))
								--sp:set(j, 0, math.pow(sop.map(j, prevStrideFrame, strideFrame, 0, 1),0.5))
								--sp:set(j, 0, pow01(sop.map(j, prevStrideFrame, strideFrame, 0, 1),0.13, 0.63, 0.5,2.0))
								sp:set(j, 0, pow01y(sop.map(j, prevStrideFrame, strideFrame, 0, 1),0.36, 0.43, 0.5,1.01))
							end
						end


						-- swing_y
						assert(leadingLeg~=ileg)
						--local swingEndLeadingLeg=aSwingEnd[leadingLeg]
						--local swingEndLeadingLeg=leadingLegSwingEnd
						--local ii=math.ceil(swingEndLeadingLeg*0.3+i*0.7)
						local ii=math.ceil(swingEnd)
						assert(ii<=i+1)
						local res=arc1(ii-prevStrideFrame+1)
						sp:column(3):range(prevStrideFrame, ii+1):assign(res)
						--
						ii=math.ceil(leadingLegSwingEnd)
						assert(ii<=i+1)
						local res=arc1(ii-prevStrideFrame+1)

						local sf=aPrevPrevStrideFrame[leadingLeg]-4
						local prevPrevStrideFrame=aPrevPrevStrideFrame[leadingLeg]*0.9+aPrevPrevStrideFrame[ileg]*0.1
						local landingFrame=aPrevStrideFrame[leadingLeg]

						local ant=g_history.anticipation
						for ii=sf-2, strideFrame do
							local w_fb=fc.getLinearSplineABS(ii, tl.mat{ {sf-4,0}, {prevPrevStrideFrame,0}, {prevPrevStrideFrame*0.5+prevStrideFrame*0.5, 0.2}, { prevStrideFrame, 1}, {prevStrideFrame*0.6+landingFrame*0.4, 0}, {landingFrame, 0}})

							local www=fc.getLinearSplineABS(ii, tl.mat{ {sf-4,0}, {sf-2,0}, {prevPrevStrideFrame*0.8+prevStrideFrame*0.2, 1},  { prevStrideFrame, 1}, {prevStrideFrame*0.6+landingFrame*0.4, 0},{landingFrame,0}})

							ant:set(ii-2,0, w_fb)
							ant:set(ii-2,1, www)
						end

						sp:column(4):range(prevStrideFrame, ii+1):assign(res)
						leadingLeg=nil
					end

				elseif config.containsShortJumps and dur>15 then
					for j=prevStrideFrame, i+1 do
						sp:set(j, 0, 
						hermite(
						sop.map(j, prevStrideFrame, strideFrame, prevStride, 1),
						0,1,
						dur/15,
						dur/15
						))
					end

				else
					for j=prevStrideFrame, i+1 do
						sp:set(j, 0, sop.map(j, prevStrideFrame, strideFrame, prevStride, 1))
					end
				end
				aPrevPrevStrideFrame[ileg]=prevStrideFrame
				prevStrideFrame=i+1
				prevStride=sp(prevStrideFrame,0)-1
			elseif pp<3 and pn>=3 then
				swingStart=sop.map(3, pp, pn, i, i+1)
				assert(isSpprt)
				isSpprt=false
			elseif pn<pp then
				swingEnd=sop.map(4, pp, pn+4, i, i+1)
				assert(swingEnd<=i+1)
				assert(swingEnd>=i)
				for j=math.floor(swingStart), math.ceil(swingEnd) do
					sp:set(j, 1, swingStart)
					sp:set(j, 2, swingEnd)
				end
				isSpprt=true
				if pn>1.5 then
					phases:set(i+1, ileg, 1.499)
				end
			end
			aPrevStride[ileg]=prevStride
			aPrevStrideFrame[ileg]=prevStrideFrame
			aSwingStart[ileg]=swingStart
			aSwingEnd[ileg]=swingEnd
			aIsSpprt[ileg]=isSpprt

			if config.getPrevStride then
				g_history.prevStride:set(i, ileg, prevStride)
				g_history.prevStrideFrame:set(i, ileg, prevStrideFrame)
			end
		end
	end		
	if not config.donotPlot then
		Imp.ChangeChartPrecision(50)
		RE.motionPanel():scrollPanel():addPanel(phases:column(0))
		RE.motionPanel():scrollPanel():addPanel(phases:column(1))
		RE.motionPanel():scrollPanel():addPanel(stridePhase[1]:column(0))
		RE.motionPanel():scrollPanel():addPanel(stridePhase[2]:column(0))
		RE.motionPanel():scrollPanel():addPanel(stridePhase[1]:column(3))
		RE.motionPanel():scrollPanel():addPanel(stridePhase[2]:column(3))
		if g_history.anticipation then
			RE.motionPanel():scrollPanel():addPanel(g_history.anticipation:column(0))
			RE.motionPanel():scrollPanel():addPanel(g_history.anticipation:column(1))
		end
	end

	g_history.stridePhase=stridePhase
	return dist_z
end
function History.updateOffset(ROOTQ, mEffectors, ileg, iframe, i, stridePhase, g_history, weight, mOffset, Ldist_z)
	local swingStart=g_history.stridePhase[ileg]:column(0):sample(g_history.stridePhase[ileg](iframe, 1))
	local swingEnd=g_history.stridePhase[ileg]:column(0):sample(g_history.stridePhase[ileg](iframe, 2))

	local pc,_swi, apc=mSampler:_sampleStridePose(ileg, stridePhase, Ldist_z)
	mSampler:legAngleScale(pc, apc,Ldist_z)
	local ps,_swi, aps=mSampler:_sampleStridePose(ileg, swingStart, Ldist_z)
	mSampler:legAngleScale(ps, aps,Ldist_z)
	local pe,_swi, ape=mSampler:_sampleStridePose(ileg, swingEnd, Ldist_z)
	mSampler:legAngleScale(pe, ape,Ldist_z)

	--local swingStart={
	--	g_history.stridePhase[1]:column(0):sample(g_history.stridePhase[ileg](iframe, 1)),
	--	g_history.stridePhase[2]:column(0):sample(g_history.stridePhase[ileg](iframe, 1)),
	--}
	--local swingEnd={
	--	g_history.stridePhase[1]:column(0):sample(g_history.stridePhase[ileg](iframe, 2)),
	--	g_history.stridePhase[2]:column(0):sample(g_history.stridePhase[ileg](iframe, 2)),
	--}
	--local pc=pose local apc=avgPose
	--local ps, aps=mSampler:sampleStridePose(swingStart, Ldist_z)
	--local pe, ape=mSampler:sampleStridePose(swingEnd, Ldist_z)

	local function calcOffset(ps, aps)
		mMot.loader:setPoseDOF(ps)

		local fs=mEffectors(i).bone:getFrame():toGlobalPos(mEffectors(i).localpos)
		local fs2=mEffectors(i+1).bone:getFrame():toGlobalPos(mEffectors(i+1).localpos)

		if true then
			mMot.loader:setPoseDOF(aps)
			local afs=mEffectors(i).bone:getFrame():toGlobalPos(mEffectors(i).localpos)
			local afs2=mEffectors(i+1).bone:getFrame():toGlobalPos(mEffectors(i+1).localpos)

			return {fs-afs, fs2-afs2}
		else
			return {fs, fs2}
		end
	end

	--local oc=calcOffset(pose, avgPose)
	local oc=calcOffset(pc, apc)
	local os=calcOffset(ps, aps)
	local oe=calcOffset(pe, ape)

	oe[1]:scale(0.5)
	oe[2]:scale(0.5)
	local od={ 
		os[1]*(1-weight)+oe[1]*weight,
		os[2]*(1-weight)+oe[2]*weight,
	}

	local rotY=quater()
	local offsetQ=quater()
	ROOTQ:decompose(rotY, offsetQ)

	local delta=offsetQ*(oc[2]-od[2])
	local s=Ldist_z/mSampler.dist_z

	s=sop.clampMap(s, 1,2,0,1)

	delta:scale(s)
	delta.z=delta.z*0.15
	delta.y=math.max(delta.y,0)
	delta.y=delta.y+g_history.stridePhase[ileg](iframe,3)*0.1 -- swing_y
	mOffset[1]:radd(delta)

	delta=offsetQ*(oc[1]-od[1])
	delta:scale(s)
	delta.z=delta.z*0.15
	delta.y=math.max(delta.y,0)
	delta.y=delta.y+g_history.stridePhase[ileg](iframe,3)*0.1
	mOffset[2]:radd(delta)
end

function History.getCutState(g_history)
	local cutState=boolN(g_history.stridePhase[1]:rows())
	cutState:setAllValue(false)
	local isL=boolN(g_history.stridePhase[1]:rows())

	local max_i=1e5
	for ileg=0,1 do
		local prev_sp=g_history.prevStride:column(ileg)
		local s_f=g_history.prevStrideFrame:column(ileg)
		local sp=g_history.stridePhase[ileg+1]:column(0)
		local pi=0

		for i=1, s_f:size()-1 do
			if s_f(i)~=s_f(i-1) then
				if(sp(pi+1)>0.3) then
					max_i=math.min(max_i, i) 
					break
				end
				assert(sp(i)>=0.6)
				if(sp(i+1)>1.0) then
					-- last segment
					max_i=math.min(max_i, i) 
					break
				end
				if(sp(i+1)>0.3) then
					max_i=math.min(max_i, i)
					break
				end
				print(pi, i)
				cutState:set(pi, true)
				cutState:set(i, true)
				if ileg==0 then
					isL:set(pi, true)
					isL:set(i, true)
				end
				pi=i
			end
		end
	end
	RE.motionPanel():scrollPanel():addPanel(cutState, CPixelRGB8(255,0,0))


	local seg=intIntervals()
	seg:runLengthEncodeCut(cutState)

	local conL=boolN(cutState:size())
	local conR=boolN(cutState:size())
	for iframe=0, cutState:size()-1 do
		local b=false
		for ifoot=0,1 do
			local cfinfo= g_history.cf[ifoot+1]
			local v=cfinfo:row(iframe)
			contact=ntob(v(3))
			b=b or contact
		end
		conL:set(iframe, b)

		local b=false
		for ifoot=2,3 do
			local cfinfo= g_history.cf[ifoot+1]
			local v=cfinfo:row(iframe)
			contact=ntob(v(3))
			b=b or contact
		end
		conR:set(iframe, b)
	end
	RE.motionPanel():scrollPanel():addPanel(conL, CPixelRGB8(255,0,0))
	RE.motionPanel():scrollPanel():addPanel(conR, CPixelRGB8(255,0,0))

	return cutState, isL, conL, conR
end
function History.getCutStateRaw(g_history)
	local cutState=boolN(g_history.phases:rows())
	cutState:setAllValue(false)

	local max_i=1e5
	local prevStride=CT.vec(1.49, 1.49)
	for iframe=0, cutState:size()-1 do
		for ileg=0,1 do
			local prev_sp=prevStride(ileg)
			local sp=g_history.phases(iframe, ileg)
			if prev_sp<1.5 and sp>=1.5 then
				cutState:set(iframe, true)
			end
		end
		prevStride:assign(g_history.phases:row(iframe))
	end
	RE.motionPanel():scrollPanel():addPanel(cutState, CPixelRGB8(255,0,0))

	return cutState
end
return History
