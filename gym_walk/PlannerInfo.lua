require("gym_walk/PlannerInfo_bar") -- common for human and quadrupeds
-- for human walking
function PlannerInfo:createUI()
	mCameraInfo={
		vpos= RE.viewpoint().vpos:copy(),
		vat=RE.viewpoint().vat:copy(),
	}
	--mEventReceiver=EVR()

	this:create("Value_Slider", "height", "height", 1)
	this:widget(0):sliderRange(0.2, 1.1)
	this:widget(0):sliderValue(0.95)

	this:create("Value_Slider", "turning", "turning", 1)
	this:widget(0):sliderRange(-2, 2)
	this:widget(0):sliderValue(0)
	this:widget(0):clearVisible()

	this:create("Value_Slider", "stride scale", "stride scale", 1)
	this:widget(0):sliderRange(0.5, 2)
	this:widget(0):sliderValue(1)

	this:create("Value_Slider", "speed", "speed", 1)
	this:widget(0):sliderRange(0.01, 14)
	-- walk : 1.15m/s
	-- horse : 14m/s
	-- bolt : 10m/s
	-- messi: 8.3m/s
	this:widget(0):sliderValue(1.15)

	this:create("Value_Slider", "footOffset", "footOffset", 1)
	this:widget(0):sliderRange(0.01, 0.2)
	this:create("Value_Slider", "COMoffset", "COMoffset", 1)
	this:widget(0):sliderRange(-0.2, 0.2)


	this:create("Value_Slider", "finalrotY", "finalrotY", 1)
	this:widget(0):sliderRange(-3.14, 3.14)
	this:widget(0):sliderValue(0)
	--this:widget(0):clearVisible()
	this:create("Check_Button", "use slope", "use slope")
	this:widget(0):checkButtonValue(false)
	this:create("Check_Button", "solve ik", "solve ik")
	this:widget(0):checkButtonValue(true)
	this:create("Check_Button", "use terrain", "use terrain")
	this:widget(0):checkButtonValue(false)

	this:create("Check_Button", "use COM cost", "use COM cost")
	this:widget(0):checkButtonValue(true)
	this:create("Check_Button", "draw box constraints", "draw box constraints")
	this:widget(0):checkButtonValue(false)
	this:create("Check_Button", "draw pend traj", "draw pend traj")
	this:widget(0):checkButtonValue(false)
	this:updateLayout()


end

PlannerInfo.initializeGlobal_original=PlannerInfo.initializeGlobal

function PlannerInfo:initializeGlobal(doNotInitializeState)
	self:initializeGlobal_original()


	g_foot=true

	if not doNotInitializeState then
		g_COMoffset=vector3(0.006,0,0) -- YUP
		local fo=0.05
		g_footOffset=vector3(0,fo,0) -- ZUP

		local height=this:findWidget('height'):sliderValue()
		if self.planner:hasTerrain() then
			height=height+ self.planner:getTerrainHeight(0,0);
		end
		local csc=fc.composeSupportCoordinate
		local I=quater(1,0,0,0)
		local initialState={
			{
				lin={ 
					vector3(0, height, 0),
					vector3(fo,0,0), -- YUP
					vector3(-fo,0,0),
				},
				quat={
					quater(1,0,0,0),
					quater(1,0,0,0),
					quater(1,0,0,0),
				},
				vector3(0,0,0), -- linvel
				vector3(0,0,0), -- angvel
			},
			{
				-- spprtCoord
				{
					csc(vector3(fo,0,0),I,I),
					csc(vector3(-fo,0,0),I,I),
				}
			}
		}
		self:setInitialState(initialState)
	end
	for i, state in ipairs(input.states) do
		local motionType=state[1]
		state.mSampler=fc.PoseSampler(mSolverInfo.effectors, mMot.loader, mMot.motionDOFcontainer.mot, input, motionType)
		state.mSeq=setInput(motionType)
	end

	local initialPose=input.states[g_currState].mSampler:samplePose(1.5,3.5)
	initialPose:set(1, 1.0)

	--mSampleSkin=fc.createSkin(mMot.loader, input.skinScale)
	--mSampleSkin:setTranslation(-150,0,0)
	--local pose=mSampler:samplePose(1.5, 3.5)
	--mSampleSkin:setPoseDOF(pose)

	--mFilter=OnlineFilter(mMot.loader, initialPose, defaultSeq.input.filterSize)
	g_prevPose=initialPose:copy()
end
function PlannerInfo:getCustomState()
	return self.refPendState
end
function PlannerInfo:setCustomState(t)
	self.refPendState=t
end


function onCallback(w, ud)
	if w:id()=="footOffset" then
		g_footOffsetX=w:sliderValue()
	elseif w:id()=="start capture" then
		RE.renderer():screenshot(true)
		RE.motionPanel():motionWin():playFrom(0)
	elseif w:id()=="COMoffset" then
		g_COMoffset=vector3(w:sliderValue(),0,0)
	elseif w:id()=="flightPhase" then
		local stancePhase=flight+2*stance
		flight=w:sliderValue()
		stance=(stancePhase-flight)*0.5

	elseif w:id()=='stancePhase' then
			
		--stancePhase=flight+2*stance
		stance=(w:sliderValue()-flight)*0.5

	elseif w:id()=='use slope' then
		if w:checkButtonValue() then
			planner.planner:changeTerrainSlope(quater(math.rad(-4), vector3(0,1,0))*vector3(0,0,1), 0)
		end
	elseif w:id()=='use terrain' then
		if w:checkButtonValue() then
			planner:changeTerrain(true)
		end
	elseif w:id()=='save state' then
		if g_lastState then
			g_savedState=g_lastState
		end
	elseif w:id()=="change motion type" then
		initiateChangeMotionType()
	elseif w:id()=='load state' and g_savedState then
		planner.refPendState=g_savedState[2]
		planner:setInitialState(g_savedState[1])
		planner:replan(getAction())
		g_iframe=0
		g_global_time=0
	end
end
function initiateChangeMotionType()
	g_prevState=g_currState
	g_desiredState=math.fmod(g_currState, #input.states)+1
	RE.output2('currstate', g_currState)
end

function PlannerInfo:changeTerrainStair(draw, image, s, h, minh)
	local planner=self
	self._terrainOffset.z=-2.0
	if minh then
		planner.planner:changeTerrain(
		image, 
		s*0.1, s, minh,h)
	else
		planner.planner:changeTerrain(
		image, 
		s*0.1, s, 0, h)
	end
	bgnode=RE.ogreSceneManager():getSceneNode("BackgroundNode")
	bgnode:setVisible(false)

	if draw then
		g_mesh=OBJloader.Terrain(image, s*10,s*100,h*100,1,1, false)
		-- scale 100 for rendering 
		g_meshToEntity=MeshToEntity(g_mesh, 'meshName')
		--meshToEntity:updatePositionsAndNormals()
		local entity=g_meshToEntity:createEntity('entityName' )
		entity:setMaterialName("lightgrey")
		--entity:setMaterialName("red")
		--entity:setMaterialName("checkboard/crowdEditink")
		local node=RE.createChildSceneNode(RE.ogreRootSceneNode(), "mesh_node")
		node:attachObject(entity)
		node:translate(-s*5,-0.1,-s*50-self._terrainOffset.z*100)
	end
end
function PlannerInfo:changeTerrain(draw, s, h, minh)
	local planner=self
	s=s or 18
	h=h or 1.3

	if minh then
		planner.planner:changeTerrain(
		"../Resource/crowdEditingScripts/terrain/heightmap1_256_256_2bytes.raw", 256, 256, 
		s, s, minh,h)
	else
		planner.planner:changeTerrain(
		"../Resource/crowdEditingScripts/terrain/heightmap1_256_256_2bytes.raw", 256, 256, 
		s, s, h)
	end
	bgnode=RE.ogreSceneManager():getSceneNode("BackgroundNode")
	bgnode:setVisible(false)

	if draw then
		g_mesh=OBJloader.Terrain("../Resource/crowdEditingScripts/terrain/heightmap1_256_256_2bytes.raw", 256, 256, s*100,s*100,h*100,1,1)
		-- scale 100 for rendering 
		g_meshToEntity=MeshToEntity(g_mesh, 'meshName')
		--meshToEntity:updatePositionsAndNormals()
		local entity=g_meshToEntity:createEntity('entityName' )
		entity:setMaterialName("CrowdEdit/Terrain1")
		--entity:setMaterialName("red")
		--entity:setMaterialName("checkboard/crowdEditink")
		local node=RE.createChildSceneNode(RE.ogreRootSceneNode(), "mesh_node")
		node:attachObject(entity)
		node:translate(-s*50,-0.1,-s*50)
	end
end
function PlannerInfo:setPhaseDurations( isL, timing, footPos, COM, prevStop, stop)

	local planner=self.planner

	local seq=input.states[g_currState].mSeq
	local defaultSeq=seq.defaultSeq
	local totalTime, aDur, switchTime, ts, minHalfCycle, halfCycleOrig, stanceDuration_orig, stanceDuration_new
	do
		local phaseL, phaseR
		local dur, dur2, durR, durR2
		-- L ________        _________
		-- R        __________       _______
		--      |                |

		halfCycleOrig=defaultSeq.touchOff:sum()
		do
			local function getSeq(halfCycle, sp_per_sw)
				return getScaledSeq(defaultSeq, halfCycle, sp_per_sw)
			end

			local _dur, _phaseStart, _phaseEnd, _toe, _heel=unpack({0,1,2,3,4})

			--timing:set(0, math.max(timing(0), halfStance+0.2*0.5))
			--timing:set(1, timing(0)+math.max(timing(1)-timing(0), halfStance+0.2*0.5))
			--timing:set(0, halfCycleOrig)
			--timing:set(1, halfCycleOrig*2)

			--local seq, sn1, so=getSeq(timing(0))
			--local seq2, sn2=getSeq(timing(1)-timing(0))
			--local seq3, sn3=getSeq(timing(2)-timing(1))
			local halfCycleOrig=defaultSeq.touchOff:sum()
			local flightOrig=defaultSeq.touchOff(2)
			local stanceOrig=halfCycleOrig-flightOrig
			local function getSpPerSw( i)
				if footPos then
					local dist=footPos[i+1]:distance(footPos[i])
					local minHalfStride=seq.input.minHalfStride or 0.5
					if dist<minHalfStride then
						dist=minHalfStride
					end
					RE.output('dist__', dist, spPerSWscale)
					if spPerSWscale then
						return stanceOrig/flightOrig*defaultSeq.input.speed* halfCycleOrig/ dist*spPerSWscale
					end
					return stanceOrig/flightOrig*defaultSeq.input.speed* halfCycleOrig/ dist
				else
					return stanceOrig/flightOrig
				end
			end
			local sw1=getSpPerSw(1)
			local sw2=getSpPerSw(2)
			local sw3=getSpPerSw(3)
			local sw=math.min(sw1, sw2, sw3)

			local seq, sn1, so=getSeq(timing(0), sw)
			local seq2, sn2=getSeq(timing(1)-timing(0), sw)
			local seq3, sn3=getSeq(timing(2)-timing(1), sw)


			minHalfCycle=math.min(timing(0), timing(1)-timing(0))
			stanceDuration_new=math.min(sn1, sn2, sn3)
			stanceDuration_orig=so

			--local seq=getSeq(halfCycle)
			--local seq2=getSeq(halfCycle)


			local function mergeLast(mat, col)
				local last=mat:column(mat:cols()-1):copy()
				last:set(_dur, last(_dur)+col(_dur))
				last:set(_phaseEnd, col(_phaseEnd))
				mat:column(mat:cols()-1):assign(last)
			end

			local function extract(mat, check)
				local out=mat:sub(0,0,0,1):copy()
				local currVal=check(mat:column(0))
				for i=1, mat:cols()-1 do
					local col=mat:column(i)
					local v=check(col)
					if v==currVal then
						mergeLast(out,  col)
					else
						out=out..col:column()
						currVal=v
					end
				end
				return out
			end
			-- 2 stride
			local L2=seq.touchOff..seq2.touchDown.. seq3.touchOff
			local R2=seq.touchDown..seq2.touchOff.. seq3.touchDown
			durLC=extract(L2, function (col) return col(_toe)== 1 or col(_heel)==1 end, 1):row(0):copy()
			durRC=extract(R2, function (col) return col(_toe)== 1 or col(_heel)==1 end, 1):row(0):copy()


			-- 1.33 stride
			local L=seq.touchOff..seq2.touchDown.. seq3.touchOff:sub(0,0,0,1)
			local R=seq.touchDown..seq2.touchOff.. seq3.touchDown:sub(0,0,0,1)

			-- 0.8 stride
			--local L=seq.touchOff..seq2.touchDown:sub(0,0,0,1)
			--local R=seq.touchDown..seq2.touchOff:sub(0,0,0,1)

			do
				local diff=R:row(0):sum()-L:row(0):sum()
				if diff>0 then
					-- R sequence is longer than L sequence, so make them be same length.
					R:set(0,R:cols()-1, R(0, R:cols()-1)-diff)
				else
					L:set(0, L:cols()-1, L(0, L:cols()-1)+diff)
				end
			end

			dur=extract(L, function (col) return col(_heel)== 1 end, 1):row(0):copy()
			dur2=extract(L, function (col) return col(_toe)== 1 end, 1):row(0):copy()
			phaseL=L:sub(0,3):copy()


			durR=extract(R, function (col) return col(_heel)== 1 end, 1):row(0):copy()
			durR2=extract(R, function (col) return col(_toe)== 1 end, 1):row(0):copy()
			phaseR=R:sub(0,3):copy()

			assert(math.abs(dur:sum()-durR:sum())<1e-3)
			for i=0, phaseL:cols()-1 do
				assert(phaseL(0,i)>-0.1)
			end
			for i=0, phaseR:cols()-1 do
				assert(phaseR(0,i)>-0.1)
			end
		end
		--switchTime= durLC(0)+durLC(1)*0.5
		switchTime= timing(0)


		totalTime= dur:sum()
		--dbg.console()
		aDur={dur, dur2, durR, durR2, phaseL, phaseR, durLC, durRC} 
	end
	if false then
		--local stancePhase=(flight+2*stance)*ts
		--planner:setDurationBasePolynomial(0.2*stancePhase/(0.5+0.17*2))
		--planner:setDurationBasePolynomial(math.min(0.2,defaultSeq.input.basePolyDur*minHalfCycle/halfCycleOrig))
		planner:setDurationBasePolynomial(math.min(0.2,defaultSeq.input.basePolyDur*stanceDuration_new/stanceDuration_orig))
		RE.output2("minHalfCycle", minHalfCycle, planner:getDurationBasePolynomial())
	end
	local dur1, dur2, dur3, dur4, phaseL, phaseR, durLC, durRC=unpack(aDur)

	local setPhaseDurations=function (planner, stopMode, i_constraint, startCon, dur) 
		if stopMode then
			local totalTime=dur:sum()
			if math.fmod(i_constraint,2)==0 then
				local ileg=math.floor(i_constraint/2)+1
				local ph=self.phases[ileg]
				local cval=self.phases[ileg](1,0)
				if startCon then
					ph:resize(3,2)
					local ferr=1e-4
					ph:column(0):assign(CT.vec(totalTime*0.5, cval,cval+ferr))
					ph:column(1):assign(CT.vec(totalTime*0.5, cval+ferr,cval))
				else
					ph:resize(3,3)
					ph:set(0,2, totalTime-ph(0,0)-ph(0,1))
				end
			end
			if startCon then
				dur=CT.vec(totalTime)
			else
				dur:resize(2)
				dur:set(1, totalTime-dur(0))
			end
		end
		RE.output2("con"..i_constraint, startCon, dur)
		planner.dur[i_constraint]={startCon, dur}
	end

	self.dur={}
	if prevStop and stop then
		self.timing= timing
		self.phases={phaseL, phaseL} -- will be modified below anyway.
		setPhaseDurations(self, stop, 0, true, dur1)
		setPhaseDurations(self, stop, 1, true, dur2)
		setPhaseDurations(self, stop, 2, true, dur1)
		setPhaseDurations(self, stop, 3, true, dur2)
		self.durC={{true, dur1}, {true, dur1}} -- unused
	else

		local startConL=true
		local startConR=false
		if prevStop then
			--[[
			print(phaseR)
			dbg.console()
			local t=phaseR:row(0):range(0, 4):sum()
			phaseR:assign(phaseR:sub(0,0,3,0):copy())
			phaseR:set(0, 0, t)
			t=dur3(0)+dur3(1)
			dur3:assign(dur3:range(1,3):copy())
			dur3:set(0, t)
			t=dur4(0)+dur4(1)
			dur4:assign(dur4:range(1,3):copy())
			dur4:set(0, t)
			startConR=true
			print(phaseR)
			dbg.console()
			]]
			phaseR:set(1,0, 3) -- originally 3.5
		end


		if isL then
			self.timing= timing
			self.phases={phaseL, phaseR}
			setPhaseDurations(self, stop, 0, startConL, dur1)
			setPhaseDurations(self, stop, 1, startConL, dur2)
			setPhaseDurations(self, stop, 2, startConR, dur3)
			setPhaseDurations(self, stop, 3, startConR, dur4)
			self.durC={{startConL, durLC}, {startConR, durRC}} -- unused
		else
			self.timing= timing
			self.phases={phaseR, phaseL}
			setPhaseDurations(self, stop, 0, startConR, dur3)
			setPhaseDurations(self, stop, 1, startConR, dur4)
			setPhaseDurations(self, stop, 2, startConL, dur1)
			setPhaseDurations(self, stop, 3, startConL, dur2)
			self.durC={{startConR, durRC},{startConL, durLC}} -- unused
		end
	end
	local durAll1=mergeDur(totalTime, self.dur[0], self.dur[1])
	local durAll2=mergeDur(totalTime, self.dur[2], self.dur[3])
	local durAll=mergeDur(totalTime,durAll1, durAll2)
	self.durAll=durAll
	self.durAllLong=mergeDur(self.durC[1][2]:sum(),self.durC[1], self.durC[2]) -- for generating COM height

	if true then

		planner:setNumForcePolynomials(3)
		local dt=math.min(0.2,defaultSeq.input.basePolyDur*stanceDuration_new/stanceDuration_orig)
		self.dt=dt
		local useUniformSampling =true -- faster..-.-
		if durAll[2]:size()==1 or self.useFD or useUniformSampling then

			local nSize=math.max(math.round(totalTime/dt),2)
			-- no flight phase
			local t=vectorn(nSize)
			t:linspace(0, totalTime)
			planner:setBasePolynomial(t)
		else
			assert(durAll[1])
			local dur=durAll[2]
			local ni=dur:size()
			local t=vectorn()
			local cur_t=0
			for i=0, ni-1, 2 do
				local nSize=math.max(math.round(dur(i)/dt),2)
				if numBasePolynomial then
					nSize=numBasePolynomial
				end
				--local nSize=5
				local lt=vectorn(nSize)
				lt:linspace(cur_t, cur_t+dur(i))
				t=t..lt:slice(0,-1)
				cur_t=cur_t+dur(i)
				print('size:',nSize, dur(i))

				if i+1<ni then
					nSize=3 -- flight phases
					local lt=vectorn(nSize)
					lt:linspace(cur_t, cur_t+dur(i+1))
					t=t..lt:slice(0,-1)
					cur_t=cur_t+dur(i+1)
				end
			end
			t:pushBack(cur_t)
			planner:setBasePolynomial(t)
			if numForcePolynomial then
				planner:setNumForcePolynomials(numForcePolynomial)
			end

			--local nSize=math.max(math.round(totalTime/dt),2)
			--print('size:', t:size(),'vs',nSize)
		end
	end

	for i=0,3 do
		local dur=self.dur[i]
		planner:setPhaseDurations(i, dur[1], dur[2])
	end
	self.totalTime=totalTime
	self.switchTime=switchTime


	return totalTime
end


function PlannerInfo:drawTrajectory()
	local pi=self
	local planner=self.planner
	local footindex=self:getFootSeq()
	local footsteps=vector3N(footindex:size())
	local footrotY=quaterN(footindex:size())
	footsteps(0):assign(pi.spprtCoord[footindex(0)/2+1].translation)
	footrotY(0):assign(pi.spprtCoord[footindex(0)/2+1].rotation)
	local function getFootRot(pi, i, t)
		local rotZ=quater(pi.planner:getFootRotZ(i, t), vector3(0,0,1))
		return pi:toYUP_ori(rotZ)
	end


	for i=0, footindex:size()-2 do
		local t=0
		if i>=1 then
			t=pi.timing(i-1)
		end
		--footrotY(i+1):assign(getFootRot(pi, footindex(i+1), t))
		local pendFrameRate=30
		footrotY(i+1):assign(self.desiredPendOri:sampleQuat(t*pendFrameRate, 0))
		footsteps(i+1):assign(pi:toYUP_pos(planner:getFootCenterPos(footindex(i+1), t)))

	end
	--dbg.draw('Axes', transf(footrotY(2), footsteps(2)), 'footsteps',100)
	
	pi.footsteps={footsteps, footrotY, footindex}

	if g_debugDraw then
		local totalTime=self.totalTime
		for t=0, totalTime, 0.2 do
			local rootpos=planner:getBasePos(t)
			local rootori=planner:getBaseOri(t)
			dbg.draw('Axes', transf(self:toYUP_ori(rootori), self:toYUP_pos(rootpos)), 'axes_'..t,100, 0.5)
		end
	end
end
function adjustDesiredCOM(durAll, p7,COM)
	if durAll[1] and durAll[2]:size()>=3 then
		assert(durAll[2]:size()>4)
		local currDurAll=durAll[2]
		local flightDur=currDurAll(1)
		if p7 and p7[1] and p7[2]:size()>=3 then
			local prevDurAll=p7[2]
			local prevFlightDur=prevDurAll(1)
			RE.output2('flightDur', prevFlightDur, flightDur, currDurAll(3))

			local s1=prevDurAll(0)*2
			local f1=prevFlightDur
			local s2=currDurAll(0)*2
			local f2=flightDur
			local s3=currDurAll(2)
			local f3=currDurAll(3)

			local py=estimateCOMheight(s1+f1+s2, s1, s1+f1)
			local cy=estimateCOMheight(s2+f2+s3, s2, s2+f2)
			local ny=estimateCOMheight(s3+f3+s3, s3, s3+f3)
			local max1=py:maximum()
			local max2=cy:maximum()
			local max3=ny:maximum()
			local avg12=(max1+max2)/2
			local avg23=(max2+max3)/2
			local avg123=(max1+max2+max3)/3
			
			local argMax
			if max1>max2 then
				if max3>max1 then
					argMax=3
				else
					argMax=1
				end
			else
				if max3>max2 then
					argMax=3
				else
					argMax=2
				end
			end

			local keytime, keyvalue

			if argMax==2 then
				keytime=CT.vec(0, s1+f1*0.5, s1+f1+s2, s1+f1+s2+f2, s1+f1+s2+f2+s3+f3*0.5, s1+f1+s2+f2+s3+f3+s3)
				keyvalue=CT.vec(0, math.min(max1-avg12,0), 0,0, math.min(max3-avg23,0), 0)
			elseif argMax==1 then
				-- see testCOMtrajEdit_gaprun.lua for comments
				keytime=CT.vec(0, s1+f1, s1+f1+s2+f2*0, s1+f1+s2+f2+s3*0.5, s1+f1+s2+f2+s3+f3+s3)
				keyvalue=CT.vec(0, 0,       math.min(max2-avg12,0)               , math.min(max3-avg23,0), 0)
			else
				keytime=CT.vec(0, s1+f1+s2*0.5, s1+f1+s2+f2, 		s1+f1+s2+f2+s3,  s1+f1+s2+f2+s3+f3+s3)
				keyvalue=CT.vec(0, math.min(max1-avg12,0), math.min(max2-avg23,0),	0,  0)
			end

			local spline=math.NonuniformSpline(keytime, keyvalue:column(), math.NonuniformSpline.ZERO_ACC)
			local time=vectorn(math.round((s1+f1+s2+f2+s3+f3+s3)*30)+1)
			for i=0, time:size()-1 do
				time:set(i, sop.map(i, 0, time:size()-1, 0, keytime(keytime:size()-1)))
			end

			local splinepoints=matrixn()
			spline:getCurve(time, splinepoints)

			splinepoints:resize(splinepoints:rows(), 2)
			local filtered=splinepoints:column(1)
			filtered:assign(splinepoints:column(0))
			for i=0, filtered:size()-1 do
				if filtered(i)>0 then
					filtered:set(i,0)
				end
			end
			math.filter(filtered:column(),5)
			
			local _f1=math.round(s2/2*30)
			local _f2=math.round((s2/2+f2)*30)
			assert(_f2<COM:rows())

			local COMy=vectorn(COM:rows()+2) -- one frame paddings at both end
			COMy:setAllValue(0.0)

			for i=-1, _f1-1 do
				local ip=i-_f1+py:size()-1
				local ic=i-_f1+math.round(s2*30)
				local w=math.smoothTransition(sop.map(i, 0, _f1-1, 0.5, 1))
				if w~=w then
					w=0.5
				end
				COMy:set(i+1,py(ip)*(1-w) + cy(ic)*w)
				--if(py(ip)<cy(ic)) then
				--	COMy:set(i+1, py(ip))
				--else
				--	COMy:set(i+1, cy(ic))
				--end
			end
			for i=_f1, _f2 do
				local ic=i-_f1+math.round(s2*30)
				COMy:set(i+1,cy(ic))
			end
			for i=_f2+1, COM:rows() do
				local ic=i-_f2+math.round((s2+f2)*30)
				local iN=i-_f2
				if ic>=cy:size() then break end
				if iN>=ny:size() then break end
				local w=math.smoothTransition(sop.map(ic, math.round((s2+f2)*30), cy:size(),0, 1))
				COMy:set(i+1,ny(iN)*w+cy(ic)*(1-w))
				--if ny(iN)<cy(ic) then
				--	COMy:set(i+1,ny(iN))
				--else
				--	COMy:set(i+1,cy(ic))
				--end
			end

			local padL=1
			local padR=1
			for i=-padL, COM:rows()+padR-1 do
				local ip=i-_f1+py:size()-1
				if ip>=filtered:size() then break end
				COMy:set(i+padL, COMy(i+padL)+filtered(ip)) 
			end
			RE.output2('COMymax', COMy:maximum(), COMy:minimum())
			if false and (f1>0.5 or f2>0.5 or f3 > 0.5 ) then
				require("subRoutines/MatplotLib")
				MatplotLib.plotFile(COMy:column(), 'comy'..g_global_time..'.png')
				print('data', s1,f1,s2,f2,s3,f3)
			end
			local dotCOMy=COMy:column():derivative(30)

			for i=0, COM:rows()-1 do
				COM:set(i,1, COM(i,1)+COMy(i+1))
				COM:set(i,4, COM(i,4)+dotCOMy(i+1,0))
			end
			assert(not COM:column(1):isnan())


			--require('MatplotLib')
			--print('flightDur', prevFlightDur, flightDur, currDurAll(3))
			--MatplotLib.plotFile(COM:column(1):range(0, math.round((currDurAll(0)+currDurAll(1)+currDurAll(2))*30)),'COMy.png')
			--dbg.console()

		end
	end
end
function estimateCOMheight(totalTime, flightStart, flightEnd)
	local totalFrame0=math.round(totalTime*30)+1
	local flightStartN=math.round(flightStart*30)
	local flightEndN=math.round(flightEnd*30)
	local height=TE.getScaledTrajManual(totalFrame0-1, flightStartN, flightEndN)
	return height:column(1):copy()
end
function changeMotionType(self)
	g_prevState=g_currState
	if g_desiredState and g_desiredState~=g_currState then
		local v=self:getInitialVel():length()
		local maxSpeed=input.states[g_desiredState].mSeq.input.maxspeed
		if v<=maxSpeed+0.1 then
			g_currState=g_desiredState
			local motionType=input.states[g_currState][1]
			if motionType=='walk' then
				spPerSWscale=nil
			else
				spPerSWscale=0.7
			end
		else
			local v=this:findWidget("speed"):sliderValue()
			this:findWidget("speed"):sliderValue(math.min(v, maxSpeed))
		end
	end
end
