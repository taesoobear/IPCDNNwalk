 
require("subRoutines/Optimizer")
function toVector2(v3)
	local vv=vector2()
	vv:assignXZ(v3)
	return vv
end
TimingOpt=LUAclass(Optimizer)

function sampleBaseTraj(ZMP, v)
	return sampleVecCol(ZMP, v, 0)
end
function sampleVecCol(ZMP, v, col)
	v=math.max(v,0)
	local vv=vectorn()
	local row1=math.floor(v*30)
	local row2=row1+1

	if row2<ZMP:rows() then
		local p1=ZMP:row(row1):toVector3(col)
		local p2=ZMP:row(row2):toVector3(col)
		local p=vector3()
		p:interpolate(v*30-row1, p1, p2)
		return p
	else
		row1=ZMP:rows()-2
		row2=ZMP:rows()-1
		local p1=ZMP:row(row1):toVector3(col)
		local p2=ZMP:row(row2):toVector3(col)
		local p=vector3()
		p:interpolate(sop.map(v*30, row1, row2,0, 1), p1, p2)
		return p
	end
	return nil
end
function sampleQuatCol(ZMP, v, col)
	v=math.max(v,0)
	local vv=vectorn()
	local row1=math.floor(v*30)
	local row2=row1+1

	if row2<ZMP:rows() then
		local p1=ZMP:row(row1):toQuater(col)
		local p2=ZMP:row(row2):toQuater(col)
		local p=quater()
		p:safeSlerp(p1, p2, v*30-row1)
		return p
	else
		row1=ZMP:rows()-2
		row2=ZMP:rows()-1
		local p1=ZMP:row(row1):toQuater(col)
		local p2=ZMP:row(row2):toQuater(col)
		local p=quater()
		p:safeSlerp(p1, p2,sop.map(v*30, row1, row2,0, 1))
		return p
	end
	return nil
end
function TimingOpt:__init(halfCycle, halfStride, totalTime, isL, spprtCoordL, spprtCoordR, ZMP, cf, boxes, config)
	if isL then
		self.prevFootPos=spprtCoordR.translation:copy()
	else
		self.prevFootPos=spprtCoordL.translation:copy()
	end
	self.ZMP=ZMP
	self.cf=cf
	self.spprtCoord={spprtCoordL, spprtCoordR}
	self.boxes=boxes
	self.isL=isL
	self.config=config

	local Lpossible=boolN()
	Lpossible:resize(ZMP:rows())
	Lpossible:setAllValue(true)
	local Rpossible=boolN()
	Rpossible:resize(ZMP:rows())
	Rpossible:setAllValue(true)

	if boxes then
		for i=0, ZMP:rows() -1 do
			local posL=getPosition(ZMP:row(i):toVector3(0), ZMP:row(i):toQuater(6), true)
			local posR=getPosition(ZMP:row(i):toVector3(0), ZMP:row(i):toQuater(6), false)
			for j,b in ipairs(boxes) do
				local vl=toVector2(posL)
				local vr=toVector2(posR)
				-- 400 foot
				local margin=0.2
				local bBreak=false
				if b:contains(vl, margin) then
					Lpossible:set(i, false)
					bBreak=true
				end
				if b:contains(vr, margin) then
					Rpossible:set(i, false)
					bBreak=true
				end
				if bBreak then break end
			end
		end
	end
	self.obstacle={Lpossible, Rpossible}
	if false and boxes then
		-- try multiple times
		local results={}
		local ntry=5
		for i=1, ntry do
			results[i]=self:_optimize(halfCycle, totalTime, halfStride, isL)
		end
		local argMin={-1, 1e9}

		for i=1, ntry do
			local res=self:objectiveFunction(results[i])
			print(res)
			if res<argMin[2] then
				argMin[1]=i
				argMin[2]=res
			end
		end
		self.result=results[argMin[1]]
		local opt_dimension=self.opt_dimension
		for i=1,#opt_dimension do
			opt_dimension[i].curval=self.result(i-1)
		end
		self:objectiveFunction(self.result, true)
	else
		self.result=self:_optimize(halfCycle, totalTime, halfStride, isL)
	end


end


function TimingOpt:_optimize(halfCycle, totalTime, halfStride, isL)
	local opt_dimension={}
	opt_dimension.totalTime=totalTime
	opt_dimension.desiredHalfCycle=halfCycle
	opt_dimension.desiredHalfStride=halfStride
	opt_dimension.isL=isL
	local c=1
	local t=halfCycle
	local _isL={}
	_isL[0]=isL
	local pos=vector3N()
	local ZMP=self.ZMP
	local totalTime=opt_dimension.totalTime
	pos:pushBack(sampleBaseTraj(ZMP, 0))

	while t<totalTime*2 do -- so that optimization dimension is enough
		isL=not isL
		if isL then
			table.insert(opt_dimension, {title="L"..c, curval=t, max_step=0.1, grad_step=0.005})
		else
			table.insert(opt_dimension, {title="R"..c, curval=t, max_step=0.1, grad_step=0.005})
		end
		_isL[c]=isL
		pos:pushBack(sampleBaseTraj(ZMP, t))
		c=c+1
		t=t+halfCycle
	end
	opt_dimension.isL=_isL -- support at the end of the segment
	opt_dimension.pos=pos
	opt_dimension.ori=quaterN(pos:size())
	if #opt_dimension>=1 then
		self.nOrigDim=#opt_dimension
		if true then
			local maxOpt=4
			self.maxOpt=maxOpt

			for i=maxOpt+1, #opt_dimension do
				opt_dimension[i]=nil
			end
		end

		_G.opt_dimension=opt_dimension
		local stepSize=0.5
		local method=Optimizer.methods.CMAes_nolog
		method.stddev=0.5
		method.maxIteration=100

		discardErrorSamples=true
		--local method=Optimizer.methods.ConjugateGradient
		--local method=Optimizer.methods.NRconjugateGradient
		Optimizer.__init(self, stepSize, opt_dimension, method)
		Optimizer.outFile=nil
		self:optimize(true)
	end
	self.opt_dimension=opt_dimension

	return self:_getResult()
end

function TimingOpt:fillNonOptVars(_t)
	local t=vectorn(self.nOrigDim)
	t:slice(0, _t:size()):assign(_t)
	local o=opt_dimension
	local desiredHalfCycle=o.desiredHalfCycle
	for i=_t:size(), t:size()-1 do
		t:set(i, t(i-1)+desiredHalfCycle)
	end
	return t
end
function TimingOpt:_getResult()
	local opt_dimension=self.opt_dimension
	local v=vectorn(#opt_dimension)
	for i=1,#opt_dimension do
		local vv= opt_dimension[i].curval
		v:set(i-1, vv)
	end
	return self:fillNonOptVars(v)
end

function TimingOpt:getResult()
	local vv=self.result
	local ZMP=self.ZMP
	local N=#opt_dimension
	local footOri=quaterN(N)
	local footPos=vector3N(N)
	for i=1,N do
		local v= vv(i-1)
		local q= sampleQuatCol(ZMP, v, 6)
		local p=getPosition(sampleBaseTraj(ZMP, v), q, opt_dimension.isL[i])
		footOri(i-1):assign(q)
		footPos(i-1):assign(p)
	end
	return vv, footOri, footPos
end

function TimingOpt:drawResult()
	local ZMP=self.ZMP
	local opt_dimension=self.opt_dimension
	local outputStr=''
	for i=1,#opt_dimension do
		local v= opt_dimension[i].curval
		if i==1 then
			print(v)
		else
			print(v-opt_dimension[i-1].curval)
		end


		local t=opt_dimension[i].title

		local q= sampleQuatCol(ZMP, v, 6)
		outputStr=outputStr..(tostring(i)..': '.. tostring(v)..', ')
		local p=getPosition(sampleBaseTraj(ZMP, v), q, opt_dimension.isL[i])
		if p then
			print(t)
			if string.sub(t,1,1)=="L" then
				dbg.draw('Sphere', p*100, tostring(i), 'green')
			else
				dbg.draw('Sphere', p*100, tostring(i), 'blue')
			end
		else
			dbg.erase('Sphere',  tostring(i))
		end
	end
	RE.output2('t1', outputStr)

	-- draw prev footstep
	if opt_dimension.isL [0] then
		local p=self.spprtCoord[2].translation
		dbg.draw('Sphere', p*100, '-r', 'blue')
	else
		local p=self.spprtCoord[1].translation
		dbg.draw('Sphere', p*100, '-l', 'green')
	end
end
function TimingOpt:checkFeasible(tt)
	local p=self.obstacle
	local isL=self.isL
	local o=self.opt_dimension
	local minT=o.desiredHalfCycle*0.5
	for ii=0,math.min(tt:size()-1, 2) do
		local t=tt(ii)
		if t<minT then return false end
		local i=math.round(t*30)
		local foot=1
		if not isL then foot=2 end
		if i<p[foot]:size() then
			if not p[foot](i) then
				return false
			end
		end
	end
	return true
end
function TimingOpt:makeFeasible(tt)
	local ttt=tt:copy()

	local c
	local cc=1

	local o=self.opt_dimension
	local minT=o.desiredHalfCycle*0.5

	local function fixOrder(ttt)
		local b=math.min(tt:size()-1, 2)
		if self.maxOpt then
			b=math.min(b, self.maxOpt-2)
		end
		for ii=0, b do
			local t=ttt(ii)
			if t<minT then 
				ttt:set(ii, minT) 
				c=c+1
			end
			if ttt(ii)>ttt(ii+1)-minT then
				ttt:set(ii+1, ttt(ii)+minT)
				c=c+1
			end
		end
	end
	repeat
		c=0
		fixOrder(ttt)
		local p=self.obstacle
		local isL=self.isL
		for ii=0,math.min(ttt:size()-1, 2) do
			local t=ttt(ii)
			local i=math.round(t*30)
			local foot=1
			if not isL then foot=2 end

			if i<p[foot]:size() then
				if not p[foot](i) then
					local n=p[foot]:findNearest(i)
					if n==-1 then
						local s=p[foot]:size()
						if i>s/2 then
							n=p[foot]:findPrev(s-1)
						else
							n=p[foot]:find(0)
							assert(n~=s)
						end
					end
					assert(n~=-1)
					ttt:set(ii, n/30)
					c=c+1
				end
			end
			isL=not isL
		end
		cc=cc+1
		fixOrder(ttt)
		if cc>10 then
			break
		end
	until c==0 
	return ttt
end

function TimingOpt:objectiveFunction(_t, verbose)

	local t=self:fillNonOptVars(_t)

	local o=opt_dimension
	local desiredHalfCycle=o.desiredHalfCycle
	local desiredHalfStride=o.desiredHalfStride

	local halfStance=(0.5+2*0.17)*0.5
	if false then
		local halfCycle=t(0)
		local flight=(halfCycle-halfStance)*2
		if flight<0.2 then
			return 100000
		end
		halfCycle=t(1)-t(0)
		flight=(halfCycle-halfStance)*2
		if flight<0.2 then
			return 100000
		end
	end


	local pos=o.pos
	local ori=o.ori
	local isL=o.isL
	local ZMP=self.ZMP

	local E1, E2, E3, E4
	do
		-- || halfCycle - desiredHalfCycle ||^2
		local len=(t(0)-desiredHalfCycle)
		len=len*len

		for i=0, t:size()-2 do
			local c=(t(i+1)-t(i)-desiredHalfCycle)
			len=len+c*c
		end
		E1=len
	end
	do
		E4=0

		local f=math.floor(t(2)*30)
		local cf=self.cf
		for i=0, f-1 do
			local ii=math.min(i, cf:size()-1)
			E4=E4+cf(ii)
		end
		E4=math.sqrt(E4)
		E4=E4*7

	end
	if true then
		-- || max(desiredHalfCycle*0.5-halfCycle, 0)||^2

		len=(t(0)-desiredHalfCycle*0.5)
		if len<0 then
			len=len*len
		else
			len=0
		end

		for i=0, t:size()-2 do
			local c=(t(i+1)-t(i)-desiredHalfCycle*0.5)
			if c<0 then
				len=len+c*c
			end
		end
		E1=E1+len*len
	end
	if false then
		--|| fullCycle - desiredFullCycle ||
		local desiredFullCycle=desiredHalfCycle*2
		local len=(t(1)-desiredFullCycle)
		len=len*len

		for i=0, t:size()-3 do
			local c=(t(i+2)-t(i)-desiredFullCycle)
			len=len+c*c
		end
		E4=len
	end

	do
		-- || halfCycle _i - halfCycle_i+1||
		local len=(t(0)-(t(1)-t(0)))
		len=len*len

		for i=1, t:size()-2 do
			local c=(t(i+1)-t(i)-(t(i)-t(i-1)))
			len=len+c*c
		end
		E3=len
	end


	ori(0):assign(sampleQuatCol(ZMP, 0, 6))
	pos(0):assign(getPosition(sampleBaseTraj(ZMP, 0), ori(0), isL[0]))

	for i=0, t:size()-1 do
		ori(i+1):assign(sampleQuatCol(ZMP, t(i), 6))
		pos(i+1):assign(getPosition(sampleBaseTraj(ZMP, t(i)), ori(i+1), isL[i+1]))
	end

	do
		E2=0
		for i=-1, t:size()-2 do

			local p1
			if i==-1 then
				p1=self.prevFootPos
			else
				p1=pos(i)
			end
			local p2=pos(i+2)
			local ps=pos(i+1)
			
			local dir=p2-p1
			dir:normalize()
			local d=vector3()
			d:cross(ps-p1, dir)

			local okayAmt=0.05
			if isL[i+1] then
				local dd=math.max(d.y-okayAmt,0)
				E2=E2+dd*dd
			else
				local dd=math.max(-d.y-okayAmt,0) E2=E2+dd*dd
			end
		end
	end
	if false then
		-- l1 distance

		local maxdd=0
		for i=0, t:size()-2 do
			local p1=pos(i)
			local p2=pos(i+2)
			local ps=pos(i+1)
			
			local dir=p2-p1
			dir:normalize()
			local d=vector3()
			d:cross(ps-p1, dir)

			if isL[i+1] then
				local dd=math.max(d.y,0)
				maxdd=math.max(maxdd, dd)
			else
				local dd=math.max(-d.y,0)
				maxdd=math.max(maxdd, dd)
			end
		end
		E2=maxdd*maxdd
	end
	local E5
	do
		-- prevent long strides
		E5=0
		for i=0, t:size()-1 do
			local p1=pos(i)
			local p2=pos(i+1)
			local stride=p1:distance(p2)
			if stride>desiredHalfStride then
				local c=stride-desiredHalfStride
				E5=E5+c*c
			end
		end
	end
	local E6
	do
		-- prevent short strides
		E6=0
		for i=0, t:size()-1 do
			local p1=pos(i)
			local p2=pos(i+1)
			local stride=p1:distance(p2)
			if stride<desiredHalfStride then
				local c=stride-desiredHalfStride
				E6=E6+c*c
			end
		end
	end
	local E_box=0
	if false and self.boxes then
		local boxes=self.boxes
		for i=0, t:size()-2 do
			local p1=pos(i)
			for j,b in ipairs(boxes) do
				local v=toVector2(p1)
				-- 400mm foot
				local margin=0.2
				if b:contains(v, margin) then

					local dx=math.min(v.x-b.min.x-margin,b.max.x+margin-v.x)
					local dz=math.min(v.y-b.min.y-margin,b.max.y+margin-v.y)
					--E_box=E_box+dx*dx+dz*dz
					--E_box=E_box+math.abs(dx)+math.abs(dz)
					E_box=E_box+1
					if verbose then
						print('box'..i)
					end
					break
				end
			end
		end
		E_box=E_box*1000
	end


	-- E1=|| halfCycle - desiredHalfCycle ||^2
	-- E4=|| fullCycle - desiredFullCycle ||
	-- E2= intersection
	-- E3= || halfCycle _i - halfCycle_i+1||
	-- E5 prevent long strides
	-- E6 prevent short strides
	--return E1+E4*0.3+E2*10+E3*0.1+E5
	--return E1+E4*0.3+E3*0.1+E5 -- without E2
	--return E1+E2*5+E3*0.3+E5*0.1+E6*0.05-- without E2 -> best for walk??
	--return E1+E2*5+E3*0.3+E5*0.05+E6*0.025-- 
	--
	if verbose then
		print('verbose')
		print(E1+E2*5+E3*0.3+E5*0.025+E6*0.0125+E4*0.00005+E_box)-- 
		print(E1, E2*5, E3*0.3, E5*0.025, E6*0.0125, E4*0.00005, E_box)
	end

	local strideWeight=1.0
	local intersectionWeight=1.0
	local cfWeight=1.0

	if self.config then
		local comvel=self.config.COMvel
		strideWeight=self.config.strideWeight or 1.0
		if comvel then 
			if comvel>5.6 then
				--strideWeight=strideWeight*1.5
				cfWeight=1.0
			end
			if self.config.desired_v and comvel>self.config.desired_v+1.0 then
				-- sudden decel
				strideWeight=1.0
			end
		end
		--intersectionWeight=self.config.intersectionWeight or 1.0
		--cfWeight=self.config.cfWeight or 1.0
	end
	return E1+E2*5*intersectionWeight+E3*0.3+(E5*0.025+E6*0.0125)*strideWeight+E4*0.00005*cfWeight+E_box-- 

end
function getPosition(pos, rotY, isL)
	if isL then
		return pos+rotY*vector3(g_footOffsetX or 0.05, 0,0)
	else
		return pos-rotY*vector3(g_footOffsetX or 0.05, 0,0)
	end
end
TimingOpt.getPosition=getPosition
function TimingOpt.updateStopMode(self, switchTime,COM,ZMP,spprtCoordL, spprtCoordR, initialrotY, finalrotY)
	local stopMode=false
	do
		-- compute convex hull for testing if stopping is necessary.
		local COMpos=sampleBaseTraj(COM, switchTime)
		local COMvel=sampleVecCol(COM, switchTime, 3)
		local rotY=sampleQuatCol(ZMP,0,6):rotationY()
		local s={spprtCoordL,spprtCoordL,spprtCoordR, spprtCoordR}
		local mEffectors=mSolverInfo.effectors
		local hull=ConvexHull2D()
		for i=0,3 do
			local tf=s[i+1]:copy()
			tf.rotation:assign(tf.rotation:rotationY())
			local v=tf*(mEffectors(i).localpos*2.5+vector3(0,0,-0.15))
			hull:addVector3(v)

			--dbg.draw('Sphere', v*100+vector3(0,20,0), 'spp'..i)
		end
		--dbg.draw('Sphere', COMpos*100, 'com..')
		hull:buildHull()

		if hull:distanceVec3(COMpos)<0 and (
			COMvel:length()<0.45 or 
			hull:distanceVec3(COMpos+COMvel*1)<0 
			) then
			local q=quater()
			q:difference(initialrotY, finalrotY) 
			local w2=q:rotationAngleAboutAxis(vector3(0,1,0))

			if ( math.abs(w2)<math.rad(15) ) then
				stopMode=true
				local centerPos=hull:calcCentroid()
				self.contactCentroid=vector3(centerPos.x,0,centerPos.y)*0.1+COMpos*0.9
				--dbg.draw('Sphere', COMpos*100, 'com..', 'solidblue')
			else
				--dbg.draw('Sphere', COMpos*100, 'com..', 'solidred')
			end
		else
			--dbg.draw('Sphere', COMpos*100, 'com..', 'solidgreen')
		end
	end
	self.prevStopMode=self.stopMode
	self.stopMode=stopMode
	self.stopModeUpdated=true
end


return TimingOpt
