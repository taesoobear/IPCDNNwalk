local M={}
function M.concatTraj(a,b)
	local c=matrixn(a:rows()+b:rows(), a:cols())
	c:sub(0,a:rows(),0,0):assign(a)
	c:sub(a:rows(), 0,0,0):assign(b)
	return c
end

function M.stitchN(tbl)
	local out=tbl[1]
	for i=2,#tbl do
		out=M.stitchTraj(out, tbl[i])
	end
	return out
end

-- 가운데 모션은 유지한채 양끝 세그먼트 변형해서 스티치
function M.stitch3(tbl)
	assert(#tbl==3)
	local out=tbl[1]
	out=M.stitchTraj_raw(out, tbl[2], math.linstitchForward())
	out=M.stitchTraj_raw(out, tbl[3], math.linstitchOnline())
	return out
end

function M.stitchTraj(a,b,maxLen, op)
	maxLen=maxLen or 15
	if true then
		-- align a and b by horizontal translation
		local diff=a:row(a:rows()-1):toVector3()-b:row(0):toVector3()
		diff.y=0
		b=M.translateTraj(b, diff)
	end
	if a:rows()>maxLen then
		return M.concatTraj(a:sub(0, a:rows()-maxLen,0,0), M.stitchTraj(a:sub(a:rows()-maxLen,0,0,0), b))
	elseif b:rows()>maxLen then
		return M.concatTraj(M.stitchTraj(a, b:sub(0,maxLen,0,0)), b:sub(maxLen,0,0,0))
	end
	assert(a:rows()>2)
	assert(b:rows()>1)
	if b:rows()==2 then
		local out=a:copy()
		out:resize(out:rows()+1, out:cols())
		out:row(out:rows()-1):assign(b:row(1))
		return out
	end
	local c=matrixn()
	if not op then
		op=math.linstitch()
	end
	op:calc(c, a, b)
	M.normalizeTraj(c)
	return c
end
function M.stitchTraj_raw(a,b,op)
	assert(a:rows()>2)
	assert(b:rows()>1)
	if b:rows()==2 then
		local out=a:copy()
		out:resize(out:rows()+1, out:cols())
		out:row(out:rows()-1):assign(b:row(1))
		return out
	end
	local c=matrixn()
	if not op then
		op=math.linstitch()
	end
	op:calc(c, a, b)
	return c
end
function M.translateTraj(rootTraj, tf)
	local out=rootTraj:copy()
	for i=0, out:rows()-1 do
		out:row(i):setVec3(0, out:row(i):toVector3(0)+tf)
	end
	return out
end
function M.scaleTraj(rootTraj, scalef)
	local out=rootTraj
	if out:cols()<7 then return end -- does not have quaternions
	local out=rootTraj*scalef
	out:sub(0,0,3,7):assign(rootTraj:sub(0,0,3,7)) -- preserve original orientation
	return out
end
function M.rotateTraj(rootTraj, rotatef, rotate2, flightStart, flightEnd)
	local out=rootTraj*1.0
	local T=rootTraj:row(0):toVector3(0)
	local Q=quater(rotatef, vector3(0,1,0))
	local tf=transf(quater(1,0,0,0),T)* transf(Q, vector3(0,0,0))*transf(quater(1,0,0,0),-T)
	for i=0, out:rows()-1 do
		out:row(i):setVec3(0, tf*rootTraj:row(i):toVector3(0))

		if rotate2 then
			local euler=sop.clampMap(i,flightStart-20, flightEnd+10, 0, rotate2)
			local q=quater(euler, vector3(0,1,0))
			out:row(i):setQuater(3, Q*rootTraj:row(i):toQuater(3)*q*Q:inverse())
		else
			out:row(i):setQuater(3, Q*rootTraj:row(i):toQuater(3)*Q:inverse())
		end
	end
	return out
end

function M.normalizeTraj(out)
	if out:cols()<7 then return end -- does not have quaternions
	for i=0, out:rows()-1 do
		out:row(i):setQuater(3, out:row(i):toQuater(3):Normalize())
	end
end
function M.resampleTraj(rootTraj, resamplef)
	local out=matrixn()
	out:resample(rootTraj, rootTraj:rows()*resamplef)
	M.normalizeTraj(rootTraj)
	return out
end


function M.calcAcc(rootTraj, flightStart, flightEnd, frameRate)
	local acc=vector3(0,0,0)
	local n=0
	for i=math.round(flightStart*frameRate), math.round(flightEnd*frameRate) do
		acc=acc+rootTraj:row(i+1):toVector3()-rootTraj:row(i):toVector3()*2+rootTraj:row(i-1):toVector3()
		n=n+1
	end
	acc:scale(1.0/n*(frameRate*frameRate))
	print(acc)
end

function M.getScaledTraj(rotY, startPos, sf, mSampler, angle, angle2, comerrY)
	angle=angle or 0
	angle2=angle2 or 0
	comerrY=comerrY or 0
	local rootTraj0=rootTraj
	local totalTime=defaultSeq.cycle:sum()
	local function getSeq()
		local seq={
			cycle=
			CT.mat(5,5, -- 5 by 3 matrix
			0,0,0, 0,0, --duration
			1.5,                  2,         3,    0,  1,   --phaseStart
			2,                    3,         4,    1, 1.5, -- phaseEnd
			1 ,               1,     0, 0, 1,  -- toe
			1,               0,     0,  1, 1),  -- heel
		}

		seq.cycle:row(0):assign(defaultSeq.cycle)
		return seq
	end


	local seq=getSeq()
	local totalFrame0=math.floor(totalTime*30)
	local rootTraj=matrixn(totalFrame0+1, 7)

	--local rotY=quater()
	--rotY:identity()
	--local startPos=mSampler:sampleCOM(1.5,1.5).translation:copy()
	--startPos.x=0
	--startPos.z=0

	local phaseL=seq.cycle:sub(0,3)
	local phaseR=seq.cycle:sub(0,3)
	--local startTF=transf(rotY, startPos)
	local startTF=transf(quater(1,0,0,0), vector3(0, startPos.y, 0))

	for i=0, totalFrame0 do
		local t=i/30.0
		local pL=fc.getLinearSpline(t, phaseL)
		local pR=fc.getLinearSpline(t, phaseR)
		if pL<1.5 or i>=totalFrame0 then pL=pL+4 end 
		if pR<1.5 or i>=totalFrame0 then pR=pR+4 end
		local tf=mSampler:sampleCOM(pL, pR)

		if i==0 then
			startTF=startTF*tf:inverse()
		end
		MotionDOF.setRootTransformation(rootTraj:row(i), startTF*tf)
		--dbg.draw('Axes', startTF*tf, 'ttaxes__'..i, 100, 0.5)
	end

	local flightStart=phaseL(0,0)+phaseL(0,1)
	local flightEnd=flightStart+phaseL(0,2)
	local frameRate=30

	local rootTraj0=rootTraj
	rootTraj=M.scaleTraj(rootTraj, sf)
	rootTraj=M.resampleTraj(rootTraj, math.sqrt(sf))
	rootTraj=M.translateTraj(rootTraj, vector3(0, 1.0-rootTraj(0,1)+comerrY, 0))
	local flightStartN=flightStart*math.sqrt(sf)
	local flightEndN=flightEnd*math.sqrt(sf)

	local function toF(t)
		return math.round(t*frameRate)
	end
	local maxLen=5
	rootTraj=M.stitchN{
		rootTraj0:sub(0, toF(flightStart)+1-maxLen,0,0), 
		rootTraj:sub(toF(flightStartN)-maxLen,toF(flightEndN)+1+maxLen,0,0),
		rootTraj0:sub(toF(flightEnd)+maxLen,0,0,0),
	}

	assert(rootTraj:rows()==toF(flightStart)+toF(flightEndN)-toF(flightStartN)+rootTraj0:rows()-toF(flightEnd))

	seq.cycle:set(0,2, (toF(flightEndN)-toF(flightStartN))/frameRate)


	local flightStart=seq.cycle(0,0)+seq.cycle(0,1)
	local flightEnd=flightStart+seq.cycle(0,2)
	flightStart=math.round(flightStart*30)
	flightEnd=math.round(flightEnd*30)

	rootTraj=M.rotateTraj(rootTraj, angle, angle2, flightStart, flightEnd)

	local startTF=transf(rotY, startPos)
	startTF.translation.y=0
	local totalFrame0=rootTraj:rows()-1
	for i=0, totalFrame0 do
		local tf=MotionDOF.rootTransformation(rootTraj:row(i))
		MotionDOF.setRootTransformation(rootTraj:row(i), startTF*tf)
	end

	--print(rootTraj:rows(),toF(flightStart)+toF(flightEndN)-toF(flightStartN)+rootTraj0:rows()-toF(flightEnd))
	return rootTraj, seq
end
function M.transition(A, B)
	assert(A:rows()==B:rows())

	local C=A:copy()
	for i=0, A:rows()-1 do
		local t=sop.map(i, 0, A:rows()-1, 0,1)
		weight=math.smoothTransition(t)
		C:row(i):interpolate(weight, A:row(i), B:row(i))
	end
	if C:cols()>=7 then
		for i=0, A:rows()-1 do
			local t=sop.map(i, 0, A:rows()-1, 0,1)
			weight=math.smoothTransition(t)
			local q=quater()
			q:safeSlerp(A:row(i):toQuater(3), B:row(i):toQuater(3), weight)
			C:row(i):setQueter(3,q)
		end
	end

	return C
end

function M.getScaledTrajManual(totalFrame0, flightStartN, flightEndN)
	-- quadratic curve
	local frameRate=30

	local function zero(frames)
		local res=matrixn(frames,7)
		res:setAllValue(0.0)
		res:column(3):setAllValue(1.0)
		return res
	end
	local function arc1(frames)
		local res=zero(frames)
		if frames<3 then return res end
		for i=0, frames-1 do
			local y=sop.map(i,0, frames-1, -1, 1)
			y=1-y*y
			res:set(i,1,y)
		end
		return res
	end

	local function arc(frames, des_acc)
		if frames<3 then return zero(frames) end
		local res=arc1(frames)
		local function calcAcc(res)
			local dt=1.0/30.0
			local acc=((res(2,1)-res(1,1))/dt-(res(1,1)-res(0,1))/dt)/dt
			return acc
		end
		local acc=calcAcc(res)
		res:column(1):rmult(des_acc/acc)
		--local acc=calcAcc(res)
		return res
	end
	local function arcH(frames, height, minAcc, maxAcc)
		if frames<3 then return zero(frames) end
		local res=arc1(frames)*height
		local function calcAcc(res)
			local dt=1.0/30.0
			local acc=((res(2,1)-res(1,1))/dt-(res(1,1)-res(0,1))/dt)/dt
			return acc
		end
		local acc=calcAcc(res)
		des_acc=acc
		if acc<minAcc then
			des_acc=minAcc
		elseif acc>maxAcc then
			des_acc=maxAcc
		end
		res:column(1):rmult(des_acc/acc)
		--local acc=calcAcc(res)
		return res
	end

	local maxLen=5
	--[[
	local rootTraj=M.stitchN{
		zero(flightStartN+1),
		arc(flightEndN-flightStartN+1),
		zero(totalFrame0- flightEndN+1)
	}
	]]
	local t1=flightStartN+1
	local t2=flightEndN-flightStartN+1
	local t3=totalFrame0- flightEndN+1
	local G=9.8
	local arcMid=arc(t2, -G)
	local maxHeight=math.min(arcMid:column(1):maximum(),0.5)
	local h2=-1*maxHeight-0.01 -- for standing phases
	local rootTraj
	if t2<=3 then
		rootTraj=M.stitchTraj_raw(
			--zero(flightStartN+1),
			arcH(t1+1,h2,-G,2*G), --*0.5+zero(t1+1)*0.5,
			arcH(t3+t2-2,h2, -G, 2*G)
		)
	else
		rootTraj=M.stitch3{
			--zero(flightStartN+1),
			arcH(t1,h2,-G, 2*G), --*0.5+zero(t1)*0.5,
			arcMid,
			arcH(t3,h2,-G, 2*G)  --*0.5+zero(t3)*0.5
		}
	end
	for i=0, rootTraj:rows()-1 do
		rootTraj:set(i,2, i/30*3)
	end

	assert(rootTraj:rows()==totalFrame0+1)
	assert(not rootTraj:column(1):isnan())
	return rootTraj
end
function M.drawTrajTraj(rootTraj, prefix)
	prefix=prefix or ''
	local totalFrame0=rootTraj:rows()
	for i=0, totalFrame0-1 do
		local tf=MotionDOF.rootTransformation(rootTraj:row(i))
		dbg.draw('Axes', tf, prefix..'_ttaxes__'..i, 100, 2.5)
	end
end
return M
