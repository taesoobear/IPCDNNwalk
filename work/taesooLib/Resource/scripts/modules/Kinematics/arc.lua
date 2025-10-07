
local M={}
function M.arcInterpolation_short(t, A, B, control)
	local center
	do 
		-- http://stackoverflow.com/questions/22791951/algorithm-to-find-an-arc-its-center-radius-and-angles-given-3-points
		-- find center
		local C=control
		local mid_AB=(A+B)/2;
		local mid_BC=(B+C)/2;
		local slope_AB=(B.z-A.z)/(B.x-A.x);
		local slope_BC=(C.z-B.z)/(C.x-B.x);
		local slope_perp_AB = -1/(slope_AB)
		local slope_perp_BC = -1/(slope_BC)
		
		-- y = m(x - a) + b = mx -ma +b
		-- line: Ax+By=C
		-- Get A,B,C of first line - perpendicular to AB
		local A1=-slope_perp_AB
		local B1=1
		local C1=-slope_perp_AB*mid_AB.x+mid_AB.z

		local A2=-slope_perp_BC
		local B2=1
		local C2=-slope_perp_BC*mid_BC.x+mid_BC.z

		-- Get delta and check if the lines are parallel
		local delta = A1*B2 - A2*B1;
		--if(delta == 0)    throw new System.Exception("Lines are parallel");
 
		center=vector3( (B2*C1 - B1*C2)/delta, 0, (A1*C2 - A2*C1)/delta)
		--dbg.draw("Sphere", center, "ball3", "green", 15)
	end


	local midpoint=(A+B)/2;
	local _bm=B-midpoint;
	local perpendicular = vector3(-_bm.z, 0, _bm.x)
	local h=perpendicular:length()/ (center-midpoint):length()

	local axis=vector3()
	axis:cross(B-A, B-control)
	if axis.y<0 then h=h*-1 end

	local _Ac=A-center
	_Ac.y=0
	local radius = _Ac:length()

	local function atan2(v)
		return math.atan2(v.z, v.x)
	end
	local startAngle = atan2(A - center);

	local endAngle = atan2(B - center);

	local angleDelta = endAngle - startAngle;
	local PI=3.141592
	if(angleDelta * h > 0) then
		if h>0 then
			angleDelta = angleDelta-2 * PI;
		else
			angleDelta = angleDelta+2 * PI;
		end
	end

	local angle = startAngle + angleDelta * t;
	local point = center + radius * vector3(math.cos(angle), 0, math.sin(angle));
	point.y=A.y
	return point, axis.y
end
local function arcInterpolation2(t, A, center, angleDelta)
	angleDelta=angleDelta*-1
	local _Ac=A-center
	_Ac.y=0
	local radius = _Ac:length()
	local function atan2(v)
		return math.atan2(v.z, v.x)
	end
	local startAngle = atan2(A - center);
	local angle = startAngle + angleDelta * t;
	local point = center + radius * vector3(math.cos(angle), 0, math.sin(angle));
	point.y=A.y
	return point
end
function M.arcInterpolation(t, A, B, control)
	local center
	do 
		-- http://stackoverflow.com/questions/22791951/algorithm-to-find-an-arc-its-center-radius-and-angles-given-3-points
		-- find center
		local C=control
		local mid_AB=(A+B)/2;
		local mid_BC=(B+C)/2;
		local slope_AB=(B.z-A.z)/(B.x-A.x);
		local slope_BC=(C.z-B.z)/(C.x-B.x);
		local slope_perp_AB = -1/(slope_AB)
		local slope_perp_BC = -1/(slope_BC)
		
		-- y = m(x - a) + b = mx -ma +b
		-- line: Ax+By=C
		-- Get A,B,C of first line - perpendicular to AB
		local A1=-slope_perp_AB
		local B1=1
		local C1=-slope_perp_AB*mid_AB.x+mid_AB.z

		local A2=-slope_perp_BC
		local B2=1
		local C2=-slope_perp_BC*mid_BC.x+mid_BC.z

		-- Get delta and check if the lines are parallel
		local delta = A1*B2 - A2*B1;
		--if(delta == 0)    throw new System.Exception("Lines are parallel");
 
		center=vector3( (B2*C1 - B1*C2)/delta, 0, (A1*C2 - A2*C1)/delta)
	end

	local midpoint=(A+B)/2;
	local _bm=B-midpoint;
	local perpendicular = vector3(-_bm.z, 0, _bm.x)
	local h=perpendicular:length()/ (center-midpoint):length()


	local function atan2(v)
		return math.atan2(v.z, v.x)
	end
	local endAngle = atan2(B - center);
	local startAngle = atan2(A - center);

	local angleDelta = endAngle - startAngle;
	local PI=3.141592
	if(angleDelta * h > 0) then
		if h>0 then
			angleDelta = angleDelta-2 * PI;
		else
			angleDelta = angleDelta+2 * PI;
		end
	end

	return arcInterpolation2(t, A, center, angleDelta*-1)
end
local function rangeTest(A,B,control, min)
	local o=(B-A)*min
	if (B-A-o):dotProduct(control-A-o)<0 then return false end
	if (A-B+o):dotProduct(control-B+o)<0 then return false end
	if (A-B):length()<min then return false end
	return true
end
function M.lineInterpolation(t,A,B)
	local p=vector3()
	p:interpolate(t,A,B)
	return p
end

function M.spline0(t,A,B,control)
	local points=vector3N(3)
	points(0):assign(A)
	points(1):assign(control)
	points(2):assign(B)
	local sl=math.NonuniformSpline(CT.vec(0,0.5,1), points:matView(), math.NonuniformSpline.ZERO_VEL)
	local points=matrixn()
	sl:getCurve(CT.vec(t), points)
	return points:row(0):toVector3(0)
end
function M.arcInterpolationL(t,_A,_B,_control)
	local A=_A:copy()
	local B=_B:copy()
	local control=_control:copy()
	A.y=0
	B.y=0
	control.y=0
	if not rangeTest(A,B,control,0.05) then return M.lineInterpolation(t,_A,_B) end
	local p,y=M.arcInterpolation_short(t,A,B,control)
	p.y=_B.y*t+_A.y*(1-t)
	if y>-0.00001 then
		--return M.lineInterpolation(t,A,B)
		return (p+M.lineInterpolation(t,_A,_B))*0.5
	end
	return p
end
function M.arcInterpolationR(t,_A,_B,_control)
	local A=_A:copy()
	local B=_B:copy()
	local control=_control:copy()
	A.y=0
	B.y=0
	control.y=0
	if not rangeTest(A,B,control,0.05) then return M.lineInterpolation(t,_A,_B) end
	local p,y=M.arcInterpolation_short(t,A,B,control)
	p.y=_B.y*t+_A.y*(1-t)
	if y<0.00001 then
		--return M.lineInterpolation(t,A,B)
		return (p+M.lineInterpolation(t,_A,_B))*0.5
	end
	return p
end
function M.getTraj(startPos, forwardDir, v, w, T, numSamples)
	if not numSamples then
		numSamples=10
	end

	local goal=matrixn(numSamples,7)
	local theta=w*T
	goal:setAllValue(0)

	local samples=CT.linspace(0,1, numSamples)
	if math.abs(w)<1e-3 then
		local endPos=startPos+forwardDir*v*T

		for i=0, numSamples-1 do
			local t=samples(i)
			local point=vector3()
			point:interpolate(t, startPos,endPos)
			goal:row(i):setVec3(0, point)
			goal:row(i):setQuater(3, quater(1,0,0,0))
		end
	else
		-- rw=v
		local R=v/w
		local side=quater(3.141592/2, vector3(0,1,0))*forwardDir
		local center=startPos+side*R

		for i=0, numSamples-1 do
			local t=samples(i)
			local point=arcInterpolation2(t, startPos, center, theta)
			goal:row(i):setVec3(0, point)
			goal:row(i):setQuater(3, quater(w*t, vector3(0,1,0)))
		end
	end

	return goal, (quater(theta, vector3(0,1,0))*forwardDir)
end
function M.getTraj_constantForwardAcc(startPos, forwardDir, v0, v1, w, T, numSamples)
	if not numSamples then
		numSamples=10
	end

	local goal=matrixn(numSamples, 7)
	local samples=CT.linspace(0,1, numSamples)

	local p0=startPos:copy()
	local q0=quater(1,0,0,0)

	local d0=forwardDir:copy()
	goal:row(0):setVec3(0, p0)
	goal:row(0):setQuater(3, q0)
	for  i=0, numSamples-2 do
		local T0=samples(i)
		local T1=samples(i+1)
		local v=sop.map(i, -1, numSamples-2, v0, v1)

		local t1, d1=M.getTraj(p0, d0, v, w, T1-T0, 2)
		p0=t1:row(1):toVector3(0)
		q0=t1:row(1):toQuater(3)
		d0=d1
		goal:row(i+1):setVec3(0, p0)
		goal:row(i+1):setQuater(3, q0)
	end

	return goal, d0
end

function M.getTraj_direction(pendRotY, startPos, initialVel, desiredVel, w0,  T, numSamples, k_p, _maxT)
	local dt=T/numSamples
	local c=numSamples
	local goal=matrixn(c, 7)

	local initialfront=pendRotY*vector3(0,0,1)
	local finalfront=desiredVel:copy()
	finalfront:normalize()

	local delta=quater()
	delta:setAxisRotation(vector3(0,1,0), initialfront, finalfront)
	delta:align(quater(1,0,0,0))
	local rr=vectorn()
	local finalAngle=delta:rotationAngleAboutAxis(vector3(0,1,0))

	_maxT=_maxT or 1.5
	if T>_maxT then
		local cc=math.floor(_maxT/dt)
		rr:setSize(c)
		rr:range(0,cc):hermite(0, 0+w0*dt, cc, finalAngle, finalAngle)
		rr:range(cc, c):setAllValue(rr(cc-1))
	else
		--rr:hermite(0, 0+w0*dt, c, finalAngle, finalAngle)

		local cc=math.floor(1.5/dt)
		rr:setSize(cc)
		rr:hermite(0, 0+w0*dt, cc, finalAngle, finalAngle)

		rr:resize(c)

	end

	local _p=startPos:copy()
	local _v=initialVel:copy()
	local v0=initialVel
	v0.y=0
	local c=0
	local qrotY

	local w=w0
	local cf=0
	local zero=vector3(0,0,0)
	local t=0
	k_p=k_p or 3
	for c=0, numSamples-1 do
		rotY=rr(c)
		qrotY=quater(rotY, vector3(0,1,0))*pendRotY

		goal:row(c):setVec3(0, _p)
		goal:row(c):setQuater(3, qrotY)

		-- one step
		_a=k_p*(desiredVel-_v)
		_v=_v+_a*dt
		_p=_p+_v*dt

		t=t+dt
	end
	return goal
end



function M.getPendDesiredVel(COMvel, desiredV, desiredW)
	-- 구심 가속도
	-- a=vw
	local amt_sideway_vel=1
	local dv=desiredW*amt_sideway_vel*COMvel:length()
	local desiredVV=vector3(dv, 0, desiredV)
	desiredVV:normalize()
	desiredVV:scale(desiredV)
	return desiredVV
end

function M.getPendTraj(m_pendulum, pendRotY, startPos, zmp, v0, zmp_vel,  v, w0, w, T)
	if not T then T=1.5 end
	local dt=1.0/30.0

	local rotY=0
	local c=0 for t=0, T+dt, dt do  c=c+1 end
	local goal=matrixn(c, 6)
	local goal2=matrixn(c, 14)

	m_pendulum:setState(zmp, startPos, zmp_vel, v0)   

	--m_pendulum:setState_ori_conservative(zmp, startPos, v0, v0)   

	local v0=(pendRotY:inverse()*v0).z
	local c=0
	local qrotY
	local cf=0
	local zero=vector3(0,0,0)

	for t=0, T+dt, dt do
		local desiredW=sop.clampMap(t, 0, T/10, w0, w)
		rotY=rotY+desiredW*dt
		qrotY=quater(rotY, vector3(0,1,0))*pendRotY
		m_pendulum:setOrientation2(qrotY)

		--local desiredV=sop.clampMap(t, 0, T/10, v0, v)
		local desiredV=v

		local currV=m_pendulum:calcCOMvel()

		-- prevent too abrubt change in forward speed.
		if math.abs(desiredV-currV:length())>2 then
			desiredV=currV:length()+math.clamp(desiredV-currV:length(), -2,2)
		end

		local desiredVV=M.getPendDesiredVel(currV, desiredV, desiredW)

		if (desiredVV-currV):length()>2 then

			local diff=desiredVV-currV
			diff:normalize()
			diff:scale(2)
			desiredVV=currV+diff
		end

		m_pendulum:setDesiredVelocity(qrotY*desiredVV)

		goal:row(c):setVec3(0, m_pendulum:calcCOMpos())
		goal:row(c):setVec3(3, m_pendulum:calcCOMvel())
		goal2:row(c):setVec3(0, m_pendulum:calcCartPos())
		local q=m_pendulum.theta:toQuater(2)
		local w=m_pendulum.dtheta:toVector3(3)
		w.y=desiredW

		goal2:row(c):setVec3(3, w)
		goal2:row(c):setQuater(6, q*qrotY)
		goal2:row(c):setVec3(10, m_pendulum:calcCartVel())
		m_pendulum:oneStep()

		local cf_c=m_pendulum.controlForce:squaredDistance(zero)
		goal2:row(c):set(13, cf_c)
		cf=cf+cf_c
		if m_pendulum.errorOccurred==true then
			dbg.console()
		end
		c=c+1
	end
	assert(c==goal:rows())

	return goal, goal2, math.sqrt(cf)
end

-- returns goal, goal2
-- s.t.
-- COMPos=goal:row(i):toVector3(0)
-- ZMPpos=goal2:row(i):toVector3(0)
-- Ori=goal2:row(i):toQuater(6)
function M.getPendTraj_direction(m_pendulum, pendRotY, startPos, zmp, v0, zmp_vel, v, w0, T)
	if not T then T=1.5 end

	if v:length()<1e-2 then
		return M.getPendTraj(m_pendulum, pendRotY, startPos, zmp, v0, zmp_vel, w0, 0, T)
	end

	local dt=1.0/30.0

	local rotY=0
	local c=0 for t=0, T+dt, dt do  c=c+1 end
	local goal=matrixn(c, 6)
	local goal2=matrixn(c, 14)


	local initialfront=pendRotY*vector3(0,0,1)
	local finalfront=v:copy()
	finalfront:normalize()
	local delta=quater()
	delta:setAxisRotation(vector3(0,1,0), initialfront, finalfront)
	delta:align(quater(1,0,0,0))
	local rr=vectorn()
	local finalAngle=delta:rotationAngleAboutAxis(vector3(0,1,0))

	if T>1.5 then

		local cc=math.floor(1.5/dt)
		rr:setSize(c)
		rr:range(0,cc):hermite(0, 0+w0*dt, cc, finalAngle, finalAngle)
		rr:range(cc, c):setAllValue(rr(cc-1))
	else
		rr:hermite(0, 0+w0*dt, c, finalAngle, finalAngle)
	end
	local ww=rr:column():derivative(1/dt)

	m_pendulum:setState(zmp, startPos, zmp_vel, v0)   
	--m_pendulum:setState(zmp, startPos, v0, v0)   

	--local v0=(pendRotY:inverse()*v0).z
	v0.y=0
	local c=0
	local qrotY

	local w=w0
	local cf=0
	local zero=vector3(0,0,0)
	for t=0, T+dt, dt do
		rotY=rr(c)
		qrotY=quater(rotY, vector3(0,1,0))*pendRotY
		m_pendulum:setOrientation2(qrotY)

		local vv=vector3()
		--vv:interpolate(sop.clampMap(t, 0, T, 0,1), v0, v)
		vv:assign(v)
		m_pendulum:setDesiredVelocity(vv)
		goal:row(c):setVec3(0, m_pendulum:calcCOMpos())
		goal:row(c):setVec3(3, m_pendulum:calcCOMvel())
		goal2:row(c):setVec3(0, m_pendulum:calcCartPos())
		local q=m_pendulum.theta:toQuater(2)
		local w=m_pendulum.dtheta:toVector3(3)
		w.y=ww(c,0)

		goal2:row(c):setVec3(3, w)
		goal2:row(c):setQuater(6, q*qrotY)
		goal2:row(c):setVec3(10, m_pendulum:calcCartVel())
		m_pendulum:oneStep()
		local cf_c=m_pendulum.controlForce:squaredDistance(zero)
		goal2:row(c):set(13, cf_c)
		cf=cf+m_pendulum.controlForce:squaredDistance(zero)
		if m_pendulum.errorOccurred==true then
			dbg.console()
		end
		c=c+1
	end
	assert(c==goal:rows())

	return goal, goal2, math.sqrt(cf)
end

-- spline: tl.mat({{time0, value0}, {time1, value1}, ...{timen, valuen}})
function M.getLinearSplineABS(t_global, spline)
	local time=spline:column(0)
	local control1=spline:column(1):range(0, spline:rows()-1)
	local control2=spline:column(1):range(1, spline:rows())
	local eps=1e-10
	assert(t_global>=0.0)
	local t=0
	local w
	local iseg=-1
	for i=0, time:size()-2 do
		t=time(i+1)
		local d=t-time(i)
		if (t >= t_global-eps) then -- at junctions, returns previous spline (=)
			iseg=i
			w=sop.map(t_global, t-d, t, 0, 1)
			break
		end
	end
	--assert(iseg~=-1)
	if iseg==-1 then
		return control2(control2:size()-1)
	end

	return sop.map(w, 0, 1, control1(iseg), control2(iseg))
end

return M
