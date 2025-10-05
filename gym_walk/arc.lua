
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

	local d1=A-B
	d1:normalize()
	local d2=control-B
	d2:normalize()
	if math.abs(d1:dotProduct(d2))>0.9993 then
		-- use line interpolation
		return false
	end
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
	--RE.output2("arcL", _A:distance(_B))
	if not rangeTest(A,B,control,0.35) then return M.lineInterpolation(t,_A,_B) end
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
	--RE.output2("arcR", _A:distance(_B))
	if not rangeTest(A,B,control,0.35) then return M.lineInterpolation(t,_A,_B) end
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
-- strength= 0 ~ 100  (the larger, the abrupt)
function M.getPendTraj_direction(m_pendulum, pendRotY, startPos, zmp, v0, zmp_vel, v, w0, T, strength, option)
	if not T then T=1.5 end

	assert(v.x==v.x)
	RE.output2('vlen', v:length())
	if v:length()<1e-3 then
		assert(false)
	end
	--if v:length()<1e-2 then
	--	-- 
	--	return M.getPendTraj(m_pendulum, pendRotY, startPos, zmp, v0, zmp_vel, w0, 0, T)
	--end

	local dt=1.0/30.0

	local rotY=0
	local c=0 for t=0, T+dt, dt do  c=c+1 end
	local goal=matrixn(c, 6)
	local goal2=matrixn(c, 14)


	local initialfront=pendRotY*vector3(0,0,1)
	local finalfront=v:copy()
	finalfront:normalize()
	RE.output2('vlen', v:length(), finalfront)
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
		if not strength or strength ==0 then
			print("warning! desiredvel needs to be smoothed")
			vv:assign(v)
		else
			local cv=m_pendulum:calcCOMvel():length()

			local strength2
			local gain=1
			if cv<2 then
				strength2=sop.clampMap(cv, 0, 2, strength*0.4, strength*0.5)
			elseif cv<4 then
				strength2=sop.clampMap(cv, 2, 4, strength*0.5, strength*0.4)
			elseif cv<8 then
				strength2=sop.clampMap(cv, 4, 8, strength*0.4, strength*0.4)
				--gain=sop.map(cv, 4, 8, 1, 4)
			else
				strength2=sop.clampMap(cv, 8, 16, strength*0.4, strength*0.2)
				--gain=sop.map(cv, 8, 16, 4, 16)
			end
			
			if cv<v:length() then
				-- accelerating
				strength2=strength2*1
			else
				strength2=strength2*1
			end

			vv:assign(m_pendulum:calcCOMvel()+ gain*math.clampVec3(v-m_pendulum:calcCOMvel(),strength2))
			assert(vv.x==vv.x)
		end

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
		--local cf_c=m_pendulum.controlForce:squaredDistance(zero)
		local fdir=qrotY*vector3(0,0,1)
		--local cf_c=m_pendulum.controlForce:squaredDistance(zero)
		local cf_c=math.max(0, m_pendulum.controlForce:dotProduct(fdir)) cf_c=cf_c*cf_c
		goal2:row(c):set(13, cf_c)
		cf=cf+m_pendulum.controlForce:squaredDistance(zero)
		if m_pendulum.errorOccurred==true then
			return nil
		end
		c=c+1
	end
	assert(c==goal:rows())

	do
		local COM=goal
		local ANG=goal2:sub(0,0,6,0)
		local ANGVEL=goal2:sub(0,0,3,6)

		-- heuristically modify rotY based trajectory tangent
		local sCOMVEL=COM:row(0):toVector3(3)
		local srootq=ANG:row(0):toQuater(0)
		local ef=COM:rows()-1
		local eCOMVEL=COM:row(ef):toVector3(3)
		local erootq=ANG:row(ef):toQuater(0)

		local function frontdirection(rootq, comvel)
			local speed=comvel:length()
			if speed>0.5 then
				local crotY=rootq:rotationY()
				local tdir=comvel:copy()
				tdir:normalize()
				local cdir=crotY*vector3(0,0,1) 
				local w=sop.clampMap(speed, 0.5, 1.0, 0, 1)
				local dir=cdir*(1-w)+tdir*w
				return dir
			end
			return rootq:rotationY()*vector3(0,0,1)
		end
		local erootqorig=erootq:copy()
		--fixRootQ(erootq, eCOMVEL)
		local pendRotY=srootq:rotationY()

		local c=ANG:rows()
		local dt=1.0/30.0
		local T=c*dt
		local function goffsetQ(q)
			local rotY=quater()
			local offsetq=quater()
			q:decomposeNoTwistTimesTwist(vector3(0,1,0), offsetq, rotY)
			return offsetq
		end

		local function goffsetZ(q)
			local rotY=quater()
			local offsetq=quater()
			q:decomposeTwistTimesNoTwist(vector3(0,1,0), rotY, offsetq)

			local offsetz=quater()
			local offsetx=quater()
			offsetq:decomposeTwistTimesNoTwist(vector3(0,0,1), offsetz, offsetx)
			return rotY*offsetz*rotY:inverse()
		end


		local removeRotX=false
		if option and option.removeRotX then 
			removeRotX=true 
		end
		-- goffstQ=rotY*offsetQ*rotY:inverse()
		for i=0, COM:rows()-1 do
			local COMVEL=COM:row(i):toVector3(3)
			local rootq=ANG:row(i):toQuater(0)
			local rotY=rr(i)
			local qrotY=quater(rotY, vector3(0,1,0))*pendRotY
			local angvel=ANGVEL:row(i):toVector3(0)
			local w=rootq:inverse()*angvel
			w.y=ww(i,0)
			if removeRotX then
				rootq=goffsetZ(rootq)*qrotY
			else
				rootq=goffsetQ(rootq)*qrotY
			end
			angvel=rootq*w
			ANG:row(i):setQuater(0, rootq)
			ANGVEL:row(i):setVec3(0, angvel)
		end
	end


	return goal, goal2, math.sqrt(cf)
end
return M
