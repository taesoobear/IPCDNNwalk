require("control/SDRE2")


--class 'IPC3d'
IPC3d=LUAclass()

function IPC3d:__deinit()
	if self.skin~=nil then
		RE.remove(self.skin)
		self.skin=nil
	end
end

function IPC3d:__finalize()
	self.skin=nil
	self.loader=nil
end

function IPC3d:__init(filename, b,g,dt,q,qd)
	self.dt=dt
	self.loader=MainLib.VRMLloader(filename)
	--self.loader:printHierarchy()

	self.skin=RE.createVRMLskin(self.loader, false)

	self.skin:scale(100,100,100)

	self.theta=vectorn()
	self.loader:getPoseDOF(self.theta)

	if ragdollTest then
		self.theta:setQuater(2,quater(math.rad(170), vector3(0,0,1))) 
	end
	self.skin:setPoseDOF(self.theta)


	self.dtheta=vectorn(6) -- (v, w1)
	self.dtheta:setAllValue(0)

	local cart=self.loader:VRMLbone(1)
	local pole=self.loader:VRMLbone(2)
	self.pole=pole
	self.cart=cart
	print(cart:name(), pole:name())
	local M=cart:mass()
	local m=pole:mass()
	local l=pole:localCOM().y
	local i=pole:inertia().x
	self.friction=b

	print(M,m,l,i)

	-- lua implementation
	self.pendX=IPCview(M, m,b, i, g, l, dt, q, qd, 1)
	self.pendZ=IPCview(M, m,b, i, g, l, dt, q, qd, 0)
	self.q=q
	self.dt=dt
	self:initState()
end

function IPC3d:initState()
	self.mode=1 -- velocity control
	self.ori=quater()	-- cart orientation
	self.ori:setValue(1,0,0,0)
	self.desiredVel=vector3(0,0,0)
	self.desiredVelGlobal=vector3(0,0,0)
	self.desiredPos=vector3(0,0,0)
	self.useDesiredPos=false
	self.desiredHip=vector3(0,0,0)
	self.desiredHipGlobal=vector3(0,0,0)
	self.controlForce=vector3(0,0,0)
	self.dv=vector3(0,0,0)
	self.dw=vector3(0,0,0)
	self.Y=matrixn()
	self.numFrame=0
end
function IPC3d:setOrientation2(q)
	--   print(q)
	self.ori:assign(q)
	self.desiredVel:rotate(self.ori:inverse()) -- added by jae
	--self:setDesiredVelocity(self.desiredVelGlobal) -- commented by jae
	--self:setDesiredVelocity(self.desiredVel)
end

function IPC3d:setOrientation(angle)
	self:setOrientation2(quater(math.rad(angle), vector3(0,1,0)))
end

function projectQuaternion(q)
	local qoffset=quater()
	local qaxis=quater()

	q:decomposeNoTwistTimesTwist(vector3(0,1,0), qoffset, qaxis)
	q:assign(qoffset)
end


function IPC3d:setDesiredParam(velOrPos)
	self.desiredVelGlobal:assign(velOrPos)
	self.desiredVel:assign(velOrPos)
	self.desiredVel:rotate(self.ori:inverse())
end

function IPC3d:setDesiredVelocity(vel) -- vector3
	if self.mode==0 then
		self.mode=1
		local q=self.q
		local pxq=useCase.pendX_q or 1
		self.pendX:setQ(1,pxq*q,(1.1-pxq)*q)
		--self.pendZ:setQ(1,q,0.1*q)-- by jae
	end
	self:setDesiredParam(vel)
end

function IPC3d:setInertia(inertia) -- local inertia
	local comHeight=self.pole:localCOM().y
	local pendX=self.pendX
	local pendZ=self.pendZ
	pendX.IPC:setInertia(comHeight, inertia.x)
	pendX.IPC:updateSDRE(CT.vec(0,0),CT.vec(0,0))
	pendZ.IPC:setInertia(comHeight, inertia.z)
	pendZ.IPC:updateSDRE(CT.vec(0,0),CT.vec(0,0))
end

function IPC3d:setDesiredPosition(pos) -- vector3
	if self.mode==1 then
		self.mode=0
		local q=self.q
		self.pendX:setQ(0, q, 0)
		self.pendZ:setQ(0, q, 0)
	end

	self:setDesiredParam(pos)
end

function IPC3d:setDesiredPosZ(pos) -- vector3
	local q=self.q
	self.mode=0
	self.pendZ:setQ(0, q, 0)
	local lpos=pos:copy()
	--lpos:rotate(self.ori:inverse())
	self.pendZ:setParam(lpos.z)
	self.mode=1

	--self:setDesiredParam(velZposX)
end

function IPC3d:numFrames()
	--		return self.Y:rows()
	return self.numFrame
end

function IPC3d:getPos(iframe)

	return vector3(self.Y(iframe,1), 0, self.Y(iframe,0))
end

function IPC3d:setDesiredSpeedX(speed)
	self.pendX:setParam(speed)
end

function IPC3d:setDesiredSpeedZ(speed)
	self.pendZ:setParam(speed)
end

function IPC3d:setDesiredPosX(pos)
	self.pendX:setParamPos(pos)
end

function IPC3d.__calcCartPos(theta)
	local pos=vector3()
	pos.x=theta(1)
	pos.y=0
	pos.z=theta(0)

	return pos
end

function IPC3d:calcCartPos()
	local pos=vector3()
	pos.x=self.theta(1)
	pos.y=0
	pos.z=self.theta(0)

	return pos
end

function IPC3d:__step(ntimes)
	local xx=self.pendX.x
	local zx=self.pendZ.x
	--print('xx:'xx)
	--print('zx:',zx)

	--RE.output2('desiredvel', self.desiredVel)
	self.pendX:setParam(self.desiredVel.x)
	--self.pendZ:setParam(self.desiredVel.z)

	local pos=self:calcCartPos()
	local d=self.dtheta:toVector3(0)


	local q=self.theta:toQuater(2)
	local w=self.dtheta:toVector3(3)

	--w:rotate(q)-- if W is local angular velocities.

	projectQuaternion(q)

	local theta=q:rotationVector()

	local toLocal=self.ori:inverse()
	--RE.output2("qtheta", theta)

	pos:rotate(toLocal)
	local ldesiredPos=self.desiredPos:copy()
	ldesiredPos:rotate(toLocal)
	w:rotate(toLocal)
	theta:rotate(toLocal)
	d:rotate(toLocal)

	if pos.x~=pos.x then 
		--dbg.console('pend NaN error') 
		--error('pend NaN error')
		self.errorOccurred=true
		return
	end


	if true then -- limit speed for numerical stability
		local limitA=math.rad(55)
		theta=math.clampVec3(theta, limitA)
		d=math.clampVec3(d, 10)
		w=math.clampVec3(w, 10)
	end

	--   if self.pendX.useBogdanov04 then
	if true  then
		xx:set(0, 0, pos.x)
		xx:set(1, 0, theta.z)
		xx:set(2, 0, d.x)
		xx:set(3, 0, w.z)

		--   RE.output("x_v", tostring(posx).." "..tostring(posz) .." "..tostring(dx).." "..tostring(dz))
		--RE.output("theta_w", tostring(theta))
		zx:set(0, 0, pos.z)
		zx:set(1, 0, -theta.x)
		zx:set(2, 0, d.z)
		zx:set(3, 0, -w.x)

		if self.useDesiredPos then
			self.pendX:setParamPos(ldesiredPos.x)
			self.pendZ:setParamPos(ldesiredPos.z)
			self.pendX.useDesiredPos=true
			self.pendZ.useDesiredPos=true
		elseif self.pendX.useDesiredPos==true then

			dbg.console()
			self.pendX.useDesiredPos=false
			self.pendZ.useDesiredPos=false
			-- restore settings
			self.pendX:setQ(self.pendX.q)
			self.pendZ:setQ(self.pendZ.q)
			self.pendX:setParam(self.desiredVel.x)
			self.pendZ:setParam(self.desiredVel.z)
		end
	end

	local prevDt=   self.dt
	self.pendX.dt=prevDt*ntimes
	self.pendZ.dt=prevDt*ntimes

	--##testmw   if g_debugOneStep then
	--##testmw      g_debugOneStep:pushBack({"oneStep", xx:copy(), zx:copy()})
	--##testmw   end
	self.pendX:oneStep()
	self.pendZ:oneStep()
	--##testmw   if g_debugOneStep then
	--##testmw      g_debugOneStep:pushBack({"oneStep2", xx:copy(), zx:copy()})
	--##testmw   end
	self.pendX.dt=prevDt
	self.pendZ.dt=prevDt

	--   if self.pendX.useBogdanov04 then
	if true then
		-- convert back to 3d states
		pos.x= xx(0, 0 )
		theta.z= xx(1, 0 )
		d.x= xx(2, 0 )
		w.z= xx(3, 0 )

		pos.z= zx(0, 0 )
		theta.x= -zx(1, 0 )
		d.z= zx(2, 0 )
		w.x= -zx(3, 0 )
	end
	local cf=self.controlForce
	cf.x=self.pendX.IPC.cf(0,0)
	cf.z=self.pendZ.IPC.cf(0,0)
	local dv=self.dv
	dv.x=self.pendX.ddtheta(0,0)
	dv.z=self.pendZ.ddtheta(0,0)
	local dw=self.dw
	dw.z=self.pendX.ddtheta(1,0)
	dw.x=-self.pendZ.ddtheta(1,0)


	local toGlobal=self.ori
	pos:rotate(toGlobal)
	w:rotate(toGlobal)
	theta:rotate(toGlobal)
	d:rotate(toGlobal)
	cf:rotate(toGlobal)

	-- calc control force
	-- local cf=vector3()
	-- cf.x=controlForce_x(0,0)
	-- cf.z=controlForce_z(0,0)
	-- cf.y=0

	-- local ct=vector3()
	-- ct.z=controlForce_x(1,0)*-1
	-- ct.x=controlForce_z(1,0)
	-- ct.y=0

	if false then -- debug draw
		self.pendX.y:assign(xx:column(0):range(0,3)) 
		self.pendX:draw()
		-- self.pendZ.y:assign(zx:column(0):range(0,3)) 
		-- self.pendZ:draw()
	end

	q:setRotation(theta)

	self.theta:set(1, pos.x)
	self.theta:set(0, pos.z)
	self.theta:setQuater(2, q)


	if pos.x~=pos.x then 
		--dbg.console('pend NaN error') 
		--error('pend NaN error')
		self.errorOccurred=true
	end
	-- w=q*W
	--> W=q:inverse()*w

	-- w:rotate(q:inverse())-- if W is local angular velocities.

	self.dtheta:setVec3(0, d)
	self.dtheta:setVec3(3, w)

	--RE.output2("theta", self.theta)
	--RE.output2("dtheta", self.dtheta)
	--RE.output2("cf", self.controlforce)
	--RE.output2("theta2", self.theta)
	--##testmw   if g_debugOneStep then
	--##testmw      g_debugOneStep:pushBack({"oneStep3", self.theta:copy(), self.dtheta:copy()})
	--##testmw   end

end
function IPC3d:oneStep()

	self:__step(1)

	--		self.Y:resize(self.Y:rows()+1, 2)
	self.Y:resize(math.max(self.Y:rows(), self.numFrame+1), 6) -- do not delete predicted result
	self.numFrame=self.numFrame+1
	local lastRow=self.numFrame-1

	self.Y:row(lastRow):assign(self.theta)

	--##testmw   if g_debugOneStep then
	--##testmw      g_debugOneStep:pushBack({"oneStep4", self.theta:copy(), self.dtheta:copy()})
	--##testmw   end


end

function IPC3d:twoSteps()
	self:__step(2)

	--		self.Y:resize(self.Y:rows()+1, 2)
	self.Y:resize(math.max(self.Y:rows(), self.numFrame+2), 6) -- do not delete predicted result
	self.numFrame=self.numFrame+2
	local prevRow=math.max(self.numFrame-3,0)
	local lastRow=self.numFrame-1

	self.Y:row(lastRow):assign(self.theta)
	self.Y:row(lastRow-1):interpolate(0.5, self.Y:row(prevRow), self.theta)
end


function IPC3d:fourSteps()

	-- do 
	-- 	for i=1,4 do self:oneStep() end return
	-- end
	-- do 
	-- 	for i=1,2 do self:twoSteps() end return
	-- end

	if debug_print then
		print('fourSteps')
	end

	self:__step(4)

	--		self.Y:resize(self.Y:rows()+1, 2)
	self.Y:resize(math.max(self.Y:rows(), self.numFrame+4), 6) -- do not delete predicted result
	self.numFrame=self.numFrame+4
	local prevRow=math.max(self.numFrame-5,0)
	local lastRow=self.numFrame-1

	self.Y:row(lastRow):assign(self.theta)
	self.Y:row(lastRow-1):interpolate(0.75, self.Y:row(prevRow), self.theta)
	self.Y:row(lastRow-2):interpolate(0.5, self.Y:row(prevRow), self.theta)
	self.Y:row(lastRow-3):interpolate(0.25, self.Y:row(prevRow), self.theta)

	--for i=1,4 do self:oneStep() end

end

function IPC3d:predictTrajectory(numSteps)
	local theta=vectorn()
	local dtheta=vectorn()
	local numFrames=self:numFrames()

	self:_saveStates(theta, dtheta)

	for i=1,numSteps do
		self:oneStep()
	end

	return self:_restoreStates(theta, dtheta, numFrames)
end



function IPC3d:_saveStates(theta, dtheta)
	theta:assign(self.theta)
	dtheta:assign(self.dtheta)
	assert(not self.errorOccurred)
end

function IPC3d:_restoreStates(theta, dtheta, numFrames)
	--		local res=matrixn()
	--		local output=self.Y:range(numFrames, self.Y:rows(), 0, self.Y:cols())
	--
	--		res:setSize(output:rows(), output:cols())
	--		res:assign(output)

	-- restore states
	--		self.Y:resize(numFrames, self.Y:cols())
	self.errorOccurred=false
	self.numFrame=numFrames
	self.theta:assign(theta)
	self.dtheta:assign(dtheta)
	return res
end


function IPC3d:calcLeaning()
	local com=self:calcCOMpos()   
	local zmp=self:calcCartPos()

	local q=quater()

	local vt=vector3(0,1,0)
	local vtt=com-zmp

	q:axisToAxis(vt, vtt)

	return q:rotationAngle()
end

function IPC3d:calcPoleAngVel()
	local q=self.theta:toQuater(2)
	local w=self.dtheta:toVector3(3)

	w:rotate(q)

	return q, w
end

function IPC3d:calcCOMvel()
	local q, w =self:calcPoleAngVel()
	local v=w:cross(self.pole:localCOM())

	local m=self.pole:mass()

	local comV=self:calcCartVel()+v
	return comV
end

function IPC3d:calcCartVel()
	return self.dtheta:toVector3(0)
end

function IPC3d:__calcCOMpos(theta)
	local q=theta:toQuater(2)
	local v0=vector3(theta(1), 0, theta(0)) -- self.__calcCartPos(theta)
	local v1=vector3()
	v1:rotate(q, self.pole:localCOM())
	return v0+v1
end

function IPC3d:calcCOMpos()
	return self:__calcCOMpos(self.theta)
end
--
-- momentum of the pendulum:
-- angular momentum : Iw
-- linear momentum : m* (v+w.cross(com-pivot)) = m*(v+skew(w)*(com-pivot))
-- dot linear momentum :m*( dv+skew(dw)*(com-pivot)+skew(w)*comvel)

-- 다른 방식으로 계산해보면,
-- momentumGlobal=dAd(Inv(T_global), I*V) where V is the body velocity
-- momentumCOM=dAd(PositionCOMglobal, momentumGlobal)
-- 			  =dAd(Inv(Inv(PositionCOMglobal)*T_global), I*V)

function IPC3d:calcDotMomentum()
	--local zmp,com=self:calcZMPCOM()
	--local dotmomentum=(zmp-com):cross(self.controlForce)
	local dotmomentum=self.pendX.IPC.I*self.dw
	return dotmomentum
end

function IPC3d:calcDotLinMomentum()
	--if true then return self.controlForce:copy() end
	local w=self.dtheta:toVector3(3)
	local zmp, com=self:calcZMPCOM()
	--dbg.namedDraw("Sphere", zmp*100+vector3(50,0,0), 'pend_zmp')
	--dbg.namedDraw("Sphere", com*100+vector3(50,0,0), 'pend_com')
	local comvel=self:calcCOMvel()
	local M=self.pendX.IPC.m
	local dotlmmt=M*(self.dv+self.dw:cross(com-zmp)+w:cross(comvel))
	--print(self.controlForce, dotlmmt)
	return dotlmmt
end

function IPC3d:calcMomentum()
	local w=self.dtheta:toVector3(3)
	local v=self:calcCOMvel()
	local M=self.pendX.IPC.m
	--print(M, 'mass')
	return self.pendX.IPC.I*w, M*v 
end

function IPC3d:calcDotMomentum2( cf)
	local zmp,com=self:calcZMPCOM()
	local dotmomentum=(zmp-com):cross(cf)
	return dotmomentum
end
function IPC3d:calcCOMposFromSkel()
	return self.pole:getFrame():toGlobalPos(self.pole:localCOM())
end

function IPC3d:addExtForce(f)
	self.simulator:addForceToBone(0,self.pole, self.pole:localCOM(), f)
end

function IPC3d.__getPosition(theta)
	local v1=vector3()

	v1.x=theta(1)
	v1.y=0
	v1.z=theta(0)

	local q=theta:toQuater(2)
	return v1, q
end


function IPC3d.__getState(v1, q)
	local theta=vectorn(6)
	theta:set(0, v1.z)
	theta:set(1, v1.x)
	theta:setQuater(2,q)
	return theta
end

function IPC3d:setState_ori(zmp_pos, com_pos, zmp_vel, com_vel)

	-- exactly set horizontal com_pos and com_vel (while possibly sacrificing zmp_pos and zmp_vel)
	-- zmp+ rotate(localCOM, q)=com

	local pole=self.loader:VRMLbone(2)
	local localCOM=pole:localCOM()

	local q=quater()
	q:axisToAxis(vector3(0,1,0), com_pos-zmp_pos)

	if false then
		local qv=q:rotationVector()
		qv=math.smoothClampVec3(qv,math.rad(40))
		q:setRotation(qv)
	end
	self.theta:setQuater(2,q)

	local zmp=com_pos-rotate(localCOM, q)
	self.theta:set(0, zmp_pos.z)
	self.theta:set(1, zmp_pos.x)

	-- comPos=zmpPos+q*localCOM
	-- comVel= zmp_vel + w.cross(q*localCOM)
	-- lc*w=zmp_vel-com_vel   where lc=tilde(q*localCOM)

	local lc=matrix3()
	lc:setTilde(q*self.pole:localCOM())


	-- number of equations: 3, number of unknowns: 3 (although w.y will be clipped later due to linearization)
	-- minimum norm solution.
	local S=matrixn(3,2)
	S:setValues(lc._11, lc._13, lc._21, lc._23, lc._31, lc._33)
	local b=vectorn()
	b:setValues(zmp_vel.x-com_vel.x, zmp_vel.y-com_vel.y, zmp_vel.z-com_vel.z)
	local x=vectorn(2)
	math.PIsolve(S,b,x)

	local w=vector3()
	w.x=x(0)
	w.y=0
	w.z=x(1)

	--self.dtheta:setVec3(0, zmp_vel)
	local zmp_vel_new=com_vel+lc*w
	zmp_vel_new.y=0
	self.dtheta:setVec3(0, zmp_vel_new )
	self.dtheta:setVec3(3,w)

end

function IPC3d:setState_PI(zmp_pos, com_pos, zmp_vel, com_vel)
	-- debug
	--dbg.namedDraw('Sphere', zmp_pos*100, 'ZMPpos', 'green', 5)	
	--dbg.namedDraw('Sphere', com_pos*100, 'COMpos', 'green', 5)	
	--RE.output2('zmp_vel', zmp_vel)
	--RE.output2('com_vel', com_vel)
	self.theta:set(0, zmp_pos.z)
	self.theta:set(1, zmp_pos.x)

	local q=quater()
	q:axisToAxis(vector3(0,1,0), com_pos-zmp_pos)

	self.theta:setQuater(2, q)

	-- comPos=zmpPos+q*localCOM

	-- comVel= zmp_vel + w.cross(q*localCOM)
	-- lc*w=zmp_vel-com_vel   where lc=tilde(q*localCOM)

	local lc=matrix3()
	lc:setTilde(q*self.pole:localCOM())


	-- number of equations: 3, number of unknowns: 3 (although w.y will be clipped later due to linearization)
	-- minimum norm solution.
	local S=matrixn(3,2)
	S:setValues(lc._11, lc._13, lc._21, lc._23, lc._31, lc._33)
	local b=vectorn()
	b:setValues(zmp_vel.x-com_vel.x, zmp_vel.y-com_vel.y, zmp_vel.z-com_vel.z)
	local x=vectorn(2)
	math.PIsolve(S,b,x)

	local w=vector3()
	w.x=x(0)
	w.y=0
	w.z=x(1)

	--self.dtheta:set(0, zmp_vel.z)
	--self.dtheta:set(1, zmp_vel.x)
	self.dtheta:setVec3(0, zmp_vel)
	self.dtheta:setVec3(3,w)
end

IPC3d.setState=IPC3d.setState_ori
--IPC3d.setState=IPC3d.setState_PI
function IPC3d:setStateRaw(v1,q, dv1, dq)
	self.theta:set(0, v1.z)
	self.theta:set(1, v1.x)
	self.theta:setQuater(2, q)
	self.dtheta:setVec3(0, dv1)
	self.dtheta:setVec3(3, dq)
end

function IPC3d:getState(frame)
	local theta
	if frame then
		theta=self.Y:row(frame)
	else
		theta=self.theta
	end
	return theta
end

function IPC3d:getPosition(frame)
	return self.__getPosition(self:getState(frame))
end

IPC3d._getPosition=IPC3d.getPosition -- for compatibility to CartPoleBall
function IPC3d:setSkeleton(frame)
	self.loader:setPoseDOF(self:getState(frame))
end

function IPC3d:draw()   
	self.skin:setPoseDOF(self.theta)

	local v=self:getPosition()*100
	--dbg.draw('Arrow', v, v+self.controlForce*0.1, "controlForce")
end

function IPC3d:setVisualTranslation(x,y,z)
	self.skin:setTranslation(x,y,z)
end

function IPC3d:getStates()
	local states={}
	states.x=vectorn()
	states.dx=vectorn()
	states.numFrames=self:numFrames()
	self:_saveStates(states.x, states.dx)
	states.ori=self.ori:copy()
	states.comHeight=self.pole:localCOM().y
	--states.inertia : unused
	return states
end

function IPC3d:restoreStates(states)
	self.pole:setLocalCOM(vector3(0,states.comHeight,0))
	self:setInertia(vector3(0, states.comHeight,0))
	self:_restoreStates(states.x, states.dx, states.numFrames)
	self.ori:assign(states.ori)
end
function IPC3d:drawFrames(startFrame, objectList)
	--local lines=vector3N()
	--for i=startFrame, self.numFrame-1 do
	--local cart, poleOri=self:_getPosition(i)
	--local pole=cart+self:_calcTip(poleOri)
	--
	--lines:pushBack(cart)
	--lines:pushBack(pole)
	--end
	--
	--if lines:rows()~=0 then
	--objectList:registerObject("zmpPredict2", "LineList", "solidred", 
	--lines:matView()*100,0)
	--end

end

function IPC3d:calcZMPCOM(iframe)

	local theta=self:getState(iframe)

	local COM=self:__calcCOMpos(theta)   
	local cart=self.__calcCartPos(theta)
	return cart,COM
end
if IPC3d_cpp then
	function IPC3d_cpp.__getPosition(theta)
		local v1=vector3()
		local q=quater()
		IPC3d_cpp.__getPosition2(theta, v1,q)
		return v1,q
	end
	function IPC3d_cpp.__getState(v1,q)
		local theta=vectorn()
		IPC3d_cpp.__getState2(theta,v1,q)
		return theta
	end
	function IPC3d_cpp:getPosition(frame)
		return self.__getPosition(self:getState(frame))
	end
	IPC3d_cpp._getPosition=IPC3d_cpp.getPosition -- for compatibility to CartPoleBall
	IPC3d_cpp.getState=IPC3d.getState
	IPC3d_cpp.getPos=IPC3d.getPos
	IPC3d_cpp.setVisualTranslation=IPC3d.setVisualTranslation
	IPC3d_cpp.drawFrames=IPC3d.drawFrames
	IPC3d_cpp.calcZMPCOM=IPC3d.calcZMPCOM
	IPC3d_cpp.restoreStates=IPC3d.restoreStates
	IPC3d_cpp.getStates=IPC3d.getStates
end
