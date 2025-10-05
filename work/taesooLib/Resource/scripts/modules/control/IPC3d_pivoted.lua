require("control/SDRE")
require("control/IPC3d_approx")


--class 'IPC3d_pivoted'
IPC3d_pivoted=LUAclass()

function IPC3d_pivoted:__deinit()
   if self.skin~=nil then
      RE.remove(self.skin)
      self.skin=nil
   end
end

function IPC3d_pivoted:__finalize()
   self.skin=nil
   self.loader=nil
end

function IPC3d_pivoted:__init(filename, b,g,dt,q)
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
   

   self.dtheta=vectorn(6) -- (w1)
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
	self.pendX=SPIPM(m, i, g, l, q, q, dt)
	self.pendZ=SPIPM(m, i, g, l, q, q, dt)
	self.q=q
	self.dt=dt
	self:initState()
end
function IPC3d_pivoted:initState()
	self.ori=quater()	-- cart orientation
	self.ori:setValue(1,0,0,0)
	self.desiredPos=vector3(0,0,0)
	self.useDesiredPos=false
	self.dw=vector3(0,0,0)
   self.Y=matrixn()
   self.numFrame=0
end

function IPC3d_pivoted:setOrientation2(q)
   self.ori:assign(q)
end

function IPC3d_pivoted:setOrientation(angle)
   self:setOrientation2(quater(math.rad(angle), vector3(0,1,0)))
end

function projectQuaternion(q)
   local qoffset=quater()
   local qaxis=quater()

   q:decomposeNoTwistTimesTwist(vector3(0,1,0), qoffset, qaxis)
   q:assign(qoffset)
end



function IPC3d_pivoted:setDesiredVelocity(vel) -- vector3
end

function IPC3d_pivoted:setInertia(inertia) -- local inertia

	-- TODO
	--local comHeight=self.pole:localCOM().y
	--local pendX=self.pendX
	--local pendZ=self.pendZ
	--pendX:setInertia(comHeight, inertia.x)
	--pendX:updateSDRE(CT.vec(0),CT.vec(0))
	--pendZ:setInertia(comHeight, inertia.z)
	--pendZ:updateSDRE(CT.vec(0),CT.vec(0))
end

function IPC3d_pivoted:setDesiredPosition(pos) -- vector3
end
function IPC3d_pivoted:setDesiredOrientation(theta)
	--local theta=q:rotationVector()
	--local toLocal=self.ori:inverse()
	--theta:rotate(toLocal)
	self.pendX:setDesiredState(theta.z,0)
	self.pendZ:setDesiredState(-theta.x,0)
end

function IPC3d_pivoted:numFrames()
   --		return self.Y:rows()
   return self.numFrame
end

function IPC3d_pivoted:getPos(iframe)
   
   return vector3(self.Y(iframe,1), 0, self.Y(iframe,0))
end

function IPC3d_pivoted:setDesiredSpeedX(speed)
end

function IPC3d_pivoted:setDesiredSpeedZ(speed)
end

function IPC3d_pivoted.__calcCartPos(theta)
   local pos=vector3()
   pos.x=theta(1)
   pos.y=0
   pos.z=theta(0)

   return pos
end

function IPC3d_pivoted:calcCartPos()
   local pos=vector3()
   pos.x=self.theta(1)
   pos.y=0
   pos.z=self.theta(0)

   return pos
end


function IPC3d_pivoted:__step(ntimes)
   local xx=self.pendX.x
   local zx=self.pendZ.x

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
	   error('pend NaN error')
	   self.errorOccurred=true
   end


   if true then -- limit speed for numerical stability
	   local limitA=math.rad(55)
	   theta=math.clampVec3(theta, limitA)
	   d=math.clampVec3(d, 10)
	   w=math.clampVec3(w, 10)
   end

   if true  then
      --xx:set(0, 0, pos.x)
      xx:set(0, 0, theta.z)
      --xx:set(2, 0, d.x)
      xx:set(1, 0, w.z)
      
      --   RE.output("x_v", tostring(posx).." "..tostring(posz) .." "..tostring(dx).." "..tostring(dz))
      --RE.output("theta_w", tostring(theta))
      --zx:set(0, 0, pos.z)
      zx:set(0, 0, -theta.x)
      --zx:set(2, 0, d.z)
      zx:set(1, 0, -w.x)
   end

   local prevDt=   self.dt
   self.pendX.dt=prevDt*ntimes
   self.pendZ.dt=prevDt*ntimes

   --##testmw   if g_debugOneStep then
   --##testmw      g_debugOneStep:pushBack({"oneStep", xx:copy(), zx:copy()})
   --##testmw   end
   self.pendX:singleStep()
   self.pendZ:singleStep()
   --##testmw   if g_debugOneStep then
   --##testmw      g_debugOneStep:pushBack({"oneStep2", xx:copy(), zx:copy()})
   --##testmw   end
   self.pendX.dt=prevDt
   self.pendZ.dt=prevDt

--   if self.pendX.useBogdanov04 then
   if true then
      -- convert back to 3d states
      theta.z= xx(0, 0 )
      w.z= xx(1, 0 )

      theta.x= -zx(0, 0 )
      w.x= -zx(1, 0 )
   end
   local cf=self.dw
   cf.z=self.pendX.ddtheta(0,0)
   cf.x=-self.pendZ.ddtheta(0,0)

   local toGlobal=self.ori
   pos:rotate(toGlobal)
   w:rotate(toGlobal)
   theta:rotate(toGlobal)
   d:rotate(toGlobal)
   cf:rotate(toGlobal)

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
function IPC3d_pivoted:oneStep()
   
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

function IPC3d_pivoted:twoSteps()
   self:__step(2)

--		self.Y:resize(self.Y:rows()+1, 2)
   self.Y:resize(math.max(self.Y:rows(), self.numFrame+2), 6) -- do not delete predicted result
   self.numFrame=self.numFrame+2
   local prevRow=math.max(self.numFrame-3,0)
   local lastRow=self.numFrame-1
   
   self.Y:row(lastRow):assign(self.theta)
   self.Y:row(lastRow-1):interpolate(0.5, self.Y:row(prevRow), self.theta)
end


function IPC3d_pivoted:fourSteps()

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

function IPC3d_pivoted:predictTrajectory(numSteps)
   local theta=vectorn()
   local dtheta=vectorn()
   local numFrames=self:numFrames()
   
   self:_saveStates(theta, dtheta)
   
   for i=1,numSteps do
      self:oneStep()
   end
   
   return self:_restoreStates(theta, dtheta, numFrames)
end



function IPC3d_pivoted:_saveStates(theta, dtheta)
   theta:assign(self.theta)
   dtheta:assign(self.dtheta)
   assert(not self.errorOccurred)
end

function IPC3d_pivoted:_restoreStates(theta, dtheta, numFrames)
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


function IPC3d_pivoted:calcLeaning()
   local com=self:calcCOMpos()   
   local zmp=self:calcCartPos()
   
   local q=quater()

   local vt=vector3(0,1,0)
   local vtt=com-zmp

   q:axisToAxis(vt, vtt)

   return q:rotationAngle()
end

function IPC3d_pivoted:calcPoleAngVel()
   local q=self.theta:toQuater(2)
   local w=self.dtheta:toVector3(3)

   w:rotate(q)

   return q, w
end

function IPC3d_pivoted:calcCOMvel()
   local q, w =self:calcPoleAngVel()
   local v=w:cross(self.pole:localCOM())

   local m=self.pole:mass()

   local comV=self:calcCartVel()+v
   return comV
end


function IPC3d_pivoted:calcCartVel()
   return self.dtheta:toVector3(0)
end

function IPC3d_pivoted:__calcCOMpos(theta)
   local q=theta:toQuater(2)
   local v0=vector3(theta(1), 0, theta(0)) -- self.__calcCartPos(theta)
   local v1=vector3()
   v1:rotate(q, self.pole:localCOM())
   return v0+v1
end

function IPC3d_pivoted:calcCOMpos()
   return self:__calcCOMpos(self.theta)
end

-- momentum of the pivoted pendulum:
-- angular momentum : Iw
-- linear momentum : m* w.cross(com-pivot) = skew(w)*(com-pivot)
-- dot linear momentum : skew(dw)*(com-pivot)+skew(w)*comvel

function IPC3d_pivoted:calcMomentum()
	local w=self.dtheta:toVector3(3)
	local v=self:calcCOMvel()
	local M=self.pendX.M
	--print(M, 'mass')
	return self.pendX.I*w, M*v 
end
function IPC3d_pivoted:calcDotMomentum()
	local pendx=self.pendX
	return pendx.I*self.dw
end

function IPC3d_pivoted:calcDotLinMomentum()
	local w=self.dtheta:toVector3(3)
	local zmp, com=self:calcZMPCOM()
	--dbg.namedDraw("Sphere", zmp*100+vector3(50,0,0), 'pend_zmp')
	--dbg.namedDraw("Sphere", com*100+vector3(50,0,0), 'pend_com')
	local comvel=self:calcCOMvel()
	local M=self.pendX.M
	local dotlmmt=M*(self.dw:cross(com-zmp)+w:cross(comvel))
	return dotlmmt
end

function IPC3d_pivoted:calcCOMposFromSkel()
   return self.pole:getFrame():toGlobalPos(self.pole:localCOM())
end

function IPC3d_pivoted.__getPosition(theta)
   local v1=vector3()

   v1.x=theta(1)
   v1.y=0
   v1.z=theta(0)
   
   local q=theta:toQuater(2)
   return v1, q
end


function IPC3d_pivoted.__getState(v1, q)
   local theta=vectorn(6)
   theta:set(0, v1.z)
   theta:set(1, v1.x)
   theta:setQuater(2,q)
   return theta
end

function IPC3d_pivoted:setState_ori(zmp_pos, com_pos, zmp_vel, com_vel)
	
	--dbg.namedDraw('Sphere', zmp_pos*100, 'ZMPpos', 'green', 5)	
	--dbg.namedDraw('Sphere', com_pos*100, 'COMpos', 'green', 5)	
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

function IPC3d_pivoted:setState_PI(zmp_pos, com_pos, zmp_vel, com_vel)
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

IPC3d_pivoted.setState=IPC3d_pivoted.setState_ori
--IPC3d_pivoted.setState=IPC3d_pivoted.setState_PI
function IPC3d_pivoted:setStateRaw(v1,q, dv1, dq)
   self.theta:set(0, v1.z)
   self.theta:set(1, v1.x)
   self.theta:setQuater(2, q)
   self.dtheta:setVec3(0, dv1)
   self.dtheta:setVec3(3, dq)
end

function IPC3d_pivoted:getState(frame)
   local theta
   if frame then
      theta=self.Y:row(frame)
   else
      theta=self.theta
   end
   return theta
end

function IPC3d_pivoted:getPosition(frame)
   return self.__getPosition(self:getState(frame))
end

IPC3d_pivoted._getPosition=IPC3d_pivoted.getPosition -- for compatibility to CartPoleBall
function IPC3d_pivoted:setSkeleton(frame)
   self.loader:setPoseDOF(self:getState(frame))
end

function IPC3d_pivoted:draw()   
   self.skin:setPoseDOF(self.theta)

   local v=self:getPosition()*100
   dbg.draw('Arrow', v, v+self.dw*0.1, "controlForce")
end

function IPC3d_pivoted:setVisualTranslation(x,y,z)
	self.skin:setTranslation(x,y,z)
end

function IPC3d_pivoted:getStates()
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

function IPC3d_pivoted:restoreStates(states)
	self.pole:setLocalCOM(vector3(0,states.comHeight,0))
	self:setInertia(vector3(0, states.comHeight,0))
	self:_restoreStates(states.x, states.dx, states.numFrames)
	self.ori:assign(states.ori)
end
function IPC3d_pivoted:drawFrames(startFrame, objectList)
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

function IPC3d_pivoted:calcZMPCOM(iframe)
   
   local theta=self:getState(iframe)

   local COM=self:__calcCOMpos(theta)   
   local cart=self.__calcCartPos(theta)
   return cart,COM
end
IPC3d_multimodal=LUAclass()

function IPC3d_multimodal:changeMode(usePositionControl)
	if usePositionControl then
		if self.mode==1 then
			self.ipc_pivoted.numFrame=self.ipc.numFrame
		end
		self.mode=2
	else
		if self.mode==2 then 
			self.ipc.numFrame=self.ipc_pivoted.numFrame
		end
		self.mode=1
	end
end
function IPC3d_multimodal:__deinit()
end


function IPC3d_multimodal:__finalize()
	self.ipc=nil
	self.ipc_pivoted=nil
	self.skin=nil
	self.theta=nil
	self.dtheta=nil
	self.pole=nil
	self.cart=nil
end



function IPC3d_multimodal:__init(filename, b,g,dt,q)
	self.ipc=IPC3d(filename,b,g,dt,q)
	self.ipc_pivoted=IPC3d_pivoted(filename,b,g,dt,q)
	self:initState()
end

function IPC3d_multimodal:initState()
	self.ipc:initState()
	self.ipc_pivoted:initState()
	
	self.loader=self.ipc.loader
	self.skin=self.ipc.skin
	self.theta=self.ipc.theta
	self.dtheta=self.ipc.dtheta
	self.pole=self.ipc.pole
	self.cart=self.ipc.cart
	self.Y=self.ipc.Y

	self.ipc_pivoted.loader=self.loader
	self.ipc_pivoted.skin=self.skin
	self.ipc_pivoted.theta =self.theta 
	self.ipc_pivoted.dtheta=self.dtheta
	self.ipc_pivoted.pole  =self.pole  
	self.ipc_pivoted.cart  =self.cart  
	self.ipc_pivoted.Y=self.Y

	self.mode=1
end

function IPC3d_multimodal:setOrientation2(q)
	if self.mode==1 then
		self.ipc:setOrientation2(q)
	else
		self.ipc_pivoted:setOrientation2(q)
	end
end

function IPC3d_multimodal:removeSkin()
	self.skin=nil
	self.ipc.skin=nil
	self.ipc_pivoted.skin=nil
	collectgarbage('collect')
end
function IPC3d_multimodal:setOrientation(angle)
   self:setOrientation2(quater(math.rad(angle), vector3(0,1,0)))
end

function IPC3d_multimodal:setDesiredVelocity(vel) -- vector3
	if self.mode==1 then
		self.ipc:setDesiredVelocity(vel)
	end
end

function IPC3d_multimodal:setInertia(inertia) -- local inertia
	self.ipc:setInertia(inertia)
	self.ipc_pivoted:setInertia(inertia)
end

function IPC3d_multimodal:setDesiredPosition(pos) -- vector3
end
function IPC3d_multimodal:setDesiredOrientation(theta)
	if self.mode==2 then
		self.ipc_pivoted:setDesiredOrientation(theta)
	end
end

function IPC3d_multimodal:numFrames()
	if self.mode==1 then
		return self.ipc:numFrames()
	else
		return self.ipc_pivoted:numFrames()
	end
end

function IPC3d_multimodal:getPos(iframe)
	if self.mode==1 then
		return self.ipc:getPos(iframe)
	else
		return self.ipc_pivoted:getPos(iframe)
	end
end

function IPC3d_multimodal:setDesiredSpeedX(speed)
end

function IPC3d_multimodal:setDesiredSpeedZ(speed)
end
function IPC3d_multimodal:oneStep()
	if self.mode==1 then
		self.ipc:oneStep()
	else
		self.ipc_pivoted:oneStep()
	end
end

function IPC3d_multimodal:twoSteps()
	if self.mode==1 then
		self.ipc:twoSteps()
	else
		self.ipc_pivoted:twoSteps()
	end
end


function IPC3d_multimodal:fourSteps()
	if self.mode==1 then
		self.ipc:fourSteps()
	else
		self.ipc_pivoted:fourSteps()
	end
end

function IPC3d_multimodal:predictTrajectory(numSteps)
	if self.mode==1 then
		self.ipc:predictTrajectory(numSteps)
	else
		self.ipc_pivoted:predictTrajectory(numSteps)
	end
end

function IPC3d_multimodal:_saveStates(theta, dtheta)
	if self.mode==1 then
		self.ipc:_saveStates(theta, dtheta)
	else
		self.ipc_pivoted:_saveStates(theta, dtheta)
	end
end

function IPC3d_multimodal:_restoreStates(theta, dtheta, numFrames)
	if self.mode==1 then
		return self.ipc:_restoreStates(theta, dtheta, numFrames)
	else
		return self.ipc_pivoted:_restoreStates(theta, dtheta, numFrames)
	end
end


function IPC3d_multimodal:calcLeaning()
	if self.mode==1 then
		return self.ipc:calcLeaning()
	else
		return self.ipc_pivoted:calcLeaning()
	end
end

function IPC3d_multimodal:calcPoleAngVel()
	if self.mode==1 then
		return self.ipc:calcPoleAngVel()
	else
		return self.ipc_pivoted:calcPoleAngVel()
	end
end

function IPC3d_multimodal:calcCOMvel()
	if self.mode==1 then
		return self.ipc:calcCOMvel()
	else
		return self.ipc_pivoted:calcCOMvel()
	end
end


function IPC3d_multimodal:calcCartVel()
	if self.mode==1 then
		return self.ipc:calcCartVel()
	else
		return self.ipc_pivoted:calcCartVel()
	end
end


-- momentum of the multimodal pendulum:
-- angular momentum : Iw
-- linear momentum : m* w.cross(com-pivot) = skew(w)*(com-pivot)
-- dot linear momentum : skew(dw)*(com-pivot)+skew(w)*comvel

function IPC3d_multimodal:calcMomentum()
	if self.mode==1 then
		return self.ipc:calcMomentum()
	else
		return self.ipc_pivoted:calcMomentum()
	end
end
function IPC3d_multimodal:calcDotMomentum()
	if self.mode==1 then
		return self.ipc:calcDotMomentum()
	else
		return self.ipc_pivoted:calcDotMomentum()
	end
end

function IPC3d_multimodal:calcDotLinMomentum()
	if self.mode==1 then
		return self.ipc:calcDotLinMomentum()
	else
		return self.ipc_pivoted:calcDotLinMomentum()
	end
end

function IPC3d_multimodal:calcCOMposFromSkel()
	if self.mode==1 then
		return self.ipc:calcCOMposFromSkel()
	else
		return self.ipc_pivoted:calcCOMposFromSkel()
	end
end

function IPC3d_multimodal:setState(zmp_pos, com_pos, zmp_vel, com_vel)
	if self.mode==1 then
		self.ipc:setState(zmp_pos, com_pos, zmp_vel, com_vel)
	else
		self.ipc_pivoted:setState(zmp_pos, com_pos, zmp_vel, com_vel)
	end
end
function IPC3d_multimodal:setStateRaw(v1,q, dv1, dq)
	if self.mode==1 then
		self.ipc:setStateRaw(v1,q,dv1,dq)
	else
		self.ipc_pivoted:setStateRaw(v1,q,dv1,dq)
	end
end

function IPC3d_multimodal:getState(frame)
	if self.mode==1 then
		return self.ipc:getState(frame)
	else
		return self.ipc_pivoted:getState(frame)
	end
end

function IPC3d_multimodal:getPosition(frame)
	if self.mode==1 then
		return self.ipc:getPosition(frame)
	else
		return self.ipc_pivoted:getPosition(frame)
	end
end

function IPC3d_multimodal:setSkeleton(frame)
	if self.mode==1 then
		return self.ipc:setSkeleton(frame)
	else
		return self.ipc_pivoted:setSkeleton(frame)
	end
end

function IPC3d_multimodal:draw()   
	if self.mode==1 then
		self.ipc:draw()
	else
		self.ipc_pivoted:draw()
	end
end

function IPC3d_multimodal:setVisualTranslation(x,y,z)
	self.ipc:setVisualTranslation(x,y,z)
	self.ipc_pivoted:setVisualTranslation(x,y,z)
end

function IPC3d_multimodal:getStates()
	local states={}
	states.mode=self.mode
	states.st1=self.ipc:getStates()
	states.st2=self.ipc_pivoted:getStates()
	return states
end

function IPC3d_multimodal:restoreStates(states)
	self.mode=states.mode
	self.ipc:restoreStates(states.st1)
	self.ipc_pivoted:restoreStates(states.st2)
end

IPC3d_multimodal._getPosition=IPC3d_multimodal.getPosition -- for compatibility to CartPoleBall
IPC3d_multimodal.__getPosition=IPC3d_pivoted.__getPosition
IPC3d_multimodal.__calcCOMpos=IPC3d_pivoted.__calcCOMpos
IPC3d_multimodal.calcCOMpos=IPC3d_pivoted.calcCOMpos

function IPC3d_multimodal:drawFrames(startFrame, objectList)
end

function IPC3d_multimodal:calcCartPos()
	return self.ipc:calcCartPos()
end
function IPC3d_multimodal:calcZMPCOM(iframe)
	if self.mode==1 then
		return self.ipc:calcZMPCOM(iframe)
	else
		return self.ipc_pivoted:calcZMPCOM(iframe)
	end
end
