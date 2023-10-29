require('control/LQR')
--M=require('scilua')

function LQR_wrap(A,B,Q,R)

   --local k=octave:call("lqr",1,{A,B,Q,R})[1]    -- octave implementation
   --local k=lqr(A,B,Q,R) -- lua implementation ported from octave implementation
   local k=matrixn() Physics.LQR(k,A,B,Q,R) -- cpp implementation ported from lua implementation

   return k
end

--class 'NonlinearController'
NonlinearController=LUAclass()

function NonlinearController:__init(dim, cdim) -- dim: # of generalized coordinates
   -- D ddot(theta) + C dot(theta) + G (theta)= H u
   self.D=matrixn(dim,dim)
   self.C=matrixn(dim,dim)
   self.G=vectorn(dim)
   self.H=matrixn(dim,cdim)
   self.dim=dim
   self.cdim=cdim
   self.dGdTheta_0=vectorn(dim)
   self.Gsd=vectorn(dim)

   -- outputs of update functions
   --: dot(x)=Ax+Bu
   self.invD=matrixn(dim,dim)
   self.A=matrixn(dim*2,dim*2)
   self.B=matrixn(dim*2,cdim)
   self.A:range(0, dim, 0, dim*2):setAllValue(0)
   self.A:range(0, dim, dim, dim*2):diag():setAllValue(1)
   self.B:range(0, dim, 0, cdim):setAllValue(0)
end

function NonlinearController:calcControlForce(x,U,Q,R,K, xd)
   local dim=self.dim
   self:updateSDRE(x:column(0):range(0,dim), x:column(0):range(dim, dim*2))

   local useSDRE=false

   if useSDRE then
      K:assign(LQR_wrap(self.A,self.B,Q,R))
   end

   --##testmw   if g_debugOneStep then
      --##testmw      g_debugOneStep:pushBack({"ccf", self.D:copy(), self.invD:copy(), self.C:copy(), self.A:copy(), self.B:copy(), Q:copy(), R:copy(), K:copy(), xd:copy(), x:copy()})
      --##testmw   end

   U:assign(K*xd)

   return U-K*x
end


function NonlinearController:oneStep(x,U,dt,maxForce,Q,R,K, xd)
   local dim=self.dim
   self.cf=self:calcControlForce(x,U,Q,R,K,xd)
   local cf=self.cf
   return self:oneStepRaw(x,cf,dt,maxForce,Q,R,K, xd)
end
function NonlinearController:oneStepRaw(x,cf,dt,maxForce,Q,R,K, xd)

   local dim=self.dim

   function clamp(mat, val)
      if val then

	 for i=0,mat:rows()-1 do
	    for j=0,mat:cols()-1 do
	       local v=mat(i,j)
	       if v>val then
		  mat:set(i,j,val)
	       elseif v<val*-1 then
		  mat:set(i,j,val*-1)
	       end
	    end
	 end

      end
      return mat
   end


   local useLSim=false

   if useLSim then
      --local k=octave:call("lsim_test",2,{self.A,self.B,dt})
      --local Ad=k[1]
      --local Bd=k[2]
	  local Ad=M.fromMatrixn(self.A)
	  local Bd=M.fromMatrixn(self.B)
	  M.c2d(Ad,Bd,dt)
	  Ad=M.toMatrixn(Ad)
	  Bd=M.toMatrixn(Bd)

      x:assign(Ad*x+Bd*clamp(cf, maxForce))
   elseif false then -- use A, B
      local xdot=self.A*x
      -- add control force
      if ctrlForceScale then
	 xdot:radd(self.B*clamp(cf, maxForce)*ctrlForceScale)	 
      else
	 xdot:radd(self.B*clamp(cf, maxForce))
      end

--      dbg.console()
      x:radd(xdot*dt)   
   else -- use D, C, G : the most accurate
      -- using D ddtheta + C dtheta + G = Hu
      local theta=x:range(0, dim,0,1)
      local dtheta=x:range(dim, dim*2,0,1)
      --##testmw      if g_debugOneStep then
	 --##testmw	 g_debugOneStep:pushBack({"theta", dtheta:copy(), theta:copy()})
	 --##testmw      end

      local Dddtheta
      if ctrlForceScale then -- U : K*x_d
	 Dddtheta=self.H*clamp(cf, maxForce)*ctrlForceScale - self.G:column() - self.C*dtheta
      else
	 Dddtheta=self.H*clamp(cf, maxForce) - self.G:column() - self.C*dtheta
      end

	  local ddtheta=self.invD*Dddtheta
      dtheta:radd(ddtheta*dt)
      theta:radd(dtheta*dt)
      --##testmw      if g_debugOneStep then
	 --##testmw	 g_debugOneStep:pushBack({"theta2", dtheta:copy(), theta:copy(),Dddtheta:copy(), cf:copy(), self.H:copy(), self.G:copy(), dt})
	 --##testmw      end

      -- if theta(1,0)>3.14 then
      -- 	 theta:set(1,0, theta(1,0)-3.14*2)
      -- elseif theta(1,0)<-3.14 then
      -- 	 theta:set(1,0, theta(1,0)+3.14*2)
      -- end

      -- if theta(2,0)>3.14 then
      -- 	 theta:set(2,0, theta(2,0)-3.14*2)
      -- elseif theta(2,0)<-3.14 then
      -- 	 theta:set(2,0, theta(2,0)+3.14*2)
      -- end

	  return ddtheta
   end
end

function NonlinearController:_updateSDRE(theta)
   -- assumes that D, C, G is already calculated.
--   self.invD:pseudoInverse(self.D)

   if not pcall(function () self.invD:inverse(self.D) end) then
      print('error')
      --##testmw      fineLog("inverse error?")
      --##testmw      if g_debugOneStep then
	 --##testmw	 fineLog("inverse error???"..tostring(self.D))
	 --##testmw	 g_debugOneStep:pushBack({"2", self.D:copy(), self.invD:copy()})
	 --##testmw      end

      error('inverse error')
   end
   local dim=self.dim
   local cdim=self.cdim

   local Gsd=self.Gsd
   local G=self.G
   local dGdTheta_0=self.dGdTheta_0

   local thr=0.0001
   for i=0, Gsd:size()-1 do
      if theta(i)<thr then
	 Gsd:set(i, dGdTheta_0(i))
      else
	 Gsd:set(i, G(i)/theta(i))
      end
   end

   local a3=self.A:range(dim, dim*2, 0, dim)
   a3:multAdiagB(self.invD, Gsd)
   a3:rmult(-1)

   local a4=self.A:range(dim, dim*2, dim, dim*2)
   a4:mult(self.invD, self.C)
   a4:rmult(-1)

   local b2=self.B:range(dim, dim*2,0, cdim)
   b2:mult(self.invD, self.H)


end



--class 'DIPC' (NonlinearController)
DIPC=LUAclass(NonlinearController)

-- m0: cart mass, m1: pole1, m2: pole2, l1: cart-to-pole1_COM, l2:hip-to-pole2_COM
function DIPC:__init(m0,m1,m2,l1,l2,I1,I2)

   local cdim=2 -- 1 or 2 or 3
   NonlinearController.__init(self, 3,cdim)

   self.m0=m0
   self.m1=m1
   self.m2=m2
   self.l1=l1
   self.l2=l2
   self.I1=I1
   self.I2=I2
   local g=9.8



   -- D ddot(theta) + C dot(theta) + G (theta)= H u

   if cdim==1 then
      self.H:setValues(1,0,0)
   elseif cdim==2 then
      self.H:setValues(1,0,   -- * (force, hipTorque)^T
		       0,-1, 
		       0,1)
   else
      self.H:setValues(1,0,0, 
		       0,1,0, 
		       0,1,1)
   end

   self.useBogdanov04=true
   
   if self.useBogdanov04 then -- bogdanov04 
      local f1=(m1*l1+m2*l1*2)*g
      local f2=m2*l2*g
      self.dGdTheta_0:setValues(0, -f1, -f2)
   else
      -- modified version of bogdanov04
      local d2=m1*l1+m2*l1*2
      local d3=m2*l2
      self.dGdTheta_0:setValues(0, (-d2-d3)*g, -d3*g) -- doesn't work because dynamic equation is not presentable in SDC form.
   end
end

function DIPC:updateSDRE(theta, dtheta)
   local cos=math.cos
   local sin=math.sin

   local g=9.8

   if self.useBogdanov04 then
      -- This implementation is based on [bogdanov04]. 
      local m0=self.m0
      local m1=self.m1
      local m2=self.m2
      local l1=self.l1
      local L1=l1*2
      local l2=self.l2
      local I1=self.I1
      local I2=self.I2

      local d1=m0+m1+m2
      local d2=m1*l1+m2*L1
      local d3=m2*l2
      local d4=m1*l1*l1+m2*L1*L1+I1
      local d5=m2*L1*l2
      local d6=m2*l2*l2+I2
      local f1=(m1*l1+m2*L1)*g
      local f2=m2*l2*g

      local theta1=theta(1)
      local theta2=theta(2)
      local dtheta1=dtheta(1)
      local dtheta2=dtheta(2)
      self.D:setSymmetric(d1, d2*cos(theta1), d3*cos(theta2),
			  d4, d5*cos(theta1-theta2),
			  d6)


      self.C:setValues(0, -d2*sin(theta1)*dtheta1, -d3*sin(theta2)*dtheta2,
		       0,0,d5*sin(theta1-theta2)*dtheta2,
		       0, -d5*sin(theta1-theta2)*dtheta1, 0)

      self.G:setValues(0, -f1*sin(theta1), -f2*sin(theta2))
   elseif true then
      -- reformulation of [bogdanov04] such that hip joint uses relative orientation. 
      -- This performs worse than Bogdanov04 because dynamic equations are not presentable in SDC form.
      local m0=self.m0
      local m1=self.m1
      local m2=self.m2
      local l1=self.l1
      local L1=l1*2
      local l2=self.l2
      local I1=self.I1
      local I2=self.I2

      local d1=m0+m1+m2
      local d2=m1*l1+m2*L1
      local d3=m2*l2
      local d4=m1*l1*l1+m2*L1*L1+I1
      local d5=m2*L1*l2
      local d6=m2*l2*l2+I2

      local y=theta(1)
      local z=theta(2)
      local dy=dtheta(1)
      local dz=dtheta(2)
      
      self.D:setValues(d1, d2*cos(y)+d3*cos(y+z), d3*cos(y+z),
		       d2*cos(y)+d3*cos(y+z),d4+2*d5*cos(z)+d6, d6+d5*cos(z),
		       d3*cos(y+z), d5*cos(z)+d6, d6)

      self.C:setValues(0, -d2*sin(y)*dy-d3*sin(y+z)*(dy+dz), -d3*sin(y+z)*(dy+dz),
		       0, -d5*sin(z)*dz,-d5*sin(z)*(dy+dz),
		       0, d5*sin(z)*dy, 0)
      
      self.G:setValues(0, -d2*g*sin(y)-d3*g*sin(y+z), -d3*g*sin(y+z))
  
   else
      -- some crappy paper.. I found the hard way that this derivation is incorrect.

      local m0=self.m0
      local M1=self.m1
      local M2=self.m2
      local g1=self.l1
      local l1=g1*2
      local g2=self.l2
      local l2=g2*2
      local I1=self.I1
      local I2=self.I2

      local d1=m0+M1+M2 
      local d2=M1*g1+M2*l1
      local d3=M2*g2
      local d4=M1*g1*g1+I1+M2*l1*l1
      local d5=d3*l1
      local d6=d3*g2+I2

      local function SQR(x) return x*x end

      -- EQN1:
      -- d1 ddx 
      -- - (d2cos(theta1)+d3cos(theta1+theta2)) ddtheta1
      -- -d3 cos(theta1+theta2)ddtheta2
      -- +d2*sin(theta1)SQR(dtheta1)
      -- +d3*SQR(dtheta1+dtheta2)sin(theta1+theta2)=F


      -- EQN2:
      -- -(d2cos(theta1)+d3cos(theta1+theta2)) ddx
      -- -(d4+2d5cos(theta2)+d6)ddtheta1
      -- +(d6+d5cos(theta2))ddtheta2
      -- - 2d5 sin(theta2) dtheta1 dtheta2
      -- - d5 sin(theta2) SQR(dtheta2)
      -- - g(d3 sin(theta1+theta2)+d2sin(theta2))=0

      -- EQN3:
      -- -d3 cos(theta1+theta2) ddx + (d5 cos(theta2)+d6)ddtheta1
      -- +d6 ddtheta2
      -- +d5 sin(theta2)SQR(dtheta1)
      -- -g d3 sin(theta1+theta2)=0

      local theta1=theta(1)
      local theta2=theta(2)
      local dtheta1=dtheta(1)
      local dtheta2=dtheta(2)
      self.D:setValues(d1, -(d2*cos(theta1)+d3*cos(theta1+theta2)), -d3*cos(theta1+theta2),
		 -(d2*cos(theta1)+d3*cos(theta1+theta2)), -( d4+2*d5*cos(theta2)+d6), d6+d5*cos(theta2),
	-d3*cos(theta1+theta2), d6+d5*cos(theta2), d6)


      self.C:setValues(0, d2*sin(theta1)*dtheta1+d3*(dtheta1+dtheta2)*sin(theta1+theta2), d3*(dtheta2+dtheta1)*sin(theta1+theta2),
		       0,-d5*sin(theta2)*dtheta2, -d5*sin(theta2)*(dtheta1+dtheta2),
		       0, d5*sin(theta2)*dtheta1, 0)

      self.G:setValues(0, -g*(d3*sin(theta1+theta2)+d2*sin(theta2)), -g*d3*sin(theta1+theta2))
   end
   self:_updateSDRE(theta)
end

-- Stationary pivot point inverted pendulum model
SPIPM=LUAclass(NonlinearController)

function SPIPM:__init(M, I, g, l, q, qd, dt)
	self.M=M
	self.I=I
	self.g=g
	self.l=l

	self.t=0
	self.dt=dt
	--I*ddot(theta)-mgl sin(theta)=0
	-- D ddot(theta) + C dot(theta) + G = H u  (where G can be a function of theta)
	NonlinearController.__init(self, 1,1)

	self.dGdTheta_0:set(0, -self.M*self.g*self.l)
	local unused=CT.vec(0)
	self:updateSDRE(unused, unused)

	self.Q=matrixn(2,2)
	self.Q:setAllValue(0)
	self.Q:diag():setValues(q,qd)

	self.R=CT.eye(1);
	self.K=LQR_wrap(self.A,self.B,self.Q,self.R)

	--print(self.A)
	--print(self.k/self.M)
	--print(self.b/self.M)
	--print(self.B)
	--print(1/self.M)
	self.x=matrixn(2,1)
	self.x:setAllValue(0)
	self.xd=matrixn(2,1)
	self.xd:setAllValue(0)
	self.U=CT.mat(1,1,0)

end
	
function SPIPM:getStates()
	local states={}
	states.x=matrixn()
	states.xd=matrixn()
	states.U=matrixn()
	states.x:assign(self.x)
	states.xd:assign(self.xd)
	states.U:assign(self.U)
	if self.cf then
		states.cf=self.cf:copy()
	end
	states.t=self.t
	return states
end

function SPIPM:restoreStates(states)
	self.x:assign(states.x)
	self.xd:assign(states.xd)
	self.U:assign(states.U)
	self.t=states.t
	if states.cf then
		self.cf=states.cf:copy()
	else
		self.cf=nil
	end
end

function SPIPM:setState(p,v)
	self.x:set(0,0,p)
	self.x:set(1,0,v)
end

function SPIPM:setDesiredState(p,v)
	self.xd:set(0,0,p)
	self.xd:set(1,0,v)
end

function SPIPM:singleStep()
	self.ddtheta=self:oneStep(self.x, self.U, self.dt, self.maxForce,self.Q, self.R, self.K, self.xd)
	self.t=self.t+self.dt
end

	-- D ddot(theta) + C dot(theta) + G (theta)= H u
function SPIPM:updateSDRE(theta, dtheta)
	local invM=1/self.M
	self.D:set(0,0,self.I)
	self.C:set(0,0,0)
	local tt=theta(0)
	self.G:set(0, -self.M*self.g*self.l*math.sin(tt))
	self.H:set(0,0,1)
	self:_updateSDRE(theta)
end

-- Mass spring damper system
SDS=LUAclass(NonlinearController)

-- m: mass of the block, B: damping constant, k: spring constant
function SDS:__init(m, B, k, q, qd,dt)
	self.M=m
	self.b=B
	self.k=k

	self.t=0
	self.dt=dt
	-- D ddot(theta) + C dot(theta) + G (theta)= H u
	NonlinearController.__init(self,1,1)
	self.dGdTheta_0:set(0, self.k/self.M)
	local unused=CT.vec(0)
	self:updateSDRE(unused, unused)

	self.Q=matrixn(2,2)
	self.Q:setAllValue(0)
	self.Q:diag():setValues(q,qd)

	self.R=CT.eye(1);
	self.K=LQR_wrap(self.A,self.B,self.Q,self.R)

	--print(self.A)
	--print(self.k/self.M)
	--print(self.b/self.M)
	--print(self.B)
	--print(1/self.M)
	self.x=matrixn(2,1)
	self.x:setAllValue(0)
	self.xd=matrixn(2,1)
	self.xd:setAllValue(0)
	self.U=CT.mat(1,1,0)

	
end
function SDS:getStates()
	local states={}
	states.x=matrixn()
	states.xd=matrixn()
	states.U=matrixn()
	states.x:assign(self.x)
	states.xd:assign(self.xd)
	states.U:assign(self.U)
	if self.cf then
		states.cf=self.cf:copy()
	end
	states.t=self.t
	return states
end

function SDS:restoreStates(states)
	self.x:assign(states.x)
	self.xd:assign(states.xd)
	self.U:assign(states.U)
	self.t=states.t
	if states.cf then
		self.cf=states.cf:copy()
	else
		self.cf=nil
	end
end

function SDS:setState(p,v)
	self.x:set(0,0,p)
	self.x:set(1,0,v)
end
function SDS:setDesiredState(p,v)
	self.xd:set(0,0,p)
	self.xd:set(1,0,v)
end
function SDS:singleStep()
	self:oneStep(self.x, self.U, self.dt, self.maxForce,self.Q, self.R, self.K, self.xd)
	self.t=self.t+self.dt
end
	-- D ddot(theta) + C dot(theta) + G (theta)= H u
function SDS:updateSDRE(theta, dtheta)
	local invM=1/self.M
	self.D:set(0,0,1)
	self.C:set(0,0,self.b*invM)
	self.G:set(0,self.k*invM)
	self.H:set(0,0,1*invM)
	self:_updateSDRE(theta)
end


--class 'IPC' (NonlinearController)
IPC=LUAclass(NonlinearController)

function IPC:__init(M,m, b, I, g,l)
   self.M=M
   self.m=m
   self.b=b
   self.I=I
   self.g=g
   self.l=l

   NonlinearController.__init(self,2,1)

   self.dGdTheta_0:setValues(0, -m*g*l)
   self.H:setValues(1,0)
end
function IPC:setInertia(l,i)
	self.l=l
	self.dGdTheta_0:setValues(0, -self.m*self.g*self.l)
end

function IPC:updateSDRE(x, dx)
   local M=self.M
   local m=self.m
   local b=self.b
   local I=self.I
   local g=self.g
   local l=self.l

   local cos=math.cos
   local sin=math.sin
   local theta=x(1)
   local dtheta=dx(1)

   self.D:setSymmetric(M+m, -m*l*cos(theta), I+m*l*l)

--   print("D", self.D)

   self.C:setValues(b, m*l*dtheta*sin(theta),
      0, 0)

   self.G:setValues(0, -m*g*l*sin(theta))
   self:_updateSDRE(x)
end



--class 'IPCview'-- support both position and velocity control
IPCview=LUAclass()

function IPCview:setQ(mode, q, qd)
	if self.mode~=mode then
		self.Q:setAllValue(0)
		if mode==0 then -- position control
			self.Q:diag():setValues(q, 0, qd,0)
		else -- velocity control
			self.Q:diag():setValues(0, q, qd,0)
		end
		self.mode=mode
		self.K=LQR_wrap(self.IPC.A,self.IPC.B,self.Q,self.R)
	end
end
function IPCview:__init(M,m,b,i,g,l,dt,q, qd,mode)
   self.IPC=IPC(M,m,b,i,g,l)
   self.IPC:updateSDRE(CT.vec(0,0),CT.vec(0,0))

   self.l=l
   self.Q=matrixn(4,4)
   self.R=CT.eye(1);
   qd=qd or q
   self:setQ(mode,q,qd) -- velocity control, if mode == 1 
   
   print("Info:",self.IPC.A, self.IPC.B, self.Q, self.R, self.K)

   -- prepare euler integration
   self.t=0
   self.dt=dt
   self.x=matrixn(4,1)
   self.x:setAllValue(0)
--   self.x:set(1,0, 0.1)
   self.U=CT.mat(1,1,0)

   -- self.y=vectorn(2)
   -- self.Y=matrixn()

   self.xd=matrixn(4,1)
   self.xd:column(0):setAllValue(0)

end

function IPCview:setParam(speed)
	local xd=self.xd
	xd:column(0):setAllValue(0)
	if self.mode==0 then
		xd:set(0,0, speed) --actually position
	else
   		xd:set(2,0, speed)
	end
end

function IPCview:setMaximumForce(force)
   self.maxForce=force
end


function IPCview:oneStep()


	self.ddtheta= self.IPC:oneStep(self.x, self.U, self.dt, self.maxForce,self.Q, self.R, self.K, self.xd)
   self.t=self.t+self.dt
   
   -- self.y:set(0, self.x:get(0,0))
   -- self.y:set(1, self.x:get(1,0))
   -- self.Y:pushBack(self.y)
end
function IPCview:oneStepRaw(U)

	local dim=self.IPC.dim
   self.IPC:updateSDRE(self.x:column(0):range(0,dim), self.x:column(0):range(dim, dim*2))

   self.IPC:oneStepRaw(self.x, U, self.dt, self.maxForce,self.Q, self.R, self.K, self.xd)
   self.t=self.t+self.dt
   
   -- self.y:set(0, self.x:get(0,0))
   -- self.y:set(1, self.x:get(1,0))
   -- self.Y:pushBack(self.y)
end


function IPCview:draw()

   pNode=RE.createEntity("cart_pole", "cube.mesh");
   v1=vector3()
   v2=vector3()
   q=quater()
   
   -- local pos=self.y:get(0)
   -- local theta=self.y:get(1)
   local pos=self.x:get(0,0)
   local theta=self.x:get(1,0)

   local scale=100
   q:setRotation(vector3(0,0,1), theta);
   v1:setValue(pos*scale,0,0);
   v2:rotate(q,vector3(0,self.IPC.l*2*scale,0));
   v2:radd(v1)
   
   thick=4;
   
   pNode:resetToInitialState();
   pNode:scale(0.01*thick, 0.01*(v2-v1):length(), 0.01*thick);
   pNode:rotate(q);
   pNode:translate((v1+v2)/2)
   
   if mv==3 then
      RE.setMaterialName(pNode, "blue")
   else
      RE.setMaterialName(pNode, "green")
   end
end






--class 'DIPCview'
DIPCview=LUAclass()

function DIPCview:setQ(q)

   self.Q:diag():assign({0,q,q,q,0,0})
   self.K=LQR_wrap(self.DIPC.A,self.DIPC.B,self.Q,self.R)

end

function DIPCview:__init(m0,m1,m2,l1,l2,I1,I2, dt,q)
   self.DIPC=DIPC(m0,m1,m2,l1,l2,I1,I2)
   self.DIPC:updateSDRE(CT.vec(0,0,0),CT.vec(0,0,0))

   self.l=l1+l2

   self.Q=matrixn(6,6)
   self.Q:setAllValue(0)
   
   local cdim=self.DIPC.cdim
   self.R=matrixn(cdim, cdim)
   self.R:setAllValue(0)

   if cdim==2 then
      self.R:diag():assign({1,10}) -- 1,1 doesn't work for running. (
   elseif cdim==3 then
      self.R:diag():assign({1,1000,1})
   else
      self.R:diag():setValues(1)
   end
   
--   assert(octave~=nil, "Create a global octave object first (octave=Octave())")

   self:setQ(q)

   self.q=q

   print("Info:",self.DIPC.A, self.DIPC.B, self.Q, self.R, self.K)

   -- prepare euler integration
   self.t=0
   self.dt=dt
   self.x=matrixn(6,1)
   -- DIPC initial solution
   self.x:setAllValue(0)
   -- self.x:set(1,0, 0)
   -- self.x:set(2,0, -0.2)
   self.U=matrixn(self.DIPC.cdim, 1)
   self.U:setAllValue(0)
   -- self.y=vectorn(3)
   -- self.Y=matrixn()

   self.xd=matrixn(6,1)
   self.xd:setAllValue(0)

end

function DIPCview:setParam(speed, hipangle)
   self.xd:set(3,0, speed)
   self.xd:set(2,0, hipangle or 0)
end

function DIPCview:setParamPos(pos, hipangle)
	self.xd:set(0,0, pos)
	self.xd:set(3,0,0)
	self.xd:set(2,0, hipangle or 0)
end
function DIPCview:setMaximumForce(force)
   self.maxForce=force
end


function DIPCview:oneStep()

	if self.useDesiredPos then
		local q=self.q
		-- setQ for position control
		self.Q:diag():assign({100*q,q,q,0,0,0})
		self.K=LQR_wrap(self.DIPC.A,self.DIPC.B,self.Q,self.R)
	end
   self.DIPC:oneStep(self.x, self.U, self.dt, self.maxForce,self.Q, self.R, self.K, self.xd)
   self.t=self.t+self.dt
   
   -- self.y:set(0, self.x:get(0,0))
   -- self.y:set(1, self.x:get(1,0))
   -- self.y:set(2, self.x:get(2,0))
   -- self.Y:pushBack(self.y)
end


function DIPCview:draw()

   local pNode=RE.createEntity("cart_pole", "cube.mesh");
   local pNode2=RE.createEntity("upperBody", "cube.mesh");
   local v1=vector3()
   local v2=vector3()
   local v3=vector3()
   local q=quater()
   local q2=quater()
   -- local pos=self.y:get(0)
   -- local theta=self.y:get(1)
   -- local theta2=self.y:get(2)
   local pos=self.x:get(0,0)
   local theta=self.x:get(1,0)
   local theta2=self.x:get(2,0)


   local axis=vector3(0,0,-1)

   if self.DIPC.useBogdanov04 then 
      theta2=theta2-theta
      RE.output("theta", util.tostring(theta, theta2))
   end

   local scale=100
   q:setRotation(axis, theta);
   q2:setRotation(axis, theta2);
   q2=q2*q
   
   v1:setValue(pos*scale,0,0);
   v2:rotate(q,vector3(0,self.DIPC.l1*2*scale,0));
   v2:radd(v1)

   v3:rotate(q2,vector3(0,self.DIPC.l2*2*scale,0));
   v3:radd(v2)
   

   thick=4;
   
   pNode:resetToInitialState();
   pNode:scale(0.01*thick, 0.01*(v2-v1):length(), 0.01*thick);
   pNode:rotate(q);
   pNode:translate((v1+v2)/2)

   pNode2:resetToInitialState();
   pNode2:scale(0.01*thick, 0.01*(v3-v2):length(), 0.01*thick);
   pNode2:rotate(q2);
   pNode2:translate((v3+v2)/2)
   
   if mv==3 then
      RE.setMaterialName(pNode, "blue")
      RE.setMaterialName(pNode2, "blue")
   else
      RE.setMaterialName(pNode, "green")
      RE.setMaterialName(pNode2, "green")
   end
end

--[[

let x=theta0 : ankle position
let y=theta1 : ankle angle (global)
let z=theta2 : hip angle with respect to pendulum 1 (local)

L=1/2*d1*(x'[t])^2 + 1/2*d4*(y'[t])^2 + 1/2*d6*(y'[t]+z'[t])^2 + d2*Cos[y[t] ]*x'[t]*y'[t] +
  d3*Cos[y[t]+z[t] ]*x'[t]*(y'[t]+z'[t]) + d5*Cos[z[t] ]*y'[t]*(y'[t]+z'[t]) - d2*g*Cos[y[t] ] - 
 d3*g*Cos[y[t]+z[t] ]


EQN1=(-d2)*Sin[y]*y'^2 - d3*Sin[y + z]*(y' + z')^2 + 
   d1*x'' + d2*Cos[y]*y'' + 
   d3*Cos[y + z]*(y'' + z'')=f

EQN2=(-d2)*g*Sin[y] - d3*g*Sin[y + z] - d5*Sin[z]*y'*z' 
   - d5*Sin[z]*z'*(y' + z') + d2*Cos[y]*x'' + 
   d3*Cos[y + z]*x'' + d4*y'' + 
   d5*Cos[z]*y'' + 
   d6*(y'' + z'') + 
   d5*Cos[z]*(y'' + z'')=0

EQN3=(-d3)*g*Sin[y + z] 
    + d5*Sin[z]*y'*y' + 
   d3*Cos[y + z]*x'' + 
   d5*Cos[z]*y'' + 
   d6*(y'' + z'')


Let's try calculating lagrange equation using a simpler formulation as in [Bogdanov04] where
z <-- y+z (global hip angle) to verify the equations in the paper.

L=1/2*d1*(x'[t])^2 + 1/2*d4*(y'[t])^2 + 1/2*d6*z'[t]^2 + d2*Cos[y[t] ]*x'[t]*y'[t] +
  d3*Cos[z[t] ]*x'[t]*z'[t] + d5*Cos[y[t]-z[t] ]*y'[t]*z'[t] - d2*g*Cos[y[t] ] - 
 d3*g*Cos[z[t] ]

-- trivial verification when using Mathmatica.


]]--
