require("module")

function pointTrackerRegisterCommon()
   if pointTracker_registered==true then
      return
   end
   local currDir=util.getCurrentDirectory()
   local aaa,bbb=string.find(currDir, "taesoo_cmu\\OgreFltk")

   local matlabpath=string.sub(currDir,1,bbb).."\\resource\\scripts\\octave"

   octave:eval("rmpath(genpath('"..matlabpath.."'))")
   octave:eval("addpath(genpath('"..matlabpath.."'))")
   octave:eval([[

function [xt, Vt, VVt, LL]=step(t, init_x, init_V, A,C,Q,R,measure)


if t==0
   initial=1;
else
   initial=0;
end

prevx=init_x;
prevV=init_V;

   u=[];
   B=[];
   ndx=[];

  if isempty(u)
  [xt, Vt, LL, VVt]=...
	kalman_update(A,C,Q,R, measure, prevx, prevV, 'initial', initial);
  else
    if isempty(ndx)
    [xt, Vt, LL, VVt]=...
	  kalman_update(A,C,Q,R, measure, prevx, prevV, ... 
			'initial', initial, 'u', u, 'B', B);
    end    
  end
]])

   pointTracker_registered=true
			   end


PointTracker=LUAclass()

function PointTracker:step2(t, init_x, init_V, A,C,Q,R,measure)-- returns [xt, Vt, VVt, LL]

   local initial
   if t==0 then
      initial=true;
   else
      initial=false;
   end
   
   local xt, Vt, VVt, LL =self:kalman_update(A,C,Q,R, measure, init_x, init_V, initial);
   return {xt, Vt, VVt, LL}
end

function PointTracker:kalman_update(A, C, Q, R, y, x, V, initial)-- returns [xnew, Vnew, VVnew, loglik, ]
-- KALMAN_UPDATE Do a one step update of the Kalman filter
-- [xnew, Vnew, loglik] = kalman_update(A, C, Q, R, y, x, V, ...)
-- %
-- % INPUTS:
-- % A - the system matrix
-- % C - the observation matrix 
-- % Q - the system covariance 
-- % R - the observation covariance
-- % y(:)   - the observation at time t
-- % x(:) - E[X | y(:, 1:t-1)] prior mean
-- % V(:,:) - Cov[X | y(:, 1:t-1)] prior covariance
-- %
-- % OPTIONAL INPUTS (string/value pairs [default in brackets])
-- % 'initial' - 1 means x and V are taken as initial conditions (so A and Q are ignored) [0]
-- % 'u'     - u(:) the control signal at time t [ [] ]
-- % 'B'     - the input regression matrix
-- %
-- % OUTPUTS (where X is the hidden state being estimated)
-- %  xnew(:) =   E[ X | y(:, 1:t) ] 
-- %  Vnew(:,:) = Var[ X(t) | y(:, 1:t) ]
-- %  VVnew(:,:) = Cov[ X(t), X(t-1) | y(:, 1:t) ]
-- %  loglik = log P(y(:,t) | y(:,1:t-1)) log-likelihood of innovatio

-- %  xpred(:) = E[X_t+1 | y(:, 1:t)]
-- %  Vpred(:,:) = Cov[X_t+1 | y(:, 1:t)]

   local xpred, Vpred
if initial then
  -- if isempty(u)
   xpred = x;
  -- else
  --  xpred = x + B*u;
  -- end
   Vpred = V;
else
  -- if isempty(u)
     xpred = A*x;
  -- else
  --  xpred = A*x + B*u;
  -- end
   Vpred = A*V*A:Transpose() + Q;
end

e = y - C*xpred; --% error (innovation)

local function length(A)
   if A:rows()>A:cols() then
      return A:rows()
   end
   return A:cols()
end

local n = length(e);
local ss = length(A);

S = C*Vpred*C:Transpose() + R;
Sinv = S:Inverse()
ss = length(V);

-- loglik = gaussian_prob(e, zeros(1,length(e)), S, 1);
K = Vpred*C:Transpose()*Sinv;-- % Kalman gain matrix
-- % If there is no observation vector, set K = zeros(ss).
 xnew = xpred + K*e;
 Vnew = (CT.eye(ss) - K*C)*Vpred;
 VVnew = (CT.eye(ss) - K*C)*A*V;


 return xnew, Vnew, VVnew, loglik
end


function PointTracker:__init(frameRate, posVar, velVar, posVar2, velVar2 )

 --  dbg.startTrace()
   -- test kalman filter
--##useOctave   pointTrackerRegisterCommon()


   dt=1.0/frameRate
   ss = 6 -- state size

   self.measureVel=(velVar2~=nil)

   if self.measureVel==true then
      self.os=6
      self.C = CT.eye(6)
      self.R= CT.diag(CT.vec(posVar2, posVar2, posVar2, velVar2, velVar2,velVar2))
   else
      self.os = 3 -- observation size
      self.C = CT.mat(3,6, 1,0,0,0,0,0,
			    0,1,0,0,0,0,
			    0,0,1,0,0,0)
      self.R = posVar2*CT.eye(self.os)
   end
   self.A = CT.mat(6,6, 1,0,0,dt,0,0,
			 0,1,0,0,dt,0,
			 0,0,1,0,0,dt,
			 0,0,0,1,0,0,
			 0,0,0,0,1,0,
			 0,0,0,0,0,1)


   self.Q = CT.diag(CT.vec(posVar,posVar, posVar,velVar, velVar,velVar))
   self.init_x=CT.mat(1,6, 0,0,0,0,0,0)
   self.init_V=CT.diag(CT.vec(10,10,10,10,10,10))
   
   self.x=matrixn()
   self.V=matrixn()
   self.x:assign(self.init_x)
   self.V:assign(self.init_V)
   self.VV=matrixn()
   
   self.loglik=0
   self.t=0


end

function PointTracker:init(init_x, p)

   self.init_x:assign(init_x:column())
   self.init_V=CT.diag(p)
   self.x:assign(self.init_x)
   self.V:assign(self.init_V)
   self.t=0



end

function PointTracker:step(measure)

--##useOctave   local out=octave:call("step",4, {self.t, self.x, self.V, self.A, self.C,self.Q,self.R,   measure:range(0,self.os):column()})

   local out=self:step2(self.t, self.x, self.V, self.A, self.C,self.Q,self.R,   measure:range(0,self.os):column())


   self.t=self.t+1

   self.x:assign(out[1])
   self.V:assign(out[2])
   self.VV:assign(out[3])
--   self.loglik=self.loglik+out[4]
   
end

function PointTracker:getAllState()
   local tbl={}
   tbl.t=self.t
   tbl.x=self.x
   tbl.V=self.V
   tbl.VV=self.VV
   return tbl
end

function PointTracker:setAllState(tbl)
   self.t=tbl.t
   self.x:assign(tbl.x)
   self.V:assign(tbl.V)
   self.VV:assign(tbl.VV)
end
   

function PointTracker:getState()
   
   return self.x:column(0)
end


--class 'PointTrackerFilter'
PointTrackerFilter=LUAclass()
function PointTrackerFilter:__init(frameRate, kernelSizeInSec)
   self.frameRate=frameRate
   self.kernelSize=math.round(self.frameRate*kernelSizeInSec)
   self.i=0
   self.data=matrixn()
end

function PointTrackerFilter:init(x)
   self.data:pushBack(x)
   self.i=self.i+1
end

function PointTrackerFilter:step(x)

   if self.data:rows()<self.kernelSize then
      self.data:pushBack(x)
      self.i=self.i+1
   else
      if self.i>= self.data:rows() then
	 self.i=0
      end
      self.data:row(self.i):assign(x)
      self.i=self.i+1
   end
end

function PointTrackerFilter:getState()
   local x=vectorn()
   x:setSize(self.data:cols())
   x:setAllValue(0)

   for i=0, self.data:rows()-1 do
      x:radd(self.data:row(i))
   end

   x:rdiv(self.data:rows())
   return x
end
      

-- 2D
--class 'PointTrackerC2'
PointTrackerC2=LUAclass()
function PointTrackerC2:__init(frameRate, posVar, velVar, accVar, posVar2, velVar2, accVar2)

--   dbg.startTrace()
   pointTrackerRegisterCommon()

   dt=1.0/frameRate
   ss = 6 -- state size
   os = 4 -- observation size
   self.A = CT.mat(6,6, {1,0,dt,0,0,0,  
			 0,1,0,dt,0,0,  
			 0,0,1,0,dt,0,
			 0,0,0,1,0,dt,
			 0,0,0,0,1,0,
			 0,0,0,0,0,1})

   self.C = CT.mat(4,6, {1,0,0,0,0,0,
			 0,1,0,0,0,0,
			 0,0,1,0,0,0,
			 0,0,0,1,0,0})

   self.Q = CT.diag(CT.vec({posVar,posVar, velVar, velVar, accVar, accVar}))
   self.R = CT.diag(CT.vec({posVar2,posVar2, velVar2, velVar2}))
   print("self.R", self.R)
   print("test" ,CT.diag(CT.vec({posVar2,posVar2, velVar2, velVar2})))
   print("tt", CT.vec({posVar2,posVar2, velVar2, velVar2}))
   self.init_x=CT.mat(1,6, {0,0,0,0,0,0})
   self.init_V=CT.diag(CT.vec({10,10,10,10,10,10}))
   
   self.x=matrixn()
   self.V=matrixn()
   self.x:assign(self.init_x)
   self.V:assign(self.init_V)
   self.VV=matrixn()
   
   self.loglik=0
   self.t=0


end

function PointTrackerC2:init(init_x, p)

   local init_x_2d=vectorn()
   init_x_2d:assign({init_x(0), init_x(1), init_x(3), init_x(4), init_x(6), init_x(7)})
   local p_2d=vectorn()
   p_2d:assign({p(0), p(1), p(3), p(4), p(6), p(7)})

   self.init_x:assign(init_x_2d:column())
   self.init_V=CT.diag(p_2d)
   self.x:assign(self.init_x)
   self.V:assign(self.init_V)



end

function PointTrackerC2:step(measure)

   local measure2d=vectorn()
   measure2d:assign({measure(0), measure(2),measure(3), measure(5)})


   local out=octave:call("step",4, {self.t, self.x, self.V, self.A, self.C,self.Q,self.R,   measure2d:column()})
   self.t=self.t+1
   self.x:assign(out[1])
   self.V:assign(out[2])
   self.VV:assign(out[3])
   self.loglik=self.loglik+out[4]
   
   print("VV",self.VV)
end

function PointTrackerC2:getState()
   
   local state2d=self.x:column(0)
   local state=vectorn()
   state:assign({state2d(0),0,state2d(1),state2d(2),0, state2d(3)})
   return state
end



--class 'PointTracker2D'
PointTracker2D=LUAclass()
function PointTracker2D:__init(frameRate, posVar, velVar, accVar, posVar2, velVar2, accVar2)

 --  dbg.startTrace()
   -- test kalman filter
   pointTrackerRegisterCommon()

   dt=1.0/frameRate
   ss = 4 -- state size
   os = 2 -- observation size
   self.A = CT.mat(4,4, {1,0,dt,0,  0,1,0,dt,  0,0,1,0,  0,0,0,1})
   self.C = CT.mat(2,4, {1,0,0,0,  0,1,0,0})
   self.Q = CT.diag(CT.vec({posVar,posVar, velVar, velVar}))
   self.R = posVar2*CT.eye(os)
   self.init_x=CT.mat(1,4, {0,0,0,0})
   self.init_V=CT.diag(CT.vec({10,10,10,10}))
   
   self.x=matrixn()
   self.V=matrixn()
   self.x:assign(self.init_x)
   self.V:assign(self.init_V)
   self.VV=matrixn()
   
   self.loglik=0
   self.t=0


end

function PointTracker2D:init(init_x, p)

   
   local init_x_2d=vectorn()
   -- 0 1 2 3 4 5
   init_x_2d:assign({init_x(0), init_x(1), init_x(3), init_x(4)})
   local p_2d=vectorn()
   p_2d:assign({p(0), p(1), p(3), p(4)})

   self.init_x:assign(init_x_2d:column())
   self.init_V=CT.diag(p_2d)
   self.x:assign(self.init_x)
   self.V:assign(self.init_V)



end

function PointTracker2D:step(measure)

   local measure2d=vectorn()
   measure2d:assign({measure(0), measure(2)})


   local out=octave:call("step",4, {self.t, self.x, self.V, self.A, self.C,self.Q,self.R,   measure2d:column()})
   self.t=self.t+1
   self.x:assign(out[1])
   self.V:assign(out[2])
   self.VV:assign(out[3])
   self.loglik=self.loglik+out[4]
   
   print("VV",self.VV)
end

function PointTracker2D:getState()
   
   local state2d=self.x:column(0)

   local state=vectorn()
   state:assign({state2d(0),0,state2d(1),state2d(2),0,state2d(3)})

   return state
end


