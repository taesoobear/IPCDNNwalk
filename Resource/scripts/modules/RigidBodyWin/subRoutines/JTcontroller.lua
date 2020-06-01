require("RigidBodyWin/subRoutines/gmbs_addon")
--class 'JacobianCache'
JacobianCache=LUAclass()

function JacobianCache:__init(vrml_loader, simulator)
	self.skel=vrml_loader
	self.skel_lfoot=MainLib.VRMLloaderView(vrml_loader, vrml_loader:getBoneByVoca(MotionLoader.LEFTANKLE), vector3(0,0,0))
	self.skel_rfoot=MainLib.VRMLloaderView(vrml_loader, vrml_loader:getBoneByVoca(MotionLoader.RIGHTANKLE), vector3(0,0,0))
	MotionLoader.setVoca(self.skel_lfoot, model.bones)
	MotionLoader.setVoca(self.skel_rfoot, model.bones)
	
	self.lfootIndex=self.skel:getBoneByVoca(MotionLoader.LEFTANKLE):treeIndex()
	self.rfootIndex=self.skel:getBoneByVoca(MotionLoader.RIGHTANKLE):treeIndex()
	self.skel_lfoot_rfootIndex=self.skel_lfoot:getBoneByVoca(MotionLoader.RIGHTANKLE):treeIndex()
	self.skel_rfoot_lfootIndex=self.skel_rfoot:getBoneByVoca(MotionLoader.LEFTANKLE):treeIndex()
	MotionLoader.setVoca(self.skel_lfoot, model.bones)
	MotionLoader.setVoca(self.skel_rfoot, model.bones)

	self.simulator=simulator
	self.simulator_lfoot=Physics.DynamicsSimulator_gmbs_penalty()
	self.simulator_lfoot:registerCharacter(self.skel_lfoot)
	self.simulator_rfoot=Physics.DynamicsSimulator_gmbs_penalty()
	self.simulator_rfoot:registerCharacter(self.skel_rfoot)

	model.simulationSubTimesteps=model.simulationSubTimesteps or 1
	self.simulator_lfoot:init(model.timestep/model.simulationSubTimesteps, Physics.DynamicsSimulator.EULER)
	self.simulator_rfoot:init(model.timestep/model.simulationSubTimesteps, Physics.DynamicsSimulator.EULER)

	self.simulator_lfoot:setGVector(vector3(0,9.8,0))
	self.simulator_rfoot:setGVector(vector3(0,9.8,0))
	
	self.simulator_lfoot:initSimulation()
	self.simulator_rfoot:initSimulation()
	self:clear()
	self.state=vectorn()
	self.dstate=vectorn()
	self.statel=vectorn()
	self.dstatel=vectorn()
	self.stater=vectorn()
	self.dstater=vectorn()
	self:setLocalFootPos()
end
function JacobianCache:setLocalFootPos()
	self.lfootpos=useCase.lfootpos
	self.rfootpos=useCase.rfootpos
end

function JacobianCache:clear()
	self.stateValid={OJ=false, O=false,R=false, L=false}
	self.jacobians={}
end

function JacobianCache:updateOState()
	if not self.stateValid.O then
		self.simulator:getLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, self.state)
		self.simulator:getLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, self.dstate)
		self.stateValid.O=true
	end
end
function JacobianCache:setOState(state, dstate)
	self:clear()
	if not self.stateValid.O then
		self.state:assign(state)
		self.dstate:assign(dstate)
		self.stateValid.O=true
	end
end

function JacobianCache:updateRState()
	if not self.stateValid.R then
		self:updateOState()
		self.simulator_rfoot:convertStateFromSource(self.simulator, self.skel_rfoot,self.state, self.dstate, self.stater, self.dstater)
		self.simulator_rfoot:setLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE,self.stater)
		self.simulator_rfoot:setLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY,self.dstater)
		self.simulator_rfoot:initSimulation()
		self.simulator_rfoot:updateJacobians(0)
		self.stateValid.R=true
	end
end
function JacobianCache:updateLState()
	if not self.stateValid.L then
		self:updateOState()
		self.simulator_lfoot:convertStateFromSource(self.simulator, self.skel_lfoot,self.state, self.dstate, self.statel, self.dstatel)
		self.simulator_lfoot:setLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE,self.statel)
		self.simulator_lfoot:setLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY,self.dstatel)
		self.simulator_lfoot:initSimulation()
		self.simulator_lfoot:updateJacobians(0)
		self.stateValid.L=true
	end
end
function JacobianCache:calcJ_momentumCOM()
	if self.jacobians.J_momentumCOM then
		return self.jacobians.J_momentumCOM
	end
	if not self.stateValid.OJ then
		self.simulator:updateJacobians(0)
		self.stateValid.OJ=true	
	end
	local comjacobian=matrixn()
	self.simulator:calcMomentumCOMjacobian(0, comjacobian)

	self.jacobians.J_momentumCOM=comjacobian
	return comjacobian
end
function JacobianCache:calcJ_COM()
	if self.jacobians.J_COM then
		return self.jacobians.J_COM
	end
	if not self.stateValid.OJ then
		self.simulator:updateJacobians(0)
		self.stateValid.OJ=true	
	end
	local comjacobian=matrixn()
	self.simulator:calcCOMjacobian(0, comjacobian)

	self.jacobians.J_COM=comjacobian
	return comjacobian
end
function JacobianCache:calcJ_L()
	if self.jacobians.J_L then
		return self.jacobians.J_L
	end
	if not self.stateValid.OJ then
		self.simulator:updateJacobians(0)
		self.stateValid.OJ=true	
	end
	local jacobian=matrixn()
	self.simulator:calcJacobian(0, self.lfootIndex, jacobian,self.lfootpos, true)

	self.jacobians.J_L=jacobian
	return jacobian
end
function JacobianCache:calcJ_R()
	if self.jacobians.J_R then
		return self.jacobians.J_R
	end
	if not self.stateValid.OJ then
		self.simulator:updateJacobians(0)
		self.stateValid.OJ=true	
	end
	local jacobian=matrixn()
	self.simulator:calcJacobian(0, self.rfootIndex, jacobian, self.rfootpos, true)

	self.jacobians.J_R=jacobian
	return jacobian
end

function JacobianCache:calcJR_COM()
	if self.jacobians.JR_COM then
		return self.jacobians.JR_COM
	end
	self:updateRState()
	local rjacobian=matrixn()
	self.simulator_rfoot:calcCOMjacobian(0, rjacobian)
	self.jacobians.JR_COM=rjacobian
	return rjacobian
end
function JacobianCache:calcJL_COM()
	if self.jacobians.JL_COM then
		return self.jacobians.JL_COM
	end
	self:updateLState()
	local ljacobian=matrixn()
	self.simulator_lfoot:calcCOMjacobian(0, ljacobian)
	self.jacobians.JL_COM=ljacobian
	return ljacobian
end
function JacobianCache:calcJL_Momentum()
	if self.jacobians.JL_Momentum then
		return self.jacobians.JL_Momentum
	end
	self:updateLState()
	local ljacobian=matrixn()
	self.simulator_lfoot:calcMomentumCOMjacobian(0, ljacobian)
	--self.simulator_lfoot:calcMomentumGlobalJacobian(0, ljacobian)
	self.jacobians.JL_Momentum=ljacobian
	return ljacobian
end
function JacobianCache:calcJR_Momentum()
	if self.jacobians.JR_Momentum then
		return self.jacobians.JR_Momentum
	end
	self:updateRState()
	local rjacobian=matrixn()
	self.simulator_rfoot:calcMomentumCOMjacobian(0, rjacobian)
	--self.simulator_rfoot:calcMomentumGlobalJacobian(0, rjacobian)
	self.jacobians.JR_Momentum=rjacobian
	return rjacobian
end
function JacobianCache:calcJR_L()
	if self.jacobians.JR_L then
		return self.jacobians.JR_L
	end
	self:updateRState()
	local rconJacobian=matrixn()
	self.simulator_rfoot:calcJacobian(0, self.skel_rfoot_lfootIndex, rconJacobian, self.lfootpos)
	self.jacobians.JR_L=rconJacobian
	return rconJacobian
end
function JacobianCache:calcJL_R()
	if self.jacobians.JL_R then
		return self.jacobians.JL_R
	end
	self:updateLState()
	local lconJacobian=matrixn()
	self.simulator_lfoot:calcJacobian(0, self.skel_lfoot_rfootIndex, lconJacobian, self.rfootpos)
	self.jacobians.JL_R=lconJacobian
	return lconJacobian
end

-- Jacobian Transpose controller
--class 'JTcontroller'
JTcontroller=LUAclass()

function JTcontroller:__init(vrml_loader, jcache)
	assert(jcache)
	self.jcache=jcache
	self.controlforce=vectorn()
end

-- generate torque 
function JTcontroller:generateTorque(simulator, virtual_force, swingFoot)
   
	local function v2mat(v)
		return CT.mat(3,1,v.x, v.y, v.z)
	end
	local vf=v2mat(virtual_force)

	local torque_g=self.controlforce
	if swingFoot~="L" then
	
		local ljacobian=self.jcache:calcJL_COM()

		if swingFoot=="N" then
			local lconJacobian=self.jcache:calcJL_R()
			local nullProjL=math.nullspaceProjector(lconJacobian)
			self.COMjt=nullProjL*ljacobian:Transpose()
--			self.COMjt=nullProjL*math.pseudoInverse(ljacobian)
		else
			assert(swingFoot=="R")
			self.COMjt=ljacobian:Transpose()
		end

		local torque1=self.COMjt*vf

		torque1:column(0):range(0,7):setAllValue(0)

		self.jcache.skel_lfoot:convertDOFexceptRoot(torque1:column(0), torque_g)
	else
		local rjacobian=self.jcache:calcJR_COM()

		self.COMjt=rjacobian:Transpose()
		local torque2=self.COMjt*vf

		torque2:column(0):range(0,7):setAllValue(0)

		self.jcache.skel_rfoot:convertDOFexceptRoot(torque2:column(0), torque_g)
	end
	
	if swingFoot=="N" then
		self.controlforce:rmult(2)
	end

	self.controlforce:range(0,7):setAllValue(0)
end

--class 'JTcontrollerFoot'
JTcontrollerFoot=LUAclass()

function JTcontrollerFoot:__init(skel, jcache)
	assert(jcache)
	self.jcache=jcache
	self.controlforce=vectorn(skel.dofInfo:numDOF())
	self.leftfoot=skel:getBoneByVoca(MotionLoader.LEFTANKLE)
	self.rightfoot=skel:getBoneByVoca(MotionLoader.RIGHTANKLE)
	self.lfootIndex=self.leftfoot:treeIndex()
	self.rfootIndex=self.rightfoot:treeIndex()
	self.lfootpos=jcache.lfootpos:copy()
	self.rfootpos=jcache.rfootpos:copy()
end

function JTcontrollerFoot:calcFootPos(fkSolver, foot)
	if foot=='L' then
		return fkSolver:globalFrame(self.leftfoot):toGlobalPos(self.lfootpos)
		---fkSolver:globalFrame(self.rightfoot):toGlobalPos(self.rfootpos)
	elseif foot=='R' then
		return fkSolver:globalFrame(self.rightfoot):toGlobalPos(self.rfootpos)
		---fkSolver:globalFrame(self.leftfoot):toGlobalPos(self.lfootpos)
	else
		return self:calcFootPos(fkSolver, 'L')*0.5+self:calcFootPos(fkSolver, 'R')*0.5
	end
end

function JTcontrollerFoot:calcFootVel(simulator, foot)
	if foot=='L' then
		return simulator:getWorldVelocity(0, self.leftfoot, self.lfootpos)
	elseif foot=='R' then
		return simulator:getWorldVelocity(0, self.rightfoot, self.rfootpos)
	else
		return self:calcFootVel(simulator, 'L')*0.5+self:calcFootVel(simulator, 'R')*0.5
	end
end

function JTcontrollerFoot:generateTorque(simulator, foot, desiredPos, desiredVel, flightTime, vfcorrection)
	if foot=='N' then self.controlforce:setAllValue(0) return end
	local worldState=simulator:getWorldState(0)
	local currentPos=self:calcFootPos(worldState, foot)
	local currentVel=self:calcFootVel(simulator, foot)
	dbg.namedDraw('Sphere', currentPos*100, 'CurrfootPos')

	local k_p=model.k_p_JTfoot or 1000
	local k_d=model.k_d_JTfoot or 10
	if foot=='B' then
		currentPos=currentPos-(simulator:calculateCOM(0))
		currentVel=currentVel-(simulator:calculateCOMvel(0))
	end
	local virtualforce= k_p*(desiredPos-currentPos)+k_d*(desiredVel-currentVel)
	local sp=vector3(0,0.9,0)*100+currentPos*100
	if foot=='B' then
	--	virtualforce.y=0
		virtualforce:scale(flightTime)
		local vfn=math.clampVec3(virtualforce, 9.8*160)
		virtualforce:assign(vfn)
	else
		if vfcorrection then
			RE.output2('vfcorrection', vfcorrection)
			virtualforce:radd(vfcorrection)
		end
	end
	RE.output2("swingFoot virtualForce",virtualforce)
	dbg.namedDraw('Line', sp, sp+virtualforce*0.1, "swingfoot vf", 'solidblue')
	local function v2mat(v)
		return CT.mat(1,3,v.x, v.y, v.z)
	end
	local function v2matr(v)
		return CT.mat(3,1,v.x, v.y, v.z)
	end
	if foot=='B' then
		if false then
			local gain=40
		--k_p=50000
		--k_d=3000
			-- use COM jacobian. heuristic that has very little mathmatical basis
			local vf=v2mat(virtualforce*gain)
			local comjacobian=self.jcache:calcJ_COM()
			local torque=vf*comjacobian
			self.controlforce:assign(torque:row(0))
			self.controlforce:range(0,7):setAllValue(0)
		else
			local footJ= self.jcache:calcJ_L()+self.jcache:calcJ_R()
			footJ:rdiv(2)
			local momentumJacobian=self.jcache:calcJ_momentumCOM()
			local footConJ=matrixn()
			self.jcache.skel_lfoot:convertJacobianExceptRoot(self.jcache:calcJL_R(), footConJ)
			local nullProj=math.nullspaceProjector(momentumJacobian)
			local nullProj2=math.nullspaceProjector(footConJ)
			local function inv(a)
				return math.pseudoInverse(a)
				--return a:Transpose()
			end
			local torque=nullProj*nullProj2*inv(footJ:sub(3,6,0,0))*v2matr(virtualforce)
			self.controlforce:assign(torque:column(0))
			self.controlforce:range(0,7):setAllValue(0)
		end
	elseif foot=='L' then
		local ljacobian=self.jcache:calcJR_L():sub(3,6)
		local lmjacobian=self.jcache:calcJR_Momentum():sub(0,3)
		local nullProj=math.nullspaceProjector(lmjacobian)

	--	local vf=v2mat(virtualforce)
	--	local torque=(vf*ljacobian*nullProj):row(0)
		local vf=v2matr(virtualforce)
		local torque=(nullProj*math.srInverse(ljacobian,0.001)*vf):column(0)
		--local torque=vf*ljacobian
		torque:range(0,7):setAllValue(0)
		self.jcache.skel_rfoot:convertDOFexceptRoot(torque, self.controlforce)
		self.controlforce:range(0,7):setAllValue(0)
	else
		assert(foot=='R')
		local rjacobian=self.jcache:calcJL_R():sub(3,6)
		local rmjacobian=self.jcache:calcJL_Momentum():sub(0,3)
		local nullProj=math.nullspaceProjector(rmjacobian)

	--	local vf=v2mat(virtualforce)
	--	local torque=(vf*rjacobian*nullProj):row(0)
		local vf=v2matr(virtualforce)
		local torque=(nullProj*math.srInverse(rjacobian,0.001)*vf):column(0)
		--local torque=vf*rjacobian
		torque:range(0,7):setAllValue(0)
		self.jcache.skel_lfoot:convertDOFexceptRoot(torque, self.controlforce)
		self.controlforce:range(0,7):setAllValue(0)
	end
end
