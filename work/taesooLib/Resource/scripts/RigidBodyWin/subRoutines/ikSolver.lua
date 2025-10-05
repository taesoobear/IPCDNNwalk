
require("RigidBodyWin/subRoutines/gmbs_addon")
IKsolver=LUAclass()

IKsolver.modes={fixedCOM=1, fixedCOMxz=2, fixedMomentum=3, fixedRoot=4}
-- effector_list={ {bone, localpos}, ...}
function IKsolver:__init(skel, effector_list, list_1dof_bones)

	self.skel=skel
	self.effector_list=effector_list
	self.list_1dof_bones=list_1dof_bones

	for i,e in ipairs(list_1dof_bones) do
		e.dofIndex=self.skel.dofInfo:DOFindex(e[1]:treeIndex(), 0)
		e[2]=math.rad(e[2])
		e[3]=math.rad(e[3])
	end
	self.numDof=6 -- 3: position only, 6: fix orientation
	local numDof=self.numDof
	self.J=matrixn(#effector_list*numDof, self.skel.dofInfo:numDOF())
	self.Error=vectorn(#effector_list*numDof)
	local idx=0
	local tj=matrixn(6*#effector_list, self.skel.dofInfo:numDOF())

	for i,e in ipairs(effector_list) do
		if e[1]:treeIndex()~=1 then
			e.J=tj -- temp matrix
			e.Error=self.Error:range(idx,idx+numDof)
			idx=idx+numDof
			--e.skel=VRMLloaderView(self.skel, e[1], e[2])
			--MotionLoader.setVoca(self.skel, model.bones)
			--e.simulator=Physics.DynamicsSimulator_gmbs_penalty()
			--e.simulator:registerChacacter(e.skel)
			--e.simulator:init(model.timestep/model.simulationSubTimesteps,  Physics.DynamicsSimulator.EULER)
		end
	end

	self.simulator=Physics.DynamicsSimulator_gmbs_penalty()
	self.simulator:registerCharacter(self.skel)
	self.simulator:init(1/3000, Physics.DynamicsSimulator.EULER)

	self.debug=false
	if self.debug then
		self.skin=RE.createVRMLskin(self.skel, true)
		self.skin:setThickness(0.03)
		self.skin:scale(100,100,100)
		self.skin:setTranslation(100,0,0)
	end

end

function IKsolver:findLimbConfigurations(rotY, pose, getPositionToo)
	-- find desired limb orientations
	local origRootTF=MotionDOF.rootTransformation(pose)
	local crotY=quater()
	local offset=quater()
	origRootTF.rotation:decompose(crotY, offset)
	origRootTF.rotation:mult(rotY, offset)
	MotionDOF.setRootTransformation(pose, origRootTF)-- only vertically rotated
	self.skel:setPoseDOF(pose)

	for i,e in ipairs(self.effector_list) do
		e.oriTarget=self.skel:fkSolver():globalFrame(e[1]).rotation:copy()
	end
	if getPositionToo then
		for i,e in ipairs(self.effector_list) do
			e.positionTarget=self.skel:fkSolver():globalFrame(e[1]).translation:copy()
		end
	end
end
-- rotY is used for defining foot global orientations.
function IKsolver:solve(rotY, roottf, pose, ...)

	local modes=self.modes
	local mode=modes.fixedRoot
	if useCase._tempIKvar then --flight phase IK
		if useCase._tempDState and useCase._tempDState:size()~=0 then
			mode=modes.fixedMomentum
		--else dbg.console()
		end
		--mode=modes.fixedCOMxz
	end
	self:_solve(mode,rotY, roottf, pose, ...)
end

function IKsolver:_solve(mode, rotY, roottf, pose, ...)
	local modes=self.modes
	local positions={...}
	-- dot com=J* dot_theta
	-- Jacobian transpose method
	--
	local footJ=matrixn()
	local footJ2=matrixn()
	local temp=matrixn()

	local timer3=util.PerfTimer2()--(1,"timer3")
	local timer4=util.PerfTimer2()--(1,"timer3")
	timer3:start()
	self:findLimbConfigurations(rotY, pose)

	--do
		---- find desired limb orientations
		--local origRootTF=MotionDOF.rootTransformation(pose)
		--local crotY=quater()
		--local offset=quater()
		--origRootTF.rotation:decompose(crotY, offset)
		--origRootTF.rotation:mult(rotY, offset)
		--MotionDOF.setRootTransformation(pose, origRootTF)-- actually modifiedRootTF
		--self.skel:setPoseDOF(pose)
--
		--for i,e in ipairs(self.effector_list) do
			--e.oriTarget=self.skel:fkSolver():globalFrame(e[1]).rotation:copy()
		--end
	--end

	MotionDOF.setRootTransformation(pose, roottf)
	
	local initialState=pose:copy()
	self.simulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, initialState)
	if mode==modes.fixedMomentum then
		self.simulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, useCase._tempDState)
	end
	self.simulator:initSimulation()

	local speedBoost=useCase.IKspeedBoost or 3

	local timestep=0.2*speedBoost  -- means 100 iterations are required
	local niter=math.floor((useCase.IKnumIter or 20)/speedBoost)
	local numDof=self.numDof
	local function inv(a)
		--return math.pseudoInverse(a)-- timestep 0.01
		--return a:Transpose() -- timestep 0.1
		return math.srInverse(a, 0.01)
	end
	local list_1dof_bones=self.list_1dof_bones
	local gJacobian
	local gJacobian2
	if mode~=fixedRoot then
		gJacobian= matrixn()
		gJacobian2= matrixn()
	end

	timer3:stopMsg('findLimbConfiguration')

	timer3:start()
	timer4:reset()
	local timer5=util.PerfTimer2()--(1,"timer3")
	local timer6=util.PerfTimer2()--(1,"timer3")
	local timer7=util.PerfTimer2()--(1,"timer3")
	local timer8=util.PerfTimer2()
	local timer9=util.PerfTimer2()
	for iter=0,niter do
		timer5:start()
		self.simulator:updateJacobians(0)
		timer5:pause()


		local nullProj
		if mode==modes.fixedCOM then
			timer6:start()
			self.simulator:calcCOMjacobian(0, gJacobian)
			timer6:pause()
			timer4:start()
			nullProj=math.nullspaceProjector(gJacobian)
			timer4:pause()
		elseif mode==modes.fixedCOMxz then
			timer6:start()
			self.simulator:calcCOMjacobian(0, gJacobian)
			gJacobian2:setSize(gJacobian:rows()-1, gJacobian:cols())
			gJacobian2:row(0):assign(gJacobian:row(0))
			gJacobian2:row(1):assign(gJacobian:row(2))
			timer6:pause()
			timer4:start()
			nullProj=math.nullspaceProjector(gJacobian2)
			timer4:pause()
		elseif mode==modes.fixedMomentum then
			timer6:start()
			self.simulator:calcMomentumCOMjacobian(0, gJacobian,timer8, timer9)
			timer6:pause()
			timer4:start()
			nullProj=math.nullspaceProjector(gJacobian:sub(0,3))
			timer4:pause()
		end

		local c=1
		local err=self.Error
		do
			-- reserve matrix size
			local n=#self.effector_list
			self.J:resize(n*numDof, self.J:cols())
			err:resize(n*numDof)
		end
		for i,e in ipairs(self.effector_list) do
			local importance=positions[i][2]
			if importance>0.01 then
				if numDof==3 then
					timer7:start()
					self.simulator:calcJacobian(0, e[1]:treeIndex(), e.J, e[2], mode~=modes.fixedRoot)
					timer7:pause()
					self.J:sub((c-1)*3,c*3):assign(e.J:sub(3,6))
					local cpos=self.simulator:getWorldState(0):globalFrame(e[1]):toGlobalPos(e[2])
					err:setVec3((c-1)*3, timestep*(positions[i][1]-cpos)*importance)
				else
					timer7:start()
					self.simulator:calcJacobian(0, e[1]:treeIndex(), e.J, e[2], mode~=modes.fixedRoot)
					timer7:pause()
					self.J:sub((c-1)*6,c*6):assign(e.J)
					local ctf=self.simulator:getWorldState(0):globalFrame(e[1])
					local cpos=ctf:toGlobalPos(e[2])
					err:setVec3((c-1)*6, timestep*ctf.rotation:rotationVecTo(e.oriTarget))
					err:setVec3((c-1)*6+3, timestep*(positions[i][1]-cpos)*importance)
				end
				c=c+1
			end
		end

		-- discard unused blocks
		self.J:resize((c-1)*numDof, self.J:cols())
		err:resize((c-1)*numDof)
		timer4:start()
		if mode==modes.fixedRoot then
			self.simulator:integrateDState(pose, (inv(self.J)*self.Error:column()):column(0))
		else
			self.simulator:integrateDState(pose, (nullProj*inv(self.J)*self.Error:column()):column(0))
		end
		timer4:pause()

		for i,e in ipairs(list_1dof_bones) do

			local minangle=e[2]
			local maxangle=e[2]*2

			-- knee damping
			
			local input=pose(e.dofIndex)
			local output
			if input>maxangle then
				output=math.min(input, e[3])
			else
				output=maxangle-(maxangle-input)*(math.smoothTransition(sop.clampMap(input, minangle,maxangle,0,1)))
			end
			pose:set(e.dofIndex, output)
		end
		self.simulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, pose)
		self.simulator:initSimulation() -- very important
		if self.debug then
			self.skin:setPoseDOF(pose)
			renderOneFrame()
		end
	end
	timer3:stopMsg('solveIK')-- 82ms -- min 64ms for walking
	--timer4:stopMsg('calcInverse')--9ms
	--timer5:stopMsg('updateJacobian') --3.96ms
--timer6:stopMsg('calcCOMJacobian')--72ms
	--timer7:stopMsg('calcJointJacobian')
	--timer8:stopMsg('calcCOM')
	--timer9:stopMsg('calcCOM_convert')
end
