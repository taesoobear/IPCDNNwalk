
FDTracker=LUAclass()
function FDTracker:__init(gravity)
	self.useZUP=true
	self.simulator=Physics.DynamicsSimulator_gmbs_penalty()
	if self.useZUP then
		self.loader=MainLib.VRMLloader('box_zup.wrl')
		self.simulator:registerCharacter(self.loader)
		self.simulator:setGVector(self:toZUP0(gravity))
	else
		self.loader=MainLib.VRMLloader('box.wrl')
		self.simulator:registerCharacter(self.loader)
		self.simulator:setGVector(gravity)
	end

	local sim_timestep=1/1500
	local rendering_step=1/30
	self.simulator:init(sim_timestep, Physics.DynamicsSimulator.EULER)
	self.niter=math.floor(rendering_step/sim_timestep+0.5)
	self.dt=sim_timestep
end

function FDTracker:processInput(mot_, extForceEndTime)
	mot_:quatViewCol(3):align()
	self.mot=matrixn()
	local niter=self.niter
	self.mot:resample(mot_, mot_:rows()*niter)
	self.mot:quatViewCol(3):normalize()

	if self.useZUP then
		for i=0, self.mot:rows()-1 do
			self:toZUP(self.mot:row(i))
		end
	end

	local frameRate=30*niter
	self.dmot=MotionDOF.calcDerivative(self.mot, frameRate)
	self.ddmot=self.dmot:derivative(frameRate)

	if true then
		self.controlforce=self.ddmot:copy()
		self.controlforce:setAllValue(0)
		self.tau=matrixn(self.ddmot:rows(), 6)

		self.c=vectorn(self.ddmot:rows())
		self.q=matrixn(self.ddmot:rows(), 6)
		self.dq=matrixn(self.ddmot:rows(), 6)
		self.ddq=matrixn(self.ddmot:rows(), 6)


		-- test rawstate
		for i=0, self.mot:rows()-1 do
			self.simulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE,self.mot:row(i))
			self.simulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY,self.dmot:row(i))
			self.simulator:initSimulation()

			local q=self.q:row(i)
			local dq=self.dq:row(i)
			local ddq=self.ddq:row(i)
			local c=self.simulator:getRawState(0, q, dq, ddq)
			self.c:set(i, c)
		end
		

		if false then
			for i=0, self.mot:rows()-1 do
				self.simulator:inverseDynamics(self.mot:row(i), self.dmot:row(i), self.ddmot:row(i), self.controlforce:row(i))

				-- tested ok.
				--self.simulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_ACCELERATION,self.ddmot:row(i))
				--local out=vectorn(self.ddmot:cols()) 
				--self.simulator:getLinkData(0, Physics.DynamicsSimulator.JOINT_ACCELERATION,out)
				--print(out-self.ddmot:row(i))

				self.simulator:getRootTau(self.tau:row(i))
			end
			self.simulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE,self.mot:row(0))
			self.simulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY,self.dmot:row(0))
			self.simulator:initSimulation()
		else
			self.dq=self.q:derivative(frameRate)
			self.ddq=self.dq:derivative(frameRate)

			for i=0, self.mot:rows()-1 do
				self.simulator:inverseDynamicsRaw(self.c(0), self.q:row(i), self.dq:row(i), self.ddq:row(i), self.tau:row(i));
			end
			local q=self.q:row(0)
			local dq=self.dq:row(0)
			local ddq=self.ddq:row(0):copy()
			self.simulator:setRawState(self.c(0), 0, q, dq, ddq)
			self.simulator:initSimulation()
		end




		--[[
		-- test failed.
		local tau=CT.vec(1,2,3,4,5,6,7)
		self.simulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_TORQUE,tau)
		self.simulator:getLinkData(0, Physics.DynamicsSimulator.JOINT_TORQUE,tau)
		dbg.console()
		]]


	else
		local box_pose=CT.vec(0, 1,0, 1,0,0,0)
		box_pose:setQuater(3, quater(math.rad(70), vector3(1,0,0)))
		self.simulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE,box_pose)
		box_pose:zero()
		self.simulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY,box_pose)
	end


	self.simulator:initSimulation()

	local output=self.mot:copy()
	if extForceEndTime then


		--local st=math.max(extForceEndTime*0.5, 0.2)
		local st=extForceEndTime


		do 
			local dt=self.dt
			local sti=math.round(st/dt)
			local pos=self:toYUP0(self.mot:row(sti):toVector3(0))
			local q=self:toYUP_ori0(self.mot:row(sti):toQuater(3))
			local vel=q*self:toYUP0(self.dmot:row(sti):toVector3(0))
			self.plannedInitial={pos, vel}
			--dbg.draw('Sphere', pos*100, 'plannedinitial', 'green', 15)
		end

		local applicationPoints=vector3N(self.mot:rows())
		local externalForces=vector3N(self.mot:rows())

		applicationPoints:matView():setAllValue(0.0)
		externalForces:matView():setAllValue(0.0)

		local f=self.force or vector3(0,0,0)
		assert(self.useZUP)

		for iframe=0, self.mot:rows()-1 do
			local t=iframe*self.dt
			applicationPoints(iframe):assign(self.mot:row(iframe):toVector3(0)+self:toZUP0(self.force_lpos or vector3(0,0,0.03)))
			--if t(i)<0.2 then
			if t>st-0.2 and t<=st then
				externalForces(iframe):assign(self:toZUP0(f))
				print(iframe)
			end
		end

		--self.force=nil
		self.applicationPoints=applicationPoints
		self.externalForces=externalForces
	end


	for iframe=0, self.mot:rows()-1 do
		if self.controlforce then
			self.simulator:setRootTau(self.tau:row(iframe))
		end
		if extForceEndTime then
			--local st=math.max(extForceEndTime*0.5, 0.2)
			local st=extForceEndTime
			local t=iframe*self.dt
			local f=self:toZUP0(self.force or vector3(0,0,0))
		
			if t>st-0.2 and t<=st then
				self.simulator:addForceToBone(0, self.loader:bone(1), self:toZUP0(self.force_lpos or vector3(0,0,0.03)), f)
			end
		end
		if false then
			self.simulator:drawDebugInformation()
		end
		self.simulator:stepSimulation()
		self.simulator:getLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, output:row(iframe))
	end
	self.force=nil
	local output2=matrixn(math.round(self.mot:rows()/niter), self.mot:cols())
	local ap=self.applicationPoints
	local ef=self.externalForces
	--local ap2=vector3N(output2:rows())
	--local ef2=vector3N(output2:rows())

	for i=0, output2:rows()-1 do
		-- downsample
		output2:row(i):assign(output:row(i*niter))
		--ap2:row(i):assign(ap:row(i*niter))
		--ef2:row(i):assign(ef:row(i*niter))
	end



	if self.useZUP then
		for i=0, self.mot:rows()-1 do
			self:toYUP(self.mot:row(i))
		end
		for i=0, output2:rows()-1 do
			self:toYUP(output2:row(i))
		end
		if ap then
			for i=0, ap:rows()-1 do
				ap:row(i):assign(toYUP0(ap:row(i)))
				ef:row(i):assign(toYUP0(ef:row(i)))
			end
		end
	end

	--self.applicationPoints=ap2
	--self.externalForces=ef2
	self.inputProcessed=true
	return output2
end

function FDTracker:clear()
	self.inputProcessed=false
	self.force=nil
	self.applicationPoints=nil
	self.externalForces=nil
end

function FDTracker:toZUP0(v)
	return vector3(v.z, v.x, v.y)
end
function FDTracker:toZUP_ori0(q)
	return quater(q.w, q.z, q.x, q.y)
end

function FDTracker:toYUP0(v)
	return vector3(v.y, v.z, v.x)
end
function FDTracker:toYUP_ori0(q)
	return quater(q.w, q.y,  q.z, q.x)
end

-- inplace
function FDTracker:toZUP(tf)
	local v=self:toZUP0(tf:toVector3(0))
	local q=self:toZUP_ori0(tf:toQuater(3))

	tf:setVec3(0, v)
	tf:setQuater(3, q)
end

function FDTracker:toYUP(tf)
	local v=self:toYUP0(tf:toVector3(0))
	local q=self:toYUP_ori0(tf:toQuater(3))

	tf:setVec3(0, v)
	tf:setQuater(3, q)
end
