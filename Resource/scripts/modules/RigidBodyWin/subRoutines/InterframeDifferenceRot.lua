do 
	-- compatibility code
	-- traj may have different format
	function InterframeDifference:extract2Dconfig(traj, iframe)
		return MotionDOF.rootTransformation(traj:row(iframe)):encode2D()
	end
	function InterframeDifference.convertFromLocalPose(localpose, deltarep)
		deltarep:assign(localpose)
	end
	function InterframeDifference.rotY(v)
		return MotionDOF.rootTransformation(v).rotation:rotationY()
	end
	function InterframeDifference.setRotY(synRoot,lastFrame, qy_new) 
		local v=synRoot:row(lastFrame)
		local root=MotionDOF.rootTransformation(v)
		local q=root.rotation
		local qY=quater()
		local qoffset=quater()
		q:decompose(qY,qoffset)
		q:mult(qy_new, qoffset)
		MotionDOF.setRootTransformation(v, root)
	end
	function InterframeDifference:getNumCol()
		return 7
	end
	function InterframeDifference.adjust(id, dTurningSpeed, dvx, dvz)
		if dTurningSpeed~=0 or dvx~=0 or dvz~=0 then
			for i=0,id:numFrames()-1 do
				id.dq(i).y=id.dq(i).y+dTurningSpeed
				id.dv(i).x=id.dv(i).x+dvx
				id.dv(i).z=id.dv(i).z+dvz
			end
		end
	end
	function InterframeDifference:retarget(synRoot,fixedGlobal, numPredictedFrames,frameRate,dTurningSpeed, dvx,dvz)

		local col=7
		local output=synRoot:range(fixedGlobal,numPredictedFrames,0,col)
		self:adjust(dTurningSpeed, dvx,dvz)
		self:reconstruct(output, frameRate)
	end
end

do
	InterframeDifferenceRot=LUAclass()

	function InterframeDifferenceRot.adjust(id, dTurningSpeed)
		if dTurningSpeed~=0 or dvx~=0 or dvz~=0 then
			for i=0,id:numFrames()-1 do
				id.dq:set(i,id.dq(i)+dTurningSpeed)
			end
		end
	end

	function InterframeDifferenceRot:extract2Dconfig(traj, iframe)
		return vector3(0, traj(iframe, 0),0)
	end

	function InterframeDifferenceRot.rotY(v)
		return quater(v(0), vector3(0,1,0))
	end
	function InterframeDifferenceRot.setRotY(synRoot,lastFrame, q) 
		local v=synRoot:row(lastFrame)
		v:set(0, q:rotationAngleAboutAxis(vector3(0,1,0)))
	end

	function InterframeDifferenceRot:getNumCol()
		return 1
	end

	function InterframeDifferenceRot:__init()
		self.dq=vectorn()
		self.startRotY=quater()
		self.axis=vector3(0,1,0)
	end
	function InterframeDifferenceRot:resize(numFrames)
		self.dq:resize(numFrames)
	end
	function InterframeDifferenceRot:numFrames()
		return self.dq:size()
	end

	function InterframeDifferenceRot.convertFromLocalPose(localpose, deltarep)
		deltarep:set(0,localpose(2))
	end

	function InterframeDifferenceRot:initFromDeltaRep(v3_start_transf, mat_input)
		local id=self
		local numFrames=mat_input:rows()
		id:resize(numFrames)
		id.startRotY:setRotation(vector3(0,1,0), v3_start_transf.y)
		local iddq=id.dq
		local set=iddq.set
		local get=mat_input.__call
		for i=0,numFrames-1 do
			set(iddq,i, get(mat_input,i,0))
		end
	end
	-- matrixn output, frameRate
	function InterframeDifferenceRot:reconstruct(output, frameRate)
		if self:numFrames()~=output:rows() then
			error("InterframeDifference::reconstruct")
		end
		local prevRotY=self.startRotY:rotationAngleAboutAxis(self.axis)
		output:set(0,0, prevRotY)
		local rotY
		local dt=1.0/frameRate;
		local midRotY=quater();
		local dq=self.dq
		local get=self.dq.__call
		local set=output.set
		for i=1,self:numFrames()-1 do
			rotY=prevRotY+get(dq,i)*dt
			set(output,i,0,rotY)
			prevRotY=rotY;
		end
	end

	function InterframeDifferenceRot:retarget(synRoot,fixedGlobal, numPredictedFrames,frameRate,dTurningSpeed, dvx,dvz) 
		local col=1
		local output=synRoot:range(fixedGlobal,numPredictedFrames,0,col)
		self:adjust(dTurningSpeed, dvx,dvz)
		self:reconstruct(output, frameRate)
	end
end

do
	InterframeDifferenceRotPDservo=LUAclass()

	function InterframeDifferenceRotPDservo:extract2Dconfig(traj, iframe)
		return vector3(0, traj(iframe, 0),0)
	end

	function InterframeDifferenceRotPDservo.rotY(v)
		return quater(v(0), vector3(0,1,0))
	end
	function InterframeDifferenceRotPDservo.rotY2(v)
		return quater(v(4), vector3(0,1,0))
	end
	function InterframeDifferenceRotPDservo.setRotY(synRoot,lastFrame, q) 
		local v=synRoot:row(lastFrame)
		local qy=q:rotationAngleAboutAxis(vector3(0,1,0))
		if lastFrame>1 then
			local qy_orig=v(0)
			local dq_orig=InterframeDifferenceRotPDservo.calcDQ(synRoot, lastFrame)
			v:set(0, qy)
			local dq=InterframeDifferenceRotPDservo.calcDQ(synRoot, lastFrame)

--			RE.output2('dq_error_src', dq_orig-dq, InterframeDifferenceRotPDservo.difference(qy, qy_orig))
--			v:set(2, (dq_orig-dq)*0.1+InterframeDifferenceRotPDservo.difference(qy, qy_orig))
			v:set(2, InterframeDifferenceRotPDservo.difference(qy, qy_orig))
		else
			v:set(0, qy)
		end
		v:set(4, qy)
	end
	function InterframeDifferenceRotPDservo.setCOM_height(synRoot,lastFrame, y) 
		local v=synRoot:row(lastFrame)
		v:set(5,y)
	end

	-- dim0 : rotY angle, 
	-- dim1 : dq_i-self.dq(i), the difference between fully-kinematic prediction and pd-controlled prediction
	-- dim2 : the difference in dq between pd-controlled prediction and full simulation
	--        to be used for integral control
	--        Note:
	--        difference in dq is identical to difference in rot_y
	--        because dq_i is defined as rotY_i -rotY_(i-1)  and rotY_(i-1) is fixed.
	--
	-- dim3 : smoothed version of dTurningSpeed
	-- dim4 : rotY angle to be used for simulation target.
	-- dim5 : the difference in COM height between pd-controlled prediction and full simulation
	--        to be used for integral control
	
	function InterframeDifferenceRotPDservo:getNumCol()
		return 6
	end

	function InterframeDifferenceRotPDservo:__init()
		self.dq=vectorn()
		self.ddq=vectorn()
		self.startRotY=quater()
		self.axis=vector3(0,1,0)
	end
	function InterframeDifferenceRotPDservo:resize(numFrames)
		self.dq:resize(numFrames)
		self.ddq:resize(numFrames)
	end
	function InterframeDifferenceRotPDservo:numFrames()
		return self.dq:size()
	end

	function InterframeDifferenceRotPDservo.convertFromLocalPose(localpose, deltarep)
		deltarep:set(0,localpose(2))
	end

	function InterframeDifferenceRotPDservo:initFromDeltaRep(v3_start_transf, mat_input, frameRate)
		local id=self
		local numFrames=mat_input:rows()
		id:resize(numFrames)
		id.startRotY:setRotation(vector3(0,1,0), v3_start_transf.y)
		for i=0,numFrames-1 do
			id.dq:set(i, mat_input(i,0))
		end
		for i=1,numFrames-1 do
			id.ddq:set(i, (id.dq(i)-id.dq(i-1))*frameRate)
		end
		id.ddq:set(0, id.ddq(1))
	end
	function InterframeDifferenceRotPDservo.difference(o1, o2)
		local axis=vector3(0,1,0)
		local q1=quater(o1,axis)
		local q2=quater(o2,axis)
		local q3=quater()
		q3:difference(q1,q2)
		return q3:rotationVector().y
	end

	function InterframeDifferenceRotPDservo.calcDQ(synRoot, fixedGlobal)
		return InterframeDifferenceRotPDservo.difference(synRoot(fixedGlobal-1,0), synRoot(fixedGlobal,0))
	end

	-- matrixn output, frameRate
	function InterframeDifferenceRotPDservo:retarget(synRoot,fixedGlobal, numPredictedFrames, frameRate, dTurningSpeed, dvx,dvz) 

		local col=self:getNumCol()
		assert(col==6)
		local output=synRoot:range(fixedGlobal,numPredictedFrames,0,col)
		local start_dq=self.dq(0)
		if self:numFrames()~=output:rows() then
			error("InterframeDifference::reconstruct")
		end

		local desired_dq_mod=0

		local prevRotY=self.startRotY:rotationAngleAboutAxis(self.axis)
		output:set(0,0, prevRotY)
		local rotY
		local dt=1.0/frameRate;
		local dq=self.dq(0)
		if fixedGlobal>0 then
			dq=self.calcDQ(synRoot, fixedGlobal)/dt
			desired_dq_mod=synRoot(fixedGlobal,3)
		end
		assert(prevRotY==prevRotY)
--		print(output:get(0,2))

		local dq_error=0
		local window=10
		if fixedGlobal>window+1 then
			for j=0,window do
				dq_error=dq_error+synRoot:get(fixedGlobal-j,2)*sop.map(j,0,window, 1,0)
			end
		end
		dq_error=dq_error/window

		if g_debugSignal==nil then
		--	g_debugSignal=matrixn(0,8)
		end
		local turnGain=useCase.turnGain or 10
		local feedbackTorque=useCase.turnFeedbackTorque or 100
		if useCase.turnOffTurnGain then
			turnGain=0
		end
			turnGain=turnGain*0
			feedbackTorque=feedbackTorque*1
		local prevRotY2=prevRotY+dq_error*turnGain
		output:set(0,4, prevRotY2)
		RE.output2('dq_error', dq_error)
		--local out=""
		--out=out..string.format("%g %g %g,",prevRotY, dq, desired_dq_mod)
		for i=1,self:numFrames()-1 do
			-- reconstruct rot
			-- dx = dx+ ddx
			-- x = x+ dx
			local ddq=self.ddq(i) -- feed-forward torque
			local dq_i=dq+ddq*dt
			--weight 5 looks good when "l ipc" is used.
			--weight 100 works when "l ipcf" is used.. so I designed a hybrid..
			desired_dq_mod=desired_dq_mod+(dTurningSpeed-desired_dq_mod)*1*dt
			local tau=((self.dq(i)+desired_dq_mod)-dq_i)*feedbackTorque-- feed-back torque . 
			
			-- dynamic integration
			--ddq=ddq+tau
			--dq=dq+ddq*dt
			--
			-- kinematic integration
			dq=desired_dq_mod
			--print(i, self.dq(i), dq)
			rotY=prevRotY+(self.dq(i)+dq)*dt
			

			if g_debugSignal~=nil and i==10 then
				g_debugSignal:resize(g_debugSignal:rows()+1, 8)
				g_debugSignal:row(g_debugSignal:rows()-1):assign(CT.vec(rotY, 0, tau, dq_error,dq_i,(self.dq(i)+desired_dq_mod),dq_i-self.dq(i),0 ))
				print(fixedGlobal)
			end
			local alpha=0.5
			local dq2=desired_dq_mod*alpha
			rotY2=prevRotY2+(self.dq(i)+dq2)*dt+dq_error*turnGain --+tau*0.000--1
			--rotY2=rotY
			output:set(i,0,rotY)
			output:set(i,1,dq_i-self.dq(i))
			output:set(i,2,0)
			output:set(i,3,desired_dq_mod)
			output:set(i,4,rotY2)
			prevRotY=rotY;
			prevRotY2=rotY2;
			--if i<4 then
				--out=out..string.format("%g %g %g,",prevRotY, dq, desired_dq_mod)
			--elseif i==4 then
				--print(tostring(fixedGlobal)..','..out)
			--end
		end
		if g_debugSignal then
			g_debugSignal:row(g_debugSignal:rows()-1):set(1, output(0,0))
			if g_debugSignal:rows()==3400 then

				local mrdplot=Physics.MRDplot()
				mrdplot:initMRD(g_debugSignal:cols(), g_debugSignal:rows(), 1)
				mrdplot.data:assign(g_debugSignal)
				mrdplot:save("debug_plotkk.mrd")
				os.execute('ogrefltkconsole_d rigidbodywin.lua ../../../../work/test_plot')
				this('exit',0)
			end
		end
	end
end
