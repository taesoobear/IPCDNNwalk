
require("subRoutines/Optimizer")
TrajOptObs=LUAclass(Optimizer)

function TrajOptObs:__init(pendulum, T, rotY, pos0, zmp, com_vel, zmp_vel, finalrotY, desired_speed ,array_obstacle,  strength)
	assert(strength)
	self.strength=strength 
	self.pendulum=pendulum
	self.numSeg=1
	self.finalrotY=finalrotY
	self.desired_speed=desired_speed
	self.T=T
	self.boxes=array_obstacle
	local opt_dimension={}


	-- optimize duration and desired velocity x, z
	--
	-- 0           1            2
	--      dur0         dur1
	-- hat_v0      hat_v1       hat_v2          (desired_velocity)


	local numSeg=1
	-- heuristic initialization
	for i=1, numSeg do
		table.insert(opt_dimension, {title="Vx"..i, curval=finalrotY, max_step=0.1, grad_step=0.005})
		table.insert(opt_dimension, {title="Vz"..i, curval=desired_speed, max_step=0.1, grad_step=0.005})
	end
	self.pendState={rotY, pos0, zmp, com_vel, zmp_vel}

	_G.opt_dimension=opt_dimension
	local stepSize=0.5
	local method=Optimizer.methods.CMAes
	--local method = Optimizer.methods.ConjugateGradient method.tol=1e-5
	--local method = Optimizer.methods.NRconjugateGradient method.tol=1e-5

	Optimizer.__init(self, stepSize, opt_dimension, method)
	Optimizer.outFile=nil


	self:optimize(true)


	local thickness=10 -- 10 cm
	--dbg.namedDraw('Traj', goal:sub(0,0,0,3)*100, 'goal', 'blueCircle', thickness, 'QuadListY' )
	--dbg.namedDraw('Traj', goal:sub(0,0,6,9)*100, 'goal2', 'greenCircle', thickness, 'QuadListV' )
	self.opt_dimension=opt_dimension

	if #opt_dimension>=1 then
		_G.opt_dimension=opt_dimension
		local stepSize=0.5
		local method=Optimizer.methods.CMAes_nolog
		method.stddev=0.5
		method.maxIteration=100

		discardErrorSamples=true
		--local method=Optimizer.methods.ConjugateGradient
		--local method=Optimizer.methods.NRconjugateGradient
		Optimizer.__init(self, stepSize, opt_dimension, method)
		Optimizer.outFile=nil
		self:optimize(true)
	end
end

function TrajOptObs:checkFeasible(x)
	--if x(1)<0.01 then
	--	return false
	--end
	if math.abs(x(0))>3 then
		return false
	end
	if x(1)<0.001 then
		return false
	end
	--local minLen=0.4
	--if x(0)<minLen then return false end
	--if x(3)<minLen then return false end
	--if x(6)<minLen then return false end
	return true
end
function TrajOptObs:makeFeasible(tt)
	if tt(1)<0.001 then
		tt:set(1,0.001)
	end
	--if tt(1)<0.01 then
	--	tt:set(1,0.01)
	--end
	return tt
end
function TrajOptObs:getResult()
	local opt_dimension=self.opt_dimension
	local v=vector3()
	v.x= opt_dimension[1].curval
	v.z= opt_dimension[2].curval
	return v
end
function toVector2(v3)
	local vv=vector2()
	vv:assignXZ(v3)
	return vv
end
function TrajOptObs:objectiveFunction(x, goal)
	local o=opt_dimension
	local numSeg=self.numSeg

	local m_pendulum=self.pendulum
	local rotY, startPos, startZMP, v0, zmp_vel=unpack(self.pendState)
	local finalrotY=quater(x(0), vector3(0,1,0))
	local vv=finalrotY*vector3(0,0,x(1))
	local w0=0
	local T=self.T
	local strength=self.strength
	local goal, goal2, cf=arc.getPendTraj_direction(m_pendulum, rotY, startPos, startZMP, v0,zmp_vel, vv, w0, T, strength)
	if not goal then
		return 10000
	end

	local boxes=self.boxes
	local count=0
	do
		local ZMP=goal2
		for i=0, ZMP:rows() -1 do
			local pos=ZMP:row(i):toVector3(0)
			for j,b in ipairs(boxes) do
				local vl=toVector2(pos)
				-- 400 foot
				local margin=0.02
				if b:contains(vl, margin) then
					--local dist=b:negativeDistance(vl)
					local center=b.min*0.5+b.max*0.5
					local max_dist=(b.max-b.min):toVector3():length()*0.5+margin*2
					local dist=max_dist-center:toVector3():distance(vl:toVector3())
					assert(dist>0)
					count=count+10*dist*dist
				end
			end
		end
	end

	return count+math.pow(x(0)-self.finalrotY,2)+ math.pow(x(1)-self.desired_speed,2)

end

return TrajOptObs
