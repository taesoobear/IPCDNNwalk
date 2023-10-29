require("gym_cdm2/deepmimiccdm-v1")

terrain_sample_loc={ 
	vector3(0,0,0.5),
	vector3(0.5,0,0.5),
	vector3(-0.5,0,0.5),
	vector3(1,0,0.5),
}

num_terrain_samples=#terrain_sample_loc


function get_dim()
	return CT.vec(11+6*numLegs+2+2+num_terrain_samples,2*numLegs+6)
end
function RagdollSim:getOBSdim()
	return 11+6*numLegs+2+2+num_terrain_samples
end



function RagdollSim:_get_obs()
	local simulator=self:getCDMsimulator()
	local position=simulator:getPoseDOF(0) -- [ x, y, z, qw, qx, qy, qz]
	local rotY=model:getRefRotY(position:toQuater(3))
	local velocity=simulator:getDQ(0) -- [ global w, global v ]
	local refCoord=self:getRefCoord(rotY, position)
	--
	-- terrain relative
	position:set(1, position(1)-refCoord.translation.y)
	local offsetQ=rotY:inverse()*position:toQuater(3)
	offsetQ:align(quater(1,0,0,0))
	position:setQuater(3, offsetQ)
	if exclude_current_positions_from_observation then
		local y=position(1)
		position=position:slice(2,0):copy()
		position:set(0, y)
	end
	-- Q=rotY*offset
	--> offset=rotY:inverse()*Q
	velocity:setVec3(0, rotY:inverse()*velocity:toVector3(0))
	velocity:setVec3(3, rotY:inverse()*velocity:toVector3(3))

	--local step_state=position..velocity..com_velocity..external_contact_forces
	
	local step_state=vectorn(position:size()+velocity:size()+6*numLegs+2+2+num_terrain_samples)
	step_state:slice(0, position:size()):assign(position)
	local c=position:size()+velocity:size()
	step_state:slice(position:size(), c):assign(velocity)

	for i=1, numLegs do
		local li=self.legInfo[i]
		--step_state:setVec3(c+(i-1)*4, refCoord:toLocalPos(li.contactPos))
		local cpos=li.contactFilter:copy()
		self:projectToGround(cpos)
		step_state:setVec3(c+(i-1)*6, refCoord:toLocalPos(cpos))

		local v3=li.dotContactFilter 
		local vv=refCoord:toLocalDir(v3)
		step_state:set(c+(i-1)*6+3,vv.x )
		step_state:set(c+(i-1)*6+4,vv.z )
		--step_state:set(c+(i-1)*4+3, li.footOffset.x)
		if li.isSwing then
			step_state:set(c+(i-1)*6+5, 0)
		else
			-- rotY*delta=footRotY
			local deltaq=(rotY:inverse()*li.lqrFilterQorigin)
			deltaq:align(quater(1,0,0,0))
			local delta=deltaq:rotationAngleAboutAxis(vector3(0,1,0))
			step_state:set(c+(i-1)*6+5, delta)
		end
	end
	c=c+6*numLegs
	if true then
		local phase=model:getPhase()
		step_state:set(c, sop.mapCos(phase, 0,0.25, 1,0)) -- phase
		step_state:set(c+1, sop.mapSin(phase, 0,0.25, 0,1)) -- phase
	end

	if true then
		local target_vec=vector3(0,0,1)
		local delta_=rotY:inverse()*self.target_Y
		target_vec:rotate(delta_)

		step_state:set(c+2, target_vec.x)
		step_state:set(c+3, target_vec.z)
	end

	assert(step_state:size()==self:getOBSdim())

	c=c+4

	for i,v in ipairs(terrain_sample_loc) do
		local pos=refCoord*v
		self:projectToGround(pos)
		step_state:set(c+i-1, pos.y-refCoord.translation.y)
	end
	assert(c+num_terrain_samples==step_state:size())

	if debug_mode then
		print('step_state', step_state)
	end
	return step_state
end
