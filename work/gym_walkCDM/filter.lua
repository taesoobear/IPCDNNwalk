require("gym_walk/IPC3d_approx")
--package.path=package.path..";../Samples/QP_controller/ILIPM/?.lua" --;"..package.path
--require("collisionAvoid/collisionAvoid")
local Filter={}

Filter.Scalar=LUAclass()


function Filter.Scalar:__init(mass, initialPos)
	self.sds=SDS(mass, 0.0001, 1, 100000000, 0,1/120)
	self.sds.xd:set(0,0,initialPos)
	self.sds.xd:set(1,0,0)
end

function Filter.Scalar:step(pos)
	local mPendulum=self.sds
	mPendulum.xd:set(0,0,pos)
	for i=1,4 do
		mPendulum:singleStep()
	end
	local pos=mPendulum.x(0,0)
	return pos
end

Filter.VecXZ=LUAclass()
function Filter.VecXZ:__init(mass, initialPos)
	self.x=Filter.Scalar(mass, initialPos.x)
	self.z=Filter.Scalar(mass, initialPos.z)
end

function Filter.VecXZ:step(pos)
	local x=self.x:step(pos.x)
	local z=self.z:step(pos.z)
	return vector3(x,0,z)
end

return Filter
