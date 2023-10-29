local Slope=LUAclass()

function Slope:__init(n)
	self.normal=n:copy()
	self.plane=Plane(self.normal, 0)
end

function Slope:getTerrainPos(xzpos)
	local h=self:height(vector2(xzpos.x, xzpos.z))
	return vector3(xzpos.x, h, xzpos.z)
end

function Slope:height(x, normal)

	local ray=Ray(vector3(x(0), 1000, x(1)), vector3(0,-1,0))
	local res=ray:intersects(self.plane)
	assert(res(0)==1)
	local pos=ray:getPoint(res(1))
	assert(pos.x==x(0))
	if normal then
		normal:assign(self.normal)
	end
	return pos.y
end
return Slope
