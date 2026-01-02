
function rayIntersectsBox(ray, bbmin, bbmax, mtrix)
	local function makePlane(plane, a, b, cow2wld, n)
		local v=a:copy()

		if(n.x==1.0) then v.x=b.x;
		elseif(n.x==-1.0) then v.x=a.x;
		else assert(n.x==0.0); end

		if(n.y==1.0) then v.y=b.y;
		elseif(n.y==-1.0) then v.y=a.y;
		else assert(n.y==0.0); end

		if(n.z==1.0) then v.z=b.z;
		elseif(n.z==-1.0) then v.z=a.z;
		else assert(n.z==0.0); end
		
		local normal=cow2wld:rotate(n)
		normal:normalize()
		plane:setPlane(normal, cow2wld*v)
	end

	local planes=Planes()
	planes:resize(6)
	makePlane(planes(0), bbmin, bbmax, mtrix, vector3(0,1,0));
	makePlane(planes(1), bbmin, bbmax, mtrix, vector3(0,-1,0));
	makePlane(planes(2), bbmin, bbmax, mtrix, vector3(1,0,0));
	makePlane(planes(3), bbmin, bbmax, mtrix, vector3(-1,0,0));
	makePlane(planes(4), bbmin, bbmax, mtrix, vector3(0,0,1));
	makePlane(planes(5), bbmin, bbmax, mtrix, vector3(0,0,-1));

	local out=ray:intersects(planes)
	local bret=out(0)
	local o=out(1)
	if bret>0.5 then
		bret=true
	else
		bret=false
	end
	return bret, o
end
