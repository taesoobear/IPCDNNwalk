require('moduleIK')
local EntityToWRL={}

local function parseMappingFile(fn)
	local file=io.open(fn)
	local tbl={}
	if file then
		local lines=file:read('*a')
		lines=string.lines(lines)
		for i,l in ipairs(lines) do
			local a=string.tokenize(l,'%s')
			if string.sub(a[1],1,1)~='#' then
				tbl[a[1]]=a[2]
			end
		end

		file:close()
	else 
		return nil
	end
	return tbl
end
local function space(level)
	return '\n'..string.rep('    ',level)
end

local function nameId(bone)
	local name=string.gsub(bone.name,'%s','__SPACE__')
	return name
end

local cylinderRadious=0.025;
local function  packShapeRobot(bone,file, level, pLoader)
	local offset
	local child_offsets=vector3N()
	local com=vector3(0,0,0)
	local mass=0.01 -- actually just the sum of child-link-lengths.
	for i,c in ipairs(bone.children) do
		child_offsets:pushBack(c.bone:getPosition());
		com:radd(c.bone:getPosition())
		mass=mass+child_offsets(i-1):length();
	end
	com:rmult(0.5*1.0/#bone.children)
	local shape=''
	local radius=0.01; -- for setting inertia
	if(child_offsets:size()>0) then
		radius=mass/child_offsets:size()/2.0;
	end
	local inertia=1.0;

	inertia=inertia*mass*2.0/5.0*radius*radius;	-- spherical inertia.
	if com.x~=com.x then
		com.x=0
		com.y=0
		com.z=0
	end

	shape=shape..string.format("Segment { \n centerOfMass %f %f %f\n mass %f momentsOfInertia [%f 0 0 0 %f 0 0 0 %f]\n", com.x, com.y, com.z, mass, inertia, inertia, inertia);
	if (child_offsets:size()>0) then
		shape=shape..string.format("children [\n");

		for i=0,child_offsets:size()-1 do
			local offset=child_offsets(i);
			local q=quater();
			local axis=vector3();
			local dir=offset:copy()
			dir:normalize()
			q:axisToAxis(vector3(0,1,0), dir)
			local angle=q:toAxisAngle(axis);
			shape=shape..string.format("Transform { rotation %f %f %f %f translation %f %f %f\n",
			axis.x, axis.y, axis.z, angle, offset.x/2.0, offset.y/2.0, offset.z/2.0);

			shape=shape..string.format( "children Shape { geometry Capsule { radius %f height %f }}}\n", cylinderRadious, offset:length());
		end
		shape=shape..string.format("]\n}\n");
	else
		shape=shape..string.format("}\n");
	end
	shape=string.gsub(shape, "\n", space(level));
	file:write(shape)
end

local function packTransformRobot(bone, file, level, skel)
	local transform='';
	transform=transform..string.format("DEF %s Joint {\n", nameId(bone));

	if(level==0) then
		transform=transform..string.format("jointType \"free\"\n");
	else
			transform=transform..string.format("jointType \"rotate\"\n");
			transform=transform..string.format("jointAxis \"ZXY\"\n");
	end
	local sss=bone.bone:getScale()
	local offset=bone.bone:getPosition()

	local scaleFactor
	if sss.x~=1.0 then
		assert(level==0)
		scaleFactor=sss.x
	end
	--print(level, nameId(bone), bone.bone:getScale())
	transform=transform..string.format("  translation %f %f %f\n", offset.x, offset.y, offset.z);

	transform=transform..string.format("  children [\n");
	transform=string.gsub(transform, "\n", space(level));
	file:write(transform)
	transform=''

	packShapeRobot(bone, file, level+1, skel)

	for i,node in ipairs(bone.children) do
		if #node.children>=0 then
			packTransformRobot(node, file, level+1);
		end
	end

	transform=transform..string.format("]\n");
	transform=transform..string.format("}");
	transform=string.gsub(transform, "\n", space(level));
	file:write(transform)
	return scaleFactor, offset
end

function EntityToWRL.getBones(entity)
	local skel=entity:getSkeleton()
	local bones={}
	local nameToIdx={}
	for i=0,skel:getNumBones()-1 do
		local tbl={}
		local bone=skel:getBone(i)
		tbl.name=bone:getName()
		tbl.bone=bone
		tbl.parent=bone:getParent()
		tbl.id=i
		tbl.children={}

		bones[i]=tbl
		nameToIdx[tbl.name]=i
	end
	for i=0,skel:getNumBones()-1 do
		if bones[i].parent and bones[i].name~='Bip01_Footsteps' then
			local pid=nameToIdx[bones[i].parent:getName()]
			local bone=bones[i]
			bone.pid=pid
			table.insert(bones[pid].children, bone)
		end
	end

	--local root='Bip01_Pelvis'
	return bones, bones[0] --nameToIdx[root]]
end

function EntityToWRL.convert(entity, filename, robotname, cylinder_radius)
	cylinderRadious=cylinder_radius;
	
	local file=filename
	if type(file)=='string' then
		file=io.open(filename, "wt");
		if(not file) then
			print("can't open "..filename.." for writing")
		end
	end
	local scaleFactor, offset
	do
		file:write(string.format( "DEF SampleRobot Humanoid { \n name \"%s\"\n", robotname));
		file:write( "humanoidBody [\n");
		local bones, root=EntityToWRL.getBones(entity)
		scaleFactor,offset=packTransformRobot(root, file,0);
		file:write( "]\n}\n");
		file:close()
	end
	return (scaleFactor or 1), offset
end

-- loader obtained by using exportOgreEntityToVRML
function EntityToWRL.getPoseFromOgreEntity(loader, entity)

	local bones, root=EntityToWRL.getBones(entity)
	local pose=Pose()
	pose:init(loader:numRotJoint(), loader:numTransJoint())

	local s=1
	for i=root.id, #bones do
		local bone=bones[i]
		local ri=loader:getRotJointIndexByName( bone.name);
		local ti=loader:getTransJointIndexByName( bone.name);
		if ri~=-1 then
			pose.rotations(ri):assign(bone.bone:getOrientation())
		end
		if ri==0 then
			pose.translations(0):assign(bone.bone:getPosition())
			--pose.translations(0):assign(vector3(0,0,0))
			--print(bone.bone:getOrientation())
			s=bone.bone:getScale().x
		end
	end
	--print(RE.getSceneNode(ogreSkin):getPosition()/config.skinScale)

	pose.translations(0):scale(1/s)
	return pose,s
end

-- loader obtained by using exportOgreEntityToVRML
function EntityToWRL.setPoseToOgreEntity(loader, entity, node, skinScale)
	local bones, root=EntityToWRL.getBones(entity)
	local pose=Pose()
	loader:getPose(pose)

	local s=1
	for i=root.id, #bones do
		local bone=bones[i]
		local ri=loader:getRotJointIndexByName( bone.name);
		local ti=loader:getTransJointIndexByName( bone.name);
		if ri~=-1 then
			local q=pose.rotations(ri)
			assert(q==q)
			bone.bone:setOrientation(q)
			--bone.bone:setOrientation(quater(0,1,0,0))
			--bone.bone:_setDerivedOrientation(quater(0,1,0,0))
		end
		if ri==0 then
			s=bone.bone:getScale().x
			--bone.bone:setPosition(pose.translations(0)*s)
			bone.bone:setPosition(vector3(0,0,0))
		end
	end
	--entity:getSkeleton():_updateTransforms()

	node:setPosition(pose.translations(0)*skinScale)
	--local original_pos=lsh.bone:_getDerivedPosition()
	--lsh.bone:_setDerivedPosition(original_pos+vector3(0,0,1))
	--print(RE.getSceneNode(ogreSkin):getPosition()/config.entityScale)
	return pose,s
end

--require ("subRoutines/AnimOgreSkin")
return EntityToWRL
