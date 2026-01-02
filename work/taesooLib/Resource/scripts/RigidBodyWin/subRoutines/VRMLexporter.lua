-- 사용법. 
-- [괄효]안에 있는 내용은 optional. 없는 경우 기본값이 사용됨.]
-- The root bone becomes bones[1]
-- and other bones follows.
-- bones={
-- 	  { name='root', offset=vector3(0,0,0), localCOM=vector3(0,0,0), id =1, [ jointType='rotate', axis='x', shape={"Box", mass=10, size=vector3(1,1,1)}] }
-- 	  { name='hipR', offset=vector3(0.05,-0.1,0), id =2, pid=1, }
-- 	  { name='hipL', offset=vector3(-0.05,-0.1,0), localCOM=vector3(0,0,0), id =3,pid=1, [ jointType='rotate', axis='x', shape={"OBJ", "a.obj", inertia=vector3(1,1,1)}] }
--  ... 
-- }
--
-- See MujocoLoader.lua for more detailed examples.

--function VRMLexporter.exportWRL(bones, filename, robotname)
--or 
-- function VRMLexporter.generateWRLstring(bones, robotname)
-- or
--function VRMLexporter.createVRMLloader(bones, robotname)
require('moduleIK')

local function space(level)
	return '\n'..string.rep('    ',level)
end
local function nspace(level)
	return string.rep('    ',level)
end

local function nameId(bone)
	return bone.name
end

local function boxInertia(mass, size)
	local 	Ix=	mass*1.0/12.0*(size.y*size.y+size.z*size.z)
	local 	Iy=	mass*1.0/12.0*(size.x*size.x+size.z*size.z)
	local 	Iz=	mass*1.0/12.0*(size.y*size.y+size.x*size.x)
	return vector3(Ix, Iy, Iz)
end
local function cylinderInertia(mass, radius, height)
	local 	Ix=	mass*1.0/12.0*(3*radius*radius+height*height)
	local 	Iy=	mass*1.0/2.0*radius*radius
	return vector3(Ix, Iy, Ix)
end

local function sphereInertia(mass, size)
	local 	Ix=	mass*1.0/5.0*(size.y*size.y+size.z*size.z)
	local 	Iy=	mass*1.0/5.0*(size.x*size.x+size.z*size.z)
	local 	Iz=	mass*1.0/5.0*(size.y*size.y+size.x*size.x)
	return vector3(Ix, Iy, Iz)
end

local function boxSizeFromInertia(mass, Ix, Iy, Iz)
	local yypzz=Ix/mass*12.0
	local xxpzz=Iy/mass*12.0
	local xxpyy=Iz/mass*12.0

	local A=matrix3()
	A:setValue(0,1,1, 1,0,1, 1,1,0)
	local invA=matrix3()
	invA:inverse(A)
	-- A*[xx;yy;zz]=[yypzz; xxpzz; xxpyy]
	local XX=invA*vector3(yypzz, xxpzz, xxpyy)
	return vector3(math.sqrt(XX.x), math.sqrt(XX.y), math.sqrt(XX.z))
end
local function geometryNode(rotation, translation, geometryString)
	local axis=vector3();
	local angle=rotation:toAxisAngle(axis);
	local shape=string.format("Transform { rotation %f %f %f %f translation %f %f %f\n",
	axis.x, axis.y, axis.z, angle, translation.x, translation.y, translation.z);
	shape=shape..string.format( "    children Shape { %s}}\n", geometryString);
	return shape
end

local cylinderRadious=0.025;
local function calcDefaultShape(bone)
	-- compute using child links
	local child_offsets=vector3N()
	local com=vector3(0,0,0)
	local mass=0.01 -- actually just the sum of child-link-lengths.
	if not bone.children then
		bone.children={}
	end
	for i,c in ipairs(bone.children) do
		child_offsets:pushBack(c.offset)
		com:radd(c.offset)
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
	inertia=math.max(inertia, 1e-6)
	if com.x~=com.x then
		com.x=0
		com.y=0
		com.z=0
	end
	local vinertia=vector3(inertia, inertia, inertia)

	local shape=''
	if (child_offsets:size()>0) then
		shape=string.format("children [\n");

		for i=0,child_offsets:size()-1 do
			local offset=child_offsets(i);
			local q=quater();
			local dir=offset:copy()
			dir:normalize()
			q:axisToAxis(vector3(0,1,0), dir)
			shape=shape..geometryNode(q, offset*0.5, string.format("geometry Capsule { radius %f height %f }", cylinderRadious, math.max(offset:length(), 1e-3)))
		end
		shape=shape..string.format("]\n");
	end
	return mass, com, vinertia, shape
end
local function  packShapeRobot(bone,file, level, pLoader)
	local offset

	local vinertia, mass, com, children_shape
	if bone.shape or bone.shapes then

		if not bone.shapes then
			assert(bone.shape)
			bone.shapes={bone.shape}
		else
			assert(not bone.shape)
		end

		children_shape=" children [\n"

		mass=0
		com=vector3(0,0,0)
		vinertia=vector3(0,0,0) --회전이 있으면 제대로 다시 구현해야함.
		for ishape, shape in ipairs(bone.shapes) do
			local smass=shape.mass
			local scom=shape.translation
			local ori=shape.rotation

			if not scom then
				scom=bone.localCOM or vector3(0,0,0)
			end
			if not ori then
				ori=quater(1,0,0,0)
			end
			if not smass then
				if not bone.mass then
					bone.mass=1e9
				end
				smass=bone.mass/#bone.shapes
			end
			if smass==0 then
				smass=1e-3
			end
			local colorStr=''
			if shape.color then
				local color=shape.color
				colorStr=string.format(" color %f %f %f %f", color.x, color.y, color.z, color.w)
			end
			if shape[1]=='Box' then
				local boxsize=shape.size
				assert(boxsize)
				assert(shape[1]=="Box")
				vinertia=vinertia+boxInertia(smass, boxsize)
				children_shape= children_shape..
				geometryNode(ori, scom, string.format("geometry Box { size %f %f %f %s}\n", boxsize.x, boxsize.y, boxsize.z, colorStr))
			elseif shape[1]=='Sphere' or shape[1]=='Ellipsoid' then
				local boxsize=shape.size
				assert(boxsize)
				vinertia=vinertia+sphereInertia(smass, boxsize)
				children_shape= children_shape..
				geometryNode(ori, scom, string.format("geometry Ellipsoid { size %f %f %f %s}\n", boxsize.x, boxsize.y, boxsize.z, colorStr))
			elseif shape[1]=='Capsule' or shape[1]=='Cylinder' then
				local boxsize=shape.size
				local radius, height
				if not boxsize then
					assert(shape.radius and shape.height)
					radius=shape.radius
					height=shape.height
				else
					radius=shape.size.x*0.5
					height=shape.size.y
				end
				vinertia=vinertia+cylinderInertia(smass, radius, height)
				if shape[1]=='Cylinder' and shape.numDivision then
					colorStr=colorStr..' numDivision '..shape.numDivision
				end
				children_shape= children_shape..
				geometryNode(ori, scom, string.format("geometry %s { radius %f height %f %s}\n", shape[1], radius, height, colorStr))
			elseif shape[1]=='OBJ' then
				children_shape= children_shape..
				geometryNode(ori, scom, string.format('geometry OBJ "%s"\n', shape[2]))
			elseif shape[1]=='OBJ_no_classify_tri' then
				if shape.color then
					local color=shape.color
					local color_str=string.format(" color %f %f %f %f\n", color.x, color.y, color.z, color.w)
					children_shape= children_shape..
					geometryNode(ori, scom, string.format('geometry OBJ_no_classify_tri "%s"\n', shape[2])..color_str)
				else
					children_shape= children_shape..
					geometryNode(ori, scom, string.format('geometry OBJ_no_classify_tri "%s"\n', shape[2]))
				end
			else
				print('not implemented yet!')
				assert(false)
			end
			mass=mass+smass
			com=com+scom*smass
		end
		com=com/mass
		children_shape=children_shape.."  ]\n"


		if #bone.shapes==0 then
			children_shape='\n'
		end
	else
		mass, com, vinertia, children_shape=calcDefaultShape(bone)
	end
	if bone.localCOM then
		com=bone.localCOM
	end
	if com.x~=com.x then
		print("?????? nan com")
		dbg.console()
		com=vector3(0,0,0)
	end
	if bone.inertia then
		vinertia=bone.inertia
	end

	if dbg.lunaType(vinertia)=='matrix3' then
		shape=string.format("    Segment { \n  centerOfMass %f %f %f\n  mass %f momentsOfInertia [%f %f %f %f %f %f %f %f %f]\n", com.x, com.y, com.z, mass, 
		vinertia._11, vinertia._12, vinertia._13,
		vinertia._21, vinertia._22, vinertia._23,
		vinertia._31, vinertia._32, vinertia._33
		);
	else
		shape=string.format("    Segment { \n  centerOfMass %f %f %f\n  mass %f momentsOfInertia [%f 0 0 0 %f 0 0 0 %f]\n", com.x, com.y, com.z, mass, vinertia.x, vinertia.y, vinertia.z);
	end

	shape=shape..'  material "use_vertexcolor"\n'
	shape=shape..children_shape.."}"
	shape=string.gsub(shape, "\n", space(level))..'\n';
	file:write(shape)
end

local function packTransformRobot(bone, file, level, skel)
	local transform=nspace(level);
	transform=transform..string.format("DEF %s Joint {\n", nameId(bone));


	if bone.jointType then
		transform=transform..string.format("  jointType \""..bone.jointType.."\"\n");
		if bone.jointAxis then
			if type(bone.jointAxis)=='string' then
				transform=transform..string.format("  jointAxis \""..bone.jointAxis.."\"\n");
			elseif type(bone.jointAxis)=='table' then
				for i, v in ipairs(bone.jointAxis) do
					transform=transform..string.format("  axis "..v.x..' '..v.y..' '..v.z.."\n");
				end
			else
				local v=bone.jointAxis
				transform=transform..string.format("  axis "..v.x..' '..v.y..' '..v.z.."\n");
			end
		elseif bone.axis then
			transform=transform..string.format("  jointAxis \""..bone.axis.."\"\n");
		end
	else
		if(level==0) then
			transform=transform..string.format("  jointType \"free\"\n");
		else
			transform=transform..string.format("  jointType \"rotate\"\n");
			transform=transform..string.format("  jointAxis \"ZXY\"\n");
		end
	end
	local offset=bone.offset
	assert(offset)

	local scaleFactor
	if bone.scale then
		assert(level==0)
		scaleFactor=bone.scale
	end
	--print(level, nameId(bone), bone.bone:getScale())
	transform=transform..string.format("  translation %f %f %f\n", offset.x, offset.y, offset.z);
	transform=transform..string.format("  children [\n");
	transform=string.gsub(transform, "\n", space(level));
	file:write(transform)

	packShapeRobot(bone, file, level+1, skel)

	if bone.children then
		for i,node in ipairs(bone.children) do
			packTransformRobot(node, file, level+1);
		end
	end
	transform=nspace(level).."  ]"..space(level);
	transform=transform.."}\n";
	file:write(transform)
	return scaleFactor, offset
end

function getBones(entity)
	local skel=entity:getSkeleton()
	local bones={}
	local nameToIdx={}
	for i=0,skel:getNumBones()-1 do
		local tbl={}
		local bone=skel:getBone(i)
		tbl.name=bone:getName()
		tbl.bone=bone
		tbl.scale=bone.bone:getScale()
		tbl.offset=bone.bone:getPosition()
		tbl.parent=bone:getParent()
		local id=i+1-- one indexing
		tbl.id=id 

		bones[id]=tbl
		nameToIdx[tbl.name]=id
	end
	for ii=0,skel:getNumBones()-1 do
		local i=ii+1 -- convert to one-indexing
		if bones[i].parent and bones[i].name~='Bip01_Footsteps' then
			local pid=nameToIdx[bones[i].parent:getName()]
			local bone=bones[i]
			bone.pid=pid
		end
	end

	--local root='Bip01_Pelvis'
	return bones, bones[1] --nameToIdx[root]]
end


-- loader obtained by using exportOgreEntityToVRML

VRMLexporter={}
function VRMLexporter.cleanupWRL(vrmlloader, filename, robotname)
	local bones={}
	for i=1, vrmlloader:numBone()-1 do
		local bone=vrmlloader:VRMLbone(i)
		bones[i]={ name=bone:name(), offset=bone:getOffset(), localCOM=bone:localCOM(), id=i}
		if i>1 then
			bones[i].pid=bone:parent():treeIndex()
		end
		local nj=bone:numHRPjoints()
		if nj>=1 then
			local jt=bone:HRPjointType(0)
			if jt==MainLib.VRMLTransform.FREE then
				bones[i].jointType='free'
			elseif jt==MainLib.VRMLTransform.ROTATE then
				local a=''
				for j=0, nj-1 do
					assert(bone:HRPjointType(j)==jt)
					a=a..bone: HRPjointAxis(j)
				end
				bones[i].jointType='rotate'
				bones[i].axis=a
			else
				dbg.console()
			end
			assert(bone:hasShape())
			local m=bone:getMesh()
			assert(m:numElements()==1)
			local e=m:element(0)
			local et=e.elementType
			local mass=bone:mass()
			local minMass=3

			if mass<minMass then
				mass=minMass
			end
			if et==OBJloader.Element.BOX then
				bones[i].shape={"Box", mass=mass, size=m:element(0).elementSize:copy()}
			elseif et==OBJloader.Element.OBJ then
				local sx=math.pow(mass/1000, 1/3)
				bones[i].shape={"Box", mass=mass, size=vector3(sx,sx,sx)}
			elseif et==OBJloader.Element.CYLINDER then
				local radius=e.elementSize.x/2
				local height=e.elementSize.y
				--bones[i].shape={"", mass=mass, size=vector3(sx,sx,sx)}
				bones[i].shape={"Box", mass=mass, size=e.tf.rotation*e.elementSize}
			else
				dbg.console()
			end
			if true then
				-- assume water density
				local s=bones[i].shape.size
				s.x=math.abs(s.x)
				s.y=math.abs(s.y)
				s.z=math.abs(s.z)
				bones[i].shape.mass=s.x*s.y*s.z*1000
				if bones[i].shape.mass<0 then
					dbg.console()
				end
				print(mass, bones[i].shape.mass)
				assert(mass>=minMass)
			end
		else
			-- not implemented yet
			assert(false)
		end
	end
	VRMLexporter.exportWRL(bones, filename, robotname)
end

local function packRootInfo(root,file)
	if root.info then
		-- root info
		local info=root.info
		if info.assetFolder then
			file:write(' assetFolder "'..info.assetFolder..'"\n')
		end
	end
end
function VRMLexporter.exportWRL(bones, filename, robotname)
	if not bones[1].childrenValid then
		for i=2,#bones do -- bone 1 is the root.
			local bone=bones[i]
			local pid=bone.pid
			if not bones[pid].children then
				bones[pid].children={}
			end
			table.insert(bones[pid].children, bone)
		end
	end
	bones[1].childrenValid=true

	local file=io.open(filename, "wt");
	if(not file) then
		print("can't open "..filename.." for writing")
	end
	local scaleFactor, offset
	do
		file:write(string.format( "DEF SampleRobot Humanoid {\n name \"%s\" ", robotname));
		local root=bones[1]
		packRootInfo(root, file)
		file:write( " humanoidBody [\n");
		scaleFactor,offset=packTransformRobot(root, file,0);
		file:write( "] } # Humanoid\n");
		file:close()
	end
	return (scaleFactor or 1), offset
end
function VRMLexporter.generateWRLstring(bones, robotname, url)
	if not bones[1].childrenValid then
		for i=2,#bones do -- bone 1 is the root.
			local bone=bones[i]
			local pid=bone.pid
			if not bones[pid].children then
				bones[pid].children={}
			end
			table.insert(bones[pid].children, bone)
		end
	end
	bones[1].childrenValid=true

	local file={}
	function file:write(str)
		table.insert(self, str)
	end
	local scaleFactor, offset
	do
		file:write(string.format( "DEF SampleRobot Humanoid {\n name \"%s\" ", robotname));
		if url then
			file:write(string.format( " url \"%s\" ", url));
		end
		local root=bones[1]
		packRootInfo(root, file)
		file:write( " humanoidBody [\n");
		scaleFactor,offset=packTransformRobot(root, file,0);
		file:write( "] } # Humanoid\n");
	end
	return table.concat(file,'\n'), (scaleFactor or 1), offset
end

function VRMLexporter.generateWRLstring_osim(bones, robotname, url)
	for i=2,#bones do -- bone 1 is the root.
		local bone=bones[i]
		local pid=bone.pid
		if not bones[pid].children then
			bones[pid].children={}
		end
		table.insert(bones[pid].children, bone)
	end

	local file={}
	function file:write(str)
		table.insert(self, str)
	end
	local scaleFactor, offset
	do
		file:write(string.format( "DEF SampleRobot Humanoid { name \"%s\" ", robotname));
		if url then
			file:write(string.format( " url \"%s\" ", url));
		end
		file:write( "humanoidBody [\n");
		local root=bones[1]
		scaleFactor,offset=packTransformRobot(root, file,0);
		file:write( "] } # Humanoid\n");
	end
	return table.concat(file,'\n'), (scaleFactor or 1), offset
end

function VRMLexporter.createVRMLloader(bones, robotname, printDebugInfo)
	local str=VRMLexporter.generateWRLstring(bones, robotname)
	if printDebugInfo then
		print(str)
	end
	local file=CTextFile()
	file:OpenMemory(str)
	local loader=MainLib.VRMLloader(file)
	return loader
end



function VRMLexporter.exportOgreEntityToWRL(entity, filename, robotname, cylinder_radius)
	local bones, root=getBones(entity)
	for i=1, #bones do
		bones[i].shape='cylinder'
		bones[i].cylinder_radius=cylinder_radius
	end
	VRMLexporter.exportWRL(bones, filename, robotname)
end
VRMLexporter.boxSizeFromInertia=boxSizeFromInertia
VRMLexporter.boxInertia=boxInertia
