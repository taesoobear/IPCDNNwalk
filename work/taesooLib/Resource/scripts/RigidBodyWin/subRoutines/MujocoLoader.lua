require('tl') -- for BaseLib <-> torch interfacing


package.path=package.path..";../Samples/sample_luatorch/lua/module/xml2lua/?.lua" --;"..package.path


package.path=package.path..";../../taesooLib/Samples/scripts/?.lua" --;"..package.path
package.path=package.path..";../../taesooLib/Samples/classification/lua/?.lua" --;"..package.path
require('module')
require('RigidBodyWin/subRoutines/VRMLexporter')
require("RigidBodyWin/subRoutines/MujocoXMLwriter")

function MujocoLoader(fn, options)
	if fn:sub(-4)=='.wrl' then
		local xmlfile=fn:sub(1,-4)..'mujoco.xml'
		local loader=MainLib.VRMLloader(fn)
		if not os.isFileExist(xmlfile) then
			writeMujocoXML(loader, xmlfile, options or { groundPlane=true})
		end
		return loader
	elseif fn:sub(-4)=='.xml' then
		local wrlfile=fn..'.wrl'
		---- set exportToMemory to false to actually write to the wrl file.
		if options then
			if options.useVisualMesh==nil then
				options.useVisualMesh=true
			end
			options.exportToMemory=true
		end
		local parser=MujocoParser(fn, wrlfile, options or {convertToYUP=false, useVisualMesh=true, exportToMemory=true} )
		return parser.loaders or parser.loader
	else
		assert(false)
	end
end
function parseV2(t)
	if t==nil then
		return vector3(0,0,0)
	end
	t=string.trimSpaces(t)
	t=string.tokenize(t,'%s+')
	assert(#t==2)
	return tonumber(t[1]), tonumber(t[2])
end
function parseV3(t)
	if t==nil then
		return vector3(0,0,0)
	end
	t=string.trimSpaces(t)
	t=string.tokenize(t,'%s+')
	assert(#t==3)
	return vector3(tonumber(t[1]), tonumber(t[2]), tonumber(t[3]))
end
function parseV4(t)
	if t==nil then
		return vector3(0,0,0)
	end
	t=string.trimSpaces(t)
	t=string.tokenize(t,'%s+')
	assert(#t==4)
	return vector3(tonumber(t[1]), tonumber(t[2]), tonumber(t[3])), tonumber(t4)
end
function parseColor(t)
	if t==nil then
		return quater(1, 1,1,0)
	end
	t=string.trimSpaces(t)
	t=string.tokenize(t,'%s+')
	assert(#t==4)
	return quater(1-tonumber(t[4]), tonumber(t[1]), tonumber(t[2]), tonumber(t[3]))
end
function parseQuat(t)
	if t==nil then
		return quater(1, 0,0,0)
	end
	t=string.trimSpaces(t)
	t=string.tokenize(t,'%s+')
	assert(#t==4)
	return quater(tonumber(t[1]), tonumber(t[2]), tonumber(t[3]), tonumber(t[4])):Normalize()
end
function parseV6(t)
	t=string.trimSpaces(t)
	t=string.tokenize(t,'%s+')
	assert(#t==6)
	return vector3(tonumber(t[1]), tonumber(t[2]), tonumber(t[3])), vector3(tonumber(t[4]), tonumber(t[5]), tonumber(t[6]))
end

MujocoParser=LUAclass()
function MujocoParser:__init(filename, wrlfn, options)
	if options==nil then 
		options={}
	elseif type(options)=='boolean' then
		options={convertToYUP=options}
	end
	self.options=options
	assert(type(options)=='table')
	local xml2lua = require("xml2lua/xml2lua")
	--Uses a handler that converts the XML to a Lua table
	local tree = require("xml2lua/xmlhandler.tree")
	local handler=tree:new()
	local xml = util.readFile(filename)
	assert(xml)
	
	--Instantiates the XML parser
	local parser = xml2lua.parser(handler)
	parser:parse(xml)

	self.convertToYUP=options.convertToYUP
	self.root=handler.root.mujoco
	self.default=self.root.default
	if self.default then
		if self.default.default then
			self.default=self.default.default
		end

		for k, v in pairs(self.default) do
			if v._attr then
				if v._attr.class  then
					--dbg.console()
				end
			else
				for i, v2 in ipairs(v) do
					local c=v2._attr.class 
					if c then
						if not self.default[c] then
							self.default[c]={}
						end
						if self.default[c][1] ~= v2 then 
							table.insert(self.default[c], v2)
						end
					end
				end
			end
		end
	else
		self.default={}
	end
	local asset=self.root.asset 
	if asset then
		local mat=asset.material
		if mat then
			local material={}
			for i, v in ipairs(mat) do
				if v._attr.rgba then
					material[v._attr.name]=parseColor(v._attr.rgba)
				end
			end
			self.material=material
		end
	end

	if (handler.root.mujoco.worldbody.body._attr) then
		self.bones={}
		self.boneNameToIdx={}
		self.boneInfo={}
		self:parseBody(quater(1,0,0,0), 0, handler.root.mujoco.worldbody.body, 1, options)
	else

		self.bodies={}
		for ibody, body in ipairs(handler.root.mujoco.worldbody.body) do
			local currBody={}
			currBody.bones={}
			currBody.boneNameToIdx={}
			currBody.boneInfo={}

			setmetatable(currBody, getmetatable(self))
			currBody:parseBody(quater(1,0,0,0), 0, body, 1, options)
			table.insert(self.bodies, currBody)
		end

		
	end
	if self.bodies or options.parseGeom then
		if not self.bodies then
			self.bodies={self}
		end
		for igeom, geom in ipairs(handler.root.mujoco.worldbody.geom) do

			local attr=geom._attr
			if attr.hfield then
				local meshes=self.root.asset.hfield
				local found=nil
				for i, mesh in ipairs(meshes) do
					if mesh._attr.name==attr.hfield then
						found=mesh
						break
					end
				end
				if found then
					attr.hfield=found._attr
				end
			end
			table.insert(self.bodies, geom._attr)
		end
	end
	self.name=handler.root.mujoco._attr.model

	local mesh
	if handler.root.mujoco.asset then
		mesh=handler.root.mujoco.asset.mesh
	end

	if self.convertToYUP then
		self.convertAxes= function (v)
			return vector3(v.y, v.z, v.x)
			--return vector3(v.z, v.x, v.y)
			--return vector3(v.x, v.y, v.z)
		end

		self.convertQuatAxes= function (self, q)
			local v=vector3(q.x, q.y, q.z)
			v=self.convertAxes(v)
			q:setValue(q.w, v.x, v.y, v.z)
		end
	end
	if self.bodies then

		self.loaders={}
		for ibody, body in ipairs(self.bodies) do
			if body.type=='plane' or body.type=='hfield' then
				if body.type=='hfield' then
					local pos= self.convertAxes(parseV3(body.pos))
					local size, min_thickness=parseV4(body.hfield.size)
					size= self.convertAxes(size)
					local quat= self.convertAxes(parseQuat(body.quat))
					local color= parseColor(body.quat)
					local _, parentPath=os.processFileName(filename)
					local l=MainLib.VRMLloader(os.joinPath(parentPath, body.hfield.file), size.x, size.z, size.y, 1, 1) 
					local newpos=pos-vector3(size.x, 0, size.z)

					if false then -- set true only in taesooLib legacy where lower resolution is enforced.
						-- heuristic correction.  
						local res=512  -- todo. use image resolution
						print (pos)
						if pos.x>0 then
							newpos:radd(vector3(-size.x*3/res, 0,0))
						else
							newpos:radd(vector3(-size.x*12/res, 0,0))
						end
						if pos.z<0 then
							newpos:radd(vector3(0, 0,-size.z*10/res))
						else
							newpos:radd(vector3(0, 0,size.z*-3/res))
						end
					end
					l:setPosition(newpos)
					self.loaders[ibody]=l
				else
					local pos= self.convertAxes(parseV3(body.pos))
					local ori= parseQuat(body.quat)
					self:convertQuatAxes(ori)
					local size=parseV3(body.size)
					local geom=Geometry()
					geom:initPlane(size.x, size.z)
					local loader=MainLib.VRMLloader(geom, true)
					local root=loader:VRMLbone(1)
					root:setJointPosition(pos)
					root:getLocalFrame().translation:assign(pos);
					root:getLocalFrame().rotation:assign(ori);
					loader:fkSolver():forwardKinematics()
					loader:setURL(table.tostring2({pos,ori}))
					self.loaders[ibody]=loader
				end
			else
				local file=CTextFile()

				local name=body.name or ((self.name or 'robot')..ibody)
				if not body.name and body.bones and body.bones[1] and body.bones[1].name then
					name=body.bones[1].name
				end

				local out=VRMLexporter.generateWRLstring(body.bones,name , wrlfn..ibody)
				file:OpenMemory(out)
				local loader=MainLib.VRMLloader(file)
				if self.convertToYUP then
					loader=loader:ZtoY()
				end
				if loader.dofInfo:numDOF()==0 then
					loader:setPosition(self.convertAxes(body.bones[1].offset))
				end
				self.loaders[ibody]=loader
			end
		end
	else
		if options.exportToMemory then
			local out=VRMLexporter.generateWRLstring(self.bones, self.name or 'robot', wrlfn)
			local file=CTextFile()
			file:OpenMemory(out)
			local loader=MainLib.VRMLloader(file)
			if self.convertToYUP then
				loader=loader:ZtoY()

				--g_loader=loader
				--g_skin=RE.createSkin(loader)
				--g_skin:setScale(100)
			end
			self.loader=loader
		else
			if self.convertToYUP then
				-- todo: load, ZtoY , then  export
				assert(false)
			else
				VRMLexporter.exportWRL(self.bones, wrlfn, self.name)
			end
		end
	end
end

function MujocoParser:getJointAxis(p)
	if p._attr.axis then
		return p._attr.axis
	elseif p._attr.class then
		local classInfo=self.default[p._attr.class]
		assert(#classInfo==1)
		return classInfo[1].joint._attr.axis
	else
		return self.default.joint._attr.axis
	end
end
function MujocoParser:getJointType(p)
	if p._attr.type then
		return p._attr.type
	elseif p._attr.class then
		local classInfo=self.default[p._attr.class]
		assert(#classInfo==1)
		return classInfo[1].joint._attr.type
	elseif p._attr.axis then
		return 'hinge'
	else
		return self.default.joint._attr.type 
	end
end
function MujocoParser:getJointAxisString(jointAxis, p)
	local axis=self:getJointAxis(p)
	if axis=='1 0 0' then
		jointAxis=jointAxis..'X'
	elseif axis=='0 1 0' then
		jointAxis=jointAxis..'Y'
	elseif axis=='0 0 1' then
		jointAxis=jointAxis..'Z'
	else
		return nil
	end
	return jointAxis
end
function getR(x,y)
	local z=vector3()
	z:cross(x,y)
	local m=matrix4()
	m:identity()
	m:setColumn(0,x)
	m:setColumn(1,y)
	m:setColumn(2,z)
	local q=quater()
	q:setRotation(m)
	return q
end
function MujocoParser:parseBody(q_parent, pid, p, level, options)
	--if level>2 then return end
	local bones=self.bones
	local boneNameToIdx=self.boneNameToIdx
	local name=p._attr.name
	print(name)

	local pos=parseV3(p._attr.pos)
	local q=q_parent:copy()
	local q_l=parseQuat(p._attr.quat)
	if p._attr.xyaxes then
		local x,y=parseV6(p._attr.xyaxes)
		q_l=getR(x,y)
	end
	q=q*q_l
	local bone
	if p.inertial then
		bone={id=#bones+1, name=name, offset=q_parent*pos, localCOM=q_parent*parseV3(p.inertial._attr.pos)}
		if pid~=0 then
			bone.pid=pid
		end
		table.insert(bones, bone)
		table.insert(self.boneInfo, 
		{
			q_parent=q_parent:copy(),
			q=q:copy(),
			q_local=q_l:copy(),
			mass=tonumber(p.inertial._attr.mass),
			fullinertia=p.inertial._attr.fullinertia,
			diaginertia=p.inertial._attr.diaginertia,
			localCOM=p.inertial._attr.pos
		}
		)

	else
		bone={id=#bones+1, name=name, offset=q_parent*pos, } 
		if pid~=0 then
			bone.pid=pid
		end
		table.insert(bones, bone)
		table.insert(self.boneInfo, 
		{
			q_parent=q_parent:copy(),
			q=q:copy(),
			q_local=q_l:copy(),
		}
		)
	end

	if p.geom then
		local geoms=p.geom
		if #geoms==0 then
			geoms={p.geom}
		end
		assert(#geoms>0)

		bone.shapes={}
		bone.classes={}
		if p._attr.childclass then 
			self.childClass = p._attr.childclass
		end
		for igeom, tgeom in ipairs(geoms) do
			local geom=tgeom._attr
			if geom.class==nil and geom.type==nil and self.childClass and self.default._attr.class == self.childClass then 
				geom.type=self.default.geom._attr.type
			elseif geom.class==nil and geom.type==nil then
				geom.type='sphere'
			end
			if geom.type=='sphere' then
				local s=tonumber(geom.size)
				local mass=tonumber(geom.mass)
				local pos=parseV3(geom.pos)
				table.insert(bone.shapes,{'Sphere', translation=pos, size=vector3(s,s,s), mass=mass })
			elseif geom.type=='capsule' then
				local a,b=parseV6(geom.fromto)
				local pos=a*0.5+b*0.5
				local s=tonumber(geom.size)
				local mass=tonumber(geom.mass)
				local q=quater()
				q:axisToAxis(vector3(0,1,0), b-a)
				table.insert(bone.shapes,{'Capsule', translation=pos, rotation=q, height=a:distance(b), radius=s, mass=mass })
			elseif geom.type=='cylinder' then
				local pos=parseV3(geom.pos)
				local radius, height=parseV2(geom.size)
				height=height*2
				local mass=tonumber(geom.mass)
				local q=parseQuat(geom.quat)
				q:rightMult(quater(math.rad(90), vector3(1,0,0)))
				table.insert(bone.shapes,{'Cylinder', translation=pos, rotation=q, height=height, radius=radius, mass=mass })
			elseif geom.type=='hfield' then
				dbg.console()

			elseif geom.type=='box' then
				local s=parseV3(geom.size)*2
				local mass=tonumber(geom.mass)
				local pos=parseV3(geom.pos)
				local ori=parseQuat(geom.quat)
				table.insert(bone.shapes,{'Box', translation=pos, rotation=ori, size=s, mass=mass })
			elseif (geom.mesh and not geom.class)or (geom.class=='visual' and options.useVisualMesh) then
				local meshes=self.root.asset.mesh
				local pos=parseV3(geom.pos)
				local ori=parseQuat(geom.quat)
				local found=nil
				for i, mesh in ipairs(meshes) do
					if mesh._attr.name==geom.mesh then
						found=mesh
						break
					end
				end
				if found then
					-- not implemented yet
					local mat
					if self.material then
						mat=self.material[geom.material]
					end
					table.insert(bone.shapes,{'OBJ_no_classify_tri', found._attr.file, rotation=q*ori, color=mat, translation=q*pos})
				else

					local mat
					if self.material then
						mat=self.material[geom.material]
					end
					table.insert(bone.shapes,{'OBJ_no_classify_tri', geom.mesh..'.obj', rotation=q*ori, color=mat, translation=q*pos})
				end
			elseif geom.class then
				-- unused yet.
				table.insert(bone.classes, geom)
			else
				assert(false)
			end
		end
	end

	if p.joint then
		if #p.joint==0 then
			local jtype=self:getJointType(p.joint) or 'rotate'
			if jtype=='hinge' then
				jtype='rotate'
			end
			if jtype=='free' then
			else
				bone.jointType=jtype
				bone.jointAxis=q*parseV3(self:getJointAxis(p.joint))
			end
		else
			local jointAxis=''
			for i=1, #p.joint do
				jointAxis=self:getJointAxisString(jointAxis, p.joint[i])
				if not jointAxis then break end
			end
			bone.jointType='rotate'
			if jointAxis then
				bone.jointAxis=jointAxis
			else
				bone.jointAxis={}
				for i=1, #p.joint do
					bone.jointAxis[i]=q*parseV3(self:getJointAxis(p.joint[i]))
				end
			end
		end
	elseif p.freejoint then
	else
		bone.jointType='fixed'
	end

	boneNameToIdx[name]=#bones
	local id=#bones
	if p.body then
		if #p.body==0 then
			self:parseBody(q, id, p.body, level+1, options)
		else
			for i, v in ipairs(p.body) do
				self:parseBody(q, id, v, level+1, options)
			end
		end
	end
	self.bones[1].info={
		assetFolder='assets',
	}
end

