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
		local parser=MujocoParser(fn, wrlfile, options or {convertToYUP=false, useVisualMesh=true, exportToMemory=true} )

		local out=VRMLexporter.generateWRLstring(parser.bones, parser.name or 'robot', wrlfile)
		local file=CTextFile()
		file:OpenMemory(out)
		local loader=MainLib.VRMLloader(file)
		--local loader=MainLib.VRMLloader(wrlfile)
		
		return loader
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
function MujocoParser:convertQuatAxes(q)
	local v=vector3(q.x, q.y, q.z)
	v=self.convertAxes(v)
	q:setValue(q.w, v.x, v.y, v.z)
end
function MujocoParser:__init(filename, wrlfn, options)
	if options==nil then 
		options={}
	elseif type(options)=='boolean' then
		options={convertToYUP=options}
	end
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
	local convertToYUP=options.convertToYUP
	if convertToYUP then
		self.convertToYUP=true
		self.convertAxes= function (v)
			return vector3(v.y, v.z, v.x)
			--return vector3(v.z, v.x, v.y)
			--return vector3(v.x, v.y, v.z)
		end
	else
		self.convertAxes= function (v)
			return v
		end
	end

	self.bones={}
	self.boneNameToIdx={}
	self.boneInfo={}
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
						table.insert(self.default[c], v2)
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

	self:parseBody(quater(1,0,0,0), 0, handler.root.mujoco.worldbody.body, 1, options)
	self.name=handler.root.mujoco._attr.model

	local mesh
	if handler.root.mujoco.asset then
		mesh=handler.root.mujoco.asset.mesh
	end

	if mesh then
		local fn,folder=os.processFileName(wrlfn)
		for i,v in ipairs(mesh) do
			local id=self.boneNameToIdx[v._attr.name]
			if id then
				local bi=self.boneInfo[id]
				local file=v._attr.file
				local objfile=folder..'/'..file:sub(1,-4)..'obj'
				if os.isFileExist(objfile) then
					local mesh=Mesh()
					mesh:loadOBJ(objfile)

					local newname='_rotated.obj'
					local function mapMesh(mesh, fcn)
						for i=0, mesh:numVertex()-1 do 
							mesh:getVertex(i):assign(fcn(mesh:getVertex(i)))
						end
						for i=0, mesh:numNormal()-1 do 
							mesh:getNormal(i):assign(fcn(mesh:getNormal(i)))
						end
					end
					if true then
						-- obj files are rotated in blender so that -Z forward, Y up
						local M=quater(math.rad(180), vector3(1,0,0))
						--local M=quater(math.rad(90), vector3(0,1,0))* quater(math.rad(90), vector3(1,0,0))* quater(math.rad(180), vector3(0,1,0))
						--local M=quater(math.rad(90), vector3(0,1,0))* quater(math.rad(90), vector3(1,0,0))* quater(math.rad(180), vector3(0,1,0))
						--local M=quater(math.rad(90), vector3(1,0,0))* quater(math.rad(180), vector3(0,1,0))
						local function invRot(v)
							--return M*vector3(v.x, v.y, v.z)
							return M*vector3(v.x, v.z, -v.y)
						end
						mapMesh(mesh, invRot)
					end
					if v._attr.scale then
						local m=matrix4()
						m:identity()
						local s=parseV3(v._attr.scale)
						m:leftMultScaling(s.x, s.y, s.z)
						mesh:transform(m)

						if m:determinant()<0 then
							for i=0, mesh:numFace()-1 do
								local f=mesh:getFace(i)
								local n0=f:normalIndex(0)
								local n1=f:normalIndex(1)
								local n2=f:normalIndex(2)
								local v0=f:vertexIndex(0)
								local v1=f:vertexIndex(1)
								local v2=f:vertexIndex(2)
								f:setIndex(n0, n2, n1, Mesh.NORMAL)
								f:setIndex(v0, v2, v1, Mesh.VERTEX)
							end
							for i=0, mesh:numNormal()-1 do
								mesh:getNormal(i):assign(mesh:getNormal(i)*-1)
							end

						end
						newname='_scaled.obj'
					end
					do 
						local m=matrix4()
						m:identity()
						m:setRotation(bi.q)
						mesh:transform(m)
					end
					local axeC=self.convertAxes
					mapMesh(mesh, axeC)

					mesh:saveOBJ(objfile..newname, true, false)

					self.bones[id].shape={"OBJ_no_classify_tri",file:sub(1,-4)..'obj'..newname, translation=vector3(0,0,0)}
				end
			end
		end
	end

	local axeC=self.convertAxes
	for i,v in ipairs(self.bones) do
		local bi=self.boneInfo[i]

		if bi.fullinertia or bi.diaginertia then
			v.mass=bi.mass
			
			local I=matrix3()
			if bi.fullinertia then
				local ixx, ixy=parseV6(bi.fullinertia)
				I:setValue(ixx.x, ixy.x, ixy.y,
				ixy.x, ixx.y, ixy.z,
				ixy.y, ixy.z, ixx.z)
			else
				local ixx=parseV3(bi.diaginertia)
				I:setValue(ixx.x, 0, 0,
				0, ixx.y, 0,
				0, 0, ixx.z)
			end
			local R=matrix3()
			R:setFromQuaternion(bi.q)
			local Ig=R*I*R:T()

			local diag=axeC(vector3(Ig._11, Ig._22, Ig._33))
			local offdiag=axeC(vector3(Ig._23, Ig._13, Ig._12))
			Ig._11=diag.x Ig._22=diag.y Ig._33=diag.z
			Ig._23=offdiag.x Ig._13=offdiag.y Ig._12=offdiag.z
			Ig._32=offdiag.x Ig._31=offdiag.y Ig._21=offdiag.z
			v.inertia=I
		end
		if v.localCOM then
			v.localCOM:assign(axeC(v.localCOM))
		end
		v.offset:assign(axeC(v.offset))
	end

	self.bones[1].info={
		assetFolder='assets',
	}
	if not options.exportToMemory then
		VRMLexporter.exportWRL(self.bones, wrlfn, self.name)
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
	else
		return self.default.joint._attr.type
	end
end
function MujocoParser:getJointAxisString(jointAxis, p)
	local axis=self:getJointAxis(p)
	if axis=='1 0 0' then
		if self.convertToYUP then
			jointAxis=jointAxis..'Z'
		else
			jointAxis=jointAxis..'X'
		end
	elseif axis=='0 1 0' then
		if self.convertToYUP then
			jointAxis=jointAxis..'X'
		else
			jointAxis=jointAxis..'Y'
		end
	elseif axis=='0 0 1' then
		if self.convertToYUP then
			jointAxis=jointAxis..'Y'
		else
			jointAxis=jointAxis..'Z'
		end
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
		for igeom, tgeom in ipairs(geoms) do
			local geom=tgeom._attr
			if geom.class==nil and geom.type==nil then
				geom.type='sphere'
			end
			if geom.type=='sphere' then
				local s=tonumber(geom.size)
				local mass=tonumber(geom.mass)
				local pos=self.convertAxes(parseV3(geom.pos))
				table.insert(bone.shapes,{'Sphere', translation=pos, size=vector3(s,s,s), mass=mass })
			elseif geom.type=='capsule' then
				local a,b=parseV6(geom.fromto)
				local pos=self.convertAxes(a*0.5+b*0.5)
				local s=tonumber(geom.size)
				local mass=tonumber(geom.mass)
				local q=quater()
				if self.convertToYUP then
					q:axisToAxis(vector3(0,0,1), b-a)
				else
					q:axisToAxis(vector3(0,1,0), b-a)
				end
				self:convertQuatAxes(q)
				table.insert(bone.shapes,{'Capsule', translation=pos, rotation=q, height=a:distance(b), radius=s, mass=mass })
			elseif geom.type=='cylinder' then
				local pos=self.convertAxes(parseV3(geom.pos))
				local radius, height=parseV2(geom.size)
				height=height*2
				local mass=tonumber(geom.mass)
				local q=parseQuat(geom.quat)
				self:convertQuatAxes(q)
				if self.convertToYUP then
					q:rightMult(quater(math.rad(90), vector3(0,1,0)))
				else
					q:rightMult(quater(math.rad(90), vector3(1,0,0)))
				end
				table.insert(bone.shapes,{'Cylinder', translation=pos, rotation=q, height=height, radius=radius, mass=mass })
			elseif geom.type=='box' then
				local s=self.convertAxes(parseV3(geom.size))*2
				local mass=tonumber(geom.mass)
				local pos=self.convertAxes(parseV3(geom.pos))
				local ori=parseQuat(geom.quat)
				self:convertQuatAxes(ori)
				table.insert(bone.shapes,{'Box', translation=pos, rotation=ori, size=s, mass=mass })
			elseif geom.class=='visual' and options.useVisualMesh then
				local meshes=self.root.asset.mesh
				local found=nil
				for i, mesh in ipairs(meshes) do
					if mesh._attr.name==geom.mesh then
						found=mesh
						break
					end
				end
				if found then
					-- not implemented yet
					assert(false)
				else

					local mat
					if self.material then
						mat=self.material[geom.material]
					end
					table.insert(bone.shapes,{'OBJ_no_classify_tri', geom.mesh..'.obj', rotation=q, color=mat, translation=vector3(0,0,0)})
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
				bone.jointAxis=q*self.convertAxes(parseV3(self:getJointAxis(p.joint)))
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
					bone.jointAxis[i]=q*self.convertAxes(parseV3(self:getJointAxis(p.joint[i])))
				end
			end
		end
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
end
