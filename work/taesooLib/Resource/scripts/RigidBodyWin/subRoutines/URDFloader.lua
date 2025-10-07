require('tl') -- for BaseLib <-> torch interfacing


package.path=package.path..";../Samples/sample_luatorch/lua/module/xml2lua/?.lua" --;"..package.path


package.path=package.path..";../../taesooLib/Samples/scripts/?.lua" --;"..package.path
package.path=package.path..";../../taesooLib/Samples/classification/lua/?.lua" --;"..package.path
require('RigidBodyWin/subRoutines/VRMLexporter')

function URDFloader(fn, options)
	if fn:sub(-5)=='.urdf' then
		local wrlfile=fn..'.wrl'
		---- set exportToMemory to false to actually write to the wrl file.
		local parser=URDFparser(fn, wrlfile, options or { useVisualMesh=true, exportToMemory=true} )

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
function parseRPY(t)
	-- roll pitch yaw
	local v=parseV3(t)
	local q=quater()
	--q:setRotation("XYZ", v)
	q:setRotation("ZYX", vector3(v.z, v.y, v.x))  -- RZ*RY*RX
	return q
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

URDFparser=LUAclass()
function URDFparser:__init(filename, wrlfn, options)
	if options==nil then 
		options={}
	end
	options.url=filename
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

	self.root=handler.root.robot
	self.bones={}
	self.boneNameToIdx={}
	self.links={}
	self.jointMap={}
	for i, link in ipairs(self.root.link) do
		self.links[i]=link
		self.boneNameToIdx[link._attr.name]=i
		link.idx=i
		link.joints={}
		link.children={}
	end
	for i, joint in ipairs(self.root.joint) do
		joint.idx=i
		self.jointMap[joint._attr.name]=joint
		local link= self.links[self.boneNameToIdx[joint.parent._attr.link]]
		local childlink= self.links[self.boneNameToIdx[joint.child._attr.link]]
		assert(link)
		assert(childlink)
		table.insert(childlink.joints, joint)
		table.insert(link.children, childlink)
		assert(childlink.idx>link.idx) -- taesooLib assumes this. otherwise, reordering is necessary.
	end

	self.boneInfo={}
	self.default=self.root.default
	if not self.default then
		self.default={}
	end

	self:parseLink(quater(1,0,0,0), 0, self.root.link[1], 1, options)
	self.name=self.root._attr.name

	for i,v in ipairs(self.bones) do
		local bi=self.boneInfo[i]

		v.mass=bi.mass or 1e-3
		if bi.inertia then
			
			local I=matrix3()
			local ixx=vector3()
			local ixy=vector3()
			ixx.x=tonumber(bi.inertia.ixx)
			ixx.y=tonumber(bi.inertia.iyy)
			ixx.z=tonumber(bi.inertia.izz)
			ixy.x=tonumber(bi.inertia.ixy)
			ixy.y=tonumber(bi.inertia.ixz)
			ixy.z=tonumber(bi.inertia.iyz)
			I:setValue(ixx.x, ixy.x, ixy.y,
			ixy.x, ixx.y, ixy.z,
			ixy.y, ixy.z, ixx.z)

			local R=matrix3()
			R:setFromQuaternion(bi.q)
			local Ig=R*I*R:T()

			local diag=vector3(Ig._11, Ig._22, Ig._33)
			local offdiag=vector3(Ig._23, Ig._13, Ig._12)
			Ig._11=diag.x Ig._22=diag.y Ig._33=diag.z
			Ig._23=offdiag.x Ig._13=offdiag.y Ig._12=offdiag.z
			Ig._32=offdiag.x Ig._31=offdiag.y Ig._21=offdiag.z
			v.inertia=I
		end
		if v.localCOM then
			v.localCOM:assign(v.localCOM)
		end
		v.offset:assign(v.offset)
	end

	self.bones[1].info={
		assetFolder='.',
	}
	if not options.exportToMemory then
		VRMLexporter.exportWRL(self.bones, wrlfn, self.name)
	end
end
function URDFparser:getJointAxis(p)
	if p.axis._attr.xyz then
		return p.axis._attr.xyz
	elseif p._attr.class then
		local classInfo=self.default[p._attr.class]
		assert(#classInfo==1)
		return classInfo[1].joint._attr.axis
	else
		return self.default.joint._attr.axis
	end
end
function URDFparser:getJointType(p)
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
function URDFparser:getJointAxisString(jointAxis, p)
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
function URDFparser.parseOrigin(tgeom)
	local pos, ori
	if tgeom.origin then
		pos=parseV3(tgeom.origin._attr.xyz)
		if tgeom.origin._attr.quat then
			dbg.console()
		else
			ori=parseRPY(tgeom.origin._attr.rpy)
		end
	else
		pos=vector3(0,0,0)
		ori=quater(1,0,0,0)
	end
	return pos, ori
end
function URDFparser:parseLink(q_parent, pid, p, level, options)
	--if level>2 then return end
	local bones=self.bones
	local boneNameToIdx=self.boneNameToIdx
	local name=p._attr.name
	local offset, q_l
	if p.idx==1 then
		assert(#p.joints==0)
		offset=vector3(0,0,0)
		q_l=quater(1, 0,0,0)
	else
		assert(#p.joints==1)
		local j=p.joints[1]
		local origin_pos, origin_ori=URDFparser.parseOrigin(j)
		offset=origin_pos
		q_l=origin_ori
	end

	local q=q_parent:copy()
	q=q*q_l
	local bone
	if p.inertial then
		bone={id=#bones+1, name=name, offset=q_parent*offset, localCOM=q_parent*parseV3(p.inertial.origin._attr.xyz)}
		assert(bone.id==p.idx)
		if pid~=0 then
			bone.pid=pid
		end
		table.insert(bones, bone)
		-- todo: rotate inertia too.
		table.insert(self.boneInfo, 
		{
			q_parent=q_parent:copy(),
			q=q:copy(),
			q_local=q_l:copy(),
			mass=tonumber(p.inertial.mass._attr.value),
			inertia=p.inertial.inertia._attr,
		}
		)
	else
		bone={id=#bones+1, name=name, offset=q_parent*offset, } 
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

	local geoms=p.collision
	if options.useVisualMesh then
		geoms=p.visual
	end

	if geoms then
		if #geoms==0 then
			geoms={geoms}
		end
		assert(#geoms>0)

		bone.shapes={}
		bone.classes={}
		local parseOrigin=URDFparser.parseOrigin
		for igeom, tgeom in ipairs(geoms) do
			if tgeom.geometry.sphere then
				local geom=tgeom.geometry.sphere
				local s=tonumber(geom._attr.radius)
				local pos, ori=parseOrigin(tgeom)
				table.insert(bone.shapes,{'Sphere', translation=pos, rotation=q*ori, size=vector3(s,s,s), mass=mass })
			elseif tgeom.geometry.capsule then
				local a,b=parseV6(geom.fromto)
				local pos=a*0.5+b*0.5
				local s=tonumber(geom.size)
				local mass=tonumber(geom.mass)
				local qq=quater()
				qq:axisToAxis(vector3(0,1,0), b-a)
				table.insert(bone.shapes,{'Capsule', translation=pos, rotation=q*qq, height=a:distance(b), radius=s, mass=mass })
			elseif tgeom.geometry.cylinder then
				local geom=tgeom.geometry.cylinder
				local radius=geom._attr.radius
				local height=geom._attr.length
				local pos, ori=parseOrigin(tgeom)
				
				ori:rightMult(quater(math.rad(90), vector3(1,0,0)))

				table.insert(bone.shapes,{'Cylinder', translation=pos, rotation=q*ori, height=height, radius=radius, mass=mass })
			elseif tgeom.geometry.box then
				local geom=tgeom.geometry.box
				local s=parseV3(geom._attr.size)
				local pos, ori=parseOrigin(tgeom)
				table.insert(bone.shapes,{'Box', translation=pos, rotation=q*ori, size=s, mass=mass })
			elseif tgeom.geometry.mesh then
				local geom=tgeom.geometry.mesh
				local pos, ori=parseOrigin(tgeom)

				local mat
				if geom.material and geom.material.color then
					mat=geom.material.color._attr.rgba
					mat=parseColor(mat)
				end
				local fn=geom._attr.filename
				local lfn, _path=os.processFileName(fn)
				local _, urdf_path=os.processFileName(options.url)
				local path=Path(urdf_path):join(fn).string -- dae_path

				if fn:sub(-4):lower()=='.dae' and not os.isFileExist(path..'.cache') then
				--if fn:sub(-4):lower()=='.dae' then -- to always reload DAE
					require('DAEloader')
					local cachefile=path..'.cache'
					print('generating '..cachefile)
					local mesh, materials=loadDAEfile(path)
					local file=util.BinaryFile(true, cachefile)
					local version=2
					file:packInt(version) -- shape cache version
					mesh:pack(file)
					file:packInt(#materials)
					for i, material in ipairs(materials) do
						file:pack(material[1])
						file:pack(material[2])
						file:pack(material[3])
						file:pack(material[4])
						file:packInt(0) -- material version
					end
					file:close()
				elseif false and fn:sub(-4):lower()=='.dae' then
					-- load cached geometry (much faster)
					local file=util.BinaryFile()
					file:openRead(path..'.cache')
					local version= file:unpackInt()
					local mesh=Geometry()
					mesh:unpack(file)
					file:close()
				end

				table.insert(bone.shapes,{'OBJ_no_classify_tri', fn, rotation=q*ori, color=mat, translation=q*pos})
			elseif geom.class then
				-- unused yet.
				table.insert(bone.classes, geom)
			else
				assert(false)
			end
		end
	end

	if #p.joints==0 then
		bone.jointType=options.rootJointType or 'free'
	elseif #p.joints==1 and p.joints[1]._attr.type=='fixed' then
		bone.jointType='fixed'
	else
		local jointAxis=''
		if math.abs(q.w-1)<1e-4 then
			-- case: identity q 
			for i=1, #p.joints do
				jointAxis=self:getJointAxisString(jointAxis, p.joints[i])
				if not jointAxis then break end
			end
		else
			jointAxis=nil
		end
		if p.joints[1]._attr.type=='revolute' then
			bone.jointType='rotate'
		else
			bone.jointType='slide'
		end
		if jointAxis then
			bone.jointAxis=jointAxis
		else
			bone.jointAxis={}
			for i=1, #p.joints do
				bone.jointAxis[i]=q*parseV3(self:getJointAxis(p.joints[i]))
			end
		end
	end

	boneNameToIdx[name]=#bones
	local id=#bones
	if p.children then
		for i, v in ipairs(p.children) do
			self:parseLink(q, id, v, level+1, options)
		end
	end
end
