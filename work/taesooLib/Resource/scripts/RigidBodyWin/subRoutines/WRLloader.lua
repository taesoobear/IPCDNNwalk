
require("subRoutines/VRMLexporter")
_WRLLoader={}
_WRLLoader.debugMode=false
function _WRLLoader.simplifyTable(t)

	table.insert(_WRLLoader.bones, t)
	t.id=#_WRLLoader.bones

	if t.segment then
		if v.geometry then
		end
	elseif t.children then
		for k, v in ipairs(t.children) do
			v.pid=t.id
			_WRLLoader.simplifyTable(v)
		end
		if #t.children then
			t.children=nil
		end
	end
end
function loadLuaFile(filename)
	local func,msg =loadfile(filename)
	local t
	if msg then
		print(msg)
		dbg.console()
	end
	t, msg=pcall(func)
	if not t then
		print(msg)
		dbg.console()
	end
	t=msg
	return t
end

-- input: filename or a lua table. the input lua table will be modified in place so use MainLib.WRLloader(deepCopyTable(input)) if input would be necessary again.
function MainLib.WRLloader(input)
	local t, url
	if type(input)=='string' then
		local filename=input
		if filename:sub(-4)==".wrl" then
			return MainLib.VRMLloader(filename)
		elseif filename:sub(-8)==".wrl.dat" then
			local loader=MainLib.VRMLloader()
			local file=util.BinaryFile()
			file:openRead(filename)
			if not file:readable() then
				error('failed to open' ..filename)
			end
			file:_unpackVRMLloader(loader)
			local pose=vectorn()
			file:unpack(pose)
			loader:setURL(filename)
			loader:setPoseDOF(pose)
			local info={}
			local markers={}
			while true do
				local code=file:unpackAny()
				if not code then break end

				if code=='markers' then
					print('unpack markers')
					for i=1, loader:numBone()-1 do
						markers[i]={}
						local nmarker=file:unpackInt()
						for j=1, nmarker do
							local v=vector3()
							file:unpack(v)
							markers[i][j]=v
						end
					end
					info.markers=markers
				elseif code=='constraints' then
					print('unpack constraints')
					local bone=intvectorn()
					local localpos=vector3N()
					file:unpack(bone)
					file:unpack(localpos)
					for i=0, localpos:size()-1, 2 do
						loader:addRelativeConstraint(bone(i), localpos(i),
						bone(i+1), localpos(i+1))
					end
				elseif code=='pose' then
					local pose=loader:pose()
					file:unpack(pose.rotations)
					file:unpack(pose.translations)
					loader:setPose(pose)
				end
			end
			file:close()
			return loader, info
		end
		-- returns an instance of MainLib.VRMLloader
		assert(filename:sub(-8)==".wrl.lua")

		t=loadLuaFile(filename)
		url=filename:sub(1,-9)
		t.url=filename
	else
		t=input
	end
	assert(t.body)

	_WRLLoader.bones={}

	local rootpos=vector3(0,0,0)
	if t.body.jointType=='rotate'  or t.body.jointType=='slide' then
		rootpos=t.body.translation:copy()
		t.body.translation:zero()
		t.body={
			name=t.body.name..'_attachpoint',
			jointType='fixed',
			translation=vector3(0,0,0),
			children={t.body},
		}
	elseif t.body.jointType=='free' and t.body.translation:length()>1e-3 then
		print('initial position should be zero for a free body!!!')
		t.body.translation=vector3(0,0,0)
	end
	_WRLLoader.simplifyTable(t.body)

	local _bones=_WRLLoader.bones 

	local bones={} -- VRMLexporter format.
	local bid=0
	for i, bone in ipairs(_bones) do

		local boneInfo={}
		boneInfo.id=bone.id
		boneInfo.pid=bone.pid 
		if bone.name then
			boneInfo.name=bone.name 
		else
			boneInfo.name='bone_'..bid
			bid=bid+1
		end

		assert(bone.translation)
		boneInfo.offset=bone.translation

		if bone.geometry then
			local function calcMass(geom)
				if geom.mass then
					return geom.mass
				else
					if geom[1]=='Box' then
						local density=997 -- water density
						return density*geom.size.x*geom.size.y*geom.size.z
					elseif geom[1]=='Ellipsoid' then
						local density=997 -- water density
						return density*4.0*3.14/3.0*geom.size.x*geom.size.y*geom.size.z
					elseif geom[1]=='Sphere' then
						local density=997 -- water density
						return density*4.0*3.14/3.0*geom.size.x*geom.size.y*geom.size.z
					elseif geom[1]=='Cylinder' or geom[1]=='Capsule' then
						local density=997 -- water density
						return density*3.14*geom.radius*geom.radius*geom.height
					elseif geom[1]=='OBJ' then
						geom.inertia=vector3(0.004, 0.004, 0.004)
						return 10 -- recalculate later using loader:setTotalMass function!
					else
						-- not implmented yet!
						assert(false)
					end
				end
				return 0
			end
			for igeom, geom in ipairs(bone.geometry) do
				if not geom.mass then
					geom.mass=calcMass(geom)
				end
			end
		end
		boneInfo.shapes=bone.geometry
		boneInfo.jointType=bone.jointType
		boneInfo.jointAxis=bone.jointAxis
		table.insert(bones, boneInfo)
	end

	if _WRLLoader.debugMode then
		VRMLexporter.exportWRL(deepCopyTable(bones), '___temp_WRLLUA.wrl', t.name or 'robot')
	end
	--local loader= MainLib.VRMLloader('___temp_WRLLUA.wrl')
	
	local out=VRMLexporter.generateWRLstring(bones, t.name or 'robot', url)
	local file=CTextFile()
	file:OpenMemory(out)
	local loader=MainLib.VRMLloader(file)
	if t.url then
		loader:setURL(t.url)
	end

	if t.body.jointType=='fixed' and rootpos:length()>0.00001 then
		loader:setPosition(rootpos)
	end
	if t.connections then
		for i, con in ipairs(t.connections) do
			local b1=loader:getBoneByName(con[1])
			local b2=loader:getBoneByName(con[3])

			loader:addRelativeConstraint(
			b1:treeIndex(), con[2],
			b2:treeIndex(), con[4])
		end
	end

	return loader, t
end

-- input: motion filename or a lua table.
-- output: an instance of MotionDOFcontainer
function MainLib.MotionContainer(loader, input)
	if input==nil or input=='' then
		local container =MotionDOFcontainer(loader.dofInfo)
		container:resize(100)
		container.mot:matView():setAllValue(0)
		container.mot:matView():column(1):setAllValue(1)
		container.mot:matView():column(3):setAllValue(1)
		return container
	else
		assert(type(input)=='string')
		local ext=string.upper(string.sub(input,-4))
		if ext=='.DOF' then
			return MotionDOFcontainer(loader.dofInfo, input)
		elseif ext=='.LUA' then
			local t=matrixn.fromTable(loadLuaFile(input))
			local out=MotionDOFcontainer(loader.dofInfo)
			out:resize(t:rows())
			out.mot:matView():assign(t)
			return out
		else
			require('moduleIK')
			local out=loadMotion(loader, input)
			return out.motionDOFcontainer
		end
	end
end



-- input: a MotionLoader (bvhloader, asfloader, and so on)
-- output: a table (that can be stored as a .wrl.lua file using 
-- util.writeFile('a.wrl.lua', 'return '..table.toHumanReadableString(wrlTable))
function MotionUtil.generateWRLforRobotSimulation(loader, robotname, cylinder_radius, url)

	local b=loader:bone(1)
	local function packTransform(bones, loader, bone, cylinder_radius)
		local binfo={
			name=bone:name(),
			translation=bone:getOffset(),
		}
		table.insert(bones, binfo)
		local tc=bone:getTranslationalChannels()
		local rc=bone:getRotationalChannels()

		if tc and rc then
			assert(tc:len()==3)
			assert(rc:len()==3)
			binfo.jointType='free'
		elseif tc then
			binfo.jointType='slide'
			binfo.jointAxis=tc
		elseif rc then
			binfo.jointType='rotate'
			binfo.jointAxis=rc
		else
			binfo.jointType='fixed'
			return nil
		end

		binfo.children={}

		local function packShape(loader, bone, binfo, cylinderRadius)
			local geom={}
			local c=bone:childHead()
			while c do
				local offset=c:getOffset()
				local q=quater()
				local axis=vector3()
				q:axisToAxis(vector3(0,1,0), offset:Normalize())
				table.insert(geom, 
				{
					-- capsule is slow to render
					--'Capsule', 
					--rotation=q,
					--translation=offset/2,
					--radius=cylinderRadius, 
					--height=math.max(offset:length(), 1e-2),

					'Box',
					rotation=q,
					translation=offset/2,
					size=vector3(cylinderRadius,math.max(offset:length(), 1e-2),cylinderRadius),
				}
				)
				c=c:sibling()
			end
			if #geom>0 then
				binfo.geometry=geom
			end
		end
		packShape(loader, bone, binfo, cylinder_radius)

		if bone:childHead() then
			local node=bone:childHead()
			while node do
				local c=packTransform(bones, loader, node, cylinder_radius)
				if c then
					table.insert(binfo.children, c)
				end
				node=node:sibling()
			end
		end
		if #binfo.children ==0 then
			binfo.children=nil
		end
		return binfo
	end
	local bones={}
	local binfo=packTransform(bones, loader, b, cylinder_radius)
	return {
		name=robotname,
		body=binfo,
		url=url,
	}, bones
end

function rotateGeom(q, geom)
	local out=deepCopyTable(geom)
	for i, geom in ipairs(out) do
		if geom.rotation then
			geom.rotation=q*geom.rotation
		else
			geom.rotation=q
		end
		geom.translation=q*geom.translation
	end
	return out
end

-- convert to a table. table[3] can be used as an input to WRLloader.
function MainLib.VRMLloader:toTable()
	local tbl={ name=self:name()}
	local bones={}
	for i=1, self:numBone()-1 do
		local bone={}
		local b=self:VRMLbone(i)
		bone.name=b:name()
		bone.jointType=b:jointType()
		bone.jointAxis=b:jointAxis()
		bone.translation=b:getOffset()
		bone.mass=b:mass()
		assert(b:hasShape())
		if b:hasShape() then
			bone.geometry=b:getMesh():toTable()[3]
		end
		if b:numChildren()>0 then
			bone.children={}
		end
		bones[i]=bone
	end
	tbl.body= bones[1]
	for i=2, self:numBone()-1 do
		local b=self:VRMLbone(i)
		local bone=bones[i]
		table.insert(bones[b:parent():treeIndex()].children, bone)
	end
	return {"__userdata", "MainLib.VRMLloader", tbl}
end
function MainLib.VRMLloader:exportLUA(filename)
	util.writeFile(filename, 'return '..table.toHumanReadableString(self:toTable()[3]))
end
function MainLib.VRMLloader:printLUA()
	print('return '..table.toHumanReadableString(self:toTable()[3]))
end

function MainLib.VRMLloader:toCollisionLoader(totalMass, nConvex)
	require("Kinematics/meshTools")
	nConvex = nConvex or 6

	local fbx_info={}
	do 
		-- create fbxinfo by merging submeshes into one
		local tot_mesh=0
		local meshes={}
		for i=1, self:numBone()-1 do
			if self:VRMLbone(i):hasShape() then
				tot_mesh=tot_mesh+1
			end
		end
		
		local mm=util.MeshMerger(tot_mesh)
		tot_mesh=0
		local imesh_to_ibone={}

		self:updateInitialBone()
		for i=1, self:numBone()-1 do
			if self:VRMLbone(i):hasShape() then
				meshes[i]=self:VRMLbone(i):getMesh():copy()
				meshes[i]:transform(matrix4(self:VRMLbone(i):getFrame()))
				mm:setInputMesh(tot_mesh, meshes[i])
				imesh_to_ibone[tot_mesh]=i
				tot_mesh=tot_mesh+1
			end
		end
		local mesh=Mesh()
		local skin=SkinnedMeshFromVertexInfo()
		if tot_mesh==1 and self:VRMLbone(1):hasShape() then
			mesh=meshes[1]
			skin:resize(mesh:numVertex())
			for i=0, mesh:numVertex()-1 do
				skin:treeIndices(i):assign(CT.ivec(1))
				skin:weights(i):assign(CT.vec(1.0))
			end
		else
			mm:mergeMeshes(mesh)
			skin:resize(mesh:numVertex())
			for i=0, mesh:numVertex()-1 do
				local imesh=mm:get_imesh(i)
				local ivert=mm:get_ivert(i)
				local ibone=imesh_to_ibone[imesh]
				skin:treeIndices(i):assign(CT.ivec(ibone))
				skin:weights(i):assign(CT.vec(1.0))
				--skin:localPos(i):setSize(1)
				--skin:localPos(i)(0):assign(mesh:getVertex(i))
			end
		end
		--skin:calcVertexPositions(self, mesh) 
		skin:calcLocalVertexPositions(self, mesh)

		local skinScale=100
		fbx_info.loader=self
		fbx_info.mesh=mesh
		fbx_info.skinningInfo=skin -- share
		fbx_info.bindPose=Pose()
		self:getPose(fbx_info.bindPose)
		local bindposes={}
		--assert(self.loader:numBone()==fbx_info.loader:numBone()) -- can be different
		local loader=fbx_info.loader
		for i=1, loader:numBone()-1 do
			bindposes[i]= matrix4(loader:bone(i):getFrame():inverse())
		end
		fbx_info.bindposes=bindposes
	end

	local temp=SkinToWRL(fbx_info, { maxConvex= nConvex })
	temp.skel:setTotalMass(totalMass)
	return temp.skel
end
