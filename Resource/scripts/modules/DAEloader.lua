
package.path=package.path..";../Samples/sample_luatorch/lua/module/xml2lua/?.lua" --;"..package.path


DAEParser=LUAclass()
local function normalizeXMLnodes(parent, childName)
	local cnodes=parent[childName]
	if cnodes and cnodes._attr then
		parent[childName]={ cnodes}
	end
end

local function parseVec(n_reserve, string)
	local vec=vectorn()
	vec:parseString(n_reserve, string)
	return vec 
end

function loadDAEfile(filename, options)
	local parser=DAEParser(filename, options)
	return parser.mesh, parser.materials
end

function DAEParser:__init(filename, options)
	if options==nil then 
		options={}
	end
	assert(type(options)=='table')
	local xml2lua = require("xml2lua/xml2lua")
	--Uses a handler that converts the XML to a Lua table
	local tree = require("xml2lua/xmlhandler.tree")
	local handler=tree:new()
	local xml = util.readFile(filename)
	assert(xml)


	if false then
		local startIndex=1
		local c=1
		while true do
			local a, b=xml:sub(startIndex):find('effect%s+id="')
			if not a then break end
			a=a+startIndex-1
			b=b+startIndex-1
			xml=xml:sub(1,a-1)..'effect'..tostring(c)..' id="'..xml:sub(b+1)
			startIndex=b
			a,b=xml:sub(startIndex):find('%/effect')
			assert(a)
			a=a+startIndex-1
			b=b+startIndex-1
			xml=xml:sub(1,a-1)..'/effect'..tostring(c)..xml:sub(b+1)
			startIndex=b

			c=c+1
		end
	end
	
	--Instantiates the XML parser
	local parser = xml2lua.parser(handler)
	parser:parse(xml)
	self.root=handler.root.COLLADA
	normalizeXMLnodes(self.root.library_materials, 'material')

	local materialMap={}
	for i, material in ipairs(self.root.library_materials.material) do
		materialMap[material.instance_effect._attr.url]=material
	end
	self.geometries=self.root.library_geometries.geometry
	assert(self.geometries._attr)

	local source={}
	for i, v in ipairs(self.geometries.mesh.source) do
		source[v._attr.id]=v
	end

	self.mesh=Geometry()
	local vertices, verticesName, triangles
	do
		local input=self.geometries.mesh.vertices.input
		-- parse vertices
		local inputType=input._attr.semantic
		local sourceName=input._attr.source:sub(2)
		local textvertices=source[sourceName]

		vertices=vectorn()
		vertices:parseString(textvertices.technique_common.accessor._attr.count*3, textvertices.float_array[1])
		verticesName=sourceName
	end
	self.mesh:_addVertices(vertices)
	local tris=self.geometries.mesh.triangles
	if #tris==0 then
		tris={ tris}
	end
	for i, tri in ipairs(tris) do
		local inputs=tri.input
		local VERTEX_OFFSET
		local NORMAL_OFFSET
		local TEXCOORD_OFFSET
		local normals,texCoords
		local all_indices=intvectorn()
		all_indices:parseString( tri._attr.count*(3+3+3), tri.p)
		local count=0
		for iinput, input in ipairs(inputs) do
			if input._attr.semantic=="VERTEX" then

				--assert(input._attr.source:sub(2)==verticesName)
				VERTEX_OFFSET=tonumber(input._attr.offset)
				count=count+3
			elseif input._attr.semantic=="NORMAL" then
				local sourceName=input._attr.source:sub(2)
				local textnormals=source[sourceName]
				normals=vectorn()
				normals:parseString(textnormals.technique_common.accessor._attr.count*3, textnormals.float_array[1])
				assert(normals:size()==textnormals.technique_common.accessor._attr.count*3)
				NORMAL_OFFSET=tonumber(input._attr.offset)
				count=count+3
			elseif input._attr.semantic=="TEXCOORD" then
				local sourceName=input._attr.source:sub(2)
				local textUVs=source[sourceName]
				texCoords=vectorn()
				texCoords:parseString(textUVs.technique_common.accessor._attr.count*2, textUVs.float_array[1])
				assert(texCoords:size()==textUVs.technique_common.accessor._attr.count*2)
				TEXCOORD_OFFSET=tonumber(input._attr.offset)
				count=count+3
			else
				dbg.console()
			end
		end
		assert(all_indices:size()==tri._attr.count*count)
		local vstart=0 -- shared vertex buffer
		local nstart=self.mesh:numNormal()
		local uvstart=self.mesh:numTexCoord()
		self.mesh:_addNormals(normals)
		if texCoords then
			self.mesh:_addTexCoords(texCoords)
			self.mesh:_addSubMesh(vstart, nstart, uvstart, VERTEX_OFFSET, NORMAL_OFFSET, TEXCOORD_OFFSET, all_indices)
		else
			self.mesh:_addSubMeshPosNormal(vstart, nstart, VERTEX_OFFSET, NORMAL_OFFSET, all_indices)
		end
		if tri._attr.material then
			self.mesh:element(self.mesh:numElements()-1):setMaterial(tri._attr.material)
		end
	end

	normalizeXMLnodes(self.root.library_effects, 'effect')
	self.effects=self.root.library_effects.effect

	self.materials={}

	for i, effect in ipairs(self.effects) do
		local id=effect._attr.id
		local lambert=effect.profile_COMMON.technique.lambert
		if lambert then
			local diffuse=vector3(1,1,1)
			local emission=vector3(1,1,1)
			local specular=vector3(1,1,1)

			if lambert.diffuse then
				diffuse=parseVec(4, lambert.diffuse.color[1]):toVector3(0)
			end
			if lambert.emission then
				emission=parseVec(4, lambert.emission.color[1]):toVector3(0)
			end
			if lambert.specular then
				specular=parseVec(4, lambert.specular.color[1]):toVector3(0)
			end

			if not RE.rendererValid or not RE.rendererValid() then

			else
				local mat_id=materialMap['#'..id]._attr.id
				print('creating material "'..mat_id..'"')
				RE.renderer():createMaterial(mat_id, diffuse, specular, 10)
				table.insert(self.materials, {mat_id, diffuse, specular, emission})
			end


			--local index_of_refraction=tonumber(lambert.index_of_refraction.float)
		end

	end
	local root_node=self.root.library_visual_scenes.visual_scene.node
	local tf=root_node.matrix
	local tf_vec=vectorn()
	tf_vec:parseString(16, tf[1])
	tf=matrix4()
	tf:identity()
	tf._11=tf_vec(0)
	tf._12=tf_vec(1)
	tf._13=tf_vec(2)
	tf._14=tf_vec(3)
	tf._21=tf_vec(4)
	tf._22=tf_vec(5)
	tf._23=tf_vec(6)
	tf._24=tf_vec(7)
	tf._31=tf_vec(8)
	tf._32=tf_vec(9)
	tf._33=tf_vec(10)
	tf._34=tf_vec(11)

	self.mesh:scaleAndRigidTransform(tf)
	assert(self.mesh:numVertex()>0)
	assert(self.mesh:numFace()>0)
end
