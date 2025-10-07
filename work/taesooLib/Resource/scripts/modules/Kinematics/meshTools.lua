Circle=LUAclass()

function Circle:__init(center, R)
	self.center=center
	self.R=R
end
function Circle:contains(v2, margin)
	local x=self.center.x
	local z=self.center.z
	local R=self.R
	local distz=(v2(1)-z)
	local distx=(v2(0)-x)
	if math.sqrt(distx*distx+distz*distz)<R+margin then
		return true
	end
	return false
end

function drawCircles(circles, name, y)

	y= y or 0
	name=name or 'circles'
	local lines=vector3N()

	local ndiv=10
	local vertices=vector3N(ndiv)
	for i, b in ipairs(circles) do

		local front=vector3(0,0,1)
		local center=b.center
		local radius=b.R
		for j=0, ndiv-1 do
			local q=quater()
			q:setRotation(vector3(0,1,0), sop.map(j, 0, ndiv, 0, math.pi*2.0));
			local nf=q*front
			vertices(j):assign(nf*radius+center)
		end
		for j=0, ndiv-1 do
			local v0=j;
			local v2=math.fmod(j+1,ndiv);

			lines:pushBack(vertices(v0))
			lines:pushBack(vertices(v2))
		end
	end

	dbg.draw('Traj', lines:matView(), name, 'solidgreen', 10, 'BillboardLineList')
end

function Mesh:checkValidity()
	local nn=self:numNormal()
	if nn>0 then
		for f=0, self:numFace()-1 do
			local face=self:getFace(f) 
			for i=0,2 do
				local ni=face:normalIndex(i)
				return 'normal error'
			end
		end
	end
end
function Mesh:createColorBuffer()
	local mesh=self
	-- low-level API
	mesh:resizeBuffer(Mesh.COLOR, mesh:numVertex())
	for i=0, mesh:numFace()-1 do
		for j=0,2 do
			mesh:getFace(i):setIndex1(j, mesh:getFace(i):vertexIndex(j), Mesh.COLOR)
		end
	end
	for i=0, mesh:numVertex()-1 do
		mesh:getColor(i):set(1,1,1,0)
	end
end
function Mesh:createTextureBuffer()
	local mesh=self
	-- low-level API
	mesh:resizeBuffer(Mesh.TEXCOORD, mesh:numVertex())
	for i=0, mesh:numFace()-1 do
		for j=0,2 do
			mesh:getFace(i):setIndex1(j, mesh:getFace(i):vertexIndex(j), Mesh.TEXCOORD)
		end
	end
	for i=0, mesh:numVertex()-1 do
		mesh:getTexCoord(i):set(0,0)
	end
end

-- returns meshToEntity,entity
function Mesh:createEntity(_optionalNodeName, _optionalDoNotUseNormal)
	_optionalNodeName=_optionalNodeName or 'node_name'
	local mesh=self

	local useTexCoord=false
	local useColor=false

	if mesh:numNormal()==0 then
		_optionalDoNotUseNormal=true
	end
	if mesh:numTexCoord()>0 then
		useTexCoord=true
	end
	if mesh:numColor()>0 then
		useColor=true
	end
	-- scale 100 for rendering 
	local meshToEntity=MeshToEntity(mesh, 'meshName'.._optionalNodeName, false, true, not _optionalDoNotUseNormal, useTexCoord, useColor)
	--meshToEntity:updatePositionsAndNormals()
	local entity=meshToEntity:createEntity('entityName'.._optionalNodeName )
	return meshToEntity, entity
end
-- returns meshToEntity,node
function Mesh:drawMesh(_optionalMaterialName, _optionalNodeName, _optionalDoNotUseNormal)
	_optionalNodeName=_optionalNodeName or 'node_name'
	local meshToEntity, entity=self:createEntity( _optionalNodeName, _optionalDoNotUseNormal)

	if entity then
		materialName=_optionalMaterialName or 'lightgrey_transparent'
		if materialName then
			entity:setMaterialName(materialName)
		else
			entity:setMaterialName("CrowdEdit/Terrain1")
		end
		--entity:setMaterialName("red")
		--entity:setMaterialName("checkboard/crowdEditink")
		local node=RE.createChildSceneNode(RE.ogreRootSceneNode(), _optionalNodeName)
		node:attachObject(entity)
		--node:translate(-s*50,-0.1,-s*50)
		return meshToEntity,node
	end
	return nil
end

function Mesh:translateVertices(indices, t)
	for i=0, indices:size()-1 do
		self:getVertex(indices(i)):radd(t)
	end
end

function Mesh:verticallyExtrudeBoundaryEdges(new_y)
	local mesh=self
	local g_ec=EdgeConnectivity(mesh)

	local nv=mesh:numVertex()
	local edges=intmatrixn()
	local vertices=boolN()
	vertices:resize(mesh:numVertex())
	vertices:setAllValue(false)
	for e=0, g_ec:numEdges()-1 do
		if g_ec:getNumFaceEdge(e)==1 then
			local fe=g_ec:getFaceEdge(e, 0)
			local s=fe:source(mesh)
			local t=fe:target(mesh)
			edges:pushBack(CT.ivec(s,t))
			vertices:set(s, true)
			vertices:set(t, true)
			--dbg.draw('Arrow', mesh:getVertex(s)+vector3(0,20,0), mesh:getVertex(t)+vector3(0,20,0), 'a'..edges:rows())
		end
	end

	-- extrude the edges
	local nNewVertices=vertices:count()
	local origvertIndices=intvectorn(nNewVertices)
	local correspondingVert=intvectorn(mesh:numVertex())
	local newVertices=vector3N(nNewVertices)
	local c=0
	for i=0, vertices:size()-1 do
		if vertices(i) then
			origvertIndices:set(c, i)
			correspondingVert:set(i, c)
			local v=mesh:getVertex(i):copy()
			v.y=new_y
			newVertices(c):assign(v)
			c=c+1
		end
	end
	local vioffset=mesh:numVertex()
	mesh:addVertices(newVertices)

	local faces=intmatrixn()
	faces:resize(edges:rows()*2, 3)
	for i=0, edges:rows()-1 do
		local s=edges(i,0)
		local t=edges(i,1)
		local cs=correspondingVert(s)+vioffset
		local ct=correspondingVert(t)+vioffset
		faces:row(i*2):assign(CT.ivec(t, s, cs))
		faces:row(i*2+1):assign(CT.ivec(t, cs, ct))
	end
	mesh:addFaces(faces)
end
function Mesh:selectVerticesIn(boxes, margin)
	margin=margin or 0
	local selectedVertices=boolN()
	selectedVertices:resize(self:numVertex())
	for i, b in ipairs(boxes) do
		for j=0, self:numVertex()-1 do
			local v=vector2()
			v:assignXZ(self:getVertex(j))
			if b:contains(v,margin) then
				selectedVertices:set(j, true)
				--dbg.draw('Sphere', self:getVertex(j), 'v'..j)
			end
		end
	end
	return selectedVertices
end
function Mesh:selectBoundaryEdges(boxes)
	local mesh=self
	local g_ec=EdgeConnectivity(mesh)
	local vertices=self:selectVerticesIn(boxes)
	local selectedEdges=boolN()
	g_ec:selectBoundaryEdgesFromVertices(vertices, selectedEdges);
	return {g_ec, selectedEdges}
end
function Mesh:selectAllEdges()
	local mesh=self
	local g_ec=EdgeConnectivity(mesh)
	local selectedEdges=boolN()
	selectedEdges:resize(g_ec:numEdges())
	selectedEdges:setAllValue(true)
	return {g_ec, selectedEdges}
end

function Mesh:removeFacesOverlapping(boxes, margin)
	local mesh=self
	local selectedVertices=self:selectVerticesIn(boxes, margin) 
	local faces=boolN()
	local ec=EdgeConnectivity(mesh)
	local mc=OBJloader.MeshConnectivity(mesh, ec)
	mc:selectFacesFromVertices(selectedVertices, faces)


	self:removeFaces(faces:findAll(true))
end

function Mesh:selectFacesFromVertices(selectedVertices, faces)
	faces:resize(self:numFace())
	faces:setAllValue(false)
	for i=0, self:numFace()-1 do
		local f= self:getFace(i)
		if selectedVertices(f:vertexIndex(0)) or
			selectedVertices(f:vertexIndex(1)) or
			selectedVertices(f:vertexIndex(2)) then
			faces:set(i, true)
		end
	end
end
function Mesh:removeFacesInside(boxes, margin)
	local margin1=margin
	local margin2=margin
	local mesh=self
	local selectedVertices=self:selectVerticesIn(boxes, margin1) 
	local faces=boolN()
	local ec=EdgeConnectivity(mesh)
	local mc=OBJloader.MeshConnectivity(mesh, ec)
	--mc:selectFacesFromVertices(selectedVertices, faces)
	self:selectFacesFromVertices(selectedVertices, faces)

	local facesSubset=boolN(faces:size())

	for i, b in ipairs(boxes) do
		-- only remove triangles which are entirely in the box
		for i=0, faces:size()-1 do
			if faces(i) then
				local f=mesh:getFace(i)
				if b:contains(toVector2(mesh:getVertex(f:vertexIndex(0))), margin2) and
					b:contains(toVector2(mesh:getVertex(f:vertexIndex(1))), margin2) and
					b:contains(toVector2(mesh:getVertex(f:vertexIndex(2))), margin2) then

					--dbg.draw('Sphere', mesh:calcFaceCenter(i), 'face0'..i)
					facesSubset:set(i, true)
				end
			end
		end
	end

	self:removeFaces(facesSubset:findAll(true))
end
function Mesh:removeFacesOutside(boxes, margin)
	local margin1=margin
	local margin2=margin
	local mesh=self
	local selectedVertices=self:selectVerticesIn(boxes, margin1) 
	local faces=boolN()
	local ec=EdgeConnectivity(mesh)
	local mc=OBJloader.MeshConnectivity(mesh, ec)
	--mc:selectFacesFromVertices(selectedVertices, faces)
	self:selectFacesFromVertices(selectedVertices, faces)

	local facesSubset=boolN(faces:size())
	facesSubset:setAllValue(true)

	for i, b in ipairs(boxes) do
		-- only remove triangles which are entirely in the box
		for i=0, faces:size()-1 do
			if faces(i) then
				local f=mesh:getFace(i)
				if b:contains(toVector2(mesh:getVertex(f:vertexIndex(0))), margin2) and
					b:contains(toVector2(mesh:getVertex(f:vertexIndex(1))), margin2) and
					b:contains(toVector2(mesh:getVertex(f:vertexIndex(2))), margin2) then

					--dbg.draw('Sphere', mesh:calcFaceCenter(i), 'face0'..i)
					facesSubset:set(i, false)
				end
			end
		end
	end

	self:removeFaces(facesSubset:findAll(true))
end

function Mesh:selectFacesFromEdges(_edges)
	local mesh=self
	local ec=_edges[1]
	local edges=_edges[2]:findAll(true)
	local faces=boolN()
	faces:resize(mesh:numFace())
	local vertexIndex=intvectorn(mesh:numFace())
	vertexIndex:setAllValue(-1)
	for i=0, edges:size()-1 do
		local e=edges(i)
		for j=0, ec:getNumFaceEdge(e)-1 do
			local fe=ec:getFaceEdge(e,j)
			faces:set(fe.faceIndex, true)
			if vertexIndex(fe.faceIndex)==-1 then
				vertexIndex:set(fe.faceIndex, fe.vertexIndex)
			else
				local count={0,0,0}
				count[vertexIndex(fe.faceIndex)+1]=1
				count[fe.vertexIndex+1]=1
				if count[1]==0 then
					vertexIndex:set(fe.faceIndex, 0)
				elseif count[2]==0 then
					vertexIndex:set(fe.faceIndex, 1)
				else
					vertexIndex:set(fe.faceIndex, 2)
				end
			end
		end
	end
	return faces, vertexIndex
end

function Mesh:drawVertices(vertices, options)
	if not options then
		options={}
	end
	local center=vector3N()
	for i=0, vertices:size()-1 do
		if vertices(i) then
			center:pushBack(self:getVertex(i)+(options.offset or vector3(0,0,0)))
			--dbg.draw('Sphere', ,RE.generateUniqueName(), 'green', 10)
		end
	end
	local thickness=2
	dbg.drawBillboard(center:matView()*(options.skinScale or 100), options.name or RE.generateUniqueName(), 'blueCircle', thickness, 'QuadListV')
end
function Mesh:drawEdges( _edges, name)
	local mesh=self
	local ec=_edges[1]
	local edges=_edges[2]:findAll(true)

	y= y or 0
	name=name or 'boxes'
	local lines=vector3N()
	lines:resize(edges:size()*2)
	for i=0, edges:size()-1 do
		local s=ec:source(edges(i))
		local t=ec:target(edges(i))
		lines(i*2):assign(mesh:getVertex(s))
		lines(i*2+1):assign(mesh:getVertex(t))
	end

	dbg.draw('Traj', lines:matView(), name, 'solidwhite', 5, 'BillboardLineList')
end
function toVector2(v3)
	local vv=vector2()
	vv:assignXZ(v3)
	return vv
end
-- fast but works only when the mesh is denser than boxes
function Mesh:splitFacesOverlappingBox(faces, vi, b)
	local planes={
		Plane(vector3(-1,0,0), b.min:toVector3()),
		Plane(vector3(1,0,0), b.max:toVector3()),
		Plane(vector3(0,0,-1), b.min:toVector3()),
		Plane(vector3(0,0,1), b.max:toVector3())
	}
	self:_splitFacesOverlappingPlanes(faces, vi, b, planes)
end

function appendBoxLines(lines, b, y, margin)
		local min=b.min:toVector3()
		local max=b.max:toVector3()
		min.x=min.x-margin
		min.z=min.z-margin
		max.x=max.x+margin
		max.z=max.z+margin

		min.y=y
		max.y=y
		lines:pushBack(min)
		lines:pushBack(vector3(min.x,y, max.z))
		lines:pushBack(vector3(min.x,y, max.z))
		lines:pushBack(max)
		lines:pushBack(max)
		lines:pushBack(vector3(max.x,y, min.z))
		lines:pushBack(vector3(max.x,y, min.z))
		lines:pushBack(min)
end
-- maxTriangleWidth 보다 큰 삼각형이 mesh에 없도록 적당히 큰 숫자.
function Mesh:splitFacesOverlappingBoxes(boxes, maxTriangleWidth)
	local mesh=self
	maxTriangleWidth=maxTriangleWidth or 50
	for i, b in ipairs(boxes) do
		--mesh:splitFacesOverlappingBox(faces, vi, b)

		--mesh:drawVertices(vertices)

		local lines=vector3N()
		appendBoxLines(lines, b, 0,0)
		local planes={
			Plane(vector3(-1,0,0), b.min:toVector3()),
			Plane(vector3(0,0,1), b.max:toVector3()),
			Plane(vector3(1,0,0), b.max:toVector3()),
			Plane(vector3(0,0,-1), b.min:toVector3()),
		}
		for side=0, 3 do
			local vertices=mesh:selectVerticesIn({b}, maxTriangleWidth)
			local faces=boolN(mesh:numFace())
			faces:setAllValue(false)
			for fi=0, mesh:numFace()-1 do
				local f=mesh:getFace(fi)
				if vertices(f:vertexIndex(0)) then
					local v0=mesh:getVertex(f:vertexIndex(0))
					local v1=mesh:getVertex(f:vertexIndex(1))
					local v2=mesh:getVertex(f:vertexIndex(2))
					local y=math.min(v0.y, v1.y, v2.y)-1
					local p0=lines(side*2)
					local p1=lines(side*2+1)
					local p2=p0*0.5+p1*0.5
					p0.y=y
					p1.y=y
					p2.y=1000

					local res=math.testTriTriOverlap(v0,v1,v2, p0,p1,p2)
					if res then
						faces:set(fi, true)
					end
				end
			end
			mesh:splitFaces(faces,nil, {planes[side+1]})
		end
	end
end


function Mesh:splitFacesOverlappingCircles(faces, vi, b)

	local ndiv=10
	local vertices=vector3N(ndiv)
	local front=vector3(0,0,1)
	local center=b.center
	local radius=b.R
	for j=0, ndiv-1 do
		local q=quater()
		q:setRotation(vector3(0,1,0), sop.map(j, 0, ndiv, 0, math.pi*2.0));
		local nf=q*front
		vertices(j):assign(nf*radius+center)
	end
	local planes={}
	for j=0, ndiv-1 do
		local v0=j;
		local v2=math.fmod(j+1,ndiv);
		local dir=vertices(v2)-vertices(v0)
		dir:normalize()
		local normal=vector3()
		normal:cross(dir, vector3(0,1,0))
		table.insert(planes, Plane(normal, vertices(v0)))
	end

	self:_splitFacesOverlappingPlanes(faces, vi, b, planes)
end
function Mesh:_splitFacesOverlappingPlanes(faces, vi, b, planes)
	local facesSubset=faces:copy()
	facesSubset:setAllValue(false)
	local mesh=self
	for i=0, faces:size()-1 do
		if faces(i) then
			local fe=OBJloader.FaceEdge(i, vi(i))
			local s=fe:source(mesh)
			local t=fe:target(mesh)
			local c=fe:cross(mesh)
			if b:contains(toVector2(mesh:getVertex(c)),0) then
				--dbg.draw('Sphere', mesh:getVertex(c),RE.generateUniqueName(), 'green', 10)
				facesSubset:set(i, true)
			elseif b:contains(toVector2(mesh:getVertex(s)),0) then
				--dbg.draw('Sphere', mesh:getVertex(c),RE.generateUniqueName(), 'red', 10)
				facesSubset:set(i, true)
			end
		end
	end
	self:splitFaces(facesSubset, vi, planes)
end
-- faces: boolN (size=numFace())
-- vi: intvectorn (size=numFace()). setAllValue(0) unless you know which face-edge to cut first. (Each triangle has three face-edges.)
function Mesh:splitFaces(faces, vi, planes)
	if vi==nil then
		vi=intvectorn(faces:size())
		vi:setAllValue(0)
	end
	local mesh=self

	local function splitFaceEdge(fe, plane)
		local s=fe:source(mesh)
		local t=fe:target(mesh)
		local c=fe:cross(mesh)

		local dc=plane:distance(mesh:getVertex(c))
		local ds=plane:distance(mesh:getVertex(s))
		local dt=plane:distance(mesh:getVertex(t))
		if ds*dc<0 and dt*dc<0 then
			-- split this face into three.

			-- first move the vertex c
			local ray=Ray(mesh:getVertex(s), mesh:getVertex(c)-mesh:getVertex(s))
			local v=ray:intersects(plane)
			assert(v(0)==1)
			mesh:resizeVertexBuffer(mesh:numVertex()+2)
			local sc=mesh:numVertex()-1
			local tc=mesh:numVertex()-2

			local rayParamSC=v(1)
			mesh:getVertex(sc):assign(ray:getPoint(rayParamSC))
			mesh:getFace(fe.faceIndex):setVertexIndex(math.fmod(fe.vertexIndex+2,3), sc)

			ray=Ray(mesh:getVertex(t), mesh:getVertex(c)-mesh:getVertex(t))
			v=ray:intersects(plane)
			assert(v(0)==1)
			local rayParamTC=v(1)
			mesh:getVertex(tc):assign(ray:getPoint(rayParamTC))



			----       s-----------------------c
			--         |            -----/
			--         |      -----/     
			--         |-----/
			--         t
			--
			----       s----------sc-----------c
			--         |    /       -----/
			--         |      ----tc     
			--         |-----/
			--         t
			-- original face : (s, t, c) -> modified to (s, t, sc)
			-- add two new faces : (sc, t, tc), (sc, tc, c)

			mesh:resizeIndexBuffer(mesh:numFace()+2)
			mesh:getFace(mesh:numFace()-1):assign(mesh:getFace(fe.faceIndex))
			mesh:getFace(mesh:numFace()-2):assign(mesh:getFace(fe.faceIndex))
			mesh:getFace(mesh:numFace()-1):setIndex(sc, t, tc)
			mesh:getFace(mesh:numFace()-2):setIndex(sc, tc, c)


			if mesh:numTexCoord()>0 then
				mesh:resizeBuffer(Mesh.TEXCOORD, mesh:numTexCoord()+2)
				local f=mesh:getFace(fe.faceIndex)
				local s=f:texCoordIndex(fe.vertexIndex)
				local t=f:texCoordIndex(math.fmod(fe.vertexIndex+1,3))
				local c=f:texCoordIndex(math.fmod(fe.vertexIndex+2,3))
				local sc=mesh:numTexCoord()-1
				local tc=mesh:numTexCoord()-2
				mesh:getTexCoord(tc):assign(mesh:getTexCoord(t)*(1-rayParamTC)+mesh:getTexCoord(c)*rayParamTC)
				mesh:getTexCoord(sc):assign(mesh:getTexCoord(s)*(1-rayParamSC)+mesh:getTexCoord(c)*rayParamSC)

				f:setIndex1(math.fmod(fe.vertexIndex+2,3),  sc, Mesh.TEXCOORD)
				mesh:getFace(mesh:numFace()-1):setIndex(sc,t,tc, Mesh.TEXCOORD)
				mesh:getFace(mesh:numFace()-2):setIndex(sc,tc,c, Mesh.TEXCOORD)
			end
			if mesh:numNormal()>0 then
				mesh:resizeBuffer(Mesh.NORMAL, mesh:numNormal()+2)
				local f=mesh:getFace(fe.faceIndex)
				local s=f:normalIndex(fe.vertexIndex)
				local t=f:normalIndex(math.fmod(fe.vertexIndex+1,3))
				local c=f:normalIndex(math.fmod(fe.vertexIndex+2,3))
				local sc=mesh:numNormal()-1
				local tc=mesh:numNormal()-2
				mesh:getNormal(tc):assign(mesh:getNormal(t)*(1-rayParamTC)+mesh:getNormal(c)*rayParamTC)
				mesh:getNormal(sc):assign(mesh:getNormal(s)*(1-rayParamSC)+mesh:getNormal(c)*rayParamSC)

				f:setIndex1(math.fmod(fe.vertexIndex+2,3),  sc, Mesh.NORMAL)
				mesh:getFace(mesh:numFace()-1):setIndex(sc,t,tc, Mesh.NORMAL)
				mesh:getFace(mesh:numFace()-2):setIndex(sc,tc,c, Mesh.NORMAL)
			end

			-- select the new triangles too for spliting.
			faces:resize(mesh:numFace())
			faces:set(mesh:numFace()-1, true)
			faces:set(mesh:numFace()-2, true)
			vi:resize(mesh:numFace())
			vi:set(mesh:numFace()-1, 0)
			vi:set(mesh:numFace()-2, 0)
			return true
		elseif math.abs(ds)<1e-6 and dt*dc<0 then
			-- split this face into three.
			----       s-----------------------c
			--         |  \         -----/
			--         |    \ -----/     
			--         |-----/ tc
			--         t

			-- first move the vertex c
			local ray=Ray(mesh:getVertex(t), mesh:getVertex(c)-mesh:getVertex(t))
			local v=ray:intersects(plane)
			assert(v(0)==1)
			mesh:resizeVertexBuffer(mesh:numVertex()+1)
			local tc=mesh:numVertex()-1

			mesh:getVertex(tc):assign(ray:getPoint(v(1)))
			-- original face : (s, t, c) -> modified to (s, t, tc)
			mesh:getFace(fe.faceIndex):setVertexIndex(math.fmod(fe.vertexIndex+2,3), tc)
			-- add a new face : (s, tc, c)

			mesh:resizeIndexBuffer(mesh:numFace()+1)
			mesh:getFace(mesh:numFace()-1):assign(mesh:getFace(fe.faceIndex))
			mesh:getFace(mesh:numFace()-1):setIndex(s, tc, c)

			if mesh:numTexCoord()>0 then
				mesh:resizeBuffer(Mesh.TEXCOORD, mesh:numTexCoord()+1)
				local f=mesh:getFace(fe.faceIndex)
				local s=f:texCoordIndex(fe.vertexIndex)
				local t=f:texCoordIndex(math.fmod(fe.vertexIndex+1,3))
				local c=f:texCoordIndex(math.fmod(fe.vertexIndex+2,3))
				local tc=mesh:numTexCoord()-1
				local rayParamTC=v(1)
				mesh:getTexCoord(tc):assign(mesh:getTexCoord(t)*(1-rayParamTC)+mesh:getTexCoord(c)*rayParamTC)

				f:setIndex1(math.fmod(fe.vertexIndex+2,3),  tc, Mesh.TEXCOORD)
				mesh:getFace(mesh:numFace()-1):setIndex(s,tc,c, Mesh.TEXCOORD)
			end
			if mesh:numNormal()>0 then
				mesh:resizeBuffer(Mesh.NORMAL, mesh:numNormal()+1)
				local f=mesh:getFace(fe.faceIndex)
				local s=f:normalIndex(fe.vertexIndex)
				local t=f:normalIndex(math.fmod(fe.vertexIndex+1,3))
				local c=f:normalIndex(math.fmod(fe.vertexIndex+2,3))
				local tc=mesh:numNormal()-1
				local rayParamTC=v(1)
				mesh:getNormal(tc):assign(mesh:getNormal(t)*(1-rayParamTC)+mesh:getNormal(c)*rayParamTC)

				f:setIndex1(math.fmod(fe.vertexIndex+2,3),  tc, Mesh.NORMAL)
				mesh:getFace(mesh:numFace()-1):setIndex(s,tc,c, Mesh.NORMAL)
			end

			-- select the new triangle for spliting.
			faces:resize(mesh:numFace())
			faces:set(mesh:numFace()-1, true)
			vi:resize(mesh:numFace())
			vi:set(mesh:numFace()-1, 0)
			return true
		end
		return false
	end

	local function splitFace(i, plane)
		local fe=OBJloader.FaceEdge(i, 0)
		splitFaceEdge(fe, plane)
		fe=OBJloader.FaceEdge(i, 1)
		splitFaceEdge(fe, plane)
		fe=OBJloader.FaceEdge(i, 2)
		splitFaceEdge(fe, plane)
	end
	for ip, plane in ipairs(planes) do
		if not (faces:size()==mesh:numFace()) then print('warning! there might be errors when there are overlapping boxes') end
		for i=0, faces:size()-1 do
			if faces(i) then
				local fe=OBJloader.FaceEdge(i, vi(i))
				if not splitFaceEdge(fe, plane) then
					fe=OBJloader.FaceEdge(i, math.fmod(vi(i)+1,3))
					if not splitFaceEdge(fe, plane) then
						fe=OBJloader.FaceEdge(i, math.fmod(vi(i)+2,3))
						splitFaceEdge(fe, plane)
					end
				end
			end
		end
	end
end
function Mesh:drawFaceEdges(faces, vi, name)
	local mesh=self
	name=name or 'boxes'
	local lines=vector3N()
	for i=0, faces:size()-1 do
		if faces(i) then
			local fe=OBJloader.FaceEdge(i, vi(i))
			local s=fe:source(mesh)
			local t=fe:target(mesh)
			lines:pushBack(mesh:getVertex(s))
			lines:pushBack(mesh:getVertex(t))

			--dbg.draw('Arrow', mesh:getVertex(s), mesh:getVertex(t),  'arrow'..i)
		end
	end

	dbg.draw('Traj', lines:matView(), name, 'solidwhiteTrailZTest', 10, 'BillboardLineList')
end
defineDerived(Mesh, 
{
	Geometry,
	OBJloader.Terrain,
}, 
{
	"drawMesh", 
	"createEntity",
	"verticallyExtrudeBoundaryEdges",
	"_splitFacesOverlappingPlanes",
	"selectVerticesIn",
	"translateVertices",
	"selectBoundaryEdges",
	"selectAllEdges",
	"removeFacesOverlapping",
	"selectFacesFromVertices",
	"removeFacesInside",
	"removeFacesOutside",
	"selectFacesFromEdges",
	"drawEdges",
	"drawVertices",
	"splitFacesOverlappingBox",
	"splitFacesOverlappingCircles",
	"splitFacesOverlappingBoxes",
	"splitFaces",
	"drawFaceEdges",
}
)

SDFcollider=LUAclass()
-- options ={ debugDraw=false, }
function SDFcollider:__init(options)
	if not options then options={} end
	self.options=options
	self.models={}
	self.detector= Physics.CollisionSequence.createCollisionDetector_gjk()
	--self.detector= Physics.CollisionSequence.createCollisionDetector_bullet()
end
function SDFcollider:testIntersectionsForDefinedPairs(bases, float_options)
	self.detector:testIntersectionsForDefinedPairs(bases)

	local function vec_abs(v)
		return vector3(math.abs(v.x), math.abs(v.y), math.abs(v.z))

	end
	local collisionLinkPairs=bases:getCollisionLinkPairs()
	if not float_options then float_options={} end
	local dm=float_options.defaultMargin or 0
	local rdm=float_options.radiusDependentMargin or 0.05
	local coldet=self.detector 
	local gmaxDepth=0
	if collisionLinkPairs:size()>=1 then
		for i=0, collisionLinkPairs:size()-1 do
			local ilinkpair=collisionLinkPairs(i)
			local collisionPoints=bases:getCollisionPoints(ilinkpair)

			if collisionPoints:size()>0 then
				local iloader1=bases:getCharacterIndex1(ilinkpair)
				local bone1=bases:getBone1(ilinkpair)
				local iloader2=bases:getCharacterIndex2(ilinkpair)
				local bone2=bases:getBone2(ilinkpair)
				-- it is recommended that bone2 is bigger (thicker) than bone1 (when specifying the coll-pairs)
				local model1=self.models[iloader1+1]
				local model2=self.models[iloader2+1]

				local skip=false
				local localnormal=nil
				local voxelI=nil
				if collisionPoints:size()>0 then
					-- find maxDepth point
					local maxDepth=0
					local argMax=-1
					for icp=0, collisionPoints:size()-1 do
						local cp=collisionPoints(icp)
						local depth=cp.idepth

						if model2.isFBX and model1.isFBX then
							-- cp.normal : toward body2 inside
							-- cp.position : body2 표면 위의 global점.
							local maxPenPoint=cp.position+cp.normal*cp.idepth -- 가장 깊이 있는 점은 body 1 surface위의 점이다. 
							local sd, n, vi=self:_sampleSDF(model2, bone2:treeIndex(), maxPenPoint)
							depth=-sd
							if depth>maxDepth then
								localnormal=n
								voxelI=vi
							end
						elseif false and model1.isFBX then
							-- 한쪽만 fbx인 경우 fbx가 피하는게 나서, if로 안들어오게 수정. 
							local maxPenPoint=cp.position -- 가장 깊이 있는점은 body2 surface위의 점이다. 
							local sd, n, vi=self:_sampleSDF(model1, bone1:treeIndex(), maxPenPoint)
							depth=-sd
							if depth>maxDepth then
								localnormal=n
								voxelI=vi
							end
						end
						if depth>maxDepth then
							argMax=icp
							maxDepth=depth
						end
					end
					if argMax==-1 then
						collisionPoints:resize(0)
						skip=true
					else
						local cp=collisionPoints(argMax)

						collisionPoints(0).normal:assign(cp.normal)
						if model2.isFBX and model1.isFBX and voxelI then
							local maxPenPoint=cp.position+cp.normal*cp.idepth -- on model2
							local new_normal=self:_calcNormalTowardOutside(model2, voxelI, localnormal)
							--dbg.draw("Line2", maxPenPoint*100, (maxPenPoint+new_normal*100)*100, tostring(iloader)..'_'..tostring(ibody))
							collisionPoints(0).normal:assign(new_normal*-1)

							cp.position:assign(maxPenPoint+new_normal*maxDepth)
						elseif false and model1.isFBX then
							local new_normal=self:_calcNormalTowardOutside(model1, voxelI, localnormal)
							collisionPoints(0).normal:assign(new_normal)
						end
						collisionPoints(0).idepth=maxDepth
						collisionPoints(0).position:assign(cp.position)
						collisionPoints:resize(1)
					end

					if true and not skip then
						-- now, let's handle margin.
						if model2.isFBX and model1.isFBX then
							-- cp.normal : toward body2 inside
							-- cp.position : body2 표면 위의 global점.
							-- only for self-collisions
							local margin=dm
							local size1=vector3()
							local size2=vector3()

							coldet:getLocalBoundingBoxSize(iloader1,bone1:treeIndex(), size1)
							coldet:getLocalBoundingBoxSize(iloader2,bone2:treeIndex(), size2)

							local b=collisionPoints(0)
							local minsize=math.min(
							vec_abs((model1.fk:globalFrame(bone1).rotation:inverse()*b.normal)):dotProduct(size1),
							vec_abs((model2.fk:globalFrame(bone2).rotation:inverse()*b.normal)):dotProduct(size2))

							margin=margin+math.max(0, minsize-0.02)*0.5*rdm
							--dbg.draw('Sphere', b.position*100, tostring(bone1:treeIndex())..tostring(bone2:treeIndex()), 'white', margin*20)

							if b.idepth>margin then
								local maxPenPoint=b.position+b.normal*b.idepth -- 가장 깊이 있는 점은 body 1 surface위의 점이다. 
								b.idepth=b.idepth-margin
								b.position:assign(maxPenPoint-b.normal*b.idepth)
								gmaxDepth=math.max(b.idepth, gmaxDepth)
							else
								collisionPoints:erase(0)
							end
						else
							local cp=collisionPoints(0)
							gmaxDepth=math.max(cp.idepth, gmaxDepth)
						end
					end
				end
			end
		end
	end
	--[[
	local collisionLinkPairs=bases:getCollisionLinkPairs()
	for i=0, collisionLinkPairs:size()-1 do
		local ilinkpair=collisionLinkPairs(i)
		local collisionPoints=bases:getCollisionPoints(ilinkpair)
		for j=0, collisionPoints:size()-1 do
			local cp=collisionPoints(j)
			print(ilinkpair, cp.depth, cp.position)
		end
	end
	]]
	return gmaxDepth
end

function SDFcollider:getModel(i)
	return self.detector:getModel(i)
end

function SDFcollider.unpackCollisionLoader(filename)
	local cache=util.BinaryFile()
	cache:openRead(filename)
	local loader=MainLib.VRMLloader()
	local version= cache:unpackInt()
	assert(version<0)
	cache:_unpackVRMLloader(loader)
	return loader
end
function SDFcollider:unpack(binaryFile, model)
	local version= self.cache:unpackInt()
	if version~=-3 then
		-- incompatible with latest cache format.
		return false
	else
		function unpackColInfo(model)
			-- wrlloader
			local loader=MainLib.VRMLloader()
			print("unpack loader...")
			self.cache:_unpackVRMLloader(loader)
			print("done.")

			model.collisionLoader=loader
			model.SDF=floatTensor()
			model.normal=Tensor()
			model.vertexIndex=floatTensor()

			local origL=model.fbx.loader
			if (origL:numBone()~=loader:numBone() ) 
				or (origL.dofInfo:hasQuaternion(1)~=loader.dofInfo:hasQuaternion(1) )
				then
				model.collisionLoader=nil
				error('incompatible1')
			end
			local updated=false
			for i=1, loader:numBone()-1 do
				if ( loader:bone(i):getRotationalChannels()~=origL:bone(i):getRotationalChannels()) then
					local bb=MainLib.VRMLloader.upcast(loader:bone(i))
					bb:setJointAxes(origL:bone(i):getRotationalChannels())
					updated=true
				end
			end
			if updated then
				loader:_initDOFinfo()
			end
			if false then
				-- no problem.
				local pose=CT.rand(loader.dofInfo:numDOF())
				loader:setPoseDOF(pose)
				origL:setPoseDOF(pose)
				for i=1, loader:numBone()-1 do
					assert(loader:bone(i):getFrame().translation:distance(
					origL:bone(i):getFrame().translation)<1e-3)
				end
			end

			self.cache:unpack(model.SDF)
			self.cache:unpack(model.normal)
			self.cache:unpack(model.vertexIndex)

			model.decomp={}
			-- world bmin and bmax
			model.decomp.bmin=self.cache:unpackAny():toVector3()
			model.decomp.bmax=self.cache:unpackAny():toVector3()
			model.decomp.mScale=self.cache:unpackAny()
			model.decomp.mVoxelScale=self.cache:unpackAny()
			model.decomp.mCenter=self.cache:unpackAny():toVector3()

			model.decomp.__init=function(self)
				self.bmin_local=(self.bmin-self.mCenter)*(1.0/self.mScale)
			end
			model.decomp:__init()

			model.decomp.getScale = function(self) return self.mScale end
			model.decomp.getVoxelScale = function(self) return self.mVoxelScale end
			model.decomp.getBoundsMin = function(self) return self.bmin end
			model.decomp.getBoundsMax = function(self) return self.bmax end
			model.decomp.getVoxelIndexContinuous = function(self, global_position) 
				local recipScale=1.0/self.mScale
				local lpos= (global_position-self.mCenter)*recipScale;
				return (lpos-self.bmin_local)*(1.0/self.mVoxelScale)
			end
			model.decomp.getWorldPosition=function(self, voxelIndex) 
				local out=voxelIndex*self.mVoxelScale+self.bmin_local
				return out*self.mScale+self.mCenter
			end
			return true
		end
		local pcall_ok, msg= pcall(unpackColInfo, model)
		if pcall_ok then
		else
			print(msg) -- this cache is invalid so let's regenerate
			model.SDF=nil
			model.normal=nil
			model.vertexIndex=nil
			return false
		end
	end
	return true
end
function SDFcollider:numModels()
	return self.detector:numModels()
end
function SDFcollider:_calculateBBOX(model, loader)
	model.bboxMin=vectorn()
	model.bboxMin:setSize(loader:numBone())
	model.bboxMin:setAllValue(0)
	local coldet=self.detector
	for i=1, loader:numBone()-1 do
		local size1=vector3()
		if loader:VRMLbone(i):hasShape() then
			coldet:getLocalBoundingBoxSize(coldet:numModels()-1, i, size1)
			model.bboxMin:set(i, math.min(size1.x, size1.y, size1.z))
		end
	end
end
function SDFcollider:addModel(fbxloader, _optionalWRLloader)
	if not fbxloader.fbxInfo then
		-- fbx아니고 일반 모델(VRMLloader)인경우 

		local model={}
		model.isFBX=false
		table.insert(self.models,model)
		self.detector:addModel(fbxloader)
		return 
	end
	if not _optionalWRLloader and fbxloader.cacheCollisionMesh then
		_optionalWRLloader={
			cacheCollisionMesh=fbxloader.cacheCollisionMesh,
			cacheCollisionLoader=fbxloader.cacheCollisionLoader
		}
	end
	--local _,n=fbxloader.fbxInfo[3][1]:drawMesh('red_transparent', 'node2')
	--n:scale(100,100,100)
	local OUTER_VOXEL=2
	local INTERIOR_VOXEL=3
	local SURFACE_VOXEL=4
	local DEBUG_DRAW=false
	local DRAW_VOXEL_CENTER=false
	local model={}

	model.isFBX=true
	model.fbx=fbxloader
	model.fk=BoneForwardKinematics(fbxloader.loader)
	model.fk:assign(fbxloader.loader:fkSolver())
	model.fk_bindpose=BoneForwardKinematics(fbxloader.loader)

	local res=100
	model.fbx_info=fbxloader:createSkinningInfo()
	fbxloader.loader:fkSolver():assign(model.fk)
	model.fk_bindpose:setPose(model.fbx_info.bindPose)

	for i=1, model.fbx_info.loader:numBone()-1 do
		assert(model.fbx_info.loader:bone(i):name()==fbxloader.loader:bone(i):name())
	end


	if _optionalWRLloader then
		if type(_optionalWRLloader)=='table' then
			if _optionalWRLloader.cacheCollisionMesh then
				if os.isFileExist(_optionalWRLloader.cacheCollisionMesh ) then
					self.cache=util.BinaryFile()
					print("loading cache:", _optionalWRLloader.cacheCollisionMesh )
					self.cache:openRead(_optionalWRLloader.cacheCollisionMesh )
					if not self:unpack(self.cache, model) then
						-- 로딩 실패. 
						os.deleteFiles(_optionalWRLloader.cacheCollisionMesh )
						self.wcache=util.BinaryFile()
						self.cache=nil
					end
					if _optionalWRLloader.cacheCollisionLoader then
						model.collisionLoader=_optionalWRLloader.cacheCollisionLoader 
						assert(model.collisionLoader:numBone()==_optionalWRLloader.cacheCollisionLoader :numBone())
					end
				else
					self.wcache=util.BinaryFile()
				end

				local n=tonumber(_optionalWRLloader.cacheCollisionMesh :sub(-2))
				if not n then
					n=tonumber(_optionalWRLloader.cacheCollisionMesh :sub(-1))
				end
				if not model.collisionLoader then
					local temp=SkinToWRL(model.fbx_info, { maxConvex= n or 1})
					model.collisionLoader=temp.skel
					assert(temp.skel:numBone()==fbxloader.loader:numBone())
				end
			end
		else
			model.collisionLoader=_optionalWRLloader
			assert(model.collisionLoader:numBone()==fbxloader.loader:numBone())
		end
	else 
		local temp=SkinToWRL(model.fbx_info, { maxConvex=1})
		model.collisionLoader=temp.skel
		assert(temp.skel:numBone()==fbxloader.loader:numBone())
	end
	self.detector:addModel(model.collisionLoader)
	self:_calculateBBOX(model, model.collisionLoader)

	local TEST_CONVEX_DECOMPOSITION=false
	local maxConvex=0
	local fbx_info=model.fbx_info
	local mesh=fbx_info.mesh
	--local _,n=fbx_info.mesh:drawMesh()
	--n:scale(100,100,100)

	local decomp
	if model.decomp then
		-- 파일에서 로딩된 정보. 
		decomp=model.decomp
	else
		if not ConvexDecomp then
			util.msgBox('Error! use sample_physX.git instead!!!')
		end
		-- 생성.(오래걸림)  
		decomp=ConvexDecomp(mesh, TEST_CONVEX_DECOMPOSITION, res*res*res, maxConvex, 1)
		model.decomp=decomp
	end

	local bmin=decomp:getBoundsMin()
	local bmax=decomp:getBoundsMax()
	local bcenter=bmin*0.5+bmax*0.5

	model.bbox=Geometry()
	model.bbox:initBox(bmax-bmin)
	model.bbox_collider=Physics.CollisionSequence.createCollisionDetector_bullet()
	model.bbox_collider:addObstacle(model.bbox)
	do
		local boxloader=model.bbox_collider:getModel(0)
		local pose=boxloader:pose()
		pose:setRootTransformation(transf(bcenter))
		boxloader:setPose(pose)
		model.bbox_collider:setWorldTransformations(0, boxloader:fkSolver())
	end

	if not model.SDF then
		local timer=util.Timer()
		timer:start()
		if true then

			if true then
				timer:start()
				print("creating full-voxel graph...")
				-- 아래 그래프의 노드 개수는 decomp:numInteriorVoxels()+decomp:numSurfaceVoxels()
				local fullgraph=decomp:createGraphFromAllOccupantVoxels()  -- faster c++ implementation
				model.interiorGraph=fullgraph
				local graph2=fullgraph.graph
				local nodeIndex2=fullgraph.nodeIndex
				local nodes2=fullgraph.nodes


				local interiorSDF=vectorn()
				if true then
					-- 이제 서피스 복셀들로부터 거리 계산.
					local sourceNodeIndex=graph2:numNodes()
					graph2:newNode()

					local nvoxel=decomp:numSurfaceVoxels()

					timer:start()
					local node_dist=interiorSDF
					--voxelcenter=vector3N(nvoxel)
					for i=0, nvoxel-1 do
						--voxelcenter(i):assign(decomp:getWorldPosition(decomp:getSurfaceVoxelIndex(i)) *scale+vector3(-offset_x,0,0))
						local voxelIndex=decomp:getSurfaceVoxelIndex(i) -- 3D index
						local nodeIndex=fullgraph.nodeIndex(voxelIndex) -- the corresponding graph node 
						graph2:addEdge(sourceNodeIndex, nodeIndex,0)
					end

					graph2:Dijkstra(sourceNodeIndex, node_dist)

					-- delete the temporary source
					graph2:resize(sourceNodeIndex)

					print('interior SDF calculation:', timer:stop2()*1e-3,'ms')
				end

				local skinScale=100
				local offset_x=100
				local draw_offset=vector3(offset_x, 0,0)
				if self.options.debugDraw then
					timer:start()
					-- visualize a 2D slice (i, j, center_k)
					local center=decomp:getDimensions()
					local center_k=math.floor(center.z/2)
					local maxDist=interiorSDF:maximum()
					local points=matrixn(0, 6) -- each row contains (R, G, B, x, y, z)
					for i=0, center.x-1 do
						for j=0, center.y-1 do
							local voxelIndex=vector3(i,j,center_k)
							local voxeltype =decomp:getVoxel(voxelIndex)
							if voxeltype~=OUTER_VOXEL then
								-- nodeIndex function uses std::map, so it can be relatively slow compared to getVoxel.
								local nodeIndex=fullgraph.nodeIndex(voxelIndex)
								assert(nodeIndex~=-1)
								local pos=decomp:getWorldPosition(voxelIndex)
								local dist=interiorSDF(nodeIndex)
								local R=sop.map(dist, 0,maxDist, 1,0)
								local G=0
								local B=sop.map(dist, 0,maxDist, 0,1)
								local row=vectorn(6)
								row:setVec3(0, vector3(R,G,B))
								row:setVec3(3,pos*skinScale+draw_offset)
								points:pushBack(row)
							end
						end
					end
					print('visualization:', timer:stop2()*1e-3,'ms')
					dbg.draw( 'Traj', points, 'centerSDF', 'Point10', 0, 'PointList')
				end

				local exteriorSDF=vectorn()
				local extgraph
				if true then
					extgraph=decomp:createGraphFromAllVacantVoxels()  -- faster c++ implementation
					local graph3=extgraph.graph
					model.exteriorGraph=extgraph
					model.exteriorGraph_toSurfVoxel=intvectorn()
					-- 이제 서피스 복셀들로부터 거리 계산.
					local sourceNodeIndex=graph3:numNodes()
					graph3:newNode()

					local nvoxel=decomp:numSurfaceVoxels()

					timer:start()
					local node_dist=exteriorSDF
					--voxelcenter=vector3N(nvoxel)
					local isSurfaceVoxel=boolN(sourceNodeIndex)
					for i=0, nvoxel-1 do
						--voxelcenter(i):assign(decomp:getWorldPosition(decomp:getSurfaceVoxelIndex(i)) *scale+vector3(-offset_x,0,0))
						local voxelIndex=decomp:getSurfaceVoxelIndex(i) -- 3D index
						local nodeIndex=extgraph.nodeIndex(voxelIndex) -- the corresponding graph node 
						graph3:addEdge(sourceNodeIndex, nodeIndex,0)
						isSurfaceVoxel:set(nodeIndex, true)
					end
					local pred= model.exteriorGraph_toSurfVoxel
					graph3:Dijkstra(sourceNodeIndex, node_dist, pred)

					print('exterior SDF calculation:', timer:stop2()*1e-3,'ms')
					timer:start()

					local path={}
					for i=0, isSurfaceVoxel:size()-1 do
						if isSurfaceVoxel(i) then
							assert(pred(i)==sourceNodeIndex)
						elseif pred(i)==sourceNodeIndex then
							-- pass
						else
							path[1]=i
							local np=1
							while pred(path[np])~=sourceNodeIndex do
								np=np+1
								path[np]=pred(path[np-1])
							end
							local last=pred(path[np-1])
							for j=1, np-2 do
								pred:set(path[j], last)
							end
						end
					end


					-- delete the temporary source
					graph3:resize(sourceNodeIndex)
					print('searching nearest vertices:', timer:stop2()*1e-3,'ms')

				end


				if self.options.debugDraw then
					-- now, draw both inside and outside
					timer:start()
					-- visualize a 2D slice (i, j, center_k)
					local center=decomp:getDimensions()
					local center_k=math.floor(center.z/2)
					local maxDist=math.max(exteriorSDF:maximum(),interiorSDF:maximum())
					local minDist=-1*maxDist
					--local maxDist=exteriorSDF:maximum()
					--local minDist=0
					local points=matrixn(0, 6) -- each row contains (R, G, B, x, y, z)
					for i=0, center.x-1 do
						for j=0, center.y-1 do
							local voxelIndex=vector3(i,j,center_k)
							local voxeltype =decomp:getVoxel(voxelIndex)
							local pos=decomp:getWorldPosition(voxelIndex)
							local dist
							if voxeltype~=OUTER_VOXEL then
								-- nodeIndex function uses std::map, so it can be relatively slow compared to getVoxel.
								local nodeIndex=fullgraph.nodeIndex(voxelIndex)
								assert(nodeIndex~=-1)
								dist=-1*interiorSDF(nodeIndex)
							else
								local nodeIndex=extgraph.nodeIndex(voxelIndex)
								assert(nodeIndex~=-1)
								dist=exteriorSDF(nodeIndex)
							end
							local R=sop.map(dist, minDist,maxDist, 1,0)
							local G=0
							local B=sop.map(dist, minDist,maxDist, 0,1)
							local row=vectorn(6)
							row:setVec3(0, vector3(R,G,B))
							row:setVec3(3,pos*skinScale+vector3(offset_x*2, 0,0))
							points:pushBack(row)
						end
					end
					print('visualization2:', timer:stop2()*1e-3,'ms')
					dbg.draw( 'Traj', points, 'centerSDF2', 'Point10', 0, 'PointList')
				end
				model.sdf={interiorSDF, exteriorSDF}
			end
		end
		print('voxelize:', timer:stop2()*1e-3,'ms')


		local dim=decomp:getDimensions()
		model.SDF=floatTensor(dim.x, dim.y, dim.z)
		model.normal=Tensor(dim.x, dim.y, dim.z,3)

		local function _calculateSDF(model, decomp, voxelIndexI)
			local voxeltype =decomp:getVoxel(voxelIndexI)
			local OUTER_VOXEL=2
			local dist
			--local dim=getDimensions()
			--dbg.console()
			if voxeltype==OUTER_VOXEL then
				local nodeIndex=model.exteriorGraph.nodeIndex(voxelIndexI)
				assert(nodeIndex~=-1)
				dist=model.sdf[2](nodeIndex)
			else
				local nodeIndex=model.interiorGraph.nodeIndex(voxelIndexI)
				assert(nodeIndex~=-1)
				dist=-1*model.sdf[1](nodeIndex)
			end
			--print(dist, decomp:getScale(), decomp:getDimensions():maximum(), decomp:getBoundsMax()-decomp:getBoundsMin())
			return dist*decomp:getScale()/decomp:getDimensions():maximum()
		end
		do
			local SDF=model.SDF
			local normal=model.normal
			for i=0, dim.x-1 do
				io.write('.')
				io.flush()
				for j=0, dim.y-1 do
					for k=0, dim.z-1 do
						SDF:set(i,j,k, _calculateSDF(model,decomp, vector3(i,j,k)))
					end
				end
			end
			local s2=2*decomp:getScale()/decomp:getDimensions():maximum() --voxel 두칸의 world 길이

			for i=1, dim.x-2 do
				io.write('.')
				io.flush()
				for j=1, dim.y-2 do
					for k=1, dim.z-2 do
						local G=vector3( 
						(SDF(i+1,j,k)-SDF(i-1,j,k))/s2,
						(SDF(i,j+1,k)-SDF(i,j-1,k))/s2,
						(SDF(i,j,k+1)-SDF(i,j,k-1))/s2)
						G:normalize()
						normal:set(i,j,k, 0, G.x)
						normal:set(i,j,k, 1, G.y)
						normal:set(i,j,k, 2, G.z)
					end
					normal:slice(i,j,0,-1):assign( normal:slice(i,j,1,-1))
					normal:slice(i,j,dim.z-1,-1):assign( normal:slice(i,j,dim.z-2,-1))
				end
				normal:slice(i,0,-1,-1):assign(normal:slice(i,1,-1,-1))
				normal:slice(i,dim.y-1,-1,-1):assign(normal:slice(i,dim.y-2,-1,-1))
			end
			normal:slice(0,-1,-1,-1):assign(normal:slice(1,-1,-1,-1))
			normal:slice(dim.x-1,-1,-1,-1):assign(normal:slice(dim.x-2,-1,-1,-1))
		end

		-- free unnecesary memory
		model.interiorGraph=nil
		model.sdf=nil
		collectgarbage()
		collectgarbage()
		collectgarbage()

		if true then
			local timer=util.Timer()
			timer:start()

			print("creating full-voxel graph for weight painting...")
			--local fullgraph=createVoxelGraph(decomp, decomp, function(decomp, i,j,k) local t=decomp:getVoxelSafe(vector3(i,j,k)) return t==SURFACE_VOXEL or t==INTERIOR_VOXEL end)
			local fullgraph=decomp:createGraphFromAllOccupantVoxels()  -- faster c++ implementation
			local graph2=fullgraph.graph
			local nodeIndex2=fullgraph.nodeIndex
			local nodes2=fullgraph.nodes

			timer:start()
			-- 이제 정점 들로부터 거리 계산.

			local loader=fbx_info.loader
			local g_mesh=fbx_info.mesh
			local ncluster=g_mesh:numVertex()

			graph2:newNode() -- sink node
			local sourceNodeIndex=graph2:numNodes()

			for i=0, ncluster -1 do
				graph2:newNode()
			end

			for i=0, ncluster-1 do
				local s=sourceNodeIndex-1
				local t=sourceNodeIndex+i
				graph2:addEdge(s, t, 0)
			end

			local OUTER_VOXEL=2
			local INTERIOR_VOXEL=3
			local SURFACE_VOXEL=4
			local count=0
			for i=0, g_mesh:numVertex()-1 do
				local v=g_mesh:getVertex(i)
				local b=decomp:getVoxelIndex(v)
				local vtype=decomp:getVoxel(b)
				if vtype==SURFACE_VOXEL then
				elseif vtype==INTERIOR_VOXEL then
				else
					b=nil
					count=count+1
				end
				if b then
					local s=sourceNodeIndex+i
					graph2:addEdge(s, nodeIndex2(b),0)
				end
			end
			print('error vertex count=', count/g_mesh:numVertex())

			local node_dist=vectorn()
			local pred=intvectorn()

			graph2:Dijkstra(sourceNodeIndex-1, node_dist, pred)

			local path={}
			for i=0, sourceNodeIndex-2 do
				if pred(i)<sourceNodeIndex-1 then
					-- update 필요. 
					path[1]=i
					local np=1
					while pred(path[np])<sourceNodeIndex-1 do
						path[np+1]=pred(path[np])
						np=np+1
					end
					local last=pred(path[np])
					assert(last>sourceNodeIndex-1)
					for j=1, np-1 do
						pred:set(path[j], last)
					end
				end
			end

			local vertexIndices=floatTensor(dim.x, dim.y, dim.z)
			vertexIndices:setAllValue(-1)

			for i=0, sourceNodeIndex-2 do
				assert(pred(i)>=sourceNodeIndex)
				local vi=nodes2(i)
				vertexIndices:set(vi.x, vi.y, vi.z, pred(i)-sourceNodeIndex)
				assert(model.SDF(vi.x, vi.y, vi.z)<=0)
			end
			print('interiorVoxelToVertexIndex:', timer:stop2()*1e-3,'ms')
			timer:start()
			do
				--print('a')
				local graph=model.exteriorGraph.graph
				local nodes=model.exteriorGraph.nodes


				local pred=model.exteriorGraph_toSurfVoxel
				for i=0, nodes:size()-1 do
					--io.write(i,',')
					local nearestSurfVoxelNode=pred(i)
					if nearestSurfVoxelNode<nodes:size() then
						local j=nodes(nearestSurfVoxelNode)
						local vi=vertexIndices(j.x, j.y, j.z)
						assert(vi~=-1)

						local k=nodes(i)
						vertexIndices:set(k.x, k.y, k.z, vi)
					end
				end
			end


			print('exteriorVoxelToVertexIndex:', timer:stop2()*1e-3,'ms')

			model.vertexIndex=vertexIndices
			model.exteriorGraph=nil
			model.exteriorGraph_toSurfVoxel=nil
			collectgarbage()
			collectgarbage()
			collectgarbage()
		end
	end
	if self.wcache then
		self.wcache:openWrite(_optionalWRLloader.cacheCollisionMesh )
		self:pack( model, self.wcache)
		self.wcache:close()

		if false then
			-- debug packing
			local cache=util.BinaryFile()
			cache:openRead(_optionalWRLloader.cacheCollisionMesh )
			local version= cache:unpackInt()
			assert(version<0)
			local loader=MainLib.VRMLloader()
			cache:_unpackVRMLloader(loader)

			--local origL=model.collisionLoader
			local origL=model.fbx.loader
			for i=1, loader:numBone()-1 do
				if ( loader:bone(i):getRotationalChannels()~=origL:bone(i):getRotationalChannels()) then
					print(i)
					print(model.fbx.loader:bone(i):getRotationalChannels())
					print(loader:bone(i):getRotationalChannels())
					print(model.collisionLoader:bone(i):getRotationalChannels())
					dbg.console()
					error('incmopatible2')
				end
			end
					dbg.console()
		end

	end

	table.insert(self.models,model)

end
function SDFcollider:addObstacle(v)
	local model={}
	model.isFBX=false
	table.insert(self.models,model)
	self.detector:addObstacle(v)
end

function SDFcollider:_calculateSignedDistanceBindPoseInsideBB(model, decomp, global_position, out_normal)
	local vi=decomp:getVoxelIndexContinuous(global_position)
	local voxelIndexI=vector3(math.floor(vi.x+0.01), math.floor(vi.y+0.01), math.floor(vi.z+0.01))
	assert(voxelIndexI.y>-1)
	out_normal:assign(model.normal:slice_1d(CT.ivec(voxelIndexI.x, voxelIndexI.y, voxelIndexI.z,-1)):toVector3())
	return model.SDF(voxelIndexI.x, voxelIndexI.y, voxelIndexI.z), voxelIndexI
end

function SDFcollider:addCollisionPair(loader,b, loader2, b2)
	self.detector:addCollisionPair(loader, b, loader2, b2)
end
function SDFcollider:getLocalBoundingBoxSize(iloader, ibody, out_size)
		local size1=vector3()
		self.detector:getLocalBoundingBoxSize(iloader, ibody, size1)
end
function SDFcollider:setWorldTransformations(iloader, fkSolver)
	local model=self.models[iloader+1]
	if model.isFBX then
		model.fk:assign(fkSolver)
	end
	self.detector:setWorldTransformations(iloader,fkSolver)

	if false then
		g_debugSkin=RE.createSkinAuto(model.fbx_info.loader)
		g_debugSkin:setScale(100,100,100)
		g_debugSkin:setTranslation(100,0,0)
		g_debugSkin:setSamePose(model.fk_bindpose)
		--model.fbx_info.skinningInfo:calcVertexPositions(model.fk, model.fbx_info.mesh)
		local _, node=model.fbx_info.mesh:drawMesh()
		node:setScale(100,100,100)
		node:setPosition(100,0,0)
	end
end

-- perform LBS
function SDFcollider:_calcNormalTowardOutside(model, voxelIndexI, new_normal_in_B)
	local new_normal=vector3(0,0,0)

	local vi=model.vertexIndex(voxelIndexI.x, voxelIndexI.y, voxelIndexI.z)

	local si=model.fbx_info.skinningInfo
	local weights=si:weights(vi)
	local ti=si:treeIndices(vi)

	for j=0,weights:size()-1 do
		local delta=model.fk:globalFrame(ti(j)).rotation*model.fk_bindpose:globalFrame(ti(j)).rotation:inverse()
		new_normal:radd((delta*new_normal_in_B )*weights(j));
	end
	new_normal:normalize()
	return new_normal
end
-- returns signed_distance and the nearest surface point to in_global_pos
function SDFcollider:calculateNearestSurfacePoint(iloader, ibody, in_global_pos)
	assert(ibody>0)
	local lowresDetector=self.detector
	local out_normal=vector3()
	local dist=lowresDetector:calculateSignedDistance(iloader, ibody, in_global_pos, out_normal)
	local model=self.models[iloader+1]

	--dbg.erase("Line2", tostring(iloader)..'_'..tostring(ibody))
	if model.isFBX then
		local radius=model.bboxMin(ibody)*0.5 -- 마진을 준이유는 lowresDetector는 body part boundary로부터의 거리를 측정하기 때문.
		if dist<radius then
			local newdist, new_normal_in_B, voxelIndexI=self:_sampleSDF(model, ibody, in_global_pos)

			if self.options.debugDraw then
				dbg.namedDraw('Sphere', bpose_pos*100+vector3(100,0,0), string.format("%d%s", iloader,model.fbx_info.loader:bone(ibody):name()),'red_transparent',1)
				dbg.draw("Line2", bpose_pos*100+vector3(100,0,0), (bpose_pos-new_normal_in_B*newdist)*100+vector3(100,0,0), tostring(iloader)..'__'..tostring(ibody))
			end

			--if voxelIndexI and (newdist<0 or dist<0.05) then
			if voxelIndexI and newdist<0 then
				dist=newdist


				local new_normal=self:_calcNormalTowardOutside(model, voxelIndexI, new_normal_in_B)
				local new_surfacepoint=in_global_pos-new_normal*newdist
				return dist, new_surfacepoint, new_normal
			end
		end
	end
	return dist, in_global_pos-out_normal*dist, out_normal
end
function SDFcollider:isSignedDistanceSupported() 
	return true
end
function SDFcollider:isSphereTestSupported() 
	return false
end
function SDFcollider:_sampleSDF(model, ibody, position)
		local bpose_pos=model.fk_bindpose:globalFrame(ibody)*(model.fk:globalFrame(ibody):toLocalPos(position))
		local new_normal_in_B=vector3()

		local newdist, voxelIndexI=self:_calculateSignedDistanceBindPose(model, bpose_pos, new_normal_in_B)
		return newdist, new_normal_in_B, voxelIndexI
end

function SDFcollider:calculateSignedDistance(iloader, ibody, position, out_normal)
	local lowresDetector=self.detector
	local dist=lowresDetector:calculateSignedDistance(iloader, ibody, position, out_normal)
	local model=self.models[iloader+1]

	local radius=model.bboxMin(ibody)*0.5 -- 마진을 준이유는 lowresDetector는 body part boundary로부터의 거리를 측정하기 때문.
	--  예를 들어 0meter로 거리가 측정되더라도, 실제 캐릭터 "표면"부터는 더 멀 수 있음 
	local debug=false
	if dist<radius and model.isFBX then

		local newdist, new_normal_inB, voxelIndexI=self:_sampleSDF(model, ibody, position)

		if voxelIndexI then
			dist=newdist

			local new_normal=self:_calcNormalTowardOutside(model, voxelIndexI, new_normal_inB)

			out_normal:assign(new_normal)
			
			if debug then
				-- debug draw
				local vi=model.vertexIndex(voxelIndexI.x, voxelIndexI.y, voxelIndexI.z)
				local nearestVertexPos=model.fbx_info.skinningInfo:calcVertexPosition(model.fk, vi)

				dbg.draw('SphereM' , nearestVertexPos,  'nv' ,'blue')
				dbg.draw('SphereM' , position+vector3(0,0.10,0),  'sphh'..iloader..ibody,'blue')
			end
		else
			if debug then
				dbg.erase('SphereM' , 'nv' )
				dbg.draw('SphereM', position+vector3(0,0.10,0), 'sphh'..iloader..ibody, 'red')
			end
		end
	else
		if debug then
			dbg.erase('SphereM' , 'nv' )
			dbg.draw('SphereM', position+vector3(0,0.10,0),  'sphh'..iloader..ibody,'green')
		end
	end
	return dist
end

function SDFcollider:_calculateSignedDistanceBindPose(model, global_position, out_normal, dbg_draw)
	local decomp=model.decomp
	local margin=0.005 -- hard-coded in CollisionDetector_bullet.cpp
	local bmin=decomp:getBoundsMin()
	local bmax=decomp:getBoundsMax()

	local bbox_dist=model.bbox_collider:calculateSignedDistance(0, 1, global_position, out_normal)+margin

	if bbox_dist<0 then
		--local test=vector3(bmin.x*0.5+bmax.x*0.5, bmin.y*0.5+ bmax.y*0.5, bmin.z)
		--print(model.bbox_collider:calculateSignedDistance(0, 1, test , out_normal)+margin)
		--dbg.erase('Sphere', 'pp')

		return self:_calculateSignedDistanceBindPoseInsideBB(model, decomp, global_position, out_normal)
	else
		local inside_point=global_position-bbox_dist*out_normal
		local out_normal2=vector3()
		local inside_dist=self:_calculateSignedDistanceBindPoseInsideBB(model, decomp, inside_point, out_normal2)
		local surfacePoint=inside_point-out_normal2*inside_dist
		if dbg_draw then
			dbg.draw('Sphere', inside_point*100,'pp')
			dbg.draw('Sphere', surfacePoint*100,'pp2')
		end
		out_normal:assign(global_position-surfacePoint)
		out_normal:normalize()

		return inside_dist+bbox_dist
	end
end

if ConvexDecomp then
	-- save computation results to a file
	function SDFcollider:pack(model, binaryFile)
		binaryFile:packInt(-3) -- cache format version. should be a negative number
		binaryFile:_packVRMLloader(model.collisionLoader)
		binaryFile:pack(model.SDF)
		binaryFile:pack(model.normal)
		binaryFile:pack(model.vertexIndex)
		local decomp=model.decomp
		binaryFile:pack(decomp:getBoundsMin())
		binaryFile:pack(decomp:getBoundsMax())
		binaryFile:packFloat(decomp:getScale())
		binaryFile:packFloat(decomp:getVoxelScale())
		binaryFile:pack(decomp:getCenter())
	end
	-- load computation results from a file

	SkinToWRL=LUAclass()
	-- input:
	--   fbx_info can be obtained using fbxloader:createSkinningInfo()
	--   default options={ debugDraw=true, res=100, skinScale=100, wrl_config=nil }
	-- output: self.skel
	function SkinToWRL:__init(fbx_info, _options)
		-- this is extracted from testSkinToWRL.lua 
		if not _options then
			_options={}
		end
		self.options=_options
		self.fbx_info=fbx_info
		if not _options.skinScale then _options.skinScale=100 end
		if not _options.maxConvex then _options.maxConvex=6 end
		do
			-- draw mesh
			local mesh=fbx_info.mesh
			--mesh:createColorBuffer()
			mesh:calculateVertexNormal()
			local scale=_options.skinScale

			local timer=util.Timer()

			--local res=53 -- good. (approximate voxel resolution)
			local res=_options.res or 100 -- good. (approximate voxel resolution)
			--local res=8 -- for debugging 

			if _options.debugDraw then
				local meshToEntity, node=mesh:drawMesh('lightgrey_transparent', 'mesh_node')
				node:scale(scale,scale,scale)
			end

			local numVertex=mesh:numVertex()
			for i=0, mesh:numFace()-1 do
				local f=mesh:getFace(i)
				assert(f:vertexIndex(0)<numVertex)
				assert(f:vertexIndex(1)<numVertex)
				assert(f:vertexIndex(2)<numVertex)
			end
			print('nf, nv: ', mesh:numFace(), mesh:numVertex())

			do
				local vertexToBone=intvectorn(mesh:numVertex())
				local skinningInfo=fbx_info.skinningInfo
				for i=0, mesh:numVertex()-1 do
					local j=skinningInfo:treeIndices(i)(skinningInfo:weights(i):argMax())
					vertexToBone:set(i, j)
				end
				self.vertexToBone=vertexToBone
			end

			timer:start()
			local maxConvex=_options.maxConvex
			local TEST_CONVEX_DECOMPOSITION=false
			decomp=ConvexDecomp(mesh, TEST_CONVEX_DECOMPOSITION, res*res*res, maxConvex, 1)
			self.decomp=decomp
			print('voxelize:', timer:stop2()*1e-3,'ms')

			--dbg.draw('Sphere',decomp:getBoundsMin() *scale+vector3(offset_x,0,0), 'bboxMin','blue', 5)
			--dbg.draw('Sphere',decomp:getBoundsMax() *scale+vector3(offset_x,0,0), 'bboxMax','blue', 5)
			local bmin=decomp:getBoundsMin()*scale
			local bmax=decomp:getBoundsMax()*scale
			if DEBUG_DRAW then
				dbg.draw("Box", transf(quater(1,0,0,0), bmin*0.5+bmax*0.5), 'box', bmax-bmin, 1, 'lightgrey_verytransparent')
			end

			--redrawVoxels()
		end
		if true then
			local timer=util.Timer()
			timer:start()

			if true then
				timer:start()
				print("creating full-voxel graph...")
				--local fullgraph=createVoxelGraph(decomp, decomp, function(decomp, i,j,k) local t=decomp:getVoxelSafe(vector3(i,j,k)) return t==SURFACE_VOXEL or t==INTERIOR_VOXEL end)
				local fullgraph=decomp:createGraphFromAllOccupantVoxels()  -- faster c++ implementation
				local graph2=fullgraph.graph
				local nodeIndex2=fullgraph.nodeIndex
				local nodes2=fullgraph.nodes

				timer:start()
				-- 이제 본 들로부터 거리 계산.

				local loader=fbx_info.loader
				local ncluster=loader:numBone()-1
				local sourceNodeIndex=graph2:numNodes()
				for i=0, ncluster -1 do
					graph2:newNode()
				end

				local OUTER_VOXEL=2
				local INTERIOR_VOXEL=3
				local SURFACE_VOXEL=4
				local g_mesh=fbx_info.mesh
				for i=0, g_mesh:numVertex()-1 do
					local v=g_mesh:getVertex(i)
					local b=decomp:getVoxelIndex(v)
					local vtype=decomp:getVoxel(b)
					if vtype==SURFACE_VOXEL then
					elseif vtype==INTERIOR_VOXEL then
					else
						b=self:findNeighboringSurfaceVoxel(decomp, b)
					end
					if b then
						local s=sourceNodeIndex+self.vertexToBone(i)-1
						graph2:addEdge(s, nodeIndex2(b),0)
					end
				end

				local colors={}
				for i=0, ncluster-1 do
					colors[i+1]=vector3(math.random(), math.random(), math.random())
				end

				local clusterNodeDist=matrixn(ncluster, sourceNodeIndex)

				for i=0, ncluster-1 do
					local node_dist=vectorn()
					graph2:Dijkstra(sourceNodeIndex+i, node_dist)
					clusterNodeDist:row(i):assign(node_dist:slice(0, sourceNodeIndex))
				end

				print('skeletonToVoxelDist:', timer:stop2()*1e-3,'ms')
				graph2:resize(sourceNodeIndex)

				do
					local g_skel2=fbx_info.loader:copy()

					self.skel=g_skel2

					local ndim=decomp:getDimensions()
					local rigidClusters=Image3D(ndim.x,ndim.y, ndim.z, ncluster)
					for j=0, graph2:numNodes()-1 do
						local argMin=clusterNodeDist:column(j):argMin()
						assert(argMin<ncluster)

						local voxelindex=nodes2(j)
						rigidClusters:setPixel(voxelindex.x, voxelindex.y, voxelindex.z, argMin)
					end

					--for i=0, ncluster-1 do
					--	drawVoxelSubsets(rigidClusters, i, colors)
					--end

					for i=0, ncluster-1 do
						decomp:performConvexDecomposition(rigidClusters, i)
						--drawConvexHull(colors[i+1], 'rigidcluster'..i)

						local mesh=Mesh()
						for j=0, decomp:centroids():size()-1 do
							mesh:mergeMesh(mesh, decomp:convex(j))
						end
						local ti=i+1
						print(i,ti)
						if ti~=-1 then
							local bone=g_skel2:VRMLbone(ti)
							--print(bone:name())

							-- 
							--
							--mesh:transform(matrix4(bone:getFrame():inverse())) -- to local
							mesh:transform(fbx_info.bindposes[ti])
							mesh:calculateVertexNormal()


							local wrl_config=self.options.wrl_config
							if mesh:numVertex()>3 then
								if wrl_config and 
									wrl_config.useOriginalGeom(bone:treeIndex()) then
									if wrl_config.capsuleRadius and bone:hasShape() then
										local mesh_orig=bone:getMesh()
										for ielt=0, mesh_orig:numElements()-1 do
											local elt=mesh_orig:element(ielt)
											if elt.elementType==2 then
												elt.elementSize.x=wrl_config.capsuleRadius
												elt.elementSize.z=wrl_config.capsuleRadius
												if elt.elementSize.y==0 then
													-- finger tip
													local pmesh=MainLib.VRMLloader.upcast(bone:parent()):getMesh()
													local pelt=pmesh:element(pmesh:numElements()-1)
													elt.elementSize.y=pelt.elementSize.y*0.6
													elt.tf:assign(pelt.tf)
													elt.elementSize:scale(0.8)
												end
											else
												assert(false)
											end
										end
										mesh_orig:_updateMeshFromElements()
										assert(bone:getMesh():numVertex()>0)
										assert(bone:getMesh().faceGroups:size()== bone:getMesh():numElements())
										local mesh=bone:getMesh()
										for i=0, mesh:numElements()-1 do
											assert(mesh.faceGroups:endI(i)-
											mesh.faceGroups:startI(i)>0)
										end

									end

								else
									bone:createNewShape()
									bone:getMesh():assignMesh(mesh)
								end

								--print(bone:getMesh():getVertex(0))
								--print(bone:getMesh():numVertex(),
								--g_skel:VRMLbone(ti):getMesh():numVertex())
							elseif wrl_config and wrl_config.useOriginalGeom(bone:treeIndex()) then

							else
								bone:removeShape()
							end
						end

					end

				end
			end

		end

	end

	function SkinToWRL:findNeighboringSurfaceVoxel(decomp, vi)
		local x=vi.x
		local y=vi.y
		local z=vi.z

		local vv
		local function ok(vv)
			return decomp:getVoxelSafe(vv)==SURFACE_VOXEL
		end

		-- dist 1
		vv=vector3(  x-1, y,   z  )  if ok(vv) then return vv end
		vv=vector3(  x+1, y,   z  )  if ok(vv) then return vv end
		vv=vector3(  x,   y-1, z  )  if ok(vv) then return vv end
		vv=vector3(  x,   y+1, z  )  if ok(vv) then return vv end
		vv=vector3(  x,   y,   z-1)  if ok(vv) then return vv end
		vv=vector3(  x,   y,   z+1)  if ok(vv) then return vv end
		if ok(vv) then return vv end
		-- dist 2                   
		vv=vector3(  x-1, y-1, z  )  if ok(vv) then return vv end
		vv=vector3(  x+1, y-1, z  )  if ok(vv) then return vv end
		vv=vector3(  x-1, y+1, z  )  if ok(vv) then return vv end
		vv=vector3(  x+1, y+1, z  )  if ok(vv) then return vv end

		vv=vector3(  x-1, y,   z-1)  if ok(vv) then return vv end
		vv=vector3(  x+1, y,   z-1)  if ok(vv) then return vv end
		vv=vector3(  x-1, y,   z+1)  if ok(vv) then return vv end
		vv=vector3(  x+1, y,   z+1)  if ok(vv) then return vv end

		vv=vector3(  x  , y-1, z-1)  if ok(vv) then return vv end
		vv=vector3(  x,   y+1, z-1)  if ok(vv) then return vv end
		vv=vector3(  x,   y-1, z+1)  if ok(vv) then return vv end
		vv=vector3(  x,   y+1, z+1)  if ok(vv) then return vv end

		-- dist 3                    
		vv=vector3(  x-1, y-1, z-1)  if ok(vv) then return vv end
		vv=vector3(  x+1, y-1, z-1)  if ok(vv) then return vv end
		vv=vector3(  x-1, y+1, z-1)  if ok(vv) then return vv end
		vv=vector3(  x+1, y+1, z-1)  if ok(vv) then return vv end

		vv=vector3(  x-1, y-1, z+1)  if ok(vv) then return vv end
		vv=vector3(  x+1, y-1, z+1)  if ok(vv) then return vv end
		vv=vector3(  x-1, y+1, z+1)  if ok(vv) then return vv end
		vv=vector3(  x+1, y+1, z+1)  if ok(vv) then return vv end

		return nil
	end
end
