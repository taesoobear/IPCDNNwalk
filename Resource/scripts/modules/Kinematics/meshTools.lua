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

function Mesh:drawMesh(materialName, _optionalNodeName, _optionalDoNotUseNormal)
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
	if entity then
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
