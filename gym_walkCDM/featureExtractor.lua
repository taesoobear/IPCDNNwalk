local M={}
function M.extractFeature(sp, COM, ZMP, ANG, motionType, sf, ef, useTerrain)
	local features=matrixn()
	local pendtrajR=quaterN(ef-sf+1)
	local pendtrajT=vector3N(ef-sf+1)
	local pendtrajDotT=vector3N(ef-sf+1)

	--local mf=math.round(sf*0.5+ef*0.5)
	local mf=sf
	local pendROOTQ0=ANG:row(mf):toQuater(0)
	local pendCOM0=COM:row(mf):toVector3(0)
	pendCOM0.y=0
	local pend2Dtf0=transf(pendROOTQ0:rotationY(), pendCOM0)

	local refTerrainHeight=0
	if useTerrain then
		local h1= g_terrain:_getHeight(ZMP:row(sf):toVector3(0))
		local h2= g_terrain:_getHeight(ZMP:row(math.round(sf*0.5+ef*0.5)):toVector3(0))
		local h3= g_terrain:_getHeight(ZMP:row(ef):toVector3(0))
		refTerrainHeight=(h1+h2+h3)/3.0
	end

	local regressor -- for terrain height

	do 
		local numPointsD2=3
		local numPoints=numPointsD2*3
		local source=matrixn(numPoints, 2)
		local target=matrixn(numPoints, 1)
		--local terref=sf*0.5+ef*0.5 -- use about 50% of a stride
		local terref=sf*0.3+ef*0.7 -- use about 70% of a stride
		for i=0, numPointsD2-1 do
			local terrainHeightL=0
			local terrainHeightR=0

			local f=sop.map(i,0,numPointsD2-1, sf, terref)

			local pendROOTQ=vectorn()
			ANG:sub(0,0,0,4):sampleRow(f, pendROOTQ)
			pendROOTQ=pendROOTQ:toQuater(0)

			local ZMPpos=vectorn()
			ZMP:sub(0,0,0,3):sampleRow(f, ZMPpos)
			ZMPpos=ZMPpos:toVector3(0)

			function setSourceTarget(irow, p)
				source:row(irow):set(0, p.x)
				source:row(irow):set(1, p.z)
				target:row(irow):set(0, p.y)
			end

			function getPos(ZMPpos, pendROOTQ, lpos)
				local p1=ZMPpos+pendROOTQ:rotationY()*lpos
				if useTerrain then
					local out=vector3()
					g_terrain:getTerrainHeight(p1, out)
					p1=out
				end
				p1.y=p1.y+math.random()*0.01
				return p1
			end

			setSourceTarget(i*3, getPos(ZMPpos, pendROOTQ, vector3(0.2,0,0)))
			setSourceTarget(i*3+1, getPos(ZMPpos, pendROOTQ, vector3(-0.2,0,0)))
			setSourceTarget(i*3+2, getPos(ZMPpos, pendROOTQ, vector3(0,0,0.2)))
		end
		regressor=BayesianLinearRegression(1, 1e-4)
		--regressor=LinearFunction()
		--local regressor=BayesianLinearRegression(2, 0.01) -- linear regression with a noise prior
		regressor:learn(source, target)
	end
	-- to retrieve approximate terrainheight
	--	local v=vectorn(1)
	--	regressor:mapping(CT.vec(x,z), v)
	--	pos= vector3(x, v(0), z)
	
	local nleg=#sp
	for iframe=sf, ef do
		-- normalized phase
		local f={}
		for ileg=1,nleg do
			local phase=sp[ileg](iframe)
			--f[ileg]=CT.vec( math.sin(phase*(2*math.pi)), math.sin(phase*(2*math.pi)+0.5*math.pi), math.sin(phase*(2*math.pi)+1.0*math.pi), math.sin(phase*(2*math.pi)+1.5*math.pi)) 
			f[ileg]=CT.vec( math.sin(phase*(2*math.pi)), math.cos(phase*(2*math.pi)))
		end

		local terrainHeightL=0
		local terrainHeightR=0

		local pendROOTQ=ANG:row(iframe):toQuater(0)

		if useTerrain then
			local out=vector3()
			g_terrain:getTerrainHeight(ZMP:row(iframe):toVector3(0)+pendROOTQ:rotationY()*vector3(0.1,0,0), out)
			terrainHeightR=out.y
			g_terrain:getTerrainHeight(ZMP:row(iframe):toVector3(0)+pendROOTQ:rotationY()*vector3(-0.1,0,0), out)
			terrainHeightL=out.y
		end


		local ffeature
		if nleg==2 then
			ffeature=f[1]..f[2]..CT.vec(ef-sf, motionType(iframe), terrainHeightL-refTerrainHeight, terrainHeightR-refTerrainHeight)
		elseif nleg==4 then
			ffeature=f[1]..f[2]..f[3]..f[4]..CT.vec(ef-sf, terrainHeightL-refTerrainHeight, terrainHeightR-refTerrainHeight)
		else
			assert(false)
		end
		features:pushBack(ffeature)

		local pendCOM=COM:row(iframe):toVector3(0)

		local pendtf=transf(pendROOTQ, pendCOM)
		-- G=G0*delta -> delta=G0:inverse()*G
		local lpendtf=pend2Dtf0:inverse()*pendtf
		pendtrajR(iframe-sf):assign(lpendtf.rotation)
		pendtrajT(iframe-sf):assign(lpendtf.translation)
		pendtrajDotT(iframe-sf):assign(pend2Dtf0.rotation:inverse()*COM:row(iframe):toVector3(3))
	end
	pendtrajR(0):align(quater(1,0,0,0))
	pendtrajR:align()
	assert(features:rows()==pendtrajR:size())

	local mfeature=features..pendtrajR:matView()..pendtrajT:matView()..pendtrajDotT:matView()
	local mfeature2=matrixn()
	local numSample=5
	mfeature2:resample(mfeature, numSample)
	return mfeature2:toVector(), refTerrainHeight, regressor
end
function LinearFunction:getHeight(p)
	local v=vectorn(1)
	self:mapping(CT.vec(p.x,p.z), v)
	return v(0)
end
BayesianLinearRegression.getHeight=LinearFunction.getHeight
Terrain=OBJloader.Terrain
function Terrain:getTerrainHeight( xPos, out)
	local x=xPos.x*100-g_terrainPos.x
	local z=xPos.z*100-g_terrainPos.z
	local p=vector2()
	p:set(0,x)
	p:set(1,z)
	local h=self:height( p)
	out.x=x+g_terrainPos.x
	out.y=h
	out.z=z+g_terrainPos.z
	out:scale(1/100.0)
end
function Terrain:_getHeight( xPos)
	local x=xPos.x*100-g_terrainPos.x
	local z=xPos.z*100-g_terrainPos.z
	local p=vector2()
	p:set(0,x)
	p:set(1,z)
	local h=self:height( p)
	return h*0.01
end

function _createTerrain()
	local s=18
	local h=1.3
	if true then
		local mesh=OBJloader.Terrain("../Resource/crowdEditingScripts/terrain/heightmap1_256_256_2bytes.raw", 256, 256, s*100,s*100,h*100,1,1)
		g_terrain=mesh
		-- scale 100 for rendering 
		local meshToEntity=MeshToEntity(mesh, 'meshName')
		--meshToEntity:updatePositionsAndNormals()
		local entity=meshToEntity:createEntity('entityName' )
		entity:setMaterialName("CrowdEdit/Terrain1")
		--entity:setMaterialName("red")
		--entity:setMaterialName("checkboard/crowdEditink")
		local node=RE.createChildSceneNode(RE.ogreRootSceneNode(), "mesh_node")
		node:attachObject(entity)
		node:translate(-s*50,0.8,-s*50)
		g_terrainPos=vector3(-s*50,0.8,-s*50)
	end
end
return M
