
if RE.taesooLibPath then -- for backward binary compatibility.
	taesooLibPath=RE.taesooLibPath()
else
	taesooLibPath='../'
end

local f=io.open(taesooLibPath..'Resource/scripts/ogreConfig_personal.lua','r')
if f then
	f:close()
	dofile(taesooLibPath..'Resource/scripts/ogreConfig_personal.lua')
else
	dofile(taesooLibPath..'Resource/scripts/ogreConfig.lua')
end

rootnode =RE.ogreRootSceneNode()
if rootnode then
lightnode=RE.createChildSceneNode(rootnode, "LightNode")

if RE.getOgreVersionMinor()>=12 then
	RE.ogreSceneManager():setAmbientLight(0.4, 0.4, 0.4)

	local function randomNormal()
		return (math.random()-0.5)*2
	end

	local textureShadow=false
	if not stencilShadow and not depthShadow then
		textureShadow=true
	end
	local lightVar=0.02
	
	local sc=math.pow(0.5, 1/numMainLights)
	local light1D=0.9
	local light1S=0.8
	local lightOD=0.0
	local lightOS=0.0
	local highQualityRendering=false -- set this true for high quality render

	if not stencilShadow then
		lightVar=0.1
		if depthShadow then
			light1D=0.8/numMainLights
			light1S=0.2/numMainLights
			lightOD=0.8/numMainLights
			lightOS=0.2/numMainLights
		end
		sc=0.9
		if textureShadow then
			sc=0.995
			highQualityRendering=true
		end
	else
		sc=0.975
	end

	if highQualityRendering then
		-- high-quality
		numMainLights=100
		lightVar=0.04

		if stencilShadow then
			sc=math.pow(0.5, 1/numMainLights)
		else
			if depthShadow then
				lightVar=0.2
				sc=0.99
			else
				numMainLights=100
				lightVar=0.3
				sc=0.998
			end
			RE.ogreSceneManager():setShadowTextureCount(numMainLights)
		end
		local function randomNormal()
			while true do
				local u = 2 * math.random() - 1;
				local v = 2 * math.random() - 1;
				local w = math.pow(u, 2) + math.pow(v, 2);
				if w<1 then
					local z = math.sqrt((-2 * math.log(w)) / w);
					local x = u * z;
					local y = v * z;
					return x
				end
			end
		end
	end
	if g_customShadowColor then
		sc=g_customShadowColor
	end

	RE.ogreSceneManager():setShadowColour(sc,sc,sc)


	for i=1,numMainLights do
		local light
		if i==1 then
			light=RE.ogreSceneManager():createLight("Mainlight")
		else
			light=RE.ogreSceneManager():createLight("Mainlight"..i)
		end
		light:setType("LT_DIRECTIONAL")

		if RE.getOgreVersionMinor()==12 then
			light:setDirection(-0.5+lightVar*(randomNormal()),-0.7,0.5+lightVar*(randomNormal()))
			lightnode:attachObject(light)
		else
			local node=lightnode:createChildSceneNode("mainlightnode"..i)
			node:setDirection(vector3(-0.5+lightVar*(randomNormal()),-0.7,0.5+lightVar*(randomNormal())))
			node:attachObject(light)
		end
		if i==1 then
			light:setDiffuseColour(light1D,light1D,light1D)
			light:setSpecularColour(light1S,light1S,light1S)
		else
			light:setDiffuseColour(lightOD,lightOD,lightOD)
			light:setSpecularColour(lightOS,lightOS,lightOS)
		end
		light:setCastShadows(true)
		
	end
	light=RE.ogreSceneManager():createLight("FillLight")
	light:setType("LT_DIRECTIONAL")
	light:setDirection(0.5,0.7,-0.5)
	light:setDiffuseColour(0.4,0.4,0.4)
	light:setSpecularColour(0.4,0.4,0.4)
	light:setCastShadows(false)
	lightnode:attachObject(light)
elseif RE.getOgreVersionMinor()<=3 then

	if RE.renderer().getConfig then
		local st=RE.renderer():getConfig('shadowTechnique')
		if st==0 then
			-- no shadow
			RE.ogreSceneManager():setAmbientLight(0.4, 0.4, 0.4)
			light=RE.ogreSceneManager():createLight("Mainlight")
			light:setType("LT_DIRECTIONAL")
			light:setDiffuseColour(2.6,2.6,2.6)
			light:setSpecularColour(0.2,0.2,0.2)

			local node=lightnode:createChildSceneNode("mainlightnode")
			local dir=vector3(-0.5,-0.7,0.5)
			dir:normalize()
			node:setDirection(dir)
			node:attachObject(light)
			return
		end
	end

	local function randomNormal()
		return (math.random()-0.5)*2
	end

	numMainLights=5
	local lightVar=0.02

	local light1D=1.4
	local light1S=0
	local lightOD=0.0
	local lightOS=0.0
	local lightFD=0.3
	local lightFS=0

	RE.ogreSceneManager():setAmbientLight(0.4, 0.4, 0.4)
	for i=1,numMainLights do
		local light
		if i==1 then
			light=RE.ogreSceneManager():createLight("Mainlight")
		else
			light=RE.ogreSceneManager():createLight("Mainlight"..i)
		end
		light:setType("LT_DIRECTIONAL")

		local node=lightnode:createChildSceneNode("mainlightnode"..i)
		local dir=vector3(-0.5+lightVar*(randomNormal()),-0.7,0.5+lightVar*(randomNormal()))
		dir:normalize()
		node:setDirection(dir)
		node:attachObject(light)
		light:setDiffuseColour(light1D,light1D,light1D)
		light:setSpecularColour(light1S,light1S,light1S)
		light:setCastShadows(true)
	end
	light=RE.ogreSceneManager():createLight("FillLight")
	local node=lightnode:createChildSceneNode("filllightnode")
	local dir=vector3(0.5,0.7,-0.5)
	dir:normalize()
	node:setDirection(dir)
	node:attachObject(light)
	light:setType("LT_DIRECTIONAL")
	light:setDiffuseColour(lightFD,lightFD,lightFD)
	light:setSpecularColour(lightFS,lightFS,lightFS)
	light:setCastShadows(false)
	--light=RE.ogreSceneManager():createLight("FillLight")
	--light:setType("LT_DIRECTIONAL")
	--light:setDirection(0.5,0.7,-0.5)
	--light:setDiffuseColour(0.4,0.4,0.4)
	--light:setSpecularColour(0.4,0.4,0.4)
	--light:setCastShadows(false)
	--lightnode:attachObject(light)

	RE.ogreSceneManager():getSceneNode("LightNode"):setOrientation(quater(math.rad(180), vector3(0,1,0)))
	local useESM=false
	RE.renderer():setupShadowNode(useESM)
else
	RE.ogreSceneManager():setAmbientLight(0.4, 0.4, 0.4)
	light=RE.ogreSceneManager():createLight("Mainlight")
	light:setType("LT_DIRECTIONAL")
	light:setDirection(-0.5,-0.7,0.5)
	light:setDiffuseColour(0.8,0.8,0.8)
	light:setSpecularColour(0.2,0.2,0.2)
	light:setCastShadows(true)
	lightnode:attachObject(light)

	light=RE.ogreSceneManager():createLight("FillLight")
	light:setType("LT_DIRECTIONAL")
	light:setDirection(0.6,-0.5,-0.4)
	light:setDiffuseColour(0.2,0.2,0.2)
	light:setSpecularColour(0,0,0)
	light:setCastShadows(false)
	lightnode:attachObject(light)
end

-- light=RE.ogreSceneManager():createLight("BackLight")
-- light:setType("LT_DIRECTIONAL")
-- light:setDirection(-0.4,-0.5,0.6)
-- light:setDiffuseColour(0.2,0.2,0.2)
-- light:setSpecularColour(0,0,0)
-- light:setCastShadows(false)
-- lightnode:attachObject(light)

-- light=RE.ogreSceneManager():createLight("BackLight2") -- ?׸??? ?׸??? ???츸 ?????? ??.
-- light:setType("LT_DIRECTIONAL")
-- light:setDirection(0.5,0.7,0.5)
-- light:setDiffuseColour(0.2,0.2,0.2)
-- light:setSpecularColour(0,0,0)
-- light:setCastShadows(false)
-- lightnode:attachObject(light)
end
