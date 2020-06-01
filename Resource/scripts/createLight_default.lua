dofile('../Resource/scripts/ogreConfig.lua')

rootnode =RE.ogreRootSceneNode()
if rootnode then
lightnode=RE.createChildSceneNode(rootnode, "LightNode")
RE.ogreSceneManager():setAmbientLight(0.4, 0.4, 0.4)

if RE.getOgreVersionMinor()==12 then


	local function randomNormal()
		return (math.random()-0.5)*2
	end


	local numMainLights=10
	if depthShadow then
		--numMainLights=1 -- no difference
	end
	local lightVar=0.01
	
	local sc=math.pow(0.5, 1/numMainLights)
	local light1D=0.8
	local light1S=0.2
	local lightOD=0
	local lightOS=0
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
	end

	if highQualityRendering then
		-- high-quality
		numMainLights=100
		lightVar=0.04

		if stencilShadow then
			sc=math.pow(0.5, 1/numMainLights)
		else
			lightVar=0.2
			sc=0.99
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

	RE.ogreSceneManager():setShadowColour(sc,sc,sc)


	for i=1,numMainLights do
		local light
		if i==1 then
			light=RE.ogreSceneManager():createLight("Mainlight")
		else
			light=RE.ogreSceneManager():createLight("Mainlight"..i)
		end
		light:setType("LT_DIRECTIONAL")
		light:setDirection(-0.5+lightVar*(randomNormal()),-0.7,0.5+lightVar*(randomNormal()))
		if i==1 then
			light:setDiffuseColour(light1D,light1D,light1D)
			light:setSpecularColour(light1S,light1S,light1S)
		else
			light:setDiffuseColour(lightOD,lightOD,lightOD)
			light:setSpecularColour(lightOS,lightOS,lightOS)
		end
		light:setCastShadows(true)
		lightnode:attachObject(light)
	end
else
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
