if RE.taesooLibPath then -- for backward binary compatibility.
	taesooLibPath=RE.taesooLibPath()
else
	taesooLibPath='../'
end

dofile(taesooLibPath..'Resource/scripts/ogreConfig.lua')


rootnode =RE.ogreRootSceneNode()
if rootnode then
	bgnode=RE.createChildSceneNode(rootnode , "BackgroundNode")
end

dofile(taesooLibPath.."Resource/scripts/createLight_default.lua")

--ent= RE.createPlane("MyPlane", 16000, 16000, 80, 80, 80, 80)
--repeat 16m x 16m floor nrep times.
if RE.getOgreVersionMinor()<=3 then

	if true then
		local nrep=3
		local rep=2
		ent= RE.createPlane("MyPlane", 3200*rep*nrep, 3200*rep*nrep, nrep, nrep, 16*rep*nrep, 16*rep*nrep)
		ent:setMaterialName("checkboard/crowdEditing")
	elseif false then
		local nrep=1
		ent= RE.createPlane("MyPlane", 400*nrep, 400*nrep, nrep, nrep, 2*nrep, 2*nrep)
		ent:setMaterialName("checkboard/crowdEditing")
	else
		-- same setting
		ent= RE.createPlane("MyPlane", 500, 500, 1, 1, 4, 4)
	end

	bgnode:attachObject(ent)
	bgnode:setPosition(vector3(0,0,0))
	return 
else
	local nrep=3
	ent= RE.createPlane("MyPlane", 1600*nrep, 1600*nrep, nrep, nrep, 8*nrep, 8*nrep)
end

-- 바닥판을 무한히 넓게 하려면, 위의 nrep을 늘리지 말고,
-- 캐릭터 위치에 따라 바닥판을 이동시키시오(패턴이 4미터 간격으로 반복된다는 가정)
-- 아래코드 참조
if false then
	-- translate the floor every 4m
	-- com_pos :  character position in meter unit.
	local gridposz=math.floor(com_pos.z/4)
	local gridposx=math.floor(com_pos.x/4)
	bgnode=RE.ogreSceneManager():getSceneNode("BackgroundNode")
	bgnode:setPosition(vector3(gridposx*4*100,0,gridposz*4*100))
end
if not ent then return end
if true then
	--ent:setMaterialName("two/Twocharacter")
	ent:setMaterialName("checkboard/crowdEditing")

	function os.isWindows()
		local isWin=string.find(string.lower(os.getenv('OS') or 'nil'),'windows')~=nil
		return isWin 
	end
	if os.isWindows() then
		-- python3 에서 사용시 코덱 로딩이 안되서 자주 쓰이는 material수작업 로딩 함.
		local image=CImage()
		image:Load(taesooLibPath..'media/materials/textures/crowdEditing.tga')
		RE.renderer():_updateDynamicTexture('crowdEditing.tga', image, false)
		RE.renderer():_linkMaterialAndTexture('checkboard/crowdEditing', 'crowdEditing.tga')

		image:Load(taesooLibPath..'media/materials/textures/BlueCircle.png')
		RE.renderer():_updateDynamicTexture('BlueCircle.png', image, false)
		RE.renderer():_linkMaterialAndTexture('blueCircle', 'BlueCircle.png')

		image:Load(taesooLibPath..'media/materials/textures/RedCircle.png')
		RE.renderer():_updateDynamicTexture('RedCircle.png', image, false)
		RE.renderer():_linkMaterialAndTexture('redCircle', 'RedCircle.png')
	end
end
--ent:setMaterialName("Examples/Rockwall")
ent:setCastShadows(false)

if bgnode then
bgnode:attachObject(ent)
bgnode:translate(0,0,0)
end


--RE.ogreSceneManager():setSkyBox(true, "My/SkyBox2") -- cloudy_noon
--RE.ogreSceneManager():setSkyBox(true, "My/SkyBox") -- evening
--RE.ogreSceneManager():setSkyBox(false, "SkyBox") -- no skybox
RE.setBackgroundColour(0.77, 0.92, 1.000)
if bgnode and RE.ogreSceneManager().setFog then
	local fogMin=1600
	RE.ogreSceneManager():setFog( 0.77,0.92,1, 0.0,fogMin, 3500 )
end


--RE.viewpoint():setFOVy(40)
--RE.viewpoint().vpos:assign({50, 20, 30})
--RE.viewpoint().vat:assign({0,20,0})
--RE.viewpoint():update()
