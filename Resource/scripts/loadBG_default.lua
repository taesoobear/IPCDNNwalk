
dofile('../Resource/scripts/ogreConfig.lua')

rootnode =RE.ogreRootSceneNode()
if rootnode then
bgnode=RE.createChildSceneNode(rootnode , "BackgroundNode")
end

dofile("../Resource/scripts/createLight_default.lua")

--ent= RE.createPlane("MyPlane", 16000, 16000, 80, 80, 80, 80)
--repeat 16m x 16m floor nrep times.
local nrep=2
ent= RE.createPlane("MyPlane", 1600*nrep, 1600*nrep, nrep, nrep, 8*nrep, 8*nrep)

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
if depthShadow then
	ent:setMaterialName('Ogre/DepthShadowmap/Receiver/RockWall/PCF')
else
	--ent:setMaterialName("two/Twocharacter")
	ent:setMaterialName("checkboard/crowdEditing")
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
	RE.ogreSceneManager():setFog( 0.77,0.92,1, 0.0,1200, 1800 )
end


--RE.viewpoint():setFOVy(40)
--RE.viewpoint().vpos:assign({50, 20, 30})
--RE.viewpoint().vat:assign({0,20,0})
--RE.viewpoint():update()
