
--ent= RE.createPlane("MyPlane", 16000, 16000, 80, 80, 80, 80)
--repeat 16m x 16m floor nrep times.
local nrep=2
ent= RE.createPlane("MyPlane2", 1600*nrep, 1600*nrep, nrep, nrep, 8*nrep, 8*nrep)

--ent:setMaterialName("two/Twocharacter")
ent:setMaterialName("invisible_shadow_receiver")
ent:setCastShadows(false)

bgnodeOrig=RE.ogreSceneManager():getSceneNode("BackgroundNode")
bgnodeOrig:setVisible(false)

local bgnode=RE.createChildSceneNode(RE.ogreRootSceneNode(), "InvisibleBackgroundNode")

bgnode:attachObject(ent)
bgnode:translate(0,0,0)

