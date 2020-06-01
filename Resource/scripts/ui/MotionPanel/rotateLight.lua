
require('config')
require('module')
local osm=RE.ogreSceneManager()
if osm:hasSceneNode("LightNode") then
	local angle=math.rad(90)
	local lightnode=osm:getSceneNode("LightNode")
	lightnode:rotate(quater(angle, vector3(0,1,0)))
end
