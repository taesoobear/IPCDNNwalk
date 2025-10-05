
require('config')
require('module')

local osm=RE.ogreSceneManager()
if osm:hasSceneNode("BackgroundNode") then
	local angle=math.rad(33)
	local lightnode=osm:getSceneNode("BackgroundNode")
	lightnode:rotate(quater(angle, vector3(0,1,0)))
end
