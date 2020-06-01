RE.viewpoint():setFOVy(45.000002)
RE.viewpoint().vpos:assign({91.281395, 229.982034, 414.878678})
RE.viewpoint().vat:assign({82.322077, -13.831379, 6.450898})
RE.viewpoint():update()

rootnode =RE.ogreRootSceneNode()
bgnode=RE.createChildSceneNode(rootnode , "BackgroundNode")

for i=1, 6 do
	for j=1, 7 do
		pNode=RE.createChildSceneNode(bgnode, "h"..i..j)
		entity=RE.ogreSceneManager():createEntity("_entity_h"..i..j, "h"..i..j..".mesh")
		pNode:setPosition(200*i, 50.000000, 200*j)
		pNode:setOrientation(1.000000, 0.000000, 0.000000, 0.000000)
		pNode:setScale(1.000000, 1.000000, 1.000000)
		entity:setNormaliseNormals(true)
		entity:setCastShadows(true)
		pNode:attachObject(entity)
	end
end

		pNode=RE.createChildSceneNode(bgnode, "mountin")
		entity=RE.ogreSceneManager():createEntity("_entity_mountin", "mountin.mesh")
		pNode:setPosition(0, -200.000000, 0)
		pNode:setOrientation(1.000000, 0.000000, 0.000000, 0.000000)
		pNode:setScale(1.000000, 1.000000, 1.000000)
		entity:setNormaliseNormals(true)
		pNode:attachObject(entity)
