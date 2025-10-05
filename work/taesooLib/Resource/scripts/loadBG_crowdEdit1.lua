RE.viewpoint():setFOVy(30.000006)
RE.viewpoint().vpos:assign({54.500742, 576.508876, -193.703920})
RE.viewpoint().vat:assign({54.500748, 0.003382, -193.713983})
RE.viewpoint():update()

rootnode =RE.ogreRootSceneNode()
bgnode=RE.createChildSceneNode(rootnode , "BackgroundNode")

pNode=RE.createChildSceneNode(bgnode, "plane")
entity=RE.createPlane("_entity__entity_plane", 4000, 4000, 20, 20, 1,1)
pNode:setPosition(0.000000, 0.000000, 0.000000)
pNode:setOrientation(1.000000, 0.000000, 0.000000, 0.000000)
pNode:setScale(1.000000, 1.000000, 1.000000)
entity:setMaterialName("lightgrey")
pNode:attachObject(entity)

