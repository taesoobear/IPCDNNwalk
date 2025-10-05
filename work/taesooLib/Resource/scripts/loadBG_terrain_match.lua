rootnode =RE.ogreRootSceneNode()
bgnode=RE.createChildSceneNode(rootnode , "BackgroundNode")

pNode=RE.createChildSceneNode(bgnode, "terrain")
entity=RE.createTerrain("_entity_terrain", "../Resource/crowdEditingScripts/terrain/heightmap1_256_256_2bytes.raw", 256, 256, 1000, 1000, 100,1,1)
pNode:setPosition(-1810.202515, -0.000000, 1275.184692)
pNode:setOrientation(0.728272, 0.000000, 0.685289, 0.000000)
pNode:setScale(2.910000, 2.910000, 2.910000)
entity:setMaterialName("CrowdEdit/Terrain1")
entity:setCastShadows(false)
entity:setNormaliseNormals(true)
pNode:attachObject(entity)

