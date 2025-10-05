rootnode =RE.ogreRootSceneNode()
bgnode=RE.createChildSceneNode(rootnode , "BackgroundNode")

--RE.ogreSceneManager():setSkyBox(true, "My/SkyBox2") -- cloudy_noon

pNode=RE.createChildSceneNode(bgnode, "terrain")
entity=RE.createTerrain("_entity_terrain", "../Resource/crowdEditingScripts/terrain/heightmap1_256_256_2bytes.raw", 256, 256, 4000, 4000, 200,1,1)
pNode:setPosition(1997.775635, 0.000000, 2001.514038)
pNode:setOrientation(0.000000, 0.000000, 1.000000, 0.000000)
pNode:setScale(1.000000, 1.000000, 1.000000)
entity:setMaterialName("CrowdEdit/Terrain1")
entity:setCastShadows(false)
entity:setNormaliseNormals(true)
pNode:attachObject(entity)

