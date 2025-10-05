RE.removeEntity(RE.ogreSceneManager():getSceneNode("BackgroundNode"))
node =RE.ogreRootSceneNode()
bgnode =node:createChildSceneNode("BackgroundNode")
node2=bgnode:createChildSceneNode()
ent1= RE.ogreSceneManager():createEntity(RE.generateUniqueName(), "pave.mesh")
node2:attachObject(ent1)

for i=1, 28 do
	local ent
	if i<10 then
		ent=RE.ogreSceneManager():createEntity(RE.generateUniqueName(), "stone0"..i..".mesh")
	else
		ent=RE.ogreSceneManager():createEntity(RE.generateUniqueName(), "stone"..i..".mesh")
	end
	node2:attachObject(ent)
end

node2:translate(-230,0,232)
bgnode:scale(3,3,3)

--[[
// Floor plane
				Ogre::Plane plane;
				plane.normal = Ogre::Vector3::UNIT_Y;
				plane.d = -0.5;
				Ogre::MeshManager::getSingleton().createPlane("Myplane",
					Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, plane,
					200,200,20,20,true,1,1,1,plane.normal );

				Ogre::Entity* pPlaneEnt = win.m_Renderer->ogreRenderer().mScene->createEntity( "plane", "Myplane" );

				pPlaneEnt->setMaterialName("Ground_Sand");
				pPlaneEnt->setCastShadows(false);

				//	pPlaneEnt ->setVisible(false);
				pNode->attachObject(pPlaneEnt);
]]--
