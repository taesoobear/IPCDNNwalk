scene={
    {
        bCastShadow =true,
        bNormaliseNormals =true,
        material ="",
        nodeId ="entity000",
        options =CT.ivec(),
        ori =quater(1,0,0,0),
        pos =vector3(0,0,0),
        scType =3,
        scale =vector3(1,1,1),
        source ="h11.mesh",
    }, {
        bCastShadow =true,
        bNormaliseNormals =true,
        material ="",
        nodeId ="entity001",
        options =CT.ivec(),
        ori =quater(1,0,0,0),
        pos =vector3(137.97200185711,0,-4.4856770104474),
        scType =3,
        scale =vector3(1,1,1),
        source ="h11.mesh",
    }, {
        bCastShadow =true,
        bNormaliseNormals =true,
        material ="",
        nodeId ="entity002",
        options =CT.ivec(),
        ori =quater(1,0,0,0),
        pos =vector3(0,0,0),
        scType =3,
        scale =vector3(1,1,1),
        source ="h11.mesh",
    }, {
        bCastShadow =true,
        bNormaliseNormals =true,
        material ="",
        nodeId ="entity003",
        options =CT.ivec(),
        ori =quater(1,0,0,0),
        pos =vector3(0,0,0),
        scType =3,
        scale =vector3(1,1,1),
        source ="h11.mesh",
    }, 
}
function loadScene(scene)
	local rootnode=RE.ogreRootSceneNode()
	local bgnode=RE.createChildSceneNode(rootnode,"backgroundNode")

	for i, info in ipairs(scene) do
		local nodeId=info.nodeId..'__'
		local pNode=RE.createChildSceneNode(bgnode, nodeId)
		if (info.scType==SceneComponent.PLANE) then
			if (info.options:size()==4) then
				entity=RE.createPlane("_entity_".. nodeId,
				info.options[0], info.options[1], info.options[2], info.options[3])
			else
				entity=RE.createPlane("_entity_"..nodeId,
				info.options[0], info.options[1], info.options[2], info.options[3], info.options[4], info.options[5])
			end
		elseif(info.scType==SceneComponent.ENTITY) then
			entity=RE.ogreSceneManager():createEntity("_entity_"..nodeId,info.source)
		else
			entity=RE.createTerrain("_entity_"..nodeId,info.source,info.options[0],info.options[1],info.options[2],info.options[3],info.options[4],info.options[5],info.options[6])
		end
		pNode:setPosition(info.pos.x,info.pos.y,info.pos.z)
		pNode:setOrientation(info.ori.w,info.ori.x,info.ori.y,info.ori.z)
		pNode:setScale(info.scale.x,info.scale.y,info.scale.z)

		if info.material~='' then
			entity:setMaterialName(info.material)
		end
		if not (info.bCastShadow) then
			entity:setCastShadows(false)
		end
		if(info.bNormaliseNormals) then
			entity:setNormaliseNormals(true)
		end

		pNode:attachObject(entity)
	end
	--bgnode:setPosition(100,0,0)
end
