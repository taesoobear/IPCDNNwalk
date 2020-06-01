RE.removeEntity(RE.ogreSceneManager():getSceneNode("BackgroundNode"))
node =RE.ogreRootSceneNode()
bgnode =node:createChildSceneNode("BackgroundNode")
node2=bgnode:createChildSceneNode()
--ent1= RE.ogreSceneManager():createEntity("BG2_floor2", "Plane01.mesh")
--node2:attachObject(ent1)

for i=2, 177 do
	local ent
	if i<10 then
		ent=RE.ogreSceneManager():createEntity("BG2_floor2_"..i, "Box0"..i..".mesh")
		node2:attachObject(ent)
	else
		local fn="Box"..i..".mesh"
		if isFileExist("../media/models/"..fn)==1 then
			ent=RE.ogreSceneManager():createEntity("BG2_floor2_"..i,fn)
			node2:attachObject(ent)	
		else
			print(fn.." does not exist")
		end
	end

end

