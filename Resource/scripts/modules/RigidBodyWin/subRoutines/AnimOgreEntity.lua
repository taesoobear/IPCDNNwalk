require('subRoutines/OgreLoader')
-- deprecated.
-- only for backward compatibility
AnimOgreEntity=LUAclass(OgreLoader)

function AnimOgreEntity:__init(meshFileName, entityScale, _optional_skinScale, _optional_skelScale, _buildEdgeList)
	OgreLoader:__init(meshFileName, entityScale, _optional_skinScale, _optional_skelScale, _buildEdgeList)

	self.trans=vector3(0,0,0)
	self.ogreNode=RE.createChildSceneNode(RE.ogreRootSceneNode(),self.entityName.."_raw_node")
	self.ogreNode:attachObject(self.ogreEntity)
end

function AnimOgreEntity:dtor()
	RE.removeEntity(self.ogreNode)
end

function AnimOgreEntity.createVRMLloader(meshFileName, entityScale, _optional_skinScale, _optional_skelScale) 
	local temp=AnimOgreEntity(meshFileName, entityScale, _optional_skinScale, _optional_skelScale) 
	local loader=temp.loader
	local sf=temp.sf

	temp:__deinit()


	return  loader, sf
end
function AnimOgreEntity.saveVRMLloader(meshFileName, wrlFile, entityScale, _optional_skinScale, _optional_skelScale) 

	local temp=AnimOgreEntity(meshFileName, entityScale, _optional_skinScale, _optional_skelScale) 
	local loader=temp.loader
	local sf=temp.sf

	temp:__deinit()
	loader:export(wrlFile)
end

function AnimOgreEntity:__deinit() -- needs to be called manually when necessary
	RE.removeEntity(self.ogreNode)
end

function AnimOgreEntity:startDebugMode()
	if not self.skin then
		self.skin=RE.createVRMLskin(self.loader,false);
		local s=self.scale
		self.skin:setScale(s,s,s)
		self.ogreEntity:setMaterialName("lightgrey_transparent")
		self.skin:setMaterial('grey')
	end
end

function AnimOgreEntity:isDebugMode()
	return self.skin~=nil
end

function AnimOgreEntity:setMaterial(name)
	self.ogreEntity:setMaterialName(name)
end

function AnimOgreEntity:setTranslation(x,y,z)
	-- not implmented yet
	self.trans.x=x
	self.trans.y=y
	self.trans.z=z
	self.ogreNode:setPosition(self.trans)
end

function AnimOgreEntity:setScale(x,y,z)
	-- not implmented yet
	dbg.console()
	--self.scale.x=x
	--self.scale.y=y
	--self.scale.z=z
end


function AnimOgreEntity:setPoseDOF(pose)
	self.loader:setPoseDOF(pose)
	local srcpose=Pose()
	self.loader:getPose(srcpose)

	return self:setPose(srcpose)
end

function AnimOgreEntity:setPose(pose, useLocal)
	local loader=self.loader
	--if useLocal then
	if true then
		local config={ entityScale=self.entityScale, skinScale=self.scale}
		local entity=self.ogreEntity
		local node=self.ogreNode

		local skel=entity:getSkeleton()

		-- set local orientation
		local RI=self.BIndex_to_RI
		for i=0, RI:size()-1 do
			local ri=RI(i)
			if ri~=-1 then
				local q=pose.rotations(ri)
				assert(q.x==q.x)
				local bone=skel:getBone(i)
				bone:setOrientation(q)
				--bone.bone:setOrientation(quater(0,1,0,0))
				--bone:_setDerivedOrientation(quater(0,1,0,0))
				if ri==0 then
					s=bone:getScale().x
					--bone.bone:setPosition(pose.translations(0)*s)
					--bone:setPosition(vector3(0,0,0))
				end
			end
		end
		--entity:getSkeleton():_updateTransforms()
		assert(pose.translations(0).x>-1000)
		assert(pose.translations(0).x<1000)

		node:setPosition(pose.translations(0)*self.scale+self.trans)
	--else
	--	-- does not work well for some skeletons
	--	-- but very useful for precise control
	--	-- if this does not work, set useLocal=true
	--	loader:setPose(pose)
	--	self:setExactPose(loader)
	end
	if self:isDebugMode() then
		PLDPrimSkin.setPose(self.skin, pose, self.loader)
	end
end

-- assumes that loader has been setPosed
function AnimOgreEntity:setExactPose(loader)
	local pose=Pose()
	loader:getPose(pose)
	self:setPose(pose)
end

function AnimOgreEntity:setExactPose_buggy(loader)
	--use setPose(pose) instead
	assert(false) --  bone:_setOrientation also necessary. (see RetargetSkin.lua)
	local config={ entityScale=self.entityScale, skinScale=self.scale}
	local entity=self.ogreEntity
	local node=self.ogreNode

	local skel=entity:getSkeleton()
	do
		local gt=loader:bone(1):getFrame().translation:copy()
		-- set global orientation
		local BI=self.BIndex_to_BI
		for i=0, BI:size()-1 do
			local bi=BI(i)
			if bi>1 then
				local bone=skel:getBone(i)
				bone:_setDerivedOrientation(loader:bone(bi):getFrame().rotation)
				bone:_setDerivedPosition((loader:bone(bi):getFrame().translation-gt)*(1/self.sf))
			end
		end
		node:setPosition(gt*self.scale+self.trans)
	end
	--local original_pos=lsh.bone:_getDerivedPosition()
	--lsh.bone:_setDerivedPosition(original_pos+vector3(0,0,1))
	--print(RE.getSceneNode(ogreSkin):getPosition()/config.entityScale)
end

function AnimOgreEntity._setPose(e, pose, loader)
	AnimOgreEntity.setPose(e, pose) -- for compatibility with other skin classes
end

function AnimOgreEntity:stopAnim()
	-- does nothing
end


-- returns VRMLloader, Mesh, SkinningInfo
function AnimOgreEntity.createVRMLloaderAndSurfaceInfo(sourceFile, skinScale, _entityScale, _skelScale, _cylinder_radius)
	if not _entityScale then _entityScale=1 end
	if not _skelScale then _skelScale=1 end
	if not _cylinder_radius then _cylinder_radius=0.07 end

	-- create mOrigLoader 
	local mOrigLoader=SkinnedMeshLoader(sourceFile, false, false)
	mOrigLoader:printHierarchy()
	mOrigLoader:gotoBindPose()

	local entityScale=_entityScale
	local skelScale=_skelScale
	
	-- copy the current skeleton pose to the skin
	local newPose= mOrigLoader:getPoseMap()

	-- step 1. copy the bad skeleton
	
	--mLoader=mOrigLoader:toVRMLloader()
	local mLoader, sf=AnimOgreEntity.createVRMLloader(sourceFile, entityScale, skinScale/skelScale)

	mLoader:removeAllRedundantBones() -- clean up the skeleton

	newPose.translations(0):scale(entityScale/skinScale)
	mLoader:setPoseMap(newPose)
	mLoader:setCurPoseAsInitialPose() -- adjust skeleton so that the current pose becomes the identity pose (all local joint orientations are identity quaternions)
	mOrigOffset=mLoader:bone(1):getOffsetTransform().translation
	mLoader:bone(1):getOffsetTransform().translation:zero()
	MotionUtil.removeSlidingJoints(mLoader)
	local ibone=2

	local function removeBone(mLoader, bone)
			mLoader:VRMLbone(bone:treeIndex()):setJointAxes('')
			mLoader:_initDOFinfo()
			mLoader:removeBone(bone)
	end
	-- remove assimp-generated unnecessary bones.
	while ibone<mLoader:numBone() do
		if select(1, string.find(mLoader:bone(ibone):name(), 'AssimpFbx._Translation')) then
			assert(select(1,string.find(mLoader:bone(ibone):childHead():name(), 'AssimpFbx._PreRotation')))

			local newName
			local bone=mLoader:bone(ibone)
			if select(1,string.find(mLoader:bone(ibone):childHead():childHead():name(), 'AssimpFbx._Rotation')) then
				newName=bone:childHead():childHead():childHead():name()
				removeBone(mLoader, bone:childHead():childHead():childHead())
			else
				newName=bone:childHead():childHead():name()
			end
			removeBone(mLoader, bone:childHead():childHead())
			removeBone(mLoader, bone:childHead())
			mLoader:bone(ibone):setName(newName)
		end
		ibone=ibone+1
	end

	local convInfoA, convInfoB
	do 
		local loaderA=mLoader
		convInfoA=TStrings()
		convInfoA:resize(loaderA:numBone()-1)
		convInfoB=TStrings()
		convInfoB:resize(loaderA:numBone()-1)
		for i=1, loaderA:numBone()-1 do
			convInfoA:set(i-1, loaderA:bone(i):name())
			convInfoB:set(i-1, loaderA:bone(i):name())
		end
	end

	-- you can further simplify mLoader as long as all the joint positions are kept.
	-- for example, you can change rotational channels so that knee cannot twist, and so on. (see skeletonEditor_GUI.lua)
	-- after that, create a mapping (PT) between the two skeletons 

	local PoseTransfer2=require("subRoutines/PoseTransfer2")
	PT=PoseTransfer2(mLoader, mOrigLoader, convInfoA, convInfoB)
	local mSkinLoader=mOrigLoader
	for i=1, mSkinLoader:numBone()-1 do
		print(i, mSkinLoader:getDerivedScale(i), mSkinLoader:getBindingPoseInverseScale(i))
	end

	local numVertex=mSkinLoader:getCurrMesh():numVertex()
	
	local mSkinningInfo=SkinnedMeshFromVertexInfo()
	mSkinLoader:getInfo(mSkinningInfo)

	local tiMap=intvectorn(mSkinLoader:numBone())
	tiMap:setAllValue(0)

	-- mLoader may have fewer bones than mSkinLoader so an index map is necesary.
	for i=1, mLoader:numBone()-1 do
		local ti=mSkinLoader:getTreeIndexByName(mLoader:bone(i):name())
		tiMap:set(ti, i)

		local frameA=mSkinLoader:bone(ti):getFrame()
		local frameB=mLoader:bone(i):getFrame()
		print(frameA.translation, frameB.translation*(skinScale/skelScale))

		-- if the following assert fails, it means that some of the scale settings are incorrect.
		assert(frameB.translation:distance(frameA.translation*(entityScale*skelScale/skinScale))<1e-3)
	end

	for i=0, numVertex-1 do
		local treeIndices=mSkinningInfo:treeIndices(i):copy()

		local treeIndicesNew=mSkinningInfo:treeIndices(i)
		for i=0, treeIndices:size() -1 do
			treeIndicesNew:set(i, tiMap(treeIndices(i)))
		end

		local localpos=mSkinningInfo:localPos(i)
		
		-- localpos needs to be converted from mSkinLoader space to mLoader space.
		for j=0, localpos:size()-1 do
			local frameA=mSkinLoader:bone(treeIndices(j)):getFrame()
			local frameB=mLoader:bone(treeIndicesNew(j)):getFrame()
			-- 1/100
			-- skelScale/skinScale
			localpos(j):assign(frameB:inverse()*((frameA*localpos(j))*(entityScale/skinScale)))
		end
	end

	local mMesh=mSkinLoader:getCurrMesh():copy() 
	-- scale이 달라서 vertex 위치 업데이트 필요.
	mSkinningInfo:calcVertexPositions(mLoader, mMesh);


	if true then
		-- 물리 시뮬레이션이 가능한 형태로 mLoader 개조.
		local radius=_cylinder_radius
		local wrltbl, bones=mLoader:toWRLtable(radius)

		-- remove co-located joints
		while true do
			if #wrltbl.body.children==1 and wrltbl.body.children[1].translation:length()<1e-3 then
				wrltbl.body=wrltbl.body.children[1]
				wrltbl.body.translation=vector3(0,0,0)
				wrltbl.body.jointType='free'
				wrltbl.body.jointAxis=nil
				local nv=mMesh:numVertex()
				for i=0, nv-1 do
					local ti= mSkinningInfo:treeIndices(i)
					for j=0, ti:size()-1 do
						ti:set(j, math.max(1, ti(j)-1))
					end
				end
			else
				break
			end
		end
		for i,bone in ipairs(bones) do

			if not bone.geometry then
				bone.geometry={
					{
						'Sphere',
						translation=vector3(0,0,0),
						size=vector3(radius, radius, radius),
					}
				}
			end
		end

		mLoader=MainLib.WRLloader(wrltbl)
	end

	return mLoader, mMesh, mSkinningInfo, wrltbl
end

AnimOgreEntity.SurfaceInfoSkin=LUAclass()


function AnimOgreEntity.SurfaceInfoSkin:__init(mLoader, mMesh, mSkinningInfo)
	self.name=RE.generateUniqueName()
	self.mesh=mMesh:copy()

	--buildEdgeList, dynamicUpdate, useNormal, useTexCoord
	self.mMeshToEntity=MeshToEntity(self.mesh, self.name..'meshName', true, true, false, true)
	self.mEntity=self.mMeshToEntity:createEntity(self.name..'entityName')
	self.mEntity:setMaterialName('lightgrey_transparent')
	self.mNode=RE.createChildSceneNode(RE.ogreRootSceneNode(), self.name..'nodeName')
	self.mNode:attachObject(self.mEntity)
	self.mNode:scale(100,100,100)
	self.mLoader=mLoader
	self.mSkinningInfo=mSkinningInfo
end
function AnimOgreEntity.SurfaceInfoSkin:setWorldState(ws)
	self.mLoader:fkSolver():assign(ws)
	self.mSkinningInfo:calcVertexPositions(self.mLoader, self.mesh);

	local center=self.mesh:calcMeshCenter()
	for i=0, self.mesh:numVertex()-1 do
		self.mesh:getVertex(i):rsub(center)
	end
	self.mNode:setPosition(center*100)

	self.mMeshToEntity:updatePositions()
end
function AnimOgreEntity.SurfaceInfoSkin:dtor()
	RE.removeEntity(self.mNode)
end
