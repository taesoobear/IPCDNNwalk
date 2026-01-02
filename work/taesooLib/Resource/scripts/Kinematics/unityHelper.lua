if isUnity3D then
	function dbg.drawSphere(objectList, pos, nameid, _materialName, _scale)
		_scale=_scale or 5 -- 5cm
		local info=vectorn(4)
		info:setVec3(0, pos*0.01) 
		info:set(3, _scale*0.01) 
		this('drawSphere', nameid, info)
	end

	function coarseLog(str)
		print(str)
	end

end

function MotionUtil.getBonePositions(loader, jointpos, jointori)
	assert(jointpos:size()==(loader:numBone()-1)*3)
	assert(jointori:size()==(loader:numBone()-1)*4)

	for i=1, loader:numBone()-1 do
		local T=loader:bone(i):getFrame()
		jointpos:setVec3((i-1)*3, T.translation)
		jointori:setQuater((i-1)*4, T.rotation)
	end
end

-- fbx_info table should contain at least loader and wrlInfo.
function PLDPrimVRML:setUnityPose(upose, fbx_info)
	local loader=fbx_info.loader
	local loaderInfo=fbx_info.wrlInfo

	if loaderInfo.targetIndex then
		-- loader has a dummy bone (0)
		assert(loader:numBone()==loaderInfo.targetIndex:size()+1)
	else
		loaderInfo.targetIndex=CT.colon(1, loader:numBone(), 1)-1
	end


	local state=loader:fkSolver()

	local targetIndex=loaderInfo.targetIndex

	local tsLength=math.round((upose:size()-2)/9)
	assert(upose(0)==tsLength)
	local function getQ(ti)
		assert(upose(ti*5+1)==ti)
		return upose:toQuater(ti*5+2)
	end

	local startT=tsLength*5+2
	local function getP(ti)
		assert(upose(ti*4+startT)==ti)
		return upose:toVector3(ti*4+startT+1)
	end

	local myRoot=loaderInfo.targetIndex(0)
	local rootTF=transf(getQ(myRoot), getP(myRoot))
	while myRoot~=0 do
		myRoot=fbx_info.parentIndex(myRoot)
		rootTF:leftMult(transf(getQ(myRoot), getP(myRoot)))
	end

	
	state:localFrame(1):assign(rootTF)

	for i=2, loader:numBone()-1 do
		state:localFrame(i).rotation:assign(getQ(loaderInfo.targetIndex(i-1)))
	end

	state:forwardKinematics()
	self:setSamePose(state)
	
	--dbg.draw('Axes', rootTF,'root', skinScale)

	--for i=0, tsLength-1 do
	--	dbg.draw('Sphere', getP(i)*100,'pos'..i)
	--end
end
function getUnityPose(varname)
	local loaderInfo=_G[varname]
	local loader=loaderInfo.loader

	if loaderInfo.targetIndex then
		-- loader has a dummy bone (0)
		assert(loader:numBone()==loaderInfo.targetIndex:size()+1)
	else
		loaderInfo.targetIndex=CT.colon(1, loader:numBone(), 1)-1
	end

	local pose=Pose()
	local targetIndex=loaderInfo.targetIndex
	loader:getPose(pose)

	local out=vectorn(2+pose.rotations:size()*5+pose.translations:size()*4)

	local c=0
	out:set(c, pose.rotations:size()) c=c+1
	for i=0, pose.rotations:size()-1 do
		local ti=loader:getTreeIndexByRotJointIndex(i)
		out:set(c, targetIndex(ti-1)) c=c+1
		out:setQuater(c, pose.rotations(i)) c=c+4
	end
	out:set(c, pose.translations:size()) c=c+1
	for i=0, pose.translations:size()-1 do
		local ti=loader:getTreeIndexByTransJointIndex(i)
		out:set(c, targetIndex(ti-1)) c=c+1
		out:setVec3(c, pose.translations(i)) c=c+3
	end
	assert(c==out:size())
	return out
end

function PLDPrimVRML:getUnityPose()
	local state=self:getState()

	local numBone=state:getSkeleton():numBone()
	local out=vectorn(2+(numBone-1)*5+1*4)

	local c=0
	out:set(c, numBone-1) c=c+1
	for ti=1, numBone-1 do
		out:set(c, ti-1) c=c+1
		out:setQuater(c, state:localFrame(ti).rotation) c=c+4
	end
	out:set(c, 1) c=c+1
	do
		ti=1
		out:set(c, ti-1) c=c+1
		out:setVec3(c, state:localFrame(ti).translation) c=c+3
	end
	assert(c==out:size())
	return out
end
-- out: pose for fbx_info.loader
function setUnityPose(loaderInfo, fbx_info, out, excludeRoot)
	assert (loaderInfo.targetIndex )
	
	local loader=loaderInfo.loader
	local pose=Pose()
	local targetIndex=loaderInfo.targetIndex
	loader:getPose(pose)

	for i=1, pose.rotations:size()-1 do
		local ti=loader:getTreeIndexByRotJointIndex(i)
		out.rotations(targetIndex(ti-1)):assign(pose.rotations(i))
	end
	if not excludeRoot then
		-- root
		out.rotations(0):assign(pose.rotations(0))
		out.translations(0):assign(pose.translations(0))
	end
end


-- jointpos, jointori: 각 본들의 global transformation (현재 자세)
--function MotionUtil.generateWRLfromRawInfo(robotname, cylinder_radius, names, parentnames, jointpos, jointori)
---> moved to module.lua

-- wrl테이블에서 EE의 원소 또는 원소의 조상이 아닌 모든 본들을 제거함.
--
-- 아래 배열에 있는 본들과 그 부모 본들만 남긴다.
-- EE={
--	'Character1_RightHand', 
--	'Character1_LeftHand', 
--	'Character1_RightFoot', 
--	'Character1_LeftFoot', 
--	'Character1_Head'
--}
-- bindPose is optional
function MotionUtil.cleanupBones(wrl, bones, EE, bindPose)
	local boneInfo={}
	local nameToTreeIndex={}
	for i=1, #bones do
		boneInfo[i]={
			visited=false,
		}
		nameToTreeIndex[bones[i].name]=i
	end

	for i=1, #bones do
		local bone=bones[i]
		if bone.children then
			for ic, c in ipairs(bone.children) do
				local child_ti=nameToTreeIndex[c.name]
				assert(child_ti)
				local parent_ti=nameToTreeIndex[bone.name]
				assert(parent_ti)
				boneInfo[child_ti].parent=parent_ti

				--print(child_ti, parent_ti)
			end
		end
	end

	for iee, ee in ipairs(EE) do
		local ti=nameToTreeIndex[ee]
		while ti do
			boneInfo[ti].visited=true
			--print(bones[ti].name)
			ti=boneInfo[ti].parent
		end
	end
	assert(boneInfo[1].visited)

	local function removeUnvisitedChildren(bone)
		local newChildren={}
		for ic, c in ipairs(bone.children) do
			if boneInfo[nameToTreeIndex[c.name]].visited then
				table.insert(newChildren, c)
				removeUnvisitedChildren(c)
			end
		end
		bone.children=newChildren
	end

	removeUnvisitedChildren(wrl.body)

	local targetIndex=intvectorn()
	targetIndex:pushBack(0)
	local function updateTargetIndex(bone, targetIndex)
		for ic, c in ipairs(bone.children) do
			targetIndex:pushBack(nameToTreeIndex[c.name]-1)
			updateTargetIndex(c, targetIndex)
		end
	end
	updateTargetIndex(wrl.body, targetIndex)
	if false then
		-- print selected bone names
		for i=0, targetIndex:size()-1 do
			print(bones[targetIndex(i)+1].name)
		end
	end

	if bindPose then
		local bindPose2=Pose()
		bindPose2:init(targetIndex:size(), 1)
		for i=0, targetIndex:size()-1 do
			bindPose2.rotations(i):assign(bindPose.rotations(targetIndex(i)))
		end
		bindPose2.translations(0):assign(bindPose.translations(0))
		return targetIndex, bindPose2
	end

	return targetIndex

end

-- the original wrl table is changed in place.
function MotionUtil.changeRoot(wrl, bones, newRootBoneIndex, bindPose, origTargetIndex)
	local nameToTreeIndex={}
	for i=1, #bones do
		nameToTreeIndex[bones[i].name]=i
	end

	local newRootBone=bones[newRootBoneIndex]
	wrl.body=newRootBone
	wrl.body.jointAxis=nil
	wrl.body.jointType=nil

	local targetIndex=intvectorn()
	targetIndex:pushBack(origTargetIndex(newRootBoneIndex-1)) -- rot-joint index== tree index -1
	local function updateTargetIndex(bone, targetIndex, origTargetIndex)

		if bone.children then
			for ic, c in ipairs(bone.children) do
				targetIndex:pushBack(origTargetIndex(nameToTreeIndex[c.name]-1)) -- rot-joint index== tree index -1
				updateTargetIndex(c, targetIndex, origTargetIndex)
			end
		end
	end

	updateTargetIndex(wrl.body, targetIndex, origTargetIndex)
	if false then
		-- print selected bone names
		for i=0, targetIndex:size()-1 do
			print(bones[targetIndex(i)+1].name)
		end
	end

	local function updateBonesArray(bones_backup, targetIndex, invMap)
		local bones={}
		for j=0, targetIndex:size()-1 do
			local bo=bones_backup[invMap(targetIndex(j))+1]
			assert(bo)
			bones[j+1]=bo
		end
		return bones
	end

	-- map from targetIndex to rotJointIndex of bindPose
	local invMap=intvectorn(origTargetIndex:maximum()+1)
	invMap:setAllValue(-1)
	for i=0, origTargetIndex:size()-1 do
		invMap:set(origTargetIndex(i), i)
	end

	local bones =updateBonesArray(bones, targetIndex, invMap)
	if bindPose then


		local bindPose2=Pose()
		bindPose2:init(targetIndex:size(), 1)
		for i=0, targetIndex:size()-1 do
			bindPose2.rotations(i):assign(bindPose.rotations(invMap(targetIndex(i))))
		end
		bindPose2.translations(0):assign(bindPose.translations(0))
		return targetIndex, bones, bindPose2
	end

	return targetIndex, bones

end

function MotionUtil.findBone(bones, name)
	for i=1, #bones do
		if bones[i].name==name then
			return i
		end
	end
	return -1
end

-- output table contains mesh, indices, weights, loader, currentPose, bindPose, wrlInfo
-- _root_bone_name  : this becomes the dummy root bone (bone 0) and won't be used.
--                      It's childrens will actually be bones.
function MotionUtil.loadFBXinfo(file, skinScale, _root_bone_name)
	if type(file)=='string' then
		local fn=file
		file=util.BinaryFile()
		file:openRead(fn)
	end

	local output={}

	local mesh=Mesh()
	do
		assert(file:unpackAny()=='Mesh')
		mesh:resize(file:unpackInt(),0)
		for i=0, mesh:numVertex()-1 do
			file:unpack(mesh:getVertex(i))
		end
		local indices=intvectorn()
		file:unpack(indices)
		mesh:resize(mesh:numVertex(), indices:size()/3)

		for i=0, indices:size()-1, 3 do
			mesh:getFace(i/3):setIndex(indices(i), indices(i+1), indices(i+2))
		end
	end
	output.mesh=mesh

	local tid=file:unpackAny()
	assert(tid=='BoneWeights')
	local indices=file:unpackAny()
	local weights=file:unpackAny()
	local rigid_indices=intvectorn(indices:size()/4)
	rigid_indices:setAllValue(-1)
	for i=0, rigid_indices:size()-1 do
		local a=weights:range(i*4, (i+1)*4):argMax()
		rigid_indices:set(i, indices(i*4+a))
	end
	output.rigid_indices=rigid_indices

	local tid=file:unpackAny()
	assert(tid=='BindPoses')
	local bindposes={}

	local n_bones=file:unpackInt()


	local function checkRigidTransform(mat)
		-- it is assumed that there are no scale components
		if not(math.abs(mat:getColumn(0):length()-1)<1e-2) then print('Warning!!! bone '..i..' contains scaling') end
		if not(math.abs(mat:getColumn(1):length()-1)<1e-2) then print('Warning!!! bone '..i..' contains scaling') end
		if not(math.abs(mat:getColumn(2):length()-1)<1e-2) then print('Warning!!! bone '..i..' contains scaling') end
	end
	for i=1, n_bones do
		local mat=matrix4()
		file:unpack(mat)
		bindposes[i]=mat
		checkRigidTransform(mat)
	end

	local tid=file:unpackAny()
	assert(tid=='boneNames')

	local n_version=file:unpackInt()
	local n_bones2
	if n_version<0 then
		n_bones2=file:unpackInt()
		local tid=file:unpackAny()
		assert(tid=='localToWorld')
		local nrenderer=file:unpackInt()
		output.localToWorld={}
		for i=1, nrenderer do
			local mat=matrix4()
			file:unpack(mat)
			output.localToWorld[i]=mat
		end
		local mat=output.localToWorld[1]

		local tf=matrix4()
		tf:inverse(mat)
		local mesh=output.mesh
		for i=0, mesh:numVertex()-1 do
			mesh:transform(tf)
		end
	else
		n_bones2=n_version
	end
	local bonenames=TStrings (n_bones2)
	for i=1, n_bones2 do
		bonenames:set(i-1, file:unpackAny())
	end

	local tid=file:unpackAny()
	assert(tid=='LoaderInfo')
	local names=file:unpackAny()
	output.names=names -- unity names which can be indexed by output.wrlInfo.targetIndex
	local parentNames=file:unpackAny()
	local jointpos=file:unpackAny()
	local jointori=file:unpackAny()

	local tbl, currentPose, bones, parent=MotionUtil.generateWRLfromRawInfo(names(0),1/5/skinScale , names, parentNames, jointpos:vec3View(), jointori:quatView())

	output.parentIndex=CT.vec(parent)-1

	for i=1, #bones do
		bones[i].name=string.gsub(bones[i].name, " ", "__SPACE__")
		bones[i].name=string.gsub(bones[i].name, "%[", "__PAREN1__")
		bones[i].name=string.gsub(bones[i].name, "%]", "__PAREN2__")
	end

	local targetIndex=CT.colon(0, #bones)
	if _root_bone_name then
		local found=false
		for i=1, #bones do
			if bones[i].name==_root_bone_name then
				-- root 바꾸기
				if(#bones[i].children==1 and bones[i].children[1]==bones[i+1]) then
					targetIndex, bones, currentPose= MotionUtil.changeRoot(tbl, bones, i+1, currentPose, targetIndex)
					currentPose.rotations(0):assign(jointori:quatView()(i))
					currentPose.translations(0):assign(jointpos:vec3View()(i))
				else
					targetIndex, bones, currentPose= MotionUtil.changeRoot(tbl, bones, i, currentPose, targetIndex)
					currentPose.rotations(0):assign(jointori:quatView()(i-1))
					currentPose.translations(0):assign(jointpos:vec3View()(i-1))
				end
				found=true
				break
			end
		end
		if not found then 
			print("Warning! cannot find bone :", _root_bone_name)
			print("Possible candidates are :")
			local out={}
			for i=1, math.min(5, #bones ) do
				table.insert(out, bones[i].name)
			end

			print(table.concat(out, ',')..' ...')
		end
	end

	local loader=MainLib.WRLloader(deepCopyTable(tbl))
	output.currentPose=currentPose
	output.loader=loader
	output.wrlInfo={ wrlTable=tbl, bones=bones, targetIndex=targetIndex}

	do 
		-- convert indices
		local treeIndexMap=intvectorn(n_bones2)
		treeIndexMap:setAllValue(-1)
		for i=0, n_bones2-1 do
			local name=string.gsub(bonenames(i), " ", "__SPACE__")
			local ti=loader:getTreeIndexByName(name)
			treeIndexMap:set(i, ti)
		end
		for i=0, rigid_indices:size()-1 do
			rigid_indices:set(i, treeIndexMap(rigid_indices(i)))
		end
		for i=0, indices:size()-1 do
			indices:set(i, treeIndexMap(indices(i)))
		end
		local bindposes_old=bindposes
		local bindposes={}
		local I=matrix4()
		I:identity()
		for i=1, loader:numBone()-1 do
			bindposes[i]=I
		end
		for i, b in ipairs(bindposes_old) do
			local ti=treeIndexMap(i-1)
			bindposes[ti]=b
		end
		output.bindposes=bindposes
	end

	local bindposes=output.bindposes
	for i=1, loader:numBone()-1 do
		if bindposes[i] then
			local t=transf(bindposes[i])
			loader:bone(i):getFrame():assign(t:inverse())
		end
		output.bindPose=Pose()
		loader:fkSolver():getPoseFromGlobal(output.bindPose)
	end
	output.indices=indices
	output.weights=weights

	return output
end

-- optional: _cleanupInfo ={ 'Head', 'Left wrist'} - 이 본과 부모 본들외에는 날린다.
function MotionUtil.createVRMLloader(names, parentNames, jointpos, jointori, _cleanupInfo)
	local _1,_2
	if type(names)=='table' then
		local o=names
		names=TStrings()
		names:fromTable(o)
	end
	if type(parentNames)=='table' then
		local o=parentNames
		parentNames=TStrings()
		parentNames:fromTable(o)
	end
	if jointpos.vec3View then
		_1=jointpos
		jointpos=jointpos:vec3View() 
	end
	if jointori.quatView then
		_2=jointori
		jointori=jointori:quatView() 
	end
	local tbl, bindPose, bones=MotionUtil.generateWRLfromRawInfo('robot', 0.05, names, parentNames, jointpos, jointori)
	local targetIndex
	if _cleanupInfo then
		targetIndex, bindPose=MotionUtil.cleanupBones(tbl, bones, _cleanupInfo, bindPose)
	else
		targetIndex=CT.colon(0, #bones)
	end
	for i, v in ipairs(bones) do
		v.name=string.gsub(v.name,'%s', '_')
	end

	local loaderA=MainLib.WRLloader(tbl)

	for i=1, loaderA:numBone()-1 do
		local name=loaderA:bone(i):name()
		assert(bones[targetIndex(i-1)+1].name==name)
		--name=string.gsub(name, '__SPACE__', '_')
		--loaderA:bone(i):setName(name)
	end
	loaderA:setPose(bindPose)
	local out ={
		loader=loaderA,
		bindPose=bindPose,
		targetIndex=targetIndex,
	}
	return out
end

function MotionUtil.createSkinningInfo(fbx_info, drawDebugSkin, skinScale)
	local si=SkinnedMeshFromVertexInfo()
	assert(fbx_info.mesh:numVertex()==fbx_info.indices:size()/4)

	local loader=fbx_info.loader
	loader:setPose(fbx_info.bindPose)

	local nv=fbx_info.mesh:numVertex()
	si:resize(nv)

	local mesh=fbx_info.mesh
	local indices=fbx_info.indices
	local weights=fbx_info.weights
	for i=0,nv-1 do
		local ind=indices:range(i*4, (i+1)*4)
		local w=weights:range(i*4, (i+1)*4)
		
		if w:sum()==0 then
			assert(false)
			si:weights(i):assign(CT.vec(1))
			si:treeIndices(i):assign(CT.ivec(ind(0)))
			si:localPos(i):setSize(1)

			local T=loader:bone(ind(0)):getFrame()
			si:localPos(i)(0):assign(T:toLocalPos(mesh:getVertex(i)))
		else
			si:localPos(i):setSize(ind:size())

			for j=0, ind:size()-1 do
				if ind(j)==-1 then
					ind:set(j, 1)
					w:set(j,0)
				end
				assert(ind(j)>=1 and ind(j)<fbx_info.loader:numBone())
			end
			si:treeIndices(i):assign(ind)
			si:weights(i):assign(w)

			if fbx_info.localToWorld then
				-- exact
				local ibp=fbx_info.bindposes
				for j=0, ind:size()-1 do
					local invT=ibp[ind(j)]
					si:localPos(i)(j):assign(invT*mesh:getVertex(i))
				end
			else
				for j=0, ind:size()-1 do
					local T=loader:bone(ind(j)):getFrame()
					si:localPos(i)(j):assign(T:toLocalPos(mesh:getVertex(i)))
				end
			end
		end
	end
	fbx_info.skinningInfo=si

	if drawDebugSkin then
		local uu=RE.generateUniqueName()
		local meshToEntity=MeshToEntity(fbx_info.mesh, 'meshName'..uu, true, true, false, false)
		local entity=meshToEntity:createEntity('entityName' ..uu)

		entity:setMaterialName('lightgrey_transparent')
		local node=RE.createChildSceneNode(RE.ogreRootSceneNode(), "mesh_node"..uu)
		node:setScale(skinScale, skinScale, skinScale)
		node:attachObject(entity)
		--node:translate(-s*50,-0.1,-s*50)
		fbx_info.meshToEntity=meshToEntity
	end
end

function MotionUtil.createSkinningInfoFromOgreMesh(meshFile, skinScale, entityScale, default_skel_thickness)

	require("subRoutines/AnimOgreEntity")
	local buildEdgeList=false -- false: no shadow
	--mOgreSkin=AnimOgreEntity(meshFile, entityScale, skinScale, 1, buildEdgeList)
	--mOgreSkin:setTranslation(offset_x*3,0,0)

	local loader, mesh2, skinningInfo=AnimOgreEntity.createVRMLloaderAndSurfaceInfo(meshFile, skinScale, entityScale, 1, 0.02)

	local fbx_info={}
	fbx_info.loader=loader
	fbx_info.mesh=mesh2
	fbx_info.skinningInfo=skinningInfo
	fbx_info.bindPose=Pose()
	loader:getPose(fbx_info.bindPose)
	local bindposes={}
	for i=1, loader:numBone()-1 do
		bindposes[i]= matrix4(loader:bone(i):getFrame():inverse())
	end
	fbx_info.bindposes=bindposes

	if false then
		local meshToEntity2, node2=mesh2:drawMesh('lightgrey_transparent', 'mesh_node2')
		node2:scale(skinScale,skinScale,skinScale)
		node2:translate(100,0,0)

		--mSkin3=RE.createSkin(loader)
		mSkin3=RE.createVRMLskin(loader,false)
		mSkin3:setScale(skinScale, skinScale, skinScale)
		mSkin3:setThickness(1)
		mSkin3:setTranslation(100,0,0)
	end
	return fbx_info
end

function MotionUtil.getMergedMesh(tempSkel)
	local mesh=Mesh()
	assert(tempSkel)
	local vert_indices=intvectorn(tempSkel:numBone())
	vert_indices:set(0,0)
	for i=1, tempSkel:numBone()-1 do
		local bone=tempSkel:VRMLbone(i)
		if bone:hasShape() then
			local gmesh=bone:getMesh():copy()
			gmesh:transform(matrix4(bone:getFrame()))
			mesh:mergeMesh(mesh, gmesh)
		end
		vert_indices:set(i,mesh:numVertex())
	end
	return mesh, vert_indices
end

function MotionUtil.getMergedMeshAndInfo(tempSkel, fn)
	local mesh, vert_indices=MotionUtil.getMergedMesh(tempSkel)

	local info=SkinnedMeshFromVertexInfo()
	info:resize(mesh:numVertex())
	for ibone=1, tempSkel:numBone()-1 do
		local sv=vert_indices(ibone-1)
		local ev=vert_indices(ibone)

		for i=sv, ev-1 do
			info:treeIndices(i):resize(1)
			info:treeIndices(i):set(0, ibone)
			info:weights(i):resize(1)
			info:weights(i):set(0, 1)
		end
	end
	if fn and not os.isFileExist(fn) then
		mesh:saveOBJ(fn, true, true)
	end
	return info, mesh
end
