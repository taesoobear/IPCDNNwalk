-- FBXloader=require('FBXloader') 
require('subRoutines/WRLloader')
-- uses software LBS skinning. so much slower than c++/shader-based OgreEntity class but I prefer simplicity here.
--
local FBXloader=LUAclass()
-- use FBXloader.motionLoader(filename ) or FBXloader(filename)

local function printTreeIndices(ti, loader_i)
	local names=''
	for i=0, ti:size()-1 do
		names=names..' '..loader_i:bone(ti(i)):name()
	end
	print(ti, names)
end
local function  mirror(v)
	-- 위치나 축을 왼손 좌표계로 변환
	return vector3(-v.x, v.y, v.z)
end
local function  mirrorQ(v)
	-- 위치나 축을 왼손 좌표계로 변환
	return quater(v.w, v.x, -v.y, -v.z)
end
local function toYUP(v)
	assert(v.w==nil)
	--return vector3(v.y, v.z, v.x);
	return vector3(v.x, v.z, -v.y);
end
local function toZUP(v)
	assert(v.w==nil)
	return vector3(v.x, -v.z, v.y);
end
local function toYUP_q(q)
	--return quater(q.w, q.y,  q.z, q.x);
	return quater(q.w, q.x,  q.z, -q.y);
end
local function toZUP_q(q)
	--return quater(q.w, q.y,  q.z, q.x);
	return quater(q.w, q.x,  -q.z, q.y);
end

function FBXloader:_getBindPoseGlobal(imesh)
	if self.bindpose_global then
		return self.bindpose_global
	end
	local bg= self.fbxInfo[imesh].bindpose_global
	assert(bg)
	return bg
end
function FBXloader:_bakeBindPose()
	-- 현재 포즈로 mesh를 굽는다. (즉, 모든 메시가 동일한 bindpose를 갖게 된다. 원본메시는 사라지지만, 뒤쪽 구현이 쉬워짐.)
	local loader=self.loader

	for i, meshInfo in ipairs(self.fbxInfo) do
		local ME=self.fbxInfo[i]
		local skin=meshInfo.skin
		local mesh=meshInfo[1]
		skin:calcVertexPositions(self.loader:fkSolver(), mesh)
		skin:calcVertexNormals(self.loader:fkSolver(), self:_getBindPoseGlobal(i), meshInfo.localNormal, mesh)
		meshInfo.bindpose_global=nil
	end
	self:_setBindPose(loader)
	-- disconnect from the original fbx file.
	-- self.fbx manages images so...
	for i, meshInfo in ipairs(self.fbxInfo) do
		if meshInfo.image then
			meshInfo.image=meshInfo.image:copy()
		end
	end

	self.fbx=nil 
end

-- slow so please export using output_loader:exportBinary('aaa.fbx.wrl.dat')
function FBXloader:toVRMLloader(totalMass, nConvex)
	require("Kinematics/meshTools")
	nConvex = nConvex or 6
	local fbx_info=self:createSkinningInfo()
	local temp=SkinToWRL(fbx_info, { maxConvex= nConvex })
	temp.skel:setTotalMass(totalMass)
	return temp.skel
end

function FBXloader:_bindPoseUpdated()
	local loader=self.loader
	for i,meshInfo in ipairs(self.fbxInfo) do
		local mesh=meshInfo[1]
		local skin=meshInfo.skin
		assert(self.bindpose_global) -- assert baked
		skin:calcLocalVertexPositions(self.loader, mesh)

		if mesh:numNormal()>0 then
			for i=0, mesh:numNormal()-1 do
				meshInfo.localNormal(i):assign(mesh:getNormal(i))
			end
		end
	end
end

function FBXloader:useBindPoseAsIdentityPose()
	self.loader:setPose(self.bindpose)
	self:setCurPoseAsInitialPose()
end
function FBXloader:numMesh()
	return #self.fbxInfo
end
function FBXloader:getMeshInfo(fbxMeshIndex)
	local meshInfo=self.fbxInfo[fbxMeshIndex]
	local mesh=meshInfo[1]
	return { mesh=meshInfo[1], skin=meshInfo.skin,  }
end
function FBXloader:setPose(pose)
	self.loader:setPose(pose)
end
function FBXloader:setCurPoseAsInitialPose()

	self:_bakeBindPose()

	-- backup the original skeleton (for creating pose map, etc...)
	self.orig_loader=self.loader:copy()

	local curPose=self.loader:pose()

	self.loader:setCurPoseAsInitialPose()

	local rootpos=curPose.translations(0)

	self.loader:bone(1):getOffsetTransform().translation:zero()
	

	self.loader:updateInitialBone()
	local poseI=self.loader:pose()
	poseI.translations(0):assign(rootpos)
	self.loader:setPose(poseI)
	self.orig_loader:setPose(curPose)

	local convertFromOriginalPose=MotionUtil.PoseTransfer(self.orig_loader, self.loader, true)

	convertFromOriginalPose:setTargetSkeleton(self.bindpose)

	self:_setBindPose(self.loader)
	self:_bindPoseUpdated()

	convertFromOriginalPose:setTargetSkeleton(self.currentPose)
	self.currentPose=self.loader:pose()

	if self.orig_loader.mMotion:numFrames()>0 then
		self.loader.mMotion:resize(self.orig_loader.mMotion:numFrames())
		for i=0, self.orig_loader.mMotion:numFrames()-1 do
			convertFromOriginalPose:setTargetSkeleton(self.orig_loader.mMotion:pose(i))
			self.loader.mMotion:pose(i):assign(self.loader:pose())
		end
	end
	self:setPose(poseI)
	return convertFromOriginalPose
end


function FBXloader:getPoseDOF()
	return self.loader:getPoseDOF()
end
function FBXloader:pose()
	return self.loader:pose()
end
function FBXloader:mirrorVA(jointpos)
	for i=0, jointpos:size()-1 do
		jointpos(i):assign(self:mirror(jointpos(i)))
	end
end
function FBXloader:mirrorQA(jointori)
	for i=0, jointori:size()-1 do
		jointori(i):assign(self:mirrorQ(jointori(i)))
	end
end

function FBXloader:fkSolver()
	return self.loader:fkSolver()
end
function FBXloader:genSkelTable(fbx, options)
	local mesh_count=self.fbx:getMeshCount()

	self.options=options
	--
	--fbx:getDefaultPose(jointpos, jointori)
	local names=TStrings()
	fbx:getBoneNames(names)
	self.fbxNumLimbNodes=names:size()

	local jointpos=vector3N()
	local jointori=quaterN()
	if options.useMeshBindPose then --or fbx:getMeshCount()==1 then
		-- ortiz.fbx 나 Kay.fbx등 fbx에 들어있는 restpose가 적절하지 않은 경우 사용.
		local argMax=0
		local max_bp=0
		for i=0, mesh_count-1 do
			local nbp=self.fbx:countBindPoses(i)
			if nbp>max_bp then
				argMax=i
				max_bp=nbp
			end
		end
		if max_bp>0 then
			self.fbx:countBindPoses(argMax) -- update.
			self:getBindPose(jointpos, jointori)
			self:checkRelativeRestPose(jointpos, jointori)  -- heuristic.  I cannot find the option  that describes this condition.
		else
			fbx:getRestPose(jointpos, jointori)
		end
	else
		fbx:getRestPose(jointpos, jointori)
	end

	--self:drawPose(jointpos, jointori, options, 'bindpose', vector3(-300,0,0))
	--self:drawRestPose(options)

	if options.mirror or options.toYUP or options.toZUP or options.toYUP_meshOnly or options.toZUP_meshOnly then
		if options.toYUP_meshOnly then
			self.convertMeshAxis=true
			options.toYUP=true
		elseif options.toZUP_meshOnly then
			self.convertMeshAxis=true
			options.toZUP=true
		else
			self.convertMeshAxis=true
			self.convertSkelAxis=true
		end

		if options.mirror and options.toYUP then
			self.mirror=function (self, v)
				local v=toYUP(v)
				v=mirror(v)
				return v
			end
			self.mirrorQ=function (self, q)
				local q=toYUP_q(q)
				q=mirrorQ(q)
				return q
			end
		elseif options.mirror and options.toZUP then
			self.mirror=function (self, v)
				local v=toZUP(v)
				v=mirror(v)
				return v
			end
			self.mirrorQ=function (self, q)
				local q=toZUP_q(q)
				q=mirrorQ(q)
				return q
			end
		elseif options.mirror then
			self.mirror=function (self, v)
				return mirror(v)
			end
			self.mirrorQ=function (self, q)
				return mirrorQ(q)
			end
		elseif options.toYUP then
			self.mirror=function (self, v)
				return toYUP(v)
			end
			self.mirrorQ=function (self, q)
				return toYUP_q(q)
			end
		else
			self.mirror=function (self, v)
				return toZUP(v)
			end
			self.mirrorQ=function (self, q)
				return toZUP_q(q)
			end
		end
	end
	
	if self.convertSkelAxis then
		self:mirrorVA(jointpos)
		self:mirrorQA(jointori)
	end


	local pid=fbx:parentIndices()
	local skinScale=options.skinScale or 100

	--dbg.drawBillboard( jointpos:matView()*skinScale, 'line3', 'redCircle', 10, 'QuadListV') -- QuadListV is view-dependent -> use drawBillboard
	if names:size()==0 then
		-- this fbx file has no skeleton
		names:resize(1)
		names:set(0, 'Root')
		pid=CT.ivec(-1)
		local pose=transf(fbx:getModelCurrPose('Model::Model'))

		jointpos=vector3N(1)
		jointpos(0):assign(pose.translation)
		jointori=quaterN(1)
		jointori(0):assign(pose.rotation)
		self.rigidBody=true
	end

	local tbl, currentPose, bones, parent=MotionUtil.generateWRLfromRawInfo(names(0),1/5/skinScale , names, pid, jointpos, jointori)

	for i=1, #bones do
		bones[i].name=string.gsub(bones[i].name, " ", "__SPACE__")
		bones[i].name=string.gsub(bones[i].name, "%[", "__PAREN1__")
		bones[i].name=string.gsub(bones[i].name, "%]", "__PAREN2__")
	end

	local targetIndex=CT.colon(0, #bones)
	local _root_bone_name=options.newRootBone
	local origTargetIndex=targetIndex
	local rootTF=transf(currentPose.rotations(0), currentPose.translations(0))

	if _root_bone_name then
		local found=false
		for i=1, #bones do
			if bones[i].name==_root_bone_name then
				-- root 바꾸기
				if(#bones[i].children==1 and bones[i].children[1]==bones[i+1]) then
					targetIndex = FBXloader.changeRoot(tbl, bones, i+1, targetIndex)
					rootTF.rotation:assign(jointori(i))
					rootTF.translation:assign(jointpos(i))
				else
					local function findParent(bones, i)
						for j=1, #bones do
							local bone=bones[j]
							if bone.children then
								for k, child in ipairs(bone.children) do
									if child.name==bones[i].name then
										return bone,k
									end
								end
							end
						end
						return nil
					end
					local pbone, cindex=findParent(bones, i)
					if pbone and #pbone.children>1 then
						-- strange. let's move children to the new rootbone
						for k, child in ipairs(pbone.children) do
							if k~=cindex then
								assert(child.name~=_root_bone_name)
								table.insert(bones[i].children, child)
							end
						end
					end
					
					targetIndex= FBXloader.changeRoot(tbl, bones, i, targetIndex)
					rootTF.rotation:assign(jointori(i-1))
					rootTF.translation:assign(jointpos(i-1))
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

	if false then
		-- print selected bone names
		for i=0, targetIndex:size()-1 do
			print(names(targetIndex(i)))
		end
	end
	do 
		local bindPose =currentPose
		local function updateBonesArray(bones_backup, targetIndex, invMap)
			local bones={}
			for j=0, targetIndex:size()-1 do
				local bo=bones_backup[invMap(targetIndex(j))+1]
				assert(bo)
				bones[j+1]=bo
			end
			return bones
		end

		-- map from targetIndex (==original rotJointIndex==original treeIndex-1)
		-- to new rotJointIndex of bindPose
		local invMap=intvectorn(origTargetIndex:maximum()+1)
		invMap:setAllValue(-1)
		for i=0, origTargetIndex:size()-1 do
			invMap:set(origTargetIndex(i), i)
		end

		bones =updateBonesArray(bones, targetIndex, invMap)
		assert(bindPose)
		if bindPose then
			local bindPose2=Pose()
			bindPose2:init(targetIndex:size(), 1)
			for i=0, targetIndex:size()-1 do
				bindPose2.rotations(i):assign(bindPose.rotations(invMap(targetIndex(i))))
			end
			bindPose2.translations(0):assign(bindPose.translations(0))
			currentPose=bindPose2
		end

		currentPose.rotations(0):assign(rootTF.rotation)
		currentPose.translations(0):assign(rootTF.translation)
	end

	self.currentPose=currentPose

	self.wrlInfo={ wrlTable=tbl, bones=bones, originalBones={names, pid}, targetIndex=targetIndex}
end
function FBXloader:scale(scaleFactor)
	local function scalePose(pose,sf)
		pose.translations(0):scale(sf)
	end
	local pose=self.loader:pose()
	scalePose(pose, scaleFactor)
	scalePose(self.bindpose, scaleFactor)
	if self.currentPose~=self.bindpose then
		scalePose(self.currentPose, scaleFactor)
	end
	self.loader:scale(scaleFactor, self.loader.mMotion)
	self.uid=self.uid..'_s_'..scaleFactor

	local scaleM=matrix4()
	scaleM:setScaling(scaleFactor, scaleFactor, scaleFactor)

	for isubMesh, subMesh in ipairs(self.fbxInfo) do
		local mesh=subMesh[1]
		mesh:transform(scaleM)
		local skin=subMesh.skin
		for i=0, skin:numVertex()-1 do
			skin:localPos(i):matView():assign(skin:localPos(i):matView()*scaleFactor)
		end
	end
	--self.loader:setPose(self.currentPose)
	self.loader:setPose(pose) -- 이유는 기억안나는데 loader.pose() != self.currentPose
end

function FBXloader:scaleSubtree(ibone, scaleFactor)
	assert(ibone>=1)
	local pose=self.loader:pose()
	if ibone==1 then
		self:scale(scaleFactor)
	else
		local CA=require("RigidBodyWin/subRoutines/CollisionIK")
		local isChildren=CA.checkAllChildrenExcluding(self.loader,self.loader:bone(ibone):name())
		local contained=CA.checkAllChildren(self.loader,self.loader:bone(ibone):name())
		local l=self.loader
		for i=1, l:numBone()-1 do
			if isChildren(i) then
				local offset= l:bone(i):getOffsetTransform()
				offset.translation:scale(scaleFactor)
			end
		end
		self.loader:fkSolver():init()
		local nb=l:numBone()

		for isubMesh, subMesh in ipairs(self.fbxInfo) do
			local skin=subMesh.skin
			for i=0, skin:numVertex()-1 do
				local TI=skin:treeIndices(i)
				for j=0, TI:size()-1 do
					assert(TI(j)>=1 and TI(j)<nb)
					if contained(TI(j)) then
						local lpos=skin:localPos(i)
						assert(lpos:size()==TI:size())
						lpos(j):scale(scaleFactor)
					end
				end
			end
		end
	end
	self.loader:setPose(pose)
end
function FBXloader:scaleBone(ibone, scaleFactor)
	local b=self.loader:bone(ibone)
	b=b:childHead()
	local isV3=type(scaleFactor)~='number'
	while b do
		local offset= b:getOffsetTransform()
		offset.translation:scale(scaleFactor)
		b=b:sibling()
	end
	self.loader:fkSolver():init()

	for isubMesh, subMesh in ipairs(self.fbxInfo) do
		assert(not subMesh.image or subMesh.image:GetWidth()>0)
		local skin=subMesh.skin
		for i=0, skin:numVertex()-1 do
			local TI=skin:treeIndices(i)
			for j=0, TI:size()-1 do
				if ibone==TI(j) then
					skin:localPos(i)(j):scale(scaleFactor)
				end
			end
		end
	end
end
function FBXloader:_updateBindPoseMesh()
	self.loader:setPose(self.bindpose)
	self:_bakeBindPose()
end

function FBXloader:createSkinningInfo()

	local subMesh=self.fbxInfo[1]
	self:_updateBindPoseMesh()

	if #self.fbxInfo>1 then
		-- 

		-- merge submeshes into one
		local tot_mesh=#self.fbxInfo
		local mm=util.MeshMerger(tot_mesh)
		for i=0, tot_mesh-1 do
			mm:setInputMesh(i, self.fbxInfo[i+1][1])
		end
		local mesh=Mesh()
		mm:mergeMeshes(mesh)

		subMesh={}
		subMesh[1]=mesh

		local skin=SkinnedMeshFromVertexInfo()

		skin:resize(mesh:numVertex())
		for i=0, mesh:numVertex()-1 do
			local imesh=mm:get_imesh(i)
			local ivert=mm:get_ivert(i)

			local skin_i=self.fbxInfo[imesh+1].skin
			local ti= skin:treeIndices(i)
			ti:assign(skin_i:treeIndices(ivert))
			skin:weights(i):assign(skin_i:weights(ivert))
		end

		subMesh.skin=skin
		assert(self.bindpose_global) -- assert baked
		skin:calcLocalVertexPositions(self.loader, mesh)
	end
	local fbx_info={}
	local skinScale=100
	if self.options and self.options.skinScale then
		skinScale=self.options.skinScale
	end
	fbx_info.loader=self.loader:toVRMLloader(2.5/skinScale) -- tree index can be changed!!!
	assert(fbx_info.loader.dofInfo:hasQuaternion(1)==self.loader.dofInfo:hasQuaternion(1)) 
	assert(fbx_info.loader:bone(1):getRotationalChannels()==self.loader:bone(1):getRotationalChannels()) 
	fbx_info.loader:setPose(self.bindpose)
	fbx_info.mesh=subMesh[1]


	if fbx_info.loader:numBone()~=self.loader:numBone() then
		fbx_info.skinningInfo=subMesh.skin:copy()
		local indexMap=FBXloader.getIndexMap(self.loader, fbx_info.loader)
		local skin=fbx_info.skinningInfo
		for i=0, skin:numVertex()-1 do
			local ti= skin:treeIndices(i)
			for i=0, ti:size()-1 do
				ti:set(i, indexMap[ti(i)])
			end
		end
	else
		fbx_info.skinningInfo=subMesh.skin -- share
	end

	fbx_info.bindPose=Pose()
	self.loader:getPose(fbx_info.bindPose)
	local bindposes={}
	--assert(self.loader:numBone()==fbx_info.loader:numBone()) -- can be different
	local loader=fbx_info.loader
	for i=1, loader:numBone()-1 do
		bindposes[i]= matrix4(loader:bone(i):getFrame():inverse())
	end
	fbx_info.bindposes=bindposes

	return fbx_info
end

function FBXloader:getTraj(ianim, targetIndex)
	local trajCache={}
	local fbx=self.fbx
	local info
	if fbx.getAnim2 then
		info=self.fbx:getAnimInfo(ianim)
	end
	for i=0, targetIndex:size()-1 do
		local ti=i+1
		local ilimb=targetIndex(i)


		local keytime=vectorn()
		local traj=matrixn()
		if fbx.getAnim2 then
			local nf=info(0)
			local dt=info(1)
			local T=info(2)
			fbx:getAnim2(ianim, nf, T, ilimb, keytime, traj);

			--local keytime2=vectorn()
			--local traj2=matrixn()
			--fbx:getAnim(ilimb, keytime2, traj2)
			--if (traj2-traj):length()>1e-4 then
			--	dbg.console()
			--end
		else
			fbx:getAnim(ilimb, keytime, traj)
		end
		trajCache[ti]={keytime, traj}
	end
	if targetIndex(0) ~=0 then
		-- root changed
		assert(targetIndex(0)==1)
		local keytime=vectorn()
		local traj=matrixn()
		local ilimb=0
		if fbx.getAnim2 then
			local nf=info(0)
			local dt=info(1)
			local T=info(2)
			fbx:getAnim2(ianim, nf, T, ilimb, keytime, traj);
		else
			fbx:getAnim(ilimb, keytime, traj)
		end
		trajCache[0]={keytime, traj}
	end
	return trajCache
end

function FBXloader:_getAnim(ianim, loader, motion)
	local nkey=-1
	local fbx=self.fbx
	local targetIndex=self.wrlInfo.targetIndex

	local function getTF(traj, frame)
		if traj:cols()==7 then
			return traj:row(frame):toTransf(0)
		else
			assert(traj:cols()==4)
			local T=transf()
			T.rotation:assign(traj:row(frame):toQuater(0))
			return T
		end
	end
	if fbx.getAnim2 then
		for i=ianim, self.fbx:getAnimCount()-1 do
			local info=self.fbx:getAnimInfo(i)
			if info(0)>0 then
				ianim=i
				break
			end
		end
	end
	
	local trajCache=FBXloader.getTraj(self, ianim, targetIndex)
	for i=1, loader:numBone()-1 do
		local keytime=vectorn()
		local traj=matrixn()
		local ilimb=i-1
		ilimb=targetIndex(ilimb);
		keytime, traj=unpack(trajCache[i])
		if keytime:size()==0 then
			-- no motion
			break
		end
		--assert(nkey==-1 or nkey==keytime:size())
		nkey=keytime:size()

		local frameTime=keytime(1)-keytime(0)
		if i==1 then
			motion:initEmpty(loader, nkey, frameTime)
			if ilimb~=0 then
				assert(ilimb==1)
				local keytime0, traj0=unpack(trajCache[0])
				for j=0, nkey-1 do
					local tf0=getTF(traj0,j)
					local tf=tf0*getTF(traj,j)

					motion:pose(j).rotations(0):assign(tf.rotation)
					motion:pose(j).translations(0):assign(tf.translation)
				end
			else
				for j=0, nkey-1 do
					local tf=getTF(traj, j)
					motion:pose(j).rotations(0):assign(tf.rotation)
					motion:pose(j).translations(0):assign(tf.translation)
				end
			end
		else
			local ri=loader:getRotJointIndexByTreeIndex(i)
			for j=0, nkey-1 do
				local tf=getTF(traj,j)
				motion:pose(j).rotations(ri):assign(tf.rotation)
			end
			local ti=loader:getTransJointIndexByTreeIndex(i)
			if ti~=-1 then
				for j=0, nkey-1 do
					local tf=getTF(traj, j)
					motion:pose(j).translations(ti):assign(tf.translation)
				end
			end
		end
	end
	if self.convertSkelAxis then
		for j=0, motion:numFrames()-1 do
			local pose=motion:pose(j)
			self:mirrorVA(pose.translations)
			self:mirrorQA(pose.rotations)
		end
	end
end

function FBXloader:getAnim(loader)
	if self.fbx.getAnim2 then
		self.trackNames={}
		self.trackIndex={}
		for i=0, self.fbx:getAnimCount()-1 do
			table.insert(self.trackNames, self.fbx:getAnimName(i))
			self.trackIndex[self.fbx:getAnimName(i)]=i
		end
	else
		self.trackNames={ 'default'}
	end

	if #self.trackNames>0 then
		FBXloader._getAnim(self, 0, loader, loader.mMotion)
	end
end

function FBXloader:loadTrack(trackName)
	local loader=self.loader
	self:_getAnim(self.trackIndex[trackName], loader, loader.mMotion)
end

function FBXloader:__mergeFBXloaders(loaders, options)
	for i=1, #loaders do
		print('numBone', i, loaders[i].loader:numBone())
		loaders[i]:_updateBindPoseMesh()
	end
	-- bone supersets
	local bones={}
	local boneNames={}
	for i=1, #loaders do
		local l=loaders[i].loader
		for b=1, l:numBone()-1 do
			local bname=l:bone(b):name()
			if boneNames[bname] then
				if b>=2 then
					local pname=l:bone(b):parent():name()
					local TI=boneNames[bname]
					local pTI=bones[TI].parent
					assert(pTI)
					assert(pname==bones[pTI].name)
				end
			else
				local new_TI=#bones+1
				boneNames[bname]=new_TI
				table.insert(bones, {
					i, b, name=bname, translation=l:bone(b):getOffset(),
					jointType=loaders[i].wrlInfo.bones.jointType,
					globalFrame=l:bone(b):getFrame():copy()
				})
				if b>=2 then 
					local pname=l:bone(b):parent():name()
					local pTI=boneNames[pname]
					assert(pTI)
					assert(bones[new_TI].parent==nil)
					bones[new_TI].parent=pTI
				end
			end
		end
	end
	local EEpatterns={'neck_02$','ball_.$', '.*_03_r$', '.*_03_l$'}
	local EE={}

	for i, bone in ipairs(bones) do
		for j, EEpattern in ipairs(EEpatterns) do
			if select(1, string.find(bone.name, EEpattern)) then
				table.insert(EE, i)
				bones[i].EEdist=0
				break
			end
		end
	end
	for i=#bones, 1,-1 do
		local EEdist=bones[i].EEdist 
		if EEdist and bones[i].parent then
			bones[bones[i].parent].EEdist=EEdist+1
		end
	end

	local subskel_bones={}
	local name_to_treeIndex={}
	for i=1, #bones do
		local EEdist=bones[i].EEdist 
		if EEdist then
			local obone=bones[i]
			local bname=obone.name
			local jointType="rotate"
			if i==1 then jointType=nil end
			local bone={ name=bname, frame=obone.globalFrame, translation=obone.translation, jointType=jointType}
			assert(name_to_treeIndex[bname]== nil)

			assert(bone.translation)
			table.insert(subskel_bones, bone)
			name_to_treeIndex[bname]=#subskel_bones

			local pid=bones[i].parent 
			if pid then
				bone.parent=name_to_treeIndex[bones[pid].name]
				assert(bone.parent)
			end
		end
	end

	local mloader=FBXloader.motionLoaderFromTable(subskel_bones)
	mloader:insertSiteBones()
	mloader:_initDOFinfo()
	for i, sbone in ipairs(subskel_bones) do
		mloader:getBoneByName(sbone.name):getFrame():assign(sbone.frame)
	end
	mloader:fkSolver():inverseKinematics()


	self:_setBindPose(mloader)
	local loader=mloader

	loader:setPose(self.bindpose)

	if options.exportBVH then
		local newLoader=loader

		newLoader.mMotion:initEmpty(newLoader, 10)
		for i=0, newLoader.mMotion:numFrames()-1 do
			newLoader.mMotion:pose(i):assign(self.bindpose)
		end
		newLoader.mMotion:exportBVH(options.exportBVH)

	end

	do
		local tot_mesh=0
		for i, fbx in ipairs(loaders) do
			tot_mesh=tot_mesh+#fbx.fbxInfo
		end
		assert(tot_mesh==#loaders) -- other cases can be implmented too but not yet.
		local mm=util.MeshMerger(tot_mesh)
		for i=0, tot_mesh-1 do
			mm:setInputMesh(i, loaders[i+1].fbxInfo[1][1])
		end
		local mesh=Mesh()
		mm:mergeMeshes(mesh)

		if options.drawBindPose then
			require("Kinematics/meshTools")
			local s=options.skinScale
			local _, n=mesh:drawMesh('red_transparent','mesh_merged')
			n:scale(s,s,s)
			n:setPosition(-100,0,0)
		end
		local meshInfo={ mesh, 'merged' }
		self.fbxInfo={ meshInfo}

		local skin=SkinnedMeshFromVertexInfo()
		local invMap={}
		for iloader, fbx in ipairs(loaders) do
			local skin_i=fbx.fbxInfo[1].skin
			local loader_i=fbx.loader

			invMap[iloader]=FBXloader.getIndexMap(loader_i, loader)
		end

		skin:resize(mesh:numVertex())
		for i=0, mesh:numVertex()-1 do
			local imesh=mm:get_imesh(i)
			local ivert=mm:get_ivert(i)

			local skin_i=loaders[imesh+1].fbxInfo[1].skin
			local ti= skin:treeIndices(i)
			ti:assign(skin_i:treeIndices(ivert))
			skin:weights(i):assign(skin_i:weights(ivert))
			for i=0, ti:size()-1 do
				ti:set(i, invMap[imesh+1][ti(i)])
			end
		end

		meshInfo.skin=skin
		assert(self.bindpose_global) -- assert baked
		skin:calcLocalVertexPositions(self.loader, mesh)

		if mesh:numNormal()>0 then

			meshInfo.localNormal=vector3N(mesh:numNormal())
			for i=0, mesh:numNormal()-1 do
				meshInfo.localNormal(i):assign(mesh:getNormal(i))
			end
		end
	end

	if loaders.scale then
		self:scale(1/100)
	end

end

function FBXloader:exportBinary(fn)
	-- save necessary information only.
	assert(string.sub(fn,-8)=='.fbx.dat')
	local s= util.BinaryFile()
	s:openWrite(fn, true) 
	self:__packBinary(s)
	s:close()
end

function FBXloader:__packBinary(s)
	if self.options and self.options.useTexture and self.fbx.saveTexturesToMemoryBuffer then
		-- texture 이미지들이 파일로 저장된적이 없으니, 여기서 저장해야 한다.
		self.fbx:saveTextures()
		-- todo: fbx.dat에 저장해야하나? fbx.texturecache를 그대로 쓰는게 날까? 일단 이대로 두겠음.
	end
	self:_updateBindPoseMesh()
	s:packInt(1) -- version
	s:_pack(self.loader)
	if self.rigidBody then
		s:packInt(1)
	else
		s:packInt(0)
	end
	local fbxInfo={}
	for i, v in ipairs(self.fbxInfo) do
		local info={}
		info.diffuseColor=v.diffuseColor
		info.specularColor=v.specularColor
		info.localNormal=v.localNormal
		if v.diffuseTexture then
			info.diffuseTexture=os.processFileName(v.diffuseTexture)
		end
		info.skin=v.skin
		info.shininess=v.shininess
		info[1]=v[1] -- Mesh
		info[2]=v[2] -- name
		fbxInfo[i]=info
	end
	s:packTable(fbxInfo)
	s:_pack(self.bindpose)
	s:_pack(self.currentPose)
	s:_pack(self.loader:pose())
	s:pack(self.bindpose_global)
	self.loader.mMotion:_pack(s,5) -- mot version5
end
function FBXloader:__unpackBinary(s, filename)
	local version=s:unpackInt() -- version
	assert(version==0 or version==1)
	self.loader=MotionLoader()
	s:_unpack(self.loader)
	self.fbx={}
	if s:unpackInt()==1 then
		self.rigidBody =true
	else
		self.rigidBody =false
	end
	self.fbxInfo= s:unpackTable()
	local bpose=Pose()
	s:_unpack(bpose)
	self.bindpose=bpose
	local cpose=Pose()
	s:_unpack(cpose)
	self.currentPose=cpose
	local cpose2=Pose()
	s:_unpack(cpose2)
	self.loader:setPose(cpose2)
	self.bindpose_global=quaterN()
	s:unpack(self.bindpose_global)
	if version==1 then
		self.loader.mMotion:initSkeleton(self.loader)
		local tid=s:unpackInt()
		assert(tid==12) -- postureip
		self.loader.mMotion:_unpack(s, 5) -- mot version 5
	end

	self.uid=RE.generateUniqueName()
	for i, meshInfo in ipairs(self.fbxInfo) do
		print('texture', filename)
		self:_loadTexture(meshInfo, filename)
	end
	self.textureLoaded=false -- not loaded into the gpu yet.
end
function FBXloader:_checkTexture()
	for i, meshInfo in ipairs(self.fbxInfo) do
		assert(not meshInfo.image or meshInfo.image:GetWidth()>100)
	end
end
function FBXloader:_loadAllTextures()
	self.textureLoaded=true
	for i, meshInfo in ipairs(self.fbxInfo) do
		if RE.ogreSceneManager() and meshInfo.diffuseTexture then
			local image=meshInfo.image
			if RE.renderer().createMaterial then
				-- latest taesooLib
				meshInfo.material=self.uid..meshInfo.diffuseTexture
				if image then
					assert(image:GetWidth()>0)
					RE.renderer():createDynamicTexture(self.uid..meshInfo.diffuseTexture,image , meshInfo.diffuseColor, meshInfo.specularColor, meshInfo.shininess or 10)
				else
					RE.renderer():createMaterial(self.uid..meshInfo.diffuseTexture, meshInfo.diffuseColor, meshInfo.specularColor, meshInfo.shininess or 10)
				end
			elseif image then
				assert(image:GetWidth()>0)
				RE.renderer():createDynamicTexture(self.uid..meshInfo.diffuseTexture,image , meshInfo.diffuseColor, meshInfo.specularColor)
				meshInfo.material=self.uid..meshInfo.diffuseTexture
				print("Error! cannot load "..meshInfo.diffuseTexture)
			end
		end
	end
end
function FBXloader:_setBindPose(mloader)
	local pose=Pose()
	mloader:getPose(pose)

	self.bindpose=pose

	if not self.currentPose then
		self.currentPose=pose:copy()
	end

	-- shared bindpose_global
	local bindpose_global=quaterN(mloader:numBone())
	for i=1, mloader:numBone()-1 do
		bindpose_global(i):assign(mloader:bone(i):getFrame().rotation)
	end
	self.bindpose_global=bindpose_global

	--local loader=mloader:toVRMLloader(2.5/options.skinScale)
	self.loader=mloader
	self.uid=RE.generateUniqueName()
end

function FBXloader:_loadTexture(meshInfo,filename)
	if RE.ogreSceneManager() and meshInfo.diffuseTexture then
		meshInfo.diffuseTexture=os.processFileName(meshInfo.diffuseTexture)

		local diffuseTexture
		local image=nil
		if self.fbx.saveTexturesToMemoryBuffer then
			-- first try to find in-memory images
			for i=0, self.fbx:numTexture()-1 do
				if os.filename(self.fbx:getTextureFileName(i))==meshInfo.diffuseTexture then
					image=self.fbx:getTexture(i)
				end
			end
		end
		if not image then
			local _,filepath=os.processFileName(filename)
			if os.isFileExist(meshInfo.diffuseTexture) then
				diffuseTexture=meshInfo.diffuseTexture
			elseif os.isFileExist(filepath..'/'..meshInfo.diffuseTexture) then
				diffuseTexture=filepath..'/'..meshInfo.diffuseTexture
			elseif os.isFileExist('../media12/mixamo/'..meshInfo.diffuseTexture) then
				diffuseTexture='../media12/mixamo/'..meshInfo.diffuseTexture
			elseif os.isFileExist('work/taesooLib/media12/mixamo/'..meshInfo.diffuseTexture) then
				diffuseTexture='work/taesooLib/media12/mixamo/'..meshInfo.diffuseTexture
			elseif (meshInfo.diffuseTexture:len()>_:len()) and os.isFileExist(filepath..'/'..(meshInfo.diffuseTexture:sub(_:len()-2))) then
				diffuseTexture=filepath..'/'..(meshInfo.diffuseTexture:sub(_:len()-2))
			else
				print('Warning! failed to find '..meshInfo.diffuseTexture)
			end

			if diffuseTexture then
				image=CImage()
				image:Load(diffuseTexture)
			end
		end
		if image and image:GetWidth()==0 then
			image=nil
		end
		meshInfo.image=image
		assert(not image or image:GetWidth()>0)
	end
end
function FBXloader.getIndexMap(loader_i, loader)
	local invMap_i={}
	for i=1, loader_i:numBone()-1 do
		local bone=loader_i:bone(i)
		local ti=loader:getTreeIndexByName(bone:name())
		if i==1 or ti~=-1 then
			assert(ti~=-1)
			invMap_i[i]=ti
		elseif ti==-1 then
			invMap_i[i]=invMap_i[bone:parent():treeIndex()]
			assert(invMap_i[i]~=-1)
		end
	end
	return invMap_i
end
-- currently load only mesh and skeleton.
-- to load animation, use FBXloader.motionLoader
function FBXloader:__init(filename, options)
	if not options then 
		options={
			skinScale=100
		} 
	end
	if type(filename)=='table' then
		if filename.loader then
			local s=util.MemoryFile()
			filename:__packBinary(s)
			self:__unpackBinary(s, '_temp.fbx.dat')
			s:close()
			self:_postprocessOptions('_temp.fbx.dat', options)
		else
			self:__mergeFBXloaders(filename, options)
		end
		return
	elseif type(filename)~='string' then
		-- init empty
		local loader=filename
		self:_setBindPose(loader)
		self.fbx={}
		local meshInfo={ Mesh(), 'merged' , skin= SkinnedMeshFromVertexInfo()}
		self.fbxInfo={ meshInfo }
		return
	elseif filename:sub(-8)=='.fbx.dat' then
		local s= util.BinaryFile(true) -- load to memory.
		if not s:openRead(filename) then
			error("openRead"..filename)
		end
		self:__unpackBinary(s, filename)
		s:close()
		self:_postprocessOptions(filename, options)
		return 
	end
	assert(os.isFileExist(filename))
	local fbx=util.FBXimporter(filename)

	self.fbx=fbx
	self:genSkelTable(fbx, options)

	local timer=util.Timer()
	timer:start()
	print('tbl Time=', timer:stop2())

	timer:start()
	local tbl=self.wrlInfo.wrlTable
	local targetIndex=self.wrlInfo.targetIndex
	local currentPose=self.currentPose

	local loader
	if options.useLua then
		local FBXloader_lua=require("FBXloader_lua")
		--local loader2=FBXloader.motionLoaderFromTable(self.wrlInfo.bones) --:toVRMLloader(0.25) -> vrmlloader conversion is slow and unnecesary.
		

		loader=FBXloader_lua.motionLoader(filename)

		local boneExists={}
		for i, binfo in ipairs(self.wrlInfo.bones) do
			boneExists[binfo.name]=true
		end


		local i=1
		while i<loader:numBone() do
			local bone=loader:bone(i)
			if bone:name()=='SITE' then
				print('removing bone' ..i, bone:name())
				loader:removeBone(bone)
			elseif not boneExists[bone:name()] then
				print('removing bone' ..i, bone:name())
				loader:removeBone(bone)
			else
				i=i+1
			end
		end

		local c=1
		for i=1, #self.wrlInfo.bones do
			local bname=self.wrlInfo.bones[i].name
			assert (loader:bone(c):name()==bname) 
			c=c+1
		end
		currentPose:assign(loader.mMotion:pose(0))
	else
		--local loader=MainLib.WRLloader(deepCopyTable(tbl))
		loader=FBXloader.motionLoaderFromTable(self.wrlInfo.bones) --:toVRMLloader(0.25) -> vrmlloader conversion is slow and unnecesary.
		print('wrlloader Time=', timer:stop2())
	end
	self.loader=loader
	self.uid=RE.generateUniqueName()

	self:getAnim(loader)  -- result goes to loader.mMotion

	--[[
	print(self.fbx:getAnimName(0))
	do
		local keytime=vectorn()
		local traj=matrixn()
		self.fbx:getAnim2(1,0,keytime, traj)
		dbg.console()
	end
	]]
	-- invMap: convert original tree index to new tree index
	local invMap 
	if targetIndex(0)~=0 or targetIndex~=CT.colon(0, loader:numBone()-1) then
		local ti=targetIndex
		invMap={}
		for i=0, ti:size()-1 do
			local treeIndex=i+1
			local origTreeIndex=ti(i)+1
			invMap[origTreeIndex]=treeIndex
		end
		if invMap[1]==nil then
			invMap[1]=1 
		end
		local originalBones, pid=unpack(self.wrlInfo.originalBones)
		local nameToOrigTreeIndex={}
		for i=0, originalBones:size()-1 do
			local originalTI=i+1
			nameToOrigTreeIndex[originalBones(i)]=originalTI
		end
		local newRootOrigTI=ti(0)+1
		for i=newRootOrigTI+1, originalBones:size() do
			local originalTI=i
			if invMap[originalTI]==nil then
				local TI=originalTI
				local cand={TI}
				while true do
					if pid(TI-1)<0 then
						break
					end
					local parentName=originalBones(pid(TI-1))
					local pTI=nameToOrigTreeIndex[parentName]
					if not pTI then
						if TI==1 then
							break
						end
						dbg.console()
					end
					if invMap[pTI] then
						for ii, ti in ipairs(cand) do
							invMap[ti]=invMap[pTI]
						end
						break
					end
					table.insert(cand, pTI)
					TI=pTI
				end
			end
		end
	end
	--self.loader:setPose(currentPose)
	local mesh_count=fbx:getMeshCount()


	self.bindpose=self.currentPose
	loader:setPose(self.bindpose)


	local chooseLOD=1
	for i=1, mesh_count do
		local meshName=fbx:getMeshName(i-1)
		if chooseLOD>0 then
			if not select(1, string.find(meshName, '_LOD.$')) then
				chooseLOD=-1
				break
			end
			local lodNo=tonumber(string.sub(meshName, -1))
			local curLodNo=tonumber(string.sub(fbx:getMeshName(chooseLOD-1), -1))

			if options.chooseLOD then
				if lodNo==options.chooseLOD then
					chooseLOD=i
				end
			else
				if lodNo>curLodNo then
					chooseLOD=i
				end
			end
		end
	end

	local meshList={}
	if chooseLOD>0 then
		meshList={chooseLOD-1}

	else
		for i=0, mesh_count-1 do
			meshList[i+1]=i
		end
	end
	if mesh_count==0 then
		meshList={}
	end
	local fbxInfo={}
	for ilist, i in ipairs(meshList) do
		local mesh=Mesh()
		local mesh_name=fbx:getMesh(i, mesh)

		local useRestPoseAsBindPose=false
		if loader.mMotion:numFrames()> 0 and self.fbx:getMeshCount()>1 then
			-- bigvegas_Walking.fbx의 bindpose가 이상해서 예외처리.
			-- (motion이 있고, meshcount가 2 이상인경우 bindpose 대신 restpose사용-.-)
			useRestPoseAsBindPose=true
		end
		if options.useMeshBindPose then
			-- override 
			useRestPoseAsBindPose=false
		end

		if not useRestPoseAsBindPose and not self.rigidBody then
			-- use mesh bind pose
			self.fbx:countBindPoses(i) -- update.
			local jointpos=vector3N()
			local jointori=quaterN()
			self:getBindPose(jointpos, jointori)
			if invMap then
				for i=0, jointpos:size()-1 do
					local newTI=invMap[i+1]
					if newTI then
						loader:bone(newTI):getFrame().rotation:assign(jointori(i))
						loader:bone(newTI):getFrame().translation:assign(jointpos(i))
					end
				end
			else
				for i=0, jointpos:size()-1 do
					loader:bone(i+1):getFrame().rotation:assign(jointori(i))
					loader:bone(i+1):getFrame().translation:assign(jointpos(i))
				end
			end
		end

		local inv_bindpose_scale
		if self.rigidBody then
			inv_bindpose_scale=1.0
		else
			inv_bindpose_scale=1.0/fbx:getBindPose(targetIndex(0)):getColumn(0):length()
		end


		local pose=fbx:getMeshCurrPose(i)
		pose:leftMultScaling(inv_bindpose_scale, inv_bindpose_scale, inv_bindpose_scale)
		mesh:transform(pose)

		if self.convertMeshAxis then
			for v=0, mesh:numVertex()-1 do
				mesh:getVertex(v):assign(self:mirror(mesh:getVertex(v)))
				mesh:getNormal(v):assign(self:mirror(mesh:getNormal(v)))
			end

			if self.options.mirror then
				for v=0, mesh:numFace()-1 do
					local f=mesh:getFace(v);
					mesh:getFace(v):setIndex(f:vertexIndex(0), f:vertexIndex(2), f:vertexIndex(1), Mesh.VERTEX)
					if mesh:numNormal()>0 then
						mesh:getFace(v):setIndex(f:normalIndex(0), f:normalIndex(2), f:normalIndex(1), Mesh.NORMAL)
					end
					if mesh:numTexCoord()>0 then
						mesh:getFace(v):setIndex(f:texCoordIndex(0), f:texCoordIndex(2), f:texCoordIndex(1), Mesh.TEXCOORD)
					end
					if mesh:numColor()>0 then
						mesh:getFace(v):setIndex(f:colorIndex(0), f:colorIndex(2), f:colorIndex(1), Mesh.COLOR)
					end
				end
			end
		end
		if self.options.flipNormal then
			for v=0, mesh:numVertex()-1 do
				mesh:getNormal(v):assign(-1*mesh:getNormal(v))
			end

			for v=0, mesh:numFace()-1 do
				local f=mesh:getFace(v);
				mesh:getFace(v):setIndex(f:vertexIndex(0), f:vertexIndex(2), f:vertexIndex(1), Mesh.VERTEX)
			end
		end

		if options.drawBindPose then
			require("Kinematics/meshTools")
			local s=options.skinScale
			local _, n=mesh:drawMesh('red_transparent','mesh'..i)
			n:scale(s,s,s)
		end

		--print(pose)
		local meshInfo={ mesh, mesh_name, }

		if chooseLOD>0 then
			self.fbxInfo={
				meshInfo
			}
		else
			fbxInfo[i+1]=meshInfo
		end


		local skin=SkinnedMeshFromVertexInfo()
		fbx:getSkinnedMesh(i, skin)

		--print(mesh_name, mesh:numVertex())
		meshInfo.skin=skin
		if invMap then
			-- root changed so...
			local nv=skin:numVertex()
			for i=0, nv-1 do
				local indices=skin:treeIndices(i)
				for j=0, indices:size()-1 do
					local newTI=invMap[indices(j)]
					assert(newTI)
					indices:set(j, newTI)
				end
			end
		end

		-- todo: options.simplifyMesh
		if options.simplifyMesh then
			local reduceFraction=0.5
			local agressiveness=7.0
			if type(options.simplifyMesh)=='table' then
				reduceFraction=options.simplifyMesh.reduceFraction
				agressiveness=options.simplifyMesh.agressiveness
			end
			local nv=skin:numVertex()
			local weights=CT.zeros(nv, loader:numBone())
			for vi=0, nv-1 do
				local indices=skin:treeIndices(vi)
				weights:row(vi):setAt(indices, skin:weights(vi))
			end
			local mesh2=Mesh()
			local weights2=matrixn()
			local succeeded=meshInfo[1]:simplify(mesh2, weights, weights2, reduceFraction, agressiveness)

			if succeeded then
				meshInfo[1]=mesh2
				skin:resize(weights2:rows())
				local nv=weights2:rows()
				for i=0, nv-1 do
					local values=weights2:row(i):extractNonZeroValues(skin:treeIndices(i))
					assert(values:sum()>0.9)
					skin:weights(i):assign(values)
				end
				mesh=mesh2
			end
		end

		local bindpose_global=quaterN(loader:numBone())
		for i=1, loader:numBone()-1 do
			bindpose_global(i):assign(loader:bone(i):getFrame().rotation)
		end
		meshInfo.bindpose_global=bindpose_global

		skin:calcLocalVertexPositions(self.loader, mesh)

		if mesh:numNormal()>0 then

			meshInfo.localNormal=vector3N(mesh:numNormal())
			for i=0, mesh:numNormal()-1 do
				meshInfo.localNormal(i):assign(mesh:getNormal(i))
			end
		end
	end
	if chooseLOD<0 then
		self.fbxInfo=fbxInfo
	end
	if options.useTexture or options.useTexCoord then
		if options.useTexture then
			if fbx.saveTexturesToMemoryBuffer then
				-- latest taesooLib
				if os.isFileExist(filename..'.texturecache') then
					-- png로딩이 느려서 미리 저장한 uncompressed를 로딩.
					local f=util.BinaryFile(false, filename..'.texturecache')

					local timer=util.Timer()
					fbx:unpackTextures(f)
					f:close()
					print('unpack textures', timer:stop2())
				else
					fbx:saveTexturesToMemoryBuffer()
					-- png로딩이 느려서 그냥 uncompressed로 다시 저장.
					local f=util.BinaryFile(true, filename..'.texturecache')
					fbx:packTextures(f)
					f:close()
				end
			else
				fbx:saveTextures()
			end
		end
		for ilist, i in ipairs(meshList) do
			local c=fbx:getMaterialCount(i)
			for j=0, c-1 do
				--print('diffuse', i, fbx:getDiffuseColor(i, j), fbx:getDiffuseTexture(i,j))
				--print('specular', i, fbx:getSpecularColor(i, j))
			end
			if c>0 then
				local j=0
				local meshInfo= self.fbxInfo[ilist]
				meshInfo.diffuseColor=fbx:getDiffuseColor(i, j)
				meshInfo.specularColor=fbx:getSpecularColor(i, j)
				assert(meshInfo.specularColor.x==meshInfo.specularColor.x)

				if fbx.getMaterialPropertyTypes then
					-- todo: support shiniess
					--local types=fbx:getMaterialPropertyTypes (i,j)
					local shininess=fbx:getMaterialProperty (i,j,'ShininessExponent')
					if shininess:size()>0 then
						meshInfo.shininess=shininess(0)
					end
				end
				if not meshInfo.shininess then
					meshInfo.shininess =10
				end
				meshInfo.diffuseTexture=filename:sub(1,-5)..'_'..os.filename(fbx:getDiffuseTexture(i,j))

				if meshInfo.diffuseTexture:endsWith('.png') then
					-- todo: support transparency
				end

				self:_loadTexture(meshInfo, filename)
			end
		end
		--print(fbx:getAllTextureNames())
	end

	self:_postprocessOptions(filename, options)
end

function FBXloader:_postprocessOptions(filename,options)
	if options.cleanupBones then
		require("FBXloader_tools").FBXloader_cleanupBones(self, options.cleanupBones)
	end
	if options.scale then
		self:scale(options.scale)
	end
	if options.boneScale then
		for k, v in pairs(options.boneScale) do
			local ibone=self.loader:getTreeIndexByName(k)
			assert(ibone~=-1)
			self:scaleSubtree(ibone, v)
		end
	end
	if options.currentPoseAsIdentity then
		self:setCurPoseAsInitialPose()
	end
	if options.useBindPoseAsIdentityPose then
		self:useBindPoseAsIdentityPose()
	end
	if options.cacheCollisionMesh then
		if type(options.cacheCollisionMesh )=='string' then
			assert( type(filename)=='string' )
			assert(
			options.cacheCollisionMesh :sub(-10,-2)=='.colcache' or 
			options.cacheCollisionMesh :sub(-11,-3)=='.colcache' 
			)
			assert(tonumber(options.cacheCollisionMesh :sub(-1))~=nil)
			self.cacheCollisionMesh=filename..options.cacheCollisionMesh 
		elseif type(filename)=='string' then
			--self.cacheCollisionMesh=filename..'.colcache' -- single convex per bone
			self.cacheCollisionMesh=filename..'.colcache6' -- six convexes per bone
		else
			print('cacheCollisionMesh: this case is not supported.')
		end
	end
	if options.boneNamePrefix then
		local loader=self.loader
		for i=1, loader:numBone()-1 do
			loader:bone(i):setName( options.boneNamePrefix.. loader:bone(i):name())
		end
	end
	if options.renameBones then
		local loader=self.loader
		for i=1, loader:numBone()-1 do
			local newname=string.gsub(loader:bone(i):name(), unpack(options.renameBones))
			loader:bone(i):setName(newname)
		end
	end
end
-- the original wrl table is changed in place.
function FBXloader.changeRoot(wrl, bones, newRootBoneIndex, origTargetIndex)
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

	return targetIndex

end
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
function FBXloader.cleanupBones(wrl, bones, origTargetIndex, EE, bindPose)
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
		if not bone.children then
			return 
		end
		local newChildren={}
		for ic, c in ipairs(bone.children) do
			if boneInfo[nameToTreeIndex[c.name]].visited then
				table.insert(newChildren, c)
				removeUnvisitedChildren(c)
			else
				print('removing '..c.name )
			end
		end
		bone.children=newChildren
	end

	removeUnvisitedChildren(wrl.body)

	local targetIndex=intvectorn()
	targetIndex:pushBack(origTargetIndex(0))
	local function updateTargetIndex(bone, targetIndex)
		if not bone.children then return end
		for ic, c in ipairs(bone.children) do
			targetIndex:pushBack(nameToTreeIndex[c.name]-1)
			updateTargetIndex(c, targetIndex)
		end
	end
	updateTargetIndex(wrl.body, targetIndex)

	return targetIndex
end

FBXloader.Skin=LUAclass()
function FBXloader.Skin:__init(fbxloader, option)
	if not option then option={} end
	self.scale=vector3(1,1,1)
	self.fbx=fbxloader
	self.fkSolver=fbxloader.loader:fkSolver():copy()
	if not fbxloader.fbxInfo then
		option.drawSkeleton=true
	end
	if option.drawSkeleton then
		self.skelSkin=RE.createSkin(fbxloader.loader, PLDPrimSkin.LINE)
		self.skelSkin:setPose(fbxloader.loader:pose())
	end
	self.uid=RE.generateUniqueName()

	self.nodes={}
	self.ME={}

	if not fbxloader.fbxInfo then
		return
	end
	for i, meshInfo in ipairs(fbxloader.fbxInfo) do
		local node, meshToEntity, entity
		if RE.ogreSceneManager() then
			local mesh=meshInfo[1]

			local useTexCoord=false
			local useColor=false
			local useNormal=true
			local buildEdgeList=not option.disableShadow
			local dynamicUpdate=true

			if mesh:numNormal()==0 then
				useNormal=false
			end
			if mesh:numTexCoord()>0 then
				useTexCoord=true
			end
			if mesh:numColor()>0 then
				useColor=true
			end
			-- scale 100 for rendering 
			meshToEntity=MeshToEntity(mesh, self.uid..'meshName'..i, buildEdgeList, dynamicUpdate, useNormal, useTexCoord, useColor)

			self.ME[i]=meshToEntity
			meshToEntity=self.ME[i]
			entity=meshToEntity:createEntity('entityName'..self.uid..'_'..i )
			--entity:setMaterialName('white')
			if option.material then
				entity:setMaterialName(option.material )
			elseif meshInfo.material then
				entity:setMaterialName(meshInfo.material )
			else
				entity:setMaterialName('grey_transparent')
			end
			node=RE.createChildSceneNode(RE.ogreRootSceneNode(), self.uid..'_'..i)
			if self.fbx.rigidBody then
				local node2=RE.createChildSceneNode(node, self.uid..'__'..i)
				node2:attachObject(entity)

				if not self.nodes2 then
					self.nodes2={}
				end
				self.nodes2[i]=node2
			else
				node:attachObject(entity)
			end
		else
			node=Ogre.SceneNode()
		end
		self.nodes[i]= {node, vector3(0,0,0), vector3N()}

	end
	self:setPose(fbxloader.loader:pose())
end

function FBXloader.Skin:applyAnim(motion)
	require('subRoutines/Timeline')
	self.timeline=Timeline('fbx', motion:numFrames(), 1/motion:frameRate())
	local EVR=LUAclass(EventReceiver)
	EVR.__init=function(self)
	end
	EVR.onFrameChanged=function (self, win, iframe)
		if not self.skin then return end
		if iframe<self.motion:numFrames() then
			self.skin:setPose(self.motion:pose(iframe))
		end
	end
	self.EVR=EVR()
	self.EVR.skin=self
	self.EVR.motion=motion
end
function FBXloader.Skin:applyMotionDOF(dof)
	require('subRoutines/Timeline')
	self.timeline=Timeline('fbx', dof:numFrames(), 1/30)
	local EVR=LUAclass(EventReceiver)
	EVR.__init=function(self)
	end
	EVR.onFrameChanged=function (self, win, iframe)
		if iframe<self.motion:numFrames() then
			self.skin:setPoseDOF(self.motion(iframe))
		end
	end
	self.EVR=EVR()
	self.EVR.skin=self
	self.EVR.motion=dof
end


function FBXloader.Skin:getNode(meshIndex)
	return self.nodes[meshIndex][1]
end
function FBXloader.Skin:getNodeInfo(meshIndex)
	return self.nodes[meshIndex]
end
-- return1 mesh: the index array is valid, but its vertex positions/normals member are invalid 
-- return2 vertex array. (corresponding to the current pose of the skin)
function FBXloader.Skin:getMesh(meshIndex)
	return self.fbx.fbxInfo[meshIndex][1], self.nodes[meshIndex][3]
end

function FBXloader.Skin:numMesh()
	return #self.nodes
end


function FBXloader.Skin:getState() 
	return self.fkSolver
end
function FBXloader.Skin:setVisible(bVisible)
	if self.skelSkin then
		self.skelSkin:setVisible(bVisible)
	end
	for i, v in ipairs(self.nodes) do
		v[1]:setVisible(bVisible)
	end
end
function FBXloader.Skin:dtor()
	self.skelSkin=nil
	for i, v in ipairs(self.nodes) do
		RE.removeEntity(v[1])
	end
	if self.EVR then
		self.EVR.skin=nil
		self.EVR=nil
	end
	self.timeline=nil
	self.nodes=nil
	self.fbx=nil
	self.fkSolver=nil
end
function FBXloader.Skin:setMaterial(name)
	for i, me in ipairs(self.ME) do
		me:getLastCreatedEntity():setMaterialName(name)
	end
end
function FBXloader.Skin:unsetMaterial()
	for i, me in ipairs(self.ME) do
		local meshInfo= self.fbx.fbxInfo[i]
		me:getLastCreatedEntity():setMaterialName(meshInfo.material )
	end
end

-- assuming self.loader==mLoader_orig
-- poseconv=MotionUtil.PoseTransfer(mLoader, mLoader_orig, true)
function FBXloader.Skin:setPoseTransfer(poseconv)
	self.poseMap=poseconv
	if poseconv.pt then
		self.angleRetarget=poseconv
		self.poseMap=poseconv.pt
	end
end
function FBXloader.Skin:setPose(pose)
	if self.poseMap then
		if self.angleRetarget then
			pose=self.angleRetarget(pose)
		else
			self.poseMap:setTargetSkeleton(pose)
			pose=Pose()
			self.poseMap:target():getPose(pose)
		end
	end
	self.fkSolver:setPose(pose)
	self:setSamePose(self.fkSolver)
end
function FBXloader.Skin:_setPose(pose, loader)
	self:setPose(pose)
end
function FBXloader.Skin:setPoseDOF(pose)
	if self.poseMap then
		if self.angleRetarget then
			pose=self.angleRetarget(pose)
		else
			self.poseMap:source():setPoseDOF(pose)
			pose=Pose()
			self.poseMap:source():getPose(pose)
			self.poseMap:setTargetSkeleton(pose)
			self.poseMap:target():getPose(pose)
		end
		self.fkSolver:setPose(pose)
	else
		self.fkSolver:setPoseDOF(pose)
	end
	self:setSamePose(self.fkSolver)
end
function FBXloader.Skin:setSamePose(fk)
	--assert(fk==self.fbx.loader:fkSolver()) BoneForwardKinematics.operator==  doesn't work yet.
	if self.skelSkin then
		self.skelSkin:setSamePose(fk)
	end
	self.fkSolver:assign(fk)
	local fbxloader=self.fbx
	if fbxloader.rigidBody then
		for i, node2 in ipairs(self.nodes2) do
			node2:setOrientation(fk:globalFrame(1).rotation)
			node2:setPosition(fk:globalFrame(1).translation)
		end
		return
	end
	local rootTrans=fk:globalFrame(1).translation:copy()
	local currJointPos=vector3N(self.fkSolver:numBone()-1)
	for ibone=1, self.fkSolver:numBone()-1 do
		--self.fkSolver:globalFrame(ibone).translation:rsub(rootTrans)
		currJointPos(ibone-1):assign(self.fkSolver:globalFrame(ibone).translation-rootTrans)
	end
	local buildEdgeList=false
	--if self.prevPose then print(self.prevPose:MSE(currJointPos)) end
	if self.prevPose==nil or self.prevPose:MSE(currJointPos)>1e-3 then
		buildEdgeList=true
		self.prevPose=currJointPos
	end

	if not fbxloader.fbxInfo then
		return 
	end
	for i, meshInfo in ipairs(fbxloader.fbxInfo) do
		local ME=self.ME[i]
		if ME then
		local node=self.nodes[i]
		local skin=meshInfo.skin
		local mesh=meshInfo[1]

		skin:calcVertexPositions(self.fkSolver, mesh)
		local removeRootTrans=matrix4()
		removeRootTrans:setTranslation(-rootTrans, false)
		mesh:transform(removeRootTrans)

		local useNormal=mesh:numNormal()>0
		ME:setBuildEdgeList(buildEdgeList)

		if useNormal then
			skin:calcVertexNormals(self.fkSolver, fbxloader:_getBindPoseGlobal(i), meshInfo.localNormal, mesh)
			ME:updatePositionsAndNormals()
		else
			ME:updatePositions()
		end
		if mesh.getVertices then
			mesh:getVertices(node[3]) -- backup current mesh pose
		end
		node[1]:setPosition(node[2]+rootTrans*self.scale.x)
		end
	end
end
function FBXloader.Skin:setScale(x,y,z)
	if not y then y=x end
	if not z then z=x end
	if self.skelSkin then
		self.skelSkin:setScale(x,y,z)
	end
	for i ,node in ipairs(self.nodes) do

		local prevPos=node[1]:getPosition()
		node[1]:setScale(x,y,z)
		node[1]:setPosition(
		prevPos.x/self.scale.x*x,
		prevPos.y/self.scale.y*y,
		prevPos.z/self.scale.z*z)
	end
	self.scale=vector3(x,y,z)
end
function FBXloader.Skin:_getVertexInWorld(meshIndex, vertexIndex)
	local meshInfo=self.fbx.fbxInfo[meshIndex]
	local skin=meshInfo.skin

	return skin:calcVertexPosition(self.fkSolver, vertexIndex)
end

-- use proper ray pick
function FBXloader.Skin:pickTriangle(x,y)
	local ray=Ray()
	RE.FltkRenderer():screenToWorldRay(x, y,ray)
	return self:_pickTriangle(ray)
end

-- vertexPositions: relative to root translation.
-- return { mesh:Mesh, skin:SkinnedMeshFromVertexInfo, vertexPositions:vector3N}
function FBXloader.Skin:getMeshInfo(fbxMeshIndex)
	if not fbxMeshIndex then
		fbxMeshIndex=1
	end
	local node=self.nodes[fbxMeshIndex]
	local meshInfo=self.fbx.fbxInfo[fbxMeshIndex]
	local mesh=meshInfo[1]
	return { mesh=meshInfo[1], skin=meshInfo.skin, vertexPositions=node[3], }
end
-- input ray will be modified!
function FBXloader.Skin:_pickTriangle(ray)
	local s=self.scale.x
	local fbxMeshIndex=1 -- TODO: loop through all meshes
	local node=self.nodes[fbxMeshIndex]
	local meshPos=node[2]+self.fkSolver:globalFrame(1).translation*s
	local out=vector3()
	ray:translate(-meshPos)
	ray:scale(1/s)

	local out=vector3()
	local baryCoeffs=vector3()
	local mesh=self.fbx.fbxInfo[fbxMeshIndex][1]
	local ti=ray:pickBarycentric(mesh,node[3],  baryCoeffs, out)

	if ti>=0 then
		return { faceIndex=ti, baryCoeffs=baryCoeffs, pos=meshPos+out*s}
	else
		return nil
	end
end
function FBXloader.Skin:samplePosition(meshIndex, ti, baryCoeffs)
	local meshInfo=self.fbx.fbxInfo[meshIndex]
	local mesh=meshInfo[1]

	local f=mesh:getFace(ti)
	local v1, v2, v3
	if mesh.getVertices then
		local rootTrans=self.fkSolver:globalFrame(1).translation:copy()
		local node=self.nodes[meshIndex]
		local vertices=node[3]
		v1=vertices(f:vertexIndex(0))+rootTrans
		v2=vertices(f:vertexIndex(1))+rootTrans
		v3=vertices(f:vertexIndex(2))+rootTrans
	else
		v1=self:_getVertexInWorld(meshIndex, f:vertexIndex(0))
		v2=self:_getVertexInWorld(meshIndex, f:vertexIndex(1))
		v3=self:_getVertexInWorld(meshIndex, f:vertexIndex(2))
	end
	local out2=v1*baryCoeffs.x+v2*baryCoeffs.y +v3*baryCoeffs.z
	return out2
end
function FBXloader.Skin:setTranslation(x,y,z)
	if y==nil then
		assert(x)
		y=x.y
		z=x.z
		x=x.x
	end
	if self.skelSkin then
		self.skelSkin:setTranslation(x,y,z)
	end
	for i ,node in ipairs(self.nodes) do
		node[1]:setPosition(node[1]:getPosition()-node[2]+vector3(x,y,z))
		node[2]=vector3(x,y,z)
	end
end

function RE.createFBXskin(fbxloader, drawSkeleton)
	if not fbxloader.textureLoaded then
		fbxloader:_loadAllTextures()
	end
	if not fbxloader.loader then
		return RE.createSkinAuto(fbxloader)
	end
	if type(drawSkeleton)=='boolean' then
		return FBXloader.Skin(fbxloader, { drawSkeleton=drawSkeleton})
	elseif drawSkeleton and drawSkeleton.adjustable then
		return FBXloader.AdjustableSkin(fbxloader, drawSkeleton)
	else
		return FBXloader.Skin(fbxloader, drawSkeleton)
	end
end

FBXloader.Converter=LUAclass()
function FBXloader.Converter:__init(filename)
	if filename then
		if type(filename)=='string' then
			assert(filename:sub(-4):upper()=='.BVH')
			self.loader=MotionLoader._create(filename)
		else
			self.loader=filename
		end
	end
end
function FBXloader.Converter:getMeshCount() return 0 end
function FBXloader.Converter:hasBindPose(i) return false end
function FBXloader.Converter:getRestPose(jointpos, jointori) 
	local l=self.loader
	local n=l:numRotJoint()
	jointpos:setSize(n);
	jointori:setSize(n);
	for i=0, n-1 do
		local ti=l:getTreeIndexByRotJointIndex(i)
		local f=l:bone(ti):getFrame()
		jointpos(i):assign(f.translation)
		jointori(i):assign(f.rotation)
	end
end
function FBXloader.Converter:getAnim(ilimb, keytime, traj)
	local l=self.loader
	local m=l.mMotion
	local nf=m:numFrames()
	keytime:resize(nf)
	local t=0
	local f=1.0/m:frameRate()
	for i=0, nf-1 do
		keytime:set(i, t)
		t=t+f
	end


	if ilimb==0  then
		traj:setSize(nf,7)
		for i=0, nf-1 do
			local q=m:pose(i).rotations(0)
			local p=m:pose(i).translations(0)
			traj:row(i):setTransf(0, transf(q,p))
		end
	else
		traj:setSize(nf,4)
		for i=0, nf-1 do
			local q=m:pose(i).rotations(ilimb)
			traj:row(i):setQuater(0, q)
		end
	end
end
function FBXloader.Converter:parentIndices()
	local l=self.loader
	local n=l:numRotJoint()
	local out=intvectorn()
	out:setSize(n)
	out:set(0,-1)
	for i=1, n-1 do
		local ti=l:getTreeIndexByRotJointIndex(i)
		out:set(i, l:bone(ti):parent():rotJointIndex())
	end
	return out
end
function FBXloader.Converter:getBoneNames(names)
	local l=self.loader
	local n=l:numRotJoint()
	names:resize(n)
	for i=0, n-1 do
		local ti=l:getTreeIndexByRotJointIndex(i)
		local name=l:bone(ti):name()
		names:set(i, name)
	end
end
function FBXloader.motionLoader(filename, options)
	if not options then options={skinScale=100} end
	local info={}
	info.mirrorVA=FBXloader.mirrorVA
	info.mirrorQA=FBXloader.mirrorQA

	local fbx
	if type(filename)=='string' and filename:sub(-4):upper()=='.FBX' then
		fbx=util.FBXimporter(filename)
	elseif type(filename)=='string' and filename:sub(-8):upper()=='.FBX.DAT' then
		local l=FBXloader(filename)
		return l.loader
	else
		fbx=FBXloader.Converter(filename)
	end
	info.fbx=fbx
	info.getBindPose=FBXloader.getBindPose
	info.checkRelativeRestPose=FBXloader.checkRelativeRestPose
	FBXloader.genSkelTable(info, fbx, options)

	local timer=util.Timer()
	timer:start()
	print('tbl Time=', timer:stop2())

	timer:start()
	local bones=info.wrlInfo.bones
	local targetIndex=info.wrlInfo.targetIndex
	local currentPose=info.currentPose

	local trajCache=FBXloader.getTraj(info, 0, targetIndex)

	for i, v in ipairs(bones) do
		if trajCache[i][2]:cols()==7 then
			v.jointType=nil-- use free joint
		end
	end
	local loader=FBXloader.motionLoaderFromTable(bones, targetIndex)

	FBXloader.getAnim(info,loader)
	loader:insertSiteBones()

	if options.getTargetIndex then
		return loader, targetIndex
	end
	return loader
end

-- returns a skeleton without site nodes, so need to add site nodes before exporting to BVH
function FBXloader.motionLoaderFromTable(bones)
	local loader=MotionLoader()
	loader:createDummyRootBone()

	local mapBones={}
	local parent={}
	for i, bone in ipairs(bones) do
		if bone.children then
			for j, child in ipairs(bone.children) do
				parent[child.name]=i
			end
		end
	end

	for i, bone in ipairs(bones) do
		mapBones[bone.name]=bone
		local parent_treeIndex=bone.parent or parent[bone.name]
		if parent_treeIndex then
			--loader:insertChildBone(loader:bone(parent_treeIndex), bone.name, false)
			loader:insertChildBone(loader:getBoneByName(bones[parent_treeIndex].name), bone.name, false)
		else
			loader:insertChildBone(loader:bone(0), bone.name, false)
		end
		local b=loader:getBoneByName(bone.name)

		if bone.translation then
			if i~=1 then
				b:getOffsetTransform().translation:assign(bone.translation)
			end
		end

		if not bone.jointType or bone.jointType=='free' then
			loader:insertJoint(b, "RT")
		elseif bone.jointType=="rotate" then
			loader:insertJoint(b, "R")
		elseif bone.jointType=="translate" then
			loader:insertJoint(b, "T")
		else
			assert(bone.jointType=='fixed')
		end
	end

	--for i=1, loader:numBone()-1 do
	--	bones[i]=loader:bone(i)
	--end

	loader:_initDOFinfo()
	--loader:printHierarchy()

	return loader
end

function FBXloader:getBindPose(jointpos, jointori)
	local fbx=self.fbx
	local numLimb=self.fbxNumLimbNodes
	if not numLimb then
		dbg.console()
	end

	local bindposes={}
	for i=0, numLimb-1 do
		local pose=fbx:getBindPose(i)
		bindposes[i+1]=pose
	end
	local inv_bindpose_scale=1.0/bindposes[1]:getColumn(0):length()
	for i, v in ipairs(bindposes) do
		v:leftMultScaling(inv_bindpose_scale, inv_bindpose_scale, inv_bindpose_scale)
	end
	jointpos:resize(numLimb)
	jointori:resize(numLimb)
	for i=0, numLimb-1 do
		local tf=transf(bindposes[i+1])
		jointpos(i):assign(tf.translation)
		jointori(i):assign(tf.rotation)
	end

	if self.convertSkelAxis then
		self:mirrorVA(jointpos)
		self:mirrorQA(jointori)
	end

end
function FBXloader:drawPose(jointpos, jointori, options, prefix, offset)
	--local delta=jointpos(1):copy()*options.skinScale
	--delta.y=0
	for i=0, jointpos:size()-1 do
		local tf=transf()
		--tf.translation:assign(jointpos(i)*options.skinScale-delta+offset)
		tf.translation:assign(jointpos(i)*options.skinScale+offset)
		tf.rotation:assign(jointori(i))
		dbg.draw('Axes', tf, prefix..i, 1)
	end
end
function FBXloader:drawRestPose(options)
	local fbx=self.fbx
	local jointpos=vector3N()
	local jointori=quaterN()
	fbx:getRestPose(jointpos, jointori)

	self:drawPose(jointpos, jointori, options, 'restpose', vector3(-200,0,0))
end
function FBXloader:checkRelativeRestPose(bindpose_pos, bindpose_ori) 
	local fbx=self.fbx
	local jointpos=vector3N()
	local jointori=quaterN()
	fbx:getRestPose(jointpos, jointori)
	local diffpos=bindpose_pos-jointpos
	local len=0
	for i=0, diffpos:size()-1 do
		if fbx:hasBindPose(i) then
			len=len+diffpos(i):length()
		end
	end
	if len<0.01 then
		-- compatible.
		-- then, use relative pose as the bindpose
		bindpose_ori:assign(jointori)
	end
end


FBXloader.AdjustableSkin=LUAclass(FBXloader.Skin,true)

function FBXloader.AdjustableSkin:__init(fbxloader, option)
	if not option then option={} end
	self.scale=vector3(1,1,1)
	self.fbx=fbxloader
	self.fkSolver=util.ScaledBoneKinematics(fbxloader.loader)
	if option.drawSkeleton then
		self.skelSkin=RE.createSkin(fbxloader.loader, PLDPrimSkin.LINE)
		self.skelSkin:setPose(fbxloader.loader:pose())
	end
	self.uid=RE.generateUniqueName()

	self.nodes={}
	self.ME={}

	for i, meshInfo in ipairs(fbxloader.fbxInfo) do
		local meshToEntity, entity, node
		if RE.ogreSceneManager() then
			local mesh=meshInfo[1]

			local useTexCoord=false
			local useColor=false
			local useNormal=true
			local buildEdgeList=not option.disableShadow
			local dynamicUpdate=true

			if mesh:numNormal()==0 then
				useNormal=false
			end
			if mesh:numTexCoord()>0 then
				useTexCoord=true
			end
			if mesh:numColor()>0 then
				useColor=true
			end
			-- scale 100 for rendering 
			meshToEntity=MeshToEntity(mesh, self.uid..'meshName'..i, buildEdgeList, dynamicUpdate, useNormal, useTexCoord, useColor)

			self.ME[i]=meshToEntity
			entity=meshToEntity:createEntity('entityName'..self.uid..'_'..i )
			--entity:setMaterialName('white')
			if option.material then
				entity:setMaterialName(option.material )
			elseif meshInfo.material then
				entity:setMaterialName(meshInfo.material )
			else
				entity:setMaterialName('grey_transparent')
			end
			node=RE.createChildSceneNode(RE.ogreRootSceneNode(), self.uid..'_'..i)
			if self.fbx.rigidBody then
				local node2=RE.createChildSceneNode(node, self.uid..'__'..i)
				node2:attachObject(entity)

				if not self.nodes2 then
					self.nodes2={}
				end
				self.nodes2[i]=node2
			else
				node:attachObject(entity)
			end
		else
			node=Ogre.SceneNode()
		end
		self.nodes[i]= {node, vector3(0,0,0), vector3N()}

	end
	self:setPose(fbxloader.loader:pose())
end
function FBXloader.AdjustableSkin:setLengthAndPose(length_scale, pose)
	self:_setLengthOnly(length_scale)
	self:setPose(pose)
end
function FBXloader.AdjustableSkin:setLengthAndPoseDOF(length_scale, pose)
	self:_setLengthOnly(length_scale)
	self:setPoseDOF(pose)
end
-- change length while maintaining pose
function FBXloader.AdjustableSkin:setLengthScale(length_scale)
	self:_setLengthOnly(length_scale)
	self.fkSolver:forwardKinematics()
	self:setSamePose(self.fkSolver)
end

function FBXloader.AdjustableSkin:_setLengthOnly(length_scale)
	if self.poseMap and self.poseMap.targetIndexAtoB:size()==length_scale:size() then
		local length_scale_orig=length_scale
		length_scale=CT.ones(self.poseMap:target():numBone())
		local AtoB=self.poseMap.targetIndexAtoB
		for i=1, AtoB:size()-1 do
			if AtoB(i)~=-1 then
				length_scale:set(AtoB(i), length_scale_orig(i))
			end
		end
	end
	self.fkSolver:setLengthScale(length_scale)
end
function FBXloader.AdjustableSkin:setSamePose(fk)
	--assert(fk==self.fbx.loader:fkSolver()) BoneForwardKinematics.operator==  doesn't work yet.
	if self.skelSkin then
		self.skelSkin:setSamePose(fk)
	end
	self.fkSolver:assign(fk)
	local fbxloader=self.fbx
	if fbxloader.rigidBody then
		for i, node2 in ipairs(self.nodes2) do
			node2:setOrientation(fk:globalFrame(1).rotation)
			node2:setPosition(fk:globalFrame(1).translation)
		end
		return
	end
	local rootTrans=fk:globalFrame(1):getTranslation():copy()

	local currJointPos=vector3N(self.fkSolver:numBone()-1)
	for ibone=1, self.fkSolver:numBone()-1 do
		--self.fkSolver:globalFrame(ibone).translation:rsub(rootTrans)
		currJointPos(ibone-1):assign(self.fkSolver:globalFrame(ibone):translation()-rootTrans)
	end

	local buildEdgeList=false
	--if self.prevPose then print(self.prevPose:MSE(currJointPos)) end
	if self.prevPose==nil or self.prevPose:MSE(currJointPos)>1e-3 then
		buildEdgeList=true
		self.prevPose=currJointPos
	end

	for i, meshInfo in ipairs(fbxloader.fbxInfo) do
		local ME=self.ME[i]
		local node=self.nodes[i]
		local skin=meshInfo.skin
		local mesh=meshInfo[1]

		skin:calcVertexPositions(self.fkSolver, mesh)
		local removeRootTrans=matrix4()
		removeRootTrans:setTranslation(-rootTrans, false)
		mesh:transform(removeRootTrans)

		local useNormal=mesh:numNormal()>0
		if ME then
			ME:setBuildEdgeList(buildEdgeList)
		end

		--ME:setBuildEdgeList(false)
		if useNormal then
			skin:calcVertexNormals(self.fkSolver, fbxloader:_getBindPoseGlobal(i), meshInfo.localNormal, mesh)
			if ME then
				ME:updatePositionsAndNormals()
			end
		else
			if ME then
				ME:updatePositions()
			end
		end
		if mesh.getVertices then
			mesh:getVertices(node[3]) -- backup current mesh pose
		end
		node[1]:setPosition(node[2]+rootTrans*self.scale.x)
	end
end
return FBXloader
