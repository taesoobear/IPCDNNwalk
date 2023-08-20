require('subRoutines/WRLloader')
-- uses software LBS skinning. so much slower than OgreEntity but I prefer simplicity here.
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

function FBXloader:genSkelTable(fbx, options)
	local jointpos=vector3N()
	local jointori=quaterN()
	if self.fbx:hasBindPose(0) then
		self:getBindPose(jointpos, jointori)
		self:checkRelativeRestPose(jointpos, jointori)  -- heuristic.  I cannot find the option  that describes this condition.
	else
		fbx:getRestPose(jointpos, jointori)
	end
	--self:drawPose(jointpos, jointori, options, 'bindpose', vector3(-300,0,0))
	--self:drawRestPose(options)


	self.options=options

	if options.mirror or options.toYUP or options.toZUP then
		self.convertAxis=true

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
	
	if self.convertAxis then
		self:mirrorVA(jointpos)
		self:mirrorQA(jointori)
	end

	--fbx:getDefaultPose(jointpos, jointori)
	local names=TStrings()
	fbx:getBoneNames(names)
	local pid=fbx:parentIndices()
	local skinScale=options.skinScale or 100

	--dbg.drawBillboard( jointpos:matView()*skinScale, 'line3', 'redCircle', 10, 'QuadListV') -- QuadListV is view-dependent -> use drawBillboard

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

	if options.cleanupBones then
		targetIndex=FBXloader.cleanupBones(tbl, bones, targetIndex, options.cleanupBones, currentPose)
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

		-- map from targetIndex to rotJointIndex of bindPose
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
	self.loader:setPose(self.currentPose)
end

function FBXloader:createSkinningInfo()


	local subMesh=self.fbxInfo[1]

	if #self.fbxInfo>1 then
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
		skin:calcLocalVertexPositions(self.loader, mesh)
	end
	local fbx_info={}
	local skinScale=100
	if self.options and self.options.skinScale then
		skinScale=self.options.skinScale
	end
	fbx_info.loader=self.loader:toVRMLloader(2.5/skinScale) -- tree index can be changed!!!
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

function FBXloader:getAnim(loader)
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
	for i=1, loader:numBone()-1 do
		local keytime=vectorn()
		local traj=matrixn()
		local ilimb=i-1
		ilimb=targetIndex(ilimb);
		if self.trajCache and self.trajCache[i] then
			keytime, traj=unpack(self.trajCache[i])
		else
			fbx:getAnim(ilimb, keytime, traj)
		end
		if keytime:size()==0 then
			-- no motion
			break
		end
		--assert(nkey==-1 or nkey==keytime:size())
		nkey=keytime:size()

		local frameTime=keytime(1)-keytime(0)
		if i==1 then
			loader.mMotion:initEmpty(loader, nkey, frameTime)
			if ilimb~=0 then
				assert(ilimb==1)
				local traj0=matrixn()
				local keytime0=vectorn()
				fbx:getAnim(0, keytime0, traj0)
				for j=0, nkey-1 do
					local tf0=getTF(traj0,j)
					local tf=tf0*getTF(traj,j)

					loader.mMotion:pose(j).rotations(0):assign(tf.rotation)
					loader.mMotion:pose(j).translations(0):assign(tf.translation)
				end
			else
				for j=0, nkey-1 do
					local tf=getTF(traj, j)
					loader.mMotion:pose(j).rotations(0):assign(tf.rotation)
					loader.mMotion:pose(j).translations(0):assign(tf.translation)
				end
			end
		else
			local ri=loader:getRotJointIndexByTreeIndex(i)
			for j=0, nkey-1 do
				local tf=getTF(traj,j)
				loader.mMotion:pose(j).rotations(ri):assign(tf.rotation)
			end
			local ti=loader:getTransJointIndexByTreeIndex(i)
			if ti~=-1 then
				for j=0, nkey-1 do
					local tf=getTF(traj, j)
					loader.mMotion:pose(j).translations(ti):assign(tf.translation)
				end
			end
		end
	end
	if self.convertAxis then
		for j=0, loader.mMotion:numFrames()-1 do
			local pose=loader.mMotion:pose(j)
			self:mirrorVA(pose.translations)
			self:mirrorQA(pose.rotations)
		end
	end
end

function FBXloader:__mergeFBXloaders(loaders, options)
	for i=1, #loaders do
		print('numBone', i, loaders[i].loader:numBone())
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

	local pose=Pose()
	mloader:getPose(pose)

	self.currentPose=pose
	self.bindpose=pose

	self.bindpose_global=quaterN(mloader:numBone())
	for i=1, mloader:numBone()-1 do
		self.bindpose_global(i):assign(mloader:bone(i):getFrame().rotation)
	end

	--local loader=mloader:toVRMLloader(2.5/options.skinScale)
	self.loader=mloader
	self.uid=RE.generateUniqueName()
	local loader=mloader

	loader:setPose(pose)

	if options.exportBVH then
		local newLoader=loader

		newLoader.mMotion:initEmpty(newLoader, 10)
		for i=0, newLoader.mMotion:numFrames()-1 do
			newLoader.mMotion:pose(i):assign(pose)
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
		self:__mergeFBXloaders(filename, options)
		return
	end
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
		local i=1
		while i<loader:numBone() do
			local bone=loader:bone(i)
			if bone:name()=='SITE' then
				loader:removeBone(bone)
			end
			i=i+1
		end
		for i=1, loader:numBone()-1 do
			assert(loader:bone(i):name()==self.wrlInfo.bones[i].name)
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

	local invMap 
	if targetIndex(0)~=0 or targetIndex~=CT.colon(0, loader:numBone()-1) then
		local ti=targetIndex
		invMap={}
		for i=0, ti:size()-1 do
			local treeIndex=i+1
			local origTreeIndex=ti(i)+1
			invMap[origTreeIndex]=treeIndex
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

	local inv_bindpose_scale=1.0/fbx:getBindPose(targetIndex(0)):getColumn(0):length()

	self.bindpose=self.currentPose
	loader:setPose(self.bindpose)
	self.bindpose_global=quaterN(loader:numBone())
	for i=1, loader:numBone()-1 do
		self.bindpose_global(i):assign(loader:bone(i):getFrame().rotation)
	end

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
	local fbxInfo={}
	for ilist, i in ipairs(meshList) do
		local mesh=Mesh()
		local mesh_name=fbx:getMesh(i, mesh)

		local pose=fbx:getMeshCurrPose(i)
		pose:leftMultScaling(inv_bindpose_scale, inv_bindpose_scale, inv_bindpose_scale)
		mesh:transform(pose)

		if self.convertAxis then
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
	if options.useTexture then
		fbx:saveTextures()
		for ilist, i in ipairs(meshList) do
			local c=fbx:getMaterialCount(i)
			for j=0, c-1 do
				print('diffuse', i, fbx:getDiffuseColor(i, j), fbx:getDiffuseTexture(i,j))
				print('specular', i, fbx:getSpecularColor(i, j))
			end
			if c>0 then
				local j=0
				local meshInfo= self.fbxInfo[ilist]
				meshInfo.diffuseColor=fbx:getDiffuseColor(i, j)
				meshInfo.specularColor=fbx:getSpecularColor(i, j)
				assert(meshInfo.specularColor.x==meshInfo.specularColor.x)
				meshInfo.diffuseTexture=filename:sub(1,-5)..'_'..os.filename(fbx:getDiffuseTexture(i,j))

				if RE.ogreSceneManager() and os.isFileExist(meshInfo.diffuseTexture) then
					local image=CImage()
					image:Load(meshInfo.diffuseTexture)
					RE.renderer():createDynamicTexture(self.uid..meshInfo.diffuseTexture,image , meshInfo.diffuseColor, meshInfo.specularColor)
					meshInfo.material=self.uid..meshInfo.diffuseTexture
				end
				
			end
		end
		print(fbx:getAllTextureNames())
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
	self.fbx=fbxloader
	if option.drawSkeleton then
		self.skelSkin=RE.createSkin(fbxloader.loader)
	end
	self.uid=RE.generateUniqueName()

	self.nodes={}
	self.ME={}
	if not RE.ogreSceneManager() then return end

	for i, meshInfo in ipairs(fbxloader.fbxInfo) do
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
			local meshToEntity=MeshToEntity(mesh, self.uid..'meshName'..i, buildEdgeList, dynamicUpdate, useNormal, useTexCoord, useColor)

			self.ME[i]=meshToEntity
		end
		local meshToEntity=self.ME[i]
		local entity=meshToEntity:createEntity('entityName'..self.uid..'_'..i )
		--entity:setMaterialName('white')
		if option.material then
			entity:setMaterialName(option.material )
		elseif meshInfo.material then
			entity:setMaterialName(meshInfo.material )
		else
			entity:setMaterialName('grey_transparent')
		end
		local node=RE.createChildSceneNode(RE.ogreRootSceneNode(), self.uid..'_'..i)
		self.nodes[i]= node
		node:attachObject(entity)
	end
end
function FBXloader.Skin:dtor()
	for i, v in ipairs(self.nodes) do
		RE.removeEntity(v)
	end
	self.nodes=nil
end
function FBXloader.Skin:setMaterial(name)
	for i, me in ipairs(self.ME) do
		me:getLastCreatedEntity():setMaterialName(name)
	end
end
function FBXloader.Skin:setPoseTransfer(poseconv)
	self.poseMap=poseconv
end
function FBXloader.Skin:setPose(pose)
	if self.poseMap then
		self.poseMap:setTargetSkeleton(pose)
		pose=Pose()
		self.poseMap:target():getPose(pose)
	end
	self.fbx.loader:setPose(pose)
	self:setSamePose(self.fbx.loader:fkSolver())
end
function FBXloader.Skin:_setPose(pose, loader)
	self:setPose(pose)
end
function FBXloader.Skin:setPoseDOF(pose)
	if self.poseMap then
		self.poseMap:source():setPoseDOF(pose)
		pose=Pose()
		self.poseMap:source():getPose(pose)
		self.poseMap:setTargetSkeleton(pose)
		self.poseMap:target():getPose(pose)
		self.fbx.loader:setPose(pose)
	else
		self.fbx.loader:setPoseDOF(pose)
	end
	self:setSamePose(self.fbx.loader:fkSolver())
end
function FBXloader.Skin:setSamePose(fk)
	--assert(fk==self.fbx.loader:fkSolver()) BoneForwardKinematics.operator==  doesn't work yet.
	if self.skelSkin then
		self.skelSkin:setSamePose(fk)
	end
	local fbxloader=self.fbx
	for i, meshInfo in ipairs(fbxloader.fbxInfo) do
		local ME=self.ME[i]
		local skin=meshInfo.skin
		local mesh=meshInfo[1]

		skin:calcVertexPositions(fbxloader.loader, mesh)
		local useNormal=mesh:numNormal()>0
		if useNormal then
			skin:calcVertexNormals(fbxloader.loader, fbxloader.bindpose_global, meshInfo.localNormal, mesh)
			ME:updatePositionsAndNormals()
		else
			ME:updatePositions()
		end
	end
end
function FBXloader.Skin:setScale(x,y,z)
	if self.skelSkin then
		self.skelSkin:setScale(x,y,z)
	end
	for i ,node in ipairs(self.nodes) do
		node:setScale(x,y,z)
	end
end
function FBXloader.Skin:setTranslation(x,y,z)
	if self.skelSkin then
		self.skelSkin:setTranslation(x,y,z)
	end
	for i ,node in ipairs(self.nodes) do
		node:setPosition(vector3(x,y,z))
	end
end

function RE.createFBXskin(fbxloader, drawSkeleton)
	if type(drawSkeleton)=='boolean' then
		return FBXloader.Skin(fbxloader, { drawSkeleton=drawSkeleton})
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

	local trajCache={}
	for i=0, targetIndex:size()-1 do
		local ti=i+1
		local ilimb=targetIndex(i)

		local keytime=vectorn()
		local traj=matrixn()
		fbx:getAnim(ilimb, keytime, traj)
		trajCache[ti]={keytime, traj}
	end

	for i, v in ipairs(bones) do
		if trajCache[i][2]:cols()==7 then
			v.jointType=nil-- use free joint
		end
	end
	local loader=FBXloader.motionLoaderFromTable(bones, targetIndex)

	FBXloader.getAnim(info, loader)
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
			loader:insertChildBone(loader:bone(parent_treeIndex), bone.name, false)
		else
			loader:insertChildBone(loader:bone(0), bone.name, false)
		end
		local b=loader:getBoneByName(bone.name)

		if bone.translation then
			if i~=1 then
				b:getOffsetTransform().translation:assign(bone.translation)
			end
		end

		if not bone.jointType then
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
	local names=TStrings()
	fbx:getBoneNames(names)

	local bindposes={}
	for i=0, names:size()-1 do
		local pose=fbx:getBindPose(i)
		bindposes[i+1]=pose
	end
	local inv_bindpose_scale=1.0/bindposes[1]:getColumn(0):length()
	for i, v in ipairs(bindposes) do
		v:leftMultScaling(inv_bindpose_scale, inv_bindpose_scale, inv_bindpose_scale)
	end
	jointpos:resize(names:size())
	jointori:resize(names:size())
	for i=0, names:size()-1 do
		local tf=transf(bindposes[i+1])
		jointpos(i):assign(tf.translation)
		jointori(i):assign(tf.rotation)

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
return FBXloader
