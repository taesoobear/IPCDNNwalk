local EW=require('subRoutines/exportOgreEntityToVRML')
OgreLoader=LUAclass()

-- OgreLoader는 OgreEntity의 skeleton을 그대로 taesooLib으로 가져올때 사용한다. (FBXloader와 사용법을 맞추려고 함)
-- entityScale : entity (표면메시) 그릴때 몇배 스케일 할건지
-- skinScale : skeleton을 그릴때 몇배 스케일할건지.
--
-- outputs:
-- self.sf  -> loader의 pose의 translation을 몇배 scale해야 표면 메시와 호환되는지.
function OgreLoader:__init(meshFileName, entityScale, _optional_skinScale, _otherOptions)
	if not _otherOptions then
		_otherOptions={}
	end
	if type(_otherOptions)=='number' then
		_otherOptions={ skelScale=_otherOptions}
	end

	local _optional_skelScale=_otherOptions.skelScale
	local _buildEdgeList=_otherOptions._buildEdgeList or true
	local s2=_optional_skelScale or 1	
	if _buildEdgeList==nil then _buildEdgeList=true end
	self.entityScale=entityScale
	self.scale=_optional_skinScale or 100
	self.meshFile=meshFileName

	if OgreLoader.count==nil then
		OgreLoader.count=0
	else
		OgreLoader.count=OgreLoader.count+1
	end
	
	self.entityName=meshFileName.."_AOE_"..OgreLoader.count
	-- ogre entity is for visualizing the bind pose
	self.ogreEntity=RE.ogreSceneManager():createEntity(self.entityName.."_raw", self.meshFile)
	if _buildEdgeList then
		RE.buildEdgeList(self.meshFile)
	end


	local skinScale=self.scale
	do
		local config={ entityScale=self.entityScale, skinScale=self.scale}

		--local file='_temp.wrl'
		local file=CTextFile() -- use a memory file.
		local sf=EW.convert(self.ogreEntity, file, 'entity', 1/entityScale*skinScale/100/s2)
		sf=sf*config.entityScale/config.skinScale*s2
		self.loader=MainLib.VRMLloader (file)
		self.loader:Scale(sf)
		self.sf=sf
		for i=1, self.loader:numBone()-1 do
			local name=self.loader:bone(i):name()
			name=string.gsub(name,'__SPACE__', ' ')
			self.loader:bone(i):setName(name)
		end

		local Pose=EW.getPoseFromOgreEntity(self.loader, self.ogreEntity, config)
		Pose.translations(0):rmult(sf)
		self.loader:setPose(Pose)


		self.ogreRootOffset=vector3(0)


		if _otherOptions.newRootBone then
			local ti=self.loader:getTreeIndexByName(_otherOptions.newRootBone)
			assert(ti~=-1)
			self.loaderOrig=self.loader
			self.loader=MainLib.VRMLloader(self.loaderOrig, ti, true)

			Pose=EW.getPoseFromOgreEntity(self.loader, self.ogreEntity, config)

			-- global=bindDelta*local
			self.bindDelta=self.loaderOrig:bone(ti):parent():getFrame():copy()
			self.bindLocal=self.loaderOrig:bone(ti):getLocalFrame():copy()

			Pose.rotations(0):assign(self.loaderOrig:bone(ti):getFrame().rotation)
			Pose.translations(0):assign(self.loaderOrig:bone(ti):getFrame().translation)
			self.loader:setPose(Pose)

			self.ogreRootOffset= self.loaderOrig:bone(1):getFrame().translation - self.loaderOrig:bone(ti):getFrame().translation

			--TempSkin=RE.createSkin(self.loaderOrig)
			--TempSkin:setScale(100)
			--TempSkin:setTranslation(200,0,0)
		end
	end
	self.bindpose=vectorn()
	self.loader:getPoseDOF(self.bindpose)

	self.BIndex_to_RI=vectorn()
	self.BIndex_to_TI=vectorn()
	self.BIndex_to_BI=vectorn()
	do
		local loader=self.loader
		local entity=self.ogreEntity
		local bones, root=EW.getBones(entity)
		assert(root.id==0)
		self.BIndex_to_RI:setSize(#bones+1)
		self.BIndex_to_TI:setSize(#bones+1)
		self.BIndex_to_BI:setSize(#bones+1)
		for i=root.id, #bones do
			local bone=bones[i]
			local bi=loader:getTreeIndexByName(bone.name)
			if bi~=-1 then
				assert(ti~=-1)
				local lbone=loader:bone(bi)
				local ri=lbone:rotJointIndex()
				local ti=lbone:transJointIndex()
				if ri~=-1 then
					--bone.bone:setManuallyControlled(true)
					entity:getSkeleton():setManualBone(bone.bone, true)
				end
				self.BIndex_to_RI:set(i, ri)
				self.BIndex_to_TI:set(i, ti)
				self.BIndex_to_BI:set(i, bi)
			else
				self.BIndex_to_RI:set(i, -1)
				self.BIndex_to_TI:set(i, -1)
				self.BIndex_to_BI:set(i, -1)
			end
		end
	end
	--self:startDebugMode()
end


OgreLoader.AdjustableSkin=LUAclass()
function RE.createOgreSkin(ogreloader, options)
	return OgreLoader.AdjustableSkin(ogreloader, options)
end
function OgreLoader.AdjustableSkin:setTranslation(x,y,z)
	self.trans.x=x
	self.trans.y=y
	self.trans.z=z
end
function OgreLoader.AdjustableSkin:setMaterial(name)
	self.ogreEntity:setMaterialName(name)
end
function OgreLoader.AdjustableSkin:__init(ogreloader, option)
	if not option then option={} end
	if type(option)=='boolean' then
		option={ drawSkeleton=option}
	end
	self.scale=vector3(1,1,1)
	self.ogreLoader=ogreloader
	self.uid=RE.generateUniqueName()
	self.trans=vector3(0,0,0)
	if not RE.ogreSceneManager() then return end

	self.ogreEntity=RE.ogreSceneManager():createEntity(self.uid..'_entity', self.ogreLoader.meshFile)

	if true then
		local entity=self.ogreEntity
		local node=self.ogreNode
		local skel=entity:getSkeleton()
		local OL=ogreloader

		local RI=OL.BIndex_to_RI
		for i=0, RI:size()-1 do
			local ri=RI(i)
			if ri~=-1 then
				local bone=skel:getBone(i)
				--bone:setManuallyControlled(true)
				entity:getSkeleton():setManualBone(bone, true)
				bone:setInheritScale(false)
			end
		end
	end

	self.ogreNode=RE.createChildSceneNode(RE.ogreRootSceneNode(),self.uid)
	self.ogreNode:attachObject(self.ogreEntity)

	self.fkSolver=util.ScaledBoneKinematics(ogreloader.loader)
	if option.drawSkeleton then
		self.skelSkin=RE.createSkin(ogreloader.loader, PLDPrimSkin.LINE)
		local s=ogreloader.scale
		self.skelSkin:setScale(s,s,s)
		self.skelSkin:setPose(ogreloader.loader:pose())
	end

	if option.boneScale then
		local defaultScale=vector3N(ogreloader.loader:numBone())
		local lengthScale=vectorn(ogreloader.loader:numBone())
		defaultScale:setAllValue(vector3(1))
		lengthScale:setAllValue(1)
		local loader=ogreloader.loader
		for boneName, s in pairs(option.boneScale) do
			local b=loader:getBoneByName(boneName)
			local children=b:children()
			for i, b in ipairs(children) do
				defaultScale(b:treeIndex()):assign(vector3(s))
			end
		end
		self.fkSolver:setScale(defaultScale)
		self.fkSolver:setLengthScale(lengthScale)
	end

	self:setPose(ogreloader.loader:pose())
end
function OgreLoader.AdjustableSkin:setScale(s)
	if s~=self.ogreLoader.scale then
		util.msgBox("doesn't support scaling yet")
	end
end
function OgreLoader.AdjustableSkin:setLengthAndPose(length_scale, pose)
	self:_setLengthOnly(length_scale)
	self:setPose(pose)
end
function OgreLoader.AdjustableSkin:setLengthAndPoseDOF(length_scale, pose)
	self:_setLengthOnly(length_scale)
	self:setPoseDOF(pose)
end
-- change length while maintaining pose
function OgreLoader.AdjustableSkin:setLengthScale(length_scale)
	self:_setLengthOnly(length_scale)
	self.fkSolver:forwardKinematics()
	self:setSamePose(self.fkSolver)
end

function OgreLoader.AdjustableSkin:_setLengthOnly(length_scale)
	if self.poseMap and self.poseMap.targetIndexAtoB:size()==length_scale:size() then
		local length_scale_orig=length_scale
		length_scale=CT.ones(self.poseMap:target():numBone())
		local AtoB=self.poseMap.targetIndexAtoB
		for i=1, AtoB:size()-1 do
			length_scale:set(AtoB(i), length_scale_orig(i))
		end
	end
	self.fkSolver:setLengthScale(length_scale)
end


-- assuming self.loader==mLoader_orig
-- poseconv=MotionUtil.PoseTransfer(mLoader, mLoader_orig, true)
function OgreLoader.AdjustableSkin:setPoseTransfer(poseconv)
	self.poseMap=poseconv
end
function OgreLoader.AdjustableSkin:setPose(pose)
	if self.poseMap then
		self.poseMap:setTargetSkeleton(pose)
		pose=Pose()
		self.poseMap:target():getPose(pose)
	end
	self.fkSolver:setPose(pose)
	self:setSamePose(self.fkSolver)
end
function OgreLoader.AdjustableSkin:_setPose(pose, loader)
	self:setPose(pose)
end
function OgreLoader.AdjustableSkin:setPoseDOF(pose)
	if self.poseMap then
		self.poseMap:source():setPoseDOF(pose)
		pose=Pose()
		self.poseMap:source():getPose(pose)
		self.poseMap:setTargetSkeleton(pose)
		self.poseMap:target():getPose(pose)
		self.fkSolver:setPose(pose)
	else
		self.fkSolver:setPoseDOF(pose)
	end
	self:setSamePose(self.fkSolver)
end

function OgreLoader.AdjustableSkin:setSamePose(fk)
	--assert(fk==self.ogreLoader.loader:fkSolver()) BoneForwardKinematics.operator==  doesn't work yet.
	if self.skelSkin then
		self.skelSkin:setSamePose(fk)
		--for i=1, fk:numBone()-1 do
		--	dbg.draw('SphereM',fk:globalFrame(i):translation(),'bone'..i,'red')
		--end
	end
	self.fkSolver:assign(fk)

	local OL=self.ogreLoader
	local loader=OL.loader
	local sf=self.ogreLoader.scale
	--if useLocal then
	--local hasScale=boolN(loader:numBone()) hasScale:setAllValue(false)
	if true then
		local config={ entityScale=self.entityScale, skinScale=self.scale}
		local entity=self.ogreEntity
		local node=self.ogreNode

		local skel=entity:getSkeleton()

		-- set local orientation
		local RI=OL.BIndex_to_RI
		for i=0, RI:size()-1 do
			local ri=RI(i)
			if ri~=-1 then
				local ti=loader:getTreeIndexByRotJointIndex(ri)
				local q=fk:localRot(ti)
				assert(q.x==q.x)
				local bone=skel:getBone(i)
				if ri==0 and OL.bindDelta then


					-- delta * M0 * M1 * M2 * bindLocal  =  M0*M1*M2 *delta2
					--> 
					-- delta2= (M0*M1*M2).inverse * delta *(M0*M1*M2* bindLocal)

					local G_ti=transf(fk:localRot(ti), OL.bindpose:toVector3(0))
					local L_ti=OL.bindDelta:inverse()*G_ti

					bone:setOrientation(L_ti.rotation)
					bone:setPosition(sf*L_ti.translation)
				else
					bone:setOrientation(q)
					--local Q=q:M44()
					--local Q=quater(math.rad(90), vector3(0,0,1)):M44()
					local s=fk:localScale(ti)


					local scaleFactor=vector3(s._11, s._22, s._33) -- ogre only supports diagonal scale.
					--if math.abs(scaleFactor:length()-1.73205)>1e-2 then hasScale:set(ti, true) end
					bone:setScale(scaleFactor)

					if ri==0 then
						bone:setPosition(vector3(0,0,0))
					else
						bone:setPosition(sf*fk:localFrame(ti):translation())
					end
				end


				--if hasScale(loader:bone(ti):parent():treeIndex()) then
				--	-- 위치 오차가 생김.  추가 보정 필요.
				--	-- doesn't work
				--	local errorPos=sf*fk:globalFrame(ti):translation()- bone:_getFullTransformPosition()
				--	print(ti, loader:bone(ti):name(),errorPos, bone:_getFullTransformPosition())


				--	bone:setPosition(sf*fk:localFrame(ti):translation()+fk:globalRot(ti):inverse()*errorPos)
				--end
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

		local taesooLibRootTF=transf(fk:globalFrame(1))
		node:setPosition((taesooLibRootTF.translation+OL.ogreRootOffset)*OL.scale+self.trans)


	--else
	--	-- does not work well for some skeletons
	--	-- but very useful for precise control
	--	-- if this does not work, set useLocal=true
	--	loader:setPose(pose)
	--	self:setExactPose(loader)
	end
end
