
require("subRoutines/AnimOgreEntity")
PoseTransfer2=require("subRoutines/PoseTransfer2")
RetargetSkin=LUAclass()
-- loaderA has to be a properly setup VRMLloader with all important bones annotated (setVoca).
-- entity has to be an instance of AnimOgreEntity
-- both loaderA and loaderB have to be currently similar to T-pose
function RetargetSkin:__init(loaderA, skinConfig, showDebugSkin)
	self.position=vector3(0,0,0)
	-- necessary paramters:
	local sourceFile=skinConfig.entity
	local boneCorrespondences=skinConfig.boneCorrespondences
	local entityScale=skinConfig.entityScale or 1
	local markers=skinConfig.markers or {}
	local buildEdgeList=skinConfig.buildEdgeList or nil
	self.enforceMarkerPos=skinConfig.enforceMarkerPos or false
	self.enforceMarkerDir=skinConfig.enforceMarkerDir or {}
	self.scaleInfo=skinConfig.scale or {}
	self.setPoseAhook=skinConfig.setPoseAhook or nil
	self.setPoseBhook=skinConfig.setPoseBhook or nil
	self.posErrorHook=skinConfig.posErrorHook or nil

	local skinScale=skinScale or 100
	assert (string.sub(sourceFile, -4)=='mesh')
	local entity=AnimOgreEntity(sourceFile, entityScale, skinScale, 1, buildEdgeList)
	--entity:startDebugMode() -- make mesh light
	-- copy the current skeleton pose to the skin
	local newPose=Pose()
	entity.loader:getPose(newPose)
	entity:_setPose(newPose, entity.loader)
	self.bindPoseB_orig=newPose

	self.loaderA=loaderA
	self.entity=entity
	self.loaderB=entity.loader

	local convInfoA=TStrings()
	local convInfoB=TStrings()
	local AtoB=intvectorn(self.loaderA:numBone())
	AtoB:setAllValue(-1)
	for k,v in pairs(boneCorrespondences) do
		local i=self.loaderA:getTreeIndexByName(k)
		local j=self.loaderB:getTreeIndexByName(v)
		if i~=-1 and j~=-1 then
			convInfoA:pushBack(k)
			convInfoB:pushBack(v)
			AtoB:set(i, j)
		else
			print(k,v, i,j)
			assert(false)
		end
	end
	self.AtoB=AtoB

	local markerPosByA=vector3N(self.loaderA:numBone())
	markerPosByA:setAllValue(vector3(0,0,0))
	self:_updateMarkerPosArray(markerPosByA, boneCorrespondences, markers)

	self.markerPosByA=markerPosByA

	local Tpose_markerPosByA=markerPosByA:copy()
	if skinConfig.Tpose_markers then
		self:_updateMarkerPosArray(Tpose_markerPosByA, boneCorrespondences, skinConfig.Tpose_markers)
	end
	self.Tpose_markerPosByA=Tpose_markerPosByA

	do 
		-- compute BtoA
		local loaderB=self.entity.loader
		local BtoA=intvectorn(loaderB:numBone())
		BtoA:setAllValue(-1)

		for i=1, loaderA:numBone()-1 do
			if AtoB(i)~=-1 then
				BtoA:set(AtoB(i),i)
			end
		end
		self.BtoA=BtoA
		self.childEndA=loaderA:getChildEnd()
		self.childEndB=loaderB:getChildEnd()
	end
	--local M=require("RigidBodyWin/retargetting/module/retarget_common")
	--M.gotoTpose(loaderA)
	loaderA:updateInitialBone() -- T skeleton에 대해서만 동작하도록 함. 이게 여러모로 쉬울 듯.

	if true then
		-- loader B는 좀더 정밀하게 marker를 사용하여 T-pose로 만든다.  
		-- 이 방법으로 마커의 위치를 조절하여 자세 alignment를 미세 조정 가능함.
		local function rotate(self, v1, v2, tdir)

			
			local ti1A=self.loaderA:getTreeIndexByVoca(v1)
			local ti2A=self.loaderA:getTreeIndexByVoca(v2)
			if ti1A==-1 or ti2A==-1 then
				return 
			end

			local ti1=self.AtoB(ti1A)
			local ti2=self.AtoB(ti2A)
			assert(ti1~=-1)
			assert(ti2~=-1)

			local marker1=self.Tpose_markerPosByA(ti1A)
			local marker2=self.Tpose_markerPosByA(ti2A)
			local loader=self.loaderB
			local shdr=loader:bone(ti1)
			local elbow=loader:bone(ti2)
			local dir=elbow:getFrame()*marker2-shdr:getFrame()*marker1
			local q=quater()
			q:axisToAxis(dir, tdir)
			loader:rotateBoneGlobal(shdr, q)
		end
		local chains=
		{
			{vector3(1,0,0), MotionLoader.LEFTSHOULDER, MotionLoader.LEFTELBOW, MotionLoader.LEFTWRIST},
			{vector3(-1,0,0), MotionLoader.RIGHTSHOULDER, MotionLoader.RIGHTELBOW, MotionLoader.RIGHTWRIST},
			{vector3(0,-1,0),MotionLoader.LEFTHIP, MotionLoader.LEFTKNEE, MotionLoader.LEFTANKLE},
			{vector3(0,-1,0),MotionLoader.RIGHTHIP, MotionLoader.RIGHTKNEE, MotionLoader.RIGHTANKLE},
		}
		local function getAxis(ii)
			local out={}
			if ii==1 then
				out[1]=vector3(1,0,0)
			else
				out[1]=vector3(-1,0,0)
			end
			return out
		end
		for ic, chain in ipairs(chains) do
			local axis=chain[1]
			local n=#chain-2
			for i=1, n do
				rotate(self, chain[i+1], chain[i+2], axis)
			end
		end
	end

	if skinConfig.bindPoseBpostprocess then
		local loader=self.loaderB
		for k, v in ipairs(skinConfig.bindPoseBpostprocess) do
			local shdr=loader:getBoneByName(v[1])
			loader:rotateBoneGlobal(shdr, v[2])
		end
	end

	self.bindPoseB=Pose()
	self.loaderB:getPose(self.bindPoseB)
	self.PT=PoseTransfer2(loaderA, self.loaderB,  convInfoA, convInfoB, 1)
	if showDebugSkin then
		-- for debugging
		self.debugSkin= RE.createSkinAuto(self.loaderB);	-- to create character .
		self.debugSkin:scale(skinScale,skinScale,skinScale);					-- motion data is in meter unit while visualization uses cm unit.
		self.debugSkin:setMaterial('green_transparent')
		self.debugSkin:setTranslation(100,0,0)
		self.debugOgreSkin=AnimOgreEntity(sourceFile, entityScale, skinScale, 1, buildEdgeList)
		self.debugOgreSkin:setMaterial('yellow_transparent')
		self.debugOgreSkin:setTranslation(100,0,0)
	end

end

function RetargetSkin:_updateMarkerPosArray(markerPosByA, boneCorrespondences, markers)
	for k,v in pairs(boneCorrespondences) do
		local i=self.loaderA:getTreeIndexByName(k)
		local j=self.loaderB:getTreeIndexByName(v)
		if i~=-1 and j~=-1 then
			local localposB=markers[k]
			if localposB then
				if type(localposB)=='table' then
					if localposB.rotate then
						localposB=self.loaderB:bone(j):getFrame().rotation:inverse()*localposB[1]
					else
						localposB=localposB[1]
					end
				end
				markerPosByA(i):assign( localposB)
			end
		else
			print(k,v, i,j)
			assert(false)
		end
	end
end

function RetargetSkin:_debugDrawMarkers(origpose, offset)

	local loaderA=self.loaderA
	local loader=self.loaderB

	local bindPoseB=self.bindPoseB
	if origpose then
		if type(origpose)=='boolean' then
			bindPoseB=self.bindPoseB_orig
		else
			bindPoseB=origpose
		end
	end

	self.entity:_setPose(bindPoseB, self.loaderB);    -- grey skeleton or textured ogre entity
	self.entity:setMaterial('lightgrey_transparent')

	loader:setPose(bindPoseB)
	for i=1, loaderA:numBone()-1 do
		local ti=self.AtoB(i)

		if ti~=-1 then
			local markerLocalPos=self.markerPosByA(i)
			local loader=self.loaderB
			local shdr=loader:bone(ti)
			local mpos=shdr:getFrame()*markerLocalPos
			dbg.namedDraw('Sphere', mpos*skinScale+(offset or vector3(0,0,0)), 'm_'..loaderA:bone(i):name(), 'red_transparent', 5)

			markerLocalPos2=self.Tpose_markerPosByA(i)
			if markerLocalPos2:distance(markerLocalPos)>0.00001 then
				local mpos=shdr:getFrame()*markerLocalPos2
				dbg.namedDraw('Sphere', mpos*skinScale+(offset or vector3(0,0,0)), 'm2_'..loaderA:bone(i):name(), 'green_transparent', 3)
			end
		end
	end
end
function RetargetSkin:__deinit()
	self.loaderA=nil
	if self.entity then self.entity:__deinit() end
	if self.debugOgreSkin then
		self.debugOgreSkin:__deinit()
	end
	self.entity=nil
	self.loaderB=nil
	self.PT=nil
end
function RetargetSkin:setPoseDOF(poseA)
	if self.setPoseAhook then
		self.loaderA:setPoseDOF(poseA)
		poseA=Pose()
		self.loaderA:getPose(poseA)
		self.setPoseAhook(self.loaderA, poseA)
	end
	self.PT:setTargetSkeleton(poseA)

	self:_setPoseFromLoaderAB()
end
function RetargetSkin:_setPose(poseA, loader)
	if self.setPoseAhook then
		poseA=poseA:copy()
		self.setPoseAhook(self.loaderA, poseA)
	end
	self.PT:setTargetSkeleton(poseA)

	self:_setPoseFromLoaderAB()
end

function RetargetSkin:_setPoseFromLoaderAB()
	local poseOrig=Pose()
	self.loaderB:getPose(poseOrig)

	local frameA=self.loaderA:bone(1):getFrame()
	local frameB=self.loaderB:bone(self.AtoB(1)):getFrame()
	poseOrig.translations(0):radd(frameA.translation-frameB.translation)

	local enforceMarkers=self.enforceMarkerPos
	-- todo2
	enforceMarkers=false

	local posError
	local animEntity=self.entity
	local BI=animEntity.BIndex_to_BI
	local entity=animEntity.ogreEntity
	local node=animEntity.ogreNode
	local skel=entity:getSkeleton()
	local loaderB=animEntity.loader
	if self.setPoseBhook then
		self:setPoseBhook(loaderB, poseOrig)
	else
		loaderB:setPose(poseOrig)
	end
	local gt=loaderB:bone(1):getFrame().translation:copy()

	local posError=vector3N(loaderB:numBone())
	local loaderA=self.loaderA
	local AtoB=self.AtoB
	local BtoA=self.BtoA
	do
		-- compute posError

		posError:setAllValue(vector3(0,0,0))

		local directDescendents=boolN(loaderB:numBone())

		local childEndA=self.childEndA
		local childEndB=self.childEndB

		local config={ entityScale=animEntity.entityScale, skinScale=animEntity.scale}
		--if true then
		if enforceMarkers then
			-- calc position error
			for i=0, BI:size()-1 do
				local bi=BI(i)
				if bi>1 then
					local biA=BtoA(bi)
					if biA~=-1 then
						local curr=loaderB:bone(bi):getFrame()*self.markerPosByA(biA)
						--dbg.draw("Sphere", curr*100, 'pos'..bi)
						local delta=loaderA:bone(biA):getFrame().translation-curr

						directDescendents:range(bi, childEndB(bi)):setAllValue(true)

						-- clear non-direct descendents
						for ii=biA+1, childEndA(biA)-1 do
							local bb=AtoB(ii)
							assert(bb~=-1)
							directDescendents:range(bb, childEndB(bb)):setAllValue(false)
						end
						--print(directDescendents:range(bi, childEndB(bi)))
						--loaderB:printHierarchy()

						for ii=bi, childEndB(bi)-1 do
							if directDescendents(ii) then
								posError(ii):radd(delta)
							end
						end
					end
				end
			end
			if self.posErrorHook then
				self.posErrorHook(self, loaderB, posError)
			end
		end
	end


	if enforceMarkers then
		-- set ogre pose
		local pose=poseOrig
		for i=0, BI:size()-1 do
			local bi=BI(i)
			local RI=animEntity.BIndex_to_RI
			local bone=skel:getBone(i)
			local ri=RI(i) local q=pose.rotations(ri)
			if ri~=-1 then bone:setOrientation(q) end -- necessary. I don't know why.
			if bi>1 then
				local gpos=loaderB:bone(bi):getFrame().translation
				bone:_setDerivedOrientation(loaderB:bone(bi):getFrame().rotation)

				local biA=BtoA(bi)
				if biA~=-1 and self.enforceMarkerDir[loaderA:bone(biA):name()] then
					local curr1=loaderB:bone(bi):getFrame()*self.markerPosByA(biA)
					local bi2=AtoB(biA+1)
					local curr2=loaderB:bone(bi2):getFrame()*self.markerPosByA(biA+1)

					local tgt1=loaderA:bone(biA):getFrame().translation
					local tgt2=loaderA:bone(biA+1):getFrame().translation
					local delta=quater()
					delta:axisToAxis(curr2-curr1, tgt2-tgt1)
					bone:_setDerivedOrientation(delta*loaderB:bone(bi):getFrame().rotation) 
				end

				local bone_sf=self.scaleInfo[bone:getName()]
				if bone_sf then
					bone:setScale(bone_sf, bone_sf, bone_sf)
				end
				bone:_setDerivedPosition((gpos+posError(bi)-gt)*(1/animEntity.sf))
			end
		end
		node:setPosition(gt*animEntity.scale+self.position)
	--local original_pos=lsh.bone:_getDerivedPosition()
	--lsh.bone:_setDerivedPosition(original_pos+vector3(0,0,1))
	--print(RE.getSceneNode(ogreSkin):getPosition()/config.entityScale)
	else
		local BtoA=self.BtoA
		loaderB:getPose(poseOrig)
		local entity=animEntity.ogreEntity
		local node=animEntity.ogreNode

		local skel=entity:getSkeleton()
		do
			local gt=loaderB:bone(1):getFrame().translation:copy()
			local RI=animEntity.BIndex_to_RI
			local pose=poseOrig
			for i=0, RI:size()-1 do
				local ri=RI(i)
				if ri~=-1 then
					local q=pose.rotations(ri)
					local bone=skel:getBone(i)
					bone:setOrientation(q)
					local bone_sf=self.scaleInfo[bone:getName()]
					if bone_sf then
						bone:setScale(bone_sf, bone_sf, bone_sf)
					end
				end
			end
			node:setPosition(gt*animEntity.scale+self.position)
		end
	end
	if self.debugSkin then
		self.debugSkin:_setPose(poseOrig, self.loaderB)
		self.debugOgreSkin:_setPose(poseOrig, self.loaderB)
		--self:_debugDrawMarkers(poseOrig, vector3(0,0,0))
	end
	return poseOrig
end

function RetargetSkin:setTranslation(x,y,z)
	self.position.x=x
	self.position.y=y
	self.position.z=z
end

function RetargetSkin:setVisible(bValue)
	self.entity.ogreNode:setVisible(bValue)
end
