
local out={}
function out.FBXloader_cleanupBones(self, cleanupBones)
	self:_bakeBindPose()
	if not self.wrlInfo then
		local wrlTable, bones=self.loader:toWRLtable()
		self.wrlInfo={
			bones=bones, 
			wrlTable=wrlTable
		}
	end

	local bones=deepCopyTable(self.wrlInfo.bones)
	local targetIndex=CT.colon(0, #bones)
	local tbl=self.wrlInfo.wrlTable
	local currentPose=self.currentPose
	targetIndex=self.cleanupBones(tbl, bones, targetIndex, cleanupBones, currentPose)

	local loader=self.loader
	local oldTItoNewTI={}
	for i=0, targetIndex:size()-1 do
		oldTItoNewTI[targetIndex(i)+1]=i+1
	end
	for i, v in ipairs(bones) do
		local bone=loader:bone(i)
		assert(v.name==bone:name())
		v.parent=bone:parent():treeIndex()
		v.children=nil
	end
	local newBones={}
	for i=0, targetIndex:size()-1 do
		local ti=targetIndex(i)+1
		local origBone=bones[ti]
		newBones[i+1]=copyTable(origBone)
		local pid=oldTItoNewTI[origBone.parent]
		assert(pid or origBone.parent==0)
		newBones[i+1].parent=pid
	end


	loaderNew=self.motionLoaderFromTable(newBones) --:toVRMLloader(0.25) -> vrmlloader conversion is slow and unnecesary.
	local newpose=loaderNew:pose()
	newpose.translations(0):assign(currentPose.translations(0))
	for i=0, newpose:numRotJoint()-1 do
		newpose.rotations(i):assign(currentPose.rotations(targetIndex(i)))
	end

	if false then
		skinNew=RE.createSkinAuto(loaderNew)
		skinNew:setTranslation(100,0,0)
		skinNew:setPose(newpose)

		skinOld=RE.createSkinAuto(self.loader)
		skinOld:setTranslation(200,0,0)
		skinOld:setPose(currentPose)

		for i=1, loaderNew:numBone()-1 do
			assert(skinNew:getState():globalFrame(i).translation:distance(
			skinOld:getState():globalFrame(targetIndex(i-1)+1).translation)
			<1e-3)
		end
		do return end
	end

	-- fill-in blanks
	for i=1, loader:numBone()-1 do
		if oldTItoNewTI[i]==nil then
			local newTI=oldTItoNewTI[loader:bone(i):parent():treeIndex()]
			assert(newTI)
			oldTItoNewTI[i]=newTI
		end
	end


	for i,meshInfo in ipairs(self.fbxInfo) do
		local mesh=meshInfo[1]
		local skin=meshInfo.skin
		for i=0, skin:numVertex()-1 do
			local ti= skin:treeIndices(i)
			for i=0, ti:size()-1 do
				ti:set(i, oldTItoNewTI[ti(i)])
			end
		end
	end
	loaderNew:setPose(newpose)
	self.loader=loaderNew
	self:_setBindPose(loaderNew)
	self:_bindPoseUpdated()
end
return out
